package com.aerodynamics4mc.runtime;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;

import net.minecraft.util.Identifier;
import net.minecraft.util.math.BlockPos;
import net.minecraft.util.math.MathHelper;

public final class BackgroundFieldGrid {
    public static final int SOLVER_INPUT_CHANNELS = 9;
    public static final int SOLVER_OUTPUT_CHANNELS = 4;

    private static final int VELOCITY_COMPONENTS = 3;
    private static final int THERMAL_COMPONENT_INDEX = 3;
    private static final int FIELD_COMPONENTS = 4;

    private static final int DEFAULT_SOLVER_GRID_SIZE = 128;
    private static final int DEFAULT_UPDATE_INTERVAL_TICKS = 20;
    private static final float DEFAULT_RELAXATION = 0.2f;
    private static final float DEFAULT_WORLD_THERMAL_WEIGHT = 1.0f;

    private static final int THERMAL_SAMPLE_STRIDE = 1;
    private static final float THERMAL_SOURCE_RELAXATION = 0.40f;
    private static final float THERMAL_DIFFUSIVITY = 0.03f;
    private static final float THERMAL_ADVECTION = 0.35f;
    private static final float THERMAL_MAX_ABS_K = 8.0f;
    private static final float BUOYANCY_TARGET_SPEED_PER_K = 0.65f;
    private static final float BUOYANCY_TARGET_SPEED_MAX = 6.0f;
    private static final float VELOCITY_FAILURE_DAMPING = 0.92f;
    private static final float VELOCITY_MAX_ABS = 16.0f;

    private static final long CONTEXT_ID_BASE = 1L << 52;

    private final int solverGridSize;
    private final int updateIntervalTicks;
    private final float relaxation;
    private final float worldThermalWeight;
    private final FlowSolver flowSolver;
    private final Map<FieldKey, DimensionField> fields = new ConcurrentHashMap<>();
    private final Map<Identifier, List<DimensionField>> fieldsByDimension = new ConcurrentHashMap<>();
    private long nextContextId = CONTEXT_ID_BASE;

    @FunctionalInterface
    public interface ThermalSampler {
        float sampleThermalAnomaly(int worldX, int worldY, int worldZ);
    }

    public interface FlowSolver {
        boolean step(int gridSize, byte[] payload, long contextId, float[] output);

        default void releaseContext(long contextId) {
        }
    }

    public record WindowSample(long windowKey, BlockPos origin) {
    }

    public BackgroundFieldGrid() {
        this(null, DEFAULT_SOLVER_GRID_SIZE, DEFAULT_UPDATE_INTERVAL_TICKS, DEFAULT_RELAXATION, DEFAULT_WORLD_THERMAL_WEIGHT);
    }

    public BackgroundFieldGrid(FlowSolver flowSolver) {
        this(flowSolver, DEFAULT_SOLVER_GRID_SIZE, DEFAULT_UPDATE_INTERVAL_TICKS, DEFAULT_RELAXATION, DEFAULT_WORLD_THERMAL_WEIGHT);
    }

    BackgroundFieldGrid(
        FlowSolver flowSolver,
        int solverGridSize,
        int updateIntervalTicks,
        float relaxation,
        float worldThermalWeight
    ) {
        this.flowSolver = flowSolver;
        this.solverGridSize = Math.max(8, solverGridSize);
        this.updateIntervalTicks = Math.max(1, updateIntervalTicks);
        this.relaxation = MathHelper.clamp(relaxation, 0.01f, 1.0f);
        this.worldThermalWeight = MathHelper.clamp(worldThermalWeight, 0.0f, 1.0f);
    }

    public int solverGridSize() {
        return solverGridSize;
    }

    public int countWindows(Identifier dimensionId) {
        List<DimensionField> dimensionFields = fieldsByDimension.get(dimensionId);
        return dimensionFields == null ? 0 : dimensionFields.size();
    }

    public void clear() {
        for (DimensionField field : fields.values()) {
            releaseContext(field);
        }
        fields.clear();
        fieldsByDimension.clear();
    }

    public void retainDimensions(Set<Identifier> activeDimensions) {
        Iterator<Map.Entry<FieldKey, DimensionField>> iterator = fields.entrySet().iterator();
        while (iterator.hasNext()) {
            Map.Entry<FieldKey, DimensionField> entry = iterator.next();
            if (activeDimensions.contains(entry.getKey().dimensionId)) {
                continue;
            }
            releaseContext(entry.getValue());
            iterator.remove();
        }
        rebuildDimensionIndex();
    }

    public void update(
        Identifier dimensionId,
        long worldTime,
        int simulationTick,
        Collection<WindowSample> windows,
        int windowSize,
        ThermalSampler thermalSampler
    ) {
        if (dimensionId == null || windows == null || windows.isEmpty()) {
            if (dimensionId != null) {
                removeDimensionFields(dimensionId);
            }
            return;
        }

        int targetWindowSize = Math.max(1, windowSize);
        Set<FieldKey> activeKeys = new HashSet<>();
        for (WindowSample window : windows) {
            if (window == null || window.origin() == null) {
                continue;
            }
            FieldKey key = new FieldKey(dimensionId, window.windowKey());
            activeKeys.add(key);

            DimensionField field = fields.get(key);
            BlockPos origin = window.origin();
            if (field == null || !field.matches(origin, targetWindowSize)) {
                DimensionField replacement = new DimensionField(
                    origin.getX(),
                    origin.getY(),
                    origin.getZ(),
                    targetWindowSize,
                    solverGridSize,
                    allocateContextId()
                );
                releaseContext(field);
                field = replacement;
            }

            if (field.lastUpdateTick >= 0 && (simulationTick - field.lastUpdateTick) < updateIntervalTicks) {
                fields.put(key, field);
                continue;
            }

            sampleThermalSources(field, thermalSampler);
            stepThermalField(field);
            boolean solved = stepVelocityField(field);
            commitStep(field, solved, simulationTick);
            fields.put(key, field);
        }

        pruneInactiveWindows(dimensionId, activeKeys);
        rebuildDimensionIndex();
    }

    public void sampleInto(Identifier dimensionId, double worldX, double worldY, double worldZ, float[] output, int offset) {
        if (output == null || output.length < offset + VELOCITY_COMPONENTS) {
            return;
        }

        List<DimensionField> candidates = fieldsByDimension.get(dimensionId);
        if (candidates == null || candidates.isEmpty()) {
            output[offset] = 0.0f;
            output[offset + 1] = 0.0f;
            output[offset + 2] = 0.0f;
            if (output.length >= offset + FIELD_COMPONENTS) {
                output[offset + THERMAL_COMPONENT_INDEX] = 0.0f;
            }
            return;
        }

        DimensionField selected = null;
        double bestContainedDist = Double.POSITIVE_INFINITY;
        double bestAnyDist = Double.POSITIVE_INFINITY;
        for (DimensionField field : candidates) {
            double distSq = field.distanceSqToCenter(worldX, worldY, worldZ);
            if (field.contains(worldX, worldY, worldZ)) {
                if (distSq < bestContainedDist) {
                    bestContainedDist = distSq;
                    selected = field;
                }
                continue;
            }
            if (selected == null && distSq < bestAnyDist) {
                bestAnyDist = distSq;
                selected = field;
            }
        }

        if (selected == null) {
            output[offset] = 0.0f;
            output[offset + 1] = 0.0f;
            output[offset + 2] = 0.0f;
            if (output.length >= offset + FIELD_COMPONENTS) {
                output[offset + THERMAL_COMPONENT_INDEX] = 0.0f;
            }
            return;
        }

        double gx = (worldX - selected.originX) * selected.invCellSize;
        double gy = (worldY - selected.originY) * selected.invCellSize;
        double gz = (worldZ - selected.originZ) * selected.invCellSize;

        gx = MathHelper.clamp(gx, 0.0, selected.solverSize - 1);
        gy = MathHelper.clamp(gy, 0.0, selected.solverSize - 1);
        gz = MathHelper.clamp(gz, 0.0, selected.solverSize - 1);

        output[offset] = sampleComponent(selected.values, selected.solverSize, gx, gy, gz, 0);
        output[offset + 1] = sampleComponent(selected.values, selected.solverSize, gx, gy, gz, 1);
        output[offset + 2] = sampleComponent(selected.values, selected.solverSize, gx, gy, gz, 2);
        if (output.length >= offset + FIELD_COMPONENTS) {
            output[offset + THERMAL_COMPONENT_INDEX] = sampleComponent(
                selected.values,
                selected.solverSize,
                gx,
                gy,
                gz,
                THERMAL_COMPONENT_INDEX
            );
        }
    }

    private void sampleThermalSources(DimensionField field, ThermalSampler thermalSampler) {
        int stride = Math.max(1, THERMAL_SAMPLE_STRIDE);
        for (int x = 0; x < field.solverSize; x += stride) {
            for (int y = 0; y < field.solverSize; y += stride) {
                for (int z = 0; z < field.solverSize; z += stride) {
                    float sampled = 0.0f;
                    if (thermalSampler != null) {
                        double sampleX = field.originX + (x + 0.5 * stride) * field.cellSize;
                        double sampleY = field.originY + (y + 0.5 * stride) * field.cellSize;
                        double sampleZ = field.originZ + (z + 0.5 * stride) * field.cellSize;
                        sampled = thermalSampler.sampleThermalAnomaly(
                            (int) Math.round(sampleX),
                            (int) Math.round(sampleY),
                            (int) Math.round(sampleZ)
                        );
                    }
                    float source = MathHelper.clamp(sampled * worldThermalWeight, -THERMAL_MAX_ABS_K, THERMAL_MAX_ABS_K);
                    int maxX = Math.min(field.solverSize, x + stride);
                    int maxY = Math.min(field.solverSize, y + stride);
                    int maxZ = Math.min(field.solverSize, z + stride);
                    for (int xx = x; xx < maxX; xx++) {
                        for (int yy = y; yy < maxY; yy++) {
                            for (int zz = z; zz < maxZ; zz++) {
                                field.thermalSource[field.cellIndex(xx, yy, zz)] = source;
                            }
                        }
                    }
                }
            }
        }
    }

    private void stepThermalField(DimensionField field) {
        float invCell = field.invCellSize;
        for (int x = 0; x < field.solverSize; x++) {
            for (int y = 0; y < field.solverSize; y++) {
                for (int z = 0; z < field.solverSize; z++) {
                    int cell = field.cellIndex(x, y, z);
                    int idx = cell * FIELD_COMPONENTS;
                    float vx = field.values[idx];
                    float vy = field.values[idx + 1];
                    float vz = field.values[idx + 2];

                    double advectX = x - (THERMAL_ADVECTION * vx * invCell);
                    double advectY = y - (THERMAL_ADVECTION * vy * invCell);
                    double advectZ = z - (THERMAL_ADVECTION * vz * invCell);
                    float advected = sampleComponent(field.values, field.solverSize, advectX, advectY, advectZ, THERMAL_COMPONENT_INDEX);

                    float laplacian = laplacianComponent(field.values, field.solverSize, x, y, z, THERMAL_COMPONENT_INDEX);
                    float diffused = advected + THERMAL_DIFFUSIVITY * laplacian;
                    float source = field.thermalSource[cell];
                    float relaxed = diffused + THERMAL_SOURCE_RELAXATION * (source - diffused);
                    field.thermalNext[cell] = MathHelper.clamp(relaxed, -THERMAL_MAX_ABS_K, THERMAL_MAX_ABS_K);
                }
            }
        }
    }

    private boolean stepVelocityField(DimensionField field) {
        ByteBuffer payload = field.payloadBuffer;
        payload.clear();
        int cells = field.cellCount;
        for (int cell = 0; cell < cells; cell++) {
            int idx = cell * FIELD_COMPONENTS;
            float thermal = field.thermalNext[cell];
            float fanMask = MathHelper.clamp(Math.abs(thermal) / THERMAL_MAX_ABS_K, 0.0f, 1.0f);
            float buoyancyTargetVy = MathHelper.clamp(
                thermal * BUOYANCY_TARGET_SPEED_PER_K,
                -BUOYANCY_TARGET_SPEED_MAX,
                BUOYANCY_TARGET_SPEED_MAX
            );

            payload.putFloat(0.0f);
            payload.putFloat(fanMask);
            payload.putFloat(0.0f);
            payload.putFloat(buoyancyTargetVy);
            payload.putFloat(0.0f);
            payload.putFloat(field.values[idx]);
            payload.putFloat(field.values[idx + 1]);
            payload.putFloat(field.values[idx + 2]);
            payload.putFloat(field.pressure[cell]);
        }
        if (flowSolver == null) {
            return false;
        }
        return flowSolver.step(field.solverSize, field.payloadBytes, field.contextId, field.solverOutput);
    }

    private void commitStep(DimensionField field, boolean solved, int simulationTick) {
        for (int cell = 0; cell < field.cellCount; cell++) {
            int idx = cell * FIELD_COMPONENTS;
            if (solved) {
                int outBase = cell * SOLVER_OUTPUT_CHANNELS;
                float vx = MathHelper.clamp(field.solverOutput[outBase], -VELOCITY_MAX_ABS, VELOCITY_MAX_ABS);
                float vy = MathHelper.clamp(field.solverOutput[outBase + 1], -VELOCITY_MAX_ABS, VELOCITY_MAX_ABS);
                float vz = MathHelper.clamp(field.solverOutput[outBase + 2], -VELOCITY_MAX_ABS, VELOCITY_MAX_ABS);
                field.values[idx] = MathHelper.lerp(relaxation, field.values[idx], vx);
                field.values[idx + 1] = MathHelper.lerp(relaxation, field.values[idx + 1], vy);
                field.values[idx + 2] = MathHelper.lerp(relaxation, field.values[idx + 2], vz);
                field.pressure[cell] = field.solverOutput[outBase + 3];
            } else {
                field.values[idx] *= VELOCITY_FAILURE_DAMPING;
                field.values[idx + 1] *= VELOCITY_FAILURE_DAMPING;
                field.values[idx + 2] *= VELOCITY_FAILURE_DAMPING;
                field.pressure[cell] *= VELOCITY_FAILURE_DAMPING;
            }
            field.values[idx + THERMAL_COMPONENT_INDEX] = field.thermalNext[cell];
        }
        field.lastUpdateTick = simulationTick;
    }

    private float sampleComponent(float[] values, int side, double gx, double gy, double gz, int component) {
        if (side <= 1) {
            return componentAt(values, side, 0, 0, 0, component);
        }
        double clampedX = MathHelper.clamp(gx, 0.0, side - 1);
        double clampedY = MathHelper.clamp(gy, 0.0, side - 1);
        double clampedZ = MathHelper.clamp(gz, 0.0, side - 1);

        int x0 = (int) Math.floor(clampedX);
        int y0 = (int) Math.floor(clampedY);
        int z0 = (int) Math.floor(clampedZ);
        int x1 = Math.min(side - 1, x0 + 1);
        int y1 = Math.min(side - 1, y0 + 1);
        int z1 = Math.min(side - 1, z0 + 1);

        float fx = (float) (clampedX - x0);
        float fy = (float) (clampedY - y0);
        float fz = (float) (clampedZ - z0);

        float c000 = componentAt(values, side, x0, y0, z0, component);
        float c100 = componentAt(values, side, x1, y0, z0, component);
        float c010 = componentAt(values, side, x0, y1, z0, component);
        float c110 = componentAt(values, side, x1, y1, z0, component);
        float c001 = componentAt(values, side, x0, y0, z1, component);
        float c101 = componentAt(values, side, x1, y0, z1, component);
        float c011 = componentAt(values, side, x0, y1, z1, component);
        float c111 = componentAt(values, side, x1, y1, z1, component);

        float c00 = MathHelper.lerp(fx, c000, c100);
        float c10 = MathHelper.lerp(fx, c010, c110);
        float c01 = MathHelper.lerp(fx, c001, c101);
        float c11 = MathHelper.lerp(fx, c011, c111);
        float c0 = MathHelper.lerp(fy, c00, c10);
        float c1 = MathHelper.lerp(fy, c01, c11);
        return MathHelper.lerp(fz, c0, c1);
    }

    private float laplacianComponent(float[] values, int side, int x, int y, int z, int component) {
        float center = componentAt(values, side, x, y, z, component);
        float xp = componentAt(values, side, x + 1, y, z, component);
        float xm = componentAt(values, side, x - 1, y, z, component);
        float yp = componentAt(values, side, x, y + 1, z, component);
        float ym = componentAt(values, side, x, y - 1, z, component);
        float zp = componentAt(values, side, x, y, z + 1, component);
        float zm = componentAt(values, side, x, y, z - 1, component);
        return xp + xm + yp + ym + zp + zm - (6.0f * center);
    }

    private float componentAt(float[] values, int side, int x, int y, int z, int component) {
        int cx = MathHelper.clamp(x, 0, side - 1);
        int cy = MathHelper.clamp(y, 0, side - 1);
        int cz = MathHelper.clamp(z, 0, side - 1);
        int idx = ((cx * side + cy) * side + cz) * FIELD_COMPONENTS + component;
        if (idx < 0 || idx >= values.length) {
            return 0.0f;
        }
        return values[idx];
    }

    private void removeDimensionFields(Identifier dimensionId) {
        Iterator<Map.Entry<FieldKey, DimensionField>> iterator = fields.entrySet().iterator();
        while (iterator.hasNext()) {
            Map.Entry<FieldKey, DimensionField> entry = iterator.next();
            if (!entry.getKey().dimensionId.equals(dimensionId)) {
                continue;
            }
            releaseContext(entry.getValue());
            iterator.remove();
        }
        rebuildDimensionIndex();
    }

    private void pruneInactiveWindows(Identifier dimensionId, Set<FieldKey> activeKeys) {
        Iterator<Map.Entry<FieldKey, DimensionField>> iterator = fields.entrySet().iterator();
        while (iterator.hasNext()) {
            Map.Entry<FieldKey, DimensionField> entry = iterator.next();
            FieldKey key = entry.getKey();
            if (!key.dimensionId.equals(dimensionId) || activeKeys.contains(key)) {
                continue;
            }
            releaseContext(entry.getValue());
            iterator.remove();
        }
    }

    private void rebuildDimensionIndex() {
        Map<Identifier, List<DimensionField>> grouped = new ConcurrentHashMap<>();
        for (Map.Entry<FieldKey, DimensionField> entry : fields.entrySet()) {
            grouped.computeIfAbsent(entry.getKey().dimensionId, ignored -> new ArrayList<>()).add(entry.getValue());
        }
        fieldsByDimension.clear();
        for (Map.Entry<Identifier, List<DimensionField>> entry : grouped.entrySet()) {
            fieldsByDimension.put(entry.getKey(), List.copyOf(entry.getValue()));
        }
    }

    private long allocateContextId() {
        long id = nextContextId++;
        if (nextContextId < CONTEXT_ID_BASE) {
            nextContextId = CONTEXT_ID_BASE;
        }
        return id;
    }

    private void releaseContext(DimensionField field) {
        if (field == null || flowSolver == null) {
            return;
        }
        flowSolver.releaseContext(field.contextId);
    }

    private static final class FieldKey {
        private final Identifier dimensionId;
        private final long windowKey;

        private FieldKey(Identifier dimensionId, long windowKey) {
            this.dimensionId = dimensionId;
            this.windowKey = windowKey;
        }

        @Override
        public boolean equals(Object obj) {
            if (this == obj) {
                return true;
            }
            if (!(obj instanceof FieldKey other)) {
                return false;
            }
            return windowKey == other.windowKey && dimensionId.equals(other.dimensionId);
        }

        @Override
        public int hashCode() {
            int result = dimensionId.hashCode();
            result = 31 * result + Long.hashCode(windowKey);
            return result;
        }
    }

    private static final class DimensionField {
        private final int originX;
        private final int originY;
        private final int originZ;
        private final int windowSize;
        private final int solverSize;
        private final int cellCount;
        private final double cellSize;
        private final float invCellSize;
        private final double centerX;
        private final double centerY;
        private final double centerZ;
        private final long contextId;
        private int lastUpdateTick = -1;

        private final float[] values;
        private final float[] pressure;
        private final float[] thermalSource;
        private final float[] thermalNext;
        private final float[] solverOutput;
        private final byte[] payloadBytes;
        private final ByteBuffer payloadBuffer;

        private DimensionField(
            int originX,
            int originY,
            int originZ,
            int windowSize,
            int solverSize,
            long contextId
        ) {
            this.originX = originX;
            this.originY = originY;
            this.originZ = originZ;
            this.windowSize = Math.max(1, windowSize);
            this.solverSize = Math.max(8, solverSize);
            this.cellCount = this.solverSize * this.solverSize * this.solverSize;
            this.cellSize = this.windowSize / (double) this.solverSize;
            this.invCellSize = (float) (this.solverSize / (double) this.windowSize);
            this.centerX = originX + (this.windowSize * 0.5);
            this.centerY = originY + (this.windowSize * 0.5);
            this.centerZ = originZ + (this.windowSize * 0.5);
            this.contextId = contextId;
            this.values = new float[cellCount * FIELD_COMPONENTS];
            this.pressure = new float[cellCount];
            this.thermalSource = new float[cellCount];
            this.thermalNext = new float[cellCount];
            this.solverOutput = new float[cellCount * SOLVER_OUTPUT_CHANNELS];
            this.payloadBytes = new byte[cellCount * SOLVER_INPUT_CHANNELS * Float.BYTES];
            this.payloadBuffer = ByteBuffer.wrap(payloadBytes).order(ByteOrder.LITTLE_ENDIAN);
        }

        private boolean matches(BlockPos origin, int windowSize) {
            return origin != null
                && this.originX == origin.getX()
                && this.originY == origin.getY()
                && this.originZ == origin.getZ()
                && this.windowSize == Math.max(1, windowSize);
        }

        private boolean contains(double worldX, double worldY, double worldZ) {
            return worldX >= originX
                && worldX < originX + windowSize
                && worldY >= originY
                && worldY < originY + windowSize
                && worldZ >= originZ
                && worldZ < originZ + windowSize;
        }

        private double distanceSqToCenter(double worldX, double worldY, double worldZ) {
            double dx = worldX - centerX;
            double dy = worldY - centerY;
            double dz = worldZ - centerZ;
            return dx * dx + dy * dy + dz * dz;
        }

        private int cellIndex(int x, int y, int z) {
            return (x * solverSize + y) * solverSize + z;
        }
    }
}
