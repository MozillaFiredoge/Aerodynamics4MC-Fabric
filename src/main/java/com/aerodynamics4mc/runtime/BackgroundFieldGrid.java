package com.aerodynamics4mc.runtime;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.Collection;
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

    private static final int DEFAULT_CELL_SIZE = 16;
    private static final int DEFAULT_PADDING_CELLS = 2;
    private static final int DEFAULT_UPDATE_INTERVAL_TICKS = 10;
    private static final float DEFAULT_RELAXATION = 0.2f;
    private static final int DEFAULT_MAX_TOTAL_CELLS = 96_000;
    private static final int DEFAULT_MAX_EFFECTIVE_CELL_SIZE = 256;
    private static final float DEFAULT_WORLD_THERMAL_WEIGHT = 1.0f;

    private static final float THERMAL_SOURCE_RELAXATION = 0.16f;
    private static final float THERMAL_DIFFUSIVITY = 0.08f;
    private static final float THERMAL_ADVECTION = 0.35f;
    private static final float THERMAL_MAX_ABS_K = 8.0f;
    private static final float BUOYANCY_TARGET_SPEED_PER_K = 0.65f;
    private static final float BUOYANCY_TARGET_SPEED_MAX = 6.0f;
    private static final float VELOCITY_FAILURE_DAMPING = 0.92f;
    private static final float VELOCITY_MAX_ABS = 16.0f;

    private static final long CONTEXT_ID_BASE = 1L << 52;

    private final int cellSize;
    private final int paddingCells;
    private final int updateIntervalTicks;
    private final float relaxation;
    private final int maxTotalCells;
    private final int maxEffectiveCellSize;
    private final float worldThermalWeight;
    private final FlowSolver flowSolver;
    private final Map<Identifier, DimensionField> fields = new ConcurrentHashMap<>();
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

    public BackgroundFieldGrid() {
        this(
            null,
            DEFAULT_CELL_SIZE,
            DEFAULT_PADDING_CELLS,
            DEFAULT_UPDATE_INTERVAL_TICKS,
            DEFAULT_RELAXATION,
            DEFAULT_MAX_TOTAL_CELLS,
            DEFAULT_MAX_EFFECTIVE_CELL_SIZE,
            DEFAULT_WORLD_THERMAL_WEIGHT
        );
    }

    public BackgroundFieldGrid(FlowSolver flowSolver) {
        this(
            flowSolver,
            DEFAULT_CELL_SIZE,
            DEFAULT_PADDING_CELLS,
            DEFAULT_UPDATE_INTERVAL_TICKS,
            DEFAULT_RELAXATION,
            DEFAULT_MAX_TOTAL_CELLS,
            DEFAULT_MAX_EFFECTIVE_CELL_SIZE,
            DEFAULT_WORLD_THERMAL_WEIGHT
        );
    }

    BackgroundFieldGrid(int cellSize, int paddingCells, int updateIntervalTicks, float relaxation) {
        this(
            null,
            cellSize,
            paddingCells,
            updateIntervalTicks,
            relaxation,
            DEFAULT_MAX_TOTAL_CELLS,
            DEFAULT_MAX_EFFECTIVE_CELL_SIZE,
            DEFAULT_WORLD_THERMAL_WEIGHT
        );
    }

    BackgroundFieldGrid(
        int cellSize,
        int paddingCells,
        int updateIntervalTicks,
        float relaxation,
        int maxTotalCells,
        int maxEffectiveCellSize,
        float worldThermalWeight
    ) {
        this(
            null,
            cellSize,
            paddingCells,
            updateIntervalTicks,
            relaxation,
            maxTotalCells,
            maxEffectiveCellSize,
            worldThermalWeight
        );
    }

    BackgroundFieldGrid(
        FlowSolver flowSolver,
        int cellSize,
        int paddingCells,
        int updateIntervalTicks,
        float relaxation,
        int maxTotalCells,
        int maxEffectiveCellSize,
        float worldThermalWeight
    ) {
        this.flowSolver = flowSolver;
        this.cellSize = Math.max(4, cellSize);
        this.paddingCells = Math.max(0, paddingCells);
        this.updateIntervalTicks = Math.max(1, updateIntervalTicks);
        this.relaxation = MathHelper.clamp(relaxation, 0.01f, 1.0f);
        this.maxTotalCells = Math.max(1024, maxTotalCells);
        this.maxEffectiveCellSize = Math.max(this.cellSize, maxEffectiveCellSize);
        this.worldThermalWeight = MathHelper.clamp(worldThermalWeight, 0.0f, 1.0f);
    }

    public void clear() {
        for (DimensionField field : fields.values()) {
            releaseContext(field);
        }
        fields.clear();
    }

    public void retainDimensions(Set<Identifier> activeDimensions) {
        for (Map.Entry<Identifier, DimensionField> entry : fields.entrySet()) {
            if (activeDimensions.contains(entry.getKey())) {
                continue;
            }
            if (fields.remove(entry.getKey(), entry.getValue())) {
                releaseContext(entry.getValue());
            }
        }
    }

    public void update(
        Identifier dimensionId,
        long worldTime,
        int simulationTick,
        Collection<BlockPos> windowOrigins,
        int windowSize
    ) {
        update(dimensionId, worldTime, simulationTick, windowOrigins, windowSize, null);
    }

    public void update(
        Identifier dimensionId,
        long worldTime,
        int simulationTick,
        Collection<BlockPos> windowOrigins,
        int windowSize,
        ThermalSampler thermalSampler
    ) {
        if (dimensionId == null || windowOrigins == null || windowOrigins.isEmpty()) {
            if (dimensionId != null) {
                DimensionField removed = fields.remove(dimensionId);
                releaseContext(removed);
            }
            return;
        }

        int minX = Integer.MAX_VALUE;
        int minY = Integer.MAX_VALUE;
        int minZ = Integer.MAX_VALUE;
        int maxX = Integer.MIN_VALUE;
        int maxY = Integer.MIN_VALUE;
        int maxZ = Integer.MIN_VALUE;
        int clampedWindowSize = Math.max(1, windowSize);
        for (BlockPos origin : windowOrigins) {
            if (origin == null) {
                continue;
            }
            minX = Math.min(minX, origin.getX());
            minY = Math.min(minY, origin.getY());
            minZ = Math.min(minZ, origin.getZ());
            maxX = Math.max(maxX, origin.getX() + clampedWindowSize - 1);
            maxY = Math.max(maxY, origin.getY() + clampedWindowSize - 1);
            maxZ = Math.max(maxZ, origin.getZ() + clampedWindowSize - 1);
        }
        if (minX == Integer.MAX_VALUE) {
            DimensionField removed = fields.remove(dimensionId);
            releaseContext(removed);
            return;
        }

        int paddedMinX = minX - (paddingCells * cellSize);
        int paddedMinY = minY - (paddingCells * cellSize);
        int paddedMinZ = minZ - (paddingCells * cellSize);
        int paddedMaxX = maxX + (paddingCells * cellSize);
        int paddedMaxY = maxY + (paddingCells * cellSize);
        int paddedMaxZ = maxZ + (paddingCells * cellSize);

        int spanX = Math.max(1, paddedMaxX - paddedMinX + 1);
        int spanY = Math.max(1, paddedMaxY - paddedMinY + 1);
        int spanZ = Math.max(1, paddedMaxZ - paddedMinZ + 1);

        int effectiveCellSize = cellSize;
        while (effectiveCellSize < maxEffectiveCellSize
            && estimateCubeCellCount(spanX, spanY, spanZ, effectiveCellSize) > maxTotalCells) {
            int next = Math.min(maxEffectiveCellSize, effectiveCellSize * 2);
            if (next == effectiveCellSize) {
                break;
            }
            effectiveCellSize = next;
        }

        int minCellX = Math.floorDiv(paddedMinX, effectiveCellSize);
        int minCellY = Math.floorDiv(paddedMinY, effectiveCellSize);
        int minCellZ = Math.floorDiv(paddedMinZ, effectiveCellSize);
        int maxCellX = Math.floorDiv(paddedMaxX, effectiveCellSize);
        int maxCellY = Math.floorDiv(paddedMaxY, effectiveCellSize);
        int maxCellZ = Math.floorDiv(paddedMaxZ, effectiveCellSize);

        int sizeX = maxCellX - minCellX + 1;
        int sizeY = maxCellY - minCellY + 1;
        int sizeZ = maxCellZ - minCellZ + 1;
        int solverSize = Math.max(sizeX, Math.max(sizeY, sizeZ));
        minCellX -= (solverSize - sizeX) / 2;
        minCellY -= (solverSize - sizeY) / 2;
        minCellZ -= (solverSize - sizeZ) / 2;

        DimensionField previous = fields.get(dimensionId);
        boolean boundsChanged = previous == null
            || !previous.matches(minCellX, minCellY, minCellZ, solverSize, effectiveCellSize);
        if (!boundsChanged
            && previous.lastUpdateTick >= 0
            && (simulationTick - previous.lastUpdateTick) < updateIntervalTicks) {
            return;
        }

        DimensionField field = previous;
        if (boundsChanged) {
            field = new DimensionField(minCellX, minCellY, minCellZ, solverSize, effectiveCellSize, allocateContextId());
            releaseContext(previous);
        }

        sampleThermalSources(field, thermalSampler);
        stepThermalField(field);
        boolean solved = stepVelocityField(field);
        commitStep(field, solved, simulationTick);
        fields.put(dimensionId, field);
    }

    public void sampleInto(Identifier dimensionId, double worldX, double worldY, double worldZ, float[] output, int offset) {
        if (output == null || output.length < offset + VELOCITY_COMPONENTS) {
            return;
        }
        DimensionField field = fields.get(dimensionId);
        if (field == null) {
            output[offset] = 0.0f;
            output[offset + 1] = 0.0f;
            output[offset + 2] = 0.0f;
            if (output.length >= offset + FIELD_COMPONENTS) {
                output[offset + THERMAL_COMPONENT_INDEX] = 0.0f;
            }
            return;
        }

        double gx = (worldX / field.cellSize) - field.minCellX;
        double gy = (worldY / field.cellSize) - field.minCellY;
        double gz = (worldZ / field.cellSize) - field.minCellZ;

        gx = MathHelper.clamp(gx, 0.0, field.solverSize - 1);
        gy = MathHelper.clamp(gy, 0.0, field.solverSize - 1);
        gz = MathHelper.clamp(gz, 0.0, field.solverSize - 1);

        output[offset] = sampleComponent(field.values, field.solverSize, gx, gy, gz, 0);
        output[offset + 1] = sampleComponent(field.values, field.solverSize, gx, gy, gz, 1);
        output[offset + 2] = sampleComponent(field.values, field.solverSize, gx, gy, gz, 2);
        if (output.length >= offset + FIELD_COMPONENTS) {
            output[offset + THERMAL_COMPONENT_INDEX] = sampleComponent(
                field.values,
                field.solverSize,
                gx,
                gy,
                gz,
                THERMAL_COMPONENT_INDEX
            );
        }
    }

    private void sampleThermalSources(DimensionField field, ThermalSampler thermalSampler) {
        for (int x = 0; x < field.solverSize; x++) {
            double worldX = (field.minCellX + x + 0.5) * field.cellSize;
            for (int y = 0; y < field.solverSize; y++) {
                double worldY = (field.minCellY + y + 0.5) * field.cellSize;
                for (int z = 0; z < field.solverSize; z++) {
                    double worldZ = (field.minCellZ + z + 0.5) * field.cellSize;
                    float sampledThermal = 0.0f;
                    if (thermalSampler != null) {
                        sampledThermal = thermalSampler.sampleThermalAnomaly(
                            (int) Math.round(worldX),
                            (int) Math.round(worldY),
                            (int) Math.round(worldZ)
                        );
                    }
                    int cell = field.cellIndex(x, y, z);
                    field.thermalSource[cell] = MathHelper.clamp(
                        sampledThermal * worldThermalWeight,
                        -THERMAL_MAX_ABS_K,
                        THERMAL_MAX_ABS_K
                    );
                }
            }
        }
    }

    private void stepThermalField(DimensionField field) {
        float invCell = 1.0f / Math.max(1.0f, field.cellSize);
        for (int x = 0; x < field.solverSize; x++) {
            for (int y = 0; y < field.solverSize; y++) {
                for (int z = 0; z < field.solverSize; z++) {
                    int cell = field.cellIndex(x, y, z);
                    int fieldIndex = cell * FIELD_COMPONENTS;
                    float vx = field.values[fieldIndex];
                    float vy = field.values[fieldIndex + 1];
                    float vz = field.values[fieldIndex + 2];

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
        int cells = field.solverSize * field.solverSize * field.solverSize;
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
        int cells = field.solverSize * field.solverSize * field.solverSize;
        for (int cell = 0; cell < cells; cell++) {
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
            int idx = component;
            return (idx >= 0 && idx < values.length) ? values[idx] : 0.0f;
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

    private long estimateCubeCellCount(int spanX, int spanY, int spanZ, int candidateCellSize) {
        long sx = Math.max(1L, (spanX + (long) candidateCellSize - 1L) / candidateCellSize);
        long sy = Math.max(1L, (spanY + (long) candidateCellSize - 1L) / candidateCellSize);
        long sz = Math.max(1L, (spanZ + (long) candidateCellSize - 1L) / candidateCellSize);
        long side = Math.max(sx, Math.max(sy, sz));
        return side * side * side;
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

    private static final class DimensionField {
        private final int minCellX;
        private final int minCellY;
        private final int minCellZ;
        private final int solverSize;
        private final int cellSize;
        private final long contextId;
        private int lastUpdateTick = -1;

        private final float[] values;
        private final float[] pressure;
        private final float[] thermalSource;
        private final float[] thermalNext;
        private final float[] solverOutput;
        private final byte[] payloadBytes;
        private final ByteBuffer payloadBuffer;

        private DimensionField(int minCellX, int minCellY, int minCellZ, int solverSize, int cellSize, long contextId) {
            this.minCellX = minCellX;
            this.minCellY = minCellY;
            this.minCellZ = minCellZ;
            this.solverSize = Math.max(1, solverSize);
            this.cellSize = Math.max(1, cellSize);
            this.contextId = contextId;
            int cells = this.solverSize * this.solverSize * this.solverSize;
            this.values = new float[cells * FIELD_COMPONENTS];
            this.pressure = new float[cells];
            this.thermalSource = new float[cells];
            this.thermalNext = new float[cells];
            this.solverOutput = new float[cells * SOLVER_OUTPUT_CHANNELS];
            this.payloadBytes = new byte[cells * SOLVER_INPUT_CHANNELS * Float.BYTES];
            this.payloadBuffer = ByteBuffer.wrap(payloadBytes).order(ByteOrder.LITTLE_ENDIAN);
        }

        private boolean matches(int minCellX, int minCellY, int minCellZ, int solverSize, int cellSize) {
            return this.minCellX == minCellX
                && this.minCellY == minCellY
                && this.minCellZ == minCellZ
                && this.solverSize == solverSize
                && this.cellSize == cellSize;
        }

        private int cellIndex(int x, int y, int z) {
            return (x * solverSize + y) * solverSize + z;
        }
    }
}
