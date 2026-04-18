package com.aerodynamics4mc.runtime;

import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;

import net.minecraft.server.world.ServerWorld;
import net.minecraft.util.math.BlockPos;
import net.minecraft.util.math.MathHelper;

final class MesoscaleGrid implements AutoCloseable {
    private static final float SURFACE_RELAXATION_PER_SECOND = 1.0f / 600.0f;
    private static final float TERRAIN_WIND_DEFLECTION = 0.35f;
    private static final float TERRAIN_TEMPERATURE_OFFSET_K = 4.0f;
    private static final float AMBIENT_LAPSE_RATE_K_PER_BLOCK = 0.0065f;
    private static final float DEFAULT_MOLECULAR_NU_M2_S = 1.5e-5f;
    private static final float DEFAULT_PRANDTL_AIR = 0.71f;
    private static final float DEFAULT_TURBULENT_PRANDTL = 0.85f;
    private static final int NATIVE_FORCING_CHANNELS = 24;
    private static final int NATIVE_STATE_CHANNELS = 5;
    private static final int CH_TERRAIN_HEIGHT = 0;
    private static final int CH_BIOME_TEMPERATURE = 1;
    private static final int CH_AMBIENT_TARGET = 2;
    private static final int CH_DEEP_GROUND_TARGET = 3;
    private static final int CH_SURFACE_TARGET = 4;
    private static final int CH_ROUGHNESS = 5;
    private static final int CH_BACKGROUND_WIND_X = 6;
    private static final int CH_BACKGROUND_WIND_Z = 7;
    private static final int CH_SURFACE_CLASS = 8;
    private static final int CH_HUMIDITY = 9;
    private static final int CH_CONVECTIVE_HEATING = 10;
    private static final int CH_CONVECTIVE_MOISTENING = 11;
    private static final int CH_CONVECTIVE_INFLOW_X = 12;
    private static final int CH_CONVECTIVE_INFLOW_Z = 13;
    private static final int CH_TORNADO_WIND_X = 14;
    private static final int CH_TORNADO_WIND_Z = 15;
    private static final int CH_TORNADO_HEATING = 16;
    private static final int CH_TORNADO_MOISTENING = 17;
    private static final int CH_TORNADO_UPDRAFT = 18;
    private static final int CH_NESTED_UPDRAFT = 19;
    private static final int CH_NESTED_WIND_X_DELTA = 20;
    private static final int CH_NESTED_WIND_Z_DELTA = 21;
    private static final int CH_NESTED_AMBIENT_DELTA = 22;
    private static final int CH_NESTED_SURFACE_DELTA = 23;
    private static final int OUT_AMBIENT = 0;
    private static final int OUT_DEEP_GROUND = 1;
    private static final int OUT_SURFACE = 2;
    private static final int OUT_WIND_X = 3;
    private static final int OUT_WIND_Z = 4;
    private static final float NESTED_FEEDBACK_WIND_BLEND = 0.45f;
    private static final float NESTED_FEEDBACK_AIR_BLEND = 0.35f;
    private static final float NESTED_FEEDBACK_SURFACE_BLEND = 0.30f;
    private static final float NESTED_FEEDBACK_UPDRAFT_BLEND = 0.60f;
    private static final float NESTED_FEEDBACK_MAX_UPDRAFT = 2.5f;

    private final int cellSizeBlocks;
    private final int radiusCells;
    private final int layerHeightBlocks;
    private final int maxLayers;
    private final float stepSeconds;
    private final int forcingRebuildTicks;
    private final Map<Long, CellColumnState> cells = new HashMap<>();
    private final MesoscaleNativeBridge nativeBridge = new MesoscaleNativeBridge();
    private long nativeContextKey;
    private float[] forcingBuffer = new float[0];
    private float[] stateBuffer = new float[0];
    private int centerCellX;
    private int centerCellZ;
    private int activeLayers = 1;
    private int verticalBaseY = 0;
    private long lastTickProcessed = Long.MIN_VALUE;
    private long lastForcingRefreshTick = Long.MIN_VALUE;
    private float accumulatedStepSeconds = 0.0f;
    private boolean forcingReady = false;
    private NestedFeedbackDiagnostics nestedFeedbackDiagnostics = NestedFeedbackDiagnostics.EMPTY;

    MesoscaleGrid(
        int cellSizeBlocks,
        int radiusCells,
        int layerHeightBlocks,
        int maxLayers,
        float stepSeconds,
        int forcingRebuildTicks
    ) {
        this.cellSizeBlocks = cellSizeBlocks;
        this.radiusCells = radiusCells;
        this.layerHeightBlocks = Math.max(1, layerHeightBlocks);
        this.maxLayers = Math.max(1, maxLayers);
        this.stepSeconds = Math.max(1.0e-3f, stepSeconds);
        this.forcingRebuildTicks = Math.max(1, forcingRebuildTicks);
    }

    synchronized void refresh(
        ServerWorld world,
        BlockPos focus,
        long tickCounter,
        float dtSeconds,
        SeedTerrainProvider provider,
        BackgroundMetGrid background
    ) {
        int nextCenterCellX = Math.floorDiv(focus.getX(), cellSizeBlocks);
        int nextCenterCellZ = Math.floorDiv(focus.getZ(), cellSizeBlocks);
        int nextVerticalBaseY = Math.max(0, world.getBottomY());
        int nextActiveLayers = computeActiveLayers(world, nextVerticalBaseY);
        boolean firstRefresh = lastTickProcessed == Long.MIN_VALUE;
        boolean layoutChanged = lastTickProcessed != Long.MIN_VALUE
            && (nextCenterCellX != centerCellX
                || nextCenterCellZ != centerCellZ
                || nextVerticalBaseY != verticalBaseY
                || nextActiveLayers != activeLayers);
        centerCellX = nextCenterCellX;
        centerCellZ = nextCenterCellZ;
        verticalBaseY = nextVerticalBaseY;
        activeLayers = nextActiveLayers;
        float elapsedSeconds = firstRefresh
            ? Math.max(1.0e-3f, dtSeconds)
            : Math.max(1L, tickCounter - lastTickProcessed) * dtSeconds;
        lastTickProcessed = tickCounter;
        accumulatedStepSeconds += elapsedSeconds;

        if (layoutChanged) {
            releaseNativeContext();
            forcingReady = false;
        }

        boolean forcingRebuildDue = !forcingReady
            || firstRefresh
            || layoutChanged
            || lastForcingRefreshTick == Long.MIN_VALUE
            || tickCounter - lastForcingRefreshTick >= forcingRebuildTicks;
        if (forcingRebuildDue) {
            buildForcing(world, provider, background, stepSeconds);
            lastForcingRefreshTick = tickCounter;
            forcingReady = true;
        }

        if (forcingRebuildDue) {
            // Seed a coherent state immediately after forcing refresh instead of waiting for the first native step.
            refreshFallback(0.0f);
        }

        Iterator<Map.Entry<Long, CellColumnState>> it = cells.entrySet().iterator();
        while (it.hasNext()) {
            Map.Entry<Long, CellColumnState> entry = it.next();
            int cx = unpackX(entry.getKey());
            int cz = unpackZ(entry.getKey());
            if (Math.abs(cx - centerCellX) > radiusCells || Math.abs(cz - centerCellZ) > radiusCells) {
                it.remove();
            }
        }
    }

    synchronized Sample sample(BlockPos pos) {
        int cellX = Math.floorDiv(pos.getX(), cellSizeBlocks);
        int cellZ = Math.floorDiv(pos.getZ(), cellSizeBlocks);
        CellColumnState state = cells.get(pack(cellX, cellZ));
        int layer = layerIndexForY(pos.getY());
        if (state == null || layer < 0 || layer >= state.layerCount()) {
            return null;
        }
        return new Sample(
            state.terrainHeightBlocks,
            state.biomeTemperature,
            state.ambientAirTemperatureKelvin[layer],
            state.deepGroundTemperatureKelvin[layer],
            state.surfaceTemperatureKelvin[layer],
            state.roughnessLengthMeters,
            state.windX[layer],
            state.windZ[layer],
            state.humidity[layer],
            state.surfaceClass
        );
    }

    synchronized boolean seedL2Window(
        BlockPos origin,
        int sizeX,
        int sizeY,
        int sizeZ,
        float[] outWindX,
        float[] outWindZ,
        float[] outAirTemperature,
        float[] outSurfaceTemperature
    ) {
        if (cells.isEmpty()
            || activeLayers <= 0
            || sizeX <= 0
            || sizeY <= 0
            || sizeZ <= 0) {
            return false;
        }
        int cellsCount = sizeX * sizeY * sizeZ;
        if (outWindX == null
            || outWindZ == null
            || outAirTemperature == null
            || outSurfaceTemperature == null
            || outWindX.length != cellsCount
            || outWindZ.length != cellsCount
            || outAirTemperature.length != cellsCount
            || outSurfaceTemperature.length != cellsCount) {
            return false;
        }

        int minCellX = centerCellX - radiusCells;
        int maxCellX = centerCellX + radiusCells;
        int minCellZ = centerCellZ - radiusCells;
        int maxCellZ = centerCellZ + radiusCells;

        int[] lowerCellX = new int[sizeX];
        int[] upperCellX = new int[sizeX];
        float[] cellBlendX = new float[sizeX];
        for (int x = 0; x < sizeX; x++) {
            double worldX = origin.getX() + x + 0.5;
            double cellCoord = (worldX - cellSizeBlocks * 0.5) / cellSizeBlocks;
            int lower = MathHelper.floor(cellCoord);
            int upper = lower + 1;
            float blend = (float) (cellCoord - lower);
            lower = MathHelper.clamp(lower, minCellX, maxCellX);
            upper = MathHelper.clamp(upper, minCellX, maxCellX);
            if (lower == upper) {
                blend = 0.0f;
            }
            lowerCellX[x] = lower;
            upperCellX[x] = upper;
            cellBlendX[x] = blend;
        }

        int[] lowerCellZ = new int[sizeZ];
        int[] upperCellZ = new int[sizeZ];
        float[] cellBlendZ = new float[sizeZ];
        for (int z = 0; z < sizeZ; z++) {
            double worldZ = origin.getZ() + z + 0.5;
            double cellCoord = (worldZ - cellSizeBlocks * 0.5) / cellSizeBlocks;
            int lower = MathHelper.floor(cellCoord);
            int upper = lower + 1;
            float blend = (float) (cellCoord - lower);
            lower = MathHelper.clamp(lower, minCellZ, maxCellZ);
            upper = MathHelper.clamp(upper, minCellZ, maxCellZ);
            if (lower == upper) {
                blend = 0.0f;
            }
            lowerCellZ[z] = lower;
            upperCellZ[z] = upper;
            cellBlendZ[z] = blend;
        }

        int[] lowerLayer = new int[sizeY];
        int[] upperLayer = new int[sizeY];
        float[] layerBlend = new float[sizeY];
        for (int y = 0; y < sizeY; y++) {
            double worldY = origin.getY() + y + 0.5;
            double layerCoord = (worldY - (verticalBaseY + layerHeightBlocks * 0.5)) / layerHeightBlocks;
            int lower = MathHelper.floor(layerCoord);
            int upper = lower + 1;
            float blend = (float) (layerCoord - lower);
            lower = MathHelper.clamp(lower, 0, activeLayers - 1);
            upper = MathHelper.clamp(upper, 0, activeLayers - 1);
            if (lower == upper) {
                blend = 0.0f;
            }
            lowerLayer[y] = lower;
            upperLayer[y] = upper;
            layerBlend[y] = blend;
        }

        for (int x = 0; x < sizeX; x++) {
            int x0 = lowerCellX[x];
            int x1 = upperCellX[x];
            float tx = cellBlendX[x];
            for (int z = 0; z < sizeZ; z++) {
                int z0 = lowerCellZ[z];
                int z1 = upperCellZ[z];
                float tz = cellBlendZ[z];

                CellColumnState c00 = columnStateAtClamped(x0, z0);
                CellColumnState c10 = columnStateAtClamped(x1, z0);
                CellColumnState c01 = columnStateAtClamped(x0, z1);
                CellColumnState c11 = columnStateAtClamped(x1, z1);
                if (c00 == null || c10 == null || c01 == null || c11 == null) {
                    return false;
                }

                for (int y = 0; y < sizeY; y++) {
                    int layer0 = lowerLayer[y];
                    int layer1 = upperLayer[y];
                    float ty = layerBlend[y];
                    int cell = (x * sizeY + y) * sizeZ + z;

                    float windX0 = bilerp(
                        c00.windX[layer0],
                        c10.windX[layer0],
                        c01.windX[layer0],
                        c11.windX[layer0],
                        tx,
                        tz
                    );
                    float windX1 = bilerp(
                        c00.windX[layer1],
                        c10.windX[layer1],
                        c01.windX[layer1],
                        c11.windX[layer1],
                        tx,
                        tz
                    );
                    float windZ0 = bilerp(
                        c00.windZ[layer0],
                        c10.windZ[layer0],
                        c01.windZ[layer0],
                        c11.windZ[layer0],
                        tx,
                        tz
                    );
                    float windZ1 = bilerp(
                        c00.windZ[layer1],
                        c10.windZ[layer1],
                        c01.windZ[layer1],
                        c11.windZ[layer1],
                        tx,
                        tz
                    );
                    float air0 = bilerp(
                        c00.ambientAirTemperatureKelvin[layer0],
                        c10.ambientAirTemperatureKelvin[layer0],
                        c01.ambientAirTemperatureKelvin[layer0],
                        c11.ambientAirTemperatureKelvin[layer0],
                        tx,
                        tz
                    );
                    float air1 = bilerp(
                        c00.ambientAirTemperatureKelvin[layer1],
                        c10.ambientAirTemperatureKelvin[layer1],
                        c01.ambientAirTemperatureKelvin[layer1],
                        c11.ambientAirTemperatureKelvin[layer1],
                        tx,
                        tz
                    );
                    float surface0 = bilerp(
                        c00.surfaceTemperatureKelvin[layer0],
                        c10.surfaceTemperatureKelvin[layer0],
                        c01.surfaceTemperatureKelvin[layer0],
                        c11.surfaceTemperatureKelvin[layer0],
                        tx,
                        tz
                    );
                    float surface1 = bilerp(
                        c00.surfaceTemperatureKelvin[layer1],
                        c10.surfaceTemperatureKelvin[layer1],
                        c01.surfaceTemperatureKelvin[layer1],
                        c11.surfaceTemperatureKelvin[layer1],
                        tx,
                        tz
                    );

                    outWindX[cell] = MathHelper.lerp(ty, windX0, windX1);
                    outWindZ[cell] = MathHelper.lerp(ty, windZ0, windZ1);
                    outAirTemperature[cell] = MathHelper.lerp(ty, air0, air1);
                    outSurfaceTemperature[cell] = MathHelper.lerp(ty, surface0, surface1);
                }
            }
        }
        return true;
    }

    synchronized int cellCount() {
        return cells.size() * activeLayers;
    }

    synchronized DiagnosticsSummary diagnosticsSummary(BlockPos focus) {
        if (cells.isEmpty() || activeLayers <= 0) {
            return DiagnosticsSummary.EMPTY;
        }
        int centerX = Math.floorDiv(focus.getX(), cellSizeBlocks);
        int centerZ = Math.floorDiv(focus.getZ(), cellSizeBlocks);
        TornadoEnvironment environment = sampleEnvironment(centerX, centerZ, 2);
        if (environment.sampledStateCount() <= 0) {
            return DiagnosticsSummary.EMPTY;
        }
        return new DiagnosticsSummary(
            environment.meanInstabilityProxy(),
            environment.maxInstabilityProxy(),
            environment.meanLowLevelShear(),
            environment.meanHumidity(),
            environment.meanPositiveMoistureConvergence(),
            environment.maxPositiveMoistureConvergence(),
            environment.meanLiftProxy(),
            environment.maxLiftProxy(),
            environment.sampledStateCount()
        );
    }

    synchronized TornadoEnvironment sampleTornadoEnvironment(BlockPos pos, int radiusCells) {
        if (cells.isEmpty() || activeLayers <= 0) {
            return TornadoEnvironment.EMPTY;
        }
        int centerX = Math.floorDiv(pos.getX(), cellSizeBlocks);
        int centerZ = Math.floorDiv(pos.getZ(), cellSizeBlocks);
        return sampleEnvironment(centerX, centerZ, Math.max(0, radiusCells));
    }

    synchronized Snapshot snapshot() {
        int gridWidth = radiusCells * 2 + 1;
        int columnCount = gridWidth * gridWidth;
        int stateCount = columnCount * activeLayers;
        float[] terrainHeightBlocks = new float[columnCount];
        float[] biomeTemperature = new float[columnCount];
        float[] roughnessLengthMeters = new float[columnCount];
        byte[] surfaceClass = new byte[columnCount];
        float[] ambientAirTemperatureKelvin = new float[stateCount];
        float[] deepGroundTemperatureKelvin = new float[stateCount];
        float[] surfaceTemperatureKelvin = new float[stateCount];
        float[] forcingAmbientTargetKelvin = new float[stateCount];
        float[] forcingSurfaceTargetKelvin = new float[stateCount];
        float[] forcingBackgroundWindX = new float[stateCount];
        float[] forcingBackgroundWindZ = new float[stateCount];
        float[] forcingNestedAmbientDeltaKelvin = new float[stateCount];
        float[] forcingNestedSurfaceDeltaKelvin = new float[stateCount];
        float[] forcingNestedWindXDelta = new float[stateCount];
        float[] forcingNestedWindZDelta = new float[stateCount];
        float[] forcingNestedUpdraft = new float[stateCount];
        float[] windX = new float[stateCount];
        float[] windZ = new float[stateCount];
        float[] humidity = new float[stateCount];
        float[] convectiveHeating = new float[stateCount];
        float[] convectiveMoistening = new float[stateCount];
        float[] convectiveInflowX = new float[stateCount];
        float[] convectiveInflowZ = new float[stateCount];
        float[] tornadoHeating = new float[stateCount];
        float[] tornadoMoistening = new float[stateCount];
        float[] tornadoWindX = new float[stateCount];
        float[] tornadoWindZ = new float[stateCount];
        float[] tornadoUpdraft = new float[stateCount];
        float[] instabilityProxy = new float[stateCount];
        float[] lowLevelShear = new float[stateCount];
        float[] moistureConvergence = new float[stateCount];
        float[] liftProxy = new float[stateCount];

        for (int cx = centerCellX - radiusCells; cx <= centerCellX + radiusCells; cx++) {
            int localX = cx - (centerCellX - radiusCells);
            for (int cz = centerCellZ - radiusCells; cz <= centerCellZ + radiusCells; cz++) {
                int localZ = cz - (centerCellZ - radiusCells);
                int columnIndex = localX * gridWidth + localZ;
                CellColumnState cell = cells.get(pack(cx, cz));
                if (cell == null) {
                    continue;
                }
                terrainHeightBlocks[columnIndex] = cell.terrainHeightBlocks;
                biomeTemperature[columnIndex] = cell.biomeTemperature;
                roughnessLengthMeters[columnIndex] = cell.roughnessLengthMeters;
                surfaceClass[columnIndex] = cell.surfaceClass;
                int layers = Math.min(activeLayers, cell.layerCount());
                for (int layer = 0; layer < layers; layer++) {
                    int stateIndex = snapshotIndex(localX, layer, localZ, gridWidth);
                    int forcingBase = forcingIndex(cx, layer, cz, gridWidth) * NATIVE_FORCING_CHANNELS;
                    ambientAirTemperatureKelvin[stateIndex] = cell.ambientAirTemperatureKelvin[layer];
                    deepGroundTemperatureKelvin[stateIndex] = cell.deepGroundTemperatureKelvin[layer];
                    surfaceTemperatureKelvin[stateIndex] = cell.surfaceTemperatureKelvin[layer];
                    forcingAmbientTargetKelvin[stateIndex] = forcingBase >= 0 && forcingBase + CH_AMBIENT_TARGET < forcingBuffer.length
                        ? forcingBuffer[forcingBase + CH_AMBIENT_TARGET]
                        : 0.0f;
                    forcingSurfaceTargetKelvin[stateIndex] = forcingBase >= 0 && forcingBase + CH_SURFACE_TARGET < forcingBuffer.length
                        ? forcingBuffer[forcingBase + CH_SURFACE_TARGET]
                        : 0.0f;
                    forcingBackgroundWindX[stateIndex] = forcingBase >= 0 && forcingBase + CH_BACKGROUND_WIND_X < forcingBuffer.length
                        ? forcingBuffer[forcingBase + CH_BACKGROUND_WIND_X]
                        : 0.0f;
                    forcingBackgroundWindZ[stateIndex] = forcingBase >= 0 && forcingBase + CH_BACKGROUND_WIND_Z < forcingBuffer.length
                        ? forcingBuffer[forcingBase + CH_BACKGROUND_WIND_Z]
                        : 0.0f;
                    forcingNestedAmbientDeltaKelvin[stateIndex] = forcingBase >= 0 && forcingBase + CH_NESTED_AMBIENT_DELTA < forcingBuffer.length
                        ? forcingBuffer[forcingBase + CH_NESTED_AMBIENT_DELTA]
                        : 0.0f;
                    forcingNestedSurfaceDeltaKelvin[stateIndex] = forcingBase >= 0 && forcingBase + CH_NESTED_SURFACE_DELTA < forcingBuffer.length
                        ? forcingBuffer[forcingBase + CH_NESTED_SURFACE_DELTA]
                        : 0.0f;
                    forcingNestedWindXDelta[stateIndex] = forcingBase >= 0 && forcingBase + CH_NESTED_WIND_X_DELTA < forcingBuffer.length
                        ? forcingBuffer[forcingBase + CH_NESTED_WIND_X_DELTA]
                        : 0.0f;
                    forcingNestedWindZDelta[stateIndex] = forcingBase >= 0 && forcingBase + CH_NESTED_WIND_Z_DELTA < forcingBuffer.length
                        ? forcingBuffer[forcingBase + CH_NESTED_WIND_Z_DELTA]
                        : 0.0f;
                    forcingNestedUpdraft[stateIndex] = forcingBase >= 0 && forcingBase + CH_NESTED_UPDRAFT < forcingBuffer.length
                        ? forcingBuffer[forcingBase + CH_NESTED_UPDRAFT]
                        : 0.0f;
                    windX[stateIndex] = cell.windX[layer];
                    windZ[stateIndex] = cell.windZ[layer];
                    humidity[stateIndex] = cell.humidity[layer];
                    convectiveHeating[stateIndex] = forcingBase >= 0 && forcingBase + CH_CONVECTIVE_HEATING < forcingBuffer.length
                        ? forcingBuffer[forcingBase + CH_CONVECTIVE_HEATING]
                        : 0.0f;
                    convectiveMoistening[stateIndex] = forcingBase >= 0 && forcingBase + CH_CONVECTIVE_MOISTENING < forcingBuffer.length
                        ? forcingBuffer[forcingBase + CH_CONVECTIVE_MOISTENING]
                        : 0.0f;
                    convectiveInflowX[stateIndex] = forcingBase >= 0 && forcingBase + CH_CONVECTIVE_INFLOW_X < forcingBuffer.length
                        ? forcingBuffer[forcingBase + CH_CONVECTIVE_INFLOW_X]
                        : 0.0f;
                    convectiveInflowZ[stateIndex] = forcingBase >= 0 && forcingBase + CH_CONVECTIVE_INFLOW_Z < forcingBuffer.length
                        ? forcingBuffer[forcingBase + CH_CONVECTIVE_INFLOW_Z]
                        : 0.0f;
                    tornadoWindX[stateIndex] = forcingBase >= 0 && forcingBase + CH_TORNADO_WIND_X < forcingBuffer.length
                        ? forcingBuffer[forcingBase + CH_TORNADO_WIND_X]
                        : 0.0f;
                    tornadoWindZ[stateIndex] = forcingBase >= 0 && forcingBase + CH_TORNADO_WIND_Z < forcingBuffer.length
                        ? forcingBuffer[forcingBase + CH_TORNADO_WIND_Z]
                        : 0.0f;
                    tornadoHeating[stateIndex] = forcingBase >= 0 && forcingBase + CH_TORNADO_HEATING < forcingBuffer.length
                        ? forcingBuffer[forcingBase + CH_TORNADO_HEATING]
                        : 0.0f;
                    tornadoMoistening[stateIndex] = forcingBase >= 0 && forcingBase + CH_TORNADO_MOISTENING < forcingBuffer.length
                        ? forcingBuffer[forcingBase + CH_TORNADO_MOISTENING]
                        : 0.0f;
                    tornadoUpdraft[stateIndex] = forcingBase >= 0 && forcingBase + CH_TORNADO_UPDRAFT < forcingBuffer.length
                        ? forcingBuffer[forcingBase + CH_TORNADO_UPDRAFT]
                            + (forcingBase + CH_NESTED_UPDRAFT < forcingBuffer.length
                                ? forcingBuffer[forcingBase + CH_NESTED_UPDRAFT]
                                : 0.0f)
                        : 0.0f;
                }
            }
        }

        computeDiagnostics(
            gridWidth,
            activeLayers,
            ambientAirTemperatureKelvin,
            deepGroundTemperatureKelvin,
            surfaceTemperatureKelvin,
            windX,
            windZ,
            humidity,
            convectiveHeating,
            convectiveMoistening,
            convectiveInflowX,
            convectiveInflowZ,
            tornadoHeating,
            tornadoMoistening,
            tornadoWindX,
            tornadoWindZ,
            tornadoUpdraft,
            instabilityProxy,
            lowLevelShear,
            moistureConvergence,
            liftProxy
        );

        return new Snapshot(
            gridWidth,
            activeLayers,
            cellSizeBlocks,
            layerHeightBlocks,
            radiusCells,
            centerCellX,
            centerCellZ,
            verticalBaseY,
            stepSeconds,
            lastTickProcessed,
            terrainHeightBlocks,
            biomeTemperature,
            roughnessLengthMeters,
            surfaceClass,
            ambientAirTemperatureKelvin,
            deepGroundTemperatureKelvin,
            surfaceTemperatureKelvin,
            forcingAmbientTargetKelvin,
            forcingSurfaceTargetKelvin,
            forcingBackgroundWindX,
            forcingBackgroundWindZ,
            forcingNestedAmbientDeltaKelvin,
            forcingNestedSurfaceDeltaKelvin,
            forcingNestedWindXDelta,
            forcingNestedWindZDelta,
            forcingNestedUpdraft,
            windX,
            windZ,
            humidity,
            instabilityProxy,
            lowLevelShear,
            moistureConvergence,
            liftProxy,
            nestedFeedbackDiagnostics
        );
    }

    @Override
    public synchronized void close() {
        releaseNativeContext();
        cells.clear();
        forcingBuffer = new float[0];
        stateBuffer = new float[0];
        lastTickProcessed = Long.MIN_VALUE;
        lastForcingRefreshTick = Long.MIN_VALUE;
        accumulatedStepSeconds = 0.0f;
        forcingReady = false;
        nestedFeedbackDiagnostics = NestedFeedbackDiagnostics.EMPTY;
    }

    synchronized void runPendingSteps() {
        if (!forcingReady) {
            return;
        }
        int stepsToRun = Math.min(4, (int) Math.floor(accumulatedStepSeconds / stepSeconds));
        if (stepsToRun <= 0) {
            return;
        }
        for (int i = 0; i < stepsToRun; i++) {
            if (!stepNative(stepSeconds)) {
                refreshFallback(stepSeconds);
            }
        }
        accumulatedStepSeconds -= stepsToRun * stepSeconds;
    }

    synchronized void applyPendingNestedFeedback(Iterable<NestedFeedbackBin> feedbackBins) {
        if (feedbackBins == null || activeLayers <= 0 || forcingBuffer.length == 0) {
            return;
        }
        Map<NestedFeedbackKey, NestedFeedbackAccumulator> aggregates = new HashMap<>();
        int inputBinCount = 0;
        int acceptedBinCount = 0;
        for (NestedFeedbackBin bin : feedbackBins) {
            inputBinCount++;
            if (bin == null
                || !(bin.volumeAverage() > 0.0f)
                || !Float.isFinite(bin.volumeAverage())
                || !Float.isFinite(bin.densityAverage())
                || !Float.isFinite(bin.momentumXAverage())
                || !Float.isFinite(bin.momentumZAverage())
                || !Float.isFinite(bin.airTemperatureVolumeAverage())
                || !Float.isFinite(bin.surfaceTemperatureVolumeAverage())
                || !Float.isFinite(bin.bottomAreaAverage())
                || !Float.isFinite(bin.bottomMassFluxAverage())
                || !Float.isFinite(bin.topAreaAverage())
                || !Float.isFinite(bin.topMassFluxAverage())) {
                continue;
            }
            int layer = MathHelper.clamp(bin.layer(), 0, activeLayers - 1);
            if (bin.cellX() < centerCellX - radiusCells
                || bin.cellX() > centerCellX + radiusCells
                || bin.cellZ() < centerCellZ - radiusCells
                || bin.cellZ() > centerCellZ + radiusCells) {
                continue;
            }
            NestedFeedbackKey key = new NestedFeedbackKey(bin.cellX(), layer, bin.cellZ());
            aggregates.computeIfAbsent(key, ignored -> new NestedFeedbackAccumulator()).add(bin);
            acceptedBinCount++;
        }
        if (aggregates.isEmpty()) {
            nestedFeedbackDiagnostics = new NestedFeedbackDiagnostics(
                lastTickProcessed,
                inputBinCount,
                acceptedBinCount,
                0,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.0f
            );
            return;
        }

        int gridWidth = radiusCells * 2 + 1;
        float coarseCellVolume = Math.max(1.0f, cellSizeBlocks * cellSizeBlocks * layerHeightBlocks);
        int appliedCellCount = 0;
        float coverageSum = 0.0f;
        float maxCoverage = 0.0f;
        float windDeltaSum = 0.0f;
        float maxWindDelta = 0.0f;
        float airDeltaSum = 0.0f;
        float maxAirDelta = 0.0f;
        float surfaceDeltaSum = 0.0f;
        float maxSurfaceDelta = 0.0f;
        float bottomFluxDensitySum = 0.0f;
        float topFluxDensitySum = 0.0f;
        float nestedUpdraftSum = 0.0f;
        float maxAbsNestedUpdraft = 0.0f;
        for (Map.Entry<NestedFeedbackKey, NestedFeedbackAccumulator> entry : aggregates.entrySet()) {
            NestedFeedbackKey key = entry.getKey();
            NestedFeedbackAccumulator aggregate = entry.getValue();
            float densityAverage = Math.max(1.0e-6f, aggregate.densityAverage);
            float coverage = MathHelper.clamp(aggregate.volumeAverage / coarseCellVolume, 0.0f, 1.0f);
            if (coverage <= 0.0f || !Float.isFinite(densityAverage) || !Float.isFinite(coverage)) {
                continue;
            }

            float meanWindX = aggregate.momentumXAverage / densityAverage;
            float meanWindZ = aggregate.momentumZAverage / densityAverage;
            float meanAirTemperature = aggregate.airTemperatureVolumeAverage / Math.max(1.0e-6f, aggregate.volumeAverage);
            float meanSurfaceTemperature = aggregate.surfaceTemperatureVolumeAverage / Math.max(1.0e-6f, aggregate.volumeAverage);
            float bottomFluxDensity = aggregate.bottomAreaAverage > 0.0f
                ? aggregate.bottomMassFluxAverage / aggregate.bottomAreaAverage
                : 0.0f;
            float topFluxDensity = aggregate.topAreaAverage > 0.0f
                ? aggregate.topMassFluxAverage / aggregate.topAreaAverage
                : 0.0f;
            float meanVerticalVelocity = 0.5f * (bottomFluxDensity + topFluxDensity) / densityAverage;
            float nestedUpdraft = MathHelper.clamp(meanVerticalVelocity, -NESTED_FEEDBACK_MAX_UPDRAFT, NESTED_FEEDBACK_MAX_UPDRAFT);
            if (!Float.isFinite(meanWindX)
                || !Float.isFinite(meanWindZ)
                || !Float.isFinite(meanAirTemperature)
                || !Float.isFinite(bottomFluxDensity)
                || !Float.isFinite(topFluxDensity)
                || !Float.isFinite(nestedUpdraft)) {
                continue;
            }

            int base = forcingIndex(key.cellX(), key.layer(), key.cellZ(), gridWidth) * NATIVE_FORCING_CHANNELS;
            float windBlend = MathHelper.clamp(NESTED_FEEDBACK_WIND_BLEND * coverage, 0.0f, 1.0f);
            float airBlend = MathHelper.clamp(NESTED_FEEDBACK_AIR_BLEND * coverage, 0.0f, 1.0f);
            float surfaceBlend = MathHelper.clamp(NESTED_FEEDBACK_SURFACE_BLEND * coverage, 0.0f, 1.0f);
            float updraftBlend = MathHelper.clamp(NESTED_FEEDBACK_UPDRAFT_BLEND * coverage, 0.0f, 1.0f);

            float previousForcingWindX = finiteOrDefault(forcingBuffer[base + CH_BACKGROUND_WIND_X], 0.0f);
            float previousForcingWindZ = finiteOrDefault(forcingBuffer[base + CH_BACKGROUND_WIND_Z], 0.0f);
            float previousForcingAmbient = finiteOrDefault(forcingBuffer[base + CH_AMBIENT_TARGET], meanAirTemperature);
            float previousForcingSurface = finiteOrDefault(forcingBuffer[base + CH_SURFACE_TARGET], meanSurfaceTemperature);
            float previousNestedWindXDelta = finiteOrDefault(forcingBuffer[base + CH_NESTED_WIND_X_DELTA], 0.0f);
            float previousNestedWindZDelta = finiteOrDefault(forcingBuffer[base + CH_NESTED_WIND_Z_DELTA], 0.0f);
            float previousNestedAmbientDelta = finiteOrDefault(forcingBuffer[base + CH_NESTED_AMBIENT_DELTA], 0.0f);
            float previousNestedSurfaceDelta = finiteOrDefault(forcingBuffer[base + CH_NESTED_SURFACE_DELTA], 0.0f);
            float previousNestedUpdraft = finiteOrDefault(forcingBuffer[base + CH_NESTED_UPDRAFT], 0.0f);
            float targetNestedWindXDelta = meanWindX - previousForcingWindX;
            float targetNestedWindZDelta = meanWindZ - previousForcingWindZ;
            float targetNestedAmbientDelta = meanAirTemperature - previousForcingAmbient;

            // Feed back into dedicated nested-delta channels only. The base background
            // and target channels stay owned by the coarse-scale model.
            forcingBuffer[base + CH_NESTED_WIND_X_DELTA] = MathHelper.lerp(windBlend, previousNestedWindXDelta, targetNestedWindXDelta);
            forcingBuffer[base + CH_NESTED_WIND_Z_DELTA] = MathHelper.lerp(windBlend, previousNestedWindZDelta, targetNestedWindZDelta);
            forcingBuffer[base + CH_NESTED_AMBIENT_DELTA] = MathHelper.lerp(airBlend, previousNestedAmbientDelta, targetNestedAmbientDelta);
            forcingBuffer[base + CH_NESTED_UPDRAFT] = MathHelper.lerp(updraftBlend, previousNestedUpdraft, nestedUpdraft);

            float appliedSurfaceDelta = 0.0f;
            if (Float.isFinite(meanSurfaceTemperature) && meanSurfaceTemperature > 0.0f) {
                float targetNestedSurfaceDelta = meanSurfaceTemperature - previousForcingSurface;
                forcingBuffer[base + CH_NESTED_SURFACE_DELTA] = MathHelper.lerp(surfaceBlend, previousNestedSurfaceDelta, targetNestedSurfaceDelta);
                appliedSurfaceDelta = Math.abs(forcingBuffer[base + CH_NESTED_SURFACE_DELTA] - previousNestedSurfaceDelta);
            }

            float appliedWindDelta = (float) Math.sqrt(
                (forcingBuffer[base + CH_NESTED_WIND_X_DELTA] - previousNestedWindXDelta)
                    * (forcingBuffer[base + CH_NESTED_WIND_X_DELTA] - previousNestedWindXDelta)
                    + (forcingBuffer[base + CH_NESTED_WIND_Z_DELTA] - previousNestedWindZDelta)
                    * (forcingBuffer[base + CH_NESTED_WIND_Z_DELTA] - previousNestedWindZDelta)
            );
            float appliedAirDelta = Math.abs(forcingBuffer[base + CH_NESTED_AMBIENT_DELTA] - previousNestedAmbientDelta);
            float appliedNestedUpdraft = forcingBuffer[base + CH_NESTED_UPDRAFT];

            appliedCellCount++;
            coverageSum += coverage;
            maxCoverage = Math.max(maxCoverage, coverage);
            windDeltaSum += appliedWindDelta;
            maxWindDelta = Math.max(maxWindDelta, appliedWindDelta);
            airDeltaSum += appliedAirDelta;
            maxAirDelta = Math.max(maxAirDelta, appliedAirDelta);
            surfaceDeltaSum += appliedSurfaceDelta;
            maxSurfaceDelta = Math.max(maxSurfaceDelta, appliedSurfaceDelta);
            bottomFluxDensitySum += bottomFluxDensity;
            topFluxDensitySum += topFluxDensity;
            nestedUpdraftSum += appliedNestedUpdraft;
            maxAbsNestedUpdraft = Math.max(maxAbsNestedUpdraft, Math.max(Math.abs(appliedNestedUpdraft), Math.abs(previousNestedUpdraft)));
        }

        if (appliedCellCount <= 0) {
            nestedFeedbackDiagnostics = new NestedFeedbackDiagnostics(
                lastTickProcessed,
                inputBinCount,
                acceptedBinCount,
                0,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.0f
            );
            return;
        }

        float appliedCellCountInv = 1.0f / appliedCellCount;
        nestedFeedbackDiagnostics = new NestedFeedbackDiagnostics(
            lastTickProcessed,
            inputBinCount,
            acceptedBinCount,
            appliedCellCount,
            coverageSum * appliedCellCountInv,
            maxCoverage,
            windDeltaSum * appliedCellCountInv,
            maxWindDelta,
            airDeltaSum * appliedCellCountInv,
            maxAirDelta,
            surfaceDeltaSum * appliedCellCountInv,
            maxSurfaceDelta,
            bottomFluxDensitySum * appliedCellCountInv,
            topFluxDensitySum * appliedCellCountInv,
            nestedUpdraftSum * appliedCellCountInv,
            maxAbsNestedUpdraft
        );
    }

    synchronized NestedFeedbackDiagnostics nestedFeedbackDiagnostics() {
        return nestedFeedbackDiagnostics;
    }

    private int cellCenterBlock(int cell) {
        return cell * cellSizeBlocks + cellSizeBlocks / 2;
    }

    private void buildForcing(
        ServerWorld world,
        SeedTerrainProvider provider,
        BackgroundMetGrid background,
        float deltaSeconds
    ) {
        int gridWidth = radiusCells * 2 + 1;
        int cellCount = gridWidth * activeLayers * gridWidth;
        ensureNativeBuffers(cellCount);
        MesoscaleNativeBridge.Transport transport = nativeBridge.deriveTransport(
            gridWidth,
            activeLayers,
            gridWidth,
            cellSizeBlocks,
            stepSeconds,
            DEFAULT_MOLECULAR_NU_M2_S,
            DEFAULT_PRANDTL_AIR,
            DEFAULT_TURBULENT_PRANDTL
        );
        float maxBackgroundWind = transport != null
            ? 0.25f * transport.velocityScaleMetersPerSecond()
            : 32.0f;

        for (int cx = centerCellX - radiusCells; cx <= centerCellX + radiusCells; cx++) {
            for (int cz = centerCellZ - radiusCells; cz <= centerCellZ + radiusCells; cz++) {
                long key = pack(cx, cz);
                int sampleX = cellCenterBlock(cx);
                int sampleZ = cellCenterBlock(cz);
                CellColumnState cell = cells.computeIfAbsent(key, ignored -> new CellColumnState(activeLayers));
                cell.ensureLayers(activeLayers);
                ensureStaticColumnState(cell, world, provider, sampleX, sampleZ);
                BackgroundMetGrid.Sample bg = background.sample(new BlockPos(sampleX, world.getSeaLevel(), sampleZ));

                float ambientAirTemperatureKelvin = bg != null
                    ? bg.ambientAirTemperatureKelvin()
                    : 288.15f;
                float deepGroundTemperatureKelvin = bg != null
                    ? bg.deepGroundTemperatureKelvin()
                    : ambientAirTemperatureKelvin + 1.5f;
                float terrainDelta = cell.terrainHeightBlocks - world.getSeaLevel();
                float temperatureAdjustment = -Math.max(0.0f, terrainDelta) * 0.01f * TERRAIN_TEMPERATURE_OFFSET_K;
                float terrainSteer = MathHelper.clamp(terrainDelta / 96.0f, -1.0f, 1.0f);
                float bgWindX = bg != null ? bg.backgroundWindX() : 0.0f;
                float bgWindZ = bg != null ? bg.backgroundWindZ() : 0.0f;
                float bgHumidity = bg != null ? bg.humidity() : 0.50f;
                float bgConvectiveHeating = bg != null ? bg.convectiveHeatingKelvin() : 0.0f;
                float bgConvectiveMoistening = bg != null ? bg.convectiveMoistening() : 0.0f;
                float bgConvectiveInflowX = bg != null ? bg.convectiveInflowX() : 0.0f;
                float bgConvectiveInflowZ = bg != null ? bg.convectiveInflowZ() : 0.0f;
                float bgConvectiveEnvelope = bg != null ? bg.convectiveEnvelope() : 0.0f;
                float bgTornadoWindX = bg != null ? bg.tornadoWindX() : 0.0f;
                float bgTornadoWindZ = bg != null ? bg.tornadoWindZ() : 0.0f;
                float bgTornadoHeating = bg != null ? bg.tornadoHeatingKelvin() : 0.0f;
                float bgTornadoMoistening = bg != null ? bg.tornadoMoistening() : 0.0f;
                float bgTornadoUpdraft = bg != null ? bg.tornadoUpdraftProxy() : 0.0f;
                float targetWindX = bgWindX * (1.0f - Math.abs(terrainSteer) * 0.25f) - terrainSteer * TERRAIN_WIND_DEFLECTION * bgWindZ;
                float targetWindZ = bgWindZ * (1.0f - Math.abs(terrainSteer) * 0.25f) + terrainSteer * TERRAIN_WIND_DEFLECTION * bgWindX;
                targetWindX = MathHelper.clamp(targetWindX, -maxBackgroundWind, maxBackgroundWind);
                targetWindZ = MathHelper.clamp(targetWindZ, -maxBackgroundWind, maxBackgroundWind);

                for (int layer = 0; layer < activeLayers; layer++) {
                    int sampleY = layerCenterBlockY(layer);
                    float layerHeightAboveBase = Math.max(0.0f, sampleY - verticalBaseY);
                    float layerAmbient = ambientAirTemperatureKelvin + temperatureAdjustment
                        - Math.max(0.0f, sampleY - world.getSeaLevel()) * AMBIENT_LAPSE_RATE_K_PER_BLOCK;
                    float layerDeep = deepGroundTemperatureKelvin + temperatureAdjustment * 0.5f;
                    float aboveGround = Math.max(0.0f, sampleY - cell.terrainHeightBlocks);
                    float surfaceInfluence = (float) Math.exp(-aboveGround / Math.max(1.0f, layerHeightBlocks * 1.5f));
                    float layerSurfaceTarget = layerAmbient + surfaceInfluence
                        * (((bg != null ? bg.surfaceTemperatureKelvin() : ambientAirTemperatureKelvin) + temperatureAdjustment) - layerAmbient);
                    float shearFactor = 0.75f + 0.45f * (float) Math.sqrt(MathHelper.clamp(layerHeightAboveBase / 320.0f, 0.0f, 1.0f));
                    float roughnessDecay = 1.0f / (1.0f + aboveGround / 64.0f);
                    float humidityDecay = 0.85f + 0.15f * surfaceInfluence;
                    float layerHumidity = MathHelper.clamp(bgHumidity * humidityDecay, 0.0f, 1.0f);
                    float convectiveLayerWeight = bgConvectiveEnvelope
                        * surfaceInfluence
                        * (1.0f - 0.60f * (layer / (float) Math.max(1, activeLayers - 1)));
                    float tornadoLayerWeight = surfaceInfluence
                        * (1.0f - 0.75f * (layer / (float) Math.max(1, activeLayers - 1)));
                    int base = forcingIndex(cx, layer, cz, gridWidth) * NATIVE_FORCING_CHANNELS;
                    forcingBuffer[base + CH_TERRAIN_HEIGHT] = cell.terrainHeightBlocks;
                    forcingBuffer[base + CH_BIOME_TEMPERATURE] = cell.biomeTemperature;
                    forcingBuffer[base + CH_AMBIENT_TARGET] = layerAmbient;
                    forcingBuffer[base + CH_DEEP_GROUND_TARGET] = layerDeep;
                    forcingBuffer[base + CH_SURFACE_TARGET] = layerSurfaceTarget;
                    forcingBuffer[base + CH_ROUGHNESS] = cell.roughnessLengthMeters * roughnessDecay;
                    forcingBuffer[base + CH_BACKGROUND_WIND_X] = targetWindX * shearFactor;
                    forcingBuffer[base + CH_BACKGROUND_WIND_Z] = targetWindZ * shearFactor;
                    forcingBuffer[base + CH_SURFACE_CLASS] = cell.surfaceClass;
                    forcingBuffer[base + CH_HUMIDITY] = layerHumidity;
                    forcingBuffer[base + CH_CONVECTIVE_HEATING] = bgConvectiveHeating * convectiveLayerWeight;
                    forcingBuffer[base + CH_CONVECTIVE_MOISTENING] = bgConvectiveMoistening * convectiveLayerWeight;
                    forcingBuffer[base + CH_CONVECTIVE_INFLOW_X] = bgConvectiveInflowX * convectiveLayerWeight;
                    forcingBuffer[base + CH_CONVECTIVE_INFLOW_Z] = bgConvectiveInflowZ * convectiveLayerWeight;
                    forcingBuffer[base + CH_TORNADO_WIND_X] = bgTornadoWindX * tornadoLayerWeight;
                    forcingBuffer[base + CH_TORNADO_WIND_Z] = bgTornadoWindZ * tornadoLayerWeight;
                    forcingBuffer[base + CH_TORNADO_HEATING] = bgTornadoHeating * tornadoLayerWeight;
                    forcingBuffer[base + CH_TORNADO_MOISTENING] = bgTornadoMoistening * tornadoLayerWeight;
                    forcingBuffer[base + CH_TORNADO_UPDRAFT] = bgTornadoUpdraft * tornadoLayerWeight;
                    forcingBuffer[base + CH_NESTED_UPDRAFT] = 0.0f;
                    forcingBuffer[base + CH_NESTED_WIND_X_DELTA] = 0.0f;
                    forcingBuffer[base + CH_NESTED_WIND_Z_DELTA] = 0.0f;
                    forcingBuffer[base + CH_NESTED_AMBIENT_DELTA] = 0.0f;
                    forcingBuffer[base + CH_NESTED_SURFACE_DELTA] = 0.0f;
                    cell.humidity[layer] = layerHumidity;
                }
            }
        }
    }

    private void ensureStaticColumnState(
        CellColumnState cell,
        ServerWorld world,
        SeedTerrainProvider provider,
        int sampleX,
        int sampleZ
    ) {
        if (cell.staticInitialized) {
            return;
        }
        SeedTerrainProvider.TerrainSample terrain = provider.sample(world, sampleX, sampleZ);
        cell.terrainHeightBlocks = terrain.terrainHeightBlocks();
        cell.biomeTemperature = terrain.biomeTemperature();
        cell.surfaceClass = terrain.surfaceClass();
        cell.roughnessLengthMeters = terrain.roughnessLengthMeters();
        cell.staticInitialized = true;
    }

    private boolean stepNative(float deltaSeconds) {
        if (!nativeBridge.isLoaded()) {
            return false;
        }
        int gridWidth = radiusCells * 2 + 1;
        if (nativeContextKey == 0L) {
            nativeContextKey = nativeBridge.createContext(
                gridWidth,
                activeLayers,
                gridWidth,
                cellSizeBlocks,
                stepSeconds,
                DEFAULT_MOLECULAR_NU_M2_S,
                DEFAULT_PRANDTL_AIR,
                DEFAULT_TURBULENT_PRANDTL
            );
            if (nativeContextKey == 0L) {
                return false;
            }
        }
        boolean ok = nativeBridge.stepContext(
            nativeContextKey,
            gridWidth,
            activeLayers,
            gridWidth,
            cellSizeBlocks,
            stepSeconds,
            DEFAULT_MOLECULAR_NU_M2_S,
            DEFAULT_PRANDTL_AIR,
            DEFAULT_TURBULENT_PRANDTL,
            forcingBuffer,
            stateBuffer
        );
        if (!ok) {
            releaseNativeContext();
            return false;
        }

        for (int cx = centerCellX - radiusCells; cx <= centerCellX + radiusCells; cx++) {
            for (int cz = centerCellZ - radiusCells; cz <= centerCellZ + radiusCells; cz++) {
                CellColumnState cell = cells.get(pack(cx, cz));
                if (cell == null) {
                    continue;
                }
                cell.ensureLayers(activeLayers);
                for (int layer = 0; layer < activeLayers; layer++) {
                    int stateBase = forcingIndex(cx, layer, cz, gridWidth) * NATIVE_STATE_CHANNELS;
                    int forcingBase = forcingIndex(cx, layer, cz, gridWidth) * NATIVE_FORCING_CHANNELS;
                    cell.ambientAirTemperatureKelvin[layer] = finiteOrDefault(
                        stateBuffer[stateBase + OUT_AMBIENT],
                        finiteOrDefault(forcingBuffer[forcingBase + CH_AMBIENT_TARGET], cell.ambientAirTemperatureKelvin[layer])
                    );
                    cell.deepGroundTemperatureKelvin[layer] = finiteOrDefault(
                        stateBuffer[stateBase + OUT_DEEP_GROUND],
                        finiteOrDefault(forcingBuffer[forcingBase + CH_DEEP_GROUND_TARGET], cell.deepGroundTemperatureKelvin[layer])
                    );
                    cell.surfaceTemperatureKelvin[layer] = finiteOrDefault(
                        stateBuffer[stateBase + OUT_SURFACE],
                        finiteOrDefault(forcingBuffer[forcingBase + CH_SURFACE_TARGET], cell.surfaceTemperatureKelvin[layer])
                    );
                    cell.windX[layer] = finiteOrDefault(
                        stateBuffer[stateBase + OUT_WIND_X],
                        finiteOrDefault(forcingBuffer[forcingBase + CH_BACKGROUND_WIND_X], cell.windX[layer])
                    );
                    cell.windZ[layer] = finiteOrDefault(
                        stateBuffer[stateBase + OUT_WIND_Z],
                        finiteOrDefault(forcingBuffer[forcingBase + CH_BACKGROUND_WIND_Z], cell.windZ[layer])
                    );
                    cell.humidity[layer] = MathHelper.clamp(
                        finiteOrDefault(forcingBuffer[forcingBase + CH_HUMIDITY], cell.humidity[layer]),
                        0.0f,
                        1.0f
                    );
                }
            }
        }
        return true;
    }

    private void refreshFallback(float deltaSeconds) {
        for (int cx = centerCellX - radiusCells; cx <= centerCellX + radiusCells; cx++) {
            for (int cz = centerCellZ - radiusCells; cz <= centerCellZ + radiusCells; cz++) {
                CellColumnState cell = cells.get(pack(cx, cz));
                if (cell == null) {
                    continue;
                }
                cell.ensureLayers(activeLayers);
                int gridWidth = radiusCells * 2 + 1;
                for (int layer = 0; layer < activeLayers; layer++) {
                    int base = forcingIndex(cx, layer, cz, gridWidth) * NATIVE_FORCING_CHANNELS;
                    float ambientTarget = forcingBuffer[base + CH_AMBIENT_TARGET] + forcingBuffer[base + CH_NESTED_AMBIENT_DELTA];
                    float surfaceTarget = forcingBuffer[base + CH_SURFACE_TARGET] + forcingBuffer[base + CH_NESTED_SURFACE_DELTA];
                    float windTargetX = forcingBuffer[base + CH_BACKGROUND_WIND_X] + forcingBuffer[base + CH_NESTED_WIND_X_DELTA];
                    float windTargetZ = forcingBuffer[base + CH_BACKGROUND_WIND_Z] + forcingBuffer[base + CH_NESTED_WIND_Z_DELTA];
                    cell.ambientAirTemperatureKelvin[layer] = relax(
                        cell.ambientAirTemperatureKelvin[layer],
                        ambientTarget,
                        deltaSeconds
                    );
                    cell.deepGroundTemperatureKelvin[layer] = relax(
                        cell.deepGroundTemperatureKelvin[layer],
                        forcingBuffer[base + CH_DEEP_GROUND_TARGET],
                        deltaSeconds * 0.25f
                    );
                    cell.surfaceTemperatureKelvin[layer] = relax(
                        cell.surfaceTemperatureKelvin[layer],
                        surfaceTarget,
                        deltaSeconds
                    );
                    cell.windX[layer] = relax(cell.windX[layer], windTargetX, deltaSeconds * 0.5f);
                    cell.windZ[layer] = relax(cell.windZ[layer], windTargetZ, deltaSeconds * 0.5f);
                    cell.humidity[layer] = relax(cell.humidity[layer], forcingBuffer[base + CH_HUMIDITY], deltaSeconds * 0.5f);
                }
            }
        }
    }

    private void ensureNativeBuffers(int cellCount) {
        int forcingLength = cellCount * NATIVE_FORCING_CHANNELS;
        int stateLength = cellCount * NATIVE_STATE_CHANNELS;
        if (forcingBuffer.length != forcingLength) {
            forcingBuffer = new float[forcingLength];
        }
        if (stateBuffer.length != stateLength) {
            stateBuffer = new float[stateLength];
        }
    }

    private void releaseNativeContext() {
        if (nativeContextKey != 0L) {
            nativeBridge.releaseContext(nativeContextKey);
            nativeContextKey = 0L;
        }
    }

    private int computeActiveLayers(ServerWorld world, int baseY) {
        int topExclusive = Math.min(320, world.getTopYInclusive() + 1);
        int extent = Math.max(layerHeightBlocks, topExclusive - baseY);
        return Math.max(1, Math.min(maxLayers, MathHelper.ceil(extent / (float) layerHeightBlocks)));
    }

    private int layerCenterBlockY(int layer) {
        return verticalBaseY + layer * layerHeightBlocks + layerHeightBlocks / 2;
    }

    private int layerIndexForY(int y) {
        if (activeLayers <= 1) {
            return 0;
        }
        return MathHelper.clamp((y - verticalBaseY) / layerHeightBlocks, 0, activeLayers - 1);
    }

    private int forcingIndex(int cellX, int layer, int cellZ, int gridWidth) {
        int localX = cellX - (centerCellX - radiusCells);
        int localZ = cellZ - (centerCellZ - radiusCells);
        return (localX * activeLayers + layer) * gridWidth + localZ;
    }

    private CellColumnState columnStateAtClamped(int cellX, int cellZ) {
        int clampedX = MathHelper.clamp(cellX, centerCellX - radiusCells, centerCellX + radiusCells);
        int clampedZ = MathHelper.clamp(cellZ, centerCellZ - radiusCells, centerCellZ + radiusCells);
        return cells.get(pack(clampedX, clampedZ));
    }

    private float bilerp(float v00, float v10, float v01, float v11, float tx, float tz) {
        float vx0 = MathHelper.lerp(tx, v00, v10);
        float vx1 = MathHelper.lerp(tx, v01, v11);
        return MathHelper.lerp(tz, vx0, vx1);
    }

    private float humidityFluxX(int cellX, int cellZ, int layer) {
        CellColumnState state = cells.get(pack(cellX, cellZ));
        if (state == null || state.layerCount() <= 0) {
            return 0.0f;
        }
        int clampedLayer = MathHelper.clamp(layer, 0, state.layerCount() - 1);
        float humidity = state.humidity[clampedLayer];
        float wind = state.windX[clampedLayer];
        if (!Float.isFinite(humidity) || !Float.isFinite(wind)) {
            return 0.0f;
        }
        return MathHelper.clamp(humidity, 0.0f, 1.0f) * wind;
    }

    private float humidityFluxZ(int cellX, int cellZ, int layer) {
        CellColumnState state = cells.get(pack(cellX, cellZ));
        if (state == null || state.layerCount() <= 0) {
            return 0.0f;
        }
        int clampedLayer = MathHelper.clamp(layer, 0, state.layerCount() - 1);
        float humidity = state.humidity[clampedLayer];
        float wind = state.windZ[clampedLayer];
        if (!Float.isFinite(humidity) || !Float.isFinite(wind)) {
            return 0.0f;
        }
        return MathHelper.clamp(humidity, 0.0f, 1.0f) * wind;
    }

    private int snapshotIndex(int localX, int layer, int localZ, int gridWidth) {
        return (localX * activeLayers + layer) * gridWidth + localZ;
    }

    private void computeDiagnostics(
        int gridWidth,
        int layers,
        float[] ambientAirTemperatureKelvin,
        float[] deepGroundTemperatureKelvin,
        float[] surfaceTemperatureKelvin,
        float[] windX,
        float[] windZ,
        float[] humidity,
        float[] convectiveHeating,
        float[] convectiveMoistening,
        float[] convectiveInflowX,
        float[] convectiveInflowZ,
        float[] tornadoHeating,
        float[] tornadoMoistening,
        float[] tornadoWindX,
        float[] tornadoWindZ,
        float[] tornadoUpdraft,
        float[] instabilityProxy,
        float[] lowLevelShear,
        float[] moistureConvergence,
        float[] liftProxy
    ) {
        float horizontalScaleMeters = Math.max(1.0f, cellSizeBlocks);
        for (int localX = 0; localX < gridWidth; localX++) {
            int leftX = Math.max(0, localX - 1);
            int rightX = Math.min(gridWidth - 1, localX + 1);
            for (int localZ = 0; localZ < gridWidth; localZ++) {
                int prevZ = Math.max(0, localZ - 1);
                int nextZ = Math.min(gridWidth - 1, localZ + 1);
                int surfaceIndex = snapshotIndex(localX, 0, localZ, gridWidth);
                float surfaceReferenceKelvin = surfaceTemperatureKelvin[surfaceIndex];
                float deepGroundReferenceKelvin = deepGroundTemperatureKelvin[surfaceIndex];
                float surfaceAmbientKelvin = ambientAirTemperatureKelvin[surfaceIndex];
                float surfaceWindX = windX[surfaceIndex];
                float surfaceWindZ = windZ[surfaceIndex];
                for (int layer = 0; layer < layers; layer++) {
                    int stateIndex = snapshotIndex(localX, layer, localZ, gridWidth);
                    float ambientLayer = ambientAirTemperatureKelvin[stateIndex];
                    float windLayerX = windX[stateIndex];
                    float windLayerZ = windZ[stateIndex];
                    float humidityLayer = humidity[stateIndex];
                    if (!Float.isFinite(surfaceReferenceKelvin)
                        || !Float.isFinite(deepGroundReferenceKelvin)
                        || !Float.isFinite(surfaceAmbientKelvin)
                        || !Float.isFinite(surfaceWindX)
                        || !Float.isFinite(surfaceWindZ)
                        || !Float.isFinite(ambientLayer)
                        || !Float.isFinite(windLayerX)
                        || !Float.isFinite(windLayerZ)
                        || !Float.isFinite(humidityLayer)) {
                        instabilityProxy[stateIndex] = 0.0f;
                        lowLevelShear[stateIndex] = 0.0f;
                        moistureConvergence[stateIndex] = 0.0f;
                        liftProxy[stateIndex] = 0.0f;
                        continue;
                    }
                    float q = MathHelper.clamp(humidityLayer, 0.0f, 1.0f);
                    float instability = computeInstabilityProxy(
                        surfaceReferenceKelvin,
                        deepGroundReferenceKelvin,
                        surfaceAmbientKelvin,
                        ambientLayer,
                        q,
                        convectiveHeating[stateIndex] + tornadoHeating[stateIndex],
                        convectiveMoistening[stateIndex] + tornadoMoistening[stateIndex],
                        layer
                    );
                    instabilityProxy[stateIndex] = instability;

                    float du = windLayerX - surfaceWindX;
                    float dv = windLayerZ - surfaceWindZ;
                    lowLevelShear[stateIndex] = MathHelper.sqrt(du * du + dv * dv);

                    float leftHumidity = humidity[snapshotIndex(leftX, layer, localZ, gridWidth)];
                    float rightHumidity = humidity[snapshotIndex(rightX, layer, localZ, gridWidth)];
                    float southHumidity = humidity[snapshotIndex(localX, layer, prevZ, gridWidth)];
                    float northHumidity = humidity[snapshotIndex(localX, layer, nextZ, gridWidth)];
                    float leftWind = windX[snapshotIndex(leftX, layer, localZ, gridWidth)];
                    float rightWind = windX[snapshotIndex(rightX, layer, localZ, gridWidth)];
                    float southWind = windZ[snapshotIndex(localX, layer, prevZ, gridWidth)];
                    float northWind = windZ[snapshotIndex(localX, layer, nextZ, gridWidth)];
                    float leftFlux = (Float.isFinite(leftHumidity) && Float.isFinite(leftWind))
                        ? MathHelper.clamp(leftHumidity, 0.0f, 1.0f) * leftWind
                        : 0.0f;
                    float rightFlux = (Float.isFinite(rightHumidity) && Float.isFinite(rightWind))
                        ? MathHelper.clamp(rightHumidity, 0.0f, 1.0f) * rightWind
                        : 0.0f;
                    float southFlux = (Float.isFinite(southHumidity) && Float.isFinite(southWind))
                        ? MathHelper.clamp(southHumidity, 0.0f, 1.0f) * southWind
                        : 0.0f;
                    float northFlux = (Float.isFinite(northHumidity) && Float.isFinite(northWind))
                        ? MathHelper.clamp(northHumidity, 0.0f, 1.0f) * northWind
                        : 0.0f;
                    float dQudx = (rightFlux - leftFlux) / (Math.max(1, rightX - leftX) * horizontalScaleMeters);
                    float dQwdz = (northFlux - southFlux) / (Math.max(1, nextZ - prevZ) * horizontalScaleMeters);
                    float convergence = -(dQudx + dQwdz);
                    moistureConvergence[stateIndex] = convergence;
                    liftProxy[stateIndex] = computeLiftProxy(
                        instability,
                        q,
                        Math.max(0.0f, convergence),
                        lowLevelShear[stateIndex],
                        convectiveHeating[stateIndex] + tornadoHeating[stateIndex] + tornadoUpdraft[stateIndex] * 0.35f,
                        convectiveMoistening[stateIndex] + tornadoMoistening[stateIndex],
                        convectiveInflowX[stateIndex] + tornadoWindX[stateIndex],
                        convectiveInflowZ[stateIndex] + tornadoWindZ[stateIndex],
                        layer,
                        layers,
                        horizontalScaleMeters
                    );
                }
            }
        }
    }

    private float computeInstabilityProxy(
        float surfaceReferenceKelvin,
        float deepGroundReferenceKelvin,
        float surfaceAmbientKelvin,
        float ambientKelvin,
        float humidity,
        float convectiveHeating,
        float convectiveMoistening,
        int layer
    ) {
        float q = MathHelper.clamp(humidity, 0.0f, 1.0f);
        float moistening = MathHelper.clamp(convectiveMoistening, 0.0f, 1.0f);
        float surfaceExcess = Math.max(0.0f, surfaceReferenceKelvin - ambientKelvin);
        float deepExcess = Math.max(0.0f, deepGroundReferenceKelvin - ambientKelvin) * 0.55f;
        float boundaryWarmth = Math.max(0.0f, surfaceReferenceKelvin - surfaceAmbientKelvin) * 0.35f;
        float expectedLapse = layer * layerHeightBlocks * AMBIENT_LAPSE_RATE_K_PER_BLOCK;
        float actualLapse = Math.max(0.0f, surfaceAmbientKelvin - ambientKelvin);
        float lapseExcess = Math.max(0.0f, actualLapse - expectedLapse * 0.5f) * 0.75f;
        float convectiveBoost = Math.max(0.0f, convectiveHeating) * (0.70f + 0.30f * moistening) * 1.40f;
        float moistureFactor = 0.30f + 0.70f * Math.max(q, moistening);
        return (surfaceExcess + deepExcess + boundaryWarmth + lapseExcess + convectiveBoost) * moistureFactor;
    }

    private float computeLiftProxy(
        float instability,
        float humidity,
        float positiveConvergence,
        float shear,
        float convectiveHeating,
        float convectiveMoistening,
        float convectiveInflowX,
        float convectiveInflowZ,
        int layer,
        int layers,
        float horizontalScaleMeters
    ) {
        float moistureNorm = Math.min(1.0f, Math.max(MathHelper.clamp(humidity, 0.0f, 1.0f), MathHelper.clamp(convectiveMoistening, 0.0f, 1.0f)));
        float instabilityNorm = Math.min(1.5f, instability / 4.0f);
        float convergenceNorm = Math.min(1.5f, positiveConvergence * horizontalScaleMeters * 8.0f);
        float shearNorm = Math.min(1.5f, shear / 6.0f);
        float forcedInflowNorm = Math.min(1.5f, MathHelper.sqrt(convectiveInflowX * convectiveInflowX + convectiveInflowZ * convectiveInflowZ) / 1.5f);
        float forcedLiftNorm = Math.min(1.5f, Math.max(0.0f, convectiveHeating) * 0.35f + forcedInflowNorm);
        float layerWeight = 1.0f - 0.65f * (layer / (float) Math.max(1, layers - 1));
        return instabilityNorm
            * (0.40f + 0.60f * moistureNorm)
            * (0.25f + Math.max(convergenceNorm, forcedLiftNorm))
            * (0.80f + 0.20f * shearNorm)
            * layerWeight;
    }

    private float forcingChannel(int cellX, int layer, int cellZ, int gridWidth, int channel) {
        int base = forcingIndex(cellX, layer, cellZ, gridWidth) * NATIVE_FORCING_CHANNELS + channel;
        if (base < 0 || base >= forcingBuffer.length) {
            return 0.0f;
        }
        float value = forcingBuffer[base];
        return Float.isFinite(value) ? value : 0.0f;
    }

    private TornadoEnvironment sampleEnvironment(int centerX, int centerZ, int sampleRadius) {
        int sampledStates = 0;
        float meanInstability = 0.0f;
        float maxInstability = 0.0f;
        float meanShear = 0.0f;
        float maxShear = 0.0f;
        float meanHumidity = 0.0f;
        float meanPositiveConvergence = 0.0f;
        float maxPositiveConvergence = 0.0f;
        float meanLift = 0.0f;
        float maxLift = 0.0f;
        float horizontalScaleMeters = Math.max(1.0f, cellSizeBlocks);
        int sampledLayers = Math.max(1, Math.min(activeLayers, 3));
        int gridWidth = radiusCells * 2 + 1;

        for (int cx = centerX - sampleRadius; cx <= centerX + sampleRadius; cx++) {
            for (int cz = centerZ - sampleRadius; cz <= centerZ + sampleRadius; cz++) {
                CellColumnState column = cells.get(pack(cx, cz));
                if (column == null || column.layerCount() <= 0) {
                    continue;
                }
                float surfaceReferenceKelvin = column.surfaceTemperatureKelvin[0];
                float deepGroundReferenceKelvin = column.deepGroundTemperatureKelvin[0];
                float surfaceAmbientKelvin = column.ambientAirTemperatureKelvin[0];
                float surfaceWindX = column.windX[0];
                float surfaceWindZ = column.windZ[0];
                if (!Float.isFinite(surfaceReferenceKelvin)
                    || !Float.isFinite(deepGroundReferenceKelvin)
                    || !Float.isFinite(surfaceAmbientKelvin)
                    || !Float.isFinite(surfaceWindX)
                    || !Float.isFinite(surfaceWindZ)) {
                    continue;
                }
                for (int layer = 0; layer < sampledLayers; layer++) {
                    float ambientLayer = column.ambientAirTemperatureKelvin[layer];
                    float windLayerX = column.windX[layer];
                    float windLayerZ = column.windZ[layer];
                    float humidityLayer = column.humidity[layer];
                    if (!Float.isFinite(ambientLayer)
                        || !Float.isFinite(windLayerX)
                        || !Float.isFinite(windLayerZ)
                        || !Float.isFinite(humidityLayer)) {
                        continue;
                    }
                    float q = MathHelper.clamp(humidityLayer, 0.0f, 1.0f);
                    float convectiveHeating = forcingChannel(cx, layer, cz, gridWidth, CH_CONVECTIVE_HEATING);
                    float convectiveMoistening = forcingChannel(cx, layer, cz, gridWidth, CH_CONVECTIVE_MOISTENING);
                    float convectiveInflowX = forcingChannel(cx, layer, cz, gridWidth, CH_CONVECTIVE_INFLOW_X);
                    float convectiveInflowZ = forcingChannel(cx, layer, cz, gridWidth, CH_CONVECTIVE_INFLOW_Z);
                    float tornadoHeating = forcingChannel(cx, layer, cz, gridWidth, CH_TORNADO_HEATING);
                    float tornadoMoistening = forcingChannel(cx, layer, cz, gridWidth, CH_TORNADO_MOISTENING);
                    float tornadoWindX = forcingChannel(cx, layer, cz, gridWidth, CH_TORNADO_WIND_X);
                    float tornadoWindZ = forcingChannel(cx, layer, cz, gridWidth, CH_TORNADO_WIND_Z);
                    float tornadoUpdraft = forcingChannel(cx, layer, cz, gridWidth, CH_TORNADO_UPDRAFT)
                        + forcingChannel(cx, layer, cz, gridWidth, CH_NESTED_UPDRAFT);
                    float instability = computeInstabilityProxy(
                        surfaceReferenceKelvin,
                        deepGroundReferenceKelvin,
                        surfaceAmbientKelvin,
                        ambientLayer,
                        q,
                        convectiveHeating + tornadoHeating,
                        convectiveMoistening + tornadoMoistening,
                        layer
                    );
                    float du = windLayerX - surfaceWindX;
                    float dv = windLayerZ - surfaceWindZ;
                    float shear = MathHelper.sqrt(du * du + dv * dv);

                    float leftFlux = humidityFluxX(cx - 1, cz, layer);
                    float rightFlux = humidityFluxX(cx + 1, cz, layer);
                    float southFlux = humidityFluxZ(cx, cz - 1, layer);
                    float northFlux = humidityFluxZ(cx, cz + 1, layer);
                    float convergence = -((rightFlux - leftFlux) / (2.0f * horizontalScaleMeters)
                        + (northFlux - southFlux) / (2.0f * horizontalScaleMeters));
                    float positiveConvergence = Math.max(0.0f, convergence);
                    float lift = computeLiftProxy(
                        instability,
                        q,
                        positiveConvergence,
                        shear,
                        convectiveHeating + tornadoHeating + tornadoUpdraft * 0.35f,
                        convectiveMoistening + tornadoMoistening,
                        convectiveInflowX + tornadoWindX,
                        convectiveInflowZ + tornadoWindZ,
                        layer,
                        sampledLayers,
                        horizontalScaleMeters
                    );
                    if (!Float.isFinite(instability)
                        || !Float.isFinite(shear)
                        || !Float.isFinite(positiveConvergence)
                        || !Float.isFinite(lift)) {
                        continue;
                    }

                    meanInstability += instability;
                    maxInstability = Math.max(maxInstability, instability);
                    meanShear += shear;
                    maxShear = Math.max(maxShear, shear);
                    meanHumidity += q;
                    meanPositiveConvergence += positiveConvergence;
                    maxPositiveConvergence = Math.max(maxPositiveConvergence, positiveConvergence);
                    meanLift += lift;
                    maxLift = Math.max(maxLift, lift);
                    sampledStates++;
                }
            }
        }

        if (sampledStates <= 0) {
            return TornadoEnvironment.EMPTY;
        }
        float inverseCount = 1.0f / sampledStates;
        return new TornadoEnvironment(
            meanInstability * inverseCount,
            maxInstability,
            meanShear * inverseCount,
            maxShear,
            meanHumidity * inverseCount,
            meanPositiveConvergence * inverseCount,
            maxPositiveConvergence,
            meanLift * inverseCount,
            maxLift,
            sampledStates
        );
    }

    private float relax(float current, float target, float deltaSeconds) {
        float safeTarget = finiteOrDefault(target, finiteOrDefault(current, 0.0f));
        if (!Float.isFinite(current) || current == 0.0f) {
            return safeTarget;
        }
        float alpha = MathHelper.clamp(deltaSeconds * SURFACE_RELAXATION_PER_SECOND, 0.0f, 1.0f);
        return MathHelper.lerp(alpha, current, safeTarget);
    }

    private float finiteOrDefault(float value, float fallback) {
        if (Float.isFinite(value)) {
            return value;
        }
        return Float.isFinite(fallback) ? fallback : 0.0f;
    }

    private long pack(int x, int z) {
        return ((long) x << 32) ^ (z & 0xffffffffL);
    }

    private int unpackX(long packed) {
        return (int) (packed >> 32);
    }

    private int unpackZ(long packed) {
        return (int) packed;
    }

    record NestedFeedbackBin(
        int cellX,
        int layer,
        int cellZ,
        float volumeAverage,
        float densityAverage,
        float momentumXAverage,
        float momentumZAverage,
        float airTemperatureVolumeAverage,
        float surfaceTemperatureVolumeAverage,
        float bottomAreaAverage,
        float bottomMassFluxAverage,
        float topAreaAverage,
        float topMassFluxAverage
    ) {
    }

    private record NestedFeedbackKey(int cellX, int layer, int cellZ) {
    }

    private static final class NestedFeedbackAccumulator {
        private float volumeAverage;
        private float densityAverage;
        private float momentumXAverage;
        private float momentumZAverage;
        private float airTemperatureVolumeAverage;
        private float surfaceTemperatureVolumeAverage;
        private float bottomAreaAverage;
        private float bottomMassFluxAverage;
        private float topAreaAverage;
        private float topMassFluxAverage;

        private void add(NestedFeedbackBin bin) {
            volumeAverage += bin.volumeAverage();
            densityAverage += bin.densityAverage();
            momentumXAverage += bin.momentumXAverage();
            momentumZAverage += bin.momentumZAverage();
            airTemperatureVolumeAverage += bin.airTemperatureVolumeAverage();
            surfaceTemperatureVolumeAverage += bin.surfaceTemperatureVolumeAverage();
            bottomAreaAverage += bin.bottomAreaAverage();
            bottomMassFluxAverage += bin.bottomMassFluxAverage();
            topAreaAverage += bin.topAreaAverage();
            topMassFluxAverage += bin.topMassFluxAverage();
        }
    }

    record Sample(
        float terrainHeightBlocks,
        float biomeTemperature,
        float ambientAirTemperatureKelvin,
        float deepGroundTemperatureKelvin,
        float surfaceTemperatureKelvin,
        float roughnessLengthMeters,
        float windX,
        float windZ,
        float humidity,
        byte surfaceClass
    ) {
    }

    record Snapshot(
        int gridWidth,
        int activeLayers,
        int cellSizeBlocks,
        int layerHeightBlocks,
        int radiusCells,
        int centerCellX,
        int centerCellZ,
        int verticalBaseY,
        float stepSeconds,
        long lastTickProcessed,
        float[] terrainHeightBlocks,
        float[] biomeTemperature,
        float[] roughnessLengthMeters,
        byte[] surfaceClass,
        float[] ambientAirTemperatureKelvin,
        float[] deepGroundTemperatureKelvin,
        float[] surfaceTemperatureKelvin,
        float[] forcingAmbientTargetKelvin,
        float[] forcingSurfaceTargetKelvin,
        float[] forcingBackgroundWindX,
        float[] forcingBackgroundWindZ,
        float[] forcingNestedAmbientDeltaKelvin,
        float[] forcingNestedSurfaceDeltaKelvin,
        float[] forcingNestedWindXDelta,
        float[] forcingNestedWindZDelta,
        float[] forcingNestedUpdraft,
        float[] windX,
        float[] windZ,
        float[] humidity,
        float[] instabilityProxy,
        float[] lowLevelShear,
        float[] moistureConvergence,
        float[] liftProxy,
        NestedFeedbackDiagnostics nestedFeedbackDiagnostics
    ) {
    }

    record NestedFeedbackDiagnostics(
        long lastAppliedTick,
        int inputBinCount,
        int acceptedBinCount,
        int appliedCellCount,
        float meanCoverage,
        float maxCoverage,
        float meanWindDelta,
        float maxWindDelta,
        float meanAirDeltaKelvin,
        float maxAirDeltaKelvin,
        float meanSurfaceDeltaKelvin,
        float maxSurfaceDeltaKelvin,
        float meanBottomFluxDensity,
        float meanTopFluxDensity,
        float meanNestedUpdraft,
        float maxAbsNestedUpdraft
    ) {
        private static final NestedFeedbackDiagnostics EMPTY = new NestedFeedbackDiagnostics(
            Long.MIN_VALUE,
            0,
            0,
            0,
            0.0f,
            0.0f,
            0.0f,
            0.0f,
            0.0f,
            0.0f,
            0.0f,
            0.0f,
            0.0f,
            0.0f,
            0.0f,
            0.0f
        );
    }

    record DiagnosticsSummary(
        float meanInstabilityProxy,
        float maxInstabilityProxy,
        float meanLowLevelShear,
        float meanHumidity,
        float meanPositiveMoistureConvergence,
        float maxPositiveMoistureConvergence,
        float meanLiftProxy,
        float maxLiftProxy,
        int sampledStateCount
    ) {
        private static final DiagnosticsSummary EMPTY = new DiagnosticsSummary(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0);
    }

    record TornadoEnvironment(
        float meanInstabilityProxy,
        float maxInstabilityProxy,
        float meanLowLevelShear,
        float maxLowLevelShear,
        float meanHumidity,
        float meanPositiveMoistureConvergence,
        float maxPositiveMoistureConvergence,
        float meanLiftProxy,
        float maxLiftProxy,
        int sampledStateCount
    ) {
        private static final TornadoEnvironment EMPTY = new TornadoEnvironment(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0);
    }

    private static final class CellColumnState {
        private boolean staticInitialized;
        private float terrainHeightBlocks;
        private float biomeTemperature;
        private float roughnessLengthMeters;
        private byte surfaceClass;
        private float[] ambientAirTemperatureKelvin;
        private float[] deepGroundTemperatureKelvin;
        private float[] surfaceTemperatureKelvin;
        private float[] windX;
        private float[] windZ;
        private float[] humidity;

        private CellColumnState(int layers) {
            ensureLayers(layers);
        }

        private void ensureLayers(int layers) {
            if (ambientAirTemperatureKelvin == null || ambientAirTemperatureKelvin.length != layers) {
                ambientAirTemperatureKelvin = new float[layers];
                deepGroundTemperatureKelvin = new float[layers];
                surfaceTemperatureKelvin = new float[layers];
                windX = new float[layers];
                windZ = new float[layers];
                humidity = new float[layers];
            }
        }

        private int layerCount() {
            return ambientAirTemperatureKelvin == null ? 0 : ambientAirTemperatureKelvin.length;
        }
    }
}
