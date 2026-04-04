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
    private static final int NATIVE_FORCING_CHANNELS = 9;
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
    private static final int OUT_AMBIENT = 0;
    private static final int OUT_DEEP_GROUND = 1;
    private static final int OUT_SURFACE = 2;
    private static final int OUT_WIND_X = 3;
    private static final int OUT_WIND_Z = 4;

    private final int cellSizeBlocks;
    private final int radiusCells;
    private final int layerHeightBlocks;
    private final int maxLayers;
    private final Map<Long, CellColumnState> cells = new HashMap<>();
    private final MesoscaleNativeBridge nativeBridge = new MesoscaleNativeBridge();
    private long nativeContextKey;
    private float[] forcingBuffer = new float[0];
    private float[] stateBuffer = new float[0];
    private int centerCellX;
    private int centerCellZ;
    private int activeLayers = 1;
    private int verticalBaseY = 0;
    private long lastRefreshTick = Long.MIN_VALUE;

    MesoscaleGrid(int cellSizeBlocks, int radiusCells, int layerHeightBlocks, int maxLayers) {
        this.cellSizeBlocks = cellSizeBlocks;
        this.radiusCells = radiusCells;
        this.layerHeightBlocks = Math.max(1, layerHeightBlocks);
        this.maxLayers = Math.max(1, maxLayers);
    }

    void refresh(
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
        boolean centerChanged = lastRefreshTick != Long.MIN_VALUE
            && (nextCenterCellX != centerCellX
                || nextCenterCellZ != centerCellZ
                || nextVerticalBaseY != verticalBaseY
                || nextActiveLayers != activeLayers);
        centerCellX = nextCenterCellX;
        centerCellZ = nextCenterCellZ;
        verticalBaseY = nextVerticalBaseY;
        activeLayers = nextActiveLayers;
        float deltaSeconds = lastRefreshTick == Long.MIN_VALUE ? dtSeconds : Math.max(1L, tickCounter - lastRefreshTick) * dtSeconds;
        lastRefreshTick = tickCounter;
        if (centerChanged) {
            releaseNativeContext();
        }

        buildForcing(world, provider, background, deltaSeconds);
        if (!stepNative(deltaSeconds)) {
            refreshFallback(deltaSeconds);
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

    Sample sample(BlockPos pos) {
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
            state.surfaceClass
        );
    }

    int cellCount() {
        return cells.size() * activeLayers;
    }

    @Override
    public void close() {
        releaseNativeContext();
        cells.clear();
        forcingBuffer = new float[0];
        stateBuffer = new float[0];
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
            Math.max(1.0e-3f, deltaSeconds),
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
                Math.max(1.0e-3f, deltaSeconds),
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
            Math.max(1.0e-3f, deltaSeconds),
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
                    int base = forcingIndex(cx, layer, cz, gridWidth) * NATIVE_STATE_CHANNELS;
                    cell.ambientAirTemperatureKelvin[layer] = stateBuffer[base + OUT_AMBIENT];
                    cell.deepGroundTemperatureKelvin[layer] = stateBuffer[base + OUT_DEEP_GROUND];
                    cell.surfaceTemperatureKelvin[layer] = stateBuffer[base + OUT_SURFACE];
                    cell.windX[layer] = stateBuffer[base + OUT_WIND_X];
                    cell.windZ[layer] = stateBuffer[base + OUT_WIND_Z];
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
                    cell.ambientAirTemperatureKelvin[layer] = relax(
                        cell.ambientAirTemperatureKelvin[layer],
                        forcingBuffer[base + CH_AMBIENT_TARGET],
                        deltaSeconds
                    );
                    cell.deepGroundTemperatureKelvin[layer] = relax(
                        cell.deepGroundTemperatureKelvin[layer],
                        forcingBuffer[base + CH_DEEP_GROUND_TARGET],
                        deltaSeconds * 0.25f
                    );
                    cell.surfaceTemperatureKelvin[layer] = relax(
                        cell.surfaceTemperatureKelvin[layer],
                        forcingBuffer[base + CH_SURFACE_TARGET],
                        deltaSeconds
                    );
                    cell.windX[layer] = relax(cell.windX[layer], forcingBuffer[base + CH_BACKGROUND_WIND_X], deltaSeconds * 0.5f);
                    cell.windZ[layer] = relax(cell.windZ[layer], forcingBuffer[base + CH_BACKGROUND_WIND_Z], deltaSeconds * 0.5f);
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

    private float relax(float current, float target, float deltaSeconds) {
        if (!Float.isFinite(current) || current == 0.0f) {
            return target;
        }
        float alpha = MathHelper.clamp(deltaSeconds * SURFACE_RELAXATION_PER_SECOND, 0.0f, 1.0f);
        return MathHelper.lerp(alpha, current, target);
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

    record Sample(
        float terrainHeightBlocks,
        float biomeTemperature,
        float ambientAirTemperatureKelvin,
        float deepGroundTemperatureKelvin,
        float surfaceTemperatureKelvin,
        float roughnessLengthMeters,
        float windX,
        float windZ,
        byte surfaceClass
    ) {
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
            }
        }

        private int layerCount() {
            return ambientAirTemperatureKelvin == null ? 0 : ambientAirTemperatureKelvin.length;
        }
    }
}
