package com.aerodynamics4mc.runtime;

import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;

import net.minecraft.registry.RegistryKey;
import net.minecraft.server.world.ServerWorld;
import net.minecraft.util.math.BlockPos;
import net.minecraft.util.math.MathHelper;
import net.minecraft.world.World;

final class BackgroundMetGrid {
    private static final float BASE_AIR_TEMPERATURE_K = 288.15f;
    private static final float BIOME_TEMPERATURE_SCALE_K = 12.0f;
    private static final float ALTITUDE_LAPSE_RATE_K_PER_BLOCK = 0.0065f;
    private static final float DEEP_GROUND_OFFSET_K = 1.5f;
    private static final float SURFACE_RELAXATION_PER_SECOND = 1.0f / 1800.0f;
    private static final float SOLAR_SURFACE_HEATING_K = 18.0f;
    private static final float CLEAR_SKY_COOLING_K = 8.0f;

    private final int cellSizeBlocks;
    private final int radiusCells;
    private final Map<Long, CellState> cells = new HashMap<>();
    private ServerWorld currentWorld;
    private SeedTerrainProvider currentProvider;
    private int centerCellX;
    private int centerCellZ;
    private long lastRefreshTick = Long.MIN_VALUE;
    private long currentRefreshTick = Long.MIN_VALUE;
    private float currentDeltaSeconds = 1.0f;
    private float currentSolarAltitude = 0.0f;
    private float currentClearSky = 1.0f;

    BackgroundMetGrid(int cellSizeBlocks, int radiusCells) {
        this.cellSizeBlocks = cellSizeBlocks;
        this.radiusCells = radiusCells;
    }

    synchronized void refresh(
        ServerWorld world,
        AeroServerRuntime.WorldEnvironmentSnapshot environmentSnapshot,
        BlockPos focus,
        long tickCounter,
        float dtSeconds,
        SeedTerrainProvider provider
    ) {
        currentWorld = world;
        currentProvider = provider;
        centerCellX = Math.floorDiv(focus.getX(), cellSizeBlocks);
        centerCellZ = Math.floorDiv(focus.getZ(), cellSizeBlocks);
        currentDeltaSeconds = lastRefreshTick == Long.MIN_VALUE ? dtSeconds : Math.max(1L, tickCounter - lastRefreshTick) * dtSeconds;
        lastRefreshTick = tickCounter;
        currentRefreshTick = tickCounter;

        long timeOfDay = environmentSnapshot == null ? world.getTimeOfDay() : environmentSnapshot.timeOfDay();
        float rainGradient = environmentSnapshot == null ? world.getRainGradient(1.0f) : environmentSnapshot.rainGradient();
        float thunderGradient = environmentSnapshot == null ? world.getThunderGradient(1.0f) : environmentSnapshot.thunderGradient();
        float dayPhase = (float) Math.floorMod(timeOfDay, 24000L) / 24000.0f;
        currentSolarAltitude = Math.max(0.0f, (float) Math.sin(dayPhase * (float) (Math.PI * 2.0)));
        currentClearSky = MathHelper.clamp(1.0f - 0.65f * rainGradient - 0.25f * thunderGradient, 0.15f, 1.0f);

        Iterator<Map.Entry<Long, CellState>> it = cells.entrySet().iterator();
        while (it.hasNext()) {
            Map.Entry<Long, CellState> entry = it.next();
            int cx = unpackX(entry.getKey());
            int cz = unpackZ(entry.getKey());
            if (Math.abs(cx - centerCellX) > radiusCells || Math.abs(cz - centerCellZ) > radiusCells) {
                it.remove();
            }
        }
    }

    synchronized Sample sample(BlockPos pos) {
        if (currentWorld == null || currentProvider == null) {
            return null;
        }
        int cellX = Math.floorDiv(pos.getX(), cellSizeBlocks);
        int cellZ = Math.floorDiv(pos.getZ(), cellSizeBlocks);
        CellState state = ensureCell(cellX, cellZ);
        return new Sample(
            state.terrainHeightBlocks,
            state.biomeTemperature,
            state.ambientAirTemperatureKelvin,
            state.deepGroundTemperatureKelvin,
            state.surfaceTemperatureKelvin,
            state.roughnessLengthMeters,
            state.backgroundWindX,
            state.backgroundWindZ,
            state.surfaceClass
        );
    }

    synchronized int cellCount() {
        return cells.size();
    }

    RegistryKey<World> worldKey(ServerWorld world) {
        return world.getRegistryKey();
    }

    private int cellCenterBlock(int cell) {
        return cell * cellSizeBlocks + cellSizeBlocks / 2;
    }

    private CellState ensureCell(int cellX, int cellZ) {
        long key = pack(cellX, cellZ);
        CellState cell = cells.get(key);
        if (cell == null) {
            int sampleX = cellCenterBlock(cellX);
            int sampleZ = cellCenterBlock(cellZ);
            SeedTerrainProvider.TerrainSample terrain = currentProvider.sample(currentWorld, sampleX, sampleZ);
            cell = new CellState();
            cell.terrainHeightBlocks = terrain.terrainHeightBlocks();
            cell.biomeTemperature = terrain.biomeTemperature();
            cell.surfaceClass = terrain.surfaceClass();
            cell.roughnessLengthMeters = terrain.roughnessLengthMeters();
            cells.put(key, cell);
        }
        updateDynamicState(cell, cellX, cellZ);
        return cell;
    }

    private void updateDynamicState(CellState cell, int cellX, int cellZ) {
        if (cell.lastUpdatedTick == currentRefreshTick) {
            return;
        }
        float ambientAirTemperatureKelvin = BASE_AIR_TEMPERATURE_K
            + (cell.biomeTemperature - 0.8f) * BIOME_TEMPERATURE_SCALE_K
            - Math.max(0.0f, cell.terrainHeightBlocks - currentWorld.getSeaLevel()) * ALTITUDE_LAPSE_RATE_K_PER_BLOCK;
        float targetSurfaceTemperatureKelvin = ambientAirTemperatureKelvin
            + currentSolarAltitude * currentClearSky * SOLAR_SURFACE_HEATING_K
            - (1.0f - currentSolarAltitude) * currentClearSky * CLEAR_SKY_COOLING_K;
        cell.ambientAirTemperatureKelvin = ambientAirTemperatureKelvin;
        cell.deepGroundTemperatureKelvin = ambientAirTemperatureKelvin + DEEP_GROUND_OFFSET_K;
        cell.surfaceTemperatureKelvin = relax(cell.surfaceTemperatureKelvin, targetSurfaceTemperatureKelvin, currentDeltaSeconds);
        cell.backgroundWindX = prevailingWindComponent(currentWorld.getSeed(), cellX, cellZ, 0x517cc1b727220a95L);
        cell.backgroundWindZ = prevailingWindComponent(currentWorld.getSeed(), cellX, cellZ, 0x9e3779b97f4a7c15L);
        cell.lastUpdatedTick = currentRefreshTick;
    }

    private float relax(float current, float target, float deltaSeconds) {
        if (!Float.isFinite(current) || current <= 0.0f) {
            return target;
        }
        float alpha = MathHelper.clamp(deltaSeconds * SURFACE_RELAXATION_PER_SECOND, 0.0f, 1.0f);
        return MathHelper.lerp(alpha, current, target);
    }

    private float prevailingWindComponent(long seed, int x, int z, long salt) {
        long h = seed ^ salt ^ ((long) x * 0x9E3779B97F4A7C15L) ^ ((long) z * 0xC2B2AE3D27D4EB4FL);
        h ^= h >>> 33;
        h *= 0xff51afd7ed558ccdl;
        h ^= h >>> 33;
        float unit = ((h >>> 40) & 0xFFFF) / 65535.0f;
        return (unit - 0.5f) * 2.0f;
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
        float backgroundWindX,
        float backgroundWindZ,
        byte surfaceClass
    ) {
    }

    private static final class CellState {
        private float terrainHeightBlocks;
        private float biomeTemperature;
        private float ambientAirTemperatureKelvin;
        private float deepGroundTemperatureKelvin;
        private float surfaceTemperatureKelvin;
        private float roughnessLengthMeters;
        private float backgroundWindX;
        private float backgroundWindZ;
        private byte surfaceClass;
        private long lastUpdatedTick = Long.MIN_VALUE;
    }
}
