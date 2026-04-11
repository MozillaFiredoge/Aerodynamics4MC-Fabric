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
    private static final float FLOW_RELAXATION_PER_SECOND = 1.0f / 240.0f;
    private static final float AIR_TEMPERATURE_RELAXATION_PER_SECOND = 1.0f / 1200.0f;
    private static final float HUMIDITY_RELAXATION_PER_SECOND = 1.0f / 900.0f;
    private static final float DEEP_GROUND_RELAXATION_PER_SECOND = 1.0f / 3600.0f;
    private static final float SURFACE_RELAXATION_PER_SECOND = 1.0f / 1800.0f;
    private static final float FLOW_DIFFUSION_BLEND = 0.16f;
    private static final float THERMAL_DIFFUSION_BLEND = 0.10f;
    private static final float HUMIDITY_DIFFUSION_BLEND = 0.08f;
    private static final float SOLAR_SURFACE_HEATING_K = 18.0f;
    private static final float CLEAR_SKY_COOLING_K = 8.0f;
    private static final float MAX_DRIVER_WIND_MPS = 14.0f;
    private static final float MAX_DYNAMIC_WIND_MPS = 18.0f;
    private static final float MAX_HUMIDITY_RESPONSE = 0.20f;
    private static final float EVAPORATION_SURFACE_DELTA_SCALE = 0.01f;
    private static final float BASE_ROUGHNESS_DRAG_PER_SECOND = 0.0025f;
    private static final float ROUGHNESS_DRAG_SCALE_PER_SECOND = 0.010f;
    private static final float MAX_ROUGHNESS_DRAG = 0.22f;

    private final int cellSizeBlocks;
    private final int radiusCells;
    private final int updateIntervalTicks;
    private final Map<Long, CellState> cells = new HashMap<>();
    private ServerWorld currentWorld;
    private SeedTerrainProvider currentProvider;
    private WorldScaleDriver currentDriver;
    private int centerCellX;
    private int centerCellZ;
    private long lastRefreshTick = Long.MIN_VALUE;
    private long currentRefreshTick = Long.MIN_VALUE;
    private float currentDeltaSeconds = 1.0f;
    private float currentSolarAltitude = 0.0f;
    private float currentClearSky = 1.0f;
    private float currentRainGradient = 0.0f;
    private float currentThunderGradient = 0.0f;

    BackgroundMetGrid(int cellSizeBlocks, int radiusCells, int updateIntervalTicks) {
        this.cellSizeBlocks = cellSizeBlocks;
        this.radiusCells = radiusCells;
        this.updateIntervalTicks = Math.max(1, updateIntervalTicks);
    }

    synchronized void refresh(
        ServerWorld world,
        AeroServerRuntime.WorldEnvironmentSnapshot environmentSnapshot,
        BlockPos focus,
        long tickCounter,
        float dtSeconds,
        SeedTerrainProvider provider,
        WorldScaleDriver driver
    ) {
        currentWorld = world;
        currentProvider = provider;
        currentDriver = driver;
        centerCellX = Math.floorDiv(focus.getX(), cellSizeBlocks);
        centerCellZ = Math.floorDiv(focus.getZ(), cellSizeBlocks);
        boolean updateDue = lastRefreshTick == Long.MIN_VALUE || tickCounter - lastRefreshTick >= updateIntervalTicks;
        if (updateDue) {
            currentDeltaSeconds = lastRefreshTick == Long.MIN_VALUE
                ? dtSeconds
                : Math.max(1L, tickCounter - lastRefreshTick) * dtSeconds;
            lastRefreshTick = tickCounter;
            currentRefreshTick = tickCounter;
        } else {
            currentDeltaSeconds = 0.0f;
            currentRefreshTick = lastRefreshTick;
        }

        long timeOfDay = environmentSnapshot == null ? world.getTimeOfDay() : environmentSnapshot.timeOfDay();
        float rainGradient = environmentSnapshot == null ? world.getRainGradient(1.0f) : environmentSnapshot.rainGradient();
        float thunderGradient = environmentSnapshot == null ? world.getThunderGradient(1.0f) : environmentSnapshot.thunderGradient();
        currentRainGradient = rainGradient;
        currentThunderGradient = thunderGradient;
        float dayPhase = (float) Math.floorMod(timeOfDay, 24000L) / 24000.0f;
        currentSolarAltitude = Math.max(0.0f, (float) Math.sin(dayPhase * (float) (Math.PI * 2.0)));
        currentClearSky = MathHelper.clamp(1.0f - 0.65f * rainGradient - 0.25f * thunderGradient, 0.15f, 1.0f);

        if (updateDue) {
            ensureActiveCells();
            advanceDynamicField();
        }

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

    private void ensureActiveCells() {
        for (int cx = centerCellX - radiusCells; cx <= centerCellX + radiusCells; cx++) {
            for (int cz = centerCellZ - radiusCells; cz <= centerCellZ + radiusCells; cz++) {
                ensureCell(cx, cz);
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
            state.humidity,
            state.convectiveHeatingKelvin,
            state.convectiveMoistening,
            state.convectiveInflowX,
            state.convectiveInflowZ,
            state.convectiveEnvelope,
            state.surfaceClass
        );
    }

    synchronized int cellCount() {
        return cells.size();
    }

    synchronized Snapshot snapshot() {
        int gridWidth = radiusCells * 2 + 1;
        int cellCount = gridWidth * gridWidth;
        float[] terrainHeightBlocks = new float[cellCount];
        float[] biomeTemperature = new float[cellCount];
        float[] roughnessLengthMeters = new float[cellCount];
        byte[] surfaceClass = new byte[cellCount];
        float[] ambientAirTemperatureKelvin = new float[cellCount];
        float[] deepGroundTemperatureKelvin = new float[cellCount];
        float[] surfaceTemperatureKelvin = new float[cellCount];
        float[] windX = new float[cellCount];
        float[] windZ = new float[cellCount];
        float[] humidity = new float[cellCount];
        float[] vorticity = new float[cellCount];
        float[] divergence = new float[cellCount];
        float[] temperatureAnomaly = new float[cellCount];

        for (int cx = centerCellX - radiusCells; cx <= centerCellX + radiusCells; cx++) {
            int localX = cx - (centerCellX - radiusCells);
            for (int cz = centerCellZ - radiusCells; cz <= centerCellZ + radiusCells; cz++) {
                int localZ = cz - (centerCellZ - radiusCells);
                int cellIndex = localX * gridWidth + localZ;
                CellState cell = cells.get(pack(cx, cz));
                if (cell == null) {
                    continue;
                }
                terrainHeightBlocks[cellIndex] = cell.terrainHeightBlocks;
                biomeTemperature[cellIndex] = cell.biomeTemperature;
                roughnessLengthMeters[cellIndex] = cell.roughnessLengthMeters;
                surfaceClass[cellIndex] = cell.surfaceClass;
                ambientAirTemperatureKelvin[cellIndex] = cell.ambientAirTemperatureKelvin;
                deepGroundTemperatureKelvin[cellIndex] = cell.deepGroundTemperatureKelvin;
                surfaceTemperatureKelvin[cellIndex] = cell.surfaceTemperatureKelvin;
                windX[cellIndex] = cell.backgroundWindX;
                windZ[cellIndex] = cell.backgroundWindZ;
                humidity[cellIndex] = cell.humidity;
            }
        }

        populateDiagnostics(
            gridWidth,
            cellSizeBlocks,
            ambientAirTemperatureKelvin,
            windX,
            windZ,
            vorticity,
            divergence,
            temperatureAnomaly
        );

        WorldScaleDriver.Snapshot driverSnapshot = currentDriver == null ? null : currentDriver.snapshot();
        return new Snapshot(
            gridWidth,
            cellSizeBlocks,
            radiusCells,
            centerCellX,
            centerCellZ,
            currentRefreshTick,
            currentDeltaSeconds,
            currentSolarAltitude,
            currentClearSky,
            currentRainGradient,
            currentThunderGradient,
            driverSnapshot,
            terrainHeightBlocks,
            biomeTemperature,
            roughnessLengthMeters,
            surfaceClass,
            ambientAirTemperatureKelvin,
            deepGroundTemperatureKelvin,
            surfaceTemperatureKelvin,
            windX,
            windZ,
            humidity,
            vorticity,
            divergence,
            temperatureAnomaly
        );
    }

    private void populateDiagnostics(
        int gridWidth,
        int cellSizeBlocks,
        float[] ambientAirTemperatureKelvin,
        float[] windX,
        float[] windZ,
        float[] vorticity,
        float[] divergence,
        float[] temperatureAnomaly
    ) {
        float meanAmbient = 0.0f;
        for (float value : ambientAirTemperatureKelvin) {
            meanAmbient += value;
        }
        meanAmbient /= Math.max(1, ambientAirTemperatureKelvin.length);
        float dxMeters = Math.max(1.0f, cellSizeBlocks);

        for (int x = 0; x < gridWidth; x++) {
            int west = Math.max(0, x - 1);
            int east = Math.min(gridWidth - 1, x + 1);
            float xSpanMeters = Math.max(1.0f, (east - west) * dxMeters);
            for (int z = 0; z < gridWidth; z++) {
                int north = Math.max(0, z - 1);
                int south = Math.min(gridWidth - 1, z + 1);
                float zSpanMeters = Math.max(1.0f, (south - north) * dxMeters);
                int index = x * gridWidth + z;
                float dWindXdx = (windX[east * gridWidth + z] - windX[west * gridWidth + z]) / xSpanMeters;
                float dWindZdx = (windZ[east * gridWidth + z] - windZ[west * gridWidth + z]) / xSpanMeters;
                float dWindXdz = (windX[x * gridWidth + south] - windX[x * gridWidth + north]) / zSpanMeters;
                float dWindZdz = (windZ[x * gridWidth + south] - windZ[x * gridWidth + north]) / zSpanMeters;
                divergence[index] = dWindXdx + dWindZdz;
                vorticity[index] = dWindZdx - dWindXdz;
                temperatureAnomaly[index] = ambientAirTemperatureKelvin[index] - meanAmbient;
            }
        }
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
        seedDynamicState(cell, cellX, cellZ);
        cell.lastUpdatedTick = currentRefreshTick;
    }

    private void advanceDynamicField() {
        if (cells.isEmpty()) {
            return;
        }
        Map<Long, StateSample> previous = new HashMap<>(cells.size());
        for (Map.Entry<Long, CellState> entry : cells.entrySet()) {
            CellState state = entry.getValue();
            previous.put(entry.getKey(), new StateSample(
                state.backgroundWindX,
                state.backgroundWindZ,
                state.ambientAirTemperatureKelvin,
                state.humidity
            ));
        }
        for (int cx = centerCellX - radiusCells; cx <= centerCellX + radiusCells; cx++) {
            for (int cz = centerCellZ - radiusCells; cz <= centerCellZ + radiusCells; cz++) {
                CellState cell = ensureCell(cx, cz);
                WorldScaleTarget target = targetState(cell, cx, cz);
                StateSample prev = sampleState(previous, cx, cz);
                float backtraceX = cx - (prev.windX * currentDeltaSeconds) / cellSizeBlocks;
                float backtraceZ = cz - (prev.windZ * currentDeltaSeconds) / cellSizeBlocks;
                StateSample advected = sampleState(previous, backtraceX, backtraceZ);
                StateSample neighborMean = sampleNeighborMean(previous, cx, cz);

                float roughnessDrag = MathHelper.clamp(
                    currentDeltaSeconds * (BASE_ROUGHNESS_DRAG_PER_SECOND + cell.roughnessLengthMeters * ROUGHNESS_DRAG_SCALE_PER_SECOND),
                    0.0f,
                    MAX_ROUGHNESS_DRAG
                );
                float nextWindX = mix(advected.windX, neighborMean.windX, FLOW_DIFFUSION_BLEND);
                float nextWindZ = mix(advected.windZ, neighborMean.windZ, FLOW_DIFFUSION_BLEND);
                nextWindX = relax(nextWindX, target.targetWindX, currentDeltaSeconds, FLOW_RELAXATION_PER_SECOND);
                nextWindZ = relax(nextWindZ, target.targetWindZ, currentDeltaSeconds, FLOW_RELAXATION_PER_SECOND);
                nextWindX *= (1.0f - roughnessDrag);
                nextWindZ *= (1.0f - roughnessDrag);
                nextWindX = MathHelper.clamp(nextWindX, -MAX_DYNAMIC_WIND_MPS, MAX_DYNAMIC_WIND_MPS);
                nextWindZ = MathHelper.clamp(nextWindZ, -MAX_DYNAMIC_WIND_MPS, MAX_DYNAMIC_WIND_MPS);

                float nextAmbient = mix(advected.ambientAirTemperatureKelvin, neighborMean.ambientAirTemperatureKelvin, THERMAL_DIFFUSION_BLEND);
                nextAmbient = relax(nextAmbient, target.targetAmbientAirTemperatureKelvin, currentDeltaSeconds, AIR_TEMPERATURE_RELAXATION_PER_SECOND);

                float nextHumidity = mix(advected.humidity, neighborMean.humidity, HUMIDITY_DIFFUSION_BLEND);
                float evaporationBoost = Math.max(0.0f, cell.surfaceTemperatureKelvin - nextAmbient) * EVAPORATION_SURFACE_DELTA_SCALE;
                float humidityTarget = MathHelper.clamp(
                    target.targetHumidity + evaporationBoost + currentRainGradient * 0.05f,
                    0.0f,
                    1.0f
                );
                nextHumidity = MathHelper.clamp(
                    relax(nextHumidity, humidityTarget, currentDeltaSeconds, HUMIDITY_RELAXATION_PER_SECOND),
                    0.0f,
                    1.0f
                );

                float targetSurfaceTemperatureKelvin = nextAmbient
                    + currentSolarAltitude * currentClearSky * SOLAR_SURFACE_HEATING_K
                    - (1.0f - currentSolarAltitude) * currentClearSky * CLEAR_SKY_COOLING_K
                    - nextHumidity * 2.0f
                    - currentRainGradient * 3.0f;
                cell.ambientAirTemperatureKelvin = nextAmbient;
                cell.deepGroundTemperatureKelvin = relax(
                    cell.deepGroundTemperatureKelvin,
                    nextAmbient + DEEP_GROUND_OFFSET_K,
                    currentDeltaSeconds,
                    DEEP_GROUND_RELAXATION_PER_SECOND
                );
                cell.surfaceTemperatureKelvin = relax(cell.surfaceTemperatureKelvin, targetSurfaceTemperatureKelvin, currentDeltaSeconds);
                cell.backgroundWindX = nextWindX;
                cell.backgroundWindZ = nextWindZ;
                cell.humidity = nextHumidity;
                cell.convectiveHeatingKelvin = target.convectiveHeatingKelvin;
                cell.convectiveMoistening = target.convectiveMoistening;
                cell.convectiveInflowX = target.convectiveInflowX;
                cell.convectiveInflowZ = target.convectiveInflowZ;
                cell.convectiveEnvelope = target.convectiveEnvelope;
                cell.lastUpdatedTick = currentRefreshTick;
            }
        }
    }

    private void seedDynamicState(CellState cell, int cellX, int cellZ) {
        WorldScaleTarget target = targetState(cell, cellX, cellZ);
        if (!Float.isFinite(cell.ambientAirTemperatureKelvin) || cell.ambientAirTemperatureKelvin <= 0.0f) {
            cell.ambientAirTemperatureKelvin = target.targetAmbientAirTemperatureKelvin;
        }
        if (!Float.isFinite(cell.deepGroundTemperatureKelvin) || cell.deepGroundTemperatureKelvin <= 0.0f) {
            cell.deepGroundTemperatureKelvin = target.targetAmbientAirTemperatureKelvin + DEEP_GROUND_OFFSET_K;
        }
        if (!Float.isFinite(cell.surfaceTemperatureKelvin) || cell.surfaceTemperatureKelvin <= 0.0f) {
            cell.surfaceTemperatureKelvin = target.targetAmbientAirTemperatureKelvin
                + currentSolarAltitude * currentClearSky * SOLAR_SURFACE_HEATING_K
                - (1.0f - currentSolarAltitude) * currentClearSky * CLEAR_SKY_COOLING_K;
        }
        if (!Float.isFinite(cell.backgroundWindX)) {
            cell.backgroundWindX = target.targetWindX;
        }
        if (!Float.isFinite(cell.backgroundWindZ)) {
            cell.backgroundWindZ = target.targetWindZ;
        }
        if (!Float.isFinite(cell.humidity) || cell.humidity < 0.0f || cell.humidity > 1.0f) {
            cell.humidity = target.targetHumidity;
        }
        cell.convectiveHeatingKelvin = target.convectiveHeatingKelvin;
        cell.convectiveMoistening = target.convectiveMoistening;
        cell.convectiveInflowX = target.convectiveInflowX;
        cell.convectiveInflowZ = target.convectiveInflowZ;
        cell.convectiveEnvelope = target.convectiveEnvelope;
    }

    private StateSample sampleNeighborMean(Map<Long, StateSample> previous, int cellX, int cellZ) {
        StateSample center = sampleState(previous, cellX, cellZ);
        StateSample west = sampleState(previous, cellX - 1.0f, cellZ);
        StateSample east = sampleState(previous, cellX + 1.0f, cellZ);
        StateSample north = sampleState(previous, cellX, cellZ - 1.0f);
        StateSample south = sampleState(previous, cellX, cellZ + 1.0f);
        return new StateSample(
            (center.windX + west.windX + east.windX + north.windX + south.windX) / 5.0f,
            (center.windZ + west.windZ + east.windZ + north.windZ + south.windZ) / 5.0f,
            (center.ambientAirTemperatureKelvin + west.ambientAirTemperatureKelvin + east.ambientAirTemperatureKelvin + north.ambientAirTemperatureKelvin + south.ambientAirTemperatureKelvin) / 5.0f,
            MathHelper.clamp(
                (center.humidity + west.humidity + east.humidity + north.humidity + south.humidity) / 5.0f,
                0.0f,
                1.0f
            )
        );
    }

    private StateSample sampleState(Map<Long, StateSample> previous, float cellX, float cellZ) {
        int x0 = MathHelper.floor(cellX);
        int z0 = MathHelper.floor(cellZ);
        int x1 = x0 + 1;
        int z1 = z0 + 1;
        float tx = cellX - x0;
        float tz = cellZ - z0;

        StateSample s00 = stateAt(previous, x0, z0);
        StateSample s10 = stateAt(previous, x1, z0);
        StateSample s01 = stateAt(previous, x0, z1);
        StateSample s11 = stateAt(previous, x1, z1);

        float windX0 = MathHelper.lerp(tx, s00.windX, s10.windX);
        float windX1 = MathHelper.lerp(tx, s01.windX, s11.windX);
        float windZ0 = MathHelper.lerp(tx, s00.windZ, s10.windZ);
        float windZ1 = MathHelper.lerp(tx, s01.windZ, s11.windZ);
        float temp0 = MathHelper.lerp(tx, s00.ambientAirTemperatureKelvin, s10.ambientAirTemperatureKelvin);
        float temp1 = MathHelper.lerp(tx, s01.ambientAirTemperatureKelvin, s11.ambientAirTemperatureKelvin);
        float humidity0 = MathHelper.lerp(tx, s00.humidity, s10.humidity);
        float humidity1 = MathHelper.lerp(tx, s01.humidity, s11.humidity);
        return new StateSample(
            MathHelper.lerp(tz, windX0, windX1),
            MathHelper.lerp(tz, windZ0, windZ1),
            MathHelper.lerp(tz, temp0, temp1),
            MathHelper.clamp(MathHelper.lerp(tz, humidity0, humidity1), 0.0f, 1.0f)
        );
    }

    private StateSample stateAt(Map<Long, StateSample> previous, int cellX, int cellZ) {
        StateSample sample = previous.get(pack(cellX, cellZ));
        if (sample != null) {
            return sample;
        }
        CellState cell = ensureCell(cellX, cellZ);
        WorldScaleTarget target = targetState(cell, cellX, cellZ);
        return new StateSample(
            target.targetWindX,
            target.targetWindZ,
            target.targetAmbientAirTemperatureKelvin,
            target.targetHumidity
        );
    }

    private WorldScaleTarget targetState(CellState cell, int cellX, int cellZ) {
        float baseAmbient = BASE_AIR_TEMPERATURE_K
            + (cell.biomeTemperature - 0.8f) * BIOME_TEMPERATURE_SCALE_K
            - Math.max(0.0f, cell.terrainHeightBlocks - currentWorld.getSeaLevel()) * ALTITUDE_LAPSE_RATE_K_PER_BLOCK;
        WorldScaleDriver.Sample driverSample = currentDriver == null ? null : currentDriver.sample(cellX, cellZ);
        float targetWindX = driverSample != null
            ? driverSample.targetWindX()
            : prevailingWindComponent(currentWorld.getSeed(), cellX, cellZ, 0x517cc1b727220a95L);
        float targetWindZ = driverSample != null
            ? driverSample.targetWindZ()
            : prevailingWindComponent(currentWorld.getSeed(), cellX, cellZ, 0x9e3779b97f4a7c15L);
        targetWindX = MathHelper.clamp(targetWindX, -MAX_DRIVER_WIND_MPS, MAX_DRIVER_WIND_MPS);
        targetWindZ = MathHelper.clamp(targetWindZ, -MAX_DRIVER_WIND_MPS, MAX_DRIVER_WIND_MPS);
        float targetAmbient = baseAmbient + (driverSample == null ? 0.0f : driverSample.temperatureBiasKelvin());
        float humidity = MathHelper.clamp(
            (driverSample == null ? 0.50f : driverSample.humidity())
                + currentRainGradient * MAX_HUMIDITY_RESPONSE
                + currentThunderGradient * 0.10f,
            0.0f,
            1.0f
        );
        return new WorldScaleTarget(
            targetWindX,
            targetWindZ,
            targetAmbient,
            humidity,
            driverSample == null ? 0.0f : driverSample.convectiveHeatingKelvin(),
            driverSample == null ? 0.0f : driverSample.convectiveMoistening(),
            driverSample == null ? 0.0f : driverSample.convectiveInflowX(),
            driverSample == null ? 0.0f : driverSample.convectiveInflowZ(),
            driverSample == null ? 0.0f : driverSample.convectiveEnvelope()
        );
    }

    private float relax(float current, float target, float deltaSeconds) {
        if (!Float.isFinite(current) || current <= 0.0f) {
            return target;
        }
        float alpha = MathHelper.clamp(deltaSeconds * SURFACE_RELAXATION_PER_SECOND, 0.0f, 1.0f);
        return MathHelper.lerp(alpha, current, target);
    }

    private float relax(float current, float target, float deltaSeconds, float ratePerSecond) {
        if (!Float.isFinite(current)) {
            return target;
        }
        float alpha = MathHelper.clamp(deltaSeconds * ratePerSecond, 0.0f, 1.0f);
        return MathHelper.lerp(alpha, current, target);
    }

    private float mix(float a, float b, float t) {
        return MathHelper.lerp(MathHelper.clamp(t, 0.0f, 1.0f), a, b);
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
        float humidity,
        float convectiveHeatingKelvin,
        float convectiveMoistening,
        float convectiveInflowX,
        float convectiveInflowZ,
        float convectiveEnvelope,
        byte surfaceClass
    ) {
    }

    record Snapshot(
        int gridWidth,
        int cellSizeBlocks,
        int radiusCells,
        int centerCellX,
        int centerCellZ,
        long tick,
        float deltaSeconds,
        float solarAltitude,
        float clearSky,
        float rainGradient,
        float thunderGradient,
        WorldScaleDriver.Snapshot driver,
        float[] terrainHeightBlocks,
        float[] biomeTemperature,
        float[] roughnessLengthMeters,
        byte[] surfaceClass,
        float[] ambientAirTemperatureKelvin,
        float[] deepGroundTemperatureKelvin,
        float[] surfaceTemperatureKelvin,
        float[] windX,
        float[] windZ,
        float[] humidity,
        float[] vorticity,
        float[] divergence,
        float[] temperatureAnomaly
    ) {
    }

    private static final class CellState {
        private float terrainHeightBlocks;
        private float biomeTemperature;
        private float ambientAirTemperatureKelvin = Float.NaN;
        private float deepGroundTemperatureKelvin = Float.NaN;
        private float surfaceTemperatureKelvin = Float.NaN;
        private float roughnessLengthMeters;
        private float backgroundWindX = Float.NaN;
        private float backgroundWindZ = Float.NaN;
        private float humidity = Float.NaN;
        private float convectiveHeatingKelvin;
        private float convectiveMoistening;
        private float convectiveInflowX;
        private float convectiveInflowZ;
        private float convectiveEnvelope;
        private byte surfaceClass;
        private long lastUpdatedTick = Long.MIN_VALUE;
    }

    private record StateSample(
        float windX,
        float windZ,
        float ambientAirTemperatureKelvin,
        float humidity
    ) {
    }

    private record WorldScaleTarget(
        float targetWindX,
        float targetWindZ,
        float targetAmbientAirTemperatureKelvin,
        float targetHumidity,
        float convectiveHeatingKelvin,
        float convectiveMoistening,
        float convectiveInflowX,
        float convectiveInflowZ,
        float convectiveEnvelope
    ) {
    }
}
