package com.aerodynamics4mc.runtime;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import net.minecraft.server.world.ServerWorld;
import net.minecraft.util.math.MathHelper;

final class WorldScaleDriver {
    private static final float TAU = (float) (Math.PI * 2.0);
    private static final float BASE_FLOW_RELAX_PER_SECOND = 1.0f / 900.0f;
    private static final float THERMAL_RELAX_PER_SECOND = 1.0f / 1800.0f;
    private static final float MOISTURE_RELAX_PER_SECOND = 1.0f / 1200.0f;
    private static final float STORM_RELAX_PER_SECOND = 1.0f / 600.0f;
    private static final long SEASON_PERIOD_TICKS = 24000L * 96L;
    private static final float PLANETARY_WAVE_RADIANS_PER_SECOND = TAU / (24000.0f * 8.0f / 20.0f);
    private static final float DRIVER_SPATIAL_SCALE_X = 0.11f;
    private static final float DRIVER_SPATIAL_SCALE_Z = 0.09f;
    private static final float MAX_DRIVER_WIND_MPS = 12.0f;

    private static final int DEFAULT_CYCLONE_CELL_COUNT = 3;
    private static final float PRESSURE_DOMAIN_CELLS = 384.0f;
    private static final float DRIVER_CELL_SIZE_BLOCKS = 256.0f;
    private static final float CYCLONE_CELL_MIN_RADIUS = 9.0f;
    private static final float CYCLONE_CELL_MAX_RADIUS = 18.0f;
    private static final float CYCLONE_CELL_MAX_SWIRL_MPS = 7.5f;
    private static final float CYCLONE_CELL_MAX_RADIAL_MPS = 2.0f;
    private static final float CYCLONE_CELL_BASE_FLOW_ADVECTION = 0.30f;
    private static final float CYCLONE_CELL_LIFECYCLE_RADIANS_PER_SECOND = TAU / (24000.0f * 2.0f / 20.0f);

    private final long worldSeed;
    private final List<CycloneCell> cycloneCells;
    private long lastTickUpdated = Long.MIN_VALUE;
    private float driverTimeSeconds;
    private float baseFlowX;
    private float baseFlowZ;
    private float airmassTemperatureBias;
    private float airmassMoistureBias;
    private float planetaryWavePhase;
    private float stormActivity;
    private float seasonPhase;

    private WorldScaleDriver(
        long worldSeed,
        float driverTimeSeconds,
        float baseFlowX,
        float baseFlowZ,
        float airmassTemperatureBias,
        float airmassMoistureBias,
        float planetaryWavePhase,
        float stormActivity,
        float seasonPhase,
        List<CycloneCell> cycloneCells
    ) {
        this.worldSeed = worldSeed;
        this.driverTimeSeconds = driverTimeSeconds;
        this.baseFlowX = baseFlowX;
        this.baseFlowZ = baseFlowZ;
        this.airmassTemperatureBias = airmassTemperatureBias;
        this.airmassMoistureBias = airmassMoistureBias;
        this.planetaryWavePhase = planetaryWavePhase;
        this.stormActivity = stormActivity;
        this.seasonPhase = seasonPhase;
        this.cycloneCells = new ArrayList<>(cycloneCells);
    }

    static WorldScaleDriver loadOrCreate(Path path, ServerWorld world) {
        if (path != null && Files.isRegularFile(path)) {
            try {
                return fromLines(world.getSeed(), Files.readAllLines(path, StandardCharsets.UTF_8));
            } catch (IOException ignored) {
                // Fall back to deterministic initialization below.
            }
        }
        return createDefault(world.getSeed());
    }

    private static WorldScaleDriver createDefault(long worldSeed) {
        float baseDirection = seededUnit(worldSeed, 0x15f11f73d3e4a5b1L) * TAU;
        float baseSpeed = 1.8f + seededUnit(worldSeed, 0x6a09e667f3bcc909L) * 2.2f;
        float baseFlowX = MathHelper.cos(baseDirection) * baseSpeed;
        float baseFlowZ = MathHelper.sin(baseDirection) * baseSpeed;
        float airmassTemperatureBias = (seededUnit(worldSeed, 0xbb67ae8584caa73bL) - 0.5f) * 6.0f;
        float airmassMoistureBias = MathHelper.clamp(
            0.45f + (seededUnit(worldSeed, 0x3c6ef372fe94f82bL) - 0.5f) * 0.30f,
            0.05f,
            0.95f
        );
        float planetaryWavePhase = seededUnit(worldSeed, 0xa54ff53a5f1d36f1L) * TAU;
        float stormActivity = MathHelper.clamp(
            0.15f + seededUnit(worldSeed, 0x510e527fade682d1L) * 0.25f,
            0.0f,
            1.0f
        );
        float seasonPhase = seededUnit(worldSeed, 0x9b05688c2b3e6c1fL);
        return new WorldScaleDriver(
            worldSeed,
            0.0f,
            baseFlowX,
            baseFlowZ,
            airmassTemperatureBias,
            airmassMoistureBias,
            planetaryWavePhase,
            stormActivity,
            seasonPhase,
            createDefaultCycloneCells(worldSeed)
        );
    }

    private static WorldScaleDriver fromLines(long worldSeed, List<String> lines) {
        WorldScaleDriver driver = createDefault(worldSeed);
        for (String rawLine : lines) {
            if (rawLine == null) {
                continue;
            }
            String line = rawLine.trim();
            if (line.isEmpty() || line.startsWith("#")) {
                continue;
            }
            int separator = line.indexOf('=');
            if (separator <= 0 || separator == line.length() - 1) {
                continue;
            }
            String key = line.substring(0, separator).trim();
            String value = line.substring(separator + 1).trim();
            try {
                switch (key) {
                    case "driver_time_seconds" -> driver.driverTimeSeconds = Float.parseFloat(value);
                    case "base_flow_x" -> driver.baseFlowX = Float.parseFloat(value);
                    case "base_flow_z" -> driver.baseFlowZ = Float.parseFloat(value);
                    case "airmass_temperature_bias" -> driver.airmassTemperatureBias = Float.parseFloat(value);
                    case "airmass_moisture_bias" -> driver.airmassMoistureBias = Float.parseFloat(value);
                    case "planetary_wave_phase" -> driver.planetaryWavePhase = Float.parseFloat(value);
                    case "storm_activity" -> driver.stormActivity = Float.parseFloat(value);
                    case "season_phase" -> driver.seasonPhase = Float.parseFloat(value);
                    default -> {
                    }
                }
            } catch (NumberFormatException ignored) {
                // Ignore malformed entries and keep deterministic defaults.
            }
        }
        driver.cycloneCells.clear();
        driver.cycloneCells.addAll(parseCycloneCells(worldSeed, lines));
        driver.airmassMoistureBias = MathHelper.clamp(driver.airmassMoistureBias, 0.0f, 1.0f);
        driver.stormActivity = MathHelper.clamp(driver.stormActivity, 0.0f, 1.0f);
        driver.seasonPhase = wrap01(driver.seasonPhase);
        driver.planetaryWavePhase = wrapTau(driver.planetaryWavePhase);
        return driver;
    }

    synchronized void save(Path path) throws IOException {
        if (path == null) {
            return;
        }
        Files.createDirectories(path.getParent());
        StringBuilder builder = new StringBuilder();
        appendProperty(builder, "driver_time_seconds", driverTimeSeconds);
        appendProperty(builder, "base_flow_x", baseFlowX);
        appendProperty(builder, "base_flow_z", baseFlowZ);
        appendProperty(builder, "airmass_temperature_bias", airmassTemperatureBias);
        appendProperty(builder, "airmass_moisture_bias", airmassMoistureBias);
        appendProperty(builder, "planetary_wave_phase", planetaryWavePhase);
        appendProperty(builder, "storm_activity", stormActivity);
        appendProperty(builder, "season_phase", seasonPhase);
        builder.append("cyclone_cell_count=").append(cycloneCells.size()).append('\n');
        for (int i = 0; i < cycloneCells.size(); i++) {
            CycloneCell cell = cycloneCells.get(i);
            appendProperty(builder, "cyclone_cell_" + i + "_center_x", cell.centerCellX);
            appendProperty(builder, "cyclone_cell_" + i + "_center_z", cell.centerCellZ);
            appendProperty(builder, "cyclone_cell_" + i + "_radius_cells", cell.radiusCells);
            appendProperty(builder, "cyclone_cell_" + i + "_intensity", cell.intensity);
            appendProperty(builder, "cyclone_cell_" + i + "_pressure_sign", cell.pressureSign);
            appendProperty(builder, "cyclone_cell_" + i + "_drift_x_cells_per_second", cell.driftCellsPerSecondX);
            appendProperty(builder, "cyclone_cell_" + i + "_drift_z_cells_per_second", cell.driftCellsPerSecondZ);
            appendProperty(builder, "cyclone_cell_" + i + "_lifecycle_phase", cell.lifecyclePhase);
            appendProperty(builder, "cyclone_cell_" + i + "_warm_core_bias_kelvin", cell.warmCoreBiasKelvin);
            appendProperty(builder, "cyclone_cell_" + i + "_moisture_core_bias", cell.moistureCoreBias);
        }
        Files.writeString(path, builder.toString(), StandardCharsets.UTF_8);
    }

    synchronized void advance(
        ServerWorld world,
        AeroServerRuntime.WorldEnvironmentSnapshot environmentSnapshot,
        long tickCounter,
        float dtSeconds
    ) {
        float elapsedSeconds = lastTickUpdated == Long.MIN_VALUE
            ? Math.max(1.0e-3f, dtSeconds)
            : Math.max(1L, tickCounter - lastTickUpdated) * dtSeconds;
        lastTickUpdated = tickCounter;
        driverTimeSeconds += elapsedSeconds;

        long worldTime = environmentSnapshot == null ? world.getTimeOfDay() : environmentSnapshot.timeOfDay();
        seasonPhase = wrap01(worldTime / (float) SEASON_PERIOD_TICKS);
        planetaryWavePhase = wrapTau(planetaryWavePhase + elapsedSeconds * PLANETARY_WAVE_RADIANS_PER_SECOND);

        float rain = environmentSnapshot == null ? world.getRainGradient(1.0f) : environmentSnapshot.rainGradient();
        float thunder = environmentSnapshot == null ? world.getThunderGradient(1.0f) : environmentSnapshot.thunderGradient();

        float preferredDirection = seededUnit(worldSeed, 0x15f11f73d3e4a5b1L) * TAU
            + 0.35f * MathHelper.sin(planetaryWavePhase * 0.35f)
            + 0.20f * MathHelper.cos(planetaryWavePhase * 0.18f + seededUnit(worldSeed, 0x428a2f98d728ae22L) * TAU);
        float preferredSpeed = 1.8f
            + seededUnit(worldSeed, 0x6a09e667f3bcc909L) * 2.2f
            + rain * 0.8f
            + thunder * 1.2f;
        float targetFlowX = MathHelper.cos(preferredDirection) * preferredSpeed;
        float targetFlowZ = MathHelper.sin(preferredDirection) * preferredSpeed;
        baseFlowX = relax(baseFlowX, targetFlowX, elapsedSeconds, BASE_FLOW_RELAX_PER_SECOND);
        baseFlowZ = relax(baseFlowZ, targetFlowZ, elapsedSeconds, BASE_FLOW_RELAX_PER_SECOND);

        float seededTempBias = (seededUnit(worldSeed, 0xbb67ae8584caa73bL) - 0.5f) * 6.0f;
        float seasonalTempBias = MathHelper.sin(seasonPhase * TAU) * 4.5f;
        float weatherTempBias = -(rain * 1.5f + thunder * 2.0f);
        float waveTempBias = 1.2f * MathHelper.sin(planetaryWavePhase * 0.6f);
        float targetTemperatureBias = seededTempBias + seasonalTempBias + weatherTempBias + waveTempBias;
        airmassTemperatureBias = relax(
            airmassTemperatureBias,
            targetTemperatureBias,
            elapsedSeconds,
            THERMAL_RELAX_PER_SECOND
        );

        float seededMoistureBias = 0.45f + (seededUnit(worldSeed, 0x3c6ef372fe94f82bL) - 0.5f) * 0.30f;
        float weatherMoistureBias = rain * 0.30f + thunder * 0.20f;
        float waveMoistureBias = 0.10f * MathHelper.cos(planetaryWavePhase * 0.55f);
        float targetMoistureBias = MathHelper.clamp(
            seededMoistureBias + weatherMoistureBias + waveMoistureBias,
            0.05f,
            0.95f
        );
        airmassMoistureBias = MathHelper.clamp(
            relax(airmassMoistureBias, targetMoistureBias, elapsedSeconds, MOISTURE_RELAX_PER_SECOND),
            0.0f,
            1.0f
        );

        float targetStormActivity = MathHelper.clamp(
            0.20f + 0.45f * airmassMoistureBias + 0.25f * rain + 0.25f * thunder,
            0.0f,
            1.0f
        );
        stormActivity = MathHelper.clamp(
            relax(stormActivity, targetStormActivity, elapsedSeconds, STORM_RELAX_PER_SECOND),
            0.0f,
            1.0f
        );

        for (CycloneCell cell : cycloneCells) {
            cell.advance(elapsedSeconds, baseFlowX, baseFlowZ);
        }
    }

    synchronized Sample sample(int cellX, int cellZ) {
        float sampleX = cellX * DRIVER_SPATIAL_SCALE_X;
        float sampleZ = cellZ * DRIVER_SPATIAL_SCALE_Z;
        float waveA = MathHelper.sin(sampleX + planetaryWavePhase * 0.70f);
        float waveB = MathHelper.cos(sampleZ - planetaryWavePhase * 0.45f);
        float eddy = MathHelper.sin((sampleX + sampleZ) * 0.55f + planetaryWavePhase * 0.25f);

        float targetWindX = baseFlowX + 0.90f * waveA + 0.35f * eddy;
        float targetWindZ = baseFlowZ + 0.90f * waveB - 0.35f * eddy;
        float temperatureBiasKelvin = airmassTemperatureBias + 1.4f * waveA + 0.8f * waveB;
        float humidity = MathHelper.clamp(airmassMoistureBias + 0.08f * waveB - 0.05f * eddy, 0.0f, 1.0f);

        for (CycloneCell cell : cycloneCells) {
            CycloneContribution contribution = cell.sample(cellX, cellZ, stormActivity);
            targetWindX += contribution.windX();
            targetWindZ += contribution.windZ();
            temperatureBiasKelvin += contribution.temperatureBiasKelvin();
            humidity += contribution.humidityBias();
        }

        targetWindX = MathHelper.clamp(targetWindX, -MAX_DRIVER_WIND_MPS, MAX_DRIVER_WIND_MPS);
        targetWindZ = MathHelper.clamp(targetWindZ, -MAX_DRIVER_WIND_MPS, MAX_DRIVER_WIND_MPS);
        humidity = MathHelper.clamp(humidity, 0.0f, 1.0f);
        return new Sample(targetWindX, targetWindZ, temperatureBiasKelvin, humidity, stormActivity);
    }

    synchronized Snapshot snapshot() {
        List<CycloneCellSnapshot> systems = new ArrayList<>(cycloneCells.size());
        for (CycloneCell cell : cycloneCells) {
            systems.add(cell.snapshot());
        }
        return new Snapshot(
            driverTimeSeconds,
            baseFlowX,
            baseFlowZ,
            airmassTemperatureBias,
            airmassMoistureBias,
            planetaryWavePhase,
            stormActivity,
            seasonPhase,
            List.copyOf(systems)
        );
    }

    private static List<CycloneCell> createDefaultCycloneCells(long worldSeed) {
        List<CycloneCell> cells = new ArrayList<>(DEFAULT_CYCLONE_CELL_COUNT);
        for (int i = 0; i < DEFAULT_CYCLONE_CELL_COUNT; i++) {
            long salt = 0x632be59bd9b4e019L + (long) i * 0x9e3779b97f4a7c15L;
            float centerX = seededUnit(worldSeed, salt ^ 0x94d049bb133111ebL) * PRESSURE_DOMAIN_CELLS;
            float centerZ = seededUnit(worldSeed, salt ^ 0x2545f4914f6cdd1dL) * PRESSURE_DOMAIN_CELLS;
            float radiusCells = MathHelper.lerp(
                seededUnit(worldSeed, salt ^ 0x4cf5ad432745937fL),
                CYCLONE_CELL_MIN_RADIUS,
                CYCLONE_CELL_MAX_RADIUS
            );
            float intensity = MathHelper.lerp(
                seededUnit(worldSeed, salt ^ 0x6c8e9cf570932bd5L),
                0.55f,
                0.95f
            );
            float pressureSign = (i & 1) == 0 ? -1.0f : 1.0f;
            float warmCoreBiasKelvin = pressureSign < 0.0f
                ? MathHelper.lerp(seededUnit(worldSeed, salt ^ 0xcbbb9d5dc1059ed8L), 1.0f, 3.6f)
                : -MathHelper.lerp(seededUnit(worldSeed, salt ^ 0x629a292a367cd507L), 0.6f, 2.2f);
            float moistureCoreBias = pressureSign < 0.0f
                ? MathHelper.lerp(seededUnit(worldSeed, salt ^ 0x9159015a3070dd17L), 0.04f, 0.14f)
                : -MathHelper.lerp(seededUnit(worldSeed, salt ^ 0x152fecd8f70e5939L), 0.03f, 0.10f);
            float driftDirection = seededUnit(worldSeed, salt ^ 0xa4093822299f31d0L) * TAU;
            float driftSpeedCellsPerSecond = MathHelper.lerp(
                seededUnit(worldSeed, salt ^ 0x082efa98ec4e6c89L),
                0.0012f,
                0.0040f
            );
            float driftX = MathHelper.cos(driftDirection) * driftSpeedCellsPerSecond;
            float driftZ = MathHelper.sin(driftDirection) * driftSpeedCellsPerSecond;
            float lifecyclePhase = seededUnit(worldSeed, salt ^ 0x452821e638d01377L) * TAU;
            cells.add(new CycloneCell(
                centerX,
                centerZ,
                radiusCells,
                intensity,
                pressureSign,
                driftX,
                driftZ,
                lifecyclePhase,
                warmCoreBiasKelvin,
                moistureCoreBias
            ));
        }
        return cells;
    }

    private static List<CycloneCell> parseCycloneCells(long worldSeed, List<String> lines) {
        int count = DEFAULT_CYCLONE_CELL_COUNT;
        for (String rawLine : lines) {
            if (rawLine == null) {
                continue;
            }
            String line = rawLine.trim();
            int separator = line.indexOf('=');
            if (separator <= 0 || separator == line.length() - 1) {
                continue;
            }
            String key = line.substring(0, separator).trim();
            if ("cyclone_cell_count".equals(key) || "pressure_cell_count".equals(key)) {
                try {
                    count = Math.max(0, Integer.parseInt(line.substring(separator + 1).trim()));
                } catch (NumberFormatException ignored) {
                    count = DEFAULT_CYCLONE_CELL_COUNT;
                }
                break;
            }
        }

        List<CycloneCell> defaults = createDefaultCycloneCells(worldSeed);
        List<CycloneCell> cells = new ArrayList<>(count);
        for (int i = 0; i < count; i++) {
            CycloneCell fallback = defaults.get(i % defaults.size());
            cells.add(fallback.copy());
        }
        if (count == 0) {
            return cells;
        }

        for (String rawLine : lines) {
            if (rawLine == null) {
                continue;
            }
            String line = rawLine.trim();
            if (line.isEmpty() || line.startsWith("#")) {
                continue;
            }
            int separator = line.indexOf('=');
            if (separator <= 0 || separator == line.length() - 1) {
                continue;
            }
            String key = line.substring(0, separator).trim();
            if (!key.startsWith("pressure_cell_") && !key.startsWith("cyclone_cell_")) {
                continue;
            }
            String suffix = key.startsWith("cyclone_cell_")
                ? key.substring("cyclone_cell_".length())
                : key.substring("pressure_cell_".length());
            int nextSeparator = suffix.indexOf('_');
            if (nextSeparator <= 0 || nextSeparator == suffix.length() - 1) {
                continue;
            }
            int index;
            try {
                index = Integer.parseInt(suffix.substring(0, nextSeparator));
            } catch (NumberFormatException ignored) {
                continue;
            }
            if (index < 0 || index >= cells.size()) {
                continue;
            }
            String field = suffix.substring(nextSeparator + 1);
            String value = line.substring(separator + 1).trim();
            try {
                cells.get(index).set(field, Float.parseFloat(value));
            } catch (NumberFormatException ignored) {
                // Ignore malformed entries and keep defaults.
            }
        }
        return cells;
    }

    private static void appendProperty(StringBuilder builder, String key, float value) {
        builder.append(key)
            .append('=')
            .append(String.format(Locale.ROOT, "%.6f", value))
            .append('\n');
    }

    private static float seededUnit(long seed, long salt) {
        long h = seed ^ salt;
        h ^= h >>> 33;
        h *= 0xff51afd7ed558ccdl;
        h ^= h >>> 33;
        h *= 0xc4ceb9fe1a85ec53L;
        h ^= h >>> 33;
        return ((h >>> 40) & 0xFFFF) / 65535.0f;
    }

    private static float relax(float current, float target, float deltaSeconds, float ratePerSecond) {
        float alpha = MathHelper.clamp(deltaSeconds * ratePerSecond, 0.0f, 1.0f);
        return MathHelper.lerp(alpha, current, target);
    }

    private static float wrap01(float value) {
        float wrapped = value % 1.0f;
        return wrapped < 0.0f ? wrapped + 1.0f : wrapped;
    }

    private static float wrapTau(float value) {
        float wrapped = value % TAU;
        return wrapped < 0.0f ? wrapped + TAU : wrapped;
    }

    private static float wrapDomain(float value) {
        float wrapped = value % PRESSURE_DOMAIN_CELLS;
        return wrapped < 0.0f ? wrapped + PRESSURE_DOMAIN_CELLS : wrapped;
    }

    private static float shortestWrappedDelta(float sample, float center) {
        float delta = wrapDomain(sample) - wrapDomain(center);
        if (delta > PRESSURE_DOMAIN_CELLS * 0.5f) {
            delta -= PRESSURE_DOMAIN_CELLS;
        } else if (delta < -PRESSURE_DOMAIN_CELLS * 0.5f) {
            delta += PRESSURE_DOMAIN_CELLS;
        }
        return delta;
    }

    record Sample(
        float targetWindX,
        float targetWindZ,
        float temperatureBiasKelvin,
        float humidity,
        float stormActivity
    ) {
    }

    record Snapshot(
        float driverTimeSeconds,
        float baseFlowX,
        float baseFlowZ,
        float airmassTemperatureBias,
        float airmassMoistureBias,
        float planetaryWavePhase,
        float stormActivity,
        float seasonPhase,
        List<CycloneCellSnapshot> cycloneCells
    ) {
    }

    record CycloneCellSnapshot(
        float centerCellX,
        float centerCellZ,
        float radiusCells,
        float intensity,
        float pressureSign,
        float driftCellsPerSecondX,
        float driftCellsPerSecondZ,
        float lifecyclePhase,
        float warmCoreBiasKelvin,
        float moistureCoreBias
    ) {
    }

    private record CycloneContribution(
        float windX,
        float windZ,
        float temperatureBiasKelvin,
        float humidityBias
    ) {
        private static final CycloneContribution ZERO = new CycloneContribution(0.0f, 0.0f, 0.0f, 0.0f);
    }

    private static final class CycloneCell {
        private float centerCellX;
        private float centerCellZ;
        private float radiusCells;
        private float intensity;
        private float pressureSign;
        private float driftCellsPerSecondX;
        private float driftCellsPerSecondZ;
        private float lifecyclePhase;
        private float warmCoreBiasKelvin;
        private float moistureCoreBias;

        private CycloneCell(
            float centerCellX,
            float centerCellZ,
            float radiusCells,
            float intensity,
            float pressureSign,
            float driftCellsPerSecondX,
            float driftCellsPerSecondZ,
            float lifecyclePhase,
            float warmCoreBiasKelvin,
            float moistureCoreBias
        ) {
            this.centerCellX = centerCellX;
            this.centerCellZ = centerCellZ;
            this.radiusCells = radiusCells;
            this.intensity = intensity;
            this.pressureSign = pressureSign;
            this.driftCellsPerSecondX = driftCellsPerSecondX;
            this.driftCellsPerSecondZ = driftCellsPerSecondZ;
            this.lifecyclePhase = lifecyclePhase;
            this.warmCoreBiasKelvin = warmCoreBiasKelvin;
            this.moistureCoreBias = moistureCoreBias;
        }

        private CycloneCell copy() {
            return new CycloneCell(
                centerCellX,
                centerCellZ,
                radiusCells,
                intensity,
                pressureSign,
                driftCellsPerSecondX,
                driftCellsPerSecondZ,
                lifecyclePhase,
                warmCoreBiasKelvin,
                moistureCoreBias
            );
        }

        private void advance(float elapsedSeconds, float baseFlowX, float baseFlowZ) {
            lifecyclePhase = wrapTau(lifecyclePhase + elapsedSeconds * CYCLONE_CELL_LIFECYCLE_RADIANS_PER_SECOND);
            float baseDriftX = (baseFlowX / DRIVER_CELL_SIZE_BLOCKS) * CYCLONE_CELL_BASE_FLOW_ADVECTION;
            float baseDriftZ = (baseFlowZ / DRIVER_CELL_SIZE_BLOCKS) * CYCLONE_CELL_BASE_FLOW_ADVECTION;
            centerCellX = wrapDomain(centerCellX + elapsedSeconds * (driftCellsPerSecondX + baseDriftX));
            centerCellZ = wrapDomain(centerCellZ + elapsedSeconds * (driftCellsPerSecondZ + baseDriftZ));
        }

        private CycloneContribution sample(float cellX, float cellZ, float stormActivity) {
            float dx = shortestWrappedDelta(cellX, centerCellX);
            float dz = shortestWrappedDelta(cellZ, centerCellZ);
            float distanceSquared = dx * dx + dz * dz;
            float influenceRadius = radiusCells * 2.0f;
            if (distanceSquared >= influenceRadius * influenceRadius) {
                return CycloneContribution.ZERO;
            }

            float distance = Math.max(1.0e-3f, MathHelper.sqrt(distanceSquared));
            float radiusNorm = distance / Math.max(1.0f, radiusCells);
            float envelope = (float) Math.exp(-radiusNorm * radiusNorm * 1.6f);
            float coreSuppression = MathHelper.clamp(distance / Math.max(1.0f, radiusCells * 0.35f), 0.0f, 1.0f);
            float stormScale = pressureSign < 0.0f
                ? MathHelper.lerp(stormActivity, 0.80f, 1.25f)
                : MathHelper.lerp(stormActivity, 1.05f, 0.90f);
            float lifecycleScale = 0.85f + 0.15f * MathHelper.sin(lifecyclePhase);
            float effectiveIntensity = intensity * stormScale * lifecycleScale;

            float rotationSign = pressureSign < 0.0f ? 1.0f : -1.0f;
            float tangentX = (-dz / distance) * rotationSign;
            float tangentZ = (dx / distance) * rotationSign;
            float radialX = dx / distance;
            float radialZ = dz / distance;
            float swirlSpeed = CYCLONE_CELL_MAX_SWIRL_MPS * effectiveIntensity * envelope * coreSuppression;
            float radialSpeed = CYCLONE_CELL_MAX_RADIAL_MPS * effectiveIntensity * envelope;
            float radialSign = pressureSign < 0.0f ? -1.0f : 1.0f;

            float windX = tangentX * swirlSpeed + radialX * radialSpeed * radialSign;
            float windZ = tangentZ * swirlSpeed + radialZ * radialSpeed * radialSign;
            float temperatureBias = warmCoreBiasKelvin * effectiveIntensity * envelope;
            float humidityBias = moistureCoreBias * effectiveIntensity * envelope;
            return new CycloneContribution(windX, windZ, temperatureBias, humidityBias);
        }

        private CycloneCellSnapshot snapshot() {
            return new CycloneCellSnapshot(
                centerCellX,
                centerCellZ,
                radiusCells,
                intensity,
                pressureSign,
                driftCellsPerSecondX,
                driftCellsPerSecondZ,
                lifecyclePhase,
                warmCoreBiasKelvin,
                moistureCoreBias
            );
        }

        private void set(String field, float value) {
            switch (field) {
                case "center_x" -> centerCellX = wrapDomain(value);
                case "center_z" -> centerCellZ = wrapDomain(value);
                case "radius_cells" -> radiusCells = Math.max(1.0f, value);
                case "intensity" -> intensity = MathHelper.clamp(value, 0.05f, 2.0f);
                case "pressure_sign" -> pressureSign = value < 0.0f ? -1.0f : 1.0f;
                case "drift_x_cells_per_second" -> driftCellsPerSecondX = value;
                case "drift_z_cells_per_second" -> driftCellsPerSecondZ = value;
                case "lifecycle_phase" -> lifecyclePhase = wrapTau(value);
                case "warm_core_bias_kelvin" -> warmCoreBiasKelvin = value;
                case "moisture_core_bias" -> moistureCoreBias = value;
                default -> {
                }
            }
        }
    }
}
