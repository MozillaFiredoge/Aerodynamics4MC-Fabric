package com.aerodynamics4mc.runtime;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import net.minecraft.server.world.ServerWorld;
import net.minecraft.util.math.BlockPos;
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

    private static final int DEFAULT_CYCLONE_CELL_COUNT = 6;
    private static final float PRESSURE_DOMAIN_CELLS = 384.0f;
    private static final float DRIVER_CELL_SIZE_BLOCKS = 256.0f;
    private static final float CYCLONE_CELL_MIN_RADIUS = 14.0f;
    private static final float CYCLONE_CELL_MAX_RADIUS = 30.0f;
    private static final float CYCLONE_CELL_MAX_SWIRL_MPS = 10.0f;
    private static final float CYCLONE_CELL_MAX_RADIAL_MPS = 3.0f;
    private static final float CYCLONE_CELL_BASE_FLOW_ADVECTION = 0.30f;
    private static final float CYCLONE_CELL_LIFECYCLE_RADIANS_PER_SECOND = TAU / (24000.0f * 2.0f / 20.0f);
    private static final float CYCLONE_CELL_OUTER_STEERING_WEIGHT = 0.55f;
    private static final float CYCLONE_CELL_CORE_SWIRL_WEIGHT = 1.40f;
    private static final float CYCLONE_CELL_CORE_RADIAL_WEIGHT = 1.20f;
    private static final float CYCLONE_CELL_CORE_RADIUS_FACTOR = 0.55f;
    private static final int DEFAULT_CONVECTIVE_CLUSTER_COUNT = 6;
    private static final float CONVECTIVE_CLUSTER_MIN_RADIUS = 5.0f;
    private static final float CONVECTIVE_CLUSTER_MAX_RADIUS = 12.0f;
    private static final float CONVECTIVE_CLUSTER_MAX_CONVERGENCE_MPS = 4.0f;
    private static final float CONVECTIVE_CLUSTER_MAX_SWIRL_MPS = 2.2f;
    private static final float CONVECTIVE_CLUSTER_BASE_FLOW_ADVECTION = 0.60f;
    private static final float CONVECTIVE_CLUSTER_LIFECYCLE_RADIANS_PER_SECOND = TAU / (24000.0f / 20.0f);
    private static final float CONVECTIVE_CLUSTER_MIN_EFFECTIVE_ENVELOPE = 0.03f;
    private static final float CONVECTIVE_CLUSTER_MIN_STORM_ACTIVITY = 0.12f;
    private static final int MAX_ACTIVE_TORNADO_VORTICES = 2;
    private static final float TORNADO_MIN_STORM_ACTIVITY = 0.35f;
    private static final int TORNADO_ENVIRONMENT_RADIUS_CELLS = 1;
    private static final float TORNADO_MIN_SUPPORT = 0.75f;
    private static final float TORNADO_MIN_SEPARATION_BLOCKS = 320.0f;
    private static final float TORNADO_MIN_LIFETIME_SECONDS = 45.0f;
    private static final float TORNADO_MAX_LIFETIME_SECONDS = 140.0f;
    private static final float TORNADO_MIN_CORE_RADIUS_BLOCKS = 10.0f;
    private static final float TORNADO_MAX_CORE_RADIUS_BLOCKS = 24.0f;
    private static final float TORNADO_MIN_INFLUENCE_RADIUS_BLOCKS = 48.0f;
    private static final float TORNADO_MAX_INFLUENCE_RADIUS_BLOCKS = 128.0f;
    private static final float TORNADO_MIN_TANGENTIAL_WIND_MPS = 18.0f;
    private static final float TORNADO_MAX_TANGENTIAL_WIND_MPS = 42.0f;
    private static final float TORNADO_MAX_RADIAL_INFLOW_SCALE = 0.35f;
    private static final float TORNADO_MIN_CLUSTER_COOLDOWN_SECONDS = 80.0f;
    private static final float TORNADO_MAX_CLUSTER_COOLDOWN_SECONDS = 180.0f;

    private final long worldSeed;
    private final List<CycloneCell> cycloneCells;
    private final List<ConvectiveCluster> convectiveClusters;
    private final List<TornadoVortex> tornadoVortices;
    private long lastTickUpdated = Long.MIN_VALUE;
    private float driverTimeSeconds;
    private float baseFlowX;
    private float baseFlowZ;
    private float airmassTemperatureBias;
    private float airmassMoistureBias;
    private float planetaryWavePhase;
    private float stormActivity;
    private float seasonPhase;
    private float mesoscaleConvectiveSupport;
    private float mesoscaleLiftSupport;
    private float mesoscaleShearSupport;
    private int nextTornadoId;

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
        List<CycloneCell> cycloneCells,
        List<ConvectiveCluster> convectiveClusters,
        List<TornadoVortex> tornadoVortices
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
        this.convectiveClusters = new ArrayList<>(convectiveClusters);
        this.tornadoVortices = new ArrayList<>(tornadoVortices);
        this.nextTornadoId = computeNextTornadoId(tornadoVortices);
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
        List<CycloneCell> defaultCycloneCells = createDefaultCycloneCells(worldSeed);
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
            defaultCycloneCells,
            createDefaultConvectiveClusters(worldSeed, defaultCycloneCells),
            List.of()
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
                    case "next_tornado_id" -> driver.nextTornadoId = Math.max(1, Math.round(Float.parseFloat(value)));
                    default -> {
                    }
                }
            } catch (NumberFormatException ignored) {
                // Ignore malformed entries and keep deterministic defaults.
            }
        }
        driver.cycloneCells.clear();
        driver.cycloneCells.addAll(parseCycloneCells(worldSeed, lines));
        driver.convectiveClusters.clear();
        driver.convectiveClusters.addAll(parseConvectiveClusters(worldSeed, lines, driver.cycloneCells));
        driver.tornadoVortices.clear();
        driver.tornadoVortices.addAll(parseTornadoVortices(lines));
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
        appendProperty(builder, "next_tornado_id", nextTornadoId);
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
        builder.append("convective_cluster_count=").append(convectiveClusters.size()).append('\n');
        for (int i = 0; i < convectiveClusters.size(); i++) {
            ConvectiveCluster cluster = convectiveClusters.get(i);
            appendProperty(builder, "convective_cluster_" + i + "_center_x", cluster.centerCellX);
            appendProperty(builder, "convective_cluster_" + i + "_center_z", cluster.centerCellZ);
            appendProperty(builder, "convective_cluster_" + i + "_radius_cells", cluster.radiusCells);
            appendProperty(builder, "convective_cluster_" + i + "_intensity", cluster.intensity);
            appendProperty(builder, "convective_cluster_" + i + "_drift_x_cells_per_second", cluster.driftCellsPerSecondX);
            appendProperty(builder, "convective_cluster_" + i + "_drift_z_cells_per_second", cluster.driftCellsPerSecondZ);
            appendProperty(builder, "convective_cluster_" + i + "_lifecycle_phase", cluster.lifecyclePhase);
            appendProperty(builder, "convective_cluster_" + i + "_warm_bias_kelvin", cluster.warmBiasKelvin);
            appendProperty(builder, "convective_cluster_" + i + "_moisture_bias", cluster.moistureBias);
            appendProperty(builder, "convective_cluster_" + i + "_convergence_mps", cluster.convergenceMps);
        }
        builder.append("tornado_vortex_count=").append(tornadoVortices.size()).append('\n');
        for (int i = 0; i < tornadoVortices.size(); i++) {
            TornadoVortex vortex = tornadoVortices.get(i);
            appendProperty(builder, "tornado_vortex_" + i + "_id", vortex.id);
            appendProperty(builder, "tornado_vortex_" + i + "_parent_convective_cluster_id", vortex.parentConvectiveClusterId);
            appendProperty(builder, "tornado_vortex_" + i + "_age_seconds", vortex.ageSeconds);
            appendProperty(builder, "tornado_vortex_" + i + "_lifetime_seconds", vortex.lifetimeSeconds);
            appendProperty(builder, "tornado_vortex_" + i + "_state_ordinal", vortex.stateOrdinal);
            appendProperty(builder, "tornado_vortex_" + i + "_center_block_x", vortex.centerBlockX);
            appendProperty(builder, "tornado_vortex_" + i + "_center_block_z", vortex.centerBlockZ);
            appendProperty(builder, "tornado_vortex_" + i + "_base_y", vortex.baseY);
            appendProperty(builder, "tornado_vortex_" + i + "_translation_x_blocks_per_second", vortex.translationXBlocksPerSecond);
            appendProperty(builder, "tornado_vortex_" + i + "_translation_z_blocks_per_second", vortex.translationZBlocksPerSecond);
            appendProperty(builder, "tornado_vortex_" + i + "_core_radius_blocks", vortex.coreRadiusBlocks);
            appendProperty(builder, "tornado_vortex_" + i + "_influence_radius_blocks", vortex.influenceRadiusBlocks);
            appendProperty(builder, "tornado_vortex_" + i + "_tangential_wind_scale_mps", vortex.tangentialWindScaleMps);
            appendProperty(builder, "tornado_vortex_" + i + "_radial_inflow_scale_mps", vortex.radialInflowScaleMps);
            appendProperty(builder, "tornado_vortex_" + i + "_updraft_scale", vortex.updraftScale);
            appendProperty(builder, "tornado_vortex_" + i + "_condensation_bias", vortex.condensationBias);
            appendProperty(builder, "tornado_vortex_" + i + "_intensity", vortex.intensity);
            appendProperty(builder, "tornado_vortex_" + i + "_rotation_sign", vortex.rotationSign);
        }
        Files.writeString(path, builder.toString(), StandardCharsets.UTF_8);
    }

    synchronized void advance(
        ServerWorld world,
        AeroServerRuntime.WorldEnvironmentSnapshot environmentSnapshot,
        long tickCounter,
        float dtSeconds,
        MesoscaleGrid.DiagnosticsSummary mesoscaleSummary,
        MesoscaleGrid mesoscaleGrid,
        BlockPos mesoscaleFocus
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
        updateMesoscaleSupport(mesoscaleSummary);
        targetStormActivity = MathHelper.clamp(
            targetStormActivity
                + 0.18f * mesoscaleConvectiveSupport
                + 0.10f * mesoscaleLiftSupport
                + 0.05f * mesoscaleShearSupport,
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
        for (int i = 0; i < convectiveClusters.size(); i++) {
            convectiveClusters.get(i).advance(
                elapsedSeconds,
                baseFlowX,
                baseFlowZ,
                stormActivity,
                worldSeed,
                driverTimeSeconds,
                cycloneCells,
                i,
                mesoscaleConvectiveSupport,
                mesoscaleLiftSupport,
                mesoscaleShearSupport
            );
        }
        maybeSpawnTornadoVortices(world, mesoscaleGrid, mesoscaleFocus);
        for (int i = tornadoVortices.size() - 1; i >= 0; i--) {
            TornadoVortex vortex = tornadoVortices.get(i);
            vortex.advance(elapsedSeconds);
            if (vortex.isDissipated()) {
                tornadoVortices.remove(i);
            }
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
        float convectiveHeatingKelvin = 0.0f;
        float convectiveMoistening = 0.0f;
        float convectiveInflowX = 0.0f;
        float convectiveInflowZ = 0.0f;
        float convectiveEnvelope = 0.0f;
        float tornadoWindX = 0.0f;
        float tornadoWindZ = 0.0f;
        float tornadoHeatingKelvin = 0.0f;
        float tornadoMoistening = 0.0f;
        float tornadoUpdraftProxy = 0.0f;

        for (CycloneCell cell : cycloneCells) {
            CycloneContribution contribution = cell.sample(cellX, cellZ, stormActivity);
            targetWindX += contribution.windX();
            targetWindZ += contribution.windZ();
            temperatureBiasKelvin += contribution.temperatureBiasKelvin();
            humidity += contribution.humidityBias();
        }
        for (ConvectiveCluster cluster : convectiveClusters) {
            ConvectiveContribution contribution = cluster.sample(cellX, cellZ, stormActivity);
            targetWindX += contribution.windX();
            targetWindZ += contribution.windZ();
            temperatureBiasKelvin += contribution.temperatureBiasKelvin();
            humidity += contribution.humidityBias();
            convectiveHeatingKelvin += contribution.heatingKelvin();
            convectiveMoistening += contribution.moistening();
            convectiveInflowX += contribution.inflowX();
            convectiveInflowZ += contribution.inflowZ();
            convectiveEnvelope = Math.max(convectiveEnvelope, contribution.envelope());
        }
        for (TornadoVortex vortex : tornadoVortices) {
            TornadoContribution contribution = vortex.sample(cellX, cellZ);
            targetWindX += contribution.windX();
            targetWindZ += contribution.windZ();
            humidity += contribution.moistening();
            tornadoWindX += contribution.windX();
            tornadoWindZ += contribution.windZ();
            tornadoHeatingKelvin += contribution.heatingKelvin();
            tornadoMoistening += contribution.moistening();
            tornadoUpdraftProxy = Math.max(tornadoUpdraftProxy, contribution.updraftProxy());
        }

        targetWindX = MathHelper.clamp(targetWindX, -MAX_DRIVER_WIND_MPS, MAX_DRIVER_WIND_MPS);
        targetWindZ = MathHelper.clamp(targetWindZ, -MAX_DRIVER_WIND_MPS, MAX_DRIVER_WIND_MPS);
        humidity = MathHelper.clamp(humidity, 0.0f, 1.0f);
        return new Sample(
            targetWindX,
            targetWindZ,
            temperatureBiasKelvin,
            humidity,
            stormActivity,
            convectiveHeatingKelvin,
            convectiveMoistening,
            convectiveInflowX,
            convectiveInflowZ,
            convectiveEnvelope,
            tornadoWindX,
            tornadoWindZ,
            tornadoHeatingKelvin,
            tornadoMoistening,
            tornadoUpdraftProxy
        );
    }

    synchronized Snapshot snapshot() {
        List<CycloneCellSnapshot> systems = new ArrayList<>(cycloneCells.size());
        for (CycloneCell cell : cycloneCells) {
            systems.add(cell.snapshot());
        }
        List<ConvectiveClusterSnapshot> convectiveSystems = new ArrayList<>(convectiveClusters.size());
        for (ConvectiveCluster cluster : convectiveClusters) {
            convectiveSystems.add(cluster.snapshot());
        }
        List<TornadoVortexSnapshot> tornadoSystems = new ArrayList<>(tornadoVortices.size());
        for (TornadoVortex vortex : tornadoVortices) {
            tornadoSystems.add(vortex.snapshot());
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
            mesoscaleConvectiveSupport,
            mesoscaleLiftSupport,
            mesoscaleShearSupport,
            List.copyOf(systems),
            List.copyOf(convectiveSystems),
            List.copyOf(tornadoSystems)
        );
    }

    private void updateMesoscaleSupport(MesoscaleGrid.DiagnosticsSummary summary) {
        if (summary == null || summary.sampledStateCount() <= 0) {
            mesoscaleConvectiveSupport = 0.0f;
            mesoscaleLiftSupport = 0.0f;
            mesoscaleShearSupport = 0.0f;
            return;
        }
        float instabilitySupport = Math.min(1.5f, summary.maxInstabilityProxy() / 8.0f);
        float liftSupport = Math.min(1.5f, summary.maxLiftProxy());
        float shearSupport = Math.min(1.5f, summary.meanLowLevelShear() / 6.0f);
        float humiditySupport = MathHelper.clamp(summary.meanHumidity(), 0.0f, 1.0f);
        float convergenceSupport = Math.min(1.5f, summary.maxPositiveMoistureConvergence() * 256.0f * 8.0f);
        mesoscaleLiftSupport = liftSupport;
        mesoscaleShearSupport = shearSupport;
        mesoscaleConvectiveSupport = MathHelper.clamp(
            0.30f * instabilitySupport
                + 0.22f * liftSupport
                + 0.18f * shearSupport
                + 0.18f * humiditySupport
                + 0.12f * convergenceSupport,
            0.0f,
            1.75f
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

    private static List<ConvectiveCluster> createDefaultConvectiveClusters(long worldSeed, List<CycloneCell> cycloneCells) {
        List<CycloneCell> lowPressureCells = new ArrayList<>();
        for (CycloneCell cell : cycloneCells) {
            if (cell.pressureSign < 0.0f) {
                lowPressureCells.add(cell);
            }
        }
        List<ConvectiveCluster> clusters = new ArrayList<>(DEFAULT_CONVECTIVE_CLUSTER_COUNT);
        for (int i = 0; i < DEFAULT_CONVECTIVE_CLUSTER_COUNT; i++) {
            long salt = 0x0d7e1f3a5b79c2e1L + (long) i * 0x9e3779b97f4a7c15L;
            CycloneCell host = lowPressureCells.isEmpty()
                ? cycloneCells.get(i % cycloneCells.size())
                : lowPressureCells.get(i % lowPressureCells.size());
            float offsetAngle = seededUnit(worldSeed, salt ^ 0x6eed0e9da4d94a4fL) * TAU;
            float offsetRadius = MathHelper.lerp(
                seededUnit(worldSeed, salt ^ 0x1f83d9abfb41bd6bL),
                host.radiusCells * 0.15f,
                host.radiusCells * 0.65f
            );
            float centerX = wrapDomain(host.centerCellX + MathHelper.cos(offsetAngle) * offsetRadius);
            float centerZ = wrapDomain(host.centerCellZ + MathHelper.sin(offsetAngle) * offsetRadius);
            float radiusCells = MathHelper.lerp(
                seededUnit(worldSeed, salt ^ 0x5be0cd19137e2179L),
                CONVECTIVE_CLUSTER_MIN_RADIUS,
                CONVECTIVE_CLUSTER_MAX_RADIUS
            );
            float intensity = MathHelper.lerp(
                seededUnit(worldSeed, salt ^ 0xcbbb9d5dc1059ed8L),
                0.45f,
                1.00f
            );
            float driftDirection = seededUnit(worldSeed, salt ^ 0x428a2f98d728ae22L) * TAU;
            float driftSpeedCellsPerSecond = MathHelper.lerp(
                seededUnit(worldSeed, salt ^ 0x7137449123ef65cdL),
                0.0015f,
                0.0065f
            );
            float warmBiasKelvin = MathHelper.lerp(
                seededUnit(worldSeed, salt ^ 0xb5c0fbcfec4d3b2fL),
                1.0f,
                3.8f
            );
            float moistureBias = MathHelper.lerp(
                seededUnit(worldSeed, salt ^ 0xe9b5dba58189dbbcL),
                0.05f,
                0.18f
            );
            float convergenceMps = MathHelper.lerp(
                seededUnit(worldSeed, salt ^ 0x3956c25bf348b538L),
                1.2f,
                CONVECTIVE_CLUSTER_MAX_CONVERGENCE_MPS
            );
            float lifecyclePhase = seededUnit(worldSeed, salt ^ 0x59f111f1b605d019L) * TAU;
            clusters.add(new ConvectiveCluster(
                centerX,
                centerZ,
                radiusCells,
                intensity,
                MathHelper.cos(driftDirection) * driftSpeedCellsPerSecond,
                MathHelper.sin(driftDirection) * driftSpeedCellsPerSecond,
                lifecyclePhase,
                warmBiasKelvin,
                moistureBias,
                convergenceMps
            ));
        }
        return clusters;
    }

    private static CycloneCell preferredCycloneHost(List<CycloneCell> cycloneCells, int clusterIndex, long worldSeed, int cycleOrdinal) {
        List<CycloneCell> lowPressureCells = new ArrayList<>();
        for (CycloneCell cell : cycloneCells) {
            if (cell.pressureSign < 0.0f) {
                lowPressureCells.add(cell);
            }
        }
        List<CycloneCell> pool = lowPressureCells.isEmpty() ? cycloneCells : lowPressureCells;
        if (pool.isEmpty()) {
            return null;
        }
        int baseIndex = Math.floorMod(clusterIndex + cycleOrdinal, pool.size());
        int jitter = Math.round(seededSigned(worldSeed, 0x7f4a7c159e3779b9L + (long) clusterIndex * 1315423911L + cycleOrdinal) * (pool.size() - 1));
        return pool.get(Math.floorMod(baseIndex + jitter, pool.size()));
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
                    count = Math.max(DEFAULT_CYCLONE_CELL_COUNT, Integer.parseInt(line.substring(separator + 1).trim()));
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

    private static List<ConvectiveCluster> parseConvectiveClusters(
        long worldSeed,
        List<String> lines,
        List<CycloneCell> cycloneCells
    ) {
        int count = DEFAULT_CONVECTIVE_CLUSTER_COUNT;
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
            if ("convective_cluster_count".equals(key)) {
                try {
                    count = Math.max(DEFAULT_CONVECTIVE_CLUSTER_COUNT, Integer.parseInt(line.substring(separator + 1).trim()));
                } catch (NumberFormatException ignored) {
                    count = DEFAULT_CONVECTIVE_CLUSTER_COUNT;
                }
                break;
            }
        }

        List<ConvectiveCluster> defaults = createDefaultConvectiveClusters(worldSeed, cycloneCells);
        List<ConvectiveCluster> clusters = new ArrayList<>(count);
        for (int i = 0; i < count; i++) {
            ConvectiveCluster fallback = defaults.get(i % defaults.size());
            clusters.add(fallback.copy());
        }
        if (count == 0) {
            return clusters;
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
            if (!key.startsWith("convective_cluster_")) {
                continue;
            }
            String suffix = key.substring("convective_cluster_".length());
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
            if (index < 0 || index >= clusters.size()) {
                continue;
            }
            String field = suffix.substring(nextSeparator + 1);
            String value = line.substring(separator + 1).trim();
            try {
                clusters.get(index).set(field, Float.parseFloat(value));
            } catch (NumberFormatException ignored) {
                // Ignore malformed entries and keep defaults.
            }
        }
        return clusters;
    }

    private static List<TornadoVortex> parseTornadoVortices(List<String> lines) {
        int count = 0;
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
            if ("tornado_vortex_count".equals(key)) {
                try {
                    count = Math.max(0, Integer.parseInt(line.substring(separator + 1).trim()));
                } catch (NumberFormatException ignored) {
                    count = 0;
                }
                break;
            }
        }

        List<TornadoVortex> vortices = new ArrayList<>(count);
        for (int i = 0; i < count; i++) {
            vortices.add(TornadoVortex.defaultVortex());
        }
        if (count == 0) {
            return vortices;
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
            if (!key.startsWith("tornado_vortex_")) {
                continue;
            }
            String suffix = key.substring("tornado_vortex_".length());
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
            if (index < 0 || index >= vortices.size()) {
                continue;
            }
            String field = suffix.substring(nextSeparator + 1);
            String value = line.substring(separator + 1).trim();
            try {
                vortices.get(index).set(field, Float.parseFloat(value));
            } catch (NumberFormatException ignored) {
                // Ignore malformed entries and keep defaults.
            }
        }
        return vortices;
    }

    private void maybeSpawnTornadoVortices(ServerWorld world, MesoscaleGrid mesoscaleGrid, BlockPos mesoscaleFocus) {
        if (mesoscaleGrid == null || mesoscaleFocus == null) {
            return;
        }
        if (stormActivity < TORNADO_MIN_STORM_ACTIVITY || tornadoVortices.size() >= MAX_ACTIVE_TORNADO_VORTICES) {
            return;
        }
        for (int i = 0; i < convectiveClusters.size(); i++) {
            if (tornadoVortices.size() >= MAX_ACTIVE_TORNADO_VORTICES) {
                return;
            }
            ConvectiveCluster cluster = convectiveClusters.get(i);
            if (!cluster.supportsTornadoGenesis(stormActivity)) {
                continue;
            }
            BlockPos clusterPos = clusterWorldPosition(cluster, mesoscaleFocus, world.getSeaLevel());
            MesoscaleGrid.TornadoEnvironment environment = mesoscaleGrid.sampleTornadoEnvironment(
                clusterPos,
                TORNADO_ENVIRONMENT_RADIUS_CELLS
            );
            if (environment.sampledStateCount() <= 0) {
                continue;
            }
            float tornadoSupport = computeTornadoSupport(cluster, environment);
            if (tornadoSupport < TORNADO_MIN_SUPPORT) {
                continue;
            }
            if (hasNearbyTornado(clusterPos.getX(), clusterPos.getZ(), TORNADO_MIN_SEPARATION_BLOCKS)) {
                continue;
            }
            CycloneCell host = preferredCycloneHost(
                cycloneCells,
                i,
                worldSeed,
                Math.max(0, (int) Math.floor(driverTimeSeconds / (24000.0f / 20.0f)))
            );
            tornadoVortices.add(spawnTornadoVortex(i, cluster, host, clusterPos, environment, tornadoSupport));
            cluster.tornadoCooldownSeconds = MathHelper.lerp(
                MathHelper.clamp(tornadoSupport - TORNADO_MIN_SUPPORT, 0.0f, 1.0f),
                TORNADO_MIN_CLUSTER_COOLDOWN_SECONDS,
                TORNADO_MAX_CLUSTER_COOLDOWN_SECONDS
            );
        }
    }

    private TornadoVortex spawnTornadoVortex(
        int clusterIndex,
        ConvectiveCluster cluster,
        CycloneCell host,
        BlockPos clusterPos,
        MesoscaleGrid.TornadoEnvironment environment,
        float tornadoSupport
    ) {
        long salt = 0x6f31e7d49a52b8c3L + (long) nextTornadoId * 0x9e3779b97f4a7c15L;
        float jitterRadiusBlocks = 24.0f + 48.0f * MathHelper.clamp(tornadoSupport - TORNADO_MIN_SUPPORT, 0.0f, 1.0f);
        float jitterAngle = seededUnit(worldSeed, salt ^ 0x94d049bb133111ebL) * TAU;
        float jitterRadius = seededUnit(worldSeed, salt ^ 0x2545f4914f6cdd1dL) * jitterRadiusBlocks;
        float centerBlockX = clusterPos.getX() + MathHelper.cos(jitterAngle) * jitterRadius;
        float centerBlockZ = clusterPos.getZ() + MathHelper.sin(jitterAngle) * jitterRadius;
        float clusterDriftXBlocksPerSecond = cluster.driftCellsPerSecondX * DRIVER_CELL_SIZE_BLOCKS;
        float clusterDriftZBlocksPerSecond = cluster.driftCellsPerSecondZ * DRIVER_CELL_SIZE_BLOCKS;
        float lifetimeSeconds = MathHelper.lerp(
            MathHelper.clamp((tornadoSupport - TORNADO_MIN_SUPPORT) / 0.7f, 0.0f, 1.0f),
            TORNADO_MIN_LIFETIME_SECONDS,
            TORNADO_MAX_LIFETIME_SECONDS
        );
        float intensity = MathHelper.clamp(
            0.55f
                + 0.18f * environment.maxInstabilityProxy()
                + 0.16f * environment.maxLiftProxy()
                + 0.08f * environment.maxLowLevelShear()
                + 0.20f * (tornadoSupport - TORNADO_MIN_SUPPORT),
            0.4f,
            2.0f
        );
        float coreRadiusBlocks = MathHelper.lerp(
            MathHelper.clamp(tornadoSupport, 0.0f, 1.5f) / 1.5f,
            TORNADO_MIN_CORE_RADIUS_BLOCKS,
            TORNADO_MAX_CORE_RADIUS_BLOCKS
        );
        float influenceRadiusBlocks = Math.max(
            TORNADO_MIN_INFLUENCE_RADIUS_BLOCKS,
            MathHelper.lerp(MathHelper.clamp(tornadoSupport, 0.0f, 1.5f) / 1.5f, TORNADO_MIN_INFLUENCE_RADIUS_BLOCKS, TORNADO_MAX_INFLUENCE_RADIUS_BLOCKS)
        );
        float tangentialWindScaleMps = MathHelper.lerp(
            MathHelper.clamp(tornadoSupport, 0.0f, 1.5f) / 1.5f,
            TORNADO_MIN_TANGENTIAL_WIND_MPS,
            TORNADO_MAX_TANGENTIAL_WIND_MPS
        );
        float radialInflowScaleMps = tangentialWindScaleMps * TORNADO_MAX_RADIAL_INFLOW_SCALE;
        float updraftScale = MathHelper.clamp(
            0.65f + 0.35f * environment.maxLiftProxy() + 0.10f * environment.meanPositiveMoistureConvergence() * 256.0f * 8.0f,
            0.4f,
            2.5f
        );
        float condensationBias = MathHelper.clamp(
            0.45f + 0.40f * environment.meanHumidity() + 0.12f * environment.meanLiftProxy(),
            0.0f,
            1.5f
        );
        float rotationSign = host != null
            ? (host.pressureSign < 0.0f ? 1.0f : -1.0f)
            : (seededSigned(worldSeed, salt ^ 0x1f83d9abfb41bd6bL) < 0.0f ? -1.0f : 1.0f);

        TornadoVortex vortex = TornadoVortex.defaultVortex();
        vortex.id = nextTornadoId++;
        vortex.parentConvectiveClusterId = clusterIndex;
        vortex.ageSeconds = 0.0f;
        vortex.lifetimeSeconds = lifetimeSeconds;
        vortex.stateOrdinal = 0;
        vortex.centerBlockX = centerBlockX;
        vortex.centerBlockZ = centerBlockZ;
        vortex.baseY = clusterPos.getY();
        vortex.translationXBlocksPerSecond = clusterDriftXBlocksPerSecond;
        vortex.translationZBlocksPerSecond = clusterDriftZBlocksPerSecond;
        vortex.coreRadiusBlocks = coreRadiusBlocks;
        vortex.influenceRadiusBlocks = influenceRadiusBlocks;
        vortex.tangentialWindScaleMps = tangentialWindScaleMps;
        vortex.radialInflowScaleMps = radialInflowScaleMps;
        vortex.updraftScale = updraftScale;
        vortex.condensationBias = condensationBias;
        vortex.intensity = intensity;
        vortex.rotationSign = rotationSign;
        return vortex;
    }

    private BlockPos clusterWorldPosition(ConvectiveCluster cluster, BlockPos mesoscaleFocus, int baseY) {
        int focusCellX = Math.floorDiv(mesoscaleFocus.getX(), Math.round(DRIVER_CELL_SIZE_BLOCKS));
        int focusCellZ = Math.floorDiv(mesoscaleFocus.getZ(), Math.round(DRIVER_CELL_SIZE_BLOCKS));
        float focusCenterBlockX = focusCellX * DRIVER_CELL_SIZE_BLOCKS + DRIVER_CELL_SIZE_BLOCKS * 0.5f;
        float focusCenterBlockZ = focusCellZ * DRIVER_CELL_SIZE_BLOCKS + DRIVER_CELL_SIZE_BLOCKS * 0.5f;
        float dxCells = shortestWrappedDelta(cluster.centerCellX, focusCellX);
        float dzCells = shortestWrappedDelta(cluster.centerCellZ, focusCellZ);
        int worldX = MathHelper.floor(focusCenterBlockX + dxCells * DRIVER_CELL_SIZE_BLOCKS);
        int worldZ = MathHelper.floor(focusCenterBlockZ + dzCells * DRIVER_CELL_SIZE_BLOCKS);
        return new BlockPos(worldX, baseY, worldZ);
    }

    private float computeTornadoSupport(ConvectiveCluster cluster, MesoscaleGrid.TornadoEnvironment environment) {
        float instabilityScore = Math.min(1.5f, environment.maxInstabilityProxy() / 4.0f);
        float shearScore = Math.min(1.5f, environment.maxLowLevelShear() / 5.5f);
        float convergenceScore = Math.min(1.5f, environment.maxPositiveMoistureConvergence() * 256.0f * 8.0f);
        float liftScore = Math.min(1.5f, environment.maxLiftProxy());
        float humidityScore = MathHelper.clamp(environment.meanHumidity(), 0.0f, 1.0f);
        float clusterScore = Math.min(1.5f, cluster.intensity / 1.1f);
        return 0.25f * instabilityScore
            + 0.22f * shearScore
            + 0.16f * convergenceScore
            + 0.20f * liftScore
            + 0.07f * humidityScore
            + 0.10f * clusterScore;
    }

    private boolean hasNearbyTornado(float centerBlockX, float centerBlockZ, float minDistanceBlocks) {
        float minDistanceSquared = minDistanceBlocks * minDistanceBlocks;
        for (TornadoVortex vortex : tornadoVortices) {
            float dx = vortex.centerBlockX - centerBlockX;
            float dz = vortex.centerBlockZ - centerBlockZ;
            if (dx * dx + dz * dz < minDistanceSquared) {
                return true;
            }
        }
        return false;
    }

    private static int computeNextTornadoId(List<TornadoVortex> tornadoVortices) {
        int nextId = 1;
        for (TornadoVortex vortex : tornadoVortices) {
            nextId = Math.max(nextId, vortex.id + 1);
        }
        return nextId;
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

    private static float seededSigned(long seed, long salt) {
        return seededUnit(seed, salt) * 2.0f - 1.0f;
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
        float stormActivity,
        float convectiveHeatingKelvin,
        float convectiveMoistening,
        float convectiveInflowX,
        float convectiveInflowZ,
        float convectiveEnvelope,
        float tornadoWindX,
        float tornadoWindZ,
        float tornadoHeatingKelvin,
        float tornadoMoistening,
        float tornadoUpdraftProxy
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
        float mesoscaleConvectiveSupport,
        float mesoscaleLiftSupport,
        float mesoscaleShearSupport,
        List<CycloneCellSnapshot> cycloneCells,
        List<ConvectiveClusterSnapshot> convectiveClusters,
        List<TornadoVortexSnapshot> tornadoVortices
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

    record ConvectiveClusterSnapshot(
        float centerCellX,
        float centerCellZ,
        float radiusCells,
        float intensity,
        float driftCellsPerSecondX,
        float driftCellsPerSecondZ,
        float lifecyclePhase,
        float warmBiasKelvin,
        float moistureBias,
        float convergenceMps
    ) {
    }

    record TornadoVortexSnapshot(
        int id,
        int parentConvectiveClusterId,
        float ageSeconds,
        float lifetimeSeconds,
        int stateOrdinal,
        float centerBlockX,
        float centerBlockZ,
        float baseY,
        float translationXBlocksPerSecond,
        float translationZBlocksPerSecond,
        float coreRadiusBlocks,
        float influenceRadiusBlocks,
        float tangentialWindScaleMps,
        float radialInflowScaleMps,
        float updraftScale,
        float condensationBias,
        float intensity,
        float rotationSign
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

    private record ConvectiveContribution(
        float windX,
        float windZ,
        float temperatureBiasKelvin,
        float humidityBias,
        float heatingKelvin,
        float moistening,
        float inflowX,
        float inflowZ,
        float envelope
    ) {
        private static final ConvectiveContribution ZERO = new ConvectiveContribution(
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

    private record TornadoContribution(
        float windX,
        float windZ,
        float heatingKelvin,
        float moistening,
        float updraftProxy
    ) {
        private static final TornadoContribution ZERO = new TornadoContribution(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
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
            float influenceRadius = radiusCells * 4.0f;
            if (distanceSquared >= influenceRadius * influenceRadius) {
                return CycloneContribution.ZERO;
            }

            float distance = Math.max(1.0e-3f, MathHelper.sqrt(distanceSquared));
            float radiusNorm = distance / Math.max(1.0f, radiusCells);
            float outerNorm = distance / Math.max(1.0f, radiusCells * 2.5f);
            float outerEnvelope = (float) Math.exp(-outerNorm * outerNorm * 0.95f);
            float coreRadius = Math.max(1.0f, radiusCells * CYCLONE_CELL_CORE_RADIUS_FACTOR);
            float coreNorm = distance / coreRadius;
            float coreEnvelope = (float) Math.exp(-coreNorm * coreNorm * 1.8f);
            float coreSuppression = MathHelper.clamp(distance / Math.max(1.0f, coreRadius * 0.25f), 0.0f, 1.0f);
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
            float outerSwirlSpeed = CYCLONE_CELL_MAX_SWIRL_MPS
                * effectiveIntensity
                * CYCLONE_CELL_OUTER_STEERING_WEIGHT
                * outerEnvelope;
            float coreSwirlSpeed = CYCLONE_CELL_MAX_SWIRL_MPS
                * effectiveIntensity
                * CYCLONE_CELL_CORE_SWIRL_WEIGHT
                * coreEnvelope
                * coreSuppression;
            float outerRadialSpeed = CYCLONE_CELL_MAX_RADIAL_MPS
                * effectiveIntensity
                * 0.70f
                * outerEnvelope;
            float coreRadialSpeed = CYCLONE_CELL_MAX_RADIAL_MPS
                * effectiveIntensity
                * CYCLONE_CELL_CORE_RADIAL_WEIGHT
                * coreEnvelope;
            float radialSign = pressureSign < 0.0f ? -1.0f : 1.0f;

            float windX = tangentX * (outerSwirlSpeed + coreSwirlSpeed)
                + radialX * (outerRadialSpeed + coreRadialSpeed) * radialSign;
            float windZ = tangentZ * (outerSwirlSpeed + coreSwirlSpeed)
                + radialZ * (outerRadialSpeed + coreRadialSpeed) * radialSign;
            float temperatureBias = warmCoreBiasKelvin * effectiveIntensity * (0.35f * outerEnvelope + 0.95f * coreEnvelope);
            float humidityBias = moistureCoreBias * effectiveIntensity * (0.35f * outerEnvelope + 0.95f * coreEnvelope);
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

    private static final class ConvectiveCluster {
        private float centerCellX;
        private float centerCellZ;
        private float radiusCells;
        private float intensity;
        private float driftCellsPerSecondX;
        private float driftCellsPerSecondZ;
        private float lifecyclePhase;
        private float warmBiasKelvin;
        private float moistureBias;
        private float convergenceMps;
        private float tornadoCooldownSeconds;

        private ConvectiveCluster(
            float centerCellX,
            float centerCellZ,
            float radiusCells,
            float intensity,
            float driftCellsPerSecondX,
            float driftCellsPerSecondZ,
            float lifecyclePhase,
            float warmBiasKelvin,
            float moistureBias,
            float convergenceMps
        ) {
            this.centerCellX = centerCellX;
            this.centerCellZ = centerCellZ;
            this.radiusCells = radiusCells;
            this.intensity = intensity;
            this.driftCellsPerSecondX = driftCellsPerSecondX;
            this.driftCellsPerSecondZ = driftCellsPerSecondZ;
            this.lifecyclePhase = lifecyclePhase;
            this.warmBiasKelvin = warmBiasKelvin;
            this.moistureBias = moistureBias;
            this.convergenceMps = convergenceMps;
            this.tornadoCooldownSeconds = 0.0f;
        }

        private ConvectiveCluster copy() {
            return new ConvectiveCluster(
                centerCellX,
                centerCellZ,
                radiusCells,
                intensity,
                driftCellsPerSecondX,
                driftCellsPerSecondZ,
                lifecyclePhase,
                warmBiasKelvin,
                moistureBias,
                convergenceMps
            );
        }

        private void advance(float elapsedSeconds, float baseFlowX, float baseFlowZ) {
            advance(elapsedSeconds, baseFlowX, baseFlowZ, 1.0f, 0L, 0.0f, List.of(), 0, 0.0f, 0.0f, 0.0f);
        }

        private void advance(
            float elapsedSeconds,
            float baseFlowX,
            float baseFlowZ,
            float stormActivity,
            long worldSeed,
            float driverTimeSeconds,
            List<CycloneCell> cycloneCells,
            int clusterIndex,
            float mesoscaleConvectiveSupport,
            float mesoscaleLiftSupport,
            float mesoscaleShearSupport
        ) {
            float previousPhase = lifecyclePhase;
            lifecyclePhase = wrapTau(lifecyclePhase + elapsedSeconds * CONVECTIVE_CLUSTER_LIFECYCLE_RADIANS_PER_SECOND);
            if (stormActivity < CONVECTIVE_CLUSTER_MIN_STORM_ACTIVITY) {
                intensity = Math.min(intensity, 0.20f);
                lifecyclePhase = Math.min(lifecyclePhase, TAU * 0.12f);
            } else if (lifecyclePhase < previousPhase) {
                respawn(
                    worldSeed,
                    driverTimeSeconds,
                    cycloneCells,
                    clusterIndex,
                    stormActivity,
                    mesoscaleConvectiveSupport,
                    mesoscaleLiftSupport,
                    mesoscaleShearSupport
                );
            }
            float baseDriftX = (baseFlowX / DRIVER_CELL_SIZE_BLOCKS) * CONVECTIVE_CLUSTER_BASE_FLOW_ADVECTION;
            float baseDriftZ = (baseFlowZ / DRIVER_CELL_SIZE_BLOCKS) * CONVECTIVE_CLUSTER_BASE_FLOW_ADVECTION;
            centerCellX = wrapDomain(centerCellX + elapsedSeconds * (driftCellsPerSecondX + baseDriftX));
            centerCellZ = wrapDomain(centerCellZ + elapsedSeconds * (driftCellsPerSecondZ + baseDriftZ));
            tornadoCooldownSeconds = Math.max(0.0f, tornadoCooldownSeconds - elapsedSeconds);
        }

        private void respawn(
            long worldSeed,
            float driverTimeSeconds,
            List<CycloneCell> cycloneCells,
            int clusterIndex,
            float stormActivity,
            float mesoscaleConvectiveSupport,
            float mesoscaleLiftSupport,
            float mesoscaleShearSupport
        ) {
            int cycleOrdinal = Math.max(0, (int) Math.floor(driverTimeSeconds / (24000.0f / 20.0f)));
            CycloneCell host = preferredCycloneHost(cycloneCells, clusterIndex, worldSeed, cycleOrdinal);
            long salt = 0x4f1bbcdc676f3a29L
                + (long) clusterIndex * 0x9e3779b97f4a7c15L
                + (long) cycleOrdinal * 0x94d049bb133111ebL;
            float hostCenterX = host == null ? seededUnit(worldSeed, salt ^ 0x2e1b21385c26c926L) * PRESSURE_DOMAIN_CELLS : host.centerCellX;
            float hostCenterZ = host == null ? seededUnit(worldSeed, salt ^ 0x27d4eb2f165667c5L) * PRESSURE_DOMAIN_CELLS : host.centerCellZ;
            float hostRadius = host == null ? 18.0f : host.radiusCells;
            float offsetAngle = seededUnit(worldSeed, salt ^ 0x7137449123ef65cdL) * TAU;
            float offsetRadius = MathHelper.lerp(
                seededUnit(worldSeed, salt ^ 0xb5c0fbcfec4d3b2fL),
                hostRadius * 0.10f,
                hostRadius * 0.55f
            );
            centerCellX = wrapDomain(hostCenterX + MathHelper.cos(offsetAngle) * offsetRadius);
            centerCellZ = wrapDomain(hostCenterZ + MathHelper.sin(offsetAngle) * offsetRadius);
            radiusCells = MathHelper.lerp(
                seededUnit(worldSeed, salt ^ 0xe9b5dba58189dbbcL),
                CONVECTIVE_CLUSTER_MIN_RADIUS,
                CONVECTIVE_CLUSTER_MAX_RADIUS
            );
            float supportGain = MathHelper.clamp(
                0.60f + 0.40f * mesoscaleConvectiveSupport + 0.20f * mesoscaleLiftSupport,
                0.40f,
                1.80f
            );
            intensity = MathHelper.lerp(
                MathHelper.clamp(stormActivity, 0.0f, 1.0f),
                0.35f,
                MathHelper.lerp(seededUnit(worldSeed, salt ^ 0x3956c25bf348b538L), 0.85f, 1.20f)
            ) * supportGain;
            float driftDirection = seededUnit(worldSeed, salt ^ 0x59f111f1b605d019L) * TAU;
            float driftSpeedCellsPerSecond = MathHelper.lerp(
                seededUnit(worldSeed, salt ^ 0x923f82a4af194f9bL),
                0.0010f,
                0.0060f
            );
            driftCellsPerSecondX = MathHelper.cos(driftDirection) * driftSpeedCellsPerSecond;
            driftCellsPerSecondZ = MathHelper.sin(driftDirection) * driftSpeedCellsPerSecond;
            warmBiasKelvin = MathHelper.lerp(
                seededUnit(worldSeed, salt ^ 0xab1c5ed5da6d8118L),
                0.8f,
                4.2f
            ) * MathHelper.lerp(stormActivity, 0.75f, 1.15f) * MathHelper.lerp(mesoscaleLiftSupport, 0.85f, 1.25f);
            moistureBias = MathHelper.lerp(
                seededUnit(worldSeed, salt ^ 0xd807aa98a3030242L),
                0.04f,
                0.20f
            ) * MathHelper.lerp(stormActivity, 0.70f, 1.20f) * MathHelper.lerp(mesoscaleConvectiveSupport, 0.80f, 1.30f);
            convergenceMps = MathHelper.lerp(
                seededUnit(worldSeed, salt ^ 0x12835b0145706fbeL),
                1.0f,
                CONVECTIVE_CLUSTER_MAX_CONVERGENCE_MPS
            ) * MathHelper.lerp(stormActivity, 0.70f, 1.25f) * MathHelper.lerp(mesoscaleShearSupport, 0.90f, 1.20f);
            lifecyclePhase = 0.0f;
        }

        private ConvectiveContribution sample(float cellX, float cellZ, float stormActivity) {
            float dx = shortestWrappedDelta(cellX, centerCellX);
            float dz = shortestWrappedDelta(cellZ, centerCellZ);
            float distanceSquared = dx * dx + dz * dz;
            float influenceRadius = radiusCells * 3.0f;
            if (distanceSquared >= influenceRadius * influenceRadius) {
                return ConvectiveContribution.ZERO;
            }

            float distance = Math.max(1.0e-3f, MathHelper.sqrt(distanceSquared));
            float coreNorm = distance / Math.max(1.0f, radiusCells);
            float anvilNorm = distance / Math.max(1.0f, radiusCells * 1.8f);
            float coreEnvelope = (float) Math.exp(-coreNorm * coreNorm * 1.6f);
            float anvilEnvelope = (float) Math.exp(-anvilNorm * anvilNorm * 0.7f);
            float lifeProgress = MathHelper.clamp(lifecyclePhase / TAU, 0.0f, 1.0f);
            float lifecycleEnvelope = MathHelper.sin(lifeProgress * (float) Math.PI);
            if (lifecycleEnvelope <= CONVECTIVE_CLUSTER_MIN_EFFECTIVE_ENVELOPE) {
                return ConvectiveContribution.ZERO;
            }
            float lifecycleScale = (0.35f + 0.65f * lifecycleEnvelope) * (0.80f + 0.20f * MathHelper.sin(lifecyclePhase));
            float effectiveIntensity = intensity * MathHelper.lerp(stormActivity, 0.70f, 1.35f) * lifecycleScale;

            float radialX = -dx / distance;
            float radialZ = -dz / distance;
            float tangentX = -dz / distance;
            float tangentZ = dx / distance;
            float convergence = convergenceMps * effectiveIntensity * (0.55f * coreEnvelope + 0.20f * anvilEnvelope);
            float swirl = CONVECTIVE_CLUSTER_MAX_SWIRL_MPS * effectiveIntensity * 0.25f * coreEnvelope;
            float windX = radialX * convergence + tangentX * swirl;
            float windZ = radialZ * convergence + tangentZ * swirl;
            float temperatureBias = warmBiasKelvin * effectiveIntensity * (0.80f * coreEnvelope + 0.30f * anvilEnvelope);
            float humidityBias = moistureBias * effectiveIntensity * (0.90f * coreEnvelope + 0.55f * anvilEnvelope);
            float envelope = MathHelper.clamp(0.85f * coreEnvelope + 0.25f * anvilEnvelope, 0.0f, 1.0f);
            float heatingKelvin = Math.max(0.0f, warmBiasKelvin) * effectiveIntensity * (0.65f * coreEnvelope + 0.20f * anvilEnvelope);
            float moistening = Math.max(0.0f, moistureBias) * effectiveIntensity * (0.75f * coreEnvelope + 0.35f * anvilEnvelope);
            float inflowX = radialX * convergence;
            float inflowZ = radialZ * convergence;
            return new ConvectiveContribution(
                windX,
                windZ,
                temperatureBias,
                humidityBias,
                heatingKelvin,
                moistening,
                inflowX,
                inflowZ,
                envelope
            );
        }

        private boolean supportsTornadoGenesis(float stormActivity) {
            if (tornadoCooldownSeconds > 0.0f) {
                return false;
            }
            if (stormActivity < TORNADO_MIN_STORM_ACTIVITY) {
                return false;
            }
            float lifecycleProgress = MathHelper.clamp(lifecyclePhase / TAU, 0.0f, 1.0f);
            float lifecycleEnvelope = MathHelper.sin(lifecycleProgress * (float) Math.PI);
            return intensity >= 0.55f
                && lifecycleProgress >= 0.20f
                && lifecycleProgress <= 0.78f
                && lifecycleEnvelope >= 0.55f;
        }

        private ConvectiveClusterSnapshot snapshot() {
            return new ConvectiveClusterSnapshot(
                centerCellX,
                centerCellZ,
                radiusCells,
                intensity,
                driftCellsPerSecondX,
                driftCellsPerSecondZ,
                lifecyclePhase,
                warmBiasKelvin,
                moistureBias,
                convergenceMps
            );
        }

        private void set(String field, float value) {
            switch (field) {
                case "center_x" -> centerCellX = wrapDomain(value);
                case "center_z" -> centerCellZ = wrapDomain(value);
                case "radius_cells" -> radiusCells = Math.max(1.0f, value);
                case "intensity" -> intensity = MathHelper.clamp(value, 0.05f, 2.0f);
                case "drift_x_cells_per_second" -> driftCellsPerSecondX = value;
                case "drift_z_cells_per_second" -> driftCellsPerSecondZ = value;
                case "lifecycle_phase" -> lifecyclePhase = wrapTau(value);
                case "warm_bias_kelvin" -> warmBiasKelvin = value;
                case "moisture_bias" -> moistureBias = MathHelper.clamp(value, -0.40f, 0.40f);
                case "convergence_mps" -> convergenceMps = MathHelper.clamp(value, 0.1f, 8.0f);
                default -> {
                }
            }
        }
    }

    private static final class TornadoVortex {
        private int id;
        private int parentConvectiveClusterId;
        private float ageSeconds;
        private float lifetimeSeconds;
        private int stateOrdinal;
        private float centerBlockX;
        private float centerBlockZ;
        private float baseY;
        private float translationXBlocksPerSecond;
        private float translationZBlocksPerSecond;
        private float coreRadiusBlocks;
        private float influenceRadiusBlocks;
        private float tangentialWindScaleMps;
        private float radialInflowScaleMps;
        private float updraftScale;
        private float condensationBias;
        private float intensity;
        private float rotationSign;

        private static TornadoVortex defaultVortex() {
            return new TornadoVortex();
        }

        private void advance(float elapsedSeconds) {
            ageSeconds = Math.max(0.0f, ageSeconds + elapsedSeconds);
            centerBlockX += translationXBlocksPerSecond * elapsedSeconds;
            centerBlockZ += translationZBlocksPerSecond * elapsedSeconds;
            float lifeRatio = lifetimeSeconds <= 1.0e-3f ? 1.0f : MathHelper.clamp(ageSeconds / lifetimeSeconds, 0.0f, 1.0f);
            if (lifeRatio < 0.18f) {
                stateOrdinal = 0;
            } else if (lifeRatio < 0.75f) {
                stateOrdinal = 1;
            } else if (lifeRatio < 1.0f) {
                stateOrdinal = 2;
            } else {
                stateOrdinal = 3;
            }
        }

        private boolean isDissipated() {
            return stateOrdinal >= 3 && ageSeconds >= lifetimeSeconds;
        }

        private TornadoVortexSnapshot snapshot() {
            return new TornadoVortexSnapshot(
                id,
                parentConvectiveClusterId,
                ageSeconds,
                lifetimeSeconds,
                stateOrdinal,
                centerBlockX,
                centerBlockZ,
                baseY,
                translationXBlocksPerSecond,
                translationZBlocksPerSecond,
                coreRadiusBlocks,
                influenceRadiusBlocks,
                tangentialWindScaleMps,
                radialInflowScaleMps,
                updraftScale,
                condensationBias,
                intensity,
                rotationSign
            );
        }

        private TornadoContribution sample(float cellX, float cellZ) {
            float sampleBlockX = cellX * DRIVER_CELL_SIZE_BLOCKS + DRIVER_CELL_SIZE_BLOCKS * 0.5f;
            float sampleBlockZ = cellZ * DRIVER_CELL_SIZE_BLOCKS + DRIVER_CELL_SIZE_BLOCKS * 0.5f;
            float dx = sampleBlockX - centerBlockX;
            float dz = sampleBlockZ - centerBlockZ;
            float distanceSquared = dx * dx + dz * dz;
            float influenceRadius = Math.max(influenceRadiusBlocks, coreRadiusBlocks);
            if (distanceSquared >= influenceRadius * influenceRadius) {
                return TornadoContribution.ZERO;
            }

            float distance = Math.max(1.0e-3f, MathHelper.sqrt(distanceSquared));
            float outerNorm = distance / Math.max(1.0f, influenceRadiusBlocks);
            float coreNorm = distance / Math.max(1.0f, coreRadiusBlocks);
            float outerEnvelope = (float) Math.exp(-outerNorm * outerNorm * 1.2f);
            float coreEnvelope = (float) Math.exp(-coreNorm * coreNorm * 2.2f);
            float lifecycleProgress = lifetimeSeconds <= 1.0e-3f ? 1.0f : MathHelper.clamp(ageSeconds / lifetimeSeconds, 0.0f, 1.0f);
            float lifecycleEnvelope = lifecycleProgress < 0.18f
                ? MathHelper.clamp(lifecycleProgress / 0.18f, 0.0f, 1.0f)
                : (lifecycleProgress > 0.78f
                    ? MathHelper.clamp((1.0f - lifecycleProgress) / 0.22f, 0.0f, 1.0f)
                    : 1.0f);
            float effectiveIntensity = Math.max(0.0f, intensity) * lifecycleEnvelope;
            if (effectiveIntensity <= 1.0e-3f) {
                return TornadoContribution.ZERO;
            }

            float tangentX = (-dz / distance) * rotationSign;
            float tangentZ = (dx / distance) * rotationSign;
            float radialX = -dx / distance;
            float radialZ = -dz / distance;
            float windX = tangentX * tangentialWindScaleMps * effectiveIntensity * (0.30f * outerEnvelope + 1.10f * coreEnvelope)
                + radialX * radialInflowScaleMps * effectiveIntensity * (0.55f * outerEnvelope + 0.35f * coreEnvelope);
            float windZ = tangentZ * tangentialWindScaleMps * effectiveIntensity * (0.30f * outerEnvelope + 1.10f * coreEnvelope)
                + radialZ * radialInflowScaleMps * effectiveIntensity * (0.55f * outerEnvelope + 0.35f * coreEnvelope);
            float heating = 0.45f * updraftScale * effectiveIntensity * (0.20f * outerEnvelope + 0.65f * coreEnvelope);
            float moistening = 0.08f * condensationBias * effectiveIntensity * (0.25f * outerEnvelope + 0.55f * coreEnvelope);
            float updraft = updraftScale * effectiveIntensity * (0.30f * outerEnvelope + 0.90f * coreEnvelope);
            return new TornadoContribution(windX, windZ, heating, moistening, updraft);
        }

        private void set(String field, float value) {
            switch (field) {
                case "id" -> id = Math.max(0, Math.round(value));
                case "parent_convective_cluster_id" -> parentConvectiveClusterId = Math.max(-1, Math.round(value));
                case "age_seconds" -> ageSeconds = Math.max(0.0f, value);
                case "lifetime_seconds" -> lifetimeSeconds = Math.max(1.0f, value);
                case "state_ordinal" -> stateOrdinal = MathHelper.clamp(Math.round(value), 0, 3);
                case "center_block_x" -> centerBlockX = value;
                case "center_block_z" -> centerBlockZ = value;
                case "base_y" -> baseY = value;
                case "translation_x_blocks_per_second" -> translationXBlocksPerSecond = value;
                case "translation_z_blocks_per_second" -> translationZBlocksPerSecond = value;
                case "core_radius_blocks" -> coreRadiusBlocks = Math.max(1.0f, value);
                case "influence_radius_blocks" -> influenceRadiusBlocks = Math.max(1.0f, value);
                case "tangential_wind_scale_mps" -> tangentialWindScaleMps = Math.max(0.0f, value);
                case "radial_inflow_scale_mps" -> radialInflowScaleMps = Math.max(0.0f, value);
                case "updraft_scale" -> updraftScale = Math.max(0.0f, value);
                case "condensation_bias" -> condensationBias = value;
                case "intensity" -> intensity = Math.max(0.0f, value);
                case "rotation_sign" -> rotationSign = value < 0.0f ? -1.0f : 1.0f;
                default -> {
                }
            }
        }
    }
}
