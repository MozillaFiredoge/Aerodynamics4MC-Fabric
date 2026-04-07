package com.aerodynamics4mc.runtime;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;

import com.aerodynamics4mc.FanBlock;
import com.aerodynamics4mc.ModBlocks;
import com.aerodynamics4mc.net.AeroFlowPayload;
import com.aerodynamics4mc.net.AeroRuntimeStatePayload;
import com.mojang.brigadier.CommandDispatcher;
import net.fabricmc.fabric.api.command.v2.CommandRegistrationCallback;
import net.fabricmc.fabric.api.event.lifecycle.v1.ServerBlockEntityEvents;
import net.fabricmc.fabric.api.event.lifecycle.v1.ServerChunkEvents;
import net.fabricmc.fabric.api.event.lifecycle.v1.ServerLifecycleEvents;
import net.fabricmc.fabric.api.event.lifecycle.v1.ServerTickEvents;
import net.fabricmc.fabric.api.event.lifecycle.v1.ServerWorldEvents;
import net.fabricmc.fabric.api.networking.v1.ServerPlayConnectionEvents;
import net.fabricmc.fabric.api.networking.v1.ServerPlayNetworking;
import net.minecraft.block.BlockState;
import net.minecraft.block.Blocks;
import net.minecraft.block.entity.BlockEntity;
import net.minecraft.command.CommandRegistryAccess;
import net.minecraft.command.CommandSource;
import net.minecraft.registry.RegistryKey;
import net.minecraft.registry.tag.FluidTags;
import net.minecraft.server.MinecraftServer;
import net.minecraft.server.command.CommandManager;
import net.minecraft.server.command.ServerCommandSource;
import net.minecraft.server.network.ServerPlayerEntity;
import net.minecraft.server.world.ServerWorld;
import net.minecraft.state.property.Properties;
import net.minecraft.text.Text;
import net.minecraft.util.Identifier;
import net.minecraft.util.math.BlockPos;
import net.minecraft.util.math.Direction;
import net.minecraft.util.math.MathHelper;
import net.minecraft.world.LightType;
import net.minecraft.world.World;
import net.minecraft.world.chunk.WorldChunk;

public final class AeroServerRuntime {
    private static final String LOG_PREFIX = "[aerodynamics4mc] ";
    private static final int GRID_SIZE = 64;
    private static final int TICKS_PER_SECOND = 20;
    private static final float BLOCK_SIZE_METERS = 1.0f;
    private static final float DOMAIN_SIZE_METERS = GRID_SIZE * BLOCK_SIZE_METERS;
    private static final float SOLVER_STEP_SECONDS = 1.0f / TICKS_PER_SECOND;
    private static final float CELL_SIZE_METERS = BLOCK_SIZE_METERS;
    private static final float CELL_FACE_AREA_SQUARE_METERS = CELL_SIZE_METERS * CELL_SIZE_METERS;
    private static final float CELL_VOLUME_CUBIC_METERS = CELL_FACE_AREA_SQUARE_METERS * CELL_SIZE_METERS;
    private static final float AIR_DENSITY_KG_PER_CUBIC_METER = 1.225f;
    private static final float AIR_SPECIFIC_HEAT_J_PER_KG_K = 1005.0f;
    private static final float RUNTIME_TEMPERATURE_SCALE_KELVIN = 20.0f;
    private static final int CHANNELS = 10;
    private static final int CH_OBSTACLE = 0;
    private static final int CH_FAN_MASK = 1;
    private static final int CH_FAN_VX = 2;
    private static final int CH_FAN_VY = 3;
    private static final int CH_FAN_VZ = 4;
    private static final int CH_STATE_VX = 5;
    private static final int CH_STATE_VY = 6;
    private static final int CH_STATE_VZ = 7;
    private static final int CH_STATE_P = 8;
    private static final int CH_THERMAL_SOURCE = 9;

    private static final float MAX_SAFE_LATTICE_SPEED = 0.28f;
    private static final float NATIVE_VELOCITY_SCALE = CELL_SIZE_METERS / SOLVER_STEP_SECONDS;
    private static final float MAX_RUNTIME_WIND_SPEED = MAX_SAFE_LATTICE_SPEED * NATIVE_VELOCITY_SCALE;
    private static final float DEFAULT_FAN_INFLOW_SPEED = 4.0f;
    private static final float INFLOW_SPEED = Math.min(DEFAULT_FAN_INFLOW_SPEED, MAX_RUNTIME_WIND_SPEED);
    private static final int FAN_RADIUS = 1;
    private static final int DUCT_SCAN_MAX = 20;
    private static final int DUCT_RING_RADIUS = 1;
    private static final float DUCT_RING_FILL_THRESHOLD = 0.80f;
    private static final int DUCT_MAX_CONSECUTIVE_GAPS = 1;
    private static final int DUCT_LEVEL_ONE_MIN = 6;
    private static final int DUCT_LEVEL_TWO_MIN = 12;
    private static final int DUCT_LEVEL_THREE_MIN = 20;
    private static final int DUCT_JET_RANGE = 20;
    private static final float DUCT_EDGE_FACTOR = 0.22f;
    private static final int RESPONSE_CHANNELS = 4;
    private static final int FLOW_COUNT = GRID_SIZE * GRID_SIZE * GRID_SIZE * RESPONSE_CHANNELS;

    private static final int WINDOW_THERMAL_REFRESH_TICKS = 10;
    private static final int PARTICLE_FLOW_SYNC_INTERVAL_TICKS = 4;
    private static final int PARTICLE_FLOW_SAMPLE_STRIDE = 4;
    private static final int MAX_SIMULATION_STEP_BACKLOG = 2;
    private static final int CHUNK_SIZE = 16;
    private static final int WINDOW_SECTION_COUNT = GRID_SIZE / CHUNK_SIZE;
    private static final int WINDOW_SECTION_VOLUME = WINDOW_SECTION_COUNT * WINDOW_SECTION_COUNT * WINDOW_SECTION_COUNT;
    private static final int SECTION_CELL_COUNT = CHUNK_SIZE * CHUNK_SIZE * CHUNK_SIZE;
    private static final int SOLVER_WORKER_COUNT = Math.max(2, Runtime.getRuntime().availableProcessors() / 2);
    private static final float NATIVE_THERMAL_SOURCE_MAX = 0.006f;
    private static final float STATE_PRESSURE_MIN = -0.03f;
    private static final float STATE_PRESSURE_MAX = 0.03f;
    private static final float THERMAL_SURFACE_INIT_MIN_K = 220.0f;
    private static final float THERMAL_SURFACE_MAX_K = 1800.0f;
    private static final float THERMAL_BASE_AMBIENT_AIR_TEMPERATURE_K = 288.15f;
    private static final float THERMAL_BIOME_TEMPERATURE_SCALE_K = 12.0f;
    private static final float THERMAL_ALTITUDE_LAPSE_RATE_K_PER_BLOCK = 0.0065f;
    private static final float THERMAL_DEEP_GROUND_OFFSET_K = 1.5f;
    private static final float THERMAL_SKY_TEMP_DROP_DAY_K = 10.0f;
    private static final float THERMAL_SKY_TEMP_DROP_NIGHT_K = 24.0f;
    private static final float THERMAL_PRECIP_TEMP_DROP_K = 4.0f;
    private static final float THERMAL_STEFAN_BOLTZMANN = 5.6703744e-8f;
    private static final float THERMAL_SOLAR_DIRECT_FLUX_W_M2 = 850.0f;
    private static final float THERMAL_SOLAR_DIFFUSE_FLUX_W_M2 = 140.0f;
    private static final float THERMAL_EMITTER_POWER_LAVA_W = 3200.0f;
    private static final float THERMAL_EMITTER_POWER_MAGMA_W = 1200.0f;
    private static final float THERMAL_EMITTER_POWER_CAMPFIRE_W = 1800.0f;
    private static final float THERMAL_EMITTER_POWER_SOUL_CAMPFIRE_W = 1200.0f;
    private static final float THERMAL_EMITTER_POWER_FIRE_W = 2200.0f;
    private static final float THERMAL_EMITTER_POWER_SOUL_FIRE_W = 1500.0f;
    private static final float THERMAL_EMITTER_POWER_TORCH_W = 80.0f;
    private static final float THERMAL_EMITTER_POWER_SOUL_TORCH_W = 50.0f;
    private static final float THERMAL_EMITTER_POWER_LANTERN_W = 60.0f;
    private static final float THERMAL_EMITTER_POWER_SOUL_LANTERN_W = 40.0f;
    private static final byte SURFACE_KIND_NONE = 0;
    private static final byte SURFACE_KIND_ROCK = 1;
    private static final byte SURFACE_KIND_SOIL = 2;
    private static final byte SURFACE_KIND_VEGETATION = 3;
    private static final byte SURFACE_KIND_SNOW_ICE = 4;
    private static final byte SURFACE_KIND_WATER = 5;
    private static final byte SURFACE_KIND_MOLTEN = 6;
    private static final Direction[] CARDINAL_DIRECTIONS = Direction.values();
    private static final int FACE_COUNT = 6;
    private static final int BACKGROUND_MET_CELL_SIZE_BLOCKS = 256;
    private static final int BACKGROUND_MET_RADIUS_CELLS = 20;
    private static final int MESOSCALE_MET_CELL_SIZE_BLOCKS = 64;
    private static final int MESOSCALE_MET_RADIUS_CELLS = 16;
    private static final int MESOSCALE_MET_LAYER_HEIGHT_BLOCKS = 40;
    private static final int MESOSCALE_MET_MAX_LAYERS = Math.max(1, 320 / MESOSCALE_MET_LAYER_HEIGHT_BLOCKS);
    private static final int MESOSCALE_FORCING_REBUILD_TICKS = TICKS_PER_SECOND * 60;
    private static final float MESOSCALE_STEP_SECONDS = MESOSCALE_MET_CELL_SIZE_BLOCKS * SOLVER_STEP_SECONDS;
    private static final int BACKGROUND_MET_REFRESH_TICKS = TICKS_PER_SECOND * 60 * 3;
    private static final int STATIC_MIRROR_BUILD_BUDGET_PER_TICK = 8;
    private static final int WINDOW_EDGE_STABILIZATION_LAYERS = 8;
    private static final float WINDOW_EDGE_STABILIZATION_MIN_KEEP = 0.15f;
    private static final int REGION_HALO_CELLS = CHUNK_SIZE;
    private static final int REGION_CORE_SIZE = GRID_SIZE - REGION_HALO_CELLS * 2;
    private static final int REGION_LATTICE_STRIDE = REGION_CORE_SIZE;
    private static final AeroServerRuntime INSTANCE = new AeroServerRuntime();

    private final Map<WindowKey, RegionRecord> regions = new HashMap<>();
    private final NativeSimulationBridge simulationBridge = new NativeSimulationBridge();
    private final SeedTerrainProvider seedTerrainProvider = new WorldgenSeedTerrainProvider();
    private final NestedBoundaryCoupler nestedBoundaryCoupler = new NestedBoundaryCoupler();
    private final WorldMirror worldMirror = new WorldMirror();
    private final DynamicStore dynamicStore = new DynamicStore();
    private final Map<RegistryKey<World>, BackgroundMetGrid> backgroundMetGrids = new HashMap<>();
    private final Map<RegistryKey<World>, MesoscaleGrid> mesoscaleMetGrids = new HashMap<>();
    private final Object simulationStateLock = new Object();
    private final ExecutorService solverExecutor = Executors.newFixedThreadPool(SOLVER_WORKER_COUNT, runnable -> {
        Thread thread = new Thread(runnable, "aero-server-solver");
        thread.setDaemon(true);
        return thread;
    });
    private final AtomicInteger activeSolveTasks = new AtomicInteger(0);
    private final AtomicInteger simulationStepBudget = new AtomicInteger(0);
    private final AtomicLong runtimeGeneration = new AtomicLong(0L);
    private final AtomicLong publishedFrameCounter = new AtomicLong(0L);
    private final AtomicReference<PublishedFrame> publishedFrame = new AtomicReference<>(null);
    private volatile Set<WindowKey> desiredWindowKeys = Set.of();
    private volatile Map<RegistryKey<World>, WorldEnvironmentSnapshot> worldEnvironmentSnapshots = Map.of();

    private boolean streamingEnabled = false;
    private int tickCounter = 0;
    private int lastWindowRefreshTick = Integer.MIN_VALUE;
    private long simulationTicks = 0L;
    private long lastObservedPublishedFrameId = 0L;
    private int secondWindowTotalTicks = 0;
    private int secondWindowSimulationTicks = 0;
    private float simulationTicksPerSecond = 0.0f;
    private float lastMaxFlowSpeed = 0.0f;
    private volatile String lastSolverError = "";
    private long simulationServiceId = 0L;
    private volatile MinecraftServer currentServer;
    private int lastSimulationFocusX = Integer.MIN_VALUE;
    private int lastSimulationFocusY = Integer.MIN_VALUE;
    private int lastSimulationFocusZ = Integer.MIN_VALUE;
    private SimulationCoordinator simulationCoordinator;

    private AeroServerRuntime() {
    }

    public static void init() {
        ServerTickEvents.END_SERVER_TICK.register(INSTANCE::onServerTick);
        ServerChunkEvents.CHUNK_LOAD.register(INSTANCE::onChunkLoad);
        ServerChunkEvents.CHUNK_UNLOAD.register(INSTANCE::onChunkUnload);
        ServerBlockEntityEvents.BLOCK_ENTITY_LOAD.register(INSTANCE::onBlockEntityLoad);
        ServerBlockEntityEvents.BLOCK_ENTITY_UNLOAD.register(INSTANCE::onBlockEntityUnload);
        ServerWorldEvents.UNLOAD.register((server, world) -> INSTANCE.onWorldUnload(world));
        ServerPlayConnectionEvents.JOIN.register((handler, sender, server) -> {
            INSTANCE.sendStateToPlayer(handler.player, server);
            INSTANCE.broadcastState(server);
            INSTANCE.sendFlowSnapshotToPlayer(handler.player, server);
        });
        ServerPlayConnectionEvents.DISCONNECT.register((handler, server) -> INSTANCE.broadcastState(server));
        ServerLifecycleEvents.SERVER_STOPPED.register(INSTANCE::shutdownAll);
        CommandRegistrationCallback.EVENT.register(INSTANCE::registerCommands);
    }

    public static void notifyBlockStateChanged(ServerWorld world, BlockPos pos, BlockState oldState, BlockState newState) {
        INSTANCE.onBlockChanged(world, pos, oldState, newState);
    }

    private void onChunkLoad(ServerWorld world, WorldChunk chunk) {
        synchronized (simulationStateLock) {
            worldMirror.onChunkLoad(world, chunk);
            submitWorldDeltaToSimulation(new NativeSimulationBridge.WorldDelta(
                NativeSimulationBridge.WORLD_DELTA_CHUNK_LOADED,
                chunk.getPos().getStartX(),
                world.getBottomY(),
                chunk.getPos().getStartZ(),
                world.getRegistryKey().getValue().hashCode(),
                0,
                0,
                0,
                0.0f,
                0.0f,
                0.0f,
                0.0f
            ));
        }
    }

    private void onChunkUnload(ServerWorld world, WorldChunk chunk) {
        synchronized (simulationStateLock) {
            worldMirror.onChunkUnload(world, chunk.getPos());
            submitWorldDeltaToSimulation(new NativeSimulationBridge.WorldDelta(
                NativeSimulationBridge.WORLD_DELTA_CHUNK_UNLOADED,
                chunk.getPos().getStartX(),
                world.getBottomY(),
                chunk.getPos().getStartZ(),
                world.getRegistryKey().getValue().hashCode(),
                0,
                0,
                0,
                0.0f,
                0.0f,
                0.0f,
                0.0f
            ));
        }
    }

    private void onBlockEntityLoad(BlockEntity blockEntity, ServerWorld world) {
        synchronized (simulationStateLock) {
            worldMirror.onBlockEntityLoad(blockEntity, world);
            BlockPos pos = blockEntity.getPos();
            submitWorldDeltaToSimulation(new NativeSimulationBridge.WorldDelta(
                NativeSimulationBridge.WORLD_DELTA_BLOCK_ENTITY_LOADED,
                pos.getX(),
                pos.getY(),
                pos.getZ(),
                world.getRegistryKey().getValue().hashCode(),
                blockEntity.getType().toString().hashCode(),
                0,
                0,
                0.0f,
                0.0f,
                0.0f,
                0.0f
            ));
        }
    }

    private void onBlockEntityUnload(BlockEntity blockEntity, ServerWorld world) {
        synchronized (simulationStateLock) {
            worldMirror.onBlockEntityUnload(blockEntity, world);
            BlockPos pos = blockEntity.getPos();
            submitWorldDeltaToSimulation(new NativeSimulationBridge.WorldDelta(
                NativeSimulationBridge.WORLD_DELTA_BLOCK_ENTITY_UNLOADED,
                pos.getX(),
                pos.getY(),
                pos.getZ(),
                world.getRegistryKey().getValue().hashCode(),
                blockEntity.getType().toString().hashCode(),
                0,
                0,
                0.0f,
                0.0f,
                0.0f,
                0.0f
            ));
        }
    }

    private void onWorldUnload(ServerWorld world) {
        synchronized (simulationStateLock) {
            worldMirror.onWorldUnload(world);
            backgroundMetGrids.remove(world.getRegistryKey());
            MesoscaleGrid grid = mesoscaleMetGrids.remove(world.getRegistryKey());
            if (grid != null) {
                grid.close();
            }
            submitWorldDeltaToSimulation(new NativeSimulationBridge.WorldDelta(
                NativeSimulationBridge.WORLD_DELTA_WORLD_UNLOADED,
                0,
                0,
                0,
                world.getRegistryKey().getValue().hashCode(),
                0,
                0,
                0,
                0.0f,
                0.0f,
                0.0f,
                0.0f
            ));
        }
    }

    private void onBlockChanged(ServerWorld world, BlockPos pos, BlockState oldState, BlockState newState) {
        synchronized (simulationStateLock) {
            worldMirror.onBlockChanged(world, pos, oldState, newState);
            invalidateDynamicRegionsForBlock(world, pos);
            submitWorldDeltaToSimulation(new NativeSimulationBridge.WorldDelta(
                NativeSimulationBridge.WORLD_DELTA_BLOCK_CHANGED,
                pos.getX(),
                pos.getY(),
                pos.getZ(),
                world.getRegistryKey().getValue().hashCode(),
                oldState.hashCode(),
                newState.hashCode(),
                0,
                0.0f,
                0.0f,
                0.0f,
                0.0f
            ));
        }
    }

    private void invalidateDynamicRegionsForBlock(ServerWorld world, BlockPos pos) {
        for (WindowKey key : overlappingWindowKeysForBlock(world.getRegistryKey(), pos)) {
            dynamicStore.invalidateRegion(world, key.worldKey(), key.origin());
        }
    }

    private List<WindowKey> overlappingWindowKeysForBlock(RegistryKey<World> worldKey, BlockPos pos) {
        int[] xs = candidateCoreCoordinates(pos.getX());
        int[] ys = candidateCoreCoordinates(pos.getY());
        int[] zs = candidateCoreCoordinates(pos.getZ());
        List<WindowKey> keys = new ArrayList<>(8);
        Set<WindowKey> dedupe = new HashSet<>();
        for (int coreX : xs) {
            for (int coreY : ys) {
                for (int coreZ : zs) {
                    BlockPos coreOrigin = new BlockPos(coreX, coreY, coreZ);
                    BlockPos windowOrigin = windowOriginFromCoreOrigin(coreOrigin);
                    if (!containsBlock(windowOrigin, pos)) {
                        continue;
                    }
                    WindowKey key = new WindowKey(worldKey, windowOrigin);
                    if (dedupe.add(key)) {
                        keys.add(key);
                    }
                }
            }
        }
        return keys;
    }

    private int[] candidateCoreCoordinates(int blockCoord) {
        int stride = REGION_LATTICE_STRIDE;
        int upperAligned = Math.floorDiv(blockCoord + REGION_HALO_CELLS, stride) * stride;
        return new int[] { upperAligned - stride, upperAligned };
    }

    private boolean containsBlock(BlockPos windowOrigin, BlockPos pos) {
        return pos.getX() >= windowOrigin.getX() && pos.getX() < windowOrigin.getX() + GRID_SIZE
            && pos.getY() >= windowOrigin.getY() && pos.getY() < windowOrigin.getY() + GRID_SIZE
            && pos.getZ() >= windowOrigin.getZ() && pos.getZ() < windowOrigin.getZ() + GRID_SIZE;
    }

    private void registerCommands(
        CommandDispatcher<ServerCommandSource> dispatcher,
        CommandRegistryAccess registryAccess,
        CommandManager.RegistrationEnvironment environment
    ) {
        dispatcher.register(CommandManager.literal("aero")
            .requires(CommandManager.requirePermissionLevel(CommandManager.ADMINS_CHECK))
            .then(CommandManager.literal("start")
                .executes(ctx -> {
                    streamingEnabled = true;
                    lastSolverError = "";
                    ensureSimulationCoordinatorRunning();
                    feedback(ctx.getSource(), "Streaming enabled");
                    broadcastState(ctx.getSource().getServer());
                    return 1;
                }))
            .then(CommandManager.literal("status")
                .executes(ctx -> {
                    PublishedFrame currentFrame = publishedFrame.get();
                    feedback(
                        ctx.getSource(),
                        "Status streaming=" + streamingEnabled
                            + " box=" + format2(DOMAIN_SIZE_METERS) + "m"
                            + " n=" + GRID_SIZE
                            + " dx=" + format4(CELL_SIZE_METERS) + "m"
                            + " dt=" + format3(SOLVER_STEP_SECONDS) + "s"
                            + " particleStride=" + PARTICLE_FLOW_SAMPLE_STRIDE
                            + " windows=" + attachedWindowCount()
                            + " simTicks=" + simulationTicks
                            + " simTickPerSec=" + format2(simulationTicksPerSecond)
                            + " maxFlow=" + format2(lastMaxFlowSpeed)
                            + " publishedRegions=" + (currentFrame == null ? 0 : currentFrame.regionAtlases().size())
                            + " l0Cells=" + backgroundMetCellCount()
                            + " l1Cells=" + mesoscaleMetCellCount()
                            + " simBridge=" + simulationBridge.runtimeInfo()
                    );
                    if (!lastSolverError.isEmpty()) {
                        feedback(ctx.getSource(), "Last solver error: " + lastSolverError);
                    }
                    return 1;
                }))
            .then(CommandManager.literal("stop")
                .executes(ctx -> {
                    stopStreaming(ctx.getSource().getServer());
                    feedback(ctx.getSource(), "Streaming disabled");
                    broadcastState(ctx.getSource().getServer());
                    return 1;
                }))
        );
    }

    private void onServerTick(MinecraftServer server) {
        currentServer = server;
        if (!streamingEnabled) {
            return;
        }
        if (server.isPaused()) {
            return;
        }
        tickCounter++;
        synchronized (simulationStateLock) {
            ensureSimulationServiceInitialized();
            updateSimulationFocus(server);
        }
        if (tickCounter == 1 || tickCounter % BACKGROUND_MET_REFRESH_TICKS == 0) {
            synchronized (simulationStateLock) {
                refreshBackgroundMetGrids(server);
            }
        }
        prepareDesiredActiveWindows(server);
        worldMirror.drainLiveBuilds(server, STATIC_MIRROR_BUILD_BUDGET_PER_TICK, this::populateMirrorSectionSnapshot);
        ensureSimulationCoordinatorRunning();

        PublishedFrame frame = publishedFrame.get();
        if (frame == null || frame.regionAtlases().isEmpty()) {
            updateSimulationRate(0);
            lastMaxFlowSpeed = 0.0f;
            return;
        }

        boolean shouldSyncFlow = tickCounter % PARTICLE_FLOW_SYNC_INTERVAL_TICKS == 0;
        if (shouldSyncFlow) {
            syncPublishedFlowToPlayers(server, frame, PARTICLE_FLOW_SAMPLE_STRIDE);
        }
        long frameId = frame.frameId();
        int publishedSteps = 0;
        if (frameId > lastObservedPublishedFrameId) {
            long delta = frameId - lastObservedPublishedFrameId;
            simulationTicks += delta;
            publishedSteps = (int) Math.min(Integer.MAX_VALUE, delta);
            lastObservedPublishedFrameId = frameId;
            lastMaxFlowSpeed = frame.maxSpeed();
        }
        updateSimulationRate(publishedSteps);
    }

    private void stopStreaming(MinecraftServer server) {
        runtimeGeneration.incrementAndGet();
        streamingEnabled = false;
        stopSimulationCoordinator();
        tickCounter = 0;
        simulationStepBudget.set(0);
        simulationTicks = 0L;
        lastObservedPublishedFrameId = 0L;
        secondWindowTotalTicks = 0;
        secondWindowSimulationTicks = 0;
        simulationTicksPerSecond = 0.0f;
        lastMaxFlowSpeed = 0.0f;
        lastSimulationFocusX = Integer.MIN_VALUE;
        lastSimulationFocusY = Integer.MIN_VALUE;
        lastSimulationFocusZ = Integer.MIN_VALUE;
        synchronized (simulationStateLock) {
            for (Map.Entry<WindowKey, RegionRecord> entry : regions.entrySet()) {
                WindowState window = detachRegionWindow(entry.getKey(), entry.getValue());
                if (window != null) {
                    deactivateWindow(entry.getKey(), window);
                } else {
                    deactivateWindowRegionInSimulation(entry.getKey());
                }
            }
            regions.clear();
        }
        backgroundMetGrids.clear();
        for (MesoscaleGrid grid : mesoscaleMetGrids.values()) {
            grid.close();
        }
        mesoscaleMetGrids.clear();
        desiredWindowKeys = Set.of();
        worldEnvironmentSnapshots = Map.of();
        publishedFrame.set(null);
        waitForSolverIdle();
        releaseSimulationService();
    }

    private void refreshBackgroundMetGrids(MinecraftServer server) {
        for (ServerWorld world : server.getWorlds()) {
            List<ServerPlayerEntity> players = world.getPlayers();
            if (players.isEmpty()) {
                backgroundMetGrids.remove(world.getRegistryKey());
                MesoscaleGrid grid = mesoscaleMetGrids.remove(world.getRegistryKey());
                if (grid != null) {
                    grid.close();
                }
                continue;
            }
            double sumX = 0.0;
            double sumZ = 0.0;
            for (ServerPlayerEntity player : players) {
                sumX += player.getX();
                sumZ += player.getZ();
            }
            int focusX = MathHelper.floor(sumX / players.size());
            int focusZ = MathHelper.floor(sumZ / players.size());
            BlockPos focus = new BlockPos(focusX, world.getSeaLevel(), focusZ);
            BackgroundMetGrid grid = backgroundMetGrids.computeIfAbsent(
                world.getRegistryKey(),
                ignored -> new BackgroundMetGrid(BACKGROUND_MET_CELL_SIZE_BLOCKS, BACKGROUND_MET_RADIUS_CELLS)
            );
            grid.refresh(world, focus, tickCounter, SOLVER_STEP_SECONDS, seedTerrainProvider);
            MesoscaleGrid mesoscale = mesoscaleMetGrids.computeIfAbsent(
                world.getRegistryKey(),
                ignored -> new MesoscaleGrid(
                    MESOSCALE_MET_CELL_SIZE_BLOCKS,
                    MESOSCALE_MET_RADIUS_CELLS,
                    MESOSCALE_MET_LAYER_HEIGHT_BLOCKS,
                    MESOSCALE_MET_MAX_LAYERS,
                    MESOSCALE_STEP_SECONDS,
                    MESOSCALE_FORCING_REBUILD_TICKS
                )
            );
            mesoscale.refresh(world, focus, tickCounter, SOLVER_STEP_SECONDS, seedTerrainProvider, grid);
        }
    }

    private void ensureSimulationServiceInitialized() {
        if (simulationServiceId != 0L || !simulationBridge.isLoaded()) {
            return;
        }
        simulationServiceId = simulationBridge.createService();
        if (simulationServiceId == 0L) {
            String error = simulationBridge.lastError();
            if (error != null && !error.isBlank() && !"not_loaded".equals(error)) {
                lastSolverError = error;
            }
        }
    }

    private void releaseSimulationService() {
        if (simulationServiceId == 0L) {
            return;
        }
        simulationBridge.releaseService(simulationServiceId);
        simulationServiceId = 0L;
    }

    private void updateSimulationFocus(MinecraftServer server) {
        if (simulationServiceId == 0L) {
            return;
        }
        List<ServerPlayerEntity> players = server.getPlayerManager().getPlayerList();
        if (players.isEmpty()) {
            return;
        }
        double sumX = 0.0;
        double sumY = 0.0;
        double sumZ = 0.0;
        for (ServerPlayerEntity player : players) {
            sumX += player.getX();
            sumY += player.getY();
            sumZ += player.getZ();
        }
        int focusX = MathHelper.floor(sumX / players.size());
        int focusY = MathHelper.floor(sumY / players.size());
        int focusZ = MathHelper.floor(sumZ / players.size());
        simulationBridge.setFocus(simulationServiceId, focusX, focusY, focusZ, GRID_SIZE);
        if (focusX != lastSimulationFocusX || focusY != lastSimulationFocusY || focusZ != lastSimulationFocusZ) {
            lastSimulationFocusX = focusX;
            lastSimulationFocusY = focusY;
            lastSimulationFocusZ = focusZ;
            simulationBridge.submitWorldDelta(
                simulationServiceId,
                new NativeSimulationBridge.WorldDelta(
                    NativeSimulationBridge.WORLD_DELTA_FOCUS_CHANGED,
                    focusX,
                    focusY,
                    focusZ,
                    players.size(),
                    0,
                    0,
                    0,
                    0.0f,
                    0.0f,
                    0.0f,
                    0.0f
                )
            );
        }
    }

    private void submitWorldDeltaToSimulation(NativeSimulationBridge.WorldDelta delta) {
        if (simulationServiceId == 0L || delta == null) {
            return;
        }
        simulationBridge.submitWorldDelta(simulationServiceId, delta);
    }

    private BackgroundMetGrid.Sample sampleBackgroundMet(RegistryKey<World> worldKey, BlockPos pos) {
        BackgroundMetGrid grid = backgroundMetGrids.get(worldKey);
        return grid == null ? null : grid.sample(pos);
    }

    private MesoscaleGrid.Sample sampleMesoscaleMet(RegistryKey<World> worldKey, BlockPos pos) {
        MesoscaleGrid grid = mesoscaleMetGrids.get(worldKey);
        return grid == null ? null : grid.sample(pos);
    }

    private int backgroundMetCellCount() {
        int total = 0;
        for (BackgroundMetGrid grid : backgroundMetGrids.values()) {
            total += grid.cellCount();
        }
        return total;
    }

    private int mesoscaleMetCellCount() {
        int total = 0;
        for (MesoscaleGrid grid : mesoscaleMetGrids.values()) {
            total += grid.cellCount();
        }
        return total;
    }

    private List<MesoscaleGrid> snapshotMesoscaleGrids() {
        synchronized (simulationStateLock) {
            return new ArrayList<>(mesoscaleMetGrids.values());
        }
    }

    private void runMesoscaleStepCycle() {
        List<MesoscaleGrid> grids = snapshotMesoscaleGrids();
        for (MesoscaleGrid grid : grids) {
            grid.runPendingSteps();
        }
    }

    private void shutdownAll(MinecraftServer server) {
        stopStreaming(server);
        synchronized (simulationStateLock) {
            worldMirror.close();
        }
        dynamicStore.close();
        currentServer = null;
    }

    private void prepareDesiredActiveWindows(MinecraftServer server) {
        Set<WindowKey> desiredWindows = activeRegionKeys(server);
        Map<RegistryKey<World>, WorldEnvironmentSnapshot> environmentSnapshots = captureWorldEnvironmentSnapshots(server);
        desiredWindowKeys = Set.copyOf(desiredWindows);
        worldEnvironmentSnapshots = Map.copyOf(environmentSnapshots);
    }

    private Map<RegistryKey<World>, WorldEnvironmentSnapshot> captureWorldEnvironmentSnapshots(MinecraftServer server) {
        Map<RegistryKey<World>, WorldEnvironmentSnapshot> snapshots = new HashMap<>();
        for (ServerWorld world : server.getWorlds()) {
            if (world.getPlayers().isEmpty()) {
                continue;
            }
            snapshots.put(
                world.getRegistryKey(),
                new WorldEnvironmentSnapshot(
                    world.getTimeOfDay(),
                    world.getRainGradient(1.0f),
                    world.getThunderGradient(1.0f),
                    world.getSeaLevel()
                )
            );
        }
        return snapshots;
    }

    private void synchronizeActiveWindowsFromMirror() {
        synchronizeDesiredRegions();

        for (Map.Entry<WindowKey, RegionRecord> regionEntry : regions.entrySet()) {
            WindowKey key = regionEntry.getKey();
            RegionRecord region = regionEntry.getValue();
            if (!synchronizeRegionRecordFromMirror(key, region)) {
                continue;
            }
            WindowState window = attachOrRefreshRegionWindow(key, region);
            if (window == null) {
                continue;
            }
            region.fans = queryFanSources(key.worldKey(), key.origin());
            if (!window.busy.get() && shouldRefreshWindowThermal(region, window)) {
                refreshWindowThermalFields(key, region, window);
            }
        }
    }

    private void synchronizeDesiredRegions() {
        Set<WindowKey> desiredWindows = desiredWindowKeys;
        MinecraftServer server = currentServer;
        Iterator<Map.Entry<WindowKey, RegionRecord>> iterator = regions.entrySet().iterator();
        while (iterator.hasNext()) {
            Map.Entry<WindowKey, RegionRecord> entry = iterator.next();
            if (desiredWindows.contains(entry.getKey())) {
                continue;
            }
            WindowState window = detachRegionWindow(entry.getKey(), entry.getValue());
            if (window != null) {
                deactivateWindow(entry.getKey(), window);
            } else {
                deactivateWindowRegionInSimulation(entry.getKey());
            }
            iterator.remove();
        }
        for (WindowKey key : desiredWindows) {
            if (regions.containsKey(key)) {
                continue;
            }
            regions.put(key, new RegionRecord());
            if (server != null) {
                requestWindowSections(server, key);
            }
        }
    }

    private void requestWindowSections(MinecraftServer server, WindowKey key) {
        for (int sx = 0; sx < WINDOW_SECTION_COUNT; sx++) {
            for (int sy = 0; sy < WINDOW_SECTION_COUNT; sy++) {
                for (int sz = 0; sz < WINDOW_SECTION_COUNT; sz++) {
                    BlockPos localOrigin = sectionOrigin(key.origin(), sx, sy, sz);
                    worldMirror.requestSectionBuild(server, key.worldKey(), localOrigin);
                }
            }
        }
    }

    private boolean synchronizeRegionRecordFromMirror(WindowKey key, RegionRecord region) {
        if (!uploadRegionStaticFromMirror(key, region)) {
            return false;
        }
        if (!region.serviceActive) {
            activateWindowRegionInSimulation(key);
        }
        refreshRegionLifecycle(key, region);
        return region.serviceReady;
    }

    private WindowState attachOrRefreshRegionWindow(WindowKey key, RegionRecord region) {
        WindowState window = region.attachedWindow();
        if (window == null) {
            window = new WindowState();
            if (!initializeWindowFromMirror(key, region, window)) {
                return null;
            }
            if (!region.dynamicRestoreAttempted) {
                tryRestoreWindowDynamicRegionFromSimulation(key, region, window);
                region.dynamicRestoreAttempted = true;
                refreshRegionLifecycle(key, region);
                if (!region.serviceReady) {
                    return null;
                }
            }
            attachRegionWindow(region, window);
            return window;
        }
        refreshWindowFromMirror(key, region, window);
        return window;
    }

    private List<FanSource> queryFanSources(RegistryKey<World> worldKey, BlockPos origin) {
        List<WorldMirror.FanRecord> fans = worldMirror.queryFans(
            worldKey,
            origin,
            GRID_SIZE,
            DUCT_JET_RANGE + FAN_RADIUS + 1
        );
        if (fans.isEmpty()) {
            return List.of();
        }
        List<FanSource> result = new ArrayList<>(fans.size());
        for (WorldMirror.FanRecord fan : fans) {
            result.add(new FanSource(fan.pos(), fan.facing(), fan.ductLength()));
        }
        return List.copyOf(result);
    }

    private boolean initializeWindowFromMirror(WindowKey key, RegionRecord region, WindowState window) {
        region.ensureSectionsInitialized();
        window.ensureDynamicStateInitialized();
        for (int sx = 0; sx < WINDOW_SECTION_COUNT; sx++) {
            for (int sy = 0; sy < WINDOW_SECTION_COUNT; sy++) {
                for (int sz = 0; sz < WINDOW_SECTION_COUNT; sz++) {
                    BlockPos localOrigin = sectionOrigin(key.origin(), sx, sy, sz);
                    WorldMirror.SectionSnapshot snapshot = worldMirror.peekSection(key.worldKey(), localOrigin);
                    if (snapshot == null) {
                        return false;
                    }
                    WindowSection section = new WindowSection();
                    applyMirrorSectionSnapshot(snapshot, section);
                    region.setSection(sx, sy, sz, section);
                }
            }
        }
        clearWindowStateChannels(window);
        for (int sx = 0; sx < WINDOW_SECTION_COUNT; sx++) {
            for (int sy = 0; sy < WINDOW_SECTION_COUNT; sy++) {
                for (int sz = 0; sz < WINDOW_SECTION_COUNT; sz++) {
                    WindowSection section = region.sectionAt(sx, sy, sz);
                    if (section == null) {
                        continue;
                    }
                    recomputeSectionThermalState(key.worldKey(), key.origin(), window, sx, sy, sz, section, SOLVER_STEP_SECONDS);
                }
            }
        }
        region.lastThermalRefreshTick = tickCounter;
        window.markTemperatureRestorePending();
        window.markSeamSyncPending();
        return true;
    }

    private void refreshWindowFromMirror(WindowKey key, RegionRecord region, WindowState window) {
        if (region.sections == null) {
            if (!initializeWindowFromMirror(key, region, window)) {
                return;
            }
            return;
        }
        window.ensureDynamicStateInitialized();
        boolean sectionUpdated = false;
        for (int sx = 0; sx < WINDOW_SECTION_COUNT; sx++) {
            for (int sy = 0; sy < WINDOW_SECTION_COUNT; sy++) {
                for (int sz = 0; sz < WINDOW_SECTION_COUNT; sz++) {
                    BlockPos localOrigin = sectionOrigin(key.origin(), sx, sy, sz);
                    WorldMirror.SectionSnapshot snapshot = worldMirror.peekSection(key.worldKey(), localOrigin);
                    if (snapshot == null) {
                        continue;
                    }
                    WindowSection section = region.sectionAt(sx, sy, sz);
                    if (section == null) {
                        section = new WindowSection();
                        region.setSection(sx, sy, sz, section);
                        sectionUpdated = true;
                    }
                    if (section.mirrorVersion == snapshot.version()) {
                        continue;
                    }
                    sectionUpdated = true;
                    boolean sectionGeometryChanged = section.mirrorVersion < 0 || geometryChanged(section, snapshot);
                    applyMirrorSectionSnapshot(snapshot, section);
                    if (sectionGeometryChanged) {
                        clearWindowSectionDynamicState(window, sx, sy, sz);
                    }
                    recomputeSectionThermalState(key.worldKey(), key.origin(), window, sx, sy, sz, section, SOLVER_STEP_SECONDS);
                    window.markTemperatureRestorePending();
                    window.markSeamSyncPending();
                }
            }
        }
        if (sectionUpdated) {
            region.lastThermalRefreshTick = tickCounter;
        }
    }

    private boolean geometryChanged(WindowSection section, WorldMirror.SectionSnapshot snapshot) {
        for (int i = 0; i < SECTION_CELL_COUNT; i++) {
            if (section.obstacle()[i] != snapshot.obstacle()[i]
                || section.air()[i] != snapshot.air()[i]
                || section.staticSurfaceKind()[i] != snapshot.surfaceKind()[i]) {
                return true;
            }
        }
        return false;
    }

    private void applyMirrorSectionSnapshot(WorldMirror.SectionSnapshot snapshot, WindowSection section) {
        section.attachSnapshot(snapshot);
    }

    private void populateMirrorSectionSnapshot(ServerWorld world, BlockPos origin, WorldMirror.SectionSnapshot snapshot) {
        Arrays.fill(snapshot.obstacle(), 0.0f);
        Arrays.fill(snapshot.air(), 0.0f);
        Arrays.fill(snapshot.surfaceKind(), SURFACE_KIND_NONE);
        Arrays.fill(snapshot.emitterPowerWatts(), 0.0f);
        Arrays.fill(snapshot.openFaceMask(), (byte) 0);
        Arrays.fill(snapshot.faceSkyExposure(), (byte) 0);
        Arrays.fill(snapshot.faceDirectExposure(), (byte) 0);
        BlockPos.Mutable cursor = new BlockPos.Mutable();
        for (int x = 0; x < CHUNK_SIZE; x++) {
            for (int y = 0; y < CHUNK_SIZE; y++) {
                for (int z = 0; z < CHUNK_SIZE; z++) {
                    int cell = localSectionCellIndex(x, y, z);
                    cursor.set(origin.getX() + x, origin.getY() + y, origin.getZ() + z);
                    BlockState state = world.getBlockState(cursor);
                    boolean solid = isSolidObstacle(world, cursor, state);
                    snapshot.obstacle()[cell] = solid ? 1.0f : 0.0f;
                    snapshot.air()[cell] = state.isAir() ? 1.0f : 0.0f;
                    ThermalMaterial material = thermalMaterial(state);
                    snapshot.surfaceKind()[cell] = material == null ? SURFACE_KIND_NONE : material.kind();
                    sectionStaticThermalFields(
                        world,
                        cursor,
                        state,
                        x,
                        y,
                        z,
                        material,
                        snapshot.emitterPowerWatts(),
                        snapshot.openFaceMask(),
                        snapshot.faceSkyExposure(),
                        snapshot.faceDirectExposure()
                    );
                }
            }
        }
    }

    private void recomputeSectionThermalState(
        RegistryKey<World> worldKey,
        BlockPos windowOrigin,
        WindowState window,
        int sx,
        int sy,
        int sz,
        WindowSection section,
        float deltaSeconds
    ) {
        float[] thermalScratch = new float[SECTION_CELL_COUNT];
        float[] sectionTemperatureScratch = new float[SECTION_CELL_COUNT];
        float[] surfaceTemperatureScratch = new float[SECTION_CELL_COUNT];
        byte[] surfaceKindScratch = new byte[SECTION_CELL_COUNT];
        copyWindowScalarSectionToBuffer(window.airTemperatureState, sx, sy, sz, sectionTemperatureScratch);
        copyWindowScalarSectionToBuffer(window.surfaceTemperatureState, sx, sy, sz, surfaceTemperatureScratch);
        sampleSectionThermalFields(
            worldKey,
            sectionOrigin(windowOrigin, sx, sy, sz),
            section,
            sectionTemperatureScratch,
            thermalScratch,
            surfaceTemperatureScratch,
            surfaceKindScratch,
            deltaSeconds
        );
        System.arraycopy(thermalScratch, 0, section.thermal, 0, SECTION_CELL_COUNT);
        System.arraycopy(surfaceKindScratch, 0, section.thermalSurfaceKind, 0, SECTION_CELL_COUNT);
        copyBufferToWindowScalarSection(surfaceTemperatureScratch, sx, sy, sz, window.surfaceTemperatureState);
    }

    private void synchronizeRegionSeams(List<ActiveWindow> activeWindows) {
        if (activeWindows.isEmpty()) {
            return;
        }
        Map<WindowKey, ActiveWindow> activeByKey = new HashMap<>(activeWindows.size());
        Set<WindowState> syncedWindows = new HashSet<>();
        for (ActiveWindow active : activeWindows) {
            activeByKey.put(active.key(), active);
        }
        for (ActiveWindow active : activeWindows) {
            WindowKey key = active.key();
            WindowState window = active.window();
            if (active.region().sections == null || window.busy.get() || !window.seamSyncPending()) {
                continue;
            }
            synchronizePositiveNeighbor(activeByKey, active, 1, 0, 0, syncedWindows);
            synchronizePositiveNeighbor(activeByKey, active, 0, 1, 0, syncedWindows);
            synchronizePositiveNeighbor(activeByKey, active, 0, 0, 1, syncedWindows);
        }
        for (WindowState window : syncedWindows) {
            window.clearSeamSyncPending();
        }
    }

    private void synchronizePositiveNeighbor(
        Map<WindowKey, ActiveWindow> activeByKey,
        ActiveWindow active,
        int dx,
        int dy,
        int dz,
        Set<WindowState> syncedWindows
    ) {
        WindowKey key = active.key();
        WindowState window = active.window();
        BlockPos neighborOrigin = key.origin().add(
            dx * REGION_LATTICE_STRIDE,
            dy * REGION_LATTICE_STRIDE,
            dz * REGION_LATTICE_STRIDE
        );
        WindowKey neighborKey = new WindowKey(key.worldKey(), neighborOrigin);
        ActiveWindow neighborActive = activeByKey.get(neighborKey);
        WindowState neighbor = neighborActive == null ? null : neighborActive.window();
        if (neighbor == null
            || neighborActive.region().sections == null
            || neighbor.busy.get()
            || window.backendResetPending()
            || neighbor.backendResetPending()
            || window.temperatureRestorePending()
            || neighbor.temperatureRestorePending()) {
            return;
        }
        if (synchronizeNeighborHalo(key, window, neighborKey, neighbor, dx, dy, dz)) {
            syncedWindows.add(window);
            syncedWindows.add(neighbor);
        }
    }

    private boolean synchronizeNeighborHalo(
        WindowKey firstKey,
        WindowState first,
        WindowKey secondKey,
        WindowState second,
        int offsetX,
        int offsetY,
        int offsetZ
    ) {
        boolean nativeHaloSynced = false;
        if (ensureSimulationL2Runtime()) {
            long firstRegionKey = simulationRegionKey(firstKey);
            long secondRegionKey = simulationRegionKey(secondKey);
            boolean firstContextReady = simulationBridge.hasRegionContext(simulationServiceId, firstRegionKey);
            boolean secondContextReady = simulationBridge.hasRegionContext(simulationServiceId, secondRegionKey);
            if (firstContextReady && secondContextReady) {
                nativeHaloSynced = simulationBridge.exchangeRegionHalo(
                    simulationServiceId,
                    firstRegionKey,
                    secondRegionKey,
                    GRID_SIZE,
                    offsetX,
                    offsetY,
                    offsetZ
                );
                if (!nativeHaloSynced) {
                    String nativeError = simulationBridge.lastError();
                    String runtimeInfo = simulationBridge.runtimeInfo();
                    StringBuilder message = new StringBuilder("Native halo exchange failed for offset ")
                        .append(offsetX).append(",").append(offsetY).append(",").append(offsetZ);
                    if (nativeError != null && !nativeError.isBlank()
                        && !"not_initialized".equals(nativeError)
                        && !"not_loaded".equals(nativeError)) {
                        message.append(": ").append(nativeError);
                    }
                    if (runtimeInfo != null && !runtimeInfo.isBlank()
                        && !"not_initialized".equals(runtimeInfo)
                        && !"not_loaded".equals(runtimeInfo)) {
                        message.append(" [runtime=").append(runtimeInfo).append("]");
                    }
                    lastSolverError = message.toString();
                }
            }
        }
        if (!nativeHaloSynced) {
            copyNeighborHaloPrism(first, second, offsetX, offsetY, offsetZ);
            copyNeighborHaloPrism(second, first, -offsetX, -offsetY, -offsetZ);
            first.markTemperatureRestorePending();
            second.markTemperatureRestorePending();
        }
        return true;
    }

    private void copyNeighborHaloPrism(WindowState srcWindow, WindowState dstWindow, int offsetX, int offsetY, int offsetZ) {
        int[] srcStart = new int[3];
        int[] dstStart = new int[3];
        int[] size = new int[3];
        if (!fillNeighborHaloBounds(offsetX, offsetY, offsetZ, srcStart, dstStart, size)) {
            return;
        }
        copySlab(
            srcWindow,
            dstWindow,
            srcStart[0],
            srcStart[1],
            srcStart[2],
            dstStart[0],
            dstStart[1],
            dstStart[2],
            size[0],
            size[1],
            size[2]
        );
    }

    private boolean fillNeighborHaloBounds(
        int offsetX,
        int offsetY,
        int offsetZ,
        int[] srcStart,
        int[] dstStart,
        int[] size
    ) {
        if (srcStart.length < 3 || dstStart.length < 3 || size.length < 3) {
            return false;
        }
        return fillNeighborAxisBounds(offsetX, srcStart, dstStart, size, 0)
            && fillNeighborAxisBounds(offsetY, srcStart, dstStart, size, 1)
            && fillNeighborAxisBounds(offsetZ, srcStart, dstStart, size, 2);
    }

    private boolean fillNeighborAxisBounds(int offset, int[] srcStart, int[] dstStart, int[] size, int axis) {
        switch (offset) {
            case -1 -> {
                srcStart[axis] = REGION_HALO_CELLS;
                dstStart[axis] = GRID_SIZE - REGION_HALO_CELLS;
                size[axis] = REGION_HALO_CELLS;
                return true;
            }
            case 0 -> {
                srcStart[axis] = REGION_HALO_CELLS;
                dstStart[axis] = REGION_HALO_CELLS;
                size[axis] = REGION_CORE_SIZE;
                return true;
            }
            case 1 -> {
                srcStart[axis] = REGION_CORE_SIZE;
                dstStart[axis] = 0;
                size[axis] = REGION_HALO_CELLS;
                return true;
            }
            default -> {
                return false;
            }
        }
    }

    private void copySlab(
        WindowState srcWindow,
        WindowState dstWindow,
        int srcX0,
        int srcY0,
        int srcZ0,
        int dstX0,
        int dstY0,
        int dstZ0,
        int sizeX,
        int sizeY,
        int sizeZ
    ) {
        srcWindow.ensureDynamicStateInitialized();
        dstWindow.ensureDynamicStateInitialized();
        for (int x = 0; x < sizeX; x++) {
            int srcX = srcX0 + x;
            int dstX = dstX0 + x;
            for (int y = 0; y < sizeY; y++) {
                    int srcY = srcY0 + y;
                    int dstY = dstY0 + y;
                    for (int z = 0; z < sizeZ; z++) {
                        int srcZ = srcZ0 + z;
                        int dstZ = dstZ0 + z;
                    int srcCell = gridCellIndex(srcX, srcY, srcZ);
                    int dstCell = gridCellIndex(dstX, dstY, dstZ);
                    int srcState = srcCell * RESPONSE_CHANNELS;
                    int dstState = dstCell * RESPONSE_CHANNELS;
                    dstWindow.flowState[dstState] = srcWindow.flowState[srcState];
                    dstWindow.flowState[dstState + 1] = srcWindow.flowState[srcState + 1];
                    dstWindow.flowState[dstState + 2] = srcWindow.flowState[srcState + 2];
                    dstWindow.flowState[dstState + 3] = srcWindow.flowState[srcState + 3];
                    dstWindow.airTemperatureState[dstCell] = srcWindow.airTemperatureState[srcCell];
                    dstWindow.surfaceTemperatureState[dstCell] = srcWindow.surfaceTemperatureState[srcCell];
                }
            }
        }
    }

    private void clearWindowStateChannels(WindowState window) {
        window.ensureDynamicStateInitialized();
        Arrays.fill(window.flowState, 0.0f);
        Arrays.fill(window.airTemperatureState, 0.0f);
        Arrays.fill(window.surfaceTemperatureState, 0.0f);
        window.clearCompletedSolveResult();
        window.markTemperatureRestorePending();
    }

    private boolean shouldRefreshWindowThermal(RegionRecord region, WindowState window) {
        return region.sections != null
            && tickCounter - region.lastThermalRefreshTick >= WINDOW_THERMAL_REFRESH_TICKS;
    }

    private boolean ensureSimulationL2Runtime() {
        return simulationServiceId != 0L
            && simulationBridge.isLoaded()
            && simulationBridge.ensureL2Runtime(simulationServiceId, GRID_SIZE, GRID_SIZE, GRID_SIZE, CHANNELS, RESPONSE_CHANNELS);
    }

    private boolean restoreTemperatureStateToNative(WindowKey key, WindowState window) {
        if (!window.temperatureRestorePending()) {
            return false;
        }
        if (!ensureSimulationL2Runtime()) {
            return false;
        }
        float[] buffer = window.ensureTemperatureTransferBuffer();
        window.ensureDynamicStateInitialized();
        System.arraycopy(window.airTemperatureState, 0, buffer, 0, buffer.length);
        boolean restored = simulationBridge.setRegionTemperatureState(
            simulationServiceId,
            simulationRegionKey(key),
            GRID_SIZE,
            GRID_SIZE,
            GRID_SIZE,
            buffer
        );
        if (restored) {
            window.clearTemperatureRestorePending();
        }
        return restored;
    }

    private SolveResult consumeCompletedSolveResult(WindowState window, BlockPos expectedOrigin) {
        SolveResult completed = window.consumeCompletedSolveResult();
        if (completed == null) {
            return null;
        }
        if (!completed.origin().equals(expectedOrigin)) {
            return null;
        }
        return completed;
    }

    private void deactivateWindow(WindowKey key, WindowState window) {
        window.markDetached();
        exportWindowDynamicRegionToSimulation(key, window);
        deactivateWindowRegionInSimulation(key);
        if (!window.busy.get()) {
            releaseWindow(key, window);
        }
    }

    private void resetWindowBackend(WindowKey key, WindowState window) {
        if (simulationServiceId != 0L) {
            simulationBridge.releaseRegionRuntime(simulationServiceId, simulationRegionKey(key));
        }
        window.clearBackendResetPending();
    }

    private Set<WindowKey> activeRegionKeys(MinecraftServer server) {
        Set<WindowKey> keys = new HashSet<>();
        for (ServerWorld world : server.getWorlds()) {
            RegistryKey<World> worldKey = world.getRegistryKey();
            for (ServerPlayerEntity player : world.getPlayers()) {
                BlockPos baseCore = coreOriginForPosition(player.getBlockPos());
                for (int dx = -1; dx <= 1; dx++) {
                    for (int dz = -1; dz <= 1; dz++) {
                        BlockPos regionCore = baseCore.add(dx * REGION_LATTICE_STRIDE, 0, dz * REGION_LATTICE_STRIDE);
                        keys.add(new WindowKey(worldKey, windowOriginFromCoreOrigin(regionCore)));
                    }
                }
                keys.add(new WindowKey(
                    worldKey,
                    windowOriginFromCoreOrigin(baseCore.add(0, REGION_LATTICE_STRIDE, 0))
                ));
                keys.add(new WindowKey(
                    worldKey,
                    windowOriginFromCoreOrigin(baseCore.add(0, -REGION_LATTICE_STRIDE, 0))
                ));
            }
        }
        return keys;
    }

    private BlockPos coreOriginForPosition(BlockPos pos) {
        int x = Math.floorDiv(pos.getX(), REGION_LATTICE_STRIDE) * REGION_LATTICE_STRIDE;
        int y = Math.floorDiv(pos.getY(), REGION_LATTICE_STRIDE) * REGION_LATTICE_STRIDE;
        int z = Math.floorDiv(pos.getZ(), REGION_LATTICE_STRIDE) * REGION_LATTICE_STRIDE;
        return new BlockPos(x, y, z);
    }

    private BlockPos windowOriginFromCoreOrigin(BlockPos coreOrigin) {
        return coreOrigin.add(-REGION_HALO_CELLS, -REGION_HALO_CELLS, -REGION_HALO_CELLS);
    }

    private int voxelIndex(int x, int y, int z) {
        return ((x * GRID_SIZE + y) * GRID_SIZE + z) * CHANNELS;
    }

    private int localSectionCellIndex(int x, int y, int z) {
        return ((x * CHUNK_SIZE + y) * CHUNK_SIZE + z);
    }

    private int windowSectionIndex(int sx, int sy, int sz) {
        return ((sx * WINDOW_SECTION_COUNT + sy) * WINDOW_SECTION_COUNT + sz);
    }

    private BlockPos sectionOrigin(BlockPos windowOrigin, int sx, int sy, int sz) {
        return new BlockPos(
            windowOrigin.getX() + sx * CHUNK_SIZE,
            windowOrigin.getY() + sy * CHUNK_SIZE,
            windowOrigin.getZ() + sz * CHUNK_SIZE
        );
    }

    private int gridCellIndex(int x, int y, int z) {
        return (x * GRID_SIZE + y) * GRID_SIZE + z;
    }

    private void clearWindowSectionDynamicState(WindowState window, int sx, int sy, int sz) {
        if (window.flowState == null || window.airTemperatureState == null || window.surfaceTemperatureState == null) {
            return;
        }
        int baseX = sx * CHUNK_SIZE;
        int baseY = sy * CHUNK_SIZE;
        int baseZ = sz * CHUNK_SIZE;
        for (int lx = 0; lx < CHUNK_SIZE; lx++) {
            int x = baseX + lx;
            for (int ly = 0; ly < CHUNK_SIZE; ly++) {
                int y = baseY + ly;
                for (int lz = 0; lz < CHUNK_SIZE; lz++) {
                    int z = baseZ + lz;
                    int cell = gridCellIndex(x, y, z);
                    int flowIndex = cell * RESPONSE_CHANNELS;
                    window.flowState[flowIndex] = 0.0f;
                    window.flowState[flowIndex + 1] = 0.0f;
                    window.flowState[flowIndex + 2] = 0.0f;
                    window.flowState[flowIndex + 3] = 0.0f;
                    window.airTemperatureState[cell] = 0.0f;
                    window.surfaceTemperatureState[cell] = 0.0f;
                }
            }
        }
    }

    private void copyWindowScalarSectionToBuffer(float[] source, int sx, int sy, int sz, float[] destination) {
        int baseX = sx * CHUNK_SIZE;
        int baseY = sy * CHUNK_SIZE;
        int baseZ = sz * CHUNK_SIZE;
        for (int lx = 0; lx < CHUNK_SIZE; lx++) {
            int x = baseX + lx;
            for (int ly = 0; ly < CHUNK_SIZE; ly++) {
                int y = baseY + ly;
                for (int lz = 0; lz < CHUNK_SIZE; lz++) {
                    int z = baseZ + lz;
                    destination[localSectionCellIndex(lx, ly, lz)] = source[gridCellIndex(x, y, z)];
                }
            }
        }
    }

    private void copyBufferToWindowScalarSection(float[] source, int sx, int sy, int sz, float[] destination) {
        int baseX = sx * CHUNK_SIZE;
        int baseY = sy * CHUNK_SIZE;
        int baseZ = sz * CHUNK_SIZE;
        for (int lx = 0; lx < CHUNK_SIZE; lx++) {
            int x = baseX + lx;
            for (int ly = 0; ly < CHUNK_SIZE; ly++) {
                int y = baseY + ly;
                for (int lz = 0; lz < CHUNK_SIZE; lz++) {
                    int z = baseZ + lz;
                    destination[gridCellIndex(x, y, z)] = source[localSectionCellIndex(lx, ly, lz)];
                }
            }
        }
    }

    private long simulationRegionKey(WindowKey key) {
        long value = 1469598103934665603L;
        value = (value ^ key.worldKey().getValue().hashCode()) * 1099511628211L;
        value = (value ^ key.origin().getX()) * 1099511628211L;
        value = (value ^ key.origin().getY()) * 1099511628211L;
        value = (value ^ key.origin().getZ()) * 1099511628211L;
        return value == 0L ? 1L : value;
    }

    private boolean uploadRegionStaticFromMirror(WindowKey key, RegionRecord region) {
        if (simulationServiceId == 0L) {
            return false;
        }
        WorldMirror.SectionSnapshot[] snapshots = new WorldMirror.SectionSnapshot[WINDOW_SECTION_VOLUME];
        long staticSignature = 1469598103934665603L;
        for (int sx = 0; sx < WINDOW_SECTION_COUNT; sx++) {
            for (int sy = 0; sy < WINDOW_SECTION_COUNT; sy++) {
                for (int sz = 0; sz < WINDOW_SECTION_COUNT; sz++) {
                    BlockPos localOrigin = sectionOrigin(key.origin(), sx, sy, sz);
                    WorldMirror.SectionSnapshot snapshot = worldMirror.peekSection(key.worldKey(), localOrigin);
                    if (snapshot == null) {
                        return false;
                    }
                    snapshots[windowSectionIndex(sx, sy, sz)] = snapshot;
                    staticSignature = (staticSignature ^ snapshot.version()) * 1099511628211L;
                }
            }
        }
        if (region.staticUploaded && region.staticSignature == staticSignature) {
            return true;
        }
        int cells = GRID_SIZE * GRID_SIZE * GRID_SIZE;
        byte[] obstacle = new byte[cells];
        byte[] surfaceKind = new byte[cells];
        short[] openFaceMask = new short[cells];
        float[] emitterPower = new float[cells];
        for (int sx = 0; sx < WINDOW_SECTION_COUNT; sx++) {
            for (int sy = 0; sy < WINDOW_SECTION_COUNT; sy++) {
                for (int sz = 0; sz < WINDOW_SECTION_COUNT; sz++) {
                    WorldMirror.SectionSnapshot snapshot = snapshots[windowSectionIndex(sx, sy, sz)];
                    int baseX = sx * CHUNK_SIZE;
                    int baseY = sy * CHUNK_SIZE;
                    int baseZ = sz * CHUNK_SIZE;
                    for (int lx = 0; lx < CHUNK_SIZE; lx++) {
                        int x = baseX + lx;
                        for (int ly = 0; ly < CHUNK_SIZE; ly++) {
                            int y = baseY + ly;
                            for (int lz = 0; lz < CHUNK_SIZE; lz++) {
                                int z = baseZ + lz;
                                int local = localSectionCellIndex(lx, ly, lz);
                                int cell = gridCellIndex(x, y, z);
                                obstacle[cell] = snapshot.obstacle()[local] >= 0.5f ? (byte) 1 : (byte) 0;
                                surfaceKind[cell] = snapshot.surfaceKind()[local];
                                openFaceMask[cell] = (short) Byte.toUnsignedInt(snapshot.openFaceMask()[local]);
                                emitterPower[cell] = snapshot.emitterPowerWatts()[local];
                            }
                        }
                    }
                }
            }
        }
        boolean uploaded = simulationBridge.uploadStaticRegion(
            simulationServiceId,
            simulationRegionKey(key),
            GRID_SIZE,
            GRID_SIZE,
            GRID_SIZE,
            obstacle,
            surfaceKind,
            openFaceMask,
            emitterPower
        );
        if (uploaded) {
            region.staticUploaded = true;
            region.staticSignature = staticSignature;
        }
        return uploaded;
    }

    private void activateWindowRegionInSimulation(WindowKey key) {
        if (simulationServiceId == 0L) {
            return;
        }
        simulationBridge.activateRegion(
            simulationServiceId,
            simulationRegionKey(key),
            GRID_SIZE,
            GRID_SIZE,
            GRID_SIZE
        );
    }

    private void deactivateWindowRegionInSimulation(WindowKey key) {
        if (simulationServiceId == 0L) {
            return;
        }
        simulationBridge.deactivateRegion(simulationServiceId, simulationRegionKey(key));
    }

    private void refreshRegionLifecycle(WindowKey key, RegionRecord region) {
        if (simulationServiceId == 0L) {
            region.serviceActive = false;
            region.serviceReady = false;
            return;
        }
        long regionKey = simulationRegionKey(key);
        boolean hasRegion = simulationBridge.hasRegion(simulationServiceId, regionKey);
        region.serviceActive = hasRegion;
        region.serviceReady = hasRegion && simulationBridge.isRegionReady(simulationServiceId, regionKey);
    }

    private void attachRegionWindow(RegionRecord region, WindowState window) {
        region.attachWindow(window);
    }

    private WindowState detachRegionWindow(WindowKey key, RegionRecord region) {
        return region.detachWindow();
    }

    private void exportWindowDynamicRegionToSimulation(WindowKey key, WindowState window) {
        if (simulationServiceId == 0L) {
            return;
        }
        window.ensureDynamicStateInitialized();
        simulationBridge.importDynamicRegion(
            simulationServiceId,
            simulationRegionKey(key),
            GRID_SIZE,
            GRID_SIZE,
            GRID_SIZE,
            window.flowState,
            window.airTemperatureState,
            window.surfaceTemperatureState
        );
        ServerWorld world = resolveWorld(key.worldKey());
        if (world != null) {
            dynamicStore.storeRegion(
                world,
                key.worldKey(),
                key.origin(),
                GRID_SIZE,
                GRID_SIZE,
                GRID_SIZE,
                window.flowState,
                window.airTemperatureState,
                window.surfaceTemperatureState
            );
        }
    }

    private void tryRestoreWindowDynamicRegionFromSimulation(WindowKey key, RegionRecord region, WindowState window) {
        if (simulationServiceId == 0L || region.sections == null) {
            return;
        }
        window.ensureDynamicStateInitialized();
        if (!simulationBridge.exportDynamicRegion(
            simulationServiceId,
            simulationRegionKey(key),
            GRID_SIZE,
            GRID_SIZE,
            GRID_SIZE,
            window.flowState,
            window.airTemperatureState,
            window.surfaceTemperatureState
        )) {
            ServerWorld world = resolveWorld(key.worldKey());
            if (world == null || !dynamicStore.loadRegion(
                world,
                key.worldKey(),
                key.origin(),
                GRID_SIZE,
                GRID_SIZE,
                GRID_SIZE,
                window.flowState,
                window.airTemperatureState,
                window.surfaceTemperatureState
            )) {
                return;
            }
            simulationBridge.importDynamicRegion(
                simulationServiceId,
                simulationRegionKey(key),
                GRID_SIZE,
                GRID_SIZE,
                GRID_SIZE,
                window.flowState,
                window.airTemperatureState,
                window.surfaceTemperatureState
            );
        }
        window.markTemperatureRestorePending();
        window.markSeamSyncPending();
    }

    private ServerWorld resolveWorld(RegistryKey<World> worldKey) {
        MinecraftServer server = currentServer;
        return server == null ? null : server.getWorld(worldKey);
    }

    private boolean inBounds(int x, int y, int z) {
        return x >= 0 && y >= 0 && z >= 0 && x < GRID_SIZE && y < GRID_SIZE && z < GRID_SIZE;
    }

    private boolean inSectionBounds(int x, int y, int z) {
        return x >= 0 && y >= 0 && z >= 0 && x < CHUNK_SIZE && y < CHUNK_SIZE && z < CHUNK_SIZE;
    }

    private static float temperatureSourceFromPowerWatts(float thermalPowerWatts) {
        float scalar = thermalPowerWatts * SOLVER_STEP_SECONDS
            / (AIR_DENSITY_KG_PER_CUBIC_METER * AIR_SPECIFIC_HEAT_J_PER_KG_K
                * CELL_VOLUME_CUBIC_METERS * RUNTIME_TEMPERATURE_SCALE_KELVIN);
        return MathHelper.clamp(scalar, -NATIVE_THERMAL_SOURCE_MAX, NATIVE_THERMAL_SOURCE_MAX);
    }

    private static float temperatureSourceFromSurfaceFlux(float heatFluxWattsPerSquareMeter) {
        float scalar = heatFluxWattsPerSquareMeter * CELL_FACE_AREA_SQUARE_METERS * SOLVER_STEP_SECONDS
            / (AIR_DENSITY_KG_PER_CUBIC_METER * AIR_SPECIFIC_HEAT_J_PER_KG_K
                * CELL_VOLUME_CUBIC_METERS * RUNTIME_TEMPERATURE_SCALE_KELVIN);
        return MathHelper.clamp(scalar, -NATIVE_THERMAL_SOURCE_MAX, NATIVE_THERMAL_SOURCE_MAX);
    }

    private float runtimeFanSpeedMetersPerSecond() {
        return INFLOW_SPEED;
    }

    private void applyFanAtVoxel(WindowState window, int x, int y, int z, float fanVx, float fanVy, float fanVz) {
        if (!inBounds(x, y, z)) {
            return;
        }
        int idx = voxelIndex(x, y, z);
        if (window.solveField[idx + CH_OBSTACLE] > 0.5f) {
            return;
        }
        window.solveField[idx + CH_FAN_MASK] = 1.0f;
        window.solveField[idx + CH_FAN_VX] += fanVx;
        window.solveField[idx + CH_FAN_VY] += fanVy;
        window.solveField[idx + CH_FAN_VZ] += fanVz;
    }

    private void applyFanSource(WindowState window, FanSource fan, int minX, int minY, int minZ) {
        BlockPos inflowPos = fan.pos().offset(fan.facing());
        int cx = inflowPos.getX() - minX;
        int cy = inflowPos.getY() - minY;
        int cz = inflowPos.getZ() - minZ;

        float inflowSpeed = runtimeFanSpeedMetersPerSecond();
        float fanVx = fan.facing().getOffsetX() * inflowSpeed;
        float fanVy = fan.facing().getOffsetY() * inflowSpeed;
        float fanVz = fan.facing().getOffsetZ() * inflowSpeed;

        int radius2 = FAN_RADIUS * FAN_RADIUS;
        switch (fan.facing().getAxis()) {
            case X -> {
                for (int y = cy - FAN_RADIUS; y <= cy + FAN_RADIUS; y++) {
                    for (int z = cz - FAN_RADIUS; z <= cz + FAN_RADIUS; z++) {
                        int dy = y - cy;
                        int dz = z - cz;
                        if (dy * dy + dz * dz > radius2) {
                            continue;
                        }
                        applyFanAtVoxel(window, cx, y, z, fanVx, fanVy, fanVz);
                    }
                }
            }
            case Y -> {
                for (int x = cx - FAN_RADIUS; x <= cx + FAN_RADIUS; x++) {
                    for (int z = cz - FAN_RADIUS; z <= cz + FAN_RADIUS; z++) {
                        int dx = x - cx;
                        int dz = z - cz;
                        if (dx * dx + dz * dz > radius2) {
                            continue;
                        }
                        applyFanAtVoxel(window, x, cy, z, fanVx, fanVy, fanVz);
                    }
                }
            }
            case Z -> {
                for (int x = cx - FAN_RADIUS; x <= cx + FAN_RADIUS; x++) {
                    for (int y = cy - FAN_RADIUS; y <= cy + FAN_RADIUS; y++) {
                        int dx = x - cx;
                        int dy = y - cy;
                        if (dx * dx + dy * dy > radius2) {
                            continue;
                        }
                        applyFanAtVoxel(window, x, y, cz, fanVx, fanVy, fanVz);
                    }
                }
            }
            default -> applyFanAtVoxel(window, cx, cy, cz, fanVx, fanVy, fanVz);
        }
        applyDuctJet(window, fan, minX, minY, minZ);
    }

    private void applyDuctJet(WindowState window, FanSource fan, int minX, int minY, int minZ) {
        int level = ductLevel(fan.ductLength());
        if (level <= 0) {
            return;
        }

        BlockPos inflowPos = fan.pos().offset(fan.facing());
        int sx = inflowPos.getX() - minX;
        int sy = inflowPos.getY() - minY;
        int sz = inflowPos.getZ() - minZ;
        int dx = fan.facing().getOffsetX();
        int dy = fan.facing().getOffsetY();
        int dz = fan.facing().getOffsetZ();
        float levelBoost = switch (level) {
            case 1 -> 1.05f;
            case 2 -> 1.25f;
            default -> 1.55f;
        };

        float inflowSpeed = runtimeFanSpeedMetersPerSecond();
        float baseVx = dx * inflowSpeed;
        float baseVy = dy * inflowSpeed;
        float baseVz = dz * inflowSpeed;
        int range = switch (level) {
            case 1 -> 8;
            case 2 -> 14;
            default -> DUCT_JET_RANGE;
        };
        for (int step = 0; step < range; step++) {
            float t = range > 1 ? (float) step / (range - 1) : 0.0f;
            float decay = 1.0f - 0.55f * t;
            float coreScale = levelBoost * Math.max(0.35f, decay);
            int cx = sx + dx * step;
            int cy = sy + dy * step;
            int cz = sz + dz * step;
            applyFanAtVoxel(window, cx, cy, cz, baseVx * coreScale, baseVy * coreScale, baseVz * coreScale);

            float edgeFalloff = Math.max(0.10f, 1.0f - 0.90f * t);
            float edgeScale = coreScale * DUCT_EDGE_FACTOR * edgeFalloff;
            switch (fan.facing().getAxis()) {
                case X -> {
                    applyFanAtVoxel(window, cx, cy + 1, cz, baseVx * edgeScale, baseVy * edgeScale, baseVz * edgeScale);
                    applyFanAtVoxel(window, cx, cy - 1, cz, baseVx * edgeScale, baseVy * edgeScale, baseVz * edgeScale);
                    applyFanAtVoxel(window, cx, cy, cz + 1, baseVx * edgeScale, baseVy * edgeScale, baseVz * edgeScale);
                    applyFanAtVoxel(window, cx, cy, cz - 1, baseVx * edgeScale, baseVy * edgeScale, baseVz * edgeScale);
                }
                case Y -> {
                    applyFanAtVoxel(window, cx + 1, cy, cz, baseVx * edgeScale, baseVy * edgeScale, baseVz * edgeScale);
                    applyFanAtVoxel(window, cx - 1, cy, cz, baseVx * edgeScale, baseVy * edgeScale, baseVz * edgeScale);
                    applyFanAtVoxel(window, cx, cy, cz + 1, baseVx * edgeScale, baseVy * edgeScale, baseVz * edgeScale);
                    applyFanAtVoxel(window, cx, cy, cz - 1, baseVx * edgeScale, baseVy * edgeScale, baseVz * edgeScale);
                }
                case Z -> {
                    applyFanAtVoxel(window, cx + 1, cy, cz, baseVx * edgeScale, baseVy * edgeScale, baseVz * edgeScale);
                    applyFanAtVoxel(window, cx - 1, cy, cz, baseVx * edgeScale, baseVy * edgeScale, baseVz * edgeScale);
                    applyFanAtVoxel(window, cx, cy + 1, cz, baseVx * edgeScale, baseVy * edgeScale, baseVz * edgeScale);
                    applyFanAtVoxel(window, cx, cy - 1, cz, baseVx * edgeScale, baseVy * edgeScale, baseVz * edgeScale);
                }
            }
        }
    }

    private int ductLevel(int ductLength) {
        if (ductLength >= DUCT_LEVEL_THREE_MIN) {
            return 3;
        }
        if (ductLength >= DUCT_LEVEL_TWO_MIN) {
            return 2;
        }
        if (ductLength >= DUCT_LEVEL_ONE_MIN) {
            return 1;
        }
        return 0;
    }

    private SolveSnapshot createSolveSnapshot(
        WindowKey key,
        RegionRecord region,
        WindowState window,
        float speedCap,
        long generation
    ) {
        return new SolveSnapshot(
            key,
            region,
            window,
            key.origin(),
            List.copyOf(region.fans),
            speedCap,
            generation,
            sampleNestedBoundaryAtWindow(key)
        );
    }

    private BackgroundMetGrid.Sample sampleBackgroundMetAtWindow(WindowKey key) {
        BackgroundMetGrid grid = backgroundMetGrids.get(key.worldKey());
        if (grid == null) {
            return null;
        }
        return grid.sample(key.origin().add(GRID_SIZE / 2, GRID_SIZE / 2, GRID_SIZE / 2));
    }

    private MesoscaleGrid.Sample sampleMesoscaleMetAtWindow(WindowKey key) {
        MesoscaleGrid grid = mesoscaleMetGrids.get(key.worldKey());
        if (grid == null) {
            return null;
        }
        return grid.sample(key.origin().add(GRID_SIZE / 2, GRID_SIZE / 2, GRID_SIZE / 2));
    }

    private NestedBoundaryCoupler.BoundarySample sampleNestedBoundaryAtWindow(WindowKey key) {
        MesoscaleGrid.Sample mesoscaleSample = sampleMesoscaleMetAtWindow(key);
        if (mesoscaleSample != null) {
            return nestedBoundaryCoupler.fromMesoscaleSample(mesoscaleSample);
        }
        return nestedBoundaryCoupler.fromBackgroundSample(sampleBackgroundMetAtWindow(key));
    }

    private PreparedPayload captureWindow(SolveSnapshot snapshot) {
        WindowState window = snapshot.window();
        RegionRecord region = snapshot.region();
        int voxelCount = GRID_SIZE * GRID_SIZE * GRID_SIZE * CHANNELS;
        window.ensureDynamicStateInitialized();
        window.ensureSolveFieldInitialized();
        float[] solveField = window.solveField;
        NestedBoundaryCoupler.BoundarySample boundarySample = snapshot.boundarySample();
        boolean thermalBoundaryTouched = false;
        int minX = snapshot.origin().getX();
        int minY = snapshot.origin().getY();
        int minZ = snapshot.origin().getZ();
        int payloadBytes = voxelCount * Float.BYTES;
        ByteBuffer buffer = window.payloadBuffer(payloadBytes);
        buffer.clear();

        for (int sx = 0; sx < WINDOW_SECTION_COUNT; sx++) {
            for (int lx = 0; lx < CHUNK_SIZE; lx++) {
                int x = sx * CHUNK_SIZE + lx;
                for (int sy = 0; sy < WINDOW_SECTION_COUNT; sy++) {
                    for (int ly = 0; ly < CHUNK_SIZE; ly++) {
                int y = sy * CHUNK_SIZE + ly;
                for (int sz = 0; sz < WINDOW_SECTION_COUNT; sz++) {
                    WindowSection section = region.sectionAt(sx, sy, sz);
                    for (int lz = 0; lz < CHUNK_SIZE; lz++) {
                        int z = sz * CHUNK_SIZE + lz;
                        int localCell = localSectionCellIndex(lx, ly, lz);
                        int cell = (x * GRID_SIZE + y) * GRID_SIZE + z;
                        int idx = cell * CHANNELS;
                        int stateIdx = cell * RESPONSE_CHANNELS;
                        boolean solid = section == null || section.obstacle()[localCell] > 0.5f;
                        if (!solid && section != null && boundarySample != null) {
                            int edgeDistance = Math.min(
                                Math.min(Math.min(x, y), z),
                                Math.min(Math.min(GRID_SIZE - 1 - x, GRID_SIZE - 1 - y), GRID_SIZE - 1 - z)
                                    );
                                    if (edgeDistance < WINDOW_EDGE_STABILIZATION_LAYERS) {
                                        float eta = (WINDOW_EDGE_STABILIZATION_LAYERS - edgeDistance)
                                            / (float) WINDOW_EDGE_STABILIZATION_LAYERS;
                                        float keep = WINDOW_EDGE_STABILIZATION_MIN_KEEP
                                            + (1.0f - WINDOW_EDGE_STABILIZATION_MIN_KEEP) * (1.0f - eta * eta);
                                        float relaxed = window.airTemperatureState[cell] * keep;
                                        if (relaxed != window.airTemperatureState[cell]) {
                                            window.airTemperatureState[cell] = relaxed;
                                            thermalBoundaryTouched = true;
                                        }
                                    }
                                }
                                float sampledThermal = section == null ? 0.0f : section.thermal[localCell];
                                sampledThermal = MathHelper.clamp(sampledThermal, -NATIVE_THERMAL_SOURCE_MAX, NATIVE_THERMAL_SOURCE_MAX);

                                solveField[idx + CH_OBSTACLE] = solid ? 1.0f : 0.0f;
                                solveField[idx + CH_FAN_MASK] = 0.0f;
                                solveField[idx + CH_FAN_VX] = 0.0f;
                                solveField[idx + CH_FAN_VY] = 0.0f;
                                solveField[idx + CH_FAN_VZ] = 0.0f;
                                solveField[idx + CH_THERMAL_SOURCE] = solid ? 0.0f : sampledThermal;
                                if (solid || section == null) {
                                    solveField[idx + CH_STATE_VX] = 0.0f;
                                    solveField[idx + CH_STATE_VY] = 0.0f;
                                    solveField[idx + CH_STATE_VZ] = 0.0f;
                                    solveField[idx + CH_STATE_P] = 0.0f;
                                } else {
                                    float vx = window.flowState[stateIdx];
                                    float vy = window.flowState[stateIdx + 1];
                                    float vz = window.flowState[stateIdx + 2];
                                    float p = window.flowState[stateIdx + 3];
                                    solveField[idx + CH_STATE_VX] = Float.isFinite(vx) ? vx : 0.0f;
                                    solveField[idx + CH_STATE_VY] = Float.isFinite(vy) ? vy : 0.0f;
                                    solveField[idx + CH_STATE_VZ] = Float.isFinite(vz) ? vz : 0.0f;
                                    solveField[idx + CH_STATE_P] = Float.isFinite(p)
                                        ? MathHelper.clamp(p, STATE_PRESSURE_MIN, STATE_PRESSURE_MAX)
                                        : 0.0f;
                                }
                            }
                        }
                    }
                }
            }
        }

        for (FanSource fan : snapshot.fans()) {
            applyFanSource(window, fan, minX, minY, minZ);
        }
        if (thermalBoundaryTouched) {
            window.markTemperatureRestorePending();
        }
        nestedBoundaryCoupler.applyBackgroundWindBoundary(
            solveField,
            boundarySample,
            GRID_SIZE,
            CHANNELS,
            CH_OBSTACLE,
            CH_STATE_VX,
            CH_STATE_VY,
            CH_STATE_VZ,
            CH_STATE_P,
            WINDOW_EDGE_STABILIZATION_LAYERS,
            WINDOW_EDGE_STABILIZATION_MIN_KEEP
        );

        float nativeInputScale = 1.0f / NATIVE_VELOCITY_SCALE;
        int cellCount = GRID_SIZE * GRID_SIZE * GRID_SIZE;
        for (int i = 0; i < cellCount; i++) {
            int idx = i * CHANNELS;
            buffer.putFloat(solveField[idx + CH_OBSTACLE]);
            buffer.putFloat(solveField[idx + CH_FAN_MASK]);
            buffer.putFloat(solveField[idx + CH_FAN_VX]);
            buffer.putFloat(solveField[idx + CH_FAN_VY]);
            buffer.putFloat(solveField[idx + CH_FAN_VZ]);
            buffer.putFloat(solveField[idx + CH_STATE_VX] * nativeInputScale);
            buffer.putFloat(solveField[idx + CH_STATE_VY] * nativeInputScale);
            buffer.putFloat(solveField[idx + CH_STATE_VZ] * nativeInputScale);
            buffer.putFloat(solveField[idx + CH_STATE_P]);
            buffer.putFloat(solveField[idx + CH_THERMAL_SOURCE]);
        }

        return new PreparedPayload(buffer);
    }

    private boolean isSolidObstacle(ServerWorld world, BlockPos pos) {
        return isSolidObstacle(world, pos, world.getBlockState(pos));
    }

    private boolean isSolidObstacle(ServerWorld world, BlockPos pos, BlockState state) {
        if (state.isAir() || state.isOf(ModBlocks.DUCT_BLOCK)) {
            return false;
        }
        return !state.getCollisionShape(world, pos).isEmpty();
    }

    private float sampleEmitterThermalPowerWatts(BlockState state) {
        float powerWatts = 0.0f;
        if (state.isOf(Blocks.LAVA) || state.isOf(Blocks.LAVA_CAULDRON)) {
            powerWatts += THERMAL_EMITTER_POWER_LAVA_W;
        }
        if (state.isOf(Blocks.MAGMA_BLOCK)) {
            powerWatts += THERMAL_EMITTER_POWER_MAGMA_W;
        }
        if (state.isOf(Blocks.CAMPFIRE)) {
            powerWatts += state.getOrEmpty(Properties.LIT).orElse(false) ? THERMAL_EMITTER_POWER_CAMPFIRE_W : 0.0f;
        }
        if (state.isOf(Blocks.SOUL_CAMPFIRE)) {
            powerWatts += state.getOrEmpty(Properties.LIT).orElse(false) ? THERMAL_EMITTER_POWER_SOUL_CAMPFIRE_W : 0.0f;
        }
        if (state.isOf(Blocks.FIRE)) {
            powerWatts += THERMAL_EMITTER_POWER_FIRE_W;
        }
        if (state.isOf(Blocks.SOUL_FIRE)) {
            powerWatts += THERMAL_EMITTER_POWER_SOUL_FIRE_W;
        }
        if (state.isOf(Blocks.TORCH) || state.isOf(Blocks.WALL_TORCH)) {
            powerWatts += THERMAL_EMITTER_POWER_TORCH_W;
        }
        if (state.isOf(Blocks.SOUL_TORCH) || state.isOf(Blocks.SOUL_WALL_TORCH)) {
            powerWatts += THERMAL_EMITTER_POWER_SOUL_TORCH_W;
        }
        if (state.isOf(Blocks.LANTERN)) {
            powerWatts += THERMAL_EMITTER_POWER_LANTERN_W;
        }
        if (state.isOf(Blocks.SOUL_LANTERN)) {
            powerWatts += THERMAL_EMITTER_POWER_SOUL_LANTERN_W;
        }
        return Math.max(powerWatts, 0.0f);
    }

    private ThermalEnvironment sampleThermalEnvironment(
        WorldEnvironmentSnapshot snapshot,
        RegistryKey<World> worldKey,
        BlockPos samplePos,
        float surfaceDeltaSeconds
    ) {
        long timeOfDay = snapshot == null ? 6000L : snapshot.timeOfDay();
        float rainGradient = snapshot == null ? 0.0f : snapshot.rainGradient();
        float thunderGradient = snapshot == null ? 0.0f : snapshot.thunderGradient();
        int seaLevel = snapshot == null ? 63 : snapshot.seaLevel();
        float dayPhase = (float) Math.floorMod(timeOfDay, 24000L) / 24000.0f;
        float solarAltitude = Math.max(0.0f, (float) Math.sin(dayPhase * (float) (Math.PI * 2.0)));
        float rain = rainGradient;
        float thunder = thunderGradient;
        float clearSky = MathHelper.clamp(1.0f - 0.65f * rain - 0.25f * thunder, 0.15f, 1.0f);
        float directRadiation = THERMAL_SOLAR_DIRECT_FLUX_W_M2 * solarAltitude * clearSky;
        float diffuseRadiation = THERMAL_SOLAR_DIFFUSE_FLUX_W_M2
            * (0.30f + 0.70f * solarAltitude)
            * (0.55f + 0.45f * clearSky);
        float precipitationStrength = Math.max(rain, thunder * 0.60f);
        MesoscaleGrid.Sample mesoscaleSample = sampleMesoscaleMet(worldKey, samplePos);
        BackgroundMetGrid.Sample backgroundSample = mesoscaleSample == null ? sampleBackgroundMet(worldKey, samplePos) : null;
        float biomeTemperature = mesoscaleSample != null
            ? mesoscaleSample.biomeTemperature()
            : backgroundSample != null
                ? backgroundSample.biomeTemperature()
            : 0.8f;
        float ambientAirTemperatureKelvin;
        float deepGroundTemperatureKelvin;
        if (mesoscaleSample != null) {
            ambientAirTemperatureKelvin = mesoscaleSample.ambientAirTemperatureKelvin();
            deepGroundTemperatureKelvin = mesoscaleSample.deepGroundTemperatureKelvin();
        } else if (backgroundSample != null) {
            ambientAirTemperatureKelvin = backgroundSample.ambientAirTemperatureKelvin();
            deepGroundTemperatureKelvin = backgroundSample.deepGroundTemperatureKelvin();
        } else {
            float altitudeOffsetK = (samplePos.getY() - seaLevel) * THERMAL_ALTITUDE_LAPSE_RATE_K_PER_BLOCK;
            ambientAirTemperatureKelvin = THERMAL_BASE_AMBIENT_AIR_TEMPERATURE_K
                + (biomeTemperature - 0.8f) * THERMAL_BIOME_TEMPERATURE_SCALE_K
                - altitudeOffsetK;
            deepGroundTemperatureKelvin = ambientAirTemperatureKelvin + THERMAL_DEEP_GROUND_OFFSET_K;
        }
        float skyTemperatureDropK = MathHelper.lerp(solarAltitude, THERMAL_SKY_TEMP_DROP_NIGHT_K, THERMAL_SKY_TEMP_DROP_DAY_K);
        float skyTemperatureKelvin = ambientAirTemperatureKelvin - skyTemperatureDropK * clearSky;
        float precipitationTemperatureKelvin = ambientAirTemperatureKelvin - THERMAL_PRECIP_TEMP_DROP_K;
        float azimuth = dayPhase * (float) (Math.PI * 2.0) - (float) (Math.PI * 0.5);
        float horizontal = (float) Math.sqrt(Math.max(0.0, 1.0 - solarAltitude * solarAltitude));
        return new ThermalEnvironment(
            directRadiation,
            diffuseRadiation,
            ambientAirTemperatureKelvin,
            deepGroundTemperatureKelvin,
            skyTemperatureKelvin,
            precipitationTemperatureKelvin,
            precipitationStrength,
            (float) Math.cos(azimuth) * horizontal,
            solarAltitude,
            (float) Math.sin(azimuth) * horizontal,
            surfaceDeltaSeconds
        );
    }

    private float sampleSkyExposure(ServerWorld world, BlockPos pos) {
        return world.getLightLevel(LightType.SKY, pos) / 15.0f;
    }

    private float sampleDirectSunExposure(ServerWorld world, BlockPos pos) {
        return world.isSkyVisibleAllowingSea(pos) ? 1.0f : 0.0f;
    }

    private float sampleNeighborFluidTemperatureKelvin(
        float[] sectionTemperatureState,
        int x,
        int y,
        int z,
        ThermalEnvironment environment
    ) {
        if (!inSectionBounds(x, y, z) || sectionTemperatureState == null) {
            return environment.ambientAirTemperatureKelvin();
        }
        int localIndex = localSectionCellIndex(x, y, z);
        return environment.ambientAirTemperatureKelvin()
            + sectionTemperatureState[localIndex] * RUNTIME_TEMPERATURE_SCALE_KELVIN;
    }

    private float initializeSurfaceTemperatureKelvin(
        ThermalMaterial material,
        ThermalEnvironment environment,
        float emitterPowerWatts,
        int openFaces
    ) {
        float exposedArea = Math.max(1, openFaces) * CELL_FACE_AREA_SQUARE_METERS;
        float ambient = 0.70f * environment.ambientAirTemperatureKelvin()
            + 0.30f * environment.deepGroundTemperatureKelvin();
        float denominator = Math.max(
            1.0f,
            material.convectiveExchangeCoefficientWm2K() * exposedArea
                + material.bulkConductanceWm2K() * exposedArea
        );
        return MathHelper.clamp(
            ambient + emitterPowerWatts / denominator,
            THERMAL_SURFACE_INIT_MIN_K,
            THERMAL_SURFACE_MAX_K
        );
    }

    private boolean isStoneLikeTerrain(BlockState state) {
        return state.isOf(Blocks.STONE)
            || state.isOf(Blocks.COBBLESTONE)
            || state.isOf(Blocks.DEEPSLATE)
            || state.isOf(Blocks.COBBLED_DEEPSLATE)
            || state.isOf(Blocks.GRANITE)
            || state.isOf(Blocks.DIORITE)
            || state.isOf(Blocks.ANDESITE)
            || state.isOf(Blocks.TUFF)
            || state.isOf(Blocks.CALCITE)
            || state.isOf(Blocks.BLACKSTONE)
            || state.isOf(Blocks.BASALT);
    }

    private boolean isSoilSurface(BlockState state) {
        return state.isOf(Blocks.DIRT)
            || state.isOf(Blocks.COARSE_DIRT)
            || state.isOf(Blocks.ROOTED_DIRT)
            || state.isOf(Blocks.PODZOL)
            || state.isOf(Blocks.SAND)
            || state.isOf(Blocks.RED_SAND)
            || state.isOf(Blocks.GRAVEL)
            || state.isOf(Blocks.CLAY)
            || state.isOf(Blocks.MUD);
    }

    private boolean isVegetatedSurface(BlockState state) {
        return state.isOf(Blocks.GRASS_BLOCK)
            || state.isOf(Blocks.MYCELIUM)
            || state.isOf(Blocks.MOSS_BLOCK);
    }

    private boolean isSnowOrIceSurface(BlockState state) {
        return state.isOf(Blocks.SNOW_BLOCK)
            || state.isOf(Blocks.ICE)
            || state.isOf(Blocks.PACKED_ICE)
            || state.isOf(Blocks.BLUE_ICE);
    }

    private boolean isWaterSurface(BlockState state) {
        return state.getFluidState().isIn(FluidTags.WATER);
    }

    private boolean isMoltenSurface(BlockState state) {
        return state.isOf(Blocks.LAVA)
            || state.isOf(Blocks.LAVA_CAULDRON);
    }

    private ThermalMaterial thermalMaterial(BlockState state) {
        if (isMoltenSurface(state)) {
            return ThermalMaterial.MOLTEN;
        }
        if (isWaterSurface(state)) {
            return ThermalMaterial.WATER;
        }
        if (isSnowOrIceSurface(state)) {
            return ThermalMaterial.SNOW_ICE;
        }
        if (isVegetatedSurface(state)) {
            return ThermalMaterial.VEGETATION;
        }
        if (isSoilSurface(state)) {
            return ThermalMaterial.SOIL;
        }
        if (isStoneLikeTerrain(state)) {
            return ThermalMaterial.ROCK;
        }
        return null;
    }

    private ThermalMaterial thermalMaterial(byte kind) {
        return switch (kind) {
            case SURFACE_KIND_ROCK -> ThermalMaterial.ROCK;
            case SURFACE_KIND_SOIL -> ThermalMaterial.SOIL;
            case SURFACE_KIND_VEGETATION -> ThermalMaterial.VEGETATION;
            case SURFACE_KIND_SNOW_ICE -> ThermalMaterial.SNOW_ICE;
            case SURFACE_KIND_WATER -> ThermalMaterial.WATER;
            case SURFACE_KIND_MOLTEN -> ThermalMaterial.MOLTEN;
            default -> null;
        };
    }

    private int faceDataIndex(int cell, Direction direction) {
        return cell * FACE_COUNT + direction.ordinal();
    }

    private byte setFaceBit(byte mask, Direction direction) {
        return (byte) (mask | (1 << direction.ordinal()));
    }

    private boolean hasFaceBit(byte mask, Direction direction) {
        return (mask & (1 << direction.ordinal())) != 0;
    }

    private byte quantizeUnitFloat(float value) {
        return (byte) MathHelper.clamp(Math.round(MathHelper.clamp(value, 0.0f, 1.0f) * 255.0f), 0, 255);
    }

    private float dequantizeUnitFloat(byte value) {
        return (value & 0xFF) / 255.0f;
    }

    private void addSectionThermalSource(float[] thermal, int x, int y, int z, float source) {
        if (source == 0.0f || !inSectionBounds(x, y, z)) {
            return;
        }
        int cell = localSectionCellIndex(x, y, z);
        float updated = thermal[cell] + source;
        thermal[cell] = MathHelper.clamp(updated, -NATIVE_THERMAL_SOURCE_MAX, NATIVE_THERMAL_SOURCE_MAX);
    }

    private void sectionStaticThermalFields(
        ServerWorld world,
        BlockPos pos,
        BlockState state,
        int x,
        int y,
        int z,
        ThermalMaterial material,
        WindowSection section
    ) {
        sectionStaticThermalFields(
            world,
            pos,
            state,
            x,
            y,
            z,
            material,
            section == null ? null : section.emitterPowerWatts(),
            section == null ? null : section.openFaceMask(),
            section == null ? null : section.faceSkyExposure(),
            section == null ? null : section.faceDirectExposure()
        );
    }

    private void sectionStaticThermalFields(
        ServerWorld world,
        BlockPos pos,
        BlockState state,
        int x,
        int y,
        int z,
        ThermalMaterial material,
        float[] emitterPowerWatts,
        byte[] openFaceMaskField,
        byte[] faceSkyExposure,
        byte[] faceDirectExposure
    ) {
        if (!inSectionBounds(x, y, z)) {
            return;
        }
        int cell = localSectionCellIndex(x, y, z);
        float emitterPower = sampleEmitterThermalPowerWatts(state);
        byte openFaceMask = 0;
        if (emitterPowerWatts != null) {
            emitterPowerWatts[cell] = emitterPower;
        }
        if (openFaceMaskField != null) {
            openFaceMaskField[cell] = 0;
        }
        if (faceSkyExposure != null) {
            Arrays.fill(faceSkyExposure, cell * FACE_COUNT, cell * FACE_COUNT + FACE_COUNT, (byte) 0);
        }
        if (faceDirectExposure != null) {
            Arrays.fill(faceDirectExposure, cell * FACE_COUNT, cell * FACE_COUNT + FACE_COUNT, (byte) 0);
        }
        if (material == null && emitterPower <= 0.0f) {
            return;
        }
        BlockPos.Mutable neighborCursor = new BlockPos.Mutable();
        for (Direction direction : CARDINAL_DIRECTIONS) {
            neighborCursor.set(
                pos.getX() + direction.getOffsetX(),
                pos.getY() + direction.getOffsetY(),
                pos.getZ() + direction.getOffsetZ()
            );
            BlockState neighborState = world.getBlockState(neighborCursor);
            boolean openFace;
            if (material != null) {
                openFace = material.atmosphericExchangeRequiresAirNeighbor()
                    ? neighborState.isAir()
                    : !isSolidObstacle(world, neighborCursor, neighborState);
            } else {
                openFace = neighborState.isAir();
            }
            if (!openFace) {
                continue;
            }
            openFaceMask = setFaceBit(openFaceMask, direction);
            if (faceSkyExposure != null && faceDirectExposure != null) {
                int faceIndex = faceDataIndex(cell, direction);
                faceSkyExposure[faceIndex] = quantizeUnitFloat(sampleSkyExposure(world, neighborCursor));
                faceDirectExposure[faceIndex] = quantizeUnitFloat(sampleDirectSunExposure(world, neighborCursor));
            }
        }
        if (openFaceMaskField != null) {
            openFaceMaskField[cell] = openFaceMask;
        }
    }

    private void recomputeSectionThermalFields(
        WindowSection section,
        float[] sectionTemperatureState,
        float[] thermalOut,
        float[] surfaceTemperatureOut,
        byte[] surfaceKindOut,
        ThermalEnvironment environment
    ) {
        Arrays.fill(thermalOut, 0.0f);
        if (section == null) {
            return;
        }
        for (int x = 0; x < CHUNK_SIZE; x++) {
            for (int y = 0; y < CHUNK_SIZE; y++) {
                for (int z = 0; z < CHUNK_SIZE; z++) {
                    int cell = localSectionCellIndex(x, y, z);
                    ThermalMaterial material = thermalMaterial(surfaceKindOut[cell]);
                    float emitterPowerWatts = section.emitterPowerWatts()[cell];
                    byte openFaceMask = section.openFaceMask()[cell];
                    if (material != null) {
                        emitSurfaceThermalSourceCached(
                            thermalOut,
                            surfaceTemperatureOut,
                            sectionTemperatureState,
                            x,
                            y,
                            z,
                            material,
                            emitterPowerWatts,
                            openFaceMask,
                            section.faceSkyExposure(),
                            section.faceDirectExposure(),
                            environment
                        );
                    } else if (emitterPowerWatts > 0.0f) {
                        emitUnsupportedEmitterThermalSourceCached(
                            thermalOut,
                            section.obstacle(),
                            x,
                            y,
                            z,
                            emitterPowerWatts,
                            openFaceMask
                        );
                    } else {
                        surfaceTemperatureOut[cell] = 0.0f;
                    }
                }
            }
        }
    }

    private void emitUnsupportedEmitterThermalSourceCached(
        float[] thermal,
        float[] obstacleField,
        int x,
        int y,
        int z,
        float emitterPowerWatts,
        byte openFaceMask
    ) {
        if (emitterPowerWatts <= 0.0f || openFaceMask == 0) {
            return;
        }
        int cell = localSectionCellIndex(x, y, z);
        float selfWeight = obstacleField[cell] != 0.0f ? 0.0f : 0.30f;
        float totalWeight = selfWeight;
        float[] faceWeights = new float[FACE_COUNT];
        for (Direction direction : CARDINAL_DIRECTIONS) {
            if (!hasFaceBit(openFaceMask, direction)) {
                continue;
            }
            float weight = switch (direction) {
                case UP -> 0.55f;
                case DOWN -> 0.03f;
                default -> 0.12f;
            };
            faceWeights[direction.ordinal()] = weight;
            totalWeight += weight;
        }
        if (totalWeight <= 1.0e-6f) {
            return;
        }
        float scalarSource = temperatureSourceFromPowerWatts(emitterPowerWatts);
        if (selfWeight > 0.0f) {
            addSectionThermalSource(thermal, x, y, z, scalarSource * (selfWeight / totalWeight));
        }
        for (Direction direction : CARDINAL_DIRECTIONS) {
            float weight = faceWeights[direction.ordinal()];
            if (weight <= 0.0f) {
                continue;
            }
            addSectionThermalSource(
                thermal,
                x + direction.getOffsetX(),
                y + direction.getOffsetY(),
                z + direction.getOffsetZ(),
                scalarSource * (weight / totalWeight)
            );
        }
    }

    private void emitSurfaceThermalSourceCached(
        float[] thermal,
        float[] surfaceTemperature,
        float[] sectionTemperatureState,
        int x,
        int y,
        int z,
        ThermalMaterial material,
        float emitterPowerWatts,
        byte openFaceMask,
        byte[] faceSkyExposure,
        byte[] faceDirectExposure,
        ThermalEnvironment environment
    ) {
        if (openFaceMask == 0) {
            surfaceTemperature[localSectionCellIndex(x, y, z)] = 0.0f;
            return;
        }
        int surfaceLocalIndex = localSectionCellIndex(x, y, z);
        int openFaces = Integer.bitCount(openFaceMask & 0xFF);
        float currentSurfaceTemperatureKelvin = surfaceTemperature[surfaceLocalIndex];
        if (!Float.isFinite(currentSurfaceTemperatureKelvin) || currentSurfaceTemperatureKelvin <= 0.0f) {
            currentSurfaceTemperatureKelvin = initializeSurfaceTemperatureKelvin(
                material,
                environment,
                emitterPowerWatts,
                openFaces
            );
        }

        float solarWatts = 0.0f;
        float longwaveWatts = 0.0f;
        float rainWatts = 0.0f;
        float convectiveWatts = 0.0f;
        for (Direction direction : CARDINAL_DIRECTIONS) {
            if (!hasFaceBit(openFaceMask, direction)) {
                continue;
            }
            int faceIndex = faceDataIndex(surfaceLocalIndex, direction);
            float diffuseSky = dequantizeUnitFloat(faceSkyExposure[faceIndex]);
            float directSky = dequantizeUnitFloat(faceDirectExposure[faceIndex]);
            float sunDot = Math.max(
                0.0f,
                direction.getOffsetX() * environment.sunX()
                    + direction.getOffsetY() * environment.sunY()
                    + direction.getOffsetZ() * environment.sunZ()
            );
            float diffuseWeight = switch (direction) {
                case UP -> 1.0f;
                case DOWN -> 0.05f;
                default -> 0.42f;
            };
            float skyWeight = switch (direction) {
                case UP -> 1.0f;
                case DOWN -> 0.08f;
                default -> 0.55f;
            };
            float rainWeight = switch (direction) {
                case UP -> 1.0f;
                case DOWN -> 0.0f;
                default -> 0.35f;
            };
            float airTemperatureKelvin = sampleNeighborFluidTemperatureKelvin(
                sectionTemperatureState,
                x + direction.getOffsetX(),
                y + direction.getOffsetY(),
                z + direction.getOffsetZ(),
                environment
            );
            solarWatts += material.solarAbsorptivity()
                * CELL_FACE_AREA_SQUARE_METERS
                * (environment.directSolarFluxWm2() * directSky * sunDot
                    + environment.diffuseSolarFluxWm2() * diffuseSky * diffuseWeight);
            float surfaceTempSq = currentSurfaceTemperatureKelvin * currentSurfaceTemperatureKelvin;
            float skyTempSq = environment.skyTemperatureKelvin() * environment.skyTemperatureKelvin();
            longwaveWatts += material.emissivity()
                * THERMAL_STEFAN_BOLTZMANN
                * CELL_FACE_AREA_SQUARE_METERS
                * (skyTempSq * skyTempSq - surfaceTempSq * surfaceTempSq)
                * diffuseSky
                * skyWeight;
            rainWatts += material.rainExchangeCoefficientWm2K()
                * CELL_FACE_AREA_SQUARE_METERS
                * environment.precipitationStrength()
                * rainWeight
                * (environment.precipitationTemperatureKelvin() - currentSurfaceTemperatureKelvin);
            convectiveWatts += material.convectiveExchangeCoefficientWm2K()
                * CELL_FACE_AREA_SQUARE_METERS
                * (airTemperatureKelvin - currentSurfaceTemperatureKelvin);
        }
        float exposedArea = openFaces * CELL_FACE_AREA_SQUARE_METERS;
        float bulkWatts = material.bulkConductanceWm2K()
            * exposedArea
            * (environment.deepGroundTemperatureKelvin() - currentSurfaceTemperatureKelvin);
        float thermalMassJPerK = Math.max(1.0f, material.surfaceHeatCapacityJm2K() * exposedArea);
        float updatedSurfaceTemperatureKelvin = MathHelper.clamp(
            currentSurfaceTemperatureKelvin
                + (solarWatts + longwaveWatts + rainWatts + bulkWatts + convectiveWatts + emitterPowerWatts)
                    * environment.surfaceDeltaSeconds()
                    / thermalMassJPerK,
            THERMAL_SURFACE_INIT_MIN_K,
            THERMAL_SURFACE_MAX_K
        );
        surfaceTemperature[surfaceLocalIndex] = updatedSurfaceTemperatureKelvin;
        for (Direction direction : CARDINAL_DIRECTIONS) {
            if (!hasFaceBit(openFaceMask, direction)) {
                continue;
            }
            float airTemperatureKelvin = sampleNeighborFluidTemperatureKelvin(
                sectionTemperatureState,
                x + direction.getOffsetX(),
                y + direction.getOffsetY(),
                z + direction.getOffsetZ(),
                environment
            );
            float convectivePowerWatts = material.convectiveExchangeCoefficientWm2K()
                * CELL_FACE_AREA_SQUARE_METERS
                * (updatedSurfaceTemperatureKelvin - airTemperatureKelvin);
            addSectionThermalSource(
                thermal,
                x + direction.getOffsetX(),
                y + direction.getOffsetY(),
                z + direction.getOffsetZ(),
                temperatureSourceFromPowerWatts(convectivePowerWatts)
            );
        }
    }

    private void sampleSectionThermalFields(
        RegistryKey<World> worldKey,
        BlockPos origin,
        WindowSection section,
        float[] sectionTemperatureState,
        float[] thermalOut,
        float[] surfaceTemperatureOut,
        byte[] surfaceKindOut,
        float surfaceDeltaSeconds
    ) {
        System.arraycopy(section.thermalSurfaceKind, 0, surfaceKindOut, 0, SECTION_CELL_COUNT);
        ThermalEnvironment environment = sampleThermalEnvironment(
            worldEnvironmentSnapshots.get(worldKey),
            worldKey,
            new BlockPos(origin.getX() + CHUNK_SIZE / 2, origin.getY() + CHUNK_SIZE / 2, origin.getZ() + CHUNK_SIZE / 2),
            surfaceDeltaSeconds
        );
        recomputeSectionThermalFields(section, sectionTemperatureState, thermalOut, surfaceTemperatureOut, surfaceKindOut, environment);
    }

    private void refreshWindowThermalFields(WindowKey key, RegionRecord region, WindowState window) {
        if (region.sections == null) {
            return;
        }
        window.ensureDynamicStateInitialized();
        float deltaSeconds = Math.max(1, tickCounter - region.lastThermalRefreshTick) * SOLVER_STEP_SECONDS;
        float[] thermalScratch = new float[SECTION_CELL_COUNT];
        float[] sectionTemperatureScratch = new float[SECTION_CELL_COUNT];
        float[] surfaceTemperatureScratch = new float[SECTION_CELL_COUNT];
        byte[] surfaceKindScratch = new byte[SECTION_CELL_COUNT];
        for (int sx = 0; sx < WINDOW_SECTION_COUNT; sx++) {
            for (int sy = 0; sy < WINDOW_SECTION_COUNT; sy++) {
                for (int sz = 0; sz < WINDOW_SECTION_COUNT; sz++) {
                    WindowSection section = region.sectionAt(sx, sy, sz);
                    if (section == null) {
                        continue;
                    }
                    copyWindowScalarSectionToBuffer(window.airTemperatureState, sx, sy, sz, sectionTemperatureScratch);
                    copyWindowScalarSectionToBuffer(window.surfaceTemperatureState, sx, sy, sz, surfaceTemperatureScratch);
                    sampleSectionThermalFields(
                        key.worldKey(),
                        sectionOrigin(key.origin(), sx, sy, sz),
                        section,
                        sectionTemperatureScratch,
                        thermalScratch,
                        surfaceTemperatureScratch,
                        surfaceKindScratch,
                        deltaSeconds
                    );
                    System.arraycopy(thermalScratch, 0, section.thermal, 0, SECTION_CELL_COUNT);
                    System.arraycopy(surfaceKindScratch, 0, section.thermalSurfaceKind, 0, SECTION_CELL_COUNT);
                    copyBufferToWindowScalarSection(surfaceTemperatureScratch, sx, sy, sz, window.surfaceTemperatureState);
                }
            }
        }
        region.lastThermalRefreshTick = tickCounter;
    }

    private float[] runSolverStep(SolveSnapshot snapshot, PreparedPayload payload) throws IOException {
        if (!simulationBridge.isLoaded()) {
            throw new IOException("Simulation bridge not loaded: " + simulationBridge.getLoadError());
        }
        if (!ensureSimulationL2Runtime()) {
            throw new IOException("Native backend initialization failed");
        }
        restoreTemperatureStateToNative(snapshot.key(), snapshot.window());
        float[] response = simulationBridge.stepRegion(
            simulationServiceId,
            simulationRegionKey(snapshot.key()),
            payload.nativePayload(),
            GRID_SIZE,
            GRID_SIZE,
            GRID_SIZE
        );
        if (response == null || response.length != FLOW_COUNT) {
            throw new IOException("Native backend returned invalid response");
        }
        scaleResponseVelocity(response, NATIVE_VELOCITY_SCALE);
        return response;
    }

    private SolveSanitizeResult sanitizeSolveResponse(float[] response, float speedCap) {
        if (response == null) {
            return new SolveSanitizeResult(0.0f, 0);
        }

        int cellCount = GRID_SIZE * GRID_SIZE * GRID_SIZE;
        float capped = Math.max(0.0f, speedCap);
        float maxSpeedThisStep = 0.0f;
        int invalidComponents = 0;
        for (int i = 0; i < cellCount; i++) {
            int respIdx = i * RESPONSE_CHANNELS;
            float vx = response[respIdx];
            float vy = response[respIdx + 1];
            float vz = response[respIdx + 2];
            float p = response[respIdx + 3];

            if (!Float.isFinite(vx)) {
                vx = 0.0f;
                invalidComponents++;
            }
            if (!Float.isFinite(vy)) {
                vy = 0.0f;
                invalidComponents++;
            }
            if (!Float.isFinite(vz)) {
                vz = 0.0f;
                invalidComponents++;
            }
            if (!Float.isFinite(p)) {
                p = 0.0f;
                invalidComponents++;
            }
            p = MathHelper.clamp(p, STATE_PRESSURE_MIN, STATE_PRESSURE_MAX);

            if (capped > 0.0f) {
                float speedSq = vx * vx + vy * vy + vz * vz;
                if (!Float.isFinite(speedSq)) {
                    vx = 0.0f;
                    vy = 0.0f;
                    vz = 0.0f;
                    invalidComponents += 3;
                    speedSq = 0.0f;
                }
                float capSq = capped * capped;
                if (speedSq > capSq) {
                    float scale = capped / (float) Math.sqrt(speedSq);
                    if (!Float.isFinite(scale)) {
                        vx = 0.0f;
                        vy = 0.0f;
                        vz = 0.0f;
                        invalidComponents += 3;
                    } else {
                        vx *= scale;
                        vy *= scale;
                        vz *= scale;
                    }
                }
            }

            response[respIdx] = vx;
            response[respIdx + 1] = vy;
            response[respIdx + 2] = vz;
            response[respIdx + 3] = p;
            float speed = (float) Math.sqrt(vx * vx + vy * vy + vz * vz);
            if (Float.isFinite(speed) && speed > maxSpeedThisStep) {
                maxSpeedThisStep = speed;
            }
        }
        return new SolveSanitizeResult(maxSpeedThisStep, invalidComponents);
    }

    private void applyStateFieldFromResponse(WindowState window, float[] response) {
        if (response == null) {
            return;
        }
        window.ensureDynamicStateInitialized();
        System.arraycopy(response, 0, window.flowState, 0, Math.min(response.length, window.flowState.length));
        window.markSeamSyncPending();
    }

    private void scaleResponseVelocity(float[] response, float velocityScale) {
        if (velocityScale == 1.0f) {
            return;
        }

        int cellCount = GRID_SIZE * GRID_SIZE * GRID_SIZE;
        for (int i = 0; i < cellCount; i++) {
            int respIdx = i * RESPONSE_CHANNELS;
            response[respIdx] *= velocityScale;
            response[respIdx + 1] *= velocityScale;
            response[respIdx + 2] *= velocityScale;
        }
    }

    private int sanitizeStride(int requested) {
        return (requested == 1 || requested == 2 || requested == 4 || requested == 8) ? requested : PARTICLE_FLOW_SAMPLE_STRIDE;
    }

    private void updateSimulationRate(int steppedTicks) {
        secondWindowTotalTicks++;
        if (steppedTicks > 0) {
            secondWindowSimulationTicks += steppedTicks;
        }
        if (secondWindowTotalTicks >= TICKS_PER_SECOND) {
            simulationTicksPerSecond = (secondWindowSimulationTicks * (float) TICKS_PER_SECOND) / secondWindowTotalTicks;
            secondWindowTotalTicks = 0;
            secondWindowSimulationTicks = 0;
        }
    }

    private void grantSimulationStepBudget() {
        while (true) {
            int current = simulationStepBudget.get();
            if (current >= MAX_SIMULATION_STEP_BACKLOG) {
                return;
            }
            if (simulationStepBudget.compareAndSet(current, current + 1)) {
                return;
            }
        }
    }

    private void sendStateToPlayer(ServerPlayerEntity player, MinecraftServer server) {
        ServerPlayNetworking.send(player, new AeroRuntimeStatePayload(streamingEnabled));
    }

    private void broadcastState(MinecraftServer server) {
        for (ServerPlayerEntity player : server.getPlayerManager().getPlayerList()) {
            sendStateToPlayer(player, server);
        }
    }

    private void sendFlowSnapshotToPlayer(ServerPlayerEntity player, MinecraftServer server) {
        PublishedFrame frame = publishedFrame.get();
        if (frame == null) {
            return;
        }
        int stride = PARTICLE_FLOW_SAMPLE_STRIDE;
        for (Map.Entry<WindowKey, short[]> entry : frame.regionAtlases().entrySet()) {
            WindowKey key = entry.getKey();
            short[] packed = entry.getValue();
            Identifier dimId = key.worldKey().getValue();
            ServerPlayNetworking.send(player, new AeroFlowPayload(dimId, key.origin(), stride, packed));
        }
    }

    private void waitForSolverIdle() {
        while (activeSolveTasks.get() != 0) {
            try {
                Thread.sleep(1L);
            } catch (InterruptedException ex) {
                Thread.currentThread().interrupt();
                return;
            }
        }
    }

    private void runSolveTask(SolveSnapshot snapshot, CountDownLatch completionLatch) {
        WindowState window = snapshot.window();
        try {
            if (snapshot.generation() != runtimeGeneration.get()) {
                return;
            }
            PreparedPayload payload = captureWindow(snapshot);
            float[] response = runSolverStep(snapshot, payload);
            SolveSanitizeResult sanitize = sanitizeSolveResponse(response, snapshot.speedCap());
            if (sanitize.invalidComponents() > 0) {
                boolean resetWasPending = window.backendResetPending();
                window.markBackendResetPending();
                String message = "Detected non-finite solver response at "
                    + formatPos(snapshot.origin())
                    + " (" + sanitize.invalidComponents() + " components sanitized)";
                if (!resetWasPending) {
                    log(message + "; backend reset scheduled");
                }
                if (snapshot.generation() == runtimeGeneration.get()) {
                    lastSolverError = message;
                }
            } else if (snapshot.generation() == runtimeGeneration.get()) {
                lastSolverError = "";
            }
            if (snapshot.generation() == runtimeGeneration.get()) {
                window.publishCompletedSolveResult(new SolveResult(response, sanitize.maxSpeed(), snapshot.origin()));
            }
        } catch (IOException ex) {
            if (snapshot.generation() == runtimeGeneration.get()) {
                lastSolverError = ex.getMessage();
            }
            log("Solver error for window " + formatPos(snapshot.origin()) + ": " + ex.getMessage());
            if (simulationServiceId != 0L) {
                simulationBridge.releaseRegionRuntime(simulationServiceId, simulationRegionKey(snapshot.key()));
            }
        } finally {
            window.busy.set(false);
            if (window.detached()) {
                releaseWindow(snapshot.key(), window);
            }
            activeSolveTasks.decrementAndGet();
            if (completionLatch != null) {
                completionLatch.countDown();
            }
        }
    }

    private void ensureSimulationCoordinatorRunning() {
        synchronized (simulationStateLock) {
            if (simulationCoordinator != null && simulationCoordinator.running()) {
                return;
            }
            simulationCoordinator = new SimulationCoordinator();
            simulationCoordinator.start();
        }
    }

    private void stopSimulationCoordinator() {
        SimulationCoordinator coordinator;
        synchronized (simulationStateLock) {
            coordinator = simulationCoordinator;
            simulationCoordinator = null;
        }
        if (coordinator != null) {
            coordinator.shutdown();
        }
    }

    private List<ActiveWindow> snapshotActiveWindowsForCoordinatorLocked() {
        if (regions.isEmpty()) {
            return List.of();
        }
        List<ActiveWindow> snapshot = new ArrayList<>(regions.size());
        for (Map.Entry<WindowKey, RegionRecord> entry : regions.entrySet()) {
            RegionRecord region = entry.getValue();
            if (!region.serviceReady) {
                continue;
            }
            WindowState window = region.attachedWindow();
            if (window == null || region.sections == null || window.detached()) {
                continue;
            }
            snapshot.add(new ActiveWindow(entry.getKey(), region, window));
        }
        return snapshot;
    }

    private float applyCompletedResults(List<ActiveWindow> activeWindows) {
        float maxSpeedThisCycle = 0.0f;
        for (ActiveWindow active : activeWindows) {
            SolveResult completed = consumeCompletedSolveResult(active.window(), active.key().origin());
            if (completed == null) {
                continue;
            }
            applyStateFieldFromResponse(active.window(), completed.flow());
            active.window().markSeamSyncPending();
            maxSpeedThisCycle = Math.max(maxSpeedThisCycle, completed.maxSpeed());
            lastSolverError = "";
        }
        return maxSpeedThisCycle;
    }

    private void resetPendingBackends(List<ActiveWindow> activeWindows) {
        for (ActiveWindow active : activeWindows) {
            WindowState window = active.window();
            if (!window.busy.get() && window.backendResetPending()) {
                resetWindowBackend(active.key(), window);
            }
        }
    }

    private CountDownLatch scheduleSolveCycle(List<ActiveWindow> activeWindows) {
        List<SolveSnapshot> scheduled = new ArrayList<>(activeWindows.size());
        for (ActiveWindow active : activeWindows) {
            WindowState window = active.window();
            if (!window.busy.compareAndSet(false, true)) {
                continue;
            }
            SolveSnapshot snapshot = createSolveSnapshot(
                active.key(),
                active.region(),
                window,
                MAX_RUNTIME_WIND_SPEED,
                runtimeGeneration.get()
            );
            scheduled.add(snapshot);
        }
        if (scheduled.isEmpty()) {
            return null;
        }
        CountDownLatch completionLatch = new CountDownLatch(scheduled.size());
        for (SolveSnapshot snapshot : scheduled) {
            activeSolveTasks.incrementAndGet();
            solverExecutor.execute(() -> runSolveTask(snapshot, completionLatch));
        }
        return completionLatch;
    }

    private void waitForScheduledSnapshots(CountDownLatch completionLatch) {
        if (completionLatch == null) {
            return;
        }
        try {
            while (streamingEnabled
                && !Thread.currentThread().isInterrupted()
                && !completionLatch.await(5L, TimeUnit.MILLISECONDS)) {
            }
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }

    private boolean activeSetStillMatches(List<ActiveWindow> snapshot) {
        synchronized (simulationStateLock) {
            if (attachedWindowCount() != snapshot.size()) {
                return false;
            }
            for (ActiveWindow active : snapshot) {
                RegionRecord region = regions.get(active.key());
                if (region == null
                    || !region.serviceReady
                    || region.attachedWindow() != active.window()
                    || active.window().detached()) {
                    return false;
                }
            }
            return true;
        }
    }

    private int attachedWindowCount() {
        int count = 0;
        for (RegionRecord region : regions.values()) {
            if (region.serviceReady && region.attachedWindow() != null) {
                count++;
            }
        }
        return count;
    }

    private void publishFrame(List<ActiveWindow> activeWindows, float maxSpeedThisCycle) {
        Map<WindowKey, short[]> regionAtlases = snapshotPublishedAtlases(activeWindows, PARTICLE_FLOW_SAMPLE_STRIDE);
        if (regionAtlases.isEmpty()) {
            return;
        }
        long frameId = publishedFrameCounter.incrementAndGet();
        PublishedFrame frame = new PublishedFrame(frameId, maxSpeedThisCycle, Map.copyOf(regionAtlases));
        publishedFrame.set(frame);
    }

    private Map<WindowKey, short[]> snapshotPublishedAtlases(List<ActiveWindow> activeWindows, int stride) {
        Map<WindowKey, short[]> regionAtlases = new HashMap<>(activeWindows.size());
        for (ActiveWindow active : activeWindows) {
            if (active.region().sections == null) {
                continue;
            }
            regionAtlases.put(active.key(), packedFlowForNetwork(active.key(), stride));
        }
        return regionAtlases;
    }

    private void syncPublishedFlowToPlayers(MinecraftServer server, PublishedFrame frame, int stride) {
        int sampleStride = sanitizeStride(stride);
        for (Map.Entry<WindowKey, short[]> entry : frame.regionAtlases().entrySet()) {
            WindowKey key = entry.getKey();
            ServerWorld world = server.getWorld(key.worldKey());
            if (world == null) {
                continue;
            }
            short[] packed = entry.getValue();
            AeroFlowPayload payload = new AeroFlowPayload(world.getRegistryKey().getValue(), key.origin(), sampleStride, packed);
            for (ServerPlayerEntity player : world.getPlayers()) {
                ServerPlayNetworking.send(player, payload);
            }
        }
    }

    private short[] packedFlowForNetwork(WindowKey key, int stride) {
        int sampleStride = sanitizeStride(stride);
        int n = (GRID_SIZE + sampleStride - 1) / sampleStride;
        short[] packed = new short[n * n * n * NativeSimulationBridge.PACKED_ATLAS_CHANNELS];
        if (simulationServiceId != 0L
            && simulationBridge.pollPackedFlowAtlas(simulationServiceId, simulationRegionKey(key), packed)) {
            return packed;
        }
        return packed;
    }

    private void releaseWindow(WindowKey key, WindowState window) {
        if (!window.markReleased()) {
            return;
        }
        if (simulationServiceId != 0L) {
            simulationBridge.releaseRegionRuntime(simulationServiceId, simulationRegionKey(key));
        }
    }

    private void feedback(ServerCommandSource source, String message) {
        source.sendFeedback(() -> Text.literal(LOG_PREFIX + message), false);
    }

    private void log(String message) {
        System.out.println(LOG_PREFIX + message);
    }

    private String formatPos(BlockPos pos) {
        return "(" + pos.getX() + ", " + pos.getY() + ", " + pos.getZ() + ")";
    }

    private String format2(float value) {
        return String.format(Locale.ROOT, "%.2f", value);
    }

    private String format3(float value) {
        return String.format(Locale.ROOT, "%.3f", value);
    }

    private String format4(float value) {
        return String.format(Locale.ROOT, "%.4f", value);
    }

    private record WindowKey(RegistryKey<World> worldKey, BlockPos origin) {
    }

    private record ActiveWindow(WindowKey key, RegionRecord region, WindowState window) {
    }

    private record ThermalEnvironment(
        float directSolarFluxWm2,
        float diffuseSolarFluxWm2,
        float ambientAirTemperatureKelvin,
        float deepGroundTemperatureKelvin,
        float skyTemperatureKelvin,
        float precipitationTemperatureKelvin,
        float precipitationStrength,
        float sunX,
        float sunY,
        float sunZ,
        float surfaceDeltaSeconds
    ) {
    }

    private record WorldEnvironmentSnapshot(
        long timeOfDay,
        float rainGradient,
        float thunderGradient,
        int seaLevel
    ) {
    }

    private record ThermalMaterial(
        byte kind,
        float solarAbsorptivity,
        float emissivity,
        float surfaceHeatCapacityJm2K,
        float convectiveExchangeCoefficientWm2K,
        float bulkConductanceWm2K,
        float rainExchangeCoefficientWm2K,
        boolean atmosphericExchangeRequiresAirNeighbor
    ) {
        private static final ThermalMaterial ROCK = new ThermalMaterial(SURFACE_KIND_ROCK, 0.78f, 0.92f, 1.60e5f, 8.0f, 2.4f, 20.0f, false);
        private static final ThermalMaterial SOIL = new ThermalMaterial(SURFACE_KIND_SOIL, 0.88f, 0.94f, 1.35e5f, 7.0f, 1.7f, 24.0f, false);
        private static final ThermalMaterial VEGETATION = new ThermalMaterial(SURFACE_KIND_VEGETATION, 0.64f, 0.96f, 1.90e5f, 9.0f, 1.2f, 32.0f, false);
        private static final ThermalMaterial SNOW_ICE = new ThermalMaterial(SURFACE_KIND_SNOW_ICE, 0.22f, 0.98f, 2.40e5f, 6.0f, 1.0f, 18.0f, false);
        private static final ThermalMaterial WATER = new ThermalMaterial(SURFACE_KIND_WATER, 0.93f, 0.96f, 1.00e6f, 10.0f, 0.6f, 40.0f, true);
        private static final ThermalMaterial MOLTEN = new ThermalMaterial(SURFACE_KIND_MOLTEN, 0.95f, 0.95f, 3.50e5f, 14.0f, 4.0f, 18.0f, false);
    }

    private record FanSource(BlockPos pos, Direction facing, int ductLength) {
    }

    private record SolveResult(float[] flow, float maxSpeed, BlockPos origin) {
    }

    private record SolveSnapshot(
        WindowKey key,
        RegionRecord region,
        WindowState window,
        BlockPos origin,
        List<FanSource> fans,
        float speedCap,
        long generation,
        NestedBoundaryCoupler.BoundarySample boundarySample
    ) {
    }

    private record PreparedPayload(ByteBuffer nativePayload) {
    }

    private record SolveSanitizeResult(float maxSpeed, int invalidComponents) {
    }

    private record PublishedFrame(long frameId, float maxSpeed, Map<WindowKey, short[]> regionAtlases) {
    }

    private static final class RegionRecord {
        private boolean serviceActive;
        private boolean serviceReady;
        private boolean staticUploaded;
        private boolean dynamicRestoreAttempted;
        private long staticSignature = Long.MIN_VALUE;
        private WindowState attachedWindow;
        private WindowSection[] sections;
        private int lastThermalRefreshTick;
        private List<FanSource> fans = List.of();

        private WindowState attachedWindow() {
            return attachedWindow;
        }

        private void attachWindow(WindowState window) {
            attachedWindow = window;
        }

        private WindowState detachWindow() {
            WindowState window = attachedWindow;
            attachedWindow = null;
            return window;
        }

        private void ensureSectionsInitialized() {
            if (sections != null) {
                return;
            }
            sections = new WindowSection[WINDOW_SECTION_VOLUME];
        }

        private WindowSection sectionAt(int sx, int sy, int sz) {
            if (sections == null) {
                return null;
            }
            return sections[((sx * WINDOW_SECTION_COUNT + sy) * WINDOW_SECTION_COUNT + sz)];
        }

        private void setSection(int sx, int sy, int sz, WindowSection section) {
            ensureSectionsInitialized();
            sections[((sx * WINDOW_SECTION_COUNT + sy) * WINDOW_SECTION_COUNT + sz)] = section;
        }
    }

    private static final class WindowState {
        private final AtomicBoolean busy = new AtomicBoolean(false);
        private final AtomicBoolean released = new AtomicBoolean(false);
        private ByteBuffer nativePayloadBuffer;
        private SolveResult completedSolveResult;
        private float[] flowState;
        private float[] airTemperatureState;
        private float[] surfaceTemperatureState;
        private float[] solveField;
        private float[] temperatureTransferBuffer;
        private volatile boolean detached;
        private volatile boolean backendResetPending;
        private volatile boolean temperatureRestorePending = true;
        private volatile boolean seamSyncPending = true;

        private WindowState() {
        }

        private void ensureSolveFieldInitialized() {
            if (solveField != null) {
                return;
            }
            solveField = new float[GRID_SIZE * GRID_SIZE * GRID_SIZE * CHANNELS];
        }

        private void ensureDynamicStateInitialized() {
            int cellCount = GRID_SIZE * GRID_SIZE * GRID_SIZE;
            if (flowState == null || flowState.length != cellCount * RESPONSE_CHANNELS) {
                flowState = new float[cellCount * RESPONSE_CHANNELS];
            }
            if (airTemperatureState == null || airTemperatureState.length != cellCount) {
                airTemperatureState = new float[cellCount];
            }
            if (surfaceTemperatureState == null || surfaceTemperatureState.length != cellCount) {
                surfaceTemperatureState = new float[cellCount];
            }
        }

        private ByteBuffer payloadBuffer(int payloadBytes) {
            if (nativePayloadBuffer == null || nativePayloadBuffer.capacity() != payloadBytes) {
                nativePayloadBuffer = ByteBuffer.allocateDirect(payloadBytes).order(ByteOrder.LITTLE_ENDIAN);
            }
            return nativePayloadBuffer;
        }

        private float[] ensureTemperatureTransferBuffer() {
            int cellCount = GRID_SIZE * GRID_SIZE * GRID_SIZE;
            if (temperatureTransferBuffer == null || temperatureTransferBuffer.length != cellCount) {
                temperatureTransferBuffer = new float[cellCount];
            }
            return temperatureTransferBuffer;
        }

        private synchronized void publishCompletedSolveResult(SolveResult result) {
            if (detached) {
                return;
            }
            completedSolveResult = result;
        }

        private synchronized SolveResult consumeCompletedSolveResult() {
            SolveResult result = completedSolveResult;
            completedSolveResult = null;
            return result;
        }

        private synchronized void clearCompletedSolveResult() {
            completedSolveResult = null;
        }

        private void markDetached() {
            detached = true;
            clearCompletedSolveResult();
        }

        private boolean detached() {
            return detached;
        }

        private void markBackendResetPending() {
            backendResetPending = true;
            temperatureRestorePending = true;
        }

        private boolean backendResetPending() {
            return backendResetPending;
        }

        private void clearBackendResetPending() {
            backendResetPending = false;
        }

        private void markTemperatureRestorePending() {
            temperatureRestorePending = true;
        }

        private boolean temperatureRestorePending() {
            return temperatureRestorePending;
        }

        private void clearTemperatureRestorePending() {
            temperatureRestorePending = false;
        }

        private void markSeamSyncPending() {
            seamSyncPending = true;
        }

        private void clearSeamSyncPending() {
            seamSyncPending = false;
        }

        private boolean seamSyncPending() {
            return seamSyncPending;
        }

        private boolean markReleased() {
            return released.compareAndSet(false, true);
        }
    }

    private static final class WindowSection {
        private final float[] thermal = new float[SECTION_CELL_COUNT];
        private final byte[] thermalSurfaceKind = new byte[SECTION_CELL_COUNT];
        private WorldMirror.SectionSnapshot snapshot;
        private long mirrorVersion = -1L;

        private void attachSnapshot(WorldMirror.SectionSnapshot snapshot) {
            this.snapshot = snapshot;
            this.mirrorVersion = snapshot.version();
            System.arraycopy(snapshot.surfaceKind(), 0, thermalSurfaceKind, 0, SECTION_CELL_COUNT);
        }

        private float[] obstacle() {
            return snapshot.obstacle();
        }

        private float[] air() {
            return snapshot.air();
        }

        private byte[] staticSurfaceKind() {
            return snapshot.surfaceKind();
        }

        private float[] emitterPowerWatts() {
            return snapshot.emitterPowerWatts();
        }

        private byte[] openFaceMask() {
            return snapshot.openFaceMask();
        }

        private byte[] faceSkyExposure() {
            return snapshot.faceSkyExposure();
        }

        private byte[] faceDirectExposure() {
            return snapshot.faceDirectExposure();
        }
    }

    private final class SimulationCoordinator implements Runnable {
        private final AtomicBoolean running = new AtomicBoolean(true);
        private final Thread thread = new Thread(this, "aero-sim-coordinator");
        private int lastBudgetTick = Integer.MIN_VALUE;
        private int lastSynchronizedTick = Integer.MIN_VALUE;

        private SimulationCoordinator() {
            thread.setDaemon(true);
        }

        private void start() {
            thread.start();
        }

        private boolean running() {
            return running.get();
        }

        private void shutdown() {
            running.set(false);
            thread.interrupt();
            try {
                thread.join();
            } catch (InterruptedException ex) {
                Thread.currentThread().interrupt();
            }
        }

        @Override
        public void run() {
            while (running.get()) {
                if (!streamingEnabled) {
                    publishedFrame.set(null);
                    sleepQuietly(20L);
                    continue;
                }

                int observedTick = tickCounter;
                if (observedTick != lastSynchronizedTick) {
                    grantStepBudgetForObservedTicks();
                    runMesoscaleStepCycle();
                    synchronized (simulationStateLock) {
                        synchronizeActiveWindowsFromMirror();
                    }
                    lastSynchronizedTick = observedTick;
                }

                List<ActiveWindow> activeWindows;
                synchronized (simulationStateLock) {
                    activeWindows = snapshotActiveWindowsForCoordinatorLocked();
                }
                if (activeWindows.isEmpty()) {
                    publishedFrame.set(null);
                    sleepQuietly(10L);
                    continue;
                }

                synchronized (simulationStateLock) {
                    applyCompletedResults(activeWindows);
                synchronizeRegionSeams(activeWindows);
                    resetPendingBackends(activeWindows);
                }

                if (hasBusyWindow(activeWindows)) {
                    sleepQuietly(5L);
                    continue;
                }
                if (!acquireSimulationStepBudget()) {
                    sleepQuietly(5L);
                    continue;
                }

                CountDownLatch completionLatch;
                synchronized (simulationStateLock) {
                    completionLatch = scheduleSolveCycle(activeWindows);
                }
                if (completionLatch == null) {
                    simulationStepBudget.incrementAndGet();
                    sleepQuietly(5L);
                    continue;
                }

                waitForScheduledSnapshots(completionLatch);
                if (!running.get()) {
                    return;
                }

                synchronized (simulationStateLock) {
                    float maxSpeedThisCycle = applyCompletedResults(activeWindows);
                    synchronizeRegionSeams(activeWindows);
                    resetPendingBackends(activeWindows);
                    if (activeSetStillMatches(activeWindows)) {
                        publishFrame(activeWindows, maxSpeedThisCycle);
                    }
                }
            }
        }

        private boolean hasBusyWindow(List<ActiveWindow> activeWindows) {
            for (ActiveWindow active : activeWindows) {
                if (active.window().busy.get()) {
                    return true;
                }
            }
            return false;
        }

        private boolean acquireSimulationStepBudget() {
            while (true) {
                int current = simulationStepBudget.get();
                if (current <= 0) {
                    return false;
                }
                if (simulationStepBudget.compareAndSet(current, current - 1)) {
                    return true;
                }
            }
        }

        private void grantStepBudgetForObservedTicks() {
            int observedTick = tickCounter;
            if (observedTick <= lastBudgetTick) {
                return;
            }
            int delta = observedTick - Math.max(lastBudgetTick, 0);
            lastBudgetTick = observedTick;
            for (int i = 0; i < delta; i++) {
                grantSimulationStepBudget();
            }
        }

        private void sleepQuietly(long millis) {
            try {
                Thread.sleep(millis);
            } catch (InterruptedException ex) {
                Thread.currentThread().interrupt();
            }
        }

    }
}
