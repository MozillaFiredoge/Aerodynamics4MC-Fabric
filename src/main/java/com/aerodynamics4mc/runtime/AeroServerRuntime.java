package com.aerodynamics4mc.runtime;

import java.io.IOException;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import java.util.UUID;
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
import net.minecraft.entity.Entity;
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
import net.minecraft.util.math.Box;
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

    private static final int WINDOW_THERMAL_REFRESH_TICKS = 40;
    private static final int PARTICLE_FLOW_SYNC_INTERVAL_TICKS = 4;
    private static final int PARTICLE_FLOW_SAMPLE_STRIDE = 4;
    private static final int MAX_SIMULATION_STEP_BACKLOG = 2;
    private static final int CHUNK_SIZE = 16;
    private static final int WINDOW_SECTION_COUNT = GRID_SIZE / CHUNK_SIZE;
    private static final int WINDOW_SECTION_VOLUME = WINDOW_SECTION_COUNT * WINDOW_SECTION_COUNT * WINDOW_SECTION_COUNT;
    private static final int SECTION_CELL_COUNT = CHUNK_SIZE * CHUNK_SIZE * CHUNK_SIZE;
    private static final int SOLVER_WORKER_COUNT = Math.max(2, Runtime.getRuntime().availableProcessors() / 2);
    private static final float STATE_PRESSURE_MIN = -0.03f;
    private static final float STATE_PRESSURE_MAX = 0.03f;
    private static final float THERMAL_BASE_AMBIENT_AIR_TEMPERATURE_K = 288.15f;
    private static final float THERMAL_BIOME_TEMPERATURE_SCALE_K = 12.0f;
    private static final float THERMAL_ALTITUDE_LAPSE_RATE_K_PER_BLOCK = 0.0065f;
    private static final float THERMAL_DEEP_GROUND_OFFSET_K = 1.5f;
    private static final float THERMAL_SKY_TEMP_DROP_DAY_K = 10.0f;
    private static final float THERMAL_SKY_TEMP_DROP_NIGHT_K = 24.0f;
    private static final float THERMAL_PRECIP_TEMP_DROP_K = 4.0f;
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
    private static final float RUNTIME_TEMPERATURE_SCALE_KELVIN = 20.0f;
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
    private static final int STATIC_MIRROR_HIGH_PRIORITY_BUILD_BUDGET_PER_TICK = 1;
    private static final int STATIC_MIRROR_LOW_PRIORITY_BUILD_INTERVAL_TICKS = TICKS_PER_SECOND;
    private static final int STATIC_MIRROR_LOW_PRIORITY_BUILD_BUDGET = 1;
    private static final int FAN_DUCT_REFRESH_BUDGET_PER_TICK = 1;
    private static final int WORLD_DELTA_FLUSH_BATCH_SIZE = 256;
    private static final boolean ENTITY_SAMPLE_COLLECTION_ENABLED = false;
    private static final int MAIN_THREAD_PHASE_SERVICE_INIT = 0;
    private static final int MAIN_THREAD_PHASE_FOCUS = 1;
    private static final int MAIN_THREAD_PHASE_BACKGROUND_BATCH = 2;
    private static final int MAIN_THREAD_PHASE_ACTIVE_BATCH = 3;
    private static final int MAIN_THREAD_PHASE_LIVE_BUILDS = 4;
    private static final int MAIN_THREAD_PHASE_FAN_REFRESHES = 5;
    private static final int MAIN_THREAD_PHASE_COORDINATOR = 6;
    private static final int MAIN_THREAD_PHASE_FLOW_SYNC = 7;
    private static final int MAIN_THREAD_PHASE_TOTAL = 8;
    private static final String[] MAIN_THREAD_PHASE_NAMES = {
        "serviceInit",
        "focus",
        "bgBatch",
        "activeBatch",
        "liveBuilds",
        "fanRefreshes",
        "coordinator",
        "flowSync",
        "total"
    };
    private static final int CALLBACK_PHASE_CHUNK_LOAD = 0;
    private static final int CALLBACK_PHASE_CHUNK_UNLOAD = 1;
    private static final int CALLBACK_PHASE_BLOCK_ENTITY_LOAD = 2;
    private static final int CALLBACK_PHASE_BLOCK_ENTITY_UNLOAD = 3;
    private static final int CALLBACK_PHASE_WORLD_UNLOAD = 4;
    private static final int CALLBACK_PHASE_BLOCK_CHANGED = 5;
    private static final String[] CALLBACK_PHASE_NAMES = {
        "chunkLoad",
        "chunkUnload",
        "blockEntityLoad",
        "blockEntityUnload",
        "worldUnload",
        "blockChanged"
    };
    private static final int WINDOW_EDGE_STABILIZATION_LAYERS = 8;
    private static final float WINDOW_EDGE_STABILIZATION_MIN_KEEP = 0.15f;
    private static final int REGION_HALO_CELLS = CHUNK_SIZE;
    private static final int REGION_CORE_SIZE = GRID_SIZE - REGION_HALO_CELLS * 2;
    private static final int REGION_LATTICE_STRIDE = REGION_CORE_SIZE;
    private static final int CORE_SECTION_MIN = REGION_HALO_CELLS / CHUNK_SIZE;
    private static final int CORE_SECTION_MAX = CORE_SECTION_MIN + (REGION_CORE_SIZE / CHUNK_SIZE) - 1;
    private static final int CORE_SECTION_COUNT = (CORE_SECTION_MAX - CORE_SECTION_MIN + 1)
        * (CORE_SECTION_MAX - CORE_SECTION_MIN + 1)
        * (CORE_SECTION_MAX - CORE_SECTION_MIN + 1);
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
    private final Object coordinatorLifecycleLock = new Object();
    private final Object pendingWorldDeltasLock = new Object();
    private final ArrayDeque<NativeSimulationBridge.WorldDelta> pendingWorldDeltas = new ArrayDeque<>();
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
    private final AtomicReference<Map<UUID, PlayerProbe>> publishedPlayerProbes = new AtomicReference<>(Map.of());
    private final AtomicReference<Map<UUID, EntitySample>> publishedEntitySamples = new AtomicReference<>(Map.of());
    private volatile Set<WindowKey> desiredWindowKeys = Set.of();
    private volatile Map<RegistryKey<World>, WorldEnvironmentSnapshot> worldEnvironmentSnapshots = Map.of();
    private volatile List<PlayerProbeRequest> activePlayerProbeRequests = List.of();
    private volatile List<EntitySampleRequest> activeEntitySampleRequests = List.of();
    private volatile ActiveRegionBatch pendingActiveRegionBatch;
    private volatile BackgroundRefreshBatch pendingBackgroundRefreshBatch;

    private boolean streamingEnabled = false;
    private int tickCounter = 0;
    private int lastWindowRefreshTick = Integer.MIN_VALUE;
    private long simulationTicks = 0L;
    private long lastObservedPublishedFrameId = 0L;
    private int lastPublishedFrameTick = Integer.MIN_VALUE;
    private int secondWindowTotalTicks = 0;
    private int secondWindowSimulationTicks = 0;
    private float simulationTicksPerSecond = 0.0f;
    private float lastMaxFlowSpeed = 0.0f;
    private volatile String lastSolverError = "";
    private volatile long simulationServiceId = 0L;
    private volatile MinecraftServer currentServer;
    private int lastSimulationFocusX = Integer.MIN_VALUE;
    private int lastSimulationFocusY = Integer.MIN_VALUE;
    private int lastSimulationFocusZ = Integer.MIN_VALUE;
    private SimulationCoordinator simulationCoordinator;
    private final long[] lastMainThreadPhaseNanos = new long[MAIN_THREAD_PHASE_NAMES.length];
    private final long[] maxMainThreadPhaseNanos = new long[MAIN_THREAD_PHASE_NAMES.length];
    private final long[] lastCallbackTotalNanos = new long[CALLBACK_PHASE_NAMES.length];
    private final long[] lastCallbackLockWaitNanos = new long[CALLBACK_PHASE_NAMES.length];
    private final long[] lastCallbackLockHeldNanos = new long[CALLBACK_PHASE_NAMES.length];
    private final long[] maxCallbackTotalNanos = new long[CALLBACK_PHASE_NAMES.length];
    private final long[] maxCallbackLockWaitNanos = new long[CALLBACK_PHASE_NAMES.length];
    private final long[] maxCallbackLockHeldNanos = new long[CALLBACK_PHASE_NAMES.length];

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

    public static PlayerProbe getPlayerProbe(UUID playerId) {
        if (playerId == null) {
            return null;
        }
        return INSTANCE.publishedPlayerProbes.get().get(playerId);
    }

    public static EntitySample getEntitySample(UUID entityId) {
        if (entityId == null) {
            return null;
        }
        return INSTANCE.publishedEntitySamples.get().get(entityId);
    }

    private void onChunkLoad(ServerWorld world, WorldChunk chunk) {
        runMainThreadCallbackProfiledUnlocked(CALLBACK_PHASE_CHUNK_LOAD, () -> {
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
        });
    }

    private void onChunkUnload(ServerWorld world, WorldChunk chunk) {
        runMainThreadCallbackProfiledUnlocked(CALLBACK_PHASE_CHUNK_UNLOAD, () -> {
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
        });
    }

    private void onBlockEntityLoad(BlockEntity blockEntity, ServerWorld world) {
        runMainThreadCallbackProfiledUnlocked(CALLBACK_PHASE_BLOCK_ENTITY_LOAD, () -> {
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
        });
    }

    private void onBlockEntityUnload(BlockEntity blockEntity, ServerWorld world) {
        runMainThreadCallbackProfiledUnlocked(CALLBACK_PHASE_BLOCK_ENTITY_UNLOAD, () -> {
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
        });
    }

    private void onWorldUnload(ServerWorld world) {
        runMainThreadCallbackProfiled(CALLBACK_PHASE_WORLD_UNLOAD, () -> {
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
        });
    }

    private void onBlockChanged(ServerWorld world, BlockPos pos, BlockState oldState, BlockState newState) {
        runMainThreadCallbackProfiledUnlocked(CALLBACK_PHASE_BLOCK_CHANGED, () -> {
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
        });
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
                            + " probes=" + publishedPlayerProbes.get().size()
                            + " entitySamples=" + publishedEntitySamples.get().size()
                            + " l0Cells=" + backgroundMetCellCount()
                            + " l1Cells=" + mesoscaleMetCellCount()
                            + " simBridge=" + simulationBridge.runtimeInfo()
                    );
                    feedback(
                        ctx.getSource(),
                        "SimDetail frameId=" + (currentFrame == null ? 0L : currentFrame.frameId())
                            + " frameAgeTicks=" + currentPublishedFrameAgeTicks()
                            + " stepBudget=" + simulationStepBudget.get()
                            + " activeSolveTasks=" + activeSolveTasks.get()
                            + " busyRegions=" + busyRegionCount()
                            + " attachedReadyRegions=" + attachedWindowCount()
                    );
                    feedback(
                        ctx.getSource(),
                        "MainThread lastTick=" + format3(nanosToMillis(lastMainThreadPhaseNanos[MAIN_THREAD_PHASE_TOTAL])) + "ms"
                            + " hot=" + hottestMainThreadPhaseSummary(lastMainThreadPhaseNanos)
                            + " maxTick=" + format3(nanosToMillis(maxMainThreadPhaseNanos[MAIN_THREAD_PHASE_TOTAL])) + "ms"
                            + " maxHot=" + hottestMainThreadPhaseSummary(maxMainThreadPhaseNanos)
                    );
                    feedback(ctx.getSource(), "MainThread breakdown=" + formatMainThreadPhaseBreakdown(lastMainThreadPhaseNanos));
                    feedback(
                        ctx.getSource(),
                        "Callbacks lastHot=" + hottestCallbackPhaseSummary(lastCallbackTotalNanos, lastCallbackLockWaitNanos, lastCallbackLockHeldNanos)
                            + " maxHot=" + hottestCallbackPhaseSummary(maxCallbackTotalNanos, maxCallbackLockWaitNanos, maxCallbackLockHeldNanos)
                    );
                    if (!lastSolverError.isEmpty()) {
                        feedback(ctx.getSource(), "Last solver error: " + lastSolverError);
                    }
                    return 1;
                }))
            .then(CommandManager.literal("stop")
                .executes(ctx -> {
                    stopStreaming(ctx.getSource().getServer(), false);
                    feedback(ctx.getSource(), "Streaming disabled");
                    broadcastState(ctx.getSource().getServer());
                    return 1;
                }))
        );
    }

    private void onServerTick(MinecraftServer server) {
        long tickStartNanos = System.nanoTime();
        currentServer = server;
        if (!streamingEnabled) {
            return;
        }
        if (server.isPaused()) {
            return;
        }
        tickCounter++;
        long phaseStartNanos = System.nanoTime();
        ensureSimulationServiceInitialized();
        recordMainThreadPhase(MAIN_THREAD_PHASE_SERVICE_INIT, System.nanoTime() - phaseStartNanos);
        phaseStartNanos = System.nanoTime();
        updateSimulationFocus(server);
        recordMainThreadPhase(MAIN_THREAD_PHASE_FOCUS, System.nanoTime() - phaseStartNanos);
        if (tickCounter == 1 || tickCounter % BACKGROUND_MET_REFRESH_TICKS == 0) {
            phaseStartNanos = System.nanoTime();
            pendingBackgroundRefreshBatch = captureBackgroundRefreshBatch(server);
            recordMainThreadPhase(MAIN_THREAD_PHASE_BACKGROUND_BATCH, System.nanoTime() - phaseStartNanos);
        } else {
            recordMainThreadPhase(MAIN_THREAD_PHASE_BACKGROUND_BATCH, 0L);
        }
        phaseStartNanos = System.nanoTime();
        pendingActiveRegionBatch = captureActiveRegionBatch(server);
        recordMainThreadPhase(MAIN_THREAD_PHASE_ACTIVE_BATCH, System.nanoTime() - phaseStartNanos);
        int lowPriorityBuildBudget = tickCounter % STATIC_MIRROR_LOW_PRIORITY_BUILD_INTERVAL_TICKS == 0
            ? STATIC_MIRROR_LOW_PRIORITY_BUILD_BUDGET
            : 0;
        phaseStartNanos = System.nanoTime();
        worldMirror.drainLiveBuilds(
            server,
            STATIC_MIRROR_HIGH_PRIORITY_BUILD_BUDGET_PER_TICK,
            lowPriorityBuildBudget,
            this::populateMirrorSectionSnapshot
        );
        recordMainThreadPhase(MAIN_THREAD_PHASE_LIVE_BUILDS, System.nanoTime() - phaseStartNanos);
        phaseStartNanos = System.nanoTime();
        worldMirror.drainFanRefreshes(server, FAN_DUCT_REFRESH_BUDGET_PER_TICK);
        recordMainThreadPhase(MAIN_THREAD_PHASE_FAN_REFRESHES, System.nanoTime() - phaseStartNanos);
        phaseStartNanos = System.nanoTime();
        ensureSimulationCoordinatorRunning();
        recordMainThreadPhase(MAIN_THREAD_PHASE_COORDINATOR, System.nanoTime() - phaseStartNanos);

        PublishedFrame frame = publishedFrame.get();
        if (frame == null || frame.regionAtlases().isEmpty()) {
            updateSimulationRate(0);
            lastMaxFlowSpeed = 0.0f;
            recordMainThreadPhase(MAIN_THREAD_PHASE_FLOW_SYNC, 0L);
            recordMainThreadPhase(MAIN_THREAD_PHASE_TOTAL, System.nanoTime() - tickStartNanos);
            return;
        }

        boolean shouldSyncFlow = tickCounter % PARTICLE_FLOW_SYNC_INTERVAL_TICKS == 0;
        if (shouldSyncFlow) {
            phaseStartNanos = System.nanoTime();
            syncPublishedFlowToPlayers(server, frame, PARTICLE_FLOW_SAMPLE_STRIDE);
            recordMainThreadPhase(MAIN_THREAD_PHASE_FLOW_SYNC, System.nanoTime() - phaseStartNanos);
        } else {
            recordMainThreadPhase(MAIN_THREAD_PHASE_FLOW_SYNC, 0L);
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
        recordMainThreadPhase(MAIN_THREAD_PHASE_TOTAL, System.nanoTime() - tickStartNanos);
    }

    private void recordMainThreadPhase(int phaseIndex, long nanos) {
        lastMainThreadPhaseNanos[phaseIndex] = nanos;
        if (nanos > maxMainThreadPhaseNanos[phaseIndex]) {
            maxMainThreadPhaseNanos[phaseIndex] = nanos;
        }
    }

    private static float nanosToMillis(long nanos) {
        return nanos / 1_000_000.0f;
    }

    private String hottestMainThreadPhaseSummary(long[] phaseNanos) {
        int hottestPhase = MAIN_THREAD_PHASE_SERVICE_INIT;
        long hottestNanos = phaseNanos[hottestPhase];
        for (int i = 1; i < MAIN_THREAD_PHASE_TOTAL; i++) {
            if (phaseNanos[i] > hottestNanos) {
                hottestNanos = phaseNanos[i];
                hottestPhase = i;
            }
        }
        return MAIN_THREAD_PHASE_NAMES[hottestPhase] + ":" + format3(nanosToMillis(hottestNanos)) + "ms";
    }

    private String formatMainThreadPhaseBreakdown(long[] phaseNanos) {
        StringBuilder builder = new StringBuilder();
        for (int i = 0; i < MAIN_THREAD_PHASE_TOTAL; i++) {
            if (i > 0) {
                builder.append(' ');
            }
            builder.append(MAIN_THREAD_PHASE_NAMES[i])
                .append('=')
                .append(format3(nanosToMillis(phaseNanos[i])))
                .append("ms");
        }
        return builder.toString();
    }

    private void runMainThreadCallbackProfiled(int phaseIndex, Runnable runnable) {
        long startNanos = System.nanoTime();
        long acquiredNanos;
        synchronized (simulationStateLock) {
            acquiredNanos = System.nanoTime();
            try {
                runnable.run();
            } finally {
                long endNanos = System.nanoTime();
                recordCallbackPhase(phaseIndex, endNanos - startNanos, acquiredNanos - startNanos, endNanos - acquiredNanos);
            }
        }
    }

    private void runMainThreadCallbackProfiledUnlocked(int phaseIndex, Runnable runnable) {
        long startNanos = System.nanoTime();
        try {
            runnable.run();
        } finally {
            long endNanos = System.nanoTime();
            recordCallbackPhase(phaseIndex, endNanos - startNanos, 0L, endNanos - startNanos);
        }
    }

    private void recordCallbackPhase(int phaseIndex, long totalNanos, long waitNanos, long heldNanos) {
        lastCallbackTotalNanos[phaseIndex] = totalNanos;
        lastCallbackLockWaitNanos[phaseIndex] = waitNanos;
        lastCallbackLockHeldNanos[phaseIndex] = heldNanos;
        if (totalNanos > maxCallbackTotalNanos[phaseIndex]) {
            maxCallbackTotalNanos[phaseIndex] = totalNanos;
        }
        if (waitNanos > maxCallbackLockWaitNanos[phaseIndex]) {
            maxCallbackLockWaitNanos[phaseIndex] = waitNanos;
        }
        if (heldNanos > maxCallbackLockHeldNanos[phaseIndex]) {
            maxCallbackLockHeldNanos[phaseIndex] = heldNanos;
        }
    }

    private String hottestCallbackPhaseSummary(long[] totalNanos, long[] waitNanos, long[] heldNanos) {
        int hottestPhase = 0;
        long hottestTotal = totalNanos[0];
        for (int i = 1; i < CALLBACK_PHASE_NAMES.length; i++) {
            if (totalNanos[i] > hottestTotal) {
                hottestTotal = totalNanos[i];
                hottestPhase = i;
            }
        }
        return CALLBACK_PHASE_NAMES[hottestPhase]
            + ":total=" + format3(nanosToMillis(totalNanos[hottestPhase])) + "ms"
            + ",wait=" + format3(nanosToMillis(waitNanos[hottestPhase])) + "ms"
            + ",held=" + format3(nanosToMillis(heldNanos[hottestPhase])) + "ms";
    }

    private void stopStreaming(MinecraftServer server, boolean persistDynamicRegions) {
        runtimeGeneration.incrementAndGet();
        streamingEnabled = false;
        stopSimulationCoordinator();
        tickCounter = 0;
        simulationStepBudget.set(0);
        simulationTicks = 0L;
        lastObservedPublishedFrameId = 0L;
        lastPublishedFrameTick = Integer.MIN_VALUE;
        secondWindowTotalTicks = 0;
        secondWindowSimulationTicks = 0;
        simulationTicksPerSecond = 0.0f;
        lastMaxFlowSpeed = 0.0f;
        lastSimulationFocusX = Integer.MIN_VALUE;
        lastSimulationFocusY = Integer.MIN_VALUE;
        lastSimulationFocusZ = Integer.MIN_VALUE;
        synchronized (simulationStateLock) {
            for (Map.Entry<WindowKey, RegionRecord> entry : regions.entrySet()) {
                RegionRecord region = entry.getValue();
                if (region.attached()) {
                    detachRegionWindow(entry.getKey(), region);
                    deactivateWindow(entry.getKey(), region, persistDynamicRegions, persistDynamicRegions);
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
        activePlayerProbeRequests = List.of();
        activeEntitySampleRequests = List.of();
        worldEnvironmentSnapshots = Map.of();
        publishedFrame.set(null);
        publishedPlayerProbes.set(Map.of());
        publishedEntitySamples.set(Map.of());
        synchronized (pendingWorldDeltasLock) {
            pendingWorldDeltas.clear();
        }
        waitForSolverIdle();
        releaseSimulationService();
    }

    private BackgroundRefreshBatch captureBackgroundRefreshBatch(MinecraftServer server) {
        Map<RegistryKey<World>, BackgroundRefreshRequest> requests = new HashMap<>();
        for (ServerWorld world : server.getWorlds()) {
            List<ServerPlayerEntity> players = world.getPlayers();
            if (players.isEmpty()) {
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
            requests.put(
                world.getRegistryKey(),
                new BackgroundRefreshRequest(
                    world,
                    new BlockPos(focusX, world.getSeaLevel(), focusZ),
                    new WorldEnvironmentSnapshot(
                        world.getTimeOfDay(),
                        world.getRainGradient(1.0f),
                        world.getThunderGradient(1.0f),
                        world.getSeaLevel()
                    )
                )
            );
        }
        return new BackgroundRefreshBatch(tickCounter, Map.copyOf(requests));
    }

    private void applyBackgroundRefreshBatch(BackgroundRefreshBatch batch) {
        Set<RegistryKey<World>> activeWorldKeys = new HashSet<>(batch.requests().keySet());
        backgroundMetGrids.keySet().removeIf(worldKey -> !activeWorldKeys.contains(worldKey));
        Iterator<Map.Entry<RegistryKey<World>, MesoscaleGrid>> mesoscaleIterator = mesoscaleMetGrids.entrySet().iterator();
        while (mesoscaleIterator.hasNext()) {
            Map.Entry<RegistryKey<World>, MesoscaleGrid> entry = mesoscaleIterator.next();
            if (activeWorldKeys.contains(entry.getKey())) {
                continue;
            }
            entry.getValue().close();
            mesoscaleIterator.remove();
        }

        for (Map.Entry<RegistryKey<World>, BackgroundRefreshRequest> entry : batch.requests().entrySet()) {
            RegistryKey<World> worldKey = entry.getKey();
            BackgroundRefreshRequest request = entry.getValue();
            BackgroundMetGrid grid = backgroundMetGrids.computeIfAbsent(
                worldKey,
                ignored -> new BackgroundMetGrid(BACKGROUND_MET_CELL_SIZE_BLOCKS, BACKGROUND_MET_RADIUS_CELLS)
            );
            grid.refresh(request.world(), request.environmentSnapshot(), request.focus(), batch.tickCounter(), SOLVER_STEP_SECONDS, seedTerrainProvider);
            MesoscaleGrid mesoscale = mesoscaleMetGrids.computeIfAbsent(
                worldKey,
                ignored -> new MesoscaleGrid(
                    MESOSCALE_MET_CELL_SIZE_BLOCKS,
                    MESOSCALE_MET_RADIUS_CELLS,
                    MESOSCALE_MET_LAYER_HEIGHT_BLOCKS,
                    MESOSCALE_MET_MAX_LAYERS,
                    MESOSCALE_STEP_SECONDS,
                    MESOSCALE_FORCING_REBUILD_TICKS
                )
            );
            mesoscale.refresh(request.world(), request.focus(), batch.tickCounter(), SOLVER_STEP_SECONDS, seedTerrainProvider, grid);
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
        if (focusX == lastSimulationFocusX && focusY == lastSimulationFocusY && focusZ == lastSimulationFocusZ) {
            return;
        }
        lastSimulationFocusX = focusX;
        lastSimulationFocusY = focusY;
        lastSimulationFocusZ = focusZ;
    }

    private void submitWorldDeltaToSimulation(NativeSimulationBridge.WorldDelta delta) {
        if (simulationServiceId == 0L || delta == null) {
            return;
        }
        synchronized (pendingWorldDeltasLock) {
            pendingWorldDeltas.addLast(delta);
        }
    }

    private void flushPendingWorldDeltas() {
        long serviceId = simulationServiceId;
        if (serviceId == 0L) {
            return;
        }
        while (true) {
            NativeSimulationBridge.WorldDelta[] batch;
            synchronized (pendingWorldDeltasLock) {
                if (pendingWorldDeltas.isEmpty()) {
                    return;
                }
                int batchSize = Math.min(WORLD_DELTA_FLUSH_BATCH_SIZE, pendingWorldDeltas.size());
                batch = new NativeSimulationBridge.WorldDelta[batchSize];
                for (int i = 0; i < batchSize; i++) {
                    batch[i] = pendingWorldDeltas.removeFirst();
                }
            }
            simulationBridge.submitWorldDeltas(serviceId, batch);
        }
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
        stopStreaming(server, true);
        synchronized (simulationStateLock) {
            worldMirror.close();
        }
        dynamicStore.close();
        currentServer = null;
    }

    private ActiveRegionBatch captureActiveRegionBatch(MinecraftServer server) {
        List<PlayerRegionAnchor> anchors = new ArrayList<>();
        List<PlayerProbeRequest> probeRequests = new ArrayList<>();
        Map<RegistryKey<World>, WorldEnvironmentSnapshot> snapshots = new HashMap<>();
        for (ServerWorld world : server.getWorlds()) {
            List<ServerPlayerEntity> players = world.getPlayers();
            if (players.isEmpty()) {
                continue;
            }
            RegistryKey<World> worldKey = world.getRegistryKey();
            snapshots.put(
                worldKey,
                new WorldEnvironmentSnapshot(
                    world.getTimeOfDay(),
                    world.getRainGradient(1.0f),
                    world.getThunderGradient(1.0f),
                    world.getSeaLevel()
                )
            );
            for (ServerPlayerEntity player : players) {
                BlockPos playerPos = player.getBlockPos();
                anchors.add(new PlayerRegionAnchor(worldKey, coreOriginForPosition(playerPos)));
                probeRequests.add(new PlayerProbeRequest(player.getUuid(), worldKey, playerPos.toImmutable()));
            }
        }
        Set<WindowKey> activeKeys = activeRegionKeys(anchors);
        List<EntitySampleRequest> entityRequests = ENTITY_SAMPLE_COLLECTION_ENABLED
            ? collectEntitySampleRequests(server, activeKeys)
            : List.of();
        return new ActiveRegionBatch(
            tickCounter,
            List.copyOf(anchors),
            List.copyOf(probeRequests),
            List.copyOf(entityRequests),
            Map.copyOf(snapshots)
        );
    }

    private List<EntitySampleRequest> collectEntitySampleRequests(MinecraftServer server, Set<WindowKey> activeKeys) {
        if (activeKeys.isEmpty()) {
            return List.of();
        }
        Map<RegistryKey<World>, List<WindowKey>> keysByWorld = new HashMap<>();
        for (WindowKey key : activeKeys) {
            keysByWorld.computeIfAbsent(key.worldKey(), ignored -> new ArrayList<>()).add(key);
        }
        Map<UUID, EntitySampleRequest> requests = new HashMap<>();
        for (ServerWorld world : server.getWorlds()) {
            List<WindowKey> worldKeys = keysByWorld.get(world.getRegistryKey());
            if (worldKeys == null || worldKeys.isEmpty()) {
                continue;
            }
            for (WindowKey key : worldKeys) {
                BlockPos origin = key.origin();
                Box regionBox = new Box(
                    origin.getX(),
                    origin.getY(),
                    origin.getZ(),
                    origin.getX() + GRID_SIZE,
                    origin.getY() + GRID_SIZE,
                    origin.getZ() + GRID_SIZE
                );
                for (Entity entity : world.getOtherEntities(
                    null,
                    regionBox,
                    candidate -> !(candidate instanceof ServerPlayerEntity)
                        && !candidate.isRemoved()
                        && !candidate.isSpectator()
                )) {
                    requests.putIfAbsent(
                        entity.getUuid(),
                        new EntitySampleRequest(
                            entity.getUuid(),
                            world.getRegistryKey(),
                            entity.getBlockPos().toImmutable()
                        )
                    );
                }
            }
        }
        return requests.isEmpty() ? List.of() : List.copyOf(requests.values());
    }

    private void synchronizeActiveWindowsFromMirror() {
        synchronizeDesiredRegions();

        for (Map.Entry<WindowKey, RegionRecord> regionEntry : regions.entrySet()) {
            WindowKey key = regionEntry.getKey();
            RegionRecord region = regionEntry.getValue();
            if (!synchronizeRegionRecordFromMirror(key, region)) {
                continue;
            }
            if (!attachOrRefreshRegionWindow(key, region)) {
                continue;
            }
            refreshRegionFansIfNeeded(key, region);
            if (!region.busy.get() && shouldRefreshWindowThermal(region)) {
                refreshRegionThermalInSimulation(key, region);
            }
            if (region.forcingDirty) {
                uploadRegionFanForcingToSimulation(key, region);
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
            RegionRecord region = entry.getValue();
            if (region.attached()) {
                deactivateWindow(entry.getKey(), region, true, false);
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
        for (int sx = CORE_SECTION_MIN; sx <= CORE_SECTION_MAX; sx++) {
            for (int sy = CORE_SECTION_MIN; sy <= CORE_SECTION_MAX; sy++) {
                for (int sz = CORE_SECTION_MIN; sz <= CORE_SECTION_MAX; sz++) {
                    BlockPos localOrigin = sectionOrigin(key.origin(), sx, sy, sz);
                    worldMirror.requestSectionBuild(server, key.worldKey(), localOrigin, true);
                }
            }
        }
        for (int sx = 0; sx < WINDOW_SECTION_COUNT; sx++) {
            for (int sy = 0; sy < WINDOW_SECTION_COUNT; sy++) {
                for (int sz = 0; sz < WINDOW_SECTION_COUNT; sz++) {
                    if (isCoreSection(sx, sy, sz)) {
                        continue;
                    }
                    BlockPos localOrigin = sectionOrigin(key.origin(), sx, sy, sz);
                    worldMirror.requestSectionBuild(server, key.worldKey(), localOrigin, false);
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

    private boolean attachOrRefreshRegionWindow(WindowKey key, RegionRecord region) {
        if (!region.attached()) {
            if (!initializeWindowFromMirror(key, region)) {
                return false;
            }
            if (!region.dynamicRestoreAttempted) {
                tryRestoreWindowDynamicRegionFromSimulation(key, region);
                region.dynamicRestoreAttempted = true;
                refreshRegionLifecycle(key, region);
                if (!region.serviceReady) {
                    return false;
                }
            }
            attachRegionWindow(region);
            return true;
        }
        refreshWindowFromMirror(key, region);
        return true;
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

    private void refreshRegionFansIfNeeded(WindowKey key, RegionRecord region) {
        if (!region.fansDirty) {
            return;
        }
        region.fans = queryFanSources(key.worldKey(), key.origin());
        region.fansDirty = false;
        region.forcingDirty = true;
    }

    private boolean initializeWindowFromMirror(WindowKey key, RegionRecord region) {
        region.ensureSectionsInitialized();
        int readyCoreSections = 0;
        for (int sx = 0; sx < WINDOW_SECTION_COUNT; sx++) {
            for (int sy = 0; sy < WINDOW_SECTION_COUNT; sy++) {
                for (int sz = 0; sz < WINDOW_SECTION_COUNT; sz++) {
                    BlockPos localOrigin = sectionOrigin(key.origin(), sx, sy, sz);
                    WorldMirror.SectionSnapshot snapshot = worldMirror.peekSection(key.worldKey(), localOrigin);
                    if (snapshot == null) {
                        requestWindowSectionIfNeeded(key, sx, sy, sz);
                        continue;
                    }
                    region.setSection(sx, sy, sz, snapshot);
                    if (isCoreSection(sx, sy, sz)) {
                        readyCoreSections++;
                    }
                }
            }
        }
        if (readyCoreSections < CORE_SECTION_COUNT) {
            return false;
        }
        region.lastThermalRefreshTick = tickCounter - WINDOW_THERMAL_REFRESH_TICKS;
        region.forcingDirty = true;
        return true;
    }

    private void refreshWindowFromMirror(WindowKey key, RegionRecord region) {
        if (region.sections == null) {
            if (!initializeWindowFromMirror(key, region)) {
                return;
            }
            return;
        }
        boolean sectionUpdated = false;
        for (int sx = 0; sx < WINDOW_SECTION_COUNT; sx++) {
            for (int sy = 0; sy < WINDOW_SECTION_COUNT; sy++) {
                for (int sz = 0; sz < WINDOW_SECTION_COUNT; sz++) {
                    BlockPos localOrigin = sectionOrigin(key.origin(), sx, sy, sz);
                    WorldMirror.SectionSnapshot snapshot = worldMirror.peekSection(key.worldKey(), localOrigin);
                    if (snapshot == null) {
                        requestWindowSectionIfNeeded(key, sx, sy, sz);
                        continue;
                    }
                    long previousVersion = region.sectionVersionAt(sx, sy, sz);
                    if (region.sectionAt(sx, sy, sz) == null) {
                        region.setSection(sx, sy, sz, snapshot);
                        sectionUpdated = true;
                        region.markBackendResetPending();
                        continue;
                    }
                    if (previousVersion == snapshot.version()) {
                        continue;
                    }
                    sectionUpdated = true;
                    region.setSection(sx, sy, sz, snapshot);
                    region.markBackendResetPending();
                    region.forcingDirty = true;
                }
            }
        }
        if (sectionUpdated) {
            region.lastThermalRefreshTick = tickCounter - WINDOW_THERMAL_REFRESH_TICKS;
        }
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

    private void synchronizeRegionSeams(List<ActiveWindow> activeWindows) {
        if (activeWindows.isEmpty()) {
            return;
        }
        Map<WindowKey, ActiveWindow> activeByKey = new HashMap<>(activeWindows.size());
        for (ActiveWindow active : activeWindows) {
            activeByKey.put(active.key(), active);
        }
        for (ActiveWindow active : activeWindows) {
            if (active.region().sections == null || active.region().busy.get()) {
                continue;
            }
            synchronizePositiveNeighbor(activeByKey, active, 1, 0, 0);
            synchronizePositiveNeighbor(activeByKey, active, 0, 1, 0);
            synchronizePositiveNeighbor(activeByKey, active, 0, 0, 1);
        }
    }

    private void synchronizePositiveNeighbor(
        Map<WindowKey, ActiveWindow> activeByKey,
        ActiveWindow active,
        int dx,
        int dy,
        int dz
    ) {
        WindowKey key = active.key();
        BlockPos neighborOrigin = key.origin().add(
            dx * REGION_LATTICE_STRIDE,
            dy * REGION_LATTICE_STRIDE,
            dz * REGION_LATTICE_STRIDE
        );
        WindowKey neighborKey = new WindowKey(key.worldKey(), neighborOrigin);
        ActiveWindow neighborActive = activeByKey.get(neighborKey);
        if (neighborActive == null
            || neighborActive.region().sections == null
            || neighborActive.region().busy.get()
            || active.region().backendResetPending()
            || neighborActive.region().backendResetPending()) {
            return;
        }
        synchronizeNeighborHalo(key, active.region(), neighborKey, neighborActive.region(), dx, dy, dz);
    }

    private boolean synchronizeNeighborHalo(
        WindowKey firstKey,
        RegionRecord first,
        WindowKey secondKey,
        RegionRecord second,
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
        return nativeHaloSynced;
    }

    private boolean shouldRefreshWindowThermal(RegionRecord region) {
        return region.sections != null
            && tickCounter - region.lastThermalRefreshTick >= WINDOW_THERMAL_REFRESH_TICKS;
    }

    private boolean ensureSimulationL2Runtime() {
        return simulationServiceId != 0L
            && simulationBridge.isLoaded()
            && simulationBridge.ensureL2Runtime(simulationServiceId, GRID_SIZE, GRID_SIZE, GRID_SIZE, CHANNELS, RESPONSE_CHANNELS);
    }

    private boolean uploadRegionFanForcingToSimulation(WindowKey key, RegionRecord region) {
        if (simulationServiceId == 0L || region.sections == null) {
            return false;
        }
        int cells = GRID_SIZE * GRID_SIZE * GRID_SIZE;
        byte[] fanMask = new byte[cells];
        float[] fanVx = new float[cells];
        float[] fanVy = new float[cells];
        float[] fanVz = new float[cells];
        for (FanSource fan : region.fans) {
            applyFanSourceToForcing(region, fanMask, fanVx, fanVy, fanVz, fan, key.origin().getX(), key.origin().getY(), key.origin().getZ());
        }
        boolean uploaded = simulationBridge.uploadRegionForcing(
            simulationServiceId,
            simulationRegionKey(key),
            GRID_SIZE,
            GRID_SIZE,
            GRID_SIZE,
            null,
            fanMask,
            fanVx,
            fanVy,
            fanVz
        );
        if (uploaded) {
            region.forcingDirty = false;
        }
        return uploaded;
    }

    private void deactivateWindow(WindowKey key, RegionRecord region, boolean persistDynamicRegion, boolean synchronousPersist) {
        region.markDetached();
        if (persistDynamicRegion) {
            persistWindowDynamicRegion(key, region, synchronousPersist);
        }
        deactivateWindowRegionInSimulation(key);
        if (!region.busy.get()) {
            releaseWindow(key, region);
        }
    }

    private void resetWindowBackend(WindowKey key, RegionRecord region) {
        if (simulationServiceId != 0L) {
            simulationBridge.releaseRegionRuntime(simulationServiceId, simulationRegionKey(key));
        }
        region.clearBackendResetPending();
    }

    private Set<WindowKey> activeRegionKeys(List<PlayerRegionAnchor> anchors) {
        Set<WindowKey> keys = new HashSet<>();
        for (PlayerRegionAnchor anchor : anchors) {
            RegistryKey<World> worldKey = anchor.worldKey();
            BlockPos baseCore = anchor.coreOrigin();
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

    private boolean isCoreSection(int sx, int sy, int sz) {
        return sx >= CORE_SECTION_MIN && sx <= CORE_SECTION_MAX
            && sy >= CORE_SECTION_MIN && sy <= CORE_SECTION_MAX
            && sz >= CORE_SECTION_MIN && sz <= CORE_SECTION_MAX;
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
        int readyCoreSections = 0;
        for (int sx = 0; sx < WINDOW_SECTION_COUNT; sx++) {
            for (int sy = 0; sy < WINDOW_SECTION_COUNT; sy++) {
                for (int sz = 0; sz < WINDOW_SECTION_COUNT; sz++) {
                    BlockPos localOrigin = sectionOrigin(key.origin(), sx, sy, sz);
                    WorldMirror.SectionSnapshot snapshot = worldMirror.peekSection(key.worldKey(), localOrigin);
                    int sectionIndex = windowSectionIndex(sx, sy, sz);
                    snapshots[sectionIndex] = snapshot;
                    if (snapshot != null) {
                        if (isCoreSection(sx, sy, sz)) {
                            readyCoreSections++;
                        }
                    } else {
                        requestWindowSectionIfNeeded(key, sx, sy, sz);
                    }
                }
            }
        }
        if (readyCoreSections < CORE_SECTION_COUNT) {
            return false;
        }
        if (!region.staticUploaded) {
            return uploadFullRegionStaticFromMirror(key, region, snapshots);
        }
        return uploadRegionStaticPatchesFromMirror(key, region, snapshots);
    }

    private boolean uploadFullRegionStaticFromMirror(
        WindowKey key,
        RegionRecord region,
        WorldMirror.SectionSnapshot[] snapshots
    ) {
        int cells = GRID_SIZE * GRID_SIZE * GRID_SIZE;
        byte[] obstacle = new byte[cells];
        byte[] surfaceKind = new byte[cells];
        short[] openFaceMask = new short[cells];
        float[] emitterPower = new float[cells];
        byte[] faceSkyExposure = new byte[cells * FACE_COUNT];
        byte[] faceDirectExposure = new byte[cells * FACE_COUNT];
        for (int sx = 0; sx < WINDOW_SECTION_COUNT; sx++) {
            for (int sy = 0; sy < WINDOW_SECTION_COUNT; sy++) {
                for (int sz = 0; sz < WINDOW_SECTION_COUNT; sz++) {
                    int sectionIndex = windowSectionIndex(sx, sy, sz);
                    WorldMirror.SectionSnapshot snapshot = snapshots[sectionIndex];
                    int baseX = sx * CHUNK_SIZE;
                    int baseY = sy * CHUNK_SIZE;
                    int baseZ = sz * CHUNK_SIZE;
                    if (snapshot == null) {
                        for (int lx = 0; lx < CHUNK_SIZE; lx++) {
                            int x = baseX + lx;
                            for (int ly = 0; ly < CHUNK_SIZE; ly++) {
                                int y = baseY + ly;
                                for (int lz = 0; lz < CHUNK_SIZE; lz++) {
                                    int z = baseZ + lz;
                                    obstacle[gridCellIndex(x, y, z)] = 1;
                                }
                            }
                        }
                        region.uploadedSectionVersions[sectionIndex] = Long.MIN_VALUE;
                        continue;
                    }
                    writeSectionSnapshotIntoRegionBuffers(
                        snapshot,
                        baseX,
                        baseY,
                        baseZ,
                        obstacle,
                        surfaceKind,
                        openFaceMask,
                        emitterPower,
                        faceSkyExposure,
                        faceDirectExposure
                    );
                    region.uploadedSectionVersions[sectionIndex] = snapshot.version();
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
            emitterPower,
            faceSkyExposure,
            faceDirectExposure
        );
        if (uploaded) {
            region.staticUploaded = true;
            region.fansDirty = true;
            region.forcingDirty = true;
        }
        return uploaded;
    }

    private boolean uploadRegionStaticPatchesFromMirror(
        WindowKey key,
        RegionRecord region,
        WorldMirror.SectionSnapshot[] snapshots
    ) {
        boolean uploadedAny = false;
        byte[] obstacle = new byte[SECTION_CELL_COUNT];
        byte[] surfaceKind = new byte[SECTION_CELL_COUNT];
        short[] openFaceMask = new short[SECTION_CELL_COUNT];
        float[] emitterPower = new float[SECTION_CELL_COUNT];
        byte[] faceSkyExposure = new byte[SECTION_CELL_COUNT * FACE_COUNT];
        byte[] faceDirectExposure = new byte[SECTION_CELL_COUNT * FACE_COUNT];
        for (int sx = 0; sx < WINDOW_SECTION_COUNT; sx++) {
            for (int sy = 0; sy < WINDOW_SECTION_COUNT; sy++) {
                for (int sz = 0; sz < WINDOW_SECTION_COUNT; sz++) {
                    int sectionIndex = windowSectionIndex(sx, sy, sz);
                    WorldMirror.SectionSnapshot snapshot = snapshots[sectionIndex];
                    if (snapshot == null) {
                        continue;
                    }
                    if (region.uploadedSectionVersions[sectionIndex] == snapshot.version()) {
                        continue;
                    }
                    writeSectionSnapshotIntoPatchBuffers(
                        snapshot,
                        obstacle,
                        surfaceKind,
                        openFaceMask,
                        emitterPower,
                        faceSkyExposure,
                        faceDirectExposure
                    );
                    boolean patched = simulationBridge.uploadStaticRegionPatch(
                        simulationServiceId,
                        simulationRegionKey(key),
                        GRID_SIZE,
                        GRID_SIZE,
                        GRID_SIZE,
                        sx * CHUNK_SIZE,
                        sy * CHUNK_SIZE,
                        sz * CHUNK_SIZE,
                        CHUNK_SIZE,
                        CHUNK_SIZE,
                        CHUNK_SIZE,
                        obstacle,
                        surfaceKind,
                        openFaceMask,
                        emitterPower,
                        faceSkyExposure,
                        faceDirectExposure
                    );
                    if (!patched) {
                        return false;
                    }
                    region.setSection(sx, sy, sz, snapshot);
                    region.uploadedSectionVersions[sectionIndex] = snapshot.version();
                    region.markBackendResetPending();
                    uploadedAny = true;
                }
            }
        }
        if (uploadedAny) {
            region.fansDirty = true;
            region.forcingDirty = true;
            region.lastThermalRefreshTick = tickCounter - WINDOW_THERMAL_REFRESH_TICKS;
        }
        return true;
    }

    private void writeSectionSnapshotIntoRegionBuffers(
        WorldMirror.SectionSnapshot snapshot,
        int baseX,
        int baseY,
        int baseZ,
        byte[] obstacle,
        byte[] surfaceKind,
        short[] openFaceMask,
        float[] emitterPower,
        byte[] faceSkyExposure,
        byte[] faceDirectExposure
    ) {
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
                    int faceBase = cell * FACE_COUNT;
                    int localFaceBase = local * FACE_COUNT;
                    System.arraycopy(snapshot.faceSkyExposure(), localFaceBase, faceSkyExposure, faceBase, FACE_COUNT);
                    System.arraycopy(snapshot.faceDirectExposure(), localFaceBase, faceDirectExposure, faceBase, FACE_COUNT);
                }
            }
        }
    }

    private void writeSectionSnapshotIntoPatchBuffers(
        WorldMirror.SectionSnapshot snapshot,
        byte[] obstacle,
        byte[] surfaceKind,
        short[] openFaceMask,
        float[] emitterPower,
        byte[] faceSkyExposure,
        byte[] faceDirectExposure
    ) {
        for (int cell = 0; cell < SECTION_CELL_COUNT; cell++) {
            obstacle[cell] = snapshot.obstacle()[cell] >= 0.5f ? (byte) 1 : (byte) 0;
            surfaceKind[cell] = snapshot.surfaceKind()[cell];
            openFaceMask[cell] = (short) Byte.toUnsignedInt(snapshot.openFaceMask()[cell]);
            emitterPower[cell] = snapshot.emitterPowerWatts()[cell];
        }
        System.arraycopy(snapshot.faceSkyExposure(), 0, faceSkyExposure, 0, SECTION_CELL_COUNT * FACE_COUNT);
        System.arraycopy(snapshot.faceDirectExposure(), 0, faceDirectExposure, 0, SECTION_CELL_COUNT * FACE_COUNT);
    }

    private void requestWindowSectionIfNeeded(WindowKey key, int sx, int sy, int sz) {
        MinecraftServer server = currentServer;
        if (server == null) {
            return;
        }
        worldMirror.requestSectionBuild(
            server,
            key.worldKey(),
            sectionOrigin(key.origin(), sx, sy, sz),
            isCoreSection(sx, sy, sz)
        );
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

    private void attachRegionWindow(RegionRecord region) {
        region.attach();
    }

    private void detachRegionWindow(WindowKey key, RegionRecord region) {
        region.detach();
    }

    private void persistWindowDynamicRegion(WindowKey key, RegionRecord region, boolean synchronousPersist) {
        if (simulationServiceId == 0L) {
            return;
        }
        float[] flowState = new float[FLOW_COUNT];
        int cells = GRID_SIZE * GRID_SIZE * GRID_SIZE;
        float[] airTemperatureState = new float[cells];
        float[] surfaceTemperatureState = new float[cells];
        if (!simulationBridge.exportDynamicRegion(
            simulationServiceId,
            simulationRegionKey(key),
            GRID_SIZE,
            GRID_SIZE,
            GRID_SIZE,
            flowState,
            airTemperatureState,
            surfaceTemperatureState
        )) {
            return;
        }
        ServerWorld world = resolveWorld(key.worldKey());
        if (world != null) {
            if (synchronousPersist) {
                dynamicStore.storeCapturedRegionSync(
                    world,
                    key.worldKey(),
                    key.origin(),
                    GRID_SIZE,
                    GRID_SIZE,
                    GRID_SIZE,
                    flowState,
                    airTemperatureState,
                    surfaceTemperatureState
                );
            } else {
                dynamicStore.storeCapturedRegion(
                    world,
                    key.worldKey(),
                    key.origin(),
                    GRID_SIZE,
                    GRID_SIZE,
                    GRID_SIZE,
                    flowState,
                    airTemperatureState,
                    surfaceTemperatureState
                );
            }
        }
    }

    private void tryRestoreWindowDynamicRegionFromSimulation(WindowKey key, RegionRecord region) {
        if (simulationServiceId == 0L || region.sections == null) {
            return;
        }
        int cells = GRID_SIZE * GRID_SIZE * GRID_SIZE;
        float[] flowState = new float[FLOW_COUNT];
        float[] airTemperatureState = new float[cells];
        float[] surfaceTemperatureState = new float[cells];
        if (!simulationBridge.exportDynamicRegion(
            simulationServiceId,
            simulationRegionKey(key),
            GRID_SIZE,
            GRID_SIZE,
            GRID_SIZE,
            flowState,
            airTemperatureState,
            surfaceTemperatureState
        )) {
            ServerWorld world = resolveWorld(key.worldKey());
            if (world == null || !dynamicStore.loadRegion(
                world,
                key.worldKey(),
                key.origin(),
                GRID_SIZE,
                GRID_SIZE,
                GRID_SIZE,
                flowState,
                airTemperatureState,
                surfaceTemperatureState
            )) {
                return;
            }
            simulationBridge.importDynamicRegion(
                simulationServiceId,
                simulationRegionKey(key),
                GRID_SIZE,
                GRID_SIZE,
                GRID_SIZE,
                flowState,
                airTemperatureState,
                surfaceTemperatureState
            );
        }
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

    private float runtimeFanSpeedMetersPerSecond() {
        return INFLOW_SPEED;
    }

    private boolean obstacleAt(RegionRecord region, int x, int y, int z) {
        if (!inBounds(x, y, z)) {
            return true;
        }
        WorldMirror.SectionSnapshot section = region.sectionAt(x / CHUNK_SIZE, y / CHUNK_SIZE, z / CHUNK_SIZE);
        if (section == null) {
            return true;
        }
        return section.obstacle()[localSectionCellIndex(x % CHUNK_SIZE, y % CHUNK_SIZE, z % CHUNK_SIZE)] > 0.5f;
    }

    private void applyFanAtVoxelToForcing(
        RegionRecord region,
        byte[] fanMask,
        float[] fanVxField,
        float[] fanVyField,
        float[] fanVzField,
        int x,
        int y,
        int z,
        float fanVx,
        float fanVy,
        float fanVz
    ) {
        if (!inBounds(x, y, z) || obstacleAt(region, x, y, z)) {
            return;
        }
        int cell = gridCellIndex(x, y, z);
        fanMask[cell] = 1;
        fanVxField[cell] += fanVx;
        fanVyField[cell] += fanVy;
        fanVzField[cell] += fanVz;
    }

    private void applyFanSourceToForcing(
        RegionRecord region,
        byte[] fanMask,
        float[] fanVxField,
        float[] fanVyField,
        float[] fanVzField,
        FanSource fan,
        int minX,
        int minY,
        int minZ
    ) {
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
                        applyFanAtVoxelToForcing(region, fanMask, fanVxField, fanVyField, fanVzField, cx, y, z, fanVx, fanVy, fanVz);
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
                        applyFanAtVoxelToForcing(region, fanMask, fanVxField, fanVyField, fanVzField, x, cy, z, fanVx, fanVy, fanVz);
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
                        applyFanAtVoxelToForcing(region, fanMask, fanVxField, fanVyField, fanVzField, x, y, cz, fanVx, fanVy, fanVz);
                    }
                }
            }
            default -> applyFanAtVoxelToForcing(region, fanMask, fanVxField, fanVyField, fanVzField, cx, cy, cz, fanVx, fanVy, fanVz);
        }
        applyDuctJetToForcing(region, fanMask, fanVxField, fanVyField, fanVzField, fan, minX, minY, minZ);
    }

    private void applyDuctJetToForcing(
        RegionRecord region,
        byte[] fanMask,
        float[] fanVxField,
        float[] fanVyField,
        float[] fanVzField,
        FanSource fan,
        int minX,
        int minY,
        int minZ
    ) {
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
            applyFanAtVoxelToForcing(
                region,
                fanMask,
                fanVxField,
                fanVyField,
                fanVzField,
                cx,
                cy,
                cz,
                baseVx * coreScale,
                baseVy * coreScale,
                baseVz * coreScale
            );

            float edgeFalloff = Math.max(0.10f, 1.0f - 0.90f * t);
            float edgeScale = coreScale * DUCT_EDGE_FACTOR * edgeFalloff;
            switch (fan.facing().getAxis()) {
                case X -> {
                    applyFanAtVoxelToForcing(region, fanMask, fanVxField, fanVyField, fanVzField, cx, cy + 1, cz, baseVx * edgeScale, baseVy * edgeScale, baseVz * edgeScale);
                    applyFanAtVoxelToForcing(region, fanMask, fanVxField, fanVyField, fanVzField, cx, cy - 1, cz, baseVx * edgeScale, baseVy * edgeScale, baseVz * edgeScale);
                    applyFanAtVoxelToForcing(region, fanMask, fanVxField, fanVyField, fanVzField, cx, cy, cz + 1, baseVx * edgeScale, baseVy * edgeScale, baseVz * edgeScale);
                    applyFanAtVoxelToForcing(region, fanMask, fanVxField, fanVyField, fanVzField, cx, cy, cz - 1, baseVx * edgeScale, baseVy * edgeScale, baseVz * edgeScale);
                }
                case Y -> {
                    applyFanAtVoxelToForcing(region, fanMask, fanVxField, fanVyField, fanVzField, cx + 1, cy, cz, baseVx * edgeScale, baseVy * edgeScale, baseVz * edgeScale);
                    applyFanAtVoxelToForcing(region, fanMask, fanVxField, fanVyField, fanVzField, cx - 1, cy, cz, baseVx * edgeScale, baseVy * edgeScale, baseVz * edgeScale);
                    applyFanAtVoxelToForcing(region, fanMask, fanVxField, fanVyField, fanVzField, cx, cy, cz + 1, baseVx * edgeScale, baseVy * edgeScale, baseVz * edgeScale);
                    applyFanAtVoxelToForcing(region, fanMask, fanVxField, fanVyField, fanVzField, cx, cy, cz - 1, baseVx * edgeScale, baseVy * edgeScale, baseVz * edgeScale);
                }
                case Z -> {
                    applyFanAtVoxelToForcing(region, fanMask, fanVxField, fanVyField, fanVzField, cx + 1, cy, cz, baseVx * edgeScale, baseVy * edgeScale, baseVz * edgeScale);
                    applyFanAtVoxelToForcing(region, fanMask, fanVxField, fanVyField, fanVzField, cx - 1, cy, cz, baseVx * edgeScale, baseVy * edgeScale, baseVz * edgeScale);
                    applyFanAtVoxelToForcing(region, fanMask, fanVxField, fanVyField, fanVzField, cx, cy + 1, cz, baseVx * edgeScale, baseVy * edgeScale, baseVz * edgeScale);
                    applyFanAtVoxelToForcing(region, fanMask, fanVxField, fanVyField, fanVzField, cx, cy - 1, cz, baseVx * edgeScale, baseVy * edgeScale, baseVz * edgeScale);
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
        float speedCap,
        long generation
    ) {
        return new SolveSnapshot(
            key,
            region,
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

    private int faceDataIndex(int cell, Direction direction) {
        return cell * FACE_COUNT + direction.ordinal();
    }

    private byte setFaceBit(byte mask, Direction direction) {
        return (byte) (mask | (1 << direction.ordinal()));
    }

    private byte quantizeUnitFloat(float value) {
        return (byte) MathHelper.clamp(Math.round(MathHelper.clamp(value, 0.0f, 1.0f) * 255.0f), 0, 255);
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

    private void refreshRegionThermalInSimulation(WindowKey key, RegionRecord region) {
        if (simulationServiceId == 0L || region.sections == null) {
            return;
        }
        float deltaSeconds = Math.max(1, tickCounter - region.lastThermalRefreshTick) * SOLVER_STEP_SECONDS;
        ThermalEnvironment environment = sampleThermalEnvironment(
            worldEnvironmentSnapshots.get(key.worldKey()),
            key.worldKey(),
            key.origin().add(GRID_SIZE / 2, GRID_SIZE / 2, GRID_SIZE / 2),
            deltaSeconds
        );
        if (simulationBridge.refreshRegionThermal(
            simulationServiceId,
            simulationRegionKey(key),
            GRID_SIZE,
            GRID_SIZE,
            GRID_SIZE,
            environment.directSolarFluxWm2(),
            environment.diffuseSolarFluxWm2(),
            environment.ambientAirTemperatureKelvin(),
            environment.deepGroundTemperatureKelvin(),
            environment.skyTemperatureKelvin(),
            environment.precipitationTemperatureKelvin(),
            environment.precipitationStrength(),
            environment.sunX(),
            environment.sunY(),
            environment.sunZ(),
            environment.surfaceDeltaSeconds()
        )) {
            region.lastThermalRefreshTick = tickCounter;
        }
    }

    private float runSolverStep(SolveSnapshot snapshot) throws IOException {
        if (!simulationBridge.isLoaded()) {
            throw new IOException("Simulation bridge not loaded: " + simulationBridge.getLoadError());
        }
        if (!ensureSimulationL2Runtime()) {
            throw new IOException("Native backend initialization failed");
        }
        float maxSpeed = simulationBridge.stepRegionStored(
            simulationServiceId,
            simulationRegionKey(snapshot.key()),
            GRID_SIZE,
            GRID_SIZE,
            GRID_SIZE,
            snapshot.boundarySample() == null ? 0.0f : snapshot.boundarySample().windX() / NATIVE_VELOCITY_SCALE,
            snapshot.boundarySample() == null ? 0.0f : snapshot.boundarySample().windY() / NATIVE_VELOCITY_SCALE,
            snapshot.boundarySample() == null ? 0.0f : snapshot.boundarySample().windZ() / NATIVE_VELOCITY_SCALE
        );
        if (!Float.isFinite(maxSpeed)) {
            String nativeError = simulationBridge.lastError();
            String runtimeInfo = simulationBridge.runtimeInfo();
            StringBuilder message = new StringBuilder("Native backend returned invalid max speed");
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
            throw new IOException(message.toString());
        }
        return maxSpeed * NATIVE_VELOCITY_SCALE;
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
        RegionRecord region = snapshot.region();
        try {
            if (snapshot.generation() != runtimeGeneration.get()) {
                return;
            }
            float maxSpeed = runSolverStep(snapshot);
            if (snapshot.generation() == runtimeGeneration.get()) {
                region.completedMaxSpeed = maxSpeed;
                lastSolverError = "";
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
            region.busy.set(false);
            if (!region.attached()) {
                releaseWindow(snapshot.key(), region);
            }
            activeSolveTasks.decrementAndGet();
            if (completionLatch != null) {
                completionLatch.countDown();
            }
        }
    }

    private void ensureSimulationCoordinatorRunning() {
        synchronized (coordinatorLifecycleLock) {
            if (simulationCoordinator != null && simulationCoordinator.running()) {
                return;
            }
            simulationCoordinator = new SimulationCoordinator();
            simulationCoordinator.start();
        }
    }

    private void stopSimulationCoordinator() {
        SimulationCoordinator coordinator;
        synchronized (coordinatorLifecycleLock) {
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
            if (!region.serviceReady || !region.attached()) {
                continue;
            }
            if (region.sections == null) {
                continue;
            }
            snapshot.add(new ActiveWindow(entry.getKey(), region));
        }
        return snapshot;
    }

    private float applyCompletedResults(List<ActiveWindow> activeWindows) {
        float maxSpeedThisCycle = 0.0f;
        for (ActiveWindow active : activeWindows) {
            float completedMaxSpeed = active.region().completedMaxSpeed;
            if (!Float.isFinite(completedMaxSpeed) || completedMaxSpeed <= 0.0f) {
                continue;
            }
            active.region().completedMaxSpeed = 0.0f;
            maxSpeedThisCycle = Math.max(maxSpeedThisCycle, completedMaxSpeed);
            lastSolverError = "";
        }
        return maxSpeedThisCycle;
    }

    private void resetPendingBackends(List<ActiveWindow> activeWindows) {
        for (ActiveWindow active : activeWindows) {
            RegionRecord region = active.region();
            if (!region.busy.get() && region.backendResetPending()) {
                resetWindowBackend(active.key(), region);
            }
        }
    }

    private CountDownLatch scheduleSolveCycle(List<ActiveWindow> activeWindows) {
        List<SolveSnapshot> scheduled = new ArrayList<>(activeWindows.size());
        for (ActiveWindow active : activeWindows) {
            RegionRecord region = active.region();
            if (!region.busy.compareAndSet(false, true)) {
                continue;
            }
            SolveSnapshot snapshot = createSolveSnapshot(
                active.key(),
                region,
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
                    || !region.attached()) {
                    return false;
                }
            }
            return true;
        }
    }

    private int attachedWindowCount() {
        int count = 0;
        for (RegionRecord region : regions.values()) {
            if (region.serviceReady && region.attached()) {
                count++;
            }
        }
        return count;
    }

    private int busyRegionCount() {
        int count = 0;
        for (RegionRecord region : regions.values()) {
            if (region.busy.get()) {
                count++;
            }
        }
        return count;
    }

    private int currentPublishedFrameAgeTicks() {
        if (lastPublishedFrameTick == Integer.MIN_VALUE) {
            return -1;
        }
        return Math.max(0, tickCounter - lastPublishedFrameTick);
    }

    private void publishFrame(List<ActiveWindow> activeWindows, float maxSpeedThisCycle) {
        Map<WindowKey, short[]> regionAtlases = snapshotPublishedAtlases(activeWindows, PARTICLE_FLOW_SAMPLE_STRIDE);
        if (regionAtlases.isEmpty()) {
            return;
        }
        long frameId = publishedFrameCounter.incrementAndGet();
        PublishedFrame frame = new PublishedFrame(frameId, maxSpeedThisCycle, Map.copyOf(regionAtlases));
        publishedFrame.set(frame);
        lastPublishedFrameTick = tickCounter;
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

    private Map<UUID, PlayerProbe> samplePlayerProbesLocked() {
        if (simulationServiceId == 0L || activePlayerProbeRequests.isEmpty()) {
            return Map.of();
        }
        Map<UUID, PlayerProbe> probes = new HashMap<>();
        float[] rawProbe = new float[NativeSimulationBridge.PLAYER_PROBE_CHANNELS];
        for (PlayerProbeRequest request : activePlayerProbeRequests) {
            SampledPoint sample = sampleRegionPointLocked(request.worldKey(), request.blockPos(), rawProbe);
            if (sample == null) {
                continue;
            }
            probes.put(
                request.playerId(),
                new PlayerProbe(
                    request.playerId(),
                    request.worldKey(),
                    request.blockPos(),
                    sample.velocityX(),
                    sample.velocityY(),
                    sample.velocityZ(),
                    sample.pressure(),
                    sample.airTemperatureKelvin(),
                    sample.surfaceTemperatureKelvin()
                )
            );
        }
        return probes.isEmpty() ? Map.of() : Map.copyOf(probes);
    }

    private Map<UUID, EntitySample> sampleEntitySamplesLocked() {
        if (!ENTITY_SAMPLE_COLLECTION_ENABLED || simulationServiceId == 0L || activeEntitySampleRequests.isEmpty()) {
            return Map.of();
        }
        Map<UUID, EntitySample> samples = new HashMap<>();
        float[] rawProbe = new float[NativeSimulationBridge.PLAYER_PROBE_CHANNELS];
        for (EntitySampleRequest request : activeEntitySampleRequests) {
            SampledPoint sample = sampleRegionPointLocked(request.worldKey(), request.blockPos(), rawProbe);
            if (sample == null) {
                continue;
            }
            samples.put(
                request.entityId(),
                new EntitySample(
                    request.entityId(),
                    request.worldKey(),
                    request.blockPos(),
                    sample.velocityX(),
                    sample.velocityY(),
                    sample.velocityZ(),
                    sample.pressure(),
                    sample.airTemperatureKelvin(),
                    sample.surfaceTemperatureKelvin()
                )
            );
        }
        return samples.isEmpty() ? Map.of() : Map.copyOf(samples);
    }

    private SampledPoint sampleRegionPointLocked(RegistryKey<World> worldKey, BlockPos probePos, float[] rawProbe) {
        WindowKey key = new WindowKey(worldKey, windowOriginFromCoreOrigin(coreOriginForPosition(probePos)));
        RegionRecord region = regions.get(key);
        if (region == null || !region.serviceReady) {
            return null;
        }
        int localX = probePos.getX() - key.origin().getX();
        int localY = probePos.getY() - key.origin().getY();
        int localZ = probePos.getZ() - key.origin().getZ();
        if (!inBounds(localX, localY, localZ)) {
            return null;
        }
        if (!simulationBridge.sampleRegionPoint(
            simulationServiceId,
            simulationRegionKey(key),
            GRID_SIZE,
            GRID_SIZE,
            GRID_SIZE,
            localX,
            localY,
            localZ,
            rawProbe
        )) {
            return null;
        }
        ThermalEnvironment environment = sampleThermalEnvironment(
            worldEnvironmentSnapshots.get(worldKey),
            worldKey,
            probePos,
            SOLVER_STEP_SECONDS
        );
        return new SampledPoint(
            rawProbe[0] * NATIVE_VELOCITY_SCALE,
            rawProbe[1] * NATIVE_VELOCITY_SCALE,
            rawProbe[2] * NATIVE_VELOCITY_SCALE,
            rawProbe[3],
            environment.ambientAirTemperatureKelvin() + rawProbe[4] * RUNTIME_TEMPERATURE_SCALE_KELVIN,
            rawProbe[5]
        );
    }

    private void releaseWindow(WindowKey key, RegionRecord region) {
        if (!region.markReleased()) {
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

    private record ActiveWindow(WindowKey key, RegionRecord region) {
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

    record WorldEnvironmentSnapshot(
        long timeOfDay,
        float rainGradient,
        float thunderGradient,
        int seaLevel
    ) {
    }

    private record BackgroundRefreshRequest(
        ServerWorld world,
        BlockPos focus,
        WorldEnvironmentSnapshot environmentSnapshot
    ) {
    }

    private record PlayerRegionAnchor(
        RegistryKey<World> worldKey,
        BlockPos coreOrigin
    ) {
    }

    private record PlayerProbeRequest(
        UUID playerId,
        RegistryKey<World> worldKey,
        BlockPos blockPos
    ) {
    }

    private record EntitySampleRequest(
        UUID entityId,
        RegistryKey<World> worldKey,
        BlockPos blockPos
    ) {
    }

    private record ActiveRegionBatch(
        int tickCounter,
        List<PlayerRegionAnchor> anchors,
        List<PlayerProbeRequest> playerProbeRequests,
        List<EntitySampleRequest> entitySampleRequests,
        Map<RegistryKey<World>, WorldEnvironmentSnapshot> environmentSnapshots
    ) {
    }

    private record BackgroundRefreshBatch(
        int tickCounter,
        Map<RegistryKey<World>, BackgroundRefreshRequest> requests
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

    public record PlayerProbe(
        UUID playerId,
        RegistryKey<World> worldKey,
        BlockPos blockPos,
        float velocityX,
        float velocityY,
        float velocityZ,
        float pressure,
        float airTemperatureKelvin,
        float surfaceTemperatureKelvin
    ) {
    }

    public record EntitySample(
        UUID entityId,
        RegistryKey<World> worldKey,
        BlockPos blockPos,
        float velocityX,
        float velocityY,
        float velocityZ,
        float pressure,
        float airTemperatureKelvin,
        float surfaceTemperatureKelvin
    ) {
    }

    private record SampledPoint(
        float velocityX,
        float velocityY,
        float velocityZ,
        float pressure,
        float airTemperatureKelvin,
        float surfaceTemperatureKelvin
    ) {
    }

    private record SolveSnapshot(
        WindowKey key,
        RegionRecord region,
        BlockPos origin,
        List<FanSource> fans,
        float speedCap,
        long generation,
        NestedBoundaryCoupler.BoundarySample boundarySample
    ) {
    }

    private record PublishedFrame(long frameId, float maxSpeed, Map<WindowKey, short[]> regionAtlases) {
    }

    private static final class RegionRecord {
        private final AtomicBoolean busy = new AtomicBoolean(false);
        private final AtomicBoolean released = new AtomicBoolean(false);
        private boolean serviceActive;
        private boolean serviceReady;
        private boolean attached;
        private boolean staticUploaded;
        private boolean dynamicRestoreAttempted;
        private boolean fansDirty = true;
        private boolean forcingDirty = true;
        private boolean backendResetPending;
        private final long[] uploadedSectionVersions = new long[WINDOW_SECTION_VOLUME];
        private WorldMirror.SectionSnapshot[] sections;
        private long[] sectionVersions;
        private int lastThermalRefreshTick;
        private float completedMaxSpeed;
        private List<FanSource> fans = List.of();

        private RegionRecord() {
            Arrays.fill(uploadedSectionVersions, Long.MIN_VALUE);
        }

        private boolean attached() {
            return attached;
        }

        private void attach() {
            attached = true;
            released.set(false);
        }

        private void detach() {
            attached = false;
        }

        private void markDetached() {
            attached = false;
        }

        private void markBackendResetPending() {
            backendResetPending = true;
        }

        private boolean backendResetPending() {
            return backendResetPending;
        }

        private void clearBackendResetPending() {
            backendResetPending = false;
        }

        private boolean markReleased() {
            return released.compareAndSet(false, true);
        }

        private void ensureSectionsInitialized() {
            if (sections != null) {
                return;
            }
            sections = new WorldMirror.SectionSnapshot[WINDOW_SECTION_VOLUME];
            sectionVersions = new long[WINDOW_SECTION_VOLUME];
            Arrays.fill(sectionVersions, Long.MIN_VALUE);
        }

        private WorldMirror.SectionSnapshot sectionAt(int sx, int sy, int sz) {
            if (sections == null) {
                return null;
            }
            return sections[((sx * WINDOW_SECTION_COUNT + sy) * WINDOW_SECTION_COUNT + sz)];
        }

        private long sectionVersionAt(int sx, int sy, int sz) {
            if (sectionVersions == null) {
                return Long.MIN_VALUE;
            }
            return sectionVersions[((sx * WINDOW_SECTION_COUNT + sy) * WINDOW_SECTION_COUNT + sz)];
        }

        private void setSection(int sx, int sy, int sz, WorldMirror.SectionSnapshot section) {
            ensureSectionsInitialized();
            int index = ((sx * WINDOW_SECTION_COUNT + sy) * WINDOW_SECTION_COUNT + sz);
            sections[index] = section;
            sectionVersions[index] = section == null ? Long.MIN_VALUE : section.version();
        }
    }

    private final class SimulationCoordinator implements Runnable {
        private final AtomicBoolean running = new AtomicBoolean(true);
        private final Thread thread = new Thread(this, "aero-sim-coordinator");
        private int lastActiveRegionBatchTick = Integer.MIN_VALUE;
        private int lastBudgetTick = Integer.MIN_VALUE;
        private int lastSynchronizedTick = Integer.MIN_VALUE;
        private int lastBackgroundRefreshTick = Integer.MIN_VALUE;

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

                flushPendingWorldDeltas();

                int observedTick = tickCounter;
                if (observedTick != lastSynchronizedTick) {
                    grantStepBudgetForObservedTicks();
                    synchronized (simulationStateLock) {
                        applyPendingActiveRegionBatchIfNeeded();
                        applyPendingBackgroundRefreshIfNeeded();
                    }
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
                    publishedPlayerProbes.set(samplePlayerProbesLocked());
                    publishedEntitySamples.set(sampleEntitySamplesLocked());
                    if (activeSetStillMatches(activeWindows)) {
                        publishFrame(activeWindows, maxSpeedThisCycle);
                    }
                }
            }
        }

        private boolean hasBusyWindow(List<ActiveWindow> activeWindows) {
            for (ActiveWindow active : activeWindows) {
                if (active.region().busy.get()) {
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

        private void applyPendingBackgroundRefreshIfNeeded() {
            BackgroundRefreshBatch batch = pendingBackgroundRefreshBatch;
            if (batch == null || batch.tickCounter() <= lastBackgroundRefreshTick) {
                return;
            }
            applyBackgroundRefreshBatch(batch);
            lastBackgroundRefreshTick = batch.tickCounter();
        }

        private void applyPendingActiveRegionBatchIfNeeded() {
            ActiveRegionBatch batch = pendingActiveRegionBatch;
            if (batch == null || batch.tickCounter() <= lastActiveRegionBatchTick) {
                return;
            }
            desiredWindowKeys = Set.copyOf(activeRegionKeys(batch.anchors()));
            activePlayerProbeRequests = batch.playerProbeRequests();
            activeEntitySampleRequests = batch.entitySampleRequests();
            worldEnvironmentSnapshots = batch.environmentSnapshots();
            lastActiveRegionBatchTick = batch.tickCounter();
        }

    }
}
