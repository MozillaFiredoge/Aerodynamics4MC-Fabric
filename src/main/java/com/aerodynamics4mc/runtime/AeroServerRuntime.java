package com.aerodynamics4mc.runtime;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import java.util.UUID;
import java.util.Collections;
import java.util.Comparator;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;

import com.aerodynamics4mc.FanBlock;
import com.aerodynamics4mc.ModBlocks;
import com.aerodynamics4mc.flow.AnalysisFlowCodec;
import com.aerodynamics4mc.net.AeroFlowPayload;
import com.aerodynamics4mc.net.AeroFlowAnalysisPayload;
import com.aerodynamics4mc.net.AeroRuntimeStatePayload;
import com.mojang.brigadier.CommandDispatcher;
import com.mojang.brigadier.arguments.IntegerArgumentType;
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
import net.minecraft.util.math.ChunkPos;
import net.minecraft.util.math.Direction;
import net.minecraft.util.math.MathHelper;
import net.minecraft.util.shape.VoxelShape;
import net.minecraft.world.LightType;
import net.minecraft.world.World;
import net.minecraft.world.chunk.WorldChunk;
import net.minecraft.util.WorldSavePath;

public final class AeroServerRuntime {
    private static final String LOG_PREFIX = "[aerodynamics4mc] ";
    private static final int GRID_SIZE = 64;
    private static final int TICKS_PER_SECOND = 20;
    private static final float BLOCK_SIZE_METERS = 1.0f;
    private static final float DOMAIN_SIZE_METERS = GRID_SIZE * BLOCK_SIZE_METERS;
    private static final float SOLVER_STEP_SECONDS = 1.0f / TICKS_PER_SECOND;
    private static final float CELL_SIZE_METERS = BLOCK_SIZE_METERS;
    private static final int CHANNELS = 11;
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
    private static final float ATLAS_VELOCITY_QUANT_RANGE = 5.6f;
    private static final float ATLAS_PRESSURE_QUANT_RANGE = 0.03f;
    private static final float ANALYSIS_FLOW_VELOCITY_TOLERANCE = 0.02f;
    private static final float ANALYSIS_FLOW_PRESSURE_TOLERANCE = 0.0005f;
    private static final int L2_CAPTURE_DEFAULT_DURATION_SECONDS = 30;
    private static final int L2_CAPTURE_DEFAULT_FPS = 2;
    private static final int L2_CAPTURE_MIN_DURATION_SECONDS = 1;
    private static final int L2_CAPTURE_MAX_DURATION_SECONDS = 600;
    private static final int L2_CAPTURE_MIN_FPS = 1;
    private static final int L2_CAPTURE_MAX_FPS = 20;
    private static final int L2_CAPTURE_MAX_PENDING_FRAMES = 4;
    private static final int INSPECTION_PATCH_DEFAULT_DOMAIN_BLOCKS = 64;
    private static final int INSPECTION_PATCH_MIN_DOMAIN_BLOCKS = 32;
    private static final int INSPECTION_PATCH_MAX_DOMAIN_BLOCKS = 128;
    private static final int INSPECTION_PATCH_DEFAULT_GRID_RESOLUTION = 128;
    private static final int INSPECTION_PATCH_MIN_GRID_RESOLUTION = 32;
    private static final int INSPECTION_PATCH_MAX_GRID_RESOLUTION = 256;
    private static final int INSPECTION_PATCH_DEFAULT_FACE_RESOLUTION = 24;
    private static final int INSPECTION_PATCH_MIN_FACE_RESOLUTION = 2;
    private static final int INSPECTION_PATCH_MAX_FACE_RESOLUTION = 24;
    private static final int INSPECTION_PATCH_DEFAULT_SOLVE_STEPS = 120;
    private static final int INSPECTION_PATCH_MIN_SOLVE_STEPS = 1;
    private static final int INSPECTION_PATCH_MAX_SOLVE_STEPS = 2000;
    private static final float INSPECTION_PATCH_BOUNDARY_VELOCITY_TOLERANCE = 0.0005f;
    private static final float INSPECTION_PATCH_BOUNDARY_TEMPERATURE_TOLERANCE = 0.01f;
    private static final float INSPECTION_PATCH_EMITTER_POWER_TOLERANCE = 0.1f;
    private static final float INSPECTION_PATCH_OUTPUT_VELOCITY_TOLERANCE = 0.001f;
    private static final float INSPECTION_PATCH_OUTPUT_PRESSURE_TOLERANCE = 0.00025f;
    private static final float INSPECTION_PATCH_OUTPUT_TEMPERATURE_TOLERANCE = 0.01f;
    private static final int INSPECTION_PATCH_SPONGE_THICKNESS_CELLS = 12;
    private static final float INSPECTION_PATCH_SPONGE_VELOCITY_RELAXATION = 0.18f;
    private static final float INSPECTION_PATCH_SPONGE_TEMPERATURE_RELAXATION = 0.08f;
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
    private static final int INSPECTION_PATCH_ALL_FACE_MASK = (1 << FACE_COUNT) - 1;
    private static final int BACKGROUND_MET_CELL_SIZE_BLOCKS = 256;
    private static final int BACKGROUND_MET_RADIUS_CELLS = 20;
    private static final int MESOSCALE_MET_CELL_SIZE_BLOCKS = 64;
    private static final int MESOSCALE_MET_RADIUS_CELLS = 16;
    private static final int MESOSCALE_MET_LAYER_HEIGHT_BLOCKS = 40;
    private static final int MESOSCALE_MET_MAX_LAYERS = Math.max(1, 320 / MESOSCALE_MET_LAYER_HEIGHT_BLOCKS);
    private static final int MESOSCALE_FORCING_REBUILD_TICKS = TICKS_PER_SECOND * 60;
    private static final float MESOSCALE_STEP_SECONDS = MESOSCALE_MET_CELL_SIZE_BLOCKS * SOLVER_STEP_SECONDS;
    private static final int MESOSCALE_REFRESH_TICKS = Math.max(1, Math.round(MESOSCALE_STEP_SECONDS / SOLVER_STEP_SECONDS));
    private static final int L2_TO_L1_FEEDBACK_STEPS = MESOSCALE_REFRESH_TICKS;
    private static final int NESTED_BOUNDARY_FACE_RESOLUTION = 6;
    private static final float NESTED_BOUNDARY_MAX_VY_RATIO = 0.60f;
    private static final int BACKGROUND_MET_REFRESH_TICKS = MESOSCALE_REFRESH_TICKS * 4;
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
    private static final int BRICK_RUNTIME_SIZE = REGION_CORE_SIZE;
    private static final int HORIZONTAL_ATTACH_MARGIN_CELLS = Math.max(4, REGION_CORE_SIZE / 4);
    private static final int VERTICAL_SOLVE_MARGIN_CELLS = Math.max(4, REGION_CORE_SIZE / 4);
    private static final int VERTICAL_ATTACH_MARGIN_CELLS = Math.max(VERTICAL_SOLVE_MARGIN_CELLS + 4, REGION_CORE_SIZE / 3);
    private static final int SHELL_WINDOW_RETENTION_TICKS = 8;
    private static final int BOUNDARY_FIELD_REFRESH_TICKS = 4;
    private static final int BRICK_DYNAMIC_SYNC_INTERVAL_TICKS = 4;
    private static final int BRICK_BOUNDARY_REFERENCE_REFRESH_INTERVAL_TICKS = 4;
    private static final String DEPRECATED_ANALYSIS_CAPTURE_MESSAGE =
        "Deprecated: analysis/capture tooling is no longer on the primary roadmap and may lag behind brick-runtime changes.";
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
    private final Map<RegistryKey<World>, WorldScaleDriver> worldScaleDrivers = new HashMap<>();
    private final Map<RegistryKey<World>, MesoscaleGrid> mesoscaleMetGrids = new HashMap<>();
    private final Map<RegistryKey<World>, ConcurrentLinkedQueue<MesoscaleGrid.NestedFeedbackBin>> pendingNestedFeedbackBins = new ConcurrentHashMap<>();
    private final Map<RegistryKey<World>, NestedFeedbackRuntimeDiagnostics> nestedFeedbackRuntimeDiagnostics = new ConcurrentHashMap<>();
    private final Set<RegistryKey<World>> brickRuntimeHintWorldKeys = new HashSet<>();
    private final Set<RegistryKey<World>> brickRuntimeKnownWorldKeys = new HashSet<>();
    private final Map<RegistryKey<World>, Map<BrickRuntimeHint, Integer>> brickBoundaryReferenceRefreshTicks = new HashMap<>();
    private final Object simulationStateLock = new Object();
    private final Object coordinatorLifecycleLock = new Object();
    private final Object pendingWorldDeltasLock = new Object();
    private final ArrayDeque<NativeSimulationBridge.WorldDelta> pendingWorldDeltas = new ArrayDeque<>();
    private final Object pendingResidentBrickStaticRefreshesLock = new Object();
    private final ArrayDeque<ChunkResidentBrickRefreshRequest> pendingResidentBrickStaticRefreshes = new ArrayDeque<>();
    private final ExecutorService diagnosticsExecutor = Executors.newSingleThreadExecutor(runnable -> {
        Thread thread = new Thread(runnable, "aero-diagnostics-writer");
        thread.setDaemon(true);
        return thread;
    });
    private final AtomicInteger simulationStepBudget = new AtomicInteger(0);
    private final AtomicLong runtimeGeneration = new AtomicLong(0L);
    private final AtomicLong publishedFrameCounter = new AtomicLong(0L);
    private final AtomicReference<PublishedFrame> publishedFrame = new AtomicReference<>(null);
    private final AtomicReference<Map<UUID, PlayerProbe>> publishedPlayerProbes = new AtomicReference<>(Map.of());
    private final AtomicReference<Map<UUID, EntitySample>> publishedEntitySamples = new AtomicReference<>(Map.of());
    private volatile Set<WindowKey> desiredWindowKeys = Set.of();
    private volatile Set<WindowKey> desiredSolveWindowKeys = Set.of();
    private volatile Set<WindowKey> anchorDesiredWindowKeys = Set.of();
    private volatile Set<WindowKey> anchorSolveWindowKeys = Set.of();
    private final Map<WindowKey, Integer> shellWindowRetainUntilTick = new HashMap<>();
    private final Set<WindowKey> mirrorOnlyPrewarmedWindowKeys = new HashSet<>();
    private final Map<WindowKey, Long> mirrorOnlyUploadedBrickStaticSignatures = new HashMap<>();
    private final Map<WindowKey, Long> uploadedBrickDynamicSeedSignatures = new HashMap<>();
    private volatile Map<RegistryKey<World>, WorldEnvironmentSnapshot> worldEnvironmentSnapshots = Map.of();
    private volatile List<PlayerProbeRequest> activePlayerProbeRequests = List.of();
    private volatile List<EntitySampleRequest> activeEntitySampleRequests = List.of();
    private volatile ActiveRegionBatch pendingActiveRegionBatch;
    private volatile BackgroundRefreshBatch pendingBackgroundRefreshBatch;
    private volatile L2CaptureSession activeL2CaptureSession;
    private volatile InspectionSolveSession activeInspectionSolveSession;

    private volatile boolean streamingEnabled = false;
    private volatile int tickCounter = 0;
    private long simulationTicks = 0L;
    private long lastObservedPublishedFrameId = 0L;
    private volatile int lastPublishedFrameTick = Integer.MIN_VALUE;
    private int secondWindowTotalTicks = 0;
    private int secondWindowSimulationTicks = 0;
    private float simulationTicksPerSecond = 0.0f;
    private float lastMaxFlowSpeed = 0.0f;
    private volatile String lastSolverError = "";
    private volatile String lastCoordinatorError = "";
    private volatile int lastCoordinatorObservedTick = Integer.MIN_VALUE;
    private volatile int lastCoordinatorActiveWindowCount = 0;
    private volatile int lastCoordinatorSolveWindowCount = 0;
    private volatile int lastCoordinatorScheduledWindowCount = 0;
    private volatile int lastCoordinatorBusyWindowCount = 0;
    private volatile int lastCoordinatorPublishTick = Integer.MIN_VALUE;
    private volatile int lastCoordinatorScheduleTick = Integer.MIN_VALUE;
    private volatile int lastCoordinatorSolveCompleteTick = Integer.MIN_VALUE;
    private volatile String lastCoordinatorState = "init";
    private volatile String lastCoordinatorNoPublishReason = "";
    private volatile float lastCoordinatorPublishedMaxSpeed = 0.0f;
    private volatile float lastCoordinatorAppliedMaxSpeed = 0.0f;
    private volatile long lastCoordinatorWaitNanos = 0L;
    private volatile long lastCoordinatorPostSolveNanos = 0L;
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
            queueResidentBrickStaticRefreshForChunk(world.getRegistryKey(), chunk.getPos());
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
            queueResidentBrickStaticRefreshForChunk(world.getRegistryKey(), chunk.getPos());
        });
    }

    private void queueResidentBrickStaticRefreshForChunk(RegistryKey<World> worldKey, ChunkPos chunkPos) {
        synchronized (pendingResidentBrickStaticRefreshesLock) {
            pendingResidentBrickStaticRefreshes.addLast(new ChunkResidentBrickRefreshRequest(worldKey, chunkPos.x, chunkPos.z));
        }
    }

    private void refreshResidentBrickStaticsForChunk(ServerWorld world, ChunkPos chunkPos) {
        long serviceId = simulationServiceId;
        if (serviceId == 0L || !simulationBridge.isLoaded()) {
            return;
        }
        long worldRuntimeKey = simulationWorldKey(world.getRegistryKey());
        int[] residentBrickCoords = simulationBridge.getBrickWorldResidentBrickCoords(serviceId, worldRuntimeKey);
        if (residentBrickCoords == null || residentBrickCoords.length == 0) {
            return;
        }
        for (int i = 0; i + 2 < residentBrickCoords.length; i += 3) {
            int brickX = residentBrickCoords[i];
            int brickY = residentBrickCoords[i + 1];
            int brickZ = residentBrickCoords[i + 2];
            if (!chunkOverlapsBrickColumn(chunkPos, brickX, brickZ)) {
                continue;
            }
            uploadResidentBrickStaticFromMirror(world.getRegistryKey(), brickX, brickY, brickZ);
        }
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
            WorldScaleDriver driver = worldScaleDrivers.remove(world.getRegistryKey());
            saveWorldScaleDriver(world, driver);
            MesoscaleGrid grid = mesoscaleMetGrids.remove(world.getRegistryKey());
            if (grid != null) {
                grid.close();
            }
            pendingNestedFeedbackBins.remove(world.getRegistryKey());
            nestedFeedbackRuntimeDiagnostics.remove(world.getRegistryKey());
            synchronized (simulationStateLock) {
                brickRuntimeHintWorldKeys.remove(world.getRegistryKey());
                brickRuntimeKnownWorldKeys.remove(world.getRegistryKey());
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
            submitBrickStaticCellPatchDeltas(world, pos);
        });
    }

    private void submitBrickStaticCellPatchDeltas(ServerWorld world, BlockPos centerPos) {
        submitBrickStaticCellPatchDelta(world, centerPos);
        for (Direction direction : CARDINAL_DIRECTIONS) {
            submitBrickStaticCellPatchDelta(world, centerPos.offset(direction));
        }
    }

    private void submitBrickStaticCellPatchDelta(ServerWorld world, BlockPos pos) {
        long serviceId = simulationServiceId;
        if (serviceId == 0L) {
            return;
        }
        NativeSimulationBridge.WorldDelta delta = buildBrickStaticCellPatchDelta(world, pos);
        if (delta != null) {
            submitWorldDeltaToSimulation(delta);
        }
    }

    private NativeSimulationBridge.WorldDelta buildBrickStaticCellPatchDelta(ServerWorld world, BlockPos pos) {
        BlockState state = world.getBlockState(pos);
        boolean solid = isSolidObstacle(world, pos, state);
        ThermalMaterial material = thermalMaterial(state);
        byte openFaceMask = sampleStaticOpenFaceMask(world, pos, state, material);
        int packedState = (solid ? 1 : 0) | (Byte.toUnsignedInt(material == null ? SURFACE_KIND_NONE : material.kind()) << 8);
        return new NativeSimulationBridge.WorldDelta(
            NativeSimulationBridge.WORLD_DELTA_BRICK_STATIC_CELL_PATCH,
            pos.getX(),
            pos.getY(),
            pos.getZ(),
            world.getRegistryKey().getValue().hashCode(),
            packedState,
            Byte.toUnsignedInt(openFaceMask),
            0,
            sampleEmitterThermalPowerWatts(state),
            0.0f,
            0.0f,
            0.0f
        );
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

    private boolean chunkOverlapsBrickColumn(ChunkPos chunkPos, int brickX, int brickZ) {
        int chunkMinX = chunkPos.getStartX();
        int chunkMinZ = chunkPos.getStartZ();
        int chunkMaxX = chunkMinX + CHUNK_SIZE;
        int chunkMaxZ = chunkMinZ + CHUNK_SIZE;
        int brickMinX = brickX * BRICK_RUNTIME_SIZE;
        int brickMinZ = brickZ * BRICK_RUNTIME_SIZE;
        int brickMaxX = brickMinX + BRICK_RUNTIME_SIZE;
        int brickMaxZ = brickMinZ + BRICK_RUNTIME_SIZE;
        return chunkMinX < brickMaxX
            && chunkMaxX > brickMinX
            && chunkMinZ < brickMaxZ
            && chunkMaxZ > brickMinZ;
    }

    private byte sampleStaticOpenFaceMask(
        ServerWorld world,
        BlockPos pos,
        BlockState state,
        ThermalMaterial material
    ) {
        float emitterPower = sampleEmitterThermalPowerWatts(state);
        if (material == null && emitterPower <= 0.0f) {
            return 0;
        }
        byte openFaceMask = 0;
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
            if (openFace) {
                openFaceMask = setFaceBit(openFaceMask, direction);
            }
        }
        return openFaceMask;
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
                    float reportedMaxFlow = currentFrame == null
                        ? lastMaxFlowSpeed
                        : Math.max(lastMaxFlowSpeed, currentFrame.maxSpeed());
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
                            + " maxFlow=" + format2(reportedMaxFlow)
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
                            + " activeSolveTasks=0"
                            + " busyRegions=0"
                            + " attachedReadyRegions=" + attachedWindowCount()
                            + " coordinatorAlive=" + isCoordinatorAlive()
                            + " coordTick=" + lastCoordinatorObservedTick
                            + " coordState=" + lastCoordinatorState
                            + " coordWindows=" + lastCoordinatorActiveWindowCount
                            + " coordSolveWindows=" + lastCoordinatorSolveWindowCount
                            + " coordScheduled=" + lastCoordinatorScheduledWindowCount
                            + " coordBusy=" + lastCoordinatorBusyWindowCount
                            + " coordPublishAge=" + currentCoordinatorPublishAgeTicks()
                            + " coordScheduleAge=" + currentCoordinatorScheduleAgeTicks()
                            + " coordSolveAge=" + currentCoordinatorSolveCompleteAgeTicks()
                            + " coordWaitMs=" + format3(nanosToMillis(lastCoordinatorWaitNanos))
                            + " coordPostMs=" + format3(nanosToMillis(lastCoordinatorPostSolveNanos))
                            + " coordAppliedMax=" + format3(lastCoordinatorAppliedMaxSpeed)
                            + " coordPublishedMax=" + format3(lastCoordinatorPublishedMaxSpeed)
                            + " coordNoPublish=" + (lastCoordinatorNoPublishReason.isEmpty() ? "-" : lastCoordinatorNoPublishReason)
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
                    if (!lastCoordinatorError.isEmpty()) {
                        feedback(ctx.getSource(), "Last coordinator error: " + lastCoordinatorError);
                    }
                    sendNestedFeedbackStatus(ctx.getSource());
                    return 1;
                }))
            .then(CommandManager.literal("stop")
                .executes(ctx -> {
                    stopStreaming(ctx.getSource().getServer(), false);
                    feedback(ctx.getSource(), "Streaming disabled");
                    broadcastState(ctx.getSource().getServer());
                    return 1;
                }))
            .then(CommandManager.literal("dumpdata")
                .executes(ctx -> dumpRuntimeData(ctx.getSource()))
                )
            .then(CommandManager.literal("dump_l1")
                .executes(ctx -> dumpMesoscaleSnapshot(ctx.getSource()))
                )
            .then(CommandManager.literal("nested_feedback")
                .executes(ctx -> nestedFeedbackStatus(ctx.getSource()))
                )
            .then(CommandManager.literal("capture_l2")
                .then(CommandManager.literal("start")
                    .executes(ctx -> startL2Capture(ctx.getSource(), L2_CAPTURE_DEFAULT_DURATION_SECONDS, L2_CAPTURE_DEFAULT_FPS))
                    .then(CommandManager.argument("duration_seconds", IntegerArgumentType.integer(L2_CAPTURE_MIN_DURATION_SECONDS, L2_CAPTURE_MAX_DURATION_SECONDS))
                        .executes(ctx -> startL2Capture(
                            ctx.getSource(),
                            IntegerArgumentType.getInteger(ctx, "duration_seconds"),
                            L2_CAPTURE_DEFAULT_FPS
                        ))
                        .then(CommandManager.argument("fps", IntegerArgumentType.integer(L2_CAPTURE_MIN_FPS, L2_CAPTURE_MAX_FPS))
                            .executes(ctx -> startL2Capture(
                                ctx.getSource(),
                                IntegerArgumentType.getInteger(ctx, "duration_seconds"),
                                IntegerArgumentType.getInteger(ctx, "fps")
                            ))
                        )
                    )
                )
                .then(CommandManager.literal("stop")
                    .executes(ctx -> stopL2Capture(ctx.getSource(), true))
                )
                .then(CommandManager.literal("status")
                    .executes(ctx -> l2CaptureStatus(ctx.getSource()))
                )
            )
            .then(CommandManager.literal("inspect_patch")
                .then(CommandManager.literal("dump")
                    .executes(ctx -> dumpInspectionPatch(
                        ctx.getSource(),
                        INSPECTION_PATCH_DEFAULT_DOMAIN_BLOCKS,
                        INSPECTION_PATCH_DEFAULT_GRID_RESOLUTION,
                        INSPECTION_PATCH_DEFAULT_FACE_RESOLUTION
                    ))
                    .then(CommandManager.argument("domain_blocks", IntegerArgumentType.integer(INSPECTION_PATCH_MIN_DOMAIN_BLOCKS, INSPECTION_PATCH_MAX_DOMAIN_BLOCKS))
                        .executes(ctx -> dumpInspectionPatch(
                            ctx.getSource(),
                            IntegerArgumentType.getInteger(ctx, "domain_blocks"),
                            defaultInspectionPatchGridResolution(IntegerArgumentType.getInteger(ctx, "domain_blocks")),
                            INSPECTION_PATCH_DEFAULT_FACE_RESOLUTION
                        ))
                        .then(CommandManager.argument("face_resolution", IntegerArgumentType.integer(INSPECTION_PATCH_MIN_FACE_RESOLUTION, INSPECTION_PATCH_MAX_FACE_RESOLUTION))
                            .executes(ctx -> dumpInspectionPatch(
                                ctx.getSource(),
                                IntegerArgumentType.getInteger(ctx, "domain_blocks"),
                                defaultInspectionPatchGridResolution(IntegerArgumentType.getInteger(ctx, "domain_blocks")),
                                IntegerArgumentType.getInteger(ctx, "face_resolution")
                            ))
                        )
                        .then(CommandManager.argument("grid_resolution", IntegerArgumentType.integer(INSPECTION_PATCH_MIN_GRID_RESOLUTION, INSPECTION_PATCH_MAX_GRID_RESOLUTION))
                            .executes(ctx -> dumpInspectionPatch(
                                ctx.getSource(),
                                IntegerArgumentType.getInteger(ctx, "domain_blocks"),
                                IntegerArgumentType.getInteger(ctx, "grid_resolution"),
                                INSPECTION_PATCH_DEFAULT_FACE_RESOLUTION
                            ))
                            .then(CommandManager.argument("face_resolution", IntegerArgumentType.integer(INSPECTION_PATCH_MIN_FACE_RESOLUTION, INSPECTION_PATCH_MAX_FACE_RESOLUTION))
                                .executes(ctx -> dumpInspectionPatch(
                                    ctx.getSource(),
                                    IntegerArgumentType.getInteger(ctx, "domain_blocks"),
                                    IntegerArgumentType.getInteger(ctx, "grid_resolution"),
                                    IntegerArgumentType.getInteger(ctx, "face_resolution")
                                ))
                            )
                        )
                    )
                )
                .then(CommandManager.literal("solve")
                    .executes(ctx -> startInspectionPatchSolve(
                        ctx.getSource(),
                        INSPECTION_PATCH_DEFAULT_DOMAIN_BLOCKS,
                        INSPECTION_PATCH_DEFAULT_GRID_RESOLUTION,
                        INSPECTION_PATCH_DEFAULT_FACE_RESOLUTION,
                        INSPECTION_PATCH_DEFAULT_SOLVE_STEPS
                    ))
                    .then(CommandManager.argument("domain_blocks", IntegerArgumentType.integer(INSPECTION_PATCH_MIN_DOMAIN_BLOCKS, INSPECTION_PATCH_MAX_DOMAIN_BLOCKS))
                        .executes(ctx -> startInspectionPatchSolve(
                            ctx.getSource(),
                            IntegerArgumentType.getInteger(ctx, "domain_blocks"),
                            defaultInspectionPatchGridResolution(IntegerArgumentType.getInteger(ctx, "domain_blocks")),
                            INSPECTION_PATCH_DEFAULT_FACE_RESOLUTION,
                            INSPECTION_PATCH_DEFAULT_SOLVE_STEPS
                        ))
                        .then(CommandManager.argument("face_resolution", IntegerArgumentType.integer(INSPECTION_PATCH_MIN_FACE_RESOLUTION, INSPECTION_PATCH_MAX_FACE_RESOLUTION))
                            .executes(ctx -> startInspectionPatchSolve(
                                ctx.getSource(),
                                IntegerArgumentType.getInteger(ctx, "domain_blocks"),
                                defaultInspectionPatchGridResolution(IntegerArgumentType.getInteger(ctx, "domain_blocks")),
                                IntegerArgumentType.getInteger(ctx, "face_resolution"),
                                INSPECTION_PATCH_DEFAULT_SOLVE_STEPS
                            ))
                            .then(CommandManager.argument("steps", IntegerArgumentType.integer(INSPECTION_PATCH_MIN_SOLVE_STEPS, INSPECTION_PATCH_MAX_SOLVE_STEPS))
                                .executes(ctx -> startInspectionPatchSolve(
                                    ctx.getSource(),
                                    IntegerArgumentType.getInteger(ctx, "domain_blocks"),
                                    defaultInspectionPatchGridResolution(IntegerArgumentType.getInteger(ctx, "domain_blocks")),
                                    IntegerArgumentType.getInteger(ctx, "face_resolution"),
                                    IntegerArgumentType.getInteger(ctx, "steps")
                                ))
                            )
                        )
                        .then(CommandManager.argument("grid_resolution", IntegerArgumentType.integer(INSPECTION_PATCH_MIN_GRID_RESOLUTION, INSPECTION_PATCH_MAX_GRID_RESOLUTION))
                            .executes(ctx -> startInspectionPatchSolve(
                                ctx.getSource(),
                                IntegerArgumentType.getInteger(ctx, "domain_blocks"),
                                IntegerArgumentType.getInteger(ctx, "grid_resolution"),
                                INSPECTION_PATCH_DEFAULT_FACE_RESOLUTION,
                                INSPECTION_PATCH_DEFAULT_SOLVE_STEPS
                            ))
                            .then(CommandManager.argument("face_resolution", IntegerArgumentType.integer(INSPECTION_PATCH_MIN_FACE_RESOLUTION, INSPECTION_PATCH_MAX_FACE_RESOLUTION))
                                .executes(ctx -> startInspectionPatchSolve(
                                    ctx.getSource(),
                                    IntegerArgumentType.getInteger(ctx, "domain_blocks"),
                                    IntegerArgumentType.getInteger(ctx, "grid_resolution"),
                                    IntegerArgumentType.getInteger(ctx, "face_resolution"),
                                    INSPECTION_PATCH_DEFAULT_SOLVE_STEPS
                                ))
                                .then(CommandManager.argument("steps", IntegerArgumentType.integer(INSPECTION_PATCH_MIN_SOLVE_STEPS, INSPECTION_PATCH_MAX_SOLVE_STEPS))
                                    .executes(ctx -> startInspectionPatchSolve(
                                        ctx.getSource(),
                                        IntegerArgumentType.getInteger(ctx, "domain_blocks"),
                                        IntegerArgumentType.getInteger(ctx, "grid_resolution"),
                                        IntegerArgumentType.getInteger(ctx, "face_resolution"),
                                        IntegerArgumentType.getInteger(ctx, "steps")
                                    ))
                                )
                            )
                        )
                    )
                )
                .then(CommandManager.literal("stop")
                    .executes(ctx -> stopInspectionPatchSolve(ctx.getSource()))
                )
                .then(CommandManager.literal("status")
                    .executes(ctx -> inspectionPatchSolveStatus(ctx.getSource()))
                )
            )
        );
    }

    private int startL2Capture(ServerCommandSource source, int durationSeconds, int fps) {
        feedback(source, DEPRECATED_ANALYSIS_CAPTURE_MESSAGE);
        if (!streamingEnabled) {
            feedback(source, "Streaming must be enabled before starting L2 capture");
            return 0;
        }
        L2CaptureSession existing = activeL2CaptureSession;
        if (existing != null && !existing.stopRequested.get()) {
            feedback(source, "L2 capture already active: " + existing.outputDir);
            return 0;
        }
        ServerWorld world = source.getWorld();
        BlockPos focus = BlockPos.ofFloored(source.getPosition());
        BlockPos anchorCoreOrigin = coreOriginForPosition(focus);
        List<L2CaptureRegionSpec> captureRegions = solveRegionKeys(List.of(new PlayerRegionAnchor(world.getRegistryKey(), anchorCoreOrigin, focus)))
            .stream()
            .sorted(Comparator
                .comparingInt((WindowKey key) -> key.origin().getY())
                .thenComparingInt(key -> key.origin().getX())
                .thenComparingInt(key -> key.origin().getZ()))
            .map(key -> new L2CaptureRegionSpec(
                key,
                key.origin().add(REGION_HALO_CELLS, REGION_HALO_CELLS, REGION_HALO_CELLS)
            ))
            .toList();
        int sampleIntervalTicks = Math.max(1, Math.round((float) TICKS_PER_SECOND / Math.max(1, fps)));
        Path outputDir = source.getServer()
            .getSavePath(WorldSavePath.ROOT)
            .resolve("aerodynamics4mc")
            .resolve("diagnostics")
            .resolve("l2_capture")
            .resolve("l2_" + storageSafeWorldId(world.getRegistryKey()) + "_tick" + tickCounter + "_" + anchorCoreOrigin.getX() + "_" + anchorCoreOrigin.getY() + "_" + anchorCoreOrigin.getZ());
        try {
            Files.createDirectories(outputDir.resolve("frames"));
        } catch (IOException e) {
            feedback(source, "Failed to create capture output dir: " + e.getMessage());
            return 0;
        }
        L2CaptureSession session = new L2CaptureSession(
            world.getRegistryKey(),
            world.getRegistryKey().getValue(),
            anchorCoreOrigin,
            captureRegions,
            outputDir,
            tickCounter,
            tickCounter + durationSeconds * TICKS_PER_SECOND,
            durationSeconds,
            fps,
            sampleIntervalTicks
        );
        activeL2CaptureSession = session;
        persistL2CaptureMetadata(session);
        feedback(
            source,
            "Started L2 capture duration=" + durationSeconds + "s fps=" + fps
                + " intervalTicks=" + sampleIntervalTicks
                + " regions=" + captureRegions.size()
                + " anchorCore=" + anchorCoreOrigin
                + " output=" + outputDir
        );
        feedback(source, "Render with: python3 eval_l2_capture.py --input \"" + outputDir + "\"");
        return 1;
    }

    private int stopL2Capture(ServerCommandSource source, boolean explicit) {
        feedback(source, DEPRECATED_ANALYSIS_CAPTURE_MESSAGE);
        L2CaptureSession session = activeL2CaptureSession;
        if (session == null) {
            feedback(source, "No L2 capture session is active");
            return 0;
        }
        session.stopRequested.set(true);
        activeL2CaptureSession = null;
        persistL2CaptureMetadata(session);
        feedback(
            source,
            (explicit ? "Stopping" : "Stopped") + " L2 capture framesWritten=" + session.framesWritten.get()
                + " dropped=" + session.framesDropped.get()
                + " failed=" + session.framesFailed.get()
                + " output=" + session.outputDir
        );
        return 1;
    }

    private int l2CaptureStatus(ServerCommandSource source) {
        feedback(source, DEPRECATED_ANALYSIS_CAPTURE_MESSAGE);
        L2CaptureSession session = activeL2CaptureSession;
        if (session == null) {
            feedback(source, "No L2 capture session is active");
            return 1;
        }
        feedback(
            source,
            "L2 capture active dim=" + session.dimensionId
                + " anchorCore=" + session.anchorCoreOrigin
                + " regions=" + session.regions.size()
                + " layout=[" + session.layoutMinX + "," + session.layoutMinY + "," + session.layoutMinZ + " -> "
                + session.layoutMaxExclusiveX + "," + session.layoutMaxExclusiveY + "," + session.layoutMaxExclusiveZ + ")"
                + " tickRange=[" + session.startTick + "," + session.endTick + "]"
                + " fps=" + session.fps
                + " intervalTicks=" + session.sampleIntervalTicks
                + " nextSampleTick=" + session.nextSampleTick
                + " inFlight=" + session.inFlightFrames.get()
                + " framesScheduled=" + session.nextFrameIndex.get()
                + " framesWritten=" + session.framesWritten.get()
                + " dropped=" + session.framesDropped.get()
                + " failed=" + session.framesFailed.get()
                + " output=" + session.outputDir
        );
        return 1;
    }

    private int dumpInspectionPatch(ServerCommandSource source, int domainBlocks, int gridResolution, int faceResolution) {
        feedback(source, DEPRECATED_ANALYSIS_CAPTURE_MESSAGE);
        InspectionPatchInput input = captureInspectionPatchInput(source, domainBlocks, gridResolution, faceResolution);
        if (input == null) {
            return 0;
        }
        try {
            persistInspectionPatchDump(input);
        } catch (IOException e) {
            feedback(source, "Failed to dump inspection patch: " + e.getMessage());
            return 0;
        }

        feedback(
            source,
            "Dumped inspection patch blocks=" + input.domainBlocks()
                + " grid=" + input.gridResolution()
                + " cellsPerBlock=" + input.cellsPerBlock()
                + " faceRes=" + input.faceResolution()
                + " origin=" + input.origin()
                + " fans=" + input.fans().size()
                + " output=" + input.outputDir()
        );
        feedback(source, "Use /aero inspect_patch solve to run a monolithic inspection solve on this patch");
        return 1;
    }

    private int startInspectionPatchSolve(ServerCommandSource source, int domainBlocks, int gridResolution, int faceResolution, int steps) {
        feedback(source, DEPRECATED_ANALYSIS_CAPTURE_MESSAGE);
        if (!streamingEnabled) {
            feedback(source, "Streaming must be enabled before solving an inspection patch");
            return 0;
        }
        if (!simulationBridge.isLoaded()) {
            feedback(source, "Native simulation bridge is not loaded: " + simulationBridge.getLoadError());
            return 0;
        }
        if (activeL2CaptureSession != null) {
            feedback(source, "Stop L2 capture before starting an inspection solve");
            return 0;
        }
        InspectionSolveSession existing = activeInspectionSolveSession;
        if (existing != null) {
            feedback(source, "Inspection solve already active: " + existing.outputDir);
            return 0;
        }
        InspectionPatchInput input = captureInspectionPatchInput(source, domainBlocks, gridResolution, faceResolution);
        if (input == null) {
            return 0;
        }
        try {
            persistInspectionPatchDump(input);
        } catch (IOException e) {
            feedback(source, "Failed to dump inspection patch input: " + e.getMessage());
            return 0;
        }
        stopSimulationCoordinator();
        waitForSolverIdle();
        InspectionSolveSession session = new InspectionSolveSession(input, steps);
        activeInspectionSolveSession = session;
        diagnosticsExecutor.execute(() -> runInspectionPatchSolve(session));
        feedback(
            source,
            "Started inspection solve blocks=" + input.domainBlocks()
                + " grid=" + input.gridResolution()
                + " cellsPerBlock=" + input.cellsPerBlock()
                + " faceRes=" + faceResolution
                + " steps=" + steps
                + " output=" + input.outputDir()
        );
        feedback(source, "Use /aero inspect_patch status to monitor progress");
        return 1;
    }

    private int stopInspectionPatchSolve(ServerCommandSource source) {
        feedback(source, DEPRECATED_ANALYSIS_CAPTURE_MESSAGE);
        InspectionSolveSession session = activeInspectionSolveSession;
        if (session == null) {
            feedback(source, "No inspection solve is active");
            return 0;
        }
        session.stopRequested.set(true);
        feedback(source, "Requested stop for inspection solve at " + session.outputDir);
        return 1;
    }

    private int inspectionPatchSolveStatus(ServerCommandSource source) {
        feedback(source, DEPRECATED_ANALYSIS_CAPTURE_MESSAGE);
        InspectionSolveSession session = activeInspectionSolveSession;
        if (session == null) {
            feedback(source, "No inspection solve is active");
            return 1;
        }
        feedback(
            source,
            "Inspection solve phase=" + session.phase.get()
                + " steps=" + session.completedSteps.get() + "/" + session.totalSteps
                + " maxSpeed=" + format3(session.maxSpeedMetersPerSecond.get()) + " m/s"
                + " output=" + session.outputDir
        );
        if (!session.lastError.get().isBlank()) {
            feedback(source, "Inspection solve error: " + session.lastError.get());
        }
        return 1;
    }

    private InspectionPatchInput captureInspectionPatchInput(ServerCommandSource source, int domainBlocks, int gridResolution, int faceResolution) {
        if ((domainBlocks % CHUNK_SIZE) != 0) {
            feedback(source, "Inspection patch domain size must be a multiple of " + CHUNK_SIZE + " blocks");
            return null;
        }
        if (!isValidInspectionPatchGridResolution(domainBlocks, gridResolution)) {
            feedback(
                source,
                "Inspection patch grid resolution must be >= domain blocks and an integer multiple of it"
                    + " (blocks=" + domainBlocks + ", grid=" + gridResolution + ")"
            );
            return null;
        }
        if (!streamingEnabled) {
            feedback(source, "Streaming must be enabled before dumping an inspection patch");
            return null;
        }
        if (!simulationBridge.isLoaded()) {
            feedback(source, "Native simulation bridge is not loaded: " + simulationBridge.getLoadError());
            return null;
        }
        ServerWorld world = source.getWorld();
        RegistryKey<World> worldKey = world.getRegistryKey();
        if (mesoscaleMetGrids.get(worldKey) == null && backgroundMetGrids.get(worldKey) == null) {
            feedback(source, "No active L0/L1 background fields are available for " + worldKey.getValue());
            return null;
        }

        BlockPos focus = BlockPos.ofFloored(source.getPosition());
        BlockPos origin = inspectionPatchOriginForFocus(world, focus, domainBlocks);
        WorldEnvironmentSnapshot environmentSnapshot = new WorldEnvironmentSnapshot(
            world.getTimeOfDay(),
            world.getRainGradient(1.0f),
            world.getThunderGradient(1.0f),
            world.getSeaLevel()
        );
        NestedBoundaryCoupler.BoundarySample fallbackBoundary = sampleNestedBoundaryAtPosition(worldKey, focus);
        BoundaryFieldData boundaryField = sampleInspectionBoundaryField(worldKey, origin, domainBlocks, faceResolution, fallbackBoundary);
        if (boundaryField == null) {
            feedback(source, "Failed to sample six-face inspection boundary field");
            return null;
        }
        ThermalEnvironment thermalEnvironment = sampleThermalEnvironment(
            environmentSnapshot,
            worldKey,
            focus,
            SOLVER_STEP_SECONDS
        );
        int cellsPerBlock = gridResolution / domainBlocks;
        InspectionPatchStaticFields staticFields = captureInspectionPatchStaticFields(world, origin, domainBlocks, gridResolution, cellsPerBlock);
        List<FanSource> fans = queryFanSources(worldKey, origin, domainBlocks);

        Path outputDir = inspectionPatchOutputDir(source.getServer(), worldKey, focus, domainBlocks, gridResolution);
        try {
            Files.createDirectories(outputDir.resolve("static"));
            Files.createDirectories(outputDir.resolve("boundary"));
        } catch (IOException e) {
            feedback(source, "Failed to create inspection patch output dir: " + e.getMessage());
            return null;
        }
        return new InspectionPatchInput(
            worldKey,
            focus,
            origin,
            domainBlocks,
            gridResolution,
            cellsPerBlock,
            faceResolution,
            outputDir,
            environmentSnapshot,
            fallbackBoundary,
            thermalEnvironment,
            boundaryField,
            staticFields,
            fans
        );
    }

    private Path inspectionPatchOutputDir(MinecraftServer server, RegistryKey<World> worldKey, BlockPos focus, int domainBlocks, int gridResolution) {
        return server.getSavePath(WorldSavePath.ROOT)
            .resolve("aerodynamics4mc")
            .resolve("diagnostics")
            .resolve("inspection_patch")
            .resolve("inspection_" + storageSafeWorldId(worldKey) + "_tick" + tickCounter + "_"
                + focus.getX() + "_" + focus.getY() + "_" + focus.getZ()
                + "_b" + domainBlocks + "_r" + gridResolution);
    }

    private void runInspectionPatchSolve(InspectionSolveSession session) {
        try {
            session.phase.set("prepare_runtime");
            long regionKey;
            synchronized (simulationStateLock) {
                ensureSimulationServiceInitialized();
                if (simulationServiceId == 0L) {
                    throw new IOException("Simulation service unavailable");
                }
                if (!simulationBridge.ensureL2Runtime(
                    simulationServiceId,
                    session.input.gridResolution(),
                    session.input.gridResolution(),
                    session.input.gridResolution(),
                    CHANNELS,
                    RESPONSE_CHANNELS
                )) {
                    throw new IOException("Failed to switch native runtime to inspection patch dimensions");
                }
                regionKey = inspectionPatchRegionKey(session.input);
                prepareInspectionPatchInSimulation(regionKey, session.input);
            }

            session.phase.set("solve");
            float maxSpeedMps = 0.0f;
            for (int step = 0; step < session.totalSteps; step++) {
                if (session.stopRequested.get()) {
                    session.phase.set("stopping");
                    break;
                }
                float latticeMaxSpeed;
                synchronized (simulationStateLock) {
                    latticeMaxSpeed = simulationBridge.stepRegionStored(
                        simulationServiceId,
                        regionKey,
                        session.input.gridResolution(),
                        session.input.gridResolution(),
                        session.input.gridResolution(),
                        session.input.fallbackBoundary().windX() / NATIVE_VELOCITY_SCALE,
                        session.input.fallbackBoundary().windY() / NATIVE_VELOCITY_SCALE,
                        session.input.fallbackBoundary().windZ() / NATIVE_VELOCITY_SCALE,
                        session.input.fallbackBoundary().ambientAirTemperatureKelvin(),
                        session.input.boundaryField().externalFaceMask(),
                        session.input.boundaryField().faceResolution(),
                        session.input.boundaryField().windX(),
                        session.input.boundaryField().windY(),
                        session.input.boundaryField().windZ(),
                        session.input.boundaryField().airTemperatureKelvin(),
                        INSPECTION_PATCH_SPONGE_THICKNESS_CELLS,
                        INSPECTION_PATCH_SPONGE_VELOCITY_RELAXATION,
                        INSPECTION_PATCH_SPONGE_TEMPERATURE_RELAXATION,
                        0,
                        null
                    );
                }
                if (!Float.isFinite(latticeMaxSpeed)) {
                    throw new IOException("Inspection solve step failed: " + simulationBridge.lastError());
                }
                maxSpeedMps = Math.max(maxSpeedMps, latticeMaxSpeed * NATIVE_VELOCITY_SCALE);
                session.completedSteps.incrementAndGet();
                session.maxSpeedMetersPerSecond.set(maxSpeedMps);
            }

            session.phase.set("export");
            InspectionPatchSolveResult result;
            synchronized (simulationStateLock) {
                result = exportInspectionPatchSolveResult(
                    regionKey,
                    session.input.domainBlocks(),
                    session.input.gridResolution(),
                    session.input.cellsPerBlock(),
                    session.completedSteps.get(),
                    maxSpeedMps
                );
            }
            persistInspectionPatchSolveResult(session.input.outputDir(), result);
            session.phase.set("done");
        } catch (Throwable t) {
            session.phase.set("failed");
            session.lastError.set(t.getClass().getSimpleName() + ": " + t.getMessage());
            log("Inspection solve failed: " + t.getMessage());
        } finally {
            synchronized (simulationStateLock) {
                restoreSimulationRuntimeAfterInspectionSolve();
            }
            session.completed.set(true);
        }
    }

    private long inspectionPatchRegionKey(InspectionPatchInput input) {
        long value = 1469598103934665603L;
        value = (value ^ input.worldKey().getValue().hashCode()) * 1099511628211L;
        value = (value ^ input.origin().getX()) * 1099511628211L;
        value = (value ^ input.origin().getY()) * 1099511628211L;
        value = (value ^ input.origin().getZ()) * 1099511628211L;
        value = (value ^ input.domainBlocks()) * 1099511628211L;
        value = (value ^ input.gridResolution()) * 1099511628211L;
        return value == 0L ? 1L : value;
    }

    private void prepareInspectionPatchInSimulation(long regionKey, InspectionPatchInput input) throws IOException {
        int size = input.gridResolution();
        int cells = size * size * size;
        if (!simulationBridge.activateRegion(simulationServiceId, regionKey, size, size, size)) {
            throw new IOException("Failed to activate inspection patch region");
        }
        if (!simulationBridge.uploadStaticRegion(
            simulationServiceId,
            regionKey,
            size,
            size,
            size,
            input.staticFields().obstacle(),
            input.staticFields().surfaceKind(),
            unsignedByteFieldToShorts(input.staticFields().openFaceMask()),
            input.staticFields().emitterPowerWatts(),
            input.staticFields().faceSkyExposure(),
            input.staticFields().faceDirectExposure()
        )) {
            throw new IOException("Failed to upload inspection patch static field");
        }
        InspectionPatchForcing forcing = buildInspectionPatchForcing(input);
        if (!simulationBridge.uploadRegionForcing(
            simulationServiceId,
            regionKey,
            size,
            size,
            size,
            forcing.thermalSource(),
            forcing.fanMask(),
            forcing.fanVx(),
            forcing.fanVy(),
            forcing.fanVz()
        )) {
            throw new IOException("Failed to upload inspection patch forcing");
        }
        InspectionPatchDynamicState initialState = buildInspectionPatchInitialState(input, cells);
        if (!simulationBridge.importDynamicRegion(
            simulationServiceId,
            regionKey,
            size,
            size,
            size,
            initialState.flowState(),
            initialState.airTemperature(),
            initialState.surfaceTemperature()
        )) {
            throw new IOException("Failed to initialize inspection patch dynamic state");
        }
        if (!simulationBridge.refreshRegionThermal(
            simulationServiceId,
            regionKey,
            size,
            size,
            size,
            input.thermalEnvironment().directSolarFluxWm2(),
            input.thermalEnvironment().diffuseSolarFluxWm2(),
            input.thermalEnvironment().ambientAirTemperatureKelvin(),
            input.thermalEnvironment().deepGroundTemperatureKelvin(),
            input.thermalEnvironment().skyTemperatureKelvin(),
            input.thermalEnvironment().precipitationTemperatureKelvin(),
            input.thermalEnvironment().precipitationStrength(),
            input.thermalEnvironment().sunX(),
            input.thermalEnvironment().sunY(),
            input.thermalEnvironment().sunZ(),
            input.thermalEnvironment().surfaceDeltaSeconds()
        )) {
            throw new IOException("Failed to initialize inspection patch thermal state");
        }
    }

    private InspectionPatchForcing buildInspectionPatchForcing(InspectionPatchInput input) {
        int size = input.gridResolution();
        int cells = size * size * size;
        byte[] fanMask = new byte[cells];
        float[] fanVx = new float[cells];
        float[] fanVy = new float[cells];
        float[] fanVz = new float[cells];
        float[] thermalSource = new float[cells];
        for (FanSource fan : input.fans()) {
            applyFanSourceToPatchForcing(
                input.staticFields().obstacle(),
                size,
                fanMask,
                fanVx,
                fanVy,
                fanVz,
                fan,
                input.cellsPerBlock(),
                input.origin().getX(),
                input.origin().getY(),
                input.origin().getZ()
            );
        }
        return new InspectionPatchForcing(thermalSource, fanMask, fanVx, fanVy, fanVz);
    }

    private InspectionPatchDynamicState buildInspectionPatchInitialState(InspectionPatchInput input, int cells) {
        float[] flowState = new float[cells * RESPONSE_CHANNELS];
        float[] airTemperature = new float[cells];
        float[] surfaceTemperature = new float[cells];
        float initialVx = input.fallbackBoundary().windX() / NATIVE_VELOCITY_SCALE;
        float initialVy = input.fallbackBoundary().windY() / NATIVE_VELOCITY_SCALE;
        float initialVz = input.fallbackBoundary().windZ() / NATIVE_VELOCITY_SCALE;
        float initialAirTemperature = input.thermalEnvironment().ambientAirTemperatureKelvin();
        float initialSurfaceTemperature = input.thermalEnvironment().deepGroundTemperatureKelvin();
        for (int cell = 0; cell < cells; cell++) {
            airTemperature[cell] = initialAirTemperature;
            surfaceTemperature[cell] = initialSurfaceTemperature;
            if (input.staticFields().obstacle()[cell] != 0) {
                continue;
            }
            int base = cell * RESPONSE_CHANNELS;
            flowState[base] = initialVx;
            flowState[base + 1] = initialVy;
            flowState[base + 2] = initialVz;
            flowState[base + 3] = 0.0f;
        }
        return new InspectionPatchDynamicState(flowState, airTemperature, surfaceTemperature);
    }

    private InspectionPatchSolveResult exportInspectionPatchSolveResult(
        long regionKey,
        int domainBlocks,
        int gridResolution,
        int cellsPerBlock,
        int completedSteps,
        float maxSpeedMps
    )
        throws IOException {
        int cells = gridResolution * gridResolution * gridResolution;
        float[] flowState = new float[cells * RESPONSE_CHANNELS];
        float[] airTemperature = new float[cells];
        float[] surfaceTemperature = new float[cells];
        if (!simulationBridge.exportDynamicRegion(
            simulationServiceId,
            regionKey,
            gridResolution,
            gridResolution,
            gridResolution,
            flowState,
            airTemperature,
            surfaceTemperature
        )) {
            throw new IOException("Failed to export inspection patch solution");
        }
        float[] vx = new float[cells];
        float[] vy = new float[cells];
        float[] vz = new float[cells];
        float[] pressure = new float[cells];
        for (int cell = 0; cell < cells; cell++) {
            int base = cell * RESPONSE_CHANNELS;
            vx[cell] = flowState[base];
            vy[cell] = flowState[base + 1];
            vz[cell] = flowState[base + 2];
            pressure[cell] = flowState[base + 3];
        }
        simulationBridge.deactivateRegion(simulationServiceId, regionKey);
        return new InspectionPatchSolveResult(
            domainBlocks,
            gridResolution,
            cellsPerBlock,
            vx,
            vy,
            vz,
            pressure,
            airTemperature,
            surfaceTemperature,
            completedSteps,
            maxSpeedMps
        );
    }

    private void persistInspectionPatchSolveResult(Path outputDir, InspectionPatchSolveResult result) throws IOException {
        Path solutionDir = outputDir.resolve("solution");
        Files.createDirectories(solutionDir);
        Path vxPath = solutionDir.resolve("velocity_x.zfp");
        Path vyPath = solutionDir.resolve("velocity_y.zfp");
        Path vzPath = solutionDir.resolve("velocity_z.zfp");
        Path pressurePath = solutionDir.resolve("pressure.zfp");
        Path airTemperaturePath = solutionDir.resolve("air_temperature.zfp");
        Path surfaceTemperaturePath = solutionDir.resolve("surface_temperature.zfp");
        int size = result.gridResolution();
        if (!writeCompressedFloatGridFile(vxPath, result.vx(), size, size, size, INSPECTION_PATCH_OUTPUT_VELOCITY_TOLERANCE)
            || !writeCompressedFloatGridFile(vyPath, result.vy(), size, size, size, INSPECTION_PATCH_OUTPUT_VELOCITY_TOLERANCE)
            || !writeCompressedFloatGridFile(vzPath, result.vz(), size, size, size, INSPECTION_PATCH_OUTPUT_VELOCITY_TOLERANCE)
            || !writeCompressedFloatGridFile(pressurePath, result.pressure(), size, size, size, INSPECTION_PATCH_OUTPUT_PRESSURE_TOLERANCE)
            || !writeCompressedFloatGridFile(airTemperaturePath, result.airTemperature(), size, size, size, INSPECTION_PATCH_OUTPUT_TEMPERATURE_TOLERANCE)
            || !writeCompressedFloatGridFile(surfaceTemperaturePath, result.surfaceTemperature(), size, size, size, INSPECTION_PATCH_OUTPUT_TEMPERATURE_TOLERANCE)) {
            throw new IOException("Failed to compress inspection patch solution fields");
        }

        StringBuilder builder = new StringBuilder(4096);
        builder.append("{\n");
        appendJsonField(builder, "format", "a4mc_inspection_patch_solution_v2", true);
        appendJsonField(builder, "domain_blocks_x", result.domainBlocks(), true);
        appendJsonField(builder, "domain_blocks_y", result.domainBlocks(), true);
        appendJsonField(builder, "domain_blocks_z", result.domainBlocks(), true);
        appendJsonField(builder, "grid_resolution_x", result.gridResolution(), true);
        appendJsonField(builder, "grid_resolution_y", result.gridResolution(), true);
        appendJsonField(builder, "grid_resolution_z", result.gridResolution(), true);
        appendJsonField(builder, "size_x", result.gridResolution(), true);
        appendJsonField(builder, "size_y", result.gridResolution(), true);
        appendJsonField(builder, "size_z", result.gridResolution(), true);
        appendJsonField(builder, "cells_per_block", result.cellsPerBlock(), true);
        appendJsonField(builder, "cell_size_blocks", 1.0f / (float) result.cellsPerBlock(), true);
        appendJsonField(builder, "completed_steps", result.completedSteps(), true);
        appendJsonField(builder, "max_speed_mps", result.maxSpeedMps(), true);
        appendJsonField(builder, "velocity_tolerance", INSPECTION_PATCH_OUTPUT_VELOCITY_TOLERANCE, true);
        appendJsonField(builder, "pressure_tolerance", INSPECTION_PATCH_OUTPUT_PRESSURE_TOLERANCE, true);
        appendJsonField(builder, "temperature_tolerance", INSPECTION_PATCH_OUTPUT_TEMPERATURE_TOLERANCE, true);
        builder.append("  \"files\": {\n");
        appendJsonField(builder, "velocity_x", outputDir.relativize(vxPath).toString(), true, 4);
        appendJsonField(builder, "velocity_y", outputDir.relativize(vyPath).toString(), true, 4);
        appendJsonField(builder, "velocity_z", outputDir.relativize(vzPath).toString(), true, 4);
        appendJsonField(builder, "pressure", outputDir.relativize(pressurePath).toString(), true, 4);
        appendJsonField(builder, "air_temperature", outputDir.relativize(airTemperaturePath).toString(), true, 4);
        appendJsonField(builder, "surface_temperature", outputDir.relativize(surfaceTemperaturePath).toString(), false, 4);
        builder.append("  }\n");
        builder.append("}\n");
        Files.writeString(solutionDir.resolve("metadata.json"), builder.toString(), StandardCharsets.UTF_8);
    }

    private void restoreSimulationRuntimeAfterInspectionSolve() {
        if (simulationServiceId == 0L) {
            return;
        }
        simulationBridge.ensureL2Runtime(simulationServiceId, GRID_SIZE, GRID_SIZE, GRID_SIZE, CHANNELS, RESPONSE_CHANNELS);
    }

    private short[] unsignedByteFieldToShorts(byte[] values) {
        short[] result = new short[values.length];
        for (int i = 0; i < values.length; i++) {
            result[i] = (short) Byte.toUnsignedInt(values[i]);
        }
        return result;
    }

    private void tickL2Capture(MinecraftServer server) {
        L2CaptureSession session = activeL2CaptureSession;
        if (session == null) {
            return;
        }
        if (session.stopRequested.get() || tickCounter > session.endTick) {
            session.stopRequested.set(true);
            activeL2CaptureSession = null;
            persistL2CaptureMetadata(session);
            return;
        }
        if (tickCounter < session.nextSampleTick) {
            return;
        }
    }

    private void captureL2Frame(L2CaptureSession session, L2CapturePendingFrame pendingFrame) {
        try {
            Path framesDir = session.outputDir.resolve("frames");
            String framePrefix = String.format(Locale.ROOT, "frame_%05d_tick%d", pendingFrame.frameIndex(), pendingFrame.tick());
            List<L2CaptureFrameRegionRecord> frameRegions = new ArrayList<>(pendingFrame.regions().size());
            for (L2CapturePendingRegionFrame regionFrame : pendingFrame.regions()) {
                byte[] vxCompressed = simulationBridge.compressFloatGrid3d(
                    regionFrame.vx(),
                    REGION_CORE_SIZE,
                    REGION_CORE_SIZE,
                    REGION_CORE_SIZE,
                    ANALYSIS_FLOW_VELOCITY_TOLERANCE
                );
                byte[] vyCompressed = simulationBridge.compressFloatGrid3d(
                    regionFrame.vy(),
                    REGION_CORE_SIZE,
                    REGION_CORE_SIZE,
                    REGION_CORE_SIZE,
                    ANALYSIS_FLOW_VELOCITY_TOLERANCE
                );
                byte[] vzCompressed = simulationBridge.compressFloatGrid3d(
                    regionFrame.vz(),
                    REGION_CORE_SIZE,
                    REGION_CORE_SIZE,
                    REGION_CORE_SIZE,
                    ANALYSIS_FLOW_VELOCITY_TOLERANCE
                );
                byte[] pressureCompressed = simulationBridge.compressFloatGrid3d(
                    regionFrame.pressure(),
                    REGION_CORE_SIZE,
                    REGION_CORE_SIZE,
                    REGION_CORE_SIZE,
                    ANALYSIS_FLOW_PRESSURE_TOLERANCE
                );
                if (vxCompressed == null || vyCompressed == null || vzCompressed == null || pressureCompressed == null) {
                    session.framesFailed.incrementAndGet();
                    return;
                }

                BlockPos origin = regionFrame.origin();
                BlockPos coreOrigin = regionFrame.coreOrigin();
                String regionSuffix = "_ox" + origin.getX() + "_oy" + origin.getY() + "_oz" + origin.getZ();
                Path vxPath = framesDir.resolve(framePrefix + regionSuffix + "_vx.zfp");
                Path vyPath = framesDir.resolve(framePrefix + regionSuffix + "_vy.zfp");
                Path vzPath = framesDir.resolve(framePrefix + regionSuffix + "_vz.zfp");
                Path pressurePath = framesDir.resolve(framePrefix + regionSuffix + "_p.zfp");
                Path obstaclePath = framesDir.resolve(framePrefix + regionSuffix + "_obstacle.mask");
                Files.write(vxPath, vxCompressed);
                Files.write(vyPath, vyCompressed);
                Files.write(vzPath, vzCompressed);
                Files.write(pressurePath, pressureCompressed);
                Files.write(obstaclePath, regionFrame.obstacle());

                frameRegions.add(new L2CaptureFrameRegionRecord(
                    origin.getX(),
                    origin.getY(),
                    origin.getZ(),
                    coreOrigin.getX(),
                    coreOrigin.getY(),
                    coreOrigin.getZ(),
                    session.outputDir.relativize(vxPath).toString(),
                    session.outputDir.relativize(vyPath).toString(),
                    session.outputDir.relativize(vzPath).toString(),
                    session.outputDir.relativize(pressurePath).toString(),
                    session.outputDir.relativize(obstaclePath).toString()
                ));
            }

            session.frames.add(new L2CaptureFrameRecord(
                pendingFrame.frameIndex(),
                pendingFrame.tick(),
                List.copyOf(frameRegions)
            ));
            session.framesWritten.incrementAndGet();
            persistL2CaptureMetadata(session);
        } catch (IOException e) {
            session.framesFailed.incrementAndGet();
        } finally {
            session.inFlightFrames.decrementAndGet();
        }
    }

    private void queueCoherentL2CaptureIfNeeded(int sampleTick) {
        L2CaptureSession session = activeL2CaptureSession;
        if (session == null || session.stopRequested.get()) {
            return;
        }
        if (sampleTick < session.nextSampleTick) {
            return;
        }
        if (session.inFlightFrames.get() >= L2_CAPTURE_MAX_PENDING_FRAMES) {
            session.framesDropped.incrementAndGet();
            session.nextSampleTick = sampleTick + session.sampleIntervalTicks;
            persistL2CaptureMetadata(session);
            return;
        }

        List<L2CapturePendingRegionFrame> pendingRegions = new ArrayList<>(session.regions.size());
        for (L2CaptureRegionSpec regionSpec : session.regions) {
            float[] fullFlowState = snapshotFullRegionFlow(regionSpec.key());
            if (fullFlowState == null) {
                session.framesFailed.incrementAndGet();
                session.nextSampleTick = sampleTick + session.sampleIntervalTicks;
                persistL2CaptureMetadata(session);
                return;
            }
            byte[] obstacle = snapshotCoreObstacleMask(regionSpec.key());
            if (obstacle == null) {
                session.framesFailed.incrementAndGet();
                session.nextSampleTick = sampleTick + session.sampleIntervalTicks;
                persistL2CaptureMetadata(session);
                return;
            }
            pendingRegions.add(new L2CapturePendingRegionFrame(
                regionSpec.key().origin(),
                regionSpec.coreOrigin(),
                extractCoreFlowChannel(fullFlowState, 0),
                extractCoreFlowChannel(fullFlowState, 1),
                extractCoreFlowChannel(fullFlowState, 2),
                extractCoreFlowChannel(fullFlowState, 3),
                obstacle
            ));
        }

        int frameIndex = session.nextFrameIndex.getAndIncrement();
        session.nextSampleTick = sampleTick + session.sampleIntervalTicks;
        session.inFlightFrames.incrementAndGet();
        L2CapturePendingFrame pendingFrame = new L2CapturePendingFrame(frameIndex, sampleTick, List.copyOf(pendingRegions));
        diagnosticsExecutor.execute(() -> captureL2Frame(session, pendingFrame));
    }

    private float[] snapshotFullRegionFlow(WindowKey key) {
        synchronized (simulationStateLock) {
            if (simulationServiceId == 0L) {
                return null;
            }
            RegionRecord region = regions.get(key);
            if (region == null || !region.serviceReady) {
                return null;
            }
            float[] fullFlowState = new float[GRID_SIZE * GRID_SIZE * GRID_SIZE * NativeSimulationBridge.FLOW_STATE_CHANNELS];
            if (!simulationBridge.getRegionFlowState(
                simulationServiceId,
                simulationRegionKey(key),
                GRID_SIZE,
                GRID_SIZE,
                GRID_SIZE,
                fullFlowState
            )) {
                return null;
            }
            return fullFlowState;
        }
    }

    private float[] extractCoreFlowChannel(float[] fullFlowState, int channel) {
        float[] values = new float[REGION_CORE_SIZE * REGION_CORE_SIZE * REGION_CORE_SIZE];
        int writeIndex = 0;
        for (int x = REGION_HALO_CELLS; x < GRID_SIZE - REGION_HALO_CELLS; x++) {
            for (int y = REGION_HALO_CELLS; y < GRID_SIZE - REGION_HALO_CELLS; y++) {
                for (int z = REGION_HALO_CELLS; z < GRID_SIZE - REGION_HALO_CELLS; z++) {
                    int cell = gridCellIndex(x, y, z);
                    values[writeIndex++] = fullFlowState[cell * NativeSimulationBridge.FLOW_STATE_CHANNELS + channel];
                }
            }
        }
        return values;
    }

    private byte[] snapshotCoreObstacleMask(WindowKey key) {
        synchronized (simulationStateLock) {
            RegionRecord region = regions.get(key);
            if (region == null) {
                return null;
            }
            byte[] values = new byte[REGION_CORE_SIZE * REGION_CORE_SIZE * REGION_CORE_SIZE];
            int writeIndex = 0;
            for (int x = REGION_HALO_CELLS; x < GRID_SIZE - REGION_HALO_CELLS; x++) {
                int sx = x / CHUNK_SIZE;
                int lx = x % CHUNK_SIZE;
                for (int y = REGION_HALO_CELLS; y < GRID_SIZE - REGION_HALO_CELLS; y++) {
                    int sy = y / CHUNK_SIZE;
                    int ly = y % CHUNK_SIZE;
                    for (int z = REGION_HALO_CELLS; z < GRID_SIZE - REGION_HALO_CELLS; z++) {
                        int sz = z / CHUNK_SIZE;
                        int lz = z % CHUNK_SIZE;
                        WorldMirror.SectionSnapshot section = region.sectionAt(sx, sy, sz);
                        values[writeIndex++] = section != null && section.obstacle()[localSectionCellIndex(lx, ly, lz)] >= 0.5f
                            ? (byte) 1
                            : (byte) 0;
                    }
                }
            }
            return values;
        }
    }

    private void persistL2CaptureMetadata(L2CaptureSession session) {
        if (session == null) {
            return;
        }
        Path metadataPath = session.outputDir.resolve("metadata.json");
        List<L2CaptureFrameRecord> framesSnapshot;
        synchronized (session.frames) {
            framesSnapshot = List.copyOf(session.frames);
        }
        StringBuilder builder = new StringBuilder(1 << 16);
        builder.append("{\n");
        appendJsonField(builder, "format", "a4mc_l2_capture_v2", true);
        appendJsonField(builder, "deprecated", true, true);
        appendJsonField(builder, "capture_mode", "active_core_bricks", true);
        appendJsonField(builder, "dimension_id", session.dimensionId.toString(), true);
        appendJsonField(builder, "anchor_core_x", session.anchorCoreOrigin.getX(), true);
        appendJsonField(builder, "anchor_core_y", session.anchorCoreOrigin.getY(), true);
        appendJsonField(builder, "anchor_core_z", session.anchorCoreOrigin.getZ(), true);
        appendJsonField(builder, "region_resolution", GRID_SIZE, true);
        appendJsonField(builder, "halo_cells", REGION_HALO_CELLS, true);
        appendJsonField(builder, "core_min", REGION_HALO_CELLS, true);
        appendJsonField(builder, "core_max_exclusive", GRID_SIZE - REGION_HALO_CELLS, true);
        appendJsonField(builder, "core_resolution", REGION_CORE_SIZE, true);
        appendJsonField(builder, "obstacle_mask_encoding", "u8_raw_0_1", true);
        appendJsonField(builder, "region_count", session.regions.size(), true);
        appendJsonField(builder, "layout_min_x", session.layoutMinX, true);
        appendJsonField(builder, "layout_min_y", session.layoutMinY, true);
        appendJsonField(builder, "layout_min_z", session.layoutMinZ, true);
        appendJsonField(builder, "layout_max_exclusive_x", session.layoutMaxExclusiveX, true);
        appendJsonField(builder, "layout_max_exclusive_y", session.layoutMaxExclusiveY, true);
        appendJsonField(builder, "layout_max_exclusive_z", session.layoutMaxExclusiveZ, true);
        appendJsonField(builder, "layout_resolution_x", session.layoutMaxExclusiveX - session.layoutMinX, true);
        appendJsonField(builder, "layout_resolution_y", session.layoutMaxExclusiveY - session.layoutMinY, true);
        appendJsonField(builder, "layout_resolution_z", session.layoutMaxExclusiveZ - session.layoutMinZ, true);
        appendJsonField(builder, "channels", NativeSimulationBridge.FLOW_STATE_CHANNELS, true);
        appendJsonField(builder, "velocity_tolerance", ANALYSIS_FLOW_VELOCITY_TOLERANCE, true);
        appendJsonField(builder, "pressure_tolerance", ANALYSIS_FLOW_PRESSURE_TOLERANCE, true);
        appendJsonField(builder, "start_tick", session.startTick, true);
        appendJsonField(builder, "end_tick", session.endTick, true);
        appendJsonField(builder, "duration_seconds", session.durationSeconds, true);
        appendJsonField(builder, "fps_requested", session.fps, true);
        appendJsonField(builder, "sample_interval_ticks", session.sampleIntervalTicks, true);
        appendJsonField(builder, "frames_written", session.framesWritten.get(), true);
        appendJsonField(builder, "frames_dropped", session.framesDropped.get(), true);
        appendJsonField(builder, "frames_failed", session.framesFailed.get(), true);
        int scheduledFrames = session.framesWritten.get() + session.framesDropped.get() + session.framesFailed.get();
        float effectiveFps = session.durationSeconds > 0
            ? (float) session.framesWritten.get() / (float) session.durationSeconds
            : 0.0f;
        float captureRatio = scheduledFrames > 0
            ? (float) session.framesWritten.get() / (float) scheduledFrames
            : 0.0f;
        appendJsonField(builder, "fps_effective", effectiveFps, true);
        appendJsonField(builder, "capture_ratio", captureRatio, true);
        appendJsonField(builder, "active", activeL2CaptureSession == session && !session.stopRequested.get(), true);
        builder.append("  \"regions\": [\n");
        for (int i = 0; i < session.regions.size(); i++) {
            L2CaptureRegionSpec region = session.regions.get(i);
            builder.append("    {\n");
            appendJsonField(builder, "origin_x", region.key().origin().getX(), true, 6);
            appendJsonField(builder, "origin_y", region.key().origin().getY(), true, 6);
            appendJsonField(builder, "origin_z", region.key().origin().getZ(), true, 6);
            appendJsonField(builder, "core_origin_x", region.coreOrigin().getX(), true, 6);
            appendJsonField(builder, "core_origin_y", region.coreOrigin().getY(), true, 6);
            appendJsonField(builder, "core_origin_z", region.coreOrigin().getZ(), false, 6);
            builder.append("    }");
            if (i + 1 < session.regions.size()) {
                builder.append(',');
            }
            builder.append('\n');
        }
        builder.append("  ],\n");
        builder.append("  \"frames\": [\n");
        for (int i = 0; i < framesSnapshot.size(); i++) {
            L2CaptureFrameRecord frame = framesSnapshot.get(i);
            builder.append("    {\n");
            appendJsonField(builder, "index", frame.index(), true, 6);
            appendJsonField(builder, "tick", frame.tick(), true, 6);
            builder.append("      \"regions\": [\n");
            for (int regionIndex = 0; regionIndex < frame.regions().size(); regionIndex++) {
                L2CaptureFrameRegionRecord region = frame.regions().get(regionIndex);
                builder.append("        {\n");
                appendJsonField(builder, "origin_x", region.originX(), true, 10);
                appendJsonField(builder, "origin_y", region.originY(), true, 10);
                appendJsonField(builder, "origin_z", region.originZ(), true, 10);
                appendJsonField(builder, "core_origin_x", region.coreOriginX(), true, 10);
                appendJsonField(builder, "core_origin_y", region.coreOriginY(), true, 10);
                appendJsonField(builder, "core_origin_z", region.coreOriginZ(), true, 10);
                appendJsonField(builder, "vx", region.vxPath(), true, 10);
                appendJsonField(builder, "vy", region.vyPath(), true, 10);
                appendJsonField(builder, "vz", region.vzPath(), true, 10);
                appendJsonField(builder, "pressure", region.pressurePath(), true, 10);
                appendJsonField(builder, "obstacle", region.obstaclePath(), false, 10);
                builder.append("        }");
                if (regionIndex + 1 < frame.regions().size()) {
                    builder.append(',');
                }
                builder.append('\n');
            }
            builder.append("      ]\n");
            builder.append("    }");
            if (i + 1 < framesSnapshot.size()) {
                builder.append(',');
            }
            builder.append('\n');
        }
        builder.append("  ]\n");
        builder.append("}\n");
        try {
            Files.writeString(metadataPath, builder.toString(), StandardCharsets.UTF_8);
        } catch (IOException ignored) {
        }
    }

    private int dumpRuntimeData(ServerCommandSource source) {
        ServerWorld world = source.getWorld();
        BackgroundMetGrid l0Grid = backgroundMetGrids.get(world.getRegistryKey());
        MesoscaleGrid l1Grid = mesoscaleMetGrids.get(world.getRegistryKey());
        if (l0Grid == null && l1Grid == null) {
            feedback(source, "No L0 or L1 grid is active for " + world.getRegistryKey().getValue());
            return 0;
        }
        String worldId = storageSafeWorldId(world.getRegistryKey());
        Path baseOutputDir = source.getServer()
            .getSavePath(WorldSavePath.ROOT)
            .resolve("aerodynamics4mc")
            .resolve("diagnostics");
        Path l0OutputPath = baseOutputDir.resolve("l0").resolve("l0_" + worldId + "_tick" + tickCounter + ".json");
        Path l1OutputPath = baseOutputDir.resolve("l1").resolve("l1_" + worldId + "_tick" + tickCounter + ".json");
        try {
            if (l0Grid != null) {
                Files.createDirectories(l0OutputPath.getParent());
                Files.writeString(
                    l0OutputPath,
                    encodeBackgroundSnapshot(world.getRegistryKey(), l0Grid.snapshot()),
                    StandardCharsets.UTF_8
                );
            }
            if (l1Grid != null) {
                Files.createDirectories(l1OutputPath.getParent());
                Files.writeString(
                    l1OutputPath,
                    encodeMesoscaleSnapshot(world.getRegistryKey(), l1Grid.snapshot()),
                    StandardCharsets.UTF_8
                );
            }
        } catch (IOException e) {
            feedback(source, "Failed to dump runtime snapshots: " + e.getMessage());
            return 0;
        }
        if (l0Grid != null) {
            feedback(source, "Dumped L0 snapshot to " + l0OutputPath);
            feedback(source, "View L0 with: python3 eval_background_snapshot.py --input \"" + l0OutputPath + "\"");
        }
        if (l1Grid != null) {
            feedback(source, "Dumped L1 snapshot to " + l1OutputPath);
            feedback(source, "View L1 with: python3 eval_mesoscale_snapshot.py --input \"" + l1OutputPath + "\"");
        }
        return 1;
    }

    private int dumpMesoscaleSnapshot(ServerCommandSource source) {
        ServerWorld world = source.getWorld();
        MesoscaleGrid grid = mesoscaleMetGrids.get(world.getRegistryKey());
        if (grid == null) {
            feedback(source, "No mesoscale grid is active for " + world.getRegistryKey().getValue());
            return 0;
        }
        MesoscaleGrid.Snapshot snapshot = grid.snapshot();
        String worldId = storageSafeWorldId(world.getRegistryKey());
        Path outputDir = source.getServer()
            .getSavePath(WorldSavePath.ROOT)
            .resolve("aerodynamics4mc")
            .resolve("diagnostics")
            .resolve("mesoscale");
        Path outputPath = outputDir.resolve("l1_" + worldId + "_tick" + tickCounter + ".json");
        try {
            Files.createDirectories(outputDir);
            Files.writeString(
                outputPath,
                encodeMesoscaleSnapshot(world.getRegistryKey(), snapshot),
                StandardCharsets.UTF_8
            );
        } catch (IOException e) {
            feedback(source, "Failed to dump L1 snapshot: " + e.getMessage());
            return 0;
        }
        feedback(source, "Dumped L1 snapshot to " + outputPath);
        feedback(source, "View with: python3 eval_mesoscale_snapshot.py --input \"" + outputPath + "\"");
        return 1;
    }

    private int nestedFeedbackStatus(ServerCommandSource source) {
        if (sendNestedFeedbackStatus(source)) {
            return 1;
        }
        feedback(source, "Nested feedback diagnostics unavailable for " + source.getWorld().getRegistryKey().getValue());
        return 0;
    }

    private boolean sendNestedFeedbackStatus(ServerCommandSource source) {
        ServerWorld world = source.getWorld();
        RegistryKey<World> worldKey = world.getRegistryKey();
        ConcurrentLinkedQueue<MesoscaleGrid.NestedFeedbackBin> queue = pendingNestedFeedbackBins.get(worldKey);
        int pendingBinCount = queue == null ? 0 : queue.size();
        NestedFeedbackRuntimeDiagnostics runtimeDiagnostics = nestedFeedbackRuntimeDiagnostics.get(worldKey);
        NativeNestedFeedbackWorldDiagnostics nativeDiagnostics = collectNativeNestedFeedbackWorldDiagnostics(worldKey);
        MesoscaleGrid grid = mesoscaleMetGrids.get(worldKey);
        MesoscaleGrid.NestedFeedbackDiagnostics applyDiagnostics = grid == null
            ? null
            : grid.nestedFeedbackDiagnostics();
        if (runtimeDiagnostics == null
            && nativeDiagnostics == null
            && (applyDiagnostics == null || applyDiagnostics.lastAppliedTick() == Long.MIN_VALUE)
            && pendingBinCount <= 0) {
            return false;
        }

        int lastPollAgeTicks = runtimeDiagnostics == null || runtimeDiagnostics.lastPolledTick() == Integer.MIN_VALUE
            ? -1
            : Math.max(0, tickCounter - runtimeDiagnostics.lastPolledTick());
        long lastAppliedAgeTicks = applyDiagnostics == null || applyDiagnostics.lastAppliedTick() == Long.MIN_VALUE
            ? -1L
            : Math.max(0L, tickCounter - applyDiagnostics.lastAppliedTick());
        feedback(
            source,
            "NestedFeedback poll pendingBins=" + pendingBinCount
                + " polledPackets=" + (runtimeDiagnostics == null ? 0L : runtimeDiagnostics.polledPacketCount())
                + " polledBins=" + (runtimeDiagnostics == null ? 0L : runtimeDiagnostics.polledBinCount())
                + " lastPacketBins=" + (runtimeDiagnostics == null ? 0 : runtimeDiagnostics.lastPacketBinCount())
                + " lastPollAge=" + lastPollAgeTicks
                + " lastMeanVolume=" + format3(runtimeDiagnostics == null ? 0.0f : runtimeDiagnostics.lastMeanVolumeAverage())
                + " lastBottomFlux=" + format3(runtimeDiagnostics == null ? 0.0f : runtimeDiagnostics.lastMeanBottomFluxDensity())
                + " lastTopFlux=" + format3(runtimeDiagnostics == null ? 0.0f : runtimeDiagnostics.lastMeanTopFluxDensity())
        );
        feedback(
            source,
            "NestedFeedback apply appliedCells=" + (applyDiagnostics == null ? 0 : applyDiagnostics.appliedCellCount())
                + " inputBins=" + (applyDiagnostics == null ? 0 : applyDiagnostics.inputBinCount())
                + " acceptedBins=" + (applyDiagnostics == null ? 0 : applyDiagnostics.acceptedBinCount())
                + " lastApplyAge=" + lastAppliedAgeTicks
                + " coverageMean=" + format3(applyDiagnostics == null ? 0.0f : applyDiagnostics.meanCoverage())
                + " coverageMax=" + format3(applyDiagnostics == null ? 0.0f : applyDiagnostics.maxCoverage())
                + " windDeltaMean=" + format3(applyDiagnostics == null ? 0.0f : applyDiagnostics.meanWindDelta())
                + " windDeltaMax=" + format3(applyDiagnostics == null ? 0.0f : applyDiagnostics.maxWindDelta())
                + " airDeltaMean=" + format3(applyDiagnostics == null ? 0.0f : applyDiagnostics.meanAirDeltaKelvin())
                + " surfaceDeltaMean=" + format3(applyDiagnostics == null ? 0.0f : applyDiagnostics.meanSurfaceDeltaKelvin())
                + " updraftMean=" + format3(applyDiagnostics == null ? 0.0f : applyDiagnostics.meanNestedUpdraft())
                + " updraftMax=" + format3(applyDiagnostics == null ? 0.0f : applyDiagnostics.maxAbsNestedUpdraft())
        );
        if (nativeDiagnostics != null) {
            int lastBackendResetAgeTicks = nativeDiagnostics.lastBackendResetTick() == Integer.MIN_VALUE
                ? -1
                : Math.max(0, tickCounter - nativeDiagnostics.lastBackendResetTick());
            feedback(
                source,
                "NestedFeedback native regions=" + nativeDiagnostics.regionCount()
                    + " bins=" + nativeDiagnostics.configuredBinCount()
                    + " steps=" + nativeDiagnostics.maxStepsAccumulated()
                    + "/" + nativeDiagnostics.stepsPerFeedback()
                    + " minSteps=" + nativeDiagnostics.minStepsAccumulated()
                    + " readyRegions=" + nativeDiagnostics.readyRegionCount()
                    + " emittedPackets=" + nativeDiagnostics.emittedPacketCount()
                    + " nativeResets=" + nativeDiagnostics.nativeResetCount()
                    + " backendResets=" + nativeDiagnostics.backendResetCount()
                    + " lastBackendResetAge=" + lastBackendResetAgeTicks
            );
        }
        return true;
    }

    private NativeNestedFeedbackWorldDiagnostics collectNativeNestedFeedbackWorldDiagnostics(RegistryKey<World> worldKey) {
        if (simulationServiceId == 0L) {
            return null;
        }
        int regionCount = 0;
        int configuredBinCount = 0;
        int stepsPerFeedback = 0;
        int minStepsAccumulated = Integer.MAX_VALUE;
        int maxStepsAccumulated = 0;
        int readyRegionCount = 0;
        long emittedPacketCount = 0L;
        long nativeResetCount = 0L;
        long backendResetCount = 0L;
        int lastBackendResetTick = Integer.MIN_VALUE;
        for (Map.Entry<WindowKey, RegionRecord> entry : regions.entrySet()) {
            WindowKey key = entry.getKey();
            if (!key.worldKey().equals(worldKey)) {
                continue;
            }
            RegionRecord region = entry.getValue();
            if (!region.serviceActive || region.nestedFeedbackLayout == null) {
                continue;
            }
            NativeSimulationBridge.NestedFeedbackStatus status = simulationBridge.getRegionNestedFeedbackStatus(
                simulationServiceId,
                simulationRegionKey(key)
            );
            if (status == null || status.configuredBinCount() <= 0) {
                continue;
            }
            regionCount++;
            configuredBinCount += status.configuredBinCount();
            stepsPerFeedback = Math.max(stepsPerFeedback, status.stepsPerFeedback());
            minStepsAccumulated = Math.min(minStepsAccumulated, status.stepsAccumulated());
            maxStepsAccumulated = Math.max(maxStepsAccumulated, status.stepsAccumulated());
            if (status.readyPacketBinCount() > 0) {
                readyRegionCount++;
            }
            emittedPacketCount += status.emittedPacketCount();
            nativeResetCount += status.resetCount();
            backendResetCount += region.backendResetCount();
            lastBackendResetTick = Math.max(lastBackendResetTick, region.lastBackendResetTick());
        }
        if (regionCount <= 0) {
            return null;
        }
        if (minStepsAccumulated == Integer.MAX_VALUE) {
            minStepsAccumulated = 0;
        }
        return new NativeNestedFeedbackWorldDiagnostics(
            regionCount,
            configuredBinCount,
            stepsPerFeedback,
            minStepsAccumulated,
            maxStepsAccumulated,
            readyRegionCount,
            emittedPacketCount,
            nativeResetCount,
            backendResetCount,
            lastBackendResetTick
        );
    }

    private String encodeBackgroundSnapshot(RegistryKey<World> worldKey, BackgroundMetGrid.Snapshot snapshot) {
        StringBuilder builder = new StringBuilder(1 << 18);
        builder.append("{\n");
        appendJsonField(builder, "dimension_id", worldKey.getValue().toString(), true);
        appendJsonField(builder, "grid_width", snapshot.gridWidth(), true);
        appendJsonField(builder, "cell_size_blocks", snapshot.cellSizeBlocks(), true);
        appendJsonField(builder, "radius_cells", snapshot.radiusCells(), true);
        appendJsonField(builder, "center_cell_x", snapshot.centerCellX(), true);
        appendJsonField(builder, "center_cell_z", snapshot.centerCellZ(), true);
        appendJsonField(builder, "tick", snapshot.tick(), true);
        appendJsonField(builder, "delta_seconds", snapshot.deltaSeconds(), true);
        appendJsonField(builder, "solar_altitude", snapshot.solarAltitude(), true);
        appendJsonField(builder, "clear_sky", snapshot.clearSky(), true);
        appendJsonField(builder, "rain_gradient", snapshot.rainGradient(), true);
        appendJsonField(builder, "thunder_gradient", snapshot.thunderGradient(), true);
        WorldScaleDriver.Snapshot driver = snapshot.driver();
        if (driver != null) {
            builder.append("  \"driver\": {\n");
            appendJsonField(builder, "driver_time_seconds", driver.driverTimeSeconds(), true, 4);
            appendJsonField(builder, "base_flow_x", driver.baseFlowX(), true, 4);
            appendJsonField(builder, "base_flow_z", driver.baseFlowZ(), true, 4);
            appendJsonField(builder, "airmass_temperature_bias", driver.airmassTemperatureBias(), true, 4);
            appendJsonField(builder, "airmass_moisture_bias", driver.airmassMoistureBias(), true, 4);
            appendJsonField(builder, "planetary_wave_phase", driver.planetaryWavePhase(), true, 4);
            appendJsonField(builder, "storm_activity", driver.stormActivity(), true, 4);
            appendJsonField(builder, "season_phase", driver.seasonPhase(), true, 4);
            appendJsonField(builder, "mesoscale_convective_support", driver.mesoscaleConvectiveSupport(), true, 4);
            appendJsonField(builder, "mesoscale_lift_support", driver.mesoscaleLiftSupport(), true, 4);
            appendJsonField(builder, "mesoscale_shear_support", driver.mesoscaleShearSupport(), true, 4);
            builder.append("    \"cyclone_cells\": [\n");
            List<WorldScaleDriver.CycloneCellSnapshot> cycloneCells = driver.cycloneCells();
            for (int i = 0; i < cycloneCells.size(); i++) {
                WorldScaleDriver.CycloneCellSnapshot cell = cycloneCells.get(i);
                builder.append("      {\n");
                appendJsonField(builder, "center_cell_x", cell.centerCellX(), true, 8);
                appendJsonField(builder, "center_cell_z", cell.centerCellZ(), true, 8);
                appendJsonField(builder, "radius_cells", cell.radiusCells(), true, 8);
                appendJsonField(builder, "intensity", cell.intensity(), true, 8);
                appendJsonField(builder, "pressure_sign", cell.pressureSign(), true, 8);
                appendJsonField(builder, "drift_x_cells_per_second", cell.driftCellsPerSecondX(), true, 8);
                appendJsonField(builder, "drift_z_cells_per_second", cell.driftCellsPerSecondZ(), true, 8);
                appendJsonField(builder, "lifecycle_phase", cell.lifecyclePhase(), true, 8);
                appendJsonField(builder, "warm_core_bias_kelvin", cell.warmCoreBiasKelvin(), true, 8);
                appendJsonField(builder, "moisture_core_bias", cell.moistureCoreBias(), false, 8);
                builder.append("      }");
                if (i + 1 < cycloneCells.size()) {
                    builder.append(',');
                }
                builder.append('\n');
            }
            builder.append("    ],\n");
            builder.append("    \"convective_clusters\": [\n");
            List<WorldScaleDriver.ConvectiveClusterSnapshot> convectiveClusters = driver.convectiveClusters();
            for (int i = 0; i < convectiveClusters.size(); i++) {
                WorldScaleDriver.ConvectiveClusterSnapshot cluster = convectiveClusters.get(i);
                builder.append("      {\n");
                appendJsonField(builder, "center_cell_x", cluster.centerCellX(), true, 8);
                appendJsonField(builder, "center_cell_z", cluster.centerCellZ(), true, 8);
                appendJsonField(builder, "radius_cells", cluster.radiusCells(), true, 8);
                appendJsonField(builder, "intensity", cluster.intensity(), true, 8);
                appendJsonField(builder, "drift_x_cells_per_second", cluster.driftCellsPerSecondX(), true, 8);
                appendJsonField(builder, "drift_z_cells_per_second", cluster.driftCellsPerSecondZ(), true, 8);
                appendJsonField(builder, "lifecycle_phase", cluster.lifecyclePhase(), true, 8);
                appendJsonField(builder, "warm_bias_kelvin", cluster.warmBiasKelvin(), true, 8);
                appendJsonField(builder, "moisture_bias", cluster.moistureBias(), true, 8);
                appendJsonField(builder, "convergence_mps", cluster.convergenceMps(), false, 8);
                builder.append("      }");
                if (i + 1 < convectiveClusters.size()) {
                    builder.append(',');
                }
                builder.append('\n');
            }
            builder.append("    ],\n");
            builder.append("    \"tornado_vortices\": [\n");
            List<WorldScaleDriver.TornadoVortexSnapshot> tornadoVortices = driver.tornadoVortices();
            for (int i = 0; i < tornadoVortices.size(); i++) {
                WorldScaleDriver.TornadoVortexSnapshot vortex = tornadoVortices.get(i);
                builder.append("      {\n");
                appendJsonField(builder, "id", vortex.id(), true, 8);
                appendJsonField(builder, "parent_convective_cluster_id", vortex.parentConvectiveClusterId(), true, 8);
                appendJsonField(builder, "age_seconds", vortex.ageSeconds(), true, 8);
                appendJsonField(builder, "lifetime_seconds", vortex.lifetimeSeconds(), true, 8);
                appendJsonField(builder, "state_ordinal", vortex.stateOrdinal(), true, 8);
                appendJsonField(builder, "center_block_x", vortex.centerBlockX(), true, 8);
                appendJsonField(builder, "center_block_z", vortex.centerBlockZ(), true, 8);
                appendJsonField(builder, "base_y", vortex.baseY(), true, 8);
                appendJsonField(builder, "translation_x_blocks_per_second", vortex.translationXBlocksPerSecond(), true, 8);
                appendJsonField(builder, "translation_z_blocks_per_second", vortex.translationZBlocksPerSecond(), true, 8);
                appendJsonField(builder, "core_radius_blocks", vortex.coreRadiusBlocks(), true, 8);
                appendJsonField(builder, "influence_radius_blocks", vortex.influenceRadiusBlocks(), true, 8);
                appendJsonField(builder, "tangential_wind_scale_mps", vortex.tangentialWindScaleMps(), true, 8);
                appendJsonField(builder, "radial_inflow_scale_mps", vortex.radialInflowScaleMps(), true, 8);
                appendJsonField(builder, "updraft_scale", vortex.updraftScale(), true, 8);
                appendJsonField(builder, "condensation_bias", vortex.condensationBias(), true, 8);
                appendJsonField(builder, "intensity", vortex.intensity(), true, 8);
                appendJsonField(builder, "rotation_sign", vortex.rotationSign(), false, 8);
                builder.append("      }");
                if (i + 1 < tornadoVortices.size()) {
                    builder.append(',');
                }
                builder.append('\n');
            }
            builder.append("    ]\n");
            builder.append("  },\n");
        }
        appendJsonArray(builder, "terrain_height_blocks", snapshot.terrainHeightBlocks(), true);
        appendJsonArray(builder, "biome_temperature", snapshot.biomeTemperature(), true);
        appendJsonArray(builder, "roughness_length_meters", snapshot.roughnessLengthMeters(), true);
        appendJsonByteArray(builder, "surface_class", snapshot.surfaceClass(), true);
        appendJsonArray(builder, "ambient_air_temperature_kelvin", snapshot.ambientAirTemperatureKelvin(), true);
        appendJsonArray(builder, "deep_ground_temperature_kelvin", snapshot.deepGroundTemperatureKelvin(), true);
        appendJsonArray(builder, "surface_temperature_kelvin", snapshot.surfaceTemperatureKelvin(), true);
        appendJsonArray(builder, "wind_x", snapshot.windX(), true);
        appendJsonArray(builder, "wind_z", snapshot.windZ(), true);
        appendJsonArray(builder, "humidity", snapshot.humidity(), true);
        appendJsonArray(builder, "vorticity", snapshot.vorticity(), true);
        appendJsonArray(builder, "divergence", snapshot.divergence(), true);
        appendJsonArray(builder, "temperature_anomaly", snapshot.temperatureAnomaly(), false);
        builder.append("\n}\n");
        return builder.toString();
    }

    private String encodeMesoscaleSnapshot(RegistryKey<World> worldKey, MesoscaleGrid.Snapshot snapshot) {
        StringBuilder builder = new StringBuilder(1 << 20);
        builder.append("{\n");
        appendJsonField(builder, "dimension_id", worldKey.getValue().toString(), true);
        appendJsonField(builder, "grid_width", snapshot.gridWidth(), true);
        appendJsonField(builder, "active_layers", snapshot.activeLayers(), true);
        appendJsonField(builder, "cell_size_blocks", snapshot.cellSizeBlocks(), true);
        appendJsonField(builder, "layer_height_blocks", snapshot.layerHeightBlocks(), true);
        appendJsonField(builder, "radius_cells", snapshot.radiusCells(), true);
        appendJsonField(builder, "center_cell_x", snapshot.centerCellX(), true);
        appendJsonField(builder, "center_cell_z", snapshot.centerCellZ(), true);
        appendJsonField(builder, "vertical_base_y", snapshot.verticalBaseY(), true);
        appendJsonField(builder, "step_seconds", snapshot.stepSeconds(), true);
        appendJsonField(builder, "tick", snapshot.lastTickProcessed(), true);
        builder.append("  \"nested_feedback_diagnostics\": {\n");
        MesoscaleGrid.NestedFeedbackDiagnostics nestedFeedbackDiagnostics = snapshot.nestedFeedbackDiagnostics();
        appendJsonField(builder, "last_applied_tick", nestedFeedbackDiagnostics.lastAppliedTick(), true, 4);
        appendJsonField(builder, "input_bin_count", nestedFeedbackDiagnostics.inputBinCount(), true, 4);
        appendJsonField(builder, "accepted_bin_count", nestedFeedbackDiagnostics.acceptedBinCount(), true, 4);
        appendJsonField(builder, "applied_cell_count", nestedFeedbackDiagnostics.appliedCellCount(), true, 4);
        appendJsonField(builder, "mean_coverage", nestedFeedbackDiagnostics.meanCoverage(), true, 4);
        appendJsonField(builder, "max_coverage", nestedFeedbackDiagnostics.maxCoverage(), true, 4);
        appendJsonField(builder, "mean_wind_delta", nestedFeedbackDiagnostics.meanWindDelta(), true, 4);
        appendJsonField(builder, "max_wind_delta", nestedFeedbackDiagnostics.maxWindDelta(), true, 4);
        appendJsonField(builder, "mean_air_delta_kelvin", nestedFeedbackDiagnostics.meanAirDeltaKelvin(), true, 4);
        appendJsonField(builder, "max_air_delta_kelvin", nestedFeedbackDiagnostics.maxAirDeltaKelvin(), true, 4);
        appendJsonField(builder, "mean_surface_delta_kelvin", nestedFeedbackDiagnostics.meanSurfaceDeltaKelvin(), true, 4);
        appendJsonField(builder, "max_surface_delta_kelvin", nestedFeedbackDiagnostics.maxSurfaceDeltaKelvin(), true, 4);
        appendJsonField(builder, "mean_bottom_flux_density", nestedFeedbackDiagnostics.meanBottomFluxDensity(), true, 4);
        appendJsonField(builder, "mean_top_flux_density", nestedFeedbackDiagnostics.meanTopFluxDensity(), true, 4);
        appendJsonField(builder, "mean_nested_updraft", nestedFeedbackDiagnostics.meanNestedUpdraft(), true, 4);
        appendJsonField(builder, "max_abs_nested_updraft", nestedFeedbackDiagnostics.maxAbsNestedUpdraft(), false, 4);
        builder.append("  },\n");
        appendJsonArray(builder, "terrain_height_blocks", snapshot.terrainHeightBlocks(), true);
        appendJsonArray(builder, "biome_temperature", snapshot.biomeTemperature(), true);
        appendJsonArray(builder, "roughness_length_meters", snapshot.roughnessLengthMeters(), true);
        appendJsonByteArray(builder, "surface_class", snapshot.surfaceClass(), true);
        appendJsonArray(builder, "ambient_air_temperature_kelvin", snapshot.ambientAirTemperatureKelvin(), true);
        appendJsonArray(builder, "deep_ground_temperature_kelvin", snapshot.deepGroundTemperatureKelvin(), true);
        appendJsonArray(builder, "surface_temperature_kelvin", snapshot.surfaceTemperatureKelvin(), true);
        appendJsonArray(builder, "forcing_ambient_target_kelvin", snapshot.forcingAmbientTargetKelvin(), true);
        appendJsonArray(builder, "forcing_surface_target_kelvin", snapshot.forcingSurfaceTargetKelvin(), true);
        appendJsonArray(builder, "forcing_background_wind_x", snapshot.forcingBackgroundWindX(), true);
        appendJsonArray(builder, "forcing_background_wind_z", snapshot.forcingBackgroundWindZ(), true);
        appendJsonArray(builder, "forcing_nested_ambient_delta_kelvin", snapshot.forcingNestedAmbientDeltaKelvin(), true);
        appendJsonArray(builder, "forcing_nested_surface_delta_kelvin", snapshot.forcingNestedSurfaceDeltaKelvin(), true);
        appendJsonArray(builder, "forcing_nested_wind_x_delta", snapshot.forcingNestedWindXDelta(), true);
        appendJsonArray(builder, "forcing_nested_wind_z_delta", snapshot.forcingNestedWindZDelta(), true);
        appendJsonArray(builder, "forcing_nested_updraft", snapshot.forcingNestedUpdraft(), true);
        appendJsonArray(builder, "wind_x", snapshot.windX(), true);
        appendJsonArray(builder, "wind_z", snapshot.windZ(), true);
        appendJsonArray(builder, "humidity", snapshot.humidity(), true);
        appendJsonArray(builder, "instability_proxy", snapshot.instabilityProxy(), true);
        appendJsonArray(builder, "low_level_shear", snapshot.lowLevelShear(), true);
        appendJsonArray(builder, "moisture_convergence", snapshot.moistureConvergence(), true);
        appendJsonArray(builder, "lift_proxy", snapshot.liftProxy(), false);
        builder.append("\n}\n");
        return builder.toString();
    }

    private String storageSafeWorldId(RegistryKey<World> worldKey) {
        return worldKey.getValue().toString()
            .replace(':', '_')
            .replace('/', '_');
    }

    private Path worldScaleDriverPath(ServerWorld world) {
        return world.getServer()
            .getSavePath(WorldSavePath.ROOT)
            .resolve("aerodynamics4mc")
            .resolve("weather")
            .resolve("driver_" + storageSafeWorldId(world.getRegistryKey()) + ".properties");
    }

    private WorldScaleDriver loadWorldScaleDriver(ServerWorld world) {
        return WorldScaleDriver.loadOrCreate(worldScaleDriverPath(world), world);
    }

    private void saveWorldScaleDriver(ServerWorld world, WorldScaleDriver driver) {
        if (world == null || driver == null) {
            return;
        }
        try {
            driver.save(worldScaleDriverPath(world));
        } catch (IOException ignored) {
            // Keep runtime alive if a diagnostics/persistence write fails.
        }
    }

    private void saveAllWorldScaleDrivers(MinecraftServer server) {
        if (server == null) {
            return;
        }
        for (Map.Entry<RegistryKey<World>, WorldScaleDriver> entry : worldScaleDrivers.entrySet()) {
            ServerWorld world = server.getWorld(entry.getKey());
            if (world == null) {
                continue;
            }
            saveWorldScaleDriver(world, entry.getValue());
        }
    }

    private void appendJsonField(StringBuilder builder, String key, String value, boolean trailingComma) {
        appendJsonField(builder, key, value, trailingComma, 2);
    }

    private void appendJsonField(StringBuilder builder, String key, String value, boolean trailingComma, int indentSpaces) {
        appendIndent(builder, indentSpaces);
        builder.append("  \"")
            .append(key)
            .append("\": \"")
            .append(value.replace("\\", "\\\\").replace("\"", "\\\""))
            .append("\"");
        if (trailingComma) {
            builder.append(',');
        }
        builder.append('\n');
    }

    private void appendJsonField(StringBuilder builder, String key, int value, boolean trailingComma) {
        appendJsonField(builder, key, value, trailingComma, 2);
    }

    private void appendJsonField(StringBuilder builder, String key, int value, boolean trailingComma, int indentSpaces) {
        appendIndent(builder, indentSpaces);
        builder.append('"')
            .append(key)
            .append("\": ")
            .append(value);
        if (trailingComma) {
            builder.append(',');
        }
        builder.append('\n');
    }

    private void appendJsonField(StringBuilder builder, String key, long value, boolean trailingComma) {
        appendJsonField(builder, key, value, trailingComma, 2);
    }

    private void appendJsonField(StringBuilder builder, String key, long value, boolean trailingComma, int indentSpaces) {
        appendIndent(builder, indentSpaces);
        builder.append('"')
            .append(key)
            .append("\": ")
            .append(value);
        if (trailingComma) {
            builder.append(',');
        }
        builder.append('\n');
    }

    private void appendJsonField(StringBuilder builder, String key, float value, boolean trailingComma) {
        appendJsonField(builder, key, value, trailingComma, 2);
    }

    private void appendJsonField(StringBuilder builder, String key, float value, boolean trailingComma, int indentSpaces) {
        appendIndent(builder, indentSpaces);
        builder.append('"')
            .append(key)
            .append("\": ")
            .append(Float.toString(value));
        if (trailingComma) {
            builder.append(',');
        }
        builder.append('\n');
    }

    private void appendJsonField(StringBuilder builder, String key, boolean value, boolean trailingComma) {
        appendJsonField(builder, key, value, trailingComma, 2);
    }

    private void appendJsonField(StringBuilder builder, String key, boolean value, boolean trailingComma, int indentSpaces) {
        appendIndent(builder, indentSpaces);
        builder.append('"')
            .append(key)
            .append("\": ")
            .append(value ? "true" : "false");
        if (trailingComma) {
            builder.append(',');
        }
        builder.append('\n');
    }

    private void appendIndent(StringBuilder builder, int indentSpaces) {
        for (int i = 0; i < indentSpaces; i++) {
            builder.append(' ');
        }
    }

    private void appendJsonFieldLegacyContextOnly(StringBuilder builder, String key, int value, boolean trailingComma) {
        builder.append("  \"")
            .append(key)
            .append("\": ")
            .append(value);
        if (trailingComma) {
            builder.append(',');
        }
        builder.append('\n');
    }

    private void appendJsonArray(StringBuilder builder, String key, float[] values, boolean trailingComma) {
        builder.append("  \"").append(key).append("\": [");
        for (int i = 0; i < values.length; i++) {
            if (i > 0) {
                builder.append(',');
            }
            builder.append(Float.toString(values[i]));
        }
        builder.append(']');
        if (trailingComma) {
            builder.append(',');
        }
        builder.append('\n');
    }

    private void appendJsonByteArray(StringBuilder builder, String key, byte[] values, boolean trailingComma) {
        builder.append("  \"").append(key).append("\": [");
        for (int i = 0; i < values.length; i++) {
            if (i > 0) {
                builder.append(',');
            }
            builder.append(Byte.toUnsignedInt(values[i]));
        }
        builder.append(']');
        if (trailingComma) {
            builder.append(',');
        }
        builder.append('\n');
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
        InspectionSolveSession inspectionSolveSession = activeInspectionSolveSession;
        if (inspectionSolveSession != null) {
            if (inspectionSolveSession.completed.get()) {
                activeInspectionSolveSession = null;
            }
            recordMainThreadPhase(MAIN_THREAD_PHASE_SERVICE_INIT, 0L);
            recordMainThreadPhase(MAIN_THREAD_PHASE_FOCUS, 0L);
            recordMainThreadPhase(MAIN_THREAD_PHASE_BACKGROUND_BATCH, 0L);
            recordMainThreadPhase(MAIN_THREAD_PHASE_ACTIVE_BATCH, 0L);
            recordMainThreadPhase(MAIN_THREAD_PHASE_LIVE_BUILDS, 0L);
            recordMainThreadPhase(MAIN_THREAD_PHASE_FAN_REFRESHES, 0L);
            recordMainThreadPhase(MAIN_THREAD_PHASE_COORDINATOR, 0L);
            recordMainThreadPhase(MAIN_THREAD_PHASE_FLOW_SYNC, 0L);
            recordMainThreadPhase(MAIN_THREAD_PHASE_TOTAL, System.nanoTime() - tickStartNanos);
            return;
        }
        long phaseStartNanos = System.nanoTime();
        ensureSimulationServiceInitialized();
        recordMainThreadPhase(MAIN_THREAD_PHASE_SERVICE_INIT, System.nanoTime() - phaseStartNanos);
        phaseStartNanos = System.nanoTime();
        updateSimulationFocus(server);
        recordMainThreadPhase(MAIN_THREAD_PHASE_FOCUS, System.nanoTime() - phaseStartNanos);
        if (tickCounter == 1 || tickCounter % MESOSCALE_REFRESH_TICKS == 0) {
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
            tickL2Capture(server);
            recordMainThreadPhase(MAIN_THREAD_PHASE_FLOW_SYNC, System.nanoTime() - phaseStartNanos);
        } else {
            tickL2Capture(server);
            recordMainThreadPhase(MAIN_THREAD_PHASE_FLOW_SYNC, 0L);
        }
        long frameId = frame.frameId();
        int publishedSteps = 0;
        if (frameId > lastObservedPublishedFrameId) {
            long delta = frameId - lastObservedPublishedFrameId;
            simulationTicks += delta;
            publishedSteps = (int) Math.min(Integer.MAX_VALUE, delta);
        }
        if (frameId > lastObservedPublishedFrameId) {
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
        L2CaptureSession captureSession = activeL2CaptureSession;
        if (captureSession != null) {
            captureSession.stopRequested.set(true);
            activeL2CaptureSession = null;
            persistL2CaptureMetadata(captureSession);
        }
        streamingEnabled = false;
        stopSimulationCoordinator();
        saveAllWorldScaleDrivers(server);
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
        pendingNestedFeedbackBins.clear();
        nestedFeedbackRuntimeDiagnostics.clear();
        desiredWindowKeys = Set.of();
        desiredSolveWindowKeys = Set.of();
        anchorDesiredWindowKeys = Set.of();
        anchorSolveWindowKeys = Set.of();
        shellWindowRetainUntilTick.clear();
        mirrorOnlyPrewarmedWindowKeys.clear();
        mirrorOnlyUploadedBrickStaticSignatures.clear();
        uploadedBrickDynamicSeedSignatures.clear();
        brickRuntimeHintWorldKeys.clear();
        brickRuntimeKnownWorldKeys.clear();
        activePlayerProbeRequests = List.of();
        activeEntitySampleRequests = List.of();
        worldEnvironmentSnapshots = Map.of();
        publishedFrame.set(null);
        publishedPlayerProbes.set(Map.of());
        publishedEntitySamples.set(Map.of());
        synchronized (pendingWorldDeltasLock) {
            pendingWorldDeltas.clear();
        }
        synchronized (pendingResidentBrickStaticRefreshesLock) {
            pendingResidentBrickStaticRefreshes.clear();
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
        pendingNestedFeedbackBins.keySet().removeIf(worldKey -> !activeWorldKeys.contains(worldKey));
        nestedFeedbackRuntimeDiagnostics.keySet().removeIf(worldKey -> !activeWorldKeys.contains(worldKey));
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
            MesoscaleGrid existingMesoscale = mesoscaleMetGrids.get(worldKey);
            MesoscaleGrid.DiagnosticsSummary diagnosticsSummary = existingMesoscale == null
                ? null
                : existingMesoscale.diagnosticsSummary(request.focus());
            WorldScaleDriver driver = worldScaleDrivers.computeIfAbsent(
                worldKey,
                ignored -> loadWorldScaleDriver(request.world())
            );
            driver.advance(
                request.world(),
                request.environmentSnapshot(),
                batch.tickCounter(),
                SOLVER_STEP_SECONDS,
                diagnosticsSummary,
                existingMesoscale,
                request.focus()
            );
            BackgroundMetGrid grid = backgroundMetGrids.computeIfAbsent(
                worldKey,
                ignored -> new BackgroundMetGrid(
                    BACKGROUND_MET_CELL_SIZE_BLOCKS,
                    BACKGROUND_MET_RADIUS_CELLS,
                    BACKGROUND_MET_REFRESH_TICKS
                )
            );
            grid.refresh(
                request.world(),
                request.environmentSnapshot(),
                request.focus(),
                batch.tickCounter(),
                SOLVER_STEP_SECONDS,
                seedTerrainProvider,
                driver
            );
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
            mesoscale.applyPendingNestedFeedback(drainPendingNestedFeedback(worldKey));
        }
    }

    private List<MesoscaleGrid.NestedFeedbackBin> drainPendingNestedFeedback(RegistryKey<World> worldKey) {
        ConcurrentLinkedQueue<MesoscaleGrid.NestedFeedbackBin> queue = pendingNestedFeedbackBins.get(worldKey);
        if (queue == null || queue.isEmpty()) {
            return List.of();
        }
        List<MesoscaleGrid.NestedFeedbackBin> drained = new ArrayList<>();
        while (true) {
            MesoscaleGrid.NestedFeedbackBin bin = queue.poll();
            if (bin == null) {
                break;
            }
            drained.add(bin);
        }
        return drained.isEmpty() ? List.of() : drained;
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
        for (RegionRecord region : regions.values()) {
            region.nestedFeedbackLayoutServiceId = 0L;
        }
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

    private void applyPendingResidentBrickStaticRefreshesIfNeeded() {
        long serviceId = simulationServiceId;
        MinecraftServer server = currentServer;
        if (serviceId == 0L || server == null) {
            return;
        }
        while (true) {
            ChunkResidentBrickRefreshRequest request;
            synchronized (pendingResidentBrickStaticRefreshesLock) {
                request = pendingResidentBrickStaticRefreshes.pollFirst();
            }
            if (request == null) {
                return;
            }
            ServerWorld world = server.getWorld(request.worldKey());
            if (world == null) {
                continue;
            }
            refreshResidentBrickStaticsForChunk(world, new ChunkPos(request.chunkX(), request.chunkZ()));
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

    private static final class L2CaptureSession {
        final RegistryKey<World> worldKey;
        final Identifier dimensionId;
        final BlockPos anchorCoreOrigin;
        final List<L2CaptureRegionSpec> regions;
        final Path outputDir;
        final int startTick;
        final int endTick;
        final int durationSeconds;
        final int fps;
        final int sampleIntervalTicks;
        final int layoutMinX;
        final int layoutMinY;
        final int layoutMinZ;
        final int layoutMaxExclusiveX;
        final int layoutMaxExclusiveY;
        final int layoutMaxExclusiveZ;
        final AtomicBoolean stopRequested = new AtomicBoolean(false);
        final AtomicInteger nextFrameIndex = new AtomicInteger(0);
        final AtomicInteger inFlightFrames = new AtomicInteger(0);
        final AtomicInteger framesWritten = new AtomicInteger(0);
        final AtomicInteger framesDropped = new AtomicInteger(0);
        final AtomicInteger framesFailed = new AtomicInteger(0);
        final List<L2CaptureFrameRecord> frames = Collections.synchronizedList(new ArrayList<>());
        volatile int nextSampleTick;

        L2CaptureSession(
            RegistryKey<World> worldKey,
            Identifier dimensionId,
            BlockPos anchorCoreOrigin,
            List<L2CaptureRegionSpec> regions,
            Path outputDir,
            int startTick,
            int endTick,
            int durationSeconds,
            int fps,
            int sampleIntervalTicks
        ) {
            this.worldKey = worldKey;
            this.dimensionId = dimensionId;
            this.anchorCoreOrigin = anchorCoreOrigin.toImmutable();
            this.regions = List.copyOf(regions);
            this.outputDir = outputDir;
            this.startTick = startTick;
            this.endTick = endTick;
            this.durationSeconds = durationSeconds;
            this.fps = fps;
            this.sampleIntervalTicks = sampleIntervalTicks;
            int minX = Integer.MAX_VALUE;
            int minY = Integer.MAX_VALUE;
            int minZ = Integer.MAX_VALUE;
            int maxX = Integer.MIN_VALUE;
            int maxY = Integer.MIN_VALUE;
            int maxZ = Integer.MIN_VALUE;
            for (L2CaptureRegionSpec region : this.regions) {
                BlockPos coreOrigin = region.coreOrigin();
                minX = Math.min(minX, coreOrigin.getX());
                minY = Math.min(minY, coreOrigin.getY());
                minZ = Math.min(minZ, coreOrigin.getZ());
                maxX = Math.max(maxX, coreOrigin.getX() + REGION_CORE_SIZE);
                maxY = Math.max(maxY, coreOrigin.getY() + REGION_CORE_SIZE);
                maxZ = Math.max(maxZ, coreOrigin.getZ() + REGION_CORE_SIZE);
            }
            this.layoutMinX = minX;
            this.layoutMinY = minY;
            this.layoutMinZ = minZ;
            this.layoutMaxExclusiveX = maxX;
            this.layoutMaxExclusiveY = maxY;
            this.layoutMaxExclusiveZ = maxZ;
            this.nextSampleTick = startTick;
        }
    }

    private record L2CaptureRegionSpec(
        WindowKey key,
        BlockPos coreOrigin
    ) {
    }

    private record L2CaptureFrameRecord(
        int index,
        int tick,
        List<L2CaptureFrameRegionRecord> regions
    ) {
    }

    private record L2CaptureFrameRegionRecord(
        int originX,
        int originY,
        int originZ,
        int coreOriginX,
        int coreOriginY,
        int coreOriginZ,
        String vxPath,
        String vyPath,
        String vzPath,
        String pressurePath,
        String obstaclePath
    ) {
    }

    private record L2CapturePendingFrame(
        int frameIndex,
        int tick,
        List<L2CapturePendingRegionFrame> regions
    ) {
    }

    private record L2CapturePendingRegionFrame(
        BlockPos origin,
        BlockPos coreOrigin,
        float[] vx,
        float[] vy,
        float[] vz,
        float[] pressure,
        byte[] obstacle
    ) {
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
                anchors.add(new PlayerRegionAnchor(worldKey, coreOriginForPosition(playerPos), playerPos.toImmutable()));
                probeRequests.add(new PlayerProbeRequest(player.getUuid(), worldKey, playerPos.toImmutable()));
            }
        }
        Set<WindowKey> activeKeys = activeRegionKeys(anchors);
        Map<RegistryKey<World>, int[]> brickRuntimeHintCoordsByWorld = brickRuntimeHintCoords(anchors);
        List<EntitySampleRequest> entityRequests = ENTITY_SAMPLE_COLLECTION_ENABLED
            ? collectEntitySampleRequests(server, activeKeys)
            : List.of();
        return new ActiveRegionBatch(
            tickCounter,
            List.copyOf(anchors),
            List.copyOf(probeRequests),
            List.copyOf(entityRequests),
            Map.copyOf(snapshots),
            brickRuntimeHintCoordsByWorld
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
        Set<WindowKey> solveWindows = desiredSolveWindowKeys;
        List<WindowKey> mirrorOnlyWindowKeysToRemove = new ArrayList<>();

        for (Map.Entry<WindowKey, RegionRecord> regionEntry : regions.entrySet()) {
            WindowKey key = regionEntry.getKey();
            RegionRecord region = regionEntry.getValue();
            boolean solveWindow = solveWindows.contains(key);
            if (!shouldActivelyRefreshWindowThisTick(region, solveWindow)) {
                continue;
            }
            if (region.attached() || region.serviceActive || region.serviceReady) {
                downgradeWindowToMirrorOnly(key, region);
            }
            if (!solveWindow) {
                if (region.sections == null) {
                    initializeWindowFromMirror(key, region);
                } else {
                    refreshWindowFromMirror(key, region);
                }
                if (!region.busy.get() && !region.attached() && !region.serviceActive && !region.serviceReady) {
                    mirrorOnlyWindowKeysToRemove.add(key);
                }
                continue;
            }
            if (!region.fullWindowSectionsRequested && currentServer != null) {
                requestWindowSections(currentServer, key, false);
                region.fullWindowSectionsRequested = true;
            }
            if (region.sections == null) {
                initializeWindowFromMirror(key, region);
            } else {
                refreshWindowFromMirror(key, region);
            }
            prewarmMirrorOnlyWindowCoreStaticBrick(key);
        }
        for (WindowKey key : mirrorOnlyWindowKeysToRemove) {
            RegionRecord region = regions.get(key);
            if (region == null || region.busy.get() || region.attached() || region.serviceActive || region.serviceReady) {
                continue;
            }
            mirrorOnlyPrewarmedWindowKeys.add(key);
            regions.remove(key);
        }
    }

    private boolean shouldActivelyRefreshWindowThisTick(RegionRecord region, boolean solveWindow) {
        if (solveWindow) {
            return true;
        }
        return region.attached()
            || region.serviceReady
            || region.serviceActive
            || region.sections == null;
    }

    private void downgradeWindowToMirrorOnly(WindowKey key, RegionRecord region) {
        if (region.busy.get()) {
            return;
        }
        if (region.attached()) {
            deactivateWindow(key, region, true, false);
        } else {
            removePublishedRegionAtlas(key);
            if (region.serviceActive || region.serviceReady) {
                deactivateWindowRegionInSimulation(key);
            }
            releaseWindow(key, region);
        }
        region.dynamicRestoreAttempted = false;
        region.completedMaxSpeed = 0.0f;
    }

    private void synchronizeDesiredRegions() {
        Set<WindowKey> desiredWindows = new HashSet<>(desiredWindowKeys);
        Set<WindowKey> solveWindows = desiredSolveWindowKeys;
        Iterator<Map.Entry<WindowKey, Integer>> retainIterator = shellWindowRetainUntilTick.entrySet().iterator();
        while (retainIterator.hasNext()) {
            Map.Entry<WindowKey, Integer> entry = retainIterator.next();
            if (entry.getValue() < tickCounter || solveWindows.contains(entry.getKey())) {
                retainIterator.remove();
                continue;
            }
            desiredWindows.add(entry.getKey());
        }
        mirrorOnlyUploadedBrickStaticSignatures.keySet().retainAll(desiredWindows);
        uploadedBrickDynamicSeedSignatures.keySet().retainAll(desiredWindows);
        MinecraftServer server = currentServer;
        Iterator<Map.Entry<WindowKey, RegionRecord>> iterator = regions.entrySet().iterator();
        while (iterator.hasNext()) {
            Map.Entry<WindowKey, RegionRecord> entry = iterator.next();
            if (desiredWindows.contains(entry.getKey())) {
                continue;
            }
            shellWindowRetainUntilTick.remove(entry.getKey());
            mirrorOnlyPrewarmedWindowKeys.remove(entry.getKey());
            mirrorOnlyUploadedBrickStaticSignatures.remove(entry.getKey());
            uploadedBrickDynamicSeedSignatures.remove(entry.getKey());
            RegionRecord region = entry.getValue();
            if (region.attached()) {
                deactivateWindow(entry.getKey(), region, true, false);
            } else {
                deactivateWindowRegionInSimulation(entry.getKey());
            }
            iterator.remove();
        }
        for (WindowKey key : desiredWindows) {
            RegionRecord existingRegion = regions.get(key);
            if (existingRegion != null) {
                if (!solveWindows.contains(key)) {
                    prewarmMirrorOnlyWindowCoreStaticBrick(key);
                }
                continue;
            }
            boolean solveWindow = solveWindows.contains(key);
            if (!solveWindow) {
                if (server != null && mirrorOnlyPrewarmedWindowKeys.add(key)) {
                    requestWindowSections(server, key, false);
                }
                prewarmMirrorOnlyWindowCoreStaticBrick(key);
                continue;
            }
            mirrorOnlyPrewarmedWindowKeys.remove(key);
            mirrorOnlyUploadedBrickStaticSignatures.remove(key);
            RegionRecord region = new RegionRecord();
            regions.put(key, region);
            if (server != null) {
                requestWindowSections(server, key, false);
                region.fullWindowSectionsRequested = true;
            }
        }
    }

    private void requestWindowSections(MinecraftServer server, WindowKey key, boolean includeHaloSections) {
        for (int sx = CORE_SECTION_MIN; sx <= CORE_SECTION_MAX; sx++) {
            for (int sy = CORE_SECTION_MIN; sy <= CORE_SECTION_MAX; sy++) {
                for (int sz = CORE_SECTION_MIN; sz <= CORE_SECTION_MAX; sz++) {
                    BlockPos localOrigin = sectionOrigin(key.origin(), sx, sy, sz);
                    worldMirror.requestSectionBuild(server, key.worldKey(), localOrigin, true);
                }
            }
        }
        if (!includeHaloSections) {
            return;
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
        if (!uploadSolveWindowCoreStaticBrickToRuntime(key, region)) {
            return false;
        }
        if (!region.serviceActive) {
            activateWindowRegionInSimulation(key);
        }
        ensureRegionNestedFeedbackLayoutInSimulation(key, region);
        refreshRegionLifecycle(key, region);
        return region.serviceReady;
    }

    private boolean attachOrRefreshRegionWindow(WindowKey key, RegionRecord region) {
        if (!region.attached()) {
            if (!initializeWindowFromMirror(key, region)) {
                return false;
            }
            if (!region.dynamicRestoreAttempted) {
                if (!ensureWindowDynamicRegionInitialized(key, region)) {
                    return false;
                }
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
        return queryFanSources(worldKey, origin, GRID_SIZE);
    }

    private List<FanSource> queryFanSources(RegistryKey<World> worldKey, BlockPos origin, int gridSize) {
        List<WorldMirror.FanRecord> fans = worldMirror.queryFans(
            worldKey,
            origin,
            gridSize,
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
        for (int sx = CORE_SECTION_MIN; sx <= CORE_SECTION_MAX; sx++) {
            for (int sy = CORE_SECTION_MIN; sy <= CORE_SECTION_MAX; sy++) {
                for (int sz = CORE_SECTION_MIN; sz <= CORE_SECTION_MAX; sz++) {
                    BlockPos localOrigin = sectionOrigin(key.origin(), sx, sy, sz);
                    WorldMirror.SectionSnapshot snapshot = worldMirror.peekSection(key.worldKey(), localOrigin);
                    if (snapshot == null) {
                        requestWindowSectionIfNeeded(key, sx, sy, sz);
                        continue;
                    }
                    region.setSection(sx, sy, sz, snapshot);
                    readyCoreSections++;
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
        for (int sx = CORE_SECTION_MIN; sx <= CORE_SECTION_MAX; sx++) {
            for (int sy = CORE_SECTION_MIN; sy <= CORE_SECTION_MAX; sy++) {
                for (int sz = CORE_SECTION_MIN; sz <= CORE_SECTION_MAX; sz++) {
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
        removePublishedRegionAtlas(key);
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
            region.noteBackendReset(tickCounter);
            if (region.sections != null) {
                refreshRegionLifecycle(key, region);
                if (!region.serviceActive) {
                    activateWindowRegionInSimulation(key);
                    refreshRegionLifecycle(key, region);
                }
                region.forcingDirty = true;
                refreshRegionFansIfNeeded(key, region);
                ensureRegionNestedFeedbackLayoutInSimulation(key, region);
                seedWindowDynamicRegionFromNestedMet(
                    key,
                    region,
                    new float[FLOW_COUNT],
                    new float[GRID_SIZE * GRID_SIZE * GRID_SIZE],
                    new float[GRID_SIZE * GRID_SIZE * GRID_SIZE]
                );
                uploadRegionFanForcingToSimulation(key, region);
                refreshRegionThermalInSimulation(key, region);
            }
        }
        region.clearBackendResetPending();
    }

    private Set<WindowKey> activeRegionKeys(List<PlayerRegionAnchor> anchors) {
        Set<WindowKey> keys = new HashSet<>();
        for (PlayerRegionAnchor anchor : anchors) {
            RegistryKey<World> worldKey = anchor.worldKey();
            BlockPos baseCore = anchor.coreOrigin();
            int localX = MathHelper.clamp(anchor.blockPos().getX() - baseCore.getX(), 0, REGION_CORE_SIZE - 1);
            int localY = MathHelper.clamp(anchor.blockPos().getY() - baseCore.getY(), 0, REGION_CORE_SIZE - 1);
            int localZ = MathHelper.clamp(anchor.blockPos().getZ() - baseCore.getZ(), 0, REGION_CORE_SIZE - 1);
            boolean attachWest = localX < HORIZONTAL_ATTACH_MARGIN_CELLS;
            boolean attachEast = localX >= REGION_CORE_SIZE - HORIZONTAL_ATTACH_MARGIN_CELLS;
            boolean attachDown = localY < VERTICAL_ATTACH_MARGIN_CELLS;
            boolean attachUp = localY >= REGION_CORE_SIZE - VERTICAL_ATTACH_MARGIN_CELLS;
            boolean attachNorth = localZ < HORIZONTAL_ATTACH_MARGIN_CELLS;
            boolean attachSouth = localZ >= REGION_CORE_SIZE - HORIZONTAL_ATTACH_MARGIN_CELLS;
            keys.add(new WindowKey(worldKey, windowOriginFromCoreOrigin(baseCore)));
            if (attachWest) {
                keys.add(new WindowKey(
                    worldKey,
                    windowOriginFromCoreOrigin(baseCore.add(-REGION_LATTICE_STRIDE, 0, 0))
                ));
            }
            if (attachEast) {
                keys.add(new WindowKey(
                    worldKey,
                    windowOriginFromCoreOrigin(baseCore.add(REGION_LATTICE_STRIDE, 0, 0))
                ));
            }
            if (attachNorth) {
                keys.add(new WindowKey(
                    worldKey,
                    windowOriginFromCoreOrigin(baseCore.add(0, 0, -REGION_LATTICE_STRIDE))
                ));
            }
            if (attachSouth) {
                keys.add(new WindowKey(
                    worldKey,
                    windowOriginFromCoreOrigin(baseCore.add(0, 0, REGION_LATTICE_STRIDE))
                ));
            }
            if (attachWest && attachNorth) {
                keys.add(new WindowKey(
                    worldKey,
                    windowOriginFromCoreOrigin(baseCore.add(-REGION_LATTICE_STRIDE, 0, -REGION_LATTICE_STRIDE))
                ));
            }
            if (attachWest && attachSouth) {
                keys.add(new WindowKey(
                    worldKey,
                    windowOriginFromCoreOrigin(baseCore.add(-REGION_LATTICE_STRIDE, 0, REGION_LATTICE_STRIDE))
                ));
            }
            if (attachEast && attachNorth) {
                keys.add(new WindowKey(
                    worldKey,
                    windowOriginFromCoreOrigin(baseCore.add(REGION_LATTICE_STRIDE, 0, -REGION_LATTICE_STRIDE))
                ));
            }
            if (attachEast && attachSouth) {
                keys.add(new WindowKey(
                    worldKey,
                    windowOriginFromCoreOrigin(baseCore.add(REGION_LATTICE_STRIDE, 0, REGION_LATTICE_STRIDE))
                ));
            }
            if (attachUp) {
                keys.add(new WindowKey(
                    worldKey,
                    windowOriginFromCoreOrigin(baseCore.add(0, REGION_LATTICE_STRIDE, 0))
                ));
            }
            if (attachDown) {
                keys.add(new WindowKey(
                    worldKey,
                    windowOriginFromCoreOrigin(baseCore.add(0, -REGION_LATTICE_STRIDE, 0))
                ));
            }
        }
        return keys;
    }

    private Set<WindowKey> solveRegionKeys(List<PlayerRegionAnchor> anchors) {
        Set<WindowKey> keys = new HashSet<>();
        for (PlayerRegionAnchor anchor : anchors) {
            RegistryKey<World> worldKey = anchor.worldKey();
            BlockPos baseCore = anchor.coreOrigin();
            keys.add(new WindowKey(worldKey, windowOriginFromCoreOrigin(baseCore)));
            int localY = MathHelper.clamp(anchor.blockPos().getY() - baseCore.getY(), 0, REGION_CORE_SIZE - 1);
            if (localY >= REGION_CORE_SIZE - VERTICAL_SOLVE_MARGIN_CELLS) {
                keys.add(new WindowKey(
                    worldKey,
                    windowOriginFromCoreOrigin(baseCore.add(0, REGION_LATTICE_STRIDE, 0))
                ));
            }
            if (localY < VERTICAL_SOLVE_MARGIN_CELLS) {
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

    private int patchCellIndex(int x, int y, int z, int size) {
        return (x * size + y) * size + z;
    }

    private int patchCellIndex3d(int x, int y, int z, int sizeY, int sizeZ) {
        return (x * sizeY + y) * sizeZ + z;
    }

    private int defaultInspectionPatchGridResolution(int domainBlocks) {
        return Math.min(INSPECTION_PATCH_MAX_GRID_RESOLUTION, domainBlocks * 2);
    }

    private boolean isValidInspectionPatchGridResolution(int domainBlocks, int gridResolution) {
        return gridResolution >= domainBlocks
            && gridResolution <= INSPECTION_PATCH_MAX_GRID_RESOLUTION
            && (gridResolution % domainBlocks) == 0;
    }

    private double inspectionPatchCellSizeBlocks(int domainBlocks, int gridResolution) {
        return (double) domainBlocks / (double) gridResolution;
    }

    private int worldToPatchCell(double worldCoord, int originCoord, int cellsPerBlock) {
        return (int) Math.floor((worldCoord - originCoord) * cellsPerBlock);
    }

    private double patchCellCenterWorld(int originCoord, int cell, int cellsPerBlock) {
        return originCoord + ((double) cell + 0.5) / (double) cellsPerBlock;
    }

    private BlockPos inspectionPatchOriginForFocus(ServerWorld world, BlockPos focus, int domainBlocks) {
        int half = domainBlocks / 2;
        int rawX = Math.floorDiv(focus.getX() - half, CHUNK_SIZE) * CHUNK_SIZE;
        int rawY = Math.floorDiv(focus.getY() - half, CHUNK_SIZE) * CHUNK_SIZE;
        int rawZ = Math.floorDiv(focus.getZ() - half, CHUNK_SIZE) * CHUNK_SIZE;
        int maxOriginY = world.getTopYInclusive() + 1 - domainBlocks;
        int clampedY = MathHelper.clamp(rawY, world.getBottomY(), maxOriginY);
        return new BlockPos(rawX, clampedY, rawZ);
    }

    private InspectionPatchStaticFields captureInspectionPatchStaticFields(
        ServerWorld world,
        BlockPos origin,
        int domainBlocks,
        int gridResolution,
        int cellsPerBlock
    ) {
        int cells = gridResolution * gridResolution * gridResolution;
        byte[] obstacle = new byte[cells];
        byte[] surfaceKind = new byte[cells];
        byte[] openFaceMask = new byte[cells];
        float[] emitterPowerWatts = new float[cells];
        byte[] faceSkyExposure = new byte[cells * FACE_COUNT];
        byte[] faceDirectExposure = new byte[cells * FACE_COUNT];
        BlockPos.Mutable cursor = new BlockPos.Mutable();
        double cellSizeBlocks = inspectionPatchCellSizeBlocks(domainBlocks, gridResolution);
        for (int x = 0; x < gridResolution; x++) {
            double sampleX = patchCellCenterWorld(origin.getX(), x, cellsPerBlock);
            for (int y = 0; y < gridResolution; y++) {
                double sampleY = patchCellCenterWorld(origin.getY(), y, cellsPerBlock);
                for (int z = 0; z < gridResolution; z++) {
                    double sampleZ = patchCellCenterWorld(origin.getZ(), z, cellsPerBlock);
                    int cell = patchCellIndex(x, y, z, gridResolution);
                    cursor.set(MathHelper.floor(sampleX), MathHelper.floor(sampleY), MathHelper.floor(sampleZ));
                    BlockState state = world.getBlockState(cursor);
                    boolean solid = isSolidObstacleAtPoint(world, cursor, state, sampleX, sampleY, sampleZ);
                    obstacle[cell] = solid ? (byte) 1 : (byte) 0;
                    ThermalMaterial material = thermalMaterial(state);
                    surfaceKind[cell] = material == null ? SURFACE_KIND_NONE : material.kind();
                    patchStaticThermalFieldsAtSample(
                        world,
                        cursor,
                        state,
                        sampleX,
                        sampleY,
                        sampleZ,
                        cellSizeBlocks,
                        cell,
                        material,
                        emitterPowerWatts,
                        openFaceMask,
                        faceSkyExposure,
                        faceDirectExposure
                    );
                }
            }
        }
        return new InspectionPatchStaticFields(
            obstacle,
            surfaceKind,
            openFaceMask,
            emitterPowerWatts,
            faceSkyExposure,
            faceDirectExposure
        );
    }

    private void patchStaticThermalFieldsAtSample(
        ServerWorld world,
        BlockPos pos,
        BlockState state,
        double sampleX,
        double sampleY,
        double sampleZ,
        double cellSizeBlocks,
        int cell,
        ThermalMaterial material,
        float[] emitterPowerWatts,
        byte[] openFaceMaskField,
        byte[] faceSkyExposure,
        byte[] faceDirectExposure
    ) {
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
            double neighborSampleX = sampleX + direction.getOffsetX() * cellSizeBlocks;
            double neighborSampleY = sampleY + direction.getOffsetY() * cellSizeBlocks;
            double neighborSampleZ = sampleZ + direction.getOffsetZ() * cellSizeBlocks;
            neighborCursor.set(
                MathHelper.floor(neighborSampleX),
                MathHelper.floor(neighborSampleY),
                MathHelper.floor(neighborSampleZ)
            );
            BlockState neighborState = world.getBlockState(neighborCursor);
            boolean openFace;
            if (material != null) {
                openFace = material.atmosphericExchangeRequiresAirNeighbor()
                    ? neighborState.isAir()
                    : !isSolidObstacleAtPoint(world, neighborCursor, neighborState, neighborSampleX, neighborSampleY, neighborSampleZ);
            } else {
                openFace = neighborState.isAir();
            }
            if (!openFace) {
                continue;
            }
            openFaceMask = setFaceBit(openFaceMask, direction);
            if (material != null && faceSkyExposure != null && faceDirectExposure != null) {
                int faceIndex = faceDataIndex(cell, direction);
                faceSkyExposure[faceIndex] = quantizeUnitFloat(sampleSkyExposure(world, neighborCursor));
                faceDirectExposure[faceIndex] = quantizeUnitFloat(sampleDirectSunExposure(world, neighborCursor));
            }
        }
        if (openFaceMaskField != null) {
            openFaceMaskField[cell] = openFaceMask;
        }
    }

    private void persistInspectionPatchDump(InspectionPatchInput input) throws IOException {
        persistInspectionPatchDump(
            input.outputDir(),
            input.worldKey(),
            input.focus(),
            input.origin(),
            input.domainBlocks(),
            input.gridResolution(),
            input.cellsPerBlock(),
            input.faceResolution(),
            input.environmentSnapshot(),
            input.fallbackBoundary(),
            input.thermalEnvironment(),
            input.boundaryField(),
            input.staticFields(),
            input.fans()
        );
    }

    private void persistInspectionPatchDump(
        Path outputDir,
        RegistryKey<World> worldKey,
        BlockPos focus,
        BlockPos origin,
        int domainBlocks,
        int gridResolution,
        int cellsPerBlock,
        int faceResolution,
        WorldEnvironmentSnapshot environmentSnapshot,
        NestedBoundaryCoupler.BoundarySample fallbackBoundary,
        ThermalEnvironment thermalEnvironment,
        BoundaryFieldData boundaryField,
        InspectionPatchStaticFields staticFields,
        List<FanSource> fans
    ) throws IOException {
        Path staticDir = outputDir.resolve("static");
        Path boundaryDir = outputDir.resolve("boundary");

        Path obstaclePath = staticDir.resolve("obstacle.mask");
        Path surfaceKindPath = staticDir.resolve("surface_kind.u8");
        Path openFaceMaskPath = staticDir.resolve("open_face_mask.u8");
        Path emitterPowerPath = staticDir.resolve("emitter_power.zfp");
        Path faceSkyExposurePath = staticDir.resolve("face_sky_exposure.u8");
        Path faceDirectExposurePath = staticDir.resolve("face_direct_exposure.u8");
        Path boundaryWindXPath = boundaryDir.resolve("wind_x.zfp");
        Path boundaryWindYPath = boundaryDir.resolve("wind_y.zfp");
        Path boundaryWindZPath = boundaryDir.resolve("wind_z.zfp");
        Path boundaryAirTemperaturePath = boundaryDir.resolve("air_temperature.zfp");

        Files.write(obstaclePath, staticFields.obstacle());
        Files.write(surfaceKindPath, staticFields.surfaceKind());
        Files.write(openFaceMaskPath, staticFields.openFaceMask());
        Files.write(faceSkyExposurePath, staticFields.faceSkyExposure());
        Files.write(faceDirectExposurePath, staticFields.faceDirectExposure());
        if (!writeCompressedFloatGridFile(
            emitterPowerPath,
            staticFields.emitterPowerWatts(),
            gridResolution,
            gridResolution,
            gridResolution,
            INSPECTION_PATCH_EMITTER_POWER_TOLERANCE
        )) {
            throw new IOException("Failed to compress emitter power field");
        }
        if (!writeCompressedFloatGridFile(
            boundaryWindXPath,
            boundaryField.windX(),
            FACE_COUNT,
            faceResolution,
            faceResolution,
            INSPECTION_PATCH_BOUNDARY_VELOCITY_TOLERANCE
        ) || !writeCompressedFloatGridFile(
            boundaryWindYPath,
            boundaryField.windY(),
            FACE_COUNT,
            faceResolution,
            faceResolution,
            INSPECTION_PATCH_BOUNDARY_VELOCITY_TOLERANCE
        ) || !writeCompressedFloatGridFile(
            boundaryWindZPath,
            boundaryField.windZ(),
            FACE_COUNT,
            faceResolution,
            faceResolution,
            INSPECTION_PATCH_BOUNDARY_VELOCITY_TOLERANCE
        ) || !writeCompressedFloatGridFile(
            boundaryAirTemperaturePath,
            boundaryField.airTemperatureKelvin(),
            FACE_COUNT,
            faceResolution,
            faceResolution,
            INSPECTION_PATCH_BOUNDARY_TEMPERATURE_TOLERANCE
        )) {
            throw new IOException("Failed to compress boundary field");
        }

        StringBuilder builder = new StringBuilder(1 << 15);
        builder.append("{\n");
        appendJsonField(builder, "format", "a4mc_inspection_patch_v2", true);
        appendJsonField(builder, "deprecated", true, true);
        appendJsonField(builder, "capture_mode", "monolithic_patch_input", true);
        appendJsonField(builder, "dimension_id", worldKey.getValue().toString(), true);
        appendJsonField(builder, "tick", tickCounter, true);
        appendJsonField(builder, "focus_x", focus.getX(), true);
        appendJsonField(builder, "focus_y", focus.getY(), true);
        appendJsonField(builder, "focus_z", focus.getZ(), true);
        appendJsonField(builder, "origin_x", origin.getX(), true);
        appendJsonField(builder, "origin_y", origin.getY(), true);
        appendJsonField(builder, "origin_z", origin.getZ(), true);
        appendJsonField(builder, "domain_blocks_x", domainBlocks, true);
        appendJsonField(builder, "domain_blocks_y", domainBlocks, true);
        appendJsonField(builder, "domain_blocks_z", domainBlocks, true);
        appendJsonField(builder, "grid_resolution_x", gridResolution, true);
        appendJsonField(builder, "grid_resolution_y", gridResolution, true);
        appendJsonField(builder, "grid_resolution_z", gridResolution, true);
        appendJsonField(builder, "size_x", gridResolution, true);
        appendJsonField(builder, "size_y", gridResolution, true);
        appendJsonField(builder, "size_z", gridResolution, true);
        appendJsonField(builder, "cells_per_block", cellsPerBlock, true);
        appendJsonField(builder, "cell_size_blocks", 1.0f / (float) cellsPerBlock, true);
        appendJsonField(builder, "obstacle_mask_encoding", "u8_raw_0_1", true);
        appendJsonField(builder, "surface_kind_encoding", "u8_enum", true);
        appendJsonField(builder, "open_face_mask_encoding", "u8_direction_bits", true);
        appendJsonField(builder, "face_exposure_encoding", "u8_unit_0_255", true);
        appendJsonField(builder, "boundary_face_resolution", faceResolution, true);
        appendJsonField(builder, "boundary_external_face_mask", boundaryField.externalFaceMask(), true);
        appendJsonField(builder, "boundary_velocity_tolerance", INSPECTION_PATCH_BOUNDARY_VELOCITY_TOLERANCE, true);
        appendJsonField(builder, "boundary_temperature_tolerance", INSPECTION_PATCH_BOUNDARY_TEMPERATURE_TOLERANCE, true);
        appendJsonField(builder, "emitter_power_tolerance", INSPECTION_PATCH_EMITTER_POWER_TOLERANCE, true);
        builder.append("  \"environment\": {\n");
        appendJsonField(builder, "time_of_day", environmentSnapshot.timeOfDay(), true, 4);
        appendJsonField(builder, "rain_gradient", environmentSnapshot.rainGradient(), true, 4);
        appendJsonField(builder, "thunder_gradient", environmentSnapshot.thunderGradient(), true, 4);
        appendJsonField(builder, "sea_level", environmentSnapshot.seaLevel(), false, 4);
        builder.append("  },\n");
        builder.append("  \"fallback_boundary_sample\": {\n");
        appendJsonField(builder, "wind_x", fallbackBoundary.windX(), true, 4);
        appendJsonField(builder, "wind_y", fallbackBoundary.windY(), true, 4);
        appendJsonField(builder, "wind_z", fallbackBoundary.windZ(), true, 4);
        appendJsonField(builder, "ambient_air_temperature_kelvin", fallbackBoundary.ambientAirTemperatureKelvin(), true, 4);
        appendJsonField(builder, "deep_ground_temperature_kelvin", fallbackBoundary.deepGroundTemperatureKelvin(), false, 4);
        builder.append("  },\n");
        builder.append("  \"thermal_environment\": {\n");
        appendJsonField(builder, "direct_solar_flux_wm2", thermalEnvironment.directSolarFluxWm2(), true, 4);
        appendJsonField(builder, "diffuse_solar_flux_wm2", thermalEnvironment.diffuseSolarFluxWm2(), true, 4);
        appendJsonField(builder, "ambient_air_temperature_kelvin", thermalEnvironment.ambientAirTemperatureKelvin(), true, 4);
        appendJsonField(builder, "deep_ground_temperature_kelvin", thermalEnvironment.deepGroundTemperatureKelvin(), true, 4);
        appendJsonField(builder, "sky_temperature_kelvin", thermalEnvironment.skyTemperatureKelvin(), true, 4);
        appendJsonField(builder, "precipitation_temperature_kelvin", thermalEnvironment.precipitationTemperatureKelvin(), true, 4);
        appendJsonField(builder, "precipitation_strength", thermalEnvironment.precipitationStrength(), true, 4);
        appendJsonField(builder, "sun_x", thermalEnvironment.sunX(), true, 4);
        appendJsonField(builder, "sun_y", thermalEnvironment.sunY(), true, 4);
        appendJsonField(builder, "sun_z", thermalEnvironment.sunZ(), true, 4);
        appendJsonField(builder, "surface_delta_seconds", thermalEnvironment.surfaceDeltaSeconds(), false, 4);
        builder.append("  },\n");
        builder.append("  \"static_files\": {\n");
        appendJsonField(builder, "obstacle", outputDir.relativize(obstaclePath).toString(), true, 4);
        appendJsonField(builder, "surface_kind", outputDir.relativize(surfaceKindPath).toString(), true, 4);
        appendJsonField(builder, "open_face_mask", outputDir.relativize(openFaceMaskPath).toString(), true, 4);
        appendJsonField(builder, "emitter_power", outputDir.relativize(emitterPowerPath).toString(), true, 4);
        appendJsonField(builder, "face_sky_exposure", outputDir.relativize(faceSkyExposurePath).toString(), true, 4);
        appendJsonField(builder, "face_direct_exposure", outputDir.relativize(faceDirectExposurePath).toString(), false, 4);
        builder.append("  },\n");
        builder.append("  \"boundary_files\": {\n");
        appendJsonField(builder, "wind_x", outputDir.relativize(boundaryWindXPath).toString(), true, 4);
        appendJsonField(builder, "wind_y", outputDir.relativize(boundaryWindYPath).toString(), true, 4);
        appendJsonField(builder, "wind_z", outputDir.relativize(boundaryWindZPath).toString(), true, 4);
        appendJsonField(builder, "air_temperature", outputDir.relativize(boundaryAirTemperaturePath).toString(), false, 4);
        builder.append("  },\n");
        builder.append("  \"fans\": [\n");
        for (int i = 0; i < fans.size(); i++) {
            FanSource fan = fans.get(i);
            builder.append("    {\n");
            appendJsonField(builder, "world_x", fan.pos().getX(), true, 6);
            appendJsonField(builder, "world_y", fan.pos().getY(), true, 6);
            appendJsonField(builder, "world_z", fan.pos().getZ(), true, 6);
            appendJsonField(builder, "local_x_blocks", fan.pos().getX() - origin.getX(), true, 6);
            appendJsonField(builder, "local_y_blocks", fan.pos().getY() - origin.getY(), true, 6);
            appendJsonField(builder, "local_z_blocks", fan.pos().getZ() - origin.getZ(), true, 6);
            appendJsonField(builder, "facing", fan.facing().asString(), true, 6);
            appendJsonField(builder, "duct_length", fan.ductLength(), false, 6);
            builder.append("    }");
            if (i + 1 < fans.size()) {
                builder.append(',');
            }
            builder.append('\n');
        }
        builder.append("  ]\n");
        builder.append("}\n");
        Files.writeString(outputDir.resolve("metadata.json"), builder.toString(), StandardCharsets.UTF_8);
    }

    private boolean writeCompressedFloatGridFile(
        Path path,
        float[] values,
        int nx,
        int ny,
        int nz,
        double tolerance
    ) throws IOException {
        byte[] compressed = simulationBridge.compressFloatGrid3d(values, nx, ny, nz, tolerance);
        if (compressed == null) {
            return false;
        }
        Files.write(path, compressed);
        return true;
    }

    private long simulationRegionKey(WindowKey key) {
        long value = 1469598103934665603L;
        value = (value ^ key.worldKey().getValue().hashCode()) * 1099511628211L;
        value = (value ^ key.origin().getX()) * 1099511628211L;
        value = (value ^ key.origin().getY()) * 1099511628211L;
        value = (value ^ key.origin().getZ()) * 1099511628211L;
        return value == 0L ? 1L : value;
    }

    private long simulationWorldKey(RegistryKey<World> worldKey) {
        long value = worldKey.getValue().hashCode();
        return value == 0L ? 1L : value;
    }

    private void syncBrickRuntimeHints(Map<RegistryKey<World>, int[]> hintCoordsByWorld) {
        if (simulationServiceId == 0L || !simulationBridge.isLoaded()) {
            return;
        }
        Set<RegistryKey<World>> activeWorldKeys = hintCoordsByWorld.keySet();
        for (RegistryKey<World> staleWorldKey : new HashSet<>(brickRuntimeHintWorldKeys)) {
            if (activeWorldKeys.contains(staleWorldKey)) {
                continue;
            }
            simulationBridge.setBrickWorldActiveHints(
                simulationServiceId,
                simulationWorldKey(staleWorldKey),
                BRICK_RUNTIME_SIZE,
                new int[0]
            );
        }
        Set<RegistryKey<World>> syncedWorldKeys = new HashSet<>();
        for (RegistryKey<World> worldKey : activeWorldKeys) {
            long worldRuntimeKey = simulationWorldKey(worldKey);
            if (!simulationBridge.ensureBrickWorldRuntime(
                simulationServiceId,
                worldRuntimeKey,
                BRICK_RUNTIME_SIZE,
                1.0f,
                SOLVER_STEP_SECONDS
            )) {
                continue;
            }
            brickRuntimeKnownWorldKeys.add(worldKey);
            int[] hintCoords = hintCoordsByWorld.getOrDefault(worldKey, new int[0]);
            if (simulationBridge.setBrickWorldActiveHints(
                simulationServiceId,
                worldRuntimeKey,
                BRICK_RUNTIME_SIZE,
                hintCoords
            )) {
                syncedWorldKeys.add(worldKey);
            }
        }
        brickRuntimeHintWorldKeys.clear();
        brickRuntimeHintWorldKeys.addAll(syncedWorldKeys);
        brickBoundaryReferenceRefreshTicks.keySet().removeIf(worldKey -> !activeWorldKeys.contains(worldKey));
        refreshBrickRuntimeBoundaryReferences(hintCoordsByWorld);
    }

    private void refreshBrickRuntimeBoundaryReferences(Map<RegistryKey<World>, int[]> hintCoordsByWorld) {
        if (simulationServiceId == 0L || !simulationBridge.isLoaded()) {
            return;
        }
        for (Map.Entry<RegistryKey<World>, int[]> entry : hintCoordsByWorld.entrySet()) {
            RegistryKey<World> worldKey = entry.getKey();
            Map<BrickRuntimeHint, Integer> refreshTicks = brickBoundaryReferenceRefreshTicks.computeIfAbsent(
                worldKey,
                ignored -> new HashMap<>()
            );
            Set<BrickRuntimeHint> closure = expandBrickHintClosure(entry.getValue());
            refreshTicks.keySet().removeIf(hint -> !closure.contains(hint));
            for (BrickRuntimeHint hint : closure) {
                int lastTick = refreshTicks.getOrDefault(hint, Integer.MIN_VALUE);
                if (tickCounter - lastTick < BRICK_BOUNDARY_REFERENCE_REFRESH_INTERVAL_TICKS) {
                    continue;
                }
                if (uploadBrickBoundaryReference(worldKey, hint)) {
                    refreshTicks.put(hint, tickCounter);
                }
            }
        }
    }

    private Set<BrickRuntimeHint> expandBrickHintClosure(int[] hintCoords) {
        LinkedHashSet<BrickRuntimeHint> closure = new LinkedHashSet<>();
        if (hintCoords == null) {
            return closure;
        }
        for (int i = 0; i + 2 < hintCoords.length; i += NativeSimulationBridge.BRICK_HINT_COORDS_PER_BRICK) {
            int brickX = hintCoords[i];
            int brickY = hintCoords[i + 1];
            int brickZ = hintCoords[i + 2];
            closure.add(new BrickRuntimeHint(brickX, brickY, brickZ));
            closure.add(new BrickRuntimeHint(brickX - 1, brickY, brickZ));
            closure.add(new BrickRuntimeHint(brickX + 1, brickY, brickZ));
            closure.add(new BrickRuntimeHint(brickX, brickY - 1, brickZ));
            closure.add(new BrickRuntimeHint(brickX, brickY + 1, brickZ));
            closure.add(new BrickRuntimeHint(brickX, brickY, brickZ - 1));
            closure.add(new BrickRuntimeHint(brickX, brickY, brickZ + 1));
        }
        return closure;
    }

    private boolean uploadBrickBoundaryReference(RegistryKey<World> worldKey, BrickRuntimeHint hint) {
        WorldMirror.SectionSnapshot[] snapshots = brickSectionSnapshotsFromMirror(
            worldKey,
            hint.brickX(),
            hint.brickY(),
            hint.brickZ()
        );
        byte[] obstacleMask = buildBrickObstacleMask(snapshots, false);
        if (obstacleMask == null) {
            obstacleMask = new byte[BRICK_RUNTIME_SIZE * BRICK_RUNTIME_SIZE * BRICK_RUNTIME_SIZE];
        }
        int cells = BRICK_RUNTIME_SIZE * BRICK_RUNTIME_SIZE * BRICK_RUNTIME_SIZE;
        float[] flowState = new float[cells * RESPONSE_CHANNELS];
        float[] airTemperatureState = new float[cells];
        float[] surfaceTemperatureState = new float[cells];
        BlockPos origin = new BlockPos(
            hint.brickX() * BRICK_RUNTIME_SIZE,
            hint.brickY() * BRICK_RUNTIME_SIZE,
            hint.brickZ() * BRICK_RUNTIME_SIZE
        );
        MesoscaleGrid mesoscaleGrid = mesoscaleMetGrids.get(worldKey);
        boolean seeded = mesoscaleGrid != null
            && seedDynamicRegionFromMesoscale(
                origin,
                BRICK_RUNTIME_SIZE,
                BRICK_RUNTIME_SIZE,
                BRICK_RUNTIME_SIZE,
                obstacleMask,
                flowState,
                airTemperatureState,
                surfaceTemperatureState,
                mesoscaleGrid
            );
        if (!seeded) {
            seedDynamicRegionFromBoundarySample(
                worldKey,
                origin,
                BRICK_RUNTIME_SIZE,
                BRICK_RUNTIME_SIZE,
                BRICK_RUNTIME_SIZE,
                obstacleMask,
                flowState,
                airTemperatureState,
                surfaceTemperatureState
            );
        }
        return simulationBridge.uploadBrickWorldBoundaryReferenceBrick(
            simulationServiceId,
            simulationWorldKey(worldKey),
            BRICK_RUNTIME_SIZE,
            hint.brickX(),
            hint.brickY(),
            hint.brickZ(),
            flowState,
            airTemperatureState,
            surfaceTemperatureState
        );
    }

    private Map<RegistryKey<World>, int[]> brickRuntimeHintCoords(List<PlayerRegionAnchor> anchors) {
        Map<RegistryKey<World>, LinkedHashSet<BrickRuntimeHint>> hintsByWorld = new HashMap<>();
        for (WindowKey solveKey : solveRegionKeys(anchors)) {
            RegistryKey<World> worldKey = solveKey.worldKey();
            BlockPos coreOrigin = solveKey.origin().add(REGION_HALO_CELLS, REGION_HALO_CELLS, REGION_HALO_CELLS);
            LinkedHashSet<BrickRuntimeHint> hints = hintsByWorld.computeIfAbsent(worldKey, ignored -> new LinkedHashSet<>());
            hints.add(new BrickRuntimeHint(
                Math.floorDiv(coreOrigin.getX(), BRICK_RUNTIME_SIZE),
                Math.floorDiv(coreOrigin.getY(), BRICK_RUNTIME_SIZE),
                Math.floorDiv(coreOrigin.getZ(), BRICK_RUNTIME_SIZE)
            ));
        }
        if (hintsByWorld.isEmpty()) {
            for (PlayerRegionAnchor anchor : anchors) {
                RegistryKey<World> worldKey = anchor.worldKey();
                BlockPos baseCore = anchor.coreOrigin();
                hintsByWorld.computeIfAbsent(worldKey, ignored -> new LinkedHashSet<>()).add(new BrickRuntimeHint(
                    Math.floorDiv(baseCore.getX(), BRICK_RUNTIME_SIZE),
                    Math.floorDiv(baseCore.getY(), BRICK_RUNTIME_SIZE),
                    Math.floorDiv(baseCore.getZ(), BRICK_RUNTIME_SIZE)
                ));
            }
        }
        Map<RegistryKey<World>, int[]> coordsByWorld = new HashMap<>();
        for (Map.Entry<RegistryKey<World>, LinkedHashSet<BrickRuntimeHint>> entry : hintsByWorld.entrySet()) {
            int[] coords = new int[entry.getValue().size() * NativeSimulationBridge.BRICK_HINT_COORDS_PER_BRICK];
            int index = 0;
            for (BrickRuntimeHint hint : entry.getValue()) {
                coords[index++] = hint.brickX();
                coords[index++] = hint.brickY();
                coords[index++] = hint.brickZ();
            }
            coordsByWorld.put(entry.getKey(), coords);
        }
        return Map.copyOf(coordsByWorld);
    }

    private void stepBrickRuntimeWorlds() {
        if (simulationServiceId == 0L || !simulationBridge.isLoaded()) {
            return;
        }
        for (RegistryKey<World> worldKey : new HashSet<>(brickRuntimeKnownWorldKeys)) {
            simulationBridge.stepBrickWorldRuntime(
                simulationServiceId,
                simulationWorldKey(worldKey),
                1
            );
        }
    }

    private void appendBrickWindowCoords(Set<WindowKey> windows, RegistryKey<World> worldKey, int[] brickCoords) {
        if (brickCoords == null) {
            return;
        }
        for (int i = 0; i + 2 < brickCoords.length; i += NativeSimulationBridge.BRICK_HINT_COORDS_PER_BRICK) {
            BlockPos coreOrigin = new BlockPos(
                brickCoords[i] * BRICK_RUNTIME_SIZE,
                brickCoords[i + 1] * BRICK_RUNTIME_SIZE,
                brickCoords[i + 2] * BRICK_RUNTIME_SIZE
            );
            windows.add(new WindowKey(worldKey, windowOriginFromCoreOrigin(coreOrigin)));
        }
    }

    private void refreshDesiredWindowsFromBrickRuntime() {
        Set<WindowKey> solveWindows = new HashSet<>(anchorSolveWindowKeys);
        Set<WindowKey> desiredWindows = new HashSet<>(anchorDesiredWindowKeys);
        desiredWindows.addAll(solveWindows);
        desiredSolveWindowKeys = Set.copyOf(solveWindows);
        desiredWindowKeys = Set.copyOf(desiredWindows);
        int retainUntilTick = tickCounter + SHELL_WINDOW_RETENTION_TICKS;
        shellWindowRetainUntilTick.entrySet().removeIf(entry -> entry.getValue() < tickCounter);
        for (WindowKey key : desiredWindowKeys) {
            if (!desiredSolveWindowKeys.contains(key)) {
                shellWindowRetainUntilTick.put(key, retainUntilTick);
            } else {
                shellWindowRetainUntilTick.remove(key);
            }
        }
        for (WindowKey key : desiredWindowKeys) {
            if (!desiredSolveWindowKeys.contains(key)) {
                removePublishedRegionAtlas(key);
            }
        }
    }

    private List<NestedFeedbackAxisSpan> buildHorizontalNestedFeedbackSpans(
        int worldMinInclusive,
        int worldMaxExclusive,
        int localOrigin,
        int coarseCellSize
    ) {
        List<NestedFeedbackAxisSpan> spans = new ArrayList<>(2);
        int spanStartWorld = worldMinInclusive;
        int currentCell = Math.floorDiv(worldMinInclusive, coarseCellSize);
        for (int world = worldMinInclusive + 1; world < worldMaxExclusive; world++) {
            int cell = Math.floorDiv(world, coarseCellSize);
            if (cell == currentCell) {
                continue;
            }
            spans.add(new NestedFeedbackAxisSpan(currentCell, spanStartWorld - localOrigin, world - localOrigin));
            spanStartWorld = world;
            currentCell = cell;
        }
        spans.add(new NestedFeedbackAxisSpan(currentCell, spanStartWorld - localOrigin, worldMaxExclusive - localOrigin));
        return spans;
    }

    private List<NestedFeedbackAxisSpan> buildVerticalNestedFeedbackSpans(
        int worldMinInclusive,
        int worldMaxExclusive,
        int localOrigin,
        int verticalBaseY
    ) {
        List<NestedFeedbackAxisSpan> spans = new ArrayList<>(2);
        int spanStartWorld = worldMinInclusive;
        int currentLayer = MathHelper.clamp(
            Math.floorDiv(worldMinInclusive - verticalBaseY, MESOSCALE_MET_LAYER_HEIGHT_BLOCKS),
            0,
            MESOSCALE_MET_MAX_LAYERS - 1
        );
        for (int world = worldMinInclusive + 1; world < worldMaxExclusive; world++) {
            int layer = MathHelper.clamp(
                Math.floorDiv(world - verticalBaseY, MESOSCALE_MET_LAYER_HEIGHT_BLOCKS),
                0,
                MESOSCALE_MET_MAX_LAYERS - 1
            );
            if (layer == currentLayer) {
                continue;
            }
            spans.add(new NestedFeedbackAxisSpan(currentLayer, spanStartWorld - localOrigin, world - localOrigin));
            spanStartWorld = world;
            currentLayer = layer;
        }
        spans.add(new NestedFeedbackAxisSpan(currentLayer, spanStartWorld - localOrigin, worldMaxExclusive - localOrigin));
        return spans;
    }

    private L2ToL1FeedbackLayout buildRegionNestedFeedbackLayout(WindowKey key) {
        ServerWorld world = resolveWorld(key.worldKey());
        int verticalBaseY = world == null ? 0 : Math.max(0, world.getBottomY());
        int coreMinX = key.origin().getX() + REGION_HALO_CELLS;
        int coreMinY = key.origin().getY() + REGION_HALO_CELLS;
        int coreMinZ = key.origin().getZ() + REGION_HALO_CELLS;
        int coreMaxX = coreMinX + REGION_CORE_SIZE;
        int coreMaxY = coreMinY + REGION_CORE_SIZE;
        int coreMaxZ = coreMinZ + REGION_CORE_SIZE;

        List<NestedFeedbackAxisSpan> xSpans = buildHorizontalNestedFeedbackSpans(
            coreMinX,
            coreMaxX,
            key.origin().getX(),
            MESOSCALE_MET_CELL_SIZE_BLOCKS
        );
        List<NestedFeedbackAxisSpan> ySpans = buildVerticalNestedFeedbackSpans(
            coreMinY,
            coreMaxY,
            key.origin().getY(),
            verticalBaseY
        );
        List<NestedFeedbackAxisSpan> zSpans = buildHorizontalNestedFeedbackSpans(
            coreMinZ,
            coreMaxZ,
            key.origin().getZ(),
            MESOSCALE_MET_CELL_SIZE_BLOCKS
        );

        int binCount = xSpans.size() * ySpans.size() * zSpans.size();
        if (binCount <= 0 || binCount > NativeSimulationBridge.NESTED_FEEDBACK_MAX_BINS) {
            return null;
        }
        int[] nativeLayout = new int[binCount * NativeSimulationBridge.NESTED_FEEDBACK_LAYOUT_INTS_PER_BIN];
        List<L2ToL1FeedbackLayoutBin> bins = new ArrayList<>(binCount);
        int index = 0;
        for (NestedFeedbackAxisSpan xSpan : xSpans) {
            for (NestedFeedbackAxisSpan ySpan : ySpans) {
                for (NestedFeedbackAxisSpan zSpan : zSpans) {
                    L2ToL1FeedbackLayoutBin bin = new L2ToL1FeedbackLayoutBin(
                        xSpan.index(),
                        ySpan.index(),
                        zSpan.index(),
                        xSpan.localMin(),
                        xSpan.localMax(),
                        ySpan.localMin(),
                        ySpan.localMax(),
                        zSpan.localMin(),
                        zSpan.localMax()
                    );
                    bins.add(bin);
                    int base = index * NativeSimulationBridge.NESTED_FEEDBACK_LAYOUT_INTS_PER_BIN;
                    nativeLayout[base] = bin.cellX();
                    nativeLayout[base + 1] = bin.layer();
                    nativeLayout[base + 2] = bin.cellZ();
                    nativeLayout[base + 3] = bin.localMinX();
                    nativeLayout[base + 4] = bin.localMaxX();
                    nativeLayout[base + 5] = bin.localMinY();
                    nativeLayout[base + 6] = bin.localMaxY();
                    nativeLayout[base + 7] = bin.localMinZ();
                    nativeLayout[base + 8] = bin.localMaxZ();
                    index++;
                }
            }
        }
        return new L2ToL1FeedbackLayout(nativeLayout, List.copyOf(bins));
    }

    private void ensureRegionNestedFeedbackLayoutInSimulation(WindowKey key, RegionRecord region) {
        if (simulationServiceId == 0L) {
            return;
        }
        if (region.nestedFeedbackLayout == null) {
            region.nestedFeedbackLayout = buildRegionNestedFeedbackLayout(key);
        }
        if (region.nestedFeedbackLayout == null) {
            return;
        }
        if (region.nestedFeedbackLayoutServiceId == simulationServiceId) {
            return;
        }
        if (simulationBridge.setRegionNestedFeedbackLayout(
            simulationServiceId,
            simulationRegionKey(key),
            L2_TO_L1_FEEDBACK_STEPS,
            region.nestedFeedbackLayout.nativeLayout()
        )) {
            region.nestedFeedbackLayoutServiceId = simulationServiceId;
        }
    }

    private void pollRegionNestedFeedback(WindowKey key, RegionRecord region) {
        if (simulationServiceId == 0L || region.nestedFeedbackLayout == null) {
            return;
        }
        List<L2ToL1FeedbackLayoutBin> bins = region.nestedFeedbackLayout.bins();
        if (bins.isEmpty()) {
            return;
        }
        float[] values = new float[bins.size() * NativeSimulationBridge.NESTED_FEEDBACK_VALUES_PER_BIN];
        if (!simulationBridge.pollRegionNestedFeedback(simulationServiceId, simulationRegionKey(key), values)) {
            return;
        }
        float scale = 1.0f / L2_TO_L1_FEEDBACK_STEPS;
        ConcurrentLinkedQueue<MesoscaleGrid.NestedFeedbackBin> queue = pendingNestedFeedbackBins.computeIfAbsent(
            key.worldKey(),
            ignored -> new ConcurrentLinkedQueue<>()
        );
        float volumeAverageSum = 0.0f;
        float bottomFluxDensitySum = 0.0f;
        float topFluxDensitySum = 0.0f;
        int packetBinCount = 0;
        for (int i = 0; i < bins.size(); i++) {
            int base = i * NativeSimulationBridge.NESTED_FEEDBACK_VALUES_PER_BIN;
            float volumeAverage = values[base] * scale;
            float densityAverage = values[base + 1] * scale;
            float momentumXAverage = values[base + 2] * scale;
            float momentumZAverage = values[base + 3] * scale;
            float airTemperatureVolumeAverage = values[base + 4] * scale;
            float surfaceTemperatureVolumeAverage = values[base + 5] * scale;
            float bottomAreaAverage = values[base + 6] * scale;
            float bottomMassFluxAverage = values[base + 7] * scale;
            float topAreaAverage = values[base + 8] * scale;
            float topMassFluxAverage = values[base + 9] * scale;
            if (!Float.isFinite(volumeAverage)
                || !Float.isFinite(densityAverage)
                || !Float.isFinite(momentumXAverage)
                || !Float.isFinite(momentumZAverage)
                || !Float.isFinite(airTemperatureVolumeAverage)
                || !Float.isFinite(surfaceTemperatureVolumeAverage)
                || !Float.isFinite(bottomAreaAverage)
                || !Float.isFinite(bottomMassFluxAverage)
                || !Float.isFinite(topAreaAverage)
                || !Float.isFinite(topMassFluxAverage)) {
                continue;
            }
            if (!(volumeAverage > 0.0f) && !(bottomAreaAverage > 0.0f) && !(topAreaAverage > 0.0f)) {
                continue;
            }
            float bottomFluxDensity = bottomAreaAverage > 0.0f ? bottomMassFluxAverage / bottomAreaAverage : 0.0f;
            float topFluxDensity = topAreaAverage > 0.0f ? topMassFluxAverage / topAreaAverage : 0.0f;
            if (!Float.isFinite(bottomFluxDensity) || !Float.isFinite(topFluxDensity)) {
                continue;
            }
            L2ToL1FeedbackLayoutBin bin = bins.get(i);
            queue.add(new MesoscaleGrid.NestedFeedbackBin(
                bin.cellX(),
                bin.layer(),
                bin.cellZ(),
                volumeAverage,
                densityAverage,
                momentumXAverage,
                momentumZAverage,
                airTemperatureVolumeAverage,
                surfaceTemperatureVolumeAverage,
                bottomAreaAverage,
                bottomMassFluxAverage,
                topAreaAverage,
                topMassFluxAverage
            ));
            packetBinCount++;
            volumeAverageSum += volumeAverage;
            bottomFluxDensitySum += bottomFluxDensity;
            topFluxDensitySum += topFluxDensity;
        }
        if (packetBinCount > 0) {
            final int packetBinCountFinal = packetBinCount;
            final float invPacketBinCount = 1.0f / packetBinCountFinal;
            final float meanVolumeAverage = volumeAverageSum * invPacketBinCount;
            final float meanBottomFluxDensity = bottomFluxDensitySum * invPacketBinCount;
            final float meanTopFluxDensity = topFluxDensitySum * invPacketBinCount;
            nestedFeedbackRuntimeDiagnostics.compute(key.worldKey(), (ignored, previous) -> new NestedFeedbackRuntimeDiagnostics(
                (previous == null ? 0L : previous.polledPacketCount()) + 1L,
                (previous == null ? 0L : previous.polledBinCount()) + packetBinCountFinal,
                packetBinCountFinal,
                tickCounter,
                meanVolumeAverage,
                meanBottomFluxDensity,
                meanTopFluxDensity
            ));
        }
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
                        region.setSection(sx, sy, sz, null);
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
                    region.setSection(sx, sy, sz, snapshot);
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
                    if (isCoreSection(sx, sy, sz)) {
                        region.markBackendResetPending();
                    }
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

    private boolean uploadSolveWindowCoreStaticBrickToRuntime(WindowKey key, RegionRecord region) {
        if (simulationServiceId == 0L || region.sections == null) {
            return false;
        }
        WorldMirror.SectionSnapshot[] coreSnapshots = coreSectionSnapshotsFromRegion(region);
        long signature = coreStaticBrickSignature(coreSnapshots);
        if (signature == Long.MIN_VALUE) {
            return false;
        }
        if (region.uploadedBrickStaticServiceId == simulationServiceId
            && region.uploadedBrickStaticSignature == signature) {
            return true;
        }
        boolean uploaded = uploadWindowCoreStaticBrickToRuntime(key, coreSnapshots);
        if (uploaded) {
            region.uploadedBrickStaticSignature = signature;
            region.uploadedBrickStaticServiceId = simulationServiceId;
            ensureWindowCoreDynamicBrickSeeded(key, coreSnapshots);
        }
        return uploaded;
    }

    private void prewarmMirrorOnlyWindowCoreStaticBrick(WindowKey key) {
        if (simulationServiceId == 0L || !simulationBridge.isLoaded() || desiredSolveWindowKeys.contains(key)) {
            return;
        }
        WorldMirror.SectionSnapshot[] coreSnapshots = coreSectionSnapshotsFromMirror(key, true);
        if (coreSnapshots == null) {
            return;
        }
        long signature = coreStaticBrickSignature(coreSnapshots);
        if (signature == Long.MIN_VALUE) {
            return;
        }
        if (Long.valueOf(signature).equals(mirrorOnlyUploadedBrickStaticSignatures.get(key))) {
            return;
        }
        if (uploadWindowCoreStaticBrickToRuntime(key, coreSnapshots)) {
            mirrorOnlyUploadedBrickStaticSignatures.put(key, signature);
            ensureWindowCoreDynamicBrickSeeded(key, coreSnapshots);
        }
    }

    private void ensureWindowCoreDynamicBrickSeeded(WindowKey key, WorldMirror.SectionSnapshot[] coreSnapshots) {
        if (simulationServiceId == 0L || !simulationBridge.isLoaded() || coreSnapshots == null || coreSnapshots.length != CORE_SECTION_COUNT) {
            return;
        }
        long signature = coreStaticBrickSignature(coreSnapshots);
        if (signature != Long.MIN_VALUE
            && Long.valueOf(signature).equals(uploadedBrickDynamicSeedSignatures.get(key))) {
            return;
        }
        BlockPos coreOrigin = key.origin().add(REGION_HALO_CELLS, REGION_HALO_CELLS, REGION_HALO_CELLS);
        int brickX = Math.floorDiv(coreOrigin.getX(), BRICK_RUNTIME_SIZE);
        int brickY = Math.floorDiv(coreOrigin.getY(), BRICK_RUNTIME_SIZE);
        int brickZ = Math.floorDiv(coreOrigin.getZ(), BRICK_RUNTIME_SIZE);
        int brickCells = BRICK_RUNTIME_SIZE * BRICK_RUNTIME_SIZE * BRICK_RUNTIME_SIZE;
        byte[] obstacleMask = buildCoreBrickObstacleMask(coreSnapshots);
        if (obstacleMask == null) {
            return;
        }
        float[] flowState = new float[brickCells * RESPONSE_CHANNELS];
        float[] airTemperatureState = new float[brickCells];
        float[] surfaceTemperatureState = new float[brickCells];
        MesoscaleGrid mesoscaleGrid = mesoscaleMetGrids.get(key.worldKey());
        boolean seeded = mesoscaleGrid != null
            && seedDynamicRegionFromMesoscale(
                coreOrigin,
                BRICK_RUNTIME_SIZE,
                BRICK_RUNTIME_SIZE,
                BRICK_RUNTIME_SIZE,
                obstacleMask,
                flowState,
                airTemperatureState,
                surfaceTemperatureState,
                mesoscaleGrid
            );
        if (!seeded) {
            seedDynamicRegionFromBoundarySample(
                key.worldKey(),
                coreOrigin,
                BRICK_RUNTIME_SIZE,
                BRICK_RUNTIME_SIZE,
                BRICK_RUNTIME_SIZE,
                obstacleMask,
                flowState,
                airTemperatureState,
                surfaceTemperatureState
            );
        }
        simulationBridge.uploadBrickWorldDynamicBrick(
            simulationServiceId,
            simulationWorldKey(key.worldKey()),
            BRICK_RUNTIME_SIZE,
            brickX,
            brickY,
            brickZ,
            flowState,
            airTemperatureState,
            surfaceTemperatureState
        );
        if (signature != Long.MIN_VALUE) {
            uploadedBrickDynamicSeedSignatures.put(key, signature);
        }
    }

    private boolean uploadWindowCoreStaticBrickToRuntime(
        WindowKey key,
        WorldMirror.SectionSnapshot[] coreSnapshots
    ) {
        if (simulationServiceId == 0L || coreSnapshots == null || coreSnapshots.length != CORE_SECTION_COUNT) {
            return false;
        }
        BlockPos coreOrigin = key.origin().add(REGION_HALO_CELLS, REGION_HALO_CELLS, REGION_HALO_CELLS);
        return uploadBrickStaticSnapshotsToRuntime(
            key.worldKey(),
            Math.floorDiv(coreOrigin.getX(), BRICK_RUNTIME_SIZE),
            Math.floorDiv(coreOrigin.getY(), BRICK_RUNTIME_SIZE),
            Math.floorDiv(coreOrigin.getZ(), BRICK_RUNTIME_SIZE),
            coreSnapshots,
            false,
            false
        );
    }

    private boolean uploadResidentBrickStaticFromMirror(
        RegistryKey<World> worldKey,
        int brickX,
        int brickY,
        int brickZ
    ) {
        WorldMirror.SectionSnapshot[] snapshots = brickSectionSnapshotsFromMirror(worldKey, brickX, brickY, brickZ);
        return uploadBrickStaticSnapshotsToRuntime(worldKey, brickX, brickY, brickZ, snapshots, true, true);
    }

    private byte[] buildCoreBrickObstacleMask(WorldMirror.SectionSnapshot[] coreSnapshots) {
        return buildBrickObstacleMask(coreSnapshots, true);
    }

    private byte[] buildBrickObstacleMask(WorldMirror.SectionSnapshot[] snapshots, boolean fillMissingAsSolid) {
        if (snapshots == null) {
            return null;
        }
        int sectionAxisCount = BRICK_RUNTIME_SIZE / CHUNK_SIZE;
        if (snapshots.length != sectionAxisCount * sectionAxisCount * sectionAxisCount) {
            return null;
        }
        byte[] obstacle = new byte[BRICK_RUNTIME_SIZE * BRICK_RUNTIME_SIZE * BRICK_RUNTIME_SIZE];
        int snapshotIndex = 0;
        for (int sx = 0; sx < sectionAxisCount; sx++) {
            for (int sy = 0; sy < sectionAxisCount; sy++) {
                for (int sz = 0; sz < sectionAxisCount; sz++) {
                    WorldMirror.SectionSnapshot snapshot = snapshots[snapshotIndex++];
                    int baseX = sx * CHUNK_SIZE;
                    int baseY = sy * CHUNK_SIZE;
                    int baseZ = sz * CHUNK_SIZE;
                    if (snapshot == null) {
                        if (!fillMissingAsSolid) {
                            return null;
                        }
                        fillBrickSectionAsSolid(baseX, baseY, baseZ, obstacle);
                    } else {
                        writeSectionSnapshotIntoBrickObstacleBuffer(snapshot, baseX, baseY, baseZ, obstacle);
                    }
                }
            }
        }
        return obstacle;
    }

    private WorldMirror.SectionSnapshot[] brickSectionSnapshotsFromMirror(
        RegistryKey<World> worldKey,
        int brickX,
        int brickY,
        int brickZ
    ) {
        int sectionAxisCount = BRICK_RUNTIME_SIZE / CHUNK_SIZE;
        WorldMirror.SectionSnapshot[] snapshots = new WorldMirror.SectionSnapshot[sectionAxisCount * sectionAxisCount * sectionAxisCount];
        int index = 0;
        int originX = brickX * BRICK_RUNTIME_SIZE;
        int originY = brickY * BRICK_RUNTIME_SIZE;
        int originZ = brickZ * BRICK_RUNTIME_SIZE;
        for (int sx = 0; sx < sectionAxisCount; sx++) {
            for (int sy = 0; sy < sectionAxisCount; sy++) {
                for (int sz = 0; sz < sectionAxisCount; sz++) {
                    snapshots[index++] = worldMirror.peekSection(
                        worldKey,
                        new BlockPos(
                            originX + sx * CHUNK_SIZE,
                            originY + sy * CHUNK_SIZE,
                            originZ + sz * CHUNK_SIZE
                        )
                    );
                }
            }
        }
        return snapshots;
    }

    private boolean uploadBrickStaticSnapshotsToRuntime(
        RegistryKey<World> worldKey,
        int brickX,
        int brickY,
        int brickZ,
        WorldMirror.SectionSnapshot[] snapshots,
        boolean fillMissingAsSolid,
        boolean deferred
    ) {
        if (simulationServiceId == 0L || snapshots == null) {
            return false;
        }
        long worldRuntimeKey = simulationWorldKey(worldKey);
        if (!simulationBridge.ensureBrickWorldRuntime(
            simulationServiceId,
            worldRuntimeKey,
            BRICK_RUNTIME_SIZE,
            1.0f,
            SOLVER_STEP_SECONDS
        )) {
            return false;
        }
        int sectionAxisCount = BRICK_RUNTIME_SIZE / CHUNK_SIZE;
        if (snapshots.length != sectionAxisCount * sectionAxisCount * sectionAxisCount) {
            return false;
        }
        int cells = BRICK_RUNTIME_SIZE * BRICK_RUNTIME_SIZE * BRICK_RUNTIME_SIZE;
        byte[] obstacle = new byte[cells];
        byte[] surfaceKind = new byte[cells];
        short[] openFaceMask = new short[cells];
        float[] emitterPower = new float[cells];
        byte[] faceSkyExposure = new byte[cells * FACE_COUNT];
        byte[] faceDirectExposure = new byte[cells * FACE_COUNT];
        int snapshotIndex = 0;
        for (int sx = 0; sx < sectionAxisCount; sx++) {
            for (int sy = 0; sy < sectionAxisCount; sy++) {
                for (int sz = 0; sz < sectionAxisCount; sz++) {
                    WorldMirror.SectionSnapshot snapshot = snapshots[snapshotIndex++];
                    int baseX = sx * CHUNK_SIZE;
                    int baseY = sy * CHUNK_SIZE;
                    int baseZ = sz * CHUNK_SIZE;
                    if (snapshot == null) {
                        if (!fillMissingAsSolid) {
                            return false;
                        }
                        fillBrickSectionAsSolid(baseX, baseY, baseZ, obstacle);
                        continue;
                    }
                    writeSectionSnapshotIntoBrickBuffers(
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
                }
            }
        }
        return deferred
            ? simulationBridge.queueBrickWorldStaticBrickUpload(
                simulationServiceId,
                worldRuntimeKey,
                BRICK_RUNTIME_SIZE,
                brickX,
                brickY,
                brickZ,
                obstacle,
                surfaceKind,
                openFaceMask,
                emitterPower,
                faceSkyExposure,
                faceDirectExposure
            )
            : simulationBridge.uploadBrickWorldStaticBrick(
                simulationServiceId,
                worldRuntimeKey,
                BRICK_RUNTIME_SIZE,
                brickX,
                brickY,
                brickZ,
                obstacle,
                surfaceKind,
                openFaceMask,
                emitterPower,
                faceSkyExposure,
                faceDirectExposure
            );
    }

    private void fillBrickSectionAsSolid(int baseX, int baseY, int baseZ, byte[] obstacle) {
        for (int lx = 0; lx < CHUNK_SIZE; lx++) {
            int x = baseX + lx;
            for (int ly = 0; ly < CHUNK_SIZE; ly++) {
                int y = baseY + ly;
                for (int lz = 0; lz < CHUNK_SIZE; lz++) {
                    int z = baseZ + lz;
                    obstacle[patchCellIndex(x, y, z, BRICK_RUNTIME_SIZE)] = 1;
                }
            }
        }
    }

    private WorldMirror.SectionSnapshot[] coreSectionSnapshotsFromRegion(RegionRecord region) {
        if (region.sections == null) {
            return null;
        }
        WorldMirror.SectionSnapshot[] snapshots = new WorldMirror.SectionSnapshot[CORE_SECTION_COUNT];
        int index = 0;
        for (int sx = CORE_SECTION_MIN; sx <= CORE_SECTION_MAX; sx++) {
            for (int sy = CORE_SECTION_MIN; sy <= CORE_SECTION_MAX; sy++) {
                for (int sz = CORE_SECTION_MIN; sz <= CORE_SECTION_MAX; sz++) {
                    snapshots[index++] = region.sectionAt(sx, sy, sz);
                }
            }
        }
        return snapshots;
    }

    private WorldMirror.SectionSnapshot[] coreSectionSnapshotsFromMirror(WindowKey key, boolean requestMissing) {
        WorldMirror.SectionSnapshot[] snapshots = new WorldMirror.SectionSnapshot[CORE_SECTION_COUNT];
        int index = 0;
        for (int sx = CORE_SECTION_MIN; sx <= CORE_SECTION_MAX; sx++) {
            for (int sy = CORE_SECTION_MIN; sy <= CORE_SECTION_MAX; sy++) {
                for (int sz = CORE_SECTION_MIN; sz <= CORE_SECTION_MAX; sz++) {
                    WorldMirror.SectionSnapshot snapshot = worldMirror.peekSection(
                        key.worldKey(),
                        sectionOrigin(key.origin(), sx, sy, sz)
                    );
                    if (snapshot == null) {
                        if (requestMissing) {
                            requestWindowSectionIfNeeded(key, sx, sy, sz);
                        }
                        return null;
                    }
                    snapshots[index++] = snapshot;
                }
            }
        }
        return snapshots;
    }

    private long coreStaticBrickSignature(WorldMirror.SectionSnapshot[] coreSnapshots) {
        if (coreSnapshots == null || coreSnapshots.length != CORE_SECTION_COUNT) {
            return Long.MIN_VALUE;
        }
        long value = 1469598103934665603L;
        for (WorldMirror.SectionSnapshot snapshot : coreSnapshots) {
            if (snapshot == null) {
                return Long.MIN_VALUE;
            }
            value = (value ^ snapshot.version()) * 1099511628211L;
        }
        return value == 0L ? 1L : value;
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

    private void writeSectionSnapshotIntoBrickBuffers(
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
                    int cell = patchCellIndex(x, y, z, BRICK_RUNTIME_SIZE);
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

    private void writeSectionSnapshotIntoBrickObstacleBuffer(
        WorldMirror.SectionSnapshot snapshot,
        int baseX,
        int baseY,
        int baseZ,
        byte[] obstacle
    ) {
        for (int lx = 0; lx < CHUNK_SIZE; lx++) {
            int x = baseX + lx;
            for (int ly = 0; ly < CHUNK_SIZE; ly++) {
                int y = baseY + ly;
                for (int lz = 0; lz < CHUNK_SIZE; lz++) {
                    int z = baseZ + lz;
                    int local = localSectionCellIndex(lx, ly, lz);
                    int cell = patchCellIndex(x, y, z, BRICK_RUNTIME_SIZE);
                    obstacle[cell] = snapshot.obstacle()[local] >= 0.5f ? (byte) 1 : (byte) 0;
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

    private boolean ensureWindowDynamicRegionInitialized(WindowKey key, RegionRecord region) {
        if (simulationServiceId == 0L || region.sections == null) {
            return false;
        }
        long regionKey = simulationRegionKey(key);
        if (simulationBridge.hasRegionContext(simulationServiceId, regionKey)) {
            return true;
        }
        int cells = GRID_SIZE * GRID_SIZE * GRID_SIZE;
        float[] flowState = new float[FLOW_COUNT];
        float[] airTemperatureState = new float[cells];
        float[] surfaceTemperatureState = new float[cells];
        ServerWorld world = resolveWorld(key.worldKey());
        if (world != null && dynamicStore.loadRegion(
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
            overlaySeedWindowCoreFromBrickRuntime(key, buildRegionObstacleMask(region), flowState, airTemperatureState, surfaceTemperatureState);
            if (simulationBridge.importDynamicRegion(
                simulationServiceId,
                regionKey,
                GRID_SIZE,
                GRID_SIZE,
                GRID_SIZE,
                flowState,
                airTemperatureState,
                surfaceTemperatureState
            )) {
                return true;
            }
        }
        return seedWindowDynamicRegionFromNestedMet(key, region, flowState, airTemperatureState, surfaceTemperatureState);
    }

    private boolean seedWindowDynamicRegionFromNestedMet(
        WindowKey key,
        RegionRecord region,
        float[] flowState,
        float[] airTemperatureState,
        float[] surfaceTemperatureState
    ) {
        byte[] obstacleMask = buildRegionObstacleMask(region);
        MesoscaleGrid mesoscaleGrid = mesoscaleMetGrids.get(key.worldKey());
        boolean seeded = mesoscaleGrid != null
            && seedWindowDynamicRegionFromMesoscale(
                key,
                obstacleMask,
                flowState,
                airTemperatureState,
                surfaceTemperatureState,
                mesoscaleGrid
            );
        if (!seeded) {
            seedWindowDynamicRegionFromBoundarySample(
                key,
                obstacleMask,
                flowState,
                airTemperatureState,
                surfaceTemperatureState
            );
        }
        overlaySeedWindowCoreFromBrickRuntime(key, obstacleMask, flowState, airTemperatureState, surfaceTemperatureState);
        return simulationBridge.importDynamicRegion(
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

    private void overlaySeedWindowCoreFromBrickRuntime(
        WindowKey key,
        byte[] obstacleMask,
        float[] flowState,
        float[] airTemperatureState,
        float[] surfaceTemperatureState
    ) {
        if (simulationServiceId == 0L || !simulationBridge.isLoaded()) {
            return;
        }
        int coreCells = BRICK_RUNTIME_SIZE * BRICK_RUNTIME_SIZE * BRICK_RUNTIME_SIZE;
        float[] coreFlowState = new float[coreCells * RESPONSE_CHANNELS];
        float[] coreAirTemperatureState = new float[coreCells];
        float[] coreSurfaceTemperatureState = new float[coreCells];
        BlockPos coreOrigin = key.origin().add(REGION_HALO_CELLS, REGION_HALO_CELLS, REGION_HALO_CELLS);
        boolean copied = simulationBridge.copyBrickWorldDynamicBrick(
            simulationServiceId,
            simulationWorldKey(key.worldKey()),
            BRICK_RUNTIME_SIZE,
            Math.floorDiv(coreOrigin.getX(), BRICK_RUNTIME_SIZE),
            Math.floorDiv(coreOrigin.getY(), BRICK_RUNTIME_SIZE),
            Math.floorDiv(coreOrigin.getZ(), BRICK_RUNTIME_SIZE),
            coreFlowState,
            coreAirTemperatureState,
            coreSurfaceTemperatureState
        );
        if (!copied) {
            return;
        }
        for (int x = 0; x < BRICK_RUNTIME_SIZE; x++) {
            for (int y = 0; y < BRICK_RUNTIME_SIZE; y++) {
                for (int z = 0; z < BRICK_RUNTIME_SIZE; z++) {
                    int dstX = REGION_HALO_CELLS + x;
                    int dstY = REGION_HALO_CELLS + y;
                    int dstZ = REGION_HALO_CELLS + z;
                    int dstCell = gridCellIndex(dstX, dstY, dstZ);
                    if (obstacleMask[dstCell] != 0) {
                        continue;
                    }
                    int srcCell = patchCellIndex(x, y, z, BRICK_RUNTIME_SIZE);
                    int dstBase = dstCell * RESPONSE_CHANNELS;
                    int srcBase = srcCell * RESPONSE_CHANNELS;
                    for (int channel = 0; channel < RESPONSE_CHANNELS; channel++) {
                        flowState[dstBase + channel] = coreFlowState[srcBase + channel];
                    }
                    airTemperatureState[dstCell] = coreAirTemperatureState[srcCell];
                    surfaceTemperatureState[dstCell] = coreSurfaceTemperatureState[srcCell];
                }
            }
        }
    }

    private boolean seedWindowDynamicRegionFromMesoscale(
        WindowKey key,
        byte[] obstacleMask,
        float[] flowState,
        float[] airTemperatureState,
        float[] surfaceTemperatureState,
        MesoscaleGrid mesoscaleGrid
    ) {
        return seedDynamicRegionFromMesoscale(
            key.origin(),
            GRID_SIZE,
            GRID_SIZE,
            GRID_SIZE,
            obstacleMask,
            flowState,
            airTemperatureState,
            surfaceTemperatureState,
            mesoscaleGrid
        );
    }

    private void seedWindowDynamicRegionFromBoundarySample(
        WindowKey key,
        byte[] obstacleMask,
        float[] flowState,
        float[] airTemperatureState,
        float[] surfaceTemperatureState
    ) {
        seedDynamicRegionFromBoundarySample(
            key.worldKey(),
            key.origin(),
            GRID_SIZE,
            GRID_SIZE,
            GRID_SIZE,
            obstacleMask,
            flowState,
            airTemperatureState,
            surfaceTemperatureState
        );
    }

    private boolean seedDynamicRegionFromMesoscale(
        BlockPos origin,
        int sizeX,
        int sizeY,
        int sizeZ,
        byte[] obstacleMask,
        float[] flowState,
        float[] airTemperatureState,
        float[] surfaceTemperatureState,
        MesoscaleGrid mesoscaleGrid
    ) {
        int cells = sizeX * sizeY * sizeZ;
        float[] windX = new float[cells];
        float[] windZ = new float[cells];
        if (!mesoscaleGrid.seedL2Window(
            origin,
            sizeX,
            sizeY,
            sizeZ,
            windX,
            windZ,
            airTemperatureState,
            surfaceTemperatureState
        )) {
            return false;
        }
        for (int cell = 0; cell < cells; cell++) {
            if (obstacleMask[cell] != 0) {
                continue;
            }
            int base = cell * RESPONSE_CHANNELS;
            flowState[base] = windX[cell] / NATIVE_VELOCITY_SCALE;
            flowState[base + 2] = windZ[cell] / NATIVE_VELOCITY_SCALE;
        }
        deriveSeedVerticalVelocity(sizeX, sizeY, sizeZ, obstacleMask, flowState);
        return true;
    }

    private void seedDynamicRegionFromBoundarySample(
        RegistryKey<World> worldKey,
        BlockPos origin,
        int sizeX,
        int sizeY,
        int sizeZ,
        byte[] obstacleMask,
        float[] flowState,
        float[] airTemperatureState,
        float[] surfaceTemperatureState
    ) {
        BlockPos center = origin.add(sizeX / 2, sizeY / 2, sizeZ / 2);
        NestedBoundaryCoupler.BoundarySample boundarySample = sampleNestedBoundaryAtPosition(worldKey, center);
        float seedVx = boundarySample == null ? 0.0f : boundarySample.windX() / NATIVE_VELOCITY_SCALE;
        float seedVz = boundarySample == null ? 0.0f : boundarySample.windZ() / NATIVE_VELOCITY_SCALE;
        float seedAirTemperature = boundarySample == null
            ? THERMAL_BASE_AMBIENT_AIR_TEMPERATURE_K
            : boundarySample.ambientAirTemperatureKelvin();
        float seedSurfaceTemperature = boundarySample == null
            ? THERMAL_BASE_AMBIENT_AIR_TEMPERATURE_K + THERMAL_DEEP_GROUND_OFFSET_K
            : boundarySample.deepGroundTemperatureKelvin();
        int cells = sizeX * sizeY * sizeZ;
        for (int cell = 0; cell < cells; cell++) {
            airTemperatureState[cell] = seedAirTemperature;
            surfaceTemperatureState[cell] = seedSurfaceTemperature;
            if (obstacleMask[cell] != 0) {
                continue;
            }
            int base = cell * RESPONSE_CHANNELS;
            flowState[base] = seedVx;
            flowState[base + 2] = seedVz;
        }
        deriveSeedVerticalVelocity(sizeX, sizeY, sizeZ, obstacleMask, flowState);
    }

    private void deriveSeedVerticalVelocity(int sizeX, int sizeY, int sizeZ, byte[] obstacleMask, float[] flowState) {
        float[] divergenceColumn = new float[sizeY];
        float[] verticalVelocityColumn = new float[sizeY];
        for (int x = 0; x < sizeX; x++) {
            for (int z = 0; z < sizeZ; z++) {
                float maxHorizontalSpeed = 0.0f;
                for (int y = 0; y < sizeY; y++) {
                    int cell = patchCellIndex3d(x, y, z, sizeY, sizeZ);
                    int base = cell * RESPONSE_CHANNELS;
                    if (obstacleMask[cell] != 0) {
                        flowState[base + 1] = 0.0f;
                        divergenceColumn[y] = 0.0f;
                        continue;
                    }
                    float vx = flowState[base];
                    float vz = flowState[base + 2];
                    float vxMinus = sampleSeedFlowComponent(sizeX, sizeY, sizeZ, obstacleMask, flowState, x - 1, y, z, 0, vx);
                    float vxPlus = sampleSeedFlowComponent(sizeX, sizeY, sizeZ, obstacleMask, flowState, x + 1, y, z, 0, vx);
                    float vzMinus = sampleSeedFlowComponent(sizeX, sizeY, sizeZ, obstacleMask, flowState, x, y, z - 1, 2, vz);
                    float vzPlus = sampleSeedFlowComponent(sizeX, sizeY, sizeZ, obstacleMask, flowState, x, y, z + 1, 2, vz);
                    divergenceColumn[y] = 0.5f * ((vxPlus - vxMinus) + (vzPlus - vzMinus));
                    maxHorizontalSpeed = Math.max(maxHorizontalSpeed, (float) Math.sqrt(vx * vx + vz * vz));
                }
                integrateSeedVerticalVelocityColumn(
                    sizeX,
                    sizeY,
                    sizeZ,
                    x,
                    z,
                    obstacleMask,
                    flowState,
                    divergenceColumn,
                    verticalVelocityColumn,
                    maxHorizontalSpeed
                );
            }
        }
    }

    private float sampleSeedFlowComponent(
        int sizeX,
        int sizeY,
        int sizeZ,
        byte[] obstacleMask,
        float[] flowState,
        int x,
        int y,
        int z,
        int componentOffset,
        float fallback
    ) {
        if (!inBounds(x, y, z, sizeX, sizeY, sizeZ)) {
            return fallback;
        }
        int cell = patchCellIndex3d(x, y, z, sizeY, sizeZ);
        if (obstacleMask[cell] != 0) {
            return fallback;
        }
        return flowState[cell * RESPONSE_CHANNELS + componentOffset];
    }

    private void integrateSeedVerticalVelocityColumn(
        int sizeX,
        int sizeY,
        int sizeZ,
        int x,
        int z,
        byte[] obstacleMask,
        float[] flowState,
        float[] divergenceColumn,
        float[] verticalVelocityColumn,
        float maxHorizontalSpeed
    ) {
        int y = 0;
        float clamp = Math.max(0.15f / NATIVE_VELOCITY_SCALE, maxHorizontalSpeed * NESTED_BOUNDARY_MAX_VY_RATIO);
        while (y < sizeY) {
            int cell = patchCellIndex3d(x, y, z, sizeY, sizeZ);
            if (obstacleMask[cell] != 0) {
                flowState[cell * RESPONSE_CHANNELS + 1] = 0.0f;
                y++;
                continue;
            }
            int startY = y;
            while (y < sizeY && obstacleMask[patchCellIndex3d(x, y, z, sizeY, sizeZ)] == 0) {
                y++;
            }
            int length = y - startY;
            verticalVelocityColumn[0] = 0.0f;
            for (int i = 1; i < length; i++) {
                int currentY = startY + i;
                int previousY = currentY - 1;
                verticalVelocityColumn[i] = verticalVelocityColumn[i - 1]
                    - 0.5f * (divergenceColumn[previousY] + divergenceColumn[currentY]);
            }
            float mean = 0.0f;
            for (int i = 0; i < length; i++) {
                mean += verticalVelocityColumn[i];
            }
            mean /= length;
            for (int i = 0; i < length; i++) {
                float vy = MathHelper.clamp(verticalVelocityColumn[i] - mean, -clamp, clamp);
                int runCell = patchCellIndex3d(x, startY + i, z, sizeY, sizeZ);
                flowState[runCell * RESPONSE_CHANNELS + 1] = vy;
            }
        }
    }

    private ServerWorld resolveWorld(RegistryKey<World> worldKey) {
        MinecraftServer server = currentServer;
        return server == null ? null : server.getWorld(worldKey);
    }

    private boolean inBounds(int x, int y, int z) {
        return x >= 0 && y >= 0 && z >= 0 && x < GRID_SIZE && y < GRID_SIZE && z < GRID_SIZE;
    }

    private boolean inBounds(int x, int y, int z, int sizeX, int sizeY, int sizeZ) {
        return x >= 0 && y >= 0 && z >= 0 && x < sizeX && y < sizeY && z < sizeZ;
    }

    private boolean inSectionBounds(int x, int y, int z) {
        return x >= 0 && y >= 0 && z >= 0 && x < CHUNK_SIZE && y < CHUNK_SIZE && z < CHUNK_SIZE;
    }

    private byte[] buildRegionObstacleMask(RegionRecord region) {
        byte[] obstacleMask = new byte[GRID_SIZE * GRID_SIZE * GRID_SIZE];
        if (region.sections == null) {
            return obstacleMask;
        }
        for (int sx = 0; sx < WINDOW_SECTION_COUNT; sx++) {
            int baseX = sx * CHUNK_SIZE;
            for (int sy = 0; sy < WINDOW_SECTION_COUNT; sy++) {
                int baseY = sy * CHUNK_SIZE;
                for (int sz = 0; sz < WINDOW_SECTION_COUNT; sz++) {
                    int baseZ = sz * CHUNK_SIZE;
                    WorldMirror.SectionSnapshot snapshot = region.sectionAt(sx, sy, sz);
                    if (snapshot == null) {
                        for (int lx = 0; lx < CHUNK_SIZE; lx++) {
                            int x = baseX + lx;
                            for (int ly = 0; ly < CHUNK_SIZE; ly++) {
                                int y = baseY + ly;
                                for (int lz = 0; lz < CHUNK_SIZE; lz++) {
                                    obstacleMask[gridCellIndex(x, y, baseZ + lz)] = 1;
                                }
                            }
                        }
                        continue;
                    }
                    for (int lx = 0; lx < CHUNK_SIZE; lx++) {
                        int x = baseX + lx;
                        for (int ly = 0; ly < CHUNK_SIZE; ly++) {
                            int y = baseY + ly;
                            for (int lz = 0; lz < CHUNK_SIZE; lz++) {
                                int cell = gridCellIndex(x, y, baseZ + lz);
                                obstacleMask[cell] = snapshot.obstacle()[localSectionCellIndex(lx, ly, lz)] >= 0.5f ? (byte) 1 : (byte) 0;
                            }
                        }
                    }
                }
            }
        }
        return obstacleMask;
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

    private boolean obstacleAtPatch(byte[] obstacleMask, int size, int x, int y, int z) {
        return x < 0
            || y < 0
            || z < 0
            || x >= size
            || y >= size
            || z >= size
            || obstacleMask[patchCellIndex(x, y, z, size)] != 0;
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

    private void applyFanAtVoxelToPatchForcing(
        byte[] obstacleMask,
        int size,
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
        if (obstacleAtPatch(obstacleMask, size, x, y, z)) {
            return;
        }
        int cell = patchCellIndex(x, y, z, size);
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

    private void applyFanSourceToPatchForcing(
        byte[] obstacleMask,
        int size,
        byte[] fanMask,
        float[] fanVxField,
        float[] fanVyField,
        float[] fanVzField,
        FanSource fan,
        int cellsPerBlock,
        int minX,
        int minY,
        int minZ
    ) {
        BlockPos inflowPos = fan.pos().offset(fan.facing());
        int cx = worldToPatchCell(inflowPos.getX() + 0.5, minX, cellsPerBlock);
        int cy = worldToPatchCell(inflowPos.getY() + 0.5, minY, cellsPerBlock);
        int cz = worldToPatchCell(inflowPos.getZ() + 0.5, minZ, cellsPerBlock);

        float inflowSpeed = runtimeFanSpeedMetersPerSecond();
        float fanVx = fan.facing().getOffsetX() * inflowSpeed;
        float fanVy = fan.facing().getOffsetY() * inflowSpeed;
        float fanVz = fan.facing().getOffsetZ() * inflowSpeed;

        int radiusCells = Math.max(1, FAN_RADIUS * cellsPerBlock);
        int radius2 = radiusCells * radiusCells;
        switch (fan.facing().getAxis()) {
            case X -> {
                for (int y = cy - radiusCells; y <= cy + radiusCells; y++) {
                    for (int z = cz - radiusCells; z <= cz + radiusCells; z++) {
                        int dy = y - cy;
                        int dz = z - cz;
                        if (dy * dy + dz * dz > radius2) {
                            continue;
                        }
                        applyFanAtVoxelToPatchForcing(
                            obstacleMask,
                            size,
                            fanMask,
                            fanVxField,
                            fanVyField,
                            fanVzField,
                            cx,
                            y,
                            z,
                            fanVx,
                            fanVy,
                            fanVz
                        );
                    }
                }
            }
            case Y -> {
                for (int x = cx - radiusCells; x <= cx + radiusCells; x++) {
                    for (int z = cz - radiusCells; z <= cz + radiusCells; z++) {
                        int dx = x - cx;
                        int dz = z - cz;
                        if (dx * dx + dz * dz > radius2) {
                            continue;
                        }
                        applyFanAtVoxelToPatchForcing(
                            obstacleMask,
                            size,
                            fanMask,
                            fanVxField,
                            fanVyField,
                            fanVzField,
                            x,
                            cy,
                            z,
                            fanVx,
                            fanVy,
                            fanVz
                        );
                    }
                }
            }
            case Z -> {
                for (int x = cx - radiusCells; x <= cx + radiusCells; x++) {
                    for (int y = cy - radiusCells; y <= cy + radiusCells; y++) {
                        int dx = x - cx;
                        int dy = y - cy;
                        if (dx * dx + dy * dy > radius2) {
                            continue;
                        }
                        applyFanAtVoxelToPatchForcing(
                            obstacleMask,
                            size,
                            fanMask,
                            fanVxField,
                            fanVyField,
                            fanVzField,
                            x,
                            y,
                            cz,
                            fanVx,
                            fanVy,
                            fanVz
                        );
                    }
                }
            }
            default -> applyFanAtVoxelToPatchForcing(
                obstacleMask,
                size,
                fanMask,
                fanVxField,
                fanVyField,
                fanVzField,
                cx,
                cy,
                cz,
                fanVx,
                fanVy,
                fanVz
            );
        }
        applyDuctJetToPatchForcing(obstacleMask, size, fanMask, fanVxField, fanVyField, fanVzField, fan, cellsPerBlock, minX, minY, minZ);
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

    private void applyDuctJetToPatchForcing(
        byte[] obstacleMask,
        int size,
        byte[] fanMask,
        float[] fanVxField,
        float[] fanVyField,
        float[] fanVzField,
        FanSource fan,
        int cellsPerBlock,
        int minX,
        int minY,
        int minZ
    ) {
        int level = ductLevel(fan.ductLength());
        if (level <= 0) {
            return;
        }

        BlockPos inflowPos = fan.pos().offset(fan.facing());
        int sx = worldToPatchCell(inflowPos.getX() + 0.5, minX, cellsPerBlock);
        int sy = worldToPatchCell(inflowPos.getY() + 0.5, minY, cellsPerBlock);
        int sz = worldToPatchCell(inflowPos.getZ() + 0.5, minZ, cellsPerBlock);
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
        } * cellsPerBlock;
        for (int step = 0; step < range; step++) {
            float t = range > 1 ? (float) step / (range - 1) : 0.0f;
            float decay = 1.0f - 0.55f * t;
            float coreScale = levelBoost * Math.max(0.35f, decay);
            int cx = sx + dx * step;
            int cy = sy + dy * step;
            int cz = sz + dz * step;
            applyFanAtVoxelToPatchForcing(
                obstacleMask,
                size,
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
                    applyFanAtVoxelToPatchForcing(obstacleMask, size, fanMask, fanVxField, fanVyField, fanVzField, cx, cy + 1, cz, baseVx * edgeScale, baseVy * edgeScale, baseVz * edgeScale);
                    applyFanAtVoxelToPatchForcing(obstacleMask, size, fanMask, fanVxField, fanVyField, fanVzField, cx, cy - 1, cz, baseVx * edgeScale, baseVy * edgeScale, baseVz * edgeScale);
                    applyFanAtVoxelToPatchForcing(obstacleMask, size, fanMask, fanVxField, fanVyField, fanVzField, cx, cy, cz + 1, baseVx * edgeScale, baseVy * edgeScale, baseVz * edgeScale);
                    applyFanAtVoxelToPatchForcing(obstacleMask, size, fanMask, fanVxField, fanVyField, fanVzField, cx, cy, cz - 1, baseVx * edgeScale, baseVy * edgeScale, baseVz * edgeScale);
                }
                case Y -> {
                    applyFanAtVoxelToPatchForcing(obstacleMask, size, fanMask, fanVxField, fanVyField, fanVzField, cx + 1, cy, cz, baseVx * edgeScale, baseVy * edgeScale, baseVz * edgeScale);
                    applyFanAtVoxelToPatchForcing(obstacleMask, size, fanMask, fanVxField, fanVyField, fanVzField, cx - 1, cy, cz, baseVx * edgeScale, baseVy * edgeScale, baseVz * edgeScale);
                    applyFanAtVoxelToPatchForcing(obstacleMask, size, fanMask, fanVxField, fanVyField, fanVzField, cx, cy, cz + 1, baseVx * edgeScale, baseVy * edgeScale, baseVz * edgeScale);
                    applyFanAtVoxelToPatchForcing(obstacleMask, size, fanMask, fanVxField, fanVyField, fanVzField, cx, cy, cz - 1, baseVx * edgeScale, baseVy * edgeScale, baseVz * edgeScale);
                }
                case Z -> {
                    applyFanAtVoxelToPatchForcing(obstacleMask, size, fanMask, fanVxField, fanVyField, fanVzField, cx + 1, cy, cz, baseVx * edgeScale, baseVy * edgeScale, baseVz * edgeScale);
                    applyFanAtVoxelToPatchForcing(obstacleMask, size, fanMask, fanVxField, fanVyField, fanVzField, cx - 1, cy, cz, baseVx * edgeScale, baseVy * edgeScale, baseVz * edgeScale);
                    applyFanAtVoxelToPatchForcing(obstacleMask, size, fanMask, fanVxField, fanVyField, fanVzField, cx, cy + 1, cz, baseVx * edgeScale, baseVy * edgeScale, baseVz * edgeScale);
                    applyFanAtVoxelToPatchForcing(obstacleMask, size, fanMask, fanVxField, fanVyField, fanVzField, cx, cy - 1, cz, baseVx * edgeScale, baseVy * edgeScale, baseVz * edgeScale);
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

    private NestedBoundaryCoupler.BoundarySample sampleNestedBoundaryAtPosition(RegistryKey<World> worldKey, BlockPos pos) {
        MesoscaleGrid.Sample mesoscaleSample = sampleMesoscaleMet(worldKey, pos);
        if (mesoscaleSample != null) {
            return nestedBoundaryCoupler.fromMesoscaleSample(mesoscaleSample);
        }
        BackgroundMetGrid.Sample backgroundSample = sampleBackgroundMet(worldKey, pos);
        if (backgroundSample != null) {
            return nestedBoundaryCoupler.fromBackgroundSample(backgroundSample);
        }
        return new NestedBoundaryCoupler.BoundarySample(
            0.0f,
            0.0f,
            0.0f,
            THERMAL_BASE_AMBIENT_AIR_TEMPERATURE_K,
            THERMAL_BASE_AMBIENT_AIR_TEMPERATURE_K + THERMAL_DEEP_GROUND_OFFSET_K
        );
    }

    private BoundaryFieldData sampleNestedBoundaryFieldAtWindow(
        WindowKey key,
        int externalFaceMask,
        NestedBoundaryCoupler.BoundarySample fallback
    ) {
        int res = NESTED_BOUNDARY_FACE_RESOLUTION;
        int faceCells = FACE_COUNT * res * res;
        float[] windX = new float[faceCells];
        float[] windY = new float[faceCells];
        float[] windZ = new float[faceCells];
        float[] airTemperature = new float[faceCells];
        BlockPos origin = key.origin();
        double minX = origin.getX();
        double minY = origin.getY();
        double minZ = origin.getZ();
        double maxX = minX + GRID_SIZE;
        double maxY = minY + GRID_SIZE;
        double maxZ = minZ + GRID_SIZE;

        if ((externalFaceMask & (1 << Direction.WEST.ordinal())) != 0) {
            fillVerticalBoundaryFace(key.worldKey(), Direction.WEST.ordinal(), minX + 0.5, minZ, maxZ, minY, maxY, res, windX, windY, windZ, airTemperature, fallback);
        }
        if ((externalFaceMask & (1 << Direction.EAST.ordinal())) != 0) {
            fillVerticalBoundaryFace(key.worldKey(), Direction.EAST.ordinal(), maxX - 0.5, minZ, maxZ, minY, maxY, res, windX, windY, windZ, airTemperature, fallback);
        }
        if ((externalFaceMask & (1 << Direction.NORTH.ordinal())) != 0) {
            fillVerticalBoundaryFace(key.worldKey(), Direction.NORTH.ordinal(), minZ + 0.5, minX, maxX, minY, maxY, res, windX, windY, windZ, airTemperature, fallback);
        }
        if ((externalFaceMask & (1 << Direction.SOUTH.ordinal())) != 0) {
            fillVerticalBoundaryFace(key.worldKey(), Direction.SOUTH.ordinal(), maxZ - 0.5, minX, maxX, minY, maxY, res, windX, windY, windZ, airTemperature, fallback);
        }
        if ((externalFaceMask & (1 << Direction.DOWN.ordinal())) != 0) {
            fillHorizontalBoundaryFace(key.worldKey(), Direction.DOWN.ordinal(), minX, maxX, minZ, maxZ, minY + 0.5, res, windX, windY, windZ, airTemperature, minY, maxY, fallback);
        }
        if ((externalFaceMask & (1 << Direction.UP.ordinal())) != 0) {
            fillHorizontalBoundaryFace(key.worldKey(), Direction.UP.ordinal(), minX, maxX, minZ, maxZ, maxY - 0.5, res, windX, windY, windZ, airTemperature, minY, maxY, fallback);
        }
        return new BoundaryFieldData(res, externalFaceMask, windX, windY, windZ, airTemperature);
    }

    private BoundaryFieldData sampleInspectionBoundaryField(
        RegistryKey<World> worldKey,
        BlockPos origin,
        int size,
        int faceResolution,
        NestedBoundaryCoupler.BoundarySample fallback
    ) {
        int faceCells = FACE_COUNT * faceResolution * faceResolution;
        float[] windX = new float[faceCells];
        float[] windY = new float[faceCells];
        float[] windZ = new float[faceCells];
        float[] airTemperature = new float[faceCells];
        double minX = origin.getX();
        double minY = origin.getY();
        double minZ = origin.getZ();
        double maxX = minX + size;
        double maxY = minY + size;
        double maxZ = minZ + size;

        fillVerticalBoundaryFace(worldKey, Direction.WEST.ordinal(), minX + 0.5, minZ, maxZ, minY, maxY, faceResolution, windX, windY, windZ, airTemperature, fallback);
        fillVerticalBoundaryFace(worldKey, Direction.EAST.ordinal(), maxX - 0.5, minZ, maxZ, minY, maxY, faceResolution, windX, windY, windZ, airTemperature, fallback);
        fillVerticalBoundaryFace(worldKey, Direction.NORTH.ordinal(), minZ + 0.5, minX, maxX, minY, maxY, faceResolution, windX, windY, windZ, airTemperature, fallback);
        fillVerticalBoundaryFace(worldKey, Direction.SOUTH.ordinal(), maxZ - 0.5, minX, maxX, minY, maxY, faceResolution, windX, windY, windZ, airTemperature, fallback);
        fillHorizontalBoundaryFace(worldKey, Direction.DOWN.ordinal(), minX, maxX, minZ, maxZ, minY + 0.5, faceResolution, windX, windY, windZ, airTemperature, minY, maxY, fallback);
        fillHorizontalBoundaryFace(worldKey, Direction.UP.ordinal(), minX, maxX, minZ, maxZ, maxY - 0.5, faceResolution, windX, windY, windZ, airTemperature, minY, maxY, fallback);
        return new BoundaryFieldData(faceResolution, INSPECTION_PATCH_ALL_FACE_MASK, windX, windY, windZ, airTemperature);
    }

    private List<TornadoRegionDescriptor> collectTornadoRegionDescriptors(WindowKey key) {
        WorldScaleDriver driver = worldScaleDrivers.get(key.worldKey());
        if (driver == null) {
            return List.of();
        }
        WorldScaleDriver.Snapshot snapshot = driver.snapshot();
        if (snapshot.tornadoVortices().isEmpty()) {
            return List.of();
        }
        BlockPos origin = key.origin();
        double minX = origin.getX();
        double minZ = origin.getZ();
        double maxX = minX + GRID_SIZE;
        double maxZ = minZ + GRID_SIZE;
        List<TornadoRegionDescriptor> descriptors = new ArrayList<>();
        for (WorldScaleDriver.TornadoVortexSnapshot vortex : snapshot.tornadoVortices()) {
            if (!intersectsRegionHorizontally(vortex, minX, minZ, maxX, maxZ)) {
                continue;
            }
            descriptors.add(new TornadoRegionDescriptor(
                vortex.id(),
                vortex.parentConvectiveClusterId(),
                vortex.centerBlockX() - origin.getX(),
                vortex.baseY() - origin.getY(),
                vortex.centerBlockZ() - origin.getZ(),
                vortex.translationXBlocksPerSecond(),
                vortex.translationZBlocksPerSecond(),
                vortex.coreRadiusBlocks(),
                vortex.influenceRadiusBlocks(),
                vortex.tangentialWindScaleMps(),
                vortex.radialInflowScaleMps(),
                vortex.updraftScale(),
                vortex.condensationBias(),
                vortex.intensity(),
                vortex.rotationSign(),
                vortex.stateOrdinal(),
                tornadoLifecycleEnvelope(vortex)
            ));
        }
        return descriptors.isEmpty() ? List.of() : List.copyOf(descriptors);
    }

    private boolean intersectsRegionHorizontally(
        WorldScaleDriver.TornadoVortexSnapshot vortex,
        double minX,
        double minZ,
        double maxX,
        double maxZ
    ) {
        double closestX = MathHelper.clamp(vortex.centerBlockX(), minX, maxX);
        double closestZ = MathHelper.clamp(vortex.centerBlockZ(), minZ, maxZ);
        double dx = vortex.centerBlockX() - closestX;
        double dz = vortex.centerBlockZ() - closestZ;
        double radius = Math.max(vortex.influenceRadiusBlocks(), vortex.coreRadiusBlocks());
        return dx * dx + dz * dz <= radius * radius;
    }

    private float tornadoLifecycleEnvelope(WorldScaleDriver.TornadoVortexSnapshot vortex) {
        float lifetime = Math.max(1.0f, vortex.lifetimeSeconds());
        float progress = MathHelper.clamp(vortex.ageSeconds() / lifetime, 0.0f, 1.0f);
        return switch (vortex.stateOrdinal()) {
            case 0 -> MathHelper.clamp(progress / 0.25f, 0.0f, 1.0f);
            case 1 -> 1.0f;
            case 2 -> MathHelper.clamp((1.0f - progress) / 0.25f, 0.0f, 1.0f);
            default -> 0.0f;
        };
    }

    private void fillVerticalBoundaryFace(
        RegistryKey<World> worldKey,
        int face,
        double fixedAxis,
        double horizontalMin,
        double horizontalMax,
        double minY,
        double maxY,
        int resolution,
        float[] windX,
        float[] windY,
        float[] windZ,
        float[] airTemperature,
        NestedBoundaryCoupler.BoundarySample fallback
    ) {
        for (int u = 0; u < resolution; u++) {
            float[] vxColumn = new float[resolution];
            float[] vzColumn = new float[resolution];
            float[] tempColumn = new float[resolution];
            float[] divColumn = new float[resolution];
            float maxHorizontalSpeed = 0.0f;
            for (int v = 0; v < resolution; v++) {
                double horizontal = lerp(horizontalMin, horizontalMax, (u + 0.5) / resolution);
                double y = lerp(minY, maxY, (v + 0.5) / resolution);
                BlockPos samplePos = switch (face) {
                    case 4, 5 -> BlockPos.ofFloored(fixedAxis, y, horizontal);
                    case 2, 3 -> BlockPos.ofFloored(horizontal, y, fixedAxis);
                    default -> BlockPos.ORIGIN;
                };
                NestedMetState state = sampleNestedMetState(worldKey, samplePos, fallback);
                vxColumn[v] = state.windX();
                vzColumn[v] = state.windZ();
                tempColumn[v] = state.airTemperatureKelvin();
                divColumn[v] = sampleHorizontalDivergence(worldKey, samplePos, fallback);
                maxHorizontalSpeed = Math.max(maxHorizontalSpeed, (float) Math.sqrt(state.windX() * state.windX() + state.windZ() * state.windZ()));
            }
            float[] vyColumn = integrateVerticalVelocity(divColumn, maxHorizontalSpeed, (float) ((maxY - minY) / Math.max(1, resolution - 1)));
            for (int v = 0; v < resolution; v++) {
                int index = boundaryFaceIndex(face, u, v, resolution);
                windX[index] = vxColumn[v] / NATIVE_VELOCITY_SCALE;
                windY[index] = vyColumn[v] / NATIVE_VELOCITY_SCALE;
                windZ[index] = vzColumn[v] / NATIVE_VELOCITY_SCALE;
                airTemperature[index] = tempColumn[v];
            }
        }
    }

    private void fillHorizontalBoundaryFace(
        RegistryKey<World> worldKey,
        int face,
        double minX,
        double maxX,
        double minZ,
        double maxZ,
        double fixedY,
        int resolution,
        float[] windX,
        float[] windY,
        float[] windZ,
        float[] airTemperature,
        double columnMinY,
        double columnMaxY,
        NestedBoundaryCoupler.BoundarySample fallback
    ) {
        for (int u = 0; u < resolution; u++) {
            for (int v = 0; v < resolution; v++) {
                double x = lerp(minX, maxX, (u + 0.5) / resolution);
                double z = lerp(minZ, maxZ, (v + 0.5) / resolution);
                BlockPos samplePos = BlockPos.ofFloored(x, fixedY, z);
                NestedMetState state = sampleNestedMetState(worldKey, samplePos, fallback);
                float vy = sampleColumnVerticalVelocity(worldKey, x, z, columnMinY, columnMaxY, face == Direction.DOWN.ordinal() ? 0 : resolution - 1, resolution, fallback);
                int index = boundaryFaceIndex(face, u, v, resolution);
                windX[index] = state.windX() / NATIVE_VELOCITY_SCALE;
                windY[index] = vy / NATIVE_VELOCITY_SCALE;
                windZ[index] = state.windZ() / NATIVE_VELOCITY_SCALE;
                airTemperature[index] = state.airTemperatureKelvin();
            }
        }
    }

    private float sampleColumnVerticalVelocity(
        RegistryKey<World> worldKey,
        double x,
        double z,
        double minY,
        double maxY,
        int targetIndex,
        int resolution,
        NestedBoundaryCoupler.BoundarySample fallback
    ) {
        float[] divColumn = new float[resolution];
        float maxHorizontalSpeed = 0.0f;
        for (int i = 0; i < resolution; i++) {
            double y = lerp(minY, maxY, (i + 0.5) / resolution);
            BlockPos samplePos = BlockPos.ofFloored(x, y, z);
            NestedMetState state = sampleNestedMetState(worldKey, samplePos, fallback);
            divColumn[i] = sampleHorizontalDivergence(worldKey, samplePos, fallback);
            maxHorizontalSpeed = Math.max(maxHorizontalSpeed, (float) Math.sqrt(state.windX() * state.windX() + state.windZ() * state.windZ()));
        }
        float[] vyColumn = integrateVerticalVelocity(divColumn, maxHorizontalSpeed, (float) ((maxY - minY) / Math.max(1, resolution - 1)));
        return vyColumn[MathHelper.clamp(targetIndex, 0, resolution - 1)];
    }

    private float[] integrateVerticalVelocity(float[] divergenceColumn, float maxHorizontalSpeed, float dyMeters) {
        float[] vy = new float[divergenceColumn.length];
        for (int i = 1; i < divergenceColumn.length; i++) {
            vy[i] = vy[i - 1] - 0.5f * (divergenceColumn[i - 1] + divergenceColumn[i]) * dyMeters;
        }
        float mean = 0.0f;
        for (float value : vy) {
            mean += value;
        }
        mean /= Math.max(1, vy.length);
        float clamp = Math.max(0.15f, maxHorizontalSpeed * NESTED_BOUNDARY_MAX_VY_RATIO);
        for (int i = 0; i < vy.length; i++) {
            vy[i] = MathHelper.clamp(vy[i] - mean, -clamp, clamp);
        }
        return vy;
    }

    private float sampleHorizontalDivergence(
        RegistryKey<World> worldKey,
        BlockPos samplePos,
        NestedBoundaryCoupler.BoundarySample fallback
    ) {
        BlockPos xMinus = samplePos.add(-MESOSCALE_MET_CELL_SIZE_BLOCKS, 0, 0);
        BlockPos xPlus = samplePos.add(MESOSCALE_MET_CELL_SIZE_BLOCKS, 0, 0);
        BlockPos zMinus = samplePos.add(0, 0, -MESOSCALE_MET_CELL_SIZE_BLOCKS);
        BlockPos zPlus = samplePos.add(0, 0, MESOSCALE_MET_CELL_SIZE_BLOCKS);
        NestedMetState sxMinus = sampleNestedMetState(worldKey, xMinus, fallback);
        NestedMetState sxPlus = sampleNestedMetState(worldKey, xPlus, fallback);
        NestedMetState szMinus = sampleNestedMetState(worldKey, zMinus, fallback);
        NestedMetState szPlus = sampleNestedMetState(worldKey, zPlus, fallback);
        float dxMeters = MESOSCALE_MET_CELL_SIZE_BLOCKS;
        float dzMeters = MESOSCALE_MET_CELL_SIZE_BLOCKS;
        float dudx = (sxPlus.windX() - sxMinus.windX()) / (2.0f * dxMeters);
        float dwdz = (szPlus.windZ() - szMinus.windZ()) / (2.0f * dzMeters);
        return dudx + dwdz;
    }

    private NestedMetState sampleNestedMetState(
        RegistryKey<World> worldKey,
        BlockPos pos,
        NestedBoundaryCoupler.BoundarySample fallback
    ) {
        MesoscaleGrid.Sample mesoscale = sampleMesoscaleMet(worldKey, pos);
        if (mesoscale != null) {
            return new NestedMetState(mesoscale.windX(), mesoscale.windZ(), mesoscale.ambientAirTemperatureKelvin());
        }
        BackgroundMetGrid.Sample background = sampleBackgroundMet(worldKey, pos);
        if (background != null) {
            return new NestedMetState(background.backgroundWindX(), background.backgroundWindZ(), background.ambientAirTemperatureKelvin());
        }
        return new NestedMetState(
            fallback == null ? 0.0f : fallback.windX(),
            fallback == null ? 0.0f : fallback.windZ(),
            fallback == null ? THERMAL_BASE_AMBIENT_AIR_TEMPERATURE_K : fallback.ambientAirTemperatureKelvin()
        );
    }

    private int boundaryFaceIndex(int face, int u, int v, int resolution) {
        return (face * resolution + u) * resolution + v;
    }

    private double lerp(double min, double max, double t) {
        return min + (max - min) * t;
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

    private boolean isSolidObstacleAtPoint(
        ServerWorld world,
        BlockPos pos,
        BlockState state,
        double worldX,
        double worldY,
        double worldZ
    ) {
        if (!isSolidObstacle(world, pos, state)) {
            return false;
        }
        VoxelShape shape = state.getCollisionShape(world, pos);
        if (shape.isEmpty()) {
            return false;
        }
        double localX = MathHelper.clamp(worldX - pos.getX(), 0.0, 0.999999);
        double localY = MathHelper.clamp(worldY - pos.getY(), 0.0, 0.999999);
        double localZ = MathHelper.clamp(worldZ - pos.getZ(), 0.0, 0.999999);
        for (Box box : shape.getBoundingBoxes()) {
            if (localX >= box.minX && localX < box.maxX
                && localY >= box.minY && localY < box.maxY
                && localZ >= box.minZ && localZ < box.maxZ) {
                return true;
            }
        }
        return false;
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

    private int sanitizeStride(int requested) {
        return (requested == 1 || requested == 2 || requested == 4 || requested == 8) ? requested : PARTICLE_FLOW_SAMPLE_STRIDE;
    }

    private BrickRuntimeDynamicState copyBrickRuntimeDynamicState(
        RegistryKey<World> worldKey,
        int brickX,
        int brickY,
        int brickZ
    ) {
        if (simulationServiceId == 0L || !simulationBridge.isLoaded()) {
            return null;
        }
        int brickCells = BRICK_RUNTIME_SIZE * BRICK_RUNTIME_SIZE * BRICK_RUNTIME_SIZE;
        float[] flowState = new float[brickCells * RESPONSE_CHANNELS];
        float[] airTemperatureState = new float[brickCells];
        float[] surfaceTemperatureState = new float[brickCells];
        boolean copied = simulationBridge.copyBrickWorldDynamicBrick(
            simulationServiceId,
            simulationWorldKey(worldKey),
            BRICK_RUNTIME_SIZE,
            brickX,
            brickY,
            brickZ,
            flowState,
            airTemperatureState,
            surfaceTemperatureState
        );
        if (!copied) {
            return null;
        }
        return new BrickRuntimeDynamicState(flowState, airTemperatureState, surfaceTemperatureState);
    }

    private SampledPoint sampleBrickRuntimePointLocked(RegistryKey<World> worldKey, BlockPos probePos) {
        int brickX = Math.floorDiv(probePos.getX(), BRICK_RUNTIME_SIZE);
        int brickY = Math.floorDiv(probePos.getY(), BRICK_RUNTIME_SIZE);
        int brickZ = Math.floorDiv(probePos.getZ(), BRICK_RUNTIME_SIZE);
        BrickRuntimeDynamicState brickState = copyBrickRuntimeDynamicState(worldKey, brickX, brickY, brickZ);
        if (brickState == null) {
            return null;
        }
        int localX = probePos.getX() - brickX * BRICK_RUNTIME_SIZE;
        int localY = probePos.getY() - brickY * BRICK_RUNTIME_SIZE;
        int localZ = probePos.getZ() - brickZ * BRICK_RUNTIME_SIZE;
        if (localX < 0 || localX >= BRICK_RUNTIME_SIZE
            || localY < 0 || localY >= BRICK_RUNTIME_SIZE
            || localZ < 0 || localZ >= BRICK_RUNTIME_SIZE) {
            return null;
        }
        int cell = patchCellIndex(localX, localY, localZ, BRICK_RUNTIME_SIZE);
        int base = cell * RESPONSE_CHANNELS;
        ThermalEnvironment environment = sampleThermalEnvironment(
            worldEnvironmentSnapshots.get(worldKey),
            worldKey,
            probePos,
            SOLVER_STEP_SECONDS
        );
        return new SampledPoint(
            brickState.flowState()[base] * NATIVE_VELOCITY_SCALE,
            brickState.flowState()[base + 1] * NATIVE_VELOCITY_SCALE,
            brickState.flowState()[base + 2] * NATIVE_VELOCITY_SCALE,
            brickState.flowState()[base + 3],
            environment.ambientAirTemperatureKelvin()
                + brickState.airTemperatureState()[cell] * RUNTIME_TEMPERATURE_SCALE_KELVIN,
            brickState.surfaceTemperatureState()[cell]
        );
    }

    private short quantizeSignedToShort(float value, float range) {
        if (!(range > 0.0f) || !Float.isFinite(value)) {
            return 0;
        }
        float normalized = MathHelper.clamp(value / range, -1.0f, 1.0f);
        return (short) Math.round(normalized * 32767.0f);
    }

    private BrickRuntimeAtlasSnapshot sampleBrickRuntimeCoreAtlas(WindowKey key, int sampleStride) {
        BlockPos coreOrigin = key.origin().add(REGION_HALO_CELLS, REGION_HALO_CELLS, REGION_HALO_CELLS);
        BrickRuntimeDynamicState brickState = copyBrickRuntimeDynamicState(
            key.worldKey(),
            Math.floorDiv(coreOrigin.getX(), BRICK_RUNTIME_SIZE),
            Math.floorDiv(coreOrigin.getY(), BRICK_RUNTIME_SIZE),
            Math.floorDiv(coreOrigin.getZ(), BRICK_RUNTIME_SIZE)
        );
        if (brickState == null) {
            return null;
        }
        float maxSpeed = 0.0f;
        for (int cell = 0; cell < BRICK_RUNTIME_SIZE * BRICK_RUNTIME_SIZE * BRICK_RUNTIME_SIZE; cell++) {
            int srcBase = cell * RESPONSE_CHANNELS;
            float vx = brickState.flowState()[srcBase] * NATIVE_VELOCITY_SCALE;
            float vy = brickState.flowState()[srcBase + 1] * NATIVE_VELOCITY_SCALE;
            float vz = brickState.flowState()[srcBase + 2] * NATIVE_VELOCITY_SCALE;
            float speed = (float) Math.sqrt(vx * vx + vy * vy + vz * vz);
            if (Float.isFinite(speed) && speed > maxSpeed) {
                maxSpeed = speed;
            }
        }
        int n = (GRID_SIZE + sampleStride - 1) / sampleStride;
        short[] packed = new short[n * n * n * NativeSimulationBridge.PACKED_ATLAS_CHANNELS];
        boolean hasCoreSamples = false;
        for (int sx = 0; sx < n; sx++) {
            int gx = Math.min(GRID_SIZE - 1, sx * sampleStride);
            if (gx < REGION_HALO_CELLS || gx >= REGION_HALO_CELLS + BRICK_RUNTIME_SIZE) {
                continue;
            }
            int bx = gx - REGION_HALO_CELLS;
            for (int sy = 0; sy < n; sy++) {
                int gy = Math.min(GRID_SIZE - 1, sy * sampleStride);
                if (gy < REGION_HALO_CELLS || gy >= REGION_HALO_CELLS + BRICK_RUNTIME_SIZE) {
                    continue;
                }
                int by = gy - REGION_HALO_CELLS;
                for (int sz = 0; sz < n; sz++) {
                    int gz = Math.min(GRID_SIZE - 1, sz * sampleStride);
                    if (gz < REGION_HALO_CELLS || gz >= REGION_HALO_CELLS + BRICK_RUNTIME_SIZE) {
                        continue;
                    }
                    int bz = gz - REGION_HALO_CELLS;
                    int srcCell = patchCellIndex(bx, by, bz, BRICK_RUNTIME_SIZE);
                    int srcBase = srcCell * RESPONSE_CHANNELS;
                    int dstCell = ((sx * n) + sy) * n + sz;
                    int dstBase = dstCell * NativeSimulationBridge.PACKED_ATLAS_CHANNELS;
                    packed[dstBase] = quantizeSignedToShort(
                        brickState.flowState()[srcBase] * NATIVE_VELOCITY_SCALE,
                        ATLAS_VELOCITY_QUANT_RANGE
                    );
                    packed[dstBase + 1] = quantizeSignedToShort(
                        brickState.flowState()[srcBase + 1] * NATIVE_VELOCITY_SCALE,
                        ATLAS_VELOCITY_QUANT_RANGE
                    );
                    packed[dstBase + 2] = quantizeSignedToShort(
                        brickState.flowState()[srcBase + 2] * NATIVE_VELOCITY_SCALE,
                        ATLAS_VELOCITY_QUANT_RANGE
                    );
                    packed[dstBase + 3] = quantizeSignedToShort(
                        brickState.flowState()[srcBase + 3],
                        ATLAS_PRESSURE_QUANT_RANGE
                    );
                    hasCoreSamples = true;
                }
            }
        }
        return hasCoreSamples ? new BrickRuntimeAtlasSnapshot(packed, maxSpeed) : null;
    }

    private float publishBrickSolveAtlases(Set<WindowKey> solveWindowKeys) {
        if (solveWindowKeys.isEmpty()) {
            publishedFrame.set(null);
            lastPublishedFrameTick = Integer.MIN_VALUE;
            lastCoordinatorPublishedMaxSpeed = 0.0f;
            return 0.0f;
        }
        Map<WindowKey, short[]> atlases = new HashMap<>();
        Map<WindowKey, Float> regionMaxSpeeds = new HashMap<>();
        for (WindowKey key : solveWindowKeys) {
            BrickRuntimeAtlasSnapshot atlas = sampleBrickRuntimeCoreAtlas(key, PARTICLE_FLOW_SAMPLE_STRIDE);
            if (atlas == null) {
                continue;
            }
            atlases.put(key, atlas.packed());
            regionMaxSpeeds.put(key, atlas.maxSpeed());
        }
        if (atlases.isEmpty()) {
            publishedFrame.set(null);
            lastPublishedFrameTick = Integer.MIN_VALUE;
            lastCoordinatorPublishedMaxSpeed = 0.0f;
            return 0.0f;
        }
        float maxSpeed = computePublishedMaxSpeed(regionMaxSpeeds);
        PublishedFrame next = new PublishedFrame(
            publishedFrameCounter.incrementAndGet(),
            maxSpeed,
            Map.copyOf(atlases),
            Map.copyOf(regionMaxSpeeds)
        );
        publishedFrame.set(next);
        lastPublishedFrameTick = tickCounter;
        lastCoordinatorPublishedMaxSpeed = maxSpeed;
        lastCoordinatorPublishTick = tickCounter;
        return maxSpeed;
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
        // Window solve tasks no longer exist; brick epochs run synchronously in the coordinator.
    }

    private void ensureSimulationCoordinatorRunning() {
        synchronized (coordinatorLifecycleLock) {
            if (simulationCoordinator != null && simulationCoordinator.running()) {
                return;
            }
            lastCoordinatorError = "";
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

    private int attachedWindowCount() {
        return desiredWindowKeys.size();
    }

    private int currentPublishedFrameAgeTicks() {
        if (publishedFrame.get() == null) {
            return -1;
        }
        if (lastPublishedFrameTick == Integer.MIN_VALUE) {
            return -1;
        }
        return Math.max(0, tickCounter - lastPublishedFrameTick);
    }

    private int currentCoordinatorPublishAgeTicks() {
        if (lastCoordinatorPublishTick == Integer.MIN_VALUE) {
            return -1;
        }
        return Math.max(0, tickCounter - lastCoordinatorPublishTick);
    }

    private int currentCoordinatorScheduleAgeTicks() {
        if (lastCoordinatorScheduleTick == Integer.MIN_VALUE) {
            return -1;
        }
        return Math.max(0, tickCounter - lastCoordinatorScheduleTick);
    }

    private int currentCoordinatorSolveCompleteAgeTicks() {
        if (lastCoordinatorSolveCompleteTick == Integer.MIN_VALUE) {
            return -1;
        }
        return Math.max(0, tickCounter - lastCoordinatorSolveCompleteTick);
    }

    private boolean isCoordinatorAlive() {
        SimulationCoordinator coordinator = simulationCoordinator;
        return coordinator != null && coordinator.running();
    }

    private void publishRegionAtlas(WindowKey key, float regionMaxSpeed) {
        if (simulationServiceId == 0L) {
            return;
        }
        short[] packed = pollPackedFlowForNetwork(key, PARTICLE_FLOW_SAMPLE_STRIDE);
        if (packed == null) {
            return;
        }
        while (true) {
            PublishedFrame current = publishedFrame.get();
            Map<WindowKey, short[]> nextAtlases = new HashMap<>(current == null ? Map.of() : current.regionAtlases());
            Map<WindowKey, Float> nextRegionMaxSpeeds = new HashMap<>(current == null ? Map.of() : current.regionMaxSpeeds());
            nextAtlases.put(key, packed);
            nextRegionMaxSpeeds.put(key, Math.max(0.0f, regionMaxSpeed));
            float nextMaxSpeed = computePublishedMaxSpeed(nextRegionMaxSpeeds);
            PublishedFrame next = new PublishedFrame(
                publishedFrameCounter.incrementAndGet(),
                nextMaxSpeed,
                Map.copyOf(nextAtlases),
                Map.copyOf(nextRegionMaxSpeeds)
            );
            if (publishedFrame.compareAndSet(current, next)) {
                lastPublishedFrameTick = tickCounter;
                lastCoordinatorPublishedMaxSpeed = nextMaxSpeed;
                lastCoordinatorPublishTick = tickCounter;
                return;
            }
        }
    }

    private void removePublishedRegionAtlas(WindowKey key) {
        while (true) {
            PublishedFrame current = publishedFrame.get();
            if (current == null || !current.regionAtlases().containsKey(key)) {
                return;
            }
            Map<WindowKey, short[]> nextAtlases = new HashMap<>(current.regionAtlases());
            Map<WindowKey, Float> nextRegionMaxSpeeds = new HashMap<>(current.regionMaxSpeeds());
            nextAtlases.remove(key);
            nextRegionMaxSpeeds.remove(key);
            PublishedFrame next = nextAtlases.isEmpty()
                ? null
                : new PublishedFrame(
                    publishedFrameCounter.incrementAndGet(),
                    computePublishedMaxSpeed(nextRegionMaxSpeeds),
                    Map.copyOf(nextAtlases),
                    Map.copyOf(nextRegionMaxSpeeds)
                );
            if (publishedFrame.compareAndSet(current, next)) {
                if (next == null) {
                    lastPublishedFrameTick = Integer.MIN_VALUE;
                    lastCoordinatorPublishedMaxSpeed = 0.0f;
                }
                return;
            }
        }
    }

    private float computePublishedMaxSpeed(Map<WindowKey, Float> regionMaxSpeeds) {
        float maxSpeed = 0.0f;
        for (float value : regionMaxSpeeds.values()) {
            if (Float.isFinite(value) && value > maxSpeed) {
                maxSpeed = value;
            }
        }
        return maxSpeed;
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

    private void syncAnalysisFlowToPlayers(MinecraftServer server, PublishedFrame frame) {
        Map<WindowKey, AeroFlowAnalysisPayload> payloadCache = new HashMap<>();
        Set<WindowKey> missingKeys = new HashSet<>();
        for (ServerPlayerEntity player : server.getPlayerManager().getPlayerList()) {
            ServerWorld world = player.getEntityWorld();
            WindowKey key = new WindowKey(world.getRegistryKey(), windowOriginFromCoreOrigin(coreOriginForPosition(player.getBlockPos())));
            short[] basePacked = frame.regionAtlases().get(key);
            if (basePacked == null || missingKeys.contains(key)) {
                continue;
            }
            AeroFlowAnalysisPayload payload = payloadCache.get(key);
            if (payload == null && !payloadCache.containsKey(key)) {
                payload = buildAnalysisFlowPayload(world, key, basePacked);
                if (payload == null) {
                    missingKeys.add(key);
                    continue;
                }
                payloadCache.put(key, payload);
            }
            if (payload != null) {
                ServerPlayNetworking.send(player, payload);
            }
        }
    }

    private AeroFlowAnalysisPayload buildAnalysisFlowPayload(ServerWorld world, WindowKey key, short[] basePacked) {
        float[] fullFlowState = null;
        synchronized (simulationStateLock) {
            if (simulationServiceId == 0L) {
                return null;
            }
            RegionRecord region = regions.get(key);
            if (region == null || !region.serviceReady) {
                return null;
            }
            fullFlowState = new float[GRID_SIZE * GRID_SIZE * GRID_SIZE * NativeSimulationBridge.FLOW_STATE_CHANNELS];
            if (!simulationBridge.getRegionFlowState(
                simulationServiceId,
                simulationRegionKey(key),
                GRID_SIZE,
                GRID_SIZE,
                GRID_SIZE,
                fullFlowState
            )) {
                return null;
            }
        }
        return AnalysisFlowCodec.encodePayload(
            simulationBridge,
            world.getRegistryKey().getValue(),
            key.origin(),
            PARTICLE_FLOW_SAMPLE_STRIDE,
            basePacked,
            GRID_SIZE,
            fullFlowState,
            ANALYSIS_FLOW_VELOCITY_TOLERANCE,
            ANALYSIS_FLOW_PRESSURE_TOLERANCE
        );
    }

    private short[] pollPackedFlowForNetwork(WindowKey key, int stride) {
        int sampleStride = sanitizeStride(stride);
        int n = (GRID_SIZE + sampleStride - 1) / sampleStride;
        short[] packed = new short[n * n * n * NativeSimulationBridge.PACKED_ATLAS_CHANNELS];
        if (simulationServiceId != 0L
            && simulationBridge.pollPackedFlowAtlas(simulationServiceId, simulationRegionKey(key), packed)) {
            return packed;
        }
        BrickRuntimeAtlasSnapshot atlas = sampleBrickRuntimeCoreAtlas(key, sampleStride);
        return atlas == null ? null : atlas.packed();
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
            probes.put(request.playerId(), buildPlayerProbe(request, sample));
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
        SampledPoint brickSample = sampleBrickRuntimePointLocked(worldKey, probePos);
        if (brickSample != null) {
            return brickSample;
        }
        WindowKey key = new WindowKey(worldKey, windowOriginFromCoreOrigin(coreOriginForPosition(probePos)));
        RegionRecord region = regions.get(key);
        if (region == null || !region.serviceReady) {
            return null;
        }
        return sampleRegionPointForKey(key, probePos, rawProbe);
    }

    private SampledPoint sampleRegionPointForKey(WindowKey key, BlockPos probePos, float[] rawProbe) {
        RegistryKey<World> worldKey = key.worldKey();
        int localX = probePos.getX() - key.origin().getX();
        int localY = probePos.getY() - key.origin().getY();
        int localZ = probePos.getZ() - key.origin().getZ();
        if (!inBounds(localX, localY, localZ)) {
            return null;
        }
        if (localX >= REGION_HALO_CELLS && localX < REGION_HALO_CELLS + BRICK_RUNTIME_SIZE
            && localY >= REGION_HALO_CELLS && localY < REGION_HALO_CELLS + BRICK_RUNTIME_SIZE
            && localZ >= REGION_HALO_CELLS && localZ < REGION_HALO_CELLS + BRICK_RUNTIME_SIZE) {
            SampledPoint brickSample = sampleBrickRuntimePointLocked(worldKey, probePos);
            if (brickSample != null) {
                return brickSample;
            }
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

    private PlayerProbe buildPlayerProbe(PlayerProbeRequest request, SampledPoint sample) {
        return new PlayerProbe(
            request.playerId(),
            request.worldKey(),
            request.blockPos(),
            sample.velocityX(),
            sample.velocityY(),
            sample.velocityZ(),
            sample.pressure(),
            sample.airTemperatureKelvin(),
            sample.surfaceTemperatureKelvin()
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

    private record InspectionPatchStaticFields(
        byte[] obstacle,
        byte[] surfaceKind,
        byte[] openFaceMask,
        float[] emitterPowerWatts,
        byte[] faceSkyExposure,
        byte[] faceDirectExposure
    ) {
    }

    private record InspectionPatchInput(
        RegistryKey<World> worldKey,
        BlockPos focus,
        BlockPos origin,
        int domainBlocks,
        int gridResolution,
        int cellsPerBlock,
        int faceResolution,
        Path outputDir,
        WorldEnvironmentSnapshot environmentSnapshot,
        NestedBoundaryCoupler.BoundarySample fallbackBoundary,
        ThermalEnvironment thermalEnvironment,
        BoundaryFieldData boundaryField,
        InspectionPatchStaticFields staticFields,
        List<FanSource> fans
    ) {
    }

    private record InspectionPatchForcing(
        float[] thermalSource,
        byte[] fanMask,
        float[] fanVx,
        float[] fanVy,
        float[] fanVz
    ) {
    }

    private record InspectionPatchDynamicState(
        float[] flowState,
        float[] airTemperature,
        float[] surfaceTemperature
    ) {
    }

    private record InspectionPatchSolveResult(
        int domainBlocks,
        int gridResolution,
        int cellsPerBlock,
        float[] vx,
        float[] vy,
        float[] vz,
        float[] pressure,
        float[] airTemperature,
        float[] surfaceTemperature,
        int completedSteps,
        float maxSpeedMps
    ) {
    }

    private record PlayerRegionAnchor(
        RegistryKey<World> worldKey,
        BlockPos coreOrigin,
        BlockPos blockPos
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

    private record ChunkResidentBrickRefreshRequest(
        RegistryKey<World> worldKey,
        int chunkX,
        int chunkZ
    ) {
    }

    private record BrickRuntimeHint(
        int brickX,
        int brickY,
        int brickZ
    ) {
    }

    private record ActiveRegionBatch(
        int tickCounter,
        List<PlayerRegionAnchor> anchors,
        List<PlayerProbeRequest> playerProbeRequests,
        List<EntitySampleRequest> entitySampleRequests,
        Map<RegistryKey<World>, WorldEnvironmentSnapshot> environmentSnapshots,
        Map<RegistryKey<World>, int[]> brickRuntimeHintCoordsByWorld
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

    private record BrickRuntimeDynamicState(
        float[] flowState,
        float[] airTemperatureState,
        float[] surfaceTemperatureState
    ) {
    }

    private record BrickRuntimeAtlasSnapshot(short[] packed, float maxSpeed) {
    }

    private record TornadoRegionDescriptor(
        int id,
        int parentConvectiveClusterId,
        float centerXBlocks,
        float centerYBlocks,
        float centerZBlocks,
        float translationXBlocksPerSecond,
        float translationZBlocksPerSecond,
        float coreRadiusBlocks,
        float influenceRadiusBlocks,
        float tangentialWindScaleMps,
        float radialInflowScaleMps,
        float updraftScale,
        float condensationBias,
        float intensity,
        float rotationSign,
        int stateOrdinal,
        float lifecycleEnvelope
    ) {
    }

    private record BoundaryFieldData(
        int faceResolution,
        int externalFaceMask,
        float[] windX,
        float[] windY,
        float[] windZ,
        float[] airTemperatureKelvin
    ) {
    }

    private record NestedMetState(
        float windX,
        float windZ,
        float airTemperatureKelvin
    ) {
    }

    private record PublishedFrame(
        long frameId,
        float maxSpeed,
        Map<WindowKey, short[]> regionAtlases,
        Map<WindowKey, Float> regionMaxSpeeds
    ) {
    }

    private record NestedFeedbackAxisSpan(int index, int localMin, int localMax) {
    }

    private record L2ToL1FeedbackLayoutBin(
        int cellX,
        int layer,
        int cellZ,
        int localMinX,
        int localMaxX,
        int localMinY,
        int localMaxY,
        int localMinZ,
        int localMaxZ
    ) {
    }

    private record L2ToL1FeedbackLayout(
        int[] nativeLayout,
        List<L2ToL1FeedbackLayoutBin> bins
    ) {
    }

    private record NestedFeedbackRuntimeDiagnostics(
        long polledPacketCount,
        long polledBinCount,
        int lastPacketBinCount,
        int lastPolledTick,
        float lastMeanVolumeAverage,
        float lastMeanBottomFluxDensity,
        float lastMeanTopFluxDensity
    ) {
    }

    private record NativeNestedFeedbackWorldDiagnostics(
        int regionCount,
        int configuredBinCount,
        int stepsPerFeedback,
        int minStepsAccumulated,
        int maxStepsAccumulated,
        int readyRegionCount,
        long emittedPacketCount,
        long nativeResetCount,
        long backendResetCount,
        int lastBackendResetTick
    ) {
    }

    private static final class RegionRecord {
        private final AtomicBoolean busy = new AtomicBoolean(false);
        private final AtomicBoolean released = new AtomicBoolean(false);
        private boolean serviceActive;
        private boolean serviceReady;
        private boolean attached;
        private boolean staticUploaded;
        private boolean dynamicRestoreAttempted;
        private boolean fullWindowSectionsRequested;
        private boolean fansDirty = true;
        private boolean forcingDirty = true;
        private boolean backendResetPending;
        private final long[] uploadedSectionVersions = new long[WINDOW_SECTION_VOLUME];
        private WorldMirror.SectionSnapshot[] sections;
        private long[] sectionVersions;
        private int lastThermalRefreshTick;
        private float completedMaxSpeed;
        private List<FanSource> fans = List.of();
        private L2ToL1FeedbackLayout nestedFeedbackLayout;
        private long nestedFeedbackLayoutServiceId;
        private long backendResetCount;
        private int lastBackendResetTick = Integer.MIN_VALUE;
        private NestedBoundaryCoupler.BoundarySample cachedBoundarySample;
        private BoundaryFieldData cachedBoundaryField;
        private int cachedBoundaryExternalFaceMask;
        private int lastBoundaryRefreshTick = Integer.MIN_VALUE;
        private long uploadedBrickStaticSignature = Long.MIN_VALUE;
        private long uploadedBrickStaticServiceId;
        private int lastBrickDynamicSyncTick = Integer.MIN_VALUE;

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
            invalidateBrickDynamicSync();
        }

        private void markDetached() {
            attached = false;
            clearBoundaryCache();
            invalidateBrickDynamicSync();
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

        private NestedBoundaryCoupler.BoundarySample cachedBoundarySample() {
            return cachedBoundarySample;
        }

        private BoundaryFieldData cachedBoundaryField() {
            return cachedBoundaryField;
        }

        private int cachedBoundaryExternalFaceMask() {
            return cachedBoundaryExternalFaceMask;
        }

        private int lastBoundaryRefreshTick() {
            return lastBoundaryRefreshTick;
        }

        private void updateBoundaryCache(
            NestedBoundaryCoupler.BoundarySample boundarySample,
            BoundaryFieldData boundaryField,
            int externalFaceMask,
            int tick
        ) {
            cachedBoundarySample = boundarySample;
            cachedBoundaryField = boundaryField;
            cachedBoundaryExternalFaceMask = externalFaceMask;
            lastBoundaryRefreshTick = tick;
        }

        private void clearBoundaryCache() {
            cachedBoundarySample = null;
            cachedBoundaryField = null;
            cachedBoundaryExternalFaceMask = 0;
            lastBoundaryRefreshTick = Integer.MIN_VALUE;
        }

        private void noteBackendReset(int tick) {
            backendResetCount++;
            lastBackendResetTick = tick;
            invalidateBrickDynamicSync();
        }

        private long backendResetCount() {
            return backendResetCount;
        }

        private int lastBackendResetTick() {
            return lastBackendResetTick;
        }

        private boolean shouldSyncBrickDynamic(int tick) {
            return tick - lastBrickDynamicSyncTick >= BRICK_DYNAMIC_SYNC_INTERVAL_TICKS;
        }

        private void noteBrickDynamicSynced(int tick) {
            lastBrickDynamicSyncTick = tick;
        }

        private void invalidateBrickDynamicSync() {
            lastBrickDynamicSyncTick = Integer.MIN_VALUE;
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

    private static final class InspectionSolveSession {
        private final InspectionPatchInput input;
        private final int totalSteps;
        private final Path outputDir;
        private final AtomicBoolean stopRequested = new AtomicBoolean(false);
        private final AtomicBoolean completed = new AtomicBoolean(false);
        private final AtomicInteger completedSteps = new AtomicInteger(0);
        private final AtomicReference<String> phase = new AtomicReference<>("queued");
        private final AtomicReference<String> lastError = new AtomicReference<>("");
        private final AtomicReference<Float> maxSpeedMetersPerSecond = new AtomicReference<>(0.0f);

        private InspectionSolveSession(InspectionPatchInput input, int totalSteps) {
            this.input = input;
            this.totalSteps = totalSteps;
            this.outputDir = input.outputDir();
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
            return running.get() && thread.isAlive();
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
            try {
                while (running.get()) {
                    if (!streamingEnabled) {
                        lastCoordinatorState = "streamingDisabled";
                        publishedFrame.set(null);
                        sleepQuietly(20L);
                        continue;
                    }

                    flushPendingWorldDeltas();

                    int observedTick = tickCounter;
                    lastCoordinatorObservedTick = observedTick;
                    if (observedTick != lastSynchronizedTick) {
                        grantStepBudgetForObservedTicks();
                        synchronized (simulationStateLock) {
                            applyPendingActiveRegionBatchIfNeeded();
                            applyPendingBackgroundRefreshIfNeeded();
                            applyPendingResidentBrickStaticRefreshesIfNeeded();
                            refreshDesiredWindowsFromBrickRuntime();
                        }
                        runMesoscaleStepCycle();
                        synchronized (simulationStateLock) {
                            synchronizeActiveWindowsFromMirror();
                        }
                        lastSynchronizedTick = observedTick;
                    }

                    Set<WindowKey> residentWindowKeys;
                    Set<WindowKey> solveWindowKeys;
                    synchronized (simulationStateLock) {
                        residentWindowKeys = Set.copyOf(desiredWindowKeys);
                        solveWindowKeys = Set.copyOf(desiredSolveWindowKeys);
                    }
                    lastCoordinatorActiveWindowCount = residentWindowKeys.size();
                    lastCoordinatorSolveWindowCount = solveWindowKeys.size();
                    if (residentWindowKeys.isEmpty()) {
                        lastCoordinatorState = "noActiveWindows";
                        lastCoordinatorNoPublishReason = "noActiveWindows";
                        lastCoordinatorBusyWindowCount = 0;
                        publishedFrame.set(null);
                        sleepQuietly(10L);
                        continue;
                    }

                    lastCoordinatorBusyWindowCount = 0;

                    if (solveWindowKeys.isEmpty()) {
                        lastCoordinatorState = "noSolveWindows";
                        lastCoordinatorNoPublishReason = "noSolveWindows";
                        sleepQuietly(5L);
                        continue;
                    }

                    synchronized (simulationStateLock) {
                        queueCoherentL2CaptureIfNeeded(observedTick);
                    }

                    if (!acquireSimulationStepBudget()) {
                        lastCoordinatorState = "waitingTick";
                        lastCoordinatorNoPublishReason = "";
                        sleepQuietly(5L);
                        continue;
                    }

                    lastCoordinatorScheduledWindowCount = solveWindowKeys.size();
                    lastCoordinatorState = "brickEpoch";
                    long waitStartNanos = System.nanoTime();
                    synchronized (simulationStateLock) {
                        stepBrickRuntimeWorlds();
                    }
                    lastCoordinatorWaitNanos = System.nanoTime() - waitStartNanos;
                    if (!running.get()) {
                        return;
                    }

                    lastCoordinatorState = "postSolve";
                    long postSolveStartNanos = System.nanoTime();
                    synchronized (simulationStateLock) {
                        lastCoordinatorState = "sampleProbes";
                        publishedPlayerProbes.set(samplePlayerProbesLocked());
                        publishedEntitySamples.set(sampleEntitySamplesLocked());
                        lastCoordinatorState = "publishBrickAtlases";
                        float maxSpeedThisCycle = publishBrickSolveAtlases(solveWindowKeys);
                        lastCoordinatorAppliedMaxSpeed = maxSpeedThisCycle;
                        lastMaxFlowSpeed = maxSpeedThisCycle;
                        lastCoordinatorSolveCompleteTick = tickCounter;
                        lastCoordinatorState = "published";
                        lastCoordinatorNoPublishReason = "";
                        PublishedFrame currentFrame = publishedFrame.get();
                        if (currentFrame != null) {
                            lastCoordinatorPublishedMaxSpeed = currentFrame.maxSpeed();
                        } else {
                            lastCoordinatorPublishedMaxSpeed = 0.0f;
                        }
                    }
                    lastCoordinatorPostSolveNanos = System.nanoTime() - postSolveStartNanos;
                }
            } catch (Throwable ex) {
                running.set(false);
                lastCoordinatorError = ex.getClass().getSimpleName() + ": " + (ex.getMessage() == null ? "" : ex.getMessage());
                log("Simulation coordinator crashed: " + ex);
            } finally {
                running.set(false);
            }
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
            anchorDesiredWindowKeys = Set.copyOf(activeRegionKeys(batch.anchors()));
            anchorSolveWindowKeys = Set.copyOf(solveRegionKeys(batch.anchors()));
            syncBrickRuntimeHints(batch.brickRuntimeHintCoordsByWorld());
            activePlayerProbeRequests = batch.playerProbeRequests();
            activeEntitySampleRequests = batch.entitySampleRequests();
            worldEnvironmentSnapshots = batch.environmentSnapshots();
            lastActiveRegionBatchTick = batch.tickCounter();
        }

    }
}
