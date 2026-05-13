package com.aerodynamics4mc.client;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.ConcurrentLinkedQueue;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.aerodynamics4mc.FanBlock;
import com.aerodynamics4mc.ModBlocks;
import com.aerodynamics4mc.api.AeroWindSample;
import com.aerodynamics4mc.api.AeroWindSamplingRules;
import com.aerodynamics4mc.net.AeroCoarseWindPayload;
import com.aerodynamics4mc.runtime.NativeSimulationBridge;

import net.fabricmc.fabric.api.client.event.lifecycle.v1.ClientLifecycleEvents;
import net.fabricmc.fabric.api.client.event.lifecycle.v1.ClientTickEvents;
import net.fabricmc.fabric.api.client.networking.v1.ClientPlayConnectionEvents;
import net.minecraft.block.BlockState;
import net.minecraft.block.Blocks;
import net.minecraft.client.MinecraftClient;
import net.minecraft.client.world.ClientWorld;
import net.minecraft.state.property.Properties;
import net.minecraft.util.Identifier;
import net.minecraft.util.math.BlockPos;
import net.minecraft.util.math.Direction;
import net.minecraft.util.math.MathHelper;
import net.minecraft.util.math.Vec3d;

final class ClientL2Solver {
    private static final Logger LOGGER = LoggerFactory.getLogger("aerodynamics4mc/ClientL2Solver");

    private enum SolverMode {
        OFF(
            "off",
            false,
            false,
            32,
            NativeSimulationBridge.REALTIME_SOLVER_CLASSIC_D3Q27
        ),
        V0_1(
            "v0.1-classic-d3q27",
            true,
            false,
            32,
            NativeSimulationBridge.REALTIME_SOLVER_CLASSIC_D3Q27
        ),
        COMPACT(
            "compact-experimental",
            true,
            true,
            128,
            NativeSimulationBridge.REALTIME_SOLVER_COMPACT_EXPERIMENTAL
        ),
        FP16_INPLACE(
            "d3q27-fp16-inplace-experimental",
            true,
            true,
            128,
            NativeSimulationBridge.REALTIME_SOLVER_D3Q27_FP16_INPLACE_EXPERIMENTAL
        );

        private final String statusName;
        private final boolean enabledByDefault;
        private final boolean experimental;
        private final int defaultBrickSize;
        private final int nativeSolverMode;

        SolverMode(
            String statusName,
            boolean enabledByDefault,
            boolean experimental,
            int defaultBrickSize,
            int nativeSolverMode
        ) {
            this.statusName = statusName;
            this.enabledByDefault = enabledByDefault;
            this.experimental = experimental;
            this.defaultBrickSize = defaultBrickSize;
            this.nativeSolverMode = nativeSolverMode;
        }

        static SolverMode parse(String value) {
            if (value == null || value.isBlank()) {
                return V0_1;
            }
            String normalized = value.trim()
                .toLowerCase(java.util.Locale.ROOT)
                .replace('-', '_')
                .replace('.', '_')
                .replace(' ', '_');
            return switch (normalized) {
                case "off", "none", "disable", "disabled" -> OFF;
                case "default", "v0", "v01", "v0_1", "classic", "classic_d3q27",
                    "cumulant", "cumulant_d3q27", "d3q27", "32", "32_3" -> V0_1;
                case "compact", "compact_path", "compact_experimental", "compact_realtime", "compact_realtime_fp16" -> COMPACT;
                case "fp16", "fp16_inplace", "fp16_inplace_d3q27", "d3q27_fp16", "d3q27_fp16_inplace",
                    "d3q27_fp16_inplace_srt", "d3q27_fp16_inplace_experimental" -> FP16_INPLACE;
                default -> throw new IllegalArgumentException("Unknown Client L2 solver mode: " + value);
            };
        }

        String statusName() {
            return statusName;
        }

        boolean enabledByDefault() {
            return enabledByDefault;
        }

        boolean experimental() {
            return experimental;
        }

        int defaultBrickSize() {
            return defaultBrickSize;
        }

        int nativeSolverMode() {
            return nativeSolverMode;
        }
    }

    private static final SolverMode CLIENT_L2_MODE = configuredSolverMode();
    private static final int BRICK_SIZE = configuredBrickSize();
    private static final int CELL_COUNT = BRICK_SIZE * BRICK_SIZE * BRICK_SIZE;
    private static final int FLOW_CHANNELS = NativeSimulationBridge.FLOW_STATE_CHANNELS;
    private static final int PACKED_CHANNELS = NativeSimulationBridge.PACKED_ATLAS_CHANNELS;
    private static final int FACE_COUNT = Direction.values().length;
    private static final int STATIC_REFRESH_TICKS = -1;
    private static final int LOCAL_PUBLISH_INTERVAL_TICKS = configuredInt(
            "a4mc.clientL2.publishIntervalTicks",
            "AERO_LBM_CLIENT_L2_PUBLISH_INTERVAL_TICKS",
            1,
            1,
            200
    );
    private static final int LOCAL_PUBLISH_SAMPLE_STRIDE = configuredInt(
            "a4mc.clientL2.publishSampleStride",
            "AERO_LBM_CLIENT_L2_PUBLISH_SAMPLE_STRIDE",
            BRICK_SIZE >= 128 ? 4 : 1,
            1,
            16
    );
    private static final int SOLVE_INTERVAL_TICKS = configuredInt(
            "a4mc.clientL2.solveIntervalTicks",
            "AERO_LBM_CLIENT_L2_SOLVE_INTERVAL_TICKS",
            1,
            1,
            200
    );
    private static final int BOUNDARY_REFERENCE_REFRESH_MIN_TICKS = 40;
    private static final int FAST_SUSPEND_COOLDOWN_TICKS = 10;
    private static final int STATIC_BUILD_CELLS_PER_TICK = configuredInt(
        "a4mc.clientL2.staticBuildCellsPerTick",
        "AERO_LBM_CLIENT_L2_STATIC_BUILD_CELLS_PER_TICK",
        BRICK_SIZE >= 128 ? 8192 : 1024,
        1,
        CELL_COUNT
    );
    private static final int COARSE_SEED_CELLS_PER_TICK = configuredInt(
        "a4mc.clientL2.coarseSeedCellsPerTick",
        "AERO_LBM_CLIENT_L2_COARSE_SEED_CELLS_PER_TICK",
        BRICK_SIZE >= 128 ? 65536 : 4096,
        1,
        CELL_COUNT
    );
    private static final int BOUNDARY_REFERENCE_CELLS_PER_TICK = configuredInt(
            "a4mc.clientL2.boundaryReferenceCellsPerTick",
            "AERO_LBM_CLIENT_L2_BOUNDARY_REFERENCE_CELLS_PER_TICK",
            BRICK_SIZE >= 128 ? 16384 : 4096,
            1,
            CELL_COUNT
    );
    private static final long STATIC_BUILD_NANOS_PER_TICK = configuredInt(
        "a4mc.clientL2.staticBuildMicrosPerTick",
        "AERO_LBM_CLIENT_L2_STATIC_BUILD_MICROS_PER_TICK",
        BRICK_SIZE >= 128 ? 1500 : 1000,
        100,
        50000
    ) * 1000L;
    private static final long COARSE_SEED_NANOS_PER_TICK = configuredInt(
        "a4mc.clientL2.coarseSeedMicrosPerTick",
        "AERO_LBM_CLIENT_L2_COARSE_SEED_MICROS_PER_TICK",
        BRICK_SIZE >= 128 ? 1000 : 1000,
        100,
        50000
    ) * 1000L;
    private static final long BOUNDARY_REFERENCE_NANOS_PER_TICK = configuredInt(
        "a4mc.clientL2.boundaryReferenceMicrosPerTick",
        "AERO_LBM_CLIENT_L2_BOUNDARY_REFERENCE_MICROS_PER_TICK",
        BRICK_SIZE >= 128 ? 1000 : 1000,
        100,
        50000
    ) * 1000L;
    private static final int STRESS_PATCHES_PER_TICK = configuredInt(
            "a4mc.clientL2.stressPatchesPerTick",
            "AERO_LBM_CLIENT_L2_STRESS_PATCHES_PER_TICK",
            BRICK_SIZE >= 128 ? 512 : 64,
            1,
            CELL_COUNT
    );
    private static final int STRESS_INTERVAL_TICKS = configuredInt(
            "a4mc.clientL2.stressIntervalTicks",
            "AERO_LBM_CLIENT_L2_STRESS_INTERVAL_TICKS",
            1,
            1,
            200
    );
    private static final int STATIC_PATCH_DEBOUNCE_TICKS = configuredInt(
            "a4mc.clientL2.staticPatchDebounceTicks",
            "AERO_LBM_CLIENT_L2_STATIC_PATCH_DEBOUNCE_TICKS",
            0,
            0,
            20
    );
    private static final int BOUNDARY_REFRESH_AFTER_STATIC_PATCH_COOLDOWN_TICKS = configuredInt(
            "a4mc.clientL2.boundaryRefreshAfterStaticPatchCooldownTicks",
            "AERO_LBM_CLIENT_L2_BOUNDARY_REFRESH_AFTER_STATIC_PATCH_COOLDOWN_TICKS",
            20,
            0,
            200
    );
    private static final int STATIC_PATCH_MAX_DEBOUNCE_TICKS = configuredInt(
            "a4mc.clientL2.staticPatchMaxDebounceTicks",
            "AERO_LBM_CLIENT_L2_STATIC_PATCH_MAX_DEBOUNCE_TICKS",
            8,
            1,
            80
    );
    private static final int STATIC_PATCH_BULK_CHANGE_THRESHOLD = configuredInt(
            "a4mc.clientL2.staticPatchBulkChangeThreshold",
            "AERO_LBM_CLIENT_L2_STATIC_PATCH_BULK_CHANGE_THRESHOLD",
            8,
            1,
            CELL_COUNT
    );
    private static final int STATIC_PATCH_BULK_CELL_THRESHOLD = configuredInt(
            "a4mc.clientL2.staticPatchBulkCellThreshold",
            "AERO_LBM_CLIENT_L2_STATIC_PATCH_BULK_CELL_THRESHOLD",
            256,
            1,
            CELL_COUNT
    );
    private static final int STATIC_CACHE_MAX_BRICKS = configuredInt(
            "a4mc.clientL2.staticCacheMaxBricks",
            "AERO_LBM_CLIENT_L2_STATIC_CACHE_MAX_BRICKS",
            BRICK_SIZE >= 128 ? 2 : 32,
            0,
            64
    );
    private static final int COUPLING_BAND_CELLS = 8;
    private static final int MAX_CLIENT_ACTIVE_BRICKS = configuredInt(
            "a4mc.clientL2.maxActiveBricks",
            "AERO_LBM_CLIENT_L2_MAX_ACTIVE_BRICKS",
            BRICK_SIZE >= 128 ? 1 : 2,
            1,
            8
    );
    private static final int MAX_STEPS_PER_CLIENT_TICK = configuredInt(
            "a4mc.clientL2.maxStepsPerClientTick",
            "AERO_LBM_CLIENT_L2_MAX_STEPS_PER_CLIENT_TICK",
            1,
            1,
            16
    );
    private static final float DT_SECONDS = 0.05f;
    private static final float DX_METERS = 1.0f;
    private static final float NATIVE_VELOCITY_SCALE = DX_METERS / DT_SECONDS;
    private static final float ATLAS_VELOCITY_RANGE = 5.6f;
    private static final float ATLAS_PRESSURE_RANGE = 0.03f;
    private static final float ZERO_DYNAMIC_MAX_SPEED_EPS_MPS = 0.02f;
    private static final float COARSE_RESEED_MIN_SPEED_MPS = 0.05f;
    private static final float FAST_RESUME_HORIZONTAL_SPEED_MPS = 6.0f;
    private static final float HEAT_COUPLING_TO_ADJACENT_AIR = 0.85f;
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
    private static final int FAN_FORCE_LENGTH_CELLS = 5;
    private static final int FAN_FORCE_RADIUS_CELLS = 1;
    private static final int HEAT_PLUME_HEIGHT_CELLS = 4;
    private static final byte SURFACE_KIND_FAN_X_NEG = 32;
    private static final byte SURFACE_KIND_FAN_X_POS = 33;
    private static final byte SURFACE_KIND_FAN_Y_NEG = 34;
    private static final byte SURFACE_KIND_FAN_Y_POS = 35;
    private static final byte SURFACE_KIND_FAN_Z_NEG = 36;
    private static final byte SURFACE_KIND_FAN_Z_POS = 37;
    private static final int WORKER_QUEUE_CAPACITY = 128;
    private static final int STRESS_QUEUE_BACKLOG_LIMIT = configuredInt(
            "a4mc.clientL2.stressQueueBacklogLimit",
            "AERO_LBM_CLIENT_L2_STRESS_QUEUE_BACKLOG_LIMIT",
            16,
            0,
            WORKER_QUEUE_CAPACITY
    );
    private static final int ALL_OPEN_FACE_MASK = (1 << FACE_COUNT) - 1;
    private static final boolean CLIENT_L2_DEFAULT_ENABLED = CLIENT_L2_MODE != SolverMode.OFF && configuredBoolean(
        "a4mc.clientL2.enabled",
        "AERO_LBM_CLIENT_L2_ENABLED",
        CLIENT_L2_MODE.enabledByDefault()
    );

    private enum StressMode {
        OFF,
        FAN,
        THERMAL,
        DIRTY,
        MIXED;

        static StressMode parse(String value) {
            if (value == null || value.isBlank()) {
                return OFF;
            }
            return switch (value.trim().toLowerCase(java.util.Locale.ROOT)) {
                case "fan", "fans" -> FAN;
                case "thermal", "heat" -> THERMAL;
                case "dirty", "obstacle", "geometry" -> DIRTY;
                case "mixed", "all" -> MIXED;
                case "off", "disable", "disabled" -> OFF;
                default -> throw new IllegalArgumentException("Unknown Client L2 stress mode: " + value);
            };
        }
    }

    private enum StaticPatchFlushResult {
        NONE,
        DELAYED,
        SUBMITTED
    }

    private final AeroVisualizer visualizer;
    private final ClientL2Worker worker = new ClientL2Worker();
    private final byte[] obstacle = new byte[CELL_COUNT];
    private final byte[] surfaceKind = new byte[CELL_COUNT];
    private final short[] openFaceMask = new short[CELL_COUNT];
    private final float[] emitterPower = new float[CELL_COUNT];
    private final byte[] sourceFanDirection = new byte[CELL_COUNT];
    private final float[] sourceEmitterPower = new float[CELL_COUNT];
    private final byte[] faceSkyExposure = new byte[CELL_COUNT * FACE_COUNT];
    private final byte[] faceDirectExposure = new byte[CELL_COUNT * FACE_COUNT];
    private final float[] flowState = new float[CELL_COUNT * FLOW_CHANNELS];
    private final float[] airTemperature = new float[CELL_COUNT];
    private final float[] surfaceTemperature = new float[CELL_COUNT];
    private final BlockPos.MutableBlockPos staticCursor = new BlockPos.MutableBlockPos();
    private final BlockPos.MutableBlockPos staticNeighbor = new BlockPos.MutableBlockPos();
    private final LinkedHashMap<StaticBrickCacheKey, StaticBrickSnapshot> staticBrickCache =
            new LinkedHashMap<>(STATIC_CACHE_MAX_BRICKS, 0.75f, true) {
                @Override
                protected boolean removeEldestEntry(Map.Entry<StaticBrickCacheKey, StaticBrickSnapshot> eldest) {
                    return size() > STATIC_CACHE_MAX_BRICKS;
                }
            };
    private final int[] activeBrickX = new int[MAX_CLIENT_ACTIVE_BRICKS];
    private final int[] activeBrickY = new int[MAX_CLIENT_ACTIVE_BRICKS];
    private final int[] activeBrickZ = new int[MAX_CLIENT_ACTIVE_BRICKS];
    private final boolean[] activeBrickReady = new boolean[MAX_CLIENT_ACTIVE_BRICKS];
    private final boolean[] activeBrickRefreshPending = new boolean[MAX_CLIENT_ACTIVE_BRICKS];
    private final boolean[] activeBrickBoundaryRefreshPending = new boolean[MAX_CLIENT_ACTIVE_BRICKS];
    private int[] activeHintCoords = new int[NativeSimulationBridge.BRICK_HINT_COORDS_PER_BRICK];
    private final java.util.LinkedHashMap<BlockPos, PendingSourcePatch> pendingSourcePatches = new java.util.LinkedHashMap<>();
    private Identifier pendingStaticPatchDimension;
    private long pendingStaticPatchWorldKey;
    private long pendingStaticPatchFirstChangeGameTime = Long.MIN_VALUE;
    private long pendingStaticPatchLastChangeGameTime = Long.MIN_VALUE;
    private int pendingStaticPatchSourceChanges;

    private long worldKey;
    private BlockPos activeOrigin;
    private Identifier activeDimension;
    private boolean streamingEnabled;
    private boolean experimentalEnabled = CLIENT_L2_DEFAULT_ENABLED;
    private boolean activeHintUploaded;
    private boolean clientSolveDisabled;
    private int activeBrickCount;
    private int prepareCursor;
    private int refreshCursor;
    private int publishCursor;
    private int stagedActiveIndex = -1;
    private int stagedBrickX;
    private int stagedBrickY;
    private int stagedBrickZ;
    private int stagedStaticCursor;
    private int stagedSeedCursor;
    private boolean stagedStaticUploaded;
    private boolean stagedDynamicUploaded;
    private boolean stagedStaticFromCache;
    private boolean stagedCoarseSeedReady;
    private float stagedSeedVx;
    private float stagedSeedVy;
    private float stagedSeedVz;
    private float stagedSeedPressure;
    private BlockPos stagedOrigin;
    private Identifier stagedDimension;
    private int boundaryRefreshActiveIndex = -1;
    private int boundaryRefreshBrickX;
    private int boundaryRefreshBrickY;
    private int boundaryRefreshBrickZ;
    private int boundaryRefreshCursor;
    private float boundaryRefreshMaxCoarseSpeed;
    private BlockPos boundaryRefreshOrigin;
    private Identifier boundaryRefreshDimension;
    private int ticksSinceStaticRefresh = STATIC_REFRESH_TICKS;
    private long lastServerTick = Long.MIN_VALUE;
    private long lastProcessedClientGameTime = Long.MIN_VALUE;
    private long lastSolveClientGameTime = Long.MIN_VALUE;
    private long lastPublishedClientGameTime = Long.MIN_VALUE;
    private long lastBoundaryRefreshClientGameTime = Long.MIN_VALUE;
    private long lastStaticPatchSubmitClientGameTime = Long.MIN_VALUE;
    private long fastSuspendUntilGameTime = Long.MIN_VALUE;
    private long lastDiagnosticGameTime = Long.MIN_VALUE;
    private int lastStaticPatchCount;
    private int lastFanPatchCellCount;
    private int lastHeatPatchCellCount;
    private StressMode stressMode = StressMode.OFF;
    private long stressStartedGameTime = Long.MIN_VALUE;
    private long lastStressSubmitGameTime = Long.MIN_VALUE;
    private long stressSubmittedTicks;
    private long stressSubmittedPatches;
    private long stressSubmittedFanCells;
    private long stressSubmittedHeatCells;
    private long stressSubmittedDirtyCells;
    private boolean stressStaticSubmittedForActiveSet;

    ClientL2Solver(AeroVisualizer visualizer) {
        this.visualizer = visualizer;
    }

    private static SolverMode configuredSolverMode() {
        String value = System.getProperty("a4mc.clientL2.mode");
        if (value == null || value.isBlank()) {
            value = System.getenv("AERO_LBM_CLIENT_L2_MODE");
        }
        try {
            return SolverMode.parse(value);
        } catch (IllegalArgumentException error) {
            LOGGER.warn("Client L2 config a4mc.clientL2.mode={} is invalid; using v0_1", value);
            return SolverMode.V0_1;
        }
    }

    private static int configuredBrickSize() {
        int requested = configuredInt(
            "a4mc.clientL2.brickSize",
            "AERO_LBM_CLIENT_L2_BRICK_SIZE",
            CLIENT_L2_MODE.defaultBrickSize(),
            16,
            128
        );
        int aligned = Math.max(16, Math.min(128, (requested / 16) * 16));
        if (aligned != requested) {
            LOGGER.warn("Client L2 brick size {} is not chunk-aligned; using {}", requested, aligned);
        }
        return aligned;
    }

    private static int configuredInt(String propertyName, String envName, int defaultValue, int min, int max) {
        String value = System.getProperty(propertyName);
        if (value == null || value.isBlank()) {
            value = System.getenv(envName);
        }
        if (value == null || value.isBlank()) {
            return Mth.clamp(defaultValue, min, max);
        }
        try {
            int parsed = Integer.parseInt(value.trim());
            int clamped = Mth.clamp(parsed, min, max);
            if (clamped != parsed) {
                LOGGER.warn(
                        "Client L2 config {}={} outside [{}, {}]; using {}",
                        propertyName,
                        parsed,
                        min,
                        max,
                        clamped
                );
            }
            return clamped;
        } catch (NumberFormatException ignored) {
            LOGGER.warn("Client L2 config {}={} is not an integer; using {}", propertyName, value, defaultValue);
            return Mth.clamp(defaultValue, min, max);
        }
    }

    private static boolean configuredBoolean(String propertyName, String envName, boolean defaultValue) {
        String value = System.getProperty(propertyName);
        if (value == null || value.isBlank()) {
            value = System.getenv(envName);
        }
        if (value == null || value.isBlank()) {
            return defaultValue;
        }
        return switch (value.trim().toLowerCase(java.util.Locale.ROOT)) {
            case "1", "true", "yes", "on", "enable", "enabled" -> true;
            case "0", "false", "no", "off", "disable", "disabled" -> false;
            default -> {
                LOGGER.warn("Client L2 config {}={} is not a boolean; using {}", propertyName, value, defaultValue);
                yield defaultValue;
            }
        };
    }

    void initialize() {
        LOGGER.info(
            "Client L2 mode={} enabledByDefault={} brickSize={} experimental={}",
            CLIENT_L2_MODE.statusName(),
            CLIENT_L2_DEFAULT_ENABLED,
            BRICK_SIZE,
            CLIENT_L2_MODE.experimental()
        );
        ClientTickEvents.END_CLIENT_TICK.register(this::onClientTick);
        ClientPlayConnectionEvents.DISCONNECT.register((handler, client) -> close());
        ClientLifecycleEvents.CLIENT_STOPPING.register(client -> close());
    }

    void onRuntimeState(boolean streamingEnabled) {
        this.streamingEnabled = streamingEnabled;
        if (!streamingEnabled || !experimentalEnabled) {
            resetActiveBrick();
            worker.reset();
        }
    }

    void onCoarseWindField(AeroCoarseWindPayload payload) {
        if (!streamingEnabled || !experimentalEnabled || payload == null) {
            return;
        }
        long serverTick = payload.serverTick();
        if (serverTick < 0L) {
            return;
        }
        if (lastServerTick == Long.MIN_VALUE || serverTick != lastServerTick) {
            markBoundaryRefreshPending();
        }
        lastServerTick = serverTick;
    }

    void onBlockStateChanged(ClientLevel world, BlockPos pos, BlockState oldState, BlockState newState) {
        if (world == null || pos == null || oldState == newState || (oldState != null && oldState.equals(newState))) {
            return;
        }
        if (!experimentalEnabled) {
            return;
        }
        Identifier dimensionId = world.getRegistryKey().getValue();
        invalidateStaticCacheForPatchFootprint(dimensionId, pos, oldState, newState);
        if (!streamingEnabled || clientSolveDisabled || worldKey == 0L) {
            return;
        }
        if (activeDimension == null || !activeDimension.equals(dimensionId) || activeBrickCount <= 0) {
            return;
        }
        if (!blockPatchTouchesActiveBrick(pos)) {
            return;
        }
        queueStaticPatchPositions(dimensionId, worldKey, world.getGameTime(), pos, oldState, newState);
    }

    private void onClientTick(Minecraft client) {
        drainWorkerAtlases();
        if (!experimentalEnabled || !streamingEnabled || client.level == null || client.player == null) {
            return;
        }
        if (clientSolveDisabled) {
            return;
        }
        ClientLevel world = client.level;
        long clientGameTime = world.getGameTime();
        if (lastProcessedClientGameTime == clientGameTime) {
            return;
        }
        lastProcessedClientGameTime = clientGameTime;

        float horizontalSpeed = AeroWindSamplingRules.horizontalSpeedMetersPerSecond(client.player.getDeltaMovement());
        if (shouldSuspendForFastMovement(horizontalSpeed, clientGameTime)) {
            suspendForFastMovement(clientGameTime);
            return;
        }
        if (!worker.isNativeLoaded()) {
            maybeLog(client, "native library not loaded: " + worker.loadError());
            return;
        }

        Identifier dimensionId = world.dimension().identifier();
        BlockPos playerBlockPos = client.player.getOnPos();
        BlockPos origin = brickOrigin(playerBlockPos);
        int brickX = Math.floorDiv(origin.getX(), BRICK_SIZE);
        int brickY = Math.floorDiv(origin.getY(), BRICK_SIZE);
        int brickZ = Math.floorDiv(origin.getZ(), BRICK_SIZE);
        int localX = playerBlockPos.getX() - origin.getX();
        int localY = playerBlockPos.getY() - origin.getY();
        int localZ = playerBlockPos.getZ() - origin.getZ();
        boolean dimensionChanged = activeDimension == null || !activeDimension.equals(dimensionId);
        boolean originChanged = activeOrigin == null
            || !activeOrigin.equals(origin)
            || dimensionChanged;
        boolean activeSetChanged = originChanged || !activeSetMatches(brickX, brickY, brickZ, localX, localY, localZ);
        if (activeSetChanged) {
            activeOrigin = origin;
            activeDimension = dimensionId;
            worldKey = worldKey(dimensionId);
            activeHintUploaded = false;
            buildActiveBrickSet(brickX, brickY, brickZ, localX, localY, localZ);
            clearPendingStaticPatches();
            cancelStagedPreparation();
            cancelBoundaryReferenceRefresh();
            lastPublishedClientGameTime = Long.MIN_VALUE;
            lastSolveClientGameTime = Long.MIN_VALUE;
            lastBoundaryRefreshClientGameTime = Long.MIN_VALUE;
            lastStaticPatchSubmitClientGameTime = Long.MIN_VALUE;
            stressStaticSubmittedForActiveSet = false;
            if (dimensionChanged) {
                visualizer.clearLocalFlowFields();
            }
            ticksSinceStaticRefresh = 0;
        }

        if (!activeHintUploaded) {
            worker.submitActiveHints(worldKey, activeHintCoords);
            activeHintUploaded = true;
        }

        prepareActiveBricks(client, world, dimensionId);
        if (!hasReadyActiveBrick()) {
            return;
        }
        if (refreshActiveBrickStatic(client, world)) {
            return;
        }
        StaticPatchFlushResult staticPatchFlush = flushPendingStaticPatches(world, dimensionId, clientGameTime);
        if (staticPatchFlush == StaticPatchFlushResult.DELAYED) {
            return;
        }
        boolean submittedStaticPatch = staticPatchFlush == StaticPatchFlushResult.SUBMITTED;
        if (!submittedStaticPatch && refreshActiveBrickBoundaryReference(client, dimensionId, clientGameTime)) {
            return;
        }
        maybeSubmitStressDeltas(dimensionId, clientGameTime);
        if (!submittedStaticPatch
                && lastSolveClientGameTime != Long.MIN_VALUE
                && clientGameTime - lastSolveClientGameTime < SOLVE_INTERVAL_TICKS) {
            return;
        }
        boolean publish = lastPublishedClientGameTime == Long.MIN_VALUE
                || clientGameTime - lastPublishedClientGameTime >= LOCAL_PUBLISH_INTERVAL_TICKS;
        worker.requestStep(worldKey, publishTargets(dimensionId, publish), MAX_STEPS_PER_CLIENT_TICK);
        lastSolveClientGameTime = clientGameTime;
        if (publish) {
            lastPublishedClientGameTime = clientGameTime;
        }
    }

    private void drainWorkerAtlases() {
        LocalAtlasSnapshot snapshot;
        while ((snapshot = worker.pollAtlas()) != null) {
            visualizer.onLocalFlowField(
                    snapshot.dimensionId(),
                    snapshot.origin(),
                    snapshot.sampleStride(),
                    snapshot.packedFlow()
            );
        }
    }

    private PublishTarget[] publishTargets(Identifier dimensionId, boolean publish) {
        if (!publish || activeBrickCount <= 0) {
            return new PublishTarget[0];
        }
        PublishTarget[] targets = new PublishTarget[activeBrickCount];
        int count = 0;
        for (int attempts = 0; attempts < activeBrickCount; attempts++) {
            int index = (publishCursor + attempts) % activeBrickCount;
            if (!activeBrickReady[index]) {
                continue;
            }
            int brickX = activeBrickX[index];
            int brickY = activeBrickY[index];
            int brickZ = activeBrickZ[index];
            targets[count++] = new PublishTarget(dimensionId, brickOrigin(brickX, brickY, brickZ), brickX, brickY, brickZ);
        }
        publishCursor = activeBrickCount <= 0 ? 0 : (publishCursor + 1) % activeBrickCount;
        return java.util.Arrays.copyOf(targets, count);
    }

    private void maybeSubmitStressDeltas(Identifier dimensionId, long clientGameTime) {
        if (stressMode == StressMode.OFF || worldKey == 0L || activeBrickCount <= 0) {
            return;
        }
        if (activeDimension == null || !activeDimension.equals(dimensionId)) {
            return;
        }
        if (lastStressSubmitGameTime != Long.MIN_VALUE
                && clientGameTime - lastStressSubmitGameTime < STRESS_INTERVAL_TICKS) {
            return;
        }
        if (stressStaticSubmittedForActiveSet && stressModeIsStatic(stressMode)) {
            return;
        }
        if (worker.queueSize() > STRESS_QUEUE_BACKLOG_LIMIT) {
            return;
        }
        int activeIndex = firstReadyActiveBrickIndex();
        if (activeIndex < 0) {
            return;
        }
        BlockPos origin = brickOrigin(
                activeBrickX[activeIndex],
                activeBrickY[activeIndex],
                activeBrickZ[activeIndex]
        );
        NativeSimulationBridge.WorldDelta[] deltas = new NativeSimulationBridge.WorldDelta[STRESS_PATCHES_PER_TICK];
        int count = 0;
        int fanCells = 0;
        int heatCells = 0;
        int dirtyCells = 0;
        for (int i = 0; i < STRESS_PATCHES_PER_TICK; i++) {
            StressMode cellMode = stressCellMode(i);
            int seed = stressSeed(activeIndex, clientGameTime, i);
            int x = stressLocalCoord(seed);
            int y = stressLocalCoord(seed >>> 7);
            int z = stressLocalCoord(seed >>> 17);
            boolean solid = false;
            byte syntheticSurfaceKind = 0;
            float syntheticEmitterPower = 0.0f;
            int syntheticOpenFaceMask = ALL_OPEN_FACE_MASK;

            if (cellMode == StressMode.FAN) {
                syntheticSurfaceKind = fanSurfaceKind(stressFanDirection(i, clientGameTime));
                fanCells++;
            } else if (cellMode == StressMode.THERMAL) {
                syntheticEmitterPower = THERMAL_EMITTER_POWER_FIRE_W;
                heatCells++;
            } else if (cellMode == StressMode.DIRTY) {
                solid = (stressMix(seed ^ (int) (clientGameTime / 8L)) & 1) == 0;
                syntheticOpenFaceMask = solid ? 0 : ALL_OPEN_FACE_MASK;
                dirtyCells++;
            }

            int packedState = (solid ? 1 : 0)
                    | ((Byte.toUnsignedInt(syntheticSurfaceKind) & 0xFF) << 8);
            deltas[count++] = new NativeSimulationBridge.WorldDelta(
                    NativeSimulationBridge.WORLD_DELTA_BRICK_STATIC_CELL_PATCH,
                    origin.getX() + x,
                    origin.getY() + y,
                    origin.getZ() + z,
                    (int) worldKey,
                    packedState,
                    syntheticOpenFaceMask,
                    0,
                    syntheticEmitterPower,
                    0.0f,
                    0.0f,
                    0.0f
            );
        }
        if (count == 0) {
            return;
        }
        NativeSimulationBridge.WorldDelta[] submitted = count == deltas.length
                ? deltas
                : java.util.Arrays.copyOf(deltas, count);
        if (stressStartedGameTime == Long.MIN_VALUE) {
            stressStartedGameTime = clientGameTime;
        }
        lastStressSubmitGameTime = clientGameTime;
        stressSubmittedTicks++;
        stressSubmittedPatches += submitted.length;
        stressSubmittedFanCells += fanCells;
        stressSubmittedHeatCells += heatCells;
        stressSubmittedDirtyCells += dirtyCells;
        lastStaticPatchCount = submitted.length;
        lastFanPatchCellCount = fanCells;
        lastHeatPatchCellCount = heatCells;
        if (stressModeIsStatic(stressMode)) {
            stressStaticSubmittedForActiveSet = true;
        }
        worker.submitWorldDeltas(worldKey, submitted);
    }

    private boolean stressModeIsStatic(StressMode mode) {
        return mode == StressMode.FAN || mode == StressMode.THERMAL;
    }

    private StressMode stressCellMode(int index) {
        if (stressMode != StressMode.MIXED) {
            return stressMode;
        }
        return switch (index & 3) {
            case 0 -> StressMode.FAN;
            case 1 -> StressMode.THERMAL;
            default -> StressMode.DIRTY;
        };
    }

    private int firstReadyActiveBrickIndex() {
        for (int index = 0; index < activeBrickCount; index++) {
            if (activeBrickReady[index]) {
                return index;
            }
        }
        return -1;
    }

    private int stressSeed(int activeIndex, long clientGameTime, int index) {
        int seed = (int) clientGameTime;
        seed ^= index * 0x9E3779B9;
        seed ^= activeBrickX[activeIndex] * 0x85EBCA6B;
        seed ^= activeBrickY[activeIndex] * 0xC2B2AE35;
        seed ^= activeBrickZ[activeIndex] * 0x27D4EB2D;
        return stressMix(seed);
    }

    private int stressLocalCoord(int seed) {
        int innerSpan = Math.max(1, BRICK_SIZE - 4);
        return 2 + Math.floorMod(stressMix(seed), innerSpan);
    }

    private Direction stressFanDirection(int index, long clientGameTime) {
        Direction[] directions = Direction.values();
        return directions[Math.floorMod(index + (int) clientGameTime, directions.length)];
    }

    private static int stressMix(int value) {
        value ^= value >>> 16;
        value *= 0x7FEB352D;
        value ^= value >>> 15;
        value *= 0x846CA68B;
        value ^= value >>> 16;
        return value;
    }

    private boolean shouldSuspendForFastMovement(float horizontalSpeedMetersPerSecond, long clientGameTime) {
        if (horizontalSpeedMetersPerSecond > AeroWindSamplingRules.FAST_PLAYER_HORIZONTAL_SPEED_THRESHOLD_MPS) {
            return true;
        }
        if (fastSuspendUntilGameTime != Long.MIN_VALUE && clientGameTime < fastSuspendUntilGameTime) {
            return horizontalSpeedMetersPerSecond > FAST_RESUME_HORIZONTAL_SPEED_MPS;
        }
        fastSuspendUntilGameTime = Long.MIN_VALUE;
        return false;
    }

    private void suspendForFastMovement(long clientGameTime) {
        fastSuspendUntilGameTime = clientGameTime + FAST_SUSPEND_COOLDOWN_TICKS;
        if (activeBrickCount > 0 || activeOrigin != null) {
            resetActiveBrick();
        } else {
            visualizer.clearLocalFlowFields();
        }
    }

    private boolean activeSetMatches(
            int coreBrickX,
            int coreBrickY,
            int coreBrickZ,
            int localX,
            int localY,
            int localZ
    ) {
        int expectedIndex = 0;
        if (!activeBrickMatches(expectedIndex++, coreBrickX, coreBrickY, coreBrickZ)) {
            return false;
        }
        int[] neighborOffset = MAX_CLIENT_ACTIVE_BRICKS > 1
                ? nearestBoundaryNeighborOffset(localX, localY, localZ)
                : null;
        if (neighborOffset != null
                && !activeBrickMatches(
                expectedIndex++,
                coreBrickX + neighborOffset[0],
                coreBrickY + neighborOffset[1],
                coreBrickZ + neighborOffset[2]
        )) {
            return false;
        }
        return activeBrickCount == expectedIndex;
    }

    private int[] nearestBoundaryNeighborOffset(int localX, int localY, int localZ) {
        int bestDistance = COUPLING_BAND_CELLS;
        int bestX = 0;
        int bestY = 0;
        int bestZ = 0;
        if (localX < bestDistance) {
            bestDistance = localX;
            bestX = -1;
            bestY = 0;
            bestZ = 0;
        }
        int distance = BRICK_SIZE - 1 - localX;
        if (distance < bestDistance) {
            bestDistance = distance;
            bestX = 1;
            bestY = 0;
            bestZ = 0;
        }
        if (localY < bestDistance) {
            bestDistance = localY;
            bestX = 0;
            bestY = -1;
            bestZ = 0;
        }
        distance = BRICK_SIZE - 1 - localY;
        if (distance < bestDistance) {
            bestDistance = distance;
            bestX = 0;
            bestY = 1;
            bestZ = 0;
        }
        if (localZ < bestDistance) {
            bestDistance = localZ;
            bestX = 0;
            bestY = 0;
            bestZ = -1;
        }
        distance = BRICK_SIZE - 1 - localZ;
        if (distance < bestDistance) {
            bestDistance = distance;
            bestX = 0;
            bestY = 0;
            bestZ = 1;
        }
        return bestDistance < COUPLING_BAND_CELLS ? new int[]{bestX, bestY, bestZ} : null;
    }

    private boolean activeBrickMatches(int index, int brickX, int brickY, int brickZ) {
        return index < activeBrickCount
                && activeBrickX[index] == brickX
                && activeBrickY[index] == brickY
                && activeBrickZ[index] == brickZ;
    }

    private void buildActiveBrickSet(
            int coreBrickX,
            int coreBrickY,
            int coreBrickZ,
            int localX,
            int localY,
            int localZ
    ) {
        int oldActiveBrickCount = activeBrickCount;
        int[] oldActiveBrickX = java.util.Arrays.copyOf(activeBrickX, oldActiveBrickCount);
        int[] oldActiveBrickY = java.util.Arrays.copyOf(activeBrickY, oldActiveBrickCount);
        int[] oldActiveBrickZ = java.util.Arrays.copyOf(activeBrickZ, oldActiveBrickCount);
        boolean[] oldActiveBrickReady = java.util.Arrays.copyOf(activeBrickReady, oldActiveBrickCount);
        activeBrickCount = 0;
        prepareCursor = 0;
        refreshCursor = 0;
        publishCursor = 0;
        activeHintCoords = new int[MAX_CLIENT_ACTIVE_BRICKS * NativeSimulationBridge.BRICK_HINT_COORDS_PER_BRICK];
        java.util.Arrays.fill(activeBrickReady, false);
        java.util.Arrays.fill(activeBrickRefreshPending, false);
        java.util.Arrays.fill(activeBrickBoundaryRefreshPending, false);
        addActiveBrick(
                coreBrickX,
                coreBrickY,
                coreBrickZ,
                oldActiveBrickX,
                oldActiveBrickY,
                oldActiveBrickZ,
                oldActiveBrickReady
        );
        int[] neighborOffset = MAX_CLIENT_ACTIVE_BRICKS > 1
                ? nearestBoundaryNeighborOffset(localX, localY, localZ)
                : null;
        if (neighborOffset != null) {
            addActiveBrick(
                    coreBrickX + neighborOffset[0],
                    coreBrickY + neighborOffset[1],
                    coreBrickZ + neighborOffset[2],
                    oldActiveBrickX,
                    oldActiveBrickY,
                    oldActiveBrickZ,
                    oldActiveBrickReady
            );
        }
        activeHintCoords = java.util.Arrays.copyOf(
                activeHintCoords,
                activeBrickCount * NativeSimulationBridge.BRICK_HINT_COORDS_PER_BRICK
        );
    }

    private void addActiveBrick(
            int brickX,
            int brickY,
            int brickZ,
            int[] oldActiveBrickX,
            int[] oldActiveBrickY,
            int[] oldActiveBrickZ,
            boolean[] oldActiveBrickReady
    ) {
        if (activeBrickCount >= MAX_CLIENT_ACTIVE_BRICKS) {
            return;
        }
        int index = activeBrickCount++;
        activeBrickX[index] = brickX;
        activeBrickY[index] = brickY;
        activeBrickZ[index] = brickZ;
        activeBrickReady[index] = wasActiveBrickReady(
                brickX,
                brickY,
                brickZ,
                oldActiveBrickX,
                oldActiveBrickY,
                oldActiveBrickZ,
                oldActiveBrickReady
        );
        activeBrickBoundaryRefreshPending[index] = true;
        int hintBase = index * NativeSimulationBridge.BRICK_HINT_COORDS_PER_BRICK;
        activeHintCoords[hintBase] = brickX;
        activeHintCoords[hintBase + 1] = brickY;
        activeHintCoords[hintBase + 2] = brickZ;
    }

    private boolean wasActiveBrickReady(
            int brickX,
            int brickY,
            int brickZ,
            int[] oldActiveBrickX,
            int[] oldActiveBrickY,
            int[] oldActiveBrickZ,
            boolean[] oldActiveBrickReady
    ) {
        for (int index = 0; index < oldActiveBrickReady.length; index++) {
            if (oldActiveBrickReady[index]
                    && oldActiveBrickX[index] == brickX
                    && oldActiveBrickY[index] == brickY
                    && oldActiveBrickZ[index] == brickZ) {
                return true;
            }
        }
        return false;
    }

    private boolean prepareActiveBricks(Minecraft client, ClientLevel world, Identifier dimensionId) {
        if (activeBrickCount <= 0) {
            return false;
        }
        for (int attempts = 0; attempts < activeBrickCount; attempts++) {
            int index = (prepareCursor + attempts) % activeBrickCount;
            if (activeBrickReady[index]) {
                continue;
            }
            BrickPreparationResult result = uploadAndSeedActiveBrick(client, world, dimensionId, index);
            if (result == BrickPreparationResult.IN_PROGRESS) {
                return false;
            }
            if (result == BrickPreparationResult.FAILED) {
                return false;
            }
            activeBrickReady[index] = true;
            activeBrickBoundaryRefreshPending[index] = false;
            prepareCursor = (index + 1) % activeBrickCount;
            return false;
        }
        return true;
    }

    private boolean hasReadyActiveBrick() {
        for (int index = 0; index < activeBrickCount; index++) {
            if (activeBrickReady[index]) {
                return true;
            }
        }
        return false;
    }

    private BrickPreparationResult uploadAndSeedActiveBrick(
            Minecraft client,
            ClientLevel world,
            Identifier dimensionId,
            int activeIndex
    ) {
        int brickX = activeBrickX[activeIndex];
        int brickY = activeBrickY[activeIndex];
        int brickZ = activeBrickZ[activeIndex];
        BlockPos origin = brickOrigin(brickX, brickY, brickZ);
        if (!stagedPreparationMatches(activeIndex, dimensionId, brickX, brickY, brickZ)) {
            beginStagedPreparation(activeIndex, dimensionId, origin, brickX, brickY, brickZ);
        }
        if (!buildStagedStaticCells(world)) {
            return BrickPreparationResult.IN_PROGRESS;
        }
        cacheStagedStaticBrickIfNeeded();
        stagedStaticUploaded = true;
        if (!buildStagedCoarseSeedCells(dimensionId)) {
            return BrickPreparationResult.IN_PROGRESS;
        }
        if (!stagedDynamicUploaded) {
            worker.submitBrickSeed(new BrickSeedCommand(
                    worldKey,
                    brickX,
                    brickY,
                    brickZ,
                    java.util.Arrays.copyOf(obstacle, obstacle.length),
                    java.util.Arrays.copyOf(surfaceKind, surfaceKind.length),
                    java.util.Arrays.copyOf(openFaceMask, openFaceMask.length),
                    java.util.Arrays.copyOf(emitterPower, emitterPower.length),
                    java.util.Arrays.copyOf(sourceFanDirection, sourceFanDirection.length),
                    java.util.Arrays.copyOf(sourceEmitterPower, sourceEmitterPower.length),
                    java.util.Arrays.copyOf(faceSkyExposure, faceSkyExposure.length),
                    java.util.Arrays.copyOf(faceDirectExposure, faceDirectExposure.length),
                    java.util.Arrays.copyOf(flowState, flowState.length),
                    java.util.Arrays.copyOf(airTemperature, airTemperature.length),
                    java.util.Arrays.copyOf(surfaceTemperature, surfaceTemperature.length)
            ));
            stagedDynamicUploaded = true;
            return BrickPreparationResult.IN_PROGRESS;
        }
        cancelStagedPreparation();
        return BrickPreparationResult.COMPLETED;
    }

    private boolean stagedPreparationMatches(
            int activeIndex,
            Identifier dimensionId,
            int brickX,
            int brickY,
            int brickZ
    ) {
        return stagedActiveIndex == activeIndex
                && stagedDimension != null
                && stagedDimension.equals(dimensionId)
                && stagedBrickX == brickX
                && stagedBrickY == brickY
                && stagedBrickZ == brickZ;
    }

    private void beginStagedPreparation(
            int activeIndex,
            Identifier dimensionId,
            BlockPos origin,
            int brickX,
            int brickY,
            int brickZ
    ) {
        stagedActiveIndex = activeIndex;
        stagedDimension = dimensionId;
        stagedOrigin = origin;
        stagedBrickX = brickX;
        stagedBrickY = brickY;
        stagedBrickZ = brickZ;
        stagedStaticCursor = 0;
        stagedSeedCursor = 0;
        stagedStaticUploaded = false;
        stagedDynamicUploaded = false;
        stagedStaticFromCache = false;
        stagedCoarseSeedReady = false;
        stagedSeedVx = 0.0f;
        stagedSeedVy = 0.0f;
        stagedSeedVz = 0.0f;
        stagedSeedPressure = 0.0f;
        java.util.Arrays.fill(obstacle, (byte) 0);
        java.util.Arrays.fill(surfaceKind, (byte) 0);
        java.util.Arrays.fill(openFaceMask, (short) 0);
        java.util.Arrays.fill(emitterPower, 0.0f);
        java.util.Arrays.fill(sourceFanDirection, (byte) 0);
        java.util.Arrays.fill(sourceEmitterPower, 0.0f);
        java.util.Arrays.fill(faceSkyExposure, (byte) 0);
        java.util.Arrays.fill(faceDirectExposure, (byte) 0);
        java.util.Arrays.fill(flowState, 0.0f);
        java.util.Arrays.fill(airTemperature, 0.0f);
        java.util.Arrays.fill(surfaceTemperature, 0.0f);
        StaticBrickSnapshot cached = staticBrickCache.get(new StaticBrickCacheKey(dimensionId, brickX, brickY, brickZ));
        if (cached != null) {
            cached.copyInto(obstacle, surfaceKind, openFaceMask, emitterPower, sourceFanDirection, sourceEmitterPower);
            stagedStaticCursor = CELL_COUNT;
            stagedStaticFromCache = true;
        }
    }

    private boolean buildStagedStaticCells(ClientLevel world) {
        if (stagedOrigin == null) {
            return false;
        }
        long deadline = System.nanoTime() + STATIC_BUILD_NANOS_PER_TICK;
        int end = Math.min(CELL_COUNT, stagedStaticCursor + STATIC_BUILD_CELLS_PER_TICK);
        int built = 0;
        while (stagedStaticCursor < end) {
            int cell = stagedStaticCursor++;
            int x = cell / (BRICK_SIZE * BRICK_SIZE);
            int rem = cell - x * BRICK_SIZE * BRICK_SIZE;
            int y = rem / BRICK_SIZE;
            int z = rem - y * BRICK_SIZE;
            populateStaticCell(world, stagedOrigin, x, y, z);
            built++;
            if ((built & 255) == 0 && System.nanoTime() >= deadline) {
                break;
            }
        }
        return stagedStaticCursor >= CELL_COUNT;
    }

    private void cacheStagedStaticBrickIfNeeded() {
        if (stagedStaticFromCache || stagedDimension == null || stagedOrigin == null) {
            return;
        }
        staticBrickCache.put(
                new StaticBrickCacheKey(stagedDimension, stagedBrickX, stagedBrickY, stagedBrickZ),
                StaticBrickSnapshot.copyFrom(obstacle, surfaceKind, openFaceMask, emitterPower, sourceFanDirection, sourceEmitterPower)
        );
        stagedStaticFromCache = true;
    }

    private boolean buildStagedCoarseSeedCells(Identifier dimensionId) {
        if (stagedOrigin == null) {
            return false;
        }
        if (!stagedCoarseSeedReady) {
            Vec3d center = new Vec3d(
                stagedOrigin.getX() + BRICK_SIZE * 0.5,
                stagedOrigin.getY() + BRICK_SIZE * 0.5,
                stagedOrigin.getZ() + BRICK_SIZE * 0.5
            );
            AeroWindSample coarse = visualizer.sampleServerCoarseFlow(dimensionId, center);
            if (!coarse.hasFlow()) {
                return false;
            }
            stagedSeedVx = coarse.velocityX() / NATIVE_VELOCITY_SCALE;
            stagedSeedVy = coarse.velocityY() / NATIVE_VELOCITY_SCALE;
            stagedSeedVz = coarse.velocityZ() / NATIVE_VELOCITY_SCALE;
            stagedSeedPressure = coarse.pressure();
            stagedCoarseSeedReady = true;
        }
        long deadline = System.nanoTime() + COARSE_SEED_NANOS_PER_TICK;
        int built = 0;
        while (stagedSeedCursor < CELL_COUNT && built < COARSE_SEED_CELLS_PER_TICK) {
            int cell = stagedSeedCursor;
            int base = cell * FLOW_CHANNELS;
            if (obstacle[cell] == 0) {
                flowState[base] = stagedSeedVx;
                flowState[base + 1] = stagedSeedVy;
                flowState[base + 2] = stagedSeedVz;
                flowState[base + 3] = stagedSeedPressure;
            } else {
                flowState[base] = 0.0f;
                flowState[base + 1] = 0.0f;
                flowState[base + 2] = 0.0f;
                flowState[base + 3] = 0.0f;
            }
            stagedSeedCursor++;
            built++;
            if ((built & 1023) == 0 && System.nanoTime() >= deadline) {
                break;
            }
        }
        return stagedSeedCursor >= CELL_COUNT;
    }

    private void cancelStagedPreparation() {
        stagedActiveIndex = -1;
        stagedOrigin = null;
        stagedDimension = null;
        stagedBrickX = 0;
        stagedBrickY = 0;
        stagedBrickZ = 0;
        stagedStaticCursor = 0;
        stagedSeedCursor = 0;
        stagedStaticUploaded = false;
        stagedDynamicUploaded = false;
        stagedStaticFromCache = false;
        stagedCoarseSeedReady = false;
        stagedSeedVx = 0.0f;
        stagedSeedVy = 0.0f;
        stagedSeedVz = 0.0f;
        stagedSeedPressure = 0.0f;
    }

    private boolean refreshActiveBrickStatic(Minecraft client, ClientLevel world) {
        if (STATIC_REFRESH_TICKS <= 0) {
            java.util.Arrays.fill(activeBrickRefreshPending, false);
            return false;
        }
        if (!hasRefreshPending()) {
            if (ticksSinceStaticRefresh++ < STATIC_REFRESH_TICKS) {
                return false;
            }
            java.util.Arrays.fill(activeBrickRefreshPending, 0, activeBrickCount, true);
            refreshCursor = 0;
            ticksSinceStaticRefresh = 0;
        }
        for (int attempts = 0; attempts < activeBrickCount; attempts++) {
            int index = (refreshCursor + attempts) % activeBrickCount;
            if (!activeBrickRefreshPending[index]) {
                continue;
            }
            int brickX = activeBrickX[index];
            int brickY = activeBrickY[index];
            int brickZ = activeBrickZ[index];
            uploadStaticBrick(world, brickOrigin(brickX, brickY, brickZ), brickX, brickY, brickZ);
            activeBrickRefreshPending[index] = false;
            refreshCursor = (index + 1) % activeBrickCount;
            return true;
        }
        return false;
    }

    private boolean hasRefreshPending() {
        for (int index = 0; index < activeBrickCount; index++) {
            if (activeBrickRefreshPending[index]) {
                return true;
            }
        }
        return false;
    }

    private void markBoundaryRefreshPending() {
        for (int index = 0; index < activeBrickCount; index++) {
            if (activeBrickReady[index]) {
                activeBrickBoundaryRefreshPending[index] = true;
            }
        }
    }

    private boolean refreshActiveBrickBoundaryReference(
            Minecraft client,
            Identifier dimensionId,
            long clientGameTime
    ) {
        if (!hasBoundaryRefreshPending() && boundaryRefreshActiveIndex < 0) {
            return false;
        }
        if (lastStaticPatchSubmitClientGameTime != Long.MIN_VALUE
                && clientGameTime - lastStaticPatchSubmitClientGameTime < BOUNDARY_REFRESH_AFTER_STATIC_PATCH_COOLDOWN_TICKS) {
            return false;
        }
        if (stagedActiveIndex >= 0) {
            return false;
        }
        if (boundaryRefreshActiveIndex < 0
                && lastBoundaryRefreshClientGameTime != Long.MIN_VALUE
                && clientGameTime - lastBoundaryRefreshClientGameTime < BOUNDARY_REFERENCE_REFRESH_MIN_TICKS) {
            return false;
        }
        for (int attempts = 0; attempts < activeBrickCount; attempts++) {
            int index = boundaryRefreshActiveIndex >= 0
                    ? boundaryRefreshActiveIndex
                    : (refreshCursor + attempts) % activeBrickCount;
            if (!activeBrickBoundaryRefreshPending[index]) {
                if (boundaryRefreshActiveIndex >= 0) {
                    cancelBoundaryReferenceRefresh();
                }
                continue;
            }
            if (!activeBrickReady[index]) {
                activeBrickBoundaryRefreshPending[index] = false;
                if (boundaryRefreshActiveIndex >= 0) {
                    cancelBoundaryReferenceRefresh();
                }
                continue;
            }
            int brickX = activeBrickX[index];
            int brickY = activeBrickY[index];
            int brickZ = activeBrickZ[index];
            BlockPos origin = brickOrigin(brickX, brickY, brickZ);
            if (!boundaryReferenceRefreshMatches(index, dimensionId, brickX, brickY, brickZ)) {
                beginBoundaryReferenceRefresh(index, dimensionId, origin, brickX, brickY, brickZ);
            }
            BoundaryReferenceBuildResult result = buildBoundaryReferenceCells(dimensionId);
            if (result == BoundaryReferenceBuildResult.WAITING_FOR_COARSE) {
                maybeLog(client, "client L2 boundary refresh waiting for coarse field");
                return false;
            }
            if (result == BoundaryReferenceBuildResult.IN_PROGRESS) {
                return false;
            }
            worker.submitBoundaryReference(new BoundaryReferenceCommand(
                    worldKey,
                    brickX,
                    brickY,
                    brickZ,
                    java.util.Arrays.copyOf(flowState, flowState.length),
                    java.util.Arrays.copyOf(airTemperature, airTemperature.length),
                    java.util.Arrays.copyOf(surfaceTemperature, surfaceTemperature.length),
                    boundaryRefreshMaxCoarseSpeed
            ));
            activeBrickBoundaryRefreshPending[index] = false;
            refreshCursor = (index + 1) % activeBrickCount;
            lastBoundaryRefreshClientGameTime = clientGameTime;
            cancelBoundaryReferenceRefresh();
            return false;
        }
        return false;
    }

    private boolean boundaryReferenceRefreshMatches(
            int activeIndex,
            Identifier dimensionId,
            int brickX,
            int brickY,
            int brickZ
    ) {
        return boundaryRefreshActiveIndex == activeIndex
                && boundaryRefreshDimension != null
                && boundaryRefreshDimension.equals(dimensionId)
                && boundaryRefreshBrickX == brickX
                && boundaryRefreshBrickY == brickY
                && boundaryRefreshBrickZ == brickZ;
    }

    private void beginBoundaryReferenceRefresh(
            int activeIndex,
            Identifier dimensionId,
            BlockPos origin,
            int brickX,
            int brickY,
            int brickZ
    ) {
        boundaryRefreshActiveIndex = activeIndex;
        boundaryRefreshDimension = dimensionId;
        boundaryRefreshOrigin = origin;
        boundaryRefreshBrickX = brickX;
        boundaryRefreshBrickY = brickY;
        boundaryRefreshBrickZ = brickZ;
        boundaryRefreshCursor = 0;
        boundaryRefreshMaxCoarseSpeed = 0.0f;
        java.util.Arrays.fill(flowState, 0.0f);
        java.util.Arrays.fill(airTemperature, 0.0f);
        java.util.Arrays.fill(surfaceTemperature, 0.0f);
    }

    private BoundaryReferenceBuildResult buildBoundaryReferenceCells(Identifier dimensionId) {
        if (boundaryRefreshOrigin == null) {
            return BoundaryReferenceBuildResult.WAITING_FOR_COARSE;
        }
        long deadline = System.nanoTime() + BOUNDARY_REFERENCE_NANOS_PER_TICK;
        int built = 0;
        while (boundaryRefreshCursor < CELL_COUNT && built < BOUNDARY_REFERENCE_CELLS_PER_TICK) {
            int cell = boundaryRefreshCursor;
            int x = cell / (BRICK_SIZE * BRICK_SIZE);
            int rem = cell - x * BRICK_SIZE * BRICK_SIZE;
            int y = rem / BRICK_SIZE;
            int z = rem - y * BRICK_SIZE;
            int base = cell * FLOW_CHANNELS;
            if (obstacle[cell] == 0 && isBoundaryReferenceCell(x, y, z)) {
                Vec3d pos = new Vec3d(
                    boundaryRefreshOrigin.getX() + x + 0.5,
                    boundaryRefreshOrigin.getY() + y + 0.5,
                    boundaryRefreshOrigin.getZ() + z + 0.5
                );
                AeroWindSample coarse = visualizer.sampleServerCoarseFlow(dimensionId, pos);
                if (!coarse.hasFlow()) {
                    return BoundaryReferenceBuildResult.WAITING_FOR_COARSE;
                }
                flowState[base] = coarse.velocityX() / NATIVE_VELOCITY_SCALE;
                flowState[base + 1] = coarse.velocityY() / NATIVE_VELOCITY_SCALE;
                flowState[base + 2] = coarse.velocityZ() / NATIVE_VELOCITY_SCALE;
                flowState[base + 3] = coarse.pressure();
                float speed = (float) coarse.velocity().length();
                if (Float.isFinite(speed) && speed > boundaryRefreshMaxCoarseSpeed) {
                    boundaryRefreshMaxCoarseSpeed = speed;
                }
            } else {
                flowState[base] = 0.0f;
                flowState[base + 1] = 0.0f;
                flowState[base + 2] = 0.0f;
                flowState[base + 3] = 0.0f;
            }
            boundaryRefreshCursor++;
            built++;
            if ((built & 255) == 0 && System.nanoTime() >= deadline) {
                break;
            }
        }
        return boundaryRefreshCursor >= CELL_COUNT
                ? BoundaryReferenceBuildResult.COMPLETED
                : BoundaryReferenceBuildResult.IN_PROGRESS;
    }

    private boolean isBoundaryReferenceCell(int x, int y, int z) {
        int layers = Math.min(8, BRICK_SIZE);
        return x < layers
                || y < layers
                || z < layers
                || x >= BRICK_SIZE - layers
                || y >= BRICK_SIZE - layers
                || z >= BRICK_SIZE - layers;
    }

    private boolean isBoundaryReferenceCell(int x, int y, int z) {
        int layers = Math.min(8, BRICK_SIZE);
        return x < layers
            || y < layers
            || z < layers
            || x >= BRICK_SIZE - layers
            || y >= BRICK_SIZE - layers
            || z >= BRICK_SIZE - layers;
    }

    private void cancelBoundaryReferenceRefresh() {
        boundaryRefreshActiveIndex = -1;
        boundaryRefreshDimension = null;
        boundaryRefreshOrigin = null;
        boundaryRefreshBrickX = 0;
        boundaryRefreshBrickY = 0;
        boundaryRefreshBrickZ = 0;
        boundaryRefreshCursor = 0;
        boundaryRefreshMaxCoarseSpeed = 0.0f;
    }

    private boolean hasBoundaryRefreshPending() {
        for (int index = 0; index < activeBrickCount; index++) {
            if (activeBrickBoundaryRefreshPending[index]) {
                return true;
            }
        }
        return false;
    }

    private boolean uploadStaticBrick(ClientLevel world, BlockPos origin, int brickX, int brickY, int brickZ) {
        StaticBrickCacheKey key = activeDimension == null ? null : new StaticBrickCacheKey(activeDimension, brickX, brickY, brickZ);
        StaticBrickSnapshot cached = key == null ? null : staticBrickCache.get(key);
        if (cached != null) {
            cached.copyInto(obstacle, surfaceKind, openFaceMask, emitterPower, sourceFanDirection, sourceEmitterPower);
            java.util.Arrays.fill(faceSkyExposure, (byte) 0);
            java.util.Arrays.fill(faceDirectExposure, (byte) 0);
            return true;
        }
        populateStaticBrickArrays(world, origin);
        if (key != null) {
            staticBrickCache.put(key, StaticBrickSnapshot.copyFrom(obstacle, surfaceKind, openFaceMask, emitterPower, sourceFanDirection, sourceEmitterPower));
        }
        return true;
    }

    private void queueStaticPatchPositions(
            Identifier dimensionId,
            long patchWorldKey,
            long clientGameTime,
            BlockPos center,
            BlockState oldState,
            BlockState newState
    ) {
        if (pendingStaticPatchWorldKey != patchWorldKey
                || pendingStaticPatchDimension == null
                || !pendingStaticPatchDimension.equals(dimensionId)) {
            pendingSourcePatches.clear();
            pendingStaticPatchDimension = dimensionId;
            pendingStaticPatchWorldKey = patchWorldKey;
            pendingStaticPatchFirstChangeGameTime = Long.MIN_VALUE;
            pendingStaticPatchLastChangeGameTime = Long.MIN_VALUE;
            pendingStaticPatchSourceChanges = 0;
        }
        if (pendingStaticPatchFirstChangeGameTime == Long.MIN_VALUE) {
            pendingStaticPatchFirstChangeGameTime = clientGameTime;
        }
        pendingStaticPatchLastChangeGameTime = clientGameTime;
        pendingStaticPatchSourceChanges++;
        BlockPos key = center.immutable();
        PendingSourcePatch existing = pendingSourcePatches.get(key);
        pendingSourcePatches.put(
                key,
                new PendingSourcePatch(key, existing == null ? oldState : existing.oldState(), newState)
        );
        lastStaticPatchCount = pendingSourcePatches.size();
        lastFanPatchCellCount = 0;
        lastHeatPatchCellCount = 0;
    }

    private StaticPatchFlushResult flushPendingStaticPatches(ClientLevel world, Identifier dimensionId, long clientGameTime) {
        if (pendingSourcePatches.isEmpty()) {
            return StaticPatchFlushResult.NONE;
        }
        if (activeDimension == null
                || !activeDimension.equals(dimensionId)
                || pendingStaticPatchDimension == null
                || !pendingStaticPatchDimension.equals(dimensionId)
                || pendingStaticPatchWorldKey != worldKey
                || worldKey == 0L) {
            clearPendingStaticPatches();
            return StaticPatchFlushResult.NONE;
        }
        if (activeBrickCount <= 0 || !hasReadyActiveBrick()) {
            return StaticPatchFlushResult.NONE;
        }
        if (shouldDelayPendingStaticPatches(clientGameTime)) {
            return StaticPatchFlushResult.DELAYED;
        }
        NativeSimulationBridge.WorldDelta[] deltas = new NativeSimulationBridge.WorldDelta[pendingSourcePatches.size()];
        int count = 0;
        int fanSourcePatches = 0;
        int heatSourcePatches = 0;
        for (PendingSourcePatch patch : pendingSourcePatches.values()) {
            NativeSimulationBridge.WorldDelta delta = buildStaticSourcePatchDelta(world, patch);
            deltas[count++] = delta;
            int oldFanDirection = (delta.data1() >> 8) & 0xFF;
            int newFanDirection = (delta.data1() >> 16) & 0xFF;
            if (oldFanDirection != 0 || newFanDirection != 0) {
                fanSourcePatches++;
            }
            if (delta.value0() > 0.0f || delta.value1() > 0.0f) {
                heatSourcePatches++;
            }
            refreshLocalStaticCellIfActive(world, patch.pos());
        }
        clearPendingStaticPatches();
        if (count == 0) {
            return StaticPatchFlushResult.NONE;
        }
        NativeSimulationBridge.WorldDelta[] submitted = count == deltas.length
                ? deltas
                : java.util.Arrays.copyOf(deltas, count);
        lastStaticPatchCount = submitted.length;
        lastFanPatchCellCount = fanSourcePatches;
        lastHeatPatchCellCount = heatSourcePatches;
        lastStaticPatchSubmitClientGameTime = clientGameTime;
        cancelBoundaryReferenceRefresh();
        worker.submitWorldDeltas(worldKey, submitted);
        return StaticPatchFlushResult.SUBMITTED;
    }

    private boolean shouldDelayPendingStaticPatches(long clientGameTime) {
        if (STATIC_PATCH_DEBOUNCE_TICKS <= 0) {
            return false;
        }
        boolean bulkPatch = pendingStaticPatchSourceChanges >= STATIC_PATCH_BULK_CHANGE_THRESHOLD
                || pendingSourcePatches.size() >= STATIC_PATCH_BULK_CELL_THRESHOLD;
        if (!bulkPatch
                || pendingStaticPatchFirstChangeGameTime == Long.MIN_VALUE
                || pendingStaticPatchLastChangeGameTime == Long.MIN_VALUE) {
            return false;
        }
        long sinceFirstChange = clientGameTime - pendingStaticPatchFirstChangeGameTime;
        long sinceLastChange = clientGameTime - pendingStaticPatchLastChangeGameTime;
        if (sinceFirstChange < 0L || sinceLastChange < 0L) {
            return false;
        }
        return sinceLastChange < STATIC_PATCH_DEBOUNCE_TICKS
                && sinceFirstChange < STATIC_PATCH_MAX_DEBOUNCE_TICKS;
    }

    private void addStaticPatchPositionsForChange(
            java.util.LinkedHashSet<BlockPos> positions,
            BlockPos center,
            BlockState oldState,
            BlockState newState
    ) {
        addStaticPatchPosition(positions, center);
        for (Direction direction : Direction.values()) {
            addStaticPatchPosition(positions, center.relative(direction));
        }
        addFanOcclusionPatchPositions(positions, center);
        addHeatOcclusionPatchPositions(positions, center);
        addForcingSourcePatchPositions(positions, center, oldState);
        addForcingSourcePatchPositions(positions, center, newState);
    }

    private void addStaticPatchPosition(java.util.LinkedHashSet<BlockPos> positions, BlockPos pos) {
        positions.add(pos.immutable());
    }

    private void addFanOcclusionPatchPositions(java.util.LinkedHashSet<BlockPos> positions, BlockPos center) {
        for (Direction direction : Direction.values()) {
            for (int distance = 1; distance <= FAN_FORCE_LENGTH_CELLS; distance++) {
                addFanDiskPatchPositions(positions, offset(center, direction, distance), direction);
            }
        }
    }

    private void addHeatOcclusionPatchPositions(java.util.LinkedHashSet<BlockPos> positions, BlockPos center) {
        for (int distance = 1; distance <= HEAT_PLUME_HEIGHT_CELLS; distance++) {
            addStaticPatchPosition(positions, offset(center, Direction.UP, distance));
        }
    }

    private void addForcingSourcePatchPositions(
            java.util.LinkedHashSet<BlockPos> positions,
            BlockPos center,
            BlockState state
    ) {
        if (state == null) {
            return;
        }
        if (state.is(ModBlocks.FAN_BLOCK)) {
            Direction direction = state.getOptionalValue(FanBlock.FACING).orElse(Direction.NORTH);
            addFanFootprintPatchPositions(positions, center, direction);
        }
        if (sampleEmitterThermalPowerWatts(state) > 0.0f) {
            for (int distance = 0; distance <= HEAT_PLUME_HEIGHT_CELLS; distance++) {
                addStaticPatchPosition(positions, offset(center, Direction.UP, distance));
            }
        }
    }

    private BlockPos offset(BlockPos pos, Direction direction, int distance) {
        return new BlockPos(
                pos.getX() + direction.getStepX() * distance,
                pos.getY() + direction.getStepY() * distance,
                pos.getZ() + direction.getStepZ() * distance
        );
    }

    private void addFanFootprintPatchPositions(
            java.util.LinkedHashSet<BlockPos> positions,
            BlockPos fanPos,
            Direction direction
    ) {
        for (int distance = 1; distance <= FAN_FORCE_LENGTH_CELLS; distance++) {
            addFanDiskPatchPositions(positions, offset(fanPos, direction, distance), direction);
        }
    }

    private void addFanDiskPatchPositions(
            java.util.LinkedHashSet<BlockPos> positions,
            BlockPos center,
            Direction direction
    ) {
        Direction.Axis axis = direction.getAxis();
        for (int a = -FAN_FORCE_RADIUS_CELLS; a <= FAN_FORCE_RADIUS_CELLS; a++) {
            for (int b = -FAN_FORCE_RADIUS_CELLS; b <= FAN_FORCE_RADIUS_CELLS; b++) {
                addStaticPatchPosition(positions, offsetPerpendicular(center, axis, a, b));
            }
        }
    }

    private BlockPos offsetPerpendicular(BlockPos pos, Direction.Axis axis, int a, int b) {
        return switch (axis) {
            case X -> new BlockPos(pos.getX(), pos.getY() + a, pos.getZ() + b);
            case Y -> new BlockPos(pos.getX() + a, pos.getY(), pos.getZ() + b);
            case Z -> new BlockPos(pos.getX() + a, pos.getY() + b, pos.getZ());
        };
    }

    private boolean blockPatchTouchesActiveBrick(BlockPos center) {
        if (blockInActiveBrick(center)) {
            return true;
        }
        for (Direction direction : Direction.values()) {
            if (blockInActiveBrick(center.relative(direction))) {
                return true;
            }
            for (int distance = 1; distance <= FAN_FORCE_LENGTH_CELLS; distance++) {
                BlockPos fanCenter = offset(center, direction, distance);
                Direction.Axis axis = direction.getAxis();
                for (int a = -FAN_FORCE_RADIUS_CELLS; a <= FAN_FORCE_RADIUS_CELLS; a++) {
                    for (int b = -FAN_FORCE_RADIUS_CELLS; b <= FAN_FORCE_RADIUS_CELLS; b++) {
                        if (blockInActiveBrick(offsetPerpendicular(fanCenter, axis, a, b))) {
                            return true;
                        }
                    }
                }
            }
        }
        for (int distance = 2; distance <= HEAT_PLUME_HEIGHT_CELLS; distance++) {
            if (blockInActiveBrick(offset(center, Direction.UP, distance))) {
                return true;
            }
        }
        return false;
    }

    private boolean blockInActiveBrick(BlockPos pos) {
        int brickX = Math.floorDiv(pos.getX(), BRICK_SIZE);
        int brickY = Math.floorDiv(pos.getY(), BRICK_SIZE);
        int brickZ = Math.floorDiv(pos.getZ(), BRICK_SIZE);
        for (int index = 0; index < activeBrickCount; index++) {
            if (activeBrickX[index] == brickX && activeBrickY[index] == brickY && activeBrickZ[index] == brickZ) {
                return true;
            }
        }
        return false;
    }

    private NativeSimulationBridge.WorldDelta buildStaticCellPatchDelta(ClientLevel world, BlockPos pos) {
        StaticCellSample sample = sampleStaticCell(world, pos);
        int packedState = (sample.solid() ? 1 : 0)
                | ((Byte.toUnsignedInt(sample.surfaceKind()) & 0xFF) << 8);
        return new NativeSimulationBridge.WorldDelta(
                NativeSimulationBridge.WORLD_DELTA_BRICK_STATIC_CELL_PATCH,
                pos.getX(),
                pos.getY(),
                pos.getZ(),
                (int) worldKey,
                packedState,
                Short.toUnsignedInt(sample.openFaceMask()),
                0,
                sample.emitterPowerWatts(),
                0.0f,
                0.0f,
                0.0f
        );
    }

    private NativeSimulationBridge.WorldDelta buildStaticSourcePatchDelta(ClientLevel world, PendingSourcePatch patch) {
        BlockPos pos = patch.pos();
        boolean oldSolid = sourceSolidForState(world, pos, patch.oldState());
        boolean newSolid = sourceSolidForState(world, pos, patch.newState());
        int oldFanDirection = sourceFanDirectionCodeForState(patch.oldState());
        int newFanDirection = sourceFanDirectionCodeForState(patch.newState());
        float oldEmitterPower = sourceEmitterPowerForState(patch.oldState());
        float newEmitterPower = sourceEmitterPowerForState(patch.newState());
        int packedState = (oldSolid ? 1 : 0)
                | (newSolid ? 2 : 0)
                | ((oldFanDirection & 0xFF) << 8)
                | ((newFanDirection & 0xFF) << 16);
        return new NativeSimulationBridge.WorldDelta(
                NativeSimulationBridge.WORLD_DELTA_BRICK_STATIC_SOURCE_PATCH,
                pos.getX(),
                pos.getY(),
                pos.getZ(),
                (int) worldKey,
                packedState,
                0,
                0,
                oldEmitterPower,
                newEmitterPower,
                0.0f,
                0.0f
        );
    }

    private void refreshLocalStaticCellIfActive(ClientLevel world, BlockPos pos) {
        for (int index = 0; index < activeBrickCount; index++) {
            int brickX = Math.floorDiv(pos.getX(), BRICK_SIZE);
            int brickY = Math.floorDiv(pos.getY(), BRICK_SIZE);
            int brickZ = Math.floorDiv(pos.getZ(), BRICK_SIZE);
            if (activeBrickX[index] != brickX || activeBrickY[index] != brickY || activeBrickZ[index] != brickZ) {
                continue;
            }
            BlockPos origin = brickOrigin(brickX, brickY, brickZ);
            int localX = pos.getX() - origin.getX();
            int localY = pos.getY() - origin.getY();
            int localZ = pos.getZ() - origin.getZ();
            if (localX >= 0 && localY >= 0 && localZ >= 0
                    && localX < BRICK_SIZE && localY < BRICK_SIZE && localZ < BRICK_SIZE) {
                populateStaticCell(world, origin, localX, localY, localZ);
            }
        }
    }

    private void invalidateStaticCacheForPatchFootprint(
            Identifier dimensionId,
            BlockPos center,
            BlockState oldState,
            BlockState newState
    ) {
        java.util.LinkedHashSet<BlockPos> positions = new java.util.LinkedHashSet<>();
        addStaticPatchPositionsForChange(positions, center, oldState, newState);
        for (BlockPos pos : positions) {
            invalidateStaticCacheForBlock(dimensionId, pos);
        }
    }

    private void invalidateStaticCacheForBlock(Identifier dimensionId, BlockPos pos) {
        staticBrickCache.remove(new StaticBrickCacheKey(
                dimensionId,
                Math.floorDiv(pos.getX(), BRICK_SIZE),
                Math.floorDiv(pos.getY(), BRICK_SIZE),
                Math.floorDiv(pos.getZ(), BRICK_SIZE)
        ));
    }

    private void markActiveBrickStaticRefreshPending(BlockPos pos) {
        int brickX = Math.floorDiv(pos.getX(), BRICK_SIZE);
        int brickY = Math.floorDiv(pos.getY(), BRICK_SIZE);
        int brickZ = Math.floorDiv(pos.getZ(), BRICK_SIZE);
        for (int index = 0; index < activeBrickCount; index++) {
            if (activeBrickX[index] == brickX && activeBrickY[index] == brickY && activeBrickZ[index] == brickZ) {
                activeBrickRefreshPending[index] = true;
                activeBrickBoundaryRefreshPending[index] = true;
                ticksSinceStaticRefresh = 0;
            }
        }
    }

    private void markAllActiveBricksStaticRefreshPending() {
        for (int index = 0; index < activeBrickCount; index++) {
            activeBrickRefreshPending[index] = true;
            activeBrickBoundaryRefreshPending[index] = true;
        }
        staticBrickCache.clear();
        ticksSinceStaticRefresh = 0;
    }

    private void populateStaticBrickArrays(ClientLevel world, BlockPos origin) {
        java.util.Arrays.fill(obstacle, (byte) 0);
        java.util.Arrays.fill(surfaceKind, (byte) 0);
        java.util.Arrays.fill(openFaceMask, (short) 0);
        java.util.Arrays.fill(emitterPower, 0.0f);
        java.util.Arrays.fill(sourceFanDirection, (byte) 0);
        java.util.Arrays.fill(sourceEmitterPower, 0.0f);
        java.util.Arrays.fill(faceSkyExposure, (byte) 0);
        java.util.Arrays.fill(faceDirectExposure, (byte) 0);

        for (int x = 0; x < BRICK_SIZE; x++) {
            for (int y = 0; y < BRICK_SIZE; y++) {
                for (int z = 0; z < BRICK_SIZE; z++) {
                    populateStaticCell(world, origin, x, y, z);
                }
            }
        }
    }

    private void populateStaticCell(ClientLevel world, BlockPos origin, int x, int y, int z) {
        staticCursor.set(origin.getX() + x, origin.getY() + y, origin.getZ() + z);
        int cell = cellIndex(x, y, z);
        StaticCellSample sample = sampleStaticSourceCell(world, staticCursor);
        obstacle[cell] = sample.solid() ? (byte) 1 : (byte) 0;
        surfaceKind[cell] = sample.surfaceKind();
        openFaceMask[cell] = sample.openFaceMask();
        emitterPower[cell] = sample.emitterPowerWatts();
        sourceFanDirection[cell] = sample.sourceFanDirection();
        sourceEmitterPower[cell] = sample.sourceEmitterPowerWatts();
    }

    private StaticCellSample sampleStaticSourceCell(ClientWorld world, BlockPos pos) {
        BlockState state = world.getBlockState(pos);
        boolean solid = isSolidObstacle(world, pos, state);
        short mask = 0;
        if (!solid) {
            for (Direction direction : Direction.values()) {
                staticNeighbor.set(
                    pos.getX() + direction.getOffsetX(),
                    pos.getY() + direction.getOffsetY(),
                    pos.getZ() + direction.getOffsetZ()
                );
                if (!isSolidObstacle(world, staticNeighbor, world.getBlockState(staticNeighbor))) {
                    mask = (short) (mask | (1 << direction.ordinal()));
                }
            }
        }
        return new StaticCellSample(
            solid,
            (byte) 0,
            mask,
            0.0f,
            (byte) sourceFanDirectionCodeForState(state),
            sourceEmitterPowerForState(state)
        );
    }

    private StaticCellSample sampleStaticCell(ClientWorld world, BlockPos pos) {
        BlockState state = world.getBlockState(pos);
        boolean solid = isSolidObstacle(world, pos, state);
        short mask = 0;
        if (!solid) {
            for (Direction direction : Direction.values()) {
                staticNeighbor.set(
                        pos.getX() + direction.getStepX(),
                        pos.getY() + direction.getStepY(),
                        pos.getZ() + direction.getStepZ()
                );
                if (!isSolidObstacle(world, staticNeighbor, world.getBlockState(staticNeighbor))) {
                    mask = (short) (mask | (1 << direction.ordinal()));
                }
            }
        }
        return new StaticCellSample(
                solid,
                (byte) 0,
                mask,
                0.0f,
                (byte) sourceFanDirectionCodeForState(state),
                sourceEmitterPowerForState(state)
        );
    }

    private StaticCellSample sampleStaticCell(ClientLevel world, BlockPos pos) {
        BlockState state = world.getBlockState(pos);
        boolean solid = isSolidObstacle(world, pos, state);
        short mask = 0;
        if (!solid) {
            for (Direction direction : Direction.values()) {
                staticNeighbor.set(
                        pos.getX() + direction.getStepX(),
                        pos.getY() + direction.getStepY(),
                        pos.getZ() + direction.getStepZ()
                );
                if (!isSolidObstacle(world, staticNeighbor, world.getBlockState(staticNeighbor))) {
                    mask = (short) (mask | (1 << direction.ordinal()));
                }
            }
        }
        byte kind = solid ? (byte) 0 : fanSurfaceKindForCell(world, pos);
        float emitter = solid ? 0.0f : emitterPowerForCell(world, pos, state);
        return new StaticCellSample(
                solid,
                kind,
                mask,
                emitter,
                (byte) sourceFanDirectionCodeForState(state),
                sourceEmitterPowerForState(state)
        );
    }

    private byte fanSurfaceKindForCell(ClientLevel world, BlockPos pos) {
        for (Direction direction : Direction.values()) {
            Direction.Axis axis = direction.getAxis();
            for (int distance = 1; distance <= FAN_FORCE_LENGTH_CELLS; distance++) {
                BlockPos axialOrigin = offset(pos, direction.getOpposite(), distance);
                for (int a = -FAN_FORCE_RADIUS_CELLS; a <= FAN_FORCE_RADIUS_CELLS; a++) {
                    for (int b = -FAN_FORCE_RADIUS_CELLS; b <= FAN_FORCE_RADIUS_CELLS; b++) {
                        BlockPos fanPos = offsetPerpendicular(axialOrigin, axis, -a, -b);
                        BlockState fanState = world.getBlockState(fanPos);
                        if (!fanState.is(ModBlocks.FAN_BLOCK)
                                || fanState.getOptionalValue(FanBlock.FACING).orElse(Direction.NORTH) != direction) {
                            continue;
                        }
                        if (!fanPathClear(world, fanPos, direction, distance, a, b)) {
                            continue;
                        }
                        return fanSurfaceKind(direction);
                    }
                }
            }
        }
        return 0;
    }

    private boolean fanPathClear(ClientLevel world, BlockPos fanPos, Direction direction, int distance, int a, int b) {
        BlockPos laneStart = offsetPerpendicular(fanPos, direction.getAxis(), a, b);
        for (int step = 1; step < distance; step++) {
            staticNeighbor.set(
                    laneStart.getX() + direction.getStepX() * step,
                    laneStart.getY() + direction.getStepY() * step,
                    laneStart.getZ() + direction.getStepZ() * step
            );
            if (isSolidObstacle(world, staticNeighbor, world.getBlockState(staticNeighbor))) {
                return false;
            }
        }
        return true;
    }

    private byte fanSurfaceKind(Direction direction) {
        return switch (direction) {
            case WEST -> SURFACE_KIND_FAN_X_NEG;
            case EAST -> SURFACE_KIND_FAN_X_POS;
            case DOWN -> SURFACE_KIND_FAN_Y_NEG;
            case UP -> SURFACE_KIND_FAN_Y_POS;
            case NORTH -> SURFACE_KIND_FAN_Z_NEG;
            case SOUTH -> SURFACE_KIND_FAN_Z_POS;
        };
    }

    private int sourceFanDirectionCodeForState(BlockState state) {
        if (state == null || !state.is(ModBlocks.FAN_BLOCK)) {
            return 0;
        }
        Direction direction = state.getOptionalValue(FanBlock.FACING).orElse(Direction.NORTH);
        return switch (direction) {
            case WEST -> 1;
            case EAST -> 2;
            case DOWN -> 3;
            case UP -> 4;
            case NORTH -> 5;
            case SOUTH -> 6;
        };
    }

    private float sourceEmitterPowerForState(BlockState state) {
        return state == null ? 0.0f : sampleEmitterThermalPowerWatts(state);
    }

    private boolean sourceSolidForState(ClientLevel world, BlockPos pos, BlockState state) {
        return state != null && isSolidObstacle(world, pos, state);
    }

    private boolean isFanSurfaceKind(int surfaceKind) {
        return surfaceKind >= Byte.toUnsignedInt(SURFACE_KIND_FAN_X_NEG)
                && surfaceKind <= Byte.toUnsignedInt(SURFACE_KIND_FAN_Z_POS);
    }

    private float emitterPowerForCell(ClientLevel world, BlockPos pos, BlockState state) {
        float directPower = sampleEmitterThermalPowerWatts(state);
        if (directPower > 0.0f) {
            return directPower;
        }
        float coupledPower = 0.0f;
        for (int distance = 1; distance <= HEAT_PLUME_HEIGHT_CELLS; distance++) {
            int sourceY = pos.getY() - distance;
            staticNeighbor.set(pos.getX(), sourceY, pos.getZ());
            float belowPower = sampleEmitterThermalPowerWatts(world.getBlockState(staticNeighbor));
            if (belowPower <= 0.0f || !heatPathClear(world, pos.getX(), sourceY, pos.getZ(), pos.getY())) {
                continue;
            }
            float falloff = HEAT_COUPLING_TO_ADJACENT_AIR / (distance * distance);
            coupledPower += belowPower * falloff;
        }
        return coupledPower;
    }

    private boolean heatPathClear(ClientLevel world, int sourceX, int sourceY, int sourceZ, int targetY) {
        for (int y = sourceY + 1; y < targetY; y++) {
            staticNeighbor.set(sourceX, y, sourceZ);
            if (isSolidObstacle(world, staticNeighbor, world.getBlockState(staticNeighbor))) {
                return false;
            }
        }
        return true;
    }

    private float sampleEmitterThermalPowerWatts(BlockState state) {
        float powerWatts = 0.0f;
        if (state.is(Blocks.LAVA) || state.is(Blocks.LAVA_CAULDRON)) {
            powerWatts += THERMAL_EMITTER_POWER_LAVA_W;
        }
        if (state.is(Blocks.MAGMA_BLOCK)) {
            powerWatts += THERMAL_EMITTER_POWER_MAGMA_W;
        }
        if (state.is(Blocks.CAMPFIRE)) {
            powerWatts += state.getOptionalValue(BlockStateProperties.LIT).orElse(false) ? THERMAL_EMITTER_POWER_CAMPFIRE_W : 0.0f;
        }
        if (state.is(Blocks.SOUL_CAMPFIRE)) {
            powerWatts += state.getOptionalValue(BlockStateProperties.LIT).orElse(false) ? THERMAL_EMITTER_POWER_SOUL_CAMPFIRE_W : 0.0f;
        }
        if (state.is(Blocks.FIRE)) {
            powerWatts += THERMAL_EMITTER_POWER_FIRE_W;
        }
        if (state.is(Blocks.SOUL_FIRE)) {
            powerWatts += THERMAL_EMITTER_POWER_SOUL_FIRE_W;
        }
        if (state.is(Blocks.TORCH) || state.is(Blocks.WALL_TORCH)) {
            powerWatts += THERMAL_EMITTER_POWER_TORCH_W;
        }
        if (state.is(Blocks.SOUL_TORCH) || state.is(Blocks.SOUL_WALL_TORCH)) {
            powerWatts += THERMAL_EMITTER_POWER_SOUL_TORCH_W;
        }
        if (state.is(Blocks.LANTERN)) {
            powerWatts += THERMAL_EMITTER_POWER_LANTERN_W;
        }
        if (state.is(Blocks.SOUL_LANTERN)) {
            powerWatts += THERMAL_EMITTER_POWER_SOUL_LANTERN_W;
        }
        return Math.max(powerWatts, 0.0f);
    }

    private CoarseSeedStats fillFlowStateFromCoarse(Identifier dimensionId, BlockPos origin) {
        java.util.Arrays.fill(flowState, 0.0f);
        java.util.Arrays.fill(airTemperature, 0.0f);
        java.util.Arrays.fill(surfaceTemperature, 0.0f);
        float maxCoarseSpeed = 0.0f;
        for (int x = 0; x < BRICK_SIZE; x++) {
            for (int y = 0; y < BRICK_SIZE; y++) {
                for (int z = 0; z < BRICK_SIZE; z++) {
                    int cell = cellIndex(x, y, z);
                    if (obstacle[cell] != 0) {
                        continue;
                    }
                    Vec3 pos = new Vec3(origin.getX() + x + 0.5, origin.getY() + y + 0.5, origin.getZ() + z + 0.5);
                    AeroWindSample coarse = visualizer.sampleServerCoarseFlow(dimensionId, pos);
                    if (!coarse.hasFlow()) {
                        return null;
                    }
                    int base = cell * FLOW_CHANNELS;
                    flowState[base] = coarse.velocityX() / NATIVE_VELOCITY_SCALE;
                    flowState[base + 1] = coarse.velocityY() / NATIVE_VELOCITY_SCALE;
                    flowState[base + 2] = coarse.velocityZ() / NATIVE_VELOCITY_SCALE;
                    flowState[base + 3] = coarse.pressure();
                    float speed = (float) coarse.velocity().length();
                    if (Float.isFinite(speed) && speed > maxCoarseSpeed) {
                        maxCoarseSpeed = speed;
                    }
                }
            }
        }
        return new CoarseSeedStats(maxCoarseSpeed);
    }

    private boolean isSolidObstacle(ClientLevel world, BlockPos pos, BlockState state) {
        if (state.isAir() || state.is(ModBlocks.DUCT_BLOCK)) {
            return false;
        }
        return !state.getCollisionShape(world, pos).isEmpty();
    }

    private static short quantizeSignedToShort(float value, float range) {
        if (!(range > 0.0f) || !Float.isFinite(value)) {
            return 0;
        }
        float normalized = Mth.clamp(value / range, -1.0f, 1.0f);
        return (short) Math.round(normalized * 32767.0f);
    }

    private static float maxFlowSpeedMetersPerSecond(float[] state) {
        float maxSpeed = 0.0f;
        for (int base = 0; base + 2 < state.length; base += FLOW_CHANNELS) {
            float vx = state[base] * NATIVE_VELOCITY_SCALE;
            float vy = state[base + 1] * NATIVE_VELOCITY_SCALE;
            float vz = state[base + 2] * NATIVE_VELOCITY_SCALE;
            float speed = (float) Math.sqrt(vx * vx + vy * vy + vz * vz);
            if (Float.isFinite(speed) && speed > maxSpeed) {
                maxSpeed = speed;
            }
        }
        return maxSpeed;
    }

    private BlockPos brickOrigin(BlockPos pos) {
        return new BlockPos(
                Math.floorDiv(pos.getX(), BRICK_SIZE) * BRICK_SIZE,
                Math.floorDiv(pos.getY(), BRICK_SIZE) * BRICK_SIZE,
                Math.floorDiv(pos.getZ(), BRICK_SIZE) * BRICK_SIZE
        );
    }

    private BlockPos brickOrigin(int brickX, int brickY, int brickZ) {
        return new BlockPos(
                brickX * BRICK_SIZE,
                brickY * BRICK_SIZE,
                brickZ * BRICK_SIZE
        );
    }

    private int cellIndex(int x, int y, int z) {
        return (x * BRICK_SIZE + y) * BRICK_SIZE + z;
    }

    private long worldKey(Identifier dimensionId) {
        long value = dimensionId.hashCode();
        return value == 0L ? 1L : value;
    }

    private record CoarseSeedStats(float maxCoarseSpeedMetersPerSecond) {
    }

    private record StaticBrickCacheKey(Identifier dimensionId, int brickX, int brickY, int brickZ) {
    }

    private record StaticCellSample(
            boolean solid,
            byte surfaceKind,
            short openFaceMask,
            float emitterPowerWatts,
            byte sourceFanDirection,
            float sourceEmitterPowerWatts
    ) {
    }

    private record StaticBrickSnapshot(
            byte[] obstacle,
            byte[] surfaceKind,
            short[] openFaceMask,
            float[] emitterPower,
            byte[] sourceFanDirection,
            float[] sourceEmitterPower
    ) {
        static StaticBrickSnapshot copyFrom(
                byte[] obstacle,
                byte[] surfaceKind,
                short[] openFaceMask,
                float[] emitterPower,
                byte[] sourceFanDirection,
                float[] sourceEmitterPower
        ) {
            return new StaticBrickSnapshot(
                    java.util.Arrays.copyOf(obstacle, obstacle.length),
                    java.util.Arrays.copyOf(surfaceKind, surfaceKind.length),
                    java.util.Arrays.copyOf(openFaceMask, openFaceMask.length),
                    java.util.Arrays.copyOf(emitterPower, emitterPower.length),
                    java.util.Arrays.copyOf(sourceFanDirection, sourceFanDirection.length),
                    java.util.Arrays.copyOf(sourceEmitterPower, sourceEmitterPower.length)
            );
        }

        void copyInto(
                byte[] outObstacle,
                byte[] outSurfaceKind,
                short[] outOpenFaceMask,
                float[] outEmitterPower,
                byte[] outSourceFanDirection,
                float[] outSourceEmitterPower
        ) {
            System.arraycopy(obstacle, 0, outObstacle, 0, Math.min(obstacle.length, outObstacle.length));
            System.arraycopy(surfaceKind, 0, outSurfaceKind, 0, Math.min(surfaceKind.length, outSurfaceKind.length));
            System.arraycopy(openFaceMask, 0, outOpenFaceMask, 0, Math.min(openFaceMask.length, outOpenFaceMask.length));
            System.arraycopy(emitterPower, 0, outEmitterPower, 0, Math.min(emitterPower.length, outEmitterPower.length));
            System.arraycopy(
                    sourceFanDirection,
                    0,
                    outSourceFanDirection,
                    0,
                    Math.min(sourceFanDirection.length, outSourceFanDirection.length)
            );
            System.arraycopy(
                    sourceEmitterPower,
                    0,
                    outSourceEmitterPower,
                    0,
                    Math.min(sourceEmitterPower.length, outSourceEmitterPower.length)
            );
        }
    }

    private record PendingSourcePatch(BlockPos pos, BlockState oldState, BlockState newState) {
    }

    private enum BrickPreparationResult {
        IN_PROGRESS,
        COMPLETED,
        FAILED
    }

    private enum BoundaryReferenceBuildResult {
        IN_PROGRESS,
        COMPLETED,
        WAITING_FOR_COARSE
    }

    private interface WorkerCommand {
    }

    private record ActiveHintsCommand(long worldKey, int[] activeHintCoords) implements WorkerCommand {
    }

    private record WorldDeltasCommand(long worldKey,
                                      NativeSimulationBridge.WorldDelta[] deltas) implements WorkerCommand {
    }

    private record BrickSeedCommand(
            long worldKey,
            int brickX,
            int brickY,
            int brickZ,
            byte[] obstacle,
            byte[] surfaceKind,
            short[] openFaceMask,
            float[] emitterPower,
            byte[] sourceFanDirection,
            float[] sourceEmitterPower,
            byte[] faceSkyExposure,
            byte[] faceDirectExposure,
            float[] flowState,
            float[] airTemperature,
            float[] surfaceTemperature
    ) implements WorkerCommand {
    }

    private record BoundaryReferenceCommand(
            long worldKey,
            int brickX,
            int brickY,
            int brickZ,
            float[] flowState,
            float[] airTemperature,
            float[] surfaceTemperature,
            float maxCoarseSpeedMetersPerSecond
    ) implements WorkerCommand {
    }

    private record StepCommand(long worldKey, PublishTarget[] publishTargets, int stepCount) implements WorkerCommand {
    }

    private record ResetCommand() implements WorkerCommand {
    }

    private record CloseCommand() implements WorkerCommand {
    }

    private record PublishTarget(Identifier dimensionId, BlockPos origin, int brickX, int brickY, int brickZ) {
    }

    private record LocalAtlasSnapshot(Identifier dimensionId, BlockPos origin, int sampleStride, short[] packedFlow) {
    }

    private record WorkerDeltaKey(int type, int x, int y, int z, int data0) {
        static WorkerDeltaKey of(NativeSimulationBridge.WorldDelta delta) {
            return new WorkerDeltaKey(delta.type(), delta.x(), delta.y(), delta.z(), delta.data0());
        }
    }

    private final class ClientL2Worker {
        private final NativeSimulationBridge bridge = new NativeSimulationBridge();
        private final BlockingQueue<WorkerCommand> commands = new ArrayBlockingQueue<>(WORKER_QUEUE_CAPACITY);
        private final ConcurrentLinkedQueue<LocalAtlasSnapshot> atlases = new ConcurrentLinkedQueue<>();
        private final float[] workerFlowState = new float[CELL_COUNT * FLOW_CHANNELS];
        private final float[] workerAirTemperature = new float[CELL_COUNT];
        private final float[] workerSurfaceTemperature = new float[CELL_COUNT];
        private volatile boolean running;
        private volatile String lastError = "-";
        private volatile String lastRuntimeInfo = "-";
        private volatile NativeSimulationBridge.BrickWorldRuntimeStatus lastNativeStatus;
        private volatile long processedCommands;
        private volatile long droppedCommands;
        private volatile long publishedAtlases;
        private volatile long lastStepNanos;
        private volatile long lastPublishNanos;
        private long serviceKey;
        private Thread thread;

        boolean isNativeLoaded() {
            return bridge.isLoaded();
        }

        String loadError() {
            return bridge.getLoadError();
        }

        void submitActiveHints(long worldKey, int[] activeHintCoords) {
            offer(new ActiveHintsCommand(worldKey, java.util.Arrays.copyOf(activeHintCoords, activeHintCoords.length)));
        }

        void submitWorldDeltas(long worldKey, NativeSimulationBridge.WorldDelta[] deltas) {
            offer(new WorldDeltasCommand(worldKey, java.util.Arrays.copyOf(deltas, deltas.length)));
        }

        void submitBrickSeed(BrickSeedCommand command) {
            offer(command);
        }

        void submitBoundaryReference(BoundaryReferenceCommand command) {
            offer(command);
        }

        void requestStep(long worldKey, PublishTarget[] publishTargets, int stepCount) {
            offer(new StepCommand(worldKey, publishTargets, stepCount));
        }

        LocalAtlasSnapshot pollAtlas() {
            return atlases.poll();
        }

        int queueSize() {
            return commands.size();
        }

        void reset() {
            commands.clear();
            atlases.clear();
            if (running) {
                offer(new ResetCommand());
            }
        }

        void close() {
            commands.clear();
            atlases.clear();
            if (!running) {
                releaseService();
                return;
            }
            offer(new CloseCommand());
        }

        String status() {
            return "running=" + running
                    + ",queue=" + commands.size()
                    + ",atlases=" + atlases.size()
                    + ",processed=" + processedCommands
                    + ",dropped=" + droppedCommands
                    + ",published=" + publishedAtlases
                    + ",lastStepMs=" + formatMillis(lastStepNanos)
                    + ",lastPublishMs=" + formatMillis(lastPublishNanos)
                    + ",native=" + formatNativeStatus(lastNativeStatus)
                    + ",runtime=" + lastRuntimeInfo
                    + ",error=" + lastError;
        }

        private void offer(WorkerCommand command) {
            if (!bridge.isLoaded()) {
                lastError = bridge.getLoadError();
                return;
            }
            startIfNeeded();
            if (command instanceof StepCommand) {
                droppedCommands += removeQueuedStepCommands();
            } else if (command instanceof WorldDeltasCommand worldDeltas) {
                command = coalesceQueuedWorldDeltas(worldDeltas);
            } else if (isPriorityCommand(command)) {
                droppedCommands += removeQueuedStepCommands();
            }
            if (!commands.offer(command)) {
                if (command instanceof StepCommand) {
                    droppedCommands++;
                    return;
                }
                if (!removeOneQueuedStepCommand()) {
                    commands.poll();
                }
                droppedCommands++;
                commands.offer(command);
            }
        }

        private boolean isPriorityCommand(WorkerCommand command) {
            return command instanceof ActiveHintsCommand
                    || command instanceof WorldDeltasCommand
                    || command instanceof BrickSeedCommand
                    || command instanceof BoundaryReferenceCommand
                    || command instanceof ResetCommand
                    || command instanceof CloseCommand;
        }

        private WorldDeltasCommand coalesceQueuedWorldDeltas(WorldDeltasCommand incoming) {
            java.util.ArrayList<NativeSimulationBridge.WorldDelta> merged = new java.util.ArrayList<>(incoming.deltas().length);
            for (WorkerCommand queued : commands.toArray(new WorkerCommand[0])) {
                if (queued instanceof WorldDeltasCommand existing
                        && existing.worldKey() == incoming.worldKey()
                        && commands.remove(queued)) {
                    java.util.Collections.addAll(merged, existing.deltas());
                }
            }
            java.util.Collections.addAll(merged, incoming.deltas());
            return new WorldDeltasCommand(incoming.worldKey(), coalesceWorldDeltas(merged));
        }

        private NativeSimulationBridge.WorldDelta[] coalesceWorldDeltas(
                java.util.List<NativeSimulationBridge.WorldDelta> deltas
        ) {
            java.util.LinkedHashMap<WorkerDeltaKey, NativeSimulationBridge.WorldDelta> byCell = new java.util.LinkedHashMap<>();
            for (NativeSimulationBridge.WorldDelta delta : deltas) {
                WorkerDeltaKey key = WorkerDeltaKey.of(delta);
                NativeSimulationBridge.WorldDelta existing = byCell.get(key);
                if (existing == null) {
                    byCell.put(key, delta);
                    continue;
                }
                if (delta.type() == NativeSimulationBridge.WORLD_DELTA_BRICK_STATIC_SOURCE_PATCH) {
                    int packedState = (existing.data1() & 0x000001)
                            | (delta.data1() & 0x000002)
                            | (existing.data1() & 0x00FF00)
                            | (delta.data1() & 0xFF0000);
                    byCell.put(key, new NativeSimulationBridge.WorldDelta(
                            delta.type(),
                            delta.x(),
                            delta.y(),
                            delta.z(),
                            delta.data0(),
                            packedState,
                            delta.data2(),
                            delta.data3(),
                            existing.value0(),
                            delta.value1(),
                            delta.value2(),
                            delta.value3()
                    ));
                } else {
                    byCell.put(key, delta);
                }
            }
            return byCell.values().toArray(new NativeSimulationBridge.WorldDelta[0]);
        }

        private int removeQueuedStepCommands() {
            int removed = 0;
            for (WorkerCommand queued : commands.toArray(new WorkerCommand[0])) {
                if (queued instanceof StepCommand && commands.remove(queued)) {
                    removed++;
                }
            }
            return removed;
        }

        private boolean removeOneQueuedStepCommand() {
            for (WorkerCommand queued : commands.toArray(new WorkerCommand[0])) {
                if (queued instanceof StepCommand && commands.remove(queued)) {
                    return true;
                }
            }
            return false;
        }

        private void startIfNeeded() {
            if (running) {
                return;
            }
            running = true;
            thread = new Thread(this::runLoop, "a4mc-client-l2-worker");
            thread.setDaemon(true);
            thread.start();
        }

        private void runLoop() {
            try {
                while (running) {
                    WorkerCommand command = commands.take();
                    processedCommands++;
                    if (command instanceof CloseCommand) {
                        running = false;
                        break;
                    }
                    if (command instanceof ResetCommand) {
                        handleReset();
                    } else if (command instanceof ActiveHintsCommand activeHints) {
                        handleActiveHints(activeHints);
                    } else if (command instanceof WorldDeltasCommand worldDeltas) {
                        handleWorldDeltas(worldDeltas);
                    } else if (command instanceof BrickSeedCommand brickSeed) {
                        handleBrickSeed(brickSeed);
                    } else if (command instanceof BoundaryReferenceCommand boundaryReference) {
                        handleBoundaryReference(boundaryReference);
                    } else if (command instanceof StepCommand step) {
                        handleStep(step);
                    }
                }
            } catch (InterruptedException interrupted) {
                Thread.currentThread().interrupt();
            } catch (Throwable t) {
                lastError = t.getClass().getSimpleName() + ": " + t.getMessage();
                LOGGER.warn("Client L2 worker stopped after unexpected error", t);
            } finally {
                releaseService();
                running = false;
            }
        }

        private void handleReset() {
            releaseService();
            atlases.clear();
            lastNativeStatus = null;
            lastRuntimeInfo = "-";
            lastError = "-";
        }

        private boolean ensureRuntime(long worldKey) {
            if (serviceKey == 0L) {
                serviceKey = bridge.createService();
            }
            if (serviceKey == 0L) {
                lastError = "failed to create native service";
                return false;
            }
            if (!bridge.ensureBrickWorldRuntime(serviceKey, worldKey, BRICK_SIZE, DX_METERS, DT_SECONDS)) {
                lastError = "ensureBrickWorldRuntime failed: " + bridge.lastError();
                return false;
            }
            if (!bridge.setBrickWorldSolverMode(serviceKey, worldKey, CLIENT_L2_MODE.nativeSolverMode())) {
                lastError = "setBrickWorldSolverMode failed: " + bridge.lastError();
                return false;
            }
            return true;
        }

        private void handleActiveHints(ActiveHintsCommand command) {
            if (!ensureRuntime(command.worldKey())) {
                return;
            }
            if (!bridge.setBrickWorldExactActiveHints(serviceKey, command.worldKey(), BRICK_SIZE, command.activeHintCoords())) {
                lastError = "setBrickWorldExactActiveHints failed: " + bridge.lastError();
                return;
            }
            updateNativeStatus(command.worldKey());
        }

        private void handleWorldDeltas(WorldDeltasCommand command) {
            if (!ensureRuntime(command.worldKey())) {
                return;
            }
            if (!bridge.submitWorldDeltas(serviceKey, command.deltas())) {
                lastError = "submitWorldDeltas failed: " + bridge.lastError();
                return;
            }
            updateNativeStatus(command.worldKey());
        }

        private void handleBrickSeed(BrickSeedCommand command) {
            if (!ensureRuntime(command.worldKey())) {
                return;
            }
            if (!bridge.uploadBrickWorldStaticBrickWithSources(
                    serviceKey,
                    command.worldKey(),
                    BRICK_SIZE,
                    command.brickX(),
                    command.brickY(),
                    command.brickZ(),
                    command.obstacle(),
                    command.surfaceKind(),
                    command.openFaceMask(),
                    command.emitterPower(),
                    command.sourceFanDirection(),
                    command.sourceEmitterPower(),
                    command.faceSkyExposure(),
                    command.faceDirectExposure()
            )) {
                lastError = "uploadBrickWorldStaticBrick failed: " + bridge.lastError();
                return;
            }
            if (!bridge.uploadBrickWorldDynamicBrick(
                    serviceKey,
                    command.worldKey(),
                    BRICK_SIZE,
                    command.brickX(),
                    command.brickY(),
                    command.brickZ(),
                    command.flowState(),
                    command.airTemperature(),
                    command.surfaceTemperature()
            )) {
                lastError = "uploadBrickWorldDynamicBrick failed: " + bridge.lastError();
                return;
            }
            if (!bridge.uploadBrickWorldBoundaryReferenceBrick(
                    serviceKey,
                    command.worldKey(),
                    BRICK_SIZE,
                    command.brickX(),
                    command.brickY(),
                    command.brickZ(),
                    command.flowState(),
                    command.airTemperature(),
                    command.surfaceTemperature()
            )) {
                lastError = "uploadBrickWorldBoundaryReferenceBrick failed: " + bridge.lastError();
                return;
            }
            updateNativeStatus(command.worldKey());
        }

        private void handleBoundaryReference(BoundaryReferenceCommand command) {
            if (!ensureRuntime(command.worldKey())) {
                return;
            }
            if (!bridge.uploadBrickWorldBoundaryReferenceBrick(
                    serviceKey,
                    command.worldKey(),
                    BRICK_SIZE,
                    command.brickX(),
                    command.brickY(),
                    command.brickZ(),
                    command.flowState(),
                    command.airTemperature(),
                    command.surfaceTemperature()
            )) {
                lastError = "boundary reference refresh failed: " + bridge.lastError();
                return;
            }
            if (command.maxCoarseSpeedMetersPerSecond() >= COARSE_RESEED_MIN_SPEED_MPS
                    && shouldReseedZeroDynamicBrick(command)) {
                bridge.uploadBrickWorldDynamicBrick(
                        serviceKey,
                        command.worldKey(),
                        BRICK_SIZE,
                        command.brickX(),
                        command.brickY(),
                        command.brickZ(),
                        command.flowState(),
                        command.airTemperature(),
                        command.surfaceTemperature()
                );
            }
            updateNativeStatus(command.worldKey());
        }

        private boolean shouldReseedZeroDynamicBrick(BoundaryReferenceCommand command) {
            if (!bridge.copyBrickWorldDynamicBrick(
                    serviceKey,
                    command.worldKey(),
                    BRICK_SIZE,
                    command.brickX(),
                    command.brickY(),
                    command.brickZ(),
                    workerFlowState,
                    workerAirTemperature,
                    workerSurfaceTemperature
            )) {
                return true;
            }
            return maxFlowSpeedMetersPerSecond(workerFlowState) < ZERO_DYNAMIC_MAX_SPEED_EPS_MPS;
        }

        private void handleStep(StepCommand command) {
            if (!ensureRuntime(command.worldKey())) {
                return;
            }
            long start = System.nanoTime();
            if (!bridge.stepBrickWorldRuntime(serviceKey, command.worldKey(), Math.max(1, command.stepCount()))) {
                lastError = "stepBrickWorldRuntime failed: " + bridge.lastError();
                return;
            }
            lastStepNanos = System.nanoTime() - start;
            if (command.publishTargets().length > 0) {
                publishTargets(command.worldKey(), command.publishTargets());
            }
            updateNativeStatus(command.worldKey());
        }

        private void publishTargets(long worldKey, PublishTarget[] targets) {
            long start = System.nanoTime();
            for (PublishTarget target : targets) {
                int sampleStride = LOCAL_PUBLISH_SAMPLE_STRIDE;
                short[] packedFlow = new short[packedValueCount(BRICK_SIZE, sampleStride)];
                if (!bridge.copyBrickWorldPackedFlowAtlas(
                        serviceKey,
                        worldKey,
                        BRICK_SIZE,
                        target.brickX(),
                        target.brickY(),
                        target.brickZ(),
                        sampleStride,
                        packedFlow
                )) {
                    if (!bridge.copyBrickWorldDynamicBrick(
                            serviceKey,
                            worldKey,
                            BRICK_SIZE,
                            target.brickX(),
                            target.brickY(),
                            target.brickZ(),
                            workerFlowState,
                            workerAirTemperature,
                            workerSurfaceTemperature
                    )) {
                        lastError = "copyBrickWorldDynamicBrick failed: " + bridge.lastError();
                        continue;
                    }
                    packFlowFromWorkerState(sampleStride, packedFlow);
                }
                atlases.offer(new LocalAtlasSnapshot(target.dimensionId(), target.origin(), sampleStride, packedFlow));
                publishedAtlases++;
            }
            lastPublishNanos = System.nanoTime() - start;
        }

        private int packedValueCount(int brickSize, int sampleStride) {
            int atlasResolution = (brickSize + sampleStride - 1) / sampleStride;
            return atlasResolution * atlasResolution * atlasResolution * PACKED_CHANNELS;
        }

        private void packFlowFromWorkerState(int sampleStride, short[] packedFlow) {
            int atlasResolution = (BRICK_SIZE + sampleStride - 1) / sampleStride;
            int dstBase = 0;
            for (int x = 0; x < atlasResolution; x++) {
                int gx = Math.min(BRICK_SIZE - 1, x * sampleStride);
                for (int y = 0; y < atlasResolution; y++) {
                    int gy = Math.min(BRICK_SIZE - 1, y * sampleStride);
                    for (int z = 0; z < atlasResolution; z++) {
                        int gz = Math.min(BRICK_SIZE - 1, z * sampleStride);
                        int srcBase = cellIndex(gx, gy, gz) * FLOW_CHANNELS;
                        packedFlow[dstBase] = quantizeSignedToShort(
                                workerFlowState[srcBase] * NATIVE_VELOCITY_SCALE,
                                ATLAS_VELOCITY_RANGE
                        );
                        packedFlow[dstBase + 1] = quantizeSignedToShort(
                                workerFlowState[srcBase + 1] * NATIVE_VELOCITY_SCALE,
                                ATLAS_VELOCITY_RANGE
                        );
                        packedFlow[dstBase + 2] = quantizeSignedToShort(
                                workerFlowState[srcBase + 2] * NATIVE_VELOCITY_SCALE,
                                ATLAS_VELOCITY_RANGE
                        );
                        packedFlow[dstBase + 3] = quantizeSignedToShort(workerFlowState[srcBase + 3], ATLAS_PRESSURE_RANGE);
                        dstBase += PACKED_CHANNELS;
                    }
                }
            }
        }

        private void updateNativeStatus(long worldKey) {
            if (serviceKey == 0L) {
                lastNativeStatus = null;
                lastRuntimeInfo = "-";
                return;
            }
            lastNativeStatus = bridge.getBrickWorldRuntimeStatus(serviceKey, worldKey);
            lastRuntimeInfo = bridge.runtimeInfo();
        }

        private void releaseService() {
            if (serviceKey != 0L) {
                bridge.releaseService(serviceKey);
                serviceKey = 0L;
            }
        }
    }

    private void resetActiveBrick() {
        activeOrigin = null;
        activeDimension = null;
        activeHintUploaded = false;
        activeBrickCount = 0;
        clearPendingStaticPatches();
        prepareCursor = 0;
        refreshCursor = 0;
        publishCursor = 0;
        java.util.Arrays.fill(activeBrickReady, false);
        java.util.Arrays.fill(activeBrickRefreshPending, false);
        java.util.Arrays.fill(activeBrickBoundaryRefreshPending, false);
        cancelStagedPreparation();
        cancelBoundaryReferenceRefresh();
        lastServerTick = Long.MIN_VALUE;
        lastProcessedClientGameTime = Long.MIN_VALUE;
        lastSolveClientGameTime = Long.MIN_VALUE;
        lastPublishedClientGameTime = Long.MIN_VALUE;
        lastBoundaryRefreshClientGameTime = Long.MIN_VALUE;
        lastStaticPatchSubmitClientGameTime = Long.MIN_VALUE;
        visualizer.clearLocalFlowFields();
    }

    private void close() {
        resetActiveBrick();
        clientSolveDisabled = false;
        staticBrickCache.clear();
        worker.close();
        fastSuspendUntilGameTime = Long.MIN_VALUE;
    }

    private void clearPendingStaticPatches() {
        pendingSourcePatches.clear();
        pendingStaticPatchDimension = null;
        pendingStaticPatchWorldKey = 0L;
        pendingStaticPatchFirstChangeGameTime = Long.MIN_VALUE;
        pendingStaticPatchLastChangeGameTime = Long.MIN_VALUE;
        pendingStaticPatchSourceChanges = 0;
    }

    String status() {
        if (!experimentalEnabled) {
            return "client L2 localSolve=off mode=" + CLIENT_L2_MODE.statusName();
        }
        return "client L2 localSolve=on mode=" + CLIENT_L2_MODE.statusName()
            + " experimental=" + CLIENT_L2_MODE.experimental()
            + " streaming=" + streamingEnabled
            + " disabled=" + clientSolveDisabled
            + " brickSize=" + BRICK_SIZE
            + " cells=" + CELL_COUNT
            + " activeBricks=" + activeBrickCount
            + " worker=" + worker.status()
            + " staticCache=" + staticBrickCache.size()
            + "/" + STATIC_CACHE_MAX_BRICKS
            + " staticPatches=" + lastStaticPatchCount
            + " pendingStaticPatches=" + pendingSourcePatches.size()
            + " pendingStaticPatchChanges=" + pendingStaticPatchSourceChanges
            + " fanPatchCells=" + lastFanPatchCellCount
            + " heatPatchCells=" + lastHeatPatchCellCount
            + " stress=" + stressStatus()
            + " solveInterval=" + SOLVE_INTERVAL_TICKS
            + " publishInterval=" + LOCAL_PUBLISH_INTERVAL_TICKS
            + " publishStride=" + LOCAL_PUBLISH_SAMPLE_STRIDE
            + " maxActive=" + MAX_CLIENT_ACTIVE_BRICKS
            + " prepBudget=" + STATIC_BUILD_CELLS_PER_TICK + "/" + (STATIC_BUILD_NANOS_PER_TICK / 1000L) + "us"
            + " seedBudget=" + COARSE_SEED_CELLS_PER_TICK + "/" + (COARSE_SEED_NANOS_PER_TICK / 1000L) + "us"
            + " boundaryBudget=" + BOUNDARY_REFERENCE_CELLS_PER_TICK + "/" + (BOUNDARY_REFERENCE_NANOS_PER_TICK / 1000L) + "us"
            + " prep=" + stagedPreparationStatus()
            + " boundaryPrep=" + boundaryReferenceRefreshStatus()
            + " fastSuspendUntil=" + fastSuspendUntilGameTime
            + " lastServerTick=" + lastServerTick;
    }

    String setStressMode(String modeName) {
        StressMode requested = StressMode.parse(modeName);
        if (stressMode != requested) {
            markAllActiveBricksStaticRefreshPending();
        }
        stressMode = requested;
        stressStartedGameTime = Long.MIN_VALUE;
        lastStressSubmitGameTime = Long.MIN_VALUE;
        stressSubmittedTicks = 0L;
        stressSubmittedPatches = 0L;
        stressSubmittedFanCells = 0L;
        stressSubmittedHeatCells = 0L;
        stressSubmittedDirtyCells = 0L;
        stressStaticSubmittedForActiveSet = false;
        return "Client L2 stress " + requested.name().toLowerCase(java.util.Locale.ROOT);
    }

    String stressStatus() {
        return stressMode.name().toLowerCase(java.util.Locale.ROOT)
                + ":ticks=" + stressSubmittedTicks
                + ":patches=" + stressSubmittedPatches
                + ":fan=" + stressSubmittedFanCells
                + ":heat=" + stressSubmittedHeatCells
                + ":dirty=" + stressSubmittedDirtyCells
                + ":staticSubmitted=" + stressStaticSubmittedForActiveSet
                + ":patchesPerTick=" + STRESS_PATCHES_PER_TICK
                + ":interval=" + STRESS_INTERVAL_TICKS
                + ":queueLimit=" + STRESS_QUEUE_BACKLOG_LIMIT;
    }

    private String formatNativeStatus(NativeSimulationBridge.BrickWorldRuntimeStatus status) {
        if (status == null) {
            return "none";
        }
        return "known=" + status.knownBrickCount()
                + ",hints=" + status.activeHintCount()
                + ",active=" + status.activeBrickCount()
                + ",dirty=" + status.geometryDirtyCount()
                + ",forcingDirty=" + status.forcingDirtyCount()
                + ",reinit=" + status.pendingReinitCount()
                + ",epoch=" + status.epoch();
    }

    private static String formatMillis(long nanos) {
        return String.format("%.3f", nanos / 1_000_000.0);
    }

    private String stagedPreparationStatus() {
        if (stagedActiveIndex < 0) {
            return "idle";
        }
        return stagedBrickX + "," + stagedBrickY + "," + stagedBrickZ
                + ":static=" + stagedStaticCursor + "/" + CELL_COUNT
                + ":seed=" + stagedSeedCursor + "/" + CELL_COUNT
                + ":staticUploaded=" + stagedStaticUploaded
                + ":dynamicUploaded=" + stagedDynamicUploaded;
    }

    private String boundaryReferenceRefreshStatus() {
        if (boundaryRefreshActiveIndex < 0) {
            return "idle";
        }
        return boundaryRefreshBrickX + "," + boundaryRefreshBrickY + "," + boundaryRefreshBrickZ
                + ":cells=" + boundaryRefreshCursor + "/" + CELL_COUNT
                + ":maxCoarse=" + String.format("%.3f", boundaryRefreshMaxCoarseSpeed);
    }

    void setExperimentalEnabled(boolean enabled) {
        if (CLIENT_L2_MODE == SolverMode.OFF) {
            enabled = false;
        }
        if (experimentalEnabled == enabled) {
            return;
        }
        experimentalEnabled = enabled;
        clientSolveDisabled = false;
        fastSuspendUntilGameTime = Long.MIN_VALUE;
        resetActiveBrick();
        if (!enabled) {
            staticBrickCache.clear();
            worker.reset();
        }
        LOGGER.info("Client L2 local solve {}", enabled ? "enabled" : "disabled");
    }

    boolean isExperimentalEnabled() {
        return experimentalEnabled;
    }

    private void disableClientSolve(Minecraft client, String reason) {
        clientSolveDisabled = true;
        resetActiveBrick();
        maybeLog(client, "disabled client L2: " + reason);
    }

    private void maybeLog(Minecraft client, String message) {
        if (client.level == null) {
            return;
        }
        long now = client.level.getGameTime();
        if (lastDiagnosticGameTime == Long.MIN_VALUE || now - lastDiagnosticGameTime >= 100) {
            LOGGER.info("Client L2 idle: {}", message);
            lastDiagnosticGameTime = now;
        }
    }
}
