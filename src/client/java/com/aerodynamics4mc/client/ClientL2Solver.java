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
import com.aerodynamics4mc.api.AeroWindSamplingRules;
import com.aerodynamics4mc.api.AeroWindSample;
import com.aerodynamics4mc.net.AeroCoarseWindPayload;
import com.aerodynamics4mc.runtime.NativeSimulationBridge;

import net.fabricmc.fabric.api.client.event.lifecycle.v1.ClientLifecycleEvents;
import net.fabricmc.fabric.api.client.event.lifecycle.v1.ClientTickEvents;
import net.fabricmc.fabric.api.client.networking.v1.ClientPlayConnectionEvents;
import net.minecraft.block.Blocks;
import net.minecraft.block.BlockState;
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

    private static final int BRICK_SIZE = 32;
    private static final int CELL_COUNT = BRICK_SIZE * BRICK_SIZE * BRICK_SIZE;
    private static final int FLOW_CHANNELS = NativeSimulationBridge.FLOW_STATE_CHANNELS;
    private static final int PACKED_CHANNELS = NativeSimulationBridge.PACKED_ATLAS_CHANNELS;
    private static final int FACE_COUNT = Direction.values().length;
    private static final int STATIC_REFRESH_TICKS = -1;
    private static final int LOCAL_PUBLISH_INTERVAL_TICKS = 4;
    private static final int SOLVE_INTERVAL_TICKS = 2;
    private static final int BOUNDARY_REFERENCE_REFRESH_MIN_TICKS = 40;
    private static final int FAST_SUSPEND_COOLDOWN_TICKS = 10;
    private static final int STATIC_BUILD_CELLS_PER_TICK = 1024;
    private static final int COARSE_SEED_CELLS_PER_TICK = 4096;
    private static final int STATIC_CACHE_MAX_BRICKS = 32;
    private static final int COUPLING_BAND_CELLS = 8;
    private static final int MAX_CLIENT_ACTIVE_BRICKS = 1;
    private static final int MAX_STEPS_PER_CLIENT_TICK = 1;
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
    private static final byte SURFACE_KIND_FAN_X_NEG = 32;
    private static final byte SURFACE_KIND_FAN_X_POS = 33;
    private static final byte SURFACE_KIND_FAN_Y_NEG = 34;
    private static final byte SURFACE_KIND_FAN_Y_POS = 35;
    private static final byte SURFACE_KIND_FAN_Z_NEG = 36;
    private static final byte SURFACE_KIND_FAN_Z_POS = 37;
    private static final int WORKER_QUEUE_CAPACITY = 128;
    private static final boolean CLIENT_L2_DEFAULT_ENABLED = false;

    private final AeroVisualizer visualizer;
    private final ClientL2Worker worker = new ClientL2Worker();
    private final byte[] obstacle = new byte[CELL_COUNT];
    private final byte[] surfaceKind = new byte[CELL_COUNT];
    private final short[] openFaceMask = new short[CELL_COUNT];
    private final float[] emitterPower = new float[CELL_COUNT];
    private final byte[] faceSkyExposure = new byte[CELL_COUNT * FACE_COUNT];
    private final byte[] faceDirectExposure = new byte[CELL_COUNT * FACE_COUNT];
    private final float[] flowState = new float[CELL_COUNT * FLOW_CHANNELS];
    private final float[] airTemperature = new float[CELL_COUNT];
    private final float[] surfaceTemperature = new float[CELL_COUNT];
    private final BlockPos.Mutable staticCursor = new BlockPos.Mutable();
    private final BlockPos.Mutable staticNeighbor = new BlockPos.Mutable();
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
    private BlockPos stagedOrigin;
    private Identifier stagedDimension;
    private int ticksSinceStaticRefresh = STATIC_REFRESH_TICKS;
    private long lastServerTick = Long.MIN_VALUE;
    private long lastProcessedClientGameTime = Long.MIN_VALUE;
    private long lastSolveClientGameTime = Long.MIN_VALUE;
    private long lastPublishedClientGameTime = Long.MIN_VALUE;
    private long lastBoundaryRefreshClientGameTime = Long.MIN_VALUE;
    private long fastSuspendUntilGameTime = Long.MIN_VALUE;
    private long lastDiagnosticGameTime = Long.MIN_VALUE;
    private int lastStaticPatchCount;

    ClientL2Solver(AeroVisualizer visualizer) {
        this.visualizer = visualizer;
    }

    void initialize() {
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

    void onBlockStateChanged(ClientWorld world, BlockPos pos, BlockState oldState, BlockState newState) {
        if (world == null || pos == null || oldState == newState || (oldState != null && oldState.equals(newState))) {
            return;
        }
        Identifier dimensionId = world.getRegistryKey().getValue();
        invalidateStaticCacheAround(dimensionId, pos);
        if (!experimentalEnabled || !streamingEnabled || clientSolveDisabled || worldKey == 0L) {
            return;
        }
        if (activeDimension == null || !activeDimension.equals(dimensionId) || activeBrickCount <= 0) {
            return;
        }
        if (!blockPatchTouchesActiveBrick(pos)) {
            return;
        }
        NativeSimulationBridge.WorldDelta[] deltas = buildStaticPatchDeltas(world, pos);
        if (deltas.length == 0) {
            return;
        }
        for (NativeSimulationBridge.WorldDelta delta : deltas) {
            refreshLocalStaticCellIfActive(world, new BlockPos(delta.x(), delta.y(), delta.z()));
        }
        lastStaticPatchCount = deltas.length;
        worker.submitWorldDeltas(worldKey, deltas);
    }

    private void onClientTick(MinecraftClient client) {
        drainWorkerAtlases();
        if (!experimentalEnabled || !streamingEnabled || client.world == null || client.player == null) {
            return;
        }
        if (clientSolveDisabled) {
            return;
        }
        ClientWorld world = client.world;
        long clientGameTime = world.getTime();
        if (lastProcessedClientGameTime == clientGameTime) {
            return;
        }
        lastProcessedClientGameTime = clientGameTime;

        float horizontalSpeed = AeroWindSamplingRules.horizontalSpeedMetersPerSecond(client.player.getVelocity());
        if (shouldSuspendForFastMovement(horizontalSpeed, clientGameTime)) {
            suspendForFastMovement(clientGameTime);
            return;
        }
        if (!worker.isNativeLoaded()) {
            maybeLog(client, "native library not loaded: " + worker.loadError());
            return;
        }

        Identifier dimensionId = world.getRegistryKey().getValue();
        BlockPos playerBlockPos = client.player.getBlockPos();
        BlockPos origin = brickOrigin(playerBlockPos);
        int brickX = Math.floorDiv(origin.getX(), BRICK_SIZE);
        int brickY = Math.floorDiv(origin.getY(), BRICK_SIZE);
        int brickZ = Math.floorDiv(origin.getZ(), BRICK_SIZE);
        int localX = playerBlockPos.getX() - origin.getX();
        int localY = playerBlockPos.getY() - origin.getY();
        int localZ = playerBlockPos.getZ() - origin.getZ();
        boolean originChanged = activeOrigin == null
            || !activeOrigin.equals(origin)
            || activeDimension == null
            || !activeDimension.equals(dimensionId);
        boolean activeSetChanged = originChanged || !activeSetMatches(brickX, brickY, brickZ, localX, localY, localZ);
        if (activeSetChanged) {
            activeOrigin = origin;
            activeDimension = dimensionId;
            worldKey = worldKey(dimensionId);
            activeHintUploaded = false;
            buildActiveBrickSet(brickX, brickY, brickZ, localX, localY, localZ);
            cancelStagedPreparation();
            lastPublishedClientGameTime = Long.MIN_VALUE;
            lastSolveClientGameTime = Long.MIN_VALUE;
            lastBoundaryRefreshClientGameTime = Long.MIN_VALUE;
            visualizer.clearLocalFlowFields();
            ticksSinceStaticRefresh = 0;
        }

        if (!activeHintUploaded) {
            worker.submitActiveHints(worldKey, activeHintCoords);
            activeHintUploaded = true;
        }

        if (!prepareActiveBricks(client, world, dimensionId)) {
            return;
        }
        if (refreshActiveBrickStatic(client, world)) {
            return;
        }
        if (refreshActiveBrickBoundaryReference(client, dimensionId, clientGameTime)) {
            return;
        }
        if (lastSolveClientGameTime != Long.MIN_VALUE
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
            visualizer.onLocalFlowField(snapshot.dimensionId(), snapshot.origin(), snapshot.packedFlow());
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
            publishCursor = (index + 1) % activeBrickCount;
            break;
        }
        return java.util.Arrays.copyOf(targets, count);
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
        return bestDistance < COUPLING_BAND_CELLS ? new int[] {bestX, bestY, bestZ} : null;
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

    private boolean prepareActiveBricks(MinecraftClient client, ClientWorld world, Identifier dimensionId) {
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

    private BrickPreparationResult uploadAndSeedActiveBrick(
        MinecraftClient client,
        ClientWorld world,
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
        java.util.Arrays.fill(obstacle, (byte) 0);
        java.util.Arrays.fill(surfaceKind, (byte) 0);
        java.util.Arrays.fill(openFaceMask, (short) 0);
        java.util.Arrays.fill(emitterPower, 0.0f);
        java.util.Arrays.fill(faceSkyExposure, (byte) 0);
        java.util.Arrays.fill(faceDirectExposure, (byte) 0);
        java.util.Arrays.fill(flowState, 0.0f);
        java.util.Arrays.fill(airTemperature, 0.0f);
        java.util.Arrays.fill(surfaceTemperature, 0.0f);
        StaticBrickSnapshot cached = staticBrickCache.get(new StaticBrickCacheKey(dimensionId, brickX, brickY, brickZ));
        if (cached != null) {
            cached.copyInto(obstacle, surfaceKind, openFaceMask, emitterPower);
            stagedStaticCursor = CELL_COUNT;
            stagedStaticFromCache = true;
        }
    }

    private boolean buildStagedStaticCells(ClientWorld world) {
        if (stagedOrigin == null) {
            return false;
        }
        int end = Math.min(CELL_COUNT, stagedStaticCursor + STATIC_BUILD_CELLS_PER_TICK);
        while (stagedStaticCursor < end) {
            int cell = stagedStaticCursor++;
            int x = cell / (BRICK_SIZE * BRICK_SIZE);
            int rem = cell - x * BRICK_SIZE * BRICK_SIZE;
            int y = rem / BRICK_SIZE;
            int z = rem - y * BRICK_SIZE;
            populateStaticCell(world, stagedOrigin, x, y, z);
        }
        return stagedStaticCursor >= CELL_COUNT;
    }

    private void cacheStagedStaticBrickIfNeeded() {
        if (stagedStaticFromCache || stagedDimension == null || stagedOrigin == null) {
            return;
        }
        staticBrickCache.put(
            new StaticBrickCacheKey(stagedDimension, stagedBrickX, stagedBrickY, stagedBrickZ),
            StaticBrickSnapshot.copyFrom(obstacle, surfaceKind, openFaceMask, emitterPower)
        );
        stagedStaticFromCache = true;
    }

    private boolean buildStagedCoarseSeedCells(Identifier dimensionId) {
        if (stagedOrigin == null) {
            return false;
        }
        int built = 0;
        while (stagedSeedCursor < CELL_COUNT && built < COARSE_SEED_CELLS_PER_TICK) {
            int cell = stagedSeedCursor;
            int x = cell / (BRICK_SIZE * BRICK_SIZE);
            int rem = cell - x * BRICK_SIZE * BRICK_SIZE;
            int y = rem / BRICK_SIZE;
            int z = rem - y * BRICK_SIZE;
            if (obstacle[cell] == 0) {
                Vec3d pos = new Vec3d(
                    stagedOrigin.getX() + x + 0.5,
                    stagedOrigin.getY() + y + 0.5,
                    stagedOrigin.getZ() + z + 0.5
                );
                AeroWindSample coarse = visualizer.sampleServerCoarseFlow(dimensionId, pos);
                if (!coarse.hasFlow()) {
                    return false;
                }
                int base = cell * FLOW_CHANNELS;
                flowState[base] = coarse.velocityX() / NATIVE_VELOCITY_SCALE;
                flowState[base + 1] = coarse.velocityY() / NATIVE_VELOCITY_SCALE;
                flowState[base + 2] = coarse.velocityZ() / NATIVE_VELOCITY_SCALE;
                flowState[base + 3] = coarse.pressure();
            }
            stagedSeedCursor++;
            built++;
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
    }

    private boolean refreshActiveBrickStatic(MinecraftClient client, ClientWorld world) {
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
        MinecraftClient client,
        Identifier dimensionId,
        long clientGameTime
    ) {
        if (!hasBoundaryRefreshPending()) {
            return false;
        }
        if (lastBoundaryRefreshClientGameTime != Long.MIN_VALUE
            && clientGameTime - lastBoundaryRefreshClientGameTime < BOUNDARY_REFERENCE_REFRESH_MIN_TICKS) {
            return false;
        }
        for (int attempts = 0; attempts < activeBrickCount; attempts++) {
            int index = (refreshCursor + attempts) % activeBrickCount;
            if (!activeBrickBoundaryRefreshPending[index]) {
                continue;
            }
            if (!activeBrickReady[index]) {
                activeBrickBoundaryRefreshPending[index] = false;
                continue;
            }
            int brickX = activeBrickX[index];
            int brickY = activeBrickY[index];
            int brickZ = activeBrickZ[index];
            BlockPos origin = brickOrigin(brickX, brickY, brickZ);
            CoarseSeedStats seedStats = fillFlowStateFromCoarse(dimensionId, origin);
            if (seedStats == null) {
                maybeLog(client, "client L2 boundary refresh waiting for coarse field");
                return true;
            }
            worker.submitBoundaryReference(new BoundaryReferenceCommand(
                worldKey,
                brickX,
                brickY,
                brickZ,
                java.util.Arrays.copyOf(flowState, flowState.length),
                java.util.Arrays.copyOf(airTemperature, airTemperature.length),
                java.util.Arrays.copyOf(surfaceTemperature, surfaceTemperature.length),
                seedStats.maxCoarseSpeedMetersPerSecond()
            ));
            activeBrickBoundaryRefreshPending[index] = false;
            refreshCursor = (index + 1) % activeBrickCount;
            lastBoundaryRefreshClientGameTime = clientGameTime;
            return true;
        }
        return false;
    }

    private boolean hasBoundaryRefreshPending() {
        for (int index = 0; index < activeBrickCount; index++) {
            if (activeBrickBoundaryRefreshPending[index]) {
                return true;
            }
        }
        return false;
    }

    private boolean uploadStaticBrick(ClientWorld world, BlockPos origin, int brickX, int brickY, int brickZ) {
        StaticBrickCacheKey key = activeDimension == null ? null : new StaticBrickCacheKey(activeDimension, brickX, brickY, brickZ);
        StaticBrickSnapshot cached = key == null ? null : staticBrickCache.get(key);
        if (cached != null) {
            cached.copyInto(obstacle, surfaceKind, openFaceMask, emitterPower);
            java.util.Arrays.fill(faceSkyExposure, (byte) 0);
            java.util.Arrays.fill(faceDirectExposure, (byte) 0);
            return true;
        }
        populateStaticBrickArrays(world, origin);
        if (key != null) {
            staticBrickCache.put(key, StaticBrickSnapshot.copyFrom(obstacle, surfaceKind, openFaceMask, emitterPower));
        }
        return true;
    }

    private NativeSimulationBridge.WorldDelta[] buildStaticPatchDeltas(ClientWorld world, BlockPos center) {
        NativeSimulationBridge.WorldDelta[] deltas = new NativeSimulationBridge.WorldDelta[1 + Direction.values().length];
        int count = 0;
        deltas[count++] = buildStaticCellPatchDelta(world, center);
        for (Direction direction : Direction.values()) {
            deltas[count++] = buildStaticCellPatchDelta(world, center.offset(direction));
        }
        return java.util.Arrays.copyOf(deltas, count);
    }

    private boolean blockPatchTouchesActiveBrick(BlockPos center) {
        if (blockInActiveBrick(center)) {
            return true;
        }
        for (Direction direction : Direction.values()) {
            if (blockInActiveBrick(center.offset(direction))) {
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

    private NativeSimulationBridge.WorldDelta buildStaticCellPatchDelta(ClientWorld world, BlockPos pos) {
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

    private void refreshLocalStaticCellIfActive(ClientWorld world, BlockPos pos) {
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

    private void invalidateStaticCacheAround(Identifier dimensionId, BlockPos pos) {
        invalidateStaticCacheForBlock(dimensionId, pos);
        for (Direction direction : Direction.values()) {
            invalidateStaticCacheForBlock(dimensionId, pos.offset(direction));
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

    private void populateStaticBrickArrays(ClientWorld world, BlockPos origin) {
        java.util.Arrays.fill(obstacle, (byte) 0);
        java.util.Arrays.fill(surfaceKind, (byte) 0);
        java.util.Arrays.fill(openFaceMask, (short) 0);
        java.util.Arrays.fill(emitterPower, 0.0f);
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

    private void populateStaticCell(ClientWorld world, BlockPos origin, int x, int y, int z) {
        staticCursor.set(origin.getX() + x, origin.getY() + y, origin.getZ() + z);
        int cell = cellIndex(x, y, z);
        StaticCellSample sample = sampleStaticCell(world, staticCursor);
        obstacle[cell] = sample.solid() ? (byte) 1 : (byte) 0;
        surfaceKind[cell] = sample.surfaceKind();
        openFaceMask[cell] = sample.openFaceMask();
        emitterPower[cell] = sample.emitterPowerWatts();
    }

    private StaticCellSample sampleStaticCell(ClientWorld world, BlockPos pos) {
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
        byte kind = solid ? (byte) 0 : fanSurfaceKindForCell(world, pos);
        float emitter = solid ? 0.0f : emitterPowerForCell(world, pos, state);
        return new StaticCellSample(solid, kind, mask, emitter);
    }

    private byte fanSurfaceKindForCell(ClientWorld world, BlockPos pos) {
        for (Direction direction : Direction.values()) {
            staticNeighbor.set(
                pos.getX() - direction.getOffsetX(),
                pos.getY() - direction.getOffsetY(),
                pos.getZ() - direction.getOffsetZ()
            );
            BlockState fanState = world.getBlockState(staticNeighbor);
            if (!fanState.isOf(ModBlocks.FAN_BLOCK)
                || fanState.getOrEmpty(FanBlock.FACING).orElse(Direction.NORTH) != direction) {
                continue;
            }
            return fanSurfaceKind(direction);
        }
        return 0;
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

    private float emitterPowerForCell(ClientWorld world, BlockPos pos, BlockState state) {
        float directPower = sampleEmitterThermalPowerWatts(state);
        if (directPower > 0.0f) {
            return directPower;
        }
        staticNeighbor.set(pos.getX(), pos.getY() - 1, pos.getZ());
        float belowPower = sampleEmitterThermalPowerWatts(world.getBlockState(staticNeighbor));
        return belowPower > 0.0f ? belowPower * HEAT_COUPLING_TO_ADJACENT_AIR : 0.0f;
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
                    Vec3d pos = new Vec3d(origin.getX() + x + 0.5, origin.getY() + y + 0.5, origin.getZ() + z + 0.5);
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

    private boolean isSolidObstacle(ClientWorld world, BlockPos pos, BlockState state) {
        if (state.isAir() || state.isOf(ModBlocks.DUCT_BLOCK)) {
            return false;
        }
        return !state.getCollisionShape(world, pos).isEmpty();
    }

    private static short quantizeSignedToShort(float value, float range) {
        if (!(range > 0.0f) || !Float.isFinite(value)) {
            return 0;
        }
        float normalized = MathHelper.clamp(value / range, -1.0f, 1.0f);
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

    private record StaticCellSample(boolean solid, byte surfaceKind, short openFaceMask, float emitterPowerWatts) {
    }

    private record StaticBrickSnapshot(byte[] obstacle, byte[] surfaceKind, short[] openFaceMask, float[] emitterPower) {
        static StaticBrickSnapshot copyFrom(
            byte[] obstacle,
            byte[] surfaceKind,
            short[] openFaceMask,
            float[] emitterPower
        ) {
            return new StaticBrickSnapshot(
                java.util.Arrays.copyOf(obstacle, obstacle.length),
                java.util.Arrays.copyOf(surfaceKind, surfaceKind.length),
                java.util.Arrays.copyOf(openFaceMask, openFaceMask.length),
                java.util.Arrays.copyOf(emitterPower, emitterPower.length)
            );
        }

        void copyInto(
            byte[] outObstacle,
            byte[] outSurfaceKind,
            short[] outOpenFaceMask,
            float[] outEmitterPower
        ) {
            System.arraycopy(obstacle, 0, outObstacle, 0, Math.min(obstacle.length, outObstacle.length));
            System.arraycopy(surfaceKind, 0, outSurfaceKind, 0, Math.min(surfaceKind.length, outSurfaceKind.length));
            System.arraycopy(openFaceMask, 0, outOpenFaceMask, 0, Math.min(openFaceMask.length, outOpenFaceMask.length));
            System.arraycopy(emitterPower, 0, outEmitterPower, 0, Math.min(emitterPower.length, outEmitterPower.length));
        }
    }

    private enum BrickPreparationResult {
        IN_PROGRESS,
        COMPLETED,
        FAILED
    }

    private interface WorkerCommand {
    }

    private record ActiveHintsCommand(long worldKey, int[] activeHintCoords) implements WorkerCommand {
    }

    private record WorldDeltasCommand(long worldKey, NativeSimulationBridge.WorldDelta[] deltas) implements WorkerCommand {
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

    private record LocalAtlasSnapshot(Identifier dimensionId, BlockPos origin, short[] packedFlow) {
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
                + ",error=" + lastError;
        }

        private void offer(WorkerCommand command) {
            if (!bridge.isLoaded()) {
                lastError = bridge.getLoadError();
                return;
            }
            startIfNeeded();
            if (!commands.offer(command)) {
                commands.poll();
                droppedCommands++;
                commands.offer(command);
            }
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
            if (!bridge.uploadBrickWorldStaticBrick(
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
                short[] packedFlow = new short[CELL_COUNT * PACKED_CHANNELS];
                for (int cell = 0; cell < CELL_COUNT; cell++) {
                    int srcBase = cell * FLOW_CHANNELS;
                    int dstBase = cell * PACKED_CHANNELS;
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
                }
                atlases.offer(new LocalAtlasSnapshot(target.dimensionId(), target.origin(), packedFlow));
                publishedAtlases++;
            }
            lastPublishNanos = System.nanoTime() - start;
        }

        private void updateNativeStatus(long worldKey) {
            lastNativeStatus = serviceKey == 0L ? null : bridge.getBrickWorldRuntimeStatus(serviceKey, worldKey);
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
        prepareCursor = 0;
        refreshCursor = 0;
        publishCursor = 0;
        java.util.Arrays.fill(activeBrickReady, false);
        java.util.Arrays.fill(activeBrickRefreshPending, false);
        java.util.Arrays.fill(activeBrickBoundaryRefreshPending, false);
        cancelStagedPreparation();
        lastServerTick = Long.MIN_VALUE;
        lastProcessedClientGameTime = Long.MIN_VALUE;
        lastSolveClientGameTime = Long.MIN_VALUE;
        lastPublishedClientGameTime = Long.MIN_VALUE;
        lastBoundaryRefreshClientGameTime = Long.MIN_VALUE;
        visualizer.clearLocalFlowFields();
    }

    private void close() {
        resetActiveBrick();
        clientSolveDisabled = false;
        staticBrickCache.clear();
        worker.close();
        fastSuspendUntilGameTime = Long.MIN_VALUE;
    }

    String status() {
        if (!experimentalEnabled) {
            return "client L2 localSolve=off";
        }
        return "client L2 localSolve=on streaming=" + streamingEnabled
            + " disabled=" + clientSolveDisabled
            + " activeBricks=" + activeBrickCount
            + " worker=" + worker.status()
            + " staticCache=" + staticBrickCache.size()
            + " staticPatches=" + lastStaticPatchCount
            + " solveInterval=" + SOLVE_INTERVAL_TICKS
            + " publishInterval=" + LOCAL_PUBLISH_INTERVAL_TICKS
            + " prep=" + stagedPreparationStatus()
            + " fastSuspendUntil=" + fastSuspendUntilGameTime
            + " lastServerTick=" + lastServerTick;
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

    void setExperimentalEnabled(boolean enabled) {
        if (experimentalEnabled == enabled) {
            return;
        }
        experimentalEnabled = enabled;
        clientSolveDisabled = false;
        fastSuspendUntilGameTime = Long.MIN_VALUE;
        resetActiveBrick();
        if (!enabled) {
            worker.reset();
        }
        LOGGER.info("Client L2 local solve {}", enabled ? "enabled" : "disabled");
    }

    boolean isExperimentalEnabled() {
        return experimentalEnabled;
    }

    private void disableClientSolve(MinecraftClient client, String reason) {
        clientSolveDisabled = true;
        resetActiveBrick();
        maybeLog(client, "disabled client L2: " + reason);
    }

    private void maybeLog(MinecraftClient client, String message) {
        if (client.world == null) {
            return;
        }
        long now = client.world.getTime();
        if (lastDiagnosticGameTime == Long.MIN_VALUE || now - lastDiagnosticGameTime >= 100) {
            LOGGER.info("Client L2 idle: {}", message);
            lastDiagnosticGameTime = now;
        }
    }
}
