package com.aerodynamics4mc.client;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.aerodynamics4mc.ModBlocks;
import com.aerodynamics4mc.api.AeroWindSample;
import com.aerodynamics4mc.net.AeroCoarseWindPayload;
import com.aerodynamics4mc.runtime.NativeSimulationBridge;

import net.fabricmc.fabric.api.client.event.lifecycle.v1.ClientLifecycleEvents;
import net.fabricmc.fabric.api.client.event.lifecycle.v1.ClientTickEvents;
import net.fabricmc.fabric.api.client.networking.v1.ClientPlayConnectionEvents;
import net.minecraft.block.BlockState;
import net.minecraft.client.MinecraftClient;
import net.minecraft.client.world.ClientWorld;
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
    private static final int STATIC_REFRESH_TICKS = 20;
    private static final int LOCAL_PUBLISH_INTERVAL_TICKS = 2;
    private static final int COUPLING_BAND_CELLS = 8;
    private static final int MAX_CLIENT_ACTIVE_BRICKS = 2;
    private static final int MAX_PENDING_SERVER_STEPS = 1;
    private static final int MAX_STEPS_PER_CLIENT_TICK = 1;
    private static final float DT_SECONDS = 0.05f;
    private static final float DX_METERS = 1.0f;
    private static final float NATIVE_VELOCITY_SCALE = DX_METERS / DT_SECONDS;
    private static final float ATLAS_VELOCITY_RANGE = 5.6f;
    private static final float ATLAS_PRESSURE_RANGE = 0.03f;
    private static final boolean EXPERIMENTAL_CLIENT_L2_DEFAULT = false;

    private final AeroVisualizer visualizer;
    private final NativeSimulationBridge bridge = new NativeSimulationBridge();
    private final byte[] obstacle = new byte[CELL_COUNT];
    private final byte[] surfaceKind = new byte[CELL_COUNT];
    private final short[] openFaceMask = new short[CELL_COUNT];
    private final float[] emitterPower = new float[CELL_COUNT];
    private final byte[] faceSkyExposure = new byte[CELL_COUNT * FACE_COUNT];
    private final byte[] faceDirectExposure = new byte[CELL_COUNT * FACE_COUNT];
    private final float[] flowState = new float[CELL_COUNT * FLOW_CHANNELS];
    private final float[] airTemperature = new float[CELL_COUNT];
    private final float[] surfaceTemperature = new float[CELL_COUNT];
    private final short[] packedFlow = new short[CELL_COUNT * PACKED_CHANNELS];
    private final int[] activeBrickX = new int[MAX_CLIENT_ACTIVE_BRICKS];
    private final int[] activeBrickY = new int[MAX_CLIENT_ACTIVE_BRICKS];
    private final int[] activeBrickZ = new int[MAX_CLIENT_ACTIVE_BRICKS];
    private final boolean[] activeBrickReady = new boolean[MAX_CLIENT_ACTIVE_BRICKS];
    private final boolean[] activeBrickRefreshPending = new boolean[MAX_CLIENT_ACTIVE_BRICKS];
    private int[] activeHintCoords = new int[NativeSimulationBridge.BRICK_HINT_COORDS_PER_BRICK];

    private long serviceKey;
    private long worldKey;
    private BlockPos activeOrigin;
    private Identifier activeDimension;
    private boolean streamingEnabled;
    private boolean experimentalEnabled = EXPERIMENTAL_CLIENT_L2_DEFAULT;
    private boolean activeHintUploaded;
    private boolean clientSolveDisabled;
    private int activeBrickCount;
    private int prepareCursor;
    private int refreshCursor;
    private int publishCursor;
    private int ticksSinceStaticRefresh = STATIC_REFRESH_TICKS;
    private int pendingServerSteps;
    private long lastServerTick = Long.MIN_VALUE;
    private long lastProcessedClientGameTime = Long.MIN_VALUE;
    private long lastPublishedClientGameTime = Long.MIN_VALUE;
    private long lastDiagnosticGameTime = Long.MIN_VALUE;

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
            pendingServerSteps = MAX_PENDING_SERVER_STEPS;
        }
        lastServerTick = serverTick;
    }

    private void onClientTick(MinecraftClient client) {
        if (!experimentalEnabled || !streamingEnabled || client.world == null || client.player == null) {
            return;
        }
        if (clientSolveDisabled) {
            return;
        }
        if (!bridge.isLoaded()) {
            maybeLog(client, "native library not loaded: " + bridge.getLoadError());
            return;
        }
        ensureService();
        if (serviceKey == 0L) {
            maybeLog(client, "failed to create native service");
            return;
        }

        ClientWorld world = client.world;
        long clientGameTime = world.getTime();
        if (lastProcessedClientGameTime == clientGameTime) {
            return;
        }
        lastProcessedClientGameTime = clientGameTime;

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
            lastPublishedClientGameTime = Long.MIN_VALUE;
            visualizer.clearLocalFlowFields();
            ticksSinceStaticRefresh = 0;
        }

        if (!bridge.ensureBrickWorldRuntime(serviceKey, worldKey, BRICK_SIZE, DX_METERS, DT_SECONDS)) {
            maybeLog(client, "ensureBrickWorldRuntime failed: " + bridge.lastError());
            return;
        }
        if (!activeHintUploaded) {
            if (!bridge.setBrickWorldExactActiveHints(serviceKey, worldKey, BRICK_SIZE, activeHintCoords)) {
                disableClientSolve(client, "exact active hints unavailable; rebuild native library");
                return;
            }
            activeHintUploaded = true;
        }

        if (!prepareActiveBricks(client, world, dimensionId)) {
            return;
        }
        if (refreshActiveBrickStatic(client, world)) {
            return;
        }
        if (pendingServerSteps <= 0) {
            return;
        }
        NativeSimulationBridge.BrickWorldRuntimeStatus status = bridge.getBrickWorldRuntimeStatus(serviceKey, worldKey);
        if (status != null && status.activeBrickCount() > MAX_CLIENT_ACTIVE_BRICKS) {
            disableClientSolve(client, "unexpected active brick count=" + status.activeBrickCount());
            return;
        }
        int stepsToRun = Math.min(pendingServerSteps, MAX_STEPS_PER_CLIENT_TICK);
        for (int i = 0; i < stepsToRun; i++) {
            if (!bridge.stepBrickWorldRuntime(serviceKey, worldKey, 1)) {
                maybeLog(client, "stepBrickWorldRuntime failed: " + bridge.lastError());
                return;
            }
            pendingServerSteps--;
        }
        if (lastPublishedClientGameTime == Long.MIN_VALUE
            || clientGameTime - lastPublishedClientGameTime >= LOCAL_PUBLISH_INTERVAL_TICKS) {
            publishNextLocalAtlas(dimensionId);
            lastPublishedClientGameTime = clientGameTime;
        }
    }

    private void ensureService() {
        if (serviceKey == 0L) {
            serviceKey = bridge.createService();
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
        int[] neighborOffset = nearestBoundaryNeighborOffset(localX, localY, localZ);
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
        addActiveBrick(
            coreBrickX,
            coreBrickY,
            coreBrickZ,
            oldActiveBrickX,
            oldActiveBrickY,
            oldActiveBrickZ,
            oldActiveBrickReady
        );
        int[] neighborOffset = nearestBoundaryNeighborOffset(localX, localY, localZ);
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
            if (!uploadAndSeedActiveBrick(client, world, dimensionId, index)) {
                return false;
            }
            activeBrickReady[index] = true;
            prepareCursor = (index + 1) % activeBrickCount;
            return false;
        }
        return true;
    }

    private boolean uploadAndSeedActiveBrick(
        MinecraftClient client,
        ClientWorld world,
        Identifier dimensionId,
        int activeIndex
    ) {
        int brickX = activeBrickX[activeIndex];
        int brickY = activeBrickY[activeIndex];
        int brickZ = activeBrickZ[activeIndex];
        BlockPos origin = brickOrigin(brickX, brickY, brickZ);
        if (!uploadStaticBrick(world, origin, brickX, brickY, brickZ)) {
            maybeLog(client, "client L2 static upload failed: " + bridge.lastError());
            return false;
        }
        if (!seedDynamicBrick(world, dimensionId, origin, brickX, brickY, brickZ)) {
            maybeLog(client, "client L2 dynamic seed failed: " + bridge.lastError());
            return false;
        }
        return true;
    }

    private boolean refreshActiveBrickStatic(MinecraftClient client, ClientWorld world) {
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
            if (!uploadStaticBrick(world, brickOrigin(brickX, brickY, brickZ), brickX, brickY, brickZ)) {
                maybeLog(client, "client L2 static refresh failed: " + bridge.lastError());
                return true;
            }
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

    private void publishNextLocalAtlas(Identifier dimensionId) {
        if (activeBrickCount <= 0) {
            return;
        }
        for (int attempts = 0; attempts < activeBrickCount; attempts++) {
            int index = (publishCursor + attempts) % activeBrickCount;
            if (!activeBrickReady[index]) {
                continue;
            }
            int brickX = activeBrickX[index];
            int brickY = activeBrickY[index];
            int brickZ = activeBrickZ[index];
            if (publishLocalAtlas(dimensionId, brickOrigin(brickX, brickY, brickZ), brickX, brickY, brickZ)) {
                publishCursor = (index + 1) % activeBrickCount;
                return;
            }
        }
    }

    private boolean uploadStaticBrick(ClientWorld world, BlockPos origin, int brickX, int brickY, int brickZ) {
        java.util.Arrays.fill(obstacle, (byte) 0);
        java.util.Arrays.fill(surfaceKind, (byte) 0);
        java.util.Arrays.fill(openFaceMask, (short) 0);
        java.util.Arrays.fill(emitterPower, 0.0f);
        java.util.Arrays.fill(faceSkyExposure, (byte) 0);
        java.util.Arrays.fill(faceDirectExposure, (byte) 0);

        BlockPos.Mutable cursor = new BlockPos.Mutable();
        BlockPos.Mutable neighbor = new BlockPos.Mutable();
        for (int x = 0; x < BRICK_SIZE; x++) {
            for (int y = 0; y < BRICK_SIZE; y++) {
                for (int z = 0; z < BRICK_SIZE; z++) {
                    cursor.set(origin.getX() + x, origin.getY() + y, origin.getZ() + z);
                    int cell = cellIndex(x, y, z);
                    BlockState state = world.getBlockState(cursor);
                    boolean solid = isSolidObstacle(world, cursor, state);
                    obstacle[cell] = solid ? (byte) 1 : (byte) 0;
                    if (solid) {
                        continue;
                    }
                    short mask = 0;
                    for (Direction direction : Direction.values()) {
                        neighbor.set(
                            cursor.getX() + direction.getOffsetX(),
                            cursor.getY() + direction.getOffsetY(),
                            cursor.getZ() + direction.getOffsetZ()
                        );
                        if (!isSolidObstacle(world, neighbor, world.getBlockState(neighbor))) {
                            mask = (short) (mask | (1 << direction.ordinal()));
                        }
                    }
                    openFaceMask[cell] = mask;
                }
            }
        }
        boolean uploaded = bridge.uploadBrickWorldStaticBrick(
            serviceKey,
            worldKey,
            BRICK_SIZE,
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
        if (!uploaded) {
            LOGGER.debug("client L2 static upload failed: {}", bridge.lastError());
        }
        return uploaded;
    }

    private boolean seedDynamicBrick(ClientWorld world, Identifier dimensionId, BlockPos origin, int brickX, int brickY, int brickZ) {
        java.util.Arrays.fill(flowState, 0.0f);
        java.util.Arrays.fill(airTemperature, 0.0f);
        java.util.Arrays.fill(surfaceTemperature, 0.0f);
        for (int x = 0; x < BRICK_SIZE; x++) {
            for (int y = 0; y < BRICK_SIZE; y++) {
                for (int z = 0; z < BRICK_SIZE; z++) {
                    int cell = cellIndex(x, y, z);
                    if (obstacle[cell] != 0) {
                        continue;
                    }
                    Vec3d pos = new Vec3d(origin.getX() + x + 0.5, origin.getY() + y + 0.5, origin.getZ() + z + 0.5);
                    AeroWindSample coarse = visualizer.sampleServerCoarseFlow(dimensionId, pos);
                    int base = cell * FLOW_CHANNELS;
                    flowState[base] = coarse.velocityX() / NATIVE_VELOCITY_SCALE;
                    flowState[base + 1] = coarse.velocityY() / NATIVE_VELOCITY_SCALE;
                    flowState[base + 2] = coarse.velocityZ() / NATIVE_VELOCITY_SCALE;
                    flowState[base + 3] = coarse.pressure();
                }
            }
        }
        boolean dynamicUploaded = bridge.uploadBrickWorldDynamicBrick(
            serviceKey,
            worldKey,
            BRICK_SIZE,
            brickX,
            brickY,
            brickZ,
            flowState,
            airTemperature,
            surfaceTemperature
        );
        boolean boundaryUploaded = bridge.uploadBrickWorldBoundaryReferenceBrick(
            serviceKey,
            worldKey,
            BRICK_SIZE,
            brickX,
            brickY,
            brickZ,
            flowState,
            airTemperature,
            surfaceTemperature
        );
        return dynamicUploaded && boundaryUploaded;
    }

    private boolean publishLocalAtlas(Identifier dimensionId, BlockPos origin, int brickX, int brickY, int brickZ) {
        if (!bridge.copyBrickWorldDynamicBrick(
            serviceKey,
            worldKey,
            BRICK_SIZE,
            brickX,
            brickY,
            brickZ,
            flowState,
            airTemperature,
            surfaceTemperature
        )) {
            return false;
        }
        for (int cell = 0; cell < CELL_COUNT; cell++) {
            int srcBase = cell * FLOW_CHANNELS;
            int dstBase = cell * PACKED_CHANNELS;
            packedFlow[dstBase] = quantizeSignedToShort(flowState[srcBase] * NATIVE_VELOCITY_SCALE, ATLAS_VELOCITY_RANGE);
            packedFlow[dstBase + 1] = quantizeSignedToShort(flowState[srcBase + 1] * NATIVE_VELOCITY_SCALE, ATLAS_VELOCITY_RANGE);
            packedFlow[dstBase + 2] = quantizeSignedToShort(flowState[srcBase + 2] * NATIVE_VELOCITY_SCALE, ATLAS_VELOCITY_RANGE);
            packedFlow[dstBase + 3] = quantizeSignedToShort(flowState[srcBase + 3], ATLAS_PRESSURE_RANGE);
        }
        visualizer.onLocalFlowField(dimensionId, origin, packedFlow);
        return true;
    }

    private boolean isSolidObstacle(ClientWorld world, BlockPos pos, BlockState state) {
        if (state.isAir() || state.isOf(ModBlocks.DUCT_BLOCK)) {
            return false;
        }
        return !state.getCollisionShape(world, pos).isEmpty();
    }

    private short quantizeSignedToShort(float value, float range) {
        if (!(range > 0.0f) || !Float.isFinite(value)) {
            return 0;
        }
        float normalized = MathHelper.clamp(value / range, -1.0f, 1.0f);
        return (short) Math.round(normalized * 32767.0f);
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
        long value = 1469598103934665603L;
        String text = dimensionId.toString();
        for (int i = 0; i < text.length(); i++) {
            value = (value ^ text.charAt(i)) * 1099511628211L;
        }
        return value == 0L ? 1L : value;
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
        pendingServerSteps = 0;
        lastServerTick = Long.MIN_VALUE;
        lastProcessedClientGameTime = Long.MIN_VALUE;
        lastPublishedClientGameTime = Long.MIN_VALUE;
        visualizer.clearLocalFlowFields();
    }

    private void close() {
        resetActiveBrick();
        clientSolveDisabled = false;
        if (serviceKey != 0L) {
            bridge.releaseService(serviceKey);
            serviceKey = 0L;
        }
    }

    String status() {
        if (!experimentalEnabled) {
            return "client L2 experimental=off";
        }
        return "client L2 experimental=on streaming=" + streamingEnabled
            + " disabled=" + clientSolveDisabled
            + " activeBricks=" + activeBrickCount
            + " pendingServerSteps=" + pendingServerSteps;
    }

    void setExperimentalEnabled(boolean enabled) {
        if (experimentalEnabled == enabled) {
            return;
        }
        experimentalEnabled = enabled;
        clientSolveDisabled = false;
        resetActiveBrick();
        LOGGER.info("Client L2 experimental {}", enabled ? "enabled" : "disabled");
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
