package com.aerodynamics4mc.client;

import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicLong;

import org.joml.Matrix4f;

import com.aerodynamics4mc.FanBlock;
import com.aerodynamics4mc.ModBlocks;
import com.aerodynamics4mc.runtime.BackgroundFieldGrid;
import com.aerodynamics4mc.runtime.WorldThermalSampler;

import net.fabricmc.fabric.api.client.rendering.v1.world.WorldRenderContext;
import net.minecraft.block.BlockState;
import net.minecraft.block.entity.BlockEntity;
import net.minecraft.client.MinecraftClient;
import net.minecraft.client.render.RenderLayers;
import net.minecraft.client.render.VertexConsumer;
import net.minecraft.client.util.math.MatrixStack;
import net.minecraft.client.world.ClientWorld;
import net.minecraft.entity.player.PlayerEntity;
import net.minecraft.util.Identifier;
import net.minecraft.util.math.BlockPos;
import net.minecraft.util.math.ChunkPos;
import net.minecraft.util.math.Direction;
import net.minecraft.util.math.MathHelper;
import net.minecraft.util.math.Vec3d;
import net.minecraft.world.chunk.WorldChunk;

final class ClientFluidRuntime {
    private static final String LOG_PREFIX = "[aerodynamics4mc/client] ";

    private static final int CHANNELS = 9;
    private static final int CH_OBSTACLE = 0;
    private static final int CH_FAN_MASK = 1;
    private static final int CH_FAN_VX = 2;
    private static final int CH_FAN_VY = 3;
    private static final int CH_FAN_VZ = 4;
    private static final int CH_STATE_VX = 5;
    private static final int CH_STATE_VY = 6;
    private static final int CH_STATE_VZ = 7;
    private static final int CH_STATE_P = 8;

    private static final float INFLOW_SPEED = 8.0f;
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
    private static final int SOCKET_PORT = 5001;
    private static final String SOCKET_HOST = "127.0.0.1";
    private static final int RESPONSE_CHANNELS = 4;
    private static final int WINDOW_REFRESH_TICKS = 20;
    private static final int OBSTACLE_REFRESH_TICKS = 200;
    private static final int FAN_SCAN_RADIUS = 48;
    private static final int SOLVER_WORKER_COUNT = Math.max(2, Runtime.getRuntime().availableProcessors() / 2);

    private static final int DEFAULT_STREAMLINE_STRIDE = 4;
    private static final float NATIVE_VELOCITY_SCALE = 30.0f;
    private static final float BACKGROUND_VELOCITY_COUPLING = 0.08f;
    private static final int BACKGROUND_THERMAL_COMPONENT = 3;
    private static final double FORCE_STRENGTH = 0.02;
    private static final int ATMOS_DEBUG_STRIDE = 16;
    private static final float ATMOS_VECTOR_SCALE = 1.2f;
    private static final float ATMOS_VECTOR_MIN_SPEED = 1e-4f;
    private static final float ATMOS_THERMAL_RANGE = 6.0f;
    private static final float ATMOS_THERMAL_MIN_MARKER = 0.10f;

    private final int gridSize;
    private final int flowCount;
    private final Map<WindowKey, WindowState> windows = new HashMap<>();
    private final List<WindowState> retiredWindows = new ArrayList<>();
    private final NativeLbmBridge nativeBackend = new NativeLbmBridge();
    private final BackgroundFieldGrid backgroundField = new BackgroundFieldGrid(new BackgroundFieldGrid.FlowSolver() {
        @Override
        public boolean step(int gridSize, byte[] payload, long contextId, float[] output) {
            return stepBackgroundFlow(gridSize, payload, contextId, output);
        }

        @Override
        public void releaseContext(long contextId) {
            releaseBackgroundFlowContext(contextId);
        }
    });
    private final ExecutorService solverExecutor = Executors.newFixedThreadPool(SOLVER_WORKER_COUNT);
    private final AtomicInteger activeSolveTasks = new AtomicInteger(0);
    private final AtomicLong runtimeGeneration = new AtomicLong(0L);

    private SolverBackend backendMode = SolverBackend.NATIVE;
    private float maxWindSpeed = INFLOW_SPEED;
    private int streamlineSampleStride = DEFAULT_STREAMLINE_STRIDE;
    private boolean streamlineRoiEnabled = false;
    private int streamlineRoiMinX = 0;
    private int streamlineRoiMaxX = 0;
    private int streamlineRoiMinY = 0;
    private int streamlineRoiMaxY = 0;
    private int streamlineRoiMinZ = 0;
    private int streamlineRoiMaxZ = 0;
    private boolean renderVelocityVectors = true;
    private boolean renderStreamlines = true;
    private boolean renderBackgroundVectors = false;
    private boolean renderThermalAnomaly = false;
    private int tickCounter = 0;
    private long nextContextId = 1L;
    private Identifier currentDimensionId;
    private String lastSolverError = "";

    ClientFluidRuntime(int gridSize) {
        this.gridSize = gridSize;
        this.flowCount = gridSize * gridSize * gridSize * RESPONSE_CHANNELS;
        this.streamlineRoiMaxX = gridSize - 1;
        this.streamlineRoiMaxY = gridSize - 1;
        this.streamlineRoiMaxZ = gridSize - 1;
    }

    void setBackendModeId(int backendModeId) {
        switchBackend(SolverBackend.fromId(backendModeId));
    }

    void setMaxWindSpeed(float maxWindSpeed) {
        this.maxWindSpeed = Math.max(1e-6f, maxWindSpeed);
    }

    void setStreamlineSampleStride(int stride) {
        this.streamlineSampleStride = sanitizeStride(stride);
    }

    void setStreamlineRoi(boolean enabled, int minX, int maxX, int minY, int maxY, int minZ, int maxZ) {
        streamlineRoiEnabled = enabled;
        streamlineRoiMinX = clampRoiCoord(minX);
        streamlineRoiMaxX = clampRoiCoord(maxX);
        streamlineRoiMinY = clampRoiCoord(minY);
        streamlineRoiMaxY = clampRoiCoord(maxY);
        streamlineRoiMinZ = clampRoiCoord(minZ);
        streamlineRoiMaxZ = clampRoiCoord(maxZ);
        if (streamlineRoiMinX > streamlineRoiMaxX || streamlineRoiMinY > streamlineRoiMaxY || streamlineRoiMinZ > streamlineRoiMaxZ) {
            streamlineRoiEnabled = false;
            streamlineRoiMinX = 0;
            streamlineRoiMaxX = gridSize - 1;
            streamlineRoiMinY = 0;
            streamlineRoiMaxY = gridSize - 1;
            streamlineRoiMinZ = 0;
            streamlineRoiMaxZ = gridSize - 1;
        }
    }

    void setRenderVelocityVectors(boolean enabled) {
        this.renderVelocityVectors = enabled;
    }

    void setRenderStreamlines(boolean enabled) {
        this.renderStreamlines = enabled;
    }

    void setRenderBackgroundVectors(boolean enabled) {
        this.renderBackgroundVectors = enabled;
    }

    void setRenderThermalAnomaly(boolean enabled) {
        this.renderThermalAnomaly = enabled;
    }

    void tick(MinecraftClient client) {
        if (client == null || client.world == null || client.player == null || client.player.isSpectator()) {
            return;
        }
        if (client.isPaused()) {
            return;
        }

        ClientWorld world = client.world;
        Identifier dimensionId = world.getRegistryKey().getValue();
        if (currentDimensionId == null || !currentDimensionId.equals(dimensionId)) {
            retireAllWindows();
            cleanupRetiredWindowsIfIdle();
            currentDimensionId = dimensionId;
            backgroundField.clear();
        }

        tickCounter++;
        if (windows.isEmpty() || tickCounter % WINDOW_REFRESH_TICKS == 0) {
            refreshWindows(world);
        }
        if (windows.isEmpty()) {
            backgroundField.clear();
            return;
        }

        List<BlockPos> windowOrigins = new ArrayList<>(windows.size());
        for (WindowKey key : windows.keySet()) {
            windowOrigins.add(key.origin());
        }
        backgroundField.update(
            currentDimensionId,
            world.getTimeOfDay(),
            tickCounter,
            windowOrigins,
            gridSize,
            (sampleX, sampleY, sampleZ) -> WorldThermalSampler.sampleAveragedAnomaly(world, sampleX, sampleY, sampleZ)
        );

        Iterator<Map.Entry<WindowKey, WindowState>> iterator = windows.entrySet().iterator();
        while (iterator.hasNext()) {
            Map.Entry<WindowKey, WindowState> entry = iterator.next();
            WindowKey key = entry.getKey();
            WindowState window = entry.getValue();
            float[] completedFlow = window.consumeCompletedFlow();
            if (completedFlow != null) {
                window.latestFlow = completedFlow;
                window.renderer.setMaxInflowSpeed(maxWindSpeed);
                window.renderer.setStreamlineSampleStride(streamlineSampleStride);
                window.renderer.setStreamlineRoi(
                    streamlineRoiEnabled,
                    streamlineRoiMinX,
                    streamlineRoiMaxX,
                    streamlineRoiMinY,
                    streamlineRoiMaxY,
                    streamlineRoiMinZ,
                    streamlineRoiMaxZ
                );
                window.renderer.setRenderVelocityVectors(renderVelocityVectors);
                window.renderer.setRenderStreamlines(renderStreamlines);
                window.renderer.updateFlowFieldNoCopy(key.origin(), 1, completedFlow);
            }

            if (window.busy.compareAndSet(false, true)) {
                BlockPos origin = key.origin();
                Identifier dimensionSnapshot = currentDimensionId;
                SolverBackend backendSnapshot = backendMode;
                float maxSpeedSnapshot = maxWindSpeed;
                long generationSnapshot = runtimeGeneration.get();
                activeSolveTasks.incrementAndGet();
                solverExecutor.execute(
                    () -> runSolveTask(window, origin, dimensionSnapshot, backendSnapshot, maxSpeedSnapshot, generationSnapshot)
                );
            }

            if (window.latestFlow != null) {
                window.physics.updateOrigin(key.origin());
                window.physics.applyForces(client, FORCE_STRENGTH);
            }
        }
    }

    private void runSolveTask(
        WindowState window,
        BlockPos origin,
        Identifier dimensionId,
        SolverBackend backend,
        float maxSpeedSnapshot,
        long generationSnapshot
    ) {
        try {
            if (generationSnapshot != runtimeGeneration.get()) {
                return;
            }
            byte[] payload = captureWindow(window, dimensionId, origin, backend);
            float[] response = runSolverStep(window, payload, backend);
            updateBaseFieldFromResponse(window, response, maxSpeedSnapshot);
            if (generationSnapshot == runtimeGeneration.get()) {
                window.publishCompletedFlow(response);
                lastSolverError = "";
            }
        } catch (IOException ex) {
            if (generationSnapshot == runtimeGeneration.get()) {
                lastSolverError = ex.getMessage();
            }
            log("Solver error for window " + formatPos(origin) + ": " + ex.getMessage());
            closeSocket(window);
        } finally {
            window.busy.set(false);
            if (activeSolveTasks.decrementAndGet() == 0) {
                cleanupRetiredWindowsIfIdle();
            }
        }
    }

    void render(WorldRenderContext context) {
        MinecraftClient client = MinecraftClient.getInstance();
        if (client.world == null) {
            return;
        }
        Identifier dimensionId = client.world.getRegistryKey().getValue();
        if (currentDimensionId == null || !currentDimensionId.equals(dimensionId)) {
            return;
        }

        for (Map.Entry<WindowKey, WindowState> entry : windows.entrySet()) {
            WindowKey key = entry.getKey();
            WindowState window = entry.getValue();
            window.renderer.setMaxInflowSpeed(maxWindSpeed);
            window.renderer.setStreamlineSampleStride(streamlineSampleStride);
            window.renderer.setStreamlineRoi(
                streamlineRoiEnabled,
                streamlineRoiMinX,
                streamlineRoiMaxX,
                streamlineRoiMinY,
                streamlineRoiMaxY,
                streamlineRoiMinZ,
                streamlineRoiMaxZ
            );
            window.renderer.setRenderVelocityVectors(renderVelocityVectors);
            window.renderer.setRenderStreamlines(renderStreamlines);
            window.renderer.render(context);
            renderAtmosphereDebugOverlay(context, key.origin());
        }
    }

    private void renderAtmosphereDebugOverlay(WorldRenderContext context, BlockPos windowOrigin) {
        if ((!renderBackgroundVectors && !renderThermalAnomaly)
            || context == null
            || context.gameRenderer() == null
            || context.consumers() == null
            || context.matrices() == null
            || windowOrigin == null
            || currentDimensionId == null) {
            return;
        }

        var camera = context.gameRenderer().getCamera();
        if (camera == null) {
            return;
        }

        MatrixStack matrices = context.matrices();
        Vec3d camPos = camera.getCameraPos();
        matrices.push();
        matrices.translate(
            windowOrigin.getX() - camPos.x,
            windowOrigin.getY() - camPos.y,
            windowOrigin.getZ() - camPos.z
        );

        VertexConsumer buffer = context.consumers().getBuffer(RenderLayers.lines());
        MatrixStack.Entry entry = matrices.peek();
        Matrix4f matrix = entry.getPositionMatrix();
        float[] sample = new float[4];
        int stride = Math.max(2, Math.min(ATMOS_DEBUG_STRIDE, gridSize));

        for (int x = 0; x < gridSize; x += stride) {
            float localX = x + 0.5f;
            for (int y = 0; y < gridSize; y += stride) {
                float localY = y + 0.5f;
                for (int z = 0; z < gridSize; z += stride) {
                    float localZ = z + 0.5f;
                    backgroundField.sampleInto(
                        currentDimensionId,
                        windowOrigin.getX() + localX,
                        windowOrigin.getY() + localY,
                        windowOrigin.getZ() + localZ,
                        sample,
                        0
                    );

                    if (renderBackgroundVectors) {
                        float vx = sample[0];
                        float vy = sample[1];
                        float vz = sample[2];
                        float speed = (float) Math.sqrt(vx * vx + vy * vy + vz * vz);
                        if (speed > ATMOS_VECTOR_MIN_SPEED) {
                            float speedNorm = MathHelper.clamp(speed / Math.max(maxWindSpeed, ATMOS_VECTOR_MIN_SPEED), 0.0f, 1.0f);
                            float invSpeed = 1.0f / speed;
                            float dx = vx * invSpeed;
                            float dy = vy * invSpeed;
                            float dz = vz * invSpeed;
                            float length = MathHelper.clamp(0.25f + (speedNorm * ATMOS_VECTOR_SCALE), 0.20f, 2.50f);
                            int r = MathHelper.clamp(Math.round(30.0f + speedNorm * 210.0f), 0, 255);
                            int g = MathHelper.clamp(Math.round(140.0f + speedNorm * 100.0f), 0, 255);
                            int b = MathHelper.clamp(Math.round(255.0f - speedNorm * 180.0f), 0, 255);
                            drawDebugLine(
                                buffer,
                                matrix,
                                entry,
                                localX,
                                localY,
                                localZ,
                                localX + dx * length,
                                localY + dy * length,
                                localZ + dz * length,
                                r,
                                g,
                                b,
                                210,
                                1.10f
                            );
                        }
                    }

                    if (renderThermalAnomaly) {
                        float thermal = sample[BACKGROUND_THERMAL_COMPONENT];
                        float thermalNorm = MathHelper.clamp(Math.abs(thermal) / ATMOS_THERMAL_RANGE, 0.0f, 1.0f);
                        if (thermalNorm >= ATMOS_THERMAL_MIN_MARKER) {
                            boolean warm = thermal >= 0.0f;
                            int r = warm ? 255 : MathHelper.clamp(Math.round(120.0f * (1.0f - thermalNorm)), 0, 255);
                            int g = MathHelper.clamp(Math.round(120.0f + 90.0f * (1.0f - thermalNorm)), 0, 255);
                            int b = warm ? MathHelper.clamp(Math.round(140.0f * (1.0f - thermalNorm)), 0, 255) : 255;
                            float markerLength = 0.15f + 0.45f * thermalNorm;
                            float markerEndY = localY + (warm ? markerLength : -markerLength);
                            drawDebugLine(
                                buffer,
                                matrix,
                                entry,
                                localX,
                                localY,
                                localZ,
                                localX,
                                markerEndY,
                                localZ,
                                r,
                                g,
                                b,
                                235,
                                1.35f
                            );
                        }
                    }
                }
            }
        }

        matrices.pop();
    }

    private static void drawDebugLine(
        VertexConsumer buffer,
        Matrix4f matrix,
        MatrixStack.Entry entry,
        float x0,
        float y0,
        float z0,
        float x1,
        float y1,
        float z1,
        int r,
        int g,
        int b,
        int a,
        float lineWidth
    ) {
        float dx = x1 - x0;
        float dy = y1 - y0;
        float dz = z1 - z0;
        float lengthSq = dx * dx + dy * dy + dz * dz;
        if (lengthSq <= 1.0e-8f) {
            return;
        }
        float invLength = (float) (1.0 / Math.sqrt(lengthSq));
        float nx = dx * invLength;
        float ny = dy * invLength;
        float nz = dz * invLength;

        buffer.vertex(matrix, x0, y0, z0).color(r, g, b, a).normal(entry, nx, ny, nz).lineWidth(lineWidth);
        buffer.vertex(matrix, x1, y1, z1).color(r, g, b, a).normal(entry, nx, ny, nz).lineWidth(lineWidth);
    }

    void clear() {
        runtimeGeneration.incrementAndGet();
        tickCounter = 0;
        currentDimensionId = null;
        lastSolverError = "";
        backgroundField.clear();
        retireAllWindows();
        cleanupRetiredWindowsIfIdle();
    }

    private void retireAllWindows() {
        if (windows.isEmpty()) {
            return;
        }
        synchronized (retiredWindows) {
            retiredWindows.addAll(windows.values());
        }
        windows.clear();
    }

    private void cleanupRetiredWindowsIfIdle() {
        if (activeSolveTasks.get() != 0) {
            return;
        }
        List<WindowState> toRelease;
        synchronized (retiredWindows) {
            if (retiredWindows.isEmpty()) {
                return;
            }
            toRelease = new ArrayList<>(retiredWindows);
            retiredWindows.clear();
        }
        for (WindowState window : toRelease) {
            releaseWindow(window);
        }
    }

    private void switchBackend(SolverBackend mode) {
        if (backendMode == mode) {
            return;
        }
        backendMode = mode;
    }

    private void refreshWindows(ClientWorld world) {
        Map<WindowKey, List<FanSource>> fansByWindow = scanFanSources(world);

        Iterator<Map.Entry<WindowKey, WindowState>> iterator = windows.entrySet().iterator();
        while (iterator.hasNext()) {
            Map.Entry<WindowKey, WindowState> entry = iterator.next();
            if (fansByWindow.containsKey(entry.getKey())) {
                continue;
            }
            releaseWindow(entry.getValue());
            iterator.remove();
        }

        for (Map.Entry<WindowKey, List<FanSource>> entry : fansByWindow.entrySet()) {
            WindowKey key = entry.getKey();
            WindowState window = windows.get(key);
            if (window == null) {
                window = new WindowState(nextContextId++, gridSize, maxWindSpeed);
                windows.put(key, window);
            }
            window.fans = List.copyOf(entry.getValue());
            refreshObstacleField(world, key.origin(), window, tickCounter);
        }
    }

    private Map<WindowKey, List<FanSource>> scanFanSources(ClientWorld world) {
        Map<WindowKey, List<FanSource>> fansByWindow = new HashMap<>();
        List<BlockPos> playerPositions = new ArrayList<>();
        for (PlayerEntity player : world.getPlayers()) {
            if (!player.isSpectator()) {
                playerPositions.add(player.getBlockPos());
            }
        }
        if (playerPositions.isEmpty()) {
            return fansByWindow;
        }

        Set<Long> seenFans = new HashSet<>();
        Set<Long> visitedChunks = new HashSet<>();
        int chunkRadius = Math.max(1, (FAN_SCAN_RADIUS + 15) / 16);
        for (BlockPos playerPos : playerPositions) {
            int playerChunkX = Math.floorDiv(playerPos.getX(), 16);
            int playerChunkZ = Math.floorDiv(playerPos.getZ(), 16);
            for (int chunkX = playerChunkX - chunkRadius; chunkX <= playerChunkX + chunkRadius; chunkX++) {
                for (int chunkZ = playerChunkZ - chunkRadius; chunkZ <= playerChunkZ + chunkRadius; chunkZ++) {
                    long chunkKey = ChunkPos.toLong(chunkX, chunkZ);
                    if (!visitedChunks.add(chunkKey)) {
                        continue;
                    }
                    WorldChunk chunk = world.getChunkManager().getWorldChunk(chunkX, chunkZ);
                    if (chunk == null) {
                        continue;
                    }
                    for (BlockEntity blockEntity : chunk.getBlockEntities().values()) {
                        BlockPos pos = blockEntity.getPos();
                        if (!isWithinAnyPlayerRadius(pos, playerPositions, FAN_SCAN_RADIUS)) {
                            continue;
                        }
                        if (!seenFans.add(pos.asLong())) {
                            continue;
                        }

                        BlockState state = world.getBlockState(pos);
                        if (!state.isOf(ModBlocks.FAN_BLOCK)) {
                            continue;
                        }

                        Direction facing = state.get(FanBlock.FACING);
                        int ductLength = computeDuctLength(world, pos, facing);
                        BlockPos origin = alignToGrid(pos);
                        WindowKey key = new WindowKey(origin);
                        fansByWindow.computeIfAbsent(key, ignored -> new ArrayList<>()).add(new FanSource(pos, facing, ductLength));
                    }
                }
            }
        }
        return fansByWindow;
    }

    private boolean isWithinAnyPlayerRadius(BlockPos pos, List<BlockPos> playerPositions, int radius) {
        int radiusSq = radius * radius;
        for (BlockPos playerPos : playerPositions) {
            int dx = pos.getX() - playerPos.getX();
            int dy = pos.getY() - playerPos.getY();
            int dz = pos.getZ() - playerPos.getZ();
            if (dx * dx + dy * dy + dz * dz <= radiusSq) {
                return true;
            }
        }
        return false;
    }

    private BlockPos alignToGrid(BlockPos pos) {
        int x = Math.floorDiv(pos.getX(), gridSize) * gridSize;
        int y = Math.floorDiv(pos.getY(), gridSize) * gridSize;
        int z = Math.floorDiv(pos.getZ(), gridSize) * gridSize;
        return new BlockPos(x, y, z);
    }

    private int voxelIndex(int x, int y, int z) {
        return ((x * gridSize + y) * gridSize + z) * CHANNELS;
    }

    private boolean inBounds(int x, int y, int z) {
        return x >= 0 && y >= 0 && z >= 0 && x < gridSize && y < gridSize && z < gridSize;
    }

    private void applyFanAtVoxel(WindowState window, int x, int y, int z, float fanVx, float fanVy, float fanVz) {
        if (!inBounds(x, y, z)) {
            return;
        }
        int idx = voxelIndex(x, y, z);
        if (window.baseField[idx + CH_OBSTACLE] > 0.5f) {
            return;
        }
        window.baseField[idx + CH_FAN_MASK] = 1.0f;
        window.baseField[idx + CH_FAN_VX] += fanVx;
        window.baseField[idx + CH_FAN_VY] += fanVy;
        window.baseField[idx + CH_FAN_VZ] += fanVz;
    }

    private void applyFanSource(WindowState window, FanSource fan, int minX, int minY, int minZ) {
        BlockPos inflowPos = fan.pos().offset(fan.facing());
        int cx = inflowPos.getX() - minX;
        int cy = inflowPos.getY() - minY;
        int cz = inflowPos.getZ() - minZ;

        float fanVx = fan.facing().getOffsetX() * INFLOW_SPEED;
        float fanVy = fan.facing().getOffsetY() * INFLOW_SPEED;
        float fanVz = fan.facing().getOffsetZ() * INFLOW_SPEED;

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

        float baseVx = dx * INFLOW_SPEED;
        float baseVy = dy * INFLOW_SPEED;
        float baseVz = dz * INFLOW_SPEED;
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

    private int computeDuctLength(ClientWorld world, BlockPos fanPos, Direction facing) {
        int runLength = 0;
        int maxRunLength = 0;
        int consecutiveGaps = 0;
        BlockPos.Mutable cursor = new BlockPos.Mutable();

        for (int step = 1; step <= DUCT_SCAN_MAX; step++) {
            cursor.set(
                fanPos.getX() + facing.getOffsetX() * step,
                fanPos.getY() + facing.getOffsetY() * step,
                fanPos.getZ() + facing.getOffsetZ() * step
            );
            if (isDuctSegment(world, cursor, facing)) {
                runLength++;
                maxRunLength = Math.max(maxRunLength, runLength);
                consecutiveGaps = 0;
            } else {
                consecutiveGaps++;
                if (consecutiveGaps > DUCT_MAX_CONSECUTIVE_GAPS) {
                    break;
                }
            }
        }
        return maxRunLength;
    }

    private boolean isDuctSegment(ClientWorld world, BlockPos center, Direction facing) {
        if (isSolidObstacle(world, center)) {
            return false;
        }

        int ringCells = 0;
        int filledRingCells = 0;
        BlockPos.Mutable cursor = new BlockPos.Mutable();
        Direction.Axis axis = facing.getAxis();
        for (int a = -DUCT_RING_RADIUS; a <= DUCT_RING_RADIUS; a++) {
            for (int b = -DUCT_RING_RADIUS; b <= DUCT_RING_RADIUS; b++) {
                if (!isDuctRingCell(a, b)) {
                    continue;
                }
                ringCells++;
                switch (axis) {
                    case X -> cursor.set(center.getX(), center.getY() + a, center.getZ() + b);
                    case Y -> cursor.set(center.getX() + a, center.getY(), center.getZ() + b);
                    case Z -> cursor.set(center.getX() + a, center.getY() + b, center.getZ());
                }
                if (world.getBlockState(cursor).isOf(ModBlocks.DUCT_BLOCK)) {
                    filledRingCells++;
                }
            }
        }

        if (ringCells == 0) {
            return false;
        }
        return ((float) filledRingCells / ringCells) >= DUCT_RING_FILL_THRESHOLD;
    }

    private boolean isDuctRingCell(int a, int b) {
        return Math.max(Math.abs(a), Math.abs(b)) == DUCT_RING_RADIUS;
    }

    private byte[] captureWindow(WindowState window, Identifier dimensionId, BlockPos origin, SolverBackend backend) {
        window.ensureBaseFieldInitialized(gridSize);
        window.ensurePayloadBuffer(gridSize);
        float[] obstacleField = window.obstacleField;
        int minX = origin.getX();
        int minY = origin.getY();
        int minZ = origin.getZ();
        ByteBuffer buffer = window.payloadBuffer;
        buffer.clear();
        float[] backgroundSample = new float[4];

        for (int x = 0; x < gridSize; x++) {
            for (int y = 0; y < gridSize; y++) {
                for (int z = 0; z < gridSize; z++) {
                    int cell = (x * gridSize + y) * gridSize + z;
                    boolean solid = obstacleField != null && cell < obstacleField.length && obstacleField[cell] > 0.5f;
                    int idx = voxelIndex(x, y, z);

                    window.baseField[idx + CH_OBSTACLE] = solid ? 1.0f : 0.0f;
                    window.baseField[idx + CH_FAN_MASK] = 0.0f;
                    window.baseField[idx + CH_FAN_VX] = 0.0f;
                    window.baseField[idx + CH_FAN_VY] = 0.0f;
                    window.baseField[idx + CH_FAN_VZ] = 0.0f;
                    if (solid) {
                        window.baseField[idx + CH_STATE_VX] = 0.0f;
                        window.baseField[idx + CH_STATE_VY] = 0.0f;
                        window.baseField[idx + CH_STATE_VZ] = 0.0f;
                        window.baseField[idx + CH_STATE_P] = 0.0f;
                    } else {
                        backgroundField.sampleInto(
                            dimensionId,
                            minX + x + 0.5,
                            minY + y + 0.5,
                            minZ + z + 0.5,
                            backgroundSample,
                            0
                        );
                        float vx = window.baseField[idx + CH_STATE_VX];
                        float vy = window.baseField[idx + CH_STATE_VY];
                        float vz = window.baseField[idx + CH_STATE_VZ];
                        window.baseField[idx + CH_STATE_VX] = vx + (backgroundSample[0] - vx) * BACKGROUND_VELOCITY_COUPLING;
                        window.baseField[idx + CH_STATE_VY] = vy + (backgroundSample[1] - vy) * BACKGROUND_VELOCITY_COUPLING;
                        window.baseField[idx + CH_STATE_VZ] = vz + (backgroundSample[2] - vz) * BACKGROUND_VELOCITY_COUPLING;
                    }
                }
            }
        }

        for (FanSource fan : window.fans) {
            applyFanSource(window, fan, minX, minY, minZ);
        }

        float nativeInputScale = backend == SolverBackend.NATIVE ? (1.0f / NATIVE_VELOCITY_SCALE) : 1.0f;
        int cellCount = gridSize * gridSize * gridSize;
        for (int i = 0; i < cellCount; i++) {
            int idx = i * CHANNELS;
            buffer.putFloat(window.baseField[idx + CH_OBSTACLE]);
            buffer.putFloat(window.baseField[idx + CH_FAN_MASK]);
            buffer.putFloat(window.baseField[idx + CH_FAN_VX]);
            buffer.putFloat(window.baseField[idx + CH_FAN_VY]);
            buffer.putFloat(window.baseField[idx + CH_FAN_VZ]);
            buffer.putFloat(window.baseField[idx + CH_STATE_VX] * nativeInputScale);
            buffer.putFloat(window.baseField[idx + CH_STATE_VY] * nativeInputScale);
            buffer.putFloat(window.baseField[idx + CH_STATE_VZ] * nativeInputScale);
            buffer.putFloat(window.baseField[idx + CH_STATE_P]);
        }

        return window.payloadBytes;
    }

    private boolean isSolidObstacle(ClientWorld world, BlockPos pos) {
        BlockState state = world.getBlockState(pos);
        if (state.isAir() || state.isOf(ModBlocks.DUCT_BLOCK)) {
            return false;
        }
        return !state.getCollisionShape(world, pos).isEmpty();
    }

    private void refreshObstacleField(ClientWorld world, BlockPos origin, WindowState window, int tickNow) {
        if (window.obstacleField != null
            && window.lastObstacleRefreshTick >= 0
            && (tickNow - window.lastObstacleRefreshTick) < OBSTACLE_REFRESH_TICKS) {
            return;
        }

        int cellCount = gridSize * gridSize * gridSize;
        float[] obstacle = new float[cellCount];
        int minX = origin.getX();
        int minY = origin.getY();
        int minZ = origin.getZ();
        BlockPos.Mutable cursor = new BlockPos.Mutable();

        for (int x = 0; x < gridSize; x++) {
            for (int y = 0; y < gridSize; y++) {
                for (int z = 0; z < gridSize; z++) {
                    int cell = (x * gridSize + y) * gridSize + z;
                    cursor.set(minX + x, minY + y, minZ + z);
                    obstacle[cell] = isSolidObstacle(world, cursor) ? 1.0f : 0.0f;
                }
            }
        }

        window.obstacleField = obstacle;
        window.lastObstacleRefreshTick = tickNow;
    }

    private boolean stepBackgroundFlow(int gridSize, byte[] payload, long contextId, float[] output) {
        if (!nativeBackend.isLoaded()) {
            return false;
        }
        if (!nativeBackend.ensureInitialized(gridSize, BackgroundFieldGrid.SOLVER_INPUT_CHANNELS, BackgroundFieldGrid.SOLVER_OUTPUT_CHANNELS)) {
            return false;
        }
        boolean ok = nativeBackend.step(payload, gridSize, BackgroundFieldGrid.SOLVER_OUTPUT_CHANNELS, contextId, output);
        if (!ok) {
            return false;
        }
        scaleResponseVelocity(output, NATIVE_VELOCITY_SCALE);
        return true;
    }

    private void releaseBackgroundFlowContext(long contextId) {
        if (!nativeBackend.isLoaded()) {
            return;
        }
        nativeBackend.releaseContext(contextId);
    }

    private float[] runSolverStep(WindowState window, byte[] payload, SolverBackend backend) throws IOException {
        float[] solverOutput = window.acquireSolveOutputBuffer(flowCount);
        if (backend == SolverBackend.NATIVE) {
            closeSocket(window);
            if (!nativeBackend.isLoaded()) {
                throw new IOException("Native backend not loaded: " + nativeBackend.getLoadError());
            }
            if (!nativeBackend.ensureInitialized(gridSize, CHANNELS, RESPONSE_CHANNELS)) {
                throw new IOException("Native backend initialization failed");
            }
            boolean ok = nativeBackend.step(payload, gridSize, RESPONSE_CHANNELS, window.nativeContextId, solverOutput);
            if (!ok) {
                throw new IOException("Native backend returned invalid response");
            }
            scaleResponseVelocity(solverOutput, NATIVE_VELOCITY_SCALE);
            return solverOutput;
        }

        nativeBackend.releaseContext(window.nativeContextId);
        ensureSocket(window);
        sendPayload(window, payload);
        receiveFlowField(window, solverOutput);
        return solverOutput;
    }

    private void updateBaseFieldFromResponse(WindowState window, float[] response, float speedCap) {
        if (window.baseField == null) {
            return;
        }

        int cellCount = gridSize * gridSize * gridSize;
        float capped = Math.max(0.0f, speedCap);
        for (int i = 0; i < cellCount; i++) {
            int baseIdx = i * CHANNELS;
            int respIdx = i * RESPONSE_CHANNELS;
            float vx = response[respIdx];
            float vy = response[respIdx + 1];
            float vz = response[respIdx + 2];

            if (capped > 0.0f) {
                float speedSq = vx * vx + vy * vy + vz * vz;
                float capSq = capped * capped;
                if (speedSq > capSq) {
                    float scale = capped / (float) Math.sqrt(speedSq);
                    vx *= scale;
                    vy *= scale;
                    vz *= scale;
                }
            }

            response[respIdx] = vx;
            response[respIdx + 1] = vy;
            response[respIdx + 2] = vz;

            window.baseField[baseIdx + CH_STATE_VX] = vx;
            window.baseField[baseIdx + CH_STATE_VY] = vy;
            window.baseField[baseIdx + CH_STATE_VZ] = vz;
            window.baseField[baseIdx + CH_STATE_P] = response[respIdx + 3];
        }
    }

    private void scaleResponseVelocity(float[] response, float velocityScale) {
        if (velocityScale == 1.0f) {
            return;
        }

        int cellCount = gridSize * gridSize * gridSize;
        for (int i = 0; i < cellCount; i++) {
            int respIdx = i * RESPONSE_CHANNELS;
            response[respIdx] *= velocityScale;
            response[respIdx + 1] *= velocityScale;
            response[respIdx + 2] *= velocityScale;
        }
    }

    private int sanitizeStride(int requested) {
        return (requested == 1 || requested == 2 || requested == 4 || requested == 8) ? requested : DEFAULT_STREAMLINE_STRIDE;
    }

    private int clampRoiCoord(int value) {
        return Math.max(0, Math.min(gridSize - 1, value));
    }

    private void sendPayload(WindowState window, byte[] payload) throws IOException {
        if (window.outputStream == null) {
            throw new IOException("Socket not ready");
        }
        window.outputStream.writeInt(payload.length);
        window.outputStream.write(payload);
        window.outputStream.flush();
    }

    private void receiveFlowField(WindowState window, float[] output) throws IOException {
        if (window.inputStream == null) {
            throw new IOException("Socket not ready");
        }
        int length = window.inputStream.readInt();
        int expectedBytes = flowCount * Float.BYTES;
        if (length != expectedBytes) {
            throw new IOException("Unexpected response length: " + length);
        }
        if (window.socketReadBuffer == null || window.socketReadBuffer.length != expectedBytes) {
            window.socketReadBuffer = new byte[expectedBytes];
        }
        window.inputStream.readFully(window.socketReadBuffer, 0, expectedBytes);
        ByteBuffer.wrap(window.socketReadBuffer).order(ByteOrder.LITTLE_ENDIAN).asFloatBuffer().get(output);
    }

    private void ensureSocket(WindowState window) throws IOException {
        if (window.socket != null && window.socket.isConnected() && !window.socket.isClosed()) {
            return;
        }
        window.socket = new Socket();
        window.socket.setTcpNoDelay(true);
        window.socket.connect(new InetSocketAddress(SOCKET_HOST, SOCKET_PORT), 2000);
        window.inputStream = new DataInputStream(new BufferedInputStream(window.socket.getInputStream()));
        window.outputStream = new DataOutputStream(new BufferedOutputStream(window.socket.getOutputStream()));
    }

    private void closeSocket(WindowState window) {
        try {
            if (window.outputStream != null) {
                window.outputStream.close();
            }
        } catch (IOException ignored) {
        }
        try {
            if (window.inputStream != null) {
                window.inputStream.close();
            }
        } catch (IOException ignored) {
        }
        try {
            if (window.socket != null) {
                window.socket.close();
            }
        } catch (IOException ignored) {
        }
        window.inputStream = null;
        window.outputStream = null;
        window.socket = null;
    }

    private void releaseWindow(WindowState window) {
        closeSocket(window);
        nativeBackend.releaseContext(window.nativeContextId);
    }

    private void log(String message) {
        System.out.println(LOG_PREFIX + message);
    }

    private String formatPos(BlockPos pos) {
        return "(" + pos.getX() + ", " + pos.getY() + ", " + pos.getZ() + ")";
    }

    private enum SolverBackend {
        SOCKET,
        NATIVE;

        private static SolverBackend fromId(int backendModeId) {
            return backendModeId == 0 ? SOCKET : NATIVE;
        }
    }

    private record WindowKey(BlockPos origin) {
    }

    private record FanSource(BlockPos pos, Direction facing, int ductLength) {
    }

    private static final class WindowState {
        private final long nativeContextId;
        private final FlowRenderer renderer;
        private final PhysicsHandler physics;
        private final AtomicBoolean busy = new AtomicBoolean(false);
        private Socket socket;
        private DataInputStream inputStream;
        private DataOutputStream outputStream;
        private byte[] payloadBytes;
        private ByteBuffer payloadBuffer;
        private float[] solverOutputA;
        private float[] solverOutputB;
        private boolean solveIntoA = true;
        private float[] completedFlow;
        private byte[] socketReadBuffer;
        private float[] baseField;
        private float[] obstacleField;
        private int lastObstacleRefreshTick = -1;
        private List<FanSource> fans = List.of();
        private float[] latestFlow;

        private WindowState(long nativeContextId, int gridSize, float maxWindSpeed) {
            this.nativeContextId = nativeContextId;
            this.renderer = new FlowRenderer(gridSize, maxWindSpeed);
            this.physics = new PhysicsHandler(renderer, gridSize);
        }

        private void ensureBaseFieldInitialized(int gridSize) {
            if (baseField != null) {
                return;
            }
            baseField = new float[gridSize * gridSize * gridSize * CHANNELS];
        }

        private void ensurePayloadBuffer(int gridSize) {
            int payloadBytesLength = gridSize * gridSize * gridSize * CHANNELS * Float.BYTES;
            if (payloadBytes != null && payloadBytes.length == payloadBytesLength && payloadBuffer != null) {
                return;
            }
            payloadBytes = new byte[payloadBytesLength];
            payloadBuffer = ByteBuffer.wrap(payloadBytes).order(ByteOrder.LITTLE_ENDIAN);
        }

        private synchronized float[] acquireSolveOutputBuffer(int flowCount) {
            if (solverOutputA == null || solverOutputA.length != flowCount) {
                solverOutputA = new float[flowCount];
            }
            if (solverOutputB == null || solverOutputB.length != flowCount) {
                solverOutputB = new float[flowCount];
            }
            float[] target = solveIntoA ? solverOutputA : solverOutputB;
            solveIntoA = !solveIntoA;
            return target;
        }

        private synchronized void publishCompletedFlow(float[] flow) {
            completedFlow = flow;
        }

        private synchronized float[] consumeCompletedFlow() {
            float[] flow = completedFlow;
            completedFlow = null;
            return flow;
        }
    }
}
