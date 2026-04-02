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

import com.aerodynamics4mc.FanBlock;
import com.aerodynamics4mc.ModBlocks;

import net.fabricmc.fabric.api.client.rendering.v1.world.WorldRenderContext;
import net.minecraft.block.BlockState;
import net.minecraft.block.Blocks;
import net.minecraft.block.entity.BlockEntity;
import net.minecraft.client.MinecraftClient;
import net.minecraft.client.world.ClientWorld;
import net.minecraft.entity.player.PlayerEntity;
import net.minecraft.registry.tag.FluidTags;
import net.minecraft.state.property.Properties;
import net.minecraft.util.Identifier;
import net.minecraft.util.math.BlockPos;
import net.minecraft.util.math.ChunkPos;
import net.minecraft.util.math.Direction;
import net.minecraft.util.math.MathHelper;
import net.minecraft.util.math.Vec3d;
import net.minecraft.world.chunk.WorldChunk;

final class ClientFluidRuntime {
    private static final String LOG_PREFIX = "[aerodynamics4mc/client] ";

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
    private static final int WINDOW_REFRESH_TICKS = 100;
    private static final int WINDOW_SAMPLE_RESYNC_TICKS = 200;
    private static final int FAN_SCAN_RADIUS = 48;
    private static final int CHUNK_SIZE = 16;
    private static final int SOLVER_WORKER_COUNT = Math.max(2, Runtime.getRuntime().availableProcessors() / 2);
    private static final int REGION_HALO_RATIO = 4;

    private static final int DEFAULT_STREAMLINE_STRIDE = 4;
    private static final float NATIVE_VELOCITY_SCALE = 30.0f;
    private static final float NATIVE_THERMAL_SOURCE_MAX = 0.006f;
    private static final float STATE_PRESSURE_MIN = -0.03f;
    private static final float STATE_PRESSURE_MAX = 0.03f;
    private static final float THERMAL_SOURCE_LAVA = NATIVE_THERMAL_SOURCE_MAX;
    private static final float THERMAL_SOURCE_MAGMA = 0.0046f;
    private static final float THERMAL_SOURCE_CAMPFIRE = 0.0044f;
    private static final float THERMAL_SOURCE_SOUL_CAMPFIRE = 0.0036f;
    private static final float THERMAL_SOURCE_FIRE = 0.0048f;
    private static final float THERMAL_SOURCE_SOUL_FIRE = 0.0038f;
    private static final float THERMAL_SOURCE_TORCH = 0.0024f;
    private static final float THERMAL_SOURCE_SOUL_TORCH = 0.0018f;
    private static final float THERMAL_SOURCE_LANTERN = 0.0016f;
    private static final float THERMAL_SOURCE_SOUL_LANTERN = 0.0012f;
    private static final float THERMAL_SOURCE_SOLID_EMISSION_SCALE = 0.85f;
    private static final int THERMAL_SOURCE_SOLID_NEIGHBOR_COUNT = 3;
    private static final float THERMAL_SOURCE_WATER_COOLING = -0.0012f;
    private static final float THERMAL_SOURCE_STONE_HEATING = 0.00045f;
    private static final float THERMAL_SOURCE_BARE_HEATING = 0.00075f;
    private static final float THERMAL_SOURCE_ALTITUDE_LAPSE_PER_BLOCK = 0.000020f;
    private static final float THERMAL_SOURCE_ALTITUDE_LAPSE_MAX = 0.0020f;
    private static final int[][] THERMAL_NEIGHBOR_OFFSETS = {
        { 1, 0, 0 },
        { -1, 0, 0 },
        { 0, 1, 0 },
        { 0, -1, 0 },
        { 0, 0, 1 },
        { 0, 0, -1 }
    };
    private static final double FORCE_STRENGTH = 0.02;

    private final int gridSize;
    private final int flowCount;
    private final Map<WindowKey, WindowState> windows = new HashMap<>();
    private final List<WindowState> retiredWindows = new ArrayList<>();
    private final NativeLbmBridge nativeBackend = new NativeLbmBridge();
    private final ExecutorService solverExecutor = Executors.newFixedThreadPool(SOLVER_WORKER_COUNT);
    private final AtomicInteger activeSolveTasks = new AtomicInteger(0);
    private final AtomicLong runtimeGeneration = new AtomicLong(0L);

    private SolverBackend backendMode = SolverBackend.NATIVE;
    private float maxWindSpeed = INFLOW_SPEED;
    private int streamlineSampleStride = DEFAULT_STREAMLINE_STRIDE;
    private boolean renderVelocityVectors = true;
    private boolean renderStreamlines = true;
    private int tickCounter = 0;
    private long nextContextId = 1L;
    private Identifier currentDimensionId;
    private String lastSolverError = "";

    ClientFluidRuntime(int gridSize) {
        this.gridSize = gridSize;
        this.flowCount = gridSize * gridSize * gridSize * RESPONSE_CHANNELS;
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

    void setRenderVelocityVectors(boolean enabled) {
        this.renderVelocityVectors = enabled;
    }

    void setRenderStreamlines(boolean enabled) {
        this.renderStreamlines = enabled;
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
        }

        tickCounter++;
        boolean windowOriginShifted = hasWindowOriginShifted(world);
        if (windows.isEmpty() || tickCounter % WINDOW_REFRESH_TICKS == 0 || windowOriginShifted) {
            refreshWindows(world, tickCounter == 1);
        }
        if (windows.isEmpty()) {
            return;
        }

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
                window.renderer.setRenderVelocityVectors(renderVelocityVectors);
                window.renderer.setRenderStreamlines(renderStreamlines);
                window.renderer.updateFlowFieldNoCopy(key.origin(), 1, completedFlow);
            }

            if (!window.busy.get() && shouldResampleWindowSamples(window)) {
                boolean geometryChanged = refreshSampleFields(world, key.origin(), window, false);
                if (geometryChanged) {
                    invalidateBackendState(window, backendMode);
                }
            }

            if (window.busy.compareAndSet(false, true)) {
                BlockPos origin = key.origin();
                SolverBackend backendSnapshot = backendMode;
                float maxSpeedSnapshot = maxWindSpeed;
                long generationSnapshot = runtimeGeneration.get();
                activeSolveTasks.incrementAndGet();
                solverExecutor.execute(() -> runSolveTask(window, origin, backendSnapshot, maxSpeedSnapshot, generationSnapshot));
            }

            if (window.latestFlow != null) {
                window.physics.updateOrigin(key.origin());
                window.physics.applyForces(client, FORCE_STRENGTH);
            }
        }

        WindowState feedbackWindow = findWindowContaining(client.player.getBoundingBox().getCenter());
        if (feedbackWindow != null && feedbackWindow.latestFlow != null) {
            feedbackWindow.physics.tickVisualFeedback(client, tickCounter);
        }
    }

    private void runSolveTask(WindowState window, BlockPos origin, SolverBackend backend, float maxSpeedSnapshot, long generationSnapshot) {
        try {
            if (generationSnapshot != runtimeGeneration.get()) {
                return;
            }
            byte[] payload = captureWindow(window, origin, backend);
            float[] response = runSolverStep(window, payload, backend);
            int invalidComponents = updateBaseFieldFromResponse(window, response, maxSpeedSnapshot);
            if (invalidComponents > 0) {
                invalidateBackendState(window, backend);
                String message = "Detected non-finite solver response for window "
                    + formatPos(origin)
                    + " (" + invalidComponents + " components sanitized)";
                log(message + "; backend state reset");
                if (generationSnapshot == runtimeGeneration.get()) {
                    lastSolverError = message;
                }
            } else if (generationSnapshot == runtimeGeneration.get()) {
                lastSolverError = "";
            }
            if (generationSnapshot == runtimeGeneration.get()) {
                window.publishCompletedFlow(response);
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

        Vec3d center = client.player != null ? client.player.getBoundingBox().getCenter() : client.gameRenderer.getCamera().getCameraPos();
        WindowState window = findWindowContaining(center);
        if (window != null) {
            window.renderer.setMaxInflowSpeed(maxWindSpeed);
            window.renderer.setStreamlineSampleStride(streamlineSampleStride);
            window.renderer.setRenderVelocityVectors(renderVelocityVectors);
            window.renderer.setRenderStreamlines(renderStreamlines);
            window.renderer.render(context);
        }
    }

    void clear() {
        runtimeGeneration.incrementAndGet();
        tickCounter = 0;
        currentDimensionId = null;
        lastSolverError = "";
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

    private void refreshWindows(ClientWorld world, boolean forceResample) {
        Map<WindowKey, List<FanSource>> fansByWindow = scanFanSources(world);
        Map<WindowKey, WindowState> nextWindows = new HashMap<>();
        Set<WindowKey> consumed = new HashSet<>();

        for (Map.Entry<WindowKey, List<FanSource>> entry : fansByWindow.entrySet()) {
            WindowKey key = entry.getKey();
            WindowState window = windows.get(key);
            boolean needFullResample = forceResample;
            if (window != null) {
                consumed.add(key);
            } else {
                window = new WindowState(nextContextId++, gridSize, maxWindSpeed);
                needFullResample = true;
            }

            window.fans = List.copyOf(entry.getValue());
            if (needFullResample || window.obstacleField == null || window.sourceField == null || window.thermalField == null) {
                refreshSampleFields(world, key.origin(), window, true);
            }
            nextWindows.put(key, window);
        }

        for (Map.Entry<WindowKey, WindowState> entry : windows.entrySet()) {
            if (!consumed.contains(entry.getKey())) {
                deactivateWindow(entry.getValue());
                releaseWindow(entry.getValue());
            }
        }
        windows.clear();
        windows.putAll(nextWindows);
    }

    private void deactivateWindow(WindowState window) {
        window.latestFlow = null;
        window.busy.set(false);
        window.consumeCompletedFlow();
        window.renderer.clearFlowData();
    }

    private boolean hasWindowOriginShifted(ClientWorld world) {
        Set<WindowKey> expected = new HashSet<>();
        for (PlayerEntity player : world.getPlayers()) {
            if (!player.isSpectator()) {
                expected.add(new WindowKey(centerWindowOnPlayer(player.getBlockPos())));
            }
        }
        return !expected.equals(windows.keySet());
    }

    private WindowState findWindowContaining(Vec3d worldPos) {
        for (Map.Entry<WindowKey, WindowState> entry : windows.entrySet()) {
            if (isInsideCore(worldPos, entry.getKey().origin())) {
                return entry.getValue();
            }
        }
        return null;
    }

    private boolean isInsideCore(Vec3d pos, BlockPos origin) {
        int halo = gridSize / REGION_HALO_RATIO;
        int core = gridSize - halo * 2;
        return pos.x >= origin.getX() + halo && pos.x < origin.getX() + halo + core
            && pos.y >= origin.getY() + halo && pos.y < origin.getY() + halo + core
            && pos.z >= origin.getZ() + halo && pos.z < origin.getZ() + halo + core;
    }

    private boolean shouldResampleWindowSamples(WindowState window) {
        return tickCounter - window.lastSampleRefreshTick >= WINDOW_SAMPLE_RESYNC_TICKS;
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

        for (BlockPos playerPos : playerPositions) {
            BlockPos origin = centerWindowOnPlayer(playerPos);
            WindowKey key = new WindowKey(origin);
            fansByWindow.computeIfAbsent(key, ignored -> new ArrayList<>());
        }

        Set<Long> seenFans = new HashSet<>();
        Set<Long> visitedChunks = new HashSet<>();
        int chunkRadius = Math.max(1, (FAN_SCAN_RADIUS + CHUNK_SIZE - 1) / CHUNK_SIZE);
        for (BlockPos playerPos : playerPositions) {
            int playerChunkX = Math.floorDiv(playerPos.getX(), CHUNK_SIZE);
            int playerChunkZ = Math.floorDiv(playerPos.getZ(), CHUNK_SIZE);
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
                        FanSource fan = new FanSource(pos, facing, ductLength);
                        addFanToContainingWindows(fansByWindow, pos, fan);
                    }
                }
            }
        }
        return fansByWindow;
    }

    private void addFanToContainingWindows(
        Map<WindowKey, List<FanSource>> fansByWindow,
        BlockPos fanPos,
        FanSource fan
    ) {
        int margin = DUCT_JET_RANGE + FAN_RADIUS + 1;
        for (Map.Entry<WindowKey, List<FanSource>> entry : fansByWindow.entrySet()) {
            WindowKey key = entry.getKey();
            if (isInsideWindow(fanPos, key.origin(), margin)) {
                entry.getValue().add(fan);
            }
        }
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

    private boolean isInsideWindow(BlockPos pos, BlockPos origin, int margin) {
        int minX = origin.getX() - margin;
        int minY = origin.getY() - margin;
        int minZ = origin.getZ() - margin;
        int maxX = origin.getX() + gridSize + margin;
        int maxY = origin.getY() + gridSize + margin;
        int maxZ = origin.getZ() + gridSize + margin;
        return pos.getX() >= minX && pos.getX() < maxX
            && pos.getY() >= minY && pos.getY() < maxY
            && pos.getZ() >= minZ && pos.getZ() < maxZ;
    }

    private boolean isInsideWindow(Vec3d pos, BlockPos origin, int margin) {
        int minX = origin.getX() - margin;
        int minY = origin.getY() - margin;
        int minZ = origin.getZ() - margin;
        int maxX = origin.getX() + gridSize + margin;
        int maxY = origin.getY() + gridSize + margin;
        int maxZ = origin.getZ() + gridSize + margin;
        return pos.x >= minX && pos.x < maxX
            && pos.y >= minY && pos.y < maxY
            && pos.z >= minZ && pos.z < maxZ;
    }

    private BlockPos centerWindowOnPlayer(BlockPos pos) {
        int half = gridSize / 2;
        int x = Math.floorDiv(pos.getX() - half, CHUNK_SIZE) * CHUNK_SIZE;
        int y = Math.floorDiv(pos.getY() - half, CHUNK_SIZE) * CHUNK_SIZE;
        int z = Math.floorDiv(pos.getZ() - half, CHUNK_SIZE) * CHUNK_SIZE;
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

    private byte[] captureWindow(WindowState window, BlockPos origin, SolverBackend backend) {
        window.ensureBaseFieldInitialized(gridSize);
        float[] obstacleField = window.obstacleField;
        float[] thermalField = window.thermalField;
        int minX = origin.getX();
        int minY = origin.getY();
        int minZ = origin.getZ();
        int payloadBytesLength = gridSize * gridSize * gridSize * CHANNELS * Float.BYTES;
        ByteBuffer buffer;
        if (backend == SolverBackend.NATIVE) {
            window.ensureNativePayloadBuffer(payloadBytesLength);
            buffer = window.nativePayloadBuffer;
            buffer.clear();
        } else {
            window.ensurePayloadBuffer(gridSize);
            buffer = window.payloadBuffer;
            buffer.clear();
        }

        for (int x = 0; x < gridSize; x++) {
            for (int y = 0; y < gridSize; y++) {
                for (int z = 0; z < gridSize; z++) {
                    int cell = (x * gridSize + y) * gridSize + z;
                    boolean solid = obstacleField != null && cell < obstacleField.length && obstacleField[cell] > 0.5f;
                    int idx = voxelIndex(x, y, z);
                    float sampledThermal = 0.0f;
                    if (thermalField != null && cell < thermalField.length) {
                        sampledThermal += thermalField[cell];
                    }
                    sampledThermal = Math.max(-NATIVE_THERMAL_SOURCE_MAX, Math.min(sampledThermal, NATIVE_THERMAL_SOURCE_MAX));

                    window.baseField[idx + CH_OBSTACLE] = solid ? 1.0f : 0.0f;
                    window.baseField[idx + CH_FAN_MASK] = 0.0f;
                    window.baseField[idx + CH_FAN_VX] = 0.0f;
                    window.baseField[idx + CH_FAN_VY] = 0.0f;
                    window.baseField[idx + CH_FAN_VZ] = 0.0f;
                    window.baseField[idx + CH_THERMAL_SOURCE] = solid ? 0.0f : sampledThermal;
                    if (solid) {
                        window.baseField[idx + CH_STATE_VX] = 0.0f;
                        window.baseField[idx + CH_STATE_VY] = 0.0f;
                        window.baseField[idx + CH_STATE_VZ] = 0.0f;
                        window.baseField[idx + CH_STATE_P] = 0.0f;
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
            float stateVx = window.baseField[idx + CH_STATE_VX];
            float stateVy = window.baseField[idx + CH_STATE_VY];
            float stateVz = window.baseField[idx + CH_STATE_VZ];
            float stateP = window.baseField[idx + CH_STATE_P];
            if (!Float.isFinite(stateVx)) {
                stateVx = 0.0f;
            }
            if (!Float.isFinite(stateVy)) {
                stateVy = 0.0f;
            }
            if (!Float.isFinite(stateVz)) {
                stateVz = 0.0f;
            }
            if (!Float.isFinite(stateP)) {
                stateP = 0.0f;
            }
            stateP = MathHelper.clamp(stateP, STATE_PRESSURE_MIN, STATE_PRESSURE_MAX);
            window.baseField[idx + CH_STATE_VX] = stateVx;
            window.baseField[idx + CH_STATE_VY] = stateVy;
            window.baseField[idx + CH_STATE_VZ] = stateVz;
            window.baseField[idx + CH_STATE_P] = stateP;
            buffer.putFloat(window.baseField[idx + CH_OBSTACLE]);
            buffer.putFloat(window.baseField[idx + CH_FAN_MASK]);
            buffer.putFloat(window.baseField[idx + CH_FAN_VX]);
            buffer.putFloat(window.baseField[idx + CH_FAN_VY]);
            buffer.putFloat(window.baseField[idx + CH_FAN_VZ]);
            buffer.putFloat(stateVx * nativeInputScale);
            buffer.putFloat(stateVy * nativeInputScale);
            buffer.putFloat(stateVz * nativeInputScale);
            buffer.putFloat(stateP);
            buffer.putFloat(window.baseField[idx + CH_THERMAL_SOURCE]);
        }

        if (backend == SolverBackend.NATIVE) {
            return null;
        }
        return window.payloadBytes;
    }

    private boolean isSolidObstacle(ClientWorld world, BlockPos pos) {
        return isSolidObstacle(world, pos, world.getBlockState(pos));
    }

    private boolean isSolidObstacle(ClientWorld world, BlockPos pos, BlockState state) {
        if (state.isAir() || state.isOf(ModBlocks.DUCT_BLOCK)) {
            return false;
        }
        return !state.getCollisionShape(world, pos).isEmpty();
    }

    private float sampleAmbientThermalBias(ClientWorld world) {
        return 0.0f;
    }

    private float sampleBlockThermalSource(ClientWorld world, BlockPos pos, BlockState state) {
        float source = 0.0f;
        if (state.isOf(Blocks.LAVA) || state.isOf(Blocks.LAVA_CAULDRON)) {
            source += THERMAL_SOURCE_LAVA;
        }
        if (state.isOf(Blocks.MAGMA_BLOCK)) {
            source += THERMAL_SOURCE_MAGMA;
        }
        if (state.isOf(Blocks.CAMPFIRE)) {
            source += state.getOrEmpty(Properties.LIT).orElse(false) ? THERMAL_SOURCE_CAMPFIRE : 0.0f;
        }
        if (state.isOf(Blocks.SOUL_CAMPFIRE)) {
            source += state.getOrEmpty(Properties.LIT).orElse(false) ? THERMAL_SOURCE_SOUL_CAMPFIRE : 0.0f;
        }
        if (state.isOf(Blocks.FIRE)) {
            source += THERMAL_SOURCE_FIRE;
        }
        if (state.isOf(Blocks.SOUL_FIRE)) {
            source += THERMAL_SOURCE_SOUL_FIRE;
        }
        if (state.isOf(Blocks.TORCH) || state.isOf(Blocks.WALL_TORCH)) {
            source += THERMAL_SOURCE_TORCH;
        }
        if (state.isOf(Blocks.SOUL_TORCH) || state.isOf(Blocks.SOUL_WALL_TORCH)) {
            source += THERMAL_SOURCE_SOUL_TORCH;
        }
        if (state.isOf(Blocks.LANTERN)) {
            source += THERMAL_SOURCE_LANTERN;
        }
        if (state.isOf(Blocks.SOUL_LANTERN)) {
            source += THERMAL_SOURCE_SOUL_LANTERN;
        }
        source += sampleTerrainThermalFlux(world, pos, state);
        return Math.max(-NATIVE_THERMAL_SOURCE_MAX, Math.min(source, NATIVE_THERMAL_SOURCE_MAX));
    }

    private float sampleTerrainThermalFlux(ClientWorld world, BlockPos pos, BlockState state) {
        float flux = sampleAltitudeThermalFlux(world, pos);
        if (state.getFluidState().isIn(FluidTags.WATER)) {
            flux += THERMAL_SOURCE_WATER_COOLING;
        } else if (isStoneLikeTerrain(state)) {
            flux += THERMAL_SOURCE_STONE_HEATING;
        } else if (isBareTerrain(state)) {
            flux += THERMAL_SOURCE_BARE_HEATING;
        }
        return Math.max(-NATIVE_THERMAL_SOURCE_MAX, Math.min(flux, NATIVE_THERMAL_SOURCE_MAX));
    }

    private float sampleAltitudeThermalFlux(ClientWorld world, BlockPos pos) {
        // float belowSeaLevel = Math.max(0.0f, world.getSeaLevel() - pos.getY());
        // float lapse = belowSeaLevel * THERMAL_SOURCE_ALTITUDE_LAPSE_PER_BLOCK;
        // return Math.max(0.0f, Math.min(lapse, THERMAL_SOURCE_ALTITUDE_LAPSE_MAX));
        return 0.0f;
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

    private boolean isBareTerrain(BlockState state) {
        return state.isOf(Blocks.DIRT)
            || state.isOf(Blocks.COARSE_DIRT)
            || state.isOf(Blocks.ROOTED_DIRT)
            || state.isOf(Blocks.GRASS_BLOCK)
            || state.isOf(Blocks.PODZOL)
            || state.isOf(Blocks.MYCELIUM)
            || state.isOf(Blocks.SAND)
            || state.isOf(Blocks.RED_SAND)
            || state.isOf(Blocks.GRAVEL)
            || state.isOf(Blocks.CLAY)
            || state.isOf(Blocks.MUD);
    }

    private void addThermalSource(float[] thermal, int x, int y, int z, float source) {
        if (source == 0.0f || !inBounds(x, y, z)) {
            return;
        }
        int cell = (x * gridSize + y) * gridSize + z;
        float updated = thermal[cell] + source;
        thermal[cell] = Math.max(-NATIVE_THERMAL_SOURCE_MAX, Math.min(updated, NATIVE_THERMAL_SOURCE_MAX));
    }

    private void emitSolidHeatToNeighbors(float[] thermal, float[] obstacle, int x, int y, int z, float source) {
        int selected = 0;
        int[] selectedX = new int[THERMAL_SOURCE_SOLID_NEIGHBOR_COUNT];
        int[] selectedY = new int[THERMAL_SOURCE_SOLID_NEIGHBOR_COUNT];
        int[] selectedZ = new int[THERMAL_SOURCE_SOLID_NEIGHBOR_COUNT];
        int start = Math.floorMod((x * 73428767) ^ (y * 912931) ^ (z * 438289), THERMAL_NEIGHBOR_OFFSETS.length);
        for (int i = 0; i < THERMAL_NEIGHBOR_OFFSETS.length && selected < THERMAL_SOURCE_SOLID_NEIGHBOR_COUNT; i++) {
            int offsetIdx = (start + i) % THERMAL_NEIGHBOR_OFFSETS.length;
            int nx = x + THERMAL_NEIGHBOR_OFFSETS[offsetIdx][0];
            int ny = y + THERMAL_NEIGHBOR_OFFSETS[offsetIdx][1];
            int nz = z + THERMAL_NEIGHBOR_OFFSETS[offsetIdx][2];
            if (!inBounds(nx, ny, nz)) {
                continue;
            }
            int cell = (nx * gridSize + ny) * gridSize + nz;
            if (obstacle[cell] > 0.5f) {
                continue;
            }
            selectedX[selected] = nx;
            selectedY[selected] = ny;
            selectedZ[selected] = nz;
            selected++;
        }
        if (selected <= 0) {
            return;
        }
        float distributed = source * THERMAL_SOURCE_SOLID_EMISSION_SCALE / selected;
        for (int i = 0; i < selected; i++) {
            addThermalSource(thermal, selectedX[i], selectedY[i], selectedZ[i], distributed);
        }
    }

    private boolean refreshSampleFields(ClientWorld world, BlockPos origin, WindowState window, boolean forceFullResample) {
        window.ambientThermalBias = sampleAmbientThermalBias(world);
        if (!forceFullResample && window.obstacleField != null && window.sourceField != null && window.thermalField != null) {
            int cellCount = gridSize * gridSize * gridSize;
            float[] obstacle = new float[cellCount];
            float[] sourceField = new float[cellCount];
            int minX = origin.getX();
            int minY = origin.getY();
            int minZ = origin.getZ();
            BlockPos.Mutable cursor = new BlockPos.Mutable();

            for (int x = 0; x < gridSize; x++) {
                for (int y = 0; y < gridSize; y++) {
                    for (int z = 0; z < gridSize; z++) {
                        int cell = (x * gridSize + y) * gridSize + z;
                        cursor.set(minX + x, minY + y, minZ + z);
                        BlockState state = world.getBlockState(cursor);
                        boolean solid = isSolidObstacle(world, cursor, state);
                        obstacle[cell] = solid ? 1.0f : 0.0f;
                        sourceField[cell] = sampleBlockThermalSource(world, cursor, state);
                    }
                }
            }

            boolean geometryChanged = false;
            if (window.baseField != null) {
                for (int cell = 0; cell < cellCount; cell++) {
                    if (window.obstacleField[cell] == obstacle[cell]) {
                        continue;
                    }
                    geometryChanged = true;
                    int idx = cell * CHANNELS;
                    window.baseField[idx + CH_STATE_VX] = 0.0f;
                    window.baseField[idx + CH_STATE_VY] = 0.0f;
                    window.baseField[idx + CH_STATE_VZ] = 0.0f;
                    window.baseField[idx + CH_STATE_P] = 0.0f;
                }
            } else {
                for (int cell = 0; cell < cellCount; cell++) {
                    if (window.obstacleField[cell] != obstacle[cell]) {
                        geometryChanged = true;
                        break;
                    }
                }
            }

            float[] thermal = new float[cellCount];
            for (int x = 0; x < gridSize; x++) {
                for (int y = 0; y < gridSize; y++) {
                    for (int z = 0; z < gridSize; z++) {
                        int cell = (x * gridSize + y) * gridSize + z;
                        float source = sourceField[cell];
                        if (source == 0.0f) {
                            continue;
                        }
                        if (obstacle[cell] > 0.5f) {
                            emitSolidHeatToNeighbors(thermal, obstacle, x, y, z, source);
                        } else {
                            addThermalSource(thermal, x, y, z, source);
                        }
                    }
                }
            }

            window.obstacleField = obstacle;
            window.sourceField = sourceField;
            window.thermalField = thermal;
            window.lastSampleRefreshTick = tickCounter;
            return geometryChanged;
        }

        int cellCount = gridSize * gridSize * gridSize;
        float[] obstacle = new float[cellCount];
        float[] sourceField = new float[cellCount];
        int minX = origin.getX();
        int minY = origin.getY();
        int minZ = origin.getZ();
        BlockPos.Mutable cursor = new BlockPos.Mutable();

        for (int x = 0; x < gridSize; x++) {
            for (int y = 0; y < gridSize; y++) {
                for (int z = 0; z < gridSize; z++) {
                    int cell = (x * gridSize + y) * gridSize + z;
                    cursor.set(minX + x, minY + y, minZ + z);
                    BlockState state = world.getBlockState(cursor);
                    boolean solid = isSolidObstacle(world, cursor, state);
                    obstacle[cell] = solid ? 1.0f : 0.0f;
                    sourceField[cell] = sampleBlockThermalSource(world, cursor, state);
                }
            }
        }

        float[] thermal = new float[cellCount];
        for (int x = 0; x < gridSize; x++) {
            for (int y = 0; y < gridSize; y++) {
                for (int z = 0; z < gridSize; z++) {
                    int cell = (x * gridSize + y) * gridSize + z;
                    float source = sourceField[cell];
                    if (source == 0.0f) {
                        continue;
                    }
                    if (obstacle[cell] > 0.5f) {
                        emitSolidHeatToNeighbors(thermal, obstacle, x, y, z, source);
                    } else {
                        addThermalSource(thermal, x, y, z, source);
                    }
                }
            }
        }

        window.obstacleField = obstacle;
        window.sourceField = sourceField;
        window.thermalField = thermal;
        window.lastSampleRefreshTick = tickCounter;
        return false;
    }

    private void slideWindowSamples(ClientWorld world, WindowState window, BlockPos oldOrigin, BlockPos newOrigin) {
        if (window.obstacleField == null || window.sourceField == null || window.thermalField == null) {
            refreshSampleFields(world, newOrigin, window, true);
            return;
        }

        int dx = newOrigin.getX() - oldOrigin.getX();
        int dy = newOrigin.getY() - oldOrigin.getY();
        int dz = newOrigin.getZ() - oldOrigin.getZ();

        if (Math.abs(dx) >= gridSize || Math.abs(dy) >= gridSize || Math.abs(dz) >= gridSize) {
            refreshSampleFields(world, newOrigin, window, true);
            return;
        }

        int cellCount = gridSize * gridSize * gridSize;
        float[] newObstacle = new float[cellCount];
        float[] newSource = new float[cellCount];

        for (int x = 0; x < gridSize; x++) {
            int oldX = x - dx;
            for (int y = 0; y < gridSize; y++) {
                int oldY = y - dy;
                for (int z = 0; z < gridSize; z++) {
                    int oldZ = z - dz;
                    int newCell = (x * gridSize + y) * gridSize + z;
                    if (inBounds(oldX, oldY, oldZ)) {
                        int oldCell = (oldX * gridSize + oldY) * gridSize + oldZ;
                        newObstacle[newCell] = window.obstacleField[oldCell];
                        newSource[newCell] = window.sourceField[oldCell];
                    } else {
                        sampleCellFields(world, newOrigin, x, y, z, newObstacle, newSource, newCell);
                    }
                }
            }
        }

        window.obstacleField = newObstacle;
        window.sourceField = newSource;
        rebuildThermalFromSource(window);
    }

    private void sampleCellFields(
        ClientWorld world,
        BlockPos origin,
        int x,
        int y,
        int z,
        float[] obstacleOut,
        float[] sourceOut,
        int cell
    ) {
        BlockPos pos = new BlockPos(origin.getX() + x, origin.getY() + y, origin.getZ() + z);
        BlockState state = world.getBlockState(pos);
        boolean solid = isSolidObstacle(world, pos, state);
        obstacleOut[cell] = solid ? 1.0f : 0.0f;
        sourceOut[cell] = sampleBlockThermalSource(world, pos, state);
    }

    private void rebuildThermalFromSource(WindowState window) {
        int cellCount = gridSize * gridSize * gridSize;
        float[] thermal = new float[cellCount];
        for (int x = 0; x < gridSize; x++) {
            for (int y = 0; y < gridSize; y++) {
                for (int z = 0; z < gridSize; z++) {
                    int cell = (x * gridSize + y) * gridSize + z;
                    float source = window.sourceField[cell];
                    if (source == 0.0f) {
                        continue;
                    }
                    if (window.obstacleField[cell] > 0.5f) {
                        emitSolidHeatToNeighbors(thermal, window.obstacleField, x, y, z, source);
                    } else {
                        addThermalSource(thermal, x, y, z, source);
                    }
                }
            }
        }
        window.thermalField = thermal;
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
            boolean ok = nativeBackend.step(
                window.nativePayloadBuffer,
                gridSize,
                RESPONSE_CHANNELS,
                window.nativeContextId,
                solverOutput
            );
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

    private int updateBaseFieldFromResponse(WindowState window, float[] response, float speedCap) {
        if (window.baseField == null) {
            return 0;
        }

        int cellCount = gridSize * gridSize * gridSize;
        float capped = Math.max(0.0f, speedCap);
        int invalidComponents = 0;
        for (int i = 0; i < cellCount; i++) {
            int baseIdx = i * CHANNELS;
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

            window.baseField[baseIdx + CH_STATE_VX] = vx;
            window.baseField[baseIdx + CH_STATE_VY] = vy;
            window.baseField[baseIdx + CH_STATE_VZ] = vz;
            window.baseField[baseIdx + CH_STATE_P] = p;
        }
        return invalidComponents;
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

    private void invalidateBackendState(WindowState window, SolverBackend backend) {
        closeSocket(window);
        if (backend == SolverBackend.NATIVE) {
            nativeBackend.releaseContext(window.nativeContextId);
        }
    }

    private void releaseWindow(WindowState window) {
        window.renderer.clearFlowData();
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
        private ByteBuffer nativePayloadBuffer;
        private float[] solverOutputA;
        private float[] solverOutputB;
        private boolean solveIntoA = true;
        private float[] completedFlow;
        private byte[] socketReadBuffer;
        private float[] baseField;
        private float[] obstacleField;
        private float[] sourceField;
        private float[] thermalField;
        private float ambientThermalBias = 0.0f;
        private int lastSampleRefreshTick;
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

        private void ensureNativePayloadBuffer(int payloadBytesLength) {
            if (nativePayloadBuffer != null && nativePayloadBuffer.capacity() == payloadBytesLength) {
                return;
            }
            nativePayloadBuffer = ByteBuffer.allocateDirect(payloadBytesLength).order(ByteOrder.LITTLE_ENDIAN);
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
