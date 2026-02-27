package com.aerodynamics4mc.runtime;

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
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import java.util.UUID;

import com.aerodynamics4mc.FanBlock;
import com.aerodynamics4mc.ModBlocks;
import com.aerodynamics4mc.net.AeroFlowPayload;
import com.aerodynamics4mc.net.AeroRuntimeStatePayload;
import com.mojang.brigadier.CommandDispatcher;
import com.mojang.brigadier.arguments.FloatArgumentType;
import com.mojang.brigadier.arguments.IntegerArgumentType;

import net.fabricmc.fabric.api.command.v2.CommandRegistrationCallback;
import net.fabricmc.fabric.api.event.lifecycle.v1.ServerLifecycleEvents;
import net.fabricmc.fabric.api.event.lifecycle.v1.ServerTickEvents;
import net.fabricmc.fabric.api.networking.v1.ServerPlayConnectionEvents;
import net.fabricmc.fabric.api.networking.v1.ServerPlayNetworking;
import net.minecraft.block.BlockState;
import net.minecraft.block.entity.BlockEntity;
import net.minecraft.command.CommandRegistryAccess;
import net.minecraft.command.CommandSource;
import net.minecraft.entity.Entity;
import net.minecraft.network.packet.s2c.play.EntityVelocityUpdateS2CPacket;
import net.minecraft.registry.RegistryKey;
import net.minecraft.server.MinecraftServer;
import net.minecraft.server.command.CommandManager;
import net.minecraft.server.command.ServerCommandSource;
import net.minecraft.server.network.ServerPlayerEntity;
import net.minecraft.server.world.ServerWorld;
import net.minecraft.text.Text;
import net.minecraft.util.Identifier;
import net.minecraft.util.math.BlockPos;
import net.minecraft.util.math.Box;
import net.minecraft.util.math.ChunkPos;
import net.minecraft.util.math.Direction;
import net.minecraft.util.math.MathHelper;
import net.minecraft.util.math.Vec3d;
import net.minecraft.world.World;
import net.minecraft.world.chunk.WorldChunk;

public final class AeroServerRuntime {
    private static final String LOG_PREFIX = "[aerodynamics4mc] ";
    private static final int GRID_SIZE = 128;
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
    private static final int SOCKET_PORT = 5001;
    private static final String SOCKET_HOST = "127.0.0.1";
    private static final int RESPONSE_CHANNELS = 4;
    private static final int FLOW_COUNT = GRID_SIZE * GRID_SIZE * GRID_SIZE * RESPONSE_CHANNELS;

    private static final int WINDOW_REFRESH_TICKS = 20;
    private static final int OBSTACLE_REFRESH_TICKS = 200;
    private static final int FLOW_SYNC_INTERVAL_TICKS = 2;
    private static final int FAN_SCAN_RADIUS = 48;

    private static final int DEFAULT_STREAMLINE_STRIDE = 4;
    private static final int MIN_STREAMLINE_STRIDE = 1;
    private static final int MAX_STREAMLINE_STRIDE = 8;

    private static final float NATIVE_VELOCITY_SCALE = 30.0f;
    private static final double FORCE_STRENGTH = 0.02;
    private static final double PLAYER_FORCE_STRENGTH = 0.02;
    private static final int TICKS_PER_SECOND = 20;
    private static final int PLAYER_VELOCITY_SYNC_MIN_INTERVAL_TICKS = 20;
    private static final int PLAYER_VELOCITY_SYNC_MAX_INTERVAL_TICKS = 40;
    private static final double PLAYER_VELOCITY_SYNC_ERROR_THRESHOLD_SQ = 2.5e-5;

    private enum BackendMode {
        SOCKET,
        NATIVE
    }

    private static final AeroServerRuntime INSTANCE = new AeroServerRuntime();

    private final Map<WindowKey, WindowState> windows = new HashMap<>();
    private final Map<UUID, Integer> entityVelocitySyncTickById = new HashMap<>();
    private final Map<UUID, Vec3d> entityLastSyncedVelocityById = new HashMap<>();
    private final NativeLbmBridge nativeBackend = new NativeLbmBridge();

    private boolean streamingEnabled = false;
    private boolean debugEnabled = false;
    private float maxWindSpeed = INFLOW_SPEED;
    private int tickCounter = 0;
    private long simulationTicks = 0L;
    private int secondWindowTotalTicks = 0;
    private int secondWindowSimulationTicks = 0;
    private float simulationTicksPerSecond = 0.0f;
    private float lastMaxFlowSpeed = 0.0f;
    private String lastSolverError = "";
    private int streamlineSampleStride = DEFAULT_STREAMLINE_STRIDE;
    private BackendMode backendMode = BackendMode.NATIVE;
    private long nextContextId = 1L;

    private AeroServerRuntime() {
    }

    public static void init() {
        ServerTickEvents.END_SERVER_TICK.register(INSTANCE::onServerTick);
        ServerPlayConnectionEvents.JOIN.register((handler, sender, server) -> {
            INSTANCE.sendStateToPlayer(handler.player);
            INSTANCE.sendFlowSnapshotToPlayer(handler.player);
        });
        ServerLifecycleEvents.SERVER_STOPPED.register(server -> INSTANCE.shutdownAll());
        CommandRegistrationCallback.EVENT.register(INSTANCE::registerCommands);
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
                    feedback(ctx.getSource(), "Streaming enabled");
                    broadcastState(ctx.getSource().getServer());
                    return 1;
                }))
            .then(CommandManager.literal("stop")
                .executes(ctx -> {
                    stopStreaming();
                    feedback(ctx.getSource(), "Streaming disabled");
                    broadcastState(ctx.getSource().getServer());
                    return 1;
                }))
            .then(CommandManager.literal("maxspeed")
                .then(CommandManager.argument("value", FloatArgumentType.floatArg(0.0f, 64.0f))
                    .suggests((context, builder) -> CommandSource.suggestMatching(List.of("4", "8", "12", "16"), builder))
                    .executes(ctx -> {
                        maxWindSpeed = FloatArgumentType.getFloat(ctx, "value");
                        feedback(ctx.getSource(), "Max wind speed set to " + format2(maxWindSpeed));
                        broadcastState(ctx.getSource().getServer());
                        return 1;
                    })))
            .then(CommandManager.literal("streamline")
                .then(CommandManager.argument("stride", IntegerArgumentType.integer(MIN_STREAMLINE_STRIDE, MAX_STREAMLINE_STRIDE))
                    .suggests((context, builder) -> CommandSource.suggestMatching(List.of("1", "2", "4", "8"), builder))
                    .executes(ctx -> {
                        int requested = IntegerArgumentType.getInteger(ctx, "stride");
                        int stride = sanitizeStride(requested);
                        if (stride != requested) {
                            error(ctx.getSource(), "Stride must be one of: 1, 2, 4, 8");
                            return 0;
                        }
                        streamlineSampleStride = stride;
                        feedback(ctx.getSource(), "Streamline sample stride set to " + stride);
                        broadcastState(ctx.getSource().getServer());
                        return 1;
                    })))
            .then(CommandManager.literal("backend")
                .then(CommandManager.literal("socket")
                    .executes(ctx -> {
                        switchBackend(BackendMode.SOCKET);
                        feedback(ctx.getSource(), "Backend set to socket");
                        return 1;
                    }))
                .then(CommandManager.literal("native")
                    .executes(ctx -> {
                        if (!nativeBackend.isLoaded()) {
                            error(ctx.getSource(), "Native backend unavailable: " + nativeBackend.getLoadError());
                            return 0;
                        }
                        if (!nativeBackend.ensureInitialized(GRID_SIZE, CHANNELS, RESPONSE_CHANNELS)) {
                            error(ctx.getSource(), "Native backend initialization failed");
                            return 0;
                        }
                        switchBackend(BackendMode.NATIVE);
                        feedback(ctx.getSource(), "Backend set to native (" + nativeBackend.runtimeInfo() + ")");
                        feedback(ctx.getSource(), "Native timing: " + nativeBackend.timingInfo());
                        return 1;
                    }))
                .then(CommandManager.literal("status")
                    .executes(ctx -> {
                        feedback(
                            ctx.getSource(),
                            "Backend=" + backendMode.name().toLowerCase(Locale.ROOT)
                                + " runtime=" + (backendMode == BackendMode.NATIVE ? nativeBackend.runtimeInfo() : "socket")
                        );
                        if (backendMode == BackendMode.NATIVE) {
                            feedback(ctx.getSource(), "Native timing: " + nativeBackend.timingInfo());
                        }
                        return 1;
                    })))
            .then(CommandManager.literal("status")
                .executes(ctx -> {
                    feedback(
                        ctx.getSource(),
                        "Status streaming=" + streamingEnabled
                            + " debug=" + debugEnabled
                            + " maxspeed=" + format2(maxWindSpeed)
                            + " stride=" + streamlineSampleStride
                            + " windows=" + windows.size()
                            + " simTicks=" + simulationTicks
                            + " simTickPerSec=" + format2(simulationTicksPerSecond)
                            + " maxFlow=" + format2(lastMaxFlowSpeed)
                    );
                    if (!lastSolverError.isEmpty()) {
                        feedback(ctx.getSource(), "Last solver error: " + lastSolverError);
                    }
                    return 1;
                }))
            .then(CommandManager.literal("debug")
                .executes(ctx -> {
                    debugEnabled = !debugEnabled;
                    feedback(ctx.getSource(), "Debug rendering " + (debugEnabled ? "enabled" : "disabled"));
                    broadcastState(ctx.getSource().getServer());
                    return 1;
                }))
        );
    }

    private void onServerTick(MinecraftServer server) {
        if (!streamingEnabled) {
            return;
        }
        if (server.isPaused()) {
            return;
        }

        tickCounter++;
        if (tickCounter % (PLAYER_VELOCITY_SYNC_MAX_INTERVAL_TICKS * 2) == 0) {
            pruneEntityVelocitySyncState();
        }
        if (windows.isEmpty() || tickCounter % WINDOW_REFRESH_TICKS == 0) {
            refreshWindows(server);
        }

        if (windows.isEmpty()) {
            updateSimulationRate(false);
            return;
        }

        boolean shouldSyncFlow = debugEnabled && (tickCounter % FLOW_SYNC_INTERVAL_TICKS == 0);
        boolean steppedThisTick = false;
        Iterator<Map.Entry<WindowKey, WindowState>> it = windows.entrySet().iterator();
        while (it.hasNext()) {
            Map.Entry<WindowKey, WindowState> entry = it.next();
            WindowKey key = entry.getKey();
            WindowState window = entry.getValue();
            ServerWorld world = server.getWorld(key.worldKey());
            if (world == null) {
                releaseWindow(window);
                it.remove();
                continue;
            }

            try {
                byte[] payload = captureWindow(window, key.origin(), backendMode);
                float[] response = runSolverStep(window, payload, backendMode);
                updateBaseFieldFromResponse(window, response, maxWindSpeed);
                window.latestFlow = response;
                steppedThisTick = true;
                lastSolverError = "";

                applyForces(world, key.origin(), response, FORCE_STRENGTH);
                if (shouldSyncFlow) {
                    syncFlowToPlayers(world, key.origin(), response, streamlineSampleStride);
                }
            } catch (IOException ex) {
                lastSolverError = ex.getMessage();
                log("Solver error for window " + formatPos(key.origin()) + ": " + lastSolverError);
                closeSocket(window);
            }
        }
        if (steppedThisTick) {
            simulationTicks++;
        }
        updateSimulationRate(steppedThisTick);
    }

    private void switchBackend(BackendMode mode) {
        if (backendMode == mode) {
            return;
        }
        if (mode == BackendMode.SOCKET) {
            for (WindowState window : windows.values()) {
                nativeBackend.releaseContext(window.nativeContextId);
            }
        } else {
            for (WindowState window : windows.values()) {
                closeSocket(window);
            }
        }
        backendMode = mode;
    }

    private void stopStreaming() {
        streamingEnabled = false;
        tickCounter = 0;
        simulationTicks = 0L;
        secondWindowTotalTicks = 0;
        secondWindowSimulationTicks = 0;
        simulationTicksPerSecond = 0.0f;
        lastMaxFlowSpeed = 0.0f;
        entityVelocitySyncTickById.clear();
        entityLastSyncedVelocityById.clear();
        for (WindowState window : windows.values()) {
            releaseWindow(window);
        }
        windows.clear();
        nativeBackend.shutdown();
    }

    private void shutdownAll() {
        stopStreaming();
    }

    private void refreshWindows(MinecraftServer server) {
        Map<WindowKey, List<FanSource>> fansByWindow = scanFanSources(server);

        Iterator<Map.Entry<WindowKey, WindowState>> it = windows.entrySet().iterator();
        while (it.hasNext()) {
            Map.Entry<WindowKey, WindowState> entry = it.next();
            if (fansByWindow.containsKey(entry.getKey())) {
                continue;
            }
            releaseWindow(entry.getValue());
            it.remove();
        }

        for (Map.Entry<WindowKey, List<FanSource>> entry : fansByWindow.entrySet()) {
            WindowKey key = entry.getKey();
            ServerWorld world = server.getWorld(key.worldKey());
            if (world == null) {
                continue;
            }

            WindowState window = windows.get(key);
            if (window == null) {
                window = new WindowState(nextContextId++);
                windows.put(key, window);
            }

            window.fans = List.copyOf(entry.getValue());
            refreshObstacleField(world, key.origin(), window, tickCounter);
        }
    }

    private Map<WindowKey, List<FanSource>> scanFanSources(MinecraftServer server) {
        Map<WindowKey, List<FanSource>> fansByWindow = new HashMap<>();
        for (ServerWorld world : server.getWorlds()) {
            List<BlockPos> playerPositions = new ArrayList<>();
            for (ServerPlayerEntity player : world.getPlayers()) {
                playerPositions.add(player.getBlockPos());
            }
            if (playerPositions.isEmpty()) {
                continue;
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
                            BlockPos origin = alignToGrid(pos);
                            WindowKey key = new WindowKey(world.getRegistryKey(), origin);
                            fansByWindow.computeIfAbsent(key, ignored -> new ArrayList<>()).add(new FanSource(pos, facing));
                        }
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
        int x = Math.floorDiv(pos.getX(), GRID_SIZE) * GRID_SIZE;
        int y = Math.floorDiv(pos.getY(), GRID_SIZE) * GRID_SIZE;
        int z = Math.floorDiv(pos.getZ(), GRID_SIZE) * GRID_SIZE;
        return new BlockPos(x, y, z);
    }

    private int voxelIndex(int x, int y, int z) {
        return ((x * GRID_SIZE + y) * GRID_SIZE + z) * CHANNELS;
    }

    private boolean inBounds(int x, int y, int z) {
        return x >= 0 && y >= 0 && z >= 0 && x < GRID_SIZE && y < GRID_SIZE && z < GRID_SIZE;
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
    }

    private byte[] captureWindow(WindowState window, BlockPos origin, BackendMode backend) {
        int voxelCount = GRID_SIZE * GRID_SIZE * GRID_SIZE * CHANNELS;
        window.ensureBaseFieldInitialized();
        float[] obstacleField = window.obstacleField;
        int minX = origin.getX();
        int minY = origin.getY();
        int minZ = origin.getZ();
        ByteBuffer buffer = ByteBuffer.allocate(voxelCount * Float.BYTES).order(ByteOrder.LITTLE_ENDIAN);

        for (int x = 0; x < GRID_SIZE; x++) {
            for (int y = 0; y < GRID_SIZE; y++) {
                for (int z = 0; z < GRID_SIZE; z++) {
                    int cell = (x * GRID_SIZE + y) * GRID_SIZE + z;
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
                    }
                }
            }
        }

        for (FanSource fan : window.fans) {
            applyFanSource(window, fan, minX, minY, minZ);
        }

        float nativeInputScale = backend == BackendMode.NATIVE ? (1.0f / NATIVE_VELOCITY_SCALE) : 1.0f;
        int cellCount = GRID_SIZE * GRID_SIZE * GRID_SIZE;
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

        return buffer.array();
    }

    private boolean isSolidObstacle(ServerWorld world, BlockPos pos) {
        BlockState state = world.getBlockState(pos);
        if (state.isAir()) {
            return false;
        }
        return !state.getCollisionShape(world, pos).isEmpty();
    }

    private void refreshObstacleField(ServerWorld world, BlockPos origin, WindowState window, int tickNow) {
        if (window.obstacleField != null
            && window.lastObstacleRefreshTick >= 0
            && (tickNow - window.lastObstacleRefreshTick) < OBSTACLE_REFRESH_TICKS) {
            return;
        }

        int cellCount = GRID_SIZE * GRID_SIZE * GRID_SIZE;
        float[] obstacle = new float[cellCount];
        int minX = origin.getX();
        int minY = origin.getY();
        int minZ = origin.getZ();
        BlockPos.Mutable cursor = new BlockPos.Mutable();

        for (int x = 0; x < GRID_SIZE; x++) {
            for (int y = 0; y < GRID_SIZE; y++) {
                for (int z = 0; z < GRID_SIZE; z++) {
                    int cell = (x * GRID_SIZE + y) * GRID_SIZE + z;
                    cursor.set(minX + x, minY + y, minZ + z);
                    obstacle[cell] = isSolidObstacle(world, cursor) ? 1.0f : 0.0f;
                }
            }
        }

        window.obstacleField = obstacle;
        window.lastObstacleRefreshTick = tickNow;
    }

    private float[] runSolverStep(WindowState window, byte[] payload, BackendMode backend) throws IOException {
        if (backend == BackendMode.NATIVE) {
            if (!nativeBackend.isLoaded()) {
                throw new IOException("Native backend not loaded: " + nativeBackend.getLoadError());
            }
            if (!nativeBackend.ensureInitialized(GRID_SIZE, CHANNELS, RESPONSE_CHANNELS)) {
                throw new IOException("Native backend initialization failed");
            }
            float[] response = nativeBackend.step(payload, GRID_SIZE, RESPONSE_CHANNELS, window.nativeContextId);
            if (response == null || response.length != FLOW_COUNT) {
                throw new IOException("Native backend returned invalid response");
            }
            scaleResponseVelocity(response, NATIVE_VELOCITY_SCALE);
            return response;
        }

        ensureSocket(window);
        sendPayload(window, payload);
        return receiveFlowField(window);
    }

    private void updateBaseFieldFromResponse(WindowState window, float[] response, float speedCap) {
        if (window.baseField == null) {
            return;
        }

        int cellCount = GRID_SIZE * GRID_SIZE * GRID_SIZE;
        float capped = Math.max(0.0f, speedCap);
        float maxSpeedThisStep = 0.0f;
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
            float speed = (float) Math.sqrt(vx * vx + vy * vy + vz * vz);
            if (speed > maxSpeedThisStep) {
                maxSpeedThisStep = speed;
            }

            window.baseField[baseIdx + CH_STATE_VX] = vx;
            window.baseField[baseIdx + CH_STATE_VY] = vy;
            window.baseField[baseIdx + CH_STATE_VZ] = vz;
            window.baseField[baseIdx + CH_STATE_P] = response[respIdx + 3];
        }
        lastMaxFlowSpeed = maxSpeedThisStep;
    }

    private void applyForces(ServerWorld world, BlockPos origin, float[] response, double strength) {
        Box box = new Box(
            origin.getX(),
            origin.getY(),
            origin.getZ(),
            origin.getX() + GRID_SIZE,
            origin.getY() + GRID_SIZE,
            origin.getZ() + GRID_SIZE
        );

        for (Entity entity : world.getEntitiesByClass(Entity.class, box, e -> e.isAlive() && !e.isSpectator())) {
            Vec3d center = entity.getBoundingBox().getCenter();
            Vec3d velocity = sampleVelocity(response, origin, center);
            double forceScale = entity instanceof ServerPlayerEntity ? PLAYER_FORCE_STRENGTH : strength;
            Vec3d delta = velocity.multiply(forceScale);
            if (delta.lengthSquared() < 1e-10) {
                continue;
            }
            entity.addVelocity(delta.x, delta.y, delta.z);
            if (shouldSyncEntityVelocity(entity)) {
                EntityVelocityUpdateS2CPacket packet = new EntityVelocityUpdateS2CPacket(entity);
                if (entity instanceof ServerPlayerEntity player) {
                    player.networkHandler.sendPacket(packet);
                    world.getChunkManager().sendToOtherNearbyPlayers(player, packet);
                } else {
                    world.getChunkManager().sendToNearbyPlayers(entity, packet);
                }
            }
        }
    }

    private boolean shouldSyncEntityVelocity(Entity entity) {
        UUID entityId = entity.getUuid();
        int now = tickCounter;
        Integer lastTick = entityVelocitySyncTickById.get(entityId);
        if (lastTick != null && (now - lastTick) < PLAYER_VELOCITY_SYNC_MIN_INTERVAL_TICKS) {
            return false;
        }

        Vec3d currentVelocity = entity.getVelocity();
        Vec3d lastSyncedVelocity = entityLastSyncedVelocityById.get(entityId);
        boolean isFirstSync = lastTick == null || lastSyncedVelocity == null;
        boolean timedOut = lastTick == null || (now - lastTick) >= PLAYER_VELOCITY_SYNC_MAX_INTERVAL_TICKS;
        boolean driftExceeded = lastSyncedVelocity != null
            && currentVelocity.squaredDistanceTo(lastSyncedVelocity) >= PLAYER_VELOCITY_SYNC_ERROR_THRESHOLD_SQ;

        if (!isFirstSync && !timedOut && !driftExceeded) {
            return false;
        }

        entityVelocitySyncTickById.put(entityId, now);
        entityLastSyncedVelocityById.put(entityId, currentVelocity);
        return true;
    }

    private void pruneEntityVelocitySyncState() {
        if (entityVelocitySyncTickById.isEmpty()) {
            return;
        }
        int staleBeforeTick = tickCounter - (PLAYER_VELOCITY_SYNC_MAX_INTERVAL_TICKS * 4);
        Iterator<Map.Entry<UUID, Integer>> iterator = entityVelocitySyncTickById.entrySet().iterator();
        while (iterator.hasNext()) {
            Map.Entry<UUID, Integer> entry = iterator.next();
            if (entry.getValue() < staleBeforeTick) {
                entityLastSyncedVelocityById.remove(entry.getKey());
                iterator.remove();
            }
        }
    }

    private Vec3d sampleVelocity(float[] response, BlockPos origin, Vec3d worldPos) {
        double localX = worldPos.x - origin.getX();
        double localY = worldPos.y - origin.getY();
        double localZ = worldPos.z - origin.getZ();

        if (localX < 0 || localY < 0 || localZ < 0 || localX >= GRID_SIZE - 1 || localY >= GRID_SIZE - 1 || localZ >= GRID_SIZE - 1) {
            return Vec3d.ZERO;
        }

        int x0 = (int) Math.floor(localX);
        int y0 = (int) Math.floor(localY);
        int z0 = (int) Math.floor(localZ);
        int x1 = x0 + 1;
        int y1 = y0 + 1;
        int z1 = z0 + 1;

        float fx = (float) (localX - x0);
        float fy = (float) (localY - y0);
        float fz = (float) (localZ - z0);

        Vec3d c000 = velocityAt(response, x0, y0, z0);
        Vec3d c100 = velocityAt(response, x1, y0, z0);
        Vec3d c010 = velocityAt(response, x0, y1, z0);
        Vec3d c110 = velocityAt(response, x1, y1, z0);
        Vec3d c001 = velocityAt(response, x0, y0, z1);
        Vec3d c101 = velocityAt(response, x1, y0, z1);
        Vec3d c011 = velocityAt(response, x0, y1, z1);
        Vec3d c111 = velocityAt(response, x1, y1, z1);

        Vec3d c00 = lerp(c000, c100, fx);
        Vec3d c10 = lerp(c010, c110, fx);
        Vec3d c01 = lerp(c001, c101, fx);
        Vec3d c11 = lerp(c011, c111, fx);

        Vec3d c0 = lerp(c00, c10, fy);
        Vec3d c1 = lerp(c01, c11, fy);
        return lerp(c0, c1, fz);
    }

    private Vec3d velocityAt(float[] response, int x, int y, int z) {
        int cx = MathHelper.clamp(x, 0, GRID_SIZE - 1);
        int cy = MathHelper.clamp(y, 0, GRID_SIZE - 1);
        int cz = MathHelper.clamp(z, 0, GRID_SIZE - 1);
        int idx = ((cx * GRID_SIZE + cy) * GRID_SIZE + cz) * RESPONSE_CHANNELS;
        return new Vec3d(response[idx], response[idx + 1], response[idx + 2]);
    }

    private Vec3d lerp(Vec3d a, Vec3d b, float t) {
        return new Vec3d(
            MathHelper.lerp(t, a.x, b.x),
            MathHelper.lerp(t, a.y, b.y),
            MathHelper.lerp(t, a.z, b.z)
        );
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

    private void syncFlowToPlayers(ServerWorld world, BlockPos origin, float[] response, int stride) {
        int sampleStride = sanitizeStride(stride);
        float[] sampled = sampleFlow(response, sampleStride);
        Identifier dimId = world.getRegistryKey().getValue();
        AeroFlowPayload payload = new AeroFlowPayload(dimId, origin, sampleStride, sampled);

        for (ServerPlayerEntity player : world.getPlayers()) {
            ServerPlayNetworking.send(player, payload);
        }
    }

    private float[] sampleFlow(float[] response, int stride) {
        int n = (GRID_SIZE + stride - 1) / stride;
        float[] sampled = new float[n * n * n * RESPONSE_CHANNELS];
        int dst = 0;

        for (int x = 0; x < n; x++) {
            int sx = Math.min(GRID_SIZE - 1, x * stride);
            for (int y = 0; y < n; y++) {
                int sy = Math.min(GRID_SIZE - 1, y * stride);
                for (int z = 0; z < n; z++) {
                    int sz = Math.min(GRID_SIZE - 1, z * stride);
                    int src = ((sx * GRID_SIZE + sy) * GRID_SIZE + sz) * RESPONSE_CHANNELS;
                    sampled[dst] = response[src];
                    sampled[dst + 1] = response[src + 1];
                    sampled[dst + 2] = response[src + 2];
                    sampled[dst + 3] = response[src + 3];
                    dst += RESPONSE_CHANNELS;
                }
            }
        }

        return sampled;
    }

    private int sanitizeStride(int requested) {
        return (requested == 1 || requested == 2 || requested == 4 || requested == 8) ? requested : DEFAULT_STREAMLINE_STRIDE;
    }

    private void updateSimulationRate(boolean steppedThisTick) {
        secondWindowTotalTicks++;
        if (steppedThisTick) {
            secondWindowSimulationTicks++;
        }
        if (secondWindowTotalTicks >= TICKS_PER_SECOND) {
            simulationTicksPerSecond = (secondWindowSimulationTicks * (float) TICKS_PER_SECOND) / secondWindowTotalTicks;
            secondWindowTotalTicks = 0;
            secondWindowSimulationTicks = 0;
        }
    }

    private void sendStateToPlayer(ServerPlayerEntity player) {
        ServerPlayNetworking.send(
            player,
            new AeroRuntimeStatePayload(streamingEnabled, debugEnabled, maxWindSpeed, streamlineSampleStride)
        );
    }

    private void broadcastState(MinecraftServer server) {
        for (ServerPlayerEntity player : server.getPlayerManager().getPlayerList()) {
            sendStateToPlayer(player);
        }
    }

    private void sendFlowSnapshotToPlayer(ServerPlayerEntity player) {
        if (!debugEnabled) {
            return;
        }

        int stride = sanitizeStride(streamlineSampleStride);

        for (Map.Entry<WindowKey, WindowState> entry : windows.entrySet()) {
            WindowKey key = entry.getKey();
            WindowState window = entry.getValue();
            if (window.latestFlow == null) {
                continue;
            }

            float[] sampled = sampleFlow(window.latestFlow, stride);
            Identifier dimId = key.worldKey().getValue();
            ServerPlayNetworking.send(player, new AeroFlowPayload(dimId, key.origin(), stride, sampled));
        }
    }

    private void sendPayload(WindowState window, byte[] payload) throws IOException {
        if (window.outputStream == null) {
            throw new IOException("Socket not ready");
        }
        window.outputStream.writeInt(payload.length);
        window.outputStream.write(payload);
        window.outputStream.flush();
    }

    private float[] receiveFlowField(WindowState window) throws IOException {
        if (window.inputStream == null) {
            throw new IOException("Socket not ready");
        }
        int length = window.inputStream.readInt();
        int expectedBytes = FLOW_COUNT * Float.BYTES;
        if (length != expectedBytes) {
            throw new IOException("Unexpected response length: " + length);
        }
        byte[] data = window.inputStream.readNBytes(length);
        ByteBuffer buffer = ByteBuffer.wrap(data).order(ByteOrder.LITTLE_ENDIAN);
        float[] values = new float[FLOW_COUNT];
        buffer.asFloatBuffer().get(values);
        return values;
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

    private void feedback(ServerCommandSource source, String message) {
        source.sendFeedback(() -> Text.literal(LOG_PREFIX + message), false);
    }

    private void error(ServerCommandSource source, String message) {
        source.sendError(Text.literal(LOG_PREFIX + message));
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

    private record WindowKey(RegistryKey<World> worldKey, BlockPos origin) {
    }

    private record FanSource(BlockPos pos, Direction facing) {
    }

    private static final class WindowState {
        private final long nativeContextId;
        private Socket socket;
        private DataInputStream inputStream;
        private DataOutputStream outputStream;
        private float[] baseField;
        private float[] obstacleField;
        private int lastObstacleRefreshTick = -1;
        private List<FanSource> fans = List.of();
        private float[] latestFlow;

        private WindowState(long nativeContextId) {
            this.nativeContextId = nativeContextId;
        }

        private void ensureBaseFieldInitialized() {
            if (baseField != null) {
                return;
            }
            baseField = new float[GRID_SIZE * GRID_SIZE * GRID_SIZE * CHANNELS];
        }
    }
}
