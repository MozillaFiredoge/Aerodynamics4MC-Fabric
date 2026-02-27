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
import java.util.List;
import java.util.Map;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;

import com.aerodynamics4mc.FanBlock;
import com.aerodynamics4mc.FanBlockEntity;
import com.aerodynamics4mc.ModBlocks;
import com.mojang.brigadier.arguments.FloatArgumentType;

import net.fabricmc.api.ClientModInitializer;
import net.fabricmc.fabric.api.client.command.v2.ClientCommandManager;
import net.fabricmc.fabric.api.client.command.v2.ClientCommandRegistrationCallback;
import net.fabricmc.fabric.api.client.event.lifecycle.v1.ClientTickEvents;
import net.fabricmc.fabric.api.client.rendering.v1.world.WorldRenderContext;
import net.fabricmc.fabric.api.client.rendering.v1.world.WorldRenderEvents;
import net.minecraft.block.BlockState;
import net.minecraft.block.entity.BlockEntity;
import net.minecraft.client.MinecraftClient;
import net.minecraft.util.math.BlockPos;
import net.minecraft.util.math.Direction;
import net.minecraft.util.math.MathHelper;
import net.minecraft.world.World;

public class AeroClientMod implements ClientModInitializer {
    private static final String LOG_PREFIX = "[aerodynamics4mc] ";
    private static final int GRID_SIZE = 128;
    // [obstacle_mask, fan_mask, fan_vx, fan_vy, fan_vz, vx, vy, vz, pressure]
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
    private static final int PARTICLE_DEBUG_INTERVAL_TICKS = 4;
    // Keep client-side state/visualization on SOCKET-scale velocities and convert at native boundary.
    private static final float NATIVE_VELOCITY_SCALE = 10.0f;

    private enum BackendMode {
        SOCKET,
        NATIVE
    }

    private final ExecutorService executor = Executors.newFixedThreadPool(Math.max(2, Runtime.getRuntime().availableProcessors() / 2));
    private final Map<BlockPos, WindowState> windows = new HashMap<>();
    private final NativeLbmBridge nativeBackend = new NativeLbmBridge();
    private boolean streamingEnabled = false;
    private boolean debugEnabled = false;
    private int tickCounter = 0;
    private volatile float maxWindSpeed = INFLOW_SPEED;
    private volatile BackendMode backendMode = BackendMode.NATIVE;

    @Override
    public void onInitializeClient() {
        ClientTickEvents.END_CLIENT_TICK.register(this::onTick);
        WorldRenderEvents.BEFORE_DEBUG_RENDER.register(this::onRender);
        registerCommands();
        if (nativeBackend.isLoaded()) {
            log("Native backend library detected (use /aero backend native to enable)");
        } else {
            log("Native backend unavailable: " + nativeBackend.getLoadError());
        }
    }

    private void onRender(WorldRenderContext context) {
        if (!debugEnabled) {
            return;
        }
        for (WindowState window : windows.values()) {
            window.renderer.render(context);
        }
    }

    private void onTick(MinecraftClient client) {
        if (client.player == null || client.world == null) {
            return;
        }
        if (!streamingEnabled) {
            return;
        }
        tickCounter++;
        if (tickCounter % WINDOW_REFRESH_TICKS == 0) {
            refreshWindows(client);
        }
        for (WindowState window : windows.values()) {
            // Apply forces every tick, even while network inference for this window is in-flight.
            window.physicsHandler.applyForces(client, 0.02);
            if (window.busy.get()) {
                continue;
            }
            window.busy.set(true);
            executor.execute(() -> {
                try {
                    BackendMode activeBackend = backendMode;
                    byte[] payload = captureWindow(window, activeBackend);
                    float[] response = runSolverStep(window, payload, activeBackend);
                    updateBaseFieldFromResponse(window, response, maxWindSpeed);
                    window.renderer.updateFlowField(window.origin, response);
                    window.physicsHandler.updateOrigin(window.origin);
                } catch (IOException ex) {
                    log("Solver error for window " + formatPos(window.origin) + ": " + ex.getMessage());
                    closeSocket(window);
                } finally {
                    window.busy.set(false);
                }
            });
            // window.renderer.spawnSourceParticles(client, 2);
            // if (debugEnabled && window.baseField != null && tickCounter % PARTICLE_DEBUG_INTERVAL_TICKS == 0) {
            //     window.renderer.renderDebug(client, window.baseField);
            // }
        }
    }

    private void refreshWindows(MinecraftClient client) {
        if (client.world == null) {
            return;
        }
        Map<BlockPos, List<FanSource>> fansByWindow = new HashMap<>();
        //log("World blockEntities count: " + client.world.getBlockEntities().size());
        for (BlockEntity entity : client.world.getBlockEntities()) {
            if (!(entity instanceof FanBlockEntity)) {
                continue;
            }
            BlockPos pos = entity.getPos();
            BlockState state = client.world.getBlockState(pos);
            if (!state.isOf(ModBlocks.FAN_BLOCK)) {
                continue;
            }
            Direction facing = state.get(FanBlock.FACING);
            BlockPos origin = alignToGrid(pos);
            fansByWindow.computeIfAbsent(origin, key -> new ArrayList<>())
                .add(new FanSource(pos, facing));
        }
        int scannedFans = fansByWindow.values().stream().mapToInt(List::size).sum();
        if (scannedFans == 0) {
            scannedFans = fallbackScanFanBlocks(client, fansByWindow);
        }
        // log("Scanned fans: " + scannedFans);

        windows.entrySet().removeIf(entry -> {
            if (fansByWindow.containsKey(entry.getKey())) {
                return false;
            }
            // log("Removing window (no fans) " + formatPos(entry.getKey()));
            nativeBackend.releaseContext(entry.getKey().asLong());
            closeSocket(entry.getValue());
            return true;
        });

        for (Map.Entry<BlockPos, List<FanSource>> entry : fansByWindow.entrySet()) {
            WindowState window = windows.get(entry.getKey());
            if (window == null) {
                window = new WindowState(entry.getKey());
                windows.put(entry.getKey(), window);
                log("Created window " + formatPos(entry.getKey()));
            }
            window.fans = List.copyOf(entry.getValue());
            refreshObstacleField(client.world, window, tickCounter);
        }
        log("Active windows: " + windows.size());
    }

    private int fallbackScanFanBlocks(MinecraftClient client, Map<BlockPos, List<FanSource>> fansByWindow) {
        // log("Fallback scan: no FanBlockEntity found, scanning nearby blocks");
        if (client.world == null || client.player == null) {
            log("Fallback scan skipped: world/player missing");
            return 0;
        }
        BlockPos center = client.player.getBlockPos();
        int radius = 48;
        int minX = center.getX() - radius;
        int maxX = center.getX() + radius;
        int minY = center.getY() - radius;
        int maxY = center.getY() + radius;
        int minZ = center.getZ() - radius;
        int maxZ = center.getZ() + radius;
        for (int x = minX; x <= maxX; x++) {
            for (int y = minY; y <= maxY; y++) {
                for (int z = minZ; z <= maxZ; z++) {
                    BlockPos pos = new BlockPos(x, y, z);
                    BlockState state = client.world.getBlockState(pos);
                    if (!state.isOf(ModBlocks.FAN_BLOCK)) {
                        continue;
                    }
                    Direction facing = state.get(FanBlock.FACING);
                    BlockPos origin = alignToGrid(pos);
                    fansByWindow.computeIfAbsent(origin, key -> new ArrayList<>())
                        .add(new FanSource(pos, facing));
                }
            }
        }
        return fansByWindow.values().stream().mapToInt(List::size).sum();
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
        BlockPos inflowPos = fan.pos.offset(fan.facing);
        int cx = inflowPos.getX() - minX;
        int cy = inflowPos.getY() - minY;
        int cz = inflowPos.getZ() - minZ;

        float fanVx = fan.facing.getOffsetX() * INFLOW_SPEED;
        float fanVy = fan.facing.getOffsetY() * INFLOW_SPEED;
        float fanVz = fan.facing.getOffsetZ() * INFLOW_SPEED;

        int radius2 = FAN_RADIUS * FAN_RADIUS;
        switch (fan.facing.getAxis()) {
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

    private byte[] captureWindow(WindowState window, BackendMode backend) {
        int voxelCount = GRID_SIZE * GRID_SIZE * GRID_SIZE * CHANNELS;
        window.ensureBaseFieldInitialized();
        float[] obstacleField = window.obstacleField;
        BlockPos origin = window.origin;
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
                    // fan conditioning is rebuilt every tick from world fan blocks
                    window.baseField[idx + CH_FAN_MASK] = 0.0f;
                    window.baseField[idx + CH_FAN_VX] = 0.0f;
                    window.baseField[idx + CH_FAN_VY] = 0.0f;
                    window.baseField[idx + CH_FAN_VZ] = 0.0f;
                    // state inside solids must stay zero
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

    private boolean isSolidObstacle(World world, BlockPos pos) {
        BlockState state = world.getBlockState(pos);
        if (state.isAir()) {
            return false;
        }
        return !state.getCollisionShape(world, pos).isEmpty();
    }

    private void refreshObstacleField(World world, WindowState window, int tickNow) {
        if (world == null) {
            return;
        }
        if (window.obstacleField != null
            && window.lastObstacleRefreshTick >= 0
            && (tickNow - window.lastObstacleRefreshTick) < OBSTACLE_REFRESH_TICKS) {
            return;
        }

        int cellCount = GRID_SIZE * GRID_SIZE * GRID_SIZE;
        float[] obstacle = new float[cellCount];
        BlockPos origin = window.origin;
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

    private void updateBaseFieldFromResponse(WindowState window, float[] response, float speedCap) {
        if (window.baseField == null) {
            return;
        }
        int cellCount = GRID_SIZE * GRID_SIZE * GRID_SIZE;
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

    private void stopStreaming() {
        streamingEnabled = false;
        nativeBackend.shutdown();
        for (WindowState window : windows.values()) {
            closeSocket(window);
        }
        windows.clear();
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

    private float[] runSolverStep(WindowState window, byte[] payload, BackendMode backend) throws IOException {
        if (backend == BackendMode.NATIVE) {
            if (!nativeBackend.isLoaded()) {
                throw new IOException("Native backend not loaded: " + nativeBackend.getLoadError());
            }
            if (!nativeBackend.ensureInitialized(GRID_SIZE, CHANNELS, RESPONSE_CHANNELS)) {
                throw new IOException("Native backend initialization failed");
            }
            long contextKey = window.origin.asLong();
            float[] response = nativeBackend.step(payload, GRID_SIZE, RESPONSE_CHANNELS, contextKey);
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

    private void registerCommands() {
        ClientCommandRegistrationCallback.EVENT.register((dispatcher, registryAccess) -> dispatcher.register(
            ClientCommandManager.literal("aero")
                .then(ClientCommandManager.literal("start")
                    .executes(context -> {
                        streamingEnabled = true;
                        MinecraftClient client = MinecraftClient.getInstance();
                        if (client.world != null) {
                            refreshWindows(client);
                        }
                        log("Command /aero start: streaming enabled");
                        return 1;
                    }))
                .then(ClientCommandManager.literal("stop")
                    .executes(context -> {
                        stopStreaming();
                        log("Command /aero stop: streaming disabled");
                        return 1;
                    }))
                .then(ClientCommandManager.literal("maxspeed")
                    .then(ClientCommandManager.argument("value", FloatArgumentType.floatArg(0.0f, 64.0f))
                        .executes(context -> {
                            float value = FloatArgumentType.getFloat(context, "value");
                            maxWindSpeed = value;
                            log("Command /aero maxspeed: set to " + MathHelper.floor(value * 100.0f) / 100.0f);
                            return 1;
                        })))
                .then(ClientCommandManager.literal("backend")
                    .then(ClientCommandManager.literal("socket")
                        .executes(context -> {
                            backendMode = BackendMode.SOCKET;
                            nativeBackend.shutdown();
                            log("Command /aero backend: socket");
                            return 1;
                        }))
                    .then(ClientCommandManager.literal("native")
                        .executes(context -> {
                            if (!nativeBackend.isLoaded()) {
                                log("Native backend unavailable: " + nativeBackend.getLoadError());
                                return 0;
                            }
                            if (!nativeBackend.ensureInitialized(GRID_SIZE, CHANNELS, RESPONSE_CHANNELS)) {
                                log("Native backend init failed, keeping socket backend");
                                backendMode = BackendMode.SOCKET;
                                return 0;
                            }
                            backendMode = BackendMode.NATIVE;
                            for (WindowState window : windows.values()) {
                                closeSocket(window);
                            }
                            log("Command /aero backend: native (" + nativeBackend.runtimeInfo() + ")");
                            log("Native timing: " + nativeBackend.timingInfo());
                            return 1;
                        }))
                    .then(ClientCommandManager.literal("status")
                        .executes(context -> {
                            if (backendMode == BackendMode.SOCKET) {
                                log("Backend status: socket");
                                return 1;
                            }
                            log("Backend status: native (" + nativeBackend.runtimeInfo() + ")");
                            log("Native timing: " + nativeBackend.timingInfo());
                            return 1;
                        })))
                .then(ClientCommandManager.literal("debug")
                    .executes(context -> {
                        debugEnabled = !debugEnabled;
                        log("Command /aero debug: " + (debugEnabled ? "on" : "off"));
                        return 1;
                    }))
        ));
    }

    private void log(String message) {
        System.out.println(LOG_PREFIX + message);
    }

    private String formatPos(BlockPos pos) {
        return "(" + pos.getX() + ", " + pos.getY() + ", " + pos.getZ() + ")";
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
            // ignore
        }
        try {
            if (window.inputStream != null) {
                window.inputStream.close();
            }
        } catch (IOException ignored) {
            // ignore
        }
        try {
            if (window.socket != null) {
                window.socket.close();
            }
        } catch (IOException ignored) {
            // ignore
        }
        window.inputStream = null;
        window.outputStream = null;
        window.socket = null;
    }

    private static final class FanSource {
        private final BlockPos pos;
        private final Direction facing;

        private FanSource(BlockPos pos, Direction facing) {
            this.pos = pos;
            this.facing = facing;
        }
    }

    private static final class WindowState {
        private final BlockPos origin;
        private final FlowRenderer renderer;
        private final PhysicsHandler physicsHandler;
        private final AtomicBoolean busy = new AtomicBoolean(false);
        private Socket socket;
        private DataInputStream inputStream;
        private DataOutputStream outputStream;
        private float[] baseField;
        private volatile float[] obstacleField;
        private int lastObstacleRefreshTick = -1;
        private List<FanSource> fans = List.of();

        private WindowState(BlockPos origin) {
            this.origin = origin;
            this.renderer = new FlowRenderer(GRID_SIZE, INFLOW_SPEED);
            this.physicsHandler = new PhysicsHandler(renderer, GRID_SIZE);
        }

        private void ensureBaseFieldInitialized() {
            if (baseField != null) {
                return;
            }
            baseField = new float[GRID_SIZE * GRID_SIZE * GRID_SIZE * CHANNELS];
            for (int i = 0; i < baseField.length; i++) {
                baseField[i] = 0.0f;
            }
        }
    }
}
