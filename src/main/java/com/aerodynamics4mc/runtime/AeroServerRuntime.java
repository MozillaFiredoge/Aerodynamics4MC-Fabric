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
    private static final int TICKS_PER_SECOND = 20;
    private static final float DOMAIN_SIZE_METERS = 4.0f;
    private static final float SOLVER_STEP_SECONDS = 1.0f / TICKS_PER_SECOND;
    private static final float CELL_SIZE_METERS = DOMAIN_SIZE_METERS / GRID_SIZE;
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
    private static final int FLOW_COUNT = GRID_SIZE * GRID_SIZE * GRID_SIZE * RESPONSE_CHANNELS;

    private static final int WINDOW_REFRESH_TICKS = 20;
    private static final int OBSTACLE_REFRESH_TICKS = 200;
    private static final int FLOW_SYNC_INTERVAL_TICKS = 2;
    private static final int FAN_SCAN_RADIUS = 48;
    private static final int TUNNEL_INFLOW_LAYERS = 2;
    private static final float DEFAULT_TUNNEL_SPEED = 8.0f;

    private static final int DEFAULT_STREAMLINE_STRIDE = 4;
    private static final int MIN_STREAMLINE_STRIDE = 1;
    private static final int MAX_STREAMLINE_STRIDE = 8;

    private static final float NATIVE_VELOCITY_SCALE = 30.0f;
    private static final float BACKGROUND_VELOCITY_COUPLING = 0.08f;
    private static final double FORCE_STRENGTH = 0.02;
    private static final double PLAYER_FORCE_STRENGTH = 0.02;
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

    private boolean streamingEnabled = false;
    private boolean debugEnabled = false;
    private float maxWindSpeed = INFLOW_SPEED;
    private boolean singleplayerClientMasterEnabled = false;
    private boolean renderVelocityVectorsEnabled = true;
    private boolean renderStreamlinesEnabled = true;
    private boolean renderBackgroundVectorsEnabled = false;
    private boolean renderThermalAnomalyEnabled = false;
    private boolean tunnelModeEnabled = false;
    private float tunnelSpeed = DEFAULT_TUNNEL_SPEED;
    private int clientMasterDetectedWindows = 0;
    private int tickCounter = 0;
    private long simulationTicks = 0L;
    private int secondWindowTotalTicks = 0;
    private int secondWindowSimulationTicks = 0;
    private float simulationTicksPerSecond = 0.0f;
    private float lastMaxFlowSpeed = 0.0f;
    private String lastSolverError = "";
    private int streamlineSampleStride = DEFAULT_STREAMLINE_STRIDE;
    private boolean streamlineRoiEnabled = false;
    private int streamlineRoiMinX = 0;
    private int streamlineRoiMaxX = GRID_SIZE - 1;
    private int streamlineRoiMinY = 0;
    private int streamlineRoiMaxY = GRID_SIZE - 1;
    private int streamlineRoiMinZ = 0;
    private int streamlineRoiMaxZ = GRID_SIZE - 1;
    private BackendMode backendMode = BackendMode.NATIVE;
    private long nextContextId = 1L;

    private AeroServerRuntime() {
    }

    public static void init() {
        ServerTickEvents.END_SERVER_TICK.register(INSTANCE::onServerTick);
        ServerPlayConnectionEvents.JOIN.register((handler, sender, server) -> {
            INSTANCE.sendStateToPlayer(handler.player, server);
            INSTANCE.broadcastState(server);
            INSTANCE.sendFlowSnapshotToPlayer(handler.player, server);
        });
        ServerPlayConnectionEvents.DISCONNECT.register((handler, server) -> INSTANCE.broadcastState(server));
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
            .then(CommandManager.literal("tunnel")
                .then(CommandManager.literal("on")
                    .executes(ctx -> {
                        tunnelModeEnabled = true;
                        feedback(
                            ctx.getSource(),
                            "Tunnel mode enabled (dir=+X, speed=" + format2(tunnelSpeed) + ")"
                        );
                        broadcastState(ctx.getSource().getServer());
                        return 1;
                    }))
                .then(CommandManager.literal("off")
                    .executes(ctx -> {
                        tunnelModeEnabled = false;
                        feedback(ctx.getSource(), "Tunnel mode disabled");
                        broadcastState(ctx.getSource().getServer());
                        return 1;
                    }))
                .then(CommandManager.literal("speed")
                    .then(CommandManager.argument("value", FloatArgumentType.floatArg(0.0f, 64.0f))
                        .suggests((context, builder) -> CommandSource.suggestMatching(List.of("1", "2", "4", "8", "12"), builder))
                        .executes(ctx -> {
                            tunnelSpeed = FloatArgumentType.getFloat(ctx, "value");
                            feedback(ctx.getSource(), "Tunnel speed set to " + format2(tunnelSpeed));
                            return 1;
                        })))
                .then(CommandManager.literal("status")
                    .executes(ctx -> {
                        feedback(
                            ctx.getSource(),
                            "Tunnel mode=" + (tunnelModeEnabled ? "on" : "off")
                                + " dir=+X speed=" + format2(tunnelSpeed)
                        );
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
                    }))
                .then(CommandManager.literal("roi")
                    .then(CommandManager.literal("off")
                        .executes(ctx -> {
                            streamlineRoiEnabled = false;
                            feedback(ctx.getSource(), "Streamline ROI disabled (full window)");
                            broadcastState(ctx.getSource().getServer());
                            return 1;
                        }))
                    .then(CommandManager.literal("set")
                        .then(CommandManager.argument("minX", IntegerArgumentType.integer(0, GRID_SIZE - 1))
                            .then(CommandManager.argument("maxX", IntegerArgumentType.integer(0, GRID_SIZE - 1))
                                .then(CommandManager.argument("minY", IntegerArgumentType.integer(0, GRID_SIZE - 1))
                                    .then(CommandManager.argument("maxY", IntegerArgumentType.integer(0, GRID_SIZE - 1))
                                        .then(CommandManager.argument("minZ", IntegerArgumentType.integer(0, GRID_SIZE - 1))
                                            .then(CommandManager.argument("maxZ", IntegerArgumentType.integer(0, GRID_SIZE - 1))
                                                .executes(ctx -> {
                                                    int minX = IntegerArgumentType.getInteger(ctx, "minX");
                                                    int maxX = IntegerArgumentType.getInteger(ctx, "maxX");
                                                    int minY = IntegerArgumentType.getInteger(ctx, "minY");
                                                    int maxY = IntegerArgumentType.getInteger(ctx, "maxY");
                                                    int minZ = IntegerArgumentType.getInteger(ctx, "minZ");
                                                    int maxZ = IntegerArgumentType.getInteger(ctx, "maxZ");
                                                    if (minX > maxX || minY > maxY || minZ > maxZ) {
                                                        error(ctx.getSource(), "ROI min must be <= max on each axis");
                                                        return 0;
                                                    }
                                                    streamlineRoiEnabled = true;
                                                    streamlineRoiMinX = minX;
                                                    streamlineRoiMaxX = maxX;
                                                    streamlineRoiMinY = minY;
                                                    streamlineRoiMaxY = maxY;
                                                    streamlineRoiMinZ = minZ;
                                                    streamlineRoiMaxZ = maxZ;
                                                    feedback(ctx.getSource(), "Streamline ROI enabled " + formatStreamlineRoiBounds());
                                                    broadcastState(ctx.getSource().getServer());
                                                    return 1;
                                                }))))))))
                    .then(CommandManager.literal("status")
                        .executes(ctx -> {
                            feedback(
                                ctx.getSource(),
                                "Streamline ROI=" + (streamlineRoiEnabled ? "on " + formatStreamlineRoiBounds() : "off (full window)")
                            );
                            return 1;
                        })))
                )
            .then(CommandManager.literal("backend")
                .then(CommandManager.literal("socket")
                    .executes(ctx -> {
                        switchBackend(BackendMode.SOCKET);
                        feedback(ctx.getSource(), "Backend set to socket");
                        broadcastState(ctx.getSource().getServer());
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
                        broadcastState(ctx.getSource().getServer());
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
            .then(CommandManager.literal("clientmaster")
                .then(CommandManager.literal("on")
                    .executes(ctx -> {
                        singleplayerClientMasterEnabled = true;
                        boolean active = isClientMasterActive(ctx.getSource().getServer());
                        feedback(
                            ctx.getSource(),
                            "Singleplayer client master enabled" + (active ? " (active)" : " (inactive: not singleplayer)")
                        );
                        broadcastState(ctx.getSource().getServer());
                        return 1;
                    }))
                .then(CommandManager.literal("off")
                    .executes(ctx -> {
                        singleplayerClientMasterEnabled = false;
                        feedback(ctx.getSource(), "Singleplayer client master disabled");
                        broadcastState(ctx.getSource().getServer());
                        return 1;
                    }))
                .then(CommandManager.literal("status")
                    .executes(ctx -> {
                        boolean active = isClientMasterActive(ctx.getSource().getServer());
                        feedback(
                            ctx.getSource(),
                            "Singleplayer client master="
                                + (singleplayerClientMasterEnabled ? "enabled" : "disabled")
                                + " active=" + active
                        );
                        return 1;
                    })))
            .then(CommandManager.literal("status")
                .executes(ctx -> {
                    int statusWindowCount = isClientMasterActive(ctx.getSource().getServer())
                        ? clientMasterDetectedWindows
                        : windows.size();
                    feedback(
                        ctx.getSource(),
                        "Status streaming=" + streamingEnabled
                            + " debug=" + debugEnabled
                            + " maxspeed=" + format2(maxWindSpeed)
                            + " tunnel=" + (tunnelModeEnabled ? "on" : "off")
                            + "@+X:" + format2(tunnelSpeed)
                            + " box=" + format2(DOMAIN_SIZE_METERS) + "m"
                            + " n=" + GRID_SIZE
                            + " dx=" + format4(CELL_SIZE_METERS) + "m"
                            + " dt=" + format3(SOLVER_STEP_SECONDS) + "s"
                            + " stride=" + streamlineSampleStride
                            + " streamlineRoi=" + (streamlineRoiEnabled ? formatStreamlineRoiBounds() : "off")
                            + " windows=" + statusWindowCount
                            + " renderVectors=" + renderVelocityVectorsEnabled
                            + " renderStreamlines=" + renderStreamlinesEnabled
                            + " renderBackgroundVectors=" + renderBackgroundVectorsEnabled
                            + " renderThermalAnomaly=" + renderThermalAnomalyEnabled
                            + " simTicks=" + simulationTicks
                            + " simTickPerSec=" + format2(simulationTicksPerSecond)
                            + " clientMaster=" + (singleplayerClientMasterEnabled ? "on" : "off")
                            + "(active=" + isClientMasterActive(ctx.getSource().getServer()) + ")"
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
            .then(CommandManager.literal("render")
                .then(CommandManager.literal("vectors")
                    .then(CommandManager.literal("on")
                        .executes(ctx -> {
                            renderVelocityVectorsEnabled = true;
                            feedback(ctx.getSource(), "Velocity vectors rendering enabled");
                            broadcastState(ctx.getSource().getServer());
                            return 1;
                        }))
                    .then(CommandManager.literal("off")
                        .executes(ctx -> {
                            renderVelocityVectorsEnabled = false;
                            feedback(ctx.getSource(), "Velocity vectors rendering disabled");
                            broadcastState(ctx.getSource().getServer());
                            return 1;
                        }))
                    .then(CommandManager.literal("status")
                        .executes(ctx -> {
                            feedback(ctx.getSource(), "Velocity vectors rendering=" + renderVelocityVectorsEnabled);
                            return 1;
                        })))
                .then(CommandManager.literal("streamlines")
                    .then(CommandManager.literal("on")
                        .executes(ctx -> {
                            renderStreamlinesEnabled = true;
                            feedback(ctx.getSource(), "Streamlines rendering enabled");
                            broadcastState(ctx.getSource().getServer());
                            return 1;
                        }))
                    .then(CommandManager.literal("off")
                        .executes(ctx -> {
                            renderStreamlinesEnabled = false;
                            feedback(ctx.getSource(), "Streamlines rendering disabled");
                            broadcastState(ctx.getSource().getServer());
                            return 1;
                        }))
                    .then(CommandManager.literal("status")
                        .executes(ctx -> {
                            feedback(ctx.getSource(), "Streamlines rendering=" + renderStreamlinesEnabled);
                            return 1;
                        })))
                .then(CommandManager.literal("background")
                    .then(CommandManager.literal("on")
                        .executes(ctx -> {
                            renderBackgroundVectorsEnabled = true;
                            feedback(ctx.getSource(), "Background vectors rendering enabled");
                            broadcastState(ctx.getSource().getServer());
                            return 1;
                        }))
                    .then(CommandManager.literal("off")
                        .executes(ctx -> {
                            renderBackgroundVectorsEnabled = false;
                            feedback(ctx.getSource(), "Background vectors rendering disabled");
                            broadcastState(ctx.getSource().getServer());
                            return 1;
                        }))
                    .then(CommandManager.literal("status")
                        .executes(ctx -> {
                            feedback(ctx.getSource(), "Background vectors rendering=" + renderBackgroundVectorsEnabled);
                            return 1;
                        })))
                .then(CommandManager.literal("thermal")
                    .then(CommandManager.literal("on")
                        .executes(ctx -> {
                            renderThermalAnomalyEnabled = true;
                            feedback(ctx.getSource(), "Thermal anomaly rendering enabled");
                            broadcastState(ctx.getSource().getServer());
                            return 1;
                        }))
                    .then(CommandManager.literal("off")
                        .executes(ctx -> {
                            renderThermalAnomalyEnabled = false;
                            feedback(ctx.getSource(), "Thermal anomaly rendering disabled");
                            broadcastState(ctx.getSource().getServer());
                            return 1;
                        }))
                    .then(CommandManager.literal("status")
                        .executes(ctx -> {
                            feedback(ctx.getSource(), "Thermal anomaly rendering=" + renderThermalAnomalyEnabled);
                            return 1;
                        }))))
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

        boolean clientMasterActive = isClientMasterActive(server);
        if (clientMasterActive) {
            if (tickCounter == 1 || tickCounter % WINDOW_REFRESH_TICKS == 0) {
                clientMasterDetectedWindows = scanFanSources(server).size();
            }
            if (!windows.isEmpty()) {
                for (WindowState window : windows.values()) {
                    releaseWindow(window);
                }
                windows.clear();
            }
            lastMaxFlowSpeed = 0.0f;
            entityVelocitySyncTickById.clear();
            entityLastSyncedVelocityById.clear();
            backgroundField.clear();
            updateSimulationRate(false);
            return;
        }

        if (windows.isEmpty() || tickCounter % WINDOW_REFRESH_TICKS == 0) {
            refreshWindows(server);
        }
        clientMasterDetectedWindows = windows.size();

        if (windows.isEmpty()) {
            backgroundField.clear();
            updateSimulationRate(false);
            return;
        }

        updateBackgroundFields(server);

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
                byte[] payload = captureWindow(window, key.worldKey().getValue(), key.origin(), backendMode);
                float[] response = runSolverStep(window, payload, backendMode);
                updateBaseFieldFromResponse(window, response, maxWindSpeed);
                window.latestFlow = response;
                steppedThisTick = true;
                lastSolverError = "";

                applyForces(world, key.origin(), response, FORCE_STRENGTH, clientMasterActive);
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
        clientMasterDetectedWindows = 0;
        backgroundField.clear();
        nativeBackend.shutdown();
    }

    private boolean isClientMasterActive(MinecraftServer server) {
        return singleplayerClientMasterEnabled
            && !server.isDedicated()
            && server.getPlayerManager().getPlayerList().size() == 1;
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

    private void updateBackgroundFields(MinecraftServer server) {
        Map<Identifier, List<BlockPos>> originsByDimension = new HashMap<>();
        for (WindowKey key : windows.keySet()) {
            originsByDimension
                .computeIfAbsent(key.worldKey().getValue(), ignored -> new ArrayList<>())
                .add(key.origin());
        }

        Set<Identifier> activeDimensions = new HashSet<>();
        for (ServerWorld world : server.getWorlds()) {
            Identifier dimensionId = world.getRegistryKey().getValue();
            List<BlockPos> origins = originsByDimension.get(dimensionId);
            if (origins == null || origins.isEmpty()) {
                continue;
            }
            backgroundField.update(
                dimensionId,
                world.getTimeOfDay(),
                tickCounter,
                origins,
                GRID_SIZE,
                (sampleX, sampleY, sampleZ) -> WorldThermalSampler.sampleAveragedAnomaly(world, sampleX, sampleY, sampleZ)
            );
            activeDimensions.add(dimensionId);
        }
        backgroundField.retainDimensions(activeDimensions);
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

            if (tunnelModeEnabled) {
                for (BlockPos playerPos : playerPositions) {
                    BlockPos origin = alignToGrid(playerPos);
                    WindowKey key = new WindowKey(world.getRegistryKey(), origin);
                    fansByWindow.computeIfAbsent(key, ignored -> new ArrayList<>());
                }
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
                            WindowKey key = new WindowKey(world.getRegistryKey(), origin);
                            fansByWindow.computeIfAbsent(key, ignored -> new ArrayList<>()).add(new FanSource(pos, facing, ductLength));
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

    private int computeDuctLength(ServerWorld world, BlockPos fanPos, Direction facing) {
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

    private boolean isDuctSegment(ServerWorld world, BlockPos center, Direction facing) {
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

    private void applyTunnelInflow(WindowState window) {
        if (!tunnelModeEnabled || tunnelSpeed <= 0.0f) {
            return;
        }

        int layers = Math.max(1, Math.min(TUNNEL_INFLOW_LAYERS, GRID_SIZE));
        for (int x = 0; x < layers; x++) {
            for (int y = 0; y < GRID_SIZE; y++) {
                for (int z = 0; z < GRID_SIZE; z++) {
                    applyFanAtVoxel(window, x, y, z, tunnelSpeed, 0.0f, 0.0f);
                }
            }
        }
    }

    private byte[] captureWindow(WindowState window, Identifier dimensionId, BlockPos origin, BackendMode backend) {
        int voxelCount = GRID_SIZE * GRID_SIZE * GRID_SIZE * CHANNELS;
        window.ensureBaseFieldInitialized();
        float[] obstacleField = window.obstacleField;
        int minX = origin.getX();
        int minY = origin.getY();
        int minZ = origin.getZ();
        ByteBuffer buffer = ByteBuffer.allocate(voxelCount * Float.BYTES).order(ByteOrder.LITTLE_ENDIAN);
        float[] backgroundSample = new float[4];

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
        applyTunnelInflow(window);

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
        if (state.isAir() || state.isOf(ModBlocks.DUCT_BLOCK)) {
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

    private boolean stepBackgroundFlow(int gridSize, byte[] payload, long contextId, float[] output) {
        if (!nativeBackend.isLoaded()) {
            return false;
        }
        if (!nativeBackend.ensureInitialized(gridSize, BackgroundFieldGrid.SOLVER_INPUT_CHANNELS, BackgroundFieldGrid.SOLVER_OUTPUT_CHANNELS)) {
            return false;
        }
        float[] solved = nativeBackend.step(payload, gridSize, BackgroundFieldGrid.SOLVER_OUTPUT_CHANNELS, contextId);
        if (solved == null || solved.length != output.length) {
            return false;
        }
        System.arraycopy(solved, 0, output, 0, output.length);
        scaleResponseVelocity(output, NATIVE_VELOCITY_SCALE);
        return true;
    }

    private void releaseBackgroundFlowContext(long contextId) {
        if (!nativeBackend.isLoaded()) {
            return;
        }
        nativeBackend.releaseContext(contextId);
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

    private void applyForces(ServerWorld world, BlockPos origin, float[] response, double strength, boolean clientMasterActive) {
        Box box = new Box(
            origin.getX(),
            origin.getY(),
            origin.getZ(),
            origin.getX() + GRID_SIZE,
            origin.getY() + GRID_SIZE,
            origin.getZ() + GRID_SIZE
        );

        for (Entity entity : world.getEntitiesByClass(Entity.class, box, e -> e.isAlive() && !e.isSpectator())) {
            boolean isPlayer = entity instanceof ServerPlayerEntity;
            if (clientMasterActive && isPlayer) {
                // Singleplayer client-master mode: player acceleration is predicted client-side.
                continue;
            }
            Vec3d center = entity.getBoundingBox().getCenter();
            Vec3d velocity = sampleVelocity(response, origin, center);
            double forceScale = isPlayer ? PLAYER_FORCE_STRENGTH : strength;
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

    private void sendStateToPlayer(ServerPlayerEntity player, MinecraftServer server) {
        ServerPlayNetworking.send(
            player,
            new AeroRuntimeStatePayload(
                streamingEnabled,
                debugEnabled,
                maxWindSpeed,
                streamlineSampleStride,
                streamlineRoiEnabled,
                streamlineRoiMinX,
                streamlineRoiMaxX,
                streamlineRoiMinY,
                streamlineRoiMaxY,
                streamlineRoiMinZ,
                streamlineRoiMaxZ,
                isClientMasterActive(server),
                backendModeId(backendMode),
                renderVelocityVectorsEnabled,
                renderStreamlinesEnabled,
                renderBackgroundVectorsEnabled,
                renderThermalAnomalyEnabled
            )
        );
    }

    private void broadcastState(MinecraftServer server) {
        for (ServerPlayerEntity player : server.getPlayerManager().getPlayerList()) {
            sendStateToPlayer(player, server);
        }
    }

    private void sendFlowSnapshotToPlayer(ServerPlayerEntity player, MinecraftServer server) {
        if (!debugEnabled) {
            return;
        }
        if (server != null && isClientMasterActive(server)) {
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

    private String formatStreamlineRoiBounds() {
        return "x=[" + streamlineRoiMinX + "," + streamlineRoiMaxX + "]"
            + " y=[" + streamlineRoiMinY + "," + streamlineRoiMaxY + "]"
            + " z=[" + streamlineRoiMinZ + "," + streamlineRoiMaxZ + "]";
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

    private int backendModeId(BackendMode mode) {
        return mode == BackendMode.NATIVE ? 1 : 0;
    }

    private record WindowKey(RegistryKey<World> worldKey, BlockPos origin) {
    }

    private record FanSource(BlockPos pos, Direction facing, int ductLength) {
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
