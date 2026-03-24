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
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicLong;

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
    private static final int FLOW_COUNT = GRID_SIZE * GRID_SIZE * GRID_SIZE * RESPONSE_CHANNELS;

    private static final int WINDOW_REFRESH_TICKS = 20;
    private static final int FLOW_SYNC_INTERVAL_TICKS = 2;
    private static final int CHUNK_SIZE = 16;
    private static final int WINDOW_SECTION_COUNT = GRID_SIZE / CHUNK_SIZE;
    private static final int WINDOW_SECTION_VOLUME = WINDOW_SECTION_COUNT * WINDOW_SECTION_COUNT * WINDOW_SECTION_COUNT;
    private static final int SECTION_CELL_COUNT = CHUNK_SIZE * CHUNK_SIZE * CHUNK_SIZE;
    private static final int SOLVER_WORKER_COUNT = Math.max(2, Runtime.getRuntime().availableProcessors() / 2);
    private static final int TUNNEL_INFLOW_LAYERS = 2;
    private static final float DEFAULT_TUNNEL_SPEED = 8.0f;

    private static final int DEFAULT_STREAMLINE_STRIDE = 4;
    private static final int MIN_STREAMLINE_STRIDE = 1;
    private static final int MAX_STREAMLINE_STRIDE = 8;

    private static final float NATIVE_VELOCITY_SCALE = 30.0f;
    private static final float NATIVE_THERMAL_SOURCE_MAX = 0.006f;
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
    private static final double PLAYER_FORCE_STRENGTH = 0.02;
    private static final int WINDOW_EDGE_STABILIZATION_LAYERS = 8;
    private static final float WINDOW_EDGE_STABILIZATION_MIN_KEEP = 0.15f;

    private enum BackendMode {
        SOCKET,
        NATIVE
    }

    private static final AeroServerRuntime INSTANCE = new AeroServerRuntime();

    private final Map<WindowKey, WindowState> windows = new HashMap<>();
    private final NativeLbmBridge nativeBackend = new NativeLbmBridge();
    private final ActiveWindowWorldCache worldCache = new ActiveWindowWorldCache();
    private final ExecutorService solverExecutor = Executors.newFixedThreadPool(SOLVER_WORKER_COUNT, runnable -> {
        Thread thread = new Thread(runnable, "aero-server-solver");
        thread.setDaemon(true);
        return thread;
    });
    private final AtomicInteger activeSolveTasks = new AtomicInteger(0);
    private final AtomicLong runtimeGeneration = new AtomicLong(0L);

    private boolean streamingEnabled = false;
    private boolean debugEnabled = false;
    private float maxWindSpeed = INFLOW_SPEED;
    private boolean singleplayerClientMasterEnabled = false;
    private boolean renderVelocityVectorsEnabled = true;
    private boolean renderStreamlinesEnabled = true;
    private boolean tunnelModeEnabled = false;
    private float tunnelSpeed = DEFAULT_TUNNEL_SPEED;
    private int clientMasterDetectedWindows = 0;
    private int tickCounter = 0;
    private long simulationTicks = 0L;
    private int secondWindowTotalTicks = 0;
    private int secondWindowSimulationTicks = 0;
    private float simulationTicksPerSecond = 0.0f;
    private float lastMaxFlowSpeed = 0.0f;
    private volatile String lastSolverError = "";
    private int streamlineSampleStride = DEFAULT_STREAMLINE_STRIDE;
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
        ServerLifecycleEvents.SERVER_STOPPED.register(INSTANCE::shutdownAll);
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
                        stopStreaming(ctx.getSource().getServer());
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
                    })))
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
                            + " windows=" + statusWindowCount
                            + " renderVectors=" + renderVelocityVectorsEnabled
                            + " renderStreamlines=" + renderStreamlinesEnabled
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

        boolean clientMasterActive = isClientMasterActive(server);
        if (clientMasterActive) {
            if (tickCounter == 1 || tickCounter % WINDOW_REFRESH_TICKS == 0) {
                clientMasterDetectedWindows = scanFanSources(server).size();
            }
            if (!windows.isEmpty()) {
                for (Map.Entry<WindowKey, WindowState> entry : windows.entrySet()) {
                    WindowState window = entry.getValue();
                    ServerWorld world = server.getWorld(entry.getKey().worldKey());
                    deactivateWindow(world, entry.getKey().origin(), window);
                }
                windows.clear();
                worldCache.saveAll();
            }
            lastMaxFlowSpeed = 0.0f;
            updateSimulationRate(false);
            return;
        }

        boolean windowOriginShifted = hasWindowOriginShifted(server);
        if (windows.isEmpty() || tickCounter % WINDOW_REFRESH_TICKS == 0 || windowOriginShifted) {
            refreshWindows(server, tickCounter == 1);
        }
        clientMasterDetectedWindows = windows.size();

        if (windows.isEmpty()) {
            updateSimulationRate(false);
            lastMaxFlowSpeed = 0.0f;
            return;
        }

        boolean shouldSyncFlow = debugEnabled && (tickCounter % FLOW_SYNC_INTERVAL_TICKS == 0);
        boolean steppedThisTick = false;
        float maxSpeedThisTick = 0.0f;
        Iterator<Map.Entry<WindowKey, WindowState>> it = windows.entrySet().iterator();
        while (it.hasNext()) {
            Map.Entry<WindowKey, WindowState> entry = it.next();
            WindowKey key = entry.getKey();
            WindowState window = entry.getValue();
            ServerWorld world = server.getWorld(key.worldKey());
            if (world == null) {
                deactivateWindow(null, key.origin(), window);
                it.remove();
                continue;
            }

            SolveResult completed = consumeCompletedSolveResult(window, key.origin());
            if (completed != null) {
                applyStateFieldFromResponse(window, completed.flow());
                steppedThisTick = true;
                maxSpeedThisTick = Math.max(maxSpeedThisTick, completed.maxSpeed());
                lastSolverError = "";
            }

            if (window.sections != null) {
                applyForces(world, key.origin(), window, FORCE_STRENGTH, clientMasterActive);
                if (shouldSyncFlow) {
                    syncFlowToPlayers(world, key.origin(), window, streamlineSampleStride);
                }
            }

            if (!window.busy.get() && window.backendResetPending()) {
                resetWindowBackend(window);
            }
            if (window.busy.compareAndSet(false, true)) {
                SolveSnapshot snapshot = createSolveSnapshot(
                    window,
                    key.origin(),
                    backendMode,
                    maxWindSpeed,
                    runtimeGeneration.get()
                );
                activeSolveTasks.incrementAndGet();
                solverExecutor.execute(() -> runSolveTask(snapshot));
            }
        }
        if (steppedThisTick) {
            simulationTicks++;
        }
        if (steppedThisTick) {
            lastMaxFlowSpeed = maxSpeedThisTick;
        }
        updateSimulationRate(steppedThisTick);
    }

    private void switchBackend(BackendMode mode) {
        if (backendMode == mode) {
            return;
        }
        if (mode == BackendMode.SOCKET) {
            for (WindowState window : windows.values()) {
                if (!window.busy.get()) {
                    nativeBackend.releaseContext(window.nativeContextId);
                }
            }
        } else {
            for (WindowState window : windows.values()) {
                if (!window.busy.get()) {
                    closeSocket(window);
                }
            }
        }
        backendMode = mode;
        if (mode == BackendMode.NATIVE) {
            for (WindowState window : windows.values()) {
                window.markTemperatureRestorePending();
            }
        }
    }

    private void stopStreaming(MinecraftServer server) {
        runtimeGeneration.incrementAndGet();
        streamingEnabled = false;
        tickCounter = 0;
        simulationTicks = 0L;
        secondWindowTotalTicks = 0;
        secondWindowSimulationTicks = 0;
        simulationTicksPerSecond = 0.0f;
        lastMaxFlowSpeed = 0.0f;
        for (Map.Entry<WindowKey, WindowState> entry : windows.entrySet()) {
            WindowState window = entry.getValue();
            if (server != null) {
                ServerWorld world = server.getWorld(entry.getKey().worldKey());
                deactivateWindow(world, entry.getKey().origin(), window);
            } else {
                deactivateWindow(null, entry.getKey().origin(), window);
            }
        }
        windows.clear();
        waitForSolverIdle();
        clientMasterDetectedWindows = 0;
        if (server != null) {
            worldCache.saveAll();
            worldCache.flushSaves();
        }
        nativeBackend.shutdown();
    }

    private boolean isClientMasterActive(MinecraftServer server) {
        return singleplayerClientMasterEnabled
            && !server.isDedicated()
            && server.getPlayerManager().getPlayerList().size() == 1;
    }

    private void shutdownAll(MinecraftServer server) {
        stopStreaming(server);
    }

    private void refreshWindows(MinecraftServer server, boolean forceResample) {
        Map<WindowKey, List<FanSource>> fansByWindow = scanFanSources(server);
        Map<WindowKey, WindowState> nextWindows = new HashMap<>();
        Set<WindowKey> consumed = new HashSet<>();

        for (Map.Entry<WindowKey, List<FanSource>> entry : fansByWindow.entrySet()) {
            WindowKey key = entry.getKey();
            ServerWorld world = server.getWorld(key.worldKey());
            if (world == null) {
                continue;
            }

            WindowState window = windows.get(key);
            if (window != null) {
                consumed.add(key);
            } else {
                WindowKey slidingSourceKey = forceResample ? null : findSlidingSourceWindowKey(key, consumed);
                if (slidingSourceKey != null) {
                    window = windows.get(slidingSourceKey);
                    consumed.add(slidingSourceKey);
                    applyPendingSolveResult(window, slidingSourceKey.origin());
                    captureTemperatureStateFromNative(window);
                    if (!slideWindowSections(world, window, slidingSourceKey.origin(), key.origin())) {
                        saveWindowStateToCache(world, slidingSourceKey.origin(), window);
                        reloadWindowFromWorldState(world, key.origin(), window);
                    } else {
                        clearWindowPressure(window);
                        stabilizeWindowEdges(window);
                    }
                    markWindowBackendDirty(window);
                } else {
                    window = new WindowState(nextContextId++);
                    reloadWindowFromWorldState(world, key.origin(), window);
                }
            }

            if (forceResample) {
                refreshSampleFields(world, key.origin(), window, true);
            }

            window.fans = List.copyOf(entry.getValue());
            nextWindows.put(key, window);
        }

        for (Map.Entry<WindowKey, WindowState> entry : windows.entrySet()) {
            if (!consumed.contains(entry.getKey())) {
                WindowState window = entry.getValue();
                ServerWorld world = server.getWorld(entry.getKey().worldKey());
                deactivateWindow(world, entry.getKey().origin(), window);
            }
        }
        windows.clear();
        windows.putAll(nextWindows);
    }

    private boolean slideWindowSections(ServerWorld world, WindowState window, BlockPos oldOrigin, BlockPos newOrigin) {
        if (window.sections == null) {
            return false;
        }

        int deltaSectionX = Math.floorDiv(newOrigin.getX() - oldOrigin.getX(), CHUNK_SIZE);
        int deltaSectionY = Math.floorDiv(newOrigin.getY() - oldOrigin.getY(), CHUNK_SIZE);
        int deltaSectionZ = Math.floorDiv(newOrigin.getZ() - oldOrigin.getZ(), CHUNK_SIZE);
        if ((newOrigin.getX() - oldOrigin.getX()) % CHUNK_SIZE != 0
            || (newOrigin.getY() - oldOrigin.getY()) % CHUNK_SIZE != 0
            || (newOrigin.getZ() - oldOrigin.getZ()) % CHUNK_SIZE != 0
            || Math.abs(deltaSectionX) >= WINDOW_SECTION_COUNT
            || Math.abs(deltaSectionY) >= WINDOW_SECTION_COUNT
            || Math.abs(deltaSectionZ) >= WINDOW_SECTION_COUNT) {
            return false;
        }

        WindowSection[] previous = window.sections;
        WindowSection[] next = new WindowSection[WINDOW_SECTION_VOLUME];
        boolean[] reused = new boolean[WINDOW_SECTION_VOLUME];

        for (int sx = 0; sx < WINDOW_SECTION_COUNT; sx++) {
            for (int sy = 0; sy < WINDOW_SECTION_COUNT; sy++) {
                for (int sz = 0; sz < WINDOW_SECTION_COUNT; sz++) {
                    // Keep sections attached to world coordinates. When the window moves +X by one section,
                    // new local section 0 should reuse old local section 1 rather than old local section 0.
                    int oldSectionX = sx + deltaSectionX;
                    int oldSectionY = sy + deltaSectionY;
                    int oldSectionZ = sz + deltaSectionZ;
                    int newIndex = windowSectionIndex(sx, sy, sz);
                    if (oldSectionX >= 0 && oldSectionX < WINDOW_SECTION_COUNT
                        && oldSectionY >= 0 && oldSectionY < WINDOW_SECTION_COUNT
                        && oldSectionZ >= 0 && oldSectionZ < WINDOW_SECTION_COUNT) {
                        int oldIndex = windowSectionIndex(oldSectionX, oldSectionY, oldSectionZ);
                        next[newIndex] = previous[oldIndex];
                        reused[oldIndex] = true;
                        continue;
                    }

                    BlockPos newSectionOrigin = sectionOrigin(newOrigin, sx, sy, sz);
                    WindowSection section = sampleWindowSection(world, newSectionOrigin);
                    worldCache.loadSection(
                        world,
                        newSectionOrigin,
                        section.obstacle,
                        section.air,
                        section.state,
                        section.temperatureState,
                        RESPONSE_CHANNELS,
                        0,
                        1,
                        2,
                        3
                    );
                    next[newIndex] = section;
                }
            }
        }

        for (int sx = 0; sx < WINDOW_SECTION_COUNT; sx++) {
            for (int sy = 0; sy < WINDOW_SECTION_COUNT; sy++) {
                for (int sz = 0; sz < WINDOW_SECTION_COUNT; sz++) {
                    int oldIndex = windowSectionIndex(sx, sy, sz);
                    if (reused[oldIndex]) {
                        continue;
                    }
                    WindowSection section = previous[oldIndex];
                    if (section == null) {
                        continue;
                    }
                    worldCache.storeSection(
                        world,
                        sectionOrigin(oldOrigin, sx, sy, sz),
                        section.obstacle,
                        section.air,
                        section.state,
                        section.temperatureState,
                        RESPONSE_CHANNELS,
                        0,
                        1,
                        2,
                        3
                    );
                }
            }
        }

        window.sections = next;
        return true;
    }

    private WindowKey findSlidingSourceWindowKey(WindowKey target, Set<WindowKey> consumed) {
        WindowKey bestKey = null;
        int bestOverlap = -1;
        for (WindowKey candidate : windows.keySet()) {
            if (consumed.contains(candidate)) {
                continue;
            }
            if (!candidate.worldKey().equals(target.worldKey())) {
                continue;
            }
            WindowState candidateWindow = windows.get(candidate);
            if (candidateWindow == null || candidateWindow.detached() || candidateWindow.busy.get()) {
                continue;
            }
            int overlap = overlappingVolume(candidate.origin(), target.origin());
            if (overlap > bestOverlap) {
                bestOverlap = overlap;
                bestKey = candidate;
            }
        }
        if (bestOverlap <= 0) {
            return null;
        }
        return bestKey;
    }

    private int overlappingVolume(BlockPos a, BlockPos b) {
        int overlapX = GRID_SIZE - Math.abs(a.getX() - b.getX());
        int overlapY = GRID_SIZE - Math.abs(a.getY() - b.getY());
        int overlapZ = GRID_SIZE - Math.abs(a.getZ() - b.getZ());
        if (overlapX <= 0 || overlapY <= 0 || overlapZ <= 0) {
            return 0;
        }
        return overlapX * overlapY * overlapZ;
    }

    private boolean hasWindowOriginShifted(MinecraftServer server) {
        Set<WindowKey> expected = new HashSet<>();
        for (ServerWorld world : server.getWorlds()) {
            for (ServerPlayerEntity player : world.getPlayers()) {
                expected.add(new WindowKey(world.getRegistryKey(), centerWindowOnPlayer(player.getBlockPos())));
            }
        }
        return !expected.equals(windows.keySet());
    }

    private boolean loadWindowStateFromCache(ServerWorld world, BlockPos origin, WindowState window) {
        if (window.sections == null) {
            return false;
        }
        boolean loadedAny = false;
        for (int sx = 0; sx < WINDOW_SECTION_COUNT; sx++) {
            for (int sy = 0; sy < WINDOW_SECTION_COUNT; sy++) {
                for (int sz = 0; sz < WINDOW_SECTION_COUNT; sz++) {
                    WindowSection section = window.sectionAt(sx, sy, sz);
                    if (section == null) {
                        continue;
                    }
                    boolean loaded = worldCache.loadSection(
                        world,
                        sectionOrigin(origin, sx, sy, sz),
                        section.obstacle,
                        section.air,
                        section.state,
                        section.temperatureState,
                        RESPONSE_CHANNELS,
                        0,
                        1,
                        2,
                        3
                    );
                    loadedAny |= loaded;
                }
            }
        }
        return loadedAny;
    }

    private void reloadWindowFromWorldState(ServerWorld world, BlockPos origin, WindowState window) {
        refreshSampleFields(world, origin, window, true);
        loadWindowStateFromCache(world, origin, window);
        clearWindowPressure(window);
        stabilizeWindowEdges(window);
        window.markTemperatureRestorePending();
    }

    private void saveWindowStateToCache(ServerWorld world, BlockPos origin, WindowState window) {
        if (window.sections == null) {
            return;
        }
        for (int sx = 0; sx < WINDOW_SECTION_COUNT; sx++) {
            for (int sy = 0; sy < WINDOW_SECTION_COUNT; sy++) {
                for (int sz = 0; sz < WINDOW_SECTION_COUNT; sz++) {
                    WindowSection section = window.sectionAt(sx, sy, sz);
                    if (section == null) {
                        continue;
                    }
                    worldCache.storeSection(
                        world,
                        sectionOrigin(origin, sx, sy, sz),
                        section.obstacle,
                        section.air,
                        section.state,
                        section.temperatureState,
                        RESPONSE_CHANNELS,
                        0,
                        1,
                        2,
                        3
                    );
                }
            }
        }
    }

    private void clearWindowStateChannels(WindowState window) {
        if (window.sections == null) {
            return;
        }
        for (WindowSection section : window.sections) {
            if (section == null) {
                continue;
            }
            for (int i = 0; i < SECTION_CELL_COUNT; i++) {
                int idx = i * RESPONSE_CHANNELS;
                section.state[idx] = 0.0f;
                section.state[idx + 1] = 0.0f;
                section.state[idx + 2] = 0.0f;
                section.state[idx + 3] = 0.0f;
                section.temperatureState[i] = 0.0f;
            }
        }
        window.clearCompletedSolveResult();
        window.markTemperatureRestorePending();
    }

    private void clearWindowPressure(WindowState window) {
        if (window.sections == null) {
            return;
        }
        for (WindowSection section : window.sections) {
            if (section == null) {
                continue;
            }
            for (int i = 0; i < SECTION_CELL_COUNT; i++) {
                section.state[i * RESPONSE_CHANNELS + 3] = 0.0f;
            }
        }
    }

    private boolean captureTemperatureStateFromNative(WindowState window) {
        if (backendMode != BackendMode.NATIVE || window.sections == null || window.busy.get()) {
            return false;
        }
        if (!nativeBackend.isLoaded()) {
            return false;
        }
        if (!nativeBackend.ensureInitialized(GRID_SIZE, CHANNELS, RESPONSE_CHANNELS)) {
            return false;
        }
        float[] buffer = window.ensureTemperatureTransferBuffer();
        if (!nativeBackend.getTemperatureState(GRID_SIZE, window.nativeContextId, buffer)) {
            return false;
        }
        copyTemperatureBufferToSections(window, buffer);
        return true;
    }

    private boolean restoreTemperatureStateToNative(WindowState window) {
        if (backendMode != BackendMode.NATIVE || !window.temperatureRestorePending()) {
            return false;
        }
        if (window.sections == null) {
            window.clearTemperatureRestorePending();
            return false;
        }
        float[] buffer = window.ensureTemperatureTransferBuffer();
        fillTemperatureBufferFromSections(window, buffer);
        boolean restored = nativeBackend.setTemperatureState(GRID_SIZE, window.nativeContextId, buffer);
        if (restored) {
            window.clearTemperatureRestorePending();
        }
        return restored;
    }

    private void fillTemperatureBufferFromSections(WindowState window, float[] buffer) {
        int dst = 0;
        for (int sx = 0; sx < WINDOW_SECTION_COUNT; sx++) {
            for (int lx = 0; lx < CHUNK_SIZE; lx++) {
                for (int sy = 0; sy < WINDOW_SECTION_COUNT; sy++) {
                    for (int ly = 0; ly < CHUNK_SIZE; ly++) {
                        for (int sz = 0; sz < WINDOW_SECTION_COUNT; sz++) {
                            WindowSection section = window.sectionAt(sx, sy, sz);
                            for (int lz = 0; lz < CHUNK_SIZE; lz++) {
                                int localIndex = localSectionCellIndex(lx, ly, lz);
                                buffer[dst++] = section == null ? 0.0f : section.temperatureState[localIndex];
                            }
                        }
                    }
                }
            }
        }
    }

    private void copyTemperatureBufferToSections(WindowState window, float[] buffer) {
        int src = 0;
        for (int sx = 0; sx < WINDOW_SECTION_COUNT; sx++) {
            for (int lx = 0; lx < CHUNK_SIZE; lx++) {
                for (int sy = 0; sy < WINDOW_SECTION_COUNT; sy++) {
                    for (int ly = 0; ly < CHUNK_SIZE; ly++) {
                        for (int sz = 0; sz < WINDOW_SECTION_COUNT; sz++) {
                            WindowSection section = window.sectionAt(sx, sy, sz);
                            for (int lz = 0; lz < CHUNK_SIZE; lz++) {
                                int localIndex = localSectionCellIndex(lx, ly, lz);
                                float value = buffer[src++];
                                if (section != null) {
                                    section.temperatureState[localIndex] = value;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    private void stabilizeWindowEdges(WindowState window) {
        if (window.sections == null) {
            return;
        }
        for (int x = 0; x < GRID_SIZE; x++) {
            for (int y = 0; y < GRID_SIZE; y++) {
                for (int z = 0; z < GRID_SIZE; z++) {
                    int edgeDistance = Math.min(
                        Math.min(Math.min(x, y), z),
                        Math.min(Math.min(GRID_SIZE - 1 - x, GRID_SIZE - 1 - y), GRID_SIZE - 1 - z)
                    );
                    if (edgeDistance >= WINDOW_EDGE_STABILIZATION_LAYERS) {
                        continue;
                    }
                    float eta = (WINDOW_EDGE_STABILIZATION_LAYERS - edgeDistance) / (float) WINDOW_EDGE_STABILIZATION_LAYERS;
                    float keep = WINDOW_EDGE_STABILIZATION_MIN_KEEP
                        + (1.0f - WINDOW_EDGE_STABILIZATION_MIN_KEEP) * (1.0f - eta * eta);
                    WindowSection section = window.sectionAt(x / CHUNK_SIZE, y / CHUNK_SIZE, z / CHUNK_SIZE);
                    if (section == null) {
                        continue;
                    }
                    int localIndex = localSectionCellIndex(x % CHUNK_SIZE, y % CHUNK_SIZE, z % CHUNK_SIZE);
                    int stateIdx = localIndex * RESPONSE_CHANNELS;
                    section.state[stateIdx] *= keep;
                    section.state[stateIdx + 1] *= keep;
                    section.state[stateIdx + 2] *= keep;
                    section.state[stateIdx + 3] = 0.0f;
                    section.temperatureState[localIndex] *= keep;
                }
            }
        }
        window.markTemperatureRestorePending();
    }

    private SolveResult consumeCompletedSolveResult(WindowState window, BlockPos expectedOrigin) {
        SolveResult completed = window.consumeCompletedSolveResult();
        if (completed == null) {
            return null;
        }
        if (!completed.origin().equals(expectedOrigin)) {
            return null;
        }
        return completed;
    }

    private void applyPendingSolveResult(WindowState window, BlockPos expectedOrigin) {
        SolveResult completed = consumeCompletedSolveResult(window, expectedOrigin);
        if (completed != null) {
            applyStateFieldFromResponse(window, completed.flow());
        }
    }

    private void deactivateWindow(ServerWorld world, BlockPos origin, WindowState window) {
        applyPendingSolveResult(window, origin);
        captureTemperatureStateFromNative(window);
        if (world != null) {
            saveWindowStateToCache(world, origin, window);
        }
        window.markDetached();
        if (!window.busy.get()) {
            releaseWindow(window);
        }
    }

    private void markWindowBackendDirty(WindowState window) {
        window.markBackendResetPending();
        if (!window.busy.get()) {
            resetWindowBackend(window);
        }
    }

    private void resetWindowBackend(WindowState window) {
        closeSocket(window);
        nativeBackend.releaseContext(window.nativeContextId);
        window.clearBackendResetPending();
    }

    private Map<WindowKey, List<FanSource>> scanFanSources(MinecraftServer server) {
        Map<WindowKey, List<FanSource>> fansByWindow = new HashMap<>();
        for (ServerWorld world : server.getWorlds()) {
            for (ServerPlayerEntity player : world.getPlayers()) {
                BlockPos origin = centerWindowOnPlayer(player.getBlockPos());
                WindowKey key = new WindowKey(world.getRegistryKey(), origin);
                fansByWindow.computeIfAbsent(key, ignored -> new ArrayList<>());
            }
        }

        if (fansByWindow.isEmpty()) {
            return fansByWindow;
        }

        int margin = DUCT_JET_RANGE + FAN_RADIUS + 1;
        for (ServerWorld world : server.getWorlds()) {
            Set<Long> seenFans = new HashSet<>();
            Set<Long> visitedChunks = new HashSet<>();
            for (WindowKey key : fansByWindow.keySet()) {
                if (!key.worldKey().equals(world.getRegistryKey())) {
                    continue;
                }
                int minChunkX = Math.floorDiv(key.origin().getX() - margin, CHUNK_SIZE);
                int maxChunkX = Math.floorDiv(key.origin().getX() + GRID_SIZE + margin - 1, CHUNK_SIZE);
                int minChunkZ = Math.floorDiv(key.origin().getZ() - margin, CHUNK_SIZE);
                int maxChunkZ = Math.floorDiv(key.origin().getZ() + GRID_SIZE + margin - 1, CHUNK_SIZE);
                for (int chunkX = minChunkX; chunkX <= maxChunkX; chunkX++) {
                    for (int chunkZ = minChunkZ; chunkZ <= maxChunkZ; chunkZ++) {
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
                            addFanToContainingWindows(fansByWindow, world.getRegistryKey(), pos, fan);
                        }
                    }
                }
            }
        }
        return fansByWindow;
    }

    private void addFanToContainingWindows(
        Map<WindowKey, List<FanSource>> fansByWindow,
        RegistryKey<World> worldKey,
        BlockPos fanPos,
        FanSource fan
    ) {
        int margin = DUCT_JET_RANGE + FAN_RADIUS + 1;
        for (Map.Entry<WindowKey, List<FanSource>> entry : fansByWindow.entrySet()) {
            WindowKey key = entry.getKey();
            if (!key.worldKey().equals(worldKey)) {
                continue;
            }
            if (isInsideWindow(fanPos, key.origin(), margin)) {
                entry.getValue().add(fan);
            }
        }
    }

    private boolean isInsideWindow(BlockPos pos, BlockPos origin, int margin) {
        int minX = origin.getX() - margin;
        int minY = origin.getY() - margin;
        int minZ = origin.getZ() - margin;
        int maxX = origin.getX() + GRID_SIZE + margin;
        int maxY = origin.getY() + GRID_SIZE + margin;
        int maxZ = origin.getZ() + GRID_SIZE + margin;
        return pos.getX() >= minX && pos.getX() < maxX
            && pos.getY() >= minY && pos.getY() < maxY
            && pos.getZ() >= minZ && pos.getZ() < maxZ;
    }

    private BlockPos centerWindowOnPlayer(BlockPos pos) {
        int halfSections = WINDOW_SECTION_COUNT / 2;
        int x = (Math.floorDiv(pos.getX(), CHUNK_SIZE) - halfSections) * CHUNK_SIZE;
        int y = (Math.floorDiv(pos.getY(), CHUNK_SIZE) - halfSections) * CHUNK_SIZE;
        int z = (Math.floorDiv(pos.getZ(), CHUNK_SIZE) - halfSections) * CHUNK_SIZE;
        return new BlockPos(x, y, z);
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

    private BlockPos sectionOrigin(BlockPos windowOrigin, int sx, int sy, int sz) {
        return new BlockPos(
            windowOrigin.getX() + sx * CHUNK_SIZE,
            windowOrigin.getY() + sy * CHUNK_SIZE,
            windowOrigin.getZ() + sz * CHUNK_SIZE
        );
    }

    private boolean inBounds(int x, int y, int z) {
        return x >= 0 && y >= 0 && z >= 0 && x < GRID_SIZE && y < GRID_SIZE && z < GRID_SIZE;
    }

    private boolean inSectionBounds(int x, int y, int z) {
        return x >= 0 && y >= 0 && z >= 0 && x < CHUNK_SIZE && y < CHUNK_SIZE && z < CHUNK_SIZE;
    }

    private void applyFanAtVoxel(WindowState window, int x, int y, int z, float fanVx, float fanVy, float fanVz) {
        if (!inBounds(x, y, z)) {
            return;
        }
        int idx = voxelIndex(x, y, z);
        if (window.solveField[idx + CH_OBSTACLE] > 0.5f) {
            return;
        }
        window.solveField[idx + CH_FAN_MASK] = 1.0f;
        window.solveField[idx + CH_FAN_VX] += fanVx;
        window.solveField[idx + CH_FAN_VY] += fanVy;
        window.solveField[idx + CH_FAN_VZ] += fanVz;
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

    private SolveSnapshot createSolveSnapshot(
        WindowState window,
        BlockPos origin,
        BackendMode backend,
        float speedCap,
        long generation
    ) {
        return new SolveSnapshot(
            window,
            origin,
            List.copyOf(window.fans),
            window.copySectionRefs(),
            backend,
            speedCap,
            generation
        );
    }

    private PreparedPayload captureWindow(SolveSnapshot snapshot) {
        WindowState window = snapshot.window();
        int voxelCount = GRID_SIZE * GRID_SIZE * GRID_SIZE * CHANNELS;
        window.ensureSolveFieldInitialized();
        float[] solveField = window.solveField;
        WindowSection[] sections = snapshot.sections();
        int minX = snapshot.origin().getX();
        int minY = snapshot.origin().getY();
        int minZ = snapshot.origin().getZ();
        int payloadBytes = voxelCount * Float.BYTES;
        ByteBuffer buffer = window.payloadBuffer(payloadBytes, snapshot.backend());
        buffer.clear();

        for (int sx = 0; sx < WINDOW_SECTION_COUNT; sx++) {
            for (int lx = 0; lx < CHUNK_SIZE; lx++) {
                int x = sx * CHUNK_SIZE + lx;
                for (int sy = 0; sy < WINDOW_SECTION_COUNT; sy++) {
                    for (int ly = 0; ly < CHUNK_SIZE; ly++) {
                        int y = sy * CHUNK_SIZE + ly;
                        for (int sz = 0; sz < WINDOW_SECTION_COUNT; sz++) {
                            WindowSection section = sections == null ? null : sections[windowSectionIndex(sx, sy, sz)];
                            for (int lz = 0; lz < CHUNK_SIZE; lz++) {
                                int z = sz * CHUNK_SIZE + lz;
                                int localCell = localSectionCellIndex(lx, ly, lz);
                                int cell = (x * GRID_SIZE + y) * GRID_SIZE + z;
                                int idx = cell * CHANNELS;
                                int stateIdx = localCell * RESPONSE_CHANNELS;
                                boolean solid = section == null || section.obstacle[localCell] > 0.5f;
                                float sampledThermal = section == null ? 0.0f : section.thermal[localCell];
                                sampledThermal = MathHelper.clamp(sampledThermal, -NATIVE_THERMAL_SOURCE_MAX, NATIVE_THERMAL_SOURCE_MAX);

                                solveField[idx + CH_OBSTACLE] = solid ? 1.0f : 0.0f;
                                solveField[idx + CH_FAN_MASK] = 0.0f;
                                solveField[idx + CH_FAN_VX] = 0.0f;
                                solveField[idx + CH_FAN_VY] = 0.0f;
                                solveField[idx + CH_FAN_VZ] = 0.0f;
                                solveField[idx + CH_THERMAL_SOURCE] = solid ? 0.0f : sampledThermal;
                                if (solid || section == null) {
                                    solveField[idx + CH_STATE_VX] = 0.0f;
                                    solveField[idx + CH_STATE_VY] = 0.0f;
                                    solveField[idx + CH_STATE_VZ] = 0.0f;
                                    solveField[idx + CH_STATE_P] = 0.0f;
                                } else {
                                    solveField[idx + CH_STATE_VX] = section.state[stateIdx];
                                    solveField[idx + CH_STATE_VY] = section.state[stateIdx + 1];
                                    solveField[idx + CH_STATE_VZ] = section.state[stateIdx + 2];
                                    solveField[idx + CH_STATE_P] = section.state[stateIdx + 3];
                                }
                            }
                        }
                    }
                }
            }
        }

        for (FanSource fan : snapshot.fans()) {
            applyFanSource(window, fan, minX, minY, minZ);
        }
        applyTunnelInflow(window);

        float nativeInputScale = snapshot.backend() == BackendMode.NATIVE ? (1.0f / NATIVE_VELOCITY_SCALE) : 1.0f;
        int cellCount = GRID_SIZE * GRID_SIZE * GRID_SIZE;
        for (int i = 0; i < cellCount; i++) {
            int idx = i * CHANNELS;
            buffer.putFloat(solveField[idx + CH_OBSTACLE]);
            buffer.putFloat(solveField[idx + CH_FAN_MASK]);
            buffer.putFloat(solveField[idx + CH_FAN_VX]);
            buffer.putFloat(solveField[idx + CH_FAN_VY]);
            buffer.putFloat(solveField[idx + CH_FAN_VZ]);
            buffer.putFloat(solveField[idx + CH_STATE_VX] * nativeInputScale);
            buffer.putFloat(solveField[idx + CH_STATE_VY] * nativeInputScale);
            buffer.putFloat(solveField[idx + CH_STATE_VZ] * nativeInputScale);
            buffer.putFloat(solveField[idx + CH_STATE_P]);
            buffer.putFloat(solveField[idx + CH_THERMAL_SOURCE]);
        }

        if (snapshot.backend() == BackendMode.NATIVE) {
            return new PreparedPayload(null, buffer);
        }
        return new PreparedPayload(buffer.array(), null);
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

    private float sampleAmbientThermalBias(ServerWorld world) {
        return 0.0f;
    }

    private float sampleBlockThermalSource(ServerWorld world, BlockPos pos, BlockState state) {
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
        return MathHelper.clamp(source, -NATIVE_THERMAL_SOURCE_MAX, NATIVE_THERMAL_SOURCE_MAX);
    }

    private float sampleTerrainThermalFlux(ServerWorld world, BlockPos pos, BlockState state) {
        float flux = sampleAltitudeThermalFlux(world, pos);
        if (state.getFluidState().isIn(FluidTags.WATER)) {
            flux += THERMAL_SOURCE_WATER_COOLING;
        } else if (isStoneLikeTerrain(state)) {
            flux += THERMAL_SOURCE_STONE_HEATING;
        } else if (isBareTerrain(state)) {
            flux += THERMAL_SOURCE_BARE_HEATING;
        }
        return MathHelper.clamp(flux, -NATIVE_THERMAL_SOURCE_MAX, NATIVE_THERMAL_SOURCE_MAX);
    }

    private float sampleAltitudeThermalFlux(World world, BlockPos pos) {
        // float belowSeaLevel = Math.max(0.0f, world.getSeaLevel() - pos.getY());
        // float lapse = belowSeaLevel * THERMAL_SOURCE_ALTITUDE_LAPSE_PER_BLOCK;
        // return MathHelper.clamp(lapse, 0.0f, THERMAL_SOURCE_ALTITUDE_LAPSE_MAX);
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

    private void addSectionThermalSource(float[] thermal, int x, int y, int z, float source) {
        if (source == 0.0f || !inSectionBounds(x, y, z)) {
            return;
        }
        int cell = localSectionCellIndex(x, y, z);
        float updated = thermal[cell] + source;
        thermal[cell] = MathHelper.clamp(updated, -NATIVE_THERMAL_SOURCE_MAX, NATIVE_THERMAL_SOURCE_MAX);
    }

    private void emitSolidHeatToSection(float[] thermal, float[] obstacle, int x, int y, int z, float source) {
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
            if (!inSectionBounds(nx, ny, nz)) {
                continue;
            }
            int cell = localSectionCellIndex(nx, ny, nz);
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
            addSectionThermalSource(thermal, selectedX[i], selectedY[i], selectedZ[i], distributed);
        }
    }

    private WindowSection sampleWindowSection(ServerWorld world, BlockPos origin) {
        WindowSection section = new WindowSection();
        BlockPos.Mutable cursor = new BlockPos.Mutable();
        for (int x = 0; x < CHUNK_SIZE; x++) {
            for (int y = 0; y < CHUNK_SIZE; y++) {
                for (int z = 0; z < CHUNK_SIZE; z++) {
                    int cell = localSectionCellIndex(x, y, z);
                    cursor.set(origin.getX() + x, origin.getY() + y, origin.getZ() + z);
                    BlockState state = world.getBlockState(cursor);
                    boolean solid = isSolidObstacle(world, cursor, state);
                    section.obstacle[cell] = solid ? 1.0f : 0.0f;
                    section.air[cell] = state.isAir() ? 1.0f : 0.0f;
                }
            }
        }

        for (int x = -1; x <= CHUNK_SIZE; x++) {
            for (int y = -1; y <= CHUNK_SIZE; y++) {
                for (int z = -1; z <= CHUNK_SIZE; z++) {
                    cursor.set(origin.getX() + x, origin.getY() + y, origin.getZ() + z);
                    BlockState state = world.getBlockState(cursor);
                    float source = sampleBlockThermalSource(world, cursor, state);
                    if (source == 0.0f) {
                        continue;
                    }
                    boolean solid = isSolidObstacle(world, cursor, state);
                    if (!solid) {
                        addSectionThermalSource(section.thermal, x, y, z, source);
                        continue;
                    }
                    emitSolidHeatToSection(section.thermal, section.obstacle, x, y, z, source);
                }
            }
        }
        return section;
    }

    private void refreshSampleFields(ServerWorld world, BlockPos origin, WindowState window, boolean forceFullResample) {
        window.ambientThermalBias = sampleAmbientThermalBias(world);
        if (!forceFullResample && window.sections != null) {
            return;
        }

        window.ensureSectionsInitialized();
        for (int sx = 0; sx < WINDOW_SECTION_COUNT; sx++) {
            for (int sy = 0; sy < WINDOW_SECTION_COUNT; sy++) {
                for (int sz = 0; sz < WINDOW_SECTION_COUNT; sz++) {
                    window.setSection(sx, sy, sz, sampleWindowSection(world, sectionOrigin(origin, sx, sy, sz)));
                }
            }
        }
        clearWindowStateChannels(window);
    }

    private float[] runSolverStep(SolveSnapshot snapshot, PreparedPayload payload) throws IOException {
        if (snapshot.backend() == BackendMode.NATIVE) {
            if (!nativeBackend.isLoaded()) {
                throw new IOException("Native backend not loaded: " + nativeBackend.getLoadError());
            }
            if (!nativeBackend.ensureInitialized(GRID_SIZE, CHANNELS, RESPONSE_CHANNELS)) {
                throw new IOException("Native backend initialization failed");
            }
            restoreTemperatureStateToNative(snapshot.window());
            float[] response = nativeBackend.step(
                payload.nativePayload(),
                GRID_SIZE,
                RESPONSE_CHANNELS,
                snapshot.window().nativeContextId
            );
            if (response == null || response.length != FLOW_COUNT) {
                throw new IOException("Native backend returned invalid response");
            }
            scaleResponseVelocity(response, NATIVE_VELOCITY_SCALE);
            return response;
        }

        ensureSocket(snapshot.window());
        sendPayload(snapshot.window(), payload.socketPayload());
        return receiveFlowField(snapshot.window());
    }

    private float sanitizeSolveResponse(float[] response, float speedCap) {
        if (response == null) {
            return 0.0f;
        }

        int cellCount = GRID_SIZE * GRID_SIZE * GRID_SIZE;
        float capped = Math.max(0.0f, speedCap);
        float maxSpeedThisStep = 0.0f;
        for (int i = 0; i < cellCount; i++) {
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
        }
        return maxSpeedThisStep;
    }

    private void applyStateFieldFromResponse(WindowState window, float[] response) {
        if (window.sections == null || response == null) {
            return;
        }
        int responseIndex = 0;
        for (int sx = 0; sx < WINDOW_SECTION_COUNT; sx++) {
            for (int lx = 0; lx < CHUNK_SIZE; lx++) {
                for (int sy = 0; sy < WINDOW_SECTION_COUNT; sy++) {
                    for (int ly = 0; ly < CHUNK_SIZE; ly++) {
                        for (int sz = 0; sz < WINDOW_SECTION_COUNT; sz++) {
                            WindowSection section = window.sectionAt(sx, sy, sz);
                            if (section == null) {
                                responseIndex += CHUNK_SIZE * RESPONSE_CHANNELS;
                                continue;
                            }
                            for (int lz = 0; lz < CHUNK_SIZE; lz++) {
                                int stateIdx = localSectionCellIndex(lx, ly, lz) * RESPONSE_CHANNELS;
                                section.state[stateIdx] = response[responseIndex];
                                section.state[stateIdx + 1] = response[responseIndex + 1];
                                section.state[stateIdx + 2] = response[responseIndex + 2];
                                section.state[stateIdx + 3] = response[responseIndex + 3];
                                responseIndex += RESPONSE_CHANNELS;
                            }
                        }
                    }
                }
            }
        }
    }

    private void applyForces(ServerWorld world, BlockPos origin, WindowState window, double strength, boolean clientMasterActive) {
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
            Vec3d velocity = sampleVelocity(window, origin, center);
            double forceScale = isPlayer ? PLAYER_FORCE_STRENGTH : strength;
            Vec3d delta = velocity.multiply(forceScale);
            if (delta.lengthSquared() < 1e-10) {
                continue;
            }
            entity.addVelocity(delta.x, delta.y, delta.z);
        }
    }

    private Vec3d sampleVelocity(WindowState window, BlockPos origin, Vec3d worldPos) {
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

        Vec3d c000 = velocityAt(window, x0, y0, z0);
        Vec3d c100 = velocityAt(window, x1, y0, z0);
        Vec3d c010 = velocityAt(window, x0, y1, z0);
        Vec3d c110 = velocityAt(window, x1, y1, z0);
        Vec3d c001 = velocityAt(window, x0, y0, z1);
        Vec3d c101 = velocityAt(window, x1, y0, z1);
        Vec3d c011 = velocityAt(window, x0, y1, z1);
        Vec3d c111 = velocityAt(window, x1, y1, z1);

        Vec3d c00 = lerp(c000, c100, fx);
        Vec3d c10 = lerp(c010, c110, fx);
        Vec3d c01 = lerp(c001, c101, fx);
        Vec3d c11 = lerp(c011, c111, fx);

        Vec3d c0 = lerp(c00, c10, fy);
        Vec3d c1 = lerp(c01, c11, fy);
        return lerp(c0, c1, fz);
    }

    private Vec3d velocityAt(WindowState window, int x, int y, int z) {
        int cx = MathHelper.clamp(x, 0, GRID_SIZE - 1);
        int cy = MathHelper.clamp(y, 0, GRID_SIZE - 1);
        int cz = MathHelper.clamp(z, 0, GRID_SIZE - 1);
        WindowSection section = window.sectionAt(cx / CHUNK_SIZE, cy / CHUNK_SIZE, cz / CHUNK_SIZE);
        if (section == null) {
            return Vec3d.ZERO;
        }
        int localIndex = localSectionCellIndex(cx % CHUNK_SIZE, cy % CHUNK_SIZE, cz % CHUNK_SIZE) * RESPONSE_CHANNELS;
        return new Vec3d(section.state[localIndex], section.state[localIndex + 1], section.state[localIndex + 2]);
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

    private void syncFlowToPlayers(ServerWorld world, BlockPos origin, WindowState window, int stride) {
        int sampleStride = sanitizeStride(stride);
        float[] sampled = sampleFlow(window, sampleStride);
        Identifier dimId = world.getRegistryKey().getValue();
        AeroFlowPayload payload = new AeroFlowPayload(dimId, origin, sampleStride, sampled);

        for (ServerPlayerEntity player : world.getPlayers()) {
            ServerPlayNetworking.send(player, payload);
        }
    }

    private float[] sampleFlow(WindowState window, int stride) {
        int n = (GRID_SIZE + stride - 1) / stride;
        float[] sampled = new float[n * n * n * RESPONSE_CHANNELS];
        int dst = 0;

        for (int x = 0; x < n; x++) {
            int sx = Math.min(GRID_SIZE - 1, x * stride);
            for (int y = 0; y < n; y++) {
                int sy = Math.min(GRID_SIZE - 1, y * stride);
                for (int z = 0; z < n; z++) {
                    int sz = Math.min(GRID_SIZE - 1, z * stride);
                    WindowSection section = window.sectionAt(sx / CHUNK_SIZE, sy / CHUNK_SIZE, sz / CHUNK_SIZE);
                    if (section == null) {
                        sampled[dst] = 0.0f;
                        sampled[dst + 1] = 0.0f;
                        sampled[dst + 2] = 0.0f;
                        sampled[dst + 3] = 0.0f;
                    } else {
                        int src = localSectionCellIndex(sx % CHUNK_SIZE, sy % CHUNK_SIZE, sz % CHUNK_SIZE) * RESPONSE_CHANNELS;
                        sampled[dst] = section.state[src];
                        sampled[dst + 1] = section.state[src + 1];
                        sampled[dst + 2] = section.state[src + 2];
                        sampled[dst + 3] = section.state[src + 3];
                    }
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
                isClientMasterActive(server),
                backendModeId(backendMode),
                renderVelocityVectorsEnabled,
                renderStreamlinesEnabled
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
            if (window.sections == null) {
                continue;
            }

            float[] sampled = sampleFlow(window, stride);
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

    private void runSolveTask(SolveSnapshot snapshot) {
        WindowState window = snapshot.window();
        try {
            if (snapshot.generation() != runtimeGeneration.get()) {
                return;
            }
            PreparedPayload payload = captureWindow(snapshot);
            float[] response = runSolverStep(snapshot, payload);
            float maxSpeed = sanitizeSolveResponse(response, snapshot.speedCap());
            if (snapshot.generation() == runtimeGeneration.get()) {
                window.publishCompletedSolveResult(new SolveResult(response, maxSpeed, snapshot.origin()));
                lastSolverError = "";
            }
        } catch (IOException ex) {
            if (snapshot.generation() == runtimeGeneration.get()) {
                lastSolverError = ex.getMessage();
            }
            log("Solver error for window " + formatPos(snapshot.origin()) + ": " + ex.getMessage());
            closeSocket(window);
        } finally {
            window.busy.set(false);
            if (window.detached()) {
                releaseWindow(window);
            }
            activeSolveTasks.decrementAndGet();
        }
    }

    private void releaseWindow(WindowState window) {
        if (!window.markReleased()) {
            return;
        }
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

    private record SolveResult(float[] flow, float maxSpeed, BlockPos origin) {
    }

    private record SolveSnapshot(
        WindowState window,
        BlockPos origin,
        List<FanSource> fans,
        WindowSection[] sections,
        BackendMode backend,
        float speedCap,
        long generation
    ) {
    }

    private record PreparedPayload(byte[] socketPayload, ByteBuffer nativePayload) {
    }

    private static final class WindowState {
        private final long nativeContextId;
        private final AtomicBoolean busy = new AtomicBoolean(false);
        private final AtomicBoolean released = new AtomicBoolean(false);
        private Socket socket;
        private DataInputStream inputStream;
        private DataOutputStream outputStream;
        private ByteBuffer nativePayloadBuffer;
        private ByteBuffer socketPayloadBuffer;
        private SolveResult completedSolveResult;
        private WindowSection[] sections;
        private float[] solveField;
        private float[] temperatureTransferBuffer;
        private float ambientThermalBias = 0.0f;
        private List<FanSource> fans = List.of();
        private volatile boolean detached;
        private volatile boolean backendResetPending;
        private volatile boolean temperatureRestorePending = true;

        private WindowState(long nativeContextId) {
            this.nativeContextId = nativeContextId;
        }

        private void ensureSectionsInitialized() {
            if (sections != null) {
                return;
            }
            sections = new WindowSection[WINDOW_SECTION_VOLUME];
        }

        private void ensureSolveFieldInitialized() {
            if (solveField != null) {
                return;
            }
            solveField = new float[GRID_SIZE * GRID_SIZE * GRID_SIZE * CHANNELS];
        }

        private ByteBuffer payloadBuffer(int payloadBytes, BackendMode backend) {
            if (backend == BackendMode.NATIVE) {
                if (nativePayloadBuffer == null || nativePayloadBuffer.capacity() != payloadBytes) {
                    nativePayloadBuffer = ByteBuffer.allocateDirect(payloadBytes).order(ByteOrder.LITTLE_ENDIAN);
                }
                return nativePayloadBuffer;
            }
            if (socketPayloadBuffer == null || socketPayloadBuffer.capacity() != payloadBytes) {
                socketPayloadBuffer = ByteBuffer.allocate(payloadBytes).order(ByteOrder.LITTLE_ENDIAN);
            }
            return socketPayloadBuffer;
        }

        private float[] ensureTemperatureTransferBuffer() {
            int cellCount = GRID_SIZE * GRID_SIZE * GRID_SIZE;
            if (temperatureTransferBuffer == null || temperatureTransferBuffer.length != cellCount) {
                temperatureTransferBuffer = new float[cellCount];
            }
            return temperatureTransferBuffer;
        }

        private WindowSection sectionAt(int sx, int sy, int sz) {
            if (sections == null) {
                return null;
            }
            return sections[((sx * WINDOW_SECTION_COUNT + sy) * WINDOW_SECTION_COUNT + sz)];
        }

        private void setSection(int sx, int sy, int sz, WindowSection section) {
            ensureSectionsInitialized();
            sections[((sx * WINDOW_SECTION_COUNT + sy) * WINDOW_SECTION_COUNT + sz)] = section;
        }

        private WindowSection[] copySectionRefs() {
            return sections == null ? null : sections.clone();
        }

        private synchronized void publishCompletedSolveResult(SolveResult result) {
            if (detached) {
                return;
            }
            completedSolveResult = result;
        }

        private synchronized SolveResult consumeCompletedSolveResult() {
            SolveResult result = completedSolveResult;
            completedSolveResult = null;
            return result;
        }

        private synchronized void clearCompletedSolveResult() {
            completedSolveResult = null;
        }

        private void markDetached() {
            detached = true;
            clearCompletedSolveResult();
        }

        private boolean detached() {
            return detached;
        }

        private void markBackendResetPending() {
            backendResetPending = true;
            temperatureRestorePending = true;
        }

        private boolean backendResetPending() {
            return backendResetPending;
        }

        private void clearBackendResetPending() {
            backendResetPending = false;
        }

        private void markTemperatureRestorePending() {
            temperatureRestorePending = true;
        }

        private boolean temperatureRestorePending() {
            return temperatureRestorePending;
        }

        private void clearTemperatureRestorePending() {
            temperatureRestorePending = false;
        }

        private boolean markReleased() {
            return released.compareAndSet(false, true);
        }
    }

    private static final class WindowSection {
        private final float[] obstacle = new float[SECTION_CELL_COUNT];
        private final float[] air = new float[SECTION_CELL_COUNT];
        private final float[] thermal = new float[SECTION_CELL_COUNT];
        private final float[] temperatureState = new float[SECTION_CELL_COUNT];
        private final float[] state = new float[SECTION_CELL_COUNT * RESPONSE_CHANNELS];
    }
}
