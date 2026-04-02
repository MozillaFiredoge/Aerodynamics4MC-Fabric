package com.aerodynamics4mc.client;

import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;

import com.aerodynamics4mc.net.AeroFlowPayload;
import com.aerodynamics4mc.net.AeroRuntimeStatePayload;

import net.fabricmc.api.ClientModInitializer;
import net.fabricmc.fabric.api.client.event.lifecycle.v1.ClientTickEvents;
import net.fabricmc.fabric.api.client.networking.v1.ClientPlayConnectionEvents;
import net.fabricmc.fabric.api.client.networking.v1.ClientPlayNetworking;
import net.fabricmc.fabric.api.client.rendering.v1.world.WorldRenderContext;
import net.fabricmc.fabric.api.client.rendering.v1.world.WorldRenderEvents;
import net.minecraft.client.MinecraftClient;
import net.minecraft.util.Identifier;
import net.minecraft.util.math.BlockPos;
import net.minecraft.util.math.Vec3d;

public class AeroClientMod implements ClientModInitializer {
    private static final int GRID_SIZE = 64;
    private static final float DEFAULT_MAX_INFLOW_SPEED = 8.0f;
    private static final int DEFAULT_BACKEND_MODE = 1;
    private static final int DEFAULT_STREAMLINE_STRIDE = 4;
    private static final double PLAYER_PREDICTION_FORCE_STRENGTH = 0.02;
    private static final double PARTICLE_FORCE_STRENGTH = 0.02;
    private static final int REMOTE_WINDOW_STALE_TICKS = 20;
    private static final int REGION_HALO_CELLS = GRID_SIZE / 4;
    private static final int REGION_CORE_SIZE = GRID_SIZE - REGION_HALO_CELLS * 2;

    private final Map<WindowKey, WindowView> remoteWindows = new HashMap<>();
    private final ClientFluidRuntime clientFluidRuntime = new ClientFluidRuntime(GRID_SIZE);

    private boolean streamingEnabled = false;
    private boolean debugEnabled = false;
    private boolean clientPlayerAuthority = false;
    private float maxWindSpeed = DEFAULT_MAX_INFLOW_SPEED;
    private int streamlineSampleStride = DEFAULT_STREAMLINE_STRIDE;
    private int backendMode = DEFAULT_BACKEND_MODE;
    private boolean renderVelocityVectors = true;
    private boolean renderStreamlines = true;
    private long clientTickCounter = 0L;

    @Override
    public void onInitializeClient() {
        WorldRenderEvents.BEFORE_DEBUG_RENDER.register(this::onRender);
        ClientTickEvents.END_CLIENT_TICK.register(this::onClientTick);
        ClientPlayNetworking.registerGlobalReceiver(AeroRuntimeStatePayload.ID, this::onRuntimeState);
        ClientPlayNetworking.registerGlobalReceiver(AeroFlowPayload.ID, this::onFlowField);
        ClientPlayConnectionEvents.DISCONNECT.register((handler, client) -> clearState());
    }

    private void onClientTick(MinecraftClient client) {
        clientTickCounter++;
        if (!streamingEnabled || client.world == null || client.player == null || client.player.isSpectator()) {
            return;
        }
        if (clientPlayerAuthority) {
            clientFluidRuntime.tick(client);
            return;
        }

        pruneStaleRemoteWindows();

        Identifier dimensionId = client.world.getRegistryKey().getValue();
        Vec3d center = client.player.getBoundingBox().getCenter();
        WindowView window = findWindowView(dimensionId, center);
        if (window == null) {
            return;
        }

        Vec3d windVelocity = window.renderer().sampleVelocity(center);
        Vec3d delta = windVelocity.multiply(PLAYER_PREDICTION_FORCE_STRENGTH);
        if (delta.lengthSquared() < 1e-10) {
            window.physics().updateOrigin(window.origin());
            window.physics().applyParticleForcesOnly(client, PARTICLE_FORCE_STRENGTH);
            window.physics().tickVisualFeedback(client, clientTickCounter);
            return;
        }
        client.player.addVelocity(delta.x, delta.y, delta.z);
        window.physics().updateOrigin(window.origin());
        window.physics().applyParticleForcesOnly(client, PARTICLE_FORCE_STRENGTH);
        window.physics().tickVisualFeedback(client, clientTickCounter);
    }

    private void onRender(WorldRenderContext context) {
        if (!streamingEnabled || !debugEnabled) {
            return;
        }
        MinecraftClient client = MinecraftClient.getInstance();
        if (client.world == null) {
            return;
        }
        if (clientPlayerAuthority) {
            clientFluidRuntime.render(context);
            return;
        }

        Identifier currentDimension = client.world.getRegistryKey().getValue();
        Vec3d center = client.player != null ? client.player.getBoundingBox().getCenter() : client.gameRenderer.getCamera().getCameraPos();
        WindowView view = findWindowView(currentDimension, center);
        if (view != null) {
            FlowRenderer renderer = view.renderer();
            renderer.setMaxInflowSpeed(maxWindSpeed);
            renderer.setStreamlineSampleStride(streamlineSampleStride);
            renderer.setRenderVelocityVectors(renderVelocityVectors);
            renderer.setRenderStreamlines(renderStreamlines);
            renderer.render(context);
        }
    }

    private void onRuntimeState(AeroRuntimeStatePayload payload, ClientPlayNetworking.Context context) {
        context.client().execute(() -> {
            streamingEnabled = payload.streamingEnabled();
            debugEnabled = payload.debugEnabled();
            clientPlayerAuthority = payload.clientPlayerAuthority();
            maxWindSpeed = Math.max(1e-6f, payload.maxWindSpeed());
            streamlineSampleStride = sanitizeStride(payload.streamlineStride());
            backendMode = payload.backendMode();
            renderVelocityVectors = payload.renderVelocityVectors();
            renderStreamlines = payload.renderStreamlines();
            clientFluidRuntime.setMaxWindSpeed(maxWindSpeed);
            clientFluidRuntime.setStreamlineSampleStride(streamlineSampleStride);
            clientFluidRuntime.setBackendModeId(backendMode);
            clientFluidRuntime.setRenderVelocityVectors(renderVelocityVectors);
            clientFluidRuntime.setRenderStreamlines(renderStreamlines);

            for (WindowView window : remoteWindows.values()) {
                FlowRenderer renderer = window.renderer();
                renderer.setMaxInflowSpeed(maxWindSpeed);
                renderer.setStreamlineSampleStride(streamlineSampleStride);
                renderer.setRenderVelocityVectors(renderVelocityVectors);
                renderer.setRenderStreamlines(renderStreamlines);
            }

            if (!streamingEnabled) {
                remoteWindows.clear();
                clientFluidRuntime.clear();
                return;
            }

            if (clientPlayerAuthority) {
                remoteWindows.clear();
            } else {
                clientFluidRuntime.clear();
            }
        });
    }

    private void onFlowField(AeroFlowPayload payload, ClientPlayNetworking.Context context) {
        context.client().execute(() -> {
            if (clientPlayerAuthority) {
                return;
            }
            int stride = sanitizeStride(payload.sampleStride());
            WindowKey key = new WindowKey(payload.dimensionId(), payload.origin());
            WindowView view = remoteWindows.computeIfAbsent(
                key,
                ignored -> new WindowView(new FlowRenderer(GRID_SIZE, maxWindSpeed), clientTickCounter)
            );
            view.lastSeenTick = clientTickCounter;
            FlowRenderer renderer = view.renderer();
            renderer.setMaxInflowSpeed(maxWindSpeed);
            renderer.setStreamlineSampleStride(streamlineSampleStride);
            renderer.setRenderVelocityVectors(renderVelocityVectors);
            renderer.setRenderStreamlines(renderStreamlines);
            renderer.updateFlowField(payload.origin(), stride, payload.flow());
        });
    }

    private void pruneStaleRemoteWindows() {
        if (remoteWindows.isEmpty()) {
            return;
        }
        long staleBefore = clientTickCounter - REMOTE_WINDOW_STALE_TICKS;
        Iterator<Map.Entry<WindowKey, WindowView>> it = remoteWindows.entrySet().iterator();
        while (it.hasNext()) {
            Map.Entry<WindowKey, WindowView> entry = it.next();
            if (entry.getValue().lastSeenTick >= staleBefore) {
                continue;
            }
            entry.getValue().renderer.clearFlowData();
            it.remove();
        }
    }

    private int sanitizeStride(int requested) {
        return (requested == 1 || requested == 2 || requested == 4 || requested == 8) ? requested : DEFAULT_STREAMLINE_STRIDE;
    }

    private WindowView findWindowView(Identifier dimensionId, Vec3d worldPos) {
        for (Map.Entry<WindowKey, WindowView> entry : remoteWindows.entrySet()) {
            WindowKey key = entry.getKey();
            if (!key.dimensionId().equals(dimensionId)) {
                continue;
            }
            if (isInsideCore(key.origin(), worldPos)) {
                WindowView view = entry.getValue();
                view.origin = key.origin();
                return view;
            }
        }
        return null;
    }

    private boolean isInsideCore(BlockPos origin, Vec3d worldPos) {
        return worldPos.x >= origin.getX() + REGION_HALO_CELLS && worldPos.x < origin.getX() + REGION_HALO_CELLS + REGION_CORE_SIZE
            && worldPos.y >= origin.getY() + REGION_HALO_CELLS && worldPos.y < origin.getY() + REGION_HALO_CELLS + REGION_CORE_SIZE
            && worldPos.z >= origin.getZ() + REGION_HALO_CELLS && worldPos.z < origin.getZ() + REGION_HALO_CELLS + REGION_CORE_SIZE;
    }

    private void clearState() {
        remoteWindows.clear();
        clientFluidRuntime.clear();
        streamingEnabled = false;
        debugEnabled = false;
        clientPlayerAuthority = false;
        maxWindSpeed = DEFAULT_MAX_INFLOW_SPEED;
        streamlineSampleStride = DEFAULT_STREAMLINE_STRIDE;
        backendMode = DEFAULT_BACKEND_MODE;
        renderVelocityVectors = true;
        renderStreamlines = true;
        clientTickCounter = 0L;
    }

    private record WindowKey(Identifier dimensionId, BlockPos origin) {
    }

    private static final class WindowView {
        private final FlowRenderer renderer;
        private final PhysicsHandler physics;
        private long lastSeenTick;
        private BlockPos origin;

        private WindowView(FlowRenderer renderer, long lastSeenTick) {
            this.renderer = renderer;
            this.physics = new PhysicsHandler(renderer, GRID_SIZE);
            this.lastSeenTick = lastSeenTick;
            this.origin = BlockPos.ORIGIN;
        }

        private FlowRenderer renderer() {
            return renderer;
        }

        private PhysicsHandler physics() {
            return physics;
        }

        private BlockPos origin() {
            return origin;
        }
    }
}
