package com.aerodynamics4mc.client;

import java.util.HashMap;
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
    private static final int GRID_SIZE = 128;
    private static final float DEFAULT_MAX_INFLOW_SPEED = 8.0f;
    private static final double PLAYER_PREDICTION_FORCE_STRENGTH = 0.02;

    private final Map<WindowKey, WindowView> windows = new HashMap<>();

    private boolean streamingEnabled = false;
    private boolean debugEnabled = false;
    private float maxWindSpeed = DEFAULT_MAX_INFLOW_SPEED;
    private int streamlineSampleStride = 4;

    @Override
    public void onInitializeClient() {
        WorldRenderEvents.BEFORE_DEBUG_RENDER.register(this::onRender);
        ClientTickEvents.END_CLIENT_TICK.register(this::onClientTick);
        ClientPlayNetworking.registerGlobalReceiver(AeroRuntimeStatePayload.ID, this::onRuntimeState);
        ClientPlayNetworking.registerGlobalReceiver(AeroFlowPayload.ID, this::onFlowField);
        ClientPlayConnectionEvents.DISCONNECT.register((handler, client) -> clearState());
    }

    private void onClientTick(MinecraftClient client) {
        if (!streamingEnabled || client.world == null || client.player == null || client.player.isSpectator()) {
            return;
        }

        Identifier dimensionId = client.world.getRegistryKey().getValue();
        Vec3d center = client.player.getBoundingBox().getCenter();
        WindowView window = findWindowView(dimensionId, center);
        if (window == null) {
            return;
        }

        Vec3d windVelocity = window.renderer().sampleVelocity(center);
        Vec3d delta = windVelocity.multiply(PLAYER_PREDICTION_FORCE_STRENGTH);
        if (delta.lengthSquared() < 1e-10) {
            return;
        }
        client.player.addVelocity(delta.x, delta.y, delta.z);
    }

    private void onRender(WorldRenderContext context) {
        if (!streamingEnabled || !debugEnabled) {
            return;
        }
        MinecraftClient client = MinecraftClient.getInstance();
        if (client.world == null) {
            return;
        }

        Identifier currentDimension = client.world.getRegistryKey().getValue();
        for (Map.Entry<WindowKey, WindowView> entry : windows.entrySet()) {
            if (!entry.getKey().dimensionId().equals(currentDimension)) {
                continue;
            }
            FlowRenderer renderer = entry.getValue().renderer();
            renderer.setMaxInflowSpeed(maxWindSpeed);
            renderer.setStreamlineSampleStride(streamlineSampleStride);
            renderer.render(context);
        }
    }

    private void onRuntimeState(AeroRuntimeStatePayload payload, ClientPlayNetworking.Context context) {
        context.client().execute(() -> {
            streamingEnabled = payload.streamingEnabled();
            debugEnabled = payload.debugEnabled();
            maxWindSpeed = Math.max(1e-6f, payload.maxWindSpeed());
            streamlineSampleStride = sanitizeStride(payload.streamlineStride());

            for (WindowView window : windows.values()) {
                FlowRenderer renderer = window.renderer();
                renderer.setMaxInflowSpeed(maxWindSpeed);
                renderer.setStreamlineSampleStride(streamlineSampleStride);
            }

            if (!streamingEnabled) {
                windows.clear();
            }
        });
    }

    private void onFlowField(AeroFlowPayload payload, ClientPlayNetworking.Context context) {
        context.client().execute(() -> {
            int stride = sanitizeStride(payload.sampleStride());
            WindowKey key = new WindowKey(payload.dimensionId(), payload.origin());
            WindowView view = windows.computeIfAbsent(
                key,
                ignored -> new WindowView(new FlowRenderer(GRID_SIZE, maxWindSpeed))
            );
            FlowRenderer renderer = view.renderer();
            renderer.setMaxInflowSpeed(maxWindSpeed);
            renderer.setStreamlineSampleStride(streamlineSampleStride);
            renderer.updateFlowField(payload.origin(), stride, payload.flow());
        });
    }

    private int sanitizeStride(int requested) {
        return (requested == 1 || requested == 2 || requested == 4 || requested == 8) ? requested : 4;
    }

    private WindowView findWindowView(Identifier dimensionId, Vec3d worldPos) {
        for (Map.Entry<WindowKey, WindowView> entry : windows.entrySet()) {
            WindowKey key = entry.getKey();
            if (!key.dimensionId().equals(dimensionId)) {
                continue;
            }
            if (isInsideWindow(key.origin(), worldPos)) {
                return entry.getValue();
            }
        }
        return null;
    }

    private boolean isInsideWindow(BlockPos origin, Vec3d worldPos) {
        return worldPos.x >= origin.getX() && worldPos.x < origin.getX() + GRID_SIZE
            && worldPos.y >= origin.getY() && worldPos.y < origin.getY() + GRID_SIZE
            && worldPos.z >= origin.getZ() && worldPos.z < origin.getZ() + GRID_SIZE;
    }

    private void clearState() {
        windows.clear();
        streamingEnabled = false;
        debugEnabled = false;
        maxWindSpeed = DEFAULT_MAX_INFLOW_SPEED;
        streamlineSampleStride = 4;
    }

    private record WindowKey(Identifier dimensionId, BlockPos origin) {
    }

    private record WindowView(FlowRenderer renderer) {
    }
}
