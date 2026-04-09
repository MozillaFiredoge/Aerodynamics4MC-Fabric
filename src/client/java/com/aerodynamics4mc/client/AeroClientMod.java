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
import net.minecraft.client.render.DrawStyle;
import net.minecraft.util.Identifier;
import net.minecraft.util.math.BlockPos;
import net.minecraft.util.math.Box;
import net.minecraft.util.math.Vec3d;
import net.minecraft.world.debug.gizmo.GizmoDrawing;

public final class AeroClientMod implements ClientModInitializer {
    private static final float ATLAS_VELOCITY_RANGE = 5.6f;
    private static final float ATLAS_PRESSURE_RANGE = 0.03f;
    private static final double MAX_RENDER_DISTANCE = 192.0;
    private static final int REGION_HALO_CELLS = 16;
    private static final int REGION_STALE_TICKS = 40;

    private final Map<WindowKey, RemoteFlowField> remoteWindows = new HashMap<>();
    private boolean streamingEnabled;
    private long clientTickCounter;

    @Override
    public void onInitializeClient() {
        ClientPlayNetworking.registerGlobalReceiver(AeroRuntimeStatePayload.ID, this::onRuntimeState);
        ClientPlayNetworking.registerGlobalReceiver(AeroFlowPayload.ID, this::onFlowField);
        ClientPlayConnectionEvents.DISCONNECT.register((handler, client) -> clearState());
        ClientTickEvents.END_CLIENT_TICK.register(client -> onClientTick());
        WorldRenderEvents.BEFORE_DEBUG_RENDER.register(this::renderAtlasOverlay);
    }

    private void onRuntimeState(AeroRuntimeStatePayload payload, ClientPlayNetworking.Context context) {
        context.client().execute(() -> {
            streamingEnabled = payload.streamingEnabled();
            if (!streamingEnabled) {
                remoteWindows.clear();
            }
        });
    }

    private void onFlowField(AeroFlowPayload payload, ClientPlayNetworking.Context context) {
        context.client().execute(() -> {
            if (!streamingEnabled) {
                return;
            }
            WindowKey key = new WindowKey(payload.dimensionId(), payload.origin());
            remoteWindows.put(key, RemoteFlowField.fromPayload(payload, clientTickCounter));
        });
    }

    private void onClientTick() {
        clientTickCounter++;
        if (!streamingEnabled || remoteWindows.isEmpty()) {
            return;
        }
        remoteWindows.entrySet().removeIf(entry -> clientTickCounter - entry.getValue().lastUpdatedTick() > REGION_STALE_TICKS);
    }

    private void renderAtlasOverlay(WorldRenderContext context) {
        if (!streamingEnabled || remoteWindows.isEmpty()) {
            return;
        }
        MinecraftClient client = MinecraftClient.getInstance();
        if (client.world == null || client.player == null) {
            return;
        }
        Identifier dimensionId = client.world.getRegistryKey().getValue();
        Vec3d cameraPos = client.gameRenderer.getCamera().getCameraPos();
        try (var ignored = client.newGizmoScope()) {
            for (RemoteFlowField field : remoteWindows.values()) {
                if (!field.dimensionId().equals(dimensionId)) {
                    continue;
                }
                if (field.visibleBox().squaredMagnitude(cameraPos) > MAX_RENDER_DISTANCE * MAX_RENDER_DISTANCE) {
                    continue;
                }
                renderRegionOverlay(field);
            }
        }
    }

    private void renderRegionOverlay(RemoteFlowField field) {
        FlowVisual visual = field.visual();
        float maxSpeedNorm = clamp01(visual.maxSpeed() / 1.5f);
        float meanSpeedNorm = clamp01(visual.meanSpeed() / 1.0f);
        float pressureNorm = clamp01(visual.meanAbsPressure() / 0.01f);

        int strokeColor = argb(
            0.70f + 0.25f * maxSpeedNorm,
            0.20f + 0.45f * pressureNorm,
            0.55f + 0.35f * maxSpeedNorm,
            0.75f + 0.20f * meanSpeedNorm
        );
        int fillColor = argb(
            0.03f + 0.09f * pressureNorm,
            0.08f + 0.12f * pressureNorm,
            0.18f + 0.30f * meanSpeedNorm,
            0.25f + 0.35f * maxSpeedNorm
        );
        DrawStyle style = DrawStyle.filledAndStroked(strokeColor, 1.5f, fillColor);
        GizmoDrawing.box(field.visibleBox(), style).ignoreOcclusion().withLifespan(1);

        Vec3d direction = visual.displayDirection();
        if (direction.lengthSquared() <= 1.0e-6 || visual.maxSpeed() <= 0.05f) {
            return;
        }
        double arrowLength = 6.0 + 20.0 * maxSpeedNorm;
        Vec3d start = field.visibleBox().getCenter();
        Vec3d end = start.add(direction.multiply(arrowLength));
        GizmoDrawing.arrow(start, end, strokeColor, 2.0f).ignoreOcclusion().withLifespan(1);
    }

    private void clearState() {
        remoteWindows.clear();
        streamingEnabled = false;
    }

    private static float decodeVelocity(short value) {
        return value / 32767.0f * ATLAS_VELOCITY_RANGE;
    }

    private static float decodePressure(short value) {
        return value / 32767.0f * ATLAS_PRESSURE_RANGE;
    }

    private static float clamp01(float value) {
        return Math.max(0.0f, Math.min(1.0f, value));
    }

    private static int argb(float alpha, float red, float green, float blue) {
        int a = Math.round(clamp01(alpha) * 255.0f);
        int r = Math.round(clamp01(red) * 255.0f);
        int g = Math.round(clamp01(green) * 255.0f);
        int b = Math.round(clamp01(blue) * 255.0f);
        return (a << 24) | (r << 16) | (g << 8) | b;
    }

    private record WindowKey(Identifier dimensionId, BlockPos origin) {
    }

    private record RemoteFlowField(
        Identifier dimensionId,
        BlockPos origin,
        int sampleStride,
        int atlasResolution,
        short[] packedFlow,
        FlowVisual visual,
        long lastUpdatedTick
    ) {
        static RemoteFlowField fromPayload(AeroFlowPayload payload, long lastUpdatedTick) {
            int sampleCount = payload.packedFlow().length / 4;
            int atlasResolution = Math.max(1, Math.round((float) Math.cbrt(sampleCount)));
            return new RemoteFlowField(
                payload.dimensionId(),
                payload.origin(),
                payload.sampleStride(),
                atlasResolution,
                payload.packedFlow(),
                FlowVisual.fromPackedFlow(payload.packedFlow()),
                lastUpdatedTick
            );
        }

        Box regionBox() {
            double size = (double) atlasResolution * (double) sampleStride;
            return new Box(
                origin.getX(), origin.getY(), origin.getZ(),
                origin.getX() + size, origin.getY() + size, origin.getZ() + size
            );
        }

        Box visibleBox() {
            Box full = regionBox();
            return new Box(
                full.minX + REGION_HALO_CELLS,
                full.minY + REGION_HALO_CELLS,
                full.minZ + REGION_HALO_CELLS,
                full.maxX - REGION_HALO_CELLS,
                full.maxY - REGION_HALO_CELLS,
                full.maxZ - REGION_HALO_CELLS
            );
        }
    }

    private record FlowVisual(
        float maxSpeed,
        float meanSpeed,
        float meanAbsPressure,
        float meanVx,
        float meanVy,
        float meanVz,
        float dominantVx,
        float dominantVy,
        float dominantVz
    ) {
        static FlowVisual fromPackedFlow(short[] packedFlow) {
            int sampleCount = packedFlow.length / 4;
            if (sampleCount <= 0) {
                return new FlowVisual(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            }
            float sumSpeed = 0.0f;
            float maxSpeed = 0.0f;
            float sumAbsPressure = 0.0f;
            float sumVx = 0.0f;
            float sumVy = 0.0f;
            float sumVz = 0.0f;
            float dominantVx = 0.0f;
            float dominantVy = 0.0f;
            float dominantVz = 0.0f;
            for (int i = 0; i < packedFlow.length; i += 4) {
                float vx = decodeVelocity(packedFlow[i]);
                float vy = decodeVelocity(packedFlow[i + 1]);
                float vz = decodeVelocity(packedFlow[i + 2]);
                float pressure = decodePressure(packedFlow[i + 3]);
                float speed = (float) Math.sqrt(vx * vx + vy * vy + vz * vz);
                sumVx += vx;
                sumVy += vy;
                sumVz += vz;
                sumSpeed += speed;
                sumAbsPressure += Math.abs(pressure);
                if (speed > maxSpeed) {
                    maxSpeed = speed;
                    dominantVx = vx;
                    dominantVy = vy;
                    dominantVz = vz;
                }
            }
            float inv = 1.0f / sampleCount;
            return new FlowVisual(
                maxSpeed,
                sumSpeed * inv,
                sumAbsPressure * inv,
                sumVx * inv,
                sumVy * inv,
                sumVz * inv,
                dominantVx,
                dominantVy,
                dominantVz
            );
        }

        Vec3d averageDirection() {
            double length = Math.sqrt(meanVx * meanVx + meanVy * meanVy + meanVz * meanVz);
            if (length <= 1.0e-6) {
                return Vec3d.ZERO;
            }
            return new Vec3d(meanVx / length, meanVy / length, meanVz / length);
        }

        Vec3d dominantDirection() {
            double length = Math.sqrt(dominantVx * dominantVx + dominantVy * dominantVy + dominantVz * dominantVz);
            if (length <= 1.0e-6) {
                return Vec3d.ZERO;
            }
            return new Vec3d(dominantVx / length, dominantVy / length, dominantVz / length);
        }

        Vec3d displayDirection() {
            Vec3d average = averageDirection();
            if (average.lengthSquared() > 1.0e-4) {
                return average;
            }
            return dominantDirection();
        }
    }
}
