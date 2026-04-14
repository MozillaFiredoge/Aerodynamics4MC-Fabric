package com.aerodynamics4mc.client;

import com.aerodynamics4mc.net.AeroFlowPayload;
import com.aerodynamics4mc.net.AeroFlowAnalysisPayload;
import com.aerodynamics4mc.net.AeroRuntimeStatePayload;

import net.fabricmc.api.ClientModInitializer;
import net.fabricmc.fabric.api.client.networking.v1.ClientPlayNetworking;
import net.minecraft.client.world.ClientWorld;
import net.minecraft.util.math.Vec3d;

public final class AeroClientMod implements ClientModInitializer {
    private static AeroClientMod instance;
    private final AeroVisualizer visualizer = new AeroVisualizer();
    private final IrisWindBridge irisWindBridge = new IrisWindBridge(visualizer);

    @Override
    public void onInitializeClient() {
        instance = this;
        ClientPlayNetworking.registerGlobalReceiver(AeroRuntimeStatePayload.ID, this::onRuntimeState);
        ClientPlayNetworking.registerGlobalReceiver(AeroFlowPayload.ID, this::onFlowField);
        ClientPlayNetworking.registerGlobalReceiver(AeroFlowAnalysisPayload.ID, this::onFlowAnalysis);
        visualizer.initialize();
        irisWindBridge.initialize();
    }

    public static Vec3d sampleWind(ClientWorld world, Vec3d position) {
        AeroClientMod active = instance;
        if (active == null || world == null) {
            return Vec3d.ZERO;
        }
        return active.visualizer.sampleWind(world.getRegistryKey().getValue(), position);
    }

    private void onRuntimeState(AeroRuntimeStatePayload payload, ClientPlayNetworking.Context context) {
        context.client().execute(() -> {
            visualizer.onRuntimeState(new AeroVisualizer.AeroFlowState(payload.streamingEnabled()));
            irisWindBridge.onRuntimeState(payload.streamingEnabled());
        });
    }

    private void onFlowField(AeroFlowPayload payload, ClientPlayNetworking.Context context) {
        context.client().execute(() -> {
            visualizer.onFlowField(payload);
            irisWindBridge.markDirty();
        });
    }

    private void onFlowAnalysis(AeroFlowAnalysisPayload payload, ClientPlayNetworking.Context context) {
        context.client().execute(() -> visualizer.onFlowAnalysis(payload));
    }
}
