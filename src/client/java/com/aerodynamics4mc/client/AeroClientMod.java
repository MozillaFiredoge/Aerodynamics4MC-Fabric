package com.aerodynamics4mc.client;

import com.aerodynamics4mc.net.AeroFlowPayload;
import com.aerodynamics4mc.net.AeroRuntimeStatePayload;

import net.fabricmc.api.ClientModInitializer;
import net.fabricmc.fabric.api.client.networking.v1.ClientPlayNetworking;

public final class AeroClientMod implements ClientModInitializer {
    private final AeroVisualizer visualizer = new AeroVisualizer();

    @Override
    public void onInitializeClient() {
        ClientPlayNetworking.registerGlobalReceiver(AeroRuntimeStatePayload.ID, this::onRuntimeState);
        ClientPlayNetworking.registerGlobalReceiver(AeroFlowPayload.ID, this::onFlowField);
        visualizer.initialize();
    }

    private void onRuntimeState(AeroRuntimeStatePayload payload, ClientPlayNetworking.Context context) {
        context.client().execute(() -> visualizer.onRuntimeState(new AeroVisualizer.AeroFlowState(payload.streamingEnabled())));
    }

    private void onFlowField(AeroFlowPayload payload, ClientPlayNetworking.Context context) {
        context.client().execute(() -> visualizer.onFlowField(payload));
    }
}
