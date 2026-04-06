package com.aerodynamics4mc.client;

import java.util.HashMap;
import java.util.Map;

import com.aerodynamics4mc.net.AeroFlowPayload;
import com.aerodynamics4mc.net.AeroRuntimeStatePayload;

import net.fabricmc.api.ClientModInitializer;
import net.fabricmc.fabric.api.client.networking.v1.ClientPlayConnectionEvents;
import net.fabricmc.fabric.api.client.networking.v1.ClientPlayNetworking;
import net.minecraft.util.Identifier;
import net.minecraft.util.math.BlockPos;

public final class AeroClientMod implements ClientModInitializer {
    private final Map<WindowKey, RemoteFlowField> remoteWindows = new HashMap<>();
    private boolean streamingEnabled;

    @Override
    public void onInitializeClient() {
        ClientPlayNetworking.registerGlobalReceiver(AeroRuntimeStatePayload.ID, this::onRuntimeState);
        ClientPlayNetworking.registerGlobalReceiver(AeroFlowPayload.ID, this::onFlowField);
        ClientPlayConnectionEvents.DISCONNECT.register((handler, client) -> clearState());
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
            remoteWindows.put(key, new RemoteFlowField(payload.origin(), payload.sampleStride(), payload.packedFlow()));
        });
    }

    private void clearState() {
        remoteWindows.clear();
        streamingEnabled = false;
    }

    private record WindowKey(Identifier dimensionId, BlockPos origin) {
    }

    private record RemoteFlowField(BlockPos origin, int sampleStride, short[] packedFlow) {
    }
}
