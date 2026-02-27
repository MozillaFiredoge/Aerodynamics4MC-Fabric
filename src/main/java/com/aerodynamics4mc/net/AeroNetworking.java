package com.aerodynamics4mc.net;

import net.fabricmc.fabric.api.networking.v1.PayloadTypeRegistry;

public final class AeroNetworking {
    private static final int FLOW_PACKET_MAX_BYTES = 40 << 20;

    private AeroNetworking() {
    }

    public static void registerPayloadTypes() {
        PayloadTypeRegistry.playS2C().registerLarge(AeroFlowPayload.ID, AeroFlowPayload.CODEC, FLOW_PACKET_MAX_BYTES);
        PayloadTypeRegistry.playS2C().register(AeroRuntimeStatePayload.ID, AeroRuntimeStatePayload.CODEC);
    }
}
