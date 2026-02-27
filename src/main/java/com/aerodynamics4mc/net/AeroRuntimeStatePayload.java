package com.aerodynamics4mc.net;

import com.aerodynamics4mc.ModBlocks;

import net.minecraft.network.RegistryByteBuf;
import net.minecraft.network.codec.PacketCodec;
import net.minecraft.network.packet.CustomPayload;
import net.minecraft.util.Identifier;

public record AeroRuntimeStatePayload(
    boolean streamingEnabled,
    boolean debugEnabled,
    float maxWindSpeed,
    int streamlineStride,
    boolean clientPlayerAuthority,
    int backendMode
) implements CustomPayload {
    public static final CustomPayload.Id<AeroRuntimeStatePayload> ID =
        new CustomPayload.Id<>(Identifier.of(ModBlocks.MOD_ID, "runtime_state"));
    public static final PacketCodec<RegistryByteBuf, AeroRuntimeStatePayload> CODEC =
        PacketCodec.of(AeroRuntimeStatePayload::write, AeroRuntimeStatePayload::new);

    private AeroRuntimeStatePayload(RegistryByteBuf buf) {
        this(
            buf.readBoolean(),
            buf.readBoolean(),
            buf.readFloat(),
            buf.readVarInt(),
            buf.readBoolean(),
            buf.readVarInt()
        );
    }

    private void write(RegistryByteBuf buf) {
        buf.writeBoolean(streamingEnabled);
        buf.writeBoolean(debugEnabled);
        buf.writeFloat(maxWindSpeed);
        buf.writeVarInt(streamlineStride);
        buf.writeBoolean(clientPlayerAuthority);
        buf.writeVarInt(backendMode);
    }

    @Override
    public Id<? extends CustomPayload> getId() {
        return ID;
    }
}
