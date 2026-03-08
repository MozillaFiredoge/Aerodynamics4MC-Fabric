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
    boolean streamlineRoiEnabled,
    int streamlineRoiMinX,
    int streamlineRoiMaxX,
    int streamlineRoiMinY,
    int streamlineRoiMaxY,
    int streamlineRoiMinZ,
    int streamlineRoiMaxZ,
    boolean clientPlayerAuthority,
    int backendMode,
    boolean renderVelocityVectors,
    boolean renderStreamlines,
    boolean renderBackgroundVectors,
    boolean renderThermalAnomaly
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
            buf.readVarInt(),
            buf.readVarInt(),
            buf.readVarInt(),
            buf.readVarInt(),
            buf.readVarInt(),
            buf.readVarInt(),
            buf.readBoolean(),
            buf.readVarInt(),
            buf.readBoolean(),
            buf.readBoolean(),
            buf.readBoolean(),
            buf.readBoolean()
        );
    }

    private void write(RegistryByteBuf buf) {
        buf.writeBoolean(streamingEnabled);
        buf.writeBoolean(debugEnabled);
        buf.writeFloat(maxWindSpeed);
        buf.writeVarInt(streamlineStride);
        buf.writeBoolean(streamlineRoiEnabled);
        buf.writeVarInt(streamlineRoiMinX);
        buf.writeVarInt(streamlineRoiMaxX);
        buf.writeVarInt(streamlineRoiMinY);
        buf.writeVarInt(streamlineRoiMaxY);
        buf.writeVarInt(streamlineRoiMinZ);
        buf.writeVarInt(streamlineRoiMaxZ);
        buf.writeBoolean(clientPlayerAuthority);
        buf.writeVarInt(backendMode);
        buf.writeBoolean(renderVelocityVectors);
        buf.writeBoolean(renderStreamlines);
        buf.writeBoolean(renderBackgroundVectors);
        buf.writeBoolean(renderThermalAnomaly);
    }

    @Override
    public Id<? extends CustomPayload> getId() {
        return ID;
    }
}
