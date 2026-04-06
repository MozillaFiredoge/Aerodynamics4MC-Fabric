package com.aerodynamics4mc.net;

import com.aerodynamics4mc.ModBlocks;

import net.minecraft.network.RegistryByteBuf;
import net.minecraft.network.codec.PacketCodec;
import net.minecraft.network.packet.CustomPayload;
import net.minecraft.util.Identifier;
import net.minecraft.util.math.BlockPos;

public record AeroFlowPayload(
    Identifier dimensionId,
    BlockPos origin,
    int sampleStride,
    short[] packedFlow
) implements CustomPayload {
    public static final CustomPayload.Id<AeroFlowPayload> ID =
        new CustomPayload.Id<>(Identifier.of(ModBlocks.MOD_ID, "flow_field"));
    public static final PacketCodec<RegistryByteBuf, AeroFlowPayload> CODEC =
        PacketCodec.of(AeroFlowPayload::write, AeroFlowPayload::new);

    private AeroFlowPayload(RegistryByteBuf buf) {
        this(
            buf.readIdentifier(),
            buf.readBlockPos(),
            buf.readVarInt(),
            readPackedFlow(buf)
        );
    }

    private static short[] readPackedFlow(RegistryByteBuf buf) {
        int length = buf.readVarInt();
        short[] data = new short[length];
        for (int i = 0; i < length; i++) {
            data[i] = buf.readShort();
        }
        return data;
    }

    private void write(RegistryByteBuf buf) {
        buf.writeIdentifier(dimensionId);
        buf.writeBlockPos(origin);
        buf.writeVarInt(sampleStride);
        buf.writeVarInt(packedFlow.length);
        for (short v : packedFlow) {
            buf.writeShort(v);
        }
    }

    @Override
    public Id<? extends CustomPayload> getId() {
        return ID;
    }
}
