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
    float[] flow
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
            readFlow(buf)
        );
    }

    private static float[] readFlow(RegistryByteBuf buf) {
        int length = buf.readVarInt();
        float[] data = new float[length];
        for (int i = 0; i < length; i++) {
            data[i] = buf.readFloat();
        }
        return data;
    }

    private void write(RegistryByteBuf buf) {
        buf.writeIdentifier(dimensionId);
        buf.writeBlockPos(origin);
        buf.writeVarInt(sampleStride);
        buf.writeVarInt(flow.length);
        for (float v : flow) {
            buf.writeFloat(v);
        }
    }

    @Override
    public Id<? extends CustomPayload> getId() {
        return ID;
    }
}
