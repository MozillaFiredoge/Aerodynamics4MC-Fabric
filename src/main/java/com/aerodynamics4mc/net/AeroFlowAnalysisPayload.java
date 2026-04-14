package com.aerodynamics4mc.net;

import com.aerodynamics4mc.ModBlocks;

import net.minecraft.network.RegistryByteBuf;
import net.minecraft.network.codec.PacketCodec;
import net.minecraft.network.packet.CustomPayload;
import net.minecraft.util.Identifier;
import net.minecraft.util.math.BlockPos;

public record AeroFlowAnalysisPayload(
    Identifier dimensionId,
    BlockPos origin,
    int baseSampleStride,
    int fullResolution,
    float velocityTolerance,
    float pressureTolerance,
    short[] basePackedFlow,
    byte[] residualVx,
    byte[] residualVy,
    byte[] residualVz,
    byte[] residualPressure
) implements CustomPayload {
    public static final CustomPayload.Id<AeroFlowAnalysisPayload> ID =
        new CustomPayload.Id<>(Identifier.of(ModBlocks.MOD_ID, "flow_field_analysis"));
    public static final PacketCodec<RegistryByteBuf, AeroFlowAnalysisPayload> CODEC =
        PacketCodec.of(AeroFlowAnalysisPayload::write, AeroFlowAnalysisPayload::new);

    private AeroFlowAnalysisPayload(RegistryByteBuf buf) {
        this(
            buf.readIdentifier(),
            buf.readBlockPos(),
            buf.readVarInt(),
            buf.readVarInt(),
            buf.readFloat(),
            buf.readFloat(),
            readPackedFlow(buf),
            buf.readByteArray(),
            buf.readByteArray(),
            buf.readByteArray(),
            buf.readByteArray()
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
        buf.writeVarInt(baseSampleStride);
        buf.writeVarInt(fullResolution);
        buf.writeFloat(velocityTolerance);
        buf.writeFloat(pressureTolerance);
        buf.writeVarInt(basePackedFlow.length);
        for (short value : basePackedFlow) {
            buf.writeShort(value);
        }
        buf.writeByteArray(residualVx);
        buf.writeByteArray(residualVy);
        buf.writeByteArray(residualVz);
        buf.writeByteArray(residualPressure);
    }

    @Override
    public Id<? extends CustomPayload> getId() {
        return ID;
    }
}
