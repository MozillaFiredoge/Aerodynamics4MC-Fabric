package com.aerodynamics4mc.net;

import com.aerodynamics4mc.block.ModBlocks;
import net.minecraft.core.BlockPos;
import net.minecraft.network.RegistryFriendlyByteBuf;
import net.minecraft.network.codec.StreamCodec;
import net.minecraft.network.protocol.common.custom.CustomPacketPayload;
import net.minecraft.resources.Identifier;

public record AeroFlowPayload(
    Identifier dimensionId,
    BlockPos origin,
    int sampleStride,
    short[] packedFlow,
    byte[] packedFlowBytes
) implements CustomPacketPayload {
    private static final int MAX_PACKED_FLOW_SHORTS = 1_048_576;

    public static final Type<AeroFlowPayload> ID =
        new Type<>(Identifier.fromNamespaceAndPath(ModBlocks.MOD_ID, "flow_field"));
    public static final StreamCodec<RegistryFriendlyByteBuf, AeroFlowPayload> CODEC =
			StreamCodec.ofMember(AeroFlowPayload::write, AeroFlowPayload::new);

    public AeroFlowPayload {
        if (packedFlow == null) {
            packedFlow = new short[0];
        }
        if (packedFlow.length > MAX_PACKED_FLOW_SHORTS) {
            throw new IllegalArgumentException("Invalid flow payload length: " + packedFlow.length);
        }
        if (packedFlowBytes == null || packedFlowBytes.length != packedFlow.length * Short.BYTES) {
            packedFlowBytes = encodePackedFlow(packedFlow);
        }
    }

    public AeroFlowPayload(Identifier dimensionId, BlockPos origin, int sampleStride, short[] packedFlow) {
        this(dimensionId, origin, sampleStride, packedFlow, encodePackedFlow(packedFlow));
    }

    public static AeroFlowPayload fromPackedBytes(
        Identifier dimensionId,
        BlockPos origin,
        int sampleStride,
        short[] packedFlow,
        byte[] packedFlowBytes
    ) {
        return new AeroFlowPayload(dimensionId, origin, sampleStride, packedFlow, packedFlowBytes);
    }

    private AeroFlowPayload(RegistryFriendlyByteBuf buf) {
        this(buf.readIdentifier(), buf.readBlockPos(), buf.readVarInt(), readPackedFlowBytes(buf));
    }

    private AeroFlowPayload(Identifier dimensionId, BlockPos origin, int sampleStride, byte[] packedFlowBytes) {
        this(dimensionId, origin, sampleStride, decodePackedFlow(packedFlowBytes), packedFlowBytes);
    }

    private static byte[] readPackedFlowBytes(RegistryFriendlyByteBuf buf) {
        int length = buf.readVarInt();
        if (length < 0 || length > MAX_PACKED_FLOW_SHORTS) {
            throw new IllegalArgumentException("Invalid flow payload length: " + length);
        }
        byte[] data = new byte[length * Short.BYTES];
        buf.readBytes(data);
        return data;
    }

    public static byte[] encodePackedFlow(short[] packedFlow) {
        if (packedFlow == null) {
            return new byte[0];
        }
        byte[] bytes = new byte[packedFlow.length * Short.BYTES];
        for (int i = 0; i < packedFlow.length; i++) {
            short value = packedFlow[i];
            int base = i * Short.BYTES;
            bytes[base] = (byte) ((value >>> 8) & 0xFF);
            bytes[base + 1] = (byte) (value & 0xFF);
        }
        return bytes;
    }

    private static short[] decodePackedFlow(byte[] packedFlowBytes) {
        if (packedFlowBytes == null || packedFlowBytes.length == 0) {
            return new short[0];
        }
        if ((packedFlowBytes.length & 1) != 0) {
            throw new IllegalArgumentException("Invalid packed flow byte length: " + packedFlowBytes.length);
        }
        int length = packedFlowBytes.length / Short.BYTES;
        if (length > MAX_PACKED_FLOW_SHORTS) {
            throw new IllegalArgumentException("Invalid flow payload length: " + length);
        }
        short[] data = new short[length];
        for (int i = 0; i < length; i++) {
            int base = i * Short.BYTES;
            data[i] = (short) (((packedFlowBytes[base] & 0xFF) << 8) | (packedFlowBytes[base + 1] & 0xFF));
        }
        return data;
    }

    private void write(RegistryFriendlyByteBuf buf) {
        buf.writeIdentifier(dimensionId);
        buf.writeBlockPos(origin);
        buf.writeVarInt(sampleStride);
        buf.writeVarInt(packedFlow.length);
        buf.writeBytes(packedFlowBytes);
    }

    @Override
    public Type<? extends CustomPacketPayload> type() {
        return ID;
    }
}
