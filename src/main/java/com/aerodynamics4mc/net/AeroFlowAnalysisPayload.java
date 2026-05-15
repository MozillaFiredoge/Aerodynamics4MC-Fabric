package com.aerodynamics4mc.net;

import com.aerodynamics4mc.ModTemplate;
import net.minecraft.core.BlockPos;
import net.minecraft.network.RegistryFriendlyByteBuf;
import net.minecraft.network.codec.StreamCodec;
import net.minecraft.network.protocol.common.custom.CustomPacketPayload;
import net.minecraft.resources.Identifier;

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
) implements CustomPacketPayload {
	private static final int MAX_PACKED_FLOW_SHORTS = 1_048_576;
	private static final int MAX_RESIDUAL_BYTES = 4_194_304;

	public static final Type<AeroFlowAnalysisPayload> ID =
			new Type<>(Identifier.fromNamespaceAndPath(ModTemplate.MOD_ID, "flow_field_analysis"));
	public static final StreamCodec<RegistryFriendlyByteBuf, AeroFlowAnalysisPayload> CODEC =
			StreamCodec.ofMember(AeroFlowAnalysisPayload::write, AeroFlowAnalysisPayload::new);

	private AeroFlowAnalysisPayload(RegistryFriendlyByteBuf buf) {
		this(
				buf.readIdentifier(),
				buf.readBlockPos(),
				buf.readVarInt(),
				buf.readVarInt(),
				buf.readFloat(),
				buf.readFloat(),
				readPackedFlow(buf),
				buf.readByteArray(MAX_RESIDUAL_BYTES),
				buf.readByteArray(MAX_RESIDUAL_BYTES),
				buf.readByteArray(MAX_RESIDUAL_BYTES),
				buf.readByteArray(MAX_RESIDUAL_BYTES)
		);
	}

	private static short[] readPackedFlow(RegistryFriendlyByteBuf buf) {
		int length = buf.readVarInt();
		if (length < 0 || length > MAX_PACKED_FLOW_SHORTS) {
			throw new IllegalArgumentException("Invalid flow analysis base length: " + length);
		}
		short[] data = new short[length];
		for (int i = 0; i < length; i++) {
			data[i] = buf.readShort();
		}
		return data;
	}

	private void write(RegistryFriendlyByteBuf buf) {
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
	public Type<? extends CustomPacketPayload> type() {
		return ID;
	}
}
