package com.aerodynamics4mc.net;

import com.aerodynamics4mc.block.ModBlocks;
import net.minecraft.core.BlockPos;
import net.minecraft.network.RegistryFriendlyByteBuf;
import net.minecraft.network.codec.StreamCodec;
import net.minecraft.network.protocol.common.custom.CustomPacketPayload;
import net.minecraft.resources.Identifier;


public record AeroCoarseWindPayload(
		Identifier dimensionType,
		BlockPos origin,
		int cellSize,
		int sizeX,
		int sizeY,
		int sizeZ,
		long serverTick,
		short[] packedFlow,
		short[] packedAtmosphere
) implements CustomPacketPayload {
	private static final int MAX_PACKED_FLOW_SHORTS = 1_048_576;
	private static final int MAX_PACKED_ATMOSPHERE_SHORTS = 1_048_576;

	public static final Type<AeroCoarseWindPayload> ID =
			new Type<>(Identifier.fromNamespaceAndPath(ModBlocks.MOD_ID, "coarse_wind"));
	public static final StreamCodec<RegistryFriendlyByteBuf, AeroCoarseWindPayload> CODEC =
			StreamCodec.ofMember(AeroCoarseWindPayload::write, AeroCoarseWindPayload::new);

	private AeroCoarseWindPayload(RegistryFriendlyByteBuf buf) {
		this(
				buf.readIdentifier(),
				buf.readBlockPos(),
				buf.readVarInt(),
				buf.readVarInt(),
				buf.readVarInt(),
				buf.readVarInt(),
				buf.readVarLong(),
				readShortArray(buf, MAX_PACKED_FLOW_SHORTS, "coarse wind payload"),
				readShortArray(buf, MAX_PACKED_ATMOSPHERE_SHORTS, "coarse atmosphere payload")
		);
	}

	private static short[] readShortArray(RegistryFriendlyByteBuf buf, int maxLength, String label) {
		int length = buf.readVarInt();
		if (length < 0 || length > maxLength) {
			throw new IllegalArgumentException("Invalid " + label + " length: " + length);
		}
		short[] data = new short[length];
		for (int i = 0; i < length; i++) {
			data[i] = buf.readShort();
		}
		return data;
	}

	private void write(RegistryFriendlyByteBuf buf) {
		buf.writeIdentifier(dimensionType);
		buf.writeBlockPos(origin);
		buf.writeVarInt(cellSize);
		buf.writeVarInt(sizeX);
		buf.writeVarInt(sizeY);
		buf.writeVarInt(sizeZ);
		buf.writeVarLong(serverTick);
		writeShortArray(buf, packedFlow);
		writeShortArray(buf, packedAtmosphere);
	}

	private static void writeShortArray(RegistryFriendlyByteBuf buf, short[] values) {
		short[] safeValues = values == null ? new short[0] : values;
		buf.writeVarInt(safeValues.length);
		for (short v : safeValues) {
			buf.writeShort(v);
		}
	}

	@Override
	public Type<? extends CustomPacketPayload> type() {
		return ID;
	}
}
