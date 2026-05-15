package com.aerodynamics4mc.net;

import com.aerodynamics4mc.block.ModBlocks;
import net.minecraft.network.RegistryFriendlyByteBuf;
import net.minecraft.network.codec.StreamCodec;
import net.minecraft.network.protocol.common.custom.CustomPacketPayload;
import net.minecraft.resources.Identifier;

public record AeroRuntimeStatePayload(
		boolean streamingEnabled,
		boolean renderVelocityVectors,
		boolean renderStreamlines
) implements CustomPacketPayload {
	public static final Type<AeroRuntimeStatePayload> ID =
			new Type<>(Identifier.fromNamespaceAndPath(ModBlocks.MOD_ID, "runtime_state"));
	public static final StreamCodec<RegistryFriendlyByteBuf, AeroRuntimeStatePayload> CODEC =
			StreamCodec.ofMember(AeroRuntimeStatePayload::write, AeroRuntimeStatePayload::new);

	private AeroRuntimeStatePayload(RegistryFriendlyByteBuf buf) {
		this(
				buf.readBoolean(),
				buf.readBoolean(),
				buf.readBoolean()
		);
	}

	private void write(RegistryFriendlyByteBuf buf) {
		buf.writeBoolean(streamingEnabled);
		buf.writeBoolean(renderVelocityVectors);
		buf.writeBoolean(renderStreamlines);
	}

	@Override
	public Type<? extends CustomPacketPayload> type() {
		return ID;
	}
}
