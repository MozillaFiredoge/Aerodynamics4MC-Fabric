package com.aerodynamics4mc.net;

import com.aerodynamics4mc.ModTemplate;
import net.minecraft.network.RegistryFriendlyByteBuf;
import net.minecraft.network.codec.StreamCodec;
import net.minecraft.network.protocol.common.custom.CustomPacketPayload;
import net.minecraft.resources.Identifier;

public record AeroClientL2PreferencePayload(boolean localL2Enabled) implements CustomPacketPayload {
	public static final Type<AeroClientL2PreferencePayload> ID =
			new Type<>(Identifier.fromNamespaceAndPath(ModTemplate.MOD_ID, "client_l2_preference"));
	public static final StreamCodec<RegistryFriendlyByteBuf, AeroClientL2PreferencePayload> CODEC =
			StreamCodec.ofMember(AeroClientL2PreferencePayload::write, AeroClientL2PreferencePayload::new);

	private AeroClientL2PreferencePayload(RegistryFriendlyByteBuf buf) {
		this(buf.readBoolean());
	}

	private void write(RegistryFriendlyByteBuf buf) {
		buf.writeBoolean(localL2Enabled);
	}

	@Override
	public Type<? extends CustomPacketPayload> type() {
		return ID;
	}
}
