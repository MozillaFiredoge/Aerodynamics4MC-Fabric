/*
package com.aerodynamics4mc.net;

import com.aerodynamics4mc.ModTemplate;
import com.github.razorplay.packet_handler.network.IPacket;
import com.github.razorplay.packet_handler.network.PacketTCP;
import com.google.common.io.ByteArrayDataInput;
import com.google.common.io.ByteStreams;
import io.netty.buffer.ByteBuf;
import net.minecraft.network.RegistryFriendlyByteBuf;
import net.minecraft.network.codec.StreamCodec;
import net.minecraft.network.protocol.common.custom.CustomPacketPayload;
import net.minecraft.resources.Identifier;

public record FabricCustomPayload(IPacket packet) implements CustomPacketPayload {
	public static final Type<FabricCustomPayload> CUSTOM_PAYLOAD_ID = new Type<>(Identifier.fromNamespaceAndPath(ModTemplate.MOD_ID, "packets_channel"));

	public static final StreamCodec<RegistryFriendlyByteBuf, FabricCustomPayload> CODEC = StreamCodec.composite(
			new StreamCodec<ByteBuf, IPacket>() {
				@Override
				public IPacket decode(ByteBuf byteBuf) {
					try {
						byte[] data = new byte[byteBuf.readableBytes()];
						byteBuf.readBytes(data);
						ByteArrayDataInput in = ByteStreams.newDataInput(data);
						return PacketTCP.read(in);
					} catch (Exception e) {
						ModTemplate.LOGGER.error("Error decoding packet: {}", e.getMessage());
						return null;
					}
				}

				@Override
				public void encode(ByteBuf byteBuf, IPacket packet) {
					try {
						byteBuf.writeBytes(PacketTCP.write(packet));
					} catch (Exception e) {
						ModTemplate.LOGGER.error("Error encoding packet: {}", e.getMessage());
					}
				}
			},
			FabricCustomPayload::packet,
			FabricCustomPayload::new
	);

	@Override
	public Type<? extends CustomPacketPayload> type() {
		return CUSTOM_PAYLOAD_ID;
	}
}
*/
