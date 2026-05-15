package com.aerodynamics4mc.block;

import com.aerodynamics4mc.api.AeroWindApi;
import com.aerodynamics4mc.api.GameplayWindSample;
import com.aerodynamics4mc.api.SamplePolicy;
import net.minecraft.ChatFormatting;
import net.minecraft.network.chat.Component;
import net.minecraft.server.level.ServerLevel;
import net.minecraft.server.level.ServerPlayer;
import net.minecraft.world.InteractionHand;
import net.minecraft.world.InteractionResult;
import net.minecraft.world.entity.player.Player;
import net.minecraft.world.item.Item;
import net.minecraft.world.item.ItemStack;
import net.minecraft.world.item.TooltipFlag;
import net.minecraft.world.item.component.TooltipDisplay;
import net.minecraft.world.level.Level;
import net.minecraft.world.phys.Vec3;

import java.util.Locale;
import java.util.function.Consumer;

public final class WindMeterItem extends Item {
	private static final double CALM_HORIZONTAL_SPEED_MPS = 0.05;
	private static final String[] DIRECTION_KEYS = {
			"north",
			"north_east",
			"east",
			"south_east",
			"south",
			"south_west",
			"west",
			"north_west"
	};

	public WindMeterItem(Properties properties) {
		super(properties);
	}

	@Override
	public InteractionResult use(Level world, Player user, InteractionHand hand) {
		if (world.isClientSide()) {
			return InteractionResult.SUCCESS;
		}
		if (!(world instanceof ServerLevel)) {
			return InteractionResult.PASS;
		}
		if (!(user instanceof ServerPlayer serverPlayer)) {
			return InteractionResult.PASS;
		}

		Vec3 samplePos = new Vec3(user.getX(), user.getY() + 1.2, user.getZ());
		GameplayWindSample sample = AeroWindApi.sampleGameplay(serverPlayer, samplePos, SamplePolicy.GAMEPLAY_SERVER_ONLY);
		user.getCooldowns().addCooldown(user.getItemInHand(hand), 10);

		if (!sample.hasFlow()) {
			user.displayClientMessage(Component.translatable("message.aerodynamics4mc.wind_meter.no_flow").withStyle(ChatFormatting.GRAY), false);
			return InteractionResult.SUCCESS_SERVER;
		}

		Vec3 effective = sample.effectiveVelocity();
		Vec3 mean = sample.meanVelocity();
		Vec3 gust = sample.gustVelocity();
		Component direction = directionComponent((float) effective.x, (float) effective.z);
		user.displayClientMessage(
				Component.translatable(
						"message.aerodynamics4mc.wind_meter.summary",
						format(sample.effectiveSpeedMetersPerSecond()),
						direction,
						signed(effective.x),
						signed(effective.y),
						signed(effective.z)
				).withStyle(ChatFormatting.AQUA),
				false
		);
		user.displayClientMessage(
				Component.translatable(
						"message.aerodynamics4mc.wind_meter.mean_gust",
						format(sample.meanSpeedMetersPerSecond()),
						signed(mean.x),
						signed(mean.y),
						signed(mean.z),
						format(gust.length()),
						signed(sample.updraftMetersPerSecond())
				).withStyle(ChatFormatting.GRAY),
				false
		);
		user.displayClientMessage(
				Component.translatable(
						"message.aerodynamics4mc.wind_meter.gameplay",
						format(sample.turbulenceIntensity()),
						format(sample.windShearMagnitudePerBlock()),
						percent(sample.shelterFactor()),
						format(sample.ablMixingStrength())
				).withStyle(ChatFormatting.DARK_AQUA),
				false
		);
		user.displayClientMessage(
				Component.translatable(
						"message.aerodynamics4mc.wind_meter.source",
						sample.sourceLevel().name(),
						sample.authority().name(),
						percent(sample.confidence()),
						signed(sample.pressure())
				).withStyle(ChatFormatting.DARK_GRAY),
				false
		);
		if (sample.hasTemperature() || sample.hasHumidity()) {
			user.displayClientMessage(
					Component.translatable(
							"message.aerodynamics4mc.wind_meter.atmosphere",
							sample.hasTemperature() ? format(sample.temperatureKelvin() - 273.15f) : "n/a",
							sample.hasHumidity() ? percent(sample.humidity()) : "n/a",
							format(sample.ablStability())
					).withStyle(ChatFormatting.DARK_AQUA),
					false
			);
		}
		return InteractionResult.SUCCESS_SERVER;
	}

	@Override
	public void appendHoverText(ItemStack stack, TooltipContext context, TooltipDisplay display, Consumer<Component> tooltip, TooltipFlag type) {
		tooltip.accept(Component.translatable("item.aerodynamics4mc.wind_meter.tooltip").withStyle(ChatFormatting.GRAY));
	}

	private static Component directionComponent(float x, float z) {
		double horizontalSpeed = Math.sqrt(x * x + z * z);
		if (horizontalSpeed < CALM_HORIZONTAL_SPEED_MPS) {
			return Component.translatable("message.aerodynamics4mc.wind_meter.direction.calm");
		}
		double degreesClockwiseFromNorth = Math.toDegrees(Math.atan2(x, -z));
		int index = Math.floorMod((int) Math.round(degreesClockwiseFromNorth / 45.0), DIRECTION_KEYS.length);
		return Component.translatable("message.aerodynamics4mc.wind_meter.direction." + DIRECTION_KEYS[index]);
	}

	private static String format(double value) {
		return String.format(Locale.ROOT, "%.2f", value);
	}

	private static String signed(double value) {
		return String.format(Locale.ROOT, "%+.2f", value);
	}

	private static String percent(double value) {
		double clamped = Math.max(0.0, Math.min(1.0, Double.isFinite(value) ? value : 0.0));
		return String.format(Locale.ROOT, "%.0f%%", clamped * 100.0);
	}
}
