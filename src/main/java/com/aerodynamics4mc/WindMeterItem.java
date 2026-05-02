package com.aerodynamics4mc;

import java.util.Locale;
import java.util.function.Consumer;

import com.aerodynamics4mc.api.AeroWindApi;
import com.aerodynamics4mc.api.AeroWindSample;
import com.aerodynamics4mc.api.SamplePolicy;

import net.minecraft.component.type.TooltipDisplayComponent;
import net.minecraft.entity.player.PlayerEntity;
import net.minecraft.item.Item;
import net.minecraft.item.ItemStack;
import net.minecraft.item.tooltip.TooltipType;
import net.minecraft.server.world.ServerWorld;
import net.minecraft.text.Text;
import net.minecraft.util.ActionResult;
import net.minecraft.util.Formatting;
import net.minecraft.util.Hand;
import net.minecraft.util.math.Vec3d;
import net.minecraft.world.World;

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

    public WindMeterItem(Settings settings) {
        super(settings);
    }

    @Override
    public ActionResult use(World world, PlayerEntity user, Hand hand) {
        if (world.isClient()) {
            return ActionResult.SUCCESS;
        }
        if (!(world instanceof ServerWorld serverWorld)) {
            return ActionResult.PASS;
        }

        Vec3d samplePos = new Vec3d(user.getX(), user.getY() + 1.2, user.getZ());
        AeroWindSample sample = AeroWindApi.sample(serverWorld, samplePos, SamplePolicy.SERVER_COARSE_ONLY);
        user.getItemCooldownManager().set(user.getStackInHand(hand), 10);

        if (!sample.hasFlow()) {
            user.sendMessage(Text.translatable("message.aerodynamics4mc.wind_meter.no_flow").formatted(Formatting.GRAY), false);
            return ActionResult.SUCCESS_SERVER;
        }

        Text direction = directionText(sample.velocityX(), sample.velocityZ());
        user.sendMessage(
            Text.translatable(
                "message.aerodynamics4mc.wind_meter.summary",
                format(sample.speedMetersPerSecond()),
                direction,
                signed(sample.velocityX()),
                signed(sample.velocityY()),
                signed(sample.velocityZ())
            ).formatted(Formatting.AQUA),
            false
        );
        user.sendMessage(
            Text.translatable(
                "message.aerodynamics4mc.wind_meter.source",
                sample.level().name(),
                sample.authority().name(),
                format(sample.confidence()),
                signed(sample.pressure())
            ).formatted(Formatting.GRAY),
            false
        );
        if (sample.hasAtmosphericDiagnostics()) {
            user.sendMessage(
                Text.translatable(
                    "message.aerodynamics4mc.wind_meter.atmosphere",
                    format(sample.gustVelocity().length()),
                    format(sample.turbulenceIntensity()),
                    format(sample.windShearMagnitudePerBlock())
                ).formatted(Formatting.DARK_AQUA),
                false
            );
        }
        return ActionResult.SUCCESS_SERVER;
    }

    @Override
    public void appendTooltip(
        ItemStack stack,
        TooltipContext context,
        TooltipDisplayComponent display,
        Consumer<Text> tooltip,
        TooltipType type
    ) {
        tooltip.accept(Text.translatable("item.aerodynamics4mc.wind_meter.tooltip").formatted(Formatting.GRAY));
    }

    private static Text directionText(float x, float z) {
        double horizontalSpeed = Math.sqrt(x * x + z * z);
        if (horizontalSpeed < CALM_HORIZONTAL_SPEED_MPS) {
            return Text.translatable("message.aerodynamics4mc.wind_meter.direction.calm");
        }
        double degreesClockwiseFromNorth = Math.toDegrees(Math.atan2(x, -z));
        int index = Math.floorMod((int) Math.round(degreesClockwiseFromNorth / 45.0), DIRECTION_KEYS.length);
        return Text.translatable("message.aerodynamics4mc.wind_meter.direction." + DIRECTION_KEYS[index]);
    }

    private static String format(double value) {
        return String.format(Locale.ROOT, "%.2f", value);
    }

    private static String signed(double value) {
        return String.format(Locale.ROOT, "%+.2f", value);
    }
}
