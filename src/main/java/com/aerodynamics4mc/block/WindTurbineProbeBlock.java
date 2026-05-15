package com.aerodynamics4mc.block;

import com.aerodynamics4mc.api.GameplayWindSample;
import com.mojang.serialization.MapCodec;

import net.minecraft.ChatFormatting;
import net.minecraft.core.BlockPos;
import net.minecraft.core.Direction;
import net.minecraft.network.chat.Component;
import net.minecraft.server.level.ServerLevel;
import net.minecraft.world.InteractionHand;
import net.minecraft.world.InteractionResult;
import net.minecraft.world.entity.player.Player;
import net.minecraft.world.item.ItemStack;
import net.minecraft.world.level.BlockGetter;
import net.minecraft.world.level.Level;
import net.minecraft.world.level.block.BaseEntityBlock;
import net.minecraft.world.level.block.RenderShape;
import net.minecraft.world.level.block.entity.BlockEntity;
import net.minecraft.world.level.block.entity.BlockEntityTicker;
import net.minecraft.world.level.block.entity.BlockEntityType;
import net.minecraft.world.level.block.state.BlockState;
import net.minecraft.world.phys.BlockHitResult;
import org.jspecify.annotations.Nullable;

import java.util.Locale;

public class WindTurbineProbeBlock extends BaseEntityBlock {
    public static final MapCodec<WindTurbineProbeBlock> CODEC = simpleCodec(WindTurbineProbeBlock::new);

    protected WindTurbineProbeBlock(Properties properties) {
        super(properties);
    }

    @Override
    protected MapCodec<? extends BaseEntityBlock> codec() {
        return CODEC;
    }

    @Override
    public @Nullable BlockEntity newBlockEntity(BlockPos blockPos, BlockState blockState) {
        return new WindTurbineProbeBlockEntity(blockPos, blockState);
    }

    @Override
    public @Nullable <T extends BlockEntity> BlockEntityTicker<T> getTicker(Level level, BlockState blockState, BlockEntityType<T> blockEntityType) {
        if (level.isClientSide()) {
            return null;
        }
        return createTickerHelper(blockEntityType, ModBlocks.WIND_TURBINE_PROBE_BLOCK_ENTITY, WindTurbineProbeBlockEntity::tick);
    }

    @Override
    protected RenderShape getRenderShape(BlockState blockState) {
        return RenderShape.MODEL;
    }


    @Override
    protected InteractionResult useWithoutItem(BlockState blockState, Level level, BlockPos blockPos, Player player, BlockHitResult blockHitResult) {
        return showStatus(blockState, level, blockPos, player);
    }


    @Override
    protected InteractionResult useItemOn(ItemStack itemStack, BlockState blockState, Level level, BlockPos blockPos, Player player, InteractionHand interactionHand, BlockHitResult blockHitResult) {
        return showStatus(blockState, level, blockPos, player);
    }

    @Override
    protected boolean isSignalSource(BlockState blockState) {
        return true;
    }

    @Override
    protected int getSignal(BlockState blockState, BlockGetter blockGetter, BlockPos blockPos, Direction direction) {
        return redstonePower(blockGetter, blockPos);
    }

    @Override
    protected int getDirectSignal(BlockState blockState, BlockGetter blockGetter, BlockPos blockPos, Direction direction) {
        return redstonePower(blockGetter, blockPos);
    }

    private InteractionResult showStatus(BlockState state, Level world, BlockPos pos, Player player) {
        if (world.isClientSide()) {
            return InteractionResult.SUCCESS;
        }
        if (!(world instanceof ServerLevel serverWorld)) {
            return InteractionResult.PASS;
        }
        BlockEntity blockEntity = world.getBlockEntity(pos);
        if (!(blockEntity instanceof WindTurbineProbeBlockEntity probe)) {
            return InteractionResult.PASS;
        }
        probe.sampleNow(serverWorld, state);
        if (!probe.hasSample()) {
            player.displayClientMessage(Component.translatable("message.aerodynamics4mc.wind_turbine_probe.no_flow").withStyle(ChatFormatting.GRAY), false);
            return InteractionResult.SUCCESS_SERVER;
        }

        GameplayWindSample sample = probe.lastSample();
        player.displayClientMessage(
                Component.translatable(
                        "message.aerodynamics4mc.wind_turbine_probe.status",
                        format(probe.lastPowerWatts()),
                        probe.redstonePower()
                ).withStyle(ChatFormatting.GOLD),
                false
        );
        player.displayClientMessage(
                Component.translatable(
                        "message.aerodynamics4mc.wind_turbine_probe.wind",
                        format(sample.effectiveSpeedMetersPerSecond()),
                        format(sample.meanSpeedMetersPerSecond()),
                        format(sample.gustVelocity().length()),
                        signed(sample.updraftMetersPerSecond())
                ).withStyle(ChatFormatting.AQUA),
                false
        );
        player.displayClientMessage(
                Component.translatable(
                        "message.aerodynamics4mc.wind_turbine_probe.source",
                        sample.sourceLevel().name(),
                        sample.authority().name(),
                        percent(sample.confidence()),
                        percent(sample.shelterFactor()),
                        format(sample.turbulenceIntensity())
                ).withStyle(ChatFormatting.GRAY),
                false
        );
        return InteractionResult.SUCCESS_SERVER;
    }

    private static int redstonePower(BlockGetter world, BlockPos pos) {
        BlockEntity blockEntity = world.getBlockEntity(pos);
        if (blockEntity instanceof WindTurbineProbeBlockEntity probe) {
            return probe.redstonePower();
        }
        return 0;
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
