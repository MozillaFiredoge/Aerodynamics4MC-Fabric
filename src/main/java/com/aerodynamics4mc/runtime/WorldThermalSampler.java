package com.aerodynamics4mc.runtime;

import net.minecraft.block.BlockState;
import net.minecraft.block.Blocks;
import net.minecraft.util.math.BlockPos;
import net.minecraft.util.math.MathHelper;
import net.minecraft.world.LightType;
import net.minecraft.world.World;

public final class WorldThermalSampler {
    private static final int SAMPLE_SPACING = 6;
    private static final float CENTER_SAMPLE_WEIGHT = 2.0f;
    private static final float THERMAL_LUMINANCE_SCALE = 3.0f;
    private static final float THERMAL_BIOME_SCALE = 2.2f;
    private static final float THERMAL_SKYLIGHT_SCALE = 2.4f;
    private static final float THERMAL_ALTITUDE_SCALE = 1.2f;
    private static final float THERMAL_MAX_ABS = 8.0f;

    private WorldThermalSampler() {
    }

    public static float sampleAveragedAnomaly(World world, int worldX, int worldY, int worldZ) {
        if (world == null) {
            return 0.0f;
        }

        BlockPos.Mutable cursor = new BlockPos.Mutable();
        float weighted = 0.0f;
        float weightSum = 0.0f;

        weighted += CENTER_SAMPLE_WEIGHT * samplePointAnomaly(world, cursor.set(worldX, worldY, worldZ));
        weightSum += CENTER_SAMPLE_WEIGHT;

        weighted += samplePointAnomaly(world, cursor.set(worldX + SAMPLE_SPACING, worldY, worldZ));
        weighted += samplePointAnomaly(world, cursor.set(worldX - SAMPLE_SPACING, worldY, worldZ));
        weighted += samplePointAnomaly(world, cursor.set(worldX, worldY + SAMPLE_SPACING, worldZ));
        weighted += samplePointAnomaly(world, cursor.set(worldX, worldY - SAMPLE_SPACING, worldZ));
        weighted += samplePointAnomaly(world, cursor.set(worldX, worldY, worldZ + SAMPLE_SPACING));
        weighted += samplePointAnomaly(world, cursor.set(worldX, worldY, worldZ - SAMPLE_SPACING));
        weightSum += 6.0f;

        if (weightSum <= 0.0f) {
            return 0.0f;
        }
        return MathHelper.clamp(weighted / weightSum, -THERMAL_MAX_ABS, THERMAL_MAX_ABS);
    }

    private static float samplePointAnomaly(World world, BlockPos pos) {
        BlockState state = world.getBlockState(pos);
        float luminanceTerm = (state.getLuminance() / 15.0f) * THERMAL_LUMINANCE_SCALE;
        float hotspotTerm = sampleHotspotTerm(state);

        int skyLight = world.getLightLevel(LightType.SKY, pos);
        float skyTerm = ((skyLight / 15.0f) - 0.35f) * THERMAL_SKYLIGHT_SCALE;

        float biomeTemperature = world.getBiome(pos).value().getTemperature();
        float biomeTerm = (biomeTemperature - 0.8f) * THERMAL_BIOME_SCALE;

        float altitudeNorm = MathHelper.clamp((pos.getY() - 70.0f) / 192.0f, -1.2f, 1.2f);
        float altitudeTerm = -altitudeNorm * THERMAL_ALTITUDE_SCALE;

        float weatherCooling = 0.0f;
        if (world.isRaining()) {
            weatherCooling += 0.6f;
        }
        if (world.isThundering()) {
            weatherCooling += 0.9f;
        }

        return MathHelper.clamp(
            luminanceTerm + hotspotTerm + skyTerm + biomeTerm + altitudeTerm - weatherCooling,
            -THERMAL_MAX_ABS,
            THERMAL_MAX_ABS
        );
    }

    private static float sampleHotspotTerm(BlockState state) {
        if (state.isOf(Blocks.LAVA) || state.isOf(Blocks.MAGMA_BLOCK) || state.isOf(Blocks.FIRE) || state.isOf(Blocks.SOUL_FIRE)) {
            return 5.0f;
        }
        if (state.isOf(Blocks.CAMPFIRE) || state.isOf(Blocks.SOUL_CAMPFIRE)) {
            return 3.0f;
        }
        if (state.isOf(Blocks.BLAST_FURNACE) || state.isOf(Blocks.FURNACE) || state.isOf(Blocks.SMOKER)) {
            return 1.2f;
        }
        if (state.isOf(Blocks.TORCH) || state.isOf(Blocks.WALL_TORCH) || state.isOf(Blocks.LANTERN) || state.isOf(Blocks.SOUL_LANTERN)) {
            return 0.8f;
        }
        return 0.0f;
    }
}
