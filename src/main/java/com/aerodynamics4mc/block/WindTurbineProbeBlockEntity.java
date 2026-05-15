package com.aerodynamics4mc.block;

import com.aerodynamics4mc.api.AeroWindApi;
import com.aerodynamics4mc.api.GameplayWindSample;
import com.aerodynamics4mc.api.SamplePolicy;
import net.minecraft.core.BlockPos;
import net.minecraft.server.level.ServerLevel;
import net.minecraft.world.level.Level;
import net.minecraft.world.level.block.entity.BlockEntity;
import net.minecraft.world.level.block.state.BlockState;
import net.minecraft.world.phys.Vec3;


public class WindTurbineProbeBlockEntity extends BlockEntity {
    private static final long SAMPLE_INTERVAL_TICKS = 20L;
    private static final double AIR_DENSITY_KG_PER_M3 = 1.225;
    // Virtual large rotor keeps the v0.1 API demo observable even under gentle L1 wind.
    private static final double ROTOR_AREA_M2 = 64.0;
    private static final double POWER_COEFFICIENT = 0.35;
    private static final double CUT_IN_SPEED_MPS = 0.1;
    private static final double RATED_POWER_WATTS = 60.0;

    private GameplayWindSample lastSample = GameplayWindSample.ZERO;
    private double lastPowerWatts;
    private int redstonePower;
    private long lastSampleTick = -1L;

    public WindTurbineProbeBlockEntity(BlockPos pos, BlockState state) {
        super(ModBlocks.WIND_TURBINE_PROBE_BLOCK_ENTITY, pos, state);
    }

    public static void tick(Level world, BlockPos pos, BlockState state, WindTurbineProbeBlockEntity blockEntity) {
        if (!(world instanceof ServerLevel serverWorld)) {
            return;
        }
        long time = serverWorld.getGameTime();
        if (Math.floorMod(time + pos.asLong(), SAMPLE_INTERVAL_TICKS) != 0L) {
            return;
        }
        blockEntity.sampleNow(serverWorld, state);
    }

    public void sampleNow(ServerLevel world, BlockState state) {
        Vec3 samplePos = new Vec3(worldPosition.getX() + 0.5, worldPosition.getY() + 1.5, worldPosition.getZ() + 0.5);
        GameplayWindSample sample = AeroWindApi.sampleGameplay(world, samplePos, SamplePolicy.GAMEPLAY_SERVER_ONLY);
        int previousPower = redstonePower;
        lastSample = sample;
        lastSampleTick = world.getGameTime();
        lastPowerWatts = sample.hasFlow() ? estimatePowerWatts(sample) : 0.0;
        redstonePower = redstoneFromPower(lastPowerWatts);
        if (redstonePower != previousPower) {
            setChanged();
            world.updateNeighborsAt(worldPosition, state.getBlock(), null);
        }
    }

    public GameplayWindSample lastSample() {
        return lastSample;
    }

    public double lastPowerWatts() {
        return lastPowerWatts;
    }

    public int redstonePower() {
        return redstonePower;
    }

    public long lastSampleTick() {
        return lastSampleTick;
    }

    public boolean hasSample() {
        return lastSampleTick >= 0L && lastSample.hasFlow();
    }

    private static double estimatePowerWatts(GameplayWindSample sample) {
        double speed = Math.max(0.0, sample.effectiveSpeedMetersPerSecond() - CUT_IN_SPEED_MPS);
        double rawPower = 0.5 * AIR_DENSITY_KG_PER_M3 * ROTOR_AREA_M2 * POWER_COEFFICIENT * speed * speed * speed;
        double shelterDerate = 1.0 - clamp01(sample.shelterFactor()) * 0.65;
        double turbulenceDerate = 1.0 / (1.0 + Math.max(0.0, sample.turbulenceIntensity()) * 0.18);
        double confidenceDerate = clamp01(sample.confidence());
        return Math.max(0.0, rawPower * shelterDerate * turbulenceDerate * confidenceDerate);
    }

    private static int redstoneFromPower(double powerWatts) {
        if (!(powerWatts > 0.0) || !Double.isFinite(powerWatts)) {
            return 0;
        }
        return (int) Math.round(clamp01(powerWatts / RATED_POWER_WATTS) * 15.0);
    }

    private static double clamp01(double value) {
        if (!Double.isFinite(value)) {
            return 0.0;
        }
        return Math.max(0.0, Math.min(1.0, value));
    }
}
