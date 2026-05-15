package com.aerodynamics4mc.api.client;

import com.aerodynamics4mc.api.AeroWindSample;
import com.aerodynamics4mc.api.SamplePolicy;
import com.aerodynamics4mc.client.AeroClientMod;
import net.minecraft.client.multiplayer.ClientLevel;
import net.minecraft.core.BlockPos;
import net.minecraft.world.phys.Vec3;

public final class AeroClientWindApi {
    private AeroClientWindApi() {
    }

    public static AeroWindSample sample(ClientLevel world, Vec3 position) {
        return AeroClientMod.sampleFlow(world, position);
    }

    public static AeroWindSample sample(ClientLevel world, Vec3 position, SamplePolicy policy) {
        return AeroClientMod.sampleFlow(world, position, policy);
    }

    public static AeroWindSample sample(ClientLevel world, BlockPos position) {
        return sample(world, center(position));
    }

    public static AeroWindSample sample(ClientLevel world, BlockPos position, SamplePolicy policy) {
        return sample(world, center(position), policy);
    }

    public static Vec3 sampleMeanVelocity(ClientLevel world, Vec3 position) {
        return sample(world, position).meanVelocity();
    }

    public static Vec3 sampleEffectiveVelocity(ClientLevel world, Vec3 position) {
        return sample(world, position).effectiveVelocity();
    }

    private static Vec3 center(BlockPos position) {
        if (position == null) {
            return Vec3.ZERO;
        }
        return new Vec3(position.getX() + 0.5, position.getY() + 0.5, position.getZ() + 0.5);
    }
}
