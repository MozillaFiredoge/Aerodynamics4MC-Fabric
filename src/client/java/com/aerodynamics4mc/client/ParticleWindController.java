package com.aerodynamics4mc.client;

import net.minecraft.client.world.ClientWorld;
import net.minecraft.util.math.MathHelper;
import net.minecraft.util.math.Vec3d;

public final class ParticleWindController {
    private static final double MIN_WIND_SPEED = 0.01;

    private ParticleWindController() {}

    public static Vec3d applyLeaves(ClientWorld world, double x, double y, double z, Vec3d velocity) {
        Vec3d wind = sampleWind(world, x, y, z);
        if (wind.lengthSquared() < MIN_WIND_SPEED * MIN_WIND_SPEED) {
            return velocity;
        }
        return applyHorizontalResponse(velocity, wind, 0.060, 0.42);
    }

    public static Vec3d applyCampfireSmoke(ClientWorld world, double x, double y, double z, Vec3d velocity) {
        Vec3d wind = sampleWind(world, x, y + 0.6, z);
        if (wind.lengthSquared() < MIN_WIND_SPEED * MIN_WIND_SPEED) {
            return velocity;
        }
        Vec3d next = applyHorizontalResponse(velocity, wind, 0.035, 0.18);
        double updraft = Math.max(0.0, wind.y) * 0.015;
        return new Vec3d(next.x, Math.min(next.y + updraft, 0.28), next.z);
    }

    public static Vec3d applyTorchSmoke(ClientWorld world, double x, double y, double z, Vec3d velocity) {
        Vec3d wind = sampleWind(world, x, y + 0.25, z);
        if (wind.lengthSquared() < MIN_WIND_SPEED * MIN_WIND_SPEED) {
            return velocity;
        }
        Vec3d next = applyHorizontalResponse(velocity, wind, 0.024, 0.10);
        return new Vec3d(next.x, next.y + Math.max(0.0, wind.y) * 0.006, next.z);
    }

    public static Vec3d applyTorchFlame(ClientWorld world, double x, double y, double z, Vec3d velocity) {
        Vec3d wind = sampleWind(world, x, y + 0.12, z);
        if (wind.lengthSquared() < MIN_WIND_SPEED * MIN_WIND_SPEED) {
            return velocity;
        }
        Vec3d next = applyHorizontalResponse(velocity, wind, 0.012, 0.050);
        double vertical = MathHelper.clamp(velocity.y + wind.y * 0.003, -0.02, 0.12);
        return new Vec3d(next.x, vertical, next.z);
    }

    private static Vec3d sampleWind(ClientWorld world, double x, double y, double z) {
        return AeroClientMod.sampleWind(world, new Vec3d(x, y, z));
    }

    private static Vec3d applyHorizontalResponse(Vec3d velocity, Vec3d wind, double response, double maxHorizontalSpeed) {
        double nextX = velocity.x + wind.x * response;
        double nextZ = velocity.z + wind.z * response;
        double horizontalSpeed = Math.sqrt(nextX * nextX + nextZ * nextZ);
        if (horizontalSpeed > maxHorizontalSpeed && horizontalSpeed > 1.0e-6) {
            double scale = maxHorizontalSpeed / horizontalSpeed;
            nextX *= scale;
            nextZ *= scale;
        }
        return new Vec3d(nextX, velocity.y, nextZ);
    }
}
