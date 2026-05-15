package com.aerodynamics4mc.client;

import com.aerodynamics4mc.api.SamplePolicy;
import com.aerodynamics4mc.api.client.AeroClientWindApi;
import net.minecraft.client.multiplayer.ClientLevel;
import net.minecraft.util.Mth;
import net.minecraft.world.phys.Vec3;

public final class ParticleWindController {
    private static final double MIN_WIND_SPEED = 0.01;

    private ParticleWindController() {}

    public static Vec3 applyLeaves(ClientLevel world, double x, double y, double z, Vec3 velocity) {
        Vec3 wind = sampleWind(world, x, y, z);
        if (wind.lengthSqr() < MIN_WIND_SPEED * MIN_WIND_SPEED) {
            return velocity;
        }
        return applyHorizontalResponse(velocity, wind, 0.060, 0.21);
    }

    public static Vec3 applyCampfireSmoke(ClientLevel world, double x, double y, double z, Vec3 velocity) {
        Vec3 wind = sampleWind(world, x, y + 0.6, z);
        if (wind.lengthSqr() < MIN_WIND_SPEED * MIN_WIND_SPEED) {
            return velocity;
        }
        Vec3 next = applyHorizontalResponse(velocity, wind, 0.035, 0.18);
        double updraft = Math.max(0.0, wind.y) * 0.015;
        return new Vec3(next.x, Math.min(next.y + updraft, 0.28), next.z);
    }

    public static Vec3 applyTorchSmoke(ClientLevel world, double x, double y, double z, Vec3 velocity) {
        Vec3 wind = sampleWind(world, x, y + 0.25, z);
        if (wind.lengthSqr() < MIN_WIND_SPEED * MIN_WIND_SPEED) {
            return velocity;
        }
        Vec3 next = applyHorizontalResponse(velocity, wind, 0.024, 0.10);
        return new Vec3(next.x, next.y + Math.max(0.0, wind.y) * 0.006, next.z);
    }

    public static Vec3 applyTorchFlame(ClientLevel world, double x, double y, double z, Vec3 velocity) {
        Vec3 wind = sampleWind(world, x, y + 0.12, z);
        if (wind.lengthSqr() < MIN_WIND_SPEED * MIN_WIND_SPEED) {
            return velocity;
        }
        Vec3 next = applyHorizontalResponse(velocity, wind, 0.012, 0.050);
        double vertical = Mth.clamp(velocity.y + wind.y * 0.003, -0.02, 0.12);
        return new Vec3(next.x, vertical, next.z);
    }

    private static Vec3 sampleWind(ClientLevel world, double x, double y, double z) {
        return AeroClientWindApi.sample(world, new Vec3(x, y, z), SamplePolicy.CLIENT_LOCAL_PREFERRED).effectiveVelocity();
    }

    private static Vec3 applyHorizontalResponse(Vec3 velocity, Vec3 wind, double response, double maxHorizontalSpeed) {
        double nextX = velocity.x + wind.x * response;
        double nextZ = velocity.z + wind.z * response;
        double horizontalSpeed = Math.sqrt(nextX * nextX + nextZ * nextZ);
        if (horizontalSpeed > maxHorizontalSpeed && horizontalSpeed > 1.0e-6) {
            double scale = maxHorizontalSpeed / horizontalSpeed;
            nextX *= scale;
            nextZ *= scale;
        }
        return new Vec3(nextX, velocity.y, nextZ);
    }
}
