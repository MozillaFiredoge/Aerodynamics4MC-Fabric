package com.aerodynamics4mc.api;

import net.minecraft.world.phys.Vec3;

public final class AeroWindSamplingRules {
    public static final float FAST_PLAYER_HORIZONTAL_SPEED_THRESHOLD_MPS = 7.0f;

    private AeroWindSamplingRules() {
    }

    public static boolean isFastPlayerVelocity(Vec3 velocity) {
        return horizontalSpeedMetersPerSecond(velocity) > FAST_PLAYER_HORIZONTAL_SPEED_THRESHOLD_MPS;
    }

    public static float horizontalSpeedMetersPerSecond(Vec3 velocity) {
        if (velocity == null) {
            return 0.0f;
        }
        double horizontalSquared = velocity.x * velocity.x + velocity.z * velocity.z;
        if (!Double.isFinite(horizontalSquared) || horizontalSquared <= 0.0) {
            return 0.0f;
        }
        return (float) (Math.sqrt(horizontalSquared) * 20.0);
    }
}
