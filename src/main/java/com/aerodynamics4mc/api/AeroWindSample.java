package com.aerodynamics4mc.api;

import net.minecraft.world.phys.Vec3;

public record AeroWindSample(
    float velocityX,
    float velocityY,
    float velocityZ,
    float pressure,
    Level level,
    Authority authority,
    long l1Epoch,
    long worldDeltaEpoch,
    long l2Epoch,
    float confidence,
    float temperatureKelvin,
    float humidity,
    float turbulenceIntensity,
    float gustX,
    float gustY,
    float gustZ,
    float windShearXPerBlock,
    float windShearZPerBlock,
    float ablStability,
    float ablMixingStrength
) {
    public static final long UNKNOWN_EPOCH = -1L;
    public static final float UNKNOWN_SCALAR = Float.NaN;
    public static final AeroWindSample ZERO = new AeroWindSample(
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        Level.NONE,
        Authority.NONE,
        UNKNOWN_EPOCH,
        UNKNOWN_EPOCH,
        UNKNOWN_EPOCH,
        0.0f,
        UNKNOWN_SCALAR,
        UNKNOWN_SCALAR,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f,
        0.0f
    );

    public AeroWindSample {
        velocityX = finiteOrZero(velocityX);
        velocityY = finiteOrZero(velocityY);
        velocityZ = finiteOrZero(velocityZ);
        pressure = finiteOrZero(pressure);
        level = level == null ? Level.NONE : level;
        authority = authority == null ? Authority.NONE : authority;
        confidence = level == Level.NONE || authority == Authority.NONE ? 0.0f : clamp01(confidence);
        temperatureKelvin = finiteOrUnknown(temperatureKelvin);
        humidity = Float.isFinite(humidity) ? clamp01(humidity) : UNKNOWN_SCALAR;
        turbulenceIntensity = nonNegativeFinite(turbulenceIntensity);
        gustX = finiteOrZero(gustX);
        gustY = finiteOrZero(gustY);
        gustZ = finiteOrZero(gustZ);
        windShearXPerBlock = finiteOrZero(windShearXPerBlock);
        windShearZPerBlock = finiteOrZero(windShearZPerBlock);
        ablStability = finiteOrZero(ablStability);
        ablMixingStrength = nonNegativeFinite(ablMixingStrength);
    }

    public AeroWindSample(
        float velocityX,
        float velocityY,
        float velocityZ,
        float pressure,
        Level level,
        Authority authority,
        long l1Epoch,
        long worldDeltaEpoch,
        long l2Epoch,
        float confidence
    ) {
        this(
            velocityX,
            velocityY,
            velocityZ,
            pressure,
            level,
            authority,
            l1Epoch,
            worldDeltaEpoch,
            l2Epoch,
            confidence,
            UNKNOWN_SCALAR,
            UNKNOWN_SCALAR,
            0.0f,
            0.0f,
            0.0f,
            0.0f,
            0.0f,
            0.0f,
            0.0f,
            0.0f
        );
    }

    public static AeroWindSample serverAuthoritative(
        float velocityX,
        float velocityY,
        float velocityZ,
        float pressure,
        Level level,
        long l1Epoch,
        long worldDeltaEpoch,
        long l2Epoch
    ) {
        return new AeroWindSample(
            velocityX,
            velocityY,
            velocityZ,
            pressure,
            level,
            Authority.SERVER_AUTHORITATIVE,
            l1Epoch,
            worldDeltaEpoch,
            l2Epoch,
            1.0f
        );
    }

    public static AeroWindSample serverAggregatedL2(
        float velocityX,
        float velocityY,
        float velocityZ,
        float pressure,
        long l1Epoch,
        long worldDeltaEpoch,
        long l2Epoch,
        float confidence
    ) {
        return new AeroWindSample(
            velocityX,
            velocityY,
            velocityZ,
            pressure,
            Level.L2,
            Authority.SERVER_AGGREGATED,
            l1Epoch,
            worldDeltaEpoch,
            l2Epoch,
            confidence
        );
    }

    public static AeroWindSample clientLocalL2(
        float velocityX,
        float velocityY,
        float velocityZ,
        float pressure,
        long l1Epoch,
        long worldDeltaEpoch,
        long l2Epoch
    ) {
        return new AeroWindSample(
            velocityX,
            velocityY,
            velocityZ,
            pressure,
            Level.L2,
            Authority.CLIENT_LOCAL,
            l1Epoch,
            worldDeltaEpoch,
            l2Epoch,
            1.0f
        );
    }

    public Vec3 velocity() {
        return new Vec3(velocityX, velocityY, velocityZ);
    }

    public Vec3 meanVelocity() {
        return velocity();
    }

    public Vec3 gustVelocity() {
        return new Vec3(gustX, gustY, gustZ);
    }

    public Vec3 velocityWithGust() {
        return new Vec3(velocityX + gustX, velocityY + gustY, velocityZ + gustZ);
    }

    public Vec3 effectiveVelocity() {
        return velocityWithGust();
    }

    public float speedMetersPerSecond() {
        return (float) Math.sqrt(velocityX * velocityX + velocityY * velocityY + velocityZ * velocityZ);
    }

    public float horizontalSpeedMetersPerSecond() {
        return (float) Math.sqrt(velocityX * velocityX + velocityZ * velocityZ);
    }

    public float windShearMagnitudePerBlock() {
        return (float) Math.sqrt(windShearXPerBlock * windShearXPerBlock + windShearZPerBlock * windShearZPerBlock);
    }

    public boolean hasTemperature() {
        return Float.isFinite(temperatureKelvin);
    }

    public boolean hasHumidity() {
        return Float.isFinite(humidity);
    }

    public boolean hasTurbulence() {
        return turbulenceIntensity > 0.0f;
    }

    public boolean hasGust() {
        return gustX != 0.0f || gustY != 0.0f || gustZ != 0.0f;
    }

    public boolean hasWindShear() {
        return windShearMagnitudePerBlock() > 0.0f;
    }

    public boolean hasAtmosphericDiagnostics() {
        return hasTemperature()
            || hasHumidity()
            || hasTurbulence()
            || hasWindShear()
            || ablMixingStrength > 0.0f;
    }

    public AeroWindSample withAtmosphere(
        float temperatureKelvin,
        float humidity,
        float turbulenceIntensity,
        float gustX,
        float gustY,
        float gustZ,
        float windShearXPerBlock,
        float windShearZPerBlock,
        float ablStability,
        float ablMixingStrength
    ) {
        return new AeroWindSample(
            velocityX,
            velocityY,
            velocityZ,
            pressure,
            level,
            authority,
            l1Epoch,
            worldDeltaEpoch,
            l2Epoch,
            confidence,
            temperatureKelvin,
            humidity,
            Math.max(0.0f, finiteOrZero(turbulenceIntensity)),
            finiteOrZero(gustX),
            finiteOrZero(gustY),
            finiteOrZero(gustZ),
            finiteOrZero(windShearXPerBlock),
            finiteOrZero(windShearZPerBlock),
            finiteOrZero(ablStability),
            Math.max(0.0f, finiteOrZero(ablMixingStrength))
        );
    }

    public AeroWindSample withVelocityAndPressure(
        float velocityX,
        float velocityY,
        float velocityZ,
        float pressure,
        Level level,
        Authority authority,
        long l1Epoch,
        long worldDeltaEpoch,
        long l2Epoch
    ) {
        return new AeroWindSample(
            velocityX,
            velocityY,
            velocityZ,
            pressure,
            level,
            authority,
            l1Epoch,
            worldDeltaEpoch,
            l2Epoch,
            confidence,
            temperatureKelvin,
            humidity,
            turbulenceIntensity,
            gustX,
            gustY,
            gustZ,
            windShearXPerBlock,
            windShearZPerBlock,
            ablStability,
            ablMixingStrength
        );
    }

    public boolean hasFlow() {
        return level != Level.NONE && authority != Authority.NONE;
    }

    public boolean isServerTrusted() {
        return authority == Authority.SERVER_AUTHORITATIVE || authority == Authority.SERVER_AGGREGATED;
    }

    public boolean isTrustedForGameplay() {
        return isServerTrusted();
    }

    public boolean isClientLocal() {
        return authority == Authority.CLIENT_LOCAL;
    }

    public boolean isLocalVoxelFlow() {
        return level == Level.L2;
    }

    public Level sourceLevel() {
        return level;
    }

    public long freshnessEpoch() {
        long latest = UNKNOWN_EPOCH;
        if (l1Epoch > latest) {
            latest = l1Epoch;
        }
        if (worldDeltaEpoch > latest) {
            latest = worldDeltaEpoch;
        }
        if (l2Epoch > latest) {
            latest = l2Epoch;
        }
        return latest;
    }

    public boolean hasFreshnessEpoch() {
        return freshnessEpoch() != UNKNOWN_EPOCH;
    }

    public long ageTicks(long currentTick) {
        long epoch = freshnessEpoch();
        if (epoch == UNKNOWN_EPOCH || currentTick < epoch) {
            return Long.MAX_VALUE;
        }
        return currentTick - epoch;
    }

    public boolean isFresh(long currentTick, long maxAgeTicks) {
        if (maxAgeTicks < 0L) {
            return false;
        }
        long age = ageTicks(currentTick);
        return age != Long.MAX_VALUE && age <= maxAgeTicks;
    }

    public AeroWindSample withConfidence(float confidence) {
        return new AeroWindSample(
            velocityX,
            velocityY,
            velocityZ,
            pressure,
            level,
            authority,
            l1Epoch,
            worldDeltaEpoch,
            l2Epoch,
            confidence,
            temperatureKelvin,
            humidity,
            turbulenceIntensity,
            gustX,
            gustY,
            gustZ,
            windShearXPerBlock,
            windShearZPerBlock,
            ablStability,
            ablMixingStrength
        );
    }

    public Source source() {
        if (level == Level.L0) {
            return Source.L0_BACKGROUND;
        }
        if (level == Level.L1) {
            return Source.L1_COARSE;
        }
        if (level == Level.L2) {
            return Source.L2_BRICK;
        }
        return Source.NONE;
    }

    private static float finiteOrZero(float value) {
        return Float.isFinite(value) ? value : 0.0f;
    }

    private static float finiteOrUnknown(float value) {
        return Float.isFinite(value) ? value : UNKNOWN_SCALAR;
    }

    private static float nonNegativeFinite(float value) {
        return Math.max(0.0f, finiteOrZero(value));
    }

    private static float clamp01(float value) {
        return Math.max(0.0f, Math.min(1.0f, finiteOrZero(value)));
    }

    public enum Level {
        NONE,
        L0,
        L1,
        L2
    }

    public enum Authority {
        NONE,
        SERVER_AUTHORITATIVE,
        SERVER_AGGREGATED,
        CLIENT_LOCAL,
        CLIENT_REMOTE,
        UNTRUSTED
    }

    @Deprecated
    public enum Source {
        NONE,
        L0_BACKGROUND,
        L1_COARSE,
        L2_BRICK
    }
}
