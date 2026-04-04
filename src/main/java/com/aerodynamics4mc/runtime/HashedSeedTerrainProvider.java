package com.aerodynamics4mc.runtime;

import net.minecraft.server.world.ServerWorld;
import net.minecraft.util.math.MathHelper;

final class HashedSeedTerrainProvider implements SeedTerrainProvider {
    static final byte SURFACE_CLASS_WATER = 0;
    static final byte SURFACE_CLASS_PLAINS = 1;
    static final byte SURFACE_CLASS_FOREST = 2;
    static final byte SURFACE_CLASS_ROCK = 3;
    static final byte SURFACE_CLASS_SNOW = 4;

    @Override
    public TerrainSample sample(ServerWorld world, int blockX, int blockZ) {
        long seed = world.getSeed();
        float largeScale = fbm(seed ^ 0x6A09E667F3BCC909L, blockX, blockZ, 1024.0f, 3, 0.55f);
        float mediumScale = fbm(seed ^ 0xBB67AE8584CAA73BL, blockX, blockZ, 256.0f, 3, 0.50f);
        float tempNoise = fbm(seed ^ 0x3C6EF372FE94F82BL, blockX, blockZ, 1536.0f, 2, 0.60f);
        float moistureNoise = fbm(seed ^ 0xA54FF53A5F1D36F1L, blockX, blockZ, 768.0f, 3, 0.55f);

        float seaLevel = world.getSeaLevel();
        float terrainHeightBlocks = seaLevel + largeScale * 48.0f + mediumScale * 14.0f;
        float elevationNormalized = MathHelper.clamp((terrainHeightBlocks - seaLevel) / 96.0f, -1.0f, 1.0f);
        float biomeTemperature = MathHelper.clamp(
            0.85f + tempNoise * 0.65f - Math.max(0.0f, elevationNormalized) * 0.45f,
            0.0f,
            2.0f
        );
        float moisture = MathHelper.clamp(0.5f + moistureNoise * 0.5f, 0.0f, 1.0f);

        byte surfaceClass;
        float roughnessLengthMeters;
        if (terrainHeightBlocks < seaLevel - 1.0f) {
            surfaceClass = SURFACE_CLASS_WATER;
            roughnessLengthMeters = 0.0003f;
        } else if (biomeTemperature < 0.25f && terrainHeightBlocks > seaLevel + 8.0f) {
            surfaceClass = SURFACE_CLASS_SNOW;
            roughnessLengthMeters = 0.003f;
        } else if (elevationNormalized > 0.45f || moisture < 0.22f) {
            surfaceClass = SURFACE_CLASS_ROCK;
            roughnessLengthMeters = 0.08f;
        } else if (moisture > 0.58f) {
            surfaceClass = SURFACE_CLASS_FOREST;
            roughnessLengthMeters = 0.9f;
        } else {
            surfaceClass = SURFACE_CLASS_PLAINS;
            roughnessLengthMeters = 0.05f;
        }

        return new TerrainSample(terrainHeightBlocks, biomeTemperature, roughnessLengthMeters, surfaceClass);
    }

    private float fbm(long seed, int x, int z, float baseScale, int octaves, float gain) {
        float amplitude = 1.0f;
        float frequency = 1.0f / Math.max(1.0f, baseScale);
        float sum = 0.0f;
        float norm = 0.0f;
        for (int i = 0; i < octaves; i++) {
            sum += amplitude * valueNoise(seed + i * 0x9E3779B97F4A7C15L, x * frequency, z * frequency);
            norm += amplitude;
            amplitude *= gain;
            frequency *= 2.0f;
        }
        return norm > 0.0f ? sum / norm : 0.0f;
    }

    private float valueNoise(long seed, float x, float z) {
        int x0 = MathHelper.floor(x);
        int z0 = MathHelper.floor(z);
        int x1 = x0 + 1;
        int z1 = z0 + 1;
        float tx = x - x0;
        float tz = z - z0;
        float sx = smoothstep(tx);
        float sz = smoothstep(tz);

        float n00 = hashToSignedUnit(seed, x0, z0);
        float n10 = hashToSignedUnit(seed, x1, z0);
        float n01 = hashToSignedUnit(seed, x0, z1);
        float n11 = hashToSignedUnit(seed, x1, z1);

        float nx0 = MathHelper.lerp(sx, n00, n10);
        float nx1 = MathHelper.lerp(sx, n01, n11);
        return MathHelper.lerp(sz, nx0, nx1);
    }

    private float smoothstep(float t) {
        return t * t * (3.0f - 2.0f * t);
    }

    private float hashToSignedUnit(long seed, int x, int z) {
        long h = seed;
        h ^= (long) x * 0x9E3779B97F4A7C15L;
        h ^= (long) z * 0xC2B2AE3D27D4EB4FL;
        h = mix64(h);
        long bits = (h >>> 40) & 0xFFFFFFL;
        return (bits / (float) 0x7FFFFF) - 1.0f;
    }

    private long mix64(long z) {
        z = (z ^ (z >>> 33)) * 0xff51afd7ed558ccdl;
        z = (z ^ (z >>> 33)) * 0xc4ceb9fe1a85ec53L;
        return z ^ (z >>> 33);
    }
}
