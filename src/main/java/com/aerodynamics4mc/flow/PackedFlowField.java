package com.aerodynamics4mc.flow;

public final class PackedFlowField {
    public static final int CHANNELS = 4;
    public static final float VELOCITY_RANGE = 5.6f;
    public static final float PRESSURE_RANGE = 0.03f;

    private PackedFlowField() {}

    public static int atlasResolution(short[] packedFlow) {
        int sampleCount = packedFlow == null ? 0 : packedFlow.length / CHANNELS;
        return Math.max(1, Math.round((float) Math.cbrt(sampleCount)));
    }

    public static float decodeVelocity(short value) {
        return value / 32767.0f * VELOCITY_RANGE;
    }

    public static float decodePressure(short value) {
        return value / 32767.0f * PRESSURE_RANGE;
    }

    public static float[] reconstructFullState(short[] packedFlow, int sampleStride, int fullResolution) {
        int atlasResolution = atlasResolution(packedFlow);
        float[] out = new float[fullResolution * fullResolution * fullResolution * CHANNELS];
        for (int x = 0; x < fullResolution; x++) {
            for (int y = 0; y < fullResolution; y++) {
                for (int z = 0; z < fullResolution; z++) {
                    int dst = ((x * fullResolution + y) * fullResolution + z) * CHANNELS;
                    samplePackedFlow(packedFlow, atlasResolution, sampleStride, x, y, z, out, dst);
                }
            }
        }
        return out;
    }

    public static void samplePackedFlow(
        short[] packedFlow,
        int atlasResolution,
        int sampleStride,
        int cellX,
        int cellY,
        int cellZ,
        float[] out,
        int outOffset
    ) {
        double gx = sampleStride <= 0 ? 0.0 : (double) cellX / sampleStride;
        double gy = sampleStride <= 0 ? 0.0 : (double) cellY / sampleStride;
        double gz = sampleStride <= 0 ? 0.0 : (double) cellZ / sampleStride;
        int x0 = clamp((int) Math.floor(gx), 0, atlasResolution - 1);
        int y0 = clamp((int) Math.floor(gy), 0, atlasResolution - 1);
        int z0 = clamp((int) Math.floor(gz), 0, atlasResolution - 1);
        int x1 = Math.min(x0 + 1, atlasResolution - 1);
        int y1 = Math.min(y0 + 1, atlasResolution - 1);
        int z1 = Math.min(z0 + 1, atlasResolution - 1);
        float fx = (float) (gx - x0);
        float fy = (float) (gy - y0);
        float fz = (float) (gz - z0);
        for (int channel = 0; channel < CHANNELS; channel++) {
            float c000 = loadComponent(packedFlow, atlasResolution, x0, y0, z0, channel);
            float c100 = loadComponent(packedFlow, atlasResolution, x1, y0, z0, channel);
            float c010 = loadComponent(packedFlow, atlasResolution, x0, y1, z0, channel);
            float c110 = loadComponent(packedFlow, atlasResolution, x1, y1, z0, channel);
            float c001 = loadComponent(packedFlow, atlasResolution, x0, y0, z1, channel);
            float c101 = loadComponent(packedFlow, atlasResolution, x1, y0, z1, channel);
            float c011 = loadComponent(packedFlow, atlasResolution, x0, y1, z1, channel);
            float c111 = loadComponent(packedFlow, atlasResolution, x1, y1, z1, channel);
            float c00 = lerp(c000, c100, fx);
            float c10 = lerp(c010, c110, fx);
            float c01 = lerp(c001, c101, fx);
            float c11 = lerp(c011, c111, fx);
            float c0 = lerp(c00, c10, fy);
            float c1 = lerp(c01, c11, fy);
            out[outOffset + channel] = lerp(c0, c1, fz);
        }
    }

    private static float loadComponent(short[] packedFlow, int atlasResolution, int x, int y, int z, int channel) {
        int index = ((x * atlasResolution + y) * atlasResolution + z) * CHANNELS + channel;
        short value = packedFlow[index];
        return channel == 3 ? decodePressure(value) : decodeVelocity(value);
    }

    private static int clamp(int value, int min, int max) {
        return Math.max(min, Math.min(max, value));
    }

    private static float lerp(float a, float b, float t) {
        return a + (b - a) * clamp01(t);
    }

    private static float clamp01(float value) {
        return Math.max(0.0f, Math.min(1.0f, value));
    }
}
