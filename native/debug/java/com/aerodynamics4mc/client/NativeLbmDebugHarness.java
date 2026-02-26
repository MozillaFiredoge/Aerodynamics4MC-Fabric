package com.aerodynamics4mc.client;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public final class NativeLbmDebugHarness {
    private static final int INPUT_CHANNELS = 9;
    private static final int OUTPUT_CHANNELS = 4;

    private static final int CH_OBS = 0;
    private static final int CH_FAN_MASK = 1;
    private static final int CH_FAN_VX = 2;
    private static final int CH_FAN_VY = 3;
    private static final int CH_FAN_VZ = 4;
    private static final int CH_VX = 5;
    private static final int CH_VY = 6;
    private static final int CH_VZ = 7;
    private static final int CH_P = 8;

    private record Metrics(double mass, double momentumX, double momentumY, double momentumZ,
                           double kineticEnergy, double pressureMean, double maxSpeed,
                           double avgDivL1, int fluidCells) {
    }

    private record TestResult(String name, boolean pass, String detail) {
    }

    private NativeLbmDebugHarness() {
    }

    public static void main(String[] args) {
        int grid = parseIntArg(args, "--grid", 16);
        int steps = parseIntArg(args, "--steps", 48);

        NativeLbmBridge bridge = new NativeLbmBridge();
        if (!bridge.isLoaded()) {
            System.err.println("[debug] native load failed: " + bridge.getLoadError());
            System.exit(2);
            return;
        }

        if (!bridge.ensureInitialized(grid, INPUT_CHANNELS, OUTPUT_CHANNELS)) {
            System.err.println("[debug] native init failed");
            System.exit(2);
            return;
        }

        List<TestResult> results = new ArrayList<>();
        results.add(runEquilibriumInvariant(bridge, grid, Math.min(steps, 24)));
        results.add(runMassConservation(bridge, grid, steps));
        results.add(runObstacleNoSlip(bridge, grid, Math.min(steps, 20)));
        results.add(runFanInjection(bridge, grid, Math.min(steps, 24)));

        boolean allPass = true;
        System.out.println("\n==== Native LBM Debug Summary ====");
        for (TestResult r : results) {
            System.out.printf("[%s] %s - %s%n", r.pass ? "PASS" : "FAIL", r.name, r.detail);
            allPass &= r.pass;
        }

        System.out.println("Runtime: " + bridge.runtimeInfo());
        System.out.println("Timing : " + bridge.timingInfo());
        bridge.shutdown();

        if (!allPass) {
            System.exit(1);
        }
    }

    private static TestResult runEquilibriumInvariant(NativeLbmBridge bridge, int n, int steps) {
        float[] state = new float[n * n * n * INPUT_CHANNELS];
        long context = 101L;

        Metrics m0 = null;
        Metrics mt = null;
        for (int s = 0; s < steps; s++) {
            float[] out = step(bridge, state, n, context);
            if (out == null) {
                return new TestResult("equilibrium_invariant", false, "native step returned null");
            }
            mt = metrics(out, state, n);
            if (m0 == null) {
                m0 = mt;
            }
            writeBackState(state, out);
        }

        bridge.releaseContext(context);
        if (m0 == null || mt == null) {
            return new TestResult("equilibrium_invariant", false, "no metrics generated");
        }

        double massDrift = relDiff(mt.mass, m0.mass);
        boolean pass = mt.maxSpeed < 1e-6 && Math.abs(mt.pressureMean) < 1e-6 && massDrift < 1e-8;
        String detail = String.format("maxSpeed=%.3e pressureMean=%.3e massDrift=%.3e", mt.maxSpeed, mt.pressureMean, massDrift);
        return new TestResult("equilibrium_invariant", pass, detail);
    }

    private static TestResult runMassConservation(NativeLbmBridge bridge, int n, int steps) {
        float[] state = new float[n * n * n * INPUT_CHANNELS];
        Random rng = new Random(7);
        int cells = n * n * n;
        for (int i = 0; i < cells; i++) {
            int base = i * INPUT_CHANNELS;
            state[base + CH_VX] = (rng.nextFloat() - 0.5f) * 0.06f;
            state[base + CH_VY] = (rng.nextFloat() - 0.5f) * 0.06f;
            state[base + CH_VZ] = (rng.nextFloat() - 0.5f) * 0.06f;
            state[base + CH_P] = 0.0f;
        }

        long context = 102L;
        Metrics m0 = null;
        double worstMassDrift = 0.0;
        double worstEnergyJump = 0.0;
        double prevEnergy = Double.NaN;

        for (int s = 0; s < steps; s++) {
            float[] out = step(bridge, state, n, context);
            if (out == null) {
                return new TestResult("mass_conservation", false, "native step returned null");
            }
            Metrics mt = metrics(out, state, n);
            if (m0 == null) {
                m0 = mt;
                prevEnergy = mt.kineticEnergy;
            } else {
                worstMassDrift = Math.max(worstMassDrift, relDiff(mt.mass, m0.mass));
                worstEnergyJump = Math.max(worstEnergyJump, relDiff(mt.kineticEnergy, prevEnergy));
                prevEnergy = mt.kineticEnergy;
            }
            writeBackState(state, out);
        }

        bridge.releaseContext(context);
        // Zero-gradient/open boundaries are not strictly mass conservative over long rollouts.
        boolean pass = worstMassDrift < 6e-3;
        String detail = String.format("worstMassDrift=%.3e worstEnergyStepJump=%.3e", worstMassDrift, worstEnergyJump);
        return new TestResult("mass_conservation", pass, detail);
    }

    private static TestResult runObstacleNoSlip(NativeLbmBridge bridge, int n, int steps) {
        float[] state = new float[n * n * n * INPUT_CHANNELS];
        Random rng = new Random(11);
        int cells = n * n * n;
        for (int i = 0; i < cells; i++) {
            int base = i * INPUT_CHANNELS;
            state[base + CH_VX] = (rng.nextFloat() - 0.5f) * 0.2f;
            state[base + CH_VY] = (rng.nextFloat() - 0.5f) * 0.2f;
            state[base + CH_VZ] = (rng.nextFloat() - 0.5f) * 0.2f;
        }

        int c0 = n / 3;
        int c1 = (2 * n) / 3;
        for (int x = c0; x < c1; x++) {
            for (int y = c0; y < c1; y++) {
                for (int z = c0; z < c1; z++) {
                    int base = idx(x, y, z, n) * INPUT_CHANNELS;
                    state[base + CH_OBS] = 1.0f;
                }
            }
        }

        long context = 103L;
        double maxInside = 0.0;
        for (int s = 0; s < steps; s++) {
            float[] out = step(bridge, state, n, context);
            if (out == null) {
                return new TestResult("obstacle_no_slip", false, "native step returned null");
            }
            maxInside = Math.max(maxInside, maxSpeedInsideObstacle(out, state, n));
            writeBackState(state, out);
        }

        bridge.releaseContext(context);
        boolean pass = maxInside < 1e-6;
        String detail = String.format("maxSpeedInsideObstacle=%.3e", maxInside);
        return new TestResult("obstacle_no_slip", pass, detail);
    }

    private static TestResult runFanInjection(NativeLbmBridge bridge, int n, int steps) {
        float[] state = new float[n * n * n * INPUT_CHANNELS];
        int cy = n / 2;
        int cz = n / 2;
        int radius = Math.max(2, n / 8);

        for (int y = 0; y < n; y++) {
            for (int z = 0; z < n; z++) {
                int dy = y - cy;
                int dz = z - cz;
                if (dy * dy + dz * dz > radius * radius) {
                    continue;
                }
                int base = idx(1, y, z, n) * INPUT_CHANNELS;
                state[base + CH_FAN_MASK] = 1.0f;
                state[base + CH_FAN_VX] = 1.5f;
                state[base + CH_FAN_VY] = 0.0f;
                state[base + CH_FAN_VZ] = 0.0f;
            }
        }

        long context = 104L;
        double nearAvg = 0.0;
        double farAvg = 0.0;
        double px = 0.0;

        for (int s = 0; s < steps; s++) {
            float[] out = step(bridge, state, n, context);
            if (out == null) {
                return new TestResult("fan_injection", false, "native step returned null");
            }
            nearAvg = averageSpeedInPlane(out, n, 2);
            farAvg = averageSpeedInPlane(out, n, n - 3);
            Metrics m = metrics(out, state, n);
            px = m.momentumX;
            writeBackState(state, out);
        }

        bridge.releaseContext(context);
        boolean pass = nearAvg > 5e-4 && nearAvg > (farAvg * 1.2) && px > 1e-2;
        String detail = String.format("nearAvg=%.3e farAvg=%.3e momentumX=%.3e", nearAvg, farAvg, px);
        return new TestResult("fan_injection", pass, detail);
    }

    private static float[] step(NativeLbmBridge bridge, float[] state, int n, long context) {
        return bridge.step(toLittleEndianBytes(state), n, OUTPUT_CHANNELS, context);
    }

    private static Metrics metrics(float[] out, float[] state, int n) {
        double mass = 0.0;
        double mx = 0.0;
        double my = 0.0;
        double mz = 0.0;
        double ke = 0.0;
        double pMean = 0.0;
        double maxSpeed = 0.0;
        double divL1 = 0.0;
        int fluidCells = 0;

        for (int x = 0; x < n; x++) {
            for (int y = 0; y < n; y++) {
                for (int z = 0; z < n; z++) {
                    int cell = idx(x, y, z, n);
                    int baseIn = cell * INPUT_CHANNELS;
                    if (state[baseIn + CH_OBS] > 0.5f) {
                        continue;
                    }
                    int baseOut = cell * OUTPUT_CHANNELS;
                    double vx = out[baseOut];
                    double vy = out[baseOut + 1];
                    double vz = out[baseOut + 2];
                    double p = out[baseOut + 3];
                    double rho = p + 1.0;

                    fluidCells += 1;
                    mass += rho;
                    mx += rho * vx;
                    my += rho * vy;
                    mz += rho * vz;
                    ke += 0.5 * rho * (vx * vx + vy * vy + vz * vz);
                    pMean += p;

                    double speed = Math.sqrt(vx * vx + vy * vy + vz * vz);
                    maxSpeed = Math.max(maxSpeed, speed);

                    double div = divergence(out, x, y, z, n);
                    divL1 += Math.abs(div);
                }
            }
        }

        if (fluidCells > 0) {
            pMean /= fluidCells;
            divL1 /= fluidCells;
        }
        return new Metrics(mass, mx, my, mz, ke, pMean, maxSpeed, divL1, fluidCells);
    }

    private static double divergence(float[] out, int x, int y, int z, int n) {
        double dvx = sampleVel(out, x + 1, y, z, n, 0) - sampleVel(out, x - 1, y, z, n, 0);
        double dvy = sampleVel(out, x, y + 1, z, n, 1) - sampleVel(out, x, y - 1, z, n, 1);
        double dvz = sampleVel(out, x, y, z + 1, n, 2) - sampleVel(out, x, y, z - 1, n, 2);
        return 0.5 * (dvx + dvy + dvz);
    }

    private static double sampleVel(float[] out, int x, int y, int z, int n, int comp) {
        int cx = clamp(x, 0, n - 1);
        int cy = clamp(y, 0, n - 1);
        int cz = clamp(z, 0, n - 1);
        int base = idx(cx, cy, cz, n) * OUTPUT_CHANNELS;
        return out[base + comp];
    }

    private static double maxSpeedInsideObstacle(float[] out, float[] state, int n) {
        double max = 0.0;
        int cells = n * n * n;
        for (int i = 0; i < cells; i++) {
            int inBase = i * INPUT_CHANNELS;
            if (state[inBase + CH_OBS] <= 0.5f) {
                continue;
            }
            int outBase = i * OUTPUT_CHANNELS;
            double vx = out[outBase];
            double vy = out[outBase + 1];
            double vz = out[outBase + 2];
            max = Math.max(max, Math.sqrt(vx * vx + vy * vy + vz * vz));
        }
        return max;
    }

    private static double averageSpeedInPlane(float[] out, int n, int xPlane) {
        int x = clamp(xPlane, 0, n - 1);
        double sum = 0.0;
        int count = 0;
        for (int y = 0; y < n; y++) {
            for (int z = 0; z < n; z++) {
                int base = idx(x, y, z, n) * OUTPUT_CHANNELS;
                double vx = out[base];
                double vy = out[base + 1];
                double vz = out[base + 2];
                sum += Math.sqrt(vx * vx + vy * vy + vz * vz);
                count += 1;
            }
        }
        return count > 0 ? (sum / count) : 0.0;
    }

    private static void writeBackState(float[] state, float[] out) {
        int cells = out.length / OUTPUT_CHANNELS;
        for (int i = 0; i < cells; i++) {
            int inBase = i * INPUT_CHANNELS;
            int outBase = i * OUTPUT_CHANNELS;
            state[inBase + CH_VX] = out[outBase];
            state[inBase + CH_VY] = out[outBase + 1];
            state[inBase + CH_VZ] = out[outBase + 2];
            state[inBase + CH_P] = out[outBase + 3];
            if (state[inBase + CH_OBS] > 0.5f) {
                state[inBase + CH_VX] = 0.0f;
                state[inBase + CH_VY] = 0.0f;
                state[inBase + CH_VZ] = 0.0f;
                state[inBase + CH_P] = 0.0f;
            }
        }
    }

    private static byte[] toLittleEndianBytes(float[] values) {
        ByteBuffer buf = ByteBuffer.allocate(values.length * Float.BYTES).order(ByteOrder.LITTLE_ENDIAN);
        for (float v : values) {
            buf.putFloat(v);
        }
        return buf.array();
    }

    private static int idx(int x, int y, int z, int n) {
        return (x * n + y) * n + z;
    }

    private static int clamp(int v, int lo, int hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static double relDiff(double a, double b) {
        return Math.abs(a - b) / Math.max(1e-12, Math.abs(b));
    }

    private static int parseIntArg(String[] args, String flag, int fallback) {
        for (int i = 0; i < args.length - 1; i++) {
            if (flag.equals(args[i])) {
                return Integer.parseInt(args[i + 1]);
            }
        }
        return fallback;
    }
}
