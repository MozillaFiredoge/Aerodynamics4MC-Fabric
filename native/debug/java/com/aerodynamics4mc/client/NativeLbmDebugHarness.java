package com.aerodynamics4mc.client;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.net.Socket;
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
    private static final float INFLOW_SPEED = 8.0f;
    private static final float NATIVE_SCALE = 30.0f;

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
        boolean socketSmoke = hasFlag(args, "--socket-smoke");
        String socketHost = parseStringArg(args, "--socket-host", "127.0.0.1");
        int socketPort = parseIntArg(args, "--socket-port", 5001);

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
        if (socketSmoke) {
            results.add(runBackendUnitConsistency(bridge, grid, Math.min(steps, 24), socketHost, socketPort));
        }

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

    private static TestResult runBackendUnitConsistency(NativeLbmBridge bridge, int n, int steps, String host, int port) {
        float[] stateNative = new float[n * n * n * INPUT_CHANNELS];
        float[] stateSocket = new float[n * n * n * INPUT_CHANNELS];
        seedFanInflow(stateNative, n);
        seedFanInflow(stateSocket, n);

        long nativeContext = 201L;
        double nativeNear = 0.0;
        double socketNear = 0.0;
        double nativeMax = 0.0;
        double socketMax = 0.0;

        try (Socket socket = new Socket(host, port);
             DataInputStream in = new DataInputStream(new BufferedInputStream(socket.getInputStream()));
             DataOutputStream out = new DataOutputStream(new BufferedOutputStream(socket.getOutputStream()))) {
            for (int s = 0; s < steps; s++) {
                float[] nativeOut = stepNativeScaled(bridge, stateNative, n, nativeContext);
                if (nativeOut == null) {
                    bridge.releaseContext(nativeContext);
                    return new TestResult("backend_unit_consistency", false, "native step returned null");
                }
                float[] socketOut = stepSocket(in, out, stateSocket, n);
                if (socketOut == null) {
                    bridge.releaseContext(nativeContext);
                    return new TestResult("backend_unit_consistency", false, "socket step returned null");
                }

                nativeNear = averageSpeedInPlane(nativeOut, n, 2);
                socketNear = averageSpeedInPlane(socketOut, n, 2);
                nativeMax = Math.max(nativeMax, maxSpeed(nativeOut));
                socketMax = Math.max(socketMax, maxSpeed(socketOut));

                writeBackState(stateNative, nativeOut);
                writeBackState(stateSocket, socketOut);
            }
        } catch (IOException e) {
            bridge.releaseContext(nativeContext);
            return new TestResult("backend_unit_consistency", false, "socket error: " + e.getMessage());
        }

        bridge.releaseContext(nativeContext);

        double nativeNorm = nativeNear / INFLOW_SPEED;
        double socketNorm = socketNear / INFLOW_SPEED;
        double ratio = nativeNorm / Math.max(1e-12, socketNorm);

        // Smoke-level consistency: both backends should stay on the same unit order wrt INFLOW_SPEED.
        boolean pass = nativeNorm > 1e-4 && socketNorm > 1e-4 && ratio > 0.20 && ratio < 5.0;
        String detail = String.format(
            "near/native=%.3e(%.3f*inflow) near/socket=%.3e(%.3f*inflow) ratio=%.3f max(native/socket)=%.3e/%.3e",
            nativeNear, nativeNorm, socketNear, socketNorm, ratio, nativeMax, socketMax
        );
        return new TestResult("backend_unit_consistency", pass, detail);
    }

    private static void seedFanInflow(float[] state, int n) {
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
                state[base + CH_FAN_VX] = INFLOW_SPEED;
                state[base + CH_FAN_VY] = 0.0f;
                state[base + CH_FAN_VZ] = 0.0f;
            }
        }
    }

    private static float[] stepNativeScaled(NativeLbmBridge bridge, float[] stateClientScale, int n, long context) {
        float[] nativePayloadState = stateClientScale.clone();
        int cells = n * n * n;
        for (int i = 0; i < cells; i++) {
            int base = i * INPUT_CHANNELS;
            nativePayloadState[base + CH_VX] /= NATIVE_SCALE;
            nativePayloadState[base + CH_VY] /= NATIVE_SCALE;
            nativePayloadState[base + CH_VZ] /= NATIVE_SCALE;
        }
        float[] out = step(bridge, nativePayloadState, n, context);
        if (out == null) {
            return null;
        }
        for (int i = 0; i < cells; i++) {
            int base = i * OUTPUT_CHANNELS;
            out[base] *= NATIVE_SCALE;
            out[base + 1] *= NATIVE_SCALE;
            out[base + 2] *= NATIVE_SCALE;
        }
        return out;
    }

    private static float[] stepSocket(DataInputStream in, DataOutputStream out, float[] state, int n) throws IOException {
        byte[] payload = toLittleEndianBytes(state);
        out.writeInt(payload.length);
        out.write(payload);
        out.flush();

        int length = in.readInt();
        int expected = n * n * n * OUTPUT_CHANNELS * Float.BYTES;
        if (length != expected) {
            throw new IOException("unexpected response length: " + length + " expected " + expected);
        }
        byte[] response = in.readNBytes(length);
        if (response.length != length) {
            throw new IOException("socket closed during read");
        }
        ByteBuffer buf = ByteBuffer.wrap(response).order(ByteOrder.LITTLE_ENDIAN);
        float[] outValues = new float[n * n * n * OUTPUT_CHANNELS];
        buf.asFloatBuffer().get(outValues);
        return outValues;
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

    private static double maxSpeed(float[] out) {
        double max = 0.0;
        int cells = out.length / OUTPUT_CHANNELS;
        for (int i = 0; i < cells; i++) {
            int base = i * OUTPUT_CHANNELS;
            double vx = out[base];
            double vy = out[base + 1];
            double vz = out[base + 2];
            max = Math.max(max, Math.sqrt(vx * vx + vy * vy + vz * vz));
        }
        return max;
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

    private static String parseStringArg(String[] args, String flag, String fallback) {
        for (int i = 0; i < args.length - 1; i++) {
            if (flag.equals(args[i])) {
                return args[i + 1];
            }
        }
        return fallback;
    }

    private static boolean hasFlag(String[] args, String flag) {
        for (String arg : args) {
            if (flag.equals(arg)) {
                return true;
            }
        }
        return false;
    }
}
