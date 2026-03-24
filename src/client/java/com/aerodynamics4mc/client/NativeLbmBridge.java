package com.aerodynamics4mc.client;

import java.nio.ByteBuffer;

public final class NativeLbmBridge {
    private static final String LIB_NAME = "aero_lbm";
    private static final boolean LOADED;
    private static final String LOAD_ERROR;

    static {
        boolean loaded = false;
        String error = "";
        try {
            NativeLibraryLoader.loadBundled(LIB_NAME);
            loaded = true;
        } catch (Throwable t) {
            error = t.getClass().getSimpleName() + ": " + t.getMessage();
        }
        LOADED = loaded;
        LOAD_ERROR = error;
    }

    private boolean initialized;
    private int gridSize = -1;
    private int inputChannels = -1;
    private int outputChannels = -1;

    public boolean isLoaded() {
        return LOADED;
    }

    public String getLoadError() {
        return LOAD_ERROR;
    }

    public synchronized boolean ensureInitialized(int grid, int inputCh, int outputCh) {
        if (!LOADED) {
            return false;
        }
        if (initialized && grid == gridSize && inputCh == inputChannels && outputCh == outputChannels) {
            return true;
        }
        if (initialized) {
            nativeShutdown();
            initialized = false;
        }
        initialized = nativeInit(grid, inputCh, outputCh);
        if (initialized) {
            gridSize = grid;
            inputChannels = inputCh;
            outputChannels = outputCh;
        }
        return initialized;
    }

    public synchronized float[] step(byte[] payload, int grid, int outputCh, long contextKey) {
        if (!initialized) {
            return null;
        }
        int cellCount = grid * grid * grid;
        float[] output = new float[cellCount * outputCh];
        boolean ok = step(payload, grid, outputCh, contextKey, output);
        if (!ok) {
            return null;
        }
        return output;
    }

    public synchronized boolean step(byte[] payload, int grid, int outputCh, long contextKey, float[] output) {
        if (!initialized) {
            return false;
        }
        int cellCount = grid * grid * grid;
        if (output == null || output.length != cellCount * outputCh) {
            return false;
        }
        boolean ok = nativeStep(payload, grid, contextKey, output);
        if (!ok) {
            return false;
        }
        return true;
    }

    public synchronized boolean step(ByteBuffer payload, int grid, int outputCh, long contextKey, float[] output) {
        if (!initialized || payload == null || !payload.isDirect()) {
            return false;
        }
        int cellCount = grid * grid * grid;
        if (output == null || output.length != cellCount * outputCh) {
            return false;
        }
        int payloadBytes = cellCount * inputChannels * Float.BYTES;
        if (payload.capacity() != payloadBytes) {
            return false;
        }
        boolean ok = nativeStepDirect(payload, grid, contextKey, output);
        if (!ok) {
            return false;
        }
        return true;
    }

    public synchronized void shutdown() {
        if (!initialized || !LOADED) {
            return;
        }
        nativeShutdown();
        initialized = false;
        gridSize = -1;
        inputChannels = -1;
        outputChannels = -1;
    }

    public synchronized void releaseContext(long contextKey) {
        if (!initialized || !LOADED) {
            return;
        }
        nativeReleaseContext(contextKey);
    }

    public synchronized String runtimeInfo() {
        if (!LOADED) {
            return "not_loaded";
        }
        if (!initialized) {
            return "not_initialized";
        }
        return nativeRuntimeInfo();
    }

    public synchronized String timingInfo() {
        if (!LOADED || !initialized) {
            return "ticks=0";
        }
        return nativeTimingInfo();
    }

    public synchronized boolean getTemperatureState(int grid, long contextKey, float[] temperatureState) {
        if (!initialized || temperatureState == null) {
            return false;
        }
        int cellCount = grid * grid * grid;
        if (temperatureState.length != cellCount) {
            return false;
        }
        return nativeGetTemperatureState(grid, contextKey, temperatureState);
    }

    public synchronized boolean setTemperatureState(int grid, long contextKey, float[] temperatureState) {
        if (!initialized || temperatureState == null) {
            return false;
        }
        int cellCount = grid * grid * grid;
        if (temperatureState.length != cellCount) {
            return false;
        }
        return nativeSetTemperatureState(grid, contextKey, temperatureState);
    }

    private static native boolean nativeInit(int gridSize, int inputChannels, int outputChannels);

    private static native boolean nativeStep(byte[] payload, int gridSize, long contextKey, float[] outputFlow);

    private static native boolean nativeStepDirect(ByteBuffer payload, int gridSize, long contextKey, float[] outputFlow);

    private static native void nativeReleaseContext(long contextKey);

    private static native void nativeShutdown();

    private static native String nativeRuntimeInfo();

    private static native String nativeTimingInfo();

    private static native boolean nativeGetTemperatureState(int gridSize, long contextKey, float[] temperatureState);

    private static native boolean nativeSetTemperatureState(int gridSize, long contextKey, float[] temperatureState);
}
