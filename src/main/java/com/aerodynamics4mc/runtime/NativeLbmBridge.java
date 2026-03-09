package com.aerodynamics4mc.runtime;

public final class NativeLbmBridge {
    private static final String LIB_NAME = "aero_lbm";
    private static final boolean LOADED;
    private static final String LOAD_ERROR;
    private static final Object GLOBAL_LOCK = new Object();
    private static boolean globalInitialized;
    private static int globalInputChannels = -1;
    private static int globalOutputChannels = -1;
    private static int globalActiveHandles = 0;

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
        synchronized (GLOBAL_LOCK) {
            if (!globalInitialized) {
                globalInitialized = nativeInit(Math.max(1, grid), inputCh, outputCh);
                if (!globalInitialized) {
                    return false;
                }
                globalInputChannels = inputCh;
                globalOutputChannels = outputCh;
            } else if (inputCh != globalInputChannels || outputCh != globalOutputChannels) {
                return false;
            }

            if (!initialized) {
                initialized = true;
                globalActiveHandles++;
            }
            gridSize = grid;
            inputChannels = inputCh;
            outputChannels = outputCh;
            return true;
        }
    }

    public synchronized float[] step(byte[] payload, int grid, int outputCh, long contextKey) {
        if (!initialized) {
            return null;
        }
        int cellCount = grid * grid * grid;
        float[] output = new float[cellCount * outputCh];
        synchronized (GLOBAL_LOCK) {
            if (!globalInitialized) {
                return null;
            }
            boolean ok = nativeStep(payload, grid, contextKey, output);
            if (!ok) {
                return null;
            }
            return output;
        }
    }

    public synchronized void shutdown() {
        if (!initialized || !LOADED) {
            return;
        }
        initialized = false;
        gridSize = -1;
        inputChannels = -1;
        outputChannels = -1;
        synchronized (GLOBAL_LOCK) {
            if (globalActiveHandles > 0) {
                globalActiveHandles--;
            }
            if (globalActiveHandles == 0 && globalInitialized) {
                nativeShutdown();
                globalInitialized = false;
                globalInputChannels = -1;
                globalOutputChannels = -1;
            }
        }
    }

    public synchronized void releaseContext(long contextKey) {
        if (!initialized || !LOADED) {
            return;
        }
        synchronized (GLOBAL_LOCK) {
            if (!globalInitialized) {
                return;
            }
            nativeReleaseContext(contextKey);
        }
    }

    public synchronized String runtimeInfo() {
        if (!LOADED) {
            return "not_loaded";
        }
        synchronized (GLOBAL_LOCK) {
            if (!globalInitialized) {
                return "not_initialized";
            }
            return nativeRuntimeInfo();
        }
    }

    public synchronized String timingInfo() {
        if (!LOADED) {
            return "ticks=0";
        }
        synchronized (GLOBAL_LOCK) {
            if (!globalInitialized) {
                return "ticks=0";
            }
            return nativeTimingInfo();
        }
    }

    private static native boolean nativeInit(int gridSize, int inputChannels, int outputChannels);

    private static native boolean nativeStep(byte[] payload, int gridSize, long contextKey, float[] outputFlow);

    private static native void nativeReleaseContext(long contextKey);

    private static native void nativeShutdown();

    private static native String nativeRuntimeInfo();

    private static native String nativeTimingInfo();
}
