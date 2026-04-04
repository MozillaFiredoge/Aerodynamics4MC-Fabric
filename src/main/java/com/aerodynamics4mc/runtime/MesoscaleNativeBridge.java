package com.aerodynamics4mc.runtime;

final class MesoscaleNativeBridge {
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

    boolean isLoaded() {
        return LOADED;
    }

    String getLoadError() {
        return LOAD_ERROR;
    }

    Transport deriveTransport(
        int nx,
        int ny,
        int nz,
        float dxMeters,
        float dtSeconds,
        float molecularNuMeters2PerSecond,
        float prandtlAir,
        float turbulentPrandtl
    ) {
        if (!LOADED) {
            return null;
        }
        float[] values = nativeDeriveTransport(
            nx,
            ny,
            nz,
            dxMeters,
            dtSeconds,
            molecularNuMeters2PerSecond,
            prandtlAir,
            turbulentPrandtl
        );
        if (values == null || values.length < 5) {
            return null;
        }
        return new Transport(values[0], values[1], values[2], values[3], values[4]);
    }

    long createContext(
        int nx,
        int ny,
        int nz,
        float dxMeters,
        float dtSeconds,
        float molecularNuMeters2PerSecond,
        float prandtlAir,
        float turbulentPrandtl
    ) {
        if (!LOADED) {
            return 0L;
        }
        return nativeCreateContext(
            nx,
            ny,
            nz,
            dxMeters,
            dtSeconds,
            molecularNuMeters2PerSecond,
            prandtlAir,
            turbulentPrandtl
        );
    }

    boolean stepContext(
        long contextKey,
        int nx,
        int ny,
        int nz,
        float dxMeters,
        float dtSeconds,
        float molecularNuMeters2PerSecond,
        float prandtlAir,
        float turbulentPrandtl,
        float[] forcing,
        float[] outState
    ) {
        return LOADED
            && contextKey != 0L
            && nativeStepContext(
                contextKey,
                nx,
                ny,
                nz,
                dxMeters,
                dtSeconds,
                molecularNuMeters2PerSecond,
                prandtlAir,
                turbulentPrandtl,
                forcing,
                outState
            );
    }

    void releaseContext(long contextKey) {
        if (!LOADED || contextKey == 0L) {
            return;
        }
        nativeReleaseContext(contextKey);
    }

    record Transport(
        float velocityScaleMetersPerSecond,
        float molecularNuLattice,
        float molecularAlphaLattice,
        float molecularTauShear,
        float molecularTauThermal
    ) {
    }

    private static native float[] nativeDeriveTransport(
        int nx,
        int ny,
        int nz,
        float dxMeters,
        float dtSeconds,
        float molecularNuMeters2PerSecond,
        float prandtlAir,
        float turbulentPrandtl
    );

    private static native long nativeCreateContext(
        int nx,
        int ny,
        int nz,
        float dxMeters,
        float dtSeconds,
        float molecularNuMeters2PerSecond,
        float prandtlAir,
        float turbulentPrandtl
    );

    private static native boolean nativeStepContext(
        long contextKey,
        int nx,
        int ny,
        int nz,
        float dxMeters,
        float dtSeconds,
        float molecularNuMeters2PerSecond,
        float prandtlAir,
        float turbulentPrandtl,
        float[] forcing,
        float[] outState
    );

    private static native void nativeReleaseContext(long contextKey);
}
