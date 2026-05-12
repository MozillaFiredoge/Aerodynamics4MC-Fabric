package com.aerodynamics4mc.api;

public enum SamplePolicy {
    /** Prefer client-local L2 when available, otherwise use server L1/L0 coarse wind. */
    CLIENT_LOCAL_PREFERRED(true, false, true),
    /** Debug/visual path that may use client-local L2, server L2, then server L1/L0. */
    VISUAL_LOCAL_FIRST(true, true, true),
    /** Server-side gameplay path: trusted server L2 when active, otherwise server L1/L0. */
    GAMEPLAY_SERVER_ONLY(false, true, true),
    /** Legacy/compat client path: server L2 atlas when present, otherwise L1/L0. */
    SERVER_AGGREGATED_PREFERRED(false, true, true),
    /** Coarse authoritative weather only: L1 first, then L0. */
    SERVER_COARSE_ONLY(false, false, true),
    /** Diagnostic path that allows every available source. Do not use for trusted gameplay. */
    DIAGNOSTIC_ALL_SOURCES(true, true, true);

    private final boolean allowClientLocalL2;
    private final boolean allowServerAggregatedL2;
    private final boolean allowServerCoarse;

    SamplePolicy(boolean allowClientLocalL2, boolean allowServerAggregatedL2, boolean allowServerCoarse) {
        this.allowClientLocalL2 = allowClientLocalL2;
        this.allowServerAggregatedL2 = allowServerAggregatedL2;
        this.allowServerCoarse = allowServerCoarse;
    }

    public boolean allowClientLocalL2() {
        return allowClientLocalL2;
    }

    public boolean allowServerAggregatedL2() {
        return allowServerAggregatedL2;
    }

    public boolean allowServerCoarse() {
        return allowServerCoarse;
    }

    public boolean allowAnyL2() {
        return allowClientLocalL2 || allowServerAggregatedL2;
    }

    public boolean serverOnly() {
        return !allowClientLocalL2;
    }

    public boolean trustedForServerGameplay() {
        return !allowClientLocalL2;
    }
}
