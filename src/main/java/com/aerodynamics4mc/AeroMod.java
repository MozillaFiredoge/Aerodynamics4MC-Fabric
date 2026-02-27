package com.aerodynamics4mc;

import com.aerodynamics4mc.net.AeroNetworking;
import com.aerodynamics4mc.runtime.AeroServerRuntime;

import net.fabricmc.api.ModInitializer;

public class AeroMod implements ModInitializer {
    @Override
    public void onInitialize() {
        ModBlocks.register();
        AeroNetworking.registerPayloadTypes();
        AeroServerRuntime.init();
    }
}
