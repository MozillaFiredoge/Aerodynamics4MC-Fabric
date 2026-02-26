package com.aerodynamics4mc;

import net.fabricmc.api.ModInitializer;

public class AeroMod implements ModInitializer {
    @Override
    public void onInitialize() {
        ModBlocks.register();
    }
}
