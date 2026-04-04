package com.aerodynamics4mc.runtime;

import net.minecraft.server.world.ServerWorld;

interface SeedTerrainProvider {
    TerrainSample sample(ServerWorld world, int blockX, int blockZ);

    record TerrainSample(
        float terrainHeightBlocks,
        float biomeTemperature,
        float roughnessLengthMeters,
        byte surfaceClass
    ) {
    }
}
