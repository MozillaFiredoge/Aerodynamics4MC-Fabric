package com.aerodynamics4mc.runtime;

import net.minecraft.registry.entry.RegistryEntry;
import net.minecraft.registry.tag.BiomeTags;
import net.minecraft.server.world.ServerChunkManager;
import net.minecraft.server.world.ServerWorld;
import net.minecraft.util.math.MathHelper;
import net.minecraft.world.Heightmap;
import net.minecraft.world.biome.Biome;
import net.minecraft.world.gen.chunk.ChunkGenerator;
import net.minecraft.world.gen.noise.NoiseConfig;

final class WorldgenSeedTerrainProvider implements SeedTerrainProvider {
    private static final float WATER_ROUGHNESS_LENGTH_METERS = 0.0003f;
    private static final float PLAINS_ROUGHNESS_LENGTH_METERS = 0.05f;
    private static final float FOREST_ROUGHNESS_LENGTH_METERS = 0.90f;
    private static final float ROCK_ROUGHNESS_LENGTH_METERS = 0.08f;
    private static final float SNOW_ROUGHNESS_LENGTH_METERS = 0.003f;

    private final SeedTerrainProvider fallback = new HashedSeedTerrainProvider();

    @Override
    public TerrainSample sample(ServerWorld world, int blockX, int blockZ) {
        try {
            ServerChunkManager chunkManager = world.getChunkManager();
            ChunkGenerator generator = chunkManager.getChunkGenerator();
            NoiseConfig noiseConfig = chunkManager.getNoiseConfig();
            if (generator == null || noiseConfig == null) {
                return fallback.sample(world, blockX, blockZ);
            }

            int terrainHeightBlocks = generator.getHeightOnGround(
                blockX,
                blockZ,
                Heightmap.Type.WORLD_SURFACE_WG,
                world,
                noiseConfig
            );
            int biomeY = MathHelper.clamp(
                terrainHeightBlocks - 1,
                world.getBottomY(),
                world.getBottomY() + world.getHeight() - 1
            );
            RegistryEntry<Biome> biomeEntry = world.getGeneratorStoredBiome(
                Math.floorDiv(blockX, 4),
                Math.floorDiv(biomeY, 4),
                Math.floorDiv(blockZ, 4)
            );
            Biome biome = biomeEntry.value();
            float biomeTemperature = biome.getTemperature();
            int seaLevel = world.getSeaLevel();

            byte surfaceClass;
            float roughnessLengthMeters;
            if (biomeEntry.isIn(BiomeTags.IS_OCEAN)
                || biomeEntry.isIn(BiomeTags.IS_RIVER)
                || terrainHeightBlocks < seaLevel - 1) {
                surfaceClass = HashedSeedTerrainProvider.SURFACE_CLASS_WATER;
                roughnessLengthMeters = WATER_ROUGHNESS_LENGTH_METERS;
            } else if (biomeTemperature < 0.20f && terrainHeightBlocks > seaLevel + 4) {
                surfaceClass = HashedSeedTerrainProvider.SURFACE_CLASS_SNOW;
                roughnessLengthMeters = SNOW_ROUGHNESS_LENGTH_METERS;
            } else if (biomeEntry.isIn(BiomeTags.IS_FOREST)
                || biomeEntry.isIn(BiomeTags.IS_JUNGLE)
                || biomeEntry.isIn(BiomeTags.IS_TAIGA)) {
                surfaceClass = HashedSeedTerrainProvider.SURFACE_CLASS_FOREST;
                roughnessLengthMeters = FOREST_ROUGHNESS_LENGTH_METERS;
            } else if (biomeEntry.isIn(BiomeTags.IS_BADLANDS)
                || biomeEntry.isIn(BiomeTags.IS_MOUNTAIN)
                || terrainHeightBlocks > seaLevel + 72) {
                surfaceClass = HashedSeedTerrainProvider.SURFACE_CLASS_ROCK;
                roughnessLengthMeters = ROCK_ROUGHNESS_LENGTH_METERS;
            } else {
                surfaceClass = HashedSeedTerrainProvider.SURFACE_CLASS_PLAINS;
                roughnessLengthMeters = PLAINS_ROUGHNESS_LENGTH_METERS;
            }

            return new TerrainSample(terrainHeightBlocks, biomeTemperature, roughnessLengthMeters, surfaceClass);
        } catch (Throwable ignored) {
            return fallback.sample(world, blockX, blockZ);
        }
    }
}
