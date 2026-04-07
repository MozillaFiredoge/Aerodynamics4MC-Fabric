package com.aerodynamics4mc.runtime;

import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

import net.minecraft.registry.RegistryKey;
import net.minecraft.server.MinecraftServer;
import net.minecraft.server.world.ServerWorld;
import net.minecraft.util.WorldSavePath;
import net.minecraft.util.math.BlockPos;
import net.minecraft.world.World;

final class DynamicStore implements AutoCloseable {
    private static final int MAGIC = 0x41344459;
    private static final int FORMAT_VERSION = 1;

    private final ExecutorService ioExecutor = Executors.newSingleThreadExecutor(runnable -> {
        Thread thread = new Thread(runnable, "aero-dynamic-store");
        thread.setDaemon(true);
        return thread;
    });
    private final AtomicBoolean closed = new AtomicBoolean(false);

    boolean loadRegion(
        ServerWorld world,
        RegistryKey<World> worldKey,
        BlockPos regionOrigin,
        int sizeX,
        int sizeY,
        int sizeZ,
        float[] flowState,
        float[] airTemperatureState,
        float[] surfaceTemperatureState
    ) {
        Path path = regionFile(world.getServer(), worldKey, regionOrigin);
        if (!Files.isRegularFile(path)) {
            return false;
        }
        try (DataInputStream in = new DataInputStream(new BufferedInputStream(Files.newInputStream(path)))) {
            if (in.readInt() != MAGIC || in.readInt() != FORMAT_VERSION) {
                return false;
            }
            if (in.readInt() != sizeX || in.readInt() != sizeY || in.readInt() != sizeZ) {
                return false;
            }
            readFloatArray(in, flowState);
            readFloatArray(in, airTemperatureState);
            readFloatArray(in, surfaceTemperatureState);
            return true;
        } catch (IOException ex) {
            return false;
        }
    }

    void storeRegion(
        ServerWorld world,
        RegistryKey<World> worldKey,
        BlockPos regionOrigin,
        int sizeX,
        int sizeY,
        int sizeZ,
        float[] flowState,
        float[] airTemperatureState,
        float[] surfaceTemperatureState
    ) {
        if (closed.get()) {
            return;
        }
        StoredRegion stored = StoredRegion.capture(regionOrigin, sizeX, sizeY, sizeZ, flowState, airTemperatureState, surfaceTemperatureState);
        ioExecutor.execute(() -> writeRegion(world.getServer(), worldKey, stored));
    }

    void storeCapturedRegion(
        ServerWorld world,
        RegistryKey<World> worldKey,
        BlockPos regionOrigin,
        int sizeX,
        int sizeY,
        int sizeZ,
        float[] flowState,
        float[] airTemperatureState,
        float[] surfaceTemperatureState
    ) {
        if (closed.get()) {
            return;
        }
        StoredRegion stored = StoredRegion.captureOwned(regionOrigin, sizeX, sizeY, sizeZ, flowState, airTemperatureState, surfaceTemperatureState);
        ioExecutor.execute(() -> writeRegion(world.getServer(), worldKey, stored));
    }

    void invalidateRegion(ServerWorld world, RegistryKey<World> worldKey, BlockPos regionOrigin) {
        if (closed.get()) {
            return;
        }
        BlockPos aligned = regionOrigin.toImmutable();
        ioExecutor.execute(() -> {
            try {
                Files.deleteIfExists(regionFile(world.getServer(), worldKey, aligned));
            } catch (IOException ignored) {
            }
        });
    }

    @Override
    public void close() {
        if (!closed.compareAndSet(false, true)) {
            return;
        }
        ioExecutor.shutdown();
        try {
            ioExecutor.awaitTermination(5, TimeUnit.SECONDS);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }

    private void writeRegion(MinecraftServer server, RegistryKey<World> worldKey, StoredRegion stored) {
        Path path = regionFile(server, worldKey, stored.origin());
        try {
            Files.createDirectories(path.getParent());
            try (DataOutputStream out = new DataOutputStream(new BufferedOutputStream(Files.newOutputStream(path)))) {
                out.writeInt(MAGIC);
                out.writeInt(FORMAT_VERSION);
                out.writeInt(stored.sizeX());
                out.writeInt(stored.sizeY());
                out.writeInt(stored.sizeZ());
                writeFloatArray(out, stored.flowState());
                writeFloatArray(out, stored.airTemperatureState());
                writeFloatArray(out, stored.surfaceTemperatureState());
            }
        } catch (IOException ignored) {
        }
    }

    private Path regionFile(MinecraftServer server, RegistryKey<World> worldKey, BlockPos regionOrigin) {
        Path root = server.getSavePath(WorldSavePath.ROOT)
            .resolve("aerodynamics4mc")
            .resolve("dynamic_store_v1");
        String namespace = worldKey.getValue().getNamespace();
        String path = worldKey.getValue().getPath().replace('/', '_');
        return root
            .resolve(namespace)
            .resolve(path)
            .resolve(regionOrigin.getX() + "_" + regionOrigin.getY() + "_" + regionOrigin.getZ() + ".bin");
    }

    private static void readFloatArray(DataInputStream in, float[] output) throws IOException {
        for (int i = 0; i < output.length; i++) {
            output[i] = in.readFloat();
        }
    }

    private static void writeFloatArray(DataOutputStream out, float[] source) throws IOException {
        for (float value : source) {
            out.writeFloat(value);
        }
    }

    private record StoredRegion(
        BlockPos origin,
        int sizeX,
        int sizeY,
        int sizeZ,
        float[] flowState,
        float[] airTemperatureState,
        float[] surfaceTemperatureState
    ) {
        private static StoredRegion capture(
            BlockPos origin,
            int sizeX,
            int sizeY,
            int sizeZ,
            float[] flowState,
            float[] airTemperatureState,
            float[] surfaceTemperatureState
        ) {
            return new StoredRegion(
                origin.toImmutable(),
                sizeX,
                sizeY,
                sizeZ,
                flowState.clone(),
                airTemperatureState.clone(),
                surfaceTemperatureState.clone()
            );
        }

        private static StoredRegion captureOwned(
            BlockPos origin,
            int sizeX,
            int sizeY,
            int sizeZ,
            float[] flowState,
            float[] airTemperatureState,
            float[] surfaceTemperatureState
        ) {
            return new StoredRegion(
                origin.toImmutable(),
                sizeX,
                sizeY,
                sizeZ,
                flowState,
                airTemperatureState,
                surfaceTemperatureState
            );
        }
    }
}
