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
import net.minecraft.util.WorldSavePath;
import net.minecraft.util.math.BlockPos;
import net.minecraft.world.World;

final class StaticStore implements AutoCloseable {
    private static final int MAGIC = 0x41345353;
    private static final int FORMAT_VERSION = 1;

    private final ExecutorService ioExecutor = Executors.newSingleThreadExecutor(runnable -> {
        Thread thread = new Thread(runnable, "aero-static-store");
        thread.setDaemon(true);
        return thread;
    });
    private final AtomicBoolean closed = new AtomicBoolean(false);

    boolean loadSection(
        MinecraftServer server,
        RegistryKey<World> worldKey,
        BlockPos sectionOrigin,
        WorldMirror.SectionSnapshot snapshot
    ) {
        Path path = sectionFile(server, worldKey, sectionOrigin);
        if (!Files.isRegularFile(path)) {
            return false;
        }
        try (DataInputStream in = new DataInputStream(new BufferedInputStream(Files.newInputStream(path)))) {
            if (in.readInt() != MAGIC) {
                return false;
            }
            if (in.readInt() != FORMAT_VERSION) {
                return false;
            }
            snapshot.setVersion(in.readLong());
            readBinaryMask(in, snapshot.obstacle());
            readBinaryMask(in, snapshot.air());
            in.readFully(snapshot.surfaceKind());
            readFloatArray(in, snapshot.emitterPowerWatts());
            in.readFully(snapshot.openFaceMask());
            in.readFully(snapshot.faceSkyExposure());
            in.readFully(snapshot.faceDirectExposure());
            return true;
        } catch (IOException ex) {
            return false;
        }
    }

    void storeSection(
        MinecraftServer server,
        RegistryKey<World> worldKey,
        BlockPos sectionOrigin,
        WorldMirror.SectionSnapshot snapshot
    ) {
        if (closed.get()) {
            return;
        }
        StoredSection stored = StoredSection.capture(snapshot, sectionOrigin);
        ioExecutor.execute(() -> writeSection(server, worldKey, stored));
    }

    void invalidateSection(MinecraftServer server, RegistryKey<World> worldKey, BlockPos sectionOrigin) {
        if (closed.get()) {
            return;
        }
        BlockPos aligned = alignSectionOrigin(sectionOrigin);
        ioExecutor.execute(() -> {
            try {
                Files.deleteIfExists(sectionFile(server, worldKey, aligned));
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

    private void writeSection(MinecraftServer server, RegistryKey<World> worldKey, StoredSection stored) {
        Path path = sectionFile(server, worldKey, stored.origin());
        try {
            Files.createDirectories(path.getParent());
            try (DataOutputStream out = new DataOutputStream(new BufferedOutputStream(Files.newOutputStream(path)))) {
                out.writeInt(MAGIC);
                out.writeInt(FORMAT_VERSION);
                out.writeLong(stored.version());
                writeBinaryMask(out, stored.obstacle());
                writeBinaryMask(out, stored.air());
                out.write(stored.surfaceKind());
                writeFloatArray(out, stored.emitterPowerWatts());
                out.write(stored.openFaceMask());
                out.write(stored.faceSkyExposure());
                out.write(stored.faceDirectExposure());
            }
        } catch (IOException ignored) {
        }
    }

    private Path sectionFile(MinecraftServer server, RegistryKey<World> worldKey, BlockPos sectionOrigin) {
        Path root = server.getSavePath(WorldSavePath.ROOT)
            .resolve("aerodynamics4mc")
            .resolve("static_store_v1");
        String namespace = worldKey.getValue().getNamespace();
        String path = worldKey.getValue().getPath().replace('/', '_');
        return root
            .resolve(namespace)
            .resolve(path)
            .resolve(sectionOrigin.getX() + "_" + sectionOrigin.getY() + "_" + sectionOrigin.getZ() + ".bin");
    }

    private static void readBinaryMask(DataInputStream in, float[] output) throws IOException {
        for (int i = 0; i < output.length; i++) {
            output[i] = in.readUnsignedByte() == 0 ? 0.0f : 1.0f;
        }
    }

    private static void writeBinaryMask(DataOutputStream out, float[] source) throws IOException {
        for (float value : source) {
            out.writeByte(value >= 0.5f ? 1 : 0);
        }
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

    private static BlockPos alignSectionOrigin(BlockPos pos) {
        int x = Math.floorDiv(pos.getX(), WorldMirror.SECTION_SIZE) * WorldMirror.SECTION_SIZE;
        int y = Math.floorDiv(pos.getY(), WorldMirror.SECTION_SIZE) * WorldMirror.SECTION_SIZE;
        int z = Math.floorDiv(pos.getZ(), WorldMirror.SECTION_SIZE) * WorldMirror.SECTION_SIZE;
        return new BlockPos(x, y, z);
    }

    private record StoredSection(
        BlockPos origin,
        long version,
        float[] obstacle,
        float[] air,
        byte[] surfaceKind,
        float[] emitterPowerWatts,
        byte[] openFaceMask,
        byte[] faceSkyExposure,
        byte[] faceDirectExposure
    ) {
        private static StoredSection capture(WorldMirror.SectionSnapshot snapshot, BlockPos origin) {
            return new StoredSection(
                alignSectionOrigin(origin),
                snapshot.version(),
                snapshot.obstacle().clone(),
                snapshot.air().clone(),
                snapshot.surfaceKind().clone(),
                snapshot.emitterPowerWatts().clone(),
                snapshot.openFaceMask().clone(),
                snapshot.faceSkyExposure().clone(),
                snapshot.faceDirectExposure().clone()
            );
        }
    }
}
