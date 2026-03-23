package com.aerodynamics4mc.runtime;

import com.github.luben.zstd.Zstd;

import net.minecraft.registry.RegistryKey;
import net.minecraft.server.MinecraftServer;
import net.minecraft.server.world.ServerWorld;
import net.minecraft.util.Identifier;
import net.minecraft.util.WorldSavePath;
import net.minecraft.util.math.BlockPos;
import net.minecraft.util.math.MathHelper;
import net.minecraft.world.World;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

final class ActiveWindowWorldCache {
    private static final String LOG_PREFIX = "[aerodynamics4mc/cache] ";
    private static final int SECTION_SIZE = 16;
    private static final int SECTION_CELLS = SECTION_SIZE * SECTION_SIZE * SECTION_SIZE;
    private static final int CHANNELS = 4; // vx vy vz p
    private static final int MASK_WORDS = SECTION_CELLS / Long.SIZE;
    private static final int VALUES_BYTES = SECTION_CELLS * CHANNELS;

    private static final int FORMAT_VERSION = 1;
    private static final int MAGIC = 0x41575743; // "AWWC"
    private static final String FILE_SUFFIX = ".awwc.zst";

    private static final float SCALE_VELOCITY = 32.0f;
    private static final float SCALE_PRESSURE = 1.0f;
    private static final float INV_127 = 1.0f / 127.0f;

    private final Map<RegistryKey<World>, DimensionCache> dimensions = new HashMap<>();
    private final ExecutorService saveExecutor = Executors.newSingleThreadExecutor(r -> {
        Thread thread = new Thread(r, "aero-awwc-save");
        thread.setDaemon(true);
        return thread;
    });
    private boolean saveRequested;
    private boolean saveWorkerRunning;

    synchronized boolean loadWindow(
        ServerWorld world,
        BlockPos origin,
        int gridSize,
        float[] obstacleField,
        float[] airField,
        float[] baseField,
        int channelsPerCell,
        int chStateVx,
        int chStateVy,
        int chStateVz,
        int chStateP
    ) {
        DimensionCache cache = ensureLoaded(world);
        if (cache == null) {
            return false;
        }
        if (cache.loadFailed) {
            return false;
        }

        int expectedAirCells = 0;
        int loadedAirCells = 0;

        for (int x = 0; x < gridSize; x++) {
            int wx = origin.getX() + x;
            for (int y = 0; y < gridSize; y++) {
                int wy = origin.getY() + y;
                for (int z = 0; z < gridSize; z++) {
                    int wz = origin.getZ() + z;
                    int cell = (x * gridSize + y) * gridSize + z;
                    int idx = cell * channelsPerCell;
                    boolean isSolid = obstacleField != null && cell < obstacleField.length && obstacleField[cell] > 0.5f;
                    boolean isAir = airField != null && cell < airField.length && airField[cell] > 0.5f;
                    if (isSolid || !isAir) {
                        baseField[idx + chStateVx] = 0.0f;
                        baseField[idx + chStateVy] = 0.0f;
                        baseField[idx + chStateVz] = 0.0f;
                        baseField[idx + chStateP] = 0.0f;
                        continue;
                    }
                    expectedAirCells++;

                    CellAddress address = cellAddress(wx, wy, wz);
                    SectionData section = cache.sections.get(address.section());
                    if (section == null || !section.isValid(address.localIndex())) {
                        continue;
                    }

                    int valueBase = address.localIndex() * CHANNELS;
                    baseField[idx + chStateVx] = dequantize(section.values[valueBase], SCALE_VELOCITY);
                    baseField[idx + chStateVy] = dequantize(section.values[valueBase + 1], SCALE_VELOCITY);
                    baseField[idx + chStateVz] = dequantize(section.values[valueBase + 2], SCALE_VELOCITY);
                    baseField[idx + chStateP] = dequantize(section.values[valueBase + 3], SCALE_PRESSURE);
                    loadedAirCells++;
                }
            }
        }
        return expectedAirCells > 0 && loadedAirCells == expectedAirCells;
    }

    synchronized boolean loadSection(
        ServerWorld world,
        BlockPos sectionOrigin,
        float[] obstacleField,
        float[] airField,
        float[] stateField,
        int channelsPerCell,
        int chStateVx,
        int chStateVy,
        int chStateVz,
        int chStateP
    ) {
        if (stateField == null || stateField.length < SECTION_CELLS * channelsPerCell) {
            return false;
        }
        DimensionCache cache = ensureLoaded(world);
        if (cache == null || cache.loadFailed) {
            return false;
        }

        SectionKey key = alignedSectionKey(sectionOrigin);
        SectionData section = cache.sections.get(key);
        int loadedAirCells = 0;
        int expectedAirCells = 0;
        for (int localIndex = 0; localIndex < SECTION_CELLS; localIndex++) {
            int idx = localIndex * channelsPerCell;
            boolean isSolid = obstacleField != null && localIndex < obstacleField.length && obstacleField[localIndex] > 0.5f;
            boolean isAir = airField != null && localIndex < airField.length && airField[localIndex] > 0.5f;
            if (isSolid || !isAir) {
                stateField[idx + chStateVx] = 0.0f;
                stateField[idx + chStateVy] = 0.0f;
                stateField[idx + chStateVz] = 0.0f;
                stateField[idx + chStateP] = 0.0f;
                continue;
            }
            expectedAirCells++;
            if (section == null || !section.isValid(localIndex)) {
                stateField[idx + chStateVx] = 0.0f;
                stateField[idx + chStateVy] = 0.0f;
                stateField[idx + chStateVz] = 0.0f;
                stateField[idx + chStateP] = 0.0f;
                continue;
            }
            int valueBase = localIndex * CHANNELS;
            stateField[idx + chStateVx] = dequantize(section.values[valueBase], SCALE_VELOCITY);
            stateField[idx + chStateVy] = dequantize(section.values[valueBase + 1], SCALE_VELOCITY);
            stateField[idx + chStateVz] = dequantize(section.values[valueBase + 2], SCALE_VELOCITY);
            stateField[idx + chStateP] = dequantize(section.values[valueBase + 3], SCALE_PRESSURE);
            loadedAirCells++;
        }
        return expectedAirCells > 0 && loadedAirCells == expectedAirCells;
    }

    synchronized void storeWindow(
        ServerWorld world,
        BlockPos origin,
        int gridSize,
        float[] obstacleField,
        float[] airField,
        float[] baseField,
        int channelsPerCell,
        int chStateVx,
        int chStateVy,
        int chStateVz,
        int chStateP
    ) {
        if (baseField == null) {
            return;
        }
        DimensionCache cache = ensureLoaded(world);
        if (cache == null) {
            return;
        }

        for (int x = 0; x < gridSize; x++) {
            int wx = origin.getX() + x;
            for (int y = 0; y < gridSize; y++) {
                int wy = origin.getY() + y;
                for (int z = 0; z < gridSize; z++) {
                    int wz = origin.getZ() + z;
                    int cell = (x * gridSize + y) * gridSize + z;
                    CellAddress address = cellAddress(wx, wy, wz);
                    SectionData section = cache.sections.computeIfAbsent(address.section(), ignored -> new SectionData());

                    boolean isSolid = obstacleField != null && cell < obstacleField.length && obstacleField[cell] > 0.5f;
                    boolean isAir = airField != null && cell < airField.length && airField[cell] > 0.5f;
                    if (isSolid || !isAir) {
                        section.clear(address.localIndex());
                        continue;
                    }

                    int idx = cell * channelsPerCell;
                    int valueBase = address.localIndex() * CHANNELS;
                    section.values[valueBase] = quantize(baseField[idx + chStateVx], SCALE_VELOCITY);
                    section.values[valueBase + 1] = quantize(baseField[idx + chStateVy], SCALE_VELOCITY);
                    section.values[valueBase + 2] = quantize(baseField[idx + chStateVz], SCALE_VELOCITY);
                    section.values[valueBase + 3] = quantize(baseField[idx + chStateP], SCALE_PRESSURE);
                    section.markValid(address.localIndex());
                }
            }
        }

        cache.removeEmptySections();
        cache.dirty = true;
        cache.revision++;
    }

    synchronized void storeSection(
        ServerWorld world,
        BlockPos sectionOrigin,
        float[] obstacleField,
        float[] airField,
        float[] stateField,
        int channelsPerCell,
        int chStateVx,
        int chStateVy,
        int chStateVz,
        int chStateP
    ) {
        if (stateField == null || stateField.length < SECTION_CELLS * channelsPerCell) {
            return;
        }
        DimensionCache cache = ensureLoaded(world);
        if (cache == null) {
            return;
        }

        SectionKey key = alignedSectionKey(sectionOrigin);
        SectionData section = cache.sections.computeIfAbsent(key, ignored -> new SectionData());
        for (int localIndex = 0; localIndex < SECTION_CELLS; localIndex++) {
            boolean isSolid = obstacleField != null && localIndex < obstacleField.length && obstacleField[localIndex] > 0.5f;
            boolean isAir = airField != null && localIndex < airField.length && airField[localIndex] > 0.5f;
            if (isSolid || !isAir) {
                section.clear(localIndex);
                continue;
            }

            int idx = localIndex * channelsPerCell;
            int valueBase = localIndex * CHANNELS;
            section.values[valueBase] = quantize(stateField[idx + chStateVx], SCALE_VELOCITY);
            section.values[valueBase + 1] = quantize(stateField[idx + chStateVy], SCALE_VELOCITY);
            section.values[valueBase + 2] = quantize(stateField[idx + chStateVz], SCALE_VELOCITY);
            section.values[valueBase + 3] = quantize(stateField[idx + chStateP], SCALE_PRESSURE);
            section.markValid(localIndex);
        }

        if (section.isEmpty()) {
            cache.sections.remove(key);
        }
        cache.dirty = true;
        cache.revision++;
    }

    synchronized void saveAll() {
        saveRequested = true;
        if (saveWorkerRunning) {
            return;
        }
        saveWorkerRunning = true;
        saveExecutor.execute(this::runSaveLoop);
    }

    synchronized void flushSaves() {
        while (saveWorkerRunning || saveRequested) {
            try {
                wait();
            } catch (InterruptedException ex) {
                Thread.currentThread().interrupt();
                log("Interrupted while waiting for pending cache saves");
                return;
            }
        }
    }

    private DimensionCache ensureLoaded(ServerWorld world) {
        RegistryKey<World> key = world.getRegistryKey();
        DimensionCache cache = dimensions.get(key);
        if (cache != null) {
            return cache;
        }

        Path file = cacheFilePath(world);
        DimensionCache loaded = new DimensionCache(file);
        if (Files.exists(file)) {
            try {
                readFromDisk(file, loaded);
                loaded.loadFailed = false;
            } catch (IOException ex) {
                loaded.loadFailed = true;
                log("Failed to load cache from " + file + ": " + ex.getMessage());
            }
        }
        dimensions.put(key, loaded);
        return loaded;
    }

    private void runSaveLoop() {
        while (true) {
            List<SaveSnapshot> snapshots;
            synchronized (this) {
                if (!saveRequested) {
                    saveWorkerRunning = false;
                    notifyAll();
                    return;
                }
                saveRequested = false;
                snapshots = snapshotDirtyDimensions();
            }

            for (SaveSnapshot snapshot : snapshots) {
                if (saveSnapshot(snapshot)) {
                    markSnapshotSaved(snapshot);
                }
            }
        }
    }

    private List<SaveSnapshot> snapshotDirtyDimensions() {
        List<SaveSnapshot> snapshots = new ArrayList<>();
        for (Map.Entry<RegistryKey<World>, DimensionCache> entry : dimensions.entrySet()) {
            DimensionCache cache = entry.getValue();
            if (!cache.dirty) {
                continue;
            }
            snapshots.add(new SaveSnapshot(entry.getKey(), cache.file, copySections(cache.sections), cache.revision));
        }
        return snapshots;
    }

    private Map<SectionKey, SectionData> copySections(Map<SectionKey, SectionData> source) {
        Map<SectionKey, SectionData> copy = new HashMap<>(source.size());
        for (Map.Entry<SectionKey, SectionData> entry : source.entrySet()) {
            copy.put(entry.getKey(), entry.getValue().copy());
        }
        return copy;
    }

    private boolean saveSnapshot(SaveSnapshot snapshot) {
        try {
            byte[] raw = serialize(snapshot.sections());
            byte[] compressed = Zstd.compress(raw, 3);

            Files.createDirectories(snapshot.file().getParent());
            Path tmp = snapshot.file().resolveSibling(snapshot.file().getFileName() + ".tmp");
            try (DataOutputStream out = new DataOutputStream(Files.newOutputStream(tmp))) {
                out.writeInt(raw.length);
                out.writeInt(compressed.length);
                out.write(compressed);
            }
            Files.move(tmp, snapshot.file(), StandardCopyOption.REPLACE_EXISTING, StandardCopyOption.ATOMIC_MOVE);
            return true;
        } catch (IOException ex) {
            log("Failed to save cache " + snapshot.file() + ": " + ex.getMessage());
            return false;
        }
    }

    private synchronized void markSnapshotSaved(SaveSnapshot snapshot) {
        DimensionCache cache = dimensions.get(snapshot.worldKey());
        if (cache == null) {
            return;
        }
        if (cache.dirty && cache.revision == snapshot.revision()) {
            cache.dirty = false;
        }
    }

    private void readFromDisk(Path file, DimensionCache target) throws IOException {
        try (DataInputStream in = new DataInputStream(Files.newInputStream(file))) {
            int rawLength = in.readInt();
            int compressedLength = in.readInt();
            if (rawLength <= 0 || compressedLength <= 0) {
                return;
            }
            byte[] compressed = in.readNBytes(compressedLength);
            if (compressed.length != compressedLength) {
                return;
            }
            byte[] raw = Zstd.decompress(compressed, rawLength);
            deserialize(raw, target);
            target.dirty = false;
        }
    }

    private byte[] serialize(Map<SectionKey, SectionData> sections) throws IOException {
        ByteArrayOutputStream bos = new ByteArrayOutputStream();
        try (DataOutputStream out = new DataOutputStream(bos)) {
            out.writeInt(MAGIC);
            out.writeInt(FORMAT_VERSION);
            out.writeInt(CHANNELS);
            out.writeInt(SECTION_SIZE);
            out.writeInt(sections.size());
            for (Map.Entry<SectionKey, SectionData> entry : sections.entrySet()) {
                SectionKey key = entry.getKey();
                SectionData section = entry.getValue();
                out.writeInt(key.sx());
                out.writeInt(key.sy());
                out.writeInt(key.sz());
                for (long word : section.validMask) {
                    out.writeLong(word);
                }
                out.write(section.values);
            }
        }
        return bos.toByteArray();
    }

    private void deserialize(byte[] raw, DimensionCache cache) throws IOException {
        cache.sections.clear();
        try (DataInputStream in = new DataInputStream(new ByteArrayInputStream(raw))) {
            int magic = in.readInt();
            if (magic != MAGIC) {
                return;
            }
            int version = in.readInt();
            if (version != FORMAT_VERSION) {
                return;
            }
            int channels = in.readInt();
            int sectionSize = in.readInt();
            if (channels != CHANNELS || sectionSize != SECTION_SIZE) {
                return;
            }
            int sectionCount = in.readInt();
            for (int i = 0; i < sectionCount; i++) {
                int sx = in.readInt();
                int sy = in.readInt();
                int sz = in.readInt();
                SectionData section = new SectionData();
                for (int w = 0; w < MASK_WORDS; w++) {
                    section.validMask[w] = in.readLong();
                }
                int read = in.readNBytes(section.values, 0, section.values.length);
                if (read != section.values.length) {
                    cache.sections.clear();
                    return;
                }
                cache.sections.put(new SectionKey(sx, sy, sz), section);
            }
        }
    }

    private Path cacheFilePath(ServerWorld world) {
        Identifier id = world.getRegistryKey().getValue();
        String name = (id.getNamespace() + "_" + id.getPath()).replace('/', '_').replace(':', '_');
        return world.getServer()
            .getSavePath(WorldSavePath.ROOT)
            .resolve("data")
            .resolve("aero_awwc")
            .resolve(name + FILE_SUFFIX);
    }

    private CellAddress cellAddress(int x, int y, int z) {
        int sx = Math.floorDiv(x, SECTION_SIZE);
        int sy = Math.floorDiv(y, SECTION_SIZE);
        int sz = Math.floorDiv(z, SECTION_SIZE);
        int lx = Math.floorMod(x, SECTION_SIZE);
        int ly = Math.floorMod(y, SECTION_SIZE);
        int lz = Math.floorMod(z, SECTION_SIZE);
        int localIndex = (lx * SECTION_SIZE + ly) * SECTION_SIZE + lz;
        return new CellAddress(new SectionKey(sx, sy, sz), localIndex);
    }

    private SectionKey alignedSectionKey(BlockPos sectionOrigin) {
        return new SectionKey(
            Math.floorDiv(sectionOrigin.getX(), SECTION_SIZE),
            Math.floorDiv(sectionOrigin.getY(), SECTION_SIZE),
            Math.floorDiv(sectionOrigin.getZ(), SECTION_SIZE)
        );
    }

    private static byte quantize(float value, float scale) {
        float normalized = MathHelper.clamp(value / scale, -1.0f, 1.0f);
        int q = Math.round(normalized * 127.0f);
        return (byte) MathHelper.clamp(q, -127, 127);
    }

    private static float dequantize(byte q, float scale) {
        return q * INV_127 * scale;
    }

    private void log(String message) {
        System.out.println(LOG_PREFIX + message);
    }

    private static final class DimensionCache {
        private final Path file;
        private final Map<SectionKey, SectionData> sections = new HashMap<>();
        private boolean dirty;
        private long revision;
        private boolean loadFailed;

        private DimensionCache(Path file) {
            this.file = file;
        }

        private void removeEmptySections() {
            sections.entrySet().removeIf(entry -> entry.getValue().isEmpty());
        }
    }

    private static final class SectionData {
        private final long[] validMask = new long[MASK_WORDS];
        private final byte[] values = new byte[VALUES_BYTES];

        private boolean isValid(int localIndex) {
            int word = localIndex >>> 6;
            long bit = 1L << (localIndex & 63);
            return (validMask[word] & bit) != 0L;
        }

        private void markValid(int localIndex) {
            int word = localIndex >>> 6;
            long bit = 1L << (localIndex & 63);
            validMask[word] |= bit;
        }

        private void clear(int localIndex) {
            int word = localIndex >>> 6;
            long bit = 1L << (localIndex & 63);
            validMask[word] &= ~bit;
            int base = localIndex * CHANNELS;
            values[base] = 0;
            values[base + 1] = 0;
            values[base + 2] = 0;
            values[base + 3] = 0;
        }

        private boolean isEmpty() {
            for (long word : validMask) {
                if (word != 0L) {
                    return false;
                }
            }
            return true;
        }

        private SectionData copy() {
            SectionData copied = new SectionData();
            System.arraycopy(validMask, 0, copied.validMask, 0, validMask.length);
            System.arraycopy(values, 0, copied.values, 0, values.length);
            return copied;
        }
    }

    private record SaveSnapshot(
        RegistryKey<World> worldKey,
        Path file,
        Map<SectionKey, SectionData> sections,
        long revision
    ) {
    }

    private record SectionKey(int sx, int sy, int sz) {
    }

    private record CellAddress(SectionKey section, int localIndex) {
    }
}
