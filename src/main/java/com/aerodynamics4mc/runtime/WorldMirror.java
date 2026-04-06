package com.aerodynamics4mc.runtime;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

import com.aerodynamics4mc.FanBlock;
import com.aerodynamics4mc.FanBlockEntity;
import com.aerodynamics4mc.ModBlocks;

import net.minecraft.block.BlockState;
import net.minecraft.block.entity.BlockEntity;
import net.minecraft.registry.RegistryKey;
import net.minecraft.server.MinecraftServer;
import net.minecraft.server.world.ServerWorld;
import net.minecraft.util.math.BlockPos;
import net.minecraft.util.math.ChunkPos;
import net.minecraft.util.math.Direction;
import net.minecraft.world.World;
import net.minecraft.world.chunk.WorldChunk;

final class WorldMirror {
    static final int SECTION_SIZE = 16;
    static final int SECTION_CELLS = SECTION_SIZE * SECTION_SIZE * SECTION_SIZE;
    static final int FACE_COUNT = 6;
    private static final int DUCT_SCAN_MAX = 20;
    private static final int DUCT_RING_RADIUS = 1;
    private static final float DUCT_RING_FILL_THRESHOLD = 0.80f;
    private static final int DUCT_MAX_CONSECUTIVE_GAPS = 1;

    @FunctionalInterface
    interface SectionBuilder {
        void build(ServerWorld world, BlockPos sectionOrigin, SectionSnapshot snapshot);
    }

    static final class SectionSnapshot {
        private final float[] obstacle = new float[SECTION_CELLS];
        private final float[] air = new float[SECTION_CELLS];
        private final byte[] surfaceKind = new byte[SECTION_CELLS];
        private final float[] emitterPowerWatts = new float[SECTION_CELLS];
        private final byte[] openFaceMask = new byte[SECTION_CELLS];
        private final byte[] faceSkyExposure = new byte[SECTION_CELLS * FACE_COUNT];
        private final byte[] faceDirectExposure = new byte[SECTION_CELLS * FACE_COUNT];
        private long version;

        float[] obstacle() {
            return obstacle;
        }

        float[] air() {
            return air;
        }

        byte[] surfaceKind() {
            return surfaceKind;
        }

        float[] emitterPowerWatts() {
            return emitterPowerWatts;
        }

        byte[] openFaceMask() {
            return openFaceMask;
        }

        byte[] faceSkyExposure() {
            return faceSkyExposure;
        }

        byte[] faceDirectExposure() {
            return faceDirectExposure;
        }

        long version() {
            return version;
        }

        void setVersion(long version) {
            this.version = version;
        }

        void bumpVersion() {
            version++;
        }
    }

    static final class FanRecord {
        private final BlockPos pos;
        private final Direction facing;
        private final int ductLength;

        FanRecord(BlockPos pos, Direction facing, int ductLength) {
            this.pos = pos.toImmutable();
            this.facing = facing;
            this.ductLength = ductLength;
        }

        BlockPos pos() {
            return pos;
        }

        Direction facing() {
            return facing;
        }

        int ductLength() {
            return ductLength;
        }
    }

    private static final class SectionEntry {
        private final SectionSnapshot snapshot = new SectionSnapshot();
        private boolean dirty = true;
        private boolean storeLoadEligible = true;
        private boolean storeLoadInFlight;
        private boolean liveBuildQueued;
        private long generation;
    }

    private record BuildRequest(RegistryKey<World> worldKey, BlockPos sectionOrigin) {
    }

    private static final class DimensionMirror {
        private final Map<Long, SectionEntry> sections = new HashMap<>();
        private final Map<Long, FanRecord> fans = new HashMap<>();
    }

    private final Map<RegistryKey<World>, DimensionMirror> dimensions = new HashMap<>();
    private final StaticStore staticStore = new StaticStore();
    private final ExecutorService loadExecutor = Executors.newSingleThreadExecutor(runnable -> {
        Thread thread = new Thread(runnable, "aero-static-load");
        thread.setDaemon(true);
        return thread;
    });
    private final ArrayDeque<BuildRequest> pendingLiveBuilds = new ArrayDeque<>();
    private final Set<BuildRequest> queuedLiveBuilds = new HashSet<>();

    synchronized void close() {
        loadExecutor.shutdown();
        try {
            loadExecutor.awaitTermination(5, TimeUnit.SECONDS);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
        staticStore.close();
    }

    synchronized void onWorldUnload(ServerWorld world) {
        dimensions.remove(world.getRegistryKey());
    }

    synchronized void onChunkLoad(ServerWorld world, WorldChunk chunk) {
        DimensionMirror dimension = dimension(world.getRegistryKey());
        int sectionX = chunk.getPos().x * SECTION_SIZE;
        int sectionZ = chunk.getPos().z * SECTION_SIZE;
        for (int sectionY = world.getBottomSectionCoord(); sectionY < world.getTopSectionCoord(); sectionY++) {
            markSectionDirty(dimension, sectionX, sectionY * SECTION_SIZE, sectionZ, true, false, world, world.getRegistryKey());
        }
        refreshChunkFans(world, dimension, chunk);
    }

    synchronized void onChunkUnload(ServerWorld world, ChunkPos chunkPos) {
        DimensionMirror dimension = dimensions.get(world.getRegistryKey());
        if (dimension == null) {
            return;
        }
        int minX = chunkPos.x * SECTION_SIZE;
        int minZ = chunkPos.z * SECTION_SIZE;
        int maxX = minX + SECTION_SIZE;
        int maxZ = minZ + SECTION_SIZE;
        Iterator<Map.Entry<Long, SectionEntry>> sectionIt = dimension.sections.entrySet().iterator();
        while (sectionIt.hasNext()) {
            long packed = sectionIt.next().getKey();
            BlockPos origin = BlockPos.fromLong(packed);
            if (origin.getX() >= minX && origin.getX() < maxX && origin.getZ() >= minZ && origin.getZ() < maxZ) {
                sectionIt.remove();
            }
        }
        Iterator<Map.Entry<Long, FanRecord>> fanIt = dimension.fans.entrySet().iterator();
        while (fanIt.hasNext()) {
            FanRecord fan = fanIt.next().getValue();
            ChunkPos fanChunk = new ChunkPos(fan.pos());
            if (fanChunk.x == chunkPos.x && fanChunk.z == chunkPos.z) {
                fanIt.remove();
            }
        }
    }

    synchronized void onBlockEntityLoad(BlockEntity blockEntity, ServerWorld world) {
        if (blockEntity instanceof FanBlockEntity) {
            upsertFan(world, blockEntity.getPos(), blockEntity.getCachedState());
        }
    }

    synchronized void onBlockEntityUnload(BlockEntity blockEntity, ServerWorld world) {
        if (blockEntity instanceof FanBlockEntity) {
            removeFan(world, blockEntity.getPos());
        }
    }

    synchronized void onBlockChanged(ServerWorld world, BlockPos pos, BlockState oldState, BlockState newState) {
        DimensionMirror dimension = dimension(world.getRegistryKey());
        markDirtyForBlockChange(world, dimension, pos);
        if (isFanState(oldState) || isFanState(newState)) {
            if (isFanState(newState)) {
                upsertFan(world, pos, newState);
            } else {
                removeFan(world, pos);
            }
        } else if (isDuctState(oldState) || isDuctState(newState)) {
            refreshNearbyFanDuctLengths(world, dimension, pos);
        }
    }

    synchronized void requestSectionBuild(MinecraftServer server, RegistryKey<World> worldKey, BlockPos sectionOrigin) {
        BlockPos alignedOrigin = alignSectionOrigin(sectionOrigin);
        DimensionMirror dimension = dimension(worldKey);
        SectionEntry entry = dimension.sections.computeIfAbsent(alignedOrigin.asLong(), ignored -> new SectionEntry());
        requestSectionBuildLocked(server, worldKey, alignedOrigin, entry);
    }

    void drainLiveBuilds(MinecraftServer server, int budget, SectionBuilder builder) {
        for (int i = 0; i < budget; i++) {
            BuildRequest request;
            synchronized (this) {
                request = pendingLiveBuilds.pollFirst();
                if (request == null) {
                    return;
                }
                queuedLiveBuilds.remove(request);
                DimensionMirror dimension = dimensions.get(request.worldKey());
                if (dimension != null) {
                    SectionEntry entry = dimension.sections.get(request.sectionOrigin().asLong());
                    if (entry != null) {
                        entry.liveBuildQueued = false;
                    }
                }
            }
            ServerWorld world = server.getWorld(request.worldKey());
            if (world == null) {
                continue;
            }
            SectionSnapshot built = new SectionSnapshot();
            builder.build(world, request.sectionOrigin(), built);
            synchronized (this) {
                DimensionMirror dimension = dimensions.get(request.worldKey());
                if (dimension == null) {
                    continue;
                }
                SectionEntry entry = dimension.sections.get(request.sectionOrigin().asLong());
                if (entry == null || !entry.dirty) {
                    continue;
                }
                built.setVersion(entry.snapshot.version() + 1L);
                copySnapshot(built, entry.snapshot);
                entry.dirty = false;
                entry.storeLoadEligible = false;
                entry.storeLoadInFlight = false;
                staticStore.storeSection(server, request.worldKey(), request.sectionOrigin(), entry.snapshot);
            }
        }
    }

    synchronized SectionSnapshot peekSection(RegistryKey<World> worldKey, BlockPos sectionOrigin) {
        DimensionMirror dimension = dimensions.get(worldKey);
        if (dimension == null) {
            return null;
        }
        BlockPos alignedOrigin = alignSectionOrigin(sectionOrigin);
        SectionEntry entry = dimension.sections.get(alignedOrigin.asLong());
        if (entry == null || entry.dirty) {
            return null;
        }
        return entry.snapshot;
    }

    synchronized List<FanRecord> queryFans(RegistryKey<World> worldKey, BlockPos origin, int gridSize, int margin) {
        DimensionMirror dimension = dimensions.get(worldKey);
        if (dimension == null || dimension.fans.isEmpty()) {
            return List.of();
        }
        int minX = origin.getX() - margin;
        int minY = origin.getY() - margin;
        int minZ = origin.getZ() - margin;
        int maxX = origin.getX() + gridSize + margin;
        int maxY = origin.getY() + gridSize + margin;
        int maxZ = origin.getZ() + gridSize + margin;
        List<FanRecord> result = new ArrayList<>();
        for (FanRecord fan : dimension.fans.values()) {
            BlockPos pos = fan.pos();
            if (pos.getX() >= minX && pos.getX() < maxX
                && pos.getY() >= minY && pos.getY() < maxY
                && pos.getZ() >= minZ && pos.getZ() < maxZ) {
                result.add(fan);
            }
        }
        return result;
    }

    private void refreshChunkFans(ServerWorld world, DimensionMirror dimension, WorldChunk chunk) {
        ChunkPos chunkPos = chunk.getPos();
        Iterator<Map.Entry<Long, FanRecord>> it = dimension.fans.entrySet().iterator();
        while (it.hasNext()) {
            FanRecord fan = it.next().getValue();
            ChunkPos fanChunk = new ChunkPos(fan.pos());
            if (fanChunk.x == chunkPos.x && fanChunk.z == chunkPos.z) {
                it.remove();
            }
        }
        for (BlockEntity blockEntity : chunk.getBlockEntities().values()) {
            if (blockEntity instanceof FanBlockEntity && blockEntity.getCachedState().isOf(ModBlocks.FAN_BLOCK)) {
                BlockState state = blockEntity.getCachedState();
                Direction facing = state.get(FanBlock.FACING);
                dimension.fans.put(
                    blockEntity.getPos().asLong(),
                    new FanRecord(blockEntity.getPos(), facing, computeDuctLength(world, blockEntity.getPos(), facing))
                );
            }
        }
    }

    private void upsertFan(ServerWorld world, BlockPos pos, BlockState state) {
        if (!isFanState(state)) {
            return;
        }
        Direction facing = state.get(FanBlock.FACING);
        dimension(world.getRegistryKey()).fans.put(
            pos.asLong(),
            new FanRecord(pos, facing, computeDuctLength(world, pos, facing))
        );
    }

    private void removeFan(ServerWorld world, BlockPos pos) {
        DimensionMirror dimension = dimensions.get(world.getRegistryKey());
        if (dimension != null) {
            dimension.fans.remove(pos.asLong());
        }
    }

    private boolean isFanState(BlockState state) {
        return state != null && state.isOf(ModBlocks.FAN_BLOCK);
    }

    private boolean isDuctState(BlockState state) {
        return state != null && state.isOf(ModBlocks.DUCT_BLOCK);
    }

    private void refreshNearbyFanDuctLengths(ServerWorld world, DimensionMirror dimension, BlockPos changedPos) {
        if (dimension.fans.isEmpty()) {
            return;
        }
        List<FanRecord> updatedFans = new ArrayList<>();
        for (FanRecord fan : dimension.fans.values()) {
            if (!isPotentiallyRelevantDuctChange(fan.pos(), changedPos)) {
                continue;
            }
            updatedFans.add(new FanRecord(
                fan.pos(),
                fan.facing(),
                computeDuctLength(world, fan.pos(), fan.facing())
            ));
        }
        for (FanRecord fan : updatedFans) {
            dimension.fans.put(fan.pos().asLong(), fan);
        }
    }

    private boolean isPotentiallyRelevantDuctChange(BlockPos fanPos, BlockPos changedPos) {
        return Math.abs(fanPos.getX() - changedPos.getX()) <= DUCT_SCAN_MAX + DUCT_RING_RADIUS + 1
            && Math.abs(fanPos.getY() - changedPos.getY()) <= DUCT_SCAN_MAX + DUCT_RING_RADIUS + 1
            && Math.abs(fanPos.getZ() - changedPos.getZ()) <= DUCT_SCAN_MAX + DUCT_RING_RADIUS + 1;
    }

    private int computeDuctLength(ServerWorld world, BlockPos fanPos, Direction facing) {
        int runLength = 0;
        int maxRunLength = 0;
        int consecutiveGaps = 0;
        BlockPos.Mutable cursor = new BlockPos.Mutable();

        for (int step = 1; step <= DUCT_SCAN_MAX; step++) {
            cursor.set(
                fanPos.getX() + facing.getOffsetX() * step,
                fanPos.getY() + facing.getOffsetY() * step,
                fanPos.getZ() + facing.getOffsetZ() * step
            );
            if (isDuctSegment(world, cursor, facing)) {
                runLength++;
                maxRunLength = Math.max(maxRunLength, runLength);
                consecutiveGaps = 0;
            } else {
                consecutiveGaps++;
                if (consecutiveGaps > DUCT_MAX_CONSECUTIVE_GAPS) {
                    break;
                }
            }
        }
        return maxRunLength;
    }

    private boolean isDuctSegment(ServerWorld world, BlockPos center, Direction facing) {
        if (isSolidObstacle(world, center)) {
            return false;
        }

        int ringCells = 0;
        int filledRingCells = 0;
        BlockPos.Mutable cursor = new BlockPos.Mutable();
        Direction.Axis axis = facing.getAxis();
        for (int a = -DUCT_RING_RADIUS; a <= DUCT_RING_RADIUS; a++) {
            for (int b = -DUCT_RING_RADIUS; b <= DUCT_RING_RADIUS; b++) {
                if (!isDuctRingCell(a, b)) {
                    continue;
                }
                ringCells++;
                switch (axis) {
                    case X -> cursor.set(center.getX(), center.getY() + a, center.getZ() + b);
                    case Y -> cursor.set(center.getX() + a, center.getY(), center.getZ() + b);
                    case Z -> cursor.set(center.getX() + a, center.getY() + b, center.getZ());
                }
                if (world.getBlockState(cursor).isOf(ModBlocks.DUCT_BLOCK)) {
                    filledRingCells++;
                }
            }
        }

        if (ringCells == 0) {
            return false;
        }
        return ((float) filledRingCells / ringCells) >= DUCT_RING_FILL_THRESHOLD;
    }

    private boolean isDuctRingCell(int a, int b) {
        return Math.max(Math.abs(a), Math.abs(b)) == DUCT_RING_RADIUS;
    }

    private boolean isSolidObstacle(ServerWorld world, BlockPos pos) {
        BlockState state = world.getBlockState(pos);
        if (state.isAir() || state.isOf(ModBlocks.DUCT_BLOCK)) {
            return false;
        }
        return !state.getCollisionShape(world, pos).isEmpty();
    }

    private void markDirtyForBlockChange(ServerWorld world, DimensionMirror dimension, BlockPos pos) {
        BlockPos aligned = alignSectionOrigin(pos);
        int baseX = aligned.getX();
        int baseY = aligned.getY();
        int baseZ = aligned.getZ();
        for (int dx = -SECTION_SIZE; dx <= SECTION_SIZE; dx += SECTION_SIZE) {
            for (int dy = -SECTION_SIZE; dy <= SECTION_SIZE; dy += SECTION_SIZE) {
                for (int dz = -SECTION_SIZE; dz <= SECTION_SIZE; dz += SECTION_SIZE) {
                    BlockPos origin = new BlockPos(baseX + dx, baseY + dy, baseZ + dz);
                    markSectionDirty(
                        dimension,
                        origin.getX(),
                        origin.getY(),
                        origin.getZ(),
                        false,
                        true,
                        world,
                        world.getRegistryKey()
                    );
                    requestSectionBuild(world.getServer(), world.getRegistryKey(), origin);
                }
            }
        }
        for (int sectionY = world.getBottomSectionCoord(); sectionY < world.getTopSectionCoord(); sectionY++) {
            BlockPos origin = new BlockPos(baseX, sectionY * SECTION_SIZE, baseZ);
            markSectionDirty(
                dimension,
                origin.getX(),
                origin.getY(),
                origin.getZ(),
                false,
                true,
                world,
                world.getRegistryKey()
            );
            requestSectionBuild(world.getServer(), world.getRegistryKey(), origin);
        }
    }

    private void markSectionDirty(
        DimensionMirror dimension,
        int originX,
        int originY,
        int originZ,
        boolean storeLoadEligible,
        boolean invalidateStoredSnapshot,
        ServerWorld world,
        RegistryKey<World> worldKey
    ) {
        long key = new BlockPos(originX, originY, originZ).asLong();
        SectionEntry entry = dimension.sections.computeIfAbsent(key, ignored -> new SectionEntry());
        entry.dirty = true;
        entry.storeLoadEligible = storeLoadEligible;
        entry.generation++;
        if (invalidateStoredSnapshot) {
            staticStore.invalidateSection(world.getServer(), worldKey, new BlockPos(originX, originY, originZ));
        }
    }

    private void requestSectionBuildLocked(
        MinecraftServer server,
        RegistryKey<World> worldKey,
        BlockPos alignedOrigin,
        SectionEntry entry
    ) {
        if (!entry.dirty) {
            return;
        }
        if (entry.storeLoadEligible) {
            if (!entry.storeLoadInFlight) {
                entry.storeLoadInFlight = true;
                long generation = entry.generation;
                loadExecutor.execute(() -> loadSectionFromStore(server, worldKey, alignedOrigin, generation));
            }
            return;
        }
        queueLiveBuildLocked(worldKey, alignedOrigin, entry);
    }

    private void loadSectionFromStore(
        MinecraftServer server,
        RegistryKey<World> worldKey,
        BlockPos alignedOrigin,
        long generation
    ) {
        SectionSnapshot loadedSnapshot = new SectionSnapshot();
        boolean loaded = staticStore.loadSection(server, worldKey, alignedOrigin, loadedSnapshot);
        synchronized (this) {
            DimensionMirror dimension = dimensions.get(worldKey);
            if (dimension == null) {
                return;
            }
            SectionEntry entry = dimension.sections.get(alignedOrigin.asLong());
            if (entry == null) {
                return;
            }
            entry.storeLoadInFlight = false;
            if (!entry.dirty) {
                return;
            }
            if (entry.generation != generation) {
                requestSectionBuildLocked(server, worldKey, alignedOrigin, entry);
                return;
            }
            if (loaded) {
                copySnapshot(loadedSnapshot, entry.snapshot);
                entry.dirty = false;
                entry.storeLoadEligible = false;
                return;
            }
            entry.storeLoadEligible = false;
            queueLiveBuildLocked(worldKey, alignedOrigin, entry);
        }
    }

    private void queueLiveBuildLocked(RegistryKey<World> worldKey, BlockPos alignedOrigin, SectionEntry entry) {
        if (entry.liveBuildQueued) {
            return;
        }
        BuildRequest request = new BuildRequest(worldKey, alignedOrigin);
        if (queuedLiveBuilds.add(request)) {
            entry.liveBuildQueued = true;
            pendingLiveBuilds.addLast(request);
        }
    }

    private static void copySnapshot(SectionSnapshot source, SectionSnapshot target) {
        System.arraycopy(source.obstacle(), 0, target.obstacle(), 0, SECTION_CELLS);
        System.arraycopy(source.air(), 0, target.air(), 0, SECTION_CELLS);
        System.arraycopy(source.surfaceKind(), 0, target.surfaceKind(), 0, SECTION_CELLS);
        System.arraycopy(source.emitterPowerWatts(), 0, target.emitterPowerWatts(), 0, SECTION_CELLS);
        System.arraycopy(source.openFaceMask(), 0, target.openFaceMask(), 0, SECTION_CELLS);
        System.arraycopy(source.faceSkyExposure(), 0, target.faceSkyExposure(), 0, SECTION_CELLS * FACE_COUNT);
        System.arraycopy(source.faceDirectExposure(), 0, target.faceDirectExposure(), 0, SECTION_CELLS * FACE_COUNT);
        target.setVersion(source.version());
    }

    private DimensionMirror dimension(RegistryKey<World> worldKey) {
        return dimensions.computeIfAbsent(worldKey, ignored -> new DimensionMirror());
    }

    private BlockPos alignSectionOrigin(BlockPos pos) {
        int x = Math.floorDiv(pos.getX(), SECTION_SIZE) * SECTION_SIZE;
        int y = Math.floorDiv(pos.getY(), SECTION_SIZE) * SECTION_SIZE;
        int z = Math.floorDiv(pos.getZ(), SECTION_SIZE) * SECTION_SIZE;
        return new BlockPos(x, y, z);
    }
}
