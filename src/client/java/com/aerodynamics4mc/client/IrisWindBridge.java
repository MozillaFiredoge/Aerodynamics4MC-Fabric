package com.aerodynamics4mc.client;

import java.lang.reflect.Method;
import java.util.Arrays;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.aerodynamics4mc.ModBlocks;
import com.aerodynamics4mc.api.AeroClientWindApi;
import com.aerodynamics4mc.api.SamplePolicy;

import net.fabricmc.fabric.api.client.event.lifecycle.v1.ClientLifecycleEvents;
import net.fabricmc.fabric.api.client.event.lifecycle.v1.ClientTickEvents;
import net.fabricmc.fabric.api.client.networking.v1.ClientPlayConnectionEvents;
import net.fabricmc.fabric.api.client.rendering.v1.world.WorldRenderEvents;
import net.fabricmc.fabric.api.client.rendering.v1.world.WorldTerrainRenderContext;
import net.fabricmc.loader.api.FabricLoader;
import net.minecraft.client.MinecraftClient;
import net.minecraft.client.texture.NativeImage;
import net.minecraft.client.texture.NativeImageBackedTexture;
import net.minecraft.client.texture.TextureManager;
import net.minecraft.util.Identifier;
import net.minecraft.util.math.MathHelper;
import net.minecraft.util.math.Vec3d;

final class IrisWindBridge {
    private static final Logger LOGGER = LoggerFactory.getLogger("aerodynamics4mc/IrisWindBridge");
    static final Identifier WIND_TEXTURE_ID = Identifier.of(ModBlocks.MOD_ID, "dynamic/foliage_wind");

    static final int GRID_X = 48;
    static final int GRID_Y = 24;
    static final int GRID_Z = 48;
    static final int CELL_SIZE_BLOCKS = 4;
    static final float ENCODED_MAX_WIND_MPS = 1.0f;
    static final int TEXTURE_WIDTH = GRID_X * GRID_Z;
    static final int TEXTURE_HEIGHT = GRID_Y;
    static final int REFRESH_INTERVAL_TICKS = 5;
    private static final int TARGET_REFRESH_CELLS_PER_TICK = 4096;
    static final float SPRING_STIFFNESS = 0.18f;
    static final float SPRING_DAMPING = 0.12f;
    static final float WIND_TO_BEND_SCALE = 9.6f;
    static final float MAX_BEND_MAGNITUDE = 1.2f;
    static final float MAX_BEND_VELOCITY_PER_TICK = 0.30f;

    private static boolean irisReflectionInitialized;
    private static Object irisApiInstance;
    private static Method irisShaderPackInUseMethod;

    private final AeroVisualizer visualizer;
    private NativeImageBackedTexture windTexture;
    private float[] targetWindField;
    private float[] bendField;
    private float[] bendVelocityField;
    private float[] shiftWindBuffer;
    private float[] shiftBendBuffer;
    private float[] shiftVelocityBuffer;
    private boolean dirty = true;
    private boolean streamingEnabled;
    private boolean targetRefreshInProgress;
    private long targetRefreshOriginX;
    private long targetRefreshOriginY;
    private long targetRefreshOriginZ;
    private int targetRefreshCursor;
    private int targetRefreshNonZeroCells;
    private double targetRefreshMaxSpeed;
    private double targetRefreshTotalSpeed;
    private long lastAnchorX = Long.MIN_VALUE;
    private long lastAnchorY = Long.MIN_VALUE;
    private long lastAnchorZ = Long.MIN_VALUE;
    private long lastUploadTick = Long.MIN_VALUE;
    private long lastTextureUploadTick = Long.MIN_VALUE;
    private long lastDiagnosticTick = Long.MIN_VALUE;
    private boolean loggedMissingIris;
    private boolean loggedInactiveShaderpack;

    IrisWindBridge(AeroVisualizer visualizer) {
        this.visualizer = visualizer;
    }

    void initialize() {
        ClientTickEvents.END_CLIENT_TICK.register(this::onClientTick);
        WorldRenderEvents.START_MAIN.register(this::onRenderFrame);
        ClientPlayConnectionEvents.DISCONNECT.register((handler, client) -> clear());
        ClientLifecycleEvents.CLIENT_STOPPING.register(client -> close());
    }

    void onRuntimeState(boolean streamingEnabled) {
        this.streamingEnabled = streamingEnabled;
        this.dirty = true;
        if (!streamingEnabled) {
            zeroTexture();
        }
    }

    void markDirty() {
        dirty = true;
    }

    private void onClientTick(MinecraftClient client) {
        boolean irisLoaded = FabricLoader.getInstance().isModLoaded("iris");
        if (!irisLoaded) {
            if (!loggedMissingIris) {
                LOGGER.info("Iris wind bridge idle: Iris mod not loaded");
                loggedMissingIris = true;
            }
            return;
        }
        loggedMissingIris = false;

        boolean shaderPackInUse = isShaderPackInUseReflective();
        if (!shaderPackInUse) {
            if (!loggedInactiveShaderpack) {
                LOGGER.info("Iris wind bridge idle: no active shaderpack detected");
                loggedInactiveShaderpack = true;
            }
            return;
        }
        loggedInactiveShaderpack = false;
        if (client.world == null || client.player == null) {
            return;
        }
        ensureTexture(client.getTextureManager());
        if (windTexture == null) {
            return;
        }
        Vec3d anchorPos = client.gameRenderer != null
            ? client.gameRenderer.getCamera().getCameraPos()
            : new Vec3d(client.player.getX(), client.player.getY(), client.player.getZ());
        long anchorX = quantizedOrigin(anchorPos.x, GRID_X);
        long anchorY = quantizedOrigin(anchorPos.y, GRID_Y);
        long anchorZ = quantizedOrigin(anchorPos.z, GRID_Z);
        boolean anchorChanged = anchorX != lastAnchorX || anchorY != lastAnchorY || anchorZ != lastAnchorZ;
        boolean periodicRefresh = lastUploadTick == Long.MIN_VALUE || client.world.getTime() - lastUploadTick >= REFRESH_INTERVAL_TICKS;
        if (!dirty && !anchorChanged && !periodicRefresh) {
            return;
        }
        RefreshStats stats = refreshTargetWindField(client, anchorX, anchorY, anchorZ, anchorChanged);
        if (!stats.complete()) {
            return;
        }
        lastAnchorX = anchorX;
        lastAnchorY = anchorY;
        lastAnchorZ = anchorZ;
        lastUploadTick = client.world.getTime();
        dirty = false;
        if (lastDiagnosticTick == Long.MIN_VALUE || client.world.getTime() - lastDiagnosticTick >= 100) {
            LOGGER.info(
                "Iris wind refresh: streaming={} nonZeroCells={} maxSpeed={} meanSpeed={} origin=({}, {}, {})",
                streamingEnabled,
                stats.nonZeroCells(),
                String.format("%.3f", stats.maxSpeed()),
                String.format("%.3f", stats.meanSpeed()),
                anchorX,
                anchorY,
                anchorZ
            );
            lastDiagnosticTick = client.world.getTime();
        }
    }

    private void onRenderFrame(WorldTerrainRenderContext context) {
        MinecraftClient client = MinecraftClient.getInstance();
        if (client == null || client.world == null || client.player == null || windTexture == null || windTexture.getImage() == null) {
            return;
        }
        if (!streamingEnabled) {
            return;
        }
        if (!FabricLoader.getInstance().isModLoaded("iris") || !isShaderPackInUseReflective()) {
            return;
        }
        long gameTime = client.world.getTime();
        if (lastTextureUploadTick == gameTime) {
            return;
        }
        float deltaTicks = MathHelper.clamp(client.getRenderTickCounter().getDynamicDeltaTicks(), 0.05f, 1.5f);
        integrateBendField(deltaTicks);
        uploadBendTexture();
        lastTextureUploadTick = gameTime;
    }

    private RefreshStats refreshTargetWindField(MinecraftClient client, long originX, long originY, long originZ, boolean anchorChanged) {
        if (windTexture == null || windTexture.getImage() == null || client.world == null) {
            return new RefreshStats(0, 0.0, 0.0, true);
        }
        ensureStateArrays();
        if (anchorChanged && !targetRefreshInProgress) {
            shiftStateFields(originX, originY, originZ);
        }
        if (!targetRefreshInProgress
            || targetRefreshOriginX != originX
            || targetRefreshOriginY != originY
            || targetRefreshOriginZ != originZ) {
            beginTargetRefresh(originX, originY, originZ);
        }

        int cellCount = GRID_X * GRID_Y * GRID_Z;
        int end = Math.min(cellCount, targetRefreshCursor + TARGET_REFRESH_CELLS_PER_TICK);
        while (targetRefreshCursor < end) {
            int cell = targetRefreshCursor++;
            int x = cell % GRID_X;
            int yz = cell / GRID_X;
            int z = yz % GRID_Z;
            int y = yz / GRID_Z;
            int windBase = cell * 3;
            double worldX = originX + (x + 0.5) * CELL_SIZE_BLOCKS;
            double worldY = originY + (y + 0.5) * CELL_SIZE_BLOCKS;
            double worldZ = originZ + (z + 0.5) * CELL_SIZE_BLOCKS;
            Vec3d sampledWind = streamingEnabled
                ? AeroClientWindApi.sample(
                    client.world,
                    new Vec3d(worldX, worldY, worldZ),
                    SamplePolicy.CLIENT_LOCAL_PREFERRED
                ).effectiveVelocity()
                : Vec3d.ZERO;
            targetWindField[windBase] = (float) sampledWind.x;
            targetWindField[windBase + 1] = (float) sampledWind.y;
            targetWindField[windBase + 2] = (float) sampledWind.z;
            double speed = sampledWind.length();
            if (speed > 0.01) {
                targetRefreshNonZeroCells++;
                targetRefreshTotalSpeed += speed;
                if (speed > targetRefreshMaxSpeed) {
                    targetRefreshMaxSpeed = speed;
                }
            }
        }

        if (targetRefreshCursor < cellCount) {
            return new RefreshStats(targetRefreshNonZeroCells, targetRefreshMaxSpeed, meanSpeed(), false);
        }
        RefreshStats stats = new RefreshStats(targetRefreshNonZeroCells, targetRefreshMaxSpeed, meanSpeed(), true);
        targetRefreshInProgress = false;
        return stats;
    }

    private void beginTargetRefresh(long originX, long originY, long originZ) {
        targetRefreshInProgress = true;
        targetRefreshOriginX = originX;
        targetRefreshOriginY = originY;
        targetRefreshOriginZ = originZ;
        targetRefreshCursor = 0;
        targetRefreshNonZeroCells = 0;
        targetRefreshMaxSpeed = 0.0;
        targetRefreshTotalSpeed = 0.0;
    }

    private double meanSpeed() {
        return targetRefreshNonZeroCells > 0 ? targetRefreshTotalSpeed / targetRefreshNonZeroCells : 0.0;
    }

    private void ensureTexture(TextureManager textureManager) {
        if (windTexture != null) {
            return;
        }
        NativeImage image = new NativeImage(TEXTURE_WIDTH, TEXTURE_HEIGHT, false);
        image.fillRect(0, 0, TEXTURE_WIDTH, TEXTURE_HEIGHT, 0x00000000);
        windTexture = new NativeImageBackedTexture(() -> "aerodynamics4mc_iris_wind_bridge", image);
        textureManager.registerTexture(WIND_TEXTURE_ID, windTexture);
        dirty = true;
    }

    private void ensureStateArrays() {
        int cellCount = GRID_X * GRID_Y * GRID_Z;
        int windComponentCount = cellCount * 3;
        int bendComponentCount = cellCount * 2;
        if (targetWindField == null || targetWindField.length != windComponentCount) {
            targetWindField = new float[windComponentCount];
        }
        if (bendField == null || bendField.length != bendComponentCount) {
            bendField = new float[bendComponentCount];
        }
        if (bendVelocityField == null || bendVelocityField.length != bendComponentCount) {
            bendVelocityField = new float[bendComponentCount];
        }
    }

    private void shiftStateFields(long originX, long originY, long originZ) {
        if (targetWindField == null || bendField == null || bendVelocityField == null
            || lastAnchorX == Long.MIN_VALUE || lastAnchorY == Long.MIN_VALUE || lastAnchorZ == Long.MIN_VALUE) {
            return;
        }
        int deltaX = (int) ((originX - lastAnchorX) / CELL_SIZE_BLOCKS);
        int deltaY = (int) ((originY - lastAnchorY) / CELL_SIZE_BLOCKS);
        int deltaZ = (int) ((originZ - lastAnchorZ) / CELL_SIZE_BLOCKS);
        if (deltaX == 0 && deltaY == 0 && deltaZ == 0) {
            return;
        }
        if (Math.abs(deltaX) >= GRID_X || Math.abs(deltaY) >= GRID_Y || Math.abs(deltaZ) >= GRID_Z) {
            Arrays.fill(targetWindField, 0.0f);
            Arrays.fill(bendField, 0.0f);
            Arrays.fill(bendVelocityField, 0.0f);
            return;
        }
        ensureShiftBuffers();
        Arrays.fill(shiftWindBuffer, 0.0f);
        Arrays.fill(shiftBendBuffer, 0.0f);
        Arrays.fill(shiftVelocityBuffer, 0.0f);
        for (int y = 0; y < GRID_Y; y++) {
            for (int z = 0; z < GRID_Z; z++) {
                for (int x = 0; x < GRID_X; x++) {
                    int sourceX = x + deltaX;
                    int sourceY = y + deltaY;
                    int sourceZ = z + deltaZ;
                    if (sourceX < 0 || sourceY < 0 || sourceZ < 0 || sourceX >= GRID_X || sourceY >= GRID_Y || sourceZ >= GRID_Z) {
                        continue;
                    }
                    int targetWindBase = cellIndex(x, y, z) * 3;
                    int sourceWindBase = cellIndex(sourceX, sourceY, sourceZ) * 3;
                    shiftWindBuffer[targetWindBase] = targetWindField[sourceWindBase];
                    shiftWindBuffer[targetWindBase + 1] = targetWindField[sourceWindBase + 1];
                    shiftWindBuffer[targetWindBase + 2] = targetWindField[sourceWindBase + 2];

                    int targetBendBase = cellIndex(x, y, z) * 2;
                    int sourceBendBase = cellIndex(sourceX, sourceY, sourceZ) * 2;
                    shiftBendBuffer[targetBendBase] = bendField[sourceBendBase];
                    shiftBendBuffer[targetBendBase + 1] = bendField[sourceBendBase + 1];
                    shiftVelocityBuffer[targetBendBase] = bendVelocityField[sourceBendBase];
                    shiftVelocityBuffer[targetBendBase + 1] = bendVelocityField[sourceBendBase + 1];
                }
            }
        }
        float[] oldWind = targetWindField;
        targetWindField = shiftWindBuffer;
        shiftWindBuffer = oldWind;
        float[] oldBend = bendField;
        bendField = shiftBendBuffer;
        shiftBendBuffer = oldBend;
        float[] oldVelocity = bendVelocityField;
        bendVelocityField = shiftVelocityBuffer;
        shiftVelocityBuffer = oldVelocity;
    }

    private void ensureShiftBuffers() {
        if (shiftWindBuffer == null || shiftWindBuffer.length != targetWindField.length) {
            shiftWindBuffer = new float[targetWindField.length];
        }
        if (shiftBendBuffer == null || shiftBendBuffer.length != bendField.length) {
            shiftBendBuffer = new float[bendField.length];
        }
        if (shiftVelocityBuffer == null || shiftVelocityBuffer.length != bendVelocityField.length) {
            shiftVelocityBuffer = new float[bendVelocityField.length];
        }
    }

    private void integrateBendField(float deltaTicks) {
        ensureStateArrays();
        float clampedDelta = Math.max(0.01f, deltaTicks);
        int cellCount = GRID_X * GRID_Y * GRID_Z;
        for (int cell = 0; cell < cellCount; cell++) {
            int windBase = cell * 3;
            int bendBase = cell * 2;
            float targetX = MathHelper.clamp(targetWindField[windBase] * WIND_TO_BEND_SCALE, -MAX_BEND_MAGNITUDE, MAX_BEND_MAGNITUDE);
            float targetZ = MathHelper.clamp(targetWindField[windBase + 2] * WIND_TO_BEND_SCALE, -MAX_BEND_MAGNITUDE, MAX_BEND_MAGNITUDE);

            float bendX = bendField[bendBase];
            float bendZ = bendField[bendBase + 1];
            float velX = bendVelocityField[bendBase];
            float velZ = bendVelocityField[bendBase + 1];

            float accelX = (targetX - bendX) * SPRING_STIFFNESS - velX * SPRING_DAMPING;
            float accelZ = (targetZ - bendZ) * SPRING_STIFFNESS - velZ * SPRING_DAMPING;

            velX += accelX * clampedDelta;
            velZ += accelZ * clampedDelta;
            velX = MathHelper.clamp(velX, -MAX_BEND_VELOCITY_PER_TICK, MAX_BEND_VELOCITY_PER_TICK);
            velZ = MathHelper.clamp(velZ, -MAX_BEND_VELOCITY_PER_TICK, MAX_BEND_VELOCITY_PER_TICK);

            bendX = MathHelper.clamp(bendX + velX * clampedDelta, -MAX_BEND_MAGNITUDE, MAX_BEND_MAGNITUDE);
            bendZ = MathHelper.clamp(bendZ + velZ * clampedDelta, -MAX_BEND_MAGNITUDE, MAX_BEND_MAGNITUDE);

            bendField[bendBase] = bendX;
            bendField[bendBase + 1] = bendZ;
            bendVelocityField[bendBase] = velX;
            bendVelocityField[bendBase + 1] = velZ;
        }
    }

    private void uploadBendTexture() {
        if (windTexture == null || windTexture.getImage() == null || bendField == null) {
            return;
        }
        NativeImage image = windTexture.getImage();
        for (int y = 0; y < GRID_Y; y++) {
            for (int z = 0; z < GRID_Z; z++) {
                for (int x = 0; x < GRID_X; x++) {
                    int bendBase = cellIndex(x, y, z) * 2;
                    image.setColorArgb(flattenX(x, z), y, encodeWind(bendField[bendBase], 0.0f, bendField[bendBase + 1]));
                }
            }
        }
        windTexture.upload();
    }

    private void zeroTexture() {
        if (windTexture == null || windTexture.getImage() == null) {
            return;
        }
        NativeImage image = windTexture.getImage();
        image.fillRect(0, 0, TEXTURE_WIDTH, TEXTURE_HEIGHT, 0x00000000);
        windTexture.upload();
        if (targetWindField != null) {
            Arrays.fill(targetWindField, 0.0f);
        }
        if (bendField != null) {
            Arrays.fill(bendField, 0.0f);
        }
        if (bendVelocityField != null) {
            Arrays.fill(bendVelocityField, 0.0f);
        }
    }

    private void clear() {
        dirty = true;
        streamingEnabled = false;
        lastAnchorX = Long.MIN_VALUE;
        lastAnchorY = Long.MIN_VALUE;
        lastAnchorZ = Long.MIN_VALUE;
        lastUploadTick = Long.MIN_VALUE;
        lastTextureUploadTick = Long.MIN_VALUE;
        lastDiagnosticTick = Long.MIN_VALUE;
        targetRefreshInProgress = false;
        targetRefreshCursor = 0;
        zeroTexture();
    }

    private void close() {
        clear();
        MinecraftClient client = MinecraftClient.getInstance();
        if (client != null) {
            client.getTextureManager().destroyTexture(WIND_TEXTURE_ID);
        }
        if (windTexture != null) {
            windTexture.close();
            windTexture = null;
        }
        targetWindField = null;
        bendField = null;
        bendVelocityField = null;
        shiftWindBuffer = null;
        shiftBendBuffer = null;
        shiftVelocityBuffer = null;
    }

    private static int flattenX(int x, int z) {
        return z * GRID_X + x;
    }

    private static int cellIndex(int x, int y, int z) {
        return (y * GRID_Z + z) * GRID_X + x;
    }

    private static long quantizedOrigin(double cameraCoord, int axisCells) {
        int extentBlocks = axisCells * CELL_SIZE_BLOCKS;
        int halfExtentBlocks = extentBlocks / 2;
        int aligned = MathHelper.floor(cameraCoord / CELL_SIZE_BLOCKS) * CELL_SIZE_BLOCKS;
        return (long) aligned - halfExtentBlocks;
    }

    private static int encodeWind(float windX, float windY, float windZ) {
        float wx = MathHelper.clamp(windX, -ENCODED_MAX_WIND_MPS, ENCODED_MAX_WIND_MPS);
        float wy = MathHelper.clamp(windY, -ENCODED_MAX_WIND_MPS, ENCODED_MAX_WIND_MPS);
        float wz = MathHelper.clamp(windZ, -ENCODED_MAX_WIND_MPS, ENCODED_MAX_WIND_MPS);
        float magnitude = MathHelper.clamp((float) Math.sqrt(wx * wx + wy * wy + wz * wz) / ENCODED_MAX_WIND_MPS, 0.0f, 1.0f);
        int a = Math.round(magnitude * 255.0f);
        int r = encodeSigned(wx);
        int g = encodeSigned(wy);
        int b = encodeSigned(wz);
        return (a << 24) | (r << 16) | (g << 8) | b;
    }

    private static int encodeSigned(float value) {
        float normalized = value / ENCODED_MAX_WIND_MPS;
        return Math.round((MathHelper.clamp(normalized, -1.0f, 1.0f) * 0.5f + 0.5f) * 255.0f);
    }

    private record RefreshStats(int nonZeroCells, double maxSpeed, double meanSpeed, boolean complete) {}

    @SuppressWarnings("unused")
    private static boolean isShaderPackInUseReflective() {
        try {
            if (!irisReflectionInitialized) {
                Class<?> irisApiClass = Class.forName("net.irisshaders.iris.api.v0.IrisApi");
                Method getInstance = irisApiClass.getMethod("getInstance");
                irisApiInstance = getInstance.invoke(null);
                irisShaderPackInUseMethod = irisApiClass.getMethod("isShaderPackInUse");
                irisReflectionInitialized = true;
            }
            if (irisApiInstance == null || irisShaderPackInUseMethod == null) {
                return false;
            }
            Object result = irisShaderPackInUseMethod.invoke(irisApiInstance);
            return result instanceof Boolean bool && bool;
        } catch (Throwable ignored) {
            irisReflectionInitialized = true;
            irisApiInstance = null;
            irisShaderPackInUseMethod = null;
            return false;
        }
    }
}
