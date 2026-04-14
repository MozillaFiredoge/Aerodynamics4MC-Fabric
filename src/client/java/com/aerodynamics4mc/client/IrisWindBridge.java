package com.aerodynamics4mc.client;

import java.lang.reflect.Method;

import com.aerodynamics4mc.ModBlocks;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import net.fabricmc.fabric.api.client.event.lifecycle.v1.ClientLifecycleEvents;
import net.fabricmc.fabric.api.client.event.lifecycle.v1.ClientTickEvents;
import net.fabricmc.fabric.api.client.networking.v1.ClientPlayConnectionEvents;
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

    private final AeroVisualizer visualizer;
    private NativeImageBackedTexture windTexture;
    private boolean dirty = true;
    private boolean streamingEnabled;
    private long lastAnchorX = Long.MIN_VALUE;
    private long lastAnchorY = Long.MIN_VALUE;
    private long lastAnchorZ = Long.MIN_VALUE;
    private long lastUploadTick = Long.MIN_VALUE;
    private long lastDiagnosticTick = Long.MIN_VALUE;
    private boolean loggedMissingIris;
    private boolean loggedInactiveShaderpack;

    IrisWindBridge(AeroVisualizer visualizer) {
        this.visualizer = visualizer;
    }

    void initialize() {
        ClientTickEvents.END_CLIENT_TICK.register(this::onClientTick);
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
        RefreshStats stats = rebuildTexture(client, anchorX, anchorY, anchorZ);
        lastAnchorX = anchorX;
        lastAnchorY = anchorY;
        lastAnchorZ = anchorZ;
        lastUploadTick = client.world.getTime();
        dirty = false;
        if (lastDiagnosticTick == Long.MIN_VALUE || client.world.getTime() - lastDiagnosticTick >= 100) {
            LOGGER.info(
                "Iris wind refresh: streaming={} nonZeroCells={} maxSpeed={} meanSpeed={} origin=({}, {}, {})",
                streamingEnabled,
                stats.nonZeroCells,
                String.format("%.3f", stats.maxSpeed),
                String.format("%.3f", stats.meanSpeed),
                anchorX,
                anchorY,
                anchorZ
            );
            lastDiagnosticTick = client.world.getTime();
        }
    }

    private RefreshStats rebuildTexture(MinecraftClient client, long originX, long originY, long originZ) {
        if (windTexture == null || windTexture.getImage() == null || client.world == null) {
            return new RefreshStats(0, 0.0, 0.0);
        }
        NativeImage image = windTexture.getImage();
        Identifier dimensionId = client.world.getRegistryKey().getValue();
        int nonZeroCells = 0;
        double maxSpeed = 0.0;
        double totalSpeed = 0.0;
        for (int y = 0; y < GRID_Y; y++) {
            for (int z = 0; z < GRID_Z; z++) {
                for (int x = 0; x < GRID_X; x++) {
                    double worldX = originX + (x + 0.5) * CELL_SIZE_BLOCKS;
                    double worldY = originY + (y + 0.5) * CELL_SIZE_BLOCKS;
                    double worldZ = originZ + (z + 0.5) * CELL_SIZE_BLOCKS;
                    Vec3d sampledWind = streamingEnabled
                        ? visualizer.sampleWind(dimensionId, new Vec3d(worldX, worldY, worldZ))
                        : Vec3d.ZERO;
                    double speed = sampledWind.length();
                    if (speed > 0.01) {
                        nonZeroCells++;
                        totalSpeed += speed;
                        if (speed > maxSpeed) {
                            maxSpeed = speed;
                        }
                    }
                    int packed = encodeWind(sampledWind);
                    image.setColorArgb(flattenX(x, z), y, packed);
                }
            }
        }
        windTexture.upload();
        double meanSpeed = nonZeroCells > 0 ? totalSpeed / nonZeroCells : 0.0;
        return new RefreshStats(nonZeroCells, maxSpeed, meanSpeed);
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

    private void zeroTexture() {
        if (windTexture == null || windTexture.getImage() == null) {
            return;
        }
        NativeImage image = windTexture.getImage();
        image.fillRect(0, 0, TEXTURE_WIDTH, TEXTURE_HEIGHT, 0x00000000);
        windTexture.upload();
    }

    private void clear() {
        dirty = true;
        streamingEnabled = false;
        lastAnchorX = Long.MIN_VALUE;
        lastAnchorY = Long.MIN_VALUE;
        lastAnchorZ = Long.MIN_VALUE;
        lastUploadTick = Long.MIN_VALUE;
        lastDiagnosticTick = Long.MIN_VALUE;
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
    }

    private static int flattenX(int x, int z) {
        return z * GRID_X + x;
    }

    private static long quantizedOrigin(double cameraCoord, int axisCells) {
        int extentBlocks = axisCells * CELL_SIZE_BLOCKS;
        int halfExtentBlocks = extentBlocks / 2;
        int aligned = MathHelper.floor(cameraCoord / CELL_SIZE_BLOCKS) * CELL_SIZE_BLOCKS;
        return (long) aligned - halfExtentBlocks;
    }

    private static int encodeWind(Vec3d wind) {
        float wx = MathHelper.clamp((float) wind.x, -ENCODED_MAX_WIND_MPS, ENCODED_MAX_WIND_MPS);
        float wy = MathHelper.clamp((float) wind.y, -ENCODED_MAX_WIND_MPS, ENCODED_MAX_WIND_MPS);
        float wz = MathHelper.clamp((float) wind.z, -ENCODED_MAX_WIND_MPS, ENCODED_MAX_WIND_MPS);
        float magnitude = MathHelper.clamp((float) wind.length() / ENCODED_MAX_WIND_MPS, 0.0f, 1.0f);
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

    private record RefreshStats(int nonZeroCells, double maxSpeed, double meanSpeed) {}

    @SuppressWarnings("unused")
    private static boolean isShaderPackInUseReflective() {
        try {
            Class<?> irisApiClass = Class.forName("net.irisshaders.iris.api.v0.IrisApi");
            Method getInstance = irisApiClass.getMethod("getInstance");
            Object api = getInstance.invoke(null);
            Method isShaderPackInUse = irisApiClass.getMethod("isShaderPackInUse");
            Object result = isShaderPackInUse.invoke(api);
            return result instanceof Boolean bool && bool;
        } catch (Throwable ignored) {
            return false;
        }
    }
}
