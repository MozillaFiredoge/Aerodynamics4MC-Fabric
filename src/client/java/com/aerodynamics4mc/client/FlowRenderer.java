package com.aerodynamics4mc.client;

import org.joml.Matrix4f;

import net.fabricmc.fabric.api.client.rendering.v1.world.WorldRenderContext;
import net.minecraft.client.render.RenderLayers;
import net.minecraft.client.render.VertexConsumer;
import net.minecraft.client.util.math.MatrixStack;
import net.minecraft.util.math.BlockPos;
import net.minecraft.util.math.MathHelper;
import net.minecraft.util.math.Vec3d;

public class FlowRenderer {
    private static final int CHANNELS = 4;
    private static final float MIN_SPEED = 1e-6f;
    private static final float RENDER_THRESHOLD_NORMALIZED = 0.01f;

    private final int gridSize;
    private float maxInflowSpeed;

    private float[] flowField;
    private int sampledGridSize;
    private int sampleStride;
    private int streamlineSampleStride;
    private BlockPos origin;
    private boolean hasFlowData;

    public FlowRenderer(int gridSize, float maxInflowSpeed) {
        this.gridSize = gridSize;
        this.maxInflowSpeed = Math.max(MIN_SPEED, maxInflowSpeed);
        this.sampleStride = 4;
        this.streamlineSampleStride = 4;
        this.sampledGridSize = (gridSize + sampleStride - 1) / sampleStride;
        this.flowField = new float[sampledGridSize * sampledGridSize * sampledGridSize * CHANNELS];
        this.origin = BlockPos.ORIGIN;
        this.hasFlowData = false;
    }

    public void setMaxInflowSpeed(float maxInflowSpeed) {
        this.maxInflowSpeed = Math.max(MIN_SPEED, maxInflowSpeed);
    }

    public void setStreamlineSampleStride(int stride) {
        this.streamlineSampleStride = sanitizeStride(stride);
    }

    public void updateFlowField(BlockPos origin, float[] data) {
        updateFlowField(origin, sampleStride, data);
    }

    public void updateFlowField(BlockPos origin, int sampleStride, float[] data) {
        if (origin == null || data == null || data.length == 0 || data.length % CHANNELS != 0) {
            return;
        }

        int cellCount = data.length / CHANNELS;
        int n = (int) Math.round(Math.cbrt(cellCount));
        if (n <= 0 || n * n * n != cellCount) {
            return;
        }

        this.origin = origin;
        this.sampleStride = Math.max(1, sampleStride);
        this.sampledGridSize = n;

        if (flowField.length != data.length) {
            flowField = new float[data.length];
        }
        System.arraycopy(data, 0, flowField, 0, data.length);
        hasFlowData = true;
    }

    public void render(WorldRenderContext context) {
        if (!hasFlowData) {
            return;
        }
        if (context == null || context.gameRenderer() == null || context.consumers() == null || context.matrices() == null) {
            return;
        }

        var camera = context.gameRenderer().getCamera();
        if (camera == null) {
            return;
        }

        MatrixStack matrices = context.matrices();
        Vec3d camPos = camera.getCameraPos();
        matrices.push();
        matrices.translate(
            origin.getX() - camPos.x,
            origin.getY() - camPos.y,
            origin.getZ() - camPos.z
        );

        VertexConsumer buffer = context.consumers().getBuffer(RenderLayers.lines());
        renderVelocityField(buffer, matrices);
        renderStreamlines(buffer, matrices);

        matrices.pop();
    }

    private void renderVelocityField(VertexConsumer buffer, MatrixStack matrices) {
        int stride = Math.max(sampleStride, streamlineSampleStride);
        float velScale = 5f;
        var entry = matrices.peek();
        Matrix4f matrix = entry.getPositionMatrix();

        for (int x = 0; x < gridSize; x += stride) {
            for (int y = 0; y < gridSize; y += stride) {
                for (int z = 0; z < gridSize; z += stride) {
                    Vec3d vel = sampleVelocityLocal(x + 0.5, y + 0.5, z + 0.5);
                    float speed = (float) vel.length();
                    float speedNorm = MathHelper.clamp(speed / maxInflowSpeed, 0.0f, 1.0f);
                    if (speedNorm < RENDER_THRESHOLD_NORMALIZED) {
                        continue;
                    }

                    Vec3d dir = vel.normalize();
                    float lineLength = MathHelper.clamp(speedNorm * velScale, 0.05f, 10f);
                    int color = getViridisColor(speedNorm * velScale);
                    int r = (color >> 16) & 0xFF;
                    int g = (color >> 8) & 0xFF;
                    int b = color & 0xFF;

                    float fx = x + 0.5f;
                    float fy = y + 0.5f;
                    float fz = z + 0.5f;

                    buffer.vertex(matrix, fx, fy, fz)
                        .color(r, g, b, 255)
                        .normal(entry, (float) dir.x, (float) dir.y, (float) dir.z)
                        .lineWidth(1.2f);
                    buffer.vertex(
                            matrix,
                            fx + (float) dir.x * lineLength,
                            fy + (float) dir.y * lineLength,
                            fz + (float) dir.z * lineLength
                        )
                        .color(r, g, b, 255)
                        .normal(entry, (float) dir.x, (float) dir.y, (float) dir.z)
                        .lineWidth(1.2f);
                }
            }
        }
    }

    private void renderStreamlines(VertexConsumer buffer, MatrixStack matrices) {
        int seedStride = streamlineSampleStride;
        float stepSize = 0.35f;
        int maxSteps = 80;
        var entry = matrices.peek();
        Matrix4f matrix = entry.getPositionMatrix();

        for (int y = 0; y < gridSize; y += seedStride) {
            for (int z = 0; z < gridSize; z += seedStride) {
                Vec3d pos = new Vec3d(0.5, y + 0.5, z + 0.5);

                for (int step = 0; step < maxSteps; step++) {
                    Vec3d vel = sampleVelocityLocal(pos.x, pos.y, pos.z);
                    float speed = (float) vel.length();
                    if (speed < MIN_SPEED) {
                        break;
                    }

                    float speedNorm = MathHelper.clamp(speed / maxInflowSpeed, 0.0f, 1.0f);
                    if (speedNorm < RENDER_THRESHOLD_NORMALIZED) {
                        break;
                    }

                    Vec3d dir = vel.normalize();
                    float advectStep = stepSize * MathHelper.clamp(speedNorm * 8.0f, 0.2f, 1.25f);
                    Vec3d nextPos = pos.add(dir.multiply(advectStep));

                    if (nextPos.x < 0 || nextPos.x >= gridSize
                        || nextPos.y < 0 || nextPos.y >= gridSize
                        || nextPos.z < 0 || nextPos.z >= gridSize) {
                        break;
                    }

                    int color = getViridisColor(speedNorm);
                    int r = (color >> 16) & 0xFF;
                    int g = (color >> 8) & 0xFF;
                    int b = color & 0xFF;
                    Vec3d segDir = nextPos.subtract(pos).normalize();

                    buffer.vertex(matrix, (float) pos.x, (float) pos.y, (float) pos.z)
                        .color(r, g, b, 255)
                        .normal(entry, (float) segDir.x, (float) segDir.y, (float) segDir.z)
                        .lineWidth(1.0f);
                    buffer.vertex(matrix, (float) nextPos.x, (float) nextPos.y, (float) nextPos.z)
                        .color(r, g, b, 255)
                        .normal(entry, (float) segDir.x, (float) segDir.y, (float) segDir.z)
                        .lineWidth(1.0f);

                    pos = nextPos;
                }
            }
        }
    }

    public Vec3d sampleVelocity(Vec3d worldPos) {
        if (!hasFlowData) {
            return Vec3d.ZERO;
        }
        return sampleVelocityLocal(
            worldPos.x - origin.getX(),
            worldPos.y - origin.getY(),
            worldPos.z - origin.getZ()
        );
    }

    public float samplePressure(Vec3d worldPos) {
        if (!hasFlowData) {
            return 0.0f;
        }
        return samplePressureLocal(
            worldPos.x - origin.getX(),
            worldPos.y - origin.getY(),
            worldPos.z - origin.getZ()
        );
    }

    private Vec3d sampleVelocityLocal(double localX, double localY, double localZ) {
        if (localX < 0 || localY < 0 || localZ < 0 || localX > gridSize - 1 || localY > gridSize - 1 || localZ > gridSize - 1) {
            return Vec3d.ZERO;
        }

        if (sampledGridSize <= 1) {
            return velocityAtSample(0, 0, 0);
        }

        double gx = localX / sampleStride;
        double gy = localY / sampleStride;
        double gz = localZ / sampleStride;

        double maxCoord = sampledGridSize - 1;
        gx = MathHelper.clamp(gx, 0.0, maxCoord);
        gy = MathHelper.clamp(gy, 0.0, maxCoord);
        gz = MathHelper.clamp(gz, 0.0, maxCoord);

        int x0 = (int) Math.floor(gx);
        int y0 = (int) Math.floor(gy);
        int z0 = (int) Math.floor(gz);
        int x1 = Math.min(sampledGridSize - 1, x0 + 1);
        int y1 = Math.min(sampledGridSize - 1, y0 + 1);
        int z1 = Math.min(sampledGridSize - 1, z0 + 1);

        float fx = (float) (gx - x0);
        float fy = (float) (gy - y0);
        float fz = (float) (gz - z0);

        Vec3d c000 = velocityAtSample(x0, y0, z0);
        Vec3d c100 = velocityAtSample(x1, y0, z0);
        Vec3d c010 = velocityAtSample(x0, y1, z0);
        Vec3d c110 = velocityAtSample(x1, y1, z0);
        Vec3d c001 = velocityAtSample(x0, y0, z1);
        Vec3d c101 = velocityAtSample(x1, y0, z1);
        Vec3d c011 = velocityAtSample(x0, y1, z1);
        Vec3d c111 = velocityAtSample(x1, y1, z1);

        Vec3d c00 = lerp(c000, c100, fx);
        Vec3d c10 = lerp(c010, c110, fx);
        Vec3d c01 = lerp(c001, c101, fx);
        Vec3d c11 = lerp(c011, c111, fx);

        Vec3d c0 = lerp(c00, c10, fy);
        Vec3d c1 = lerp(c01, c11, fy);
        return lerp(c0, c1, fz);
    }

    private float samplePressureLocal(double localX, double localY, double localZ) {
        if (localX < 0 || localY < 0 || localZ < 0 || localX > gridSize - 1 || localY > gridSize - 1 || localZ > gridSize - 1) {
            return 0.0f;
        }

        if (sampledGridSize <= 1) {
            return pressureAtSample(0, 0, 0);
        }

        double gx = MathHelper.clamp(localX / sampleStride, 0.0, sampledGridSize - 1);
        double gy = MathHelper.clamp(localY / sampleStride, 0.0, sampledGridSize - 1);
        double gz = MathHelper.clamp(localZ / sampleStride, 0.0, sampledGridSize - 1);

        int x0 = (int) Math.floor(gx);
        int y0 = (int) Math.floor(gy);
        int z0 = (int) Math.floor(gz);
        int x1 = Math.min(sampledGridSize - 1, x0 + 1);
        int y1 = Math.min(sampledGridSize - 1, y0 + 1);
        int z1 = Math.min(sampledGridSize - 1, z0 + 1);

        float fx = (float) (gx - x0);
        float fy = (float) (gy - y0);
        float fz = (float) (gz - z0);

        float c000 = pressureAtSample(x0, y0, z0);
        float c100 = pressureAtSample(x1, y0, z0);
        float c010 = pressureAtSample(x0, y1, z0);
        float c110 = pressureAtSample(x1, y1, z0);
        float c001 = pressureAtSample(x0, y0, z1);
        float c101 = pressureAtSample(x1, y0, z1);
        float c011 = pressureAtSample(x0, y1, z1);
        float c111 = pressureAtSample(x1, y1, z1);

        float c00 = MathHelper.lerp(fx, c000, c100);
        float c10 = MathHelper.lerp(fx, c010, c110);
        float c01 = MathHelper.lerp(fx, c001, c101);
        float c11 = MathHelper.lerp(fx, c011, c111);

        float c0 = MathHelper.lerp(fy, c00, c10);
        float c1 = MathHelper.lerp(fy, c01, c11);
        return MathHelper.lerp(fz, c0, c1);
    }

    private Vec3d velocityAtSample(int x, int y, int z) {
        int cx = MathHelper.clamp(x, 0, sampledGridSize - 1);
        int cy = MathHelper.clamp(y, 0, sampledGridSize - 1);
        int cz = MathHelper.clamp(z, 0, sampledGridSize - 1);
        int idx = ((cx * sampledGridSize + cy) * sampledGridSize + cz) * CHANNELS;
        if (idx < 0 || idx + 2 >= flowField.length) {
            return Vec3d.ZERO;
        }
        return new Vec3d(flowField[idx], flowField[idx + 1], flowField[idx + 2]);
    }

    private float pressureAtSample(int x, int y, int z) {
        int cx = MathHelper.clamp(x, 0, sampledGridSize - 1);
        int cy = MathHelper.clamp(y, 0, sampledGridSize - 1);
        int cz = MathHelper.clamp(z, 0, sampledGridSize - 1);
        int idx = ((cx * sampledGridSize + cy) * sampledGridSize + cz) * CHANNELS + 3;
        if (idx < 0 || idx >= flowField.length) {
            return 0.0f;
        }
        return flowField[idx];
    }

    private Vec3d lerp(Vec3d a, Vec3d b, float t) {
        return new Vec3d(
            MathHelper.lerp(t, a.x, b.x),
            MathHelper.lerp(t, a.y, b.y),
            MathHelper.lerp(t, a.z, b.z)
        );
    }

    private int sanitizeStride(int stride) {
        return (stride == 1 || stride == 2 || stride == 4 || stride == 8) ? stride : 4;
    }

    private int getViridisColor(float t) {
        t = MathHelper.clamp(t, 0.0f, 1.0f);
        float r;
        float g;
        float b;
        if (t < 0.25f) {
            float local = t / 0.25f;
            r = 0;
            g = local;
            b = 1;
        } else if (t < 0.5f) {
            float local = (t - 0.25f) / 0.25f;
            r = 0;
            g = 1;
            b = 1 - local;
        } else if (t < 0.75f) {
            float local = (t - 0.5f) / 0.25f;
            r = local;
            g = 1;
            b = 0;
        } else {
            float local = (t - 0.75f) / 0.25f;
            r = 1;
            g = 1 - local;
            b = 0;
        }
        return ((int) (r * 255) << 16) | ((int) (g * 255) << 8) | (int) (b * 255);
    }
}
