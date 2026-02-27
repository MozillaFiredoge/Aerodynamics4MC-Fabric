package com.aerodynamics4mc.client;

import java.util.concurrent.ThreadLocalRandom;

import org.joml.Matrix4f;

import net.fabricmc.fabric.api.client.rendering.v1.world.WorldRenderContext;
import net.minecraft.client.MinecraftClient;
import net.minecraft.client.render.RenderLayers;
import net.minecraft.client.render.VertexConsumer;
import net.minecraft.client.util.math.MatrixStack;
import net.minecraft.particle.ParticleTypes;
import net.minecraft.util.math.BlockPos;
import net.minecraft.util.math.MathHelper;
import net.minecraft.util.math.Vec3d;

public class FlowRenderer {
    private final int gridSize;
    private final int channels;
    private final float maxInflowSpeed;
    private final float[] flowField; // flattened [x,y,z,4]
    private BlockPos origin;

    public FlowRenderer(int gridSize, float maxInflowSpeed) {
        this.gridSize = gridSize;
        this.channels = 4;
        this.maxInflowSpeed = Math.max(1e-6f, maxInflowSpeed);
        this.flowField = new float[gridSize * gridSize * gridSize * channels];
        this.origin = BlockPos.ORIGIN;
    }

    public void updateFlowField(BlockPos origin, float[] data) {
        if (data.length != flowField.length) {
            return;
        }
        this.origin = origin;
        System.arraycopy(data, 0, flowField, 0, data.length);
    }
    
    public void render(WorldRenderContext context) {
        if (origin == null) {
            return;
        }
        if (context == null || context.gameRenderer() == null) {
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
        int stride = 1;
        float velScale = 50.0f;
        var entry = matrices.peek();
        Matrix4f matrix = entry.getPositionMatrix();

        for (int x = 0; x < gridSize; x += stride) {
            for (int y = 0; y < gridSize; y += stride) {
                for (int z = 0; z < gridSize; z += stride) {
                    Vec3d vel = getVelocityAt(x, y, z);
                    float speed = (float) vel.length();
                    if (speed < 8e-4f) continue;

                    Vec3d dir = vel.normalize();
                    
                    int color = getViridisColor(speed * 40.0f);
                    int r = (color >> 16) & 0xFF;
                    int g = (color >> 8) & 0xFF;
                    int b = color & 0xFF;
                    int a = 255;

                    float fx = x + 0.5f;
                    float fy = y + 0.5f;
                    float fz = z + 0.5f;
                    float lineLength = MathHelper.clamp(speed * velScale, 0.05f, 0.9f);
                    
                    // Base of arrow
                    buffer.vertex(matrix, fx, fy, fz)
                        .color(r, g, b, a)
                        .normal(entry, (float) dir.x, (float) dir.y, (float) dir.z)
                        .lineWidth(1.5f);
                    // Tip of arrow
                    buffer.vertex(
                            matrix,
                            fx + (float) dir.x * lineLength,
                            fy + (float) dir.y * lineLength,
                            fz + (float) dir.z * lineLength
                        )
                        .color(r, g, b, a)
                        .normal(entry, (float) dir.x, (float) dir.y, (float) dir.z)
                        .lineWidth(2.0f);
                }
            }
        }
    }
    
    private void renderStreamlines(VertexConsumer buffer, MatrixStack matrices) {
        // Simple streamlines from inflow plane
        int seeds = 48;
        float stepSize = 0.35f;
        int maxSteps = 80;
        var entry = matrices.peek();
        Matrix4f matrix = entry.getPositionMatrix();
        
        for (int i = 0; i < seeds; i++) {
             // Random seed on X=0 plane or random in volume
             double sx = ThreadLocalRandom.current().nextDouble(0, gridSize/4.0);
             double sy = ThreadLocalRandom.current().nextDouble(0, gridSize);
             double sz = ThreadLocalRandom.current().nextDouble(0, gridSize);
             
             Vec3d pos = new Vec3d(sx, sy, sz);
             
             // Trace
             for (int s = 0; s < maxSteps; s++) {
                 Vec3d vel = sampleVelocity(pos);
                 float speed = (float) vel.length();
                 float speedNorm = MathHelper.clamp(speed / maxInflowSpeed, 0.0f, 1.0f);
                 if (speedNorm < 3e-4f) break;

                 Vec3d dir = vel.normalize();
                 float advectStep = stepSize * MathHelper.clamp(speedNorm * 12.0f, 0.15f, 1.2f);
                 Vec3d nextPos = pos.add(dir.multiply(advectStep));
                 Vec3d segDir = nextPos.subtract(pos).normalize();
                 
                 // Check bounds
                 if (nextPos.x < 0 || nextPos.x >= gridSize || nextPos.y < 0 || nextPos.y >= gridSize || nextPos.z < 0 || nextPos.z >= gridSize) {
                     break;
                 }
                 
                 int color = getViridisColor(speedNorm);
                 int r = (color >> 16) & 0xFF;
                 int g = (color >> 8) & 0xFF;
                 int b = color & 0xFF;
                 
                 buffer.vertex(matrix, (float)pos.x, (float)pos.y, (float)pos.z)
                     .color(r, g, b, 255)
                     .normal(entry, (float) segDir.x, (float) segDir.y, (float) segDir.z)
                     .lineWidth(1.0f);
                 buffer.vertex(matrix, (float)nextPos.x, (float)nextPos.y, (float)nextPos.z)
                     .color(r, g, b, 255)
                     .normal(entry, (float) segDir.x, (float) segDir.y, (float) segDir.z)
                     .lineWidth(1.0f);
                 
                 pos = nextPos;
             }
        }
    }

    // ... viridis ...
    private int getViridisColor(float t) {
        t = MathHelper.clamp(t, 0.0f, 1.0f);
        // Resurrecting the color logic
        float r, g, b;
        if (t < 0.25f) { float local = t / 0.25f; r = 0; g = local; b = 1; }
        else if (t < 0.5f) { float local = (t - 0.25f) / 0.25f; r = 0; g = 1; b = 1 - local; }
        else if (t < 0.75f) { float local = (t - 0.5f) / 0.25f; r = local; g = 1; b = 0; }
        else { float local = (t - 0.75f) / 0.25f; r = 1; g = 1 - local; b = 0; }
        return ((int)(r * 255) << 16) | ((int)(g * 255) << 8) | (int)(b * 255);
    }

    public void spawnSourceParticles(MinecraftClient client, int count) {
        if (client.world == null) {
            return;
        }
        for (int i = 0; i < count; i++) {
            double px = origin.getX() + 0.5 + ThreadLocalRandom.current().nextDouble(0.0, 2.0);
            double py = origin.getY() + ThreadLocalRandom.current().nextDouble(0.0, gridSize);
            double pz = origin.getZ() + ThreadLocalRandom.current().nextDouble(0.0, gridSize);
            Vec3d sampled = sampleVelocity(new Vec3d(px, py, pz));
            client.particleManager.addParticle(
                ParticleTypes.CLOUD,
                px,
                py,
                pz,
                sampled.x * 0.05,
                sampled.y * 0.05,
                sampled.z * 0.05
            );
        }
    }

    public void stepParticles(MinecraftClient client, double strength) {
        var player = client.player;
        if (player == null) {
            return;
        }
        Vec3d start = new Vec3d(player.getX(), player.getY(), player.getZ());
        Vec3d velocity = sampleVelocity(start);
        player.addVelocity(
            velocity.x * strength,
            velocity.y * strength,
            velocity.z * strength
        );
    }

    public void renderDebug(MinecraftClient client, float[] obstacleMask) {
        if (client.world == null) {
            return;
        }
        int cellCount = gridSize * gridSize * gridSize;
        if (obstacleMask == null || obstacleMask.length < cellCount) {
            return;
        }
        int maskStride = Math.max(1, obstacleMask.length / cellCount);
        int stride = 1;
        double velScale = 0.05;
        double pressureScale = 0.03;
        for (int x = 0; x < gridSize; x += stride) {
            for (int y = 0; y < gridSize; y += stride) {
                for (int z = 0; z < gridSize; z += stride) {
                    int idx = ((x * gridSize + y) * gridSize + z) * maskStride;
                    if (idx < 0 || idx >= obstacleMask.length) {
                        continue;
                    }
                    if (obstacleMask[idx] > 0.5f) {
                        continue;
                    }
                    double px = origin.getX() + x + 0.5;
                    double py = origin.getY() + y + 0.5;
                    double pz = origin.getZ() + z + 0.5;
                    Vec3d vel = getVelocityAt(x, y, z);
                    double vx = vel.x * velScale;
                    double vy = vel.y * velScale;
                    double vz = vel.z * velScale;
                    client.particleManager.addParticle(ParticleTypes.END_ROD, px, py, pz, vx, vy, vz);

                    float pressure = flowField[((x * gridSize + y) * gridSize + z) * channels + 3];
                    double pyVel = MathHelper.clamp(pressure * pressureScale, -0.1, 0.1);
                    client.particleManager.addParticle(ParticleTypes.CLOUD, px, py, pz, 0.0, pyVel, 0.0);
                }
            }
        }
    }

    public Vec3d sampleVelocity(Vec3d worldPos) {
        double localX = worldPos.x - origin.getX();
        double localY = worldPos.y - origin.getY();
        double localZ = worldPos.z - origin.getZ();

        if (localX < 0 || localY < 0 || localZ < 0 || localX >= gridSize - 1 || localY >= gridSize - 1 || localZ >= gridSize - 1) {
            return Vec3d.ZERO;
        }

        int x0 = (int) Math.floor(localX);
        int y0 = (int) Math.floor(localY);
        int z0 = (int) Math.floor(localZ);
        int x1 = x0 + 1;
        int y1 = y0 + 1;
        int z1 = z0 + 1;

        float fx = (float) (localX - x0);
        float fy = (float) (localY - y0);
        float fz = (float) (localZ - z0);

        Vec3d c000 = getVelocityAt(x0, y0, z0);
        Vec3d c100 = getVelocityAt(x1, y0, z0);
        Vec3d c010 = getVelocityAt(x0, y1, z0);
        Vec3d c110 = getVelocityAt(x1, y1, z0);
        Vec3d c001 = getVelocityAt(x0, y0, z1);
        Vec3d c101 = getVelocityAt(x1, y0, z1);
        Vec3d c011 = getVelocityAt(x0, y1, z1);
        Vec3d c111 = getVelocityAt(x1, y1, z1);

        Vec3d c00 = lerp(c000, c100, fx);
        Vec3d c10 = lerp(c010, c110, fx);
        Vec3d c01 = lerp(c001, c101, fx);
        Vec3d c11 = lerp(c011, c111, fx);

        Vec3d c0 = lerp(c00, c10, fy);
        Vec3d c1 = lerp(c01, c11, fy);

        return lerp(c0, c1, fz);
    }

    public float samplePressure(Vec3d worldPos) {
        double localX = worldPos.x - origin.getX();
        double localY = worldPos.y - origin.getY();
        double localZ = worldPos.z - origin.getZ();

        if (localX < 0 || localY < 0 || localZ < 0 || localX >= gridSize - 1 || localY >= gridSize - 1 || localZ >= gridSize - 1) {
            return 0.0f;
        }

        int x0 = (int) Math.floor(localX);
        int y0 = (int) Math.floor(localY);
        int z0 = (int) Math.floor(localZ);
        int x1 = x0 + 1;
        int y1 = y0 + 1;
        int z1 = z0 + 1;

        float fx = (float) (localX - x0);
        float fy = (float) (localY - y0);
        float fz = (float) (localZ - z0);

        float c000 = getPressureAt(x0, y0, z0);
        float c100 = getPressureAt(x1, y0, z0);
        float c010 = getPressureAt(x0, y1, z0);
        float c110 = getPressureAt(x1, y1, z0);
        float c001 = getPressureAt(x0, y0, z1);
        float c101 = getPressureAt(x1, y0, z1);
        float c011 = getPressureAt(x0, y1, z1);
        float c111 = getPressureAt(x1, y1, z1);

        float c00 = MathHelper.lerp(fx, c000, c100);
        float c10 = MathHelper.lerp(fx, c010, c110);
        float c01 = MathHelper.lerp(fx, c001, c101);
        float c11 = MathHelper.lerp(fx, c011, c111);

        float c0 = MathHelper.lerp(fy, c00, c10);
        float c1 = MathHelper.lerp(fy, c01, c11);

        return MathHelper.lerp(fz, c0, c1);
    }

    private Vec3d getVelocityAt(int x, int y, int z) {
        int idx = ((x * gridSize + y) * gridSize + z) * channels;
        float vx = flowField[idx];
        float vy = flowField[idx + 1];
        float vz = flowField[idx + 2];
        return new Vec3d(vx, vy, vz);
    }

    private float getPressureAt(int x, int y, int z) {
        int idx = ((x * gridSize + y) * gridSize + z) * channels;
        return flowField[idx + 3];
    }

    private Vec3d lerp(Vec3d a, Vec3d b, float t) {
        return new Vec3d(
            MathHelper.lerp(t, a.x, b.x),
            MathHelper.lerp(t, a.y, b.y),
            MathHelper.lerp(t, a.z, b.z)
        );
    }
}
