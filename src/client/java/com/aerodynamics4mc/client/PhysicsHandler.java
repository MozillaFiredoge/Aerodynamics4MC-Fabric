package com.aerodynamics4mc.client;

import com.aerodynamics4mc.mixin.client.ParticleAccessor;
import com.aerodynamics4mc.mixin.client.ParticleManagerAccessor;

import net.minecraft.client.MinecraftClient;
import net.minecraft.client.particle.Particle;
import net.minecraft.client.particle.ParticleManager;
import net.minecraft.client.particle.ParticleRenderer;
import net.minecraft.entity.Entity;
import net.minecraft.entity.player.PlayerEntity;
import net.minecraft.server.MinecraftServer;
import net.minecraft.server.world.ServerWorld;
import net.minecraft.registry.RegistryKey;
import net.minecraft.util.math.BlockPos;
import net.minecraft.util.math.Box;
import net.minecraft.util.math.Vec3d;
import net.minecraft.world.World;

import java.util.ArrayList;
import java.util.List;
import java.util.UUID;

public class PhysicsHandler {
    private static final double NON_PLAYER_FORCE_MULTIPLIER = 5.0;
    private final FlowRenderer flowRenderer;
    private final int gridSize;
    private BlockPos origin;

    public PhysicsHandler(FlowRenderer flowRenderer, int gridSize) {
        this.flowRenderer = flowRenderer;
        this.gridSize = gridSize;
        this.origin = BlockPos.ORIGIN;
    }

    public void updateOrigin(BlockPos origin) {
        this.origin = origin;
    }

    public void applyForces(MinecraftClient client, double strength) {
        var world = client.world;
        if (world == null) {
            return;
        }
        Box box = new Box(
            origin.getX(),
            origin.getY(),
            origin.getZ(),
            origin.getX() + gridSize,
            origin.getY() + gridSize,
            origin.getZ() + gridSize
        );

        List<ForceEntry> remoteForces = new ArrayList<>();
        PlayerEntity localPlayer = client.player;
        for (Entity entity : world.getEntitiesByClass(Entity.class, box, e -> e.isAlive() && !e.isSpectator())) {
            Vec3d position = entity.getBoundingBox().getCenter();
            Vec3d velocity = flowRenderer.sampleVelocity(position);
            Vec3d delta = velocity.multiply(strength);
            if (delta.lengthSquared() < 1e-10) {
                continue;
            }
            if (localPlayer != null && entity.getId() == localPlayer.getId()) {
                // Player is client-controlled; apply immediately on client for responsiveness.
                entity.addVelocity(delta.x, delta.y, delta.z);
            } else {
                // Non-player entities are server-authoritative; apply client prediction and queue server-side force.
                Vec3d boosted = delta.multiply(NON_PLAYER_FORCE_MULTIPLIER);
                entity.addVelocity(boosted.x, boosted.y, boosted.z);
                remoteForces.add(new ForceEntry(entity.getUuid(), boosted));
            }
        }

        applyIntegratedServerForces(client, world.getRegistryKey(), remoteForces);
        applyParticleForces(client, box, strength);
    }

    private void applyIntegratedServerForces(MinecraftClient client, RegistryKey<World> worldKey, List<ForceEntry> forces) {
        if (forces.isEmpty()) {
            return;
        }
        MinecraftServer server = client.getServer();
        if (server == null) {
            // Dedicated multiplayer server cannot be driven from client-side code.
            return;
        }
        server.execute(() -> {
            ServerWorld serverWorld = server.getWorld(worldKey);
            if (serverWorld == null) {
                return;
            }
            for (ForceEntry entry : forces) {
                Entity entity = serverWorld.getEntity(entry.entityUuid());
                if (entity == null || !entity.isAlive()) {
                    continue;
                }
                Vec3d delta = entry.delta();
                entity.addVelocity(delta.x, delta.y, delta.z);
            }
        });
    }

    private void applyParticleForces(MinecraftClient client, Box box, double strength) {
        ParticleManager manager = client.particleManager;
        if (!(manager instanceof ParticleManagerAccessor accessor)) {
            return;
        }
        for (ParticleRenderer<?> renderer : accessor.getParticles().values()) {
            for (Particle particle : renderer.getParticles()) {
                if (particle == null) {
                    continue;
                }
                if (!particle.isAlive() || !(particle instanceof ParticleAccessor particleAccessor)) {
                    continue;
                }
                double px = particleAccessor.getX();
                double py = particleAccessor.getY();
                double pz = particleAccessor.getZ();
                if (!box.contains(px, py, pz)) {
                    continue;
                }
                Vec3d velocity = flowRenderer.sampleVelocity(new Vec3d(px, py, pz));
                particle.setVelocity(
                    particleAccessor.getVelocityX() + velocity.x * strength,
                    particleAccessor.getVelocityY() + velocity.y * strength,
                    particleAccessor.getVelocityZ() + velocity.z * strength
                );
            }
        }
    }

    private record ForceEntry(UUID entityUuid, Vec3d delta) {
    }
}
