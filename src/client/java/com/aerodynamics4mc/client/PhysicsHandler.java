package com.aerodynamics4mc.client;

import com.aerodynamics4mc.mixin.client.ParticleAccessor;
import com.aerodynamics4mc.mixin.client.ParticleManagerAccessor;

import net.minecraft.block.BlockState;
import net.minecraft.client.MinecraftClient;
import net.minecraft.client.particle.Particle;
import net.minecraft.client.particle.ParticleManager;
import net.minecraft.client.particle.ParticleRenderer;
import net.minecraft.client.sound.MovingSoundInstance;
import net.minecraft.client.sound.SoundInstance;
import net.minecraft.client.sound.SoundSystem;
import net.minecraft.entity.Entity;
import net.minecraft.entity.player.PlayerEntity;
import net.minecraft.particle.ParticleTypes;
import net.minecraft.registry.RegistryKey;
import net.minecraft.server.MinecraftServer;
import net.minecraft.server.world.ServerWorld;
import net.minecraft.sound.SoundCategory;
import net.minecraft.sound.SoundEvents;
import net.minecraft.util.math.BlockPos;
import net.minecraft.util.math.Box;
import net.minecraft.util.math.Direction;
import net.minecraft.util.math.MathHelper;
import net.minecraft.util.math.Vec3d;
import net.minecraft.world.World;

import java.util.ArrayList;
import java.util.List;
import java.util.UUID;

public class PhysicsHandler {
    private static final double NON_PLAYER_FORCE_MULTIPLIER = 1.0;
    private static final double PARTICLE_FORCE_MULTIPLIER = 1.0;
    private static final float AMBIENT_MIN_SPEED = 0.03f;
    private static final float STREAK_MIN_SPEED = 0.18f;
    private static final float THERMAL_MIN_UPDRAFT = 0.06f;
    private static final int AMBIENT_PARTICLE_INTERVAL = 2;
    private static final int THERMAL_PARTICLE_INTERVAL = 3;
    private static final int SURFACE_SAMPLE_HORIZONTAL_RADIUS = 16;
    private static final int SURFACE_SAMPLE_VERTICAL_RADIUS = 10;
    private static final int AMBIENT_SURFACE_SAMPLE_ATTEMPTS = 18;
    private static final int THERMAL_SURFACE_SAMPLE_ATTEMPTS = 24;
    private static final float SURFACE_OUTSET = 0.55f;
    private static final float AUDIO_START_SPEED = 0.05f;
    private static final float AUDIO_STOP_SPEED = 0.03f;
    private static final float AUDIO_FULL_SPEED = 0.30f;
    private static final float AUDIO_MAX_VOLUME = 0.26f;
    private static final float AUDIO_BOOTSTRAP_VOLUME = 0.05f;
    private static final long AUDIO_STALE_TICKS = 5L;

    private final FlowRenderer flowRenderer;
    private final int gridSize;
    private BlockPos origin;
    private WindLoopSound windLoopSound;
    private float smoothedAudioStrength = 0.0f;

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
        Box box = windBox();

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
                entity.addVelocity(delta.x, delta.y, delta.z);
            } else {
                Vec3d boosted = delta.multiply(NON_PLAYER_FORCE_MULTIPLIER);
                entity.addVelocity(boosted.x, boosted.y, boosted.z);
                remoteForces.add(new ForceEntry(entity.getUuid(), boosted));
            }
        }

        applyIntegratedServerForces(client, world.getRegistryKey(), remoteForces);
        applyParticleForces(client, box, strength);
    }

    public void applyParticleForcesOnly(MinecraftClient client, double strength) {
        if (client == null || client.world == null) {
            return;
        }
        applyParticleForces(client, windBox(), strength);
    }

    public void tickVisualFeedback(MinecraftClient client, long tickCounter) {
        if (client == null || client.world == null || client.player == null) {
            stopWindAudio();
            return;
        }
        spawnAmbientWindParticles(client, tickCounter);
        spawnThermalShimmer(client, tickCounter);
        tickWindAudio(client, tickCounter);
    }

    private void applyIntegratedServerForces(MinecraftClient client, RegistryKey<World> worldKey, List<ForceEntry> forces) {
        if (forces.isEmpty()) {
            return;
        }
        MinecraftServer server = client.getServer();
        if (server == null) {
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

    private Box windBox() {
        return new Box(
            origin.getX(),
            origin.getY(),
            origin.getZ(),
            origin.getX() + gridSize,
            origin.getY() + gridSize,
            origin.getZ() + gridSize
        );
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
                if (velocity.lengthSquared() < 1e-10) {
                    continue;
                }
                particle.setVelocity(
                    particleAccessor.getVelocityX() + velocity.x * strength * PARTICLE_FORCE_MULTIPLIER,
                    particleAccessor.getVelocityY() + velocity.y * strength * PARTICLE_FORCE_MULTIPLIER,
                    particleAccessor.getVelocityZ() + velocity.z * strength * PARTICLE_FORCE_MULTIPLIER
                );
            }
        }
    }

    private void spawnAmbientWindParticles(MinecraftClient client, long tickCounter) {
        if (tickCounter % AMBIENT_PARTICLE_INTERVAL != 0L) {
            return;
        }
        PlayerEntity player = client.player;
        if (player == null) {
            return;
        }
        for (int i = 0; i < 3; i++) {
            SurfaceSample sample = sampleNearbySurface(client, player.getBlockPos(), false, AMBIENT_SURFACE_SAMPLE_ATTEMPTS);
            if (sample == null) {
                continue;
            }
            Vec3d pos = jitterSurfaceSpawn(client, sample);
            float speed = (float) sample.velocity().length();
            if (speed >= STREAK_MIN_SPEED) {
                spawnStreakParticle(client, pos, sample.velocity());
            } else {
                spawnDustParticle(client, pos, sample.velocity());
            }
        }
    }

    private void spawnDustParticle(MinecraftClient client, Vec3d pos, Vec3d velocity) {
        client.world.addParticleClient(
            ParticleTypes.WHITE_ASH,
            pos.x,
            pos.y,
            pos.z,
            velocity.x * 0.10 + randomCentered(client) * 0.01,
            velocity.y * 0.08 + randomCentered(client) * 0.006,
            velocity.z * 0.10 + randomCentered(client) * 0.01
        );
    }

    private void spawnStreakParticle(MinecraftClient client, Vec3d pos, Vec3d velocity) {
        client.world.addParticleClient(
            ParticleTypes.SMALL_GUST,
            pos.x,
            pos.y,
            pos.z,
            velocity.x * 0.16,
            velocity.y * 0.12,
            velocity.z * 0.16
        );
        client.world.addParticleClient(
            ParticleTypes.CLOUD,
            pos.x - velocity.x * 0.08,
            pos.y - velocity.y * 0.08,
            pos.z - velocity.z * 0.08,
            velocity.x * 0.08,
            velocity.y * 0.06,
            velocity.z * 0.08
        );
    }

    private void spawnThermalShimmer(MinecraftClient client, long tickCounter) {
        if (tickCounter % THERMAL_PARTICLE_INTERVAL != 0L) {
            return;
        }
        PlayerEntity player = client.player;
        if (player == null) {
            return;
        }
        for (int i = 0; i < 3; i++) {
            SurfaceSample sample = sampleNearbySurface(client, player.getBlockPos(), true, THERMAL_SURFACE_SAMPLE_ATTEMPTS);
            if (sample == null) {
                continue;
            }
            Vec3d pos = jitterSurfaceSpawn(client, sample);
            Vec3d velocity = sample.velocity();
            client.world.addParticleClient(
                velocity.y > THERMAL_MIN_UPDRAFT * 2.0f ? ParticleTypes.DUST_PLUME : ParticleTypes.WHITE_SMOKE,
                pos.x,
                pos.y,
                pos.z,
                velocity.x * 0.05,
                Math.max(0.02, velocity.y * 0.10 - sample.pressure() * 0.04),
                velocity.z * 0.05
            );
        }
    }

    private void tickWindAudio(MinecraftClient client, long tickCounter) {
        PlayerEntity player = client.player;
        if (player == null || client.world == null) {
            stopWindAudio();
            return;
        }
        float speed = sampleAudioWindSpeed(player);
        float rawNormalized = 0.0f;
        if (speed >= AUDIO_STOP_SPEED) {
            rawNormalized = MathHelper.clamp((speed - AUDIO_START_SPEED) / (AUDIO_FULL_SPEED - AUDIO_START_SPEED), 0.0f, 1.0f);
        }
        float smoothing = rawNormalized > smoothedAudioStrength ? 0.35f : 0.10f;
        smoothedAudioStrength = MathHelper.lerp(smoothing, smoothedAudioStrength, rawNormalized);
        if (smoothedAudioStrength < 0.01f) {
            smoothedAudioStrength = 0.0f;
        }
        boolean startedThisTick = false;
        if (windLoopSound == null || windLoopSound.isDone()) {
            if (smoothedAudioStrength <= 0.0f) {
                return;
            }
            windLoopSound = new WindLoopSound(client);
            windLoopSound.refresh(player, smoothedAudioStrength);
            windLoopSound.primeForPlayback();
            SoundSystem.PlayResult result = client.getSoundManager().play(windLoopSound);
            if (result == SoundSystem.PlayResult.NOT_STARTED) {
                windLoopSound = null;
                return;
            }
            startedThisTick = true;
        }
        if (windLoopSound == null) {
            return;
        }
        windLoopSound.refresh(player, smoothedAudioStrength);
        if (!startedThisTick && !client.getSoundManager().isPlaying(windLoopSound) && !windLoopSound.isDone()) {
            windLoopSound.primeForPlayback();
            SoundSystem.PlayResult result = client.getSoundManager().play(windLoopSound);
            if (result == SoundSystem.PlayResult.NOT_STARTED) {
                windLoopSound = null;
            }
        }
    }

    private void stopWindAudio() {
        smoothedAudioStrength = 0.0f;
        if (windLoopSound == null) {
            return;
        }
        windLoopSound.stop();
        windLoopSound = null;
    }

    private float sampleAudioWindSpeed(PlayerEntity player) {
        Vec3d center = player.getBoundingBox().getCenter();
        Vec3d[] samplePoints = {
            center,
            center.add(0.75, 0.0, 0.0),
            center.add(-0.75, 0.0, 0.0),
            center.add(0.0, 0.0, 0.75),
            center.add(0.0, 0.0, -0.75),
            center.add(0.0, 0.8, 0.0)
        };
        float maxSpeed = 0.0f;
        for (Vec3d point : samplePoints) {
            if (!windBox().contains(point)) {
                continue;
            }
            maxSpeed = Math.max(maxSpeed, (float) flowRenderer.sampleVelocity(point).length());
        }
        return maxSpeed;
    }

    private SurfaceSample sampleNearbySurface(MinecraftClient client, BlockPos center, boolean thermalOnly, int attempts) {
        if (client.world == null) {
            return null;
        }
        Box box = windBox();
        SurfaceSample best = null;
        double bestScore = thermalOnly ? THERMAL_MIN_UPDRAFT : AMBIENT_MIN_SPEED;
        for (int attempt = 0; attempt < attempts; attempt++) {
            BlockPos surfacePos = center.add(
                randomInt(client, -SURFACE_SAMPLE_HORIZONTAL_RADIUS, SURFACE_SAMPLE_HORIZONTAL_RADIUS),
                randomInt(client, -SURFACE_SAMPLE_VERTICAL_RADIUS, SURFACE_SAMPLE_VERTICAL_RADIUS),
                randomInt(client, -SURFACE_SAMPLE_HORIZONTAL_RADIUS, SURFACE_SAMPLE_HORIZONTAL_RADIUS)
            );
            if (!box.contains(Vec3d.ofCenter(surfacePos))) {
                continue;
            }
            BlockState state = client.world.getBlockState(surfacePos);
            if (!isRenderableSurface(client, surfacePos, state)) {
                continue;
            }
            SurfaceSample sample = sampleBestExposedFace(client, surfacePos, thermalOnly, box);
            if (sample == null) {
                continue;
            }
            double score = scoreSurfaceSample(sample, thermalOnly);
            if (score > bestScore) {
                bestScore = score;
                best = sample;
            }
        }
        return best;
    }

    private SurfaceSample sampleBestExposedFace(MinecraftClient client, BlockPos surfacePos, boolean thermalOnly, Box box) {
        if (client.world == null) {
            return null;
        }
        SurfaceSample best = null;
        double bestScore = thermalOnly ? THERMAL_MIN_UPDRAFT : AMBIENT_MIN_SPEED;
        for (Direction face : Direction.values()) {
            if (face == Direction.DOWN) {
                continue;
            }
            BlockPos airPos = surfacePos.offset(face);
            if (!client.world.getBlockState(airPos).isAir()) {
                continue;
            }
            Vec3d spawnPos = Vec3d.ofCenter(surfacePos).add(
                face.getOffsetX() * SURFACE_OUTSET,
                face.getOffsetY() * SURFACE_OUTSET,
                face.getOffsetZ() * SURFACE_OUTSET
            );
            if (!box.contains(spawnPos)) {
                continue;
            }
            Vec3d velocity = flowRenderer.sampleVelocity(spawnPos);
            if (thermalOnly) {
                if (velocity.y < THERMAL_MIN_UPDRAFT || velocity.y < (Math.abs(velocity.x) + Math.abs(velocity.z)) * 0.70) {
                    continue;
                }
            } else if (velocity.lengthSquared() < AMBIENT_MIN_SPEED * AMBIENT_MIN_SPEED) {
                continue;
            }
            SurfaceSample sample = new SurfaceSample(spawnPos, velocity, flowRenderer.samplePressure(spawnPos), face);
            double score = scoreSurfaceSample(sample, thermalOnly);
            if (score > bestScore) {
                bestScore = score;
                best = sample;
            }
        }
        return best;
    }

    private double scoreSurfaceSample(SurfaceSample sample, boolean thermalOnly) {
        Vec3d velocity = sample.velocity();
        if (thermalOnly) {
            double lateral = Math.abs(velocity.x) + Math.abs(velocity.z);
            return velocity.y - lateral * 0.40 + (sample.face() == Direction.UP ? 0.04 : 0.0);
        }
        Vec3d normal = Vec3d.of(sample.face().getVector());
        double outward = Math.max(0.0, velocity.dotProduct(normal));
        return velocity.length() + outward * 0.25 + (sample.face() == Direction.UP ? 0.02 : 0.0);
    }

    private boolean isRenderableSurface(MinecraftClient client, BlockPos pos, BlockState state) {
        return client.world != null
            && !state.isAir()
            && !state.getCollisionShape(client.world, pos).isEmpty();
    }

    private Vec3d jitterSurfaceSpawn(MinecraftClient client, SurfaceSample sample) {
        Vec3d pos = sample.spawnPos();
        return switch (sample.face().getAxis()) {
            case X -> pos.add(0.0, randomCentered(client) * 0.36, randomCentered(client) * 0.36);
            case Y -> pos.add(randomCentered(client) * 0.36, 0.0, randomCentered(client) * 0.36);
            case Z -> pos.add(randomCentered(client) * 0.36, randomCentered(client) * 0.36, 0.0);
        };
    }

    private int randomInt(MinecraftClient client, int minInclusive, int maxInclusive) {
        return minInclusive + client.world.random.nextInt(maxInclusive - minInclusive + 1);
    }

    private double audioVolumeFor(float normalized) {
        if (normalized <= 0.0f) {
            return 0.0f;
        }
        return 0.05f + normalized * AUDIO_MAX_VOLUME;
    }

    private float audioPitchFor(float normalized) {
        return 0.82f + normalized * 0.28f;
    }

    private double randomCentered(MinecraftClient client) {
        return client.world == null ? 0.0 : client.world.random.nextDouble() - 0.5;
    }

    private record ForceEntry(UUID entityUuid, Vec3d delta) {
    }

    private record SurfaceSample(Vec3d spawnPos, Vec3d velocity, float pressure, Direction face) {
    }

    private final class WindLoopSound extends MovingSoundInstance {
        private final MinecraftClient client;
        private float targetVolume = 0.0f;
        private float targetPitch = 0.82f;
        private long lastRefreshWorldTime = Long.MIN_VALUE;

        private WindLoopSound(MinecraftClient client) {
            super(SoundEvents.ITEM_ELYTRA_FLYING, SoundCategory.AMBIENT, SoundInstance.createRandom());
            this.client = client;
            this.repeat = true;
            this.repeatDelay = 0;
            this.relative = true;
            this.attenuationType = SoundInstance.AttenuationType.NONE;
            this.volume = 0.0f;
            this.pitch = targetPitch;
        }

        private void refresh(PlayerEntity player, float normalized) {
            this.x = player.getX();
            this.y = player.getEyeY();
            this.z = player.getZ();
            this.targetVolume = (float) audioVolumeFor(normalized);
            this.targetPitch = audioPitchFor(normalized);
            if (client.world != null) {
                this.lastRefreshWorldTime = client.world.getTime();
            }
        }

        private void stop() {
            this.targetVolume = 0.0f;
            this.volume = 0.0f;
            setDone();
        }

        private void primeForPlayback() {
            this.volume = Math.max(this.volume, Math.max(AUDIO_BOOTSTRAP_VOLUME, this.targetVolume));
            this.pitch = this.targetPitch;
        }

        @Override
        public void tick() {
            if (client.player == null || client.world == null) {
                stop();
                return;
            }
            this.x = client.player.getX();
            this.y = client.player.getEyeY();
            this.z = client.player.getZ();
            if (client.world.getTime() - lastRefreshWorldTime > AUDIO_STALE_TICKS) {
                targetVolume = 0.0f;
            }
            volume = MathHelper.lerp(0.22f, volume, targetVolume);
            pitch = MathHelper.lerp(0.20f, pitch, targetPitch);
            if (targetVolume <= 0.0f && volume < 0.005f) {
                stop();
                windLoopSound = null;
            }
        }
    }
}
