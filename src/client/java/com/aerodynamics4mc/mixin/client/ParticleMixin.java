package com.aerodynamics4mc.mixin.client;

import com.aerodynamics4mc.client.ParticleWindController;

import net.minecraft.client.particle.FlameParticle;
import net.minecraft.client.particle.Particle;
import net.minecraft.client.world.ClientWorld;
import net.minecraft.util.math.Vec3d;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.Shadow;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfo;

@Mixin(Particle.class)
abstract class ParticleMixin {
    @Shadow protected ClientWorld world;
    @Shadow protected double x;
    @Shadow protected double y;
    @Shadow protected double z;
    @Shadow protected double velocityX;
    @Shadow protected double velocityY;
    @Shadow protected double velocityZ;

    @Inject(method = "tick", at = @At("TAIL"))
    private void a4mc$applyBaseParticleWind(CallbackInfo ci) {
        if (!(((Object) this) instanceof FlameParticle)) {
            return;
        }
        Vec3d next = ParticleWindController.applyTorchFlame(
            this.world,
            this.x,
            this.y,
            this.z,
            new Vec3d(this.velocityX, this.velocityY, this.velocityZ)
        );
        this.velocityX = next.x;
        this.velocityY = next.y;
        this.velocityZ = next.z;
    }
}
