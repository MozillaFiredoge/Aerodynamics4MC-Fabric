package com.aerodynamics4mc.mixin.client;

import com.aerodynamics4mc.client.ParticleWindController;

import net.minecraft.client.multiplayer.ClientLevel;
import net.minecraft.client.particle.FlameParticle;
import net.minecraft.client.particle.Particle;
import net.minecraft.world.phys.Vec3;
import org.spongepowered.asm.mixin.Final;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.Shadow;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfo;

@Mixin(Particle.class)
abstract class ParticleMixin {
    @Shadow
    @Final
    protected ClientLevel level;

    @Shadow
    protected double xd;

    @Shadow
    protected double yd;

    @Shadow
    protected double zd;

    @Shadow
    protected double x;

    @Shadow
    protected double y;

    @Shadow
    protected double z;

    @Inject(method = "tick", at = @At("TAIL"))
    private void a4mc$applyBaseParticleWind(CallbackInfo ci) {
        if (!(((Object) this) instanceof FlameParticle)) {
            return;
        }
        Vec3 next = ParticleWindController.applyTorchFlame(
                this.level,
                this.x,
                this.y,
                this.z,
                new Vec3(this.xd, this.yd, this.zd)
        );
        this.xd = next.x;
        this.yd = next.y;
        this.zd = next.z;
    }
}
