package com.aerodynamics4mc.mixin.client;

import com.aerodynamics4mc.client.ParticleWindController;

import net.minecraft.client.particle.BaseAshSmokeParticle;
import net.minecraft.client.particle.SmokeParticle;
import net.minecraft.client.particle.WhiteSmokeParticle;
import net.minecraft.world.phys.Vec3;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfo;

@Mixin(BaseAshSmokeParticle.class)
abstract class AscendingParticleMixin {
    @Inject(method = "tick", at = @At("TAIL"))
    private void a4mc$applyAscendingWind(CallbackInfo ci) {
        Object self = this;
        if (!(self instanceof SmokeParticle) && !(self instanceof WhiteSmokeParticle)) {
            return;
        }
        ParticleAccessor accessor = (ParticleAccessor) this;
        Vec3 next = ParticleWindController.applyTorchSmoke(
                accessor.a4mc$getWorld(),
                accessor.a4mc$getX(),
                accessor.a4mc$getY(),
                accessor.a4mc$getZ(),
                new Vec3(accessor.a4mc$getVelocityX(), accessor.a4mc$getVelocityY(), accessor.a4mc$getVelocityZ())
        );
        accessor.a4mc$setVelocityX(next.x);
        accessor.a4mc$setVelocityY(next.y);
        accessor.a4mc$setVelocityZ(next.z);
    }
}
