package com.aerodynamics4mc.mixin.client;

import com.aerodynamics4mc.client.ParticleWindController;

import net.minecraft.client.particle.CampfireSmokeParticle;
import net.minecraft.util.math.Vec3d;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfo;

@Mixin(CampfireSmokeParticle.class)
abstract class CampfireSmokeParticleMixin {
    @Inject(method = "tick", at = @At("TAIL"))
    private void a4mc$applyCampfireWind(CallbackInfo ci) {
        ParticleAccessor accessor = (ParticleAccessor) this;
        Vec3d next = ParticleWindController.applyCampfireSmoke(
            accessor.a4mc$getWorld(),
            accessor.a4mc$getX(),
            accessor.a4mc$getY(),
            accessor.a4mc$getZ(),
            new Vec3d(accessor.a4mc$getVelocityX(), accessor.a4mc$getVelocityY(), accessor.a4mc$getVelocityZ())
        );
        accessor.a4mc$setVelocityX(next.x);
        accessor.a4mc$setVelocityY(next.y);
        accessor.a4mc$setVelocityZ(next.z);
    }
}
