package com.aerodynamics4mc.mixin.client;

import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.gen.Accessor;

import net.minecraft.client.particle.Particle;

@Mixin(Particle.class)
public interface ParticleAccessor {
    @Accessor("x")
    double getX();

    @Accessor("y")
    double getY();

    @Accessor("z")
    double getZ();

    @Accessor("velocityX")
    double getVelocityX();

    @Accessor("velocityY")
    double getVelocityY();

    @Accessor("velocityZ")
    double getVelocityZ();
}
