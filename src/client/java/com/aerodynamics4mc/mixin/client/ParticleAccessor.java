package com.aerodynamics4mc.mixin.client;

import net.minecraft.client.particle.Particle;
import net.minecraft.client.world.ClientWorld;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.gen.Accessor;

@Mixin(Particle.class)
public interface ParticleAccessor {
    @Accessor("world")
    ClientWorld a4mc$getWorld();

    @Accessor("x")
    double a4mc$getX();

    @Accessor("y")
    double a4mc$getY();

    @Accessor("z")
    double a4mc$getZ();

    @Accessor("velocityX")
    double a4mc$getVelocityX();

    @Accessor("velocityY")
    double a4mc$getVelocityY();

    @Accessor("velocityZ")
    double a4mc$getVelocityZ();

    @Accessor("velocityX")
    void a4mc$setVelocityX(double value);

    @Accessor("velocityY")
    void a4mc$setVelocityY(double value);

    @Accessor("velocityZ")
    void a4mc$setVelocityZ(double value);
}
