package com.aerodynamics4mc.mixin.client;

import net.minecraft.client.multiplayer.ClientLevel;
import net.minecraft.client.particle.Particle;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.gen.Accessor;

@Mixin(Particle.class)
public interface ParticleAccessor {
    @Accessor("level")
    ClientLevel a4mc$getWorld();

    @Accessor("x")
    double a4mc$getX();

    @Accessor("y")
    double a4mc$getY();

    @Accessor("z")
    double a4mc$getZ();

    @Accessor("xd")
    double a4mc$getVelocityX();

    @Accessor("yd")
    double a4mc$getVelocityY();

    @Accessor("zd")
    double a4mc$getVelocityZ();

    @Accessor("xd")
    void a4mc$setVelocityX(double value);

    @Accessor("yd")
    void a4mc$setVelocityY(double value);

    @Accessor("zd")
    void a4mc$setVelocityZ(double value);
}
