package com.aerodynamics4mc.mixin.client;

import java.util.Map;

import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.gen.Accessor;

import net.minecraft.client.particle.ParticleManager;
import net.minecraft.client.particle.ParticleRenderer;
import net.minecraft.client.particle.ParticleTextureSheet;

@Mixin(ParticleManager.class)
public interface ParticleManagerAccessor {
    @Accessor("particles")
    Map<ParticleTextureSheet, ParticleRenderer<?>> getParticles();
}
