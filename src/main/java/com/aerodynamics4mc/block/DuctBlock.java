package com.aerodynamics4mc.block;

import com.mojang.serialization.MapCodec;
import net.minecraft.world.level.block.Block;

public class DuctBlock extends Block {
    public static final MapCodec<DuctBlock> CODEC = simpleCodec(DuctBlock::new);

	public DuctBlock(Properties properties) {
		super(properties);
	}

	@Override
    protected MapCodec<? extends Block> codec() {
        return CODEC;
    }
}
