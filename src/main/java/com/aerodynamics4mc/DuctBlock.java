package com.aerodynamics4mc;

import com.mojang.serialization.MapCodec;

import net.minecraft.block.Block;

public class DuctBlock extends Block {
    public static final MapCodec<DuctBlock> CODEC = createCodec(DuctBlock::new);

    public DuctBlock(Settings settings) {
        super(settings);
    }

    @Override
    protected MapCodec<? extends Block> getCodec() {
        return CODEC;
    }
}
