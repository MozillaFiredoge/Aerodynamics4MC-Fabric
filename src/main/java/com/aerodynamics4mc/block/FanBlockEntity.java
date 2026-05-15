package com.aerodynamics4mc.block;

import net.minecraft.core.BlockPos;
import net.minecraft.world.level.block.entity.BlockEntity;
import net.minecraft.world.level.block.state.BlockState;

public class FanBlockEntity extends BlockEntity {
    public FanBlockEntity(BlockPos pos, BlockState state) {
        super(ModBlocks.FAN_BLOCK_ENTITY, pos, state);
    }
}
