package com.aerodynamics4mc.mixin.client;

import com.aerodynamics4mc.client.AeroClientMod;
import net.minecraft.client.multiplayer.ClientLevel;
import net.minecraft.core.BlockPos;
import net.minecraft.world.level.Level;
import net.minecraft.world.level.block.state.BlockState;
import org.spongepowered.asm.mixin.Mixin;
import org.spongepowered.asm.mixin.Unique;
import org.spongepowered.asm.mixin.injection.At;
import org.spongepowered.asm.mixin.injection.Inject;
import org.spongepowered.asm.mixin.injection.callback.CallbackInfoReturnable;

import java.util.ArrayDeque;

@Mixin(Level.class)
abstract class ClientWorldBlockStateMixin {
    @Unique
    private static final ThreadLocal<ArrayDeque<ChangeContext>> A4MC_CLIENT_BLOCK_CHANGE_STACK =
            ThreadLocal.withInitial(ArrayDeque::new);

    @Inject(
            method = "setBlockAndUpdate(Lnet/minecraft/core/BlockPos;Lnet/minecraft/world/level/block/state/BlockState;)Z",
            at = @At("HEAD")
    )
    private void a4mc$captureClientOldState(
            BlockPos blockPos, BlockState blockState, CallbackInfoReturnable<Boolean> cir
    ) {
        if (!((Object) this instanceof ClientLevel world)) {
            return;
        }
        A4MC_CLIENT_BLOCK_CHANGE_STACK.get().push(new ChangeContext(blockPos.immutable(), world.getBlockState(blockPos)));
    }

    @Inject(
            method = "setBlockAndUpdate(Lnet/minecraft/core/BlockPos;Lnet/minecraft/world/level/block/state/BlockState;)Z",
            at = @At("RETURN")
    )
    private void a4mc$notifyClientBlockChange(
            BlockPos blockPos, BlockState blockState, CallbackInfoReturnable<Boolean> cir
    ) {
        ArrayDeque<ChangeContext> stack = A4MC_CLIENT_BLOCK_CHANGE_STACK.get();
        ChangeContext context = stack.isEmpty() ? null : stack.pop();
        if (stack.isEmpty()) {
            A4MC_CLIENT_BLOCK_CHANGE_STACK.remove();
        }
        if (context == null || !cir.getReturnValueZ()) {
            return;
        }
        if (!((Object) this instanceof ClientLevel world)) {
            return;
        }
        AeroClientMod.notifyBlockStateChanged(world, context.pos(), context.oldState(), world.getBlockState(context.pos()));
    }

    @Unique
    private record ChangeContext(BlockPos pos, BlockState oldState) {
    }
}
