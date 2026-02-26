package com.aerodynamics4mc;

import net.minecraft.block.Block;
import net.minecraft.block.entity.BlockEntityType;
import net.minecraft.item.BlockItem;
import net.minecraft.item.Item;
import net.minecraft.registry.Registries;
import net.minecraft.registry.Registry;
import net.minecraft.registry.RegistryKey;
import net.minecraft.registry.RegistryKeys;
import net.minecraft.util.Identifier;
import net.fabricmc.fabric.api.object.builder.v1.block.entity.FabricBlockEntityTypeBuilder;

public final class ModBlocks {
    public static final String MOD_ID = "aerodynamics4mc";
    public static final Identifier FAN_ID = Identifier.of(MOD_ID, "fan");
    public static Block FAN_BLOCK;
    public static BlockEntityType<FanBlockEntity> FAN_BLOCK_ENTITY;

    private ModBlocks() {
    }

    public static void register() {
        RegistryKey<Block> key = RegistryKey.of(RegistryKeys.BLOCK, FAN_ID);
        FAN_BLOCK = new FanBlock(Block.Settings.create().registryKey(key).strength(1.5f));
        Registry.register(Registries.BLOCK, FAN_ID, FAN_BLOCK);
        Registry.register(Registries.ITEM, FAN_ID, new BlockItem(FAN_BLOCK, new Item.Settings().registryKey(RegistryKey.of(RegistryKeys.ITEM, FAN_ID))));
        FAN_BLOCK_ENTITY = Registry.register(
            Registries.BLOCK_ENTITY_TYPE,
            FAN_ID,
            FabricBlockEntityTypeBuilder.create(FanBlockEntity::new, FAN_BLOCK).build()
        );
    }
}
