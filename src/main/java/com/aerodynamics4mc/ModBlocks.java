package com.aerodynamics4mc;

import net.fabricmc.fabric.api.itemgroup.v1.ItemGroupEvents;
import net.minecraft.block.Block;
import net.minecraft.block.entity.BlockEntityType;
import net.minecraft.item.BlockItem;
import net.minecraft.item.Item;
import net.minecraft.item.ItemGroups;
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
    public static Item FAN_ITEM;
    public static BlockEntityType<FanBlockEntity> FAN_BLOCK_ENTITY;

    private ModBlocks() {
    }

    public static void register() {
        RegistryKey<Block> key = RegistryKey.of(RegistryKeys.BLOCK, FAN_ID);
        FAN_BLOCK = new FanBlock(Block.Settings.create().registryKey(key).strength(1.5f));
        Registry.register(Registries.BLOCK, FAN_ID, FAN_BLOCK);
        FAN_ITEM = Registry.register(
            Registries.ITEM,
            FAN_ID,
            new BlockItem(FAN_BLOCK, new Item.Settings().registryKey(RegistryKey.of(RegistryKeys.ITEM, FAN_ID)))
        );
        ItemGroupEvents.modifyEntriesEvent(ItemGroups.FUNCTIONAL).register(entries -> entries.add(FAN_ITEM));
        FAN_BLOCK_ENTITY = Registry.register(
            Registries.BLOCK_ENTITY_TYPE,
            FAN_ID,
            FabricBlockEntityTypeBuilder.create(FanBlockEntity::new, FAN_BLOCK).build()
        );
    }
}
