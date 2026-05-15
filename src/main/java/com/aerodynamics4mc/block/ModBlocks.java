package com.aerodynamics4mc.block;

import com.aerodynamics4mc.ModTemplate;
import net.fabricmc.fabric.api.itemgroup.v1.ItemGroupEvents;
import net.fabricmc.fabric.api.object.builder.v1.block.entity.FabricBlockEntityTypeBuilder;
import net.minecraft.core.Registry;
import net.minecraft.core.registries.BuiltInRegistries;
import net.minecraft.core.registries.Registries;
import net.minecraft.resources.Identifier;
import net.minecraft.resources.ResourceKey;
import net.minecraft.world.item.BlockItem;
import net.minecraft.world.item.CreativeModeTabs;
import net.minecraft.world.item.Item;
import net.minecraft.world.level.block.Block;
import net.minecraft.world.level.block.entity.BlockEntity;
import net.minecraft.world.level.block.entity.BlockEntityType;
import net.minecraft.world.level.block.state.BlockBehaviour;

import java.util.function.Function;

public final class ModBlocks {
    public static final Identifier FAN_ID = Identifier.fromNamespaceAndPath(ModTemplate.MOD_ID, "fan");
    public static final Identifier DUCT_ID = Identifier.fromNamespaceAndPath(ModTemplate.MOD_ID, "duct");
    public static final Identifier WIND_METER_ID = Identifier.fromNamespaceAndPath(ModTemplate.MOD_ID, "wind_meter");
    public static final Identifier WIND_TURBINE_PROBE_ID = Identifier.fromNamespaceAndPath(ModTemplate.MOD_ID, "wind_turbine_probe");
    public static Block FAN_BLOCK = register(FAN_ID.getPath(), FanBlock::new, Block.Properties.of().strength(1.5f), true);
    public static Block DUCT_BLOCK = register(DUCT_ID.getPath(), DuctBlock::new, Block.Properties.of().strength(1.0f), true);
    public static Item WIND_METER_ITEM = register(WIND_METER_ID.getPath(), WindMeterItem::new, new Item.Properties().stacksTo(1));
    public static Block WIND_TURBINE_PROBE_BLOCK = register(WIND_TURBINE_PROBE_ID.getPath(), WindTurbineProbeBlock::new, Block.Properties.of().strength(1.5f), true);
    public static BlockEntityType<FanBlockEntity> FAN_BLOCK_ENTITY = register(FAN_ID.getPath(), FanBlockEntity::new, ModBlocks.FAN_BLOCK);
    public static BlockEntityType<WindTurbineProbeBlockEntity> WIND_TURBINE_PROBE_BLOCK_ENTITY = register(WIND_TURBINE_PROBE_ID.getPath(), WindTurbineProbeBlockEntity::new, ModBlocks.WIND_TURBINE_PROBE_BLOCK);

    private ModBlocks() {
    }

    public static void register() {
        ItemGroupEvents.modifyEntriesEvent(CreativeModeTabs.TOOLS_AND_UTILITIES).register(entries -> {
            entries.prepend(FAN_BLOCK);
            entries.prepend(DUCT_BLOCK);
            entries.prepend(WIND_METER_ITEM);
            entries.prepend(WIND_TURBINE_PROBE_BLOCK);
        });
    }

    public static <T extends Item> T register(String name, Function<Item.Properties, T> itemFactory, Item.Properties settings) {
        ResourceKey<Item> itemKey = keyOfItem(name);

        T item = itemFactory.apply(settings.setId(itemKey));

        Registry.register(BuiltInRegistries.ITEM, itemKey, item);

        return item;
    }

    private static Block register(String name, Function<BlockBehaviour.Properties, Block> blockFactory, BlockBehaviour.Properties settings, boolean shouldRegisterItem) {
        ResourceKey<Block> blockKey = keyOfBlock(name);
        Block block = blockFactory.apply(settings.setId(blockKey));

        if (shouldRegisterItem) {
            ResourceKey<Item> itemKey = keyOfItem(name);

            BlockItem blockItem = new BlockItem(block, new Item.Properties().setId(itemKey).useBlockDescriptionPrefix());
            Registry.register(BuiltInRegistries.ITEM, itemKey, blockItem);
        }

        return Registry.register(BuiltInRegistries.BLOCK, blockKey, block);
    }

    private static ResourceKey<Block> keyOfBlock(String name) {
        return ResourceKey.create(Registries.BLOCK, Identifier.fromNamespaceAndPath(ModTemplate.MOD_ID, name));
    }

    private static ResourceKey<Item> keyOfItem(String name) {
        return ResourceKey.create(Registries.ITEM, Identifier.fromNamespaceAndPath(ModTemplate.MOD_ID, name));
    }

    private static <T extends BlockEntity> BlockEntityType<T> register(
            String name,
            FabricBlockEntityTypeBuilder.Factory<? extends T> entityFactory,
            Block... blocks
    ) {
        Identifier id = Identifier.fromNamespaceAndPath(ModTemplate.MOD_ID, name);
        return Registry.register(BuiltInRegistries.BLOCK_ENTITY_TYPE, id, FabricBlockEntityTypeBuilder.<T>create(entityFactory, blocks).build());
    }
}
