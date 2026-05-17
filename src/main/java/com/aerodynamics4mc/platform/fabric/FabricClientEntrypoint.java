package com.aerodynamics4mc.platform.fabric;

//? fabric {

import com.aerodynamics4mc.ModTemplate;
import com.aerodynamics4mc.client.AeroClientCommands;
import dev.kikugie.fletching_table.annotation.fabric.Entrypoint;
import net.fabricmc.api.ClientModInitializer;
import net.fabricmc.fabric.api.command.v2.CommandRegistrationCallback;

@Entrypoint("client")
public class FabricClientEntrypoint implements ClientModInitializer {

	@Override
	public void onInitializeClient() {
		ModTemplate.onInitializeClient();
		CommandRegistrationCallback.EVENT.register((dispatcher, registryAccess, environment) -> {
			if (!environment.includeDedicated) {
				AeroClientCommands.register(dispatcher, registryAccess);
			}
		});
	}

}
//?}
