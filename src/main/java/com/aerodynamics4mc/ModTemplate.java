package com.aerodynamics4mc;

import com.aerodynamics4mc.block.ModBlocks;
import com.aerodynamics4mc.client.AeroClientMod;
import com.aerodynamics4mc.net.AeroNetworking;
import com.aerodynamics4mc.platform.Platform;

import com.aerodynamics4mc.runtime.AeroServerRuntime;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

//? fabric {
import com.aerodynamics4mc.platform.fabric.FabricPlatform;
//?} neoforge {
/*import com.aerodynamics4mc.platform.neoforge.NeoforgePlatform;
 *///?} forge {
/*import com.aerodynamics4mc.platform.forge.ForgePlatform;
 *///?}

@SuppressWarnings("LoggingSimilarMessage")
public class ModTemplate {

	public static final String MOD_ID = /*$ mod_id*/ "aerodynamics4mc";
	public static final String MOD_VERSION = /*$ mod_version*/ "0.1.0";
	public static final String MOD_FRIENDLY_NAME = /*$ mod_name*/ "Aerodynamics4MC";
	public static final Logger LOGGER = LoggerFactory.getLogger(MOD_ID);

	private static final Platform PLATFORM = createPlatformInstance();

	public static void onInitialize() {
		LOGGER.info("Initializing {} on {}", MOD_ID, ModTemplate.xplat().loader());
		LOGGER.debug("{}: { version: {}; friendly_name: {} }", MOD_ID, MOD_VERSION, MOD_FRIENDLY_NAME);
		ModBlocks.register();
		AeroNetworking.registerPayloadTypes();
		AeroServerRuntime.init();
	}

	public static void onInitializeClient() {
		LOGGER.info("Initializing {} Client on {}", MOD_ID, ModTemplate.xplat().loader());
		LOGGER.debug("{}: { version: {}; friendly_name: {} }", MOD_ID, MOD_VERSION, MOD_FRIENDLY_NAME);
		AeroClientMod.getInstance().onInitializeClient();
	}

	static Platform xplat() {
		return PLATFORM;
	}

	private static Platform createPlatformInstance() {
		//? fabric {
		return new FabricPlatform();
		//?} neoforge {
		/*return new NeoforgePlatform();
		 *///?} forge {
		/*return new ForgePlatform();
		 *///?}
	}
}
