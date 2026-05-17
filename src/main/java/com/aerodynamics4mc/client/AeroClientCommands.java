package com.aerodynamics4mc.client;

import com.mojang.brigadier.CommandDispatcher;
import net.minecraft.commands.CommandBuildContext;
import net.minecraft.commands.CommandSourceStack;
import net.minecraft.commands.Commands;
import net.minecraft.network.chat.Component;

public class AeroClientCommands {

	private AeroClientCommands() {
		// private constructor
	}

	public static void register(final CommandDispatcher<CommandSourceStack> dispatcher,
	                            final CommandBuildContext buildContext) {

		dispatcher.register(
				Commands.literal("aero_client_l2")
						.executes(ctx -> clientL2Status(ctx.getSource()))
						.then(Commands.literal("status")
								.executes(ctx -> clientL2Status(ctx.getSource())))
						.then(Commands.literal("on")
								.executes(ctx -> setClientL2Experimental(ctx.getSource(), true)))
						.then(Commands.literal("off")
								.executes(ctx -> setClientL2Experimental(ctx.getSource(), false)))
						.then(Commands.literal("stress")
								.executes(ctx -> clientL2StressStatus(ctx.getSource()))
								.then(Commands.literal("status")
										.executes(ctx -> clientL2StressStatus(ctx.getSource())))
								.then(Commands.literal("off")
										.executes(ctx -> setClientL2Stress(ctx.getSource(), "off")))
								.then(Commands.literal("fan")
										.executes(ctx -> setClientL2Stress(ctx.getSource(), "fan")))
								.then(Commands.literal("thermal")
										.executes(ctx -> setClientL2Stress(ctx.getSource(), "thermal")))
								.then(Commands.literal("dirty")
										.executes(ctx -> setClientL2Stress(ctx.getSource(), "dirty")))
								.then(Commands.literal("mixed")
										.executes(ctx -> setClientL2Stress(ctx.getSource(), "mixed"))))
		);

		dispatcher.register(
				Commands.literal("aero")
						.then(Commands.literal("render")
								.executes(ctx -> renderStatus(ctx.getSource()))
								.then(Commands.literal("vectors")
										.then(Commands.literal("on")
												.executes(ctx -> setRenderVelocityVectors(ctx.getSource(), true)))
										.then(Commands.literal("off")
												.executes(ctx -> setRenderVelocityVectors(ctx.getSource(), false))))
								.then(Commands.literal("streamlines")
										.then(Commands.literal("on")
												.executes(ctx -> setRenderStreamlines(ctx.getSource(), true)))
										.then(Commands.literal("off")
												.executes(ctx -> setRenderStreamlines(ctx.getSource(), false)))))
		);
	}

	// ==================== Command Handlers ====================

	private static int clientL2Status(CommandSourceStack source) {
		AeroClientMod mod = AeroClientMod.getInstance();
		source.sendSuccess(() -> Component.literal(mod.getClientL2Solver().status()), false);
		return 1;
	}

	private static int setClientL2Experimental(CommandSourceStack source, boolean enabled) {
		AeroClientMod mod = AeroClientMod.getInstance();
		mod.getClientL2Solver().setExperimentalEnabled(enabled);
		if (enabled) {
			mod.getVisualizer().clearRemoteFlowFields();
		}
		mod.sendClientL2Preference(enabled);
		source.sendSuccess(() -> Component.literal("Client L2 local solve " + (enabled ? "enabled" : "disabled")), false);
		return 1;
	}

	private static int clientL2StressStatus(CommandSourceStack source) {
		AeroClientMod mod = AeroClientMod.getInstance();
		source.sendSuccess(() -> Component.literal("Client L2 stress " + mod.getClientL2Solver().stressStatus()), false);
		return 1;
	}

	private static int setClientL2Stress(CommandSourceStack source, String mode) {
		AeroClientMod mod = AeroClientMod.getInstance();
		try {
			String message = mod.getClientL2Solver().setStressMode(mode);
			source.sendSuccess(() -> Component.literal(message), false);
			return 1;
		} catch (IllegalArgumentException e) {
			source.sendFailure(Component.literal(e.getMessage()));
			return 0;
		}
	}

	private static int renderStatus(CommandSourceStack source) {
		AeroClientMod mod = AeroClientMod.getInstance();
		source.sendSuccess(mod::renderStatusText, false);
		return 1;
	}

	private static int setRenderVelocityVectors(CommandSourceStack source, boolean enabled) {
		AeroClientMod mod = AeroClientMod.getInstance();
		mod.getVisualizer().setRenderVelocityVectors(enabled);
		source.sendSuccess(() -> Component.literal("Render vectors " + (enabled ? "enabled" : "disabled")), false);
		return 1;
	}

	private static int setRenderStreamlines(CommandSourceStack source, boolean enabled) {
		AeroClientMod mod = AeroClientMod.getInstance();
		mod.getVisualizer().setRenderStreamlines(enabled);
		source.sendSuccess(() -> Component.literal("Render streamlines " + (enabled ? "enabled" : "disabled")), false);
		return 1;
	}
}
