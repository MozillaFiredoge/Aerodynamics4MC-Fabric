package com.aerodynamics4mc.client;

import com.aerodynamics4mc.api.AeroWindSample;
import com.aerodynamics4mc.api.SamplePolicy;
import com.aerodynamics4mc.net.AeroClientL2PreferencePayload;
import com.aerodynamics4mc.net.AeroCoarseWindPayload;
import com.aerodynamics4mc.net.AeroFlowAnalysisPayload;
import com.aerodynamics4mc.net.AeroFlowPayload;
import com.aerodynamics4mc.net.AeroRuntimeStatePayload;
import net.fabricmc.fabric.api.client.networking.v1.ClientPlayNetworking;
import net.minecraft.client.multiplayer.ClientLevel;
import net.minecraft.core.BlockPos;
import net.minecraft.network.chat.Component;
import net.minecraft.world.level.block.state.BlockState;
import net.minecraft.world.phys.Vec3;

public final class AeroClientMod {

	private static AeroClientMod instance = null;
	private final AeroVisualizer visualizer = new AeroVisualizer();
	private final IrisWindBridge irisWindBridge = new IrisWindBridge(visualizer);
	private final ClientL2Solver clientL2Solver = new ClientL2Solver(visualizer);

	private AeroClientMod() {
		// private constructor
	}

	public static synchronized AeroClientMod getInstance() {
		if (instance == null)
			instance = new AeroClientMod();

		return instance;
	}

	public void onInitializeClient() {
		ClientPlayNetworking.registerGlobalReceiver(AeroRuntimeStatePayload.ID, this::onRuntimeState);
		ClientPlayNetworking.registerGlobalReceiver(AeroFlowPayload.ID, this::onFlowField);
		ClientPlayNetworking.registerGlobalReceiver(AeroCoarseWindPayload.ID, this::onCoarseWindField);
		ClientPlayNetworking.registerGlobalReceiver(AeroFlowAnalysisPayload.ID, this::onFlowAnalysis);

		visualizer.initialize();
		irisWindBridge.initialize();
		clientL2Solver.initialize();
	}

	// ==================== Getters ====================

	AeroVisualizer getVisualizer() {
		return visualizer;
	}

	ClientL2Solver getClientL2Solver() {
		return clientL2Solver;
	}

	Component renderStatusText() {
		return Component.literal(
				"Render vectors=" + visualizer.renderVelocityVectorsEnabled()
						+ " streamlines=" + visualizer.renderStreamlinesEnabled()
		);
	}

	void sendClientL2Preference(boolean enabled) {
		try {
			if (ClientPlayNetworking.canSend(AeroClientL2PreferencePayload.ID)) {
				ClientPlayNetworking.send(new AeroClientL2PreferencePayload(enabled));
			}
		} catch (IllegalStateException ignored) {
			// The client may be between play-networking sessions
		}
	}

	// ====================== Network Handlers ======================

	private void onRuntimeState(AeroRuntimeStatePayload payload, ClientPlayNetworking.Context context) {
		context.client().execute(() -> {
			visualizer.onRuntimeState(new AeroVisualizer.AeroFlowState(
					payload.streamingEnabled(),
					payload.renderVelocityVectors(),
					payload.renderStreamlines()
			));
			irisWindBridge.onRuntimeState(payload.streamingEnabled());
			clientL2Solver.onRuntimeState(payload.streamingEnabled());
			sendClientL2Preference(clientL2Solver.isExperimentalEnabled() && payload.streamingEnabled());
		});
	}

	private void onFlowField(AeroFlowPayload payload, ClientPlayNetworking.Context context) {
		context.client().execute(() -> {
			visualizer.onFlowField(payload);
			irisWindBridge.markDirty();
		});
	}

	private void onCoarseWindField(AeroCoarseWindPayload payload, ClientPlayNetworking.Context context) {
		context.client().execute(() -> {
			visualizer.onCoarseWindField(payload);
			clientL2Solver.onCoarseWindField(payload);
			irisWindBridge.markDirty();
		});
	}

	private void onFlowAnalysis(AeroFlowAnalysisPayload payload, ClientPlayNetworking.Context context) {
		context.client().execute(() -> visualizer.onFlowAnalysis(payload));
	}

	// ====================== Static API ======================

	public static AeroWindSample sampleFlow(ClientLevel world, Vec3 position) {
		AeroClientMod active = instance;
		SamplePolicy policy = active != null && active.clientL2Solver.isExperimentalEnabled()
				? SamplePolicy.CLIENT_LOCAL_PREFERRED
				: SamplePolicy.SERVER_COARSE_ONLY;
		return sampleFlow(world, position, policy);
	}

	public static AeroWindSample sampleFlow(ClientLevel world, Vec3 position, SamplePolicy policy) {
		AeroClientMod active = instance;
		if (active == null || world == null) {
			return AeroWindSample.ZERO;
		}
		return active.visualizer.sampleFlow(world.dimension().identifier(), position, policy);
	}

	public static Vec3 sampleWind(ClientLevel world, Vec3 position) {
		return sampleFlow(world, position).velocity();
	}

	public static void notifyBlockStateChanged(ClientLevel world, BlockPos pos, BlockState oldState, BlockState newState) {
		AeroClientMod active = instance;
		if (active == null) {
			return;
		}
		active.clientL2Solver.onBlockStateChanged(world, pos, oldState, newState);
	}
}
