package com.aerodynamics4mc.client;

import com.aerodynamics4mc.api.AeroWindSample;
import com.aerodynamics4mc.api.SamplePolicy;
import com.aerodynamics4mc.net.AeroCoarseWindPayload;
import com.aerodynamics4mc.net.AeroFlowPayload;
import com.aerodynamics4mc.net.AeroFlowAnalysisPayload;
import com.aerodynamics4mc.net.AeroRuntimeStatePayload;

import net.fabricmc.api.ClientModInitializer;
import net.fabricmc.fabric.api.client.command.v2.ClientCommandManager;
import net.fabricmc.fabric.api.client.command.v2.ClientCommandRegistrationCallback;
import net.fabricmc.fabric.api.client.networking.v1.ClientPlayNetworking;
import net.minecraft.client.world.ClientWorld;
import net.minecraft.text.Text;
import net.minecraft.util.math.Vec3d;

public final class AeroClientMod implements ClientModInitializer {
    private static AeroClientMod instance;
    private final AeroVisualizer visualizer = new AeroVisualizer();
    private final IrisWindBridge irisWindBridge = new IrisWindBridge(visualizer);
    private final ClientL2Solver clientL2Solver = new ClientL2Solver(visualizer);

    @Override
    public void onInitializeClient() {
        instance = this;
        ClientPlayNetworking.registerGlobalReceiver(AeroRuntimeStatePayload.ID, this::onRuntimeState);
        ClientPlayNetworking.registerGlobalReceiver(AeroFlowPayload.ID, this::onFlowField);
        ClientPlayNetworking.registerGlobalReceiver(AeroCoarseWindPayload.ID, this::onCoarseWindField);
        ClientPlayNetworking.registerGlobalReceiver(AeroFlowAnalysisPayload.ID, this::onFlowAnalysis);
        visualizer.initialize();
        irisWindBridge.initialize();
        clientL2Solver.initialize();
        registerClientCommands();
    }

    public static AeroWindSample sampleFlow(ClientWorld world, Vec3d position) {
        return sampleFlow(world, position, SamplePolicy.SERVER_AGGREGATED_PREFERRED);
    }

    public static AeroWindSample sampleFlow(ClientWorld world, Vec3d position, SamplePolicy policy) {
        AeroClientMod active = instance;
        if (active == null || world == null) {
            return AeroWindSample.ZERO;
        }
        return active.visualizer.sampleFlow(world.getRegistryKey().getValue(), position, policy);
    }

    public static Vec3d sampleWind(ClientWorld world, Vec3d position) {
        return sampleFlow(world, position).velocity();
    }

    private void onRuntimeState(AeroRuntimeStatePayload payload, ClientPlayNetworking.Context context) {
        context.client().execute(() -> {
            visualizer.onRuntimeState(new AeroVisualizer.AeroFlowState(
                payload.streamingEnabled(),
                payload.renderVelocityVectors(),
                payload.renderStreamlines()
            ));
            irisWindBridge.onRuntimeState(payload.streamingEnabled());
            clientL2Solver.onRuntimeState(payload.streamingEnabled());
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

    private void registerClientCommands() {
        ClientCommandRegistrationCallback.EVENT.register((dispatcher, registryAccess) -> dispatcher.register(
            ClientCommandManager.literal("aero_client_l2")
                .executes(ctx -> clientL2Status(ctx.getSource()))
                .then(ClientCommandManager.literal("status")
                    .executes(ctx -> clientL2Status(ctx.getSource())))
                .then(ClientCommandManager.literal("on")
                    .executes(ctx -> setClientL2Experimental(ctx.getSource(), true)))
                .then(ClientCommandManager.literal("off")
                    .executes(ctx -> setClientL2Experimental(ctx.getSource(), false)))
        ));
    }

    private int clientL2Status(net.fabricmc.fabric.api.client.command.v2.FabricClientCommandSource source) {
        source.sendFeedback(Text.literal(clientL2Solver.status()));
        return 1;
    }

    private int setClientL2Experimental(
        net.fabricmc.fabric.api.client.command.v2.FabricClientCommandSource source,
        boolean enabled
    ) {
        clientL2Solver.setExperimentalEnabled(enabled);
        source.sendFeedback(Text.literal("Client L2 experimental " + (enabled ? "enabled" : "disabled")));
        return 1;
    }
}
