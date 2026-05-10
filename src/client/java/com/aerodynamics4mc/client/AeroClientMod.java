package com.aerodynamics4mc.client;

import com.aerodynamics4mc.api.AeroWindSample;
import com.aerodynamics4mc.api.SamplePolicy;
import com.aerodynamics4mc.net.AeroClientL2PreferencePayload;
import com.aerodynamics4mc.net.AeroCoarseWindPayload;
import com.aerodynamics4mc.net.AeroFlowPayload;
import com.aerodynamics4mc.net.AeroFlowAnalysisPayload;
import com.aerodynamics4mc.net.AeroRuntimeStatePayload;

import net.fabricmc.api.ClientModInitializer;
import net.fabricmc.fabric.api.client.command.v2.ClientCommandManager;
import net.fabricmc.fabric.api.client.command.v2.ClientCommandRegistrationCallback;
import net.fabricmc.fabric.api.client.message.v1.ClientSendMessageEvents;
import net.fabricmc.fabric.api.client.networking.v1.ClientPlayNetworking;
import net.minecraft.block.BlockState;
import net.minecraft.client.MinecraftClient;
import net.minecraft.client.world.ClientWorld;
import net.minecraft.text.Text;
import net.minecraft.util.math.BlockPos;
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
        registerClientCommandInterceptor();
    }

    public static AeroWindSample sampleFlow(ClientWorld world, Vec3d position) {
        AeroClientMod active = instance;
        SamplePolicy policy = active != null && active.clientL2Solver.isExperimentalEnabled()
            ? SamplePolicy.CLIENT_LOCAL_PREFERRED
            : SamplePolicy.SERVER_AGGREGATED_PREFERRED;
        return sampleFlow(world, position, policy);
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

    public static void notifyBlockStateChanged(ClientWorld world, BlockPos pos, BlockState oldState, BlockState newState) {
        AeroClientMod active = instance;
        if (active == null) {
            return;
        }
        active.clientL2Solver.onBlockStateChanged(world, pos, oldState, newState);
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

    private void registerClientCommands() {
        ClientCommandRegistrationCallback.EVENT.register((dispatcher, registryAccess) -> {
            dispatcher.register(
                ClientCommandManager.literal("aero_client_l2")
                    .executes(ctx -> clientL2Status(ctx.getSource()))
                    .then(ClientCommandManager.literal("status")
                        .executes(ctx -> clientL2Status(ctx.getSource())))
                    .then(ClientCommandManager.literal("on")
                        .executes(ctx -> setClientL2Experimental(ctx.getSource(), true)))
                    .then(ClientCommandManager.literal("off")
                        .executes(ctx -> setClientL2Experimental(ctx.getSource(), false)))
                    .then(ClientCommandManager.literal("stress")
                        .executes(ctx -> clientL2StressStatus(ctx.getSource()))
                        .then(ClientCommandManager.literal("status")
                            .executes(ctx -> clientL2StressStatus(ctx.getSource())))
                        .then(ClientCommandManager.literal("off")
                            .executes(ctx -> setClientL2Stress(ctx.getSource(), "off")))
                        .then(ClientCommandManager.literal("fan")
                            .executes(ctx -> setClientL2Stress(ctx.getSource(), "fan")))
                        .then(ClientCommandManager.literal("thermal")
                            .executes(ctx -> setClientL2Stress(ctx.getSource(), "thermal")))
                        .then(ClientCommandManager.literal("dirty")
                            .executes(ctx -> setClientL2Stress(ctx.getSource(), "dirty")))
                        .then(ClientCommandManager.literal("mixed")
                            .executes(ctx -> setClientL2Stress(ctx.getSource(), "mixed"))))
            );
            dispatcher.register(
                ClientCommandManager.literal("aero")
                    .then(ClientCommandManager.literal("render")
                        .executes(ctx -> renderStatus(ctx.getSource()))
                        .then(ClientCommandManager.literal("vectors")
                            .then(ClientCommandManager.literal("on")
                                .executes(ctx -> setRenderVelocityVectors(ctx.getSource(), true)))
                            .then(ClientCommandManager.literal("off")
                                .executes(ctx -> setRenderVelocityVectors(ctx.getSource(), false))))
                        .then(ClientCommandManager.literal("streamlines")
                            .then(ClientCommandManager.literal("on")
                                .executes(ctx -> setRenderStreamlines(ctx.getSource(), true)))
                            .then(ClientCommandManager.literal("off")
                                .executes(ctx -> setRenderStreamlines(ctx.getSource(), false)))))
            );
        });
    }

    private void registerClientCommandInterceptor() {
        ClientSendMessageEvents.ALLOW_COMMAND.register(command -> {
            String normalized = command == null ? "" : command.trim().replaceAll("\\s+", " ");
            if (!normalized.equals("aero render") && !normalized.startsWith("aero render ")) {
                return true;
            }
            handleRenderCommand(normalized);
            return false;
        });
    }

    private void handleRenderCommand(String command) {
        String[] parts = command.split(" ");
        if (parts.length == 2) {
            sendClientFeedback(renderStatusText());
            return;
        }
        if (parts.length != 4) {
            sendClientFeedback(Text.literal("Usage: /aero render <vectors|streamlines> <on|off>"));
            return;
        }
        boolean enabled;
        if ("on".equals(parts[3])) {
            enabled = true;
        } else if ("off".equals(parts[3])) {
            enabled = false;
        } else {
            sendClientFeedback(Text.literal("Usage: /aero render <vectors|streamlines> <on|off>"));
            return;
        }
        switch (parts[2]) {
            case "vectors" -> {
                visualizer.setRenderVelocityVectors(enabled);
                sendClientFeedback(Text.literal("Render vectors " + (enabled ? "enabled" : "disabled")));
            }
            case "streamlines" -> {
                visualizer.setRenderStreamlines(enabled);
                sendClientFeedback(Text.literal("Render streamlines " + (enabled ? "enabled" : "disabled")));
            }
            default -> sendClientFeedback(Text.literal("Usage: /aero render <vectors|streamlines> <on|off>"));
        }
    }

    private void sendClientFeedback(Text message) {
        MinecraftClient client = MinecraftClient.getInstance();
        if (client.player != null) {
            client.player.sendMessage(message, false);
        }
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
        if (enabled) {
            visualizer.clearRemoteFlowFields();
        }
        sendClientL2Preference(enabled);
        source.sendFeedback(Text.literal("Client L2 local solve " + (enabled ? "enabled" : "disabled")));
        return 1;
    }

    private int clientL2StressStatus(net.fabricmc.fabric.api.client.command.v2.FabricClientCommandSource source) {
        source.sendFeedback(Text.literal("Client L2 stress " + clientL2Solver.stressStatus()));
        return 1;
    }

    private int setClientL2Stress(
        net.fabricmc.fabric.api.client.command.v2.FabricClientCommandSource source,
        String mode
    ) {
        try {
            source.sendFeedback(Text.literal(clientL2Solver.setStressMode(mode)));
        } catch (IllegalArgumentException error) {
            source.sendFeedback(Text.literal(error.getMessage()));
            return 0;
        }
        return 1;
    }

    private int renderStatus(net.fabricmc.fabric.api.client.command.v2.FabricClientCommandSource source) {
        source.sendFeedback(renderStatusText());
        return 1;
    }

    private Text renderStatusText() {
        return Text.literal(
            "Render vectors=" + visualizer.renderVelocityVectorsEnabled()
                + " streamlines=" + visualizer.renderStreamlinesEnabled()
        );
    }

    private int setRenderVelocityVectors(
        net.fabricmc.fabric.api.client.command.v2.FabricClientCommandSource source,
        boolean enabled
    ) {
        visualizer.setRenderVelocityVectors(enabled);
        source.sendFeedback(Text.literal("Render vectors " + (enabled ? "enabled" : "disabled")));
        return 1;
    }

    private int setRenderStreamlines(
        net.fabricmc.fabric.api.client.command.v2.FabricClientCommandSource source,
        boolean enabled
    ) {
        visualizer.setRenderStreamlines(enabled);
        source.sendFeedback(Text.literal("Render streamlines " + (enabled ? "enabled" : "disabled")));
        return 1;
    }

    private void sendClientL2Preference(boolean enabled) {
        try {
            if (ClientPlayNetworking.canSend(AeroClientL2PreferencePayload.ID)) {
                ClientPlayNetworking.send(new AeroClientL2PreferencePayload(enabled));
            }
        } catch (IllegalStateException ignored) {
            // The client may be between play-networking sessions while commands/state callbacks settle.
        }
    }
}
