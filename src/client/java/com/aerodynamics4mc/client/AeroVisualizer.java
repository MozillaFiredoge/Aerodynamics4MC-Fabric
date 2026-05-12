package com.aerodynamics4mc.client;

import java.util.HashMap;
import java.util.Map;

import com.mojang.blaze3d.vertex.PoseStack;
import com.mojang.blaze3d.vertex.VertexConsumer;
import org.joml.Matrix4f;

import com.aerodynamics4mc.api.AeroWindSamplingRules;
import com.aerodynamics4mc.api.AeroWindSample;
import com.aerodynamics4mc.api.SamplePolicy;
import com.aerodynamics4mc.flow.AnalysisFlowCodec;
import com.aerodynamics4mc.net.AeroCoarseWindPayload;
import com.aerodynamics4mc.net.AeroFlowAnalysisPayload;
import com.aerodynamics4mc.net.AeroFlowPayload;
import com.aerodynamics4mc.runtime.NativeSimulationBridge;

import net.fabricmc.fabric.api.client.event.lifecycle.v1.ClientTickEvents;
import net.fabricmc.fabric.api.client.networking.v1.ClientPlayConnectionEvents;
import net.fabricmc.fabric.api.client.rendering.v1.world.WorldRenderContext;
import net.fabricmc.fabric.api.client.rendering.v1.world.WorldRenderEvents;
import net.minecraft.client.Minecraft;
import net.minecraft.client.renderer.rendertype.RenderTypes;
import net.minecraft.core.BlockPos;
import net.minecraft.gizmos.GizmoStyle;
import net.minecraft.gizmos.Gizmos;
import net.minecraft.gizmos.TextGizmo;
import net.minecraft.resources.Identifier;
import net.minecraft.util.Mth;
import net.minecraft.world.phys.AABB;
import net.minecraft.world.phys.Vec3;
import org.joml.Matrix4f;

import java.util.HashMap;
import java.util.Map;

final class AeroVisualizer {
    private static final float ATLAS_VELOCITY_RANGE = 5.6f;
    private static final float ATLAS_PRESSURE_RANGE = 0.03f;
    private static final int COARSE_ATMOSPHERE_CHANNELS = 10;
    private static final float COARSE_TEMPERATURE_ANOMALY_RANGE_K = 64.0f;
    private static final float COARSE_TURBULENCE_RANGE_MPS = 3.0f;
    private static final float COARSE_SHEAR_RANGE_PER_BLOCK = 0.08f;
    private static final double MAX_RENDER_DISTANCE = 192.0;
    private static final double REMOTE_VECTOR_RENDER_DISTANCE = 144.0;
    private static final double ANALYSIS_RENDER_DISTANCE = 192.0;
    private static final int REGION_STALE_TICKS = 120;
    private static final int REGION_STALE_CLEANUP_INTERVAL_TICKS = 20;
    private static final int REMOTE_VECTOR_FIELD_STRIDE = 1;
    private static final int REMOTE_STREAMLINE_SEED_STRIDE = 1;
    private static final int REMOTE_STREAMLINE_FACE_BUCKETS = 1;
    private static final int REMOTE_STREAMLINE_SEEDS_PER_BUCKET = 5;
    private static final int REMOTE_STREAMLINE_SEEDS_PER_FACE =
            REMOTE_STREAMLINE_FACE_BUCKETS * REMOTE_STREAMLINE_FACE_BUCKETS * REMOTE_STREAMLINE_SEEDS_PER_BUCKET;
    private static final int REMOTE_STREAMLINE_MAX_STEPS = 192;
    private static final double REMOTE_STREAMLINE_MAX_LENGTH = 96.0;
    private static final float MIN_SPEED = 0.01f;
    private static final float RENDER_THRESHOLD_NORMALIZED = 0.02f;
    private static final float MAX_INFLOW_SPEED = 4.0f;
    private static final float REMOTE_STREAMLINE_MIN_INFLOW = MAX_INFLOW_SPEED * RENDER_THRESHOLD_NORMALIZED;
    private static final int ANALYSIS_SLICE_GLYPH_STEP = 2;
    private static final float ANALYSIS_SLICE_MIN_SPEED = 0.005f;
    private static final float ANALYSIS_SLICE_HEIGHT_OFFSET = 0.045f;
    private static final float ANALYSIS_SLICE_MIN_RANGE = 0.04f;
    private static final float ANALYSIS_SLICE_GLYPH_LENGTH = 1.4f;
    private static final float ANALYSIS_SLICE_GLYPH_WIDTH = 1.0f;
    private static final double ANALYSIS_SLICE_VIEW_OFFSET_Y = -1.25;

    private final Map<WindowKey, RemoteFlowField> remoteWindows = new HashMap<>();
    private final Map<WindowKey, RemoteFlowField> localWindows = new HashMap<>();
    private final Map<WindowKey, CoarseWindField> coarseWindFields = new HashMap<>();
    private final Map<WindowKey, AnalysisFlowField> analysisWindows = new HashMap<>();
    private final NativeSimulationBridge analysisCodecBridge = new NativeSimulationBridge();
    private boolean streamingEnabled;
    private boolean renderVelocityVectors = false;
    private boolean renderStreamlines = true;
    private long clientTickCounter;

    void initialize() {
        ClientPlayConnectionEvents.DISCONNECT.register((handler, client) -> clearState());
        ClientTickEvents.END_CLIENT_TICK.register(client -> onClientTick());
        WorldRenderEvents.BEFORE_DEBUG_RENDER.register(this::renderAtlasOverlay);
    }

    void onRuntimeState(AeroFlowState state) {
        streamingEnabled = state.streamingEnabled();
        if (!streamingEnabled) {
            remoteWindows.clear();
            localWindows.clear();
            coarseWindFields.clear();
        }
    }

    void onFlowField(AeroFlowPayload payload) {
        if (!streamingEnabled) {
            return;
        }
        WindowKey key = new WindowKey(payload.dimensionId(), payload.origin());
        remoteWindows.put(key, RemoteFlowField.fromPayload(payload, clientTickCounter));
    }

    void onLocalFlowField(Identifier dimensionId, BlockPos origin, int sampleStride, short[] packedFlow) {
        if (!streamingEnabled || packedFlow == null || packedFlow.length == 0) {
            return;
        }
        short[] localPackedFlow = java.util.Arrays.copyOf(packedFlow, packedFlow.length);
        WindowKey key = new WindowKey(dimensionId, origin);
        localWindows.put(
                key,
                RemoteFlowField.fromPacked(
                        dimensionId,
                        origin,
                        Math.max(1, sampleStride),
                        localPackedFlow,
                        AeroWindSample.Authority.CLIENT_LOCAL,
                        clientTickCounter
                )
        );
    }

    void clearLocalFlowFields() {
        localWindows.clear();
    }

    void clearRemoteFlowFields() {
        remoteWindows.clear();
    }

    void onCoarseWindField(AeroCoarseWindPayload payload) {
        if (!streamingEnabled) {
            return;
        }
        WindowKey key = new WindowKey(payload.dimensionType(), payload.origin());
        coarseWindFields.put(key, CoarseWindField.fromPayload(payload, clientTickCounter));
    }

    void onFlowAnalysis(AeroFlowAnalysisPayload payload) {
        if (!streamingEnabled) {
            return;
        }
        float[] flowState = AnalysisFlowCodec.decodePayload(analysisCodecBridge, payload);
        if (flowState == null) {
            return;
        }
        WindowKey key = new WindowKey(payload.dimensionId(), payload.origin());
        analysisWindows.put(
                key,
                new AnalysisFlowField(
                        payload.dimensionId(),
                        payload.origin(),
                        payload.fullResolution(),
                        flowState,
                        clientTickCounter
                )
        );
    }

    void clearState() {
        remoteWindows.clear();
        localWindows.clear();
        coarseWindFields.clear();
        analysisWindows.clear();
        streamingEnabled = false;
    }

    void setRenderVelocityVectors(boolean enabled) {
        renderVelocityVectors = enabled;
    }

    void setRenderStreamlines(boolean enabled) {
        renderStreamlines = enabled;
    }

    boolean renderVelocityVectorsEnabled() {
        return renderVelocityVectors;
    }

    boolean renderStreamlinesEnabled() {
        return renderStreamlines;
    }

    AeroWindSample sampleFlow(Identifier dimensionId, Vec3 position) {
        return sampleFlow(dimensionId, position, SamplePolicy.SERVER_COARSE_ONLY);
    }

    AeroWindSample sampleFlow(Identifier dimensionId, Vec3 position, SamplePolicy policy) {
        if (!streamingEnabled) {
            return AeroWindSample.ZERO;
        }
        SamplePolicy effectivePolicy = effectiveSamplePolicyForLocalPlayer(policy);
        RemoteFlowField l2Field = effectivePolicy.allowClientLocalL2()
                ? findNewestField(localWindows, dimensionId, position)
                : null;
        if (l2Field != null) {
            return l2Field.sampleFlowTrilinear(position);
        }
        if (effectivePolicy.allowServerAggregatedL2()) {
            l2Field = findNewestRemoteField(dimensionId, position);
            if (l2Field != null) {
                return l2Field.sampleFlowTrilinear(position);
            }
        }
        if (!effectivePolicy.allowServerCoarse()) {
            return AeroWindSample.ZERO;
        }
        CoarseWindField coarseField = findNewestCoarseField(dimensionId, position);
        if (coarseField != null) {
            return coarseField.sampleFlowTrilinear(position);
        }
        return AeroWindSample.ZERO;
    }

    private SamplePolicy effectiveSamplePolicyForLocalPlayer(SamplePolicy policy) {
        SamplePolicy effectivePolicy = policy == null ? SamplePolicy.SERVER_COARSE_ONLY : policy;
        Minecraft client = Minecraft.getInstance();
        if (effectivePolicy != SamplePolicy.DIAGNOSTIC_ALL_SOURCES
                && client != null
                && client.player != null
                && AeroWindSamplingRules.isFastPlayerVelocity(client.player.getDeltaMovement())) {
            return SamplePolicy.SERVER_COARSE_ONLY;
        }
        return effectivePolicy;
    }

    AeroWindSample sampleServerCoarseFlow(Identifier dimensionId, Vec3 position) {
        if (!streamingEnabled) {
            return AeroWindSample.ZERO;
        }
        CoarseWindField coarseField = findNewestCoarseField(dimensionId, position);
        return coarseField == null ? AeroWindSample.ZERO : coarseField.sampleFlowTrilinear(position);
    }

    Vec3 sampleWind(Identifier dimensionId, Vec3 position) {
        return sampleFlow(dimensionId, position).velocity();
    }

    private RemoteFlowField findNewestRemoteField(Identifier dimensionId, Vec3 position) {
        return findNewestField(remoteWindows, dimensionId, position);
    }

    private RemoteFlowField findNewestField(Map<WindowKey, RemoteFlowField> fields, Identifier dimensionId, Vec3 position) {
        RemoteFlowField best = null;
        long bestTick = Long.MIN_VALUE;
        for (RemoteFlowField field : fields.values()) {
            if (!field.dimensionId().equals(dimensionId)) {
                continue;
            }
            if (field.visibleBox().contains(position) && field.lastUpdatedTick() >= bestTick) {
                best = field;
                bestTick = field.lastUpdatedTick();
            }
        }
        return best;
    }

    private CoarseWindField findNewestCoarseField(Identifier dimensionId, Vec3 position) {
        CoarseWindField best = null;
        long bestTick = Long.MIN_VALUE;
        for (CoarseWindField field : coarseWindFields.values()) {
            if (!field.dimensionId().equals(dimensionId)) {
                continue;
            }
            if (field.regionBox().contains(position) && field.lastUpdatedTick() >= bestTick) {
                best = field;
                bestTick = field.lastUpdatedTick();
            }
        }
        return best;
    }

    private void onClientTick() {
        clientTickCounter++;
        if (!streamingEnabled) {
            return;
        }
        if (clientTickCounter % REGION_STALE_CLEANUP_INTERVAL_TICKS != 0L) {
            return;
        }
        remoteWindows.entrySet().removeIf(entry -> {
            return clientTickCounter - entry.getValue().lastUpdatedTick() > REGION_STALE_TICKS;
        });
        localWindows.entrySet().removeIf(entry -> {
            return clientTickCounter - entry.getValue().lastUpdatedTick() > REGION_STALE_TICKS;
        });
        coarseWindFields.entrySet().removeIf(entry -> {
            return clientTickCounter - entry.getValue().lastUpdatedTick() > REGION_STALE_TICKS;
        });
        analysisWindows.entrySet().removeIf(entry -> {
            return clientTickCounter - entry.getValue().lastUpdatedTick() > REGION_STALE_TICKS * 4L;
        });
    }

    private void renderAtlasOverlay(WorldRenderContext context) {
        if (!streamingEnabled || (remoteWindows.isEmpty() && localWindows.isEmpty())) {
            return;
        }
        Minecraft client = Minecraft.getInstance();
        if (client.level == null || client.player == null || !client.getDebugOverlay().showDebugScreen()) {
            return;
        }
        Identifier dimensionId = client.level.dimension().identifier();
        Vec3 cameraPos = client.gameRenderer.getMainCamera().position();
        VertexConsumer lineBuffer = context.consumers() == null ? null : context.consumers().getBuffer(RenderTypes.lines());
        PoseStack matrices = context.matrices();
        try (var ignored = client.collectPerTickGizmos()) {
            for (RemoteFlowField field : localWindows.values()) {
                if (!field.dimensionId().equals(dimensionId)) {
                    continue;
                }
                double distanceSq = field.visibleBox().distanceToSqr(cameraPos);
                if (distanceSq > MAX_RENDER_DISTANCE * MAX_RENDER_DISTANCE) {
                    continue;
                }
                renderRegionOutline(field, distanceSq);
                if (distanceSq <= REMOTE_VECTOR_RENDER_DISTANCE * REMOTE_VECTOR_RENDER_DISTANCE) {
                    renderFlowRendererField(lineBuffer, matrices, field, cameraPos);
                }
            }
            for (RemoteFlowField field : remoteWindows.values()) {
                if (!field.dimensionId().equals(dimensionId)) {
                    continue;
                }
                if (localWindows.containsKey(new WindowKey(field.dimensionId(), field.origin()))) {
                    continue;
                }
                double distanceSq = field.visibleBox().distanceToSqr(cameraPos);
                if (distanceSq > MAX_RENDER_DISTANCE * MAX_RENDER_DISTANCE) {
                    continue;
                }
                renderRegionOutline(field, distanceSq);
                if (distanceSq <= REMOTE_VECTOR_RENDER_DISTANCE * REMOTE_VECTOR_RENDER_DISTANCE) {
                    renderFlowRendererField(lineBuffer, matrices, field, cameraPos);
                }
            }
            AnalysisSliceView analysisSlice = prepareAnalysisSlice(client, dimensionId, cameraPos);
            if (analysisSlice != null) {
                renderAnalysisOverlay(analysisSlice);
            }
        }
    }

    private void renderAnalysisOverlay(AnalysisSliceView analysisSlice) {
        AnalysisFlowField field = analysisSlice.field();
        int sliceY = analysisSlice.sliceY();
        float speedRange = analysisSlice.speedRange();
        SliceStats sliceStats = analysisSlice.sliceStats();
        renderAnalysisSliceGlyphs(field, sliceY, speedRange);

        Vec3 labelPos = new Vec3(
                field.origin().getX() + field.fullResolution() * 0.5,
                field.origin().getY() + sliceY + 0.9,
                field.origin().getZ() + field.fullResolution() * 0.5
        );
        String label = "CFD slice |v| y=" + (field.origin().getY() + sliceY)
                + " max=" + String.format("%.2f", sliceStats.maxSpeed())
                + " p95=" + String.format("%.2f", sliceStats.p95Speed());
        Gizmos.billboardText(
                label,
                labelPos,
                TextGizmo.Style.forColorAndCentered(argb(0.92f, 0.96f, 0.98f, 1.0f)).withScale(0.03f)
        ).setAlwaysOnTop().persistForMillis(1);
    }

    private AnalysisSliceView prepareAnalysisSlice(Minecraft client, Identifier dimensionId, Vec3 cameraPos) {
        AnalysisFlowField field = selectAnalysisField(dimensionId, cameraPos);
        if (field == null) {
            return null;
        }
        double distanceSq = field.regionBox().distanceToSqr(cameraPos);
        if (distanceSq > ANALYSIS_RENDER_DISTANCE * ANALYSIS_RENDER_DISTANCE) {
            return null;
        }
        double sliceWorldY = client.player != null ? client.player.getY() + ANALYSIS_SLICE_VIEW_OFFSET_Y : cameraPos.y + ANALYSIS_SLICE_VIEW_OFFSET_Y;
        int sliceY = field.sliceIndexForWorldY(sliceWorldY);
        SliceStats sliceStats = field.sliceStats(sliceY);
        float speedRange = Math.max(ANALYSIS_SLICE_MIN_RANGE, sliceStats.colorRange());
        return new AnalysisSliceView(field, sliceY, sliceStats, speedRange);
    }

    private AnalysisFlowField selectAnalysisField(Identifier dimensionId, Vec3 cameraPos) {
        AnalysisFlowField bestContaining = null;
        double bestContainingDistanceSq = Double.POSITIVE_INFINITY;
        AnalysisFlowField bestNearest = null;
        double bestNearestDistanceSq = Double.POSITIVE_INFINITY;
        for (AnalysisFlowField field : analysisWindows.values()) {
            if (!field.dimensionId().equals(dimensionId)) {
                continue;
            }
            AABB regionBox = field.regionBox();
            double distanceSq = regionBox.distanceToSqr(cameraPos);
            if (regionBox.contains(cameraPos)) {
                if (distanceSq < bestContainingDistanceSq) {
                    bestContaining = field;
                    bestContainingDistanceSq = distanceSq;
                }
            } else if (distanceSq < bestNearestDistanceSq) {
                bestNearest = field;
                bestNearestDistanceSq = distanceSq;
            }
        }
        return bestContaining != null ? bestContaining : bestNearest;
    }

    private void renderAnalysisSliceGlyphs(AnalysisFlowField field, int sliceY, float speedRange) {
        int resolution = field.fullResolution();
        double arrowBaseY = field.origin().getY() + sliceY + ANALYSIS_SLICE_HEIGHT_OFFSET + 0.08;
        for (int x = 0; x < resolution; x += ANALYSIS_SLICE_GLYPH_STEP) {
            for (int z = 0; z < resolution; z += ANALYSIS_SLICE_GLYPH_STEP) {
                Vec3 velocity = field.velocity(x, sliceY, z);
                Vec3 horizontal = new Vec3(velocity.x, 0.0, velocity.z);
                double horizontalSpeed = horizontal.length();
                if (horizontalSpeed < ANALYSIS_SLICE_MIN_SPEED) {
                    continue;
                }
                Vec3 direction = horizontal.scale(1.0 / horizontalSpeed);
                float speedNorm = Math.max(0.05f, clamp01((float) horizontalSpeed / speedRange));
                double arrowLength = ANALYSIS_SLICE_GLYPH_LENGTH + ANALYSIS_SLICE_GLYPH_LENGTH * speedNorm;
                Vec3 start = new Vec3(field.origin().getX() + x + 0.5, arrowBaseY, field.origin().getZ() + z + 0.5);
                Vec3 end = start.add(direction.scale(arrowLength));
                int glyphColor = analysisScalarColor(speedNorm, 0.72f);
                Gizmos.arrow(start, end, glyphColor, ANALYSIS_SLICE_GLYPH_WIDTH).setAlwaysOnTop().persistForMillis(1);
            }
        }
    }

    private void renderFlowRendererField(VertexConsumer buffer, PoseStack matrices, RemoteFlowField field, Vec3 cameraPos) {
        if (buffer == null || matrices == null || (!renderVelocityVectors && !renderStreamlines)) {
            return;
        }
        if (renderVelocityVectors) {
            matrices.pushPose();
            matrices.translate(
                    field.origin().getX() - cameraPos.x,
                    field.origin().getY() - cameraPos.y,
                    field.origin().getZ() - cameraPos.z
            );
            renderVelocityField(buffer, matrices, field);
            matrices.popPose();
        }
        if (renderStreamlines) {
            matrices.pushPose();
            renderStreamlines(buffer, matrices, field, cameraPos);
            matrices.popPose();
        }
    }

    private void renderVelocityField(VertexConsumer buffer, PoseStack matrices, RemoteFlowField field) {
        int gridSize = field.fullResolution();
        int stride = REMOTE_VECTOR_FIELD_STRIDE;
        float velScale = 1.0f;
        var entry = matrices.last();
        Matrix4f matrix = entry.pose();

        for (int x = 0; x < gridSize; x += stride) {
            for (int y = 0; y < gridSize; y += stride) {
                for (int z = 0; z < gridSize; z += stride) {
                    Vec3 velocity = field.sampleVelocityLocal(x + 0.5, y + 0.5, z + 0.5);
                    float speed = (float) velocity.length();
                    float speedNorm = Mth.clamp(speed / MAX_INFLOW_SPEED, 0.0f, 1.0f);
                    if (speedNorm < RENDER_THRESHOLD_NORMALIZED) {
                        continue;
                    }
                    Vec3 direction = velocity.normalize();
                    float lineLength = Mth.clamp(speedNorm * velScale, 0.05f, 10f);
                    int color = getViridisColor(speedNorm * velScale);
                    int r = (color >> 16) & 0xFF;
                    int g = (color >> 8) & 0xFF;
                    int b = color & 0xFF;
                    float fx = x + 0.5f;
                    float fy = y + 0.5f;
                    float fz = z + 0.5f;

                    buffer.addVertex(matrix, fx, fy, fz)
                            .setColor(r, g, b, 255)
                            .setNormal(entry, (float) direction.x, (float) direction.y, (float) direction.z)
                            .setLineWidth(1.2f);
                    buffer.addVertex(
                                    matrix,
                                    fx + (float) direction.x * lineLength,
                                    fy + (float) direction.y * lineLength,
                                    fz + (float) direction.z * lineLength
                            )
                            .setColor(r, g, b, 255)
                            .setNormal(entry, (float) direction.x, (float) direction.y, (float) direction.z)
                            .setLineWidth(1.2f);
                }
            }
        }
    }

    private void renderStreamlines(VertexConsumer buffer, PoseStack matrices, RemoteFlowField field, Vec3 cameraPos) {
        int gridSize = field.fullResolution();
        int seedStride = REMOTE_STREAMLINE_SEED_STRIDE;
        float stepSize = 0.35f;
        int maxSteps = REMOTE_STREAMLINE_MAX_STEPS;
        var entry = matrices.last();
        Matrix4f matrix = entry.pose();

        renderInflowFaceStreamlines(buffer, entry, matrix, field, cameraPos, gridSize, seedStride, stepSize, maxSteps, 0, -1);
        renderInflowFaceStreamlines(buffer, entry, matrix, field, cameraPos, gridSize, seedStride, stepSize, maxSteps, 0, 1);
        renderInflowFaceStreamlines(buffer, entry, matrix, field, cameraPos, gridSize, seedStride, stepSize, maxSteps, 1, -1);
        renderInflowFaceStreamlines(buffer, entry, matrix, field, cameraPos, gridSize, seedStride, stepSize, maxSteps, 1, 1);
        renderInflowFaceStreamlines(buffer, entry, matrix, field, cameraPos, gridSize, seedStride, stepSize, maxSteps, 2, -1);
        renderInflowFaceStreamlines(buffer, entry, matrix, field, cameraPos, gridSize, seedStride, stepSize, maxSteps, 2, 1);
    }

    private void renderInflowFaceStreamlines(
            VertexConsumer buffer,
            PoseStack.Pose entry,
            Matrix4f matrix,
            RemoteFlowField field,
            Vec3 cameraPos,
            int gridSize,
            int seedStride,
            float stepSize,
            int maxSteps,
            int axis,
            int side
    ) {
        if (hasNeighborAcrossFace(field, axis, side)) {
            return;
        }
        float[] bestScores = new float[REMOTE_STREAMLINE_SEEDS_PER_FACE];
        float[] bestX = new float[REMOTE_STREAMLINE_SEEDS_PER_FACE];
        float[] bestY = new float[REMOTE_STREAMLINE_SEEDS_PER_FACE];
        float[] bestZ = new float[REMOTE_STREAMLINE_SEEDS_PER_FACE];
        for (int i = 0; i < bestScores.length; i++) {
            bestScores[i] = Float.NEGATIVE_INFINITY;
        }
        int inwardSign = side < 0 ? 1 : -1;
        float faceCoord = side < 0 ? 0.5f : gridSize - 0.5f;

        for (int u = 0; u < gridSize; u += seedStride) {
            for (int v = 0; v < gridSize; v += seedStride) {
                float x = axis == 0 ? faceCoord : u + 0.5f;
                float y = axis == 1 ? faceCoord : (axis == 0 ? u + 0.5f : v + 0.5f);
                float z = axis == 2 ? faceCoord : v + 0.5f;
                Vec3 velocity = field.sampleVelocityLocal(x, y, z);
                float inward = inwardComponent(velocity, axis, inwardSign);
                if (inward < REMOTE_STREAMLINE_MIN_INFLOW) {
                    continue;
                }
                float speed = (float) velocity.length();
                if (speed < MIN_SPEED) {
                    continue;
                }
                int bucket = streamlineFaceBucket(u, v, gridSize);
                float score = inward * (0.35f + 0.65f * Mth.clamp(speed / MAX_INFLOW_SPEED, 0.0f, 1.0f));
                int bucketBase = bucket * REMOTE_STREAMLINE_SEEDS_PER_BUCKET;
                for (int slot = 0; slot < REMOTE_STREAMLINE_SEEDS_PER_BUCKET; slot++) {
                    int seedIndex = bucketBase + slot;
                    if (score <= bestScores[seedIndex]) {
                        continue;
                    }
                    for (int shift = REMOTE_STREAMLINE_SEEDS_PER_BUCKET - 1; shift > slot; shift--) {
                        int dst = bucketBase + shift;
                        int src = dst - 1;
                        bestScores[dst] = bestScores[src];
                        bestX[dst] = bestX[src];
                        bestY[dst] = bestY[src];
                        bestZ[dst] = bestZ[src];
                    }
                    bestScores[seedIndex] = score;
                    bestX[seedIndex] = x;
                    bestY[seedIndex] = y;
                    bestZ[seedIndex] = z;
                    break;
                }
            }
        }

        for (int i = 0; i < bestScores.length; i++) {
            if (bestScores[i] > Float.NEGATIVE_INFINITY) {
                renderStreamlineFromSeed(
                        buffer,
                        entry,
                        matrix,
                        field,
                        field.localToWorld(new Vec3(bestX[i], bestY[i], bestZ[i])),
                        cameraPos,
                        stepSize,
                        maxSteps
                );
            }
        }
    }

    private int streamlineFaceBucket(int u, int v, int gridSize) {
        int bucketU = Math.min(REMOTE_STREAMLINE_FACE_BUCKETS - 1, Math.max(0, u * REMOTE_STREAMLINE_FACE_BUCKETS / gridSize));
        int bucketV = Math.min(REMOTE_STREAMLINE_FACE_BUCKETS - 1, Math.max(0, v * REMOTE_STREAMLINE_FACE_BUCKETS / gridSize));
        return bucketU * REMOTE_STREAMLINE_FACE_BUCKETS + bucketV;
    }

    private float inwardComponent(Vec3 velocity, int axis, int inwardSign) {
        double component = axis == 0 ? velocity.x : (axis == 1 ? velocity.y : velocity.z);
        return (float) component * inwardSign;
    }

    private void renderStreamlineFromSeed(
            VertexConsumer buffer,
            PoseStack.Pose entry,
            Matrix4f matrix,
            RemoteFlowField field,
            Vec3 seed,
            Vec3 cameraPos,
            float stepSize,
            int maxSteps
    ) {
        Vec3 pos = seed;
        RemoteFlowField currentField = field;
        double traveled = 0.0;
        for (int step = 0; step < maxSteps; step++) {
            FlowSample sample = sampleGlobalVelocity(field.dimensionId(), pos, currentField);
            if (sample == null) {
                break;
            }
            currentField = sample.field();
            Vec3 velocity = sample.velocity();
            float speed = (float) velocity.length();
            if (speed < MIN_SPEED) {
                break;
            }

            float speedNorm = Mth.clamp(speed / MAX_INFLOW_SPEED, 0.0f, 1.0f);
            if (speedNorm < RENDER_THRESHOLD_NORMALIZED) {
                break;
            }

            Vec3 dir = velocity.normalize();
            float advectStep = stepSize * Mth.clamp(speedNorm * 8.0f, 0.2f, 1.25f);
            Vec3 nextPos = pos.add(dir.scale(advectStep));
            RemoteFlowField nextField = findFieldForPosition(field.dimensionId(), nextPos, currentField);
            if (nextField == null) {
                break;
            }
            double segmentLength = pos.distanceTo(nextPos);
            if (traveled + segmentLength > REMOTE_STREAMLINE_MAX_LENGTH) {
                break;
            }

            int color = getViridisColor(speedNorm);
            int r = (color >> 16) & 0xFF;
            int g = (color >> 8) & 0xFF;
            int b = color & 0xFF;
            Vec3 segDir = nextPos.subtract(pos).normalize();

            buffer.addVertex(
                            matrix,
                            (float) (pos.x - cameraPos.x),
                            (float) (pos.y - cameraPos.y),
                            (float) (pos.z - cameraPos.z)
                    )
                    .setColor(r, g, b, 255)
                    .setNormal(entry, (float) segDir.x, (float) segDir.y, (float) segDir.z)
                    .setLineWidth(3.0f);
            buffer.addVertex(
                            matrix,
                            (float) (nextPos.x - cameraPos.x),
                            (float) (nextPos.y - cameraPos.y),
                            (float) (nextPos.z - cameraPos.z)
                    )
                    .setColor(r, g, b, 255)
                    .setNormal(entry, (float) segDir.x, (float) segDir.y, (float) segDir.z)
                    .setLineWidth(3.0f);

            pos = nextPos;
            currentField = nextField;
            traveled += segmentLength;
        }
    }

    private boolean hasNeighborAcrossFace(RemoteFlowField field, int axis, int side) {
        int offset = side * field.fullResolution();
        BlockPos neighborOrigin = switch (axis) {
            case 0 -> field.origin().offset(offset, 0, 0);
            case 1 -> field.origin().offset(0, offset, 0);
            default -> field.origin().offset(0, 0, offset);
        };
        return remoteWindows.containsKey(new WindowKey(field.dimensionId(), neighborOrigin));
    }

    private FlowSample sampleGlobalVelocity(Identifier dimensionId, Vec3 worldPos, RemoteFlowField preferredField) {
        RemoteFlowField field = findFieldForPosition(dimensionId, worldPos, preferredField);
        if (field == null) {
            return null;
        }
        return new FlowSample(field.sampleVelocityTrilinearClamped(worldPos), field);
    }

    private RemoteFlowField findFieldForPosition(Identifier dimensionId, Vec3 worldPos, RemoteFlowField preferredField) {
        if (preferredField != null
                && preferredField.dimensionId().equals(dimensionId)
                && containsHalfOpen(preferredField.regionBox(), worldPos)) {
            return preferredField;
        }
        if (preferredField != null) {
            RemoteFlowField aligned = findAlignedFieldForPosition(dimensionId, worldPos, preferredField);
            if (aligned != null) {
                return aligned;
            }
        }
        for (RemoteFlowField field : remoteWindows.values()) {
            if (field.dimensionId().equals(dimensionId) && containsHalfOpen(field.regionBox(), worldPos)) {
                return field;
            }
        }
        return null;
    }

    private RemoteFlowField findAlignedFieldForPosition(Identifier dimensionId, Vec3 worldPos, RemoteFlowField referenceField) {
        int size = Math.max(1, referenceField.fullResolution());
        int x = referenceField.origin().getX() + Math.floorDiv((int) Math.floor(worldPos.x) - referenceField.origin().getX(), size) * size;
        int y = referenceField.origin().getY() + Math.floorDiv((int) Math.floor(worldPos.y) - referenceField.origin().getY(), size) * size;
        int z = referenceField.origin().getZ() + Math.floorDiv((int) Math.floor(worldPos.z) - referenceField.origin().getZ(), size) * size;
        RemoteFlowField aligned = remoteWindows.get(new WindowKey(dimensionId, new BlockPos(x, y, z)));
        return aligned != null && containsHalfOpen(aligned.regionBox(), worldPos) ? aligned : null;
    }

    private boolean containsHalfOpen(AABB box, Vec3 position) {
        return position.x >= box.minX && position.x < box.maxX
                && position.y >= box.minY && position.y < box.maxY
                && position.z >= box.minZ && position.z < box.maxZ;
    }

    private int getViridisColor(float t) {
        t = Mth.clamp(t, 0.0f, 1.0f);
        float r;
        float g;
        float b;
        if (t < 0.25f) {
            float local = t / 0.25f;
            r = 0.0f;
            g = local;
            b = 1.0f;
        } else if (t < 0.5f) {
            float local = (t - 0.25f) / 0.25f;
            r = 0.0f;
            g = 1.0f;
            b = 1.0f - local;
        } else if (t < 0.75f) {
            float local = (t - 0.5f) / 0.25f;
            r = local;
            g = 1.0f;
            b = 0.0f;
        } else {
            float local = (t - 0.75f) / 0.25f;
            r = 1.0f;
            g = 1.0f - local;
            b = 0.0f;
        }
        return ((int) (r * 255.0f) << 16) | ((int) (g * 255.0f) << 8) | (int) (b * 255.0f);
    }

    private void renderRegionOutline(RemoteFlowField field, double distanceSq) {
        FlowVisual visual = field.visual();
        float maxSpeedNorm = clamp01(visual.maxSpeed() / 1.5f);
        float distanceNorm = clamp01((float) (Math.sqrt(distanceSq) / MAX_RENDER_DISTANCE));
        float distanceFade = 1.0f - smoothstep(0.20f, 1.0f, distanceNorm);
        int strokeColor = viridisColor(maxSpeedNorm, 0.35f + 0.35f * distanceFade);
        int fillColor = viridisColor(maxSpeedNorm * 0.65f, 0.03f + 0.04f * distanceFade);
        GizmoStyle style = GizmoStyle.strokeAndFill(strokeColor, 1.0f + 0.6f * maxSpeedNorm, fillColor);
        Gizmos.cuboid(field.visibleBox(), style).setAlwaysOnTop().persistForMillis(1);
    }

    private static float decodeVelocity(short value) {
        return value / 32767.0f * ATLAS_VELOCITY_RANGE;
    }

    private static float decodePressure(short value) {
        return value / 32767.0f * ATLAS_PRESSURE_RANGE;
    }

    private static float decodeTemperatureKelvin(short value) {
        return 288.15f + value / 32767.0f * COARSE_TEMPERATURE_ANOMALY_RANGE_K;
    }

    private static float decodeUnit01(short value) {
        return clamp01(value / 32767.0f * 0.5f + 0.5f);
    }

    private static float decodeTurbulence(short value) {
        return Math.max(0.0f, value / 32767.0f * COARSE_TURBULENCE_RANGE_MPS);
    }

    private static float decodeShear(short value) {
        return value / 32767.0f * COARSE_SHEAR_RANGE_PER_BLOCK;
    }

    private static float clamp01(float value) {
        return Math.max(0.0f, Math.min(1.0f, value));
    }

    private static float lerp(float a, float b, float t) {
        return a + (b - a) * clamp01(t);
    }

    private static float smoothstep(float edge0, float edge1, float value) {
        if (edge0 == edge1) {
            return value < edge0 ? 0.0f : 1.0f;
        }
        float t = clamp01((value - edge0) / (edge1 - edge0));
        return t * t * (3.0f - 2.0f * t);
    }

    private static int argb(float alpha, float red, float green, float blue) {
        int a = Math.round(clamp01(alpha) * 255.0f);
        int r = Math.round(clamp01(red) * 255.0f);
        int g = Math.round(clamp01(green) * 255.0f);
        int b = Math.round(clamp01(blue) * 255.0f);
        return (a << 24) | (r << 16) | (g << 8) | b;
    }

    private static int analysisScalarColor(float value, float alpha) {
        float t = clamp01(value);
        float r;
        float g;
        float b;
        if (t < 0.25f) {
            float u = t / 0.25f;
            r = lerp(0.08f, 0.10f, u);
            g = lerp(0.14f, 0.55f, u);
            b = lerp(0.56f, 0.96f, u);
        } else if (t < 0.50f) {
            float u = (t - 0.25f) / 0.25f;
            r = lerp(0.10f, 0.18f, u);
            g = lerp(0.55f, 0.92f, u);
            b = lerp(0.96f, 0.38f, u);
        } else if (t < 0.75f) {
            float u = (t - 0.50f) / 0.25f;
            r = lerp(0.18f, 0.96f, u);
            g = lerp(0.92f, 0.90f, u);
            b = lerp(0.38f, 0.12f, u);
        } else {
            float u = (t - 0.75f) / 0.25f;
            r = lerp(0.96f, 1.00f, u);
            g = lerp(0.90f, 0.24f, u);
            b = lerp(0.12f, 0.06f, u);
        }
        return argb(alpha, r, g, b);
    }

    private static int viridisColor(float value, float alpha) {
        float t = clamp01(value);
        float[] c0;
        float[] c1;
        float u;
        if (t < 0.25f) {
            c0 = new float[]{0.267f, 0.005f, 0.329f};
            c1 = new float[]{0.283f, 0.141f, 0.458f};
            u = t / 0.25f;
        } else if (t < 0.50f) {
            c0 = new float[]{0.283f, 0.141f, 0.458f};
            c1 = new float[]{0.254f, 0.265f, 0.530f};
            u = (t - 0.25f) / 0.25f;
        } else if (t < 0.75f) {
            c0 = new float[]{0.254f, 0.265f, 0.530f};
            c1 = new float[]{0.207f, 0.372f, 0.553f};
            u = (t - 0.50f) / 0.25f;
        } else {
            c0 = new float[]{0.207f, 0.372f, 0.553f};
            c1 = new float[]{0.993f, 0.906f, 0.144f};
            u = (t - 0.75f) / 0.25f;
        }
        return argb(alpha, lerp(c0[0], c1[0], u), lerp(c0[1], c1[1], u), lerp(c0[2], c1[2], u));
    }

    private record WindowKey(Identifier dimensionId, BlockPos origin) {
    }

    private record AnalysisSliceView(
            AnalysisFlowField field,
            int sliceY,
            SliceStats sliceStats,
            float speedRange
    ) {
    }

    private record AnalysisFlowField(
            Identifier dimensionId,
            BlockPos origin,
            int fullResolution,
            float[] flowState,
            long lastUpdatedTick
    ) {
        AABB regionBox() {
            return new AABB(
                    origin.getX(),
                    origin.getY(),
                    origin.getZ(),
                    origin.getX() + fullResolution,
                    origin.getY() + fullResolution,
                    origin.getZ() + fullResolution
            );
        }

        int sliceIndexForWorldY(double worldY) {
            int local = (int) Math.floor(worldY - origin.getY());
            return Math.max(0, Math.min(fullResolution - 1, local));
        }

        Vec3 velocity(int x, int y, int z) {
            int index = (((x * fullResolution) + y) * fullResolution + z) * 4;
            if (index < 0 || index + 2 >= flowState.length) {
                return Vec3.ZERO;
            }
            return new Vec3(flowState[index], flowState[index + 1], flowState[index + 2]);
        }

        float pressure(int x, int y, int z) {
            int index = (((x * fullResolution) + y) * fullResolution + z) * 4 + 3;
            if (index < 0 || index >= flowState.length) {
                return 0.0f;
            }
            return flowState[index];
        }

        float speed(int x, int y, int z) {
            Vec3 velocity = velocity(x, y, z);
            return (float) velocity.length();
        }

        float maxSpeedOnSlice(int y) {
            return sliceStats(y).maxSpeed();
        }

        SliceStats sliceStats(int y) {
            int sampleCount = fullResolution * fullResolution;
            float[] speeds = new float[sampleCount];
            float max = 0.0f;
            float sum = 0.0f;
            int cursor = 0;
            for (int x = 0; x < fullResolution; x++) {
                for (int z = 0; z < fullResolution; z++) {
                    float speed = speed(x, y, z);
                    speeds[cursor++] = speed;
                    max = Math.max(max, speed);
                    sum += speed;
                }
            }
            java.util.Arrays.sort(speeds);
            float p80 = speeds[Math.max(0, Math.min(speeds.length - 1, (int) Math.floor((speeds.length - 1) * 0.80)))];
            float p95 = speeds[Math.max(0, Math.min(speeds.length - 1, (int) Math.floor((speeds.length - 1) * 0.95)))];
            float mean = sum / Math.max(1, sampleCount);
            float colorRange = Math.max(Math.max(p80, mean * 1.15f), max * 0.12f);
            return new SliceStats(max, mean, p95, colorRange);
        }
    }

    private record SliceStats(
            float maxSpeed,
            float meanSpeed,
            float p95Speed,
            float colorRange
    ) {
    }

    private record FlowSample(Vec3 velocity, RemoteFlowField field) {
    }

    private record PackedFlowSample(float velocityX, float velocityY, float velocityZ, float pressure) {
        PackedFlowSample lerp(PackedFlowSample other, double t) {
            float f = (float) t;
            return new PackedFlowSample(
                    velocityX + (other.velocityX - velocityX) * f,
                    velocityY + (other.velocityY - velocityY) * f,
                    velocityZ + (other.velocityZ - velocityZ) * f,
                    pressure + (other.pressure - pressure) * f
            );
        }
    }

    private record CoarseWindField(
            Identifier dimensionId,
            BlockPos origin,
            int cellSize,
            int sizeX,
            int sizeY,
            int sizeZ,
            short[] packedFlow,
            short[] packedAtmosphere,
            long lastUpdatedTick
    ) {
        static CoarseWindField fromPayload(AeroCoarseWindPayload payload, long lastUpdatedTick) {
            return new CoarseWindField(
                    payload.dimensionType(),
                    payload.origin(),
                    Math.max(1, payload.cellSize()),
                    Math.max(1, payload.sizeX()),
                    Math.max(1, payload.sizeY()),
                    Math.max(1, payload.sizeZ()),
                    payload.packedFlow(),
                    payload.packedAtmosphere(),
                    lastUpdatedTick
            );
        }

        AeroWindSample sampleFlowTrilinear(Vec3 position) {
            AeroWindSample flow = samplePackedFlow(
                    origin,
                    cellSize,
                    sizeX,
                    sizeY,
                    sizeZ,
                    packedFlow,
                    position,
                    AeroWindSample.Level.L1,
                    AeroWindSample.Authority.SERVER_AUTHORITATIVE
            );
            if (!flow.hasFlow() || packedAtmosphere == null || packedAtmosphere.length <= 0) {
                return flow;
            }
            PackedAtmosphereSample atmosphere = samplePackedAtmosphere(
                    origin,
                    cellSize,
                    sizeX,
                    sizeY,
                    sizeZ,
                    packedAtmosphere,
                    position
            );
            return flow.withAtmosphere(
                    atmosphere.temperatureKelvin(),
                    atmosphere.humidity(),
                    atmosphere.turbulenceIntensity(),
                    atmosphere.gustX(),
                    atmosphere.gustY(),
                    atmosphere.gustZ(),
                    atmosphere.windShearXPerBlock(),
                    atmosphere.windShearZPerBlock(),
                    atmosphere.ablStability(),
                    atmosphere.ablMixingStrength()
            );
        }

        AABB regionBox() {
            return new AABB(
                    origin.getX(), origin.getY(), origin.getZ(),
                    origin.getX() + (double) sizeX * cellSize,
                    origin.getY() + (double) sizeY * cellSize,
                    origin.getZ() + (double) sizeZ * cellSize
            );
        }
    }

    private record RemoteFlowField(
            Identifier dimensionId,
            BlockPos origin,
            int sampleStride,
            int atlasResolution,
            short[] packedFlow,
            FlowVisual visual,
            AeroWindSample.Authority authority,
            long lastUpdatedTick
    ) {
        static RemoteFlowField fromPayload(AeroFlowPayload payload, long lastUpdatedTick) {
            return fromPacked(
                    payload.dimensionId(),
                    payload.origin(),
                    payload.sampleStride(),
                    payload.packedFlow(),
                    AeroWindSample.Authority.SERVER_AUTHORITATIVE,
                    lastUpdatedTick
            );
        }

        static RemoteFlowField fromPacked(
                Identifier dimensionId,
                BlockPos origin,
                int sampleStride,
                short[] packedFlow,
                AeroWindSample.Authority authority,
                long lastUpdatedTick
        ) {
            int sampleCount = packedFlow.length / 4;
            int atlasResolution = Math.max(1, Math.round((float) Math.cbrt(sampleCount)));
            return new RemoteFlowField(
                    dimensionId,
                    origin,
                    sampleStride,
                    atlasResolution,
                    packedFlow,
                    FlowVisual.fromPackedFlow(packedFlow),
                    authority,
                    lastUpdatedTick
            );
        }

        AeroWindSample sampleFlowTrilinear(Vec3 position) {
            return samplePackedFlow(
                    origin,
                    sampleStride,
                    atlasResolution,
                    atlasResolution,
                    atlasResolution,
                    packedFlow,
                    position,
                    AeroWindSample.Level.L2,
                    authority
            );
        }

        Vec3 sampleVelocity(Vec3 position) {
            double lx = (position.x - origin.getX()) / sampleStride;
            double ly = (position.y - origin.getY()) / sampleStride;
            double lz = (position.z - origin.getZ()) / sampleStride;
            int ix = (int) Math.round(lx - 0.5);
            int iy = (int) Math.round(ly - 0.5);
            int iz = (int) Math.round(lz - 0.5);
            if (ix < 0 || iy < 0 || iz < 0 || ix >= atlasResolution || iy >= atlasResolution || iz >= atlasResolution) {
                return Vec3.ZERO;
            }
            int cell = ((ix * atlasResolution + iy) * atlasResolution + iz) * 4;
            if (cell + 3 >= packedFlow.length) {
                return Vec3.ZERO;
            }
            return new Vec3(
                    decodeVelocity(packedFlow[cell]),
                    decodeVelocity(packedFlow[cell + 1]),
                    decodeVelocity(packedFlow[cell + 2])
            );
        }

        Vec3 sampleVelocityTrilinear(Vec3 position) {
            return sampleVelocity(origin, sampleStride, atlasResolution, packedFlow, position);
        }

        Vec3 sampleVelocityTrilinearClamped(Vec3 position) {
            return sampleVelocityClamped(origin, sampleStride, atlasResolution, packedFlow, position);
        }

        Vec3 sampleVelocityLocal(double x, double y, double z) {
            return sampleVelocityTrilinear(localToWorld(new Vec3(x, y, z)));
        }

        Vec3 localToWorld(Vec3 local) {
            return new Vec3(origin.getX() + local.x, origin.getY() + local.y, origin.getZ() + local.z);
        }

        AABB regionBox() {
            double size = (double) atlasResolution * (double) sampleStride;
            return new AABB(
                    origin.getX(), origin.getY(), origin.getZ(),
                    origin.getX() + size, origin.getY() + size, origin.getZ() + size
            );
        }

        AABB visibleBox() {
            return regionBox();
        }

        int fullResolution() {
            return atlasResolution * sampleStride;
        }

        int sliceIndexForWorldY(double worldY) {
            int local = (int) Math.floor(worldY - origin.getY());
            return Math.max(0, Math.min(fullResolution() - 1, local));
        }

        Vec3 velocity(int x, int y, int z) {
            return sampleVelocityTrilinear(new Vec3(
                    origin.getX() + x + 0.5,
                    origin.getY() + y + 0.5,
                    origin.getZ() + z + 0.5
            ));
        }

        float speed(int x, int y, int z) {
            return (float) velocity(x, y, z).length();
        }

        SliceStats sliceStats(int y) {
            int resolution = fullResolution();
            int sampleCount = resolution * resolution;
            float[] speeds = new float[sampleCount];
            float max = 0.0f;
            float sum = 0.0f;
            int cursor = 0;
            for (int x = 0; x < resolution; x++) {
                for (int z = 0; z < resolution; z++) {
                    float speed = speed(x, y, z);
                    speeds[cursor++] = speed;
                    max = Math.max(max, speed);
                    sum += speed;
                }
            }
            java.util.Arrays.sort(speeds);
            float p80 = speeds[Math.max(0, Math.min(speeds.length - 1, (int) Math.floor((speeds.length - 1) * 0.80)))];
            float p95 = speeds[Math.max(0, Math.min(speeds.length - 1, (int) Math.floor((speeds.length - 1) * 0.95)))];
            float mean = sum / Math.max(1, sampleCount);
            float colorRange = Math.max(Math.max(p80, mean * 1.15f), max * 0.12f);
            return new SliceStats(max, mean, p95, colorRange);
        }

        private static Vec3 sampleVelocity(BlockPos origin, int sampleStride, int atlasResolution, short[] packedFlow, Vec3 position) {
            double lx = (position.x - origin.getX()) / sampleStride - 0.5;
            double ly = (position.y - origin.getY()) / sampleStride - 0.5;
            double lz = (position.z - origin.getZ()) / sampleStride - 0.5;
            if (lx < 0.0 || ly < 0.0 || lz < 0.0
                    || lx > atlasResolution - 1.0 || ly > atlasResolution - 1.0 || lz > atlasResolution - 1.0) {
                return Vec3.ZERO;
            }
            return sampleVelocityAtSampleCoordinates(atlasResolution, packedFlow, lx, ly, lz);
        }

        private static Vec3 sampleVelocityClamped(BlockPos origin, int sampleStride, int atlasResolution, short[] packedFlow, Vec3 position) {
            double lx = Mth.clamp((position.x - origin.getX()) / sampleStride - 0.5, 0.0, atlasResolution - 1.0);
            double ly = Mth.clamp((position.y - origin.getY()) / sampleStride - 0.5, 0.0, atlasResolution - 1.0);
            double lz = Mth.clamp((position.z - origin.getZ()) / sampleStride - 0.5, 0.0, atlasResolution - 1.0);
            return sampleVelocityAtSampleCoordinates(atlasResolution, packedFlow, lx, ly, lz);
        }

        private static Vec3 sampleVelocityAtSampleCoordinates(int atlasResolution, short[] packedFlow, double lx, double ly, double lz) {
            int x0 = (int) Math.floor(lx);
            int y0 = (int) Math.floor(ly);
            int z0 = (int) Math.floor(lz);
            int x1 = Math.min(atlasResolution - 1, x0 + 1);
            int y1 = Math.min(atlasResolution - 1, y0 + 1);
            int z1 = Math.min(atlasResolution - 1, z0 + 1);
            double fx = x1 == x0 ? 0.0 : lx - x0;
            double fy = y1 == y0 ? 0.0 : ly - y0;
            double fz = z1 == z0 ? 0.0 : lz - z0;
            Vec3 c000 = loadVelocity(atlasResolution, packedFlow, x0, y0, z0);
            Vec3 c100 = loadVelocity(atlasResolution, packedFlow, x1, y0, z0);
            Vec3 c010 = loadVelocity(atlasResolution, packedFlow, x0, y1, z0);
            Vec3 c110 = loadVelocity(atlasResolution, packedFlow, x1, y1, z0);
            Vec3 c001 = loadVelocity(atlasResolution, packedFlow, x0, y0, z1);
            Vec3 c101 = loadVelocity(atlasResolution, packedFlow, x1, y0, z1);
            Vec3 c011 = loadVelocity(atlasResolution, packedFlow, x0, y1, z1);
            Vec3 c111 = loadVelocity(atlasResolution, packedFlow, x1, y1, z1);
            Vec3 c00 = c000.lerp(c100, fx);
            Vec3 c10 = c010.lerp(c110, fx);
            Vec3 c01 = c001.lerp(c101, fx);
            Vec3 c11 = c011.lerp(c111, fx);
            Vec3 c0 = c00.lerp(c10, fy);
            Vec3 c1 = c01.lerp(c11, fy);
            return c0.lerp(c1, fz);
        }

        private static Vec3 loadVelocity(int atlasResolution, short[] packedFlow, int x, int y, int z) {
            int cell = ((x * atlasResolution + y) * atlasResolution + z) * 4;
            return new Vec3(
                    decodeVelocity(packedFlow[cell]),
                    decodeVelocity(packedFlow[cell + 1]),
                    decodeVelocity(packedFlow[cell + 2])
            );
        }

    }

    private record PackedAtmosphereSample(
            float temperatureKelvin,
            float humidity,
            float turbulenceIntensity,
            float gustX,
            float gustY,
            float gustZ,
            float windShearXPerBlock,
            float windShearZPerBlock,
            float ablStability,
            float ablMixingStrength
    ) {
        private static final PackedAtmosphereSample EMPTY = new PackedAtmosphereSample(
                AeroWindSample.UNKNOWN_SCALAR,
                AeroWindSample.UNKNOWN_SCALAR,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.0f,
                0.0f
        );

        PackedAtmosphereSample lerp(PackedAtmosphereSample other, double t) {
            float f = (float) t;
            return new PackedAtmosphereSample(
                    temperatureKelvin + (other.temperatureKelvin - temperatureKelvin) * f,
                    humidity + (other.humidity - humidity) * f,
                    turbulenceIntensity + (other.turbulenceIntensity - turbulenceIntensity) * f,
                    gustX + (other.gustX - gustX) * f,
                    gustY + (other.gustY - gustY) * f,
                    gustZ + (other.gustZ - gustZ) * f,
                    windShearXPerBlock + (other.windShearXPerBlock - windShearXPerBlock) * f,
                    windShearZPerBlock + (other.windShearZPerBlock - windShearZPerBlock) * f,
                    ablStability + (other.ablStability - ablStability) * f,
                    ablMixingStrength + (other.ablMixingStrength - ablMixingStrength) * f
            );
        }
    }

    private static AeroWindSample samplePackedFlow(
            BlockPos origin,
            int sampleStride,
            int sizeX,
            int sizeY,
            int sizeZ,
            short[] packedFlow,
            Vec3 position,
            AeroWindSample.Level level,
            AeroWindSample.Authority authority
    ) {
        double rx = (position.x - origin.getX()) / sampleStride;
        double ry = (position.y - origin.getY()) / sampleStride;
        double rz = (position.z - origin.getZ()) / sampleStride;
        if (rx < 0.0 || ry < 0.0 || rz < 0.0
                || rx >= sizeX || ry >= sizeY || rz >= sizeZ) {
            return AeroWindSample.ZERO;
        }
        double lx = Mth.clamp(rx - 0.5, 0.0, sizeX - 1.0);
        double ly = Mth.clamp(ry - 0.5, 0.0, sizeY - 1.0);
        double lz = Mth.clamp(rz - 0.5, 0.0, sizeZ - 1.0);
        PackedFlowSample sample = samplePackedFlowAtSampleCoordinates(sizeX, sizeY, sizeZ, packedFlow, lx, ly, lz);
        return new AeroWindSample(
                sample.velocityX(),
                sample.velocityY(),
                sample.velocityZ(),
                sample.pressure(),
                level,
                authority,
                AeroWindSample.UNKNOWN_EPOCH,
                AeroWindSample.UNKNOWN_EPOCH,
                AeroWindSample.UNKNOWN_EPOCH,
                1.0f
        );
    }

    private static PackedFlowSample samplePackedFlowAtSampleCoordinates(
            int sizeX,
            int sizeY,
            int sizeZ,
            short[] packedFlow,
            double lx,
            double ly,
            double lz
    ) {
        int x0 = (int) Math.floor(lx);
        int y0 = (int) Math.floor(ly);
        int z0 = (int) Math.floor(lz);
        int x1 = Math.min(sizeX - 1, x0 + 1);
        int y1 = Math.min(sizeY - 1, y0 + 1);
        int z1 = Math.min(sizeZ - 1, z0 + 1);
        double fx = x1 == x0 ? 0.0 : lx - x0;
        double fy = y1 == y0 ? 0.0 : ly - y0;
        double fz = z1 == z0 ? 0.0 : lz - z0;
        PackedFlowSample c000 = loadPackedFlow(sizeX, sizeY, sizeZ, packedFlow, x0, y0, z0);
        PackedFlowSample c100 = loadPackedFlow(sizeX, sizeY, sizeZ, packedFlow, x1, y0, z0);
        PackedFlowSample c010 = loadPackedFlow(sizeX, sizeY, sizeZ, packedFlow, x0, y1, z0);
        PackedFlowSample c110 = loadPackedFlow(sizeX, sizeY, sizeZ, packedFlow, x1, y1, z0);
        PackedFlowSample c001 = loadPackedFlow(sizeX, sizeY, sizeZ, packedFlow, x0, y0, z1);
        PackedFlowSample c101 = loadPackedFlow(sizeX, sizeY, sizeZ, packedFlow, x1, y0, z1);
        PackedFlowSample c011 = loadPackedFlow(sizeX, sizeY, sizeZ, packedFlow, x0, y1, z1);
        PackedFlowSample c111 = loadPackedFlow(sizeX, sizeY, sizeZ, packedFlow, x1, y1, z1);
        PackedFlowSample c00 = c000.lerp(c100, fx);
        PackedFlowSample c10 = c010.lerp(c110, fx);
        PackedFlowSample c01 = c001.lerp(c101, fx);
        PackedFlowSample c11 = c011.lerp(c111, fx);
        PackedFlowSample c0 = c00.lerp(c10, fy);
        PackedFlowSample c1 = c01.lerp(c11, fy);
        return c0.lerp(c1, fz);
    }

    private static PackedFlowSample loadPackedFlow(int sizeX, int sizeY, int sizeZ, short[] packedFlow, int x, int y, int z) {
        int cell = ((x * sizeY + y) * sizeZ + z) * 4;
        if (cell < 0 || cell + 3 >= packedFlow.length) {
            return new PackedFlowSample(0.0f, 0.0f, 0.0f, 0.0f);
        }
        return new PackedFlowSample(
                decodeVelocity(packedFlow[cell]),
                decodeVelocity(packedFlow[cell + 1]),
                decodeVelocity(packedFlow[cell + 2]),
                decodePressure(packedFlow[cell + 3])
        );
    }

    private static PackedAtmosphereSample samplePackedAtmosphere(
            BlockPos origin,
            int sampleStride,
            int sizeX,
            int sizeY,
            int sizeZ,
            short[] packedAtmosphere,
            Vec3 position
    ) {
        double rx = (position.x - origin.getX()) / sampleStride;
        double ry = (position.y - origin.getY()) / sampleStride;
        double rz = (position.z - origin.getZ()) / sampleStride;
        if (rx < 0.0 || ry < 0.0 || rz < 0.0
                || rx >= sizeX || ry >= sizeY || rz >= sizeZ) {
            return PackedAtmosphereSample.EMPTY;
        }
        double lx = Mth.clamp(rx - 0.5, 0.0, sizeX - 1.0);
        double ly = Mth.clamp(ry - 0.5, 0.0, sizeY - 1.0);
        double lz = Mth.clamp(rz - 0.5, 0.0, sizeZ - 1.0);
        return samplePackedAtmosphereAtSampleCoordinates(sizeX, sizeY, sizeZ, packedAtmosphere, lx, ly, lz);
    }

    private static PackedAtmosphereSample samplePackedAtmosphereAtSampleCoordinates(
            int sizeX,
            int sizeY,
            int sizeZ,
            short[] packedAtmosphere,
            double lx,
            double ly,
            double lz
    ) {
        int x0 = (int) Math.floor(lx);
        int y0 = (int) Math.floor(ly);
        int z0 = (int) Math.floor(lz);
        int x1 = Math.min(sizeX - 1, x0 + 1);
        int y1 = Math.min(sizeY - 1, y0 + 1);
        int z1 = Math.min(sizeZ - 1, z0 + 1);
        double fx = x1 == x0 ? 0.0 : lx - x0;
        double fy = y1 == y0 ? 0.0 : ly - y0;
        double fz = z1 == z0 ? 0.0 : lz - z0;
        PackedAtmosphereSample c000 = loadPackedAtmosphere(sizeX, sizeY, sizeZ, packedAtmosphere, x0, y0, z0);
        PackedAtmosphereSample c100 = loadPackedAtmosphere(sizeX, sizeY, sizeZ, packedAtmosphere, x1, y0, z0);
        PackedAtmosphereSample c010 = loadPackedAtmosphere(sizeX, sizeY, sizeZ, packedAtmosphere, x0, y1, z0);
        PackedAtmosphereSample c110 = loadPackedAtmosphere(sizeX, sizeY, sizeZ, packedAtmosphere, x1, y1, z0);
        PackedAtmosphereSample c001 = loadPackedAtmosphere(sizeX, sizeY, sizeZ, packedAtmosphere, x0, y0, z1);
        PackedAtmosphereSample c101 = loadPackedAtmosphere(sizeX, sizeY, sizeZ, packedAtmosphere, x1, y0, z1);
        PackedAtmosphereSample c011 = loadPackedAtmosphere(sizeX, sizeY, sizeZ, packedAtmosphere, x0, y1, z1);
        PackedAtmosphereSample c111 = loadPackedAtmosphere(sizeX, sizeY, sizeZ, packedAtmosphere, x1, y1, z1);
        PackedAtmosphereSample c00 = c000.lerp(c100, fx);
        PackedAtmosphereSample c10 = c010.lerp(c110, fx);
        PackedAtmosphereSample c01 = c001.lerp(c101, fx);
        PackedAtmosphereSample c11 = c011.lerp(c111, fx);
        PackedAtmosphereSample c0 = c00.lerp(c10, fy);
        PackedAtmosphereSample c1 = c01.lerp(c11, fy);
        return c0.lerp(c1, fz);
    }

    private static PackedAtmosphereSample loadPackedAtmosphere(
            int sizeX,
            int sizeY,
            int sizeZ,
            short[] packedAtmosphere,
            int x,
            int y,
            int z
    ) {
        int cell = ((x * sizeY + y) * sizeZ + z) * COARSE_ATMOSPHERE_CHANNELS;
        if (cell < 0 || cell + COARSE_ATMOSPHERE_CHANNELS - 1 >= packedAtmosphere.length) {
            return PackedAtmosphereSample.EMPTY;
        }
        return new PackedAtmosphereSample(
                decodeTemperatureKelvin(packedAtmosphere[cell]),
                decodeUnit01(packedAtmosphere[cell + 1]),
                decodeTurbulence(packedAtmosphere[cell + 2]),
                decodeVelocity(packedAtmosphere[cell + 3]),
                decodeVelocity(packedAtmosphere[cell + 4]),
                decodeVelocity(packedAtmosphere[cell + 5]),
                decodeShear(packedAtmosphere[cell + 6]),
                decodeShear(packedAtmosphere[cell + 7]),
                packedAtmosphere[cell + 8] / 32767.0f,
                decodeUnit01(packedAtmosphere[cell + 9])
        );
    }

    private record FlowVisual(
            float maxSpeed,
            float meanSpeed,
            float meanAbsPressure,
            float meanVx,
            float meanVy,
            float meanVz,
            float dominantVx,
            float dominantVy,
            float dominantVz
    ) {
        static FlowVisual fromPackedFlow(short[] packedFlow) {
            int sampleCount = packedFlow.length / 4;
            if (sampleCount <= 0) {
                return new FlowVisual(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            }
            float sumSpeed = 0.0f;
            float maxSpeed = 0.0f;
            float sumAbsPressure = 0.0f;
            float sumVx = 0.0f;
            float sumVy = 0.0f;
            float sumVz = 0.0f;
            float dominantVx = 0.0f;
            float dominantVy = 0.0f;
            float dominantVz = 0.0f;
            for (int i = 0; i < packedFlow.length; i += 4) {
                float vx = decodeVelocity(packedFlow[i]);
                float vy = decodeVelocity(packedFlow[i + 1]);
                float vz = decodeVelocity(packedFlow[i + 2]);
                float pressure = decodePressure(packedFlow[i + 3]);
                float speed = (float) Math.sqrt(vx * vx + vy * vy + vz * vz);
                sumVx += vx;
                sumVy += vy;
                sumVz += vz;
                sumSpeed += speed;
                sumAbsPressure += Math.abs(pressure);
                if (speed > maxSpeed) {
                    maxSpeed = speed;
                    dominantVx = vx;
                    dominantVy = vy;
                    dominantVz = vz;
                }
            }
            float inv = 1.0f / sampleCount;
            return new FlowVisual(
                    maxSpeed,
                    sumSpeed * inv,
                    sumAbsPressure * inv,
                    sumVx * inv,
                    sumVy * inv,
                    sumVz * inv,
                    dominantVx,
                    dominantVy,
                    dominantVz
            );
        }

        Vec3 averageDirection() {
            double length = Math.sqrt(meanVx * meanVx + meanVy * meanVy + meanVz * meanVz);
            if (length <= 1.0e-6) {
                return Vec3.ZERO;
            }
            return new Vec3(meanVx / length, meanVy / length, meanVz / length);
        }

        Vec3 dominantDirection() {
            double length = Math.sqrt(dominantVx * dominantVx + dominantVy * dominantVy + dominantVz * dominantVz);
            if (length <= 1.0e-6) {
                return Vec3.ZERO;
            }
            return new Vec3(dominantVx / length, dominantVy / length, dominantVz / length);
        }

        Vec3 displayDirection() {
            Vec3 average = averageDirection();
            if (average.lengthSqr() > 1.0e-4) {
                return average;
            }
            return dominantDirection();
        }
    }

    record AeroFlowState(boolean streamingEnabled, boolean renderVelocityVectors, boolean renderStreamlines) {
    }
}
