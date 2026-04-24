package com.aerodynamics4mc.client;

import java.util.HashMap;
import java.util.Map;

import org.joml.Matrix4f;

import com.aerodynamics4mc.flow.AnalysisFlowCodec;
import com.aerodynamics4mc.net.AeroFlowAnalysisPayload;
import com.aerodynamics4mc.net.AeroFlowPayload;
import com.aerodynamics4mc.runtime.NativeSimulationBridge;

import net.fabricmc.fabric.api.client.event.lifecycle.v1.ClientTickEvents;
import net.fabricmc.fabric.api.client.networking.v1.ClientPlayConnectionEvents;
import net.fabricmc.fabric.api.client.rendering.v1.world.WorldRenderContext;
import net.fabricmc.fabric.api.client.rendering.v1.world.WorldRenderEvents;
import net.minecraft.client.MinecraftClient;
import net.minecraft.client.render.DrawStyle;
import net.minecraft.client.render.RenderLayers;
import net.minecraft.client.render.VertexConsumer;
import net.minecraft.client.util.math.MatrixStack;
import net.minecraft.util.Identifier;
import net.minecraft.util.math.BlockPos;
import net.minecraft.util.math.Box;
import net.minecraft.util.math.MathHelper;
import net.minecraft.util.math.Vec3d;
import net.minecraft.world.debug.gizmo.GizmoDrawing;

final class AeroVisualizer {
    private static final float ATLAS_VELOCITY_RANGE = 5.6f;
    private static final float ATLAS_PRESSURE_RANGE = 0.03f;
    private static final double MAX_RENDER_DISTANCE = 192.0;
    private static final double REMOTE_VECTOR_RENDER_DISTANCE = 144.0;
    private static final double ANALYSIS_RENDER_DISTANCE = 192.0;
    private static final int REGION_STALE_TICKS = 40;
    private static final int REMOTE_VECTOR_FIELD_STRIDE = 1;
    private static final int REMOTE_STREAMLINE_SEED_STRIDE = 1;
    private static final int REMOTE_STREAMLINE_FACE_BUCKETS = 4;
    private static final int REMOTE_STREAMLINE_SEEDS_PER_FACE =
        REMOTE_STREAMLINE_FACE_BUCKETS * REMOTE_STREAMLINE_FACE_BUCKETS;
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
    private final Map<WindowKey, AnalysisFlowField> analysisWindows = new HashMap<>();
    private final NativeSimulationBridge analysisCodecBridge = new NativeSimulationBridge();
    private boolean streamingEnabled;
    private boolean renderVelocityVectors = true;
    private boolean renderStreamlines = true;
    private long clientTickCounter;

    void initialize() {
        ClientPlayConnectionEvents.DISCONNECT.register((handler, client) -> clearState());
        ClientTickEvents.END_CLIENT_TICK.register(client -> onClientTick());
        WorldRenderEvents.BEFORE_DEBUG_RENDER.register(this::renderAtlasOverlay);
    }

    void onRuntimeState(AeroFlowState state) {
        streamingEnabled = state.streamingEnabled();
        renderVelocityVectors = state.renderVelocityVectors();
        renderStreamlines = state.renderStreamlines();
        if (!streamingEnabled) {
            remoteWindows.clear();
        }
    }

    void onFlowField(AeroFlowPayload payload) {
        if (!streamingEnabled) {
            return;
        }
        WindowKey key = new WindowKey(payload.dimensionId(), payload.origin());
        remoteWindows.put(key, RemoteFlowField.fromPayload(payload, clientTickCounter));
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
        analysisWindows.clear();
        streamingEnabled = false;
        renderVelocityVectors = true;
        renderStreamlines = true;
    }

    Vec3d sampleWind(Identifier dimensionId, Vec3d position) {
        if (!streamingEnabled || remoteWindows.isEmpty()) {
            return Vec3d.ZERO;
        }
        Vec3d bestVisible = Vec3d.ZERO;
        double bestVisibleSpeedSq = 0.0;
        Vec3d bestFallback = Vec3d.ZERO;
        double bestFallbackSpeedSq = 0.0;
        for (RemoteFlowField field : remoteWindows.values()) {
            if (!field.dimensionId().equals(dimensionId)) {
                continue;
            }
            if (field.visibleBox().contains(position)) {
                Vec3d sampled = field.sampleVelocityTrilinear(position);
                double speedSq = sampled.lengthSquared();
                if (speedSq > bestVisibleSpeedSq) {
                    bestVisible = sampled;
                    bestVisibleSpeedSq = speedSq;
                }
                continue;
            }
            if (field.regionBox().contains(position)) {
                Vec3d sampled = field.sampleVelocityTrilinear(position);
                double speedSq = sampled.lengthSquared();
                if (speedSq > bestFallbackSpeedSq) {
                    bestFallback = sampled;
                    bestFallbackSpeedSq = speedSq;
                }
            }
        }
        return bestVisibleSpeedSq > 0.0 ? bestVisible : bestFallback;
    }

    private void onClientTick() {
        clientTickCounter++;
        if (!streamingEnabled || remoteWindows.isEmpty()) {
            return;
        }
        remoteWindows.entrySet().removeIf(entry -> {
            return clientTickCounter - entry.getValue().lastUpdatedTick() > REGION_STALE_TICKS;
        });
        analysisWindows.entrySet().removeIf(entry -> {
            return clientTickCounter - entry.getValue().lastUpdatedTick() > REGION_STALE_TICKS * 4L;
        });
    }

    private void renderAtlasOverlay(WorldRenderContext context) {
        if (!streamingEnabled || remoteWindows.isEmpty()) {
            return;
        }
        MinecraftClient client = MinecraftClient.getInstance();
        if (client.world == null || client.player == null || !client.getDebugHud().shouldShowDebugHud()) {
            return;
        }
        Identifier dimensionId = client.world.getRegistryKey().getValue();
        Vec3d cameraPos = client.gameRenderer.getCamera().getCameraPos();
        VertexConsumer lineBuffer = context.consumers() == null ? null : context.consumers().getBuffer(RenderLayers.lines());
        MatrixStack matrices = context.matrices();
        try (var ignored = client.newGizmoScope()) {
            for (RemoteFlowField field : remoteWindows.values()) {
                if (!field.dimensionId().equals(dimensionId)) {
                    continue;
                }
                double distanceSq = field.visibleBox().squaredMagnitude(cameraPos);
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

        Vec3d labelPos = new Vec3d(
            field.origin().getX() + field.fullResolution() * 0.5,
            field.origin().getY() + sliceY + 0.9,
            field.origin().getZ() + field.fullResolution() * 0.5
        );
        String label = "CFD slice |v| y=" + (field.origin().getY() + sliceY)
            + " max=" + String.format("%.2f", sliceStats.maxSpeed())
            + " p95=" + String.format("%.2f", sliceStats.p95Speed());
        GizmoDrawing.text(
            label,
            labelPos,
            net.minecraft.world.debug.gizmo.TextGizmo.Style.centered(argb(0.92f, 0.96f, 0.98f, 1.0f)).scaled(0.03f)
        ).ignoreOcclusion().withLifespan(1);
    }

    private AnalysisSliceView prepareAnalysisSlice(MinecraftClient client, Identifier dimensionId, Vec3d cameraPos) {
        AnalysisFlowField field = selectAnalysisField(dimensionId, cameraPos);
        if (field == null) {
            return null;
        }
        double distanceSq = field.regionBox().squaredMagnitude(cameraPos);
        if (distanceSq > ANALYSIS_RENDER_DISTANCE * ANALYSIS_RENDER_DISTANCE) {
            return null;
        }
        double sliceWorldY = client.player != null ? client.player.getY() + ANALYSIS_SLICE_VIEW_OFFSET_Y : cameraPos.y + ANALYSIS_SLICE_VIEW_OFFSET_Y;
        int sliceY = field.sliceIndexForWorldY(sliceWorldY);
        SliceStats sliceStats = field.sliceStats(sliceY);
        float speedRange = Math.max(ANALYSIS_SLICE_MIN_RANGE, sliceStats.colorRange());
        return new AnalysisSliceView(field, sliceY, sliceStats, speedRange);
    }

    private AnalysisFlowField selectAnalysisField(Identifier dimensionId, Vec3d cameraPos) {
        AnalysisFlowField bestContaining = null;
        double bestContainingDistanceSq = Double.POSITIVE_INFINITY;
        AnalysisFlowField bestNearest = null;
        double bestNearestDistanceSq = Double.POSITIVE_INFINITY;
        for (AnalysisFlowField field : analysisWindows.values()) {
            if (!field.dimensionId().equals(dimensionId)) {
                continue;
            }
            Box regionBox = field.regionBox();
            double distanceSq = regionBox.squaredMagnitude(cameraPos);
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
                Vec3d velocity = field.velocity(x, sliceY, z);
                Vec3d horizontal = new Vec3d(velocity.x, 0.0, velocity.z);
                double horizontalSpeed = horizontal.length();
                if (horizontalSpeed < ANALYSIS_SLICE_MIN_SPEED) {
                    continue;
                }
                Vec3d direction = horizontal.multiply(1.0 / horizontalSpeed);
                float speedNorm = Math.max(0.05f, clamp01((float) horizontalSpeed / speedRange));
                double arrowLength = ANALYSIS_SLICE_GLYPH_LENGTH + ANALYSIS_SLICE_GLYPH_LENGTH * speedNorm;
                Vec3d start = new Vec3d(field.origin().getX() + x + 0.5, arrowBaseY, field.origin().getZ() + z + 0.5);
                Vec3d end = start.add(direction.multiply(arrowLength));
                int glyphColor = analysisScalarColor(speedNorm, 0.72f);
                GizmoDrawing.arrow(start, end, glyphColor, ANALYSIS_SLICE_GLYPH_WIDTH).ignoreOcclusion().withLifespan(1);
            }
        }
    }

    private void renderFlowRendererField(VertexConsumer buffer, MatrixStack matrices, RemoteFlowField field, Vec3d cameraPos) {
        if (buffer == null || matrices == null || (!renderVelocityVectors && !renderStreamlines)) {
            return;
        }
        if (renderVelocityVectors) {
            matrices.push();
            matrices.translate(
                field.origin().getX() - cameraPos.x,
                field.origin().getY() - cameraPos.y,
                field.origin().getZ() - cameraPos.z
            );
            renderVelocityField(buffer, matrices, field);
            matrices.pop();
        }
        if (renderStreamlines) {
            matrices.push();
            renderStreamlines(buffer, matrices, field, cameraPos);
            matrices.pop();
        }
    }

    private void renderVelocityField(VertexConsumer buffer, MatrixStack matrices, RemoteFlowField field) {
        int gridSize = field.fullResolution();
        int stride = REMOTE_VECTOR_FIELD_STRIDE;
        float velScale = 1.0f;
        var entry = matrices.peek();
        Matrix4f matrix = entry.getPositionMatrix();

        for (int x = 0; x < gridSize; x += stride) {
            for (int y = 0; y < gridSize; y += stride) {
                for (int z = 0; z < gridSize; z += stride) {
                    Vec3d velocity = field.sampleVelocityLocal(x + 0.5, y + 0.5, z + 0.5);
                    float speed = (float) velocity.length();
                    float speedNorm = MathHelper.clamp(speed / MAX_INFLOW_SPEED, 0.0f, 1.0f);
                    if (speedNorm < RENDER_THRESHOLD_NORMALIZED) {
                        continue;
                    }
                    Vec3d direction = velocity.normalize();
                    float lineLength = MathHelper.clamp(speedNorm * velScale, 0.05f, 10f);
                    int color = getViridisColor(speedNorm * velScale);
                    int r = (color >> 16) & 0xFF;
                    int g = (color >> 8) & 0xFF;
                    int b = color & 0xFF;
                    float fx = x + 0.5f;
                    float fy = y + 0.5f;
                    float fz = z + 0.5f;

                    buffer.vertex(matrix, fx, fy, fz)
                        .color(r, g, b, 255)
                        .normal(entry, (float) direction.x, (float) direction.y, (float) direction.z)
                        .lineWidth(1.2f);
                    buffer.vertex(
                            matrix,
                            fx + (float) direction.x * lineLength,
                            fy + (float) direction.y * lineLength,
                            fz + (float) direction.z * lineLength
                        )
                        .color(r, g, b, 255)
                        .normal(entry, (float) direction.x, (float) direction.y, (float) direction.z)
                        .lineWidth(1.2f);
                }
            }
        }
    }

    private void renderStreamlines(VertexConsumer buffer, MatrixStack matrices, RemoteFlowField field, Vec3d cameraPos) {
        int gridSize = field.fullResolution();
        int seedStride = REMOTE_STREAMLINE_SEED_STRIDE;
        float stepSize = 0.35f;
        int maxSteps = REMOTE_STREAMLINE_MAX_STEPS;
        var entry = matrices.peek();
        Matrix4f matrix = entry.getPositionMatrix();

        renderInflowFaceStreamlines(buffer, entry, matrix, field, cameraPos, gridSize, seedStride, stepSize, maxSteps, 0, -1);
        renderInflowFaceStreamlines(buffer, entry, matrix, field, cameraPos, gridSize, seedStride, stepSize, maxSteps, 0, 1);
        renderInflowFaceStreamlines(buffer, entry, matrix, field, cameraPos, gridSize, seedStride, stepSize, maxSteps, 1, -1);
        renderInflowFaceStreamlines(buffer, entry, matrix, field, cameraPos, gridSize, seedStride, stepSize, maxSteps, 1, 1);
        renderInflowFaceStreamlines(buffer, entry, matrix, field, cameraPos, gridSize, seedStride, stepSize, maxSteps, 2, -1);
        renderInflowFaceStreamlines(buffer, entry, matrix, field, cameraPos, gridSize, seedStride, stepSize, maxSteps, 2, 1);
    }

    private void renderInflowFaceStreamlines(
        VertexConsumer buffer,
        MatrixStack.Entry entry,
        Matrix4f matrix,
        RemoteFlowField field,
        Vec3d cameraPos,
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
                Vec3d velocity = field.sampleVelocityLocal(x, y, z);
                float inward = inwardComponent(velocity, axis, inwardSign);
                if (inward < REMOTE_STREAMLINE_MIN_INFLOW) {
                    continue;
                }
                float speed = (float) velocity.length();
                if (speed < MIN_SPEED) {
                    continue;
                }
                int bucket = streamlineFaceBucket(u, v, gridSize);
                float score = inward * (0.35f + 0.65f * MathHelper.clamp(speed / MAX_INFLOW_SPEED, 0.0f, 1.0f));
                if (score > bestScores[bucket]) {
                    bestScores[bucket] = score;
                    bestX[bucket] = x;
                    bestY[bucket] = y;
                    bestZ[bucket] = z;
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
                    field.localToWorld(new Vec3d(bestX[i], bestY[i], bestZ[i])),
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

    private float inwardComponent(Vec3d velocity, int axis, int inwardSign) {
        double component = axis == 0 ? velocity.x : (axis == 1 ? velocity.y : velocity.z);
        return (float) component * inwardSign;
    }

    private void renderStreamlineFromSeed(
        VertexConsumer buffer,
        MatrixStack.Entry entry,
        Matrix4f matrix,
        RemoteFlowField field,
        Vec3d seed,
        Vec3d cameraPos,
        float stepSize,
        int maxSteps
    ) {
        Vec3d pos = seed;
        RemoteFlowField currentField = field;
        double traveled = 0.0;
        for (int step = 0; step < maxSteps; step++) {
            FlowSample sample = sampleGlobalVelocity(field.dimensionId(), pos, currentField);
            if (sample == null) {
                break;
            }
            currentField = sample.field();
            Vec3d velocity = sample.velocity();
            float speed = (float) velocity.length();
            if (speed < MIN_SPEED) {
                break;
            }

            float speedNorm = MathHelper.clamp(speed / MAX_INFLOW_SPEED, 0.0f, 1.0f);
            if (speedNorm < RENDER_THRESHOLD_NORMALIZED) {
                break;
            }

            Vec3d dir = velocity.normalize();
            float advectStep = stepSize * MathHelper.clamp(speedNorm * 8.0f, 0.2f, 1.25f);
            Vec3d nextPos = pos.add(dir.multiply(advectStep));
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
            Vec3d segDir = nextPos.subtract(pos).normalize();

            buffer.vertex(
                    matrix,
                    (float) (pos.x - cameraPos.x),
                    (float) (pos.y - cameraPos.y),
                    (float) (pos.z - cameraPos.z)
                )
                .color(r, g, b, 255)
                .normal(entry, (float) segDir.x, (float) segDir.y, (float) segDir.z)
                .lineWidth(1.0f);
            buffer.vertex(
                    matrix,
                    (float) (nextPos.x - cameraPos.x),
                    (float) (nextPos.y - cameraPos.y),
                    (float) (nextPos.z - cameraPos.z)
                )
                .color(r, g, b, 255)
                .normal(entry, (float) segDir.x, (float) segDir.y, (float) segDir.z)
                .lineWidth(1.0f);

            pos = nextPos;
            currentField = nextField;
            traveled += segmentLength;
        }
    }

    private boolean hasNeighborAcrossFace(RemoteFlowField field, int axis, int side) {
        int offset = side * field.fullResolution();
        BlockPos neighborOrigin = switch (axis) {
            case 0 -> field.origin().add(offset, 0, 0);
            case 1 -> field.origin().add(0, offset, 0);
            default -> field.origin().add(0, 0, offset);
        };
        return remoteWindows.containsKey(new WindowKey(field.dimensionId(), neighborOrigin));
    }

    private FlowSample sampleGlobalVelocity(Identifier dimensionId, Vec3d worldPos, RemoteFlowField preferredField) {
        RemoteFlowField field = findFieldForPosition(dimensionId, worldPos, preferredField);
        if (field == null) {
            return null;
        }
        return new FlowSample(field.sampleVelocityTrilinearClamped(worldPos), field);
    }

    private RemoteFlowField findFieldForPosition(Identifier dimensionId, Vec3d worldPos, RemoteFlowField preferredField) {
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

    private RemoteFlowField findAlignedFieldForPosition(Identifier dimensionId, Vec3d worldPos, RemoteFlowField referenceField) {
        int size = Math.max(1, referenceField.fullResolution());
        int x = referenceField.origin().getX() + Math.floorDiv((int) Math.floor(worldPos.x) - referenceField.origin().getX(), size) * size;
        int y = referenceField.origin().getY() + Math.floorDiv((int) Math.floor(worldPos.y) - referenceField.origin().getY(), size) * size;
        int z = referenceField.origin().getZ() + Math.floorDiv((int) Math.floor(worldPos.z) - referenceField.origin().getZ(), size) * size;
        RemoteFlowField aligned = remoteWindows.get(new WindowKey(dimensionId, new BlockPos(x, y, z)));
        return aligned != null && containsHalfOpen(aligned.regionBox(), worldPos) ? aligned : null;
    }

    private boolean containsHalfOpen(Box box, Vec3d position) {
        return position.x >= box.minX && position.x < box.maxX
            && position.y >= box.minY && position.y < box.maxY
            && position.z >= box.minZ && position.z < box.maxZ;
    }

    private int getViridisColor(float t) {
        t = MathHelper.clamp(t, 0.0f, 1.0f);
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
        DrawStyle style = DrawStyle.filledAndStroked(strokeColor, 1.0f + 0.6f * maxSpeedNorm, fillColor);
        GizmoDrawing.box(field.visibleBox(), style).ignoreOcclusion().withLifespan(1);
    }

    private static float decodeVelocity(short value) {
        return value / 32767.0f * ATLAS_VELOCITY_RANGE;
    }

    private static float decodePressure(short value) {
        return value / 32767.0f * ATLAS_PRESSURE_RANGE;
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
            c0 = new float[] {0.267f, 0.005f, 0.329f};
            c1 = new float[] {0.283f, 0.141f, 0.458f};
            u = t / 0.25f;
        } else if (t < 0.50f) {
            c0 = new float[] {0.283f, 0.141f, 0.458f};
            c1 = new float[] {0.254f, 0.265f, 0.530f};
            u = (t - 0.25f) / 0.25f;
        } else if (t < 0.75f) {
            c0 = new float[] {0.254f, 0.265f, 0.530f};
            c1 = new float[] {0.207f, 0.372f, 0.553f};
            u = (t - 0.50f) / 0.25f;
        } else {
            c0 = new float[] {0.207f, 0.372f, 0.553f};
            c1 = new float[] {0.993f, 0.906f, 0.144f};
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
        Box regionBox() {
            return new Box(
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

        Vec3d velocity(int x, int y, int z) {
            int index = (((x * fullResolution) + y) * fullResolution + z) * 4;
            if (index < 0 || index + 2 >= flowState.length) {
                return Vec3d.ZERO;
            }
            return new Vec3d(flowState[index], flowState[index + 1], flowState[index + 2]);
        }

        float pressure(int x, int y, int z) {
            int index = (((x * fullResolution) + y) * fullResolution + z) * 4 + 3;
            if (index < 0 || index >= flowState.length) {
                return 0.0f;
            }
            return flowState[index];
        }

        float speed(int x, int y, int z) {
            Vec3d velocity = velocity(x, y, z);
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

    private record FlowSample(Vec3d velocity, RemoteFlowField field) {
    }

    private record RemoteFlowField(
        Identifier dimensionId,
        BlockPos origin,
        int sampleStride,
        int atlasResolution,
        short[] packedFlow,
        FlowVisual visual,
        long lastUpdatedTick
    ) {
        static RemoteFlowField fromPayload(AeroFlowPayload payload, long lastUpdatedTick) {
            int sampleCount = payload.packedFlow().length / 4;
            int atlasResolution = Math.max(1, Math.round((float) Math.cbrt(sampleCount)));
            return new RemoteFlowField(
                payload.dimensionId(),
                payload.origin(),
                payload.sampleStride(),
                atlasResolution,
                payload.packedFlow(),
                FlowVisual.fromPackedFlow(payload.packedFlow()),
                lastUpdatedTick
            );
        }

        Vec3d sampleVelocity(Vec3d position) {
            double lx = (position.x - origin.getX()) / sampleStride;
            double ly = (position.y - origin.getY()) / sampleStride;
            double lz = (position.z - origin.getZ()) / sampleStride;
            int ix = (int) Math.round(lx - 0.5);
            int iy = (int) Math.round(ly - 0.5);
            int iz = (int) Math.round(lz - 0.5);
            if (ix < 0 || iy < 0 || iz < 0 || ix >= atlasResolution || iy >= atlasResolution || iz >= atlasResolution) {
                return Vec3d.ZERO;
            }
            int cell = ((ix * atlasResolution + iy) * atlasResolution + iz) * 4;
            if (cell + 3 >= packedFlow.length) {
                return Vec3d.ZERO;
            }
            return new Vec3d(
                decodeVelocity(packedFlow[cell]),
                decodeVelocity(packedFlow[cell + 1]),
                decodeVelocity(packedFlow[cell + 2])
            );
        }

        Vec3d sampleVelocityTrilinear(Vec3d position) {
            return sampleVelocity(origin, sampleStride, atlasResolution, packedFlow, position);
        }

        Vec3d sampleVelocityTrilinearClamped(Vec3d position) {
            return sampleVelocityClamped(origin, sampleStride, atlasResolution, packedFlow, position);
        }

        Vec3d sampleVelocityLocal(double x, double y, double z) {
            return sampleVelocityTrilinear(localToWorld(new Vec3d(x, y, z)));
        }

        Vec3d localToWorld(Vec3d local) {
            return new Vec3d(origin.getX() + local.x, origin.getY() + local.y, origin.getZ() + local.z);
        }

        Box regionBox() {
            double size = (double) atlasResolution * (double) sampleStride;
            return new Box(
                origin.getX(), origin.getY(), origin.getZ(),
                origin.getX() + size, origin.getY() + size, origin.getZ() + size
            );
        }

        Box visibleBox() {
            return regionBox();
        }

        int fullResolution() {
            return atlasResolution * sampleStride;
        }

        int sliceIndexForWorldY(double worldY) {
            int local = (int) Math.floor(worldY - origin.getY());
            return Math.max(0, Math.min(fullResolution() - 1, local));
        }

        Vec3d velocity(int x, int y, int z) {
            return sampleVelocityTrilinear(new Vec3d(
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

        private static Vec3d sampleVelocity(BlockPos origin, int sampleStride, int atlasResolution, short[] packedFlow, Vec3d position) {
            double lx = (position.x - origin.getX()) / sampleStride - 0.5;
            double ly = (position.y - origin.getY()) / sampleStride - 0.5;
            double lz = (position.z - origin.getZ()) / sampleStride - 0.5;
            if (lx < 0.0 || ly < 0.0 || lz < 0.0
                || lx > atlasResolution - 1.0 || ly > atlasResolution - 1.0 || lz > atlasResolution - 1.0) {
                return Vec3d.ZERO;
            }
            return sampleVelocityAtSampleCoordinates(atlasResolution, packedFlow, lx, ly, lz);
        }

        private static Vec3d sampleVelocityClamped(BlockPos origin, int sampleStride, int atlasResolution, short[] packedFlow, Vec3d position) {
            double lx = MathHelper.clamp((position.x - origin.getX()) / sampleStride - 0.5, 0.0, atlasResolution - 1.0);
            double ly = MathHelper.clamp((position.y - origin.getY()) / sampleStride - 0.5, 0.0, atlasResolution - 1.0);
            double lz = MathHelper.clamp((position.z - origin.getZ()) / sampleStride - 0.5, 0.0, atlasResolution - 1.0);
            return sampleVelocityAtSampleCoordinates(atlasResolution, packedFlow, lx, ly, lz);
        }

        private static Vec3d sampleVelocityAtSampleCoordinates(int atlasResolution, short[] packedFlow, double lx, double ly, double lz) {
            int x0 = (int) Math.floor(lx);
            int y0 = (int) Math.floor(ly);
            int z0 = (int) Math.floor(lz);
            int x1 = Math.min(atlasResolution - 1, x0 + 1);
            int y1 = Math.min(atlasResolution - 1, y0 + 1);
            int z1 = Math.min(atlasResolution - 1, z0 + 1);
            double fx = x1 == x0 ? 0.0 : lx - x0;
            double fy = y1 == y0 ? 0.0 : ly - y0;
            double fz = z1 == z0 ? 0.0 : lz - z0;
            Vec3d c000 = loadVelocity(atlasResolution, packedFlow, x0, y0, z0);
            Vec3d c100 = loadVelocity(atlasResolution, packedFlow, x1, y0, z0);
            Vec3d c010 = loadVelocity(atlasResolution, packedFlow, x0, y1, z0);
            Vec3d c110 = loadVelocity(atlasResolution, packedFlow, x1, y1, z0);
            Vec3d c001 = loadVelocity(atlasResolution, packedFlow, x0, y0, z1);
            Vec3d c101 = loadVelocity(atlasResolution, packedFlow, x1, y0, z1);
            Vec3d c011 = loadVelocity(atlasResolution, packedFlow, x0, y1, z1);
            Vec3d c111 = loadVelocity(atlasResolution, packedFlow, x1, y1, z1);
            Vec3d c00 = c000.lerp(c100, fx);
            Vec3d c10 = c010.lerp(c110, fx);
            Vec3d c01 = c001.lerp(c101, fx);
            Vec3d c11 = c011.lerp(c111, fx);
            Vec3d c0 = c00.lerp(c10, fy);
            Vec3d c1 = c01.lerp(c11, fy);
            return c0.lerp(c1, fz);
        }

        private static Vec3d loadVelocity(int atlasResolution, short[] packedFlow, int x, int y, int z) {
            int cell = ((x * atlasResolution + y) * atlasResolution + z) * 4;
            return new Vec3d(
                decodeVelocity(packedFlow[cell]),
                decodeVelocity(packedFlow[cell + 1]),
                decodeVelocity(packedFlow[cell + 2])
            );
        }

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

        Vec3d averageDirection() {
            double length = Math.sqrt(meanVx * meanVx + meanVy * meanVy + meanVz * meanVz);
            if (length <= 1.0e-6) {
                return Vec3d.ZERO;
            }
            return new Vec3d(meanVx / length, meanVy / length, meanVz / length);
        }

        Vec3d dominantDirection() {
            double length = Math.sqrt(dominantVx * dominantVx + dominantVy * dominantVy + dominantVz * dominantVz);
            if (length <= 1.0e-6) {
                return Vec3d.ZERO;
            }
            return new Vec3d(dominantVx / length, dominantVy / length, dominantVz / length);
        }

        Vec3d displayDirection() {
            Vec3d average = averageDirection();
            if (average.lengthSquared() > 1.0e-4) {
                return average;
            }
            return dominantDirection();
        }
    }

    record AeroFlowState(boolean streamingEnabled, boolean renderVelocityVectors, boolean renderStreamlines) {
    }
}
