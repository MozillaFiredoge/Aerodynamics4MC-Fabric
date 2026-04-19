package com.aerodynamics4mc.client;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.joml.Matrix4f;
import org.joml.Vector3f;
import org.joml.Vector4f;

import com.aerodynamics4mc.ModBlocks;
import com.aerodynamics4mc.flow.AnalysisFlowCodec;
import com.aerodynamics4mc.net.AeroFlowAnalysisPayload;
import com.aerodynamics4mc.net.AeroFlowPayload;
import com.aerodynamics4mc.runtime.NativeSimulationBridge;
import com.mojang.blaze3d.pipeline.BlendFunction;
import com.mojang.blaze3d.pipeline.RenderPipeline;
import com.mojang.blaze3d.platform.DepthTestFunction;
import com.mojang.blaze3d.systems.CommandEncoder;
import com.mojang.blaze3d.systems.RenderPass;
import com.mojang.blaze3d.systems.RenderSystem;
import com.mojang.blaze3d.vertex.VertexFormat;
import com.mojang.blaze3d.vertex.VertexFormat.DrawMode;

import net.fabricmc.fabric.api.client.event.lifecycle.v1.ClientTickEvents;
import net.fabricmc.fabric.api.client.networking.v1.ClientPlayConnectionEvents;
import net.fabricmc.fabric.api.client.rendering.v1.world.WorldRenderContext;
import net.fabricmc.fabric.api.client.rendering.v1.world.WorldRenderEvents;
import net.minecraft.client.MinecraftClient;
import net.minecraft.client.gl.DynamicUniforms;
import net.minecraft.client.gl.Framebuffer;
import net.minecraft.client.gl.UniformType;
import net.minecraft.client.render.BufferBuilder;
import net.minecraft.client.render.BuiltBuffer;
import net.minecraft.client.render.DrawStyle;
import net.minecraft.client.render.Tessellator;
import net.minecraft.client.render.VertexFormats;
import net.minecraft.util.Identifier;
import net.minecraft.util.math.BlockPos;
import net.minecraft.util.math.Box;
import net.minecraft.util.math.Vec3d;
import net.minecraft.world.debug.gizmo.GizmoDrawing;

final class AeroVisualizer {
    static final Identifier WIND_TRAIL_SHADER_ID = Identifier.of(ModBlocks.MOD_ID, "core/wind_trail");
    static final Identifier ANALYSIS_SLICE_SHADER_ID = Identifier.of(ModBlocks.MOD_ID, "core/analysis_slice");

    private static final float ATLAS_VELOCITY_RANGE = 5.6f;
    private static final float ATLAS_PRESSURE_RANGE = 0.03f;
    private static final double MAX_RENDER_DISTANCE = 192.0;
    private static final double GLYPH_RENDER_DISTANCE = 144.0;
    private static final double TRACER_RENDER_DISTANCE = 168.0;
    private static final double ANALYSIS_RENDER_DISTANCE = 192.0;
    private static final int REGION_HALO_CELLS = 16;
    private static final int REGION_STALE_TICKS = 40;
    private static final int GLYPH_ATLAS_STEP = 1;
    private static final int MAX_GLYPHS_PER_REGION = 48;
    private static final float GLYPH_MIN_SPEED = 0.02f;
    private static final int GLYPH_BUCKETS_PER_AXIS = 3;
    private static final int MAX_STREAMLINES_PER_REGION = 5;
    private static final float STREAMLINE_STEP_LENGTH = 3.8f;
    private static final int STREAMLINE_MAX_STEPS_PER_DIRECTION = 20;
    private static final double STREAMLINE_MAX_LENGTH = 46.0;
    private static final float STREAMLINE_MIN_SPEED = 0.05f;
    private static final float TUBE_WIDTH = 0.24f;
    private static final float TUBE_BASE_WIDTH = 0.06f;
    private static final int ANALYSIS_SLICE_GLYPH_STEP = 2;
    private static final float ANALYSIS_SLICE_MIN_SPEED = 0.005f;
    private static final float ANALYSIS_SLICE_ALPHA = 0.8f;
    private static final float ANALYSIS_SLICE_HEIGHT_OFFSET = 0.045f;
    private static final float ANALYSIS_SLICE_MIN_RANGE = 0.04f;
    private static final float ANALYSIS_SLICE_GLYPH_LENGTH = 1.4f;
    private static final float ANALYSIS_SLICE_GLYPH_WIDTH = 1.0f;
    private static final double ANALYSIS_SLICE_VIEW_OFFSET_Y = -1.25;
    private static final RenderPipeline WIND_TRAIL_PIPELINE = RenderPipeline.builder()
        .withLocation(Identifier.of(ModBlocks.MOD_ID, "wind_trail_pipeline"))
        .withVertexShader(WIND_TRAIL_SHADER_ID)
        .withFragmentShader(WIND_TRAIL_SHADER_ID)
        .withUniform("DynamicTransforms", UniformType.UNIFORM_BUFFER)
        .withUniform("Projection", UniformType.UNIFORM_BUFFER)
        .withBlend(BlendFunction.TRANSLUCENT)
        .withDepthTestFunction(DepthTestFunction.LEQUAL_DEPTH_TEST)
        .withCull(false)
        .withDepthWrite(false)
        .withVertexFormat(VertexFormats.POSITION_TEXTURE_COLOR, DrawMode.QUADS)
        .build();
    private static final RenderPipeline ANALYSIS_SLICE_PIPELINE = RenderPipeline.builder()
        .withLocation(Identifier.of(ModBlocks.MOD_ID, "analysis_slice_pipeline"))
        .withVertexShader(ANALYSIS_SLICE_SHADER_ID)
        .withFragmentShader(ANALYSIS_SLICE_SHADER_ID)
        .withUniform("DynamicTransforms", UniformType.UNIFORM_BUFFER)
        .withUniform("Projection", UniformType.UNIFORM_BUFFER)
        .withBlend(BlendFunction.TRANSLUCENT)
        .withDepthTestFunction(DepthTestFunction.NO_DEPTH_TEST)
        .withCull(false)
        .withDepthWrite(false)
        .withVertexFormat(VertexFormats.POSITION_COLOR, DrawMode.QUADS)
        .build();

    private final Map<WindowKey, RemoteFlowField> remoteWindows = new HashMap<>();
    private final Map<WindowKey, AnalysisFlowField> analysisWindows = new HashMap<>();
    private final NativeSimulationBridge analysisCodecBridge = new NativeSimulationBridge();
    private boolean streamingEnabled;
    private long clientTickCounter;

    void initialize() {
        ClientPlayConnectionEvents.DISCONNECT.register((handler, client) -> clearState());
        ClientTickEvents.END_CLIENT_TICK.register(client -> onClientTick());
        WorldRenderEvents.BEFORE_TRANSLUCENT.register(this::renderAnalysisSlicePass);
        WorldRenderEvents.BEFORE_TRANSLUCENT.register(this::renderWindTrailsPass);
        WorldRenderEvents.BEFORE_DEBUG_RENDER.register(this::renderAtlasOverlay);
    }

    void onRuntimeState(AeroFlowState state) {
        streamingEnabled = state.streamingEnabled();
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
        try (var ignored = client.newGizmoScope()) {
            for (RemoteFlowField field : remoteWindows.values()) {
                if (!field.dimensionId().equals(dimensionId)) {
                    continue;
                }
                double distanceSq = field.visibleBox().squaredMagnitude(cameraPos);
                if (distanceSq > MAX_RENDER_DISTANCE * MAX_RENDER_DISTANCE) {
                    continue;
                }
                renderRegionOverlay(field, distanceSq);
            }
            AnalysisSliceView analysisSlice = prepareAnalysisSlice(client, dimensionId, cameraPos);
            if (analysisSlice != null) {
                renderAnalysisOverlay(analysisSlice);
            }
        }
    }

    private void renderAnalysisSlicePass(WorldRenderContext context) {
        if (!streamingEnabled || analysisWindows.isEmpty()) {
            return;
        }
        MinecraftClient client = MinecraftClient.getInstance();
        if (client.world == null || client.player == null || !client.getDebugHud().shouldShowDebugHud()) {
            return;
        }
        Identifier dimensionId = client.world.getRegistryKey().getValue();
        Vec3d cameraPos = client.gameRenderer.getCamera().getCameraPos();
        AnalysisSliceView analysisSlice = prepareAnalysisSlice(client, dimensionId, cameraPos);
        if (analysisSlice == null) {
            return;
        }
        BufferBuilder buffer = Tessellator.getInstance().begin(DrawMode.QUADS, VertexFormats.POSITION_COLOR);
        int quadCount = appendAnalysisSliceQuads(buffer, analysisSlice, cameraPos);
        if (quadCount <= 0) {
            return;
        }
        try (BuiltBuffer built = buffer.end()) {
            renderBuiltAnalysisSlice(client, built);
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

    private int appendAnalysisSliceQuads(BufferBuilder buffer, AnalysisSliceView analysisSlice, Vec3d cameraPos) {
        AnalysisFlowField field = analysisSlice.field();
        int sliceY = analysisSlice.sliceY();
        float speedRange = analysisSlice.speedRange();
        int resolution = field.fullResolution();
        double y = field.origin().getY() + sliceY + ANALYSIS_SLICE_HEIGHT_OFFSET;
        int quadCount = 0;
        for (int x = 0; x < resolution; x++) {
            for (int z = 0; z < resolution; z++) {
                float speed = field.speed(x, sliceY, z);
                float speedNorm = Math.max(0.05f, clamp01(speed / speedRange));
                int fillColor = analysisScalarColor(speedNorm, ANALYSIS_SLICE_ALPHA);
                if ((fillColor >>> 24) == 0) {
                    continue;
                }
                float x0 = field.origin().getX() + x;
                float z0 = field.origin().getZ() + z;
                float x1 = x0 + 1.0f;
                float z1 = z0 + 1.0f;
                float py = (float) (y - cameraPos.y);
                buffer.vertex((float) (x0 - cameraPos.x), py, (float) (z0 - cameraPos.z)).color(fillColor);
                buffer.vertex((float) (x1 - cameraPos.x), py, (float) (z0 - cameraPos.z)).color(fillColor);
                buffer.vertex((float) (x1 - cameraPos.x), py, (float) (z1 - cameraPos.z)).color(fillColor);
                buffer.vertex((float) (x0 - cameraPos.x), py, (float) (z1 - cameraPos.z)).color(fillColor);
                quadCount++;
            }
        }
        return quadCount;
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

    private void renderWindTrailsPass(WorldRenderContext context) {
        if (!streamingEnabled || remoteWindows.isEmpty()) {
            return;
        }
        MinecraftClient client = MinecraftClient.getInstance();
        if (client.world == null || client.player == null) {
            return;
        }
        Identifier dimensionId = client.world.getRegistryKey().getValue();
        Vec3d cameraPos = client.gameRenderer.getCamera().getCameraPos();
        BufferBuilder buffer = Tessellator.getInstance().begin(DrawMode.QUADS, VertexFormats.POSITION_TEXTURE_COLOR);
        int quadCount = 0;
        float timeShift = (clientTickCounter + client.getRenderTickCounter().getTickProgress(true)) * -0.05f;
        for (Map.Entry<WindowKey, RemoteFlowField> entry : remoteWindows.entrySet()) {
            RemoteFlowField field = entry.getValue();
            if (!field.dimensionId().equals(dimensionId)) {
                continue;
            }
            double distanceSq = field.visibleBox().squaredMagnitude(cameraPos);
            if (distanceSq > TRACER_RENDER_DISTANCE * TRACER_RENDER_DISTANCE) {
                continue;
            }
            FlowStreamline[] streamlines = field.streamlines();
            if (streamlines.length == 0) {
                continue;
            }
            float distanceNorm = clamp01((float) (Math.sqrt(distanceSq) / TRACER_RENDER_DISTANCE));
            float distanceFade = 1.0f - smoothstep(0.4f, 1.0f, distanceNorm);
            for (FlowStreamline streamline : streamlines) {
                if (streamline.points().length < 2) {
                    continue;
                }
                quadCount += appendStreamTube(buffer, streamline, distanceFade, cameraPos, timeShift);
            }
        }
        if (quadCount <= 0) {
            return;
        }
        try (BuiltBuffer built = buffer.end()) {
            renderBuiltWindTrails(client, built);
        }
    }

    private void renderRegionOverlay(RemoteFlowField field, double distanceSq) {
        FlowVisual visual = field.visual();
        Vec3d regionDirection = visual.displayDirection();
        float maxSpeedNorm = clamp01(visual.maxSpeed() / 1.5f);
        float meanSpeedNorm = clamp01(visual.meanSpeed() / 1.0f);
        float pressureNorm = clamp01(visual.meanAbsPressure() / 0.01f);
        float distanceNorm = clamp01((float) (Math.sqrt(distanceSq) / MAX_RENDER_DISTANCE));
        float distanceFade = 1.0f - smoothstep(0.20f, 1.0f, distanceNorm);
        float structureFade = 0.25f + 0.75f * distanceFade;

        int strokeColor = flowColor(
            regionDirection,
            maxSpeedNorm,
            pressureNorm,
            (0.18f + 0.26f * maxSpeedNorm + 0.06f * pressureNorm) * structureFade,
            0.02f
        );
        int fillColor = flowColor(
            regionDirection,
            meanSpeedNorm,
            pressureNorm,
            (0.010f + 0.018f * pressureNorm + 0.012f * meanSpeedNorm) * structureFade,
            -0.16f
        );
        DrawStyle style = DrawStyle.filledAndStroked(strokeColor, 1.1f + 0.8f * maxSpeedNorm, fillColor);
        GizmoDrawing.box(field.visibleBox(), style).ignoreOcclusion().withLifespan(1);

        if (regionDirection.lengthSquared() <= 1.0e-6 || visual.maxSpeed() <= 0.05f) {
            return;
        }
        double arrowLength = 5.0 + 18.0 * maxSpeedNorm * (0.65 + 0.35 * distanceFade);
        Vec3d start = field.visibleBox().getCenter();
        Vec3d end = start.add(regionDirection.multiply(arrowLength));
        int arrowColor = flowColor(
            regionDirection,
            maxSpeedNorm,
            pressureNorm,
            (0.28f + 0.30f * maxSpeedNorm) * structureFade,
            0.10f
        );
        GizmoDrawing.arrow(start, end, arrowColor, 1.8f).ignoreOcclusion().withLifespan(1);

        if (distanceSq > GLYPH_RENDER_DISTANCE * GLYPH_RENDER_DISTANCE) {
            return;
        }
        for (FlowGlyph glyph : field.glyphs()) {
            float glyphSpeedNorm = clamp01(glyph.speed() / 1.4f);
            float glyphPressureNorm = clamp01(Math.abs(glyph.pressure()) / 0.01f);
            int glyphColor = flowColor(
                glyph.direction(),
                glyphSpeedNorm,
                glyphPressureNorm,
                (0.22f + 0.24f * glyphSpeedNorm) * distanceFade,
                0.14f
            );
            double glyphLength = 2.0 + 5.5 * glyphSpeedNorm;
            Vec3d glyphEnd = glyph.start().add(glyph.direction().multiply(glyphLength));
            GizmoDrawing.arrow(glyph.start(), glyphEnd, glyphColor, 1.1f).ignoreOcclusion().withLifespan(1);
            if (distanceSq <= TRACER_RENDER_DISTANCE * TRACER_RENDER_DISTANCE) {
                renderGlyphTracer(glyph, glyphSpeedNorm);
            }
        }
    }

    private void renderGlyphTracer(FlowGlyph glyph, float glyphSpeedNorm) {
        double timeSeconds = System.nanoTime() * 1.0e-9;
        double phase = (glyph.phaseOffset() + timeSeconds * (0.25 + glyph.speed() * 0.12)) % 1.0;
        double sweepDistance = 1.0 + 5.0 * glyphSpeedNorm;
        double tracerOffset = (phase - 0.5) * sweepDistance;
        double tracerLength = 0.6 + 2.2 * glyphSpeedNorm;
        Vec3d tracerHead = glyph.start().add(glyph.direction().multiply(tracerOffset));
        Vec3d tracerTail = tracerHead.subtract(glyph.direction().multiply(tracerLength));
        int tracerColor = flowColor(
            glyph.direction(),
            glyphSpeedNorm,
            clamp01(Math.abs(glyph.pressure()) / 0.01f),
            0.30f + 0.28f * glyphSpeedNorm,
            0.24f
        );
        GizmoDrawing.line(tracerTail, tracerHead, tracerColor, 1.0f).ignoreOcclusion().withLifespan(1);
        GizmoDrawing.point(tracerHead, tracerColor, 3.0f + 2.0f * glyphSpeedNorm).ignoreOcclusion().withLifespan(1);
    }

    private int appendStreamTube(BufferBuilder buffer, FlowStreamline streamline, float distanceFade, Vec3d cameraPos, float timeShift) {
        Vec3d[] points = streamline.points();
        if (points.length < 2) {
            return 0;
        }
        int quads = 0;
        double lineLength = streamline.length();
        if (lineLength <= 0.05) {
            return 0;
        }
        Vec3d head = points[points.length - 1];
        Vec3d tail = points[0];
        Vec3d overall = head.subtract(tail);
        Vec3d overallDirection = overall.lengthSquared() > 1.0e-5 ? overall.normalize() : new Vec3d(0.0, 1.0, 0.0);
        float speedNorm = clamp01(streamline.seedSpeed() / 1.4f);
        int color = flowColor(
            overallDirection,
            speedNorm,
            clamp01(Math.abs(streamline.pressure()) / 0.01f),
            (0.90f + 0.52f * speedNorm) * distanceFade,
            0.42f
        );
        double accumulated = 0.0;
        int maxSegments = points.length;
        for (int i = 0; i < maxSegments - 1; i++) {
            Vec3d curr = points[i];
            Vec3d next = points[i + 1];
            Vec3d forward = next.subtract(curr);
            if (forward.lengthSquared() < 1.0e-5) {
                continue;
            }
            forward = forward.normalize();
            Vec3d worldUp = Math.abs(forward.y) > 0.92 ? new Vec3d(1.0, 0.0, 0.0) : new Vec3d(0.0, 1.0, 0.0);
            Vec3d side = forward.crossProduct(worldUp);
            if (side.lengthSquared() <= 1.0e-6) {
                side = forward.crossProduct(new Vec3d(0.0, 0.0, 1.0));
            }
            if (side.lengthSquared() <= 1.0e-6) {
                continue;
            }
            side = side.normalize();
            Vec3d up = side.crossProduct(forward);
            if (up.lengthSquared() <= 1.0e-6) {
                continue;
            }
            up = up.normalize();
            double progressCurr = accumulated / lineLength;
            double segmentLength = curr.distanceTo(next);
            double progressNext = (accumulated + segmentLength) / lineLength;
            double widthCurr = (TUBE_BASE_WIDTH + TUBE_WIDTH * (0.55 + 0.45 * speedNorm)) * (0.78 + 0.22 * Math.sqrt(Math.max(0.0, 1.0 - progressCurr)));
            double widthNext = (TUBE_BASE_WIDTH + TUBE_WIDTH * (0.55 + 0.45 * speedNorm)) * (0.78 + 0.22 * Math.sqrt(Math.max(0.0, 1.0 - progressNext)));
            float flowPhase = timeShift * (0.70f + 0.55f * speedNorm) + streamline.phaseOffset();
            float u1 = (float) (progressCurr * 2.1 + flowPhase);
            float u2 = (float) (progressNext * 2.1 + flowPhase);
            quads += appendTubeQuad(buffer, curr, next, side, widthCurr, widthNext, cameraPos, u1, u2, color);
            quads += appendTubeQuad(buffer, curr, next, up, widthCurr * 0.85, widthNext * 0.85, cameraPos, u1, u2, color);
            accumulated += segmentLength;
        }
        return quads;
    }

    private int appendTubeQuad(
        BufferBuilder buffer,
        Vec3d curr,
        Vec3d next,
        Vec3d axis,
        double widthCurr,
        double widthNext,
        Vec3d cameraPos,
        float u1,
        float u2,
        int color
    ) {
        Vec3d v1 = curr.add(axis.multiply(widthCurr)).subtract(cameraPos);
        Vec3d v2 = curr.subtract(axis.multiply(widthCurr)).subtract(cameraPos);
        Vec3d v3 = next.subtract(axis.multiply(widthNext)).subtract(cameraPos);
        Vec3d v4 = next.add(axis.multiply(widthNext)).subtract(cameraPos);
        putTrailVertex(buffer, v1, u1, 1.0f, color);
        putTrailVertex(buffer, v2, u1, 0.0f, color);
        putTrailVertex(buffer, v3, u2, 0.0f, color);
        putTrailVertex(buffer, v4, u2, 1.0f, color);
        return 1;
    }

    private void putTrailVertex(BufferBuilder buffer, Vec3d position, float u, float v, int color) {
        buffer.vertex((float) position.x, (float) position.y, (float) position.z)
            .texture(u, v)
            .color(color);
    }

    private void renderBuiltWindTrails(MinecraftClient client, BuiltBuffer built) {
        BuiltBuffer.DrawParameters drawParameters = built.getDrawParameters();
        if (drawParameters.vertexCount() <= 0 || drawParameters.indexCount() <= 0) {
            return;
        }
        VertexFormat format = drawParameters.format();
        var vertexBuffer = format.uploadImmediateVertexBuffer(built.getBuffer());
        var indexBuffer = RenderSystem.getSequentialBuffer(drawParameters.mode()).getIndexBuffer(drawParameters.indexCount());
        var indexType = RenderSystem.getSequentialBuffer(drawParameters.mode()).getIndexType();
        DynamicUniforms dynamicUniforms = RenderSystem.getDynamicUniforms();
        var transforms = dynamicUniforms.write(
            RenderSystem.getModelViewMatrix(),
            new Vector4f(1.0f, 1.0f, 1.0f, 1.0f),
            new Vector3f(),
            new Matrix4f()
        );
        Framebuffer framebuffer = client.getFramebuffer();
        CommandEncoder encoder = RenderSystem.getDevice().createCommandEncoder();
        try (RenderPass renderPass = encoder.createRenderPass(
            () -> "aerodynamics4mc_wind_trails",
            framebuffer.getColorAttachmentView(),
            java.util.OptionalInt.empty(),
            framebuffer.getDepthAttachmentView(),
            java.util.OptionalDouble.empty()
        )) {
            renderPass.setPipeline(WIND_TRAIL_PIPELINE);
            RenderSystem.bindDefaultUniforms(renderPass);
            renderPass.setUniform("DynamicTransforms", transforms);
            renderPass.setVertexBuffer(0, vertexBuffer);
            renderPass.setIndexBuffer(indexBuffer, indexType);
            renderPass.drawIndexed(0, 0, drawParameters.indexCount(), 1);
        }
    }

    private void renderBuiltAnalysisSlice(MinecraftClient client, BuiltBuffer built) {
        BuiltBuffer.DrawParameters drawParameters = built.getDrawParameters();
        if (drawParameters.vertexCount() <= 0 || drawParameters.indexCount() <= 0) {
            return;
        }
        VertexFormat format = drawParameters.format();
        var vertexBuffer = format.uploadImmediateVertexBuffer(built.getBuffer());
        var indexBuffer = RenderSystem.getSequentialBuffer(drawParameters.mode()).getIndexBuffer(drawParameters.indexCount());
        var indexType = RenderSystem.getSequentialBuffer(drawParameters.mode()).getIndexType();
        DynamicUniforms dynamicUniforms = RenderSystem.getDynamicUniforms();
        var transforms = dynamicUniforms.write(
            RenderSystem.getModelViewMatrix(),
            new Vector4f(1.0f, 1.0f, 1.0f, 1.0f),
            new Vector3f(),
            new Matrix4f()
        );
        Framebuffer framebuffer = client.getFramebuffer();
        CommandEncoder encoder = RenderSystem.getDevice().createCommandEncoder();
        try (RenderPass renderPass = encoder.createRenderPass(
            () -> "aerodynamics4mc_analysis_slice",
            framebuffer.getColorAttachmentView(),
            java.util.OptionalInt.empty(),
            framebuffer.getDepthAttachmentView(),
            java.util.OptionalDouble.empty()
        )) {
            renderPass.setPipeline(ANALYSIS_SLICE_PIPELINE);
            RenderSystem.bindDefaultUniforms(renderPass);
            renderPass.setUniform("DynamicTransforms", transforms);
            renderPass.setVertexBuffer(0, vertexBuffer);
            renderPass.setIndexBuffer(indexBuffer, indexType);
            renderPass.drawIndexed(0, 0, drawParameters.indexCount(), 1);
        }
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

    private static int flowColor(Vec3d direction, float speedNorm, float pressureNorm, float alpha, float brightnessBias) {
        float lift = clamp01((float) (direction.y * 0.5 + 0.5));
        float r = lerp(0.42f, 0.88f, lift);
        float g = lerp(0.70f, 0.78f, lift);
        float b = lerp(0.96f, 0.60f, lift);
        float brightness = 0.84f + 0.24f * speedNorm + 0.10f * pressureNorm + brightnessBias;
        return argb(alpha, r * brightness, g * brightness, b * brightness);
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

    private record FlowStreamline(
        Vec3d[] points,
        float seedSpeed,
        float pressure,
        float phaseOffset,
        double length
    ) {
    }

    private record RemoteFlowField(
        Identifier dimensionId,
        BlockPos origin,
        int sampleStride,
        int atlasResolution,
        short[] packedFlow,
        FlowVisual visual,
        FlowGlyph[] glyphs,
        FlowStreamline[] streamlines,
        long lastUpdatedTick
    ) {
        static RemoteFlowField fromPayload(AeroFlowPayload payload, long lastUpdatedTick) {
            int sampleCount = payload.packedFlow().length / 4;
            int atlasResolution = Math.max(1, Math.round((float) Math.cbrt(sampleCount)));
            FlowGlyph[] glyphs = buildGlyphs(payload.origin(), payload.sampleStride(), atlasResolution, payload.packedFlow());
            return new RemoteFlowField(
                payload.dimensionId(),
                payload.origin(),
                payload.sampleStride(),
                atlasResolution,
                payload.packedFlow(),
                FlowVisual.fromPackedFlow(payload.packedFlow()),
                glyphs,
                buildStreamlines(payload.origin(), payload.sampleStride(), atlasResolution, payload.packedFlow(), glyphs),
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

        Box regionBox() {
            double size = (double) atlasResolution * (double) sampleStride;
            return new Box(
                origin.getX(), origin.getY(), origin.getZ(),
                origin.getX() + size, origin.getY() + size, origin.getZ() + size
            );
        }

        Box visibleBox() {
            Box full = regionBox();
            return new Box(
                full.minX + REGION_HALO_CELLS,
                full.minY + REGION_HALO_CELLS,
                full.minZ + REGION_HALO_CELLS,
                full.maxX - REGION_HALO_CELLS,
                full.maxY - REGION_HALO_CELLS,
                full.maxZ - REGION_HALO_CELLS
            );
        }

        private static FlowGlyph[] buildGlyphs(BlockPos origin, int sampleStride, int atlasResolution, short[] packedFlow) {
            record GlyphCandidate(FlowGlyph glyph, float score) {
            }
            int channels = 4;
            int atlasHalo = Math.min(REGION_HALO_CELLS / Math.max(1, sampleStride), atlasResolution / 2);
            int min = atlasHalo;
            int maxX = Math.max(min, atlasResolution - atlasHalo);
            int maxY = Math.max(min, atlasResolution - atlasHalo);
            int maxZ = Math.max(min, atlasResolution - atlasHalo);
            int spanX = Math.max(1, maxX - min);
            int spanY = Math.max(1, maxY - min);
            int spanZ = Math.max(1, maxZ - min);
            int bucketCount = GLYPH_BUCKETS_PER_AXIS * GLYPH_BUCKETS_PER_AXIS * GLYPH_BUCKETS_PER_AXIS;
            GlyphCandidate[] bestByBucket = new GlyphCandidate[bucketCount];
            List<GlyphCandidate> allCandidates = new ArrayList<>();
            for (int x = min; x < maxX; x += GLYPH_ATLAS_STEP) {
                for (int y = min; y < maxY; y += GLYPH_ATLAS_STEP) {
                    for (int z = min; z < maxZ; z += GLYPH_ATLAS_STEP) {
                        int cell = ((x * atlasResolution + y) * atlasResolution + z) * channels;
                        if (cell + 3 >= packedFlow.length) {
                            continue;
                        }
                        float vx = decodeVelocity(packedFlow[cell]);
                        float vy = decodeVelocity(packedFlow[cell + 1]);
                        float vz = decodeVelocity(packedFlow[cell + 2]);
                        float pressure = decodePressure(packedFlow[cell + 3]);
                        float speed = (float) Math.sqrt(vx * vx + vy * vy + vz * vz);
                        if (speed < GLYPH_MIN_SPEED) {
                            continue;
                        }
                        double invLength = 1.0 / speed;
                        Vec3d direction = new Vec3d(vx * invLength, vy * invLength, vz * invLength);
                        Vec3d start = new Vec3d(
                            origin.getX() + (x + 0.5) * sampleStride,
                            origin.getY() + (y + 0.5) * sampleStride,
                            origin.getZ() + (z + 0.5) * sampleStride
                        );
                        FlowGlyph glyph = new FlowGlyph(start, direction, speed, pressure, hashPhase(x, y, z));
                        float edgeWeight = edgeWeight(x, y, z, min, maxX, maxY, maxZ);
                        float score = speed * (0.30f + 0.70f * edgeWeight);
                        GlyphCandidate candidate = new GlyphCandidate(glyph, score);
                        allCandidates.add(candidate);

                        int bx = Math.min(GLYPH_BUCKETS_PER_AXIS - 1, Math.max(0, (x - min) * GLYPH_BUCKETS_PER_AXIS / spanX));
                        int by = Math.min(GLYPH_BUCKETS_PER_AXIS - 1, Math.max(0, (y - min) * GLYPH_BUCKETS_PER_AXIS / spanY));
                        int bz = Math.min(GLYPH_BUCKETS_PER_AXIS - 1, Math.max(0, (z - min) * GLYPH_BUCKETS_PER_AXIS / spanZ));
                        int bucket = (bx * GLYPH_BUCKETS_PER_AXIS + by) * GLYPH_BUCKETS_PER_AXIS + bz;
                        GlyphCandidate current = bestByBucket[bucket];
                        if (current == null || candidate.score() > current.score()) {
                            bestByBucket[bucket] = candidate;
                        }
                    }
                }
            }
            List<FlowGlyph> selected = new ArrayList<>();
            for (GlyphCandidate candidate : bestByBucket) {
                if (candidate != null) {
                    selected.add(candidate.glyph());
                }
            }
            if (selected.size() < MAX_GLYPHS_PER_REGION) {
                allCandidates.sort(Comparator.comparingDouble(GlyphCandidate::score).reversed());
                for (GlyphCandidate candidate : allCandidates) {
                    if (selected.size() >= MAX_GLYPHS_PER_REGION) {
                        break;
                    }
                    if (selected.contains(candidate.glyph())) {
                        continue;
                    }
                    selected.add(candidate.glyph());
                }
            }
            return selected.toArray(FlowGlyph[]::new);
        }

        private static FlowStreamline[] buildStreamlines(BlockPos origin, int sampleStride, int atlasResolution, short[] packedFlow, FlowGlyph[] glyphs) {
            if (glyphs.length == 0) {
                return new FlowStreamline[0];
            }
            Box visibleBox = new Box(
                origin.getX() + REGION_HALO_CELLS,
                origin.getY() + REGION_HALO_CELLS,
                origin.getZ() + REGION_HALO_CELLS,
                origin.getX() + atlasResolution * sampleStride - REGION_HALO_CELLS,
                origin.getY() + atlasResolution * sampleStride - REGION_HALO_CELLS,
                origin.getZ() + atlasResolution * sampleStride - REGION_HALO_CELLS
            );
            List<FlowStreamline> lines = new ArrayList<>();
            int seedCount = Math.min(MAX_STREAMLINES_PER_REGION, glyphs.length);
            for (int i = 0; i < seedCount; i++) {
                FlowGlyph glyph = glyphs[i];
                FlowStreamline streamline = integrateStreamline(origin, sampleStride, atlasResolution, packedFlow, visibleBox, glyph);
                if (streamline != null) {
                    lines.add(streamline);
                }
            }
            return lines.toArray(FlowStreamline[]::new);
        }

        private static FlowStreamline integrateStreamline(
            BlockPos origin,
            int sampleStride,
            int atlasResolution,
            short[] packedFlow,
            Box visibleBox,
            FlowGlyph glyph
        ) {
            List<Vec3d> backward = traceDirection(origin, sampleStride, atlasResolution, packedFlow, visibleBox, glyph.start(), -1.0);
            List<Vec3d> forward = traceDirection(origin, sampleStride, atlasResolution, packedFlow, visibleBox, glyph.start(), 1.0);
            int totalPoints = backward.size() + 1 + forward.size();
            if (totalPoints < 3) {
                return null;
            }
            Vec3d[] points = new Vec3d[totalPoints];
            int cursor = 0;
            for (int i = backward.size() - 1; i >= 0; i--) {
                points[cursor++] = backward.get(i);
            }
            points[cursor++] = glyph.start();
            for (Vec3d point : forward) {
                points[cursor++] = point;
            }
            double length = 0.0;
            for (int i = 0; i < points.length - 1; i++) {
                length += points[i].distanceTo(points[i + 1]);
            }
            if (length < sampleStride * 2.0) {
                return null;
            }
            return new FlowStreamline(points, glyph.speed(), glyph.pressure(), glyph.phaseOffset(), length);
        }

        private static List<Vec3d> traceDirection(
            BlockPos origin,
            int sampleStride,
            int atlasResolution,
            short[] packedFlow,
            Box visibleBox,
            Vec3d start,
            double sign
        ) {
            List<Vec3d> points = new ArrayList<>();
            Vec3d position = start;
            double traveled = 0.0;
            for (int step = 0; step < STREAMLINE_MAX_STEPS_PER_DIRECTION && traveled < STREAMLINE_MAX_LENGTH; step++) {
                Vec3d v0 = sampleVelocity(origin, sampleStride, atlasResolution, packedFlow, position);
                double speed0 = v0.length();
                if (speed0 < STREAMLINE_MIN_SPEED) {
                    break;
                }
                Vec3d dir0 = v0.multiply(1.0 / speed0);
                Vec3d mid = position.add(dir0.multiply(sign * STREAMLINE_STEP_LENGTH * 0.5));
                Vec3d v1 = sampleVelocity(origin, sampleStride, atlasResolution, packedFlow, mid);
                double speed1 = v1.length();
                if (speed1 < STREAMLINE_MIN_SPEED) {
                    break;
                }
                Vec3d dir1 = v1.multiply(1.0 / speed1);
                Vec3d next = position.add(dir1.multiply(sign * STREAMLINE_STEP_LENGTH));
                if (!visibleBox.contains(next)) {
                    break;
                }
                points.add(next);
                traveled += position.distanceTo(next);
                position = next;
            }
            return points;
        }

        private static Vec3d sampleVelocity(BlockPos origin, int sampleStride, int atlasResolution, short[] packedFlow, Vec3d position) {
            double lx = (position.x - origin.getX()) / sampleStride - 0.5;
            double ly = (position.y - origin.getY()) / sampleStride - 0.5;
            double lz = (position.z - origin.getZ()) / sampleStride - 0.5;
            int x0 = (int) Math.floor(lx);
            int y0 = (int) Math.floor(ly);
            int z0 = (int) Math.floor(lz);
            double fx = lx - x0;
            double fy = ly - y0;
            double fz = lz - z0;
            int x1 = x0 + 1;
            int y1 = y0 + 1;
            int z1 = z0 + 1;
            if (x0 < 0 || y0 < 0 || z0 < 0 || x1 >= atlasResolution || y1 >= atlasResolution || z1 >= atlasResolution) {
                return Vec3d.ZERO;
            }
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

        private static float hashPhase(int x, int y, int z) {
            int hash = x * 73428767 ^ y * 912931 ^ z * 19349663;
            hash ^= (hash >>> 16);
            return (hash & 0x7fffffff) / (float) Integer.MAX_VALUE;
        }

        private static float edgeWeight(int x, int y, int z, int min, int maxX, int maxY, int maxZ) {
            int dx = Math.min(x - min, (maxX - 1) - x);
            int dy = Math.min(y - min, (maxY - 1) - y);
            int dz = Math.min(z - min, (maxZ - 1) - z);
            int edgeDistance = Math.max(0, Math.min(dx, Math.min(dy, dz)));
            int halfSpan = Math.max(1, Math.min(maxX - min, Math.min(maxY - min, maxZ - min)) / 2);
            return clamp01((edgeDistance + 1.0f) / (halfSpan + 1.0f));
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

    private record FlowGlyph(
        Vec3d start,
        Vec3d direction,
        float speed,
        float pressure,
        float phaseOffset,
        Vec3d sideDirection,
        Vec3d upDirection
    ) {
        FlowGlyph(Vec3d start, Vec3d direction, float speed, float pressure, float phaseOffset) {
            this(start, direction, speed, pressure, phaseOffset, computeSide(direction), computeUp(direction));
        }

        private static Vec3d computeSide(Vec3d direction) {
            Vec3d reference = Math.abs(direction.y) > 0.8 ? new Vec3d(1.0, 0.0, 0.0) : new Vec3d(0.0, 1.0, 0.0);
            Vec3d side = direction.crossProduct(reference);
            if (side.lengthSquared() <= 1.0e-6) {
                return new Vec3d(1.0, 0.0, 0.0);
            }
            return side.normalize();
        }

        private static Vec3d computeUp(Vec3d direction) {
            Vec3d side = computeSide(direction);
            Vec3d up = side.crossProduct(direction);
            if (up.lengthSquared() <= 1.0e-6) {
                return new Vec3d(0.0, 1.0, 0.0);
            }
            return up.normalize();
        }
    }

    record AeroFlowState(boolean streamingEnabled) {
    }
}
