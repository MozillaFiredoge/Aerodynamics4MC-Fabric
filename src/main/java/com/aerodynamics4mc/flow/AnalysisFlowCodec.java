package com.aerodynamics4mc.flow;

import com.aerodynamics4mc.net.AeroFlowAnalysisPayload;
import com.aerodynamics4mc.runtime.NativeSimulationBridge;

import net.minecraft.util.Identifier;
import net.minecraft.util.math.BlockPos;

public final class AnalysisFlowCodec {
    private AnalysisFlowCodec() {}

    public static AeroFlowAnalysisPayload encodePayload(
        NativeSimulationBridge bridge,
        Identifier dimensionId,
        BlockPos origin,
        int baseSampleStride,
        short[] basePackedFlow,
        int fullResolution,
        float[] fullFlowState,
        float velocityTolerance,
        float pressureTolerance
    ) {
        if (bridge == null || basePackedFlow == null || fullFlowState == null || fullResolution <= 0) {
            return null;
        }
        int cells = fullResolution * fullResolution * fullResolution;
        if (fullFlowState.length != cells * PackedFlowField.CHANNELS) {
            return null;
        }

        float[] predicted = PackedFlowField.reconstructFullState(basePackedFlow, baseSampleStride, fullResolution);
        float[] residualVx = new float[cells];
        float[] residualVy = new float[cells];
        float[] residualVz = new float[cells];
        float[] residualPressure = new float[cells];
        for (int cell = 0; cell < cells; cell++) {
            int base = cell * PackedFlowField.CHANNELS;
            residualVx[cell] = fullFlowState[base] - predicted[base];
            residualVy[cell] = fullFlowState[base + 1] - predicted[base + 1];
            residualVz[cell] = fullFlowState[base + 2] - predicted[base + 2];
            residualPressure[cell] = fullFlowState[base + 3] - predicted[base + 3];
        }

        byte[] compressedVx = bridge.compressFloatGrid3d(residualVx, fullResolution, fullResolution, fullResolution, velocityTolerance);
        byte[] compressedVy = bridge.compressFloatGrid3d(residualVy, fullResolution, fullResolution, fullResolution, velocityTolerance);
        byte[] compressedVz = bridge.compressFloatGrid3d(residualVz, fullResolution, fullResolution, fullResolution, velocityTolerance);
        byte[] compressedPressure = bridge.compressFloatGrid3d(residualPressure, fullResolution, fullResolution, fullResolution, pressureTolerance);
        if (compressedVx == null || compressedVy == null || compressedVz == null || compressedPressure == null) {
            return null;
        }

        return new AeroFlowAnalysisPayload(
            dimensionId,
            origin,
            baseSampleStride,
            fullResolution,
            velocityTolerance,
            pressureTolerance,
            basePackedFlow,
            compressedVx,
            compressedVy,
            compressedVz,
            compressedPressure
        );
    }

    public static float[] decodePayload(NativeSimulationBridge bridge, AeroFlowAnalysisPayload payload) {
        if (bridge == null || payload == null || payload.fullResolution() <= 0) {
            return null;
        }
        int resolution = payload.fullResolution();
        int cells = resolution * resolution * resolution;
        float[] flowState = PackedFlowField.reconstructFullState(payload.basePackedFlow(), payload.baseSampleStride(), resolution);
        float[] residual = new float[cells];
        if (!bridge.decompressFloatGrid3d(payload.residualVx(), resolution, resolution, resolution, residual)) {
            return null;
        }
        addResidualChannel(flowState, 0, residual);
        if (!bridge.decompressFloatGrid3d(payload.residualVy(), resolution, resolution, resolution, residual)) {
            return null;
        }
        addResidualChannel(flowState, 1, residual);
        if (!bridge.decompressFloatGrid3d(payload.residualVz(), resolution, resolution, resolution, residual)) {
            return null;
        }
        addResidualChannel(flowState, 2, residual);
        if (!bridge.decompressFloatGrid3d(payload.residualPressure(), resolution, resolution, resolution, residual)) {
            return null;
        }
        addResidualChannel(flowState, 3, residual);
        return flowState;
    }

    private static void addResidualChannel(float[] flowState, int channel, float[] residual) {
        for (int cell = 0; cell < residual.length; cell++) {
            flowState[cell * PackedFlowField.CHANNELS + channel] += residual[cell];
        }
    }
}
