package com.aerodynamics4mc.runtime;

final class NestedBoundaryCoupler {
    BoundarySample fromBackgroundSample(BackgroundMetGrid.Sample sample) {
        if (sample == null) {
            return null;
        }
        return new BoundarySample(
            sample.backgroundWindX(),
            0.0f,
            sample.backgroundWindZ(),
            sample.ambientAirTemperatureKelvin(),
            sample.deepGroundTemperatureKelvin()
        );
    }

    BoundarySample fromMesoscaleSample(MesoscaleGrid.Sample sample) {
        if (sample == null) {
            return null;
        }
        return new BoundarySample(
            sample.windX(),
            0.0f,
            sample.windZ(),
            sample.ambientAirTemperatureKelvin(),
            sample.deepGroundTemperatureKelvin()
        );
    }

    void applyBackgroundWindBoundary(
        float[] solveField,
        BoundarySample sample,
        int gridSize,
        int channels,
        int obstacleChannel,
        int stateVxChannel,
        int stateVyChannel,
        int stateVzChannel,
        int statePChannel,
        int boundaryLayers,
        float minKeep
    ) {
        if (solveField == null || sample == null || boundaryLayers <= 0) {
            return;
        }
        for (int x = 0; x < gridSize; x++) {
            for (int y = 0; y < gridSize; y++) {
                for (int z = 0; z < gridSize; z++) {
                    int edgeDistance = Math.min(
                        Math.min(Math.min(x, y), z),
                        Math.min(Math.min(gridSize - 1 - x, gridSize - 1 - y), gridSize - 1 - z)
                    );
                    if (edgeDistance >= boundaryLayers) {
                        continue;
                    }
                    int idx = ((x * gridSize + y) * gridSize + z) * channels;
                    if (solveField[idx + obstacleChannel] > 0.5f) {
                        continue;
                    }
                    float eta = (boundaryLayers - edgeDistance) / (float) boundaryLayers;
                    float keep = minKeep + (1.0f - minKeep) * (1.0f - eta * eta);
                    float relax = 1.0f - keep;
                    solveField[idx + stateVxChannel] = solveField[idx + stateVxChannel] * keep + sample.windX() * relax;
                    solveField[idx + stateVyChannel] = solveField[idx + stateVyChannel] * keep + sample.windY() * relax;
                    solveField[idx + stateVzChannel] = solveField[idx + stateVzChannel] * keep + sample.windZ() * relax;
                    solveField[idx + statePChannel] *= keep;
                }
            }
        }
    }

    record BoundarySample(
        float windX,
        float windY,
        float windZ,
        float ambientAirTemperatureKelvin,
        float deepGroundTemperatureKelvin
    ) {
    }
}
