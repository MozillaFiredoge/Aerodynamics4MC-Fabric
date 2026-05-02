# Aerodynamics4MC Fabric

[![Build Status](https://github.com/MozillaFiredoge/Aerodynamics4MC-Fabric/workflows/build/badge.svg)](https://github.com/MozillaFiredoge/Aerodynamics4MC-Fabric/actions)
[![Native Matrix](https://github.com/MozillaFiredoge/Aerodynamics4MC-Fabric/workflows/native-matrix/badge.svg)](https://github.com/MozillaFiredoge/Aerodynamics4MC-Fabric/actions)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

Aerodynamics4MC is a Fabric mod that brings a multi-scale wind system to Minecraft. The current release direction is API-first: the server owns the coarse weather field, clients may run local L2 CFD for visualization and local effects, and other mods should consume wind through the public sampling API instead of reading internal solver buffers.

This README documents the current `fabric-mod/` project. Older root-level notes in the parent repository may describe obsolete architecture.

## Current Status

The mod currently has three wind layers:

| Layer | Owner | Resolution | Role | Gameplay trust |
|-------|-------|------------|------|----------------|
| L0 | Server | 256 blocks/cell | World-scale pressure, geostrophic wind, terrain and roughness driver | Trusted |
| L1 | Server | 64x64x40 blocks/cell, 8 layers | Mesoscale terrain-following wind, ABL shear, terrain bounce-back, turbulence/gust diagnostics | Trusted |
| L2 | Client-local, optional | 32x32x32 cells per brick | Local voxel LBM around the player, visual vectors/streamlines, particles and building-scale feedback | Not server-trusted by default |

The server authoritative path is L0/L1. Client L2 is an experimental fine layer that uses server coarse wind as initial/boundary reference, but it is not a trusted source for server-side game balance unless a future validation/aggregation path is explicitly added.

Legacy server L2, capture, analysis, and patch inspection code still exists for diagnostics and research. Do not treat those paths as the primary release architecture.

## Requirements

| Component | Version |
|-----------|---------|
| Minecraft | 1.21.11 |
| Yarn mappings | 1.21.11+build.4 |
| Fabric Loader | 0.18.4 |
| Fabric API | 0.141.2+1.21.11 |
| Java | 21+ |

Supported native targets:

| Platform | Library |
|----------|---------|
| Windows x86_64 | `aero_lbm.dll` |
| Linux x86_64 / ARM64 | `libaero_lbm.so` |
| macOS ARM64 | `libaero_lbm.dylib` |

OpenCL is optional at runtime but required for the GPU backend. CPU fallback exists for compatibility and diagnostics.

## Build And Run

From `fabric-mod/`:

```bash
./gradlew runClient
```

Compile Java and prepare bundled native resources:

```bash
./gradlew compileJava compileClientJava prepareNativeResources
```

Build a development jar:

```bash
./gradlew build
```

Build a remapped mod jar:

```bash
./gradlew remapJar
```

Build the native library manually:

```bash
cd native
cmake -S . -B build
cmake --build build -j
```

Native build details are in [native/README.md](native/README.md).

## Runtime Architecture

```text
Minecraft server
  AeroServerRuntime
    L0 BackgroundMetGrid
      pressure driver
      geostrophic wind
      terrain and roughness
    L1 MesoscaleGrid
      terrain-following vertical layers
      ABL shear and Ekman turning
      terrain solid mask / bounce-back
      turbulence, gust, wind-shear diagnostics
    Coarse wind sync payload
      compact server-trusted L1/L0 field for clients

Minecraft client
  AeroClientMod
    receives server coarse wind payload
    exposes client-side AeroClientWindApi
    renders vectors and streamlines
  ClientL2Solver
    optional experimental 32^3 brick LBM
    seeded and bounded by server coarse wind
    used for local visualization and particle effects

Native library
  JNI bridge used by the mod runtime
  C ABI wind-tunnel API for external validation tools
  OpenCL/CPU LBM backends
```

Important scale constants in the current code:

| Constant | Value | Meaning |
|----------|-------|---------|
| `BACKGROUND_MET_CELL_SIZE_BLOCKS` | 256 | L0 horizontal cell size |
| `MESOSCALE_MET_CELL_SIZE_BLOCKS` | 64 | L1 horizontal cell size |
| `MESOSCALE_MET_LAYER_HEIGHT_BLOCKS` | 40 | L1 vertical layer thickness |
| `MESOSCALE_MET_MAX_LAYERS` | 8 | L1 vertical layer count |
| `COARSE_WIND_SYNC_CELL_SIZE_BLOCKS` | 32 | Server-to-client coarse wind payload cell size |
| `ClientL2Solver.BRICK_SIZE` | 32 | Client local L2 brick size |

## Gameplay Components

The current gameplay-facing components are intentionally thin. Most systems should consume wind through the API rather than hardcoding block or solver internals.

| Component | Role |
|-----------|------|
| Fan block | Adds local forcing for ducts, ventilation tests, and visual flow feedback |
| Duct block | Helps shape local airflow paths |
| Wind Meter item | Right-click instrument that samples server-trusted L1/L0 wind at the player |
| Terrain and blocks | Feed world-delta/static masks into the wind system |
| Particles and Iris bridge | Should sample wind through the public API path |
| External vehicle/machine mods | Should sample server-trusted L0/L1 unless they explicitly need local visual L2 |

## Wind Sampling API

Other mods should use the public API under `com.aerodynamics4mc.api`.

Server side:

```java
import com.aerodynamics4mc.api.AeroWindApi;
import com.aerodynamics4mc.api.AeroWindSample;
import com.aerodynamics4mc.api.SamplePolicy;
import net.minecraft.util.math.Vec3d;

AeroWindSample sample = AeroWindApi.sample(serverWorld, position);
AeroWindSample coarse = AeroWindApi.sample(player, position, SamplePolicy.SERVER_COARSE_ONLY);

if (coarse.isTrustedForGameplay()) {
    Vec3d wind = coarse.meanVelocity();
    float speed = coarse.speedMetersPerSecond();
}
```

Client side:

```java
import com.aerodynamics4mc.api.AeroClientWindApi;
import com.aerodynamics4mc.api.AeroWindSample;
import com.aerodynamics4mc.api.SamplePolicy;
import net.minecraft.util.math.Vec3d;

AeroWindSample sample = AeroClientWindApi.sample(
    clientWorld,
    position,
    SamplePolicy.CLIENT_LOCAL_PREFERRED
);

Vec3d visualDrift = sample.effectiveVelocity();
```

`AeroWindSample` contains:

| Field or method | Meaning |
|-----------------|---------|
| `velocityX/Y/Z` | Mean wind velocity in m/s |
| `pressure` | Pressure anomaly or local solver pressure proxy |
| `level()` | Source scale: `NONE`, `L0`, `L1`, or `L2` |
| `authority()` | Source authority: server authoritative, server aggregated, client local, remote, or none |
| `confidence()` | Normalized confidence from 0 to 1 |
| `temperatureKelvin`, `humidity` | Atmospheric diagnostics when available |
| `turbulenceIntensity` | Local turbulence/gust proxy |
| `gustX/Y/Z` | Gust velocity contribution |
| `windShearXPerBlock`, `windShearZPerBlock` | Horizontal wind-shear gradient |
| `ablStability`, `ablMixingStrength` | ABL diagnostics |
| `meanVelocity()` | Mean velocity as `Vec3d` |
| `effectiveVelocity()` | Mean velocity plus gust |
| `isTrustedForGameplay()` | True when the sample can be used by server-authoritative gameplay |
| `isClientLocal()` | True when the sample came from local client L2 |
| `isLocalVoxelFlow()` | True for local L2 voxel-scale flow |
| `ageTicks(...)`, `isFresh(...)` | Freshness helpers |

### Sample Policies

| Policy | Recommended use |
|--------|-----------------|
| `SERVER_COARSE_ONLY` | Server-authoritative gameplay, vehicles, Create/Aeronautics cruise-scale wind |
| `GAMEPLAY_SERVER_ONLY` | Server-trusted L2 when available, otherwise L1/L0 |
| `SERVER_AGGREGATED_PREFERRED` | Client path that prefers server-published L2 atlas when present |
| `CLIENT_LOCAL_PREFERRED` | Client visuals, smoke, particles, local player feedback |
| `VISUAL_LOCAL_FIRST` | Visualization and engineering overlays |
| `DIAGNOSTIC_ALL_SOURCES` | Debug only |

Trust rule:

```java
if (!sample.isTrustedForGameplay()) {
    return;
}
```

Do not use client-local L2 for server-side balance, aircraft physics, or machine output unless the result is independently validated.

Full API notes are in [../docs/wind-sampling-api.md](../docs/wind-sampling-api.md).

## Recommended Integration Patterns

For aircraft, airships, turbines, and outdoor machines:

```java
AeroWindSample sample = AeroWindApi.sample(player, pos, SamplePolicy.SERVER_COARSE_ONLY);
Vec3d wind = sample.meanVelocity();
float turbulence = sample.turbulenceIntensity();
```

For client-only particles such as smoke, steam, ash, or dust:

```java
AeroWindSample sample = AeroClientWindApi.sample(world, pos, SamplePolicy.CLIENT_LOCAL_PREFERRED);
Vec3d drift = sample.effectiveVelocity();
```

For engineering overlays:

```java
AeroWindSample sample = AeroClientWindApi.sample(world, pos, SamplePolicy.VISUAL_LOCAL_FIRST);
if (sample.level() == AeroWindSample.Level.L2) {
    // Show local voxel flow.
}
```

Consumers should not depend on packet formats, internal grid classes, native handles, atlas layouts, or visualizer buffers.

## Commands

Server commands require permission level 2+:

| Command | Purpose |
|---------|---------|
| `/aero start` | Start server wind runtime and coarse sync |
| `/aero stop` | Stop runtime and clear active state |
| `/aero status` | Print server runtime, L0/L1, coordinator, and native status |
| `/aero render` | Print current client render mode status |
| `/aero render vectors on/off` | Toggle velocity vector rendering |
| `/aero render streamlines on/off` | Toggle streamline rendering |
| `/aero dumpdata` | Dump runtime diagnostics |
| `/aero dump_l1` | Dump L1 mesoscale snapshot |
| `/aero nested_feedback` | Print legacy nested-feedback diagnostics |
| `/aero capture_l2 ...` | Deprecated diagnostic capture path |
| `/aero inspect_patch ...` | Deprecated inspection/analysis path |

Client commands:

| Command | Purpose |
|---------|---------|
| `/aero_client_l2` | Print client L2 status |
| `/aero_client_l2 status` | Print client L2 status |
| `/aero_client_l2 on` | Enable experimental client-local L2 |
| `/aero_client_l2 off` | Disable experimental client-local L2 |

Client L2 is currently default-off in code. If enabled, it is intended for local visualization/effects and should not be interpreted as server-authoritative gameplay state.

## Visualization

The current visualizer favors a CFD-style viridis line renderer:

| Mode | Description |
|------|-------------|
| Vectors | One line segment per sampled cell; direction is velocity direction, length and color encode speed |
| Streamlines | Seeded integration through the sampled field, also colored by speed |

The old glyph path is no longer the preferred presentation layer.

## Native Solver API

The native library also exposes a small C ABI for standalone wind-tunnel validation and collaboration tools. It is independent of Minecraft, Fabric, and Create: Aeronautics internals.

Relevant docs:

| Document | Purpose |
|----------|---------|
| [native/docs/wind_tunnel_solver_api.md](native/docs/wind_tunnel_solver_api.md) | C ABI for rectangular wind-tunnel cases |
| [native/docs/jni_dll_usage_zh.md](native/docs/jni_dll_usage_zh.md) | Chinese Java/JNA/JNI DLL usage notes |
| [native/tools/benchmark_solver_dll.py](native/tools/benchmark_solver_dll.py) | Python smoke benchmark for DLL/shared-library output |

The C ABI is a low-Mach static wind-tunnel interface. It accepts grid size, voxel solid mask, boundary settings, and optional previous macro state, then outputs velocity and pressure-proxy fields.

## Known Boundaries

Current release assumptions:

| Area | Current position |
|------|------------------|
| Near-sonic aircraft | Not supported by the in-game world solver |
| Dynamic propeller geometry | Not implemented in the public wind-tunnel API |
| Server-trusted local L2 | Not the default path |
| Multiplayer L2 feedback | Requires future validation/aggregation design |
| Analysis/capture paths | Deprecated diagnostics, not release architecture |
| Voxel-to-airfoil design workflow | Out of scope for the in-game runtime API |

For Create: Aeronautics style integration, use `SERVER_COARSE_ONLY` for world wind and a separate vehicle-relative offline/proxy aerodynamic model for wing/propeller coefficients.

## Documentation

Additional project notes:

| Document | Purpose |
|----------|---------|
| [../docs/wind-sampling-api.md](../docs/wind-sampling-api.md) | Stable sampling API contract |
| [../docs/player-facing-wind-design.md](../docs/player-facing-wind-design.md) | Player-facing wind design direction |
| [native/README.md](native/README.md) | Native build details |
| [native/docs/wind_tunnel_solver_api.md](native/docs/wind_tunnel_solver_api.md) | Standalone C ABI |

## License

MIT. See repository license files for details.
