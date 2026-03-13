# Native Solver Plan (JNI + OpenCL LBM)

This directory contains the native integration scaffold for embedding the CFD solver directly into `fabric-mod`.

## Chosen architecture

- Solver model: `D3Q27 LBM`
- Collision: cumulant-based relaxation + SGS closure
- Obstacles: voxel bounce-back (Minecraft block-aligned)
- Thermal coupling: internal scalar temperature + Boussinesq buoyancy force
- Compute backend: OpenCL kernels
- Host bridge: JNI (`NativeLbmBridge`)

Reasoning:

- LBM is local and highly parallel, better suited for per-tick real-time constraints than pressure-projection Poisson solves.
- OpenCL keeps vendor flexibility (NVIDIA / AMD / Intel) and can run on CPU fallback.
- JNI keeps the Java side minimal and avoids large socket transfer overhead.

## Java integration status

- `AeroClientMod` supports runtime backend selection:
  - `/aero backend socket`
  - `/aero backend native`
- `NativeLbmBridge` loads `libaero_lbm` and exposes:
  - `nativeInit(grid, inChannels, outChannels)`
  - `nativeStep(payload, grid, contextKey, output)`
  - `nativeStepDirect(directPayloadBuffer, grid, contextKey, output)`
  - `nativeReleaseContext(contextKey)`
  - `nativeShutdown()`

Current native code includes:

- OpenCL path (required when `AERO_LBM_ENABLE_OPENCL=ON`, default):
  - kernels: `init_distributions`, `stream_collide_step`, `output_macro`
  - per-window GPU buffers keyed by `contextKey`
  - cumulant + SGS + thermal scalar + Boussinesq force
- CPU reference `D3Q27` path (automatic fallback):
  - per-window context state (`contextKey`)
  - cumulant + SGS + thermal scalar + Boussinesq force
  - bounce-back obstacle handling
  - fan forcing, thermal source injection, and mild state nudging

Use `/aero backend status` after switching to native to confirm runtime path (`opencl|cumulant-d3q27+sgs+bouss:<device>` or `cpu|cumulant-d3q27+sgs+bouss...`).
The status command also prints tick timing breakdown: payload copy / solver / readback / total.

### Boussinesq coupling notes

- Native payload now supports `10 -> 4` (extra `thermal_source` at channel index 9).
- Backward compatibility is preserved: native still accepts `9 -> 4`, and socket backend accepts `10/9/5` input forms.
- Temperature is maintained only inside native contexts and is not sent back to Java yet.
- Runtime thermal source is now composed from:
  - sampled world heat blocks (torch/campfire/lava/magma/fire + low-weight luminance fallback),
  - day-night ambient thermal bias,
  - fan-induced heating (existing behavior, additive).
- For solid heat blocks, runtime sampling emits heat into neighboring fluid voxels before sending channel 9.
- Key constants live in `native/src/aero_lbm_jni.cpp`:
  - `kThermalDiffusivity`, `kThermalCooling`
  - `kThermalSourceScale`, `kThermalSourceMax`
  - `kThermalUpdateStride` (multi-rate thermal update interval)
  - `kBoussinesqBeta`, `kBoussinesqForceMax`

## Next implementation steps

1. Add configurable thermal source modes (fan-derived / block-defined / scripted regions).
2. Reduce PCIe traffic by uploading only obstacle/fan deltas (not full state tensor every tick).
3. Add mixed precision path (fp16/int16 storage, fp32 accumulate) for higher occupancy.
4. Add boundary-condition variants (inflow/outflow/slip tuning) and expose via command/config.
5. Validate buoyancy scenarios against reference cases and record perf per grid size.

## Build (Linux example)

```bash
cd fabric-mod/native
# Ubuntu/Debian:
# sudo apt-get install -y ocl-icd-opencl-dev opencl-c-headers opencl-clhpp-headers
cmake -S . -B build
cmake --build build -j
```

For standalone JNI tests, you can place the built shared library (`libaero_lbm.so` / `aero_lbm.dll`) on `java.library.path`.
For the Fabric mod runtime, prefer the jar packaging flow below.

## One-shot multi-platform build (4 binaries)

This repo now supports a CMake superbuild that tries to produce all of the following in one run:

- `natives/linux-x86_64/libaero_lbm.so`
- `natives/linux-arm64/libaero_lbm.so`
- `natives/windows-x86_64/aero_lbm.dll`
- `natives/macos-arm64/libaero_lbm.dylib`

Windows target is intentionally limited to `x86_64`.
macOS target is intentionally `arm64` only.

Command:

```bash
cd fabric-mod/native
cmake -S . -B build-all -DAERO_LBM_SUPERBUILD=ON -DAERO_LBM_DIST_DIR=$PWD/dist/natives
cmake --build build-all -j
```

If your cross toolchains do not provide target OpenCL SDK/libs, you can override:

```bash
cmake -S . -B build-all -DAERO_LBM_SUPERBUILD=ON -DAERO_LBM_SUPERBUILD_ENABLE_OPENCL=OFF
```

Compiler env overrides (if defaults are unavailable):

- Linux amd64: `AERO_CC_LINUX_X86_64`, `AERO_CXX_LINUX_X86_64`
- Linux arm64: `AERO_CC_LINUX_ARM64`, `AERO_CXX_LINUX_ARM64`
- Windows amd64: `AERO_CC_WINDOWS_X86_64`, `AERO_CXX_WINDOWS_X86_64`, optional `AERO_RC_WINDOWS_X86_64`
- macOS arm64: `AERO_CC_MACOS_ARM64`, `AERO_CXX_MACOS_ARM64`
- macOS SDK (optional): `AERO_MACOS_SYSROOT`

Note: this requires valid cross-compilers/toolchains to be installed on the build machine.

## GitHub Actions

Workflow: `fabric-mod/.github/workflows/native-matrix.yml`

- builds 4 native binaries with matrix jobs
- OpenCL mode in CI:
  - Linux amd64/arm64: `ON`
  - Windows x86_64: `ON` (OpenCL SDK provided via vcpkg in workflow)
  - macOS arm64: `OFF` (GitHub macOS runner OpenCL SDK availability is inconsistent)
- verifies all 4 files exist
- packs them into mod jar via `./gradlew remapJar`
- uploads `mod-jar` and `natives-bundle` artifacts

To force CPU fallback even when OpenCL exists:

```bash
export AERO_LBM_CPU_ONLY=1
```

To build native without OpenCL support (not default):

```bash
cmake -S . -B build -DAERO_LBM_ENABLE_OPENCL=OFF
```

## Packaging in mod JAR

`fabric-mod/build.gradle` now includes task `prepareNativeResources`:

- copies all files from `fabric-mod/native/dist/natives/**` (superbuild outputs) into `natives/**`
- copies optional prebuilt files from `fabric-mod/native/prebuilt/**` into resources as-is
- falls back to local dev build outputs in `fabric-mod/native/build*` when present

During `processResources`, these files are injected into the final mod jar.

If you provide prebuilt files manually, use these paths:

- `fabric-mod/native/prebuilt/natives/linux-x86_64/libaero_lbm.so`
- `fabric-mod/native/prebuilt/natives/linux-arm64/libaero_lbm.so`
- `fabric-mod/native/prebuilt/natives/windows-x86_64/aero_lbm.dll`
- `fabric-mod/native/prebuilt/natives/macos-arm64/libaero_lbm.dylib`

At runtime, `NativeLibraryLoader` extracts the matching native file from `jar:/natives/<os>-<arch>/` into temp dir and loads it via `System.load(...)`.
