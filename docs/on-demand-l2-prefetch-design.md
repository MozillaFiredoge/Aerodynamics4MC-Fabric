# On-Demand L2 Prefetch Design

> Status: historical / optional refinement design.
> The current runtime strategy is `docs/runtime-aerodynamics-strategy.md`.
> On-demand `L2` remains useful as a bounded CFD tool, but it is no longer the default path for player-facing wind.

## Purpose

This document defines a simpler player-facing local-air architecture for `aerodynamics4mc`.

The intent is:

- keep `L1` as the always-on world background
- use `L2` only when nearby local airflow is valuable to compute
- decouple heavy native solving from Java-side consumers
- make local air readable without trying to solve everything all the time

This is not a global CFD architecture.
It is a **local, on-demand, prefetched airflow service** for smoke, flame, foliage, and focused inspection.

## Core Idea

Instead of treating `L2` as a stitched continuously active world layer, use it as a fixed-size local solver instance.

Pipeline:

```text
L1 background
    ->
Sampler(player neighborhood)
    ->
Discriminator(score / anchor / reason mask)
    ->
if valuable:
    native C++ solves one fixed local patch
    and precomputes the next 5 frames
    ->
    quantize to int16
    ->
    publish via DirectByteBuffer ring buffer
    ->
    Java consumers interpolate and render
else:
    consumers use L1 only
```

The important separation is:

- `L1` answers the broad wind direction and ambient thermal state
- `L2` answers whether the nearby scene deserves local resolved airflow

## Product Position

This architecture exists to improve:

- smoke / steam / flame behavior
- chimney / ventilation behavior
- local foliage gust response
- inspection overlays

It is not trying to prove:

- that the whole world is under one continuous CFD solve
- that every wake is worth resolving
- that stitched runtime `L2` is the right primary showcase

## Local L2 Patch

Recommended first configuration:

- physical domain: `64 x 64 x 64 blocks`
- solver resolution: `128 x 128 x 128`
- one active patch only at first
- boundary conditions sourced from `L1`
- native solves and precomputes `5` future frames

Rationale:

- `64 blocks` is large enough for meaningful local airflow structure
- `128^3` is fine enough to stop looking like one air cell per block
- `5` frames is long enough for smooth Java-side interpolation without making predictions stale too quickly

## Sampler

The sampler is a cheap CPU-side neighborhood summarizer.
It should inspect only the player neighborhood, not the whole world.

Suggested first neighborhood:

- horizontal radius: `32 blocks`
- vertical half-range: `24 blocks`

### Sampler inputs

The sampler should produce low-frequency descriptors:

- `l1_wind_x`
- `l1_wind_y`
- `l1_wind_z`
- `ambient_air_temperature`
- `filtered_occupancy`
- `obstacle_complexity`
- `heat_source_strength`
- `forced_flow_strength`
- `consumer_interest`
- `player_proximity`
- `previous_active_state`

### Geometry filtering

Do not feed raw Minecraft block noise into the discriminator.

The sampler should summarize nearby geometry into aerodynamic descriptors:

- occupancy / solidity
- obstacle gradient / edge strength
- porosity / drag proxy
- enclosure hints

This is necessary because the raw collision world is too high-frequency for stable local-air decisions.

## Discriminator

The discriminator decides whether local `L2` is worth activating.

### Outputs

It should output:

- `score` in `[0, 1]`
- `anchor`
- `reason_mask`

The `reason_mask` exists for debugging and later CNN replacement.

### First implementation

The first discriminator should be a handwritten value function.

Suggested baseline:

```text
score =
  0.35 * player_proximity
+ 0.20 * obstacle_complexity
+ 0.20 * heat_source
+ 0.15 * forced_flow
+ 0.10 * consumer_interest
+ 0.20 * already_active
```

This is only a baseline.
The important property is that it is explainable.

### Later implementation

Later, this can be replaced by a tiny CNN that predicts refine value directly.

That network should:

- read sampler outputs
- predict local importance
- not attempt to predict a full flow field

## L2 Activation State Machine

Use a small state machine:

- `inactive`
- `warming`
- `active`
- `cooldown`

### Activation

Enter `warming` if:

- `score > T_on`

Suggested first threshold:

- `T_on = 0.55`

### Keep-active hysteresis

Stay active while:

- `score > T_off`

Suggested first threshold:

- `T_off = 0.35`

### Warming

The warming state allows the native patch to build and precompute frames before consumers trust it fully.

Suggested first warming window:

- one completed prefetch batch

### Cooldown

After score drops below `T_off`, do not destroy the patch immediately.

Suggested first cooldown:

- `1-2 seconds`

This avoids activation flicker.

## Anchor Policy

The local patch anchor should not follow the player every frame.

Recommended first policy:

- quantized anchor lattice spacing: `16 blocks`
- recenter only if player drifts more than `16-24 blocks`

### Fast movement policy

When the player is moving quickly:

- do not try to chase them with constant patch rebuilds
- allow `L1` to dominate until speed falls again
- keep the previous patch alive if still useful

This keeps the system stable during traversal.

## Native Solve Model

The native side owns:

- patch preparation
- warm start
- stepping
- prefetch
- quantization
- ring buffer publication

The native side should solve one fixed local patch and precompute a short horizon.

### Solve horizon

First implementation:

- precompute `5` frames

Do not start with `10`:

- more memory
- more stale output risk
- less benefit for consumers

### Frame cadence

The solve cadence should be fixed and explicit.

Suggested first target:

- `20 Hz` local patch update step

This gives a `5` frame horizon of about `0.25 seconds`.

## Output Fields

Do not export more channels than consumers need.

First implementation should export:

- `vx`
- `vy`
- `vz`

Optional later:

- `temperature` or plume proxy

Do not export pressure in the first version unless a concrete consumer needs it.

## Quantization

First implementation should use:

- signed `int16` per velocity component

### Why int16 first

- simple
- cheap to dequantize
- deterministic
- no extra codec complexity
- good enough for same-process JNI exchange

### Suggested layout

Per frame:

- `vx`: `int16[cells]`
- `vy`: `int16[cells]`
- `vz`: `int16[cells]`

Metadata:

- `scale_vx`
- `scale_vy`
- `scale_vz`
- patch origin
- patch domain size
- solver resolution
- frame timestamp or simulation tick

### Quantization rule

Simple symmetric quantization:

```text
q = clamp(round(v / scale), -32767, 32767)
v = q * scale
```

The first version can use one scale per component per frame.

## Direct Buffer Ring Buffer

Because Java and C++ are in the same process, do not use heavy compression first.

Use a native ring buffer exposed through JNI as a `DirectByteBuffer`.

### Ownership

The native side owns the memory.
Java only reads.

### Ring entries

Each slot should contain:

- slot header
- frame metadata
- packed `vx`
- packed `vy`
- packed `vz`

### Suggested slot count

First implementation:

- `8` slots

This is enough for:

- current readable horizon
- one or two extra in-flight replacements
- no complicated allocator behavior

### Header contents

Each slot header should contain:

- `generation`
- `frame_index`
- `simulation_tick`
- `valid`
- `origin_x/y/z`
- `domain_blocks`
- `grid_resolution`
- `scale_vx/vy/vz`
- byte offsets to channel payloads

### Publication model

Publication should be monotonic:

1. native writes payload
2. native writes metadata
3. native flips `valid` / generation last

Java must only consume fully published slots.

## Java Consumption

Java does not solve.
Java only:

- reads the latest available slot sequence
- picks two neighboring frames
- dequantizes
- interpolates
- feeds consumers

### Consumer interpolation

Consumers should interpolate between two prefetched frames by local render time.

This makes the output look continuous even when the patch solver runs at a lower frequency than rendering.

### Fallback

If no valid local patch frames exist:

- consumers fall back to `L1`

If a patch exists but no future frame is available:

- consumers hold or extrapolate very conservatively
- prefer hold-over to noisy prediction

## Consumers

First consumers should be:

1. smoke / steam / flame
2. foliage bend target
3. inspection overlays

Do not start by feeding:

- global weather
- arbitrary entity aerodynamics
- large gameplay systems

## Warm Start

The local patch should warm-start from world background state, not from zero.

Required warm-start fields:

- `vx`
- `vy`
- `vz`
- air temperature if used

Use:

- `L1` first
- `L0` fallback
- conservative pressure init if pressure later matters

This reduces cold-start artifacts before the first prefetched frames are consumed.

## Invalidation Rules

The current prefetch batch must be invalidated and rebuilt if:

- player moved too far from anchor
- large nearby block edits occurred
- strong nearby forcing changed
- activation state exited `active`

The batch should not be invalidated on every small movement.

## Why This Is Better Than Heavy Same-Process Compression

For same-process JNI exchange:

- memory bandwidth is not the primary problem
- simplicity matters more than maximum compression ratio

Therefore:

- first use `int16` quantization only
- do not start with `zfp`, `zstd`, or similar codecs in the runtime path

Heavy compression can be revisited later only if profiling proves it necessary.

## Validation Criteria

This architecture is successful if:

- local consumers visibly look smoother and more air-like than pure `L1`
- Java-side frame consumption is cheap and stable
- patch rebuilds do not dominate runtime during normal movement
- high-speed traversal does not force constant local patch thrashing

It is not necessary to prove:

- global flow continuity
- wake correctness everywhere
- perfect numerical fidelity during traversal

## Immediate Implementation Order

1. keep `L1` as always-on fallback field
2. implement sampler and handwritten discriminator
3. implement one active local `L2` patch only
4. implement native `5`-frame prefetch
5. implement `int16` quantized direct-buffer ring buffer
6. implement Java-side frame interpolation
7. connect smoke / flame first
8. connect foliage second

## Non-Goals

Do not turn this into:

- a stitched world CFD showcase
- a multi-patch scheduler immediately
- an octree solver immediately
- a neural solver immediately

The point is to produce legible nearby air at acceptable cost.
