# Local Air Patch Design

> Status: historical / optional refinement design.
> The current runtime strategy is `docs/runtime-aerodynamics-strategy.md`.
> A `128^3` local patch is no longer the main player-facing airflow direction; it should be used only for explicit debug, calibration, or high-value local refinement.

## Purpose

This document defines the next local-air direction for `aerodynamics4mc`.

The goal is not to make the global runtime more scientifically complete.
The goal is to make nearby air feel more clearly like **air**:

- shaped by geometry
- continuous rather than block-by-block
- legible enough for players and viewers to infer simple rules

This is a local inspection-and-gameplay solver, not a replacement for the full weather stack.

## Product Position

The mod already has a working multiscale backbone:

- `WorldScaleDriver`
- `L0`
- `L1`
- current runtime `L2`

That backbone is valuable as infrastructure, but it has reached diminishing returns as the main player-facing airflow layer.

In particular:

- stitched active `L2` regions are acceptable for runtime environment support
- stitched active `L2` regions are **not** a good primary way to show “continuous CFD air” to players or viewers
- halo exchange improves continuity, but does not make a dynamic cross-shaped set of runtime regions equivalent to one monolithic domain

Therefore the next step is a **single monolithic local patch** near the player.

## Scope

The local air patch is intended to support:

- smoke / steam / fire directionality
- chimney / ventilation behavior
- building wake / corner gust / roof-edge acceleration
- local inspection views:
  - slice
  - quiver
  - obstacle overlay

It is not intended to become:

- a new global weather layer
- a stitched multi-region replacement
- a proof that the entire world is solved as one continuous CFD volume

## Core Configuration

Recommended first configuration:

- physical domain: `64 x 64 x 64 blocks`
- solver grid: `128 x 128 x 128`
- effective spatial resolution: `0.5 block / cell`
- one patch only
- centered near player or camera

Reasoning:

- `64 blocks` is large enough for local wakes and recirculation to develop
- `128^3` is fine enough to stop looking like “one air cell per block”
- a single patch avoids stitched-region edge artifacts

## Relationship To Existing Runtime

### Keep

- `WorldScaleDriver`
- `L0`
- `L1`
- existing runtime `L2` for current gameplay/weather infrastructure

### Stop treating as the main local-air showcase

- stitched `L2` active-window captures
- stitched `L2` CFD-style visualization as the main proof of continuity

The local patch is a separate layer:

- background state comes from the existing stack
- local resolved airflow comes from the monolithic patch

## Boundary Conditions

The local patch is **not** a single-inlet wind tunnel.

It uses six-face background boundary conditions:

- west / east
- north / south
- down / up

Each face carries spatially varying background fields sampled from the world-scale stack:

- `wind_x`
- `wind_y`
- `wind_z`
- `air_temperature`

Recommended first face sample density:

- `24 x 24` per face

### Sponge Layer

Do not hard-clamp the outermost cells directly to the sampled boundary values.

Instead:

- reserve an outer shell of `12` cells
- apply relaxation toward the target face fields there
- let the patch interior evolve freely

Purpose:

- reduce hard boundary artifacts
- preserve a readable interior flow structure
- keep the patch visually closer to “air moving through space” than “a box with scripted walls”

## Geometry Representation

The local patch must read obstacle geometry from real world blocks.

### Phase 1 geometry

- voxelized binary obstacle field at `128^3`
- built from actual block collision shapes

This is already a significant improvement over `1 block / cell`.

### Phase 2 geometry

If Phase 1 still looks too blocky, the first improvement should be:

- `solid fraction`
- `occupancy fraction`
- or another sub-voxel obstacle representation

This is a higher-value improvement than adding more weather objects or more analysis tooling.

## Initial State

Recommended first implementation:

- fill patch velocity from the sampled background flow
- fill patch temperature from sampled background air temperature
- initialize pressure conservatively
- then run a fixed number of solve steps until local structures appear

This is a simple warm start, not the final best approach.

It is good enough to test whether the product direction works.

## Execution Model

The local patch should not be treated as a permanent full-rate runtime layer at first.

Recommended model:

- one patch only
- solved asynchronously
- rendering/consumers read the most recent completed state
- patch rebuild occurs only when needed

Suggested rebuild triggers:

- player moved more than `8 blocks`
- large nearby block edits
- explicit manual refresh / inspection request

Suggested update target:

- approximately `5-10 Hz`

This keeps the patch responsive without forcing it into the main tick critical path.

## Solver Strategy

### Short-term

Reuse the existing native solver path first.

Reason:

- fewer variables change at once
- easier to evaluate whether `64 blocks -> 128^3` actually improves “air feel”
- avoids conflating solver replacement with patch architecture changes

### Long-term

If the local patch proves valuable but the current solver is still too heavy, then reconsider the solver.

At that point:

- a lighter Eulerian projection/MAC solver is a more plausible direction than FLIP for this air use case

Do not switch to FLIP as the first reaction.

## Player-Facing Consumers

The patch should feed only the most legible local systems first:

1. smoke / steam / flame
2. chimney / ventilation / enclosed-space airflow
3. local debug inspection:
   - speed slice
   - quiver
   - obstacle

Do not start by feeding:

- global weather
- foliage as the main source
- multi-patch world coverage
- generalized entity aerodynamics

## Inspection View Positioning

If an in-game inspection mode is exposed, it must be described honestly.

Correct description:

- a local CFD inspection solve based on current world geometry and background state

Incorrect description:

- implying that the entire world is currently being solved as one monolithic volume

The local patch is legitimate analysis, not a cheat, as long as its scope is stated correctly.

## Non-Goals

This design is explicitly not trying to:

- make current stitched `L2` perfectly Fluent-like
- prove strict global continuity across the whole runtime stack
- maximize scientific fidelity at any cost
- add more hidden layers that players cannot perceive

The design target is:

**simple inputs, computed rules, complex but legible local airflow.**

## Success Criteria

The direction is working if:

- smoke around buildings looks less like grid noise and more like air
- wake and recirculation are visibly tied to geometry
- local airflow can be used for simple building decisions
- viewers can tell that the field is computed rather than procedural

The direction is failing if:

- the patch is expensive but still looks blocky and ambiguous
- most of the improvement only appears in technical plots
- players still cannot infer simple rules from world behavior

## Recommended Phase Plan

### Phase 1

- single local patch
- `64 blocks -> 128^3`
- six-face background boundary
- sponge layer
- binary obstacle field
- output:
  - obstacle
  - speed slice
  - quiver

### Phase 2

- smoke / fire / chimney consumers
- verify local-air readability in normal play

### Phase 3

- sub-voxel obstacle representation
- better warm start
- smoother recenter / state reuse

### Phase 4

- re-evaluate solver only if needed
- keep the architecture if the experience is working

## Decision Summary

The recommended next local-air architecture is:

- keep `L0/L1` as world background infrastructure
- stop pushing stitched `L2` as the main player-facing CFD layer
- add one local monolithic patch:
  - `64 blocks`
  - `128^3`
  - six-face sampled boundary
  - `12-cell` sponge shell
  - local airflow consumers only

This is the smallest step that still has a real chance of making air feel computed, continuous, and worth the cost.
