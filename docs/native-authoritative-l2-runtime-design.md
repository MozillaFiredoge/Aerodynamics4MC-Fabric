# Native Authoritative L2 Runtime Design

> Status: historical / optional engine-grade L2 reference.
> The current runtime strategy is `docs/runtime-aerodynamics-strategy.md`.
> Native authoritative `L2` should not be treated as the default product direction unless a future feature has a clear CFD requirement.

## Purpose

This document defines how to migrate the current stitched `L2` runtime toward an engine-grade architecture:

- one native authoritative runtime per world
- sparse brick-based storage instead of per-window canonical state
- no post-step seam stitching
- no full `L2` flow-state readback into Java
- direct world-update ingestion at the solver boundary

The goal is to keep the existing `L0/L1/L2` weather stack, but replace the current `L2` execution model with something that can scale as a built-in game-engine subsystem.

This document is intentionally implementation-oriented. It is not a physics paper and it is not a product pitch.

## Why The Current Runtime Does Not Scale

The current `fabric-mod` runtime was a pragmatic mod integration, not an engine-native CFD architecture.

The main structural problems are:

- `L2` is partitioned into overlapping player-driven windows instead of one continuous active domain.
- each window owns an independent native context
- neighboring windows are made consistent only after solve, through seam synchronization
- seam synchronization is followed by a full flow-state readback into Java so the Java-side region cache stays authoritative
- world changes do not directly update the solver's hot geometry path; they mostly propagate through `WorldMirror -> section rebuild -> static patch -> region reset`
- the scheduler is organized around game ticks and Java orchestration rather than one native simulation timeline

These choices are acceptable for a mod prototype, but they become the bottleneck if `L2` is treated as an engine subsystem.

### Current Expensive Path

Today the expensive part of one `L2` cycle is not just the solve itself.

The current high-cost path is:

```text
block change
    ->
WorldMirror dirties sections
    ->
window region rebuild / patch upload
    ->
independent native region solves
    ->
seam sync between neighboring regions
    ->
full flow-state readback for touched regions
    ->
Java-side dynamic region state becomes authoritative again
```

The two most expensive structural costs are:

- halo exchange between independent region contexts
- full `64^3` macro-flow readback after seam sync

Those costs exist because the runtime is stitched after the fact instead of evolving as one continuous native state.

## Goals

- keep one authoritative `L2` state in native code
- treat Java as a client of the runtime, not the owner of dynamic flow state
- support sparse active simulation instead of dense whole-world simulation
- align geometry updates with Minecraft chunk-section scale
- keep `L1/L0` as the coarse background model outside active `L2`
- preserve observability through diagnostics, point sampling, and downsampled atlas extraction

## Non-Goals

- solving the entire Minecraft world at `L2` resolution all the time
- making `L2` the only atmosphere model
- introducing arbitrary asynchronous per-brick time steps in the first implementation
- implementing perfect fine-grained local reinitialization for every block change in the first iteration

## Core Architecture

### One Runtime Per World

Each server world owns one native `FluidWorldRuntime`.

Java-side responsibilities:

- submit world updates
- submit activity hints
- query diagnostics
- request render or gameplay samples
- manage save/load boundaries

Native-side responsibilities:

- own all dynamic `L2` state
- own all geometry/material/source state needed for `L2`
- schedule active bricks
- execute the fixed-step solver
- produce compact outputs for Java

### Brick-Based Internal Layout

`brick` is a storage and scheduling unit, not an independent mini-solver.

Recommended first brick size:

- `32 x 32 x 32` `L2` cells

Rationale:

- much better utilization than the current `64` grid with `16`-cell halo and `32`-cell core
- still large enough for cache-friendly vectorized loops
- easy to map to `2 x 2 x 2` Minecraft chunk sections

Later, `16^3` bricks may still be useful for persistence or geometry-update granularity, but `32^3` is a better first runtime brick size.

### Brick Data Model

Each brick stores:

- brick coordinates
- lifecycle flags: `active`, `sleeping`, `geometry_dirty`, `forcing_dirty`, `pending_reinit`
- obstacle / occupancy field
- material / thermal classification field
- source fields such as fan forcing and thermal emission
- hydrodynamic populations
- thermal state
- optional scalar channels needed by the local runtime
- diagnostics cache
- atlas cache for rendering extraction
- last-active tick / last-modified tick / persistence metadata

The authoritative hydrodynamic and thermal state remains native-only.

Java should not hold a canonical copy of per-brick flow state.

## Solver Epoch Model

All active bricks advance on one shared simulation epoch.

This is the key to physical consistency. Bricks are only a decomposition of storage and work. They are not separate simulations.

### Epoch Invariants

- one global `dt` for all active `L2` bricks
- every active brick reads from the same previous epoch
- every active brick writes to the same next epoch buffer
- world modifications are applied only at epoch boundaries
- cross-brick coupling happens during the step, not after the step

### Recommended Epoch Flow

```text
1. drain_updates
2. apply_geometry_at_epoch_boundary
3. resolve_active_set
4. prepare_boundaries_and_ghost_reads
5. collide_stream
6. diagnostics_and_restriction
7. swap_buffers
8. sleep_wake_evict
```

### 1. drain_updates

Collect updates coming from Java:

- block place / break
- block-state transitions that affect geometry or materials
- thermal emitters
- fans / ducts / scripted flow sources
- player/weather activity hints

These are appended to native queues and consumed at the next epoch boundary.

### 2. apply_geometry_at_epoch_boundary

Apply queued geometry and source changes before the next step begins.

This phase:

- updates the affected bricks' obstacle/material/source fields
- marks those bricks as `geometry_dirty`
- marks a small local closure around them as `pending_reinit`

The first implementation should not attempt perfect cell-local repair.
It is acceptable to reinitialize the affected brick and its immediate brick neighbors.

### 3. resolve_active_set

Build the set of bricks that must participate in this epoch.

The active set should include:

- bricks near players
- bricks near severe weather objects or scripted events
- recently modified bricks
- one-ring or two-ring closure around those bricks

The closure matters. If only the exact visible bricks are active, boundary consistency will degrade immediately.

### 4. prepare_boundaries_and_ghost_reads

Each active brick must be able to read the previous-epoch state of its neighbors.

This can be implemented in either of two ways:

- thin ghost layers refreshed from neighboring bricks before the step
- direct neighbor-buffer lookup during pull streaming

The second option is conceptually cleaner and avoids rebuilding a separate seam-sync phase.

The outer boundary of the active `L2` domain is supplied by `L1/L0`.

### 5. collide_stream

This is the actual native `L2` step.

Recommended first implementation:

- double-buffered populations and thermal state
- pull streaming
- per-brick parallel work scheduling

Pull streaming is preferred because it keeps the global-step mental model intact:

- every output cell reads the previous epoch
- no brick writes directly into a neighbor's current-step state

### 6. diagnostics_and_restriction

After the step, but still inside native code, compute compact derived outputs:

- max speed
- render atlas downsample
- probe samples
- `L2 -> L1` restriction packets
- runtime diagnostics

Do not read back the full brick state into Java.

### 7. swap_buffers

After every active brick has finished this epoch:

- swap old/new hydrodynamic buffers
- swap old/new thermal buffers
- advance the runtime epoch counter

This is the single commit point for the whole active domain.

### 8. sleep_wake_evict

At the end of the epoch:

- bricks with no recent activity can transition toward sleep
- newly demanded bricks become active
- cold bricks may be checkpointed for persistence

This is how the runtime stays sparse instead of becoming a dense world simulation.

## Why This Preserves Global Physical Consistency

The brick decomposition remains physically consistent because:

- all active bricks use the same `dt`
- all active bricks read the same previous state
- all active bricks commit together
- boundary exchange is part of the step itself, not a repair stage afterward

This is not "many little CFD worlds stitched together".
It is one continuous sparse domain with block-structured execution.

## Multiscale Interface

`L0/L1/L2` remain distinct responsibilities.

- `L0`: world-scale background and long-term drivers
- `L1`: mesoscale background and weather forcing
- `L2`: high-resolution local resolved airflow

### `L1 -> L2`

`L1` continues to supply:

- large-scale wind background
- ambient thermal targets
- coarse severe-weather forcing
- outer active-domain boundary state

### `L2 -> L1`

`L2 -> L1` should remain forcing-oriented.

That means:

- conservative restriction
- face-flux sampling
- temporal averaging
- nudging / tendency channels in `L1`

It should not directly overwrite `L1` live macrostate.

## World Update Path

### Current Path

Today a block update effectively travels through:

```text
block change
    ->
WorldMirror section dirtying
    ->
section rebuild
    ->
static patch upload
    ->
region backend reset
```

This path is too indirect for an engine-grade solver.

### Target Path

The target path is:

```text
block change
    ->
brick lookup
    ->
native queued update
    ->
epoch-boundary geometry apply
    ->
local reinit
    ->
next solver step sees the new geometry
```

This path is simpler, lower latency, and native-authoritative.

## Java / Native API Shape

The runtime should converge toward a narrow Java-facing API.

### World Lifecycle

- `create_world_runtime(world_id, config)`
- `destroy_world_runtime(world_id)`
- `load_world_snapshot(world_id, snapshot)`
- `save_dirty_world_snapshot(world_id)`

### Updates And Scheduling

- `submit_block_updates(world_id, updates)`
- `submit_source_updates(world_id, updates)`
- `set_activity_hints(world_id, hints)`
- `advance_world(world_id, sim_budget_ns)`

### Queries And Extraction

- `sample_point(world_id, x, y, z)`
- `sample_box(world_id, bounds, stride)`
- `extract_atlas(world_id, bounds, stride)`
- `poll_nested_feedback(world_id)`
- `get_runtime_diagnostics(world_id)`

The key design rule is:

- extraction APIs are allowed
- full canonical flow-state round-trips are not

## Persistence

Persistence should be brick-based, not window-based.

Persist only what is needed:

- active or recently active bricks
- diagnostic snapshots if useful
- geometry-derived metadata only if it cannot be reconstructed cheaply

Dynamic persistence should not be keyed by player window origin.

## Migration Plan

The migration should be staged. Do not combine architecture replacement and new physics behavior in one step.

### Phase 0: Instrument And Stabilize The Current Runtime

Before major refactors:

- keep `/aero status` useful
- keep seam/post timings visible
- keep nested feedback diagnostics visible
- keep atlas extraction stable

This phase already exists partially and should continue during migration.

### Phase 1: Make Native `L2` State Authoritative

Keep the current windowed runtime temporarily, but change authority rules:

- native context owns the true `L2` dynamic state
- Java no longer treats full region flow state as canonical
- publish only compact outputs: atlas, probes, diagnostics, restriction packets

Expected result:

- remove the need for full post-step flow-state readback
- cut the largest current `coordPostMs` cost

### Phase 2: Remove Seam-Driven State Repair

Still with temporary windows:

- stop depending on seam sync followed by Java-side canonical repair
- ensure the next step no longer reconstructs native state from Java dynamic copies

This is the point where the runtime stops being architecturally dependent on full readback.

### Phase 3: Introduce Native Brick Store

Replace the current per-region maps with a native brick store:

- remove canonical `dynamic_regions`
- replace region packet templates with brick-centered storage
- keep atlas/probe extraction as the Java-facing publication layer

This phase creates the real data model needed for an engine subsystem.

### Phase 4: Route World Updates Directly Into Native Bricks

Replace the hot geometry path:

- block updates should no longer depend on `WorldMirror -> section rebuild -> static patch` for solver-visible geometry
- world changes should be queued directly into the native brick runtime

`WorldMirror` can remain as a visualization/debug helper if still useful, but it should no longer be the authoritative geometry path for `L2`.

### Phase 5: Replace Window Scheduler With Activity-Driven Brick Scheduler

Move scheduling authority into native:

- Java submits activity hints
- native selects active bricks and closures
- native runs a fixed-step epoch loop under a budget

This is the point where the runtime stops being fundamentally player-window-centric.

### Phase 6: Brick Persistence And Sleep/Wake

Only after the runtime is stable:

- add brick snapshot persistence
- add sleep/wake rules
- tune hot/cold brick transitions

This phase is important for scale, but should not be attempted before the authoritative brick runtime is already working.

## Performance Rules

These rules should remain explicit during implementation.

- no full `L2` field readback to Java in the hot path
- no post-step seam repair stage
- no block-change path that requires whole-window rebuilds for ordinary local edits
- no Java-owned canonical `L2` dynamic state
- no uncontrolled activation churn from player movement

## Risks

### Geometry Change Stability

Block changes are the hardest numerical issue after the architecture shift.

Simply toggling obstacle bits is not enough.
The runtime must support local reinitialization or damping around geometry changes.

### Active-Set Churn

If bricks wake and sleep too aggressively:

- cache locality collapses
- boundary quality degrades
- the runtime spends too much time preparing bricks instead of solving

### Cross-Scale Drift

If the `L1 <-> L2` contract is too weak:

- `L2` drifts from the coarse weather state

If it is too strong:

- `L2` becomes numerically over-constrained

This is why `L2 -> L1` should remain forcing-style rather than direct state overwrite.

## What Can Be Deferred

The following should not block the core migration:

- perfect local reinit for every geometry edit
- advanced `L2 -> L1` moment transfer
- client-facing rendering refinements
- multi-rate substepping across different brick classes

Those are later optimization problems, not prerequisites for the architecture shift.

## Summary

The correct long-term direction is not "more optimized window stitching".

It is:

- one native authoritative `L2` runtime per world
- sparse brick-based storage
- shared epoch stepping
- direct brick-level world updates
- compact extraction to Java

That is a known engineering pattern adapted to Minecraft's world model.
The solver architecture is standard block-structured native simulation design.
The Minecraft-specific work is in:

- chunk-section alignment
- block/material/source mapping
- activity-driven sparse scheduling
- `L0/L1/L2` multiscale coupling
- render/gameplay extraction
