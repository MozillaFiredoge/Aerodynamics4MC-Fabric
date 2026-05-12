# Native Physics Engine Todo

This is the working backlog for the Fabric mod runtime, ordered by practical impact rather than abstract completeness.

## Current Baseline

The following are already present in code:

- `L0` dynamic background met grid
- `L1` mesoscale grid with humidity and severe-weather diagnostics
- one-way nested `L1 -> L2` boundary fields
- `WorldScaleDriver`
- `CycloneCell`
- `ConvectiveCluster`
- `TornadoVortex`
- `L0/L1` dump pipeline
- client `AeroVisualizer` wind-trail/streamline overlay
- client-local `128^3` D3Q27 FP16 in-place SRT solver path
- source-delta fan forcing and sparse static patch upload for client-local `L2`

The main missing areas are usability, observability, and the foliage/shaderpack integration that was lost from the working tree.

There is also a strategic change:

- stitched runtime `L2` should no longer be treated as the main player-facing CFD showcase
- `L1` should become the default gameplay wind and weather layer
- local `L2` / CFD should be optional debug, calibration, or special-case refinement infrastructure
- machine gameplay should prefer reduced-order models calibrated by CFD, not full CFD solves every tick
- see `docs/runtime-aerodynamics-strategy.md`
- older local-patch and native-authoritative L2 documents should be read as historical or optional design references

## P0: Restore Missing Tooling

These are required to get back to the last useful working state.

- restore `docs/` design notes
- restore `eval_background_snapshot.py`
- restore `eval_mesoscale_snapshot.py`
- restore foliage real-wind client path
- restore shaderpack compatibility scaffolding

Status:

- `eval_background_snapshot.py`: restored
- `eval_mesoscale_snapshot.py`: restored
- `docs/`: restored
- foliage/shaderpack path: still missing

## P1: Runtime Observability

These make the system maintainable.

- keep `/aero dumpdata` stable
- keep dump JSON schemas documented
- add explicit versioning if snapshot schema changes again
- add clearer severe-weather diagnostics if tornado tuning resumes
- improve `/aero status` only when it helps operations, not as a vanity dashboard

## P2: Weather Stack

### Already landed

- world-scale persistent weather driver
- cyclone cells
- convective clusters
- tornado generation logic
- tornado forcing into `L1`
- tornado descriptor staging toward `L2`

### Still open

- validate `TornadoVortex -> L2` in practice
- decide whether tornado rarity is final behavior or just a conservative default
- cloud/condensation proxy
- cloud rendering

## P3: Gameplay Integration

These are still largely not done.

### Player/environment feedback

- player probe exposed to gameplay
- thermal feedback
- wind feedback
- sound/particle feedback

### Entity integration

- entity sample collection
- lightweight entity aerodynamics coupling
- gameplay-safe interaction rules

### Block/environment integration

- fire/smoke/rain directional response
- foliage real-wind motion
- duct/fan explanation and debugging polish
- chimney / ventilation airflow mechanics

## P3.5: Optional Local L2 / CFD Calibration

This is no longer the main runtime direction.
The current strategy is to make `L1` the default player-facing wind layer and keep CFD for controlled local refinement, debug visualization, and offline/runtime calibration data.

Solver-core status:

- complete enough for the current game-facing target
- tested path: `d3q27-fp16-inplace-srt`
- target shape: one client-local `128^3` brick
- steady-state objective: stay inside a `20 ms` gameplay budget including fan forcing and ordinary block edits
- memory objective: stay below `200 MB` for one active brick
- do not spend the next phase on cumulant/SGS unless SRT produces a visible gameplay defect

Open architecture work:

- keep static/dynamic rebuild work off the client hot path when L2 is enabled
- keep the one-brick `L1` boundary shell as the practical boundary model
- preserve physical continuity only for explicit refinement/debug sessions
- make atlas/probe publication independent from rebuild work, so optional visualization does not disappear during geometry bursts

Target:

- one optional local patch
- `64 blocks -> 128^3`
- six-face background boundary
- sponge shell
- native precompute of `5` local frames
- `int16` quantized direct-buffer publication to Java
- debug/refinement/calibration consumers only

Primary goals:

- provide controlled CFD reference behavior
- generate or validate coefficients for reduced-order runtime models
- debug local flow scenes when L1 or reduced-order behavior looks wrong

First implementation priorities:

- expose explicit debug/calibration commands
- keep six-face boundary sampling from the existing weather stack
- publish compact reference frames for visualization
- compare reduced-order fan, vent, plume, and obstacle models against CFD scenes

## P4: Client Visualization

Current client state:

- `AeroVisualizer` exists
- wind-trail streamline path exists
- debug overlay exists

Missing from the later working tree:

- `FoliageWindField`
- `FoliageWindRenderer`
- `FoliageWindReactivity`
- `ShaderpackCompatManager`
- client mixin/config scaffolding for foliage path
- `foliage_terrain` shaders
- BSL/Iris patch tools

This is the largest missing functional chunk outside the weather runtime itself.

Important scope note:

- do not continue investing in stitched `L2` CFD-style visualization as the primary proof path
- use client visualization mainly to support readable local-air gameplay and focused inspection

## P5: Shaderpack Compatibility

Planned direction:

- Iris-first
- BSL as first supported base pack
- runtime wind contract owned by the mod
- pack-side support owned by shaderpacks or explicit optional integrations
- keep built-in foliage path as fallback/debug path

Developer tooling that may exist for this line:

- `inspect_shaderpack.py`
- `patch_bsl_shaderpack.py`
- `shaderpack-compat/bsl/*`

Those tools are useful for development and validation, but they should not define the shipping product model.
- `docs/shaderpack-wind-compat-design.md`

## P6: Guardrails

When adding new systems:

- no silent schema drift in dump JSON
- no opaque runtime magic without diagnostics
- no large client rendering branch without a fallback path
- no new weather object without persistence and snapshot visibility

## Suggested Recovery Order

1. Finish restoring docs
2. Restore foliage real-wind client path
3. Restore shaderpack/BSL compatibility scaffold
4. Re-test `/aero dumpdata` end-to-end
5. Resume gameplay integration only after observability is back in place
