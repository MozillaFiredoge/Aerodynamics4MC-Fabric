# Runtime Aerodynamics Strategy

## Purpose

This document records the strategic shift for `aerodynamics4mc`.

The project should no longer be framed as a real-time CFD engine for Minecraft.
That direction is physically and computationally over-constrained:

- small `dx` requires very small explicit time steps when velocity rises
- real air viscosity pushes LBM relaxation too close to the stability limit
- high-Reynolds-number air at block scale cannot be resolved by a real-time grid
- large local CFD volumes do not automatically create better gameplay
- player-facing wind behavior is driven more by coherent environmental structure than by local Navier-Stokes fidelity

The better framing is:

> Aerodynamics4MC is a real-time atmospheric and aerodynamics gameplay system.
> It uses mesoscale simulation for environmental wind and weather, and uses CFD offline to validate and calibrate reduced-order runtime models.

## Core Decision

Use CFD where it is actually useful:

- offline validation
- coefficient generation
- wind-tunnel analysis
- debugging and visualization
- optional local refinement for special cases

Do not use CFD as the default way to update large real-time gameplay wind fields.

The default in-game experience should come from:

- computed environment wind
- coherent gusts and circulation
- reduced-order force and machine models
- visual sampling by foliage, particles, smoke, entities, and blocks

## Runtime Layers

### Layer 1: Environment

This is the real-time wind and weather layer.

It is responsible for:

- background wind
- terrain-scale flow tendencies
- mesoscale circulation
- gust fronts
- cyclones
- tornado / vortex descriptors
- thermal tendency fields
- foliage waves
- smoke / particle advection inputs
- default wind sampled by gameplay systems

The current `L1` mesoscale solver is the only CFD-like layer that fits this role.
It has a coarse enough spatial scale and a stable enough time scale to operate as a continuous game runtime.

This layer should be strengthened rather than replaced by local CFD.

### Layer 2: Mechanics

This layer should run real-time reduced-order models.

It is responsible for:

- fan and blower behavior
- duct and pipe pressure loss
- vent and nozzle jets
- heat exchanger behavior
- chimney and plume behavior
- machine cooling
- local block/entity force effects
- obstacle drag and sheltering approximations

Runtime models should use:

- lookup tables
- empirical or semi-empirical curves
- analytic jet / plume / vortex models
- pressure-node or flow-network solves
- precomputed coefficients
- simple local source terms injected back into the environment layer

CFD should be used to validate or generate these data, not to solve the machine every tick.

## CFD Role

### Keep Existing CFD

The existing solvers remain valuable, but their role changes.

`32^3` cumulant D3Q27:

- physically richer than the fast SRT path
- useful as a legacy local wind-tunnel solver
- useful for debugging and validating model behavior
- useful for small controlled scenes
- not the main global or default gameplay wind field

`128^3` D3Q27 FP16 in-place SRT:

- useful as a high-performance local refinement experiment
- useful for batch calibration
- useful for large machine or wind-tunnel test cases
- useful for comparing candidate reduced-order models
- not the default environment solver

Future mixed-precision cumulant / MRT work should be evaluated only if it serves a clear offline or special-purpose calibration target.
It should not be treated as a requirement for the default runtime.

### CFD Outputs

CFD tools should produce data that the runtime can consume cheaply:

- fan curves: pressure rise vs. flow rate vs. RPM
- duct coefficients: pressure loss vs. flow rate and geometry
- nozzle / jet profiles: centerline velocity, spread angle, decay
- plume profiles: heat source strength vs. buoyant rise and spread
- obstacle coefficients: drag, shelter factor, wake length
- machine coefficients: cooling efficiency, exhaust strength, heat rejection
- visualization references: expected smoke / particle behavior in test scenes

The runtime should consume these as fixed data or compact parameter sets.

## Why Real-Time CFD Is Not The Default

### Time Step Constraint

For explicit CFD / LBM, the lattice velocity scales as:

```text
u_lattice = U_phys * dt / dx
```

If `dt` is fixed to the Minecraft tick (`0.05s`), reducing `dx` quickly makes stable high-speed flow impossible.

For example, at `U = 20m/s`:

- `dx = 1m` gives `u_lattice = 1.0`
- `dx = 0.5m` gives `u_lattice = 2.0`
- `dx = 0.1m` gives `u_lattice = 10.0`

Those values are not compatible with ordinary low-Mach LBM assumptions.
Even when Mach accuracy is ignored, the explicit solver still becomes numerically fragile.

### Viscosity Constraint

Real air viscosity is approximately:

```text
nu_air = 1.5e-5 m^2/s
```

At `dx = 1m`, `dt = 0.05s`:

```text
nu_lattice = nu_air * dt / dx^2 = 7.5e-7
tau = 0.5 + 3 * nu_lattice = 0.50000225
```

This is extremely close to the LBM stability boundary.
It is especially unsuitable for an FP16 SRT runtime path.

At smaller `dx`, machine-scale geometry becomes more detailed, but the time-step and velocity constraints become worse.

### Gameplay Constraint

Even when a solver runs fast enough, local CFD does not automatically produce better gameplay.

Large-scale wind feeling comes from:

- coherent gusts
- visible wave propagation through foliage
- terrain and weather structure
- player-readable force feedback
- smoke and particle motion

Those are better supplied by the environment layer.

Local CFD is most useful when there is a clear local input/output problem:

- a wind tunnel
- a fan
- a duct
- a nozzle
- a heat source
- a controlled obstacle test

## Product Direction

The mod should aim to make air feel computed and consequential, not to prove that every nearby cell is a high-fidelity CFD solution.

The player-facing product goals are:

- wind changes with weather and terrain
- gusts can be seen moving through grass, leaves, smoke, and particles
- machines interact with air through understandable rules
- fans, vents, heat sources, and ducts have gameplay behavior
- local wind affects entities and blocks where it matters
- debugging tools can explain the airflow model

The scientific tool goals are:

- local wind-tunnel scenes can be solved with CFD
- CFD can generate reference behavior
- reduced-order runtime models can be checked against CFD
- solver performance and memory can still be measured

## Proposed Roadmap

### Phase A: Environment Runtime

- make `L1` the authoritative gameplay wind layer
- improve gust and circulation structure
- expose stable sampling APIs for gameplay, foliage, particles, and shaders
- make tornado / cyclone descriptors player-visible through environment wind, not through local CFD

### Phase B: Visual Wind Feedback

- restore and improve foliage wind motion
- drive grass and leaves from `L1`
- add particle / smoke / fire directionality
- make wind waves legible without requiring a CFD overlay

### Phase C: Reduced-Order Mechanics

- define fan / blower model
- define duct / vent pressure-loss model
- define nozzle / jet plume model
- define heat-source plume and cooling model
- expose machine-facing force, flow, pressure, and temperature APIs

### Phase D: CFD Calibration Toolkit

- keep `32^3` cumulant D3Q27 as a controlled validation solver
- keep `128^3` FP16 D3Q27 as a high-performance calibration/refinement solver
- add scripts or commands for generating lookup tables from CFD runs
- compare reduced-order models against CFD scenes

### Phase E: Optional Local Refinement

- allow explicit local CFD refinement only for high-value cases
- keep it opt-in, bounded, and observable
- do not let it become a prerequisite for normal gameplay

## Immediate Engineering Implications

- The default runtime should not depend on client-local `128^3` CFD.
- `ClientL2Solver` should be treated as optional/debug/refinement infrastructure.
- `L1` sampling and visualization should receive priority over further L2 solver optimization.
- Machine systems should ask for coefficients and local source terms, not full CFD solves.
- Future solver work should start from a product or calibration need, not from a desire to make the CFD grid larger.

## Naming

The project name `Aerodynamics4MC` remains appropriate if the project focuses on air as gameplay:

- environment wind
- forces
- drag
- lift-like effects where relevant
- machines that move or heat air
- visible wind interaction

The name does not require the mod to be a real-time CFD engine.
It should be understood as an aerodynamics gameplay system with CFD-backed tooling.

