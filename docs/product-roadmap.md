# Product Roadmap

## Product Thesis

`aerodynamics4mc` should not sell itself as a solver stack.

The player-facing promise is simpler:

**Minecraft gains an air world that can be observed, understood, and used.**

That means the mod should make these three things true:

- air is **readable**
- air is **self-consistent**
- air is **useful**

If the player only learns that the code contains `L0/L1/L2`, the mod has failed.
If the player learns that valleys, rooftops, chimneys, storms, and shelters behave differently because air actually moves, the mod has succeeded.

## What The Mod Is

The mod is best positioned as:

**a microclimate, airflow, and weather environment mod**

It sits closer to:

- Ambient Sounds
- shaderpacks
- weather mods
- building/automation mods that care about heat and ventilation

than to:

- aircraft construction mods
- hard-simulation vehicle mods

This does not forbid future integrations with flight or vehicle mods, but that should not define the core product.

## What The Mod Is Not

These are not the mainline product goals right now:

- building a general aircraft design sandbox
- competing with Valkyrien Skies as a vehicle platform
- replacing `L0/L1/L2` with neural solvers
- adding a new `L3` just because ML is available
- shipping patched copies of third-party shaderpacks

Those directions either duplicate existing ecosystems or add technical weight without improving player-facing value enough.

## Core Player Fantasy

The player should gradually realize:

- wind has direction and structure
- heat rises and changes local airflow
- buildings can trap, vent, or redirect air
- storms have precursors and local consequences
- terrain shapes the air in ways that can be learned

The strongest version of the mod is not “the most advanced CFD.”
It is “the first time Minecraft air feels like part of the world.”

## Roadmap Principles

### 1. Prioritize perception over hidden fidelity

A physically richer system that cannot be seen or used is lower priority than a simpler system with strong feedback.

### 2. Prefer reusable environmental rules over one-off spectacle

A chimney that always works, a wind shadow behind a hill, and a roof edge gust are more valuable than rare isolated effects with no broader pattern.

### 3. Make the player discover rules naturally

The player should infer air behavior from repeated world interactions.
The mod should not depend on the player reading a technical manual.

### 4. Keep the physics stack as infrastructure

`L0/L1/L2` exist to support the world fantasy.
They are not the feature list.

## Roadmap Phases

## Phase 1: Make Air Readable

Goal:

The player should be able to see and hear that air is behaving differently from place to place.

Deliverables:

- foliage motion that combines stable background wind with local `L2` boosts
- directional smoke and steam
- directional fire/flame behavior
- rain streak tilt and storm gust cues
- stronger wind/audio ambience in exposed places
- minimal debug/readability tools for development:
  - keep `/aero dumpdata`
  - keep snapshot scripts stable
  - optionally add one very small in-game indicator block/item later

Why this phase comes first:

Right now most of the system is still “backend truth.”
This phase turns it into something players can notice without explanation.

Success criteria:

- players can tell sheltered and exposed places apart without opening debug overlays
- vegetation and smoke make strong flows obvious
- storms feel spatially structured rather than globally cosmetic

## Phase 2: Make Air Useful In Building Gameplay

Goal:

The player should be able to exploit airflow and heat in construction.

Deliverables:

- chimney / vent / exhaust behavior
- indoor vs outdoor ventilation differences
- heat accumulation in enclosed spaces
- greenhouse / shelter / windbreak behavior
- ducts and fans that are easier to reason about from in-world feedback

Target player behaviors:

- placing vents to improve airflow
- using chimneys to remove smoke and hot air
- designing shelters for storms or cold winds
- building around terrain wind shadows and ridge gusts

Success criteria:

- there are clear build choices that feel better or worse because of air behavior
- players can improve a build by reasoning about circulation
- the mod creates new architectural decisions instead of only visual ambience

## Phase 3: Make Weather Feel Like A System

Goal:

Weather should stop feeling like a skybox event and start feeling like a spatial environmental process.

Deliverables:

- cyclone / convective structure made perceptible through the world
- pre-storm cues:
  - wind shift
  - stronger gusts
  - humidity / haze / audio changes
- rare tornadoes kept as exceptional events
- storm-safe vs storm-exposed builds becoming legible

Important constraint:

Tornadoes should stay rare and meaningful.
They are not the backbone of the product.

Success criteria:

- players can notice weather changing before the visible event peaks
- terrain and buildings matter during storms
- tornadoes feel like rare outcomes of a broader system, not random special effects

## Phase 4: Lightweight Gameplay Coupling

Goal:

Air starts to matter to motion and survival in restrained ways.

Deliverables:

- light gliding / updraft interaction
- light projectile drift
- wind chill / heat stress style feedback if it remains readable
- selective entity/environment coupling where it adds clarity

Guardrail:

Do not let this phase turn into constant player-control frustration.
The world should feel more believable, not more annoying.

Success criteria:

- the player can use or compensate for airflow in limited but meaningful ways
- effects are noticeable in interesting contexts, not constantly intrusive

## Phase 5: Public Integration Surface

Goal:

Other mods can consume the air world without this mod becoming a vehicle platform itself.

Deliverables:

- stable local wind sampling API
- documented shader/runtime bridge contract
- optional interoperability points for:
  - gliders
  - projectiles
  - weather-aware blocks
  - shaderpacks

This is the correct place to support vehicle or flight mods indirectly.
It is not necessary to own that gameplay ourselves.

## Immediate Next Steps

The highest-value next work is:

1. stop treating stitched `L2` as the main player-facing airflow showcase
2. make `L1` the default player-facing wind layer:
   - coherent gusts and circulation
   - visible wind waves through foliage, smoke, particles, and fire
   - stable sampling APIs for gameplay and rendering
   - tornado / cyclone descriptors expressed through environment wind
3. use reduced-order mechanics for machines:
   - fans, vents, ducts, jets, plumes, and cooling use coefficients and compact models
   - CFD generates or validates those coefficients instead of running every tick
4. connect the runtime first to:
   - directional smoke / steam / fire
   - one building-useful airflow mechanic such as chimney draft

That sequence is deliberate:

- L1 proves “this is air, not noise”
- smoke/fire proves “air has direction”
- chimneys/vents prove “air can be used”

## Current Recommended Backlog

### Do next

- write and freeze the runtime aerodynamics strategy
- make foliage, particles, smoke, and flame respond visibly to `L1`
- define the first reduced-order fan / vent / chimney mechanics
- one architecture-facing airflow mechanic
- maintain `dumpdata` and snapshot tooling for the world-scale stack, but stop expanding stitched `L2` visualization as the main product path

### Defer until later

- generalized monolithic inspection tooling beyond what is needed to validate the patch
- always-on client-local `128^3` CFD as a default gameplay layer
- cloud rendering
- ML `L3`
- aircraft gameplay
- broad entity physics coupling
- generalized shaderpack family support beyond Iris/BSL

## ML Position

Machine learning is still allowed, but it should be subordinate to the product roadmap.

The right future use of ML is:

- short-horizon local refinement
- subgrid turbulence / wake enhancement
- derived environmental proxies

The wrong use of ML right now is:

- adding an `L3` before the current system has enough player-facing consumers

In short:

**first make the world feel alive, then decide whether ML is needed to deepen it.**

## One-Sentence Roadmap Summary

The next version of the mod should focus on making air **visible**, then **useful**, then **systemic** before it becomes more technically ambitious.
