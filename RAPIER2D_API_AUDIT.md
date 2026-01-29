# Rapier2D API Parity Audit (MoonBit Port)

Reference: `rapier2d` v0.32.0 (see `rapier-reference/crates/rapier2d/Cargo.toml`).

This repository currently validates 2D behavior primarily via strict parity tests against
`rapier-reference/examples2d/*` and `rapier-reference/examples2d/stress_tests/*`.
Those tests passing **does not imply** full parity with the complete public API surface of the
`rapier2d` crate.

## What We Audited

We audited the MoonBit packages' exported APIs using:

- MoonBit exported surface: `*/spec.mbt` and `*/pkg.generated.mbti`
- Rapier reference source: `rapier-reference/src/**`

We then compared the presence of public identifiers by name (token match). This is a heuristic:

- It **over-approximates** Rapier public API (includes items behind feature gates unless they are
  trivially detected as `dim3`-only).
- It **under-approximates** MoonBit parity (some items may exist but be named differently).
- Even when a name matches, semantics may still differ.

## High-Level Findings

### 1) 2D Example Parity: Yes

2D parity suites exist and pass for the `examples2d` set and the 2D stress tests.
This indicates the engine covers the behaviors exercised by those examples.

### 2) Full `rapier2d` Public API Parity: No

Several Rapier public modules/types/features are not implemented (or not exposed) in this MoonBit
port. The most significant gaps include:

- **Missing whole public modules**
  - `counters::*` (e.g. `Counters`, stage timers/counters).
  - `data::*` was previously missing, but is now present as the `Milky2018/moon_rapier/data` package
    (Arena/Index/Coarena/ModifiedObjects/pubsub). Re-run this audit to update the name-based counts.
- **Geometry/public API gaps**
  - Missing/unsupported public types (examples): `MeshConverter`, `MeshConverterError`,
    `BvhOptimizationStrategy`, `BroadPhasePairEvent`.
  - Missing Parry-style public shape exposure (`SharedShape`, many `parry::shape::*` re-exports).
  - Missing shape types like `Triangle` and a real `HeightField` type (this port often uses a 2D
    polyline approximation).
- **Query pipeline gaps**
  - No `QueryPipelineMut`-style API.
  - No persistent query dispatcher surface.
  - The current query pipeline construction is closer to a snapshot builder than Rapier's incremental
    updated query pipeline.
- **Hooks/solver contact APIs**
  - Missing `SolverContact` / `SolverFlags`-level types used by advanced contact modification.
- **Feature flags not implemented**
  - `serde-serialize` (snapshot/restore, serialization).
  - `parallel` (rayon-based parallelism).
  - `simd-stable` / `simd-nightly`.
  - `debug-render` pipeline surface.

## Quantitative Snapshot (Name-Based Heuristic)

From `rapier-reference/src/**`:

- Rapier public identifiers (heuristic dim2/neutral): **943**
- Present in MoonBit exports by token match: **413**
- Missing by token match: **530**

This is a directional indicator only; it is not a definitive parity score.

## Recommended Next Steps (If Full API Parity Is a Goal)

1) Add missing public modules:
   - Port and expose `counters::*` (Timers and stage counters).
2) Expand geometry/public surface:
   - Add missing shapes (e.g. `Triangle`) and clarify/implement `HeightField` semantics.
   - Add `MeshConverter` surface if desired.
3) Query pipeline parity:
   - Provide `QueryPipeline` incremental update semantics and a `QueryPipelineMut`-like API.
4) Hooks/solver contacts parity:
   - Expose solver-contact data structures needed for `modify_solver_contacts`-style hooks.
5) Decide feature-goals:
   - Serialization support vs. pure runtime simulation.
   - Parallelism and SIMD: decide if in-scope for MoonBit/targets.
