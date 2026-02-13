# Optional Feature Parity Plan (Rapier v0.32.0)

Date: 2026-02-13
Scope baseline already complete: `rapier2d(dim2,f32)` + `rapier3d(dim3,f32)` default features.

This plan tracks all remaining parity slices outside default scope.

## Slice A — `parallel`
- Goal: align parallel pipeline semantics and exposed APIs for stepping/query paths.
- Reference focus: `rapier-reference/src/pipeline/**`, `src/dynamics/solver/**`, feature-gated rayon paths.
- Deliverables: feature-gated MoonBit equivalent scheduling/execution path, parity tests for deterministic observable behavior.

## Slice B — `serde-serialize`
- Goal: parity for serialization/deserialization public surface and data fidelity.
- Reference focus: all `#[cfg(feature = "serde-serialize")]` public derives/helpers.
- Deliverables: MoonBit serialization module equivalents, round-trip tests for core world state.

## Slice C — `simd-*`
- Goal: parity for SIMD-specific behavior deltas while preserving public semantics.
- Reference focus: `simd-is-enabled`, `simd-stable`, `simd-nightly` code paths and conditional constants/helpers.
- Deliverables: equivalent fast path abstractions or mapped fallbacks, benchmark+correctness checks.

## Slice D — `debug-render`
- Goal: parity for debug render public API and output contract.
- Reference focus: debug render modules and feature-gated exports.
- Deliverables: MoonBit debug-render interfaces + snapshot-style tests for emitted primitives.

## Slice E — `profiler`
- Goal: parity for profiling counters/hooks public surface.
- Reference focus: profiler-gated counters/timing APIs.
- Deliverables: mapped profiling interface + invariants tests.

## Slice F — `enhanced-determinism`
- Goal: parity for determinism-oriented algorithmic branches and public toggles.
- Reference focus: deterministic-gated solver/integration branches.
- Deliverables: feature switch parity + cross-run deterministic outcome tests.

## Slice G — `f64` (dim2/dim3)
- Goal: parity for `f64` scalar variants in both 2D and 3D.
- Reference focus: scalar-generic math/dynamics/geometry exports.
- Deliverables: complete `f64` public surface mapping, numeric-tolerance-adjusted tests.

## Slice H — dev-only accessor paths
- Goal: explicitly classify and map/ignore dev-only paths appearing in source.
- Reference focus: test-only/debug-only public helpers referenced by audits.
- Deliverables: audit policy entries with reasoned classification and CI gate to prevent accidental default-scope regression.

## Execution order (priority)
1. `f64`
2. `parallel`
3. `enhanced-determinism`
4. `serde-serialize`
5. `debug-render`
6. `profiler`
7. `simd-*`
8. dev-only accessor cleanup

## Shared gates per slice
- `python3 tools/rapier_pub_audit.py run` with feature-specific mode/report.
- `moon check`
- `moon test` (package-targeted + full suite where applicable)
- Mapping file updates with zero unjustified ignores.
