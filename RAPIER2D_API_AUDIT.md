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

### 2) Full `rapier2d` Public API Parity: Unverified (This Document Is Stale)

This document originally captured early gaps in the MoonBit port. Since then (notably on
**2026-01-29**), the project implemented the initial missing parity buckets referenced by the
follow-up epic tasks (data/counters/geometry/query pipeline/hooks/features).

Even with those additions, **full crate-level public API parity is still not mechanically proven**
here. The only hard evidence captured in this repo remains: parity tests against
`rapier-reference/examples2d/*` and `rapier-reference/examples2d/stress_tests/*`.

If full public API parity is still a goal, re-run an automated name-based audit (or a stronger
semantic audit) and update the sections below accordingly.

## Quantitative Snapshot (Name-Based Heuristic)

This section is retained for historical context, but the counts below are **stale** and likely
incorrect after the recent parity work.

From `rapier-reference/src/**` (stale):

- Rapier public identifiers (heuristic dim2/neutral): **943**
- Present in MoonBit exports by token match: **413**
- Missing by token match: **530**

This is a directional indicator only; it is not a definitive parity score.

## Recommended Next Steps (If Full API Parity Is a Goal)

1) Re-run an API audit:
   - Regenerate `*/pkg.generated.mbti` with `moon info`.
   - Recompute missing identifiers vs. `rapier-reference` using an automated script.
2) For any remaining gaps, create one issue per module/bucket:
   - Prefer "API + semantics + determinism test" per gap.
