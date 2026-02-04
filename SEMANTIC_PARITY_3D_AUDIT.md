# 3D Semantic Parity Gap Audit (Rapier v0.32.0)

This repository already tracks parity with upstream Rapier in two ways:

1) **Crate-level public surface audit** for `rapier3d`/`rapier2d` default features.
2) **Behavioral parity tests** under `rapier_full/` for representative scenes/examples.

This document is the starting checklist for “3D semantic parity” work that goes beyond
`pub`-surface coverage.

## Current Baseline (as of this audit)

- `python3 tools/rapier_pub_audit.py run`
  - `rapier2d`: `missing=0` (default features)
  - `rapier3d`: `missing=0` (default features)
- `moon test --frozen`: all tests passing.

## Gap Checklist (3D)

### Collision events
- Started/Stopped should be emitted for:
  - **Sensor intersections** (flags indicate `sensor`).
  - **Non-sensor contacts** (flags indicate non-sensor).
- Events must not repeat while the interaction persists (diff-based).
- Regression coverage:
  - `pipeline/physics_pipeline3d_real_test.mbt`:
    - sensor intersections Started/Stopped
    - non-sensor contacts Started/Stopped
    - “no repeat while persists” for both

### CCD (continuous collision detection)
- Fast-moving balls should not tunnel through thin obstacles when CCD is enabled.
- Regression coverage:
  - `pipeline/physics_pipeline3d_real_test.mbt`: “CCD prevents tunneling for fast balls”.
- Implementation note:
  - When splitting at TOI, a tiny overshoot is applied so the next step starts in a strictly
    intersecting configuration (avoids broad-phase boundary cases on exact touching).

### Narrow-phase / manifolds
- Verify contact normal direction, penetration sign, and contact point ordering for:
  - ball/cuboid, ball/ball, capsule/cuboid, cylinder/cuboid, halfspace/*,
    trimesh/ball, heightfield/ball, voxels/ball.
- Verify manifold persistence/stability where applicable (no jittery contact flipping).
- Add focused deterministic unit tests when parity tests don’t cover a case.

### Joints / multibody
- Verify joint limits/motors and multibody IK behaviors match rapier-reference for common setups.
- Prefer porting upstream unit tests when possible; otherwise add deterministic MoonBit tests.

### Query pipeline
- Verify ray casting, shape casting, and filtering flags match rapier-reference.
- Add edge-case tests for filters (exclude collider/body, collision groups, sensors).

## How to run

```bash
python3 tools/rapier_pub_audit.py run
moon test --frozen
moon test -p pipeline -f physics_pipeline3d_real_test.mbt
```

