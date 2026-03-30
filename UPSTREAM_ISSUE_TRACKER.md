# Upstream Alignment Issue Tracker

This tracker records remaining and completed parity items between `moon_rapier` and `rapier-reference`.

Reference upstream sources:
- `rapier-reference/src/pipeline/physics_pipeline.rs`
- `rapier-reference/src/dynamics/ccd/ccd_solver.rs`

## Issues

| ID | Area | Status | Summary | Upstream Reference |
| --- | --- | --- | --- | --- |
| UAI-001 | 3DReal CCD active flag lifecycle | Done | Split `ccd_enabled` and `ccd_active` semantics for 3D rigid bodies and update active flags each substep. | `CCDSolver::update_ccd_active_flags` |
| UAI-002 | 3DReal first-impact estimation | Done | Replaced ball-only TOI shortcut with generic shape-cast based first-impact search for CCD-active bodies. | `CCDSolver::find_first_impact` |
| UAI-003 | 3DReal motion clamping path | Done | Clamping now runs only when CCD is enabled and at least one CCD body is active, with minimum TOI guard. | `PhysicsPipeline::run_ccd_motion_clamping` + `CCDSolver::clamp_motions` |
| UAI-004 | 3DReal substep policy | In Progress | Upstream-style CCD substep flow is aligned for hard-CCD bodies; non-CCD fallback substep/clamp stabilization is still kept for current 3DReal parity stability. | `PhysicsPipeline::step` CCD substep loop |
| UAI-005 | CCD geometry metrics | Done | Added per-body CCD thickness/radius metrics derived from attached collider shapes for fast-motion activation and clamping thresholds. | `RigidBodyCcd::{is_moving_fast,max_point_velocity}` |

## Validation

- `moon check`
- `moon test`
