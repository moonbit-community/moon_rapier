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
| UAI-004 | 3DReal substep policy | Done | Removed the non-CCD fallback substep path and kept CCD substep splitting driven by `max_ccd_substeps`, with first-impact selection. | `PhysicsPipeline::step` CCD substep loop |
| UAI-005 | CCD geometry metrics | Done | Added per-body CCD thickness/radius metrics derived from attached collider shapes for fast-motion activation and clamping thresholds. | `RigidBodyCcd::{is_moving_fast,max_point_velocity}` |
| UAI-006 | 3DReal joint-phase CCD parity | In Progress | 3DReal still uses post-integration joint solve semantics and an extra high-speed clamp pass after joints to avoid tunneling regressions; this is not yet a 1:1 upstream solver phase ordering. | `PhysicsPipeline::step` + `CCDSolver::clamp_motions` |

## Validation

- `moon check`
- `moon test`
