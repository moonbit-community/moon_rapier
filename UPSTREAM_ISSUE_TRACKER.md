# Upstream Alignment Issue Tracker

Last updated: 2026-03-30

## Status Legend

- `TODO`: not started
- `IN_PROGRESS`: currently being fixed
- `BLOCKED_PLATFORM`: blocked by platform/public API limitations or external dependency constraints
- `DONE`: fixed and verified locally

## Issues

| ID | Source | Problem | Status | Notes |
| --- | --- | --- | --- | --- |
| UAI-001 | `rapier-reference/src/dynamics/ccd/ccd_solver.rs::update_ccd_active_flags` | 3DReal mixed `ccd_enabled` and `ccd_active` semantics. | `DONE` | Fixed in `dynamics/rigid_body3d.mbt` and `pipeline/physics_pipeline3d_real.mbt`: introduced explicit `ccd_active` state, kept `ccd_enabled` as configuration flag, and update active flags each substep. Validated with `moon check` (0 errors) and `moon test` (`518 passed, 0 failed`). |
| UAI-002 | `rapier-reference/src/dynamics/ccd/ccd_solver.rs::find_first_impact` | 3DReal first-impact detection was ball-shape-specific. | `DONE` | Fixed in `pipeline/physics_pipeline3d_real.mbt`: replaced ball-only TOI shortcut with generic shape-cast first-impact search for CCD-active colliders. Validated with `moon test -p .../rapier_full_parity -f examples3d_trimesh_parity_test.mbt` and full `moon test` (`518 passed`). |
| UAI-003 | `rapier-reference/src/pipeline/physics_pipeline.rs::run_ccd_motion_clamping` + `ccd_solver.rs::clamp_motions` | 3DReal clamp path did not follow hard-CCD activation and minimum-TOI semantics. | `DONE` | Fixed in `pipeline/physics_pipeline3d_real.mbt`: clamp now runs through CCD-active gating with body CCD metrics and minimum-TOI guard for hard-CCD path. Validated with parity regressions (`examples3d_real_debug_more_parity_test.mbt`, `examples3d_trimesh_parity_test.mbt`) and full `moon test` (`518 passed`). |
| UAI-004 | `rapier-reference/src/pipeline/physics_pipeline.rs` CCD substep loop | 3DReal previously used non-CCD fallback substep splitting logic. | `DONE` | Fixed in `pipeline/physics_pipeline3d_real.mbt`: removed non-CCD fallback substep path; substep splitting now follows CCD loop controlled by `max_ccd_substeps` and first-impact estimate. Validated with `moon check` and full `moon test` (`518 passed`). |
| UAI-005 | `rapier-reference/src/dynamics/rigid_body_components.rs::{max_point_velocity,is_moving_fast}` | 3DReal lacked per-body CCD geometry metrics parity. | `DONE` | Fixed in `pipeline/physics_pipeline3d_real.mbt`: added per-body CCD thickness and max-distance metrics from attached colliders and used them for moving-fast thresholding. Validated with parity tests and full `moon test` (`518 passed`). |
| UAI-006 | `rapier-reference/src/pipeline/physics_pipeline.rs` solver phase ordering | 3DReal `JointSet3DReal` is still solved in a post-integration pass (not 1:1 with upstream joint/contact integrated solver phase). | `DONE` | Refactored in `pipeline/physics_pipeline3d_real.mbt` and `dynamics/joint_set3d_real.mbt`: moved joint-set solving into the pre-integration constraint phase, added a refined post-joint contact solve before integration, tightened high-speed fallback clamping to contact-coupled joint scenes, and increased `JointSet3DReal` solver iterations for motor convergence. Also included a bounded post-integration stabilization pass for pure-joint scenes. Validated with `moon check`, targeted parity suites (`examples3d_real_debug_more_parity_test.mbt`, `examples3d_real_vehicle_joints3_parity_test.mbt`, `examples3d_real_unique_parity_test.mbt`, `examples3d_trimesh_parity_test.mbt`), and full `moon test` (`518 passed, 0 failed`). |
| UAI-007 | `rapier-reference/src/pipeline/physics_pipeline.rs` end-of-step collision state update | 3DReal narrow-phase state could be stale when no event handler was passed. | `DONE` | Fixed in `pipeline/physics_pipeline3d_real.mbt`: always refresh broad-phase/narrow-phase after integration, then emit events from that refreshed state only when handler exists. Validated with `examples3d_trimesh_parity_test.mbt`, `examples3d_real_unique_parity_test.mbt`, and full `moon test` (`518 passed`). |

## Current Work Queue

1. None.
