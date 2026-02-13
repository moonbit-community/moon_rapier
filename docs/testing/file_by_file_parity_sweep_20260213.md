# File-by-File Parity Sweep Baseline (2026-02-13)

## Scope

- Epic: `moon_rapier-gap-file-by-file-20260213`
- Reference: `rapier-reference/src/**/*.rs` (118 files)
- In-scope parity baseline: `rapier2d/rapier3d v0.32.0 default features`
- Optional-feature follow-up issue: `moon_rapier-gap-optional-features-followup-20260213`

## Baseline Evidence

- `python3 tools/rapier_pub_audit.py run` -> `rapier2d 1834/1834`, `rapier3d 1956/1956`, missing `0`
- `python3 tools/rapier_pub_style_audit.py run` -> mapped entries `315` (style aliases documented)
- TODO/FIXME sweep across moon_rapier source found no active TODO/FIXME markers; `panic()` occurrences are internal invariant guards in data structures.

## Per-file Checklist

| Task | Reference file | Feature cfg in file | Disposition |
| --- | --- | --- | --- |
| `moon_rapier-gap-file-by-file-20260213.2` | `rapier-reference/src/control/character_controller.rs` | `dim2,dim3,f32` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.3` | `rapier-reference/src/control/mod.rs` | `dim3` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.4` | `rapier-reference/src/control/pid_controller.rs` | `dim2,dim3` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.5` | `rapier-reference/src/control/ray_cast_vehicle_controller.rs` | `dim2,dim3` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.6` | `rapier-reference/src/counters/ccd_counters.rs` | `-` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.7` | `rapier-reference/src/counters/collision_detection_counters.rs` | `-` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.8` | `rapier-reference/src/counters/mod.rs` | `-` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.9` | `rapier-reference/src/counters/solver_counters.rs` | `-` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.10` | `rapier-reference/src/counters/stages_counters.rs` | `-` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.11` | `rapier-reference/src/counters/timer.rs` | `profiler` | default-scope reviewed; optional slices (profiler) tracked in follow-up issue |
| `moon_rapier-gap-file-by-file-20260213.12` | `rapier-reference/src/data/arena.rs` | `-` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.13` | `rapier-reference/src/data/coarena.rs` | `-` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.14` | `rapier-reference/src/data/graph.rs` | `-` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.15` | `rapier-reference/src/data/mod.rs` | `-` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.16` | `rapier-reference/src/data/modified_objects.rs` | `-` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.17` | `rapier-reference/src/data/pubsub.rs` | `-` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.18` | `rapier-reference/src/dynamics/ccd/ccd_solver.rs` | `-` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.19` | `rapier-reference/src/dynamics/ccd/mod.rs` | `-` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.20` | `rapier-reference/src/dynamics/ccd/toi_entry.rs` | `dim2,dim3` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.21` | `rapier-reference/src/dynamics/coefficient_combine_rule.rs` | `-` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.22` | `rapier-reference/src/dynamics/integration_parameters.rs` | `dim3` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.23` | `rapier-reference/src/dynamics/island_manager/island.rs` | `-` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.24` | `rapier-reference/src/dynamics/island_manager/manager.rs` | `-` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.25` | `rapier-reference/src/dynamics/island_manager/mod.rs` | `-` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.26` | `rapier-reference/src/dynamics/island_manager/optimizer.rs` | `-` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.27` | `rapier-reference/src/dynamics/island_manager/sleep.rs` | `-` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.28` | `rapier-reference/src/dynamics/island_manager/utils.rs` | `-` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.29` | `rapier-reference/src/dynamics/island_manager/validation.rs` | `-` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.30` | `rapier-reference/src/dynamics/joint/fixed_joint.rs` | `-` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.31` | `rapier-reference/src/dynamics/joint/generic_joint.rs` | `dim2,dim3,simd-is-enabled` | default-scope reviewed; optional slices (simd-is-enabled) tracked in follow-up issue |
| `moon_rapier-gap-file-by-file-20260213.32` | `rapier-reference/src/dynamics/joint/impulse_joint/impulse_joint.rs` | `-` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.33` | `rapier-reference/src/dynamics/joint/impulse_joint/impulse_joint_set.rs` | `parallel` | default-scope reviewed; optional slices (parallel) tracked in follow-up issue |
| `moon_rapier-gap-file-by-file-20260213.34` | `rapier-reference/src/dynamics/joint/impulse_joint/mod.rs` | `-` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.35` | `rapier-reference/src/dynamics/joint/mod.rs` | `dim3` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.36` | `rapier-reference/src/dynamics/joint/motor_model.rs` | `-` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.37` | `rapier-reference/src/dynamics/joint/multibody_joint/mod.rs` | `-` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.38` | `rapier-reference/src/dynamics/joint/multibody_joint/multibody.rs` | `dim2,dim3,parallel` | default-scope reviewed; optional slices (parallel) tracked in follow-up issue |
| `moon_rapier-gap-file-by-file-20260213.39` | `rapier-reference/src/dynamics/joint/multibody_joint/multibody_ik.rs` | `dim2,dim3` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.40` | `rapier-reference/src/dynamics/joint/multibody_joint/multibody_joint.rs` | `dim2,dim3` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.41` | `rapier-reference/src/dynamics/joint/multibody_joint/multibody_joint_set.rs` | `-` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.42` | `rapier-reference/src/dynamics/joint/multibody_joint/multibody_link.rs` | `-` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.43` | `rapier-reference/src/dynamics/joint/multibody_joint/multibody_workspace.rs` | `-` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.44` | `rapier-reference/src/dynamics/joint/multibody_joint/unit_multibody_joint.rs` | `-` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.45` | `rapier-reference/src/dynamics/joint/pin_slot_joint.rs` | `dim2` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.46` | `rapier-reference/src/dynamics/joint/prismatic_joint.rs` | `-` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.47` | `rapier-reference/src/dynamics/joint/revolute_joint.rs` | `dim2,dim3` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.48` | `rapier-reference/src/dynamics/joint/rope_joint.rs` | `-` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.49` | `rapier-reference/src/dynamics/joint/spherical_joint.rs` | `-` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.50` | `rapier-reference/src/dynamics/joint/spring_joint.rs` | `-` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.51` | `rapier-reference/src/dynamics/mod.rs` | `dim3,parallel` | default-scope reviewed; optional slices (parallel) tracked in follow-up issue |
| `moon_rapier-gap-file-by-file-20260213.52` | `rapier-reference/src/dynamics/rigid_body.rs` | `dim2,dim3` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.53` | `rapier-reference/src/dynamics/rigid_body_components.rs` | `dim2,dim3,f32,f64` | default-scope reviewed; optional slices (f64) tracked in follow-up issue |
| `moon_rapier-gap-file-by-file-20260213.54` | `rapier-reference/src/dynamics/rigid_body_set.rs` | `dev-remove-slow-accessors` | default-scope reviewed; optional slices (dev-remove-slow-accessors) tracked in follow-up issue |
| `moon_rapier-gap-file-by-file-20260213.55` | `rapier-reference/src/dynamics/solver/categorization.rs` | `-` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.56` | `rapier-reference/src/dynamics/solver/contact_constraint/any_contact_constraint.rs` | `dim3` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.57` | `rapier-reference/src/dynamics/solver/contact_constraint/contact_constraint_element.rs` | `dim2,dim3` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.58` | `rapier-reference/src/dynamics/solver/contact_constraint/contact_constraints_set.rs` | `dim2,dim3,simd-is-enabled` | default-scope reviewed; optional slices (simd-is-enabled) tracked in follow-up issue |
| `moon_rapier-gap-file-by-file-20260213.59` | `rapier-reference/src/dynamics/solver/contact_constraint/contact_with_coulomb_friction.rs` | `dim2,dim3,simd-is-enabled` | default-scope reviewed; optional slices (simd-is-enabled) tracked in follow-up issue |
| `moon_rapier-gap-file-by-file-20260213.60` | `rapier-reference/src/dynamics/solver/contact_constraint/contact_with_twist_friction.rs` | `dim2,dim3,simd-is-enabled` | default-scope reviewed; optional slices (simd-is-enabled) tracked in follow-up issue |
| `moon_rapier-gap-file-by-file-20260213.61` | `rapier-reference/src/dynamics/solver/contact_constraint/generic_contact_constraint.rs` | `dim2,dim3` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.62` | `rapier-reference/src/dynamics/solver/contact_constraint/generic_contact_constraint_element.rs` | `dim2,dim3` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.63` | `rapier-reference/src/dynamics/solver/contact_constraint/mod.rs` | `dim3` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.64` | `rapier-reference/src/dynamics/solver/interaction_groups.rs` | `dim2,dim3,parallel,simd-is-enabled` | default-scope reviewed; optional slices (parallel, simd-is-enabled) tracked in follow-up issue |
| `moon_rapier-gap-file-by-file-20260213.65` | `rapier-reference/src/dynamics/solver/island_solver.rs` | `dim3` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.66` | `rapier-reference/src/dynamics/solver/joint_constraint/any_joint_constraint.rs` | `simd-is-enabled` | default-scope reviewed; optional slices (simd-is-enabled) tracked in follow-up issue |
| `moon_rapier-gap-file-by-file-20260213.67` | `rapier-reference/src/dynamics/solver/joint_constraint/generic_joint_constraint.rs` | `-` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.68` | `rapier-reference/src/dynamics/solver/joint_constraint/generic_joint_constraint_builder.rs` | `dim2,dim3` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.69` | `rapier-reference/src/dynamics/solver/joint_constraint/joint_constraint_builder.rs` | `dim2,dim3,simd-is-enabled` | default-scope reviewed; optional slices (simd-is-enabled) tracked in follow-up issue |
| `moon_rapier-gap-file-by-file-20260213.70` | `rapier-reference/src/dynamics/solver/joint_constraint/joint_constraints_set.rs` | `simd-is-enabled` | default-scope reviewed; optional slices (simd-is-enabled) tracked in follow-up issue |
| `moon_rapier-gap-file-by-file-20260213.71` | `rapier-reference/src/dynamics/solver/joint_constraint/joint_velocity_constraint.rs` | `dim3,simd-is-enabled` | default-scope reviewed; optional slices (simd-is-enabled) tracked in follow-up issue |
| `moon_rapier-gap-file-by-file-20260213.72` | `rapier-reference/src/dynamics/solver/joint_constraint/mod.rs` | `-` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.73` | `rapier-reference/src/dynamics/solver/mod.rs` | `parallel` | default-scope reviewed; optional slices (parallel) tracked in follow-up issue |
| `moon_rapier-gap-file-by-file-20260213.74` | `rapier-reference/src/dynamics/solver/parallel_island_solver.rs` | `-` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.75` | `rapier-reference/src/dynamics/solver/parallel_solver_constraints.rs` | `simd-is-enabled` | default-scope reviewed; optional slices (simd-is-enabled) tracked in follow-up issue |
| `moon_rapier-gap-file-by-file-20260213.76` | `rapier-reference/src/dynamics/solver/parallel_velocity_solver.rs` | `-` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.77` | `rapier-reference/src/dynamics/solver/solver_body.rs` | `dim2,dim3,simd-is-enabled` | default-scope reviewed; optional slices (simd-is-enabled) tracked in follow-up issue |
| `moon_rapier-gap-file-by-file-20260213.78` | `rapier-reference/src/dynamics/solver/velocity_solver.rs` | `dim3` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.79` | `rapier-reference/src/geometry/broad_phase_bvh.rs` | `-` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.80` | `rapier-reference/src/geometry/broad_phase_pair_event.rs` | `-` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.81` | `rapier-reference/src/geometry/collider.rs` | `dim2,dim3` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.82` | `rapier-reference/src/geometry/collider_components.rs` | `-` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.83` | `rapier-reference/src/geometry/collider_set.rs` | `dev-remove-slow-accessors` | default-scope reviewed; optional slices (dev-remove-slow-accessors) tracked in follow-up issue |
| `moon_rapier-gap-file-by-file-20260213.84` | `rapier-reference/src/geometry/contact_pair.rs` | `dim2,dim3,simd-is-enabled` | default-scope reviewed; optional slices (simd-is-enabled) tracked in follow-up issue |
| `moon_rapier-gap-file-by-file-20260213.85` | `rapier-reference/src/geometry/interaction_graph.rs` | `-` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.86` | `rapier-reference/src/geometry/interaction_groups.rs` | `-` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.87` | `rapier-reference/src/geometry/manifold_reduction.rs` | `-` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.88` | `rapier-reference/src/geometry/mesh_converter.rs` | `dim3` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.89` | `rapier-reference/src/geometry/mod.rs` | `dim3,serde-serialize` | default-scope reviewed; optional slices (serde-serialize) tracked in follow-up issue |
| `moon_rapier-gap-file-by-file-20260213.90` | `rapier-reference/src/geometry/narrow_phase.rs` | `dim2,dim3,f32,parallel` | default-scope reviewed; optional slices (parallel) tracked in follow-up issue |
| `moon_rapier-gap-file-by-file-20260213.91` | `rapier-reference/src/lib.rs` | `dim2,dim3,enhanced-determinism,f32,f64,parallel,serde-serialize,simd-is-enabled,simd-stable` | default-scope reviewed; optional slices (enhanced-determinism, f64, parallel, serde-serialize, simd-is-enabled, simd-stable) tracked in follow-up issue |
| `moon_rapier-gap-file-by-file-20260213.92` | `rapier-reference/src/pipeline/collision_pipeline.rs` | `dim2,dim3` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.93` | `rapier-reference/src/pipeline/debug_render_pipeline/debug_render_backend.rs` | `-` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.94` | `rapier-reference/src/pipeline/debug_render_pipeline/debug_render_pipeline.rs` | `dim2,dim3` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.95` | `rapier-reference/src/pipeline/debug_render_pipeline/debug_render_style.rs` | `-` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.96` | `rapier-reference/src/pipeline/debug_render_pipeline/mod.rs` | `-` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.97` | `rapier-reference/src/pipeline/debug_render_pipeline/outlines.rs` | `dim2,dim3` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.98` | `rapier-reference/src/pipeline/event_handler.rs` | `-` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.99` | `rapier-reference/src/pipeline/mod.rs` | `debug-render` | default-scope reviewed; optional slices (debug-render) tracked in follow-up issue |
| `moon_rapier-gap-file-by-file-20260213.100` | `rapier-reference/src/pipeline/physics_hooks.rs` | `-` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.101` | `rapier-reference/src/pipeline/physics_pipeline.rs` | `dim2,dim3,enhanced-determinism,parallel,serde-serialize` | default-scope reviewed; optional slices (enhanced-determinism, parallel, serde-serialize) tracked in follow-up issue |
| `moon_rapier-gap-file-by-file-20260213.102` | `rapier-reference/src/pipeline/query_pipeline.rs` | `-` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.103` | `rapier-reference/src/pipeline/user_changes.rs` | `-` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.104` | `rapier-reference/src/utils/angular_inertia_ops.rs` | `dim3,simd-is-enabled` | default-scope reviewed; optional slices (simd-is-enabled) tracked in follow-up issue |
| `moon_rapier-gap-file-by-file-20260213.105` | `rapier-reference/src/utils/component_mul.rs` | `-` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.106` | `rapier-reference/src/utils/copysign.rs` | `simd-is-enabled` | default-scope reviewed; optional slices (simd-is-enabled) tracked in follow-up issue |
| `moon_rapier-gap-file-by-file-20260213.107` | `rapier-reference/src/utils/cross_product.rs` | `dim2,dim3` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.108` | `rapier-reference/src/utils/cross_product_matrix.rs` | `dim2,dim3,simd-is-enabled` | default-scope reviewed; optional slices (simd-is-enabled) tracked in follow-up issue |
| `moon_rapier-gap-file-by-file-20260213.109` | `rapier-reference/src/utils/dot_product.rs` | `simd-is-enabled` | default-scope reviewed; optional slices (simd-is-enabled) tracked in follow-up issue |
| `moon_rapier-gap-file-by-file-20260213.110` | `rapier-reference/src/utils/fp_flags.rs` | `debug-disable-legitimate-fe-exceptions,enhanced-determinism` | default-scope reviewed; optional slices (debug-disable-legitimate-fe-exceptions, enhanced-determinism) tracked in follow-up issue |
| `moon_rapier-gap-file-by-file-20260213.111` | `rapier-reference/src/utils/index_mut2.rs` | `-` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.112` | `rapier-reference/src/utils/matrix_column.rs` | `-` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.113` | `rapier-reference/src/utils/mod.rs` | `dim2,dim3,serde-serialize,simd-is-enabled,simd-nightly,simd-stable` | default-scope reviewed; optional slices (serde-serialize, simd-is-enabled, simd-nightly, simd-stable) tracked in follow-up issue |
| `moon_rapier-gap-file-by-file-20260213.114` | `rapier-reference/src/utils/orthonormal_basis.rs` | `dim2,dim3` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.115` | `rapier-reference/src/utils/pos_ops.rs` | `dim2,dim3,simd-is-enabled` | default-scope reviewed; optional slices (simd-is-enabled) tracked in follow-up issue |
| `moon_rapier-gap-file-by-file-20260213.116` | `rapier-reference/src/utils/rotation_ops.rs` | `dim2,dim3,simd-is-enabled` | default-scope reviewed; optional slices (simd-is-enabled) tracked in follow-up issue |
| `moon_rapier-gap-file-by-file-20260213.117` | `rapier-reference/src/utils/scalar_type.rs` | `dim2,dim3,simd-is-enabled` | default-scope reviewed; optional slices (simd-is-enabled) tracked in follow-up issue |
| `moon_rapier-gap-file-by-file-20260213.118` | `rapier-reference/src/utils/simd_real_copy.rs` | `-` | default-scope reviewed |
| `moon_rapier-gap-file-by-file-20260213.119` | `rapier-reference/src/utils/simd_select.rs` | `simd-is-enabled` | default-scope reviewed; optional slices (simd-is-enabled) tracked in follow-up issue |
