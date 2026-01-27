// Copyright 2025 International Digital Economy Academy
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

///|
/// Parity tests for `rapier-reference/examples3d/*.rs`.
///
/// NOTE: This package is currently 2D-first. We still land the test surface
/// early (red is ok) using `declare` stubs so we can implement 3D incrementally.
declare fn run_examples3d_all_examples3() -> Unit

///|
declare fn run_examples3d_ccd3() -> Unit

///|
declare fn run_examples3d_character_controller3() -> Unit

///|
declare fn run_examples3d_collision_groups3() -> Unit

///|
declare fn run_examples3d_compound3() -> Unit

///|
declare fn run_examples3d_convex_decomposition3() -> Unit

///|
declare fn run_examples3d_convex_polyhedron3() -> Unit

///|
declare fn run_examples3d_damping3() -> Unit

///|
declare fn run_examples3d_debug_add_remove_collider3() -> Unit

///|
declare fn run_examples3d_debug_articulations3() -> Unit

///|
declare fn run_examples3d_debug_balls3() -> Unit

///|
declare fn run_examples3d_debug_big_colliders3() -> Unit

///|
declare fn run_examples3d_debug_boxes3() -> Unit

///|
declare fn run_examples3d_debug_chain_high_mass_ratio3() -> Unit

///|
declare fn run_examples3d_debug_cube_high_mass_ratio3() -> Unit

///|
declare fn run_examples3d_debug_cylinder3() -> Unit

///|
declare fn run_examples3d_debug_deserialize3() -> Unit

///|
declare fn run_examples3d_debug_disabled3() -> Unit

///|
declare fn run_examples3d_debug_dynamic_collider_add3() -> Unit

///|
declare fn run_examples3d_debug_eccentric_boxes3() -> Unit

///|
declare fn run_examples3d_debug_friction3() -> Unit

///|
declare fn run_examples3d_debug_infinite_fall3() -> Unit

///|
declare fn run_examples3d_debug_internal_edges3() -> Unit

///|
declare fn run_examples3d_debug_long_chain3() -> Unit

///|
declare fn run_examples3d_debug_multibody_ang_motor_pos3() -> Unit

///|
declare fn run_examples3d_debug_pop3() -> Unit

///|
declare fn run_examples3d_debug_prismatic3() -> Unit

///|
declare fn run_examples3d_debug_rollback3() -> Unit

///|
declare fn run_examples3d_debug_shape_modification3() -> Unit

///|
declare fn run_examples3d_debug_sleeping_kinematic3() -> Unit

///|
declare fn run_examples3d_debug_thin_cube_on_mesh3() -> Unit

///|
declare fn run_examples3d_debug_triangle3() -> Unit

///|
declare fn run_examples3d_debug_trimesh3() -> Unit

///|
declare fn run_examples3d_debug_two_cubes3() -> Unit

///|
declare fn run_examples3d_domino3() -> Unit

///|
declare fn run_examples3d_dynamic_trimesh3() -> Unit

///|
declare fn run_examples3d_fountain3() -> Unit

///|
declare fn run_examples3d_gyroscopic3() -> Unit

///|
declare fn run_examples3d_harness_capsules3() -> Unit

///|
declare fn run_examples3d_heightfield3() -> Unit

///|
declare fn run_examples3d_inverse_kinematics3() -> Unit

///|
declare fn run_examples3d_joint_motor_position3() -> Unit

///|
declare fn run_examples3d_joints3() -> Unit

///|
declare fn run_examples3d_keva3() -> Unit

///|
declare fn run_examples3d_locked_rotations3() -> Unit

///|
declare fn run_examples3d_newton_cradle3() -> Unit

///|
declare fn run_examples3d_one_way_platforms3() -> Unit

///|
declare fn run_examples3d_platform3() -> Unit

///|
declare fn run_examples3d_primitives3() -> Unit

///|
declare fn run_examples3d_restitution3() -> Unit

///|
declare fn run_examples3d_rope_joints3() -> Unit

///|
declare fn run_examples3d_sensor3() -> Unit

///|
declare fn run_examples3d_spring_joints3() -> Unit

///|
declare fn run_examples3d_trimesh3() -> Unit

///|
declare fn run_examples3d_urdf3() -> Unit

///|
declare fn run_examples3d_vehicle_controller3() -> Unit

///|
declare fn run_examples3d_vehicle_joints3() -> Unit

///|
declare fn run_examples3d_voxels3() -> Unit

///|
test "examples3d/all_examples3.rs parity" {
  run_examples3d_all_examples3()
}

///|
test "examples3d/ccd3.rs parity" {
  run_examples3d_ccd3()
}

///|
test "examples3d/character_controller3.rs parity" {
  run_examples3d_character_controller3()
}

///|
test "examples3d/collision_groups3.rs parity" {
  run_examples3d_collision_groups3()
}

///|
test "examples3d/compound3.rs parity" {
  run_examples3d_compound3()
}

///|
test "examples3d/convex_decomposition3.rs parity" {
  run_examples3d_convex_decomposition3()
}

///|
test "examples3d/convex_polyhedron3.rs parity" {
  run_examples3d_convex_polyhedron3()
}

///|
test "examples3d/damping3.rs parity" {
  run_examples3d_damping3()
}

///|
test "examples3d/debug_add_remove_collider3.rs parity" {
  run_examples3d_debug_add_remove_collider3()
}

///|
test "examples3d/debug_articulations3.rs parity" {
  run_examples3d_debug_articulations3()
}

///|
test "examples3d/debug_balls3.rs parity" {
  run_examples3d_debug_balls3()
}

///|
test "examples3d/debug_big_colliders3.rs parity" {
  run_examples3d_debug_big_colliders3()
}

///|
test "examples3d/debug_boxes3.rs parity" {
  run_examples3d_debug_boxes3()
}

///|
test "examples3d/debug_chain_high_mass_ratio3.rs parity" {
  run_examples3d_debug_chain_high_mass_ratio3()
}

///|
test "examples3d/debug_cube_high_mass_ratio3.rs parity" {
  run_examples3d_debug_cube_high_mass_ratio3()
}

///|
test "examples3d/debug_cylinder3.rs parity" {
  run_examples3d_debug_cylinder3()
}

///|
test "examples3d/debug_deserialize3.rs parity" {
  run_examples3d_debug_deserialize3()
}

///|
test "examples3d/debug_disabled3.rs parity" {
  run_examples3d_debug_disabled3()
}

///|
test "examples3d/debug_dynamic_collider_add3.rs parity" {
  run_examples3d_debug_dynamic_collider_add3()
}

///|
test "examples3d/debug_eccentric_boxes3.rs parity" {
  run_examples3d_debug_eccentric_boxes3()
}

///|
test "examples3d/debug_friction3.rs parity" {
  run_examples3d_debug_friction3()
}

///|
test "examples3d/debug_infinite_fall3.rs parity" {
  run_examples3d_debug_infinite_fall3()
}

///|
test "examples3d/debug_internal_edges3.rs parity" {
  run_examples3d_debug_internal_edges3()
}

///|
test "examples3d/debug_long_chain3.rs parity" {
  run_examples3d_debug_long_chain3()
}

///|
test "examples3d/debug_multibody_ang_motor_pos3.rs parity" {
  run_examples3d_debug_multibody_ang_motor_pos3()
}

///|
test "examples3d/debug_pop3.rs parity" {
  run_examples3d_debug_pop3()
}

///|
test "examples3d/debug_prismatic3.rs parity" {
  run_examples3d_debug_prismatic3()
}

///|
test "examples3d/debug_rollback3.rs parity" {
  run_examples3d_debug_rollback3()
}

///|
test "examples3d/debug_shape_modification3.rs parity" {
  run_examples3d_debug_shape_modification3()
}

///|
test "examples3d/debug_sleeping_kinematic3.rs parity" {
  run_examples3d_debug_sleeping_kinematic3()
}

///|
test "examples3d/debug_thin_cube_on_mesh3.rs parity" {
  run_examples3d_debug_thin_cube_on_mesh3()
}

///|
test "examples3d/debug_triangle3.rs parity" {
  run_examples3d_debug_triangle3()
}

///|
test "examples3d/debug_trimesh3.rs parity" {
  run_examples3d_debug_trimesh3()
}

///|
test "examples3d/debug_two_cubes3.rs parity" {
  run_examples3d_debug_two_cubes3()
}

///|
test "examples3d/domino3.rs parity" {
  run_examples3d_domino3()
}

///|
test "examples3d/dynamic_trimesh3.rs parity" {
  run_examples3d_dynamic_trimesh3()
}

///|
test "examples3d/fountain3.rs parity" {
  run_examples3d_fountain3()
}

///|
test "examples3d/gyroscopic3.rs parity" {
  run_examples3d_gyroscopic3()
}

///|
test "examples3d/harness_capsules3.rs parity" {
  run_examples3d_harness_capsules3()
}

///|
test "examples3d/heightfield3.rs parity" {
  run_examples3d_heightfield3()
}

///|
test "examples3d/inverse_kinematics3.rs parity" {
  run_examples3d_inverse_kinematics3()
}

///|
test "examples3d/joint_motor_position3.rs parity" {
  run_examples3d_joint_motor_position3()
}

///|
test "examples3d/joints3.rs parity" {
  run_examples3d_joints3()
}

///|
test "examples3d/keva3.rs parity" {
  run_examples3d_keva3()
}

///|
test "examples3d/locked_rotations3.rs parity" {
  run_examples3d_locked_rotations3()
}

///|
test "examples3d/newton_cradle3.rs parity" {
  run_examples3d_newton_cradle3()
}

///|
test "examples3d/one_way_platforms3.rs parity" {
  run_examples3d_one_way_platforms3()
}

///|
test "examples3d/platform3.rs parity" {
  run_examples3d_platform3()
}

///|
test "examples3d/primitives3.rs parity" {
  run_examples3d_primitives3()
}

///|
test "examples3d/restitution3.rs parity" {
  run_examples3d_restitution3()
}

///|
test "examples3d/rope_joints3.rs parity" {
  run_examples3d_rope_joints3()
}

///|
test "examples3d/sensor3.rs parity" {
  run_examples3d_sensor3()
}

///|
test "examples3d/spring_joints3.rs parity" {
  run_examples3d_spring_joints3()
}

///|
test "examples3d/trimesh3.rs parity" {
  run_examples3d_trimesh3()
}

///|
test "examples3d/urdf3.rs parity" {
  run_examples3d_urdf3()
}

///|
test "examples3d/vehicle_controller3.rs parity" {
  run_examples3d_vehicle_controller3()
}

///|
test "examples3d/vehicle_joints3.rs parity" {
  run_examples3d_vehicle_joints3()
}

///|
test "examples3d/voxels3.rs parity" {
  run_examples3d_voxels3()
}
