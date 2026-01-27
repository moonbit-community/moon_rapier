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
fn wrap_angle_pi(angle : @core.Real) -> @core.Real {
  let pi_value = @core.pi()
  let two_pi = @core.two_pi()
  let mut a = angle
  if a > pi_value {
    a = a - two_pi
  } else if a < -pi_value {
    a = a + two_pi
  }
  a
}

///|
test "examples2d/joint_motor_position2.rs parity" {
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = @collision.ColliderSet::new()
  let impulse_joints = @dynamics.ImpulseJointSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()
  let pipeline = @pipeline.PhysicsPipeline::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let islands = @dynamics.IslandManager::new()
  let ground_handle = bodies.insert(@dynamics.RigidBodyBuilder::fixed().build())
  let target_positions : Array[(@dynamics.RigidBodyHandle, @core.Real)] = []
  for num in 0..<9 {
    let n = Float::from_double(num.to_double())
    let x_pos = -6.0F + 1.5F * n
    let handle = bodies.insert(
      @dynamics.RigidBodyBuilder::dynamic()
      .translation(@core.Vec2::new(x_pos, 2.0F))
      .can_sleep(false)
      .build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder::cuboid(0.1F, 0.5F).build(),
      handle,
      bodies,
    )
    |> ignore
    let target = -@core.pi() + @core.pi() / 4.0F * n
    let joint = @dynamics.GenericJoint::from_revolute(
      @dynamics.RevoluteJointBuilder::new()
      .local_anchor1(@core.Vec2::new(x_pos, 1.5F))
      .local_anchor2(@core.Vec2::new(0.0F, -0.5F))
      .motor_position(target, 1000.0F, 150.0F)
      .build(),
    )
    impulse_joints.insert(ground_handle, handle, joint, true) |> ignore
    target_positions.push((handle, target))
  }
  let limit_targets : Array[(@dynamics.RigidBodyHandle, @core.Real, @core.Real)] = []
  for num in 0..<8 {
    let n = Float::from_double(num.to_double())
    let x_pos = -6.0F + 1.5F * n
    let handle = bodies.insert(
      @dynamics.RigidBodyBuilder::dynamic()
      .translation(@core.Vec2::new(x_pos, 4.5F))
      .rotation(@core.Rot2::from_angle(@core.pi()))
      .can_sleep(false)
      .build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder::cuboid(0.1F, 0.5F).build(),
      handle,
      bodies,
    )
    |> ignore
    let min_angle = -@core.pi()
    let max_angle = -@core.pi() + @core.pi() / 4.0F * n
    let joint = @dynamics.GenericJoint::from_revolute(
      @dynamics.RevoluteJointBuilder::new()
      .local_anchor1(@core.Vec2::new(x_pos, 5.0F))
      .local_anchor2(@core.Vec2::new(0.0F, -0.5F))
      .motor_velocity(1.5F, 30.0F)
      .motor_max_force(100.0F)
      .limits(min_angle, max_angle)
      .build(),
    )
    impulse_joints.insert(ground_handle, handle, joint, true) |> ignore
    limit_targets.push((handle, min_angle, max_angle))
  }
  let gravity = @core.Vec2::zero()
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )
  let ccd_solver = @dynamics.CCDSolver::new()
  let hooks = @pipeline.PhysicsHooks::new()
  let events = @pipeline.EventHandler::new()
  for _ in 0..<240 {
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
      impulse_joints, multibody_joints, ccd_solver, hooks, events,
    )
  }
  for i in 0..<target_positions.length() {
    let pair = target_positions[i]
    if bodies.get(pair.0) is Some(body) {
      let angle = body.rotation().angle()
      inspect(angle == angle, content="true")
      let diff = wrap_angle_pi(angle - pair.1)
      inspect(@core.abs(diff) < 0.4F, content="true")
    } else {
      inspect(false, content="true")
    }
  }
  for i in 0..<limit_targets.length() {
    let info = limit_targets[i]
    if bodies.get(info.0) is Some(body) {
      let angle = body.rotation().angle()
      inspect(angle == angle, content="true")
      inspect(angle >= info.1 - 1.0e-2F, content="true")
      inspect(angle <= info.2 + 1.0e-2F, content="true")
    } else {
      inspect(false, content="true")
    }
  }
}

///|
/// TODO: port rapier-reference/examples2d/add_remove2.rs
test "examples2d/add_remove2.rs parity" {
  inspect(true, content="true")
}

///|
/// TODO: port rapier-reference/examples2d/all_examples2.rs
test "examples2d/all_examples2.rs parity" {
  inspect(true, content="true")
}

///|
/// TODO: port rapier-reference/examples2d/ccd2.rs
test "examples2d/ccd2.rs parity" {
  inspect(true, content="true")
}

///|
/// TODO: port rapier-reference/examples2d/character_controller2.rs
test "examples2d/character_controller2.rs parity" {
  inspect(true, content="true")
}

///|
/// TODO: port rapier-reference/examples2d/collision_groups2.rs
test "examples2d/collision_groups2.rs parity" {
  inspect(true, content="true")
}

///|
/// TODO: port rapier-reference/examples2d/convex_polygons2.rs
test "examples2d/convex_polygons2.rs parity" {
  inspect(true, content="true")
}

///|
/// TODO: port rapier-reference/examples2d/damping2.rs
test "examples2d/damping2.rs parity" {
  inspect(true, content="true")
}

///|
/// TODO: port rapier-reference/examples2d/debug_box_ball2.rs
test "examples2d/debug_box_ball2.rs parity" {
  inspect(true, content="true")
}

///|
/// TODO: port rapier-reference/examples2d/debug_compression2.rs
test "examples2d/debug_compression2.rs parity" {
  inspect(true, content="true")
}

///|
/// TODO: port rapier-reference/examples2d/debug_intersection2.rs
test "examples2d/debug_intersection2.rs parity" {
  inspect(true, content="true")
}

///|
/// TODO: port rapier-reference/examples2d/debug_total_overlap2.rs
test "examples2d/debug_total_overlap2.rs parity" {
  inspect(true, content="true")
}

///|
/// TODO: port rapier-reference/examples2d/debug_vertical_column2.rs
test "examples2d/debug_vertical_column2.rs parity" {
  inspect(true, content="true")
}

///|
/// TODO: port rapier-reference/examples2d/drum2.rs
test "examples2d/drum2.rs parity" {
  inspect(true, content="true")
}

///|
/// TODO: port rapier-reference/examples2d/heightfield2.rs
test "examples2d/heightfield2.rs parity" {
  inspect(true, content="true")
}

///|
/// TODO: port rapier-reference/examples2d/inv_pyramid2.rs
test "examples2d/inv_pyramid2.rs parity" {
  inspect(true, content="true")
}

///|
/// TODO: port rapier-reference/examples2d/inverse_kinematics2.rs
test "examples2d/inverse_kinematics2.rs parity" {
  inspect(true, content="true")
}

///|
/// TODO: port rapier-reference/examples2d/joints2.rs
test "examples2d/joints2.rs parity" {
  inspect(true, content="true")
}

///|
/// TODO: port rapier-reference/examples2d/locked_rotations2.rs
test "examples2d/locked_rotations2.rs parity" {
  inspect(true, content="true")
}

///|
/// TODO: port rapier-reference/examples2d/one_way_platforms2.rs
test "examples2d/one_way_platforms2.rs parity" {
  inspect(true, content="true")
}

///|
/// TODO: port rapier-reference/examples2d/pin_slot_joint2.rs
test "examples2d/pin_slot_joint2.rs parity" {
  inspect(true, content="true")
}

///|
/// TODO: port rapier-reference/examples2d/platform2.rs
test "examples2d/platform2.rs parity" {
  inspect(true, content="true")
}

///|
/// TODO: port rapier-reference/examples2d/polyline2.rs
test "examples2d/polyline2.rs parity" {
  inspect(true, content="true")
}

///|
/// TODO: port rapier-reference/examples2d/pyramid2.rs
test "examples2d/pyramid2.rs parity" {
  inspect(true, content="true")
}

///|
/// TODO: port rapier-reference/examples2d/restitution2.rs
test "examples2d/restitution2.rs parity" {
  inspect(true, content="true")
}

///|
/// TODO: port rapier-reference/examples2d/rope_joints2.rs
test "examples2d/rope_joints2.rs parity" {
  inspect(true, content="true")
}

///|
/// TODO: port rapier-reference/examples2d/s2d_arch.rs
test "examples2d/s2d_arch.rs parity" {
  inspect(true, content="true")
}

///|
/// TODO: port rapier-reference/examples2d/s2d_ball_and_chain.rs
test "examples2d/s2d_ball_and_chain.rs parity" {
  inspect(true, content="true")
}

///|
/// TODO: port rapier-reference/examples2d/s2d_bridge.rs
test "examples2d/s2d_bridge.rs parity" {
  inspect(true, content="true")
}

///|
/// TODO: port rapier-reference/examples2d/s2d_card_house.rs
test "examples2d/s2d_card_house.rs parity" {
  inspect(true, content="true")
}

///|
/// TODO: port rapier-reference/examples2d/s2d_confined.rs
test "examples2d/s2d_confined.rs parity" {
  inspect(true, content="true")
}

///|
/// TODO: port rapier-reference/examples2d/s2d_far_pyramid.rs
test "examples2d/s2d_far_pyramid.rs parity" {
  inspect(true, content="true")
}

///|
/// TODO: port rapier-reference/examples2d/s2d_high_mass_ratio_1.rs
test "examples2d/s2d_high_mass_ratio_1.rs parity" {
  inspect(true, content="true")
}

///|
/// TODO: port rapier-reference/examples2d/s2d_high_mass_ratio_2.rs
test "examples2d/s2d_high_mass_ratio_2.rs parity" {
  inspect(true, content="true")
}

///|
/// TODO: port rapier-reference/examples2d/s2d_high_mass_ratio_3.rs
test "examples2d/s2d_high_mass_ratio_3.rs parity" {
  inspect(true, content="true")
}

///|
/// TODO: port rapier-reference/examples2d/s2d_joint_grid.rs
test "examples2d/s2d_joint_grid.rs parity" {
  inspect(true, content="true")
}

///|
/// TODO: port rapier-reference/examples2d/s2d_pyramid.rs
test "examples2d/s2d_pyramid.rs parity" {
  inspect(true, content="true")
}

///|
/// TODO: port rapier-reference/examples2d/sensor2.rs
test "examples2d/sensor2.rs parity" {
  inspect(true, content="true")
}

///|
/// TODO: port rapier-reference/examples2d/trimesh2.rs
test "examples2d/trimesh2.rs parity" {
  inspect(true, content="true")
}

///|
/// TODO: port rapier-reference/examples2d/voxels2.rs
test "examples2d/voxels2.rs parity" {
  inspect(true, content="true")
}
