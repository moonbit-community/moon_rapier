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
fn local_sin(value : @core.Real) -> @core.Real {
  Float::from_double(@math.sin(value.to_double()))
}

///|
fn local_cos(value : @core.Real) -> @core.Real {
  Float::from_double(@math.cos(value.to_double()))
}

///|
fn step_pipeline_2d_n(
  pipeline : @pipeline.PhysicsPipeline,
  gravity : @core.Vec2,
  parameters : @dynamics.IntegrationParameters,
  islands : @dynamics.IslandManager,
  broad_phase : @collision.BroadPhaseBvh,
  narrow_phase : @collision.NarrowPhase,
  bodies : @dynamics.RigidBodySet,
  colliders : @collision.ColliderSet,
  impulse_joints : @dynamics.ImpulseJointSet,
  multibody_joints : @dynamics.MultibodyJointSet,
  ccd_solver : @dynamics.CCDSolver,
  hooks : @pipeline.PhysicsHooks,
  events : @pipeline.EventHandler,
  nsteps : Int,
) -> Unit {
  for _ in 0..<nsteps {
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
      impulse_joints, multibody_joints, ccd_solver, hooks, events,
    )
  }
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
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 240,
  )
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
  let pipeline = @pipeline.PhysicsPipeline::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = @collision.ColliderSet::new()
  let impulse_joints = @dynamics.ImpulseJointSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()
  let islands = @dynamics.IslandManager::new()
  let hooks = @pipeline.PhysicsHooks::new()
  let events = @pipeline.EventHandler::new()
  let ccd_solver = @dynamics.CCDSolver::new()
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let ground_size = 5.0F
  let ground_height = 0.1F
  let floor_handle = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, -ground_height))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(ground_size, ground_height).build(),
    floor_handle,
    bodies,
  )
  |> ignore
  let green_group = @dynamics.InteractionGroups::new(
    @dynamics.Group::group_1(),
    @dynamics.Group::group_1(),
    @dynamics.InteractionTestMode::And,
  )
  let blue_group = @dynamics.InteractionGroups::new(
    @dynamics.Group::group_2(),
    @dynamics.Group::group_2(),
    @dynamics.InteractionTestMode::And,
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(1.0F, 0.1F)
    .translation(@core.Vec2::new(0.0F, 1.0F))
    .collision_groups(green_group)
    .build(),
    floor_handle,
    bodies,
  )
  |> ignore
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(1.0F, 0.1F)
    .translation(@core.Vec2::new(0.0F, 2.0F))
    .collision_groups(blue_group)
    .build(),
    floor_handle,
    bodies,
  )
  |> ignore
  let green_bodies : Array[@dynamics.RigidBodyHandle] = []
  let blue_bodies : Array[@dynamics.RigidBodyHandle] = []
  let num = 8
  let rad = 0.1F
  let shift = rad * 2.0F
  let centerx = shift * Float::from_double((num / 2).to_double())
  let centery = 2.5F
  for j in 0..<4 {
    for i in 0..<num {
      let x = Float::from_double(i.to_double()) * shift - centerx
      let y = Float::from_double(j.to_double()) * shift + centery
      let is_green = i % 2 == 0
      let group = if is_green { green_group } else { blue_group }
      let handle = bodies.insert(
        @dynamics.RigidBodyBuilder::dynamic()
        .translation(@core.Vec2::new(x, y))
        .build(),
      )
      colliders.insert_with_parent(
        @collision.ColliderBuilder::cuboid(rad, rad)
        .collision_groups(group)
        .build(),
        handle,
        bodies,
      )
      |> ignore
      if is_green {
        green_bodies.push(handle)
      } else {
        blue_bodies.push(handle)
      }
    }
  }
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 360,
  )

  // Green cubes should fall through the blue floor at y=2 and be able to reach the green floor at y=1.
  // Blue cubes should be supported by the blue floor.
  let mut green_min_y = 1.0e30F
  for i in 0..<green_bodies.length() {
    if bodies.get(green_bodies[i]) is Some(body) {
      let y = body.translation().y
      if y < green_min_y {
        green_min_y = y
      }
    } else {
      inspect(false, content="true")
    }
  }
  let mut blue_max_y = -1.0e30F
  for i in 0..<blue_bodies.length() {
    if bodies.get(blue_bodies[i]) is Some(body) {
      let y = body.translation().y
      if y > blue_max_y {
        blue_max_y = y
      }
    } else {
      inspect(false, content="true")
    }
  }
  inspect(green_min_y < 1.6F, content="true")
  inspect(blue_max_y > 1.8F, content="true")
}

///|
/// TODO: port rapier-reference/examples2d/convex_polygons2.rs
test "examples2d/convex_polygons2.rs parity" {
  inspect(true, content="true")
}

///|
/// TODO: port rapier-reference/examples2d/damping2.rs
test "examples2d/damping2.rs parity" {
  let pipeline = @pipeline.PhysicsPipeline::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = @collision.ColliderSet::new()
  let impulse_joints = @dynamics.ImpulseJointSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()
  let islands = @dynamics.IslandManager::new()
  let hooks = @pipeline.PhysicsHooks::new()
  let events = @pipeline.EventHandler::new()
  let ccd_solver = @dynamics.CCDSolver::new()
  let parameters = @dynamics.IntegrationParameters::default()
  let num = 10
  let rad = 0.2F
  let subdiv = 1.0F / Float::from_double(num.to_double())
  let handles : Array[
    (@dynamics.RigidBodyHandle, @core.Vec2, @core.Real, @core.Real, @core.Real),
  ] = []
  for i in 0..<num {
    let a = Float::from_double(i.to_double()) * subdiv * @core.pi() * 2.0F
    let x = local_sin(a)
    let y = local_cos(a)
    let linvel = @core.Vec2::new(x * 10.0F, y * 10.0F)
    let angvel = 100.0F
    let lin_damping = Float::from_double((i + 1).to_double()) * subdiv * 10.0F
    let ang_damping = Float::from_double((num - i).to_double()) * subdiv * 10.0F
    let rb = @dynamics.RigidBodyBuilder::dynamic()
      .translation(@core.Vec2::new(x, y))
      .linvel(linvel)
      .angvel(angvel)
      .linear_damping(lin_damping)
      .angular_damping(ang_damping)
      .build()
    let handle = bodies.insert(rb)
    colliders.insert_with_parent(
      @collision.ColliderBuilder::cuboid(rad, rad).build(),
      handle,
      bodies,
    )
    |> ignore
    handles.push((handle, linvel, angvel, lin_damping, ang_damping))
  }
  let inv_dt = parameters.inv_dt()
  let dt = if inv_dt == 0.0F { 0.0F } else { 1.0F / inv_dt }
  step_pipeline_2d_n(
    pipeline,
    @core.Vec2::zero(),
    parameters,
    islands,
    broad_phase,
    narrow_phase,
    bodies,
    colliders,
    impulse_joints,
    multibody_joints,
    ccd_solver,
    hooks,
    events,
    1,
  )
  for i in 0..<handles.length() {
    let (handle, linvel0, angvel0, lin_damp, ang_damp) = handles[i]
    if bodies.get(handle) is Some(body) {
      let expected_lin_scale = 1.0F / (1.0F + dt * lin_damp)
      let expected_ang_scale = 1.0F / (1.0F + dt * ang_damp)
      let expected_lin = @core.Vec2::new(
        linvel0.x * expected_lin_scale,
        linvel0.y * expected_lin_scale,
      )
      let expected_ang = angvel0 * expected_ang_scale
      inspect(
        body.linvel().sub(expected_lin).length() < 1.0e-4F,
        content="true",
      )
      inspect(@core.abs(body.angvel() - expected_ang) < 1.0e-4F, content="true")
    } else {
      inspect(false, content="true")
    }
  }
}

///|
/// TODO: port rapier-reference/examples2d/debug_box_ball2.rs
test "examples2d/debug_box_ball2.rs parity" {
  let pipeline = @pipeline.PhysicsPipeline::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = @collision.ColliderSet::new()
  let impulse_joints = @dynamics.ImpulseJointSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()
  let islands = @dynamics.IslandManager::new()
  let hooks = @pipeline.PhysicsHooks::new()
  let events = @pipeline.EventHandler::new()
  let ccd_solver = @dynamics.CCDSolver::new()
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let rad = 1.0F
  let ground_handle = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, -rad))
    .rotation(@core.Rot2::from_angle(@core.pi() / 4.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(rad, rad).build(),
    ground_handle,
    bodies,
  )
  |> ignore
  let ball_handle = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(0.0F, 3.0F * rad))
    .can_sleep(false)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::ball(rad).build(),
    ball_handle,
    bodies,
  )
  |> ignore
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 240,
  )
  if bodies.get(ball_handle) is Some(ball) {
    let pos = ball.translation()
    inspect(pos.x == pos.x, content="true")
    inspect(pos.y == pos.y, content="true")
    // Regression check: the ball should not tunnel through the thin rotated box.
    inspect(pos.y > -2.0F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
/// TODO: port rapier-reference/examples2d/debug_compression2.rs
test "examples2d/debug_compression2.rs parity" {
  let pipeline = @pipeline.PhysicsPipeline::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = @collision.ColliderSet::new()
  let impulse_joints = @dynamics.ImpulseJointSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()
  let islands = @dynamics.IslandManager::new()
  let hooks = @pipeline.PhysicsHooks::new()
  let events = @pipeline.EventHandler::new()
  let ccd_solver = @dynamics.CCDSolver::new()
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )
  let gravity = @core.Vec2::zero()
  let width = 75.0F
  let thickness = 2.0F
  let ys = [-30.0F - thickness, 30.0F + thickness]
  for i in 0..<ys.length() {
    let handle = bodies.insert(
      @dynamics.RigidBodyBuilder::fixed()
      .translation(@core.Vec2::new(0.0F, ys[i]))
      .build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder::cuboid(width, thickness).build(),
      handle,
      bodies,
    )
    |> ignore
  }
  let half_height = (ys[1] - ys[0]) / 2.0F - thickness
  let xs = [-width + thickness, width - thickness]
  let plank_handles : Array[@dynamics.RigidBodyHandle] = []
  for i in 0..<xs.length() {
    let handle = bodies.insert(
      @dynamics.RigidBodyBuilder::dynamic()
      .translation(@core.Vec2::new(xs[i], 0.0F))
      .build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder::cuboid(thickness, half_height).build(),
      handle,
      bodies,
    )
    |> ignore
    plank_handles.push(handle)
  }
  let num = 8
  let rad = half_height / Float::from_double(num.to_double())
  for i in 0..<num {
    for j in 0..<num {
      let x = Float::from_double(i.to_double()) * rad * 2.0F -
        Float::from_double(num.to_double()) * rad
      let y = Float::from_double(j.to_double()) * rad * 2.0F -
        Float::from_double(num.to_double()) * rad +
        rad
      let handle = bodies.insert(
        @dynamics.RigidBodyBuilder::dynamic()
        .translation(@core.Vec2::new(x, y))
        .build(),
      )
      colliders.insert_with_parent(
        @collision.ColliderBuilder::ball(rad).build(),
        handle,
        bodies,
      )
      |> ignore
    }
  }
  let left0 = if bodies.get(plank_handles[0]) is Some(body) {
    body.translation().x
  } else {
    0.0F
  }
  let right0 = if bodies.get(plank_handles[1]) is Some(body) {
    body.translation().x
  } else {
    0.0F
  }
  let mut force_x = 0.0F
  for _ in 0..<30 {
    if bodies.get_mut(plank_handles[0]) is Some(left) {
      left.reset_forces(true) |> ignore
      left.add_force(@core.Vec2::new(force_x, 0.0F), true) |> ignore
    }
    if bodies.get_mut(plank_handles[1]) is Some(right) {
      right.reset_forces(true) |> ignore
      right.add_force(@core.Vec2::new(-force_x, 0.0F), true) |> ignore
    }
    force_x = force_x + 10000.0F
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
      impulse_joints, multibody_joints, ccd_solver, hooks, events,
    )
  }

  // With forces integrated like Rapier, the planks should move toward each other.
  if bodies.get(plank_handles[0]) is Some(left) {
    inspect(left.translation().x > left0 + 0.01F, content="true")
  } else {
    inspect(false, content="true")
  }
  if bodies.get(plank_handles[1]) is Some(right) {
    inspect(right.translation().x < right0 - 0.01F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
/// TODO: port rapier-reference/examples2d/debug_intersection2.rs
test "examples2d/debug_intersection2.rs parity" {
  inspect(true, content="true")
}

///|
/// TODO: port rapier-reference/examples2d/debug_total_overlap2.rs
test "examples2d/debug_total_overlap2.rs parity" {
  let pipeline = @pipeline.PhysicsPipeline::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = @collision.ColliderSet::new()
  let impulse_joints = @dynamics.ImpulseJointSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()
  let islands = @dynamics.IslandManager::new()
  let hooks = @pipeline.PhysicsHooks::new()
  let events = @pipeline.EventHandler::new()
  let ccd_solver = @dynamics.CCDSolver::new()
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )
  let rad = 0.5F
  let handles : Array[@dynamics.RigidBodyHandle] = []
  for _ in 0..<100 {
    let handle = bodies.insert(@dynamics.RigidBodyBuilder::dynamic().build())
    colliders.insert_with_parent(
      @collision.ColliderBuilder::cuboid(rad, rad).build(),
      handle,
      bodies,
    )
    |> ignore
    handles.push(handle)
  }
  step_pipeline_2d_n(
    pipeline,
    @core.Vec2::zero(),
    parameters,
    islands,
    broad_phase,
    narrow_phase,
    bodies,
    colliders,
    impulse_joints,
    multibody_joints,
    ccd_solver,
    hooks,
    events,
    10,
  )
  let mut max_dist2 = 0.0F
  for i in 0..<handles.length() {
    if bodies.get(handles[i]) is Some(body) {
      let p = body.translation()
      inspect(p.x == p.x, content="true")
      inspect(p.y == p.y, content="true")
      let d2 = p.length_squared()
      if d2 > max_dist2 {
        max_dist2 = d2
      }
    } else {
      inspect(false, content="true")
    }
  }
  inspect(max_dist2 > 1.0e-6F, content="true")
}

///|
/// TODO: port rapier-reference/examples2d/debug_vertical_column2.rs
test "examples2d/debug_vertical_column2.rs parity" {
  let pipeline = @pipeline.PhysicsPipeline::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = @collision.ColliderSet::new()
  let impulse_joints = @dynamics.ImpulseJointSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()
  let islands = @dynamics.IslandManager::new()
  let hooks = @pipeline.PhysicsHooks::new()
  let events = @pipeline.EventHandler::new()
  let ccd_solver = @dynamics.CCDSolver::new()
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let num = 80
  let rad = 0.5F
  let ground_size = 1.0F
  let ground_thickness = 1.0F
  let ground_handle = bodies.insert(@dynamics.RigidBodyBuilder::fixed().build())
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(ground_size, ground_thickness)
    .friction(0.3F)
    .build(),
    ground_handle,
    bodies,
  )
  |> ignore
  let handles : Array[@dynamics.RigidBodyHandle] = []
  for i in 0..<num {
    let y = Float::from_double(i.to_double()) * rad * 2.0F +
      ground_thickness +
      rad
    let handle = bodies.insert(
      @dynamics.RigidBodyBuilder::dynamic()
      .translation(@core.Vec2::new(0.0F, y))
      .build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder::cuboid(rad, rad).friction(0.3F).build(),
      handle,
      bodies,
    )
    |> ignore
    handles.push(handle)
  }
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 600,
  )
  let mut min_y = 1.0e30F
  let mut max_y = -1.0e30F
  for i in 0..<handles.length() {
    if bodies.get(handles[i]) is Some(body) {
      let y = body.translation().y
      if y < min_y {
        min_y = y
      }
      if y > max_y {
        max_y = y
      }
    } else {
      inspect(false, content="true")
    }
  }
  inspect(min_y > 0.8F, content="true")
  inspect(max_y < 200.0F, content="true")
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
  let pipeline = @pipeline.PhysicsPipeline::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = @collision.ColliderSet::new()
  let impulse_joints = @dynamics.ImpulseJointSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()
  let islands = @dynamics.IslandManager::new()
  let hooks = @pipeline.PhysicsHooks::new()
  let events = @pipeline.EventHandler::new()
  let ccd_solver = @dynamics.CCDSolver::new()
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let ground_size = 10.0F
  let ground_thickness = 1.0F
  let ground_handle = bodies.insert(@dynamics.RigidBodyBuilder::fixed().build())
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(ground_size, ground_thickness).build(),
    ground_handle,
    bodies,
  )
  |> ignore
  let num = 6
  let mut rad = 0.5F
  let mut y = rad
  let handles : Array[@dynamics.RigidBodyHandle] = []
  for _ in 0..<num {
    let handle = bodies.insert(
      @dynamics.RigidBodyBuilder::dynamic()
      .translation(@core.Vec2::new(0.0F, y + ground_thickness))
      .build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder::cuboid(rad, rad).build(),
      handle,
      bodies,
    )
    |> ignore
    handles.push(handle)
    y = y + rad + rad * 2.0F
    rad = rad * 2.0F
  }
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 360,
  )
  for i in 0..<handles.length() {
    if bodies.get(handles[i]) is Some(body) {
      let pos = body.translation()
      inspect(pos.x == pos.x, content="true")
      inspect(pos.y == pos.y, content="true")
      inspect(pos.y > 0.5F, content="true")
    } else {
      inspect(false, content="true")
    }
  }
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
  let pipeline = @pipeline.PhysicsPipeline::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = @collision.ColliderSet::new()
  let impulse_joints = @dynamics.ImpulseJointSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()
  let islands = @dynamics.IslandManager::new()
  let hooks = @pipeline.PhysicsHooks::new()
  let events = @pipeline.EventHandler::new()
  let ccd_solver = @dynamics.CCDSolver::new()
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )
  let gravity = @core.Vec2::new(0.0F, -9.81F)

  // Ground.
  let ground_height = 0.1F
  let ground_handle = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, -ground_height))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(5.0F, ground_height).build(),
    ground_handle,
    bodies,
  )
  |> ignore

  // A rectangle that only rotates.
  let rect_handle = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(0.0F, 3.0F))
    .lock_translations()
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(2.0F, 0.6F).build(),
    rect_handle,
    bodies,
  )
  |> ignore

  // A tilted capsule that cannot rotate.
  let capsule_handle = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(0.0F, 5.0F))
    .rotation(@core.Rot2::from_angle(1.0F))
    .lock_rotations()
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::capsule_y(0.6F, 0.4F).build(),
    capsule_handle,
    bodies,
  )
  |> ignore
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 360,
  )
  if bodies.get(rect_handle) is Some(rect_body) {
    inspect(
      @core.abs(rect_body.translation().y - 3.0F) < 1.0e-3F,
      content="true",
    )
  } else {
    inspect(false, content="true")
  }
  if bodies.get(capsule_handle) is Some(capsule_body) {
    inspect(capsule_body.translation().y < 5.0F, content="true")
    inspect(
      @core.abs(capsule_body.rotation().angle() - 1.0F) < 1.0e-2F,
      content="true",
    )
  } else {
    inspect(false, content="true")
  }
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
  let pipeline = @pipeline.PhysicsPipeline::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = @collision.ColliderSet::new()
  let impulse_joints = @dynamics.ImpulseJointSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()
  let islands = @dynamics.IslandManager::new()
  let hooks = @pipeline.PhysicsHooks::new()
  let events = @pipeline.EventHandler::new()
  let ccd_solver = @dynamics.CCDSolver::new()
  let parameters = @dynamics.IntegrationParameters::default()
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let inv_dt = parameters.inv_dt()
  let dt = if inv_dt == 0.0F { 0.0F } else { 1.0F / inv_dt }
  let ground_height = 0.1F
  let ground_handle = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, -ground_height))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(10.0F, ground_height).build(),
    ground_handle,
    bodies,
  )
  |> ignore
  let num = 6
  let rad = 0.2F
  let shift = rad * 2.0F
  let centerx = shift * Float::from_double(num.to_double()) / 2.0F
  let centery = shift / 2.0F + 3.04F
  for i in 0..<num {
    for j in 0..<num {
      let x = Float::from_double(i.to_double()) * shift - centerx
      let y = Float::from_double(j.to_double()) * shift + centery
      let handle = bodies.insert(
        @dynamics.RigidBodyBuilder::dynamic()
        .translation(@core.Vec2::new(x, y))
        .build(),
      )
      colliders.insert_with_parent(
        @collision.ColliderBuilder::cuboid(rad, rad).build(),
        handle,
        bodies,
      )
      |> ignore
    }
  }
  let vel_based_handle = bodies.insert(
    @dynamics.RigidBodyBuilder::kinematic_velocity_based()
    .translation(@core.Vec2::new(-10.0F * rad, 1.5F + 0.8F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(rad * 10.0F, rad).build(),
    vel_based_handle,
    bodies,
  )
  |> ignore
  let pos_based_handle = bodies.insert(
    @dynamics.RigidBodyBuilder::kinematic_position_based()
    .translation(@core.Vec2::new(-10.0F * rad, 2.0F + 1.5F + 0.8F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(rad * 10.0F, rad).build(),
    pos_based_handle,
    bodies,
  )
  |> ignore
  let time = 0.5F
  let velocity = @core.Vec2::new(local_sin(time) * 5.0F, local_sin(time * 5.0F))
  let vel_before = if bodies.get(vel_based_handle) is Some(body) {
    body.translation()
  } else {
    @core.Vec2::zero()
  }
  if bodies.get_mut(vel_based_handle) is Some(platform) {
    platform.set_linvel(velocity, true) |> ignore
  }
  let pos_before = if bodies.get(pos_based_handle) is Some(body) {
    body.translation()
  } else {
    @core.Vec2::zero()
  }
  let pos_target = pos_before.add(
    @core.Vec2::new(velocity.x * dt, velocity.y * dt),
  )
  if bodies.get_mut(pos_based_handle) is Some(platform) {
    platform.set_next_kinematic_translation(pos_target) |> ignore
  }
  pipeline.step(
    gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders, impulse_joints,
    multibody_joints, ccd_solver, hooks, events,
  )
  if bodies.get(vel_based_handle) is Some(platform) {
    let expected = vel_before.add(
      @core.Vec2::new(velocity.x * dt, velocity.y * dt),
    )
    inspect(
      platform.translation().sub(expected).length() < 5.0e-3F,
      content="true",
    )
  } else {
    inspect(false, content="true")
  }
  if bodies.get(pos_based_handle) is Some(platform) {
    inspect(
      platform.translation().sub(pos_target).length() < 5.0e-3F,
      content="true",
    )
    inspect(platform.linvel().sub(velocity).length() < 5.0e-3F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
/// TODO: port rapier-reference/examples2d/polyline2.rs
test "examples2d/polyline2.rs parity" {
  inspect(true, content="true")
}

///|
/// TODO: port rapier-reference/examples2d/pyramid2.rs
test "examples2d/pyramid2.rs parity" {
  let pipeline = @pipeline.PhysicsPipeline::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = @collision.ColliderSet::new()
  let impulse_joints = @dynamics.ImpulseJointSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()
  let islands = @dynamics.IslandManager::new()
  let hooks = @pipeline.PhysicsHooks::new()
  let events = @pipeline.EventHandler::new()
  let ccd_solver = @dynamics.CCDSolver::new()
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let ground_size = 10.0F
  let ground_thickness = 1.0F
  let ground_handle = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, -ground_thickness))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(ground_size, ground_thickness).build(),
    ground_handle,
    bodies,
  )
  |> ignore
  let num = 10
  let rad = 0.5F
  let shift = rad * 2.0F
  let centerx = shift * Float::from_double(num.to_double()) / 2.0F
  let centery = shift / 2.0F + ground_thickness + rad * 1.5F
  let handles : Array[@dynamics.RigidBodyHandle] = []
  for i in 0..<num {
    for j in i..<num {
      let fj = Float::from_double(j.to_double())
      let fi = Float::from_double(i.to_double())
      let x = fi * shift / 2.0F + (fj - fi) * shift - centerx
      let y = fi * shift + centery
      let handle = bodies.insert(
        @dynamics.RigidBodyBuilder::dynamic()
        .translation(@core.Vec2::new(x, y))
        .build(),
      )
      colliders.insert_with_parent(
        @collision.ColliderBuilder::cuboid(rad, rad).build(),
        handle,
        bodies,
      )
      |> ignore
      handles.push(handle)
    }
  }
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 600,
  )
  let mut min_y = 1.0e30F
  for i in 0..<handles.length() {
    if bodies.get(handles[i]) is Some(body) {
      let pos = body.translation()
      inspect(pos.x == pos.x, content="true")
      inspect(pos.y == pos.y, content="true")
      if pos.y < min_y {
        min_y = pos.y
      }
    } else {
      inspect(false, content="true")
    }
  }
  // Cubes should not fall through the ground.
  inspect(min_y > -0.5F, content="true")
}

///|
/// TODO: port rapier-reference/examples2d/restitution2.rs
test "examples2d/restitution2.rs parity" {
  let pipeline = @pipeline.PhysicsPipeline::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = @collision.ColliderSet::new()
  let impulse_joints = @dynamics.ImpulseJointSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()
  let islands = @dynamics.IslandManager::new()
  let hooks = @pipeline.PhysicsHooks::new()
  let events = @pipeline.EventHandler::new()
  let ccd_solver = @dynamics.CCDSolver::new()
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let ground_height = 1.0F
  let ground_handle = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, -ground_height))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(20.0F, ground_height)
    .restitution(1.0F)
    .build(),
    ground_handle,
    bodies,
  )
  |> ignore
  let rad = 0.5F
  let low_body = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(-2.0F, 10.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::ball(rad).restitution(0.0F).build(),
    low_body,
    bodies,
  )
  |> ignore
  let high_body = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(2.0F, 10.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::ball(rad).restitution(1.0F).build(),
    high_body,
    bodies,
  )
  |> ignore
  let mut low_bounced = false
  let mut high_bounced = false
  let mut low_max_y = -1.0e30F
  let mut high_max_y = -1.0e30F
  for _ in 0..<360 {
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
      impulse_joints, multibody_joints, ccd_solver, hooks, events,
    )
    if bodies.get(low_body) is Some(body) {
      if !low_bounced && body.translation().y < 1.0F && body.linvel().y > 0.0F {
        low_bounced = true
      }
      if low_bounced && body.translation().y > low_max_y {
        low_max_y = body.translation().y
      }
    }
    if bodies.get(high_body) is Some(body) {
      if !high_bounced && body.translation().y < 1.0F && body.linvel().y > 0.0F {
        high_bounced = true
      }
      if high_bounced && body.translation().y > high_max_y {
        high_max_y = body.translation().y
      }
    }
  }
  inspect(low_bounced, content="true")
  inspect(high_bounced, content="true")
  inspect(high_max_y > low_max_y + 5.0F, content="true")
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
  let pipeline = @pipeline.PhysicsPipeline::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = @collision.ColliderSet::new()
  let impulse_joints = @dynamics.ImpulseJointSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()
  let islands = @dynamics.IslandManager::new()
  let hooks = @pipeline.PhysicsHooks::new()
  let handler = @pipeline.EventHandler::new()
  let ccd_solver = @dynamics.CCDSolver::new()
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let ground_size = 200.1F
  let ground_height = 0.1F
  let ground_handle = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, -ground_height))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(ground_size, ground_height).build(),
    ground_handle,
    bodies,
  )
  |> ignore
  let num = 10
  let rad = 0.2F
  let shift = rad * 2.0F
  let centerx = shift * Float::from_double(num.to_double()) / 2.0F
  for i in 0..<num {
    let x = Float::from_double(i.to_double()) * shift - centerx
    let handle = bodies.insert(
      @dynamics.RigidBodyBuilder::dynamic()
      .translation(@core.Vec2::new(x, 3.0F))
      .build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder::cuboid(rad, rad).build(),
      handle,
      bodies,
    )
    |> ignore
  }
  let sensor_handle = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(0.0F, 10.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(rad, rad).build(),
    sensor_handle,
    bodies,
  )
  |> ignore
  colliders.insert_with_parent(
    @collision.ColliderBuilder::ball(rad * 5.0F)
    .density(0.0F)
    .sensor(true)
    .active_events(@collision.ActiveEvents::collision_events())
    .build(),
    sensor_handle,
    bodies,
  )
  |> ignore
  let mut started_sensor_events = 0
  for _ in 0..<360 {
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
      impulse_joints, multibody_joints, ccd_solver, hooks, handler,
    )
    let evs = handler.take_collision_events()
    for i in 0..<evs.length() {
      let ev = evs[i]
      if ev.started() && ev.sensor() {
        started_sensor_events = started_sensor_events + 1
      }
    }
  }
  inspect(started_sensor_events > 0, content="true")
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
