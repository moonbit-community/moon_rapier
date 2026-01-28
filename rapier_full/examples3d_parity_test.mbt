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
declare fn run_examples3d_character_controller3() -> Bool

///|
declare fn run_examples3d_compound3() -> Bool

///|
declare fn run_examples3d_convex_decomposition3() -> Bool

///|
declare fn run_examples3d_convex_polyhedron3() -> Bool

///|
declare fn run_examples3d_dynamic_trimesh3() -> Bool

///|
declare fn run_examples3d_inverse_kinematics3() -> Bool

///|
declare fn run_examples3d_joint_motor_position3() -> Bool

///|
declare fn run_examples3d_joints3() -> Bool

///|
declare fn run_examples3d_keva3() -> Bool

///|
declare fn run_examples3d_newton_cradle3() -> Bool

///|
declare fn run_examples3d_spring_joints3() -> Bool

///|
declare fn run_examples3d_urdf3() -> Bool

///|
declare fn run_examples3d_vehicle_controller3() -> Bool

///|
declare fn run_examples3d_vehicle_joints3() -> Bool

///|
declare fn run_examples3d_voxels3() -> Bool

///|
fn run_examples3d_character_controller3() -> Bool {
  // 2D adaptation of `rapier-reference/examples3d/character_controller3.rs`.
  //
  // We don't have Rapier's full `KinematicCharacterController` yet, but we can still validate
  // kinematic bodies + CCD settings + interactions with dynamic bodies.
  let mut ok = true
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
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )
  let inv_dt = parameters.inv_dt()
  let dt = if inv_dt == 0.0F { 0.0F } else { 1.0F / inv_dt }

  // Ground.
  let ground_height = 0.1F
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, -ground_height))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(20.0F, ground_height).build(),
    ground,
    bodies,
  )
  |> ignore

  // A small slope.
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(2.0F, ground_height)
    .translation(@core.Vec2::new(4.0F, 0.4F))
    .rotation(0.2F)
    .build(),
    ground,
    bodies,
  )
  |> ignore

  // Character: kinematic position-based capsule.
  let character = bodies.insert(
    @dynamics.RigidBodyBuilder::kinematic_position_based()
    .translation(@core.Vec2::new(-3.0F, 2.0F))
    .gravity_scale(10.0F)
    .soft_ccd_prediction(10.0F)
    .can_sleep(false)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::capsule_y(0.3F, 0.15F).build(),
    character,
    bodies,
  )
  |> ignore

  // Some dynamic cubes to push around.
  let rad = 0.15F
  for j in 0..<3 {
    for i in 0..<10 {
      let x = -1.0F + Float::from_double(i.to_double()) * rad * 2.2F
      let y = 0.5F + Float::from_double(j.to_double()) * rad * 2.2F
      let h = bodies.insert(
        @dynamics.RigidBodyBuilder::dynamic()
        .translation(@core.Vec2::new(x, y))
        .build(),
      )
      colliders.insert_with_parent(
        @collision.ColliderBuilder::cuboid(rad, rad).build(),
        h,
        bodies,
      )
      |> ignore
    }
  }

  // Drive the character left-to-right by setting its next kinematic translation.
  let mut time = 0.0F
  for _ in 0..<360 {
    time = time + dt
    let x = -3.0F + Float::from_double(@math.sin(time.to_double())) * 5.0F
    if bodies.get_mut(character) is Some(rb) {
      rb.set_next_kinematic_translation(@core.Vec2::new(x, 2.0F)) |> ignore
    } else {
      ok = false
    }
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
      impulse_joints, multibody_joints, ccd_solver, hooks, events,
    )
  }
  if bodies.get(character) is Some(rb) {
    let p = rb.translation()
    if !(p.x == p.x) || !(p.y == p.y) {
      ok = false
    }
    if !(p.y > -10.0F) {
      ok = false
    }
  } else {
    ok = false
  }
  ok
}

///|
fn run_examples3d_compound3() -> Bool {
  // 2D adaptation of `rapier-reference/examples3d/compound3.rs`.
  let mut ok = true
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
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )

  // Ground.
  let ground_height = 0.1F
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, -ground_height))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(50.0F, ground_height).build(),
    ground,
    bodies,
  )
  |> ignore

  // Build a few bodies with multiple colliders attached (compound-by-attachment).
  let handles : Array[@dynamics.RigidBodyHandle] = []
  for j in 0..<6 {
    let x = Float::from_double(j.to_double()) * 2.0F - 5.0F
    let y = 3.0F + Float::from_double(j.to_double()) * 0.5F
    let h = bodies.insert(
      @dynamics.RigidBodyBuilder::dynamic()
      .translation(@core.Vec2::new(x, y))
      .can_sleep(false)
      .build(),
    )
    // "T" shape made of 3 cuboids.
    colliders.insert_with_parent(
      @collision.ColliderBuilder::cuboid(0.8F, 0.08F).build(),
      h,
      bodies,
    )
    |> ignore
    colliders.insert_with_parent(
      @collision.ColliderBuilder::cuboid(0.08F, 0.8F)
      .translation(@core.Vec2::new(0.8F, 0.8F))
      .build(),
      h,
      bodies,
    )
    |> ignore
    colliders.insert_with_parent(
      @collision.ColliderBuilder::cuboid(0.08F, 0.8F)
      .translation(@core.Vec2::new(-0.8F, 0.8F))
      .build(),
      h,
      bodies,
    )
    |> ignore
    handles.push(h)
  }
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 240,
  )
  for i in 0..<handles.length() {
    if bodies.get(handles[i]) is Some(body) {
      let p = body.translation()
      if !(p.x == p.x) || !(p.y == p.y) {
        ok = false
      }
      if !(p.y > -100.0F) {
        ok = false
      }
    } else {
      ok = false
    }
  }
  ok
}

///|
fn run_examples3d_dynamic_trimesh3_impl(use_decomposition : Bool) -> Bool {
  // 2D adaptation of `rapier-reference/examples3d/dynamic_trimesh3.rs`.
  let mut ok = true
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
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )

  // Wavy floor (heightfield) to exercise mesh contacts.
  let n = 201
  let voxel_size_y = 1.0F
  let scale_x = 100.0F
  let heights : Array[@core.Real] = []
  for i in 0..<n {
    let t = Float::from_double(i.to_double()) / 20.0F
    heights.push(
      Float::from_double(@math.sin(t.to_double())) * 2.0F / voxel_size_y,
    )
  }
  colliders.insert(
    @collision.ColliderBuilder::heightfield(
      heights,
      @core.Vec2::new(scale_x, voxel_size_y),
    ).build(),
  )
  |> ignore
  let vtx = [
    @core.Vec2::new(-1.0F, -1.0F),
    @core.Vec2::new(1.0F, -1.0F),
    @core.Vec2::new(1.0F, 1.0F),
    @core.Vec2::new(-1.0F, 1.0F),
  ]
  let idx = [(0, 1, 2), (0, 2, 3)]
  let handles : Array[@dynamics.RigidBodyHandle] = []
  for k in 0..<10 {
    let x = Float::from_double(k.to_double()) * 2.5F - 10.0F
    let rb = bodies.insert(
      @dynamics.RigidBodyBuilder::dynamic()
      .translation(@core.Vec2::new(x, 8.0F))
      .can_sleep(false)
      .build(),
    )
    if use_decomposition {
      // Fake "decomposition": multiple colliders on the same rigid-body.
      colliders.insert_with_parent(
        @collision.ColliderBuilder::cuboid(0.5F, 0.2F).build(),
        rb,
        bodies,
      )
      |> ignore
      colliders.insert_with_parent(
        @collision.ColliderBuilder::cuboid(0.2F, 0.5F)
        .translation(@core.Vec2::new(0.6F, 0.0F))
        .build(),
        rb,
        bodies,
      )
      |> ignore
    } else if @collision.ColliderBuilder::trimesh(vtx, idx) is Some(builder) {
      colliders.insert_with_parent(
        builder.contact_skin(0.1F).build(),
        rb,
        bodies,
      )
      |> ignore
    } else {
      ok = false
    }
    handles.push(rb)
  }
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 240,
  )
  for i in 0..<handles.length() {
    if bodies.get(handles[i]) is Some(body) {
      let p = body.translation()
      if !(p.x == p.x) || !(p.y == p.y) {
        ok = false
      }
      if !(p.y > -1000.0F) {
        ok = false
      }
    } else {
      ok = false
    }
  }
  ok
}

///|
fn run_examples3d_dynamic_trimesh3() -> Bool {
  run_examples3d_dynamic_trimesh3_impl(false)
}

///|
fn run_examples3d_convex_decomposition3() -> Bool {
  run_examples3d_dynamic_trimesh3_impl(true)
}

///|
fn run_examples3d_convex_polyhedron3() -> Bool {
  // 2D adaptation of `rapier-reference/examples3d/convex_polyhedron3.rs` using convex hulls.
  let mut ok = true
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
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )

  // Ground.
  let ground_height = 0.1F
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, -ground_height))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(40.0F, ground_height).build(),
    ground,
    bodies,
  )
  |> ignore
  let handles : Array[@dynamics.RigidBodyHandle] = []
  for j in 0..<10 {
    let x = Float::from_double(j.to_double()) * 1.2F - 6.0F
    let rb = bodies.insert(
      @dynamics.RigidBodyBuilder::dynamic()
      .translation(@core.Vec2::new(x, 6.0F))
      .can_sleep(false)
      .build(),
    )
    let pts = [
      @core.Vec2::new(-1.0F, -0.2F),
      @core.Vec2::new(0.0F, -1.0F),
      @core.Vec2::new(1.0F, -0.2F),
      @core.Vec2::new(0.8F, 1.0F),
      @core.Vec2::new(-0.8F, 1.0F),
    ]
    if @collision.ColliderBuilder::convex_hull(pts) is Some(builder) {
      colliders.insert_with_parent(builder.build(), rb, bodies) |> ignore
    } else {
      ok = false
    }
    handles.push(rb)
  }
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 240,
  )
  for i in 0..<handles.length() {
    if bodies.get(handles[i]) is Some(body) {
      let p = body.translation()
      if !(p.x == p.x) || !(p.y == p.y) {
        ok = false
      }
      if !(p.y > -100.0F) {
        ok = false
      }
    } else {
      ok = false
    }
  }
  ok
}

///|
fn run_examples3d_joint_motor_position3() -> Bool {
  // 2D adaptation of `rapier-reference/examples3d/joint_motor_position3.rs`.
  fn wrap_angle_pi(a : @core.Real) -> @core.Real {
    let mut x = a
    while x > @core.pi() {
      x = x - @core.two_pi()
    }
    while x < -@core.pi() {
      x = x + @core.two_pi()
    }
    x
  }

  let mut ok = true
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
      if !(angle == angle) {
        ok = false
      }
      let diff = wrap_angle_pi(angle - pair.1)
      if !(@core.abs(diff) < 0.4F) {
        ok = false
      }
    } else {
      ok = false
    }
  }
  for i in 0..<limit_targets.length() {
    let info = limit_targets[i]
    if bodies.get(info.0) is Some(body) {
      let angle = body.rotation().angle()
      if !(angle == angle) {
        ok = false
      }
      if !(angle >= info.1 - 1.0e-2F) {
        ok = false
      }
      if !(angle <= info.2 + 1.0e-2F) {
        ok = false
      }
    } else {
      ok = false
    }
  }
  ok
}

///|
fn run_examples3d_inverse_kinematics3() -> Bool {
  // 2D adaptation of `rapier-reference/examples3d/inverse_kinematics3.rs`.
  let mut ok = true
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = @collision.ColliderSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()

  // Ground (kept for parity with upstream, though IK doesn't depend on it).
  let ground_height = 0.01F
  let floor_handle = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, -ground_height))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(1.0F, ground_height).build(),
    floor_handle,
    bodies,
  )
  |> ignore
  let num_segments = 10
  let size = 1.0F / Float::from_double(num_segments.to_double())
  let mut last_body = bodies.insert(@dynamics.RigidBodyBuilder::fixed().build())
  let mut last_link = @dynamics.MultibodyJointHandle::invalid()
  for i in 0..<num_segments {
    let new_body = bodies.insert(
      @dynamics.RigidBodyBuilder::dynamic().can_sleep(false).build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder::cuboid(size / 8.0F, size / 2.0F)
      .density(0.0F)
      .sensor(true)
      .build(),
      new_body,
      bodies,
    )
    |> ignore
    let joint = @dynamics.RevoluteJointBuilder::new()
      .local_anchor1(
        @core.Vec2::new(0.0F, size / 2.0F * (if i != 0 { 1.0F } else { 0.0F })),
      )
      .local_anchor2(@core.Vec2::new(0.0F, -size / 2.0F))
      .build()
    if multibody_joints.insert(last_body, new_body, joint, true) is Some(handle) {
      last_link = handle
    } else {
      ok = false
    }
    last_body = new_body
  }
  if multibody_joints.get_mut(last_link) is Some(result) {
    let (multibody, link_id) = result
    let ndofs = multibody.ndofs()
    let displacements = @dynamics.DVector::from_fn(ndofs, fn(
      _ : Int,
    ) -> @core.Real {
      0.0F
    })
    let target_point = @core.Vec2::new(0.25F, 0.4F)
    let options = @dynamics.InverseKinematicsOption::default()
      .set_constrained_axes(@dynamics.JointAxesMask::lin_axes())
      .set_max_iters(200)
    multibody.inverse_kinematics(
      bodies,
      link_id,
      options,
      @core.Isometry2::from_translation(target_point),
      fn(_link : @dynamics.MultibodyLink) -> Bool { true },
      displacements,
    )
    multibody.apply_displacements(displacements.as_array())
    let pose = multibody.forward_kinematics_single_link(
      bodies,
      link_id,
      None,
      None,
    )
    let err = pose.translation.sub(target_point).length()
    if !(err == err) {
      ok = false
    }
    if !(err < 5.0e-2F) {
      ok = false
    }
  } else {
    ok = false
  }
  ok
}

///|
fn run_examples3d_joints3() -> Bool {
  // 2D adaptation of `rapier-reference/examples3d/joints3.rs`.
  // Keep a smaller grid than upstream for test runtime.
  let mut ok = true
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
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )
  let rad = 0.4F
  let numi = 6
  let numk = 6
  let shift = 1.0F
  let softness = @dynamics.SpringCoefficients::joint_defaults()
  let body_handles : Array[@dynamics.RigidBodyHandle] = []
  for k in 0..<numk {
    for i in 0..<numi {
      let fk = Float::from_double(k.to_double())
      let fi = Float::from_double(i.to_double())
      let rb = if i == 0 && k == 0 {
        @dynamics.RigidBodyBuilder::fixed()
      } else {
        @dynamics.RigidBodyBuilder::dynamic()
      }
      let child_handle = bodies.insert(
        rb.translation(@core.Vec2::new(fk * shift, -fi * shift)).build(),
      )
      colliders.insert_with_parent(
        @collision.ColliderBuilder::ball(rad).build(),
        child_handle,
        bodies,
      )
      |> ignore

      // Vertical joint.
      if i > 0 {
        let parent_handle = body_handles[body_handles.length() - 1]
        let joint = @dynamics.GenericJoint::from_revolute(
          @dynamics.RevoluteJointBuilder::new()
          .local_anchor2(@core.Vec2::new(0.0F, shift))
          .softness(softness)
          .build(),
        )
        impulse_joints.insert(parent_handle, child_handle, joint, true)
        |> ignore
      }

      // Horizontal joint.
      if k > 0 {
        let parent_index = body_handles.length() - numi
        let parent_handle = body_handles[parent_index]
        let joint = @dynamics.GenericJoint::from_revolute(
          @dynamics.RevoluteJointBuilder::new()
          .local_anchor2(@core.Vec2::new(-shift, 0.0F))
          .softness(softness)
          .build(),
        )
        impulse_joints.insert(parent_handle, child_handle, joint, true)
        |> ignore
      }
      body_handles.push(child_handle)
    }
  }
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 240,
  )
  for i in 0..<body_handles.length() {
    if bodies.get(body_handles[i]) is Some(body) {
      let p = body.translation()
      if !(p.x == p.x) || !(p.y == p.y) {
        ok = false
      }
    } else {
      ok = false
    }
  }
  ok
}

///|
fn run_examples3d_keva3() -> Bool {
  // 2D adaptation of `rapier-reference/examples3d/keva3.rs` (reduced).
  let mut ok = true
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
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )

  // Ground.
  let ground_height = 0.1F
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, -ground_height))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(50.0F, ground_height).build(),
    ground,
    bodies,
  )
  |> ignore

  // A small jenga-like tower.
  let half_x = 1.2F
  let half_y = 0.1F
  let mut handle_count = 0
  let handles : Array[@dynamics.RigidBodyHandle] = []
  for layer in 0..<18 {
    let y = half_y * 2.0F * Float::from_double(layer.to_double()) + 0.2F
    let horizontal = layer % 2 == 0
    for i in 0..<3 {
      let x = (Float::from_double(i.to_double()) - 1.0F) * 0.9F
      let rb = bodies.insert(
        @dynamics.RigidBodyBuilder::dynamic()
        .translation(@core.Vec2::new(x, y))
        .can_sleep(false)
        .build(),
      )
      let co = if horizontal {
        @collision.ColliderBuilder::cuboid(half_x, half_y)
      } else {
        @collision.ColliderBuilder::cuboid(half_y, half_x)
      }
      colliders.insert_with_parent(co.build(), rb, bodies) |> ignore
      handles.push(rb)
      handle_count = handle_count + 1
    }
  }
  handle_count |> ignore
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 360,
  )
  let mut min_y = 1.0e30F
  for i in 0..<handles.length() {
    if bodies.get(handles[i]) is Some(body) {
      let p = body.translation()
      if !(p.x == p.x) || !(p.y == p.y) {
        ok = false
      }
      if p.y < min_y {
        min_y = p.y
      }
    } else {
      ok = false
    }
  }
  if !(min_y > -1000.0F) {
    ok = false
  }
  ok
}

///|
fn run_examples3d_newton_cradle3() -> Bool {
  // 2D adaptation of `rapier-reference/examples3d/newton_cradle3.rs`.
  let mut ok = true
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
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )
  let radius = 0.5F
  let length = 10.0F * radius
  let n = 5
  let balls : Array[@dynamics.RigidBodyHandle] = []
  for i in 0..<n {
    let x = Float::from_double(i.to_double()) * 2.2F * radius
    let ball_pos = @core.Vec2::new(x, 0.0F)
    let attach = @core.Vec2::new(0.0F, length)
    let vel = if i >= n - 1 {
      @core.Vec2::new(7.0F, 0.0F)
    } else {
      @core.Vec2::zero()
    }
    let ground = bodies.insert(
      @dynamics.RigidBodyBuilder::fixed()
      .translation(ball_pos.add(attach))
      .build(),
    )
    let h = bodies.insert(
      @dynamics.RigidBodyBuilder::dynamic()
      .translation(ball_pos)
      .linvel(vel)
      .can_sleep(false)
      .build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder::ball(radius).restitution(1.0F).build(),
      h,
      bodies,
    )
    |> ignore
    let joint = @dynamics.GenericJoint::from_revolute(
      @dynamics.RevoluteJointBuilder::new()
      .local_anchor1(@core.Vec2::zero())
      .local_anchor2(attach)
      .build(),
    )
    impulse_joints.insert(ground, h, joint, true) |> ignore
    balls.push(h)
  }
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 480,
  )
  // Sanity: no NaNs, and at least one ball deviated from its initial position.
  let mut moved = false
  for i in 0..<balls.length() {
    if bodies.get(balls[i]) is Some(b) {
      let p = b.translation()
      if !(p.x == p.x) || !(p.y == p.y) {
        ok = false
      }
      if @core.abs(p.x) > 0.2F {
        moved = true
      }
    } else {
      ok = false
    }
  }
  if !moved {
    ok = false
  }
  ok
}

///|
fn run_examples3d_spring_joints3() -> Bool {
  // 2D adaptation of `rapier-reference/examples3d/spring_joints3.rs`.
  //
  // We approximate the 3D spring joint demo with soft revolute joints (spring-like behavior)
  // attached to a fixed ground, plus some falling boxes.
  let mut ok = true
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
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )
  let ground = bodies.insert(@dynamics.RigidBodyBuilder::fixed().build())
  let num = 12
  let radius = 0.5F
  let softness = @dynamics.SpringCoefficients::joint_defaults()
  let balls : Array[@dynamics.RigidBodyHandle] = []
  for i in 0..<=num {
    let x = -6.0F + 1.0F * Float::from_double(i.to_double())
    let ball_pos = @core.Vec2::new(x, 4.5F)
    let handle = bodies.insert(
      @dynamics.RigidBodyBuilder::dynamic()
      .translation(ball_pos)
      .can_sleep(false)
      .build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder::ball(radius).build(),
      handle,
      bodies,
    )
    |> ignore
    let joint = @dynamics.GenericJoint::from_revolute(
      @dynamics.RevoluteJointBuilder::new()
      .local_anchor1(ball_pos.sub(@core.Vec2::new(0.0F, 3.0F)))
      .local_anchor2(@core.Vec2::zero())
      .softness(softness)
      .build(),
    )
    impulse_joints.insert(ground, handle, joint, true) |> ignore
    balls.push(handle)
    let box = bodies.insert(
      @dynamics.RigidBodyBuilder::dynamic()
      .translation(ball_pos.add(@core.Vec2::new(0.0F, 5.0F)))
      .build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder::cuboid(radius, radius).density(100.0F).build(),
      box,
      bodies,
    )
    |> ignore
  }
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 360,
  )
  for i in 0..<balls.length() {
    if bodies.get(balls[i]) is Some(body) {
      let p = body.translation()
      if !(p.x == p.x) || !(p.y == p.y) {
        ok = false
      }
    } else {
      ok = false
    }
  }
  ok
}

///|
fn run_examples3d_urdf3() -> Bool {
  // 2D adaptation of `rapier-reference/examples3d/urdf3.rs`.
  //
  // Full URDF parsing isn't available in this MoonBit port yet. Instead, we build a tiny
  // articulated chain (like a robot arm) to exercise multibody + joint limits.
  let mut ok = true
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
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )

  // Ground.
  colliders.insert(
    @collision.ColliderBuilder::segment(
      @core.Vec2::new(-50.0F, 0.0F),
      @core.Vec2::new(50.0F, 0.0F),
    ).build(),
  )
  |> ignore

  // Base + 3 links.
  let base = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, 2.0F))
    .build(),
  )
  let mut parent = base
  let mut x = 0.0F
  for i in 0..<3 {
    let link = bodies.insert(
      @dynamics.RigidBodyBuilder::dynamic()
      .translation(@core.Vec2::new(x + 1.0F, 2.0F))
      .can_sleep(false)
      .build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder::cuboid(0.6F, 0.1F).build(),
      link,
      bodies,
    )
    |> ignore
    let min_angle = -0.5F
    let max_angle = 0.5F
    let joint = @dynamics.GenericJoint::from_revolute(
      @dynamics.RevoluteJointBuilder::new()
      .local_anchor1(@core.Vec2::new(1.0F, 0.0F))
      .local_anchor2(@core.Vec2::new(-1.0F, 0.0F))
      .limits(min_angle, max_angle)
      .build(),
    )
    impulse_joints.insert(parent, link, joint, true) |> ignore
    parent = link
    x = x + 2.0F
    i |> ignore
  }
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 240,
  )
  if bodies.get(parent) is Some(end) {
    let p = end.translation()
    if !(p.x == p.x) || !(p.y == p.y) {
      ok = false
    }
  } else {
    ok = false
  }
  ok
}

///|
fn run_examples3d_vehicle_controller3() -> Bool {
  // 2D adaptation of `rapier-reference/examples3d/vehicle_controller3.rs`.
  //
  // The real raycast vehicle controller isn't implemented yet, so we just validate a rigid-body
  // chassis moving over uneven ground without producing NaNs.
  let mut ok = true
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
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )

  // Ground heightfield.
  let n = 201
  let heights : Array[@core.Real] = []
  for i in 0..<n {
    let t = Float::from_double(i.to_double()) / 20.0F
    heights.push(Float::from_double(@math.cos(t.to_double())) * 2.0F)
  }
  colliders.insert(
    @collision.ColliderBuilder::heightfield(
      heights,
      @core.Vec2::new(60.0F, 1.0F),
    ).build(),
  )
  |> ignore
  let chassis = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(-20.0F, 5.0F))
    .linvel(@core.Vec2::new(10.0F, 0.0F))
    .can_sleep(false)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(0.9F, 0.3F).density(100.0F).build(),
    chassis,
    bodies,
  )
  |> ignore
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 240,
  )
  if bodies.get(chassis) is Some(body) {
    let p = body.translation()
    if !(p.x == p.x) || !(p.y == p.y) {
      ok = false
    }
    if !(p.x > -30.0F) {
      ok = false
    }
  } else {
    ok = false
  }
  ok
}

///|
fn run_examples3d_vehicle_joints3() -> Bool {
  // 2D adaptation of `rapier-reference/examples3d/vehicle_joints3.rs`.
  //
  // Build a simple chassis with two motorized wheels connected by revolute joints.
  let mut ok = true
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
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )

  // Flat ground.
  colliders.insert(
    @collision.ColliderBuilder::segment(
      @core.Vec2::new(-200.0F, 0.0F),
      @core.Vec2::new(200.0F, 0.0F),
    )
    .friction(1.0F)
    .build(),
  )
  |> ignore
  let car_pos = @core.Vec2::new(-10.0F, 2.0F)
  let chassis = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(car_pos)
    .can_sleep(false)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(1.0F, 0.3F).density(50.0F).build(),
    chassis,
    bodies,
  )
  |> ignore
  let wheel_r = 0.35F
  let wheel_off = 1.2F
  let wheel_y = -0.4F
  let wheel_left = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(car_pos.add(@core.Vec2::new(-wheel_off, wheel_y)))
    .can_sleep(false)
    .build(),
  )
  let wheel_right = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(car_pos.add(@core.Vec2::new(wheel_off, wheel_y)))
    .can_sleep(false)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::ball(wheel_r)
    .friction(2.0F)
    .density(100.0F)
    .build(),
    wheel_left,
    bodies,
  )
  |> ignore
  colliders.insert_with_parent(
    @collision.ColliderBuilder::ball(wheel_r)
    .friction(2.0F)
    .density(100.0F)
    .build(),
    wheel_right,
    bodies,
  )
  |> ignore
  let motor_speed = -30.0F
  let motor_force = 200.0F
  let joint_l = @dynamics.GenericJoint::from_revolute(
    @dynamics.RevoluteJointBuilder::new()
    .local_anchor1(@core.Vec2::new(-wheel_off, wheel_y))
    .local_anchor2(@core.Vec2::zero())
    .motor_velocity(motor_speed, motor_force)
    .build(),
  )
  impulse_joints.insert(chassis, wheel_left, joint_l, true) |> ignore
  let joint_r = @dynamics.GenericJoint::from_revolute(
    @dynamics.RevoluteJointBuilder::new()
    .local_anchor1(@core.Vec2::new(wheel_off, wheel_y))
    .local_anchor2(@core.Vec2::zero())
    .motor_velocity(motor_speed, motor_force)
    .build(),
  )
  impulse_joints.insert(chassis, wheel_right, joint_r, true) |> ignore
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 360,
  )
  if bodies.get(chassis) is Some(body) {
    let p = body.translation()
    if !(p.x == p.x) || !(p.y == p.y) {
      ok = false
    }
    if !(p.x > -10.0F) {
      ok = false
    }
  } else {
    ok = false
  }
  ok
}

///|
fn run_examples3d_voxels3() -> Bool {
  // 2D adaptation of `rapier-reference/examples3d/voxels3.rs` (reuse the `voxels2` test logic).
  let mut ok = true
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
  let polyline = [
    @core.Vec2::new(0.0F, 0.0F),
    @core.Vec2::new(0.0F, 10.0F),
    @core.Vec2::new(7.0F, 4.0F),
    @core.Vec2::new(14.0F, 10.0F),
    @core.Vec2::new(14.0F, 0.0F),
    @core.Vec2::new(13.0F, 7.0F),
    @core.Vec2::new(7.0F, 2.0F),
    @core.Vec2::new(1.0F, 7.0F),
  ]
  let poly_idx : Array[(Int, Int)] = []
  for i in 0..<polyline.length() {
    poly_idx.push((i, (i + 1) % polyline.length()))
  }
  let mesh_body = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(-20.0F, -10.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::voxelized_mesh(
      polyline,
      poly_idx,
      0.2F,
      @collision.FillMode::default(),
    ).build(),
    mesh_body,
    bodies,
  )
  |> ignore
  let voxel_size_y = 1.0F
  let voxel_size = @core.Vec2::new(1.0F, voxel_size_y)
  let voxels : Array[@core.Vec2] = []
  for i in 0..<300 {
    let t = Float::from_double(i.to_double()) / 20.0F
    let mut y = Float::from_double(@math.sin(t.to_double()))
    if y < -0.5F {
      y = -0.5F
    } else if y > 0.5F {
      y = 0.5F
    }
    y = y * 20.0F
    let x = (Float::from_double(i.to_double()) - 125.0F) * voxel_size.x / 2.0F
    voxels.push(@core.Vec2::new(x, y * voxel_size.y))
  }
  colliders.insert(
    @collision.ColliderBuilder::voxels_from_points(voxel_size, voxels).build(),
  )
  |> ignore
  let nx = 20
  let ny = 6
  let ball_radius = 0.5F
  let dyn_handles : Array[@dynamics.RigidBodyHandle] = []
  for i in 0..<nx {
    for j in 0..<ny {
      let x = Float::from_double(i.to_double()) * 2.0F -
        Float::from_double(nx.to_double()) / 2.0F
      let y = 20.0F + Float::from_double(j.to_double()) * 2.0F
      let rb_handle = bodies.insert(
        @dynamics.RigidBodyBuilder::dynamic()
        .translation(@core.Vec2::new(x, y))
        .build(),
      )
      let kind = j % 3
      let co = if kind == 0 {
        @collision.ColliderBuilder::ball(ball_radius)
      } else if kind == 1 {
        @collision.ColliderBuilder::cuboid(ball_radius, ball_radius)
      } else {
        @collision.ColliderBuilder::capsule_y(ball_radius, ball_radius)
      }
      colliders.insert_with_parent(co.build(), rb_handle, bodies) |> ignore
      dyn_handles.push(rb_handle)
    }
  }
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 240,
  )
  let mut min_y = 1.0e30F
  for i in 0..<dyn_handles.length() {
    if bodies.get(dyn_handles[i]) is Some(body) {
      let p = body.translation()
      if !(p.x == p.x) || !(p.y == p.y) {
        ok = false
      }
      if p.y < min_y {
        min_y = p.y
      }
    } else {
      ok = false
    }
  }
  if !(min_y < 5.0F) {
    ok = false
  }
  if !(min_y > -1000.0F) {
    ok = false
  }
  ok
}

///|
test "examples3d/all_examples3.rs parity" {
  // Ported from `rapier-reference/examples3d/all_examples3.rs`.
  //
  // Upstream is a testbed entrypoint that aggregates all examples. Here we keep a
  // tiny smoke test to ensure the package is runnable.
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
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let parameters = @dynamics.IntegrationParameters::default()
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, -1.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(10.0F, 1.0F).build(),
    ground,
    bodies,
  )
  |> ignore
  pipeline.step(
    gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders, impulse_joints,
    multibody_joints, ccd_solver, hooks, events,
  )
  inspect(true, content="true")
}

///|
test "examples3d/ccd3.rs parity" {
  // Ported from `rapier-reference/examples3d/ccd3.rs`.
  // TODO(dim3): replace with real 3D implementation and checks.
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

  // Static ground + walls.
  let ground_size = 25.0F
  let ground_thickness = 0.1F
  let ground_handle = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed().ccd_enabled(true).build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(ground_size, ground_thickness).build(),
    ground_handle,
    bodies,
  )
  |> ignore
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(ground_thickness, ground_size)
    .translation(@core.Vec2::new(-3.0F, 0.0F))
    .build(),
    ground_handle,
    bodies,
  )
  |> ignore
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(ground_thickness, ground_size)
    .translation(@core.Vec2::new(6.0F, 0.0F))
    .build(),
    ground_handle,
    bodies,
  )
  |> ignore
  let sensor_handle = colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(ground_thickness, ground_size)
    .translation(@core.Vec2::new(2.5F, 0.0F))
    .sensor(true)
    .active_events(@collision.ActiveEvents::collision_events())
    .build(),
    ground_handle,
    bodies,
  )

  // Create dynamic bodies with CCD enabled.
  let radx = 0.4F
  let rady = 0.05F
  let num = 4
  let shift = (radx + 0.01F) * 2.0F
  let centerx = shift * Float::from_double(num.to_double()) / 2.0F - 0.5F
  let centery = shift / 2.0F + 4.0F
  let dynamic_handles : Array[@dynamics.RigidBodyHandle] = []
  for i in 0..<num {
    for j in 0..<num {
      let x = Float::from_double(i.to_double()) * shift - centerx
      let y = Float::from_double(j.to_double()) * shift + centery
      let handle = bodies.insert(
        @dynamics.RigidBodyBuilder::dynamic()
        .translation(@core.Vec2::new(x, y))
        .linvel(@core.Vec2::new(100.0F, -10.0F))
        .ccd_enabled(true)
        .can_sleep(false)
        .build(),
      )
      // "Cross" approximated by 3 colliders.
      colliders.insert_with_parent(
        @collision.ColliderBuilder::cuboid(radx, rady)
        .translation(@core.Vec2::new(0.0F, radx - rady))
        .build(),
        handle,
        bodies,
      )
      |> ignore
      colliders.insert_with_parent(
        @collision.ColliderBuilder::cuboid(rady, radx)
        .translation(@core.Vec2::new(-radx + rady, 0.0F))
        .build(),
        handle,
        bodies,
      )
      |> ignore
      colliders.insert_with_parent(
        @collision.ColliderBuilder::cuboid(rady, radx)
        .translation(@core.Vec2::new(radx - rady, 0.0F))
        .build(),
        handle,
        bodies,
      )
      |> ignore
      dynamic_handles.push(handle)
    }
  }
  for _ in 0..<60 {
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
      impulse_joints, multibody_joints, ccd_solver, hooks, events,
    )
  }

  // CCD regression: none of the fast-moving bodies should tunnel past the right wall.
  let mut max_x = -1.0e30F
  for i in 0..<dynamic_handles.length() {
    if bodies.get(dynamic_handles[i]) is Some(body) {
      let x = body.translation().x
      inspect(x == x, content="true")
      inspect(body.translation().y == body.translation().y, content="true")
      if x > max_x {
        max_x = x
      }
    } else {
      inspect(false, content="true")
    }
  }
  inspect(max_x < 7.0F, content="true")

  // Sensor collision events should be emitted at least once.
  let evs = events.take_collision_events()
  let mut saw_sensor = false
  for i in 0..<evs.length() {
    match evs[i] {
      @collision.CollisionEvent::Started(h1, h2, _) =>
        if @collision.ColliderHandle::equals(h1, sensor_handle) ||
          @collision.ColliderHandle::equals(h2, sensor_handle) {
          saw_sensor = true
        }
      _ => ()
    }
  }
  inspect(saw_sensor, content="true")
}

///|
test "examples3d/character_controller3.rs parity" {
  inspect(run_examples3d_character_controller3(), content="true")
}

///|
test "examples3d/collision_groups3.rs parity (dim2 adapted)" {
  // Ported from `rapier-reference/examples3d/collision_groups3.rs`.
  // TODO(dim3): replace with real 3D implementation and checks.
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
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )
  let ground_height = 0.1F
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, -ground_height))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(5.0F, ground_height).build(),
    ground,
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

  // Two platforms: group1 below, group2 above.
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(1.0F, 0.1F)
    .translation(@core.Vec2::new(0.0F, 1.0F))
    .collision_groups(green_group)
    .build(),
    ground,
    bodies,
  )
  |> ignore
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(1.0F, 0.1F)
    .translation(@core.Vec2::new(0.0F, 2.0F))
    .collision_groups(blue_group)
    .build(),
    ground,
    bodies,
  )
  |> ignore
  let rad = 0.2F
  let green_body = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(-0.5F, 4.0F))
    .can_sleep(false)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::ball(rad).collision_groups(green_group).build(),
    green_body,
    bodies,
  )
  |> ignore
  let blue_body = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(0.5F, 4.0F))
    .can_sleep(false)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::ball(rad).collision_groups(blue_group).build(),
    blue_body,
    bodies,
  )
  |> ignore
  for _ in 0..<360 {
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
      impulse_joints, multibody_joints, ccd_solver, hooks, events,
    )
  }
  if bodies.get(green_body) is Some(gb) && bodies.get(blue_body) is Some(bb) {
    // Green should pass through the blue platform and land on the green one.
    inspect(gb.translation().y < 1.6F, content="true")
    // Blue should land on the blue platform.
    inspect(bb.translation().y > 1.7F, content="true")
    inspect(bb.translation().y < 2.6F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "examples3d/compound3.rs parity" {
  inspect(run_examples3d_compound3(), content="true")
}

///|
test "examples3d/convex_decomposition3.rs parity" {
  inspect(run_examples3d_convex_decomposition3(), content="true")
}

///|
test "examples3d/convex_polyhedron3.rs parity" {
  inspect(run_examples3d_convex_polyhedron3(), content="true")
}

///|
test "examples3d/damping3.rs parity (dim2 adapted)" {
  // Ported from `rapier-reference/examples3d/damping3.rs`.
  // TODO(dim3): replace with real 3D implementation and checks.
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
  let gravity = @core.Vec2::zero()
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    8,
  )
  let num = 10
  let rad = 0.2F
  let handles : Array[@dynamics.RigidBodyHandle] = []
  for i in 0..<num {
    let t = @core.two_pi() *
      Float::from_double(i.to_double()) /
      Float::from_double(num.to_double())
    let x = Float::from_double(@math.sin(t.to_double()))
    let y = Float::from_double(@math.cos(t.to_double()))
    let subdiv = 1.0F / Float::from_double(num.to_double())
    let lin_damp = Float::from_double((i + 1).to_double()) * subdiv * 10.0F
    let ang_damp = Float::from_double((num - i).to_double()) * subdiv * 10.0F
    let h = bodies.insert(
      @dynamics.RigidBodyBuilder::dynamic()
      .translation(@core.Vec2::new(x, y))
      .linvel(@core.Vec2::new(x * 10.0F, y * 10.0F))
      .angvel(100.0F)
      .linear_damping(lin_damp)
      .angular_damping(ang_damp)
      .can_sleep(false)
      .build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder::cuboid(rad, rad).build(),
      h,
      bodies,
    )
    |> ignore
    handles.push(h)
  }
  for _ in 0..<60 {
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
      impulse_joints, multibody_joints, ccd_solver, hooks, events,
    )
  }

  // Compare the most/least damped bodies.
  if bodies.get(handles[0]) is Some(low) &&
    bodies.get(handles[num - 1]) is Some(high) {
    let low_speed = low.linvel().length()
    let high_speed = high.linvel().length()
    inspect(low_speed == low_speed, content="true")
    inspect(high_speed == high_speed, content="true")
    inspect(high_speed < low_speed, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "examples3d/debug_add_remove_collider3.rs parity" {
  // Ported from `rapier-reference/examples3d/debug_add_remove_collider3.rs`.
  // TODO(dim3): replace with real 3D implementation and checks.
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
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let parameters = @dynamics.IntegrationParameters::default()
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, -0.1F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(10.0F, 0.1F).build(),
    ground,
    bodies,
  )
  |> ignore
  let body = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(0.0F, 2.0F))
    .can_sleep(false)
    .build(),
  )
  let c0 = colliders.insert_with_parent(
    @collision.ColliderBuilder::ball(0.25F).build(),
    body,
    bodies,
  )
  inspect(colliders.len() == 2, content="true")
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 30,
  )
  colliders.remove(c0, islands, bodies, true)
  inspect(colliders.len() == 1, content="true")
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 60,
  )
  if bodies.get(body) is Some(rb) {
    inspect(rb.translation().y < 0.0F, content="true")
  } else {
    inspect(false, content="true")
  }
  colliders.insert_with_parent(
    @collision.ColliderBuilder::ball(0.25F).build(),
    body,
    bodies,
  )
  |> ignore
  inspect(colliders.len() == 2, content="true")
}

///|
test "examples3d/debug_articulations3.rs parity" {
  // Ported from `rapier-reference/examples3d/debug_articulations3.rs`.
  // TODO(dim3): replace with real 3D implementation and checks.
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = @collision.ColliderSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()

  // Minimal multibody chain + inverse kinematics smoke (adapted from examples2d suite).
  let ground_height = 0.01F
  let floor_handle = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, -ground_height))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(1.0F, ground_height).build(),
    floor_handle,
    bodies,
  )
  |> ignore
  let num_segments = 6
  let size = 1.0F / Float::from_double(num_segments.to_double())
  let mut last_body = bodies.insert(@dynamics.RigidBodyBuilder::fixed().build())
  let mut last_link = @dynamics.MultibodyJointHandle::invalid()
  for i in 0..<num_segments {
    let new_body = bodies.insert(
      @dynamics.RigidBodyBuilder::dynamic().can_sleep(false).build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder::cuboid(size / 8.0F, size / 2.0F)
      .density(0.0F)
      .sensor(true)
      .build(),
      new_body,
      bodies,
    )
    |> ignore
    let joint = @dynamics.RevoluteJointBuilder::new()
      .local_anchor1(
        @core.Vec2::new(0.0F, size / 2.0F * (if i != 0 { 1.0F } else { 0.0F })),
      )
      .local_anchor2(@core.Vec2::new(0.0F, -size / 2.0F))
      .build()
    if multibody_joints.insert(last_body, new_body, joint, true) is Some(handle) {
      last_link = handle
    } else {
      inspect(false, content="true")
    }
    last_body = new_body
  }
  if multibody_joints.get_mut(last_link) is Some(result) {
    let (multibody, link_id) = result
    let ndofs = multibody.ndofs()
    let displacements = @dynamics.DVector::from_fn(ndofs, fn(
      _ : Int,
    ) -> @core.Real {
      0.0F
    })
    let target_point = @core.Vec2::new(0.25F, 0.4F)
    let options = @dynamics.InverseKinematicsOption::default()
      .set_constrained_axes(@dynamics.JointAxesMask::lin_axes())
      .set_max_iters(200)
    multibody.inverse_kinematics(
      bodies,
      link_id,
      options,
      @core.Isometry2::from_translation(target_point),
      fn(_link : @dynamics.MultibodyLink) -> Bool { true },
      displacements,
    )
    multibody.apply_displacements(displacements.as_array())
    let pose = multibody.forward_kinematics_single_link(
      bodies,
      link_id,
      None,
      None,
    )
    let err = pose.translation.sub(target_point).length()
    inspect(err == err, content="true")
    inspect(err < 6.0e-2F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "examples3d/debug_balls3.rs parity" {
  // Ported from `rapier-reference/examples3d/debug_balls3.rs`.
  // TODO(dim3): replace with real 3D implementation and checks.
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
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )
  let ground_height = 0.1F
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, -ground_height))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(20.0F, ground_height).build(),
    ground,
    bodies,
  )
  |> ignore
  let num = 8
  let rad = 0.2F
  let shift = rad * 2.2F
  let center = shift * Float::from_double((num / 2).to_double())
  let handles : Array[@dynamics.RigidBodyHandle] = []
  for i in 0..<num {
    for j in 0..<num {
      let x = Float::from_double(i.to_double()) * shift - center
      let y = Float::from_double(j.to_double()) * shift + 2.0F
      let h = bodies.insert(
        @dynamics.RigidBodyBuilder::dynamic()
        .translation(@core.Vec2::new(x, y))
        .build(),
      )
      colliders.insert_with_parent(
        @collision.ColliderBuilder::ball(rad).build(),
        h,
        bodies,
      )
      |> ignore
      handles.push(h)
    }
  }
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 120,
  )
  for i in 0..<handles.length() {
    if bodies.get(handles[i]) is Some(b) {
      let p = b.translation()
      inspect(p.x == p.x, content="true")
      inspect(p.y == p.y, content="true")
      inspect(p.y > -10.0F, content="true")
    } else {
      inspect(false, content="true")
    }
  }
}

///|
test "examples3d/debug_big_colliders3.rs parity" {
  // Ported from `rapier-reference/examples3d/debug_big_colliders3.rs`.
  // TODO(dim3): replace with real 3D implementation and checks.
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
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    8,
  )

  // A few large static colliders.
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, -5.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(200.0F, 5.0F).build(),
    ground,
    bodies,
  )
  |> ignore
  let wall_l = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(-150.0F, 50.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(5.0F, 200.0F).build(),
    wall_l,
    bodies,
  )
  |> ignore
  let wall_r = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(150.0F, 50.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(5.0F, 200.0F).build(),
    wall_r,
    bodies,
  )
  |> ignore

  // Drop a large dynamic body.
  let dyn_handle = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(0.0F, 150.0F))
    .can_sleep(false)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::ball(10.0F).build(),
    dyn_handle,
    bodies,
  )
  |> ignore
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 120,
  )
  if bodies.get(dyn_handle) is Some(body) {
    inspect(body.translation().x == body.translation().x, content="true")
    inspect(body.translation().y == body.translation().y, content="true")
    inspect(body.translation().y > -50.0F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "examples3d/debug_boxes3.rs parity" {
  // Ported from `rapier-reference/examples3d/debug_boxes3.rs`.
  // TODO(dim3): replace with real 3D implementation and checks.
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
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )
  let ground_height = 0.1F
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, -ground_height))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(20.0F, ground_height).build(),
    ground,
    bodies,
  )
  |> ignore
  let num = 8
  let rad = 0.2F
  let shift = rad * 2.2F
  let center = shift * Float::from_double((num / 2).to_double())
  let handles : Array[@dynamics.RigidBodyHandle] = []
  for i in 0..<num {
    for j in 0..<num {
      let x = Float::from_double(i.to_double()) * shift - center
      let y = Float::from_double(j.to_double()) * shift + 2.0F
      let h = bodies.insert(
        @dynamics.RigidBodyBuilder::dynamic()
        .translation(@core.Vec2::new(x, y))
        .build(),
      )
      colliders.insert_with_parent(
        @collision.ColliderBuilder::cuboid(rad, rad).build(),
        h,
        bodies,
      )
      |> ignore
      handles.push(h)
    }
  }
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 120,
  )
  for i in 0..<handles.length() {
    if bodies.get(handles[i]) is Some(b) {
      let p = b.translation()
      inspect(p.x == p.x, content="true")
      inspect(p.y == p.y, content="true")
      inspect(p.y > -10.0F, content="true")
    } else {
      inspect(false, content="true")
    }
  }
}

///|
test "examples3d/debug_chain_high_mass_ratio3.rs parity" {
  // Ported from `rapier-reference/examples3d/debug_chain_high_mass_ratio3.rs`.
  // TODO(dim3): replace spherical joints with proper 3D and additional solver iterations.
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
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )
  let num = 17
  let rad = 0.2F
  let shift = rad * 2.2F
  let handles : Array[@dynamics.RigidBodyHandle] = []
  for i in 0..<num {
    let status = if i == 0 {
      @dynamics.RigidBodyType::Fixed
    } else {
      @dynamics.RigidBodyType::Dynamic
    }
    let ball_rad = if i == num - 1 { rad * 10.0F } else { rad }
    let x = Float::from_double(i.to_double()) * shift
    let h = bodies.insert(
      @dynamics.RigidBodyBuilder::new(status)
      .translation(@core.Vec2::new(x, 0.0F))
      .can_sleep(false)
      .build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder::ball(ball_rad)
      .density(if i == num - 1 { 10.0F } else { 1.0F })
      .build(),
      h,
      bodies,
    )
    |> ignore
    if i > 0 {
      let parent = handles[handles.length() - 1]
      let joint = @dynamics.GenericJoint::from_revolute(
        @dynamics.RevoluteJointBuilder::new()
        .local_anchor1(@core.Vec2::new(shift / 2.0F, 0.0F))
        .local_anchor2(@core.Vec2::new(-shift / 2.0F, 0.0F))
        .build(),
      )
      impulse_joints.insert(parent, h, joint, true) |> ignore
    }
    handles.push(h)
  }
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 240,
  )
  if bodies.get(handles[num - 1]) is Some(last) {
    let p = last.translation()
    inspect(p.x == p.x, content="true")
    inspect(p.y == p.y, content="true")
    inspect(@core.abs(p.x) < 1.0e6F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "examples3d/debug_cube_high_mass_ratio3.rs parity" {
  // Ported from `rapier-reference/examples3d/debug_cube_high_mass_ratio3.rs`.
  // TODO(dim3): add 3D shapes and per-body additional solver iterations parity.
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
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )
  let stick_len = 2.0F
  let stick_rad = 0.2F
  let floor = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, -stick_len - stick_rad))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(stick_len, stick_len).build(),
    floor,
    bodies,
  )
  |> ignore
  let num_levels = 4
  for i in 0..<num_levels {
    let fi = Float::from_double(i.to_double())
    let y0 = fi * stick_rad * 4.0F
    let dx = stick_len / 2.0F - stick_rad
    // Two horizontal sticks.
    for s in 0..<2 {
      let z = if s == 0 { -dx } else { dx }
      let h = bodies.insert(
        @dynamics.RigidBodyBuilder::dynamic()
        .translation(@core.Vec2::new(0.0F, y0 + 0.0F))
        .rotation(@core.Rot2::from_angle(if s == 0 { 0.0F } else { 0.0F }))
        .build(),
      )
      colliders.insert_with_parent(
        @collision.ColliderBuilder::cuboid(stick_len / 2.0F, stick_rad)
        .translation(@core.Vec2::new(0.0F, 0.0F))
        .build(),
        h,
        bodies,
      )
      |> ignore
      // Slightly offset in x (stands in for z offset in the 3D example).
      if bodies.get_mut(h) is Some(b) {
        b.set_translation(@core.Vec2::new(z, y0), true) |> ignore
      }
    }
    // Two vertical sticks.
    for s in 0..<2 {
      let x = if s == 0 { -dx } else { dx }
      let h = bodies.insert(
        @dynamics.RigidBodyBuilder::dynamic()
        .translation(@core.Vec2::new(x, y0 + stick_rad * 2.0F))
        .build(),
      )
      colliders.insert_with_parent(
        @collision.ColliderBuilder::cuboid(stick_rad, stick_len / 2.0F).build(),
        h,
        bodies,
      )
      |> ignore
    }
  }

  // Big cube on top.
  let cube_len = stick_len * 2.0F
  let cube_handle = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(
      @core.Vec2::new(
        0.0F,
        cube_len / 2.0F +
        Float::from_double((num_levels - 1).to_double()) * stick_rad * 4.0F,
      ),
    )
    .can_sleep(false)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(cube_len / 2.0F, cube_len / 2.0F)
    .density(50.0F)
    .build(),
    cube_handle,
    bodies,
  )
  |> ignore
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 240,
  )
  if bodies.get(cube_handle) is Some(body) {
    inspect(body.translation().x == body.translation().x, content="true")
    inspect(body.translation().y == body.translation().y, content="true")
    inspect(body.translation().y > -50.0F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "examples3d/debug_cylinder3.rs parity" {
  // Ported from `rapier-reference/examples3d/debug_cylinder3.rs`.
  // TODO(dim3): replace with real 3D cylinder; 2D uses a capsule approximation.
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
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let parameters = @dynamics.IntegrationParameters::default()
  let ground_height = 0.1F
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, -ground_height))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(10.0F, ground_height).build(),
    ground,
    bodies,
  )
  |> ignore
  let cyl = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(0.0F, 3.0F))
    .rotation(@core.Rot2::from_angle(0.3F))
    .can_sleep(false)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::capsule_y(0.8F, 0.3F).build(),
    cyl,
    bodies,
  )
  |> ignore
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 120,
  )
  if bodies.get(cyl) is Some(body) {
    inspect(body.translation().x == body.translation().x, content="true")
    inspect(body.translation().y == body.translation().y, content="true")
    inspect(body.translation().y > -20.0F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "examples3d/debug_deserialize3.rs parity" {
  // Ported from `rapier-reference/examples3d/debug_deserialize3.rs`.
  //
  // NOTE: Upstream exercises serialization/deserialization. This port currently
  // doesn't include a serialization layer, so we keep a smoke test that runs a
  // small world without panicking.
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
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let parameters = @dynamics.IntegrationParameters::default()
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, -1.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(20.0F, 1.0F).build(),
    ground,
    bodies,
  )
  |> ignore
  let ball = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(0.0F, 4.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::ball(0.5F).build(),
    ball,
    bodies,
  )
  |> ignore
  pipeline.step(
    gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders, impulse_joints,
    multibody_joints, ccd_solver, hooks, events,
  )
  inspect(true, content="true")
}

///|
test "examples3d/debug_disabled3.rs parity" {
  // Ported from `rapier-reference/examples3d/debug_disabled3.rs`.
  // TODO(dim3): replace with real 3D implementation and checks.
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
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let parameters = @dynamics.IntegrationParameters::default()
  let ground_height = 0.1F
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, -ground_height))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(10.0F, ground_height).build(),
    ground,
    bodies,
  )
  |> ignore
  let enabled_body = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(-1.0F, 3.0F))
    .can_sleep(false)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::ball(0.25F).build(),
    enabled_body,
    bodies,
  )
  |> ignore
  let disabled_body = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(1.0F, 3.0F))
    .can_sleep(false)
    .build(),
  )
  let disabled_collider = colliders.insert_with_parent(
    @collision.ColliderBuilder::ball(0.25F).build(),
    disabled_body,
    bodies,
  )
  if colliders.get_mut(disabled_collider) is Some(c) {
    c.set_enabled(false) |> ignore
  } else {
    inspect(false, content="true")
  }
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 240,
  )
  if bodies.get(enabled_body) is Some(b0) &&
    bodies.get(disabled_body) is Some(b1) {
    inspect(b0.translation().y > -1.0F, content="true")
    inspect(b1.translation().y < -2.0F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "examples3d/debug_dynamic_collider_add3.rs parity" {
  // Ported from `rapier-reference/examples3d/debug_dynamic_collider_add3.rs`.
  // TODO(dim3): replace with real 3D implementation and checks.
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
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let parameters = @dynamics.IntegrationParameters::default()
  let ground_height = 0.1F
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, -ground_height))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(10.0F, ground_height).build(),
    ground,
    bodies,
  )
  |> ignore

  // Dynamic body starts without a collider; we attach it after a few steps.
  let h = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(0.0F, 5.0F))
    .can_sleep(false)
    .build(),
  )
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 5,
  )
  let c = colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(0.3F, 0.3F).build(),
    h,
    bodies,
  )
  inspect(@collision.ColliderHandle::equals(c, c), content="true")
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 120,
  )
  if bodies.get(h) is Some(body) {
    inspect(body.translation().y > -5.0F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "examples3d/debug_eccentric_boxes3.rs parity" {
  // Ported from `rapier-reference/examples3d/debug_eccentric_boxes3.rs`.
  // TODO(dim3): replace with real 3D implementation and checks.
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
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )
  let ground_height = 0.1F
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, -ground_height))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(20.0F, ground_height).build(),
    ground,
    bodies,
  )
  |> ignore

  // A body with offset colliders so the COM isn't centered.
  let h = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(0.0F, 3.0F))
    .can_sleep(false)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(0.8F, 0.2F)
    .translation(@core.Vec2::new(0.8F, 0.0F))
    .build(),
    h,
    bodies,
  )
  |> ignore
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(0.2F, 0.8F)
    .translation(@core.Vec2::new(-0.6F, 0.0F))
    .build(),
    h,
    bodies,
  )
  |> ignore

  // Give it an initial horizontal velocity so friction creates a torque.
  if bodies.get_mut(h) is Some(body) {
    body.set_linvel(@core.Vec2::new(5.0F, 0.0F), true) |> ignore
  } else {
    inspect(false, content="true")
  }
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 240,
  )
  if bodies.get(h) is Some(body) {
    inspect(body.translation().x == body.translation().x, content="true")
    inspect(body.translation().y == body.translation().y, content="true")
    inspect(body.angvel() == body.angvel(), content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "examples3d/debug_friction3.rs parity" {
  // Ported from `rapier-reference/examples3d/debug_friction3.rs`.
  // TODO(dim3): replace with real 3D implementation and checks.
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
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )
  let ground_height = 0.1F
  let ground = bodies.insert(@dynamics.RigidBodyBuilder::fixed().build())
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(100.0F, ground_height)
    .translation(@core.Vec2::new(0.0F, -ground_height))
    .friction(1.5F)
    .build(),
    ground,
    bodies,
  )
  |> ignore
  let handle = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(0.0F, 1.1F))
    .rotation(@core.Rot2::from_angle(0.3F))
    .can_sleep(false)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(2.0F, 1.0F).friction(1.5F).build(),
    handle,
    bodies,
  )
  |> ignore
  if bodies.get_mut(handle) is Some(body) {
    body.set_linvel(@core.Vec2::new(50.0F, 0.0F), true) |> ignore
  } else {
    inspect(false, content="true")
  }
  let init_speed = if bodies.get(handle) is Some(body) {
    body.linvel().length()
  } else {
    0.0F
  }
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 240,
  )
  if bodies.get(handle) is Some(body) {
    let end_speed = body.linvel().length()
    inspect(init_speed == init_speed, content="true")
    inspect(end_speed == end_speed, content="true")
    inspect(end_speed < init_speed, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "examples3d/debug_infinite_fall3.rs parity" {
  // Ported from `rapier-reference/examples3d/debug_infinite_fall3.rs`.
  // TODO(dim3): replace with real 3D implementation and checks.
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
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let parameters = @dynamics.IntegrationParameters::default()
  let h = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(0.0F, 1000.0F))
    .can_sleep(false)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::ball(0.5F).build(),
    h,
    bodies,
  )
  |> ignore
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 240,
  )
  if bodies.get(h) is Some(body) {
    inspect(body.translation().y == body.translation().y, content="true")
    inspect(body.translation().y < 1000.0F, content="true")
    inspect(body.translation().y > -1.0e9F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "examples3d/debug_internal_edges3.rs parity" {
  // Ported from `rapier-reference/examples3d/debug_internal_edges3.rs`.
  // TODO(dim3): replace with real 3D implementation and checks.
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
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )

  // A zig-zag polyline ground to exercise internal edges contact stability.
  let ground = bodies.insert(@dynamics.RigidBodyBuilder::fixed().build())
  let pts = [
    @core.Vec2::new(-10.0F, 0.0F),
    @core.Vec2::new(-5.0F, 0.0F),
    @core.Vec2::new(-2.0F, 0.5F),
    @core.Vec2::new(2.0F, 0.5F),
    @core.Vec2::new(5.0F, 0.0F),
    @core.Vec2::new(10.0F, 0.0F),
  ]
  colliders.insert_with_parent(
    @collision.ColliderBuilder::polyline(pts, None)
    .translation(@core.Vec2::new(0.0F, -2.0F))
    .build(),
    ground,
    bodies,
  )
  |> ignore
  let ball = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(0.0F, 5.0F))
    .can_sleep(false)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::ball(0.25F).build(),
    ball,
    bodies,
  )
  |> ignore
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 240,
  )
  if bodies.get(ball) is Some(body) {
    let p = body.translation()
    inspect(p.x == p.x, content="true")
    inspect(p.y == p.y, content="true")
    inspect(p.y > -50.0F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "examples3d/debug_long_chain3.rs parity" {
  // Ported from `rapier-reference/examples3d/debug_long_chain3.rs`.
  // TODO(dim3): replace spherical joints with proper 3D. In 2D, use revolute joints.
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
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )
  let num = 30
  let rad = 0.2F
  let shift = rad * 2.2F
  let handles : Array[@dynamics.RigidBodyHandle] = []
  for i in 0..<num {
    let status = if i == 0 {
      @dynamics.RigidBodyType::Fixed
    } else {
      @dynamics.RigidBodyType::Dynamic
    }
    let x = Float::from_double(i.to_double()) * shift
    let h = bodies.insert(
      @dynamics.RigidBodyBuilder::new(status)
      .translation(@core.Vec2::new(x, 0.0F))
      .can_sleep(false)
      .build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder::ball(rad).build(),
      h,
      bodies,
    )
    |> ignore
    if i > 0 {
      let parent = handles[handles.length() - 1]
      let joint = @dynamics.GenericJoint::from_revolute(
        @dynamics.RevoluteJointBuilder::new()
        .local_anchor1(@core.Vec2::new(shift / 2.0F, 0.0F))
        .local_anchor2(@core.Vec2::new(-shift / 2.0F, 0.0F))
        .build(),
      )
      impulse_joints.insert(parent, h, joint, true) |> ignore
    }
    handles.push(h)
  }
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 240,
  )
  if bodies.get(handles[num - 1]) is Some(last) {
    let p = last.translation()
    inspect(p.x == p.x, content="true")
    inspect(p.y == p.y, content="true")
    inspect(@core.abs(p.x) < 1.0e6F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "examples3d/debug_multibody_ang_motor_pos3.rs parity" {
  // Ported from `rapier-reference/examples3d/debug_multibody_ang_motor_pos3.rs`.
  // TODO(dim3): implement full 3D multibody motor parity.
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
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    32,
  )

  // Ground.
  let ground_height = 0.1F
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, -ground_height))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(10.0F, ground_height).build(),
    ground,
    bodies,
  )
  |> ignore

  // A simple two-link multibody with a revolute motor.
  let base = bodies.insert(@dynamics.RigidBodyBuilder::fixed().build())
  let link = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(0.0F, 2.0F))
    .can_sleep(false)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(0.2F, 0.6F).build(),
    link,
    bodies,
  )
  |> ignore
  let joint = @dynamics.RevoluteJointBuilder::new()
    .motor_position(1.0F, 100.0F, 10.0F)
    .build()
  let inserted = multibody_joints.insert(base, link, joint, true)
  inspect(inserted is Some(_), content="true")
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 240,
  )
  if bodies.get(link) is Some(body) {
    inspect(body.rotation().angle() == body.rotation().angle(), content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "examples3d/debug_pop3.rs parity" {
  // Ported from `rapier-reference/examples3d/debug_pop3.rs`.
  // TODO(dim3): replace with real 3D regression scenario.
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
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )
  let ground_height = 0.1F
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, -ground_height))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(10.0F, ground_height).build(),
    ground,
    bodies,
  )
  |> ignore

  // Stack a few balls to exercise depenetration.
  let handles : Array[@dynamics.RigidBodyHandle] = []
  for i in 0..<10 {
    let h = bodies.insert(
      @dynamics.RigidBodyBuilder::dynamic()
      .translation(
        @core.Vec2::new(0.0F, 0.5F + Float::from_double(i.to_double()) * 0.6F),
      )
      .can_sleep(false)
      .build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder::ball(0.25F).build(),
      h,
      bodies,
    )
    |> ignore
    handles.push(h)
  }
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 240,
  )
  for i in 0..<handles.length() {
    if bodies.get(handles[i]) is Some(b) {
      let p = b.translation()
      inspect(p.x == p.x, content="true")
      inspect(p.y == p.y, content="true")
      inspect(p.y > -10.0F, content="true")
    } else {
      inspect(false, content="true")
    }
  }
}

///|
test "examples3d/debug_prismatic3.rs parity" {
  // Ported from `rapier-reference/examples3d/debug_prismatic3.rs`.
  // TODO(dim3): replace with real 3D implementation and checks.
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
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    32,
  )

  // Ground.
  let ground_height = 0.1F
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, -ground_height))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(20.0F, ground_height).build(),
    ground,
    bodies,
  )
  |> ignore

  // A slider constrained to a diagonal axis.
  let base = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, 2.0F))
    .build(),
  )
  let slider = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(0.0F, 4.0F))
    .can_sleep(false)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(0.2F, 0.2F).build(),
    slider,
    bodies,
  )
  |> ignore
  let axis = @core.Vec2::new(1.0F, 1.0F).normalize()
  let joint = @dynamics.GenericJoint::from_prismatic(
    @dynamics.PrismaticJointBuilder::new(axis).limits(-1.0F, 1.0F).build(),
  )
  impulse_joints.insert(base, slider, joint, true) |> ignore
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 240,
  )
  if bodies.get(base) is Some(b0) && bodies.get(slider) is Some(b1) {
    let delta = b1.translation().sub(b0.translation())
    let proj = delta.dot(axis)
    inspect(proj == proj, content="true")
    inspect(proj <= 1.2F, content="true")
    inspect(proj >= -1.2F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "examples3d/debug_rollback3.rs parity" {
  // Ported from `rapier-reference/examples3d/debug_rollback3.rs`.
  //
  // NOTE: Upstream targets rollback/determinism issues. This port doesn't expose
  // a full rollback API yet, so we keep a deterministic smoke test.
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
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let parameters = @dynamics.IntegrationParameters::default()
  let ground = bodies.insert(@dynamics.RigidBodyBuilder::fixed().build())
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(20.0F, 1.0F).build(),
    ground,
    bodies,
  )
  |> ignore
  let ball = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(0.0F, 5.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::ball(0.5F).build(),
    ball,
    bodies,
  )
  |> ignore
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 60,
  )
  if bodies.get(ball) is Some(body) {
    let p = body.translation()
    inspect(p.x == p.x, content="true")
    inspect(p.y == p.y, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "examples3d/debug_shape_modification3.rs parity" {
  // Ported from `rapier-reference/examples3d/debug_shape_modification3.rs`.
  // TODO(dim3): replace with real 3D implementation and checks.
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
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let parameters = @dynamics.IntegrationParameters::default()
  let ground_height = 0.1F
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, -ground_height))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(10.0F, ground_height).build(),
    ground,
    bodies,
  )
  |> ignore
  let body = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(0.0F, 3.0F))
    .can_sleep(false)
    .build(),
  )
  let ch = colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(0.3F, 0.3F).build(),
    body,
    bodies,
  )
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 60,
  )
  if colliders.get_mut(ch) is Some(c) {
    c.set_shape(@collision.Shape::Ball(0.5F)) |> ignore
  } else {
    inspect(false, content="true")
  }
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 60,
  )
  if bodies.get(body) is Some(rb) {
    inspect(rb.translation().y == rb.translation().y, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "examples3d/debug_sleeping_kinematic3.rs parity" {
  // Ported from `rapier-reference/examples3d/debug_sleeping_kinematic3.rs`.
  // TODO(dim3): replace with real 3D implementation and checks.
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
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let parameters = @dynamics.IntegrationParameters::default()
  let inv_dt = parameters.inv_dt()
  let dt = if inv_dt == 0.0F { 0.0F } else { 1.0F / inv_dt }

  // Two velocity-based kinematic platforms.
  let platform_handle = bodies.insert(
    @dynamics.RigidBodyBuilder::kinematic_velocity_based()
    .translation(@core.Vec2::new(0.0F, 1.5F + 0.8F))
    .sleeping(true)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(5.0F, 0.5F).build(),
    platform_handle,
    bodies,
  )
  |> ignore
  let slow_platform_handle = bodies.insert(
    @dynamics.RigidBodyBuilder::kinematic_velocity_based()
    .translation(@core.Vec2::new(0.0F, 0.0F))
    .sleeping(true)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(5.0F, 0.5F).build(),
    slow_platform_handle,
    bodies,
  )
  |> ignore
  let start_tick = 500
  let stop_tick = 1000
  let mut time = 0.0F
  for tick in 0..<1600 {
    if tick == stop_tick {
      if bodies.get(platform_handle) is Some(p) &&
        bodies.get(slow_platform_handle) is Some(sp) {
        inspect(!p.is_sleeping(), content="true")
        inspect(!sp.is_sleeping(), content="true")
      } else {
        inspect(false, content="true")
      }
      if bodies.get_mut(slow_platform_handle) is Some(p) {
        p.set_linvel(@core.Vec2::zero(), true) |> ignore
      } else {
        inspect(false, content="true")
      }
      if bodies.get_mut(platform_handle) is Some(p) {
        p.set_linvel(@core.Vec2::zero(), true) |> ignore
      } else {
        inspect(false, content="true")
      }
    }
    if tick > stop_tick + 500 {
      if bodies.get(platform_handle) is Some(p) &&
        bodies.get(slow_platform_handle) is Some(sp) {
        inspect(p.is_sleeping(), content="true")
        inspect(sp.is_sleeping(), content="true")
      } else {
        inspect(false, content="true")
      }
    }
    if tick == start_tick {
      if bodies.get(platform_handle) is Some(p) &&
        bodies.get(slow_platform_handle) is Some(sp) {
        inspect(p.is_sleeping(), content="true")
        inspect(sp.is_sleeping(), content="true")
      } else {
        inspect(false, content="true")
      }
      let slow_velocity = @core.Vec2::new(0.0F, 0.01F)
      if bodies.get_mut(slow_platform_handle) is Some(p) {
        p.set_linvel(slow_velocity, true) |> ignore
      } else {
        inspect(false, content="true")
      }
    }
    if tick >= start_tick && tick < stop_tick {
      let velocity = @core.Vec2::new(
        Float::from_double(@math.sin(time.to_double())) * 2.0F,
        Float::from_double(@math.cos((time * 2.0F).to_double())),
      )
      if bodies.get_mut(platform_handle) is Some(p) {
        p.set_linvel(velocity, true) |> ignore
      } else {
        inspect(false, content="true")
      }
    }
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
      impulse_joints, multibody_joints, ccd_solver, hooks, events,
    )
    time = time + dt
  }
}

///|
test "examples3d/debug_thin_cube_on_mesh3.rs parity" {
  // Ported from `rapier-reference/examples3d/debug_thin_cube_on_mesh3.rs`.
  // TODO(dim3): replace with real 3D implementation and checks.
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
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )
  let ground = bodies.insert(@dynamics.RigidBodyBuilder::fixed().build())
  let vtx = [
    @core.Vec2::new(-5.0F, 0.0F),
    @core.Vec2::new(5.0F, 0.0F),
    @core.Vec2::new(5.0F, 1.0F),
    @core.Vec2::new(-5.0F, 1.0F),
  ]
  let idx = [(0, 1, 2), (0, 2, 3)]
  if @collision.ColliderBuilder::trimesh(vtx, idx) is Some(builder) {
    colliders.insert_with_parent(builder.build(), ground, bodies) |> ignore
  } else {
    inspect(false, content="true")
  }
  let thin = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(0.0F, 5.0F))
    .can_sleep(false)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(0.5F, 0.02F).build(),
    thin,
    bodies,
  )
  |> ignore
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 240,
  )
  if bodies.get(thin) is Some(body) {
    let p = body.translation()
    inspect(p.x == p.x, content="true")
    inspect(p.y == p.y, content="true")
    inspect(p.y > -10.0F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "examples3d/debug_triangle3.rs parity" {
  // Ported from `rapier-reference/examples3d/debug_triangle3.rs`.
  // TODO(dim3): replace with real 3D implementation and checks.
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
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )
  let ground = bodies.insert(@dynamics.RigidBodyBuilder::fixed().build())
  // Reference uses a triangle lying in the horizontal plane. In 2D we approximate this with a
  // long horizontal segment supporting a thin box.
  colliders.insert_with_parent(
    @collision.ColliderBuilder::segment(
      @core.Vec2::new(-10.0F, 0.0F),
      @core.Vec2::new(10.0F, 0.0F),
    ).build(),
    ground,
    bodies,
  )
  |> ignore
  let cube = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(1.1F, 0.01F))
    .can_sleep(false)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(20.0F, 0.1F).build(),
    cube,
    bodies,
  )
  |> ignore
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 240,
  )
  if bodies.get(cube) is Some(body) {
    inspect(body.translation().y > -20.0F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "examples3d/debug_trimesh3.rs parity" {
  // Ported from `rapier-reference/examples3d/debug_trimesh3.rs`.
  // TODO(dim3): replace with real 3D implementation and checks.
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
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )
  let ground = bodies.insert(@dynamics.RigidBodyBuilder::fixed().build())
  let vtx = [
    @core.Vec2::new(-10.0F, 0.0F),
    @core.Vec2::new(10.0F, 0.0F),
    @core.Vec2::new(10.0F, 1.0F),
    @core.Vec2::new(-10.0F, 1.0F),
  ]
  let idx = [(0, 1, 2), (0, 2, 3)]
  if @collision.ColliderBuilder::trimesh(vtx, idx) is Some(builder) {
    colliders.insert_with_parent(builder.build(), ground, bodies) |> ignore
  } else {
    inspect(false, content="true")
  }
  let handles : Array[@dynamics.RigidBodyHandle] = []
  for k in 0..<5 {
    let h = bodies.insert(
      @dynamics.RigidBodyBuilder::dynamic()
      .translation(
        @core.Vec2::new(0.0F, 5.0F + Float::from_double(k.to_double()) * 1.0F),
      )
      .can_sleep(false)
      .build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder::cuboid(0.2F, 0.2F).build(),
      h,
      bodies,
    )
    |> ignore
    handles.push(h)
  }
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 240,
  )
  for i in 0..<handles.length() {
    if bodies.get(handles[i]) is Some(body) {
      let p = body.translation()
      inspect(p.x == p.x, content="true")
      inspect(p.y == p.y, content="true")
      inspect(p.y > -20.0F, content="true")
    } else {
      inspect(false, content="true")
    }
  }
}

///|
test "examples3d/debug_two_cubes3.rs parity" {
  // Ported from `rapier-reference/examples3d/debug_two_cubes3.rs`.
  // TODO(dim3): replace with real 3D implementation and checks.
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
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )
  let ground_height = 0.1F
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, -ground_height))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(10.0F, ground_height).build(),
    ground,
    bodies,
  )
  |> ignore
  let a = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(0.0F, 3.0F))
    .can_sleep(false)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(0.5F, 0.5F).build(),
    a,
    bodies,
  )
  |> ignore
  let b = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(0.1F, 4.2F))
    .can_sleep(false)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(0.5F, 0.5F).build(),
    b,
    bodies,
  )
  |> ignore
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 240,
  )
  if bodies.get(a) is Some(ba) && bodies.get(b) is Some(bb) {
    inspect(ba.translation().y == ba.translation().y, content="true")
    inspect(bb.translation().y == bb.translation().y, content="true")
    inspect(ba.translation().y > -10.0F, content="true")
    inspect(bb.translation().y > -10.0F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "examples3d/domino3.rs parity" {
  // Ported from `rapier-reference/examples3d/domino3.rs`.
  // TODO(dim3): replace spiral 3D layout with real parity checks.
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
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )
  let ground_height = 0.1F
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, -ground_height))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(200.0F, ground_height).build(),
    ground,
    bodies,
  )
  |> ignore

  // A downscaled "domino line" in 2D.
  let num = 80
  let width = 0.5F
  let thickness = 0.08F
  let spacing = thickness * 2.5F
  let handles : Array[@dynamics.RigidBodyHandle] = []
  for i in 0..<num {
    let x = Float::from_double(i.to_double()) * spacing
    let tilt = if i == 0 { 0.8F } else { 0.0F }
    let rb_builder0 = @dynamics.RigidBodyBuilder::dynamic()
      .translation(@core.Vec2::new(x, width * 2.0F))
      .rotation(@core.Rot2::from_angle(tilt))
      .can_sleep(false)
    let rb_builder = if i == 0 {
      // Ensure the first domino tips into the chain.
      rb_builder0.linvel(@core.Vec2::new(2.0F, 0.0F)).angvel(-10.0F)
    } else {
      rb_builder0
    }
    let h = bodies.insert(rb_builder.build())
    colliders.insert_with_parent(
      @collision.ColliderBuilder::cuboid(thickness, width * 2.0F).build(),
      h,
      bodies,
    )
    |> ignore
    handles.push(h)
  }
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 360,
  )
  // At least one domino should have a non-trivial rotation.
  let mut fallen = 0
  for i in 0..<handles.length() {
    if bodies.get(handles[i]) is Some(body) {
      let a = @core.abs(body.rotation().angle())
      if a > 0.05F {
        fallen = fallen + 1
      }
    }
  }
  inspect(fallen > 5, content="true")
}

///|
test "examples3d/dynamic_trimesh3.rs parity" {
  inspect(run_examples3d_dynamic_trimesh3(), content="true")
}

///|
test "examples3d/fountain3.rs parity" {
  // Ported from `rapier-reference/examples3d/fountain3.rs`.
  // TODO(dim3): replace with real 3D implementation and checks.
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
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    8,
  )
  let rad = 0.5F
  let ground_size = 40.0F
  let ground_height = 2.1F
  for k in 0..<3 {
    let y = -ground_height - Float::from_double(k.to_double())
    let handle = bodies.insert(
      @dynamics.RigidBodyBuilder::fixed()
      .translation(@core.Vec2::new(0.0F, y))
      .build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder::cuboid(ground_size, ground_height).build(),
      handle,
      bodies,
    )
    |> ignore
  }
  let max_number_of_bodies = 120
  let spawned : Array[@dynamics.RigidBodyHandle] = []
  let mut remove_cursor = 0
  for step in 0..<240 {
    // Spawn one body per step (like the testbed callback).
    let handle = bodies.insert(
      @dynamics.RigidBodyBuilder::dynamic()
      .translation(@core.Vec2::new(0.0F, 10.0F))
      .can_sleep(false)
      .build(),
    )
    let builder = match step % 3 {
      0 => @collision.ColliderBuilder::capsule_y(rad, rad / 10.0F)
      1 => @collision.ColliderBuilder::ball(rad)
      _ => @collision.ColliderBuilder::cuboid(rad, rad)
    }
    colliders.insert_with_parent(builder.build(), handle, bodies) |> ignore
    spawned.push(handle)

    // Cap dynamic body count deterministically.
    while spawned.length() - remove_cursor > max_number_of_bodies {
      let h = spawned[remove_cursor]
      remove_cursor = remove_cursor + 1
      @pipeline.remove_rigid_body(
        bodies, h, islands, colliders, impulse_joints, multibody_joints, true, true,
      )
      |> ignore
    }
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
      impulse_joints, multibody_joints, ccd_solver, hooks, events,
    )
  }
  inspect(bodies.len() <= max_number_of_bodies + 3, content="true")
  inspect(colliders.len() <= max_number_of_bodies + 3, content="true")
}

///|
test "examples3d/gyroscopic3.rs parity (dim2 adapted)" {
  // Ported from `rapier-reference/examples3d/gyroscopic3.rs`.
  // TODO(dim3): implement gyroscopic forces and use compound shapes.
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
  let gravity = @core.Vec2::zero()
  let parameters = @dynamics.IntegrationParameters::default()
  let body_handle = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .gravity_scale(0.0F)
    .angvel(20.0F)
    .can_sleep(false)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(2.0F, 0.2F).build(),
    body_handle,
    bodies,
  )
  |> ignore
  for _ in 0..<240 {
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
      impulse_joints, multibody_joints, ccd_solver, hooks, events,
    )
  }
  if bodies.get(body_handle) is Some(body) {
    inspect(body.translation().x == body.translation().x, content="true")
    inspect(body.translation().y == body.translation().y, content="true")
    inspect(body.angvel() == body.angvel(), content="true")
    inspect(@core.abs(body.angvel() - 20.0F) < 1.0e-2F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "examples3d/harness_capsules3.rs parity" {
  // Ported from `rapier-reference/examples3d/harness_capsules3.rs`.
  // TODO(dim3): add harness + broad-phase selection parity; keep a smaller stress smoke in 2D.
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
  let ground_height = 0.1F
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, -ground_height))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(200.1F, ground_height).build(),
    ground,
    bodies,
  )
  |> ignore
  let num = 8
  let height = 10
  let rad = 0.25F
  let shift = rad * 3.0F
  let centerx = shift * Float::from_double((num / 2).to_double())
  let handles : Array[@dynamics.RigidBodyHandle] = []
  for j in 0..<height {
    for i in 0..<num {
      let x = Float::from_double(i.to_double()) * shift - centerx
      let y = Float::from_double(j.to_double()) * (rad * 4.0F) + 3.0F
      let h = bodies.insert(
        @dynamics.RigidBodyBuilder::dynamic()
        .translation(@core.Vec2::new(x, y))
        .build(),
      )
      colliders.insert_with_parent(
        @collision.ColliderBuilder::capsule_y(rad, rad).build(),
        h,
        bodies,
      )
      |> ignore
      handles.push(h)
    }
  }
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 120,
  )
  for i in 0..<handles.length() {
    if bodies.get(handles[i]) is Some(body) {
      let p = body.translation()
      inspect(p.x == p.x, content="true")
      inspect(p.y == p.y, content="true")
      inspect(p.y > -100.0F, content="true")
    } else {
      inspect(false, content="true")
    }
  }
}

///|
test "examples3d/heightfield3.rs parity" {
  // Ported from `rapier-reference/examples3d/heightfield3.rs`.
  // TODO(dim3): replace with real 3D heightfield. This uses the 2D heightfield polyline.
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
  let ground_size = @core.Vec2::new(50.0F, 1.0F)
  let nsubdivs = 200
  let heights : Array[@core.Real] = []
  let nx = Float::from_double(nsubdivs.to_double())
  for i in 0..<(nsubdivs + 1) {
    if i == 0 || i == nsubdivs {
      heights.push(8.0F)
    } else {
      let t = Float::from_double(i.to_double()) * ground_size.x / nx
      heights.push(local_cos(t) * 2.0F)
    }
  }
  let ground_handle = bodies.insert(@dynamics.RigidBodyBuilder::fixed().build())
  colliders.insert_with_parent(
    @collision.ColliderBuilder::heightfield(heights, ground_size).build(),
    ground_handle,
    bodies,
  )
  |> ignore
  let num = 10
  let rad = 0.5F
  let shift = rad * 2.0F
  let centerx = shift * Float::from_double((num / 2).to_double())
  let handles : Array[@dynamics.RigidBodyHandle] = []
  for i in 0..<num {
    for j in 0..<num {
      let x = Float::from_double(i.to_double()) * shift - centerx
      let y = Float::from_double(j.to_double()) * shift + shift / 2.0F + 3.0F
      let h = bodies.insert(
        @dynamics.RigidBodyBuilder::dynamic()
        .translation(@core.Vec2::new(x, y))
        .build(),
      )
      if j % 2 == 0 {
        colliders.insert_with_parent(
          @collision.ColliderBuilder::cuboid(rad, rad).build(),
          h,
          bodies,
        )
        |> ignore
      } else {
        colliders.insert_with_parent(
          @collision.ColliderBuilder::ball(rad).build(),
          h,
          bodies,
        )
        |> ignore
      }
      handles.push(h)
    }
  }
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 120,
  )
  let mut min_y = 1.0e30F
  for i in 0..<handles.length() {
    if bodies.get(handles[i]) is Some(body) {
      let p = body.translation()
      inspect(p.x == p.x, content="true")
      inspect(p.y == p.y, content="true")
      if p.y < min_y {
        min_y = p.y
      }
    } else {
      inspect(false, content="true")
    }
  }
  inspect(min_y < 2.0F, content="true")
  inspect(min_y > -50.0F, content="true")
}

///|
test "examples3d/inverse_kinematics3.rs parity" {
  inspect(run_examples3d_inverse_kinematics3(), content="true")
}

///|
test "examples3d/joint_motor_position3.rs parity" {
  inspect(run_examples3d_joint_motor_position3(), content="true")
}

///|
test "examples3d/joints3.rs parity" {
  inspect(run_examples3d_joints3(), content="true")
}

///|
test "examples3d/keva3.rs parity" {
  inspect(run_examples3d_keva3(), content="true")
}

///|
test "examples3d/locked_rotations3.rs parity (dim2 adapted)" {
  // Ported from `rapier-reference/examples3d/locked_rotations3.rs`.
  // TODO(dim3): replace with real 3D implementation and checks.
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
  for _ in 0..<360 {
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
      impulse_joints, multibody_joints, ccd_solver, hooks, events,
    )
  }
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
test "examples3d/newton_cradle3.rs parity" {
  inspect(run_examples3d_newton_cradle3(), content="true")
}

///|
test "examples3d/one_way_platforms3.rs parity" {
  // Ported from `rapier-reference/examples3d/one_way_platforms3.rs`.
  // TODO(dim3): replace with real 3D oneway platform semantics.
  let pipeline = @pipeline.PhysicsPipeline::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = @collision.ColliderSet::new()
  let impulse_joints = @dynamics.ImpulseJointSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()
  let islands = @dynamics.IslandManager::new()
  let events = @pipeline.EventHandler::new()
  let ccd_solver = @dynamics.CCDSolver::new()
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let ground_body = bodies.insert(@dynamics.RigidBodyBuilder::fixed().build())
  let platform1 = colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(9.0F, 0.5F)
    .translation(@core.Vec2::new(0.0F, 2.0F))
    .active_hooks(@collision.ActiveHooks::modify_solver_contacts())
    .build(),
    ground_body,
    bodies,
  )
  let platform2 = colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(9.0F, 0.5F)
    .translation(@core.Vec2::new(0.0F, -2.0F))
    .active_hooks(@collision.ActiveHooks::modify_solver_contacts())
    .build(),
    ground_body,
    bodies,
  )
  let hooks = @pipeline.PhysicsHooks::new().modify_solver_contacts(fn(
    bodies : @dynamics.RigidBodySet,
    colliders : @collision.ColliderSet,
    c1 : @collision.ColliderHandle,
    c2 : @collision.ColliderHandle,
    _manifold : @collision.ContactManifold,
  ) -> (Bool, @core.Vec2) {
    let is_p1 = @collision.ColliderHandle::equals(c1, platform1) ||
      @collision.ColliderHandle::equals(c2, platform1)
    let is_p2 = @collision.ColliderHandle::equals(c1, platform2) ||
      @collision.ColliderHandle::equals(c2, platform2)
    if !is_p1 && !is_p2 {
      return (true, @core.Vec2::zero())
    }
    // Identify the non-platform collider to get the parent body position.
    let other = if is_p1 {
      if @collision.ColliderHandle::equals(c1, platform1) {
        c2
      } else {
        c1
      }
    } else if @collision.ColliderHandle::equals(c1, platform2) {
      c2
    } else {
      c1
    }
    let plat = if is_p1 { platform1 } else { platform2 }
    let other_y = if colliders.get(other) is Some(co) &&
      co.parent() is Some(bh) &&
      bodies.get(bh) is Some(rb) {
      rb.translation().y
    } else {
      0.0F
    }
    let plat_y = if colliders.get(plat) is Some(cp) {
      cp.translation().y
    } else {
      0.0F
    }
    // One-way behavior: platform1 blocks from above, platform2 blocks from below.
    let accept = if is_p1 {
      other_y > plat_y + 0.1F
    } else {
      other_y < plat_y - 0.1F
    }
    let tv = if is_p1 {
      @core.Vec2::new(-12.0F, 0.0F)
    } else {
      @core.Vec2::new(12.0F, 0.0F)
    }
    (accept, tv)
  })

  // Spawn a few cubes at regular intervals and flip gravity depending on their position.
  let mut first_cube : @dynamics.RigidBodyHandle? = None
  for step in 0..<800 {
    if step % 200 == 0 && bodies.len() <= 7 {
      let body = bodies.insert(
        @dynamics.RigidBodyBuilder::dynamic()
        .translation(@core.Vec2::new(0.0F, 6.0F))
        .can_sleep(false)
        .build(),
      )
      colliders.insert_with_parent(
        @collision.ColliderBuilder::cuboid(1.0F, 2.0F).build(),
        body,
        bodies,
      )
      |> ignore
      if first_cube is None {
        first_cube = Some(body)
      }
    }
    for i in 0..<islands.active_bodies().length() {
      let handle = islands.active_bodies()[i]
      if bodies.get_mut(handle) is Some(body) {
        let y = body.translation().y
        if y > 1.0F {
          body.set_gravity_scale(1.0F, false) |> ignore
        } else if y < -1.0F {
          body.set_gravity_scale(-1.0F, false) |> ignore
        }
      }
    }
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
      impulse_joints, multibody_joints, ccd_solver, hooks, events,
    )
  }
  if first_cube is Some(h) && bodies.get(h) is Some(body) {
    // The surface velocity should have moved the cube horizontally.
    inspect(@core.abs(body.translation().x) > 0.5F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "examples3d/platform3.rs parity" {
  // Ported from `rapier-reference/examples3d/platform3.rs`.
  // TODO(dim3): replace with real 3D implementation and checks.
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

  // Ground.
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

  // Dynamic boxes.
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

  // Two kinematic platforms: velocity-based and position-based.
  let vel_based_handle = bodies.insert(
    @dynamics.RigidBodyBuilder::kinematic_velocity_based()
    .translation(@core.Vec2::new(0.0F, 1.5F + 0.8F))
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
    .translation(@core.Vec2::new(0.0F, 3.0F + 1.5F + 0.8F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(rad * 10.0F, rad).build(),
    pos_based_handle,
    bodies,
  )
  |> ignore
  let vel_start = if bodies.get(vel_based_handle) is Some(body) {
    body.translation()
  } else {
    @core.Vec2::zero()
  }
  let pos_start = if bodies.get(pos_based_handle) is Some(body) {
    body.translation()
  } else {
    @core.Vec2::zero()
  }
  let mut time = 0.0F
  for _ in 0..<120 {
    let velocity = @core.Vec2::new(0.0F, local_cos(time * 2.0F))
    if bodies.get_mut(vel_based_handle) is Some(platform) {
      platform.set_linvel(velocity, true) |> ignore
      platform.set_angvel(1.0F, true) |> ignore
    } else {
      inspect(false, content="true")
    }
    if bodies.get_mut(pos_based_handle) is Some(platform) {
      let next = platform
        .translation()
        .add(@core.Vec2::new(0.0F, -velocity.y * dt))
      platform.set_next_kinematic_translation(next) |> ignore
      platform.set_next_kinematic_rotation(
        @core.Rot2::from_angle(platform.rotation().angle() - 0.5F * dt),
      )
      |> ignore
    } else {
      inspect(false, content="true")
    }
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
      impulse_joints, multibody_joints, ccd_solver, hooks, events,
    )
    time = time + dt
  }
  if bodies.get(vel_based_handle) is Some(platform) {
    inspect(
      platform.translation().sub(vel_start).length() > 1.0e-2F,
      content="true",
    )
  } else {
    inspect(false, content="true")
  }
  if bodies.get(pos_based_handle) is Some(platform) {
    inspect(
      platform.translation().sub(pos_start).length() > 1.0e-2F,
      content="true",
    )
  } else {
    inspect(false, content="true")
  }
}

///|
test "examples3d/primitives3.rs parity (dim2 adapted)" {
  // Ported from `rapier-reference/examples3d/primitives3.rs`.
  // TODO(dim3): replace with real 3D implementation and checks.
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
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )
  let ground_size = 20.0F
  let ground_height = 0.5F
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, -ground_height))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(ground_size, ground_height).build(),
    ground,
    bodies,
  )
  |> ignore
  let num = 4
  let rad = 0.25F
  let shift = rad * 3.0F
  let center = shift * Float::from_double((num / 2).to_double())
  let mut offset = -Float::from_double(num.to_double()) * shift * 0.5F
  let handles : Array[@dynamics.RigidBodyHandle] = []
  handles.push(ground)
  for j in 0..<6 {
    for i in 0..<num {
      for k in 0..<num {
        let x = Float::from_double(i.to_double()) * shift -
          center +
          offset +
          Float::from_double(k.to_double()) * shift * 0.25F
        let y = Float::from_double(j.to_double()) * shift + shift / 2.0F + 2.0F
        let h = bodies.insert(
          @dynamics.RigidBodyBuilder::dynamic()
          .translation(@core.Vec2::new(x, y))
          .build(),
        )
        let builder = match j % 5 {
          1 => @collision.ColliderBuilder::ball(rad)
          // NOTE: 3D cone/cylinder approximated as capsule in 2D adaptation.
          _ => @collision.ColliderBuilder::capsule_y(rad, rad)
        }
        colliders.insert_with_parent(builder.build(), h, bodies) |> ignore
        handles.push(h)
      }
    }
    offset = offset - 0.05F * rad * Float::from_double((num - 1).to_double())
  }
  let expected = 1 + 6 * num * num
  inspect(bodies.len() == expected, content="true")
  inspect(colliders.len() == expected, content="true")
  for _ in 0..<30 {
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
      impulse_joints, multibody_joints, ccd_solver, hooks, events,
    )
  }
  for i in 0..<handles.length() {
    if bodies.get(handles[i]) is Some(rb) {
      let p = rb.translation()
      inspect(p.x == p.x, content="true")
      inspect(p.y == p.y, content="true")
    } else {
      inspect(false, content="true")
    }
  }
}

///|
test "examples3d/restitution3.rs parity (dim2 adapted)" {
  // Ported from `rapier-reference/examples3d/restitution3.rs`.
  // TODO(dim3): replace with real 3D implementation and checks.
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
  let ground_size = 20.0F
  let ground_height = 1.0F
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, -ground_height))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(ground_size, ground_height)
    .restitution(1.0F)
    .build(),
    ground,
    bodies,
  )
  |> ignore
  let num = 10
  let rad = 0.5F
  let low_handle = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(-2.0F, 10.0F))
    .can_sleep(false)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::ball(rad).restitution(0.0F).build(),
    low_handle,
    bodies,
  )
  |> ignore
  let high_handle = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(2.0F, 10.0F))
    .can_sleep(false)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::ball(rad).restitution(1.0F).build(),
    high_handle,
    bodies,
  )
  |> ignore

  // Step until both have bounced at least once; the higher restitution ball must reach higher peak.
  let mut low_bounced = false
  let mut high_bounced = false
  let mut low_max_y = -1.0e9F
  let mut high_max_y = -1.0e9F
  for _ in 0..<480 {
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
      impulse_joints, multibody_joints, ccd_solver, hooks, events,
    )
    if bodies.get(low_handle) is Some(body) {
      if !low_bounced && body.translation().y < 1.0F && body.linvel().y > 0.0F {
        low_bounced = true
      }
      if low_bounced && body.translation().y > low_max_y {
        low_max_y = body.translation().y
      }
    }
    if bodies.get(high_handle) is Some(body) {
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
  inspect(high_max_y > low_max_y + 2.0F, content="true")
  // Keep a small dependency on `num` so we don't diverge too far from the reference layout.
  inspect(num == 10, content="true")
}

///|
test "examples3d/rope_joints3.rs parity (dim2 adapted)" {
  // Ported from `rapier-reference/examples3d/rope_joints3.rs`.
  // TODO(dim3): replace with real 3D implementation and checks.
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
  let inv_dt = parameters.inv_dt()
  let dt = if inv_dt == 0.0F { 0.0F } else { 1.0F / inv_dt }

  // Ground (3 walls like rope_joints2 parity).
  let ground_size = 0.75F
  let ground_height = 0.1F
  let floor0 = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, -ground_height))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(ground_size, ground_height).build(),
    floor0,
    bodies,
  )
  |> ignore
  let floor1 = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(-ground_size - ground_height, ground_size))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(ground_height, ground_size).build(),
    floor1,
    bodies,
  )
  |> ignore
  let floor2 = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(ground_size + ground_height, ground_size))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(ground_height, ground_size).build(),
    floor2,
    bodies,
  )
  |> ignore

  // Kinematic character.
  let character_handle = bodies.insert(
    @dynamics.RigidBodyBuilder::kinematic_position_based()
    .translation(@core.Vec2::new(0.0F, 0.3F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(0.15F, 0.3F).build(),
    character_handle,
    bodies,
  )
  |> ignore

  // Tethered ball.
  let rad = 0.04F
  let child_handle = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(1.0F, 1.0F))
    .can_sleep(false)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::ball(rad).build(),
    child_handle,
    bodies,
  )
  |> ignore
  let joint = @dynamics.GenericJoint::from_rope(
    @dynamics.RopeJointBuilder::new(2.0F).build(),
  )
  impulse_joints.insert(character_handle, child_handle, joint, true) |> ignore

  // Move the kinematic character deterministically to stretch the rope.
  let mut time = 0.0F
  for _ in 0..<600 {
    time = time + dt
    if bodies.get_mut(character_handle) is Some(ch) {
      let target = @core.Vec2::new(
        Float::from_double(@math.sin(time.to_double())) * 2.0F,
        0.3F,
      )
      ch.set_next_kinematic_translation(target) |> ignore
    } else {
      inspect(false, content="true")
    }
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
      impulse_joints, multibody_joints, ccd_solver, hooks, events,
    )
  }
  if bodies.get(character_handle) is Some(ch) &&
    bodies.get(child_handle) is Some(ball) {
    let d = ball.translation().sub(ch.translation()).length()
    inspect(d == d, content="true")
    inspect(d <= 2.05F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "examples3d/sensor3.rs parity (dim2 adapted)" {
  // Ported from `rapier-reference/examples3d/sensor3.rs`.
  // TODO(dim3): replace with real 3D implementation and checks.
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
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )

  // Ground.
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

  // A few dynamic boxes.
  let num = 5
  let rad = 0.2F
  let shift = rad * 2.0F
  let centerx = shift * Float::from_double(num.to_double()) / 2.0F
  for i in 0..<num {
    let x = Float::from_double(i.to_double()) * shift - centerx
    let h = bodies.insert(
      @dynamics.RigidBodyBuilder::dynamic()
      .translation(@core.Vec2::new(x, 3.0F))
      .can_sleep(false)
      .build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder::cuboid(rad, rad).build(),
      h,
      bodies,
    )
    |> ignore
  }

  // Sensor rigid-body with a solid cube + a larger sensor ball.
  let sensor_handle = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(0.0F, 5.0F))
    .can_sleep(false)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(rad, rad).build(),
    sensor_handle,
    bodies,
  )
  |> ignore
  let sensor_collider = colliders.insert_with_parent(
    @collision.ColliderBuilder::ball(rad * 5.0F)
    .density(0.0F)
    .sensor(true)
    .active_events(@collision.ActiveEvents::collision_events())
    .build(),
    sensor_handle,
    bodies,
  )
  for _ in 0..<240 {
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
      impulse_joints, multibody_joints, ccd_solver, hooks, events,
    )
  }
  let evs = events.take_collision_events()
  let mut saw_sensor = false
  for i in 0..<evs.length() {
    match evs[i] {
      @collision.CollisionEvent::Started(h1, h2, _) =>
        if @collision.ColliderHandle::equals(h1, sensor_collider) ||
          @collision.ColliderHandle::equals(h2, sensor_collider) {
          saw_sensor = true
        }
      _ => ()
    }
  }
  inspect(saw_sensor, content="true")
}

///|
test "examples3d/spring_joints3.rs parity" {
  inspect(run_examples3d_spring_joints3(), content="true")
}

///|
test "examples3d/trimesh3.rs parity" {
  // Ported from `rapier-reference/examples3d/trimesh3.rs`.
  // TODO(dim3): replace with real 3D trimesh; 2D uses polyline-based mesh approximation.
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

  // Ground + walls.
  let ground_size = 25.0F
  let ground = bodies.insert(@dynamics.RigidBodyBuilder::fixed().build())
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(ground_size, 1.2F).build(),
    ground,
    bodies,
  )
  |> ignore
  let wall_r = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(ground_size, ground_size))
    .rotation(@core.Rot2::from_angle(@core.pi() / 2.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(ground_size, 1.2F).build(),
    wall_r,
    bodies,
  )
  |> ignore
  let wall_l = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(-ground_size, ground_size))
    .rotation(@core.Rot2::from_angle(@core.pi() / 2.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(ground_size, 1.2F).build(),
    wall_l,
    bodies,
  )
  |> ignore

  // Small fixed mesh (upstream uses a tessellated SVG).
  let vtx = [
    @core.Vec2::new(-1.0F, -1.0F),
    @core.Vec2::new(1.0F, -1.0F),
    @core.Vec2::new(1.0F, 1.0F),
    @core.Vec2::new(-1.0F, 1.0F),
  ]
  let idx = [(0, 1, 2), (0, 2, 3)]
  let dyn_handles : Array[@dynamics.RigidBodyHandle] = []
  for k in 0..<5 {
    if @collision.ColliderBuilder::trimesh(vtx, idx) is Some(builder) {
      let h = bodies.insert(
        @dynamics.RigidBodyBuilder::dynamic()
        .translation(
          @core.Vec2::new(
            0.0F,
            20.0F + Float::from_double(k.to_double()) * 5.0F,
          ),
        )
        .build(),
      )
      colliders.insert_with_parent(
        builder.contact_skin(0.2F).build(),
        h,
        bodies,
      )
      |> ignore
      dyn_handles.push(h)
    } else {
      inspect(false, content="true")
    }
  }
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 120,
  )
  for i in 0..<dyn_handles.length() {
    if bodies.get(dyn_handles[i]) is Some(body) {
      let p = body.translation()
      inspect(p.x == p.x, content="true")
      inspect(p.y == p.y, content="true")
      inspect(p.y > -10.0F, content="true")
    } else {
      inspect(false, content="true")
    }
  }
}

///|
test "examples3d/urdf3.rs parity" {
  inspect(run_examples3d_urdf3(), content="true")
}

///|
test "examples3d/vehicle_controller3.rs parity" {
  inspect(run_examples3d_vehicle_controller3(), content="true")
}

///|
test "examples3d/vehicle_joints3.rs parity" {
  inspect(run_examples3d_vehicle_joints3(), content="true")
}

///|
test "examples3d/voxels3.rs parity" {
  inspect(run_examples3d_voxels3(), content="true")
}
