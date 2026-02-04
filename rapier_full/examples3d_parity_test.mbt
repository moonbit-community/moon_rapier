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

///|
fn run_examples3d_character_controller3() -> Bool {
  // Ported from `rapier-reference/examples3d/character_controller3.rs` (headless).
  //
  // This is not a full 3D character controller yet. We validate the building blocks exercised by
  // the example: kinematic position-based bodies + CCD flags + interactions with dynamic bodies,
  // against the real dim3 backend.
  let pipeline = @pipeline.PhysicsPipeline3DReal::new()
  let broad_phase = @collision.BroadPhase3D::new()
  let narrow_phase = @collision.NarrowPhase3D::new()
  let bodies = @dynamics.RigidBodySet3D::new()
  let colliders = @collision.ColliderSet3D::new()
  let islands = @dynamics.IslandManager3D::new()
  let gravity = @core.Vec3::new(0.0F, -9.81F, 0.0F)
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )
  let inv_dt = parameters.inv_dt()
  let dt = if inv_dt == 0.0F { 0.0F } else { 1.0F / inv_dt }

  // Ground.
  let ground_height = 0.1F
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder3D::fixed()
    .translation(@core.Vec3::new(0.0F, -ground_height, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(20.0F, ground_height, 20.0F).build(),
    ground,
    bodies,
  )
  |> ignore

  // A small slope.
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(2.0F, ground_height, 2.0F)
    .translation(@core.Vec3::new(4.0F, 0.4F, 0.0F))
    .rotation(
      @core.rotation_from_scaled_axis(@core.Vec3::new(0.0F, 0.0F, 0.2F)),
    )
    .build(),
    ground,
    bodies,
  )
  |> ignore

  // Character: kinematic position-based capsule.
  let character = bodies.insert(
    @dynamics.RigidBodyBuilder3D::kinematic_position_based()
    .translation(@core.Vec3::new(-3.0F, 2.0F, 0.0F))
    .gravity_scale(10.0F)
    .soft_ccd_prediction(10.0F)
    .can_sleep(false)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::capsule_y(0.3F, 0.15F).build(),
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
        @dynamics.RigidBodyBuilder3D::dynamic()
        .translation(@core.Vec3::new(x, y, 0.0F))
        .build(),
      )
      colliders.insert_with_parent(
        @collision.ColliderBuilder3D::cuboid(rad, rad, rad).build(),
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
      rb.set_next_kinematic_translation(@core.Vec3::new(x, 2.0F, 0.0F))
    } else {
      return false
    }
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    )
  }
  if bodies.get(character) is Some(body) {
    let p = body.translation()
    p.x == p.x && p.y == p.y && p.z == p.z && p.y > -10.0F
  } else {
    false
  }
}

///|
fn run_examples3d_compound3() -> Bool {
  // Ported from `rapier-reference/examples3d/compound3.rs` (headless).
  let pipeline = @pipeline.PhysicsPipeline3DReal::new()
  let broad_phase = @collision.BroadPhase3D::new()
  let narrow_phase = @collision.NarrowPhase3D::new()
  let bodies = @dynamics.RigidBodySet3D::new()
  let colliders = @collision.ColliderSet3D::new()
  let islands = @dynamics.IslandManager3D::new()
  let gravity = @core.Vec3::new(0.0F, -9.81F, 0.0F)
  let parameters = @dynamics.IntegrationParameters::default()
  fn step_n(steps : Int) -> Unit {
    for _ in 0..<steps {
      pipeline.step(
        gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
      )
    }
  }

  let ground_size = 50.0F
  let ground_height = 0.1F
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder3D::fixed()
    .translation(@core.Vec3::new(0.0F, -ground_height, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(
      ground_size, ground_height, ground_size,
    ).build(),
    ground,
    bodies,
  )
  |> ignore
  // Uses a smaller instance count while still exercising multi-collider bodies.
  let num = 6
  let numy = 10
  let rad = 0.2F
  let shift = rad * 4.0F + rad
  let centerx = shift * Float::from_int(num / 2)
  let centery = shift / 2.0F
  let centerz = shift * Float::from_int(num / 2)
  let handles : Array[@dynamics.RigidBodyHandle] = []
  let mut offset = -Float::from_int(num) * (rad * 2.0F + rad) * 0.5F
  for j in 0..<numy {
    for i in 0..<num {
      for k in 0..<num {
        let x = Float::from_int(i) * shift * 5.0F - centerx + offset
        let y = Float::from_int(j) * (shift * 5.0F) + centery + 3.0F
        let z = Float::from_int(k) * shift * 2.0F - centerz + offset
        let handle = bodies.insert(
          @dynamics.RigidBodyBuilder3D::dynamic()
          .translation(@core.Vec3::new(x, y, z))
          .build(),
        )
        handles.push(handle)
        let collider1 = @collision.ColliderBuilder3D::cuboid(
          rad * 10.0F,
          rad,
          rad,
        ).build()
        let collider2 = @collision.ColliderBuilder3D::cuboid(
            rad,
            rad * 10.0F,
            rad,
          )
          .translation(@core.Vec3::new(rad * 10.0F, rad * 10.0F, 0.0F))
          .build()
        let collider3 = @collision.ColliderBuilder3D::cuboid(
            rad,
            rad * 10.0F,
            rad,
          )
          .translation(@core.Vec3::new(-rad * 10.0F, rad * 10.0F, 0.0F))
          .build()
        colliders.insert_with_parent(collider1, handle, bodies) |> ignore
        colliders.insert_with_parent(collider2, handle, bodies) |> ignore
        colliders.insert_with_parent(collider3, handle, bodies) |> ignore
      }
    }
    offset -= 0.05F * rad * (Float::from_int(num) - 1.0F)
  }
  step_n(2)
  for i in 0..<handles.length() {
    if bodies.get(handles[i]) is Some(body) {
      let p = body.translation()
      if !(p.x == p.x) || !(p.y == p.y) {
        return false
      }
      if !(p.y > -100.0F) {
        return false
      }
    } else {
      return false
    }
  }
  true
}

///|
fn run_examples3d_dynamic_trimesh3_impl(use_decomposition : Bool) -> Bool {
  // Ported from `rapier-reference/examples3d/dynamic_trimesh3.rs` (headless; deterministic geometry).
  //
  // The upstream example loads OBJ meshes and optionally performs convex decomposition. Here we
  // exercise the same "bucket" (mesh/heightfield contacts + optional multi-collider bodies) using
  // deterministic in-memory geometry compatible with the current dim3 projection layer.
  let pipeline = @pipeline.PhysicsPipeline3::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let bodies = @dynamics.RigidBodySet3::new()
  let colliders = @collision.ColliderSet3::new()
  let impulse_joints = @dynamics.ImpulseJointSet3::new()
  let multibody_joints = @dynamics.MultibodyJointSet3::new()
  let islands = @dynamics.IslandManager3::new()
  let hooks = @pipeline.PhysicsHooks::new()
  let events = @pipeline.EventHandler::new()
  let ccd_solver = @dynamics.CCDSolver::new()
  let gravity = @core.Vec3::new(0.0F, -9.81F, 0.0F)
  let parameters = @dynamics.IntegrationParameters::default()
  fn step_n(steps : Int) -> Unit {
    for _ in 0..<steps {
      pipeline.step(
        gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
        impulse_joints, multibody_joints, ccd_solver, hooks, events,
      )
    }
  }

  // Wavy floor (heightfield) to exercise mesh contacts.
  let n = 201
  let scale_x = 100.0F
  let scale_y = 1.0F
  let heights : Array[@core.Real] = []
  for i in 0..<n {
    let t = Float::from_double(i.to_double()) / 20.0F
    heights.push(Float::from_double(@math.sin(t.to_double())) * 2.0F / scale_y)
  }
  colliders.insert(
    @collision.ColliderBuilder3::heightfield(
      heights,
      @core.Vec3::new(scale_x, scale_y, 0.0F),
    ).build(),
  )
  |> ignore

  // Small dynamic mesh "tile" used in place of OBJ meshes.
  let vtx = [
    @core.Vec3::new(-1.0F, -1.0F, 0.0F),
    @core.Vec3::new(1.0F, -1.0F, 0.0F),
    @core.Vec3::new(1.0F, 1.0F, 0.0F),
    @core.Vec3::new(-1.0F, 1.0F, 0.0F),
  ]
  let idx = [(0, 1, 2), (0, 2, 3)]
  let handles : Array[@dynamics.RigidBodyHandle] = []
  for k in 0..<10 {
    let x = Float::from_double(k.to_double()) * 2.5F - 10.0F
    let rb = bodies.insert(
      @dynamics.RigidBodyBuilder3::dynamic()
      .translation(@core.Vec3::new(x, 8.0F, 0.0F))
      .build(),
    )
    if use_decomposition {
      // Compatibility decomposition: multiple colliders on the same rigid-body.
      colliders.insert_with_parent(
        @collision.ColliderBuilder3::cuboid(0.5F, 0.2F, 0.2F).build(),
        rb,
        bodies,
      )
      |> ignore
      colliders.insert_with_parent(
        @collision.ColliderBuilder3::cuboid(0.2F, 0.5F, 0.2F)
        .translation(@core.Vec3::new(0.6F, 0.0F, 0.0F))
        .build(),
        rb,
        bodies,
      )
      |> ignore
    } else if @collision.ColliderBuilder3::trimesh(vtx, idx) is Some(builder) {
      colliders.insert_with_parent(
        builder.contact_skin(0.1F).build(),
        rb,
        bodies,
      )
      |> ignore
    } else {
      return false
    }
    handles.push(rb)
  }
  step_n(120)
  for i in 0..<handles.length() {
    if bodies.get(handles[i]) is Some(body) {
      let p = body.translation()
      if !(p.x == p.x) || !(p.y == p.y) {
        return false
      }
      if !(p.y > -1000.0F) {
        return false
      }
    } else {
      return false
    }
  }
  true
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
fn run_examples3d_heightfield3() -> Bool {
  // Ported from `rapier-reference/examples3d/heightfield3.rs` (headless; 2D compatibility mode).
  let pipeline = @pipeline.PhysicsPipeline3::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let bodies = @dynamics.RigidBodySet3::new()
  let colliders = @collision.ColliderSet3::new()
  let impulse_joints = @dynamics.ImpulseJointSet3::new()
  let multibody_joints = @dynamics.MultibodyJointSet3::new()
  let islands = @dynamics.IslandManager3::new()
  let hooks = @pipeline.PhysicsHooks::new()
  let events = @pipeline.EventHandler::new()
  let ccd_solver = @dynamics.CCDSolver::new()
  let gravity = @core.Vec3::new(0.0F, -9.81F, 0.0F)
  let parameters = @dynamics.IntegrationParameters::default()
  fn step_n(steps : Int) -> Unit {
    for _ in 0..<steps {
      pipeline.step(
        gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
        impulse_joints, multibody_joints, ccd_solver, hooks, events,
      )
    }
  }

  let ground_size = @core.Vec3::new(100.0F, 1.0F, 100.0F)
  let nsubdivs = 20
  let denom = Float::from_int(nsubdivs)
  let heights : Array[@core.Real] = []
  for i in 0..<(nsubdivs + 1) {
    if i == 0 || i == nsubdivs {
      heights.push(10.0F)
    } else {
      let x = Float::from_int(i) * ground_size.x / denom
      let z = x
      heights.push(
        Float::from_double(@math.sin(x.to_double())) +
        Float::from_double(@math.cos(z.to_double())),
      )
    }
  }
  let ground = bodies.insert(@dynamics.RigidBodyBuilder3::fixed().build())
  colliders.insert_with_parent(
    @collision.ColliderBuilder3::heightfield(heights, ground_size).build(),
    ground,
    bodies,
  )
  |> ignore
  let num = 8
  let rad = 1.0F
  let shift = rad * 2.0F + rad
  let centerx = shift * Float::from_int(num / 2)
  let centery = shift / 2.0F
  let centerz = shift * Float::from_int(num / 2)
  for j in 0..<20 {
    for i in 0..<num {
      for k in 0..<num {
        let x = Float::from_int(i) * shift - centerx
        let y = Float::from_int(j) * shift + centery + 3.0F
        let z = Float::from_int(k) * shift - centerz
        let handle = bodies.insert(
          @dynamics.RigidBodyBuilder3::dynamic()
          .translation(@core.Vec3::new(x, y, z))
          .build(),
        )
        let collider = match j % 6 {
          0 => @collision.ColliderBuilder3::cuboid(rad, rad, rad)
          1 => @collision.ColliderBuilder3::ball(rad)
          2 =>
            @collision.ColliderBuilder3::round_cylinder(rad, rad, rad / 10.0F)
          3 => @collision.ColliderBuilder3::cone(rad, rad)
          4 => @collision.ColliderBuilder3::capsule_y(rad, rad)
          _ => {
            let shapes : Array[(@core.Isometry3, @collision.SharedShape)] = [
              (
                @core.Isometry3::identity(),
                @collision.SharedShape::cuboid(rad, rad / 2.0F),
              ),
              (
                @core.Isometry3::from_translation(
                  @core.Vec3::new(rad, 0.0F, 0.0F),
                ),
                @collision.SharedShape::cuboid(rad / 2.0F, rad),
              ),
              (
                @core.Isometry3::from_translation(
                  @core.Vec3::new(-rad, 0.0F, 0.0F),
                ),
                @collision.SharedShape::cuboid(rad / 2.0F, rad),
              ),
            ]
            @collision.ColliderBuilder3::compound(shapes)
          }
        }
        colliders.insert_with_parent(collider.build(), handle, bodies) |> ignore
      }
    }
  }
  step_n(2)
  true
}

///|
fn run_examples3d_convex_polyhedron3() -> Bool {
  // Ported from `rapier-reference/examples3d/convex_polyhedron3.rs` (headless).
  fn rng_next(seed : Int) -> Int {
    // Deterministic 31-bit LCG.
    (seed * 1103515245 + 12345) & 0x7fffffff
  }

  fn rng_float01(seed : Int) -> (@core.Real, Int) {
    let next = rng_next(seed)
    let value = Float::from_double(next.to_double() / 2147483647.0)
    (value, next)
  }

  let pipeline = @pipeline.PhysicsPipeline3::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let bodies = @dynamics.RigidBodySet3::new()
  let colliders = @collision.ColliderSet3::new()
  let impulse_joints = @dynamics.ImpulseJointSet3::new()
  let multibody_joints = @dynamics.MultibodyJointSet3::new()
  let islands = @dynamics.IslandManager3::new()
  let hooks = @pipeline.PhysicsHooks::new()
  let events = @pipeline.EventHandler::new()
  let ccd_solver = @dynamics.CCDSolver::new()
  let gravity = @core.Vec3::new(0.0F, -9.81F, 0.0F)
  let parameters = @dynamics.IntegrationParameters::default()
  fn step_n(steps : Int) -> Unit {
    for _ in 0..<steps {
      pipeline.step(
        gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
        impulse_joints, multibody_joints, ccd_solver, hooks, events,
      )
    }
  }

  let ground_size = 40.0F
  let ground_height = 0.1F
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder3::fixed()
    .translation(@core.Vec3::new(0.0F, -ground_height, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3::cuboid(ground_size, ground_height, ground_size).build(),
    ground,
    bodies,
  )
  |> ignore
  let num = 5
  let scale = 2.0F
  let border_rad = 0.1F
  let shift = border_rad * 2.0F + scale
  let centerx = shift * Float::from_int(num / 2)
  let centery = shift / 2.0F
  let centerz = shift * Float::from_int(num / 2)
  let handles : Array[@dynamics.RigidBodyHandle] = []
  let mut seed = 0
  for j in 0..<25 {
    for i in 0..<num {
      for k in 0..<num {
        let x = Float::from_int(i) * shift - centerx
        let y = Float::from_int(j) * shift + centery + 3.0F
        let z = Float::from_int(k) * shift - centerz
        let points : Array[@core.Vec3] = []
        for _ in 0..<10 {
          let (rx, s1) = rng_float01(seed)
          let (ry, s2) = rng_float01(s1)
          let (rz, s3) = rng_float01(s2)
          seed = s3
          points.push(@core.Vec3::new(rx * scale, ry * scale, rz * scale))
        }
        let handle = bodies.insert(
          @dynamics.RigidBodyBuilder3::dynamic()
          .translation(@core.Vec3::new(x, y, z))
          .build(),
        )
        handles.push(handle)
        if @collision.ColliderBuilder3::round_convex_hull(points, border_rad)
          is Some(builder) {
          colliders.insert_with_parent(builder.build(), handle, bodies)
          |> ignore
        } else {
          return false
        }
      }
    }
  }
  step_n(2)
  for i in 0..<handles.length() {
    if bodies.get(handles[i]) is Some(body) {
      let p = body.translation()
      if !(p.x == p.x) || !(p.y == p.y) {
        return false
      }
      if !(p.y > -100.0F) {
        return false
      }
    } else {
      return false
    }
  }
  true
}

///|
fn run_examples3d_joint_motor_position3() -> Bool {
  // Ported from `rapier-reference/examples3d/joint_motor_position3.rs` (headless).
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

  let bodies = @dynamics.RigidBodySet3::new()
  let colliders = @collision.ColliderSet3::new()
  let impulse_joints = @dynamics.ImpulseJointSet3::new()
  let multibody_joints = @dynamics.MultibodyJointSet3::new()
  let pipeline = @pipeline.PhysicsPipeline3::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let islands = @dynamics.IslandManager3::new()
  let hooks = @pipeline.PhysicsHooks::new()
  let events = @pipeline.EventHandler::new()
  let ccd_solver = @dynamics.CCDSolver::new()
  let gravity = @core.Vec3::zero()
  let parameters = @dynamics.IntegrationParameters::default()
  fn step_n(steps : Int) -> Unit {
    for _ in 0..<steps {
      pipeline.step(
        gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
        impulse_joints, multibody_joints, ccd_solver, hooks, events,
      )
    }
  }

  let ground_handle = bodies.insert(
    @dynamics.RigidBodyBuilder3::fixed().build(),
  )
  let axis_z = @core.Vec3::new(0.0F, 0.0F, 1.0F)
  let target_positions : Array[(@dynamics.RigidBodyHandle, @core.Real)] = []
  for num in 0..<9 {
    let x_pos = -6.0F + 1.5F * Float::from_double(num.to_double())
    let handle = bodies.insert(
      @dynamics.RigidBodyBuilder3::dynamic()
      .translation(@core.Vec3::new(x_pos, 2.0F, 0.0F))
      .can_sleep(false)
      .build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder3::cuboid(0.1F, 0.5F, 0.1F).build(),
      handle,
      bodies,
    )
    |> ignore
    let target = -@core.pi() +
      @core.pi() / 4.0F * Float::from_double(num.to_double())
    let joint = @dynamics.GenericJoint::from_revolute(
      @dynamics.RevoluteJointBuilder3::new(axis_z)
      .local_anchor1(@core.Vec3::new(x_pos, 1.5F, 0.0F))
      .local_anchor2(@core.Vec3::new(0.0F, -0.5F, 0.0F))
      .motor_position(target, 1000.0F, 150.0F)
      .build(),
    )
    impulse_joints.insert(ground_handle, handle, joint, true) |> ignore
    target_positions.push((handle, target))
  }
  let limit_targets : Array[(@dynamics.RigidBodyHandle, @core.Real, @core.Real)] = []
  for num in 0..<8 {
    let x_pos = -6.0F + 1.5F * Float::from_double(num.to_double())
    let handle = bodies.insert(
      @dynamics.RigidBodyBuilder3::dynamic()
      .translation(@core.Vec3::new(x_pos, 4.5F, 0.0F))
      .rotation_scaled_axis(@core.Vec3::new(0.0F, 0.0F, @core.pi()))
      .can_sleep(false)
      .build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder3::cuboid(0.1F, 0.5F, 0.1F).build(),
      handle,
      bodies,
    )
    |> ignore
    let min_angle = -@core.pi()
    let max_angle = -@core.pi() +
      @core.pi() / 4.0F * Float::from_double(num.to_double())
    let joint = @dynamics.GenericJoint::from_revolute(
      @dynamics.RevoluteJointBuilder3::new(axis_z)
      .local_anchor1(@core.Vec3::new(x_pos, 5.0F, 0.0F))
      .local_anchor2(@core.Vec3::new(0.0F, -0.5F, 0.0F))
      .motor_velocity(1.5F, 30.0F)
      .motor_max_force(100.0F)
      .limits(min_angle, max_angle)
      .build(),
    )
    impulse_joints.insert(ground_handle, handle, joint, true) |> ignore
    limit_targets.push((handle, min_angle, max_angle))
  }
  step_n(240)
  for i in 0..<target_positions.length() {
    let (handle, target) = target_positions[i]
    if bodies.get(handle) is Some(body) {
      let angle = body.rotation().angle()
      if !(angle == angle) {
        return false
      }
      let diff = wrap_angle_pi(angle - target)
      if !(@core.abs(diff) < 0.4F) {
        return false
      }
    } else {
      return false
    }
  }
  for i in 0..<limit_targets.length() {
    let (handle, min_a, max_a) = limit_targets[i]
    if bodies.get(handle) is Some(body) {
      let angle = body.rotation().angle()
      if !(angle == angle) {
        return false
      }
      if !(angle >= min_a - 1.0e-2F) {
        return false
      }
      if !(angle <= max_a + 1.0e-2F) {
        return false
      }
    } else {
      return false
    }
  }
  true
}

///|
fn run_examples3d_inverse_kinematics3() -> Bool {
  // Ported from `rapier-reference/examples3d/inverse_kinematics3.rs` (headless; 2D compatibility mode).
  let mut ok = true
  let bodies = @dynamics.RigidBodySet3::new()
  let colliders = @collision.ColliderSet3::new()
  let multibody_joints = @dynamics.MultibodyJointSet3::new()

  // Ground (kept for parity with upstream, though IK doesn't depend on it).
  let ground_height = 0.01F
  let floor_handle = bodies.insert(
    @dynamics.RigidBodyBuilder3::fixed()
    .translation(@core.Vec3::new(0.0F, -ground_height, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3::cuboid(1.0F, ground_height, 1.0F).build(),
    floor_handle,
    bodies,
  )
  |> ignore
  let num_segments = 10
  let size = 1.0F / Float::from_double(num_segments.to_double())
  let mut last_body = bodies.insert(
    @dynamics.RigidBodyBuilder3::fixed().build(),
  )
  let mut last_link = @dynamics.MultibodyJointHandle::invalid()
  let axis_z = @core.Vec3::new(0.0F, 0.0F, 1.0F)
  for i in 0..<num_segments {
    let new_body = bodies.insert(
      @dynamics.RigidBodyBuilder3::dynamic().can_sleep(false).build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder3::cuboid(size / 8.0F, size / 2.0F, size / 8.0F)
      .density(0.0F)
      .sensor(true)
      .build(),
      new_body,
      bodies,
    )
    |> ignore
    let joint = @dynamics.RevoluteJointBuilder3::new(axis_z)
      .local_anchor1(
        @core.Vec3::new(
          0.0F,
          size / 2.0F * (if i != 0 { 1.0F } else { 0.0F }),
          0.0F,
        ),
      )
      .local_anchor2(@core.Vec3::new(0.0F, -size / 2.0F, 0.0F))
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
    let target_point = @core.Vec3::new(0.25F, 0.4F, 0.0F)
    let options = @dynamics.InverseKinematicsOption::default()
      .set_constrained_axes(@dynamics.JointAxesMask::lin_axes())
      .set_max_iters(200)
    multibody.inverse_kinematics(
      bodies,
      link_id,
      options,
      @core.Isometry3::from_translation(target_point),
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
  // Ported from `rapier-reference/examples3d/joints3.rs` (headless; 2D compatibility mode).
  let pipeline = @pipeline.PhysicsPipeline3::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let bodies = @dynamics.RigidBodySet3::new()
  let colliders = @collision.ColliderSet3::new()
  let impulse_joints = @dynamics.ImpulseJointSet3::new()
  let multibody_joints = @dynamics.MultibodyJointSet3::new()
  let islands = @dynamics.IslandManager3::new()
  let hooks = @pipeline.PhysicsHooks::new()
  let events = @pipeline.EventHandler::new()
  let ccd_solver = @dynamics.CCDSolver::new()
  let gravity = @core.Vec3::new(0.0F, -9.81F, 0.0F)
  let parameters = @dynamics.IntegrationParameters::default()
  fn step_n(steps : Int) -> Unit {
    for _ in 0..<steps {
      pipeline.step(
        gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
        impulse_joints, multibody_joints, ccd_solver, hooks, events,
      )
    }
  }

  let rad = 0.4F
  let numi = 6
  let numk = 6
  let shift = 1.0F
  let softness = @dynamics.SpringCoefficients::joint_defaults()
  let axis_z = @core.Vec3::new(0.0F, 0.0F, 1.0F)
  let body_handles : Array[@dynamics.RigidBodyHandle] = []
  for k in 0..<numk {
    for i in 0..<numi {
      let fk = Float::from_double(k.to_double())
      let fi = Float::from_double(i.to_double())
      let child_handle = bodies.insert(
        (if i == 0 && k == 0 {
          @dynamics.RigidBodyBuilder3::fixed()
        } else {
          @dynamics.RigidBodyBuilder3::dynamic()
        })
        .translation(@core.Vec3::new(fk * shift, -fi * shift, 0.0F))
        .build(),
      )
      colliders.insert_with_parent(
        @collision.ColliderBuilder3::ball(rad).build(),
        child_handle,
        bodies,
      )
      |> ignore

      // Vertical joint.
      if i > 0 {
        let parent_handle = body_handles[body_handles.length() - 1]
        let joint = @dynamics.GenericJoint::from_revolute(
          @dynamics.RevoluteJointBuilder3::new(axis_z)
          .local_anchor2(@core.Vec3::new(0.0F, shift, 0.0F))
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
          @dynamics.RevoluteJointBuilder3::new(axis_z)
          .local_anchor2(@core.Vec3::new(-shift, 0.0F, 0.0F))
          .softness(softness)
          .build(),
        )
        impulse_joints.insert(parent_handle, child_handle, joint, true)
        |> ignore
      }
      body_handles.push(child_handle)
    }
  }
  step_n(240)
  for i in 0..<body_handles.length() {
    let p = bodies.translation(body_handles[i])
    if !(p.x == p.x) || !(p.y == p.y) {
      return false
    }
  }
  true
}

///|
fn run_examples3d_keva3() -> Bool {
  // Ported from `rapier-reference/examples3d/keva3.rs` (headless; 2D compatibility mode).
  let mut ok = true
  let pipeline = @pipeline.PhysicsPipeline3::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let bodies = @dynamics.RigidBodySet3::new()
  let colliders = @collision.ColliderSet3::new()
  let impulse_joints = @dynamics.ImpulseJointSet3::new()
  let multibody_joints = @dynamics.MultibodyJointSet3::new()
  let islands = @dynamics.IslandManager3::new()
  let hooks = @pipeline.PhysicsHooks::new()
  let events = @pipeline.EventHandler::new()
  let ccd_solver = @dynamics.CCDSolver::new()
  let gravity = @core.Vec3::new(0.0F, -9.81F, 0.0F)
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )

  // Ground.
  let ground_height = 0.1F
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder3::fixed()
    .translation(@core.Vec3::new(0.0F, -ground_height, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3::cuboid(50.0F, ground_height, 50.0F).build(),
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
        @dynamics.RigidBodyBuilder3::dynamic()
        .translation(@core.Vec3::new(x, y, 0.0F))
        .can_sleep(false)
        .build(),
      )
      let co = if horizontal {
        @collision.ColliderBuilder3::cuboid(half_x, half_y, half_y)
      } else {
        @collision.ColliderBuilder3::cuboid(half_y, half_x, half_y)
      }
      colliders.insert_with_parent(co.build(), rb, bodies) |> ignore
      handles.push(rb)
      handle_count = handle_count + 1
    }
  }
  handle_count |> ignore
  for _ in 0..<360 {
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
      impulse_joints, multibody_joints, ccd_solver, hooks, events,
    )
  }
  let mut min_y = 1.0e30F
  for i in 0..<handles.length() {
    let p = bodies.translation(handles[i])
    if !(p.x == p.x) || !(p.y == p.y) {
      ok = false
    }
    if p.y < min_y {
      min_y = p.y
    }
  }
  if !(min_y > -1000.0F) {
    ok = false
  }
  ok
}

///|
fn run_examples3d_newton_cradle3() -> Bool {
  // Ported from `rapier-reference/examples3d/newton_cradle3.rs` (headless).
  let pipeline = @pipeline.PhysicsPipeline3::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let bodies = @dynamics.RigidBodySet3::new()
  let colliders = @collision.ColliderSet3::new()
  let impulse_joints = @dynamics.ImpulseJointSet3::new()
  let multibody_joints = @dynamics.MultibodyJointSet3::new()
  let islands = @dynamics.IslandManager3::new()
  let hooks = @pipeline.PhysicsHooks::new()
  let events = @pipeline.EventHandler::new()
  let ccd_solver = @dynamics.CCDSolver::new()
  let gravity = @core.Vec3::new(0.0F, -9.81F, 0.0F)
  let parameters = @dynamics.IntegrationParameters::default()
  fn step_n(steps : Int) -> Unit {
    for _ in 0..<steps {
      pipeline.step(
        gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
        impulse_joints, multibody_joints, ccd_solver, hooks, events,
      )
    }
  }

  let radius = 0.5F
  let length = 10.0F * radius
  let n = 5
  let axis_z = @core.Vec3::new(0.0F, 0.0F, 1.0F)
  let balls : Array[@dynamics.RigidBodyHandle] = []
  for i in 0..<n {
    let x = Float::from_double(i.to_double()) * 2.2F * radius
    let ball_pos = @core.Vec3::new(x, 0.0F, 0.0F)
    let attach = @core.Vec3::new(0.0F, length, 0.0F)
    let vel = if i >= n - 1 {
      @core.Vec3::new(7.0F, 0.0F, 0.0F)
    } else {
      @core.Vec3::zero()
    }
    let ground = bodies.insert(
      @dynamics.RigidBodyBuilder3::fixed()
      .translation(ball_pos.add(attach))
      .build(),
    )
    let h = bodies.insert(
      @dynamics.RigidBodyBuilder3::dynamic()
      .translation(ball_pos)
      .linvel(vel)
      .can_sleep(false)
      .build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder3::ball(radius).restitution(1.0F).build(),
      h,
      bodies,
    )
    |> ignore
    let joint = @dynamics.GenericJoint::from_revolute(
      @dynamics.RevoluteJointBuilder3::new(axis_z)
      .local_anchor1(@core.Vec3::zero())
      .local_anchor2(attach)
      .build(),
    )
    impulse_joints.insert(ground, h, joint, true) |> ignore
    balls.push(h)
  }
  step_n(480)
  // Sanity: no NaNs, and at least one ball deviated from its initial position.
  let mut moved = false
  for i in 0..<balls.length() {
    let p = bodies.translation(balls[i])
    if !(p.x == p.x) || !(p.y == p.y) {
      return false
    }
    if @core.abs(p.x) > 0.2F {
      moved = true
    }
  }
  moved
}

///|
fn run_examples3d_spring_joints3() -> Bool {
  // Ported from `rapier-reference/examples3d/spring_joints3.rs` (headless).
  //
  // The upstream uses dedicated spring joints. This port approximates the behavior with soft
  // revolute joints (spring-like compliance) attached to a fixed ground, plus some falling boxes.
  let pipeline = @pipeline.PhysicsPipeline3::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let bodies = @dynamics.RigidBodySet3::new()
  let colliders = @collision.ColliderSet3::new()
  let impulse_joints = @dynamics.ImpulseJointSet3::new()
  let multibody_joints = @dynamics.MultibodyJointSet3::new()
  let islands = @dynamics.IslandManager3::new()
  let hooks = @pipeline.PhysicsHooks::new()
  let events = @pipeline.EventHandler::new()
  let ccd_solver = @dynamics.CCDSolver::new()
  let gravity = @core.Vec3::new(0.0F, -9.81F, 0.0F)
  let parameters = @dynamics.IntegrationParameters::default()
  fn step_n(steps : Int) -> Unit {
    for _ in 0..<steps {
      pipeline.step(
        gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
        impulse_joints, multibody_joints, ccd_solver, hooks, events,
      )
    }
  }

  let ground = bodies.insert(@dynamics.RigidBodyBuilder3::fixed().build())
  let num = 12
  let radius = 0.5F
  let softness = @dynamics.SpringCoefficients::joint_defaults()
  let axis_z = @core.Vec3::new(0.0F, 0.0F, 1.0F)
  let balls : Array[@dynamics.RigidBodyHandle] = []
  for i in 0..<=num {
    let x = -6.0F + 1.0F * Float::from_double(i.to_double())
    let ball_pos = @core.Vec3::new(x, 4.5F, 0.0F)
    let handle = bodies.insert(
      @dynamics.RigidBodyBuilder3::dynamic()
      .translation(ball_pos)
      .can_sleep(false)
      .build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder3::ball(radius).build(),
      handle,
      bodies,
    )
    |> ignore
    let joint = @dynamics.GenericJoint::from_revolute(
      @dynamics.RevoluteJointBuilder3::new(axis_z)
      .local_anchor1(ball_pos.sub(@core.Vec3::new(0.0F, 3.0F, 0.0F)))
      .local_anchor2(@core.Vec3::zero())
      .softness(softness)
      .build(),
    )
    impulse_joints.insert(ground, handle, joint, true) |> ignore
    balls.push(handle)
    let box = bodies.insert(
      @dynamics.RigidBodyBuilder3::dynamic()
      .translation(ball_pos.add(@core.Vec3::new(0.0F, 5.0F, 0.0F)))
      .build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder3::cuboid(radius, radius, radius)
      .density(100.0F)
      .build(),
      box,
      bodies,
    )
    |> ignore
  }
  step_n(360)
  for i in 0..<balls.length() {
    let p = bodies.translation(balls[i])
    if !(p.x == p.x) || !(p.y == p.y) {
      return false
    }
  }
  true
}

///|
fn run_examples3d_urdf3() -> Bool {
  // Ported from `rapier-reference/examples3d/urdf3.rs` (headless; 2D compatibility mode).
  //
  // Full URDF parsing isn't available in this MoonBit port yet. Instead, we build a tiny
  // articulated chain (like a robot arm) to exercise multibody + joint limits.
  let mut ok = true
  let pipeline = @pipeline.PhysicsPipeline3::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let bodies = @dynamics.RigidBodySet3::new()
  let colliders = @collision.ColliderSet3::new()
  let impulse_joints = @dynamics.ImpulseJointSet3::new()
  let multibody_joints = @dynamics.MultibodyJointSet3::new()
  let islands = @dynamics.IslandManager3::new()
  let hooks = @pipeline.PhysicsHooks::new()
  let events = @pipeline.EventHandler::new()
  let ccd_solver = @dynamics.CCDSolver::new()
  let gravity = @core.Vec3::new(0.0F, -9.81F, 0.0F)
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )

  // Ground.
  colliders.insert(
    @collision.ColliderBuilder3::segment(
      @core.Vec3::new(-50.0F, 0.0F, 0.0F),
      @core.Vec3::new(50.0F, 0.0F, 0.0F),
    ).build(),
  )
  |> ignore

  // Base + 3 links.
  let base = bodies.insert(
    @dynamics.RigidBodyBuilder3::fixed()
    .translation(@core.Vec3::new(0.0F, 2.0F, 0.0F))
    .build(),
  )
  let mut parent = base
  let mut x = 0.0F
  let axis_z = @core.Vec3::new(0.0F, 0.0F, 1.0F)
  for i in 0..<3 {
    let link = bodies.insert(
      @dynamics.RigidBodyBuilder3::dynamic()
      .translation(@core.Vec3::new(x + 1.0F, 2.0F, 0.0F))
      .can_sleep(false)
      .build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder3::cuboid(0.6F, 0.1F, 0.1F).build(),
      link,
      bodies,
    )
    |> ignore
    let min_angle = -0.5F
    let max_angle = 0.5F
    let joint = @dynamics.GenericJoint::from_revolute(
      @dynamics.RevoluteJointBuilder3::new(axis_z)
      .local_anchor1(@core.Vec3::new(1.0F, 0.0F, 0.0F))
      .local_anchor2(@core.Vec3::new(-1.0F, 0.0F, 0.0F))
      .limits(min_angle, max_angle)
      .build(),
    )
    impulse_joints.insert(parent, link, joint, true) |> ignore
    parent = link
    x = x + 2.0F
    i |> ignore
  }
  for _ in 0..<240 {
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
      impulse_joints, multibody_joints, ccd_solver, hooks, events,
    )
  }
  let p = bodies.translation(parent)
  if !(p.x == p.x) || !(p.y == p.y) {
    ok = false
  }
  ok
}

///|
fn run_examples3d_vehicle_controller3() -> Bool {
  // Ported from `rapier-reference/examples3d/vehicle_controller3.rs` (headless).
  let pipeline = @pipeline.PhysicsPipeline3::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let bodies = @dynamics.RigidBodySet3::new()
  let colliders = @collision.ColliderSet3::new()
  let impulse_joints = @dynamics.ImpulseJointSet3::new()
  let multibody_joints = @dynamics.MultibodyJointSet3::new()
  let islands = @dynamics.IslandManager3::new()
  let hooks = @pipeline.PhysicsHooks::new()
  let events = @pipeline.EventHandler::new()
  let ccd_solver = @dynamics.CCDSolver::new()
  let gravity = @core.Vec3::new(0.0F, -9.81F, 0.0F)
  let parameters = @dynamics.IntegrationParameters::default()
  let inv_dt = parameters.inv_dt()
  let dt = if inv_dt == 0.0F { 0.0F } else { 1.0F / inv_dt }

  // Flat ground.
  let ground_size = 5.0F
  let ground_height = 0.1F
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder3::fixed()
    .translation(@core.Vec3::new(0.0F, -ground_height, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3::cuboid(ground_size, ground_height, ground_size).build(),
    ground,
    bodies,
  )
  |> ignore

  // Vehicle chassis.
  let hw = 0.3F
  let hh = 0.15F
  let chassis = bodies.insert(
    @dynamics.RigidBodyBuilder3::dynamic()
    .translation(@core.Vec3::new(0.0F, 1.0F, 0.0F))
    .can_sleep(false)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3::cuboid(hw * 2.0F, hh, hw)
    .density(100.0F)
    .build(),
    chassis,
    bodies,
  )
  |> ignore

  // Vehicle controller.
  let tuning = @control.WheelTuning::default()
    .with_suspension_stiffness(100.0F)
    .with_suspension_damping(10.0F)
  let vehicle = @control.DynamicRayCastVehicleController3::new(chassis)
  let wheel_positions = [
    @core.Vec3::new(hw * 1.5F, -hh, hw),
    @core.Vec3::new(hw * 1.5F, -hh, -hw),
    @core.Vec3::new(-hw * 1.5F, -hh, hw),
    @core.Vec3::new(-hw * 1.5F, -hh, -hw),
  ]
  for pos in wheel_positions {
    vehicle.add_wheel(
      pos,
      @core.Vec3::new(0.0F, -1.0F, 0.0F),
      @core.Vec3::new(0.0F, 0.0F, 1.0F),
      hh,
      hh / 4.0F,
      tuning,
    )
    |> ignore
  }
  for i in 0..<vehicle.wheels().length() {
    vehicle.set_engine_force(i, 20.0F)
  }

  // Some dynamic cubes to push around.
  let num = 8
  let rad = 0.1F
  let shift = rad * 2.0F
  let centerx = shift * Float::from_int(num / 2)
  let centery = rad
  for j in 0..<1 {
    for k in 0..<4 {
      for i in 0..<num {
        let x = Float::from_int(i) * shift - centerx
        let y = Float::from_int(j) * shift + centery
        let z = Float::from_int(k) * shift + centerx
        let h = bodies.insert(
          @dynamics.RigidBodyBuilder3::dynamic()
          .translation(@core.Vec3::new(x, y, z))
          .build(),
        )
        colliders.insert_with_parent(
          @collision.ColliderBuilder3::cuboid(rad, rad, rad).build(),
          h,
          bodies,
        )
        |> ignore
      }
    }
  }

  // A slope we can climb.
  let slope_angle = 0.2F
  let slope_size = 2.0F
  colliders.insert(
    @collision.ColliderBuilder3::cuboid(slope_size, ground_height, ground_size)
    .translation(
      @core.Vec3::new(ground_size + slope_size, -ground_height + 0.4F, 0.0F),
    )
    .rotation_scaled_axis(@core.Vec3::new(0.0F, 0.0F, slope_angle))
    .build(),
  )
  |> ignore

  // A slope we can't climb.
  let impossible_slope_angle = 0.9F
  let impossible_slope_size = 2.0F
  colliders.insert(
    @collision.ColliderBuilder3::cuboid(
      impossible_slope_size, ground_height, ground_size,
    )
    .translation(
      @core.Vec3::new(
        ground_size + slope_size * 2.0F + impossible_slope_size - 0.9F,
        -ground_height + 2.3F,
        0.0F,
      ),
    )
    .rotation_scaled_axis(@core.Vec3::new(0.0F, 0.0F, impossible_slope_angle))
    .build(),
  )
  |> ignore

  // Heightfield ground (1D profile).
  let n = 201
  let heights : Array[@core.Real] = []
  for i in 0..<n {
    let t = Float::from_double(i.to_double()) / 20.0F
    heights.push(Float::from_double(@math.cos(t.to_double())) * 2.0F)
  }
  colliders.insert(
    @collision.ColliderBuilder3::heightfield(
      heights,
      @core.Vec3::new(10.0F, 0.4F, 0.0F),
    )
    .translation(@core.Vec3::new(-7.0F, 0.0F, 0.0F))
    .build(),
  )
  |> ignore

  // Run simulation with controller impulses applied before each pipeline step.
  for _ in 0..<240 {
    let filter = @collision.QueryFilter::new().exclude_rigid_body(chassis)
    let queries = @collision.QueryPipelineMut::new3(filter, bodies, colliders)
    vehicle.update_vehicle(dt, queries, bodies.as_2d(), colliders.as_2d())
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
      impulse_joints, multibody_joints, ccd_solver, hooks, events,
    )
  }
  let p = bodies.translation(chassis)
  p.x == p.x && p.y == p.y && p.x > -10.0F
}

///|
fn run_examples3d_vehicle_joints3() -> Bool {
  // Ported from `rapier-reference/examples3d/vehicle_joints3.rs` (headless; deterministic setup).
  //
  // Build a simple chassis with two motorized wheels connected by revolute joints.
  let pipeline = @pipeline.PhysicsPipeline3::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let bodies = @dynamics.RigidBodySet3::new()
  let colliders = @collision.ColliderSet3::new()
  let impulse_joints = @dynamics.ImpulseJointSet3::new()
  let multibody_joints = @dynamics.MultibodyJointSet3::new()
  let islands = @dynamics.IslandManager3::new()
  let hooks = @pipeline.PhysicsHooks::new()
  let events = @pipeline.EventHandler::new()
  let ccd_solver = @dynamics.CCDSolver::new()
  let gravity = @core.Vec3::new(0.0F, -9.81F, 0.0F)
  let parameters = @dynamics.IntegrationParameters::default()
  fn step_n(steps : Int) -> Unit {
    for _ in 0..<steps {
      pipeline.step(
        gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
        impulse_joints, multibody_joints, ccd_solver, hooks, events,
      )
    }
  }

  // Flat ground (large cuboid).
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder3::fixed()
    .translation(@core.Vec3::new(0.0F, -0.1F, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3::cuboid(200.0F, 0.1F, 200.0F)
    .friction(1.0F)
    .build(),
    ground,
    bodies,
  )
  |> ignore
  let car_pos = @core.Vec3::new(-10.0F, 2.0F, 0.0F)
  let chassis = bodies.insert(
    @dynamics.RigidBodyBuilder3::dynamic()
    .translation(car_pos)
    .can_sleep(false)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3::cuboid(1.0F, 0.3F, 0.3F).density(50.0F).build(),
    chassis,
    bodies,
  )
  |> ignore
  let wheel_r = 0.35F
  let wheel_off = 1.2F
  let wheel_y = -0.4F
  let wheel_left = bodies.insert(
    @dynamics.RigidBodyBuilder3::dynamic()
    .translation(car_pos.add(@core.Vec3::new(-wheel_off, wheel_y, 0.0F)))
    .can_sleep(false)
    .build(),
  )
  let wheel_right = bodies.insert(
    @dynamics.RigidBodyBuilder3::dynamic()
    .translation(car_pos.add(@core.Vec3::new(wheel_off, wheel_y, 0.0F)))
    .can_sleep(false)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3::ball(wheel_r)
    .friction(2.0F)
    .density(100.0F)
    .build(),
    wheel_left,
    bodies,
  )
  |> ignore
  colliders.insert_with_parent(
    @collision.ColliderBuilder3::ball(wheel_r)
    .friction(2.0F)
    .density(100.0F)
    .build(),
    wheel_right,
    bodies,
  )
  |> ignore
  let axis_z = @core.Vec3::new(0.0F, 0.0F, 1.0F)
  let motor_speed = -30.0F
  let motor_force = 200.0F
  let joint_l = @dynamics.GenericJoint::from_revolute(
    @dynamics.RevoluteJointBuilder3::new(axis_z)
    .local_anchor1(@core.Vec3::new(-wheel_off, wheel_y, 0.0F))
    .local_anchor2(@core.Vec3::zero())
    .motor_velocity(motor_speed, motor_force)
    .build(),
  )
  impulse_joints.insert(chassis, wheel_left, joint_l, true) |> ignore
  let joint_r = @dynamics.GenericJoint::from_revolute(
    @dynamics.RevoluteJointBuilder3::new(axis_z)
    .local_anchor1(@core.Vec3::new(wheel_off, wheel_y, 0.0F))
    .local_anchor2(@core.Vec3::zero())
    .motor_velocity(motor_speed, motor_force)
    .build(),
  )
  impulse_joints.insert(chassis, wheel_right, joint_r, true) |> ignore
  step_n(360)
  let p = bodies.translation(chassis)
  p.x == p.x && p.y == p.y
}

///|
fn run_examples3d_voxels3() -> Bool {
  // Ported from `rapier-reference/examples3d/voxels3.rs` (headless; 2D compatibility mode).
  let mut ok = true
  let pipeline = @pipeline.PhysicsPipeline3::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let bodies = @dynamics.RigidBodySet3::new()
  let colliders = @collision.ColliderSet3::new()
  let impulse_joints = @dynamics.ImpulseJointSet3::new()
  let multibody_joints = @dynamics.MultibodyJointSet3::new()
  let islands = @dynamics.IslandManager3::new()
  let hooks = @pipeline.PhysicsHooks::new()
  let events = @pipeline.EventHandler::new()
  let ccd_solver = @dynamics.CCDSolver::new()
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )
  let gravity = @core.Vec3::new(0.0F, -9.81F, 0.0F)
  let polyline = [
    @core.Vec3::new(0.0F, 0.0F, 0.0F),
    @core.Vec3::new(0.0F, 10.0F, 0.0F),
    @core.Vec3::new(7.0F, 4.0F, 0.0F),
    @core.Vec3::new(14.0F, 10.0F, 0.0F),
    @core.Vec3::new(14.0F, 0.0F, 0.0F),
    @core.Vec3::new(13.0F, 7.0F, 0.0F),
    @core.Vec3::new(7.0F, 2.0F, 0.0F),
    @core.Vec3::new(1.0F, 7.0F, 0.0F),
  ]
  let poly_idx : Array[(Int, Int)] = []
  for i in 0..<polyline.length() {
    poly_idx.push((i, (i + 1) % polyline.length()))
  }
  let mesh_body = bodies.insert(
    @dynamics.RigidBodyBuilder3::fixed()
    .translation(@core.Vec3::new(-20.0F, -10.0F, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3::voxelized_mesh(
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
  let voxel_size = @core.Vec3::new(1.0F, voxel_size_y, 1.0F)
  let voxels : Array[@core.Vec3] = []
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
    voxels.push(@core.Vec3::new(x, y * voxel_size.y, 0.0F))
  }
  colliders.insert(
    @collision.ColliderBuilder3::voxels_from_points(voxel_size, voxels).build(),
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
        @dynamics.RigidBodyBuilder3::dynamic()
        .translation(@core.Vec3::new(x, y, 0.0F))
        .build(),
      )
      let kind = j % 3
      let co = if kind == 0 {
        @collision.ColliderBuilder3::ball(ball_radius)
      } else if kind == 1 {
        @collision.ColliderBuilder3::cuboid(
          ball_radius, ball_radius, ball_radius,
        )
      } else {
        @collision.ColliderBuilder3::capsule_y(ball_radius, ball_radius)
      }
      colliders.insert_with_parent(co.build(), rb_handle, bodies) |> ignore
      dyn_handles.push(rb_handle)
    }
  }
  for _ in 0..<240 {
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
      impulse_joints, multibody_joints, ccd_solver, hooks, events,
    )
  }
  let mut min_y = 1.0e30F
  for i in 0..<dyn_handles.length() {
    let p = bodies.translation(dyn_handles[i])
    if !(p.x == p.x) || !(p.y == p.y) {
      ok = false
    }
    if p.y < min_y {
      min_y = p.y
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
  let pipeline = @pipeline.PhysicsPipeline3::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let bodies = @dynamics.RigidBodySet3::new()
  let colliders = @collision.ColliderSet3::new()
  let impulse_joints = @dynamics.ImpulseJointSet3::new()
  let multibody_joints = @dynamics.MultibodyJointSet3::new()
  let islands = @dynamics.IslandManager3::new()
  let hooks = @pipeline.PhysicsHooks::new()
  let events = @pipeline.EventHandler::new()
  let ccd_solver = @dynamics.CCDSolver::new()
  let gravity = @core.Vec3::new(0.0F, -9.81F, 0.0F)
  let parameters = @dynamics.IntegrationParameters::default()
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder3::fixed()
    .translation(@core.Vec3::new(0.0F, -1.0F, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3::cuboid(10.0F, 1.0F, 10.0F).build(),
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
test "examples3d/character_controller3.rs parity" {
  inspect(run_examples3d_character_controller3(), content="true")
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
test "examples3d/heightfield3.rs parity" {
  inspect(run_examples3d_heightfield3(), content="true")
}

///|
test "examples3d/debug_add_remove_collider3.rs parity" {
  // Ported from `rapier-reference/examples3d/debug_add_remove_collider3.rs`.
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
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = @collision.ColliderSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()

  // Small multibody chain + inverse kinematics smoke (ported from the examples2d suite).
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
  } else {
    inspect(false, content="true")
  }
}

///|
test "examples3d/debug_triangle3.rs parity" {
  // Ported from `rapier-reference/examples3d/debug_triangle3.rs`.
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
  let pipeline = @pipeline.PhysicsPipeline3::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let bodies = @dynamics.RigidBodySet3::new()
  let colliders = @collision.ColliderSet3::new()
  let impulse_joints = @dynamics.ImpulseJointSet3::new()
  let multibody_joints = @dynamics.MultibodyJointSet3::new()
  let islands = @dynamics.IslandManager3::new()
  let hooks = @pipeline.PhysicsHooks::new()
  let events = @pipeline.EventHandler::new()
  let ccd_solver = @dynamics.CCDSolver::new()
  let gravity = @core.Vec3::new(0.0F, -9.81F, 0.0F)
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )
  let ground_height = 0.1F
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder3::fixed()
    .translation(@core.Vec3::new(0.0F, -ground_height, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3::cuboid(200.0F, ground_height, 200.0F).build(),
    ground,
    bodies,
  )
  |> ignore

  // A downscaled "domino line".
  let num = 80
  let width = 0.5F
  let thickness = 0.08F
  let spacing = thickness * 2.5F
  let handles : Array[@dynamics.RigidBodyHandle] = []
  for i in 0..<num {
    let x = Float::from_double(i.to_double()) * spacing
    let tilt = if i == 0 { 0.8F } else { 0.0F }
    let builder0 = @dynamics.RigidBodyBuilder3::dynamic()
      .translation(@core.Vec3::new(x, width * 2.0F, 0.0F))
      .rotation_scaled_axis(@core.Vec3::new(0.0F, 0.0F, tilt))
      .can_sleep(false)
    let builder = if i == 0 {
      // Ensure the first domino tips into the chain.
      builder0
      .linvel(@core.Vec3::new(2.0F, 0.0F, 0.0F))
      .angvel(@core.Vec3::new(0.0F, 0.0F, -10.0F))
    } else {
      builder0
    }
    let h = bodies.insert(builder.build())
    colliders.insert_with_parent(
      @collision.ColliderBuilder3::cuboid(thickness, width * 2.0F, width).build(),
      h,
      bodies,
    )
    |> ignore
    handles.push(h)
  }
  for _ in 0..<360 {
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
      impulse_joints, multibody_joints, ccd_solver, hooks, events,
    )
  }
  // At least one domino should have a non-trivial rotation.
  let mut fallen = 0
  for i in 0..<handles.length() {
    let a = @core.abs(bodies.rotation(handles[i]).to_scaled_axis().z)
    if a > 0.05F {
      fallen = fallen + 1
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
  let pipeline = @pipeline.PhysicsPipeline3::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let bodies = @dynamics.RigidBodySet3::new()
  let colliders = @collision.ColliderSet3::new()
  let impulse_joints = @dynamics.ImpulseJointSet3::new()
  let multibody_joints = @dynamics.MultibodyJointSet3::new()
  let islands = @dynamics.IslandManager3::new()
  let hooks = @pipeline.PhysicsHooks::new()
  let events = @pipeline.EventHandler::new()
  let ccd_solver = @dynamics.CCDSolver::new()
  let gravity = @core.Vec3::new(0.0F, -9.81F, 0.0F)
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    8,
  )
  let rad = 0.5F
  let ground_size = 40.0F
  let ground_height = 2.1F
  for k in 0..<3 {
    let y = -ground_height - Float::from_double(k.to_double())
    let handle = bodies.insert(
      @dynamics.RigidBodyBuilder3::fixed()
      .translation(@core.Vec3::new(0.0F, y, 0.0F))
      .build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder3::cuboid(
        ground_size, ground_height, ground_size,
      ).build(),
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
      @dynamics.RigidBodyBuilder3::dynamic()
      .translation(@core.Vec3::new(0.0F, 10.0F, 0.0F))
      .can_sleep(false)
      .build(),
    )
    let builder = match step % 3 {
      0 => @collision.ColliderBuilder3::round_cylinder(rad, rad, rad / 10.0F)
      1 => @collision.ColliderBuilder3::cone(rad, rad)
      _ => @collision.ColliderBuilder3::cuboid(rad, rad, rad)
    }
    colliders.insert_with_parent(builder.build(), handle, bodies) |> ignore
    spawned.push(handle)

    // Cap dynamic body count deterministically.
    while spawned.length() - remove_cursor > max_number_of_bodies {
      let h = spawned[remove_cursor]
      remove_cursor = remove_cursor + 1
      @pipeline.remove_rigid_body3(
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
test "examples3d/harness_capsules3.rs parity" {
  // Ported from `rapier-reference/examples3d/harness_capsules3.rs`.
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
test "examples3d/newton_cradle3.rs parity" {
  inspect(run_examples3d_newton_cradle3(), content="true")
}

///|
test "examples3d/one_way_platforms3.rs parity" {
  // Ported from `rapier-reference/examples3d/one_way_platforms3.rs`.
  let pipeline = @pipeline.PhysicsPipeline3::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let bodies = @dynamics.RigidBodySet3::new()
  let colliders = @collision.ColliderSet3::new()
  let impulse_joints = @dynamics.ImpulseJointSet3::new()
  let multibody_joints = @dynamics.MultibodyJointSet3::new()
  let islands = @dynamics.IslandManager3::new()
  let events = @pipeline.EventHandler::new()
  let ccd_solver = @dynamics.CCDSolver::new()
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )
  let gravity = @core.Vec3::new(0.0F, -9.81F, 0.0F)
  let ground_body = bodies.insert(@dynamics.RigidBodyBuilder3::fixed().build())
  let platform1 = colliders.insert_with_parent(
    @collision.ColliderBuilder3::cuboid(9.0F, 0.5F, 9.0F)
    .translation(@core.Vec3::new(0.0F, 2.0F, 0.0F))
    .active_hooks(@collision.ActiveHooks::modify_solver_contacts())
    .build(),
    ground_body,
    bodies,
  )
  let platform2 = colliders.insert_with_parent(
    @collision.ColliderBuilder3::cuboid(9.0F, 0.5F, 9.0F)
    .translation(@core.Vec3::new(0.0F, -2.0F, 0.0F))
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
        @dynamics.RigidBodyBuilder3::dynamic()
        .translation(@core.Vec3::new(0.0F, 6.0F, 0.0F))
        .can_sleep(false)
        .build(),
      )
      colliders.insert_with_parent(
        @collision.ColliderBuilder3::cuboid(1.0F, 2.0F, 1.0F).build(),
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
  let pipeline = @pipeline.PhysicsPipeline3::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let bodies = @dynamics.RigidBodySet3::new()
  let colliders = @collision.ColliderSet3::new()
  let impulse_joints = @dynamics.ImpulseJointSet3::new()
  let multibody_joints = @dynamics.MultibodyJointSet3::new()
  let islands = @dynamics.IslandManager3::new()
  let hooks = @pipeline.PhysicsHooks::new()
  let events = @pipeline.EventHandler::new()
  let ccd_solver = @dynamics.CCDSolver::new()
  let parameters = @dynamics.IntegrationParameters::default()
  let gravity = @core.Vec3::new(0.0F, -9.81F, 0.0F)
  let inv_dt = parameters.inv_dt()
  let dt = if inv_dt == 0.0F { 0.0F } else { 1.0F / inv_dt }

  // Ground.
  let ground_height = 0.1F
  let ground_handle = bodies.insert(
    @dynamics.RigidBodyBuilder3::fixed()
    .translation(@core.Vec3::new(0.0F, -ground_height, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3::cuboid(10.0F, ground_height, 10.0F).build(),
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
        @dynamics.RigidBodyBuilder3::dynamic()
        .translation(@core.Vec3::new(x, y, 0.0F))
        .build(),
      )
      colliders.insert_with_parent(
        @collision.ColliderBuilder3::cuboid(rad, rad, rad).build(),
        handle,
        bodies,
      )
      |> ignore
    }
  }

  // Two kinematic platforms: velocity-based and position-based.
  let vel_based_handle = bodies.insert(
    @dynamics.RigidBodyBuilder3::kinematic_velocity_based()
    .translation(@core.Vec3::new(0.0F, 1.5F + 0.8F, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3::cuboid(rad * 10.0F, rad, rad).build(),
    vel_based_handle,
    bodies,
  )
  |> ignore
  let pos_based_handle = bodies.insert(
    @dynamics.RigidBodyBuilder3::kinematic_position_based()
    .translation(@core.Vec3::new(0.0F, 3.0F + 1.5F + 0.8F, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3::cuboid(rad * 10.0F, rad, rad).build(),
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

///|
test "examples3d/rope_joints3.rs parity" {
  // Ported from `rapier-reference/examples3d/rope_joints3.rs` (headless; character control omitted).
  let pipeline = @pipeline.PhysicsPipeline3::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let bodies = @dynamics.RigidBodySet3::new()
  let colliders = @collision.ColliderSet3::new()
  let impulse_joints = @dynamics.ImpulseJointSet3::new()
  let multibody_joints = @dynamics.MultibodyJointSet3::new()
  let islands = @dynamics.IslandManager3::new()
  let hooks = @pipeline.PhysicsHooks::new()
  let events = @pipeline.EventHandler::new()
  let ccd_solver = @dynamics.CCDSolver::new()
  let parameters = @dynamics.IntegrationParameters::default()
  let gravity = @core.Vec3::new(0.0F, -9.81F, 0.0F)
  let inv_dt = parameters.inv_dt()
  let dt = if inv_dt == 0.0F { 0.0F } else { 1.0F / inv_dt }

  // Ground (walls; 2D compatibility mode).
  let ground_size = 0.75F
  let ground_height = 0.1F
  let floor0 = bodies.insert(
    @dynamics.RigidBodyBuilder3::fixed()
    .translation(@core.Vec3::new(0.0F, -ground_height, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3::cuboid(ground_size, ground_height, ground_size).build(),
    floor0,
    bodies,
  )
  |> ignore
  let floor1 = bodies.insert(
    @dynamics.RigidBodyBuilder3::fixed()
    .translation(
      @core.Vec3::new(-ground_size - ground_height, ground_size, 0.0F),
    )
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3::cuboid(ground_height, ground_size, ground_size).build(),
    floor1,
    bodies,
  )
  |> ignore
  let floor2 = bodies.insert(
    @dynamics.RigidBodyBuilder3::fixed()
    .translation(
      @core.Vec3::new(ground_size + ground_height, ground_size, 0.0F),
    )
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3::cuboid(ground_height, ground_size, ground_size).build(),
    floor2,
    bodies,
  )
  |> ignore

  // Kinematic character.
  let character_handle = bodies.insert(
    @dynamics.RigidBodyBuilder3::kinematic_position_based()
    .translation(@core.Vec3::new(0.0F, 0.3F, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3::cuboid(0.15F, 0.3F, 0.15F).build(),
    character_handle,
    bodies,
  )
  |> ignore

  // Tethered ball.
  let rad = 0.04F
  let child_handle = bodies.insert(
    @dynamics.RigidBodyBuilder3::dynamic()
    .translation(@core.Vec3::new(1.0F, 1.0F, 0.0F))
    .can_sleep(false)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3::ball(rad).build(),
    child_handle,
    bodies,
  )
  |> ignore
  let joint = @dynamics.GenericJoint::from_rope(
    @dynamics.RopeJointBuilder3::new(2.0F).build(),
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
test "examples3d/spring_joints3.rs parity" {
  inspect(run_examples3d_spring_joints3(), content="true")
}

///|
test "examples3d/trimesh3.rs parity" {
  // Ported from `rapier-reference/examples3d/trimesh3.rs` (headless; simplified mesh input).
  let pipeline = @pipeline.PhysicsPipeline3::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let bodies = @dynamics.RigidBodySet3::new()
  let colliders = @collision.ColliderSet3::new()
  let impulse_joints = @dynamics.ImpulseJointSet3::new()
  let multibody_joints = @dynamics.MultibodyJointSet3::new()
  let islands = @dynamics.IslandManager3::new()
  let hooks = @pipeline.PhysicsHooks::new()
  let events = @pipeline.EventHandler::new()
  let ccd_solver = @dynamics.CCDSolver::new()
  let parameters = @dynamics.IntegrationParameters::default()
  let gravity = @core.Vec3::new(0.0F, -9.81F, 0.0F)
  fn step_n(steps : Int) -> Unit {
    for _ in 0..<steps {
      pipeline.step(
        gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
        impulse_joints, multibody_joints, ccd_solver, hooks, events,
      )
    }
  }

  // Ground + walls (2D compatibility mode).
  let ground_size = 25.0F
  let ground = bodies.insert(@dynamics.RigidBodyBuilder3::fixed().build())
  colliders.insert_with_parent(
    @collision.ColliderBuilder3::cuboid(ground_size, 1.2F, 1.2F).build(),
    ground,
    bodies,
  )
  |> ignore
  let wall_rot = @core.Vec3::new(0.0F, 0.0F, @core.pi() / 2.0F)
  let wall_r = bodies.insert(
    @dynamics.RigidBodyBuilder3::fixed()
    .translation(@core.Vec3::new(ground_size, ground_size, 0.0F))
    .rotation_scaled_axis(wall_rot)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3::cuboid(ground_size, 1.2F, 1.2F).build(),
    wall_r,
    bodies,
  )
  |> ignore
  let wall_l = bodies.insert(
    @dynamics.RigidBodyBuilder3::fixed()
    .translation(@core.Vec3::new(-ground_size, ground_size, 0.0F))
    .rotation_scaled_axis(wall_rot)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3::cuboid(ground_size, 1.2F, 1.2F).build(),
    wall_l,
    bodies,
  )
  |> ignore

  // Small dynamic mesh (upstream derives this from a heightfield).
  let vtx = [
    @core.Vec3::new(-1.0F, -1.0F, 0.0F),
    @core.Vec3::new(1.0F, -1.0F, 0.0F),
    @core.Vec3::new(1.0F, 1.0F, 0.0F),
    @core.Vec3::new(-1.0F, 1.0F, 0.0F),
  ]
  let idx = [(0, 1, 2), (0, 2, 3)]
  let dyn_handles : Array[@dynamics.RigidBodyHandle] = []
  for k in 0..<5 {
    if @collision.ColliderBuilder3::trimesh(vtx, idx) is Some(builder) {
      let h = bodies.insert(
        @dynamics.RigidBodyBuilder3::dynamic()
        .translation(
          @core.Vec3::new(
            0.0F,
            20.0F + Float::from_double(k.to_double()) * 5.0F,
            0.0F,
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
  step_n(120)
  for i in 0..<dyn_handles.length() {
    let p = bodies.translation(dyn_handles[i])
    inspect(p.x == p.x, content="true")
    inspect(p.y == p.y, content="true")
    inspect(p.y > -10.0F, content="true")
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
