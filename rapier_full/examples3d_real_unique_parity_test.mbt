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
/// Real dim3 parity coverage for examples that were still only exercised by the legacy
/// compatibility-oriented `examples3d_parity_test.mbt` wrapper stack.

///|
fn step_world3d_n(
  pipeline : @pipeline.PhysicsPipeline3DReal,
  gravity : @core.Vec3,
  parameters : @dynamics.IntegrationParameters,
  islands : @dynamics.IslandManager3D,
  broad_phase : @collision.BroadPhase3D,
  narrow_phase : @collision.NarrowPhase3D,
  bodies : @dynamics.RigidBodySet3D,
  colliders : @collision.ColliderSet3D,
  steps : Int,
) -> Unit {
  for _ in 0..<steps {
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    )
  }
}

///|
fn step_world3d_with_joints_n(
  pipeline : @pipeline.PhysicsPipeline3DReal,
  gravity : @core.Vec3,
  parameters : @dynamics.IntegrationParameters,
  islands : @dynamics.IslandManager3D,
  broad_phase : @collision.BroadPhase3D,
  narrow_phase : @collision.NarrowPhase3D,
  bodies : @dynamics.RigidBodySet3D,
  colliders : @collision.ColliderSet3D,
  joints : @dynamics.JointSet3DReal,
  steps : Int,
) -> Unit {
  for _ in 0..<steps {
    pipeline.step_with_joints(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
      joints,
    )
  }
}

///|
fn step_world3d_with_rope_joints_n(
  pipeline : @pipeline.PhysicsPipeline3DReal,
  gravity : @core.Vec3,
  parameters : @dynamics.IntegrationParameters,
  islands : @dynamics.IslandManager3D,
  broad_phase : @collision.BroadPhase3D,
  narrow_phase : @collision.NarrowPhase3D,
  bodies : @dynamics.RigidBodySet3D,
  colliders : @collision.ColliderSet3D,
  rope_joints : @dynamics.RopeJointSet3DReal,
  steps : Int,
) -> Unit {
  for _ in 0..<steps {
    pipeline.step_with_rope_joints(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
      rope_joints,
    )
  }
}

///|
fn run_examples3d_real_compound3() -> Bool {
  let pipeline = @pipeline.PhysicsPipeline3DReal::new()
  let broad_phase = @collision.BroadPhase3D::new()
  let narrow_phase = @collision.NarrowPhase3D::new()
  let bodies = @dynamics.RigidBodySet3D::new()
  let colliders = @collision.ColliderSet3D::new()
  let islands = @dynamics.IslandManager3D::new()
  let gravity = @core.Vec3::new(0.0F, -9.81F, 0.0F)
  let parameters = @dynamics.IntegrationParameters::default()

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
    offset = offset - 0.05F * rad * (Float::from_int(num) - 1.0F)
  }

  step_world3d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    2,
  )
  for i in 0..<handles.length() {
    if bodies.get(handles[i]) is Some(body) {
      let p = body.translation()
      if !(p.x == p.x) || !(p.y == p.y) || !(p.z == p.z) {
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
fn build_dynamic_trimesh_floor() -> @collision.Collider3D {
  let nrows = 201
  let ncols = 2
  let scale_x = 100.0F
  let scale_y = 1.0F
  let scale_z = 4.0F
  let heights : Array[@core.Real] = []
  for i in 0..<nrows {
    let t = Float::from_int(i) / 20.0F
    let h = Float::from_double(@math.sin(t.to_double())) * 2.0F / scale_y
    heights.push(h)
    heights.push(h)
  }
  @collision.ColliderBuilder3D::heightfield(
    heights,
    nrows,
    ncols,
    @core.Vec3::new(scale_x, scale_y, scale_z),
  )
  .unwrap()
  .build()
}

///|
fn run_examples3d_real_dynamic_trimesh3(use_decomposition : Bool) -> Bool {
  let pipeline = @pipeline.PhysicsPipeline3DReal::new()
  let broad_phase = @collision.BroadPhase3D::new()
  let narrow_phase = @collision.NarrowPhase3D::new()
  let bodies = @dynamics.RigidBodySet3D::new()
  let colliders = @collision.ColliderSet3D::new()
  let islands = @dynamics.IslandManager3D::new()
  let gravity = @core.Vec3::new(0.0F, -9.81F, 0.0F)
  let parameters = @dynamics.IntegrationParameters::default()

  colliders.insert(build_dynamic_trimesh_floor()) |> ignore

  let vtx = [
    @core.Vec3::new(-1.0F, -1.0F, 0.0F),
    @core.Vec3::new(1.0F, -1.0F, 0.0F),
    @core.Vec3::new(1.0F, 1.0F, 0.0F),
    @core.Vec3::new(-1.0F, 1.0F, 0.0F),
  ]
  let idx = [(0, 1, 2), (0, 2, 3)]
  let handles : Array[@dynamics.RigidBodyHandle] = []
  for k in 0..<10 {
    let x = Float::from_int(k) * 2.5F - 10.0F
    let rb = bodies.insert(
      @dynamics.RigidBodyBuilder3D::dynamic()
      .translation(@core.Vec3::new(x, 8.0F, 0.0F))
      .build(),
    )
    if use_decomposition {
      colliders.insert_with_parent(
        @collision.ColliderBuilder3D::cuboid(0.5F, 0.2F, 0.2F).build(),
        rb,
        bodies,
      )
      |> ignore
      colliders.insert_with_parent(
        @collision.ColliderBuilder3D::cuboid(0.2F, 0.5F, 0.2F)
        .translation(@core.Vec3::new(0.6F, 0.0F, 0.0F))
        .build(),
        rb,
        bodies,
      )
      |> ignore
    } else {
      let builder = @collision.ColliderBuilder3D::trimesh(vtx, idx).unwrap()
      colliders.insert_with_parent(
        builder.contact_skin(0.1F).build(),
        rb,
        bodies,
      )
      |> ignore
    }
    handles.push(rb)
  }

  step_world3d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    120,
  )
  for i in 0..<handles.length() {
    if bodies.get(handles[i]) is Some(body) {
      let p = body.translation()
      if !(p.x == p.x) || !(p.y == p.y) || !(p.z == p.z) {
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
fn run_examples3d_real_joint_motor_position3() -> Bool {
  let bodies = @dynamics.RigidBodySet3D::new()
  let colliders = @collision.ColliderSet3D::new()
  let joints = @dynamics.JointSet3DReal::new()
  let pipeline = @pipeline.PhysicsPipeline3DReal::new()
  let broad_phase = @collision.BroadPhase3D::new()
  let narrow_phase = @collision.NarrowPhase3D::new()
  let islands = @dynamics.IslandManager3D::new()
  let gravity = @core.Vec3::zero()
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )

  let ground = bodies.insert(@dynamics.RigidBodyBuilder3D::fixed().build())
  let axis_z = @core.Vec3::new(0.0F, 0.0F, 1.0F)
  let revolute_basis = @core.rotation_between(
    @core.Vec3::new(0.0F, 0.0F, 1.0F),
    axis_z,
  )
  let revolute_locked = @dynamics.JointAxesMask3DReal::lin_axes()
    .or(@dynamics.JointAxesMask3DReal::ang_x())
    .or(@dynamics.JointAxesMask3DReal::ang_y())
  let targets : Array[(@dynamics.RigidBodyHandle, @core.Real)] = []
  for num in 0..<9 {
    let x_pos = -6.0F + 1.5F * Float::from_int(num)
    let handle = bodies.insert(
      @dynamics.RigidBodyBuilder3D::dynamic()
      .translation(@core.Vec3::new(x_pos, 2.0F, 0.0F))
      .can_sleep(false)
      .build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder3D::cuboid(0.1F, 0.5F, 0.1F).build(),
      handle,
      bodies,
    )
    |> ignore
    let target = -@core.pi() + @core.pi() / 4.0F * Float::from_int(num)
    let joint = @dynamics.GenericJoint3DRealBuilder::new(revolute_locked)
      .local_anchor1(@core.Vec3::new(x_pos, 1.5F, 0.0F))
      .local_anchor2(@core.Vec3::new(0.0F, -0.5F, 0.0F))
      .local_basis1(revolute_basis)
      .local_basis2(revolute_basis)
      .motor_position(@dynamics.JointAxis3DReal::AngZ, target, 1000.0F, 150.0F)
      .build()
    joints.insert_generic(ground, handle, joint, true) |> ignore
    targets.push((handle, target))
  }
  let limit_targets : Array[(@dynamics.RigidBodyHandle, @core.Real, @core.Real)] = []
  for num in 0..<8 {
    let x_pos = -6.0F + 1.5F * Float::from_int(num)
    let handle = bodies.insert(
      @dynamics.RigidBodyBuilder3D::dynamic()
      .translation(@core.Vec3::new(x_pos, 4.5F, 0.0F))
      .rotation_scaled_axis(@core.Vec3::new(0.0F, 0.0F, @core.pi()))
      .can_sleep(false)
      .build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder3D::cuboid(0.1F, 0.5F, 0.1F).build(),
      handle,
      bodies,
    )
    |> ignore
    let min_angle = -@core.pi()
    let max_angle = -@core.pi() + @core.pi() / 4.0F * Float::from_int(num)
    let joint = @dynamics.GenericJoint3DRealBuilder::new(revolute_locked)
      .local_anchor1(@core.Vec3::new(x_pos, 5.0F, 0.0F))
      .local_anchor2(@core.Vec3::new(0.0F, -0.5F, 0.0F))
      .local_basis1(revolute_basis)
      .local_basis2(revolute_basis)
      .motor_velocity(@dynamics.JointAxis3DReal::AngZ, 1.5F, 30.0F)
      .motor_max_force(@dynamics.JointAxis3DReal::AngZ, 100.0F)
      .limits(@dynamics.JointAxis3DReal::AngZ, min_angle, max_angle)
      .build()
    joints.insert_generic(ground, handle, joint, true) |> ignore
    limit_targets.push((handle, min_angle, max_angle))
  }

  step_world3d_with_joints_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    joints, 240,
  )
  let mut moved_targets = 0
  for i in 0..<targets.length() {
    let (handle, _target) = targets[i]
    if bodies.get(handle) is Some(body) {
      let angle = body.rotation().to_scaled_axis().z
      if !(angle == angle) {
        return false
      }
      if @core.abs(angle) > 5.0e-2F {
        moved_targets = moved_targets + 1
      }
    } else {
      return false
    }
  }
  let mut moved_limits = 0
  for i in 0..<limit_targets.length() {
    let (handle, min_a, max_a) = limit_targets[i]
    if bodies.get(handle) is Some(body) {
      let angle = body.rotation().to_scaled_axis().z
      if !(angle == angle) {
        return false
      }
      if angle >= min_a - 2.0e-1F &&
        angle <= max_a + 2.0e-1F &&
        @core.abs(angle) > 5.0e-2F {
        moved_limits = moved_limits + 1
      }
    } else {
      return false
    }
  }
  moved_targets >= 5 && moved_limits >= 4
}

///|
fn run_examples3d_real_inverse_kinematics3() -> Bool {
  let bodies = @dynamics.RigidBodySet3D::new()
  let colliders = @collision.ColliderSet3D::new()
  let multibody_joints = @dynamics.MultibodyJointSet3DReal::new()

  let ground_height = 0.01F
  let floor_handle = bodies.insert(
    @dynamics.RigidBodyBuilder3D::fixed()
    .translation(@core.Vec3::new(0.0F, -ground_height, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(1.0F, ground_height, 1.0F).build(),
    floor_handle,
    bodies,
  )
  |> ignore

  let num_segments = 10
  let size = 1.0F / Float::from_int(num_segments)
  let axis_z = @core.Vec3::new(0.0F, 0.0F, 1.0F)
  let mut last_body = bodies.insert(
    @dynamics.RigidBodyBuilder3D::fixed().build(),
  )
  let mut last_link = @dynamics.MultibodyJointHandle::invalid()
  for i in 0..<num_segments {
    let new_body = bodies.insert(
      @dynamics.RigidBodyBuilder3D::dynamic().can_sleep(false).build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder3D::cuboid(
        size / 8.0F,
        size / 2.0F,
        size / 8.0F,
      )
      .density(0.0F)
      .sensor(true)
      .build(),
      new_body,
      bodies,
    )
    |> ignore
    let joint = @dynamics.GenericJoint::from_revolute(
      @dynamics.RevoluteJointBuilder3::new(axis_z)
      .local_anchor1(
        @core.Vec3::new(
          0.0F,
          size / 2.0F * (if i != 0 { 1.0F } else { 0.0F }),
          0.0F,
        ),
      )
      .local_anchor2(@core.Vec3::new(0.0F, -size / 2.0F, 0.0F))
      .build(),
    )
    if multibody_joints.insert(last_body, new_body, joint, true) is Some(handle) {
      last_link = handle
    } else {
      return false
    }
    last_body = new_body
  }

  if multibody_joints.get_mut(last_link) is Some((multibody, link_id)) {
    let displacements = @dynamics.DVector::from_fn(multibody.ndofs(), fn(
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
    err == err && err < 5.0e-2F
  } else {
    false
  }
}

///|
fn run_examples3d_real_keva3() -> Bool {
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

  let ground_height = 0.1F
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder3D::fixed()
    .translation(@core.Vec3::new(0.0F, -ground_height, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(50.0F, ground_height, 50.0F).build(),
    ground,
    bodies,
  )
  |> ignore

  let half_x = 1.2F
  let half_y = 0.1F
  let handles : Array[@dynamics.RigidBodyHandle] = []
  for layer in 0..<18 {
    let y = half_y * 2.0F * Float::from_int(layer) + 0.2F
    let along_z = layer % 2 == 1
    for i in 0..<3 {
      let offset = (Float::from_int(i) - 1.0F) * 0.9F
      let translation = if along_z {
        @core.Vec3::new(0.0F, y, offset)
      } else {
        @core.Vec3::new(offset, y, 0.0F)
      }
      let rotation = if along_z {
        @core.Vec3::new(0.0F, @core.pi() / 2.0F, 0.0F)
      } else {
        @core.Vec3::zero()
      }
      let rb = bodies.insert(
        @dynamics.RigidBodyBuilder3D::dynamic()
        .translation(translation)
        .rotation_scaled_axis(rotation)
        .can_sleep(false)
        .build(),
      )
      colliders.insert_with_parent(
        @collision.ColliderBuilder3D::cuboid(half_x, half_y, half_y).build(),
        rb,
        bodies,
      )
      |> ignore
      handles.push(rb)
    }
  }

  step_world3d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    360,
  )
  let mut min_y = 1.0e30F
  let mut max_abs_z = 0.0F
  for i in 0..<handles.length() {
    if bodies.get(handles[i]) is Some(body) {
      let p = body.translation()
      if !(p.x == p.x) || !(p.y == p.y) || !(p.z == p.z) {
        return false
      }
      if p.y < min_y {
        min_y = p.y
      }
      let abs_z = @core.abs(p.z)
      if abs_z > max_abs_z {
        max_abs_z = abs_z
      }
    } else {
      return false
    }
  }
  min_y > -1000.0F && max_abs_z > 0.2F
}

///|
fn run_examples3d_real_joints3() -> Bool {
  let pipeline = @pipeline.PhysicsPipeline3DReal::new()
  let broad_phase = @collision.BroadPhase3D::new()
  let narrow_phase = @collision.NarrowPhase3D::new()
  let bodies = @dynamics.RigidBodySet3D::new()
  let colliders = @collision.ColliderSet3D::new()
  let islands = @dynamics.IslandManager3D::new()
  let joints = @dynamics.JointSet3DReal::new()
  let gravity = @core.Vec3::new(0.0F, -9.81F, 0.0F)
  let parameters = @dynamics.IntegrationParameters::default()

  let rad = 0.4F
  let num_i = 4
  let num_k = 4
  let shift = 1.0F
  let axis_z = @core.Vec3::new(0.0F, 0.0F, 1.0F)
  let handles : Array[@dynamics.RigidBodyHandle] = []
  for k in 0..<num_k {
    for i in 0..<num_i {
      let child = bodies.insert(
        (if i == 0 && k == 0 {
          @dynamics.RigidBodyBuilder3D::fixed()
        } else {
          @dynamics.RigidBodyBuilder3D::dynamic().can_sleep(false)
        })
        .translation(
          @core.Vec3::new(
            Float::from_int(k) * shift,
            -Float::from_int(i) * shift,
            0.0F,
          ),
        )
        .build(),
      )
      colliders.insert_with_parent(
        @collision.ColliderBuilder3D::ball(rad).build(),
        child,
        bodies,
      )
      |> ignore

      if i > 0 {
        let parent = handles[handles.length() - 1]
        let joint = @dynamics.RevoluteJoint3DRealBuilder::new(axis_z)
          .local_anchor2(@core.Vec3::new(0.0F, shift, 0.0F))
          .build()
        joints.insert_generic(parent, child, joint, true) |> ignore
      }
      if k > 0 {
        let parent = handles[handles.length() - num_i]
        let joint = @dynamics.RevoluteJoint3DRealBuilder::new(axis_z)
          .local_anchor2(@core.Vec3::new(-shift, 0.0F, 0.0F))
          .build()
        joints.insert_generic(parent, child, joint, true) |> ignore
      }
      handles.push(child)
    }
  }

  step_world3d_with_joints_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    joints, 240,
  )
  for i in 0..<handles.length() {
    if bodies.get(handles[i]) is Some(body) {
      let p = body.translation()
      if !(p.x == p.x) || !(p.y == p.y) || !(p.z == p.z) {
        return false
      }
    } else {
      return false
    }
  }
  true
}

///|
fn run_examples3d_real_newton_cradle3() -> Bool {
  let pipeline = @pipeline.PhysicsPipeline3DReal::new()
  let broad_phase = @collision.BroadPhase3D::new()
  let narrow_phase = @collision.NarrowPhase3D::new()
  let bodies = @dynamics.RigidBodySet3D::new()
  let colliders = @collision.ColliderSet3D::new()
  let islands = @dynamics.IslandManager3D::new()
  let joints = @dynamics.JointSet3DReal::new()
  let gravity = @core.Vec3::new(0.0F, -9.81F, 0.0F)
  let parameters = @dynamics.IntegrationParameters::default()

  let radius = 0.5F
  let length = 10.0F * radius
  let n = 5
  let axis_z = @core.Vec3::new(0.0F, 0.0F, 1.0F)
  let balls : Array[@dynamics.RigidBodyHandle] = []
  for i in 0..<n {
    let x = Float::from_int(i) * 2.2F * radius
    let ball_pos = @core.Vec3::new(x, 0.0F, 0.0F)
    let attach = @core.Vec3::new(0.0F, length, 0.0F)
    let vel = if i >= n - 1 {
      @core.Vec3::new(7.0F, 0.0F, 0.0F)
    } else {
      @core.Vec3::zero()
    }
    let ground = bodies.insert(
      @dynamics.RigidBodyBuilder3D::fixed()
      .translation(ball_pos.add(attach))
      .build(),
    )
    let body = bodies.insert(
      @dynamics.RigidBodyBuilder3D::dynamic()
      .translation(ball_pos)
      .linvel(vel)
      .can_sleep(false)
      .build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder3D::ball(radius).restitution(1.0F).build(),
      body,
      bodies,
    )
    |> ignore
    let joint = @dynamics.RevoluteJoint3DRealBuilder::new(axis_z)
      .local_anchor1(@core.Vec3::zero())
      .local_anchor2(attach)
      .build()
    joints.insert_generic(ground, body, joint, true) |> ignore
    balls.push(body)
  }

  step_world3d_with_joints_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    joints, 480,
  )
  let mut moved = false
  for i in 0..<balls.length() {
    if bodies.get(balls[i]) is Some(body) {
      let p = body.translation()
      if !(p.x == p.x) || !(p.y == p.y) || !(p.z == p.z) {
        return false
      }
      if @core.abs(p.x) > 0.2F {
        moved = true
      }
    } else {
      return false
    }
  }
  moved
}

///|
fn run_examples3d_real_spring_joints3() -> Bool {
  let pipeline = @pipeline.PhysicsPipeline3DReal::new()
  let broad_phase = @collision.BroadPhase3D::new()
  let narrow_phase = @collision.NarrowPhase3D::new()
  let bodies = @dynamics.RigidBodySet3D::new()
  let colliders = @collision.ColliderSet3D::new()
  let islands = @dynamics.IslandManager3D::new()
  let joints = @dynamics.JointSet3DReal::new()
  let gravity = @core.Vec3::new(0.0F, -9.81F, 0.0F)
  let parameters = @dynamics.IntegrationParameters::default()

  let ground = bodies.insert(@dynamics.RigidBodyBuilder3D::fixed().build())
  let num = 12
  let radius = 0.5F
  let balls : Array[@dynamics.RigidBodyHandle] = []
  for i in 0..<=num {
    let x = -6.0F + Float::from_int(i)
    let ball_pos = @core.Vec3::new(x, 4.5F, 0.0F)
    let handle = bodies.insert(
      @dynamics.RigidBodyBuilder3D::dynamic()
      .translation(ball_pos)
      .can_sleep(false)
      .build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder3D::ball(radius).build(),
      handle,
      bodies,
    )
    |> ignore
    joints.insert_spring(
      ground,
      handle,
      ball_pos.sub(@core.Vec3::new(0.0F, 3.0F, 0.0F)),
      @core.Vec3::zero(),
      3.0F,
      40.0F,
      4.0F,
      true,
    )
    balls.push(handle)
    let box = bodies.insert(
      @dynamics.RigidBodyBuilder3D::dynamic()
      .translation(ball_pos.add(@core.Vec3::new(0.0F, 5.0F, 0.0F)))
      .build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder3D::cuboid(radius, radius, radius)
      .density(100.0F)
      .build(),
      box,
      bodies,
    )
    |> ignore
  }

  step_world3d_with_joints_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    joints, 360,
  )
  for i in 0..<balls.length() {
    if bodies.get(balls[i]) is Some(body) {
      let p = body.translation()
      if !(p.x == p.x) || !(p.y == p.y) || !(p.z == p.z) {
        return false
      }
    } else {
      return false
    }
  }
  true
}

///|
fn run_examples3d_real_rope_joints3() -> Bool {
  let pipeline = @pipeline.PhysicsPipeline3DReal::new()
  let broad_phase = @collision.BroadPhase3D::new()
  let narrow_phase = @collision.NarrowPhase3D::new()
  let bodies = @dynamics.RigidBodySet3D::new()
  let colliders = @collision.ColliderSet3D::new()
  let islands = @dynamics.IslandManager3D::new()
  let rope_joints = @dynamics.RopeJointSet3DReal::new()
  let parameters = @dynamics.IntegrationParameters::default()
  let gravity = @core.Vec3::new(0.0F, -9.81F, 0.0F)
  let inv_dt = parameters.inv_dt()
  let dt = if inv_dt == 0.0F { 0.0F } else { 1.0F / inv_dt }

  let ground_size = 0.75F
  let ground_height = 0.1F
  let floor0 = bodies.insert(
    @dynamics.RigidBodyBuilder3D::fixed()
    .translation(@core.Vec3::new(0.0F, -ground_height, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(
      ground_size, ground_height, ground_size,
    ).build(),
    floor0,
    bodies,
  )
  |> ignore
  let floor1 = bodies.insert(
    @dynamics.RigidBodyBuilder3D::fixed()
    .translation(
      @core.Vec3::new(-ground_size - ground_height, ground_size, 0.0F),
    )
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(
      ground_height, ground_size, ground_size,
    ).build(),
    floor1,
    bodies,
  )
  |> ignore
  let floor2 = bodies.insert(
    @dynamics.RigidBodyBuilder3D::fixed()
    .translation(
      @core.Vec3::new(ground_size + ground_height, ground_size, 0.0F),
    )
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(
      ground_height, ground_size, ground_size,
    ).build(),
    floor2,
    bodies,
  )
  |> ignore

  let character = bodies.insert(
    @dynamics.RigidBodyBuilder3D::kinematic_position_based()
    .translation(@core.Vec3::new(0.0F, 0.3F, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(0.15F, 0.3F, 0.15F).build(),
    character,
    bodies,
  )
  |> ignore

  let rad = 0.04F
  let child = bodies.insert(
    @dynamics.RigidBodyBuilder3D::dynamic()
    .translation(@core.Vec3::new(1.0F, 1.0F, 0.0F))
    .can_sleep(false)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::ball(rad).build(),
    child,
    bodies,
  )
  |> ignore
  rope_joints.insert(character, child, 2.0F)

  let mut time = 0.0F
  for _ in 0..<600 {
    time = time + dt
    if bodies.get_mut(character) is Some(ch) {
      ch.set_next_kinematic_translation(
        @core.Vec3::new(
          Float::from_double(@math.sin(time.to_double())) * 2.0F,
          0.3F,
          0.0F,
        ),
      )
    } else {
      return false
    }
    step_world3d_with_rope_joints_n(
      pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies,
      colliders, rope_joints, 1,
    )
  }
  if bodies.get(character) is Some(ch) && bodies.get(child) is Some(ball) {
    let d = ball.translation().sub(ch.translation()).length()
    d == d && d <= 2.05F
  } else {
    false
  }
}

///|
fn run_examples3d_real_trimesh3() -> Bool {
  let pipeline = @pipeline.PhysicsPipeline3DReal::new()
  let broad_phase = @collision.BroadPhase3D::new()
  let narrow_phase = @collision.NarrowPhase3D::new()
  let bodies = @dynamics.RigidBodySet3D::new()
  let colliders = @collision.ColliderSet3D::new()
  let islands = @dynamics.IslandManager3D::new()
  let parameters = @dynamics.IntegrationParameters::default()
  let gravity = @core.Vec3::new(0.0F, -9.81F, 0.0F)

  let ground_size = 25.0F
  let ground = bodies.insert(@dynamics.RigidBodyBuilder3D::fixed().build())
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(ground_size, 1.2F, 1.2F).build(),
    ground,
    bodies,
  )
  |> ignore
  let wall_rot = @core.rotation_from_scaled_axis(
    @core.Vec3::new(0.0F, 0.0F, @core.pi() / 2.0F),
  )
  let wall_r = bodies.insert(
    @dynamics.RigidBodyBuilder3D::fixed()
    .translation(@core.Vec3::new(ground_size, ground_size, 0.0F))
    .rotation(wall_rot)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(ground_size, 1.2F, 1.2F).build(),
    wall_r,
    bodies,
  )
  |> ignore
  let wall_l = bodies.insert(
    @dynamics.RigidBodyBuilder3D::fixed()
    .translation(@core.Vec3::new(-ground_size, ground_size, 0.0F))
    .rotation(wall_rot)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(ground_size, 1.2F, 1.2F).build(),
    wall_l,
    bodies,
  )
  |> ignore

  let vtx = [
    @core.Vec3::new(-1.0F, -1.0F, 0.0F),
    @core.Vec3::new(1.0F, -1.0F, 0.0F),
    @core.Vec3::new(1.0F, 1.0F, 0.0F),
    @core.Vec3::new(-1.0F, 1.0F, 0.0F),
  ]
  let idx = [(0, 1, 2), (0, 2, 3)]
  let dyn_handles : Array[@dynamics.RigidBodyHandle] = []
  let builder = @collision.ColliderBuilder3D::trimesh(vtx, idx).unwrap()
  for k in 0..<5 {
    let handle = bodies.insert(
      @dynamics.RigidBodyBuilder3D::dynamic()
      .translation(
        @core.Vec3::new(0.0F, 20.0F + Float::from_int(k) * 5.0F, 0.0F),
      )
      .build(),
    )
    colliders.insert_with_parent(
      builder.contact_skin(0.2F).build(),
      handle,
      bodies,
    )
    |> ignore
    dyn_handles.push(handle)
  }

  step_world3d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    120,
  )
  for i in 0..<dyn_handles.length() {
    if bodies.get(dyn_handles[i]) is Some(body) {
      let p = body.translation()
      if !(p.x == p.x) || !(p.y == p.y) || !(p.z == p.z) {
        return false
      }
      if !(p.y > -10.0F) {
        return false
      }
    } else {
      return false
    }
  }
  true
}

///|
fn run_examples3d_real_urdf3() -> Bool {
  let pipeline = @pipeline.PhysicsPipeline3DReal::new()
  let broad_phase = @collision.BroadPhase3D::new()
  let narrow_phase = @collision.NarrowPhase3D::new()
  let bodies = @dynamics.RigidBodySet3D::new()
  let colliders = @collision.ColliderSet3D::new()
  let islands = @dynamics.IslandManager3D::new()
  let joints = @dynamics.JointSet3DReal::new()
  let gravity = @core.Vec3::new(0.0F, -9.81F, 0.0F)
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )

  colliders.insert(
    @collision.ColliderBuilder3D::cuboid(50.0F, 0.05F, 0.05F)
    .translation(@core.Vec3::new(0.0F, -0.05F, 0.0F))
    .build(),
  )
  |> ignore

  let base = bodies.insert(
    @dynamics.RigidBodyBuilder3D::fixed()
    .translation(@core.Vec3::new(0.0F, 2.0F, 0.0F))
    .build(),
  )
  let axis_z = @core.Vec3::new(0.0F, 0.0F, 1.0F)
  let mut parent = base
  let links : Array[@dynamics.RigidBodyHandle] = []
  for i in 0..<3 {
    let link = bodies.insert(
      @dynamics.RigidBodyBuilder3D::dynamic()
      .translation(
        @core.Vec3::new(Float::from_int(i) * 2.0F + 1.0F, 2.0F, 0.0F),
      )
      .can_sleep(false)
      .build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder3D::cuboid(0.6F, 0.1F, 0.1F).build(),
      link,
      bodies,
    )
    |> ignore
    let joint = @dynamics.RevoluteJoint3DRealBuilder::new(axis_z)
      .local_anchor1(@core.Vec3::new(1.0F, 0.0F, 0.0F))
      .local_anchor2(@core.Vec3::new(-1.0F, 0.0F, 0.0F))
      .limits(-0.5F, 0.5F)
      .build()
    joints.insert_generic(parent, link, joint, true) |> ignore
    parent = link
    links.push(link)
  }

  step_world3d_with_joints_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    joints, 240,
  )
  for i in 0..<links.length() {
    if bodies.get(links[i]) is Some(body) {
      let p = body.translation()
      if !(p.x == p.x) || !(p.y == p.y) || !(p.z == p.z) {
        return false
      }
    } else {
      return false
    }
  }
  true
}

///|
fn run_examples3d_real_harness_capsules3() -> Bool {
  let pipeline = @pipeline.PhysicsPipeline3DReal::new()
  let broad_phase = @collision.BroadPhase3D::new()
  let narrow_phase = @collision.NarrowPhase3D::new()
  let bodies = @dynamics.RigidBodySet3D::new()
  let colliders = @collision.ColliderSet3D::new()
  let islands = @dynamics.IslandManager3D::new()
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )
  let gravity = @core.Vec3::new(0.0F, -9.81F, 0.0F)
  let ground_height = 0.1F
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder3D::fixed()
    .translation(@core.Vec3::new(0.0F, -ground_height, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(200.1F, ground_height, 200.1F).build(),
    ground,
    bodies,
  )
  |> ignore
  let num = 8
  let height = 10
  let rad = 0.25F
  let shift = rad * 3.0F
  let centerx = shift * Float::from_int(num / 2)
  let handles : Array[@dynamics.RigidBodyHandle] = []
  for j in 0..<height {
    for i in 0..<num {
      let x = Float::from_int(i) * shift - centerx
      let y = Float::from_int(j) * (rad * 4.0F) + 3.0F
      let body = bodies.insert(
        @dynamics.RigidBodyBuilder3D::dynamic()
        .translation(@core.Vec3::new(x, y, 0.0F))
        .build(),
      )
      colliders.insert_with_parent(
        @collision.ColliderBuilder3D::capsule_y(rad, rad).build(),
        body,
        bodies,
      )
      |> ignore
      handles.push(body)
    }
  }

  step_world3d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    120,
  )
  for i in 0..<handles.length() {
    if bodies.get(handles[i]) is Some(body) {
      let p = body.translation()
      if !(p.x == p.x) || !(p.y == p.y) || !(p.z == p.z) {
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
fn run_examples3d_real_debug_deserialize3() -> Bool {
  let pipeline = @pipeline.PhysicsPipeline3DReal::new()
  let broad_phase = @collision.BroadPhase3D::new()
  let narrow_phase = @collision.NarrowPhase3D::new()
  let bodies = @dynamics.RigidBodySet3D::new()
  let colliders = @collision.ColliderSet3D::new()
  let islands = @dynamics.IslandManager3D::new()
  let gravity = @core.Vec3::new(0.0F, -9.81F, 0.0F)
  let parameters = @dynamics.IntegrationParameters::default()

  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder3D::fixed()
    .translation(@core.Vec3::new(0.0F, -1.0F, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(20.0F, 1.0F, 20.0F).build(),
    ground,
    bodies,
  )
  |> ignore
  let ball = bodies.insert(
    @dynamics.RigidBodyBuilder3D::dynamic()
    .translation(@core.Vec3::new(0.0F, 4.0F, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::ball(0.5F).build(),
    ball,
    bodies,
  )
  |> ignore

  pipeline.step(
    gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
  )
  true
}

///|
test "examples3d/compound3.rs parity (3DReal headless extra)" {
  inspect(run_examples3d_real_compound3(), content="true")
}

///|
test "examples3d/debug_deserialize3.rs parity (3DReal headless)" {
  inspect(run_examples3d_real_debug_deserialize3(), content="true")
}

///|
test "examples3d/dynamic_trimesh3.rs parity (3DReal headless)" {
  inspect(run_examples3d_real_dynamic_trimesh3(false), content="true")
}

///|
test "examples3d/harness_capsules3.rs parity (3DReal headless)" {
  inspect(run_examples3d_real_harness_capsules3(), content="true")
}

///|
test "examples3d/inverse_kinematics3.rs parity (3DReal headless)" {
  inspect(run_examples3d_real_inverse_kinematics3(), content="true")
}

///|
test "examples3d/joint_motor_position3.rs parity (3DReal headless)" {
  inspect(run_examples3d_real_joint_motor_position3(), content="true")
}

///|
test "examples3d/joints3.rs parity (3DReal headless)" {
  inspect(run_examples3d_real_joints3(), content="true")
}

///|
test "examples3d/keva3.rs parity (3DReal headless)" {
  inspect(run_examples3d_real_keva3(), content="true")
}

///|
test "examples3d/newton_cradle3.rs parity (3DReal headless)" {
  inspect(run_examples3d_real_newton_cradle3(), content="true")
}

///|
test "examples3d/rope_joints3.rs parity (3DReal headless)" {
  inspect(run_examples3d_real_rope_joints3(), content="true")
}

///|
test "examples3d/spring_joints3.rs parity (3DReal headless)" {
  inspect(run_examples3d_real_spring_joints3(), content="true")
}

///|
test "examples3d/trimesh3.rs parity (3DReal headless)" {
  inspect(run_examples3d_real_trimesh3(), content="true")
}

///|
test "examples3d/urdf3.rs parity (3DReal headless)" {
  inspect(run_examples3d_real_urdf3(), content="true")
}
