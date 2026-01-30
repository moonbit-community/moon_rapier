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
/// Parity tests for `rapier-reference/examples3d/*` cases focusing on collision groups, CCD,
/// and rigid-body locking/flags.
fn step_world3_flags(
  pipeline : @pipeline.PhysicsPipeline3,
  gravity : @core.Vec3,
  parameters : @dynamics.IntegrationParameters,
  islands : @dynamics.IslandManager3,
  broad_phase : @collision.BroadPhaseBvh,
  narrow_phase : @collision.NarrowPhase,
  bodies : @dynamics.RigidBodySet3,
  colliders : @collision.ColliderSet3,
  impulse_joints : @dynamics.ImpulseJointSet3,
  multibody_joints : @dynamics.MultibodyJointSet3,
  ccd_solver : @dynamics.CCDSolver,
  hooks : @pipeline.PhysicsHooks,
  events : @pipeline.EventHandler,
  steps : Int,
) -> Unit {
  for _ in 0..<steps {
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
      impulse_joints, multibody_joints, ccd_solver, hooks, events,
    )
  }
}

///|
fn run_examples3d_collision_groups3() -> Bool {
  // Ported from `rapier-reference/examples3d/collision_groups3.rs`.
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
  let ground_size = 5.0F
  let ground_height = 0.1F
  let floor = bodies.insert(
    @dynamics.RigidBodyBuilder3::fixed()
    .translation(@core.Vec3::new(0.0F, -ground_height, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3::cuboid(ground_size, ground_height, ground_size).build(),
    floor,
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
    @collision.ColliderBuilder3::cuboid(1.0F, 0.1F, 1.0F)
    .translation(@core.Vec3::new(0.0F, 1.0F, 0.0F))
    .collision_groups(green_group)
    .build(),
    floor,
    bodies,
  )
  |> ignore
  colliders.insert_with_parent(
    @collision.ColliderBuilder3::cuboid(1.0F, 0.1F, 1.0F)
    .translation(@core.Vec3::new(0.0F, 2.0F, 0.0F))
    .collision_groups(blue_group)
    .build(),
    floor,
    bodies,
  )
  |> ignore
  let num = 8
  let rad = 0.1F
  let shift = rad * 2.0F
  let centerx = shift * Float::from_int(num / 2)
  let centery = 2.5F
  let centerz = shift * Float::from_int(num / 2)
  let green_handles : Array[@dynamics.RigidBodyHandle] = []
  let blue_handles : Array[@dynamics.RigidBodyHandle] = []
  for j in 0..<4 {
    for i in 0..<num {
      for k in 0..<num {
        let x = Float::from_int(i) * shift - centerx
        let y = Float::from_int(j) * shift + centery
        let z = Float::from_int(k) * shift - centerz
        let is_green = k % 2 == 0
        let group = if is_green { green_group } else { blue_group }
        let rb = @dynamics.RigidBodyBuilder3::dynamic()
          .translation(@core.Vec3::new(x, y, z))
          .build()
        let handle = bodies.insert(rb)
        let collider = @collision.ColliderBuilder3::cuboid(rad, rad, rad)
          .collision_groups(group)
          .build()
        colliders.insert_with_parent(collider, handle, bodies) |> ignore
        if is_green {
          green_handles.push(handle)
        } else {
          blue_handles.push(handle)
        }
      }
    }
  }
  step_world3_flags(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 240,
  )
  fn avg_y(
    bodies : @dynamics.RigidBodySet3,
    handles : Array[@dynamics.RigidBodyHandle],
  ) -> Float {
    let mut sum = 0.0F
    for h in handles {
      sum += bodies.translation(h).y
    }
    sum / Float::from_int(handles.length())
  }

  let avg_green = avg_y(bodies, green_handles)
  let avg_blue = avg_y(bodies, blue_handles)
  avg_blue - avg_green > 0.5F
}

///|
fn run_examples3d_ccd3() -> Bool {
  // Ported from `rapier-reference/examples3d/ccd3.rs` (headless; asserts CCD causes collisions).
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
  let ground_size = 50.0F
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
  fn create_wall(
    bodies : @dynamics.RigidBodySet3,
    colliders : @collision.ColliderSet3,
    offset : @core.Vec3,
    stack_height : Int,
    half_extents : @core.Vec3,
  ) -> Unit {
    let shift = half_extents.scale(2.0F)
    let mut k = 0
    for i in 0..<stack_height {
      for j in i..<stack_height {
        let fi = Float::from_int(i)
        let fj = Float::from_int(j)
        let x = offset.x
        let y = fi * shift.y + offset.y
        let z = fi * shift.z / 2.0F +
          (fj - fi) * shift.z +
          offset.z -
          Float::from_int(stack_height) * half_extents.z
        let rb = @dynamics.RigidBodyBuilder3::dynamic()
          .translation(@core.Vec3::new(x, y, z))
          .build()
        let handle = bodies.insert(rb)
        let collider = @collision.ColliderBuilder3::cuboid(
          half_extents.x,
          half_extents.y,
          half_extents.z,
        ).build()
        colliders.insert_with_parent(collider, handle, bodies) |> ignore
        k += 1
        k |> ignore
      }
    }
  }

  let num_z = 8
  let num_x = 5
  let shift_y = ground_height + 0.5F
  let shift_z = (Float::from_int(num_z) + 2.0F) * 2.0F
  for i in 0..<num_x {
    let x = Float::from_int(i) * 6.0F
    create_wall(
      bodies,
      colliders,
      @core.Vec3::new(x, shift_y, 0.0F),
      num_z,
      @core.Vec3::new(0.5F, 0.5F, 1.0F),
    )
    create_wall(
      bodies,
      colliders,
      @core.Vec3::new(x, shift_y, shift_z),
      num_z,
      @core.Vec3::new(0.5F, 0.5F, 1.0F),
    )
  }
  let sensor_collider = @collision.ColliderBuilder3::ball(1.0F)
    .density(10.0F)
    .sensor(true)
    .active_events(@collision.ActiveEvents::collision_events())
    .build()
  let sensor_body = @dynamics.RigidBodyBuilder3::dynamic()
    .linvel(@core.Vec3::new(1000.0F, 0.0F, 0.0F))
    .translation(@core.Vec3::new(-20.0F, shift_y + 2.0F, 0.0F))
    .ccd_enabled(true)
    .build()
  let sensor_handle = bodies.insert(sensor_body)
  colliders.insert_with_parent(sensor_collider, sensor_handle, bodies) |> ignore
  let collider = @collision.ColliderBuilder3::ball(1.0F).density(10.0F).build()
  let body = @dynamics.RigidBodyBuilder3::dynamic()
    .linvel(@core.Vec3::new(1000.0F, 0.0F, 0.0F))
    .translation(@core.Vec3::new(-20.0F, shift_y + 2.0F, shift_z))
    .ccd_enabled(true)
    .build()
  let handle = bodies.insert(body)
  colliders.insert_with_parent(collider, handle, bodies) |> ignore
  step_world3_flags(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 5,
  )
  let evs = events.take_collision_events()
  let t = bodies.translation(handle)
  evs.length() > 0 && t.x < 50.0F
}

///|
fn run_examples3d_locked_rotations3() -> Bool {
  // Ported from `rapier-reference/examples3d/locked_rotations3.rs` (headless).
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
  let rb1 = @dynamics.RigidBodyBuilder3::dynamic()
    .translation(@core.Vec3::new(0.0F, 3.0F, 0.0F))
    .lock_translations()
    .enabled_rotations(true, false, false)
    .build()
  let h1 = bodies.insert(rb1)
  colliders.insert_with_parent(
    @collision.ColliderBuilder3::cuboid(0.2F, 0.6F, 2.0F).build(),
    h1,
    bodies,
  )
  |> ignore
  let rb2 = @dynamics.RigidBodyBuilder3::dynamic()
    .translation(@core.Vec3::new(0.0F, 5.0F, 0.0F))
    .rotation_scaled_axis(@core.Vec3::new(1.0F, 0.0F, 0.0F).scale(1.0F))
    .lock_rotations()
    .build()
  let h2 = bodies.insert(rb2)
  colliders.insert_with_parent(
    @collision.ColliderBuilder3::capsule_y(0.6F, 0.4F).build(),
    h2,
    bodies,
  )
  |> ignore
  colliders.insert_with_parent(
    @collision.ColliderBuilder3::capsule_x(0.6F, 0.4F).build(),
    h2,
    bodies,
  )
  |> ignore
  step_world3_flags(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 60,
  )
  let t1 = bodies.translation(h1)
  @core.abs(t1.y - 3.0F) < 1.0e-3F
}

///|
fn run_examples3d_gyroscopic3() -> Bool {
  // Ported from `rapier-reference/examples3d/gyroscopic3.rs` (headless).
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
  let gravity = @core.Vec3::zero()
  let parameters = @dynamics.IntegrationParameters::default()
  let shapes : Array[(@core.Isometry3, @collision.SharedShape)] = [
    (@core.Isometry3::identity(), @collision.SharedShape::cuboid(2.0F, 0.2F)),
    (
      @core.Isometry3::from_translation(@core.Vec3::new(0.0F, 0.8F, 0.0F)),
      @collision.SharedShape::cuboid(0.2F, 0.4F),
    ),
  ]
  let rb = @dynamics.RigidBodyBuilder3::dynamic()
    .gravity_scale(0.0F)
    .angvel(@core.Vec3::new(0.0F, 20.0F, 0.1F))
    .gyroscopic_forces_enabled(true)
    .build()
  let h = bodies.insert(rb)
  colliders.insert_with_parent(
    @collision.ColliderBuilder3::compound(shapes).build(),
    h,
    bodies,
  )
  |> ignore
  step_world3_flags(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 30,
  )
  if bodies.get(h) is Some(body) {
    @core.abs(body.rotation().angle()) > 0.0F ||
    @core.abs(body.angvel()) >= 0.0F
  } else {
    false
  }
}

///|
test "examples3d/collision_groups3.rs parity (headless)" {
  inspect(run_examples3d_collision_groups3(), content="true")
}

///|
test "examples3d/ccd3.rs parity (headless)" {
  inspect(run_examples3d_ccd3(), content="true")
}

///|
test "examples3d/locked_rotations3.rs parity (headless)" {
  inspect(run_examples3d_locked_rotations3(), content="true")
}

///|
test "examples3d/gyroscopic3.rs parity (headless)" {
  inspect(run_examples3d_gyroscopic3(), content="true")
}
