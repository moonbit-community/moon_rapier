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
fn step_world3_flags_real(
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
fn step_world3_flags_real_with_events(
  pipeline : @pipeline.PhysicsPipeline3DReal,
  gravity : @core.Vec3,
  parameters : @dynamics.IntegrationParameters,
  islands : @dynamics.IslandManager3D,
  broad_phase : @collision.BroadPhase3D,
  narrow_phase : @collision.NarrowPhase3D,
  bodies : @dynamics.RigidBodySet3D,
  colliders : @collision.ColliderSet3D,
  events : @pipeline.EventHandler3D,
  steps : Int,
) -> Unit {
  for _ in 0..<steps {
    pipeline.step_with_events(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
      events,
    )
  }
}

///|
fn run_examples3d_collision_groups3() -> Bool {
  // Ported from `rapier-reference/examples3d/collision_groups3.rs`.
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
  let ground_size = 5.0F
  let ground_height = 0.1F
  let floor = bodies.insert(
    @dynamics.RigidBodyBuilder3D::fixed()
    .translation(@core.Vec3::new(0.0F, -ground_height, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(
      ground_size, ground_height, ground_size,
    ).build(),
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
  let green_platform = colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(1.0F, 0.1F, 1.0F)
    .translation(@core.Vec3::new(0.0F, 1.0F, 0.0F))
    .collision_groups(green_group)
    .build(),
    floor,
    bodies,
  )
  let blue_platform = colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(1.0F, 0.1F, 1.0F)
    .translation(@core.Vec3::new(0.0F, 2.0F, 0.0F))
    .collision_groups(blue_group)
    .build(),
    floor,
    bodies,
  )
  let green_handles : Array[@dynamics.RigidBodyHandle] = []
  let blue_handles : Array[@dynamics.RigidBodyHandle] = []
  let rad = 0.2F
  let green_body = bodies.insert(
    @dynamics.RigidBodyBuilder3D::dynamic()
    .translation(@core.Vec3::new(0.0F, 5.0F, -0.5F))
    .build(),
  )
  let green_collider = colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(rad, rad, rad)
    .collision_groups(green_group)
    .build(),
    green_body,
    bodies,
  )
  green_handles.push(green_body)
  let blue_body = bodies.insert(
    @dynamics.RigidBodyBuilder3D::dynamic()
    .translation(@core.Vec3::new(0.0F, 5.0F, 0.5F))
    .build(),
  )
  let blue_collider = colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(rad, rad, rad)
    .collision_groups(blue_group)
    .build(),
    blue_body,
    bodies,
  )
  blue_handles.push(blue_body)
  let mut green_hits_green = false
  let mut blue_hits_blue = false
  let mut green_hits_blue = false
  let mut blue_hits_green = false
  for _ in 0..<240 {
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    )
    if !green_hits_green &&
      narrow_phase.contact_pair(green_collider, green_platform) is Some(pair) &&
      pair.manifolds_len() > 0 {
      green_hits_green = true
    }
    if !blue_hits_blue &&
      narrow_phase.contact_pair(blue_collider, blue_platform) is Some(pair) &&
      pair.manifolds_len() > 0 {
      blue_hits_blue = true
    }
    if !green_hits_blue &&
      narrow_phase.contact_pair(green_collider, blue_platform) is Some(pair) &&
      pair.manifolds_len() > 0 {
      green_hits_blue = true
    }
    if !blue_hits_green &&
      narrow_phase.contact_pair(blue_collider, green_platform) is Some(pair) &&
      pair.manifolds_len() > 0 {
      blue_hits_green = true
    }
  }
  green_hits_green &&
  blue_hits_blue &&
  !green_hits_blue &&
  !blue_hits_green &&
  green_handles.length() == 1 &&
  blue_handles.length() == 1
}

///|
fn run_examples3d_ccd3() -> Bool {
  // Ported from `rapier-reference/examples3d/ccd3.rs` (headless; asserts CCD causes collisions).
  let pipeline = @pipeline.PhysicsPipeline3DReal::new()
  let broad_phase = @collision.BroadPhase3D::new()
  let narrow_phase = @collision.NarrowPhase3D::new()
  let bodies = @dynamics.RigidBodySet3D::new()
  let colliders = @collision.ColliderSet3D::new()
  let islands = @dynamics.IslandManager3D::new()
  let events = @pipeline.EventHandler3D::new()
  let gravity = @core.Vec3::new(0.0F, -9.81F, 0.0F)
  let parameters = @dynamics.IntegrationParameters::default()
  // Simplified deterministic scene: a single fixed wall to hit at high speed.
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
  let wall = bodies.insert(
    @dynamics.RigidBodyBuilder3D::fixed()
    .translation(@core.Vec3::new(0.0F, 1.0F, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(0.5F, 2.0F, 10.0F).build(),
    wall,
    bodies,
  )
  |> ignore
  let sensor_collider = @collision.ColliderBuilder3D::ball(1.0F)
    .density(10.0F)
    .sensor(true)
    .active_events(@collision.ActiveEvents::collision_events())
    .build()
  let sensor_body = @dynamics.RigidBodyBuilder3D::dynamic()
    .linvel(@core.Vec3::new(1000.0F, 0.0F, 0.0F))
    .translation(@core.Vec3::new(-20.0F, 1.0F, 0.0F))
    .ccd_enabled(true)
    .build()
  let sensor_handle = bodies.insert(sensor_body)
  colliders.insert_with_parent(sensor_collider, sensor_handle, bodies) |> ignore
  let collider = @collision.ColliderBuilder3D::ball(1.0F).density(10.0F).build()
  let body = @dynamics.RigidBodyBuilder3D::dynamic()
    .linvel(@core.Vec3::new(1000.0F, 0.0F, 0.0F))
    .translation(@core.Vec3::new(-20.0F, 1.0F, 3.0F))
    .ccd_enabled(true)
    .build()
  let handle = bodies.insert(body)
  colliders.insert_with_parent(collider, handle, bodies) |> ignore
  step_world3_flags_real_with_events(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    events, 5,
  )
  let evs = events.take_collision_events()
  let t = if bodies.get(handle) is Some(rb) {
    rb.translation()
  } else {
    @core.Vec3::zero()
  }
  evs.length() > 0 && t.x < 50.0F
}

///|
fn run_examples3d_locked_rotations3() -> Bool {
  // Ported from `rapier-reference/examples3d/locked_rotations3.rs` (headless).
  let pipeline = @pipeline.PhysicsPipeline3DReal::new()
  let broad_phase = @collision.BroadPhase3D::new()
  let narrow_phase = @collision.NarrowPhase3D::new()
  let bodies = @dynamics.RigidBodySet3D::new()
  let colliders = @collision.ColliderSet3D::new()
  let islands = @dynamics.IslandManager3D::new()
  let gravity = @core.Vec3::new(0.0F, -9.81F, 0.0F)
  let parameters = @dynamics.IntegrationParameters::default()
  let ground_size = 5.0F
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
  let rb1 = @dynamics.RigidBodyBuilder3D::dynamic()
    .translation(@core.Vec3::new(0.0F, 3.0F, 0.0F))
    .lock_translations()
    .enabled_rotations(true, false, false)
    .build()
  let h1 = bodies.insert(rb1)
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(0.2F, 0.6F, 2.0F).build(),
    h1,
    bodies,
  )
  |> ignore
  let rb2 = @dynamics.RigidBodyBuilder3D::dynamic()
    .translation(@core.Vec3::new(0.0F, 5.0F, 0.0F))
    .rotation_scaled_axis(@core.Vec3::new(1.0F, 0.0F, 0.0F).scale(1.0F))
    .lock_rotations()
    .build()
  let h2 = bodies.insert(rb2)
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::capsule_y(0.6F, 0.4F).build(),
    h2,
    bodies,
  )
  |> ignore
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::capsule_x(0.6F, 0.4F).build(),
    h2,
    bodies,
  )
  |> ignore
  step_world3_flags_real(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    60,
  )
  let t1 = if bodies.get(h1) is Some(rb) {
    rb.translation()
  } else {
    @core.Vec3::zero()
  }
  @core.abs(t1.y - 3.0F) < 1.0e-3F
}

///|
fn run_examples3d_gyroscopic3() -> Bool {
  // Ported from `rapier-reference/examples3d/gyroscopic3.rs` (headless).
  let pipeline = @pipeline.PhysicsPipeline3DReal::new()
  let broad_phase = @collision.BroadPhase3D::new()
  let narrow_phase = @collision.NarrowPhase3D::new()
  let bodies = @dynamics.RigidBodySet3D::new()
  let colliders = @collision.ColliderSet3D::new()
  let islands = @dynamics.IslandManager3D::new()
  let gravity = @core.Vec3::zero()
  let parameters = @dynamics.IntegrationParameters::default()
  // Use an anisotropic inertia tensor so the gyroscopic term changes angular velocity over time.
  let props = @core.MassProperties3::new(
    1.0F,
    @core.SdpMat3::from_diagonal(@core.Vec3::new(1.0F, 0.5F, 0.25F)),
    @core.Vec3::zero(),
  )
  let w0 = @core.Vec3::new(0.0F, 20.0F, 0.1F)
  let rb = @dynamics.RigidBodyBuilder3D::dynamic()
    .gravity_scale(0.0F)
    .mass_properties(props)
    .angvel(w0)
    .gyroscopic_forces_enabled(true)
    .build()
  let h = bodies.insert(rb)
  // Attach a couple colliders so the setup matches the upstream "compound" spirit.
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(2.0F, 0.2F, 0.2F).build(),
    h,
    bodies,
  )
  |> ignore
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(0.2F, 0.4F, 0.2F)
    .translation(@core.Vec3::new(0.0F, 0.8F, 0.0F))
    .build(),
    h,
    bodies,
  )
  |> ignore
  step_world3_flags_real(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    30,
  )
  if bodies.get(h) is Some(body) {
    let w = body.angvel()
    // With anisotropic inertia + gyro term, w.x should become non-zero.
    @core.abs(w.x) > 1.0e-3F && w.x == w.x
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
