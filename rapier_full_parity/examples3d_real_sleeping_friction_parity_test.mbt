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
/// Parity tests focusing on sleeping and friction behaviors for the real dim3 backend.

///|
test "examples3d/debug_friction3.rs parity (3DReal headless)" {
  // Ported from `rapier-reference/examples3d/debug_friction3.rs` (headless).
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
  let ground_size = 100.0F
  let ground_height = 0.1F
  let ground = bodies.insert(@dynamics.RigidBodyBuilder3D::fixed().build())
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(
      ground_size, ground_height, ground_size,
    )
    .friction(1.5F)
    .build(),
    ground,
    bodies,
  )
  |> ignore
  let body = bodies.insert(
    @dynamics.RigidBodyBuilder3D::dynamic()
    .translation(@core.Vec3::new(0.0F, 1.1F, 0.0F))
    .rotation(
      @core.rotation_from_scaled_axis(@core.Vec3::new(0.0F, 0.3F, 0.0F)),
    )
    .can_sleep(false)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(2.0F, 1.0F, 3.0F)
    .friction(1.5F)
    .build(),
    body,
    bodies,
  )
  |> ignore
  if bodies.get_mut(body) is Some(rb) {
    // force = rotation * Z * 50
    let force = rb.rotation().rotate_vec3(@core.Vec3::new(0.0F, 0.0F, 50.0F))
    rb.set_linvel(force)
  }
  for _ in 0..<240 {
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    )
    if bodies.get(body) is Some(rb) {
      let speed = rb.linvel().length()
      inspect(speed == speed, content="true")
      inspect(speed < 200.0F, content="true")
    }
  }
  if bodies.get(body) is Some(rb) {
    let p = rb.translation()
    inspect(p.x == p.x && p.y == p.y && p.z == p.z, content="true")
    inspect(p.y > -10.0F, content="true")
    inspect(rb.linvel().length() < 60.0F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "examples3d/debug_sleeping_kinematic3.rs parity (3DReal headless)" {
  // Ported from `rapier-reference/examples3d/debug_sleeping_kinematic3.rs` (headless).
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

  // Velocity-based kinematic platform.
  let platform = bodies.insert(
    @dynamics.RigidBodyBuilder3D::kinematic_velocity_based()
    .translation(@core.Vec3::new(0.0F, 1.5F + 0.8F, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(5.0F, 0.5F, 5.0F).build(),
    platform,
    bodies,
  )
  |> ignore

  // Slow platform.
  let slow_platform = bodies.insert(
    @dynamics.RigidBodyBuilder3D::kinematic_velocity_based()
    .translation(@core.Vec3::new(0.0F, 0.0F, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(5.0F, 0.5F, 5.0F).build(),
    slow_platform,
    bodies,
  )
  |> ignore
  let start_tick = 200
  let stop_tick = 400
  let mut time = 0.0F
  for step in 0..<900 {
    // Simulate the callback logic from the rapier-reference example.
    if step == stop_tick {
      if bodies.get(platform) is Some(p) {
        inspect(!p.is_sleeping(), content="true")
      } else {
        inspect(false, content="true")
      }
      if bodies.get(slow_platform) is Some(p) {
        inspect(!p.is_sleeping(), content="true")
      } else {
        inspect(false, content="true")
      }
      if bodies.get_mut(slow_platform) is Some(p) {
        p.set_linvel(@core.Vec3::zero())
      }
      if bodies.get_mut(platform) is Some(p) {
        p.set_linvel(@core.Vec3::zero())
      }
    }
    if step > stop_tick + 200 {
      if bodies.get(platform) is Some(p) {
        inspect(p.is_sleeping(), content="true")
      } else {
        inspect(false, content="true")
      }
      if bodies.get(slow_platform) is Some(p) {
        inspect(p.is_sleeping(), content="true")
      } else {
        inspect(false, content="true")
      }
    }
    if step >= start_tick && step < stop_tick {
      if step == start_tick {
        if bodies.get(platform) is Some(p) {
          inspect(p.is_sleeping(), content="true")
        } else {
          inspect(false, content="true")
        }
        if bodies.get(slow_platform) is Some(p) {
          inspect(p.is_sleeping(), content="true")
        } else {
          inspect(false, content="true")
        }
        let slow_velocity = @core.Vec3::new(0.0F, 0.01F, 0.0F)
        if bodies.get_mut(slow_platform) is Some(p) {
          p.set_linvel(slow_velocity)
          p.wake_up()
        }
      }
      let velocity = @core.Vec3::new(
        0.0F,
        Float::from_double(@math.cos((time * 2.0F).to_double())),
        Float::from_double(@math.sin(time.to_double())) * 2.0F,
      )
      if bodies.get_mut(platform) is Some(p) {
        p.set_linvel(velocity)
        p.wake_up()
      }
    }
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    )
    time = time + dt
  }
}
