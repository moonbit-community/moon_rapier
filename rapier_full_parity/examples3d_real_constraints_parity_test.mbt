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
/// Parity tests for constraint/hook-like behaviors in the real dim3 backend.

///|
test "examples3d/one_way_platforms3.rs parity (3DReal headless)" {
  // Ported from `rapier-reference/examples3d/one_way_platforms3.rs` (headless).
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
  let ground = bodies.insert(@dynamics.RigidBodyBuilder3D::fixed().build())
  let platform1 = colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(9.0F, 0.5F, 9.0F)
    .translation(@core.Vec3::new(0.0F, 2.0F, 0.0F))
    .one_way_above(0.1F)
    .surface_velocity(@core.Vec3::new(-12.0F, 0.0F, 0.0F))
    .build(),
    ground,
    bodies,
  )
  let platform2 = colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(9.0F, 0.5F, 9.0F)
    .translation(@core.Vec3::new(0.0F, -2.0F, 0.0F))
    .one_way_below(0.1F)
    .surface_velocity(@core.Vec3::new(12.0F, 0.0F, 0.0F))
    .build(),
    ground,
    bodies,
  )

  // Spawn a few cubes at regular intervals and flip gravity depending on their position.
  let mut first_cube : @dynamics.RigidBodyHandle? = None
  for step in 0..<800 {
    if step % 200 == 0 && bodies.len() <= 7 {
      let body = bodies.insert(
        @dynamics.RigidBodyBuilder3D::dynamic()
        .translation(@core.Vec3::new(0.0F, 6.0F, 0.0F))
        .can_sleep(false)
        .build(),
      )
      colliders.insert_with_parent(
        @collision.ColliderBuilder3D::cuboid(1.0F, 2.0F, 1.0F).build(),
        body,
        bodies,
      )
      |> ignore
      if first_cube is None {
        first_cube = Some(body)
      }
    }
    // Touch the platform colliders so they are kept alive by the compiler (used for contact ids).
    platform1 |> ignore
    platform2 |> ignore
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
    )
  }
  if first_cube is Some(h) && bodies.get(h) is Some(body) {
    inspect(@core.abs(body.translation().x) > 0.1F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "examples3d/rope_joints3.rs parity (3DReal headless)" {
  // Ported from `rapier-reference/examples3d/rope_joints3.rs` (headless; character control omitted).
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

  // Ground walls forming a small box.
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
  let wall_x0 = bodies.insert(
    @dynamics.RigidBodyBuilder3D::fixed()
    .translation(
      @core.Vec3::new(-ground_size - ground_height, ground_height, 0.0F),
    )
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(
      ground_height, ground_height, ground_size,
    ).build(),
    wall_x0,
    bodies,
  )
  |> ignore
  let wall_x1 = bodies.insert(
    @dynamics.RigidBodyBuilder3D::fixed()
    .translation(
      @core.Vec3::new(ground_size + ground_height, ground_height, 0.0F),
    )
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(
      ground_height, ground_height, ground_size,
    ).build(),
    wall_x1,
    bodies,
  )
  |> ignore
  let wall_z0 = bodies.insert(
    @dynamics.RigidBodyBuilder3D::fixed()
    .translation(
      @core.Vec3::new(0.0F, ground_height, -ground_size - ground_height),
    )
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(
      ground_size, ground_height, ground_height,
    ).build(),
    wall_z0,
    bodies,
  )
  |> ignore
  let wall_z1 = bodies.insert(
    @dynamics.RigidBodyBuilder3D::fixed()
    .translation(
      @core.Vec3::new(0.0F, ground_height, ground_size + ground_height),
    )
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(
      ground_size, ground_height, ground_height,
    ).build(),
    wall_z1,
    bodies,
  )
  |> ignore

  // Kinematic character.
  let character_handle = bodies.insert(
    @dynamics.RigidBodyBuilder3D::kinematic_position_based()
    .translation(@core.Vec3::new(0.0F, 0.3F, 0.0F))
    .can_sleep(false)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(0.15F, 0.3F, 0.15F).build(),
    character_handle,
    bodies,
  )
  |> ignore

  // Tethered ball.
  let rad = 0.04F
  let child_handle = bodies.insert(
    @dynamics.RigidBodyBuilder3D::dynamic()
    .translation(@core.Vec3::new(1.0F, 1.0F, 0.0F))
    .can_sleep(false)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::ball(rad).build(),
    child_handle,
    bodies,
  )
  |> ignore
  let rope = @dynamics.RopeJointSet3DReal::new()
  rope.insert(character_handle, child_handle, 2.0F)

  // Move the kinematic character deterministically to stretch the rope.
  let mut time = 0.0F
  for _ in 0..<600 {
    time = time + dt
    let x = Float::from_double(@math.sin(time.to_double())) * 1.2F
    let z = Float::from_double(@math.cos(time.to_double())) * 0.4F
    if bodies.get_mut(character_handle) is Some(rb) {
      rb.set_next_kinematic_translation(@core.Vec3::new(x, 0.3F, z))
    } else {
      inspect(false, content="true")
    }
    pipeline.step_with_rope_joints(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
      rope,
    )
  }
  if bodies.get(character_handle) is Some(ch) &&
    bodies.get(child_handle) is Some(ball) {
    let d = ball.translation().sub(ch.translation()).length()
    inspect(d <= 2.1F, content="true")
    inspect(ball.translation().y > -10.0F, content="true")
  } else {
    inspect(false, content="true")
  }
}
