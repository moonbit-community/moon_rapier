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
/// 3DReal parity tests for `rapier-reference/examples3d/character_controller3.rs`.

///|
test "examples3d/character_controller3.rs parity (3DReal headless; reduced)" {
  // Ported from `rapier-reference/examples3d/character_controller3.rs` (headless; reduced).
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

  // Character (kinematic position-based capsule).
  let character = bodies.insert(
    @dynamics.RigidBodyBuilder3D::kinematic_position_based()
    .translation(@core.Vec3::new(0.0F, 0.5F, 0.0F))
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

  // A few cubes (reduced).
  for i in 0..<8 {
    let x = Float::from_int(i) * 0.2F - 0.6F
    let z = 0.8F
    let h = bodies.insert(
      @dynamics.RigidBodyBuilder3D::dynamic()
      .translation(@core.Vec3::new(x, 0.2F, z))
      .build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder3D::cuboid(0.1F, 0.1F, 0.1F).build(),
      h,
      bodies,
    )
    |> ignore
  }

  // Stairs (reduced).
  let stair_width = 1.0F
  let stair_height = 0.1F
  for i in 0..<6 {
    let x = Float::from_int(i) * stair_width / 2.0F
    let y = Float::from_int(i) * stair_height * 1.5F + 1.0F
    colliders.insert(
      @collision.ColliderBuilder3D::cuboid(
        stair_width / 2.0F,
        stair_height / 2.0F,
        stair_width,
      )
      .translation(@core.Vec3::new(x, y, 0.0F))
      .build(),
    )
    |> ignore
  }

  // Climbable slope.
  colliders.insert(
    @collision.ColliderBuilder3D::cuboid(2.0F, ground_height, 2.0F)
    .translation(@core.Vec3::new(2.1F, -ground_height + 0.4F, 0.0F))
    .rotation(
      @core.rotation_from_scaled_axis(@core.Vec3::new(0.0F, 0.0F, 0.2F)),
    )
    .build(),
  )
  |> ignore

  // Too-steep slope to block movement.
  colliders.insert(
    @collision.ColliderBuilder3D::cuboid(2.0F, ground_height, ground_size)
    .translation(@core.Vec3::new(4.2F, -ground_height + 1.7F, 0.0F))
    .rotation(
      @core.rotation_from_scaled_axis(@core.Vec3::new(0.0F, 0.0F, 0.8F)),
    )
    .build(),
  )
  |> ignore
  let queries = @collision.QueryPipelineMut3DReal::new(
    @collision.QueryFilter3DReal::new(),
    bodies,
    colliders,
  )
  let controller = @control.KinematicCharacterController3DReal::default()
    .with_offset(0.05F)
    .with_max_slope_climb_angle(0.6F)
    .with_min_slope_slide_angle(0.6F)
  let speed = 2.0F
  for _ in 0..<400 {
    controller.move_character(
      bodies,
      colliders,
      queries,
      character,
      @core.Vec3::new(speed * dt, 0.0F, 0.0F),
    )
    |> ignore
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    )
  }
  if bodies.get(character) is Some(rb) {
    let p = rb.translation()
    inspect(p.x == p.x && p.y == p.y && p.z == p.z, content="true")
    // The character moved forward before being blocked by the steep slope.
    inspect(p.x > 1.0F, content="true")
    inspect(p.x < 6.0F, content="true")
    inspect(p.y > -50.0F, content="true")
  } else {
    inspect(false, content="true")
  }
}
