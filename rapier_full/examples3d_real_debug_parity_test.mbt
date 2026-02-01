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
/// Headless parity tests for a subset of `rapier-reference/examples3d/debug_*.rs` against the
/// real dim3 backend.

///|
test "examples3d/debug_disabled3.rs parity (3DReal headless)" {
  // Ported from `rapier-reference/examples3d/debug_disabled3.rs` (headless; reduced runtime).
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
  let rad = 0.5F

  // Ground.
  let ground_size = 10.1F
  let ground_height = 2.1F
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

  // Platform that will be enabled/disabled.
  let platform_body = bodies.insert(
    @dynamics.RigidBodyBuilder3D::dynamic()
    .translation(@core.Vec3::new(0.0F, 5.0F, 0.0F))
    .build(),
  )
  let platform = colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(5.0F, 1.0F, 5.0F).build(),
    platform_body,
    bodies,
  )
  for step in 0..<400 {
    if step % 250 == 0 {
      if colliders.get_mut(platform) is Some(co) {
        co.set_enabled(!co.enabled()) |> ignore
      }
    }
    if step % 25 == 0 {
      let rb = bodies.insert(
        @dynamics.RigidBodyBuilder3D::dynamic()
        .translation(@core.Vec3::new(0.0F, 20.0F, 0.0F))
        .build(),
      )
      colliders.insert_with_parent(
        @collision.ColliderBuilder3D::cuboid(rad, rad, rad).build(),
        rb,
        bodies,
      )
      |> ignore
    }
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    )
  }
  // Sanity: no NaNs on a few representative bodies.
  if bodies.get(platform_body) is Some(rb) {
    let p = rb.translation()
    inspect(p.x == p.x && p.y == p.y && p.z == p.z, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "examples3d/debug_add_remove_collider3.rs parity (3DReal headless)" {
  // Ported from `rapier-reference/examples3d/debug_add_remove_collider3.rs` (headless).
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

  // Ground.
  let ground_size = 3.0F
  let ground_height = 0.1F
  let ground_body = bodies.insert(
    @dynamics.RigidBodyBuilder3D::fixed()
    .translation(@core.Vec3::new(0.0F, -ground_height, 0.0F))
    .build(),
  )
  let mut ground_collider = colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(ground_size, ground_height, 0.4F).build(),
    ground_body,
    bodies,
  )

  // Rolling ball.
  let ball_rad = 0.1F
  let ball = bodies.insert(
    @dynamics.RigidBodyBuilder3D::dynamic()
    .translation(@core.Vec3::new(0.0F, 0.2F, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::ball(ball_rad).build(),
    ball,
    bodies,
  )
  |> ignore
  for _ in 0..<240 {
    // Remove then re-add the ground collider.
    let removed = ground_collider
    if colliders.remove(removed) is Some(co) {
      ground_collider = colliders.insert_with_parent(co, ground_body, bodies)
    } else {
      inspect(false, content="true")
    }
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    )
  }
  if bodies.get(ball) is Some(rb) {
    let p = rb.translation()
    inspect(p.x == p.x && p.y == p.y && p.z == p.z, content="true")
    inspect(p.y > -10.0F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "examples3d/debug_dynamic_collider_add3.rs parity (3DReal headless)" {
  // Ported from `rapier-reference/examples3d/debug_dynamic_collider_add3.rs` (headless; reduced).
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

  // Ground.
  let ground_size = 20.0F
  let ground_height = 0.1F
  let ground_body = bodies.insert(
    @dynamics.RigidBodyBuilder3D::fixed()
    .translation(@core.Vec3::new(0.0F, -ground_height, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(ground_size, ground_height, 0.4F)
    .friction(0.15F)
    .build(),
    ground_body,
    bodies,
  )
  |> ignore

  // Rolling ball.
  let ball_rad = 0.1F
  let ball = bodies.insert(
    @dynamics.RigidBodyBuilder3D::dynamic()
    .translation(@core.Vec3::new(0.0F, 0.2F, 0.0F))
    .linvel(@core.Vec3::new(10.0F, 0.0F, 0.0F))
    .can_sleep(false)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::ball(ball_rad).build(),
    ball,
    bodies,
  )
  |> ignore
  let mut step = 0
  let extra : Array[@collision.ColliderHandle3D] = []
  let snapped_frame = 20
  let mut snap_linvel = @core.Vec3::zero()
  let mut snap_angvel = @core.Vec3::zero()
  let mut snap_pos = @core.Isometry3::identity()
  for _ in 0..<120 {
    step = step + 1
    // Add a slightly bigger ball collider.
    let co = @collision.ColliderBuilder3D::ball(
      ball_rad + 0.005F * Float::from_int(step),
    ).build()
    let h = colliders.insert_with_parent(co, ball, bodies)
    extra.push(h)
    if bodies.get(ball) is Some(rb) && step == snapped_frame {
      snap_linvel = rb.linvel()
      snap_angvel = rb.angvel()
      snap_pos = rb.position()
    }
    if step == 60 {
      if bodies.get_mut(ball) is Some(rb) {
        rb.set_linvel(snap_linvel)
        rb.set_angvel(snap_angvel)
        rb.set_translation(snap_pos.translation)
        rb.set_rotation(snap_pos.rotation)
      }
      for i in 0..<extra.length() {
        colliders.remove(extra[i]) |> ignore
      }
      extra.clear()
      step = snapped_frame
    }
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    )
  }
  if bodies.get(ball) is Some(rb) {
    let p = rb.translation()
    inspect(p.x == p.x && p.y == p.y && p.z == p.z, content="true")
    inspect(p.y > -10.0F, content="true")
  } else {
    inspect(false, content="true")
  }
}
