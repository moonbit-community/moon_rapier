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
/// Additional regression-style debug parity tests for the real dim3 backend.

///|
test "examples3d/debug_internal_edges3.rs parity (3DReal headless; reduced heightfield)" {
  // Ported from `rapier-reference/examples3d/debug_internal_edges3.rs` (headless).
  //
  // The upstream example uses a 100x100 heightfield. We reduce the resolution to keep the
  // brute-force trimesh contact generation manageable.
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
  let n = 20
  let heights : Array[@core.Real] = []
  for _ in 0..<(n * n) {
    heights.push(0.0F)
  }
  let hf = @collision.ColliderBuilder3D::heightfield(
      heights,
      n,
      n,
      @core.Vec3::new(60.0F, 1.0F, 60.0F),
    )
    .unwrap()
    .build()
  colliders.insert(hf) |> ignore

  // Dynamic rigid bodies.
  let fast_ball = bodies.insert(
    @dynamics.RigidBodyBuilder3D::dynamic()
    .translation(@core.Vec3::new(4.0F, 0.5F, 0.0F))
    .linvel(@core.Vec3::new(0.0F, -40.0F, 20.0F))
    .can_sleep(false)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::ball(0.5F).build(),
    fast_ball,
    bodies,
  )
  |> ignore
  let cube = bodies.insert(
    @dynamics.RigidBodyBuilder3D::dynamic()
    .translation(@core.Vec3::new(-3.0F, 5.0F, 0.0F))
    .linvel(@core.Vec3::new(0.0F, -4.0F, 20.0F))
    .can_sleep(false)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(0.5F, 0.5F, 0.5F).build(),
    cube,
    bodies,
  )
  |> ignore
  let cyl = bodies.insert(
    @dynamics.RigidBodyBuilder3D::dynamic()
    .translation(@core.Vec3::new(8.0F, 0.2F, 0.0F))
    .linvel(@core.Vec3::new(0.0F, -4.0F, 20.0F))
    .can_sleep(false)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cylinder(0.5F, 0.2F)
    .rotation(
      @core.rotation_from_scaled_axis(@core.Vec3::new(0.0F, 0.0F, 1.5707964F)),
    )
    .build(),
    cyl,
    bodies,
  )
  |> ignore
  for _ in 0..<120 {
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    )
  }
  for h in [fast_ball, cube, cyl] {
    if bodies.get(h) is Some(rb) {
      let p = rb.translation()
      inspect(p.x == p.x && p.y == p.y && p.z == p.z, content="true")
      inspect(p.y > -50.0F, content="true")
    } else {
      inspect(false, content="true")
    }
  }
}

///|
test "examples3d/debug_pop3.rs parity (3DReal headless)" {
  // Ported from `rapier-reference/examples3d/debug_pop3.rs` (headless).
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
  let ground_size = 10.0F
  let ground_height = 10.0F
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

  // Dynamic cube starting at the origin (overlapping the ground volume).
  let body = bodies.insert(
    @dynamics.RigidBodyBuilder3D::dynamic().can_sleep(false).build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(1.0F, 1.0F, 1.0F).build(),
    body,
    bodies,
  )
  |> ignore
  for _ in 0..<240 {
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    )
  }
  if bodies.get(body) is Some(rb) {
    let p = rb.translation()
    inspect(p.x == p.x && p.y == p.y && p.z == p.z, content="true")
    // The cube should have been pushed upward (out of the ground).
    inspect(p.y > -1.0F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "examples3d/debug_infinite_fall3.rs parity (3DReal headless)" {
  // Ported from `rapier-reference/examples3d/debug_infinite_fall3.rs` (headless).
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
  let ground_size = 100.1F
  let ground_height = 2.1F
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder3D::fixed()
    .translation(@core.Vec3::new(0.0F, 4.0F, 0.0F))
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
  let rad = 1.0F
  let b1 = bodies.insert(
    @dynamics.RigidBodyBuilder3D::dynamic()
    .translation(@core.Vec3::new(0.0F, 7.0F * rad, 0.0F))
    .can_sleep(false)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::ball(rad).build(),
    b1,
    bodies,
  )
  |> ignore
  let b2 = bodies.insert(
    @dynamics.RigidBodyBuilder3D::dynamic()
    .translation(@core.Vec3::new(0.0F, 2.0F * rad, 0.0F))
    .can_sleep(false)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::ball(rad).build(),
    b2,
    bodies,
  )
  |> ignore
  for _ in 0..<600 {
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    )
  }
  for h in [b1, b2] {
    if bodies.get(h) is Some(rb) {
      let p = rb.translation()
      inspect(p.x == p.x && p.y == p.y && p.z == p.z, content="true")
      inspect(p.y > -50.0F, content="true")
    } else {
      inspect(false, content="true")
    }
  }
}
