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
/// Additional headless parity tests for `rapier-reference/examples3d/debug_*.rs` against the
/// real dim3 backend (adapted where Rapier uses shapes not available yet).

///|
test "examples3d/debug_two_cubes3.rs parity (3DReal headless)" {
  // Ported from `rapier-reference/examples3d/debug_two_cubes3.rs` (headless).
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

  // Dynamic cube.
  let dyn_body = bodies.insert(
    @dynamics.RigidBodyBuilder3D::dynamic()
    .translation(@core.Vec3::new(0.0F, 2.0F, 0.0F))
    .can_sleep(false)
    .build(),
  )
  let dyn_co = colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(0.5F, 0.5F, 0.5F).build(),
    dyn_body,
    bodies,
  )

  // Fixed cube.
  let fixed = bodies.insert(@dynamics.RigidBodyBuilder3D::fixed().build())
  let fixed_co = colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(0.5F, 0.5F, 0.5F).build(),
    fixed,
    bodies,
  )
  let mut saw_contact = false
  for _ in 0..<240 {
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    )
    if narrow_phase.contact_pair(dyn_co, fixed_co) is Some(pair) &&
      pair.manifolds_len() > 0 {
      saw_contact = true
    }
  }
  inspect(saw_contact, content="true")
  if bodies.get(dyn_body) is Some(rb) {
    let p = rb.translation()
    inspect(p.x == p.x && p.y == p.y && p.z == p.z, content="true")
    inspect(p.y > -10.0F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "examples3d/debug_boxes3.rs parity (3DReal headless; reduced)" {
  // Ported from `rapier-reference/examples3d/debug_boxes3.rs` (headless).
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
  let ground_size = 100.1F
  let ground_height = 0.1F
  // The upstream example inserts multiple identical ground colliders; we keep just two to reduce
  // the broadphase load while still testing duplicate shapes.
  for _ in 0..<2 {
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
  }
  for _ in 0..<2 {
    let handle = bodies.insert(
      @dynamics.RigidBodyBuilder3D::dynamic()
      .translation(@core.Vec3::new(1.1F, 0.0F, 0.0F))
      .can_sleep(false)
      .build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder3D::cuboid(2.0F, 0.1F, 1.0F).build(),
      handle,
      bodies,
    )
    |> ignore
  }
  for _ in 0..<240 {
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    )
  }
  // Sanity: all active bodies remain finite.
  for h in islands.active_bodies() {
    if bodies.get(h) is Some(rb) {
      let p = rb.translation()
      inspect(p.x == p.x && p.y == p.y && p.z == p.z, content="true")
    }
  }
}

///|
test "examples3d/debug_triangle3.rs parity (3DReal headless; trimesh triangle)" {
  // Ported from `rapier-reference/examples3d/debug_triangle3.rs`.
  //
  // Rapier uses a dedicated triangle shape; for the real dim3 backend we approximate this with
  // a trimesh containing a single triangle.
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

  // Triangle ground.
  let vtx = [
    @core.Vec3::new(-10.0F, 0.0F, -10.0F),
    @core.Vec3::new(10.0F, 0.0F, -10.0F),
    @core.Vec3::new(0.0F, 0.0F, 10.0F),
  ]
  let idx = [(0, 1, 2)]
  let ground = bodies.insert(@dynamics.RigidBodyBuilder3D::fixed().build())
  let tri = @collision.ColliderBuilder3D::trimesh(vtx, idx).unwrap().build()
  colliders.insert_with_parent(tri, ground, bodies) |> ignore

  // Dynamic "box" (thin cuboid).
  let body = bodies.insert(
    @dynamics.RigidBodyBuilder3D::dynamic()
    .translation(@core.Vec3::new(1.1F, 0.01F, 0.0F))
    .can_sleep(false)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(20.0F, 0.1F, 1.0F).build(),
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
    inspect(p.y > -10.0F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "examples3d/debug_big_colliders3.rs parity (3DReal headless; adapted ground)" {
  // Ported from `rapier-reference/examples3d/debug_big_colliders3.rs` (headless).
  //
  // Rapier uses a HalfSpace ground; we approximate with a very large cuboid.
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
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder3D::fixed()
    .translation(@core.Vec3::new(0.0F, -1.0F, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(2000.0F, 1.0F, 2000.0F).build(),
    ground,
    bodies,
  )
  |> ignore
  let mut curr_y = 0.0F
  let mut curr_width = 200.0F
  // Reduced iterations vs rapier-reference (12) to keep runtime small.
  for _ in 0..<8 {
    let curr_height = if 0.1F < curr_width { 0.1F } else { curr_width }
    curr_y = curr_y + curr_height * 4.0F
    let handle = bodies.insert(
      @dynamics.RigidBodyBuilder3D::dynamic()
      .translation(@core.Vec3::new(0.0F, curr_y, 0.0F))
      .can_sleep(false)
      .build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder3D::cuboid(curr_width, curr_height, curr_width).build(),
      handle,
      bodies,
    )
    |> ignore
    curr_width = curr_width / 5.0F
  }
  for _ in 0..<240 {
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    )
  }
  for h in islands.active_bodies() {
    if bodies.get(h) is Some(rb) {
      let p = rb.translation()
      inspect(p.x == p.x && p.y == p.y && p.z == p.z, content="true")
      inspect(p.y > -100.0F, content="true")
    }
  }
}

///|
test "examples3d/debug_balls3.rs parity (3DReal headless; reduced)" {
  // Ported from `rapier-reference/examples3d/debug_balls3.rs` (headless; reduced).
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
  let num_j = 4
  let num_ik = 4
  let rad = 0.5F
  let shift = rad * 2.0F
  let centerx = shift * Float::from_int(num_ik) / 2.0F
  let centery = shift / 2.0F
  let centerz = shift * Float::from_int(num_ik) / 2.0F
  for i in 0..<num_ik {
    for j in 0..<num_j {
      for k in 0..<num_ik {
        let x = Float::from_int(i) * shift - centerx
        let z = Float::from_int(k) * shift - centerz
        let fixed = j == 0 ||
          i == 0 ||
          k == 0 ||
          i == num_ik - 1 ||
          k == num_ik - 1
        let y = if fixed {
          Float::from_int(j) * shift + centery
        } else {
          Float::from_int(j) * shift * 2.0F + centery
        }
        let body = bodies.insert(
          (if fixed {
            @dynamics.RigidBodyBuilder3D::fixed()
          } else {
            @dynamics.RigidBodyBuilder3D::dynamic().can_sleep(false)
          })
          .translation(@core.Vec3::new(x, y, z))
          .build(),
        )
        colliders.insert_with_parent(
          @collision.ColliderBuilder3D::ball(rad).friction(0.0F).build(),
          body,
          bodies,
        )
        |> ignore
      }
    }
  }
  for _ in 0..<120 {
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    )
  }
  for h in islands.active_bodies() {
    if bodies.get(h) is Some(rb) {
      let p = rb.translation()
      inspect(p.x == p.x && p.y == p.y && p.z == p.z, content="true")
    }
  }
}

///|
test "examples3d/debug_cube_high_mass_ratio3.rs parity (3DReal headless; reduced)" {
  // Ported from `rapier-reference/examples3d/debug_cube_high_mass_ratio3.rs` (headless; reduced).
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
  let num_levels = 2
  let stick_len = 2.0F
  let stick_rad = 0.2F

  // Floor.
  let floor_body = bodies.insert(
    @dynamics.RigidBodyBuilder3D::fixed()
    .translation(@core.Vec3::new(0.0F, -stick_len - stick_rad, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(stick_len, stick_len, stick_len).build(),
    floor_body,
    bodies,
  )
  |> ignore

  // Sticks.
  for i in 0..<num_levels {
    let fi = Float::from_int(i)
    let y0 = fi * stick_rad * 4.0F
    let y1 = (fi + 0.5F) * stick_rad * 4.0F
    for z in [-(stick_len / 2.0F - stick_rad), stick_len / 2.0F - stick_rad] {
      let b = bodies.insert(
        @dynamics.RigidBodyBuilder3D::dynamic()
        .translation(@core.Vec3::new(0.0F, y0, z))
        .can_sleep(false)
        .build(),
      )
      colliders.insert_with_parent(
        @collision.ColliderBuilder3D::cuboid(
          stick_len / 2.0F,
          stick_rad,
          stick_rad,
        ).build(),
        b,
        bodies,
      )
      |> ignore
    }
    for x in [-(stick_len / 2.0F - stick_rad), stick_len / 2.0F - stick_rad] {
      let b = bodies.insert(
        @dynamics.RigidBodyBuilder3D::dynamic()
        .translation(@core.Vec3::new(x, y1, 0.0F))
        .can_sleep(false)
        .build(),
      )
      colliders.insert_with_parent(
        @collision.ColliderBuilder3D::cuboid(
          stick_rad,
          stick_rad,
          stick_len / 2.0F,
        ).build(),
        b,
        bodies,
      )
      |> ignore
    }
  }

  // Big cube on top.
  let cube_len = stick_len * 2.0F
  let cube = bodies.insert(
    @dynamics.RigidBodyBuilder3D::dynamic()
    .translation(
      @core.Vec3::new(
        0.0F,
        cube_len / 2.0F +
        (Float::from_int(num_levels) - 0.25F) * stick_rad * 4.0F,
        0.0F,
      ),
    )
    .can_sleep(false)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(
      cube_len / 2.0F,
      cube_len / 2.0F,
      cube_len / 2.0F,
    ).build(),
    cube,
    bodies,
  )
  |> ignore
  for _ in 0..<240 {
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    )
  }
  if bodies.get(cube) is Some(rb) {
    let p = rb.translation()
    inspect(p.x == p.x && p.y == p.y && p.z == p.z, content="true")
    inspect(p.y > -50.0F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "examples3d/debug_eccentric_boxes3.rs parity (3DReal headless; trimesh offset)" {
  // Ported from `rapier-reference/examples3d/debug_eccentric_boxes3.rs` (headless; adapted).
  //
  // The upstream uses a convex mesh with vertices far from the origin; we approximate with a
  // trimesh cuboid whose vertices are shifted by a large offset.
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

  // Trimesh cuboid with vertices far from the local origin.
  let he = 1.0F
  let vtx = [
    @core.Vec3::new(-he, -he, -he),
    @core.Vec3::new(he, -he, -he),
    @core.Vec3::new(he, -he, he),
    @core.Vec3::new(-he, -he, he),
    @core.Vec3::new(-he, he, -he),
    @core.Vec3::new(he, he, -he),
    @core.Vec3::new(he, he, he),
    @core.Vec3::new(-he, he, he),
  ]
  let off = @core.Vec3::new(100.0F, 100.0F, 100.0F)
  for i in 0..<vtx.length() {
    vtx[i] = vtx[i].add(off)
  }
  let idx = [
    (0, 2, 1),
    (0, 3, 2),
    (4, 5, 6),
    (4, 6, 7),
    (0, 4, 7),
    (0, 7, 3),
    (1, 6, 5),
    (1, 2, 6),
    (3, 7, 2),
    (2, 7, 6),
    (0, 1, 5),
    (0, 5, 4),
  ]
  for _ in 0..<2 {
    let rb = bodies.insert(
      @dynamics.RigidBodyBuilder3D::dynamic()
      .translation(@core.Vec3::new(-100.0F, -90.0F, -100.0F))
      .can_sleep(false)
      .build(),
    )
    let mesh = @collision.ColliderBuilder3D::trimesh(vtx, idx).unwrap().build()
    colliders.insert_with_parent(mesh, rb, bodies) |> ignore
  }
  for _ in 0..<240 {
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    )
  }
  for h in islands.active_bodies() {
    if bodies.get(h) is Some(rb) {
      let p = rb.translation()
      inspect(p.x == p.x && p.y == p.y && p.z == p.z, content="true")
    }
  }
}

///|
test "examples3d/debug_thin_cube_on_mesh3.rs parity (3DReal headless)" {
  // Ported from `rapier-reference/examples3d/debug_thin_cube_on_mesh3.rs` (headless; adapted).
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

  // Heightfield ground (2x2 flat).
  let heights = [0.0F, 0.0F, 0.0F, 0.0F]
  let hf = @collision.ColliderBuilder3D::heightfield(
      heights,
      2,
      2,
      @core.Vec3::new(50.0F, 1.0F, 50.0F),
    )
    .unwrap()
    .build()
  colliders.insert(hf) |> ignore
  let body = bodies.insert(
    @dynamics.RigidBodyBuilder3D::dynamic()
    .translation(@core.Vec3::new(0.0F, 5.0F, 0.0F))
    .rotation(
      @core.rotation_from_scaled_axis(@core.Vec3::new(0.5F, 0.0F, 0.5F)),
    )
    .linvel(@core.Vec3::new(0.0F, -100.0F, 0.0F))
    .soft_ccd_prediction(10.0F)
    .can_sleep(false)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(5.0F, 0.015F, 5.0F).build(),
    body,
    bodies,
  )
  |> ignore
  for _ in 0..<120 {
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    )
  }
  if bodies.get(body) is Some(rb) {
    let p = rb.translation()
    inspect(p.x == p.x && p.y == p.y && p.z == p.z, content="true")
    inspect(p.y > -100.0F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "examples3d/debug_trimesh3.rs parity (3DReal headless)" {
  // Ported from `rapier-reference/examples3d/debug_trimesh3.rs` (headless).
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
  let width = 0.5F
  let vtx = [
    @core.Vec3::new(-width, 0.0F, -width),
    @core.Vec3::new(width, 0.0F, -width),
    @core.Vec3::new(width, 0.0F, width),
    @core.Vec3::new(-width, 0.0F, width),
    @core.Vec3::new(-width, -width, -width),
    @core.Vec3::new(width, -width, -width),
    @core.Vec3::new(width, -width, width),
    @core.Vec3::new(-width, -width, width),
  ]
  let idx = [
    (0, 2, 1),
    (0, 3, 2),
    (4, 5, 6),
    (4, 6, 7),
    (0, 4, 7),
    (0, 7, 3),
    (1, 6, 5),
    (1, 2, 6),
    (3, 7, 2),
    (2, 7, 6),
    (0, 1, 5),
    (0, 5, 4),
  ]
  let dyn_body = bodies.insert(
    @dynamics.RigidBodyBuilder3D::dynamic()
    .translation(@core.Vec3::new(0.0F, 35.0F, 0.0F))
    .can_sleep(false)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(1.0F, 2.0F, 1.0F).build(),
    dyn_body,
    bodies,
  )
  |> ignore
  let ground = bodies.insert(@dynamics.RigidBodyBuilder3D::fixed().build())
  let mesh = @collision.ColliderBuilder3D::trimesh(vtx, idx).unwrap().build()
  colliders.insert_with_parent(mesh, ground, bodies) |> ignore
  for _ in 0..<360 {
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    )
  }
  if bodies.get(dyn_body) is Some(rb) {
    let p = rb.translation()
    inspect(p.x == p.x && p.y == p.y && p.z == p.z, content="true")
    inspect(p.y > -50.0F, content="true")
  } else {
    inspect(false, content="true")
  }
}
