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
/// Trimesh parity tests (subset) for the real dim3 backend.

///|
fn run_examples3d_debug_trimesh3_ball() -> Bool {
  // Ported from `rapier-reference/examples3d/debug_trimesh3.rs` (headless; ball-only).
  //
  // We validate that a ball collides with a simple trimesh "cube shell" instead of tunneling
  // through it. Cuboid-vs-trimesh contact will be added later.
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

  // Trimesh "cube" (12 triangles).
  let w = 0.5F
  let vtx = [
    @core.Vec3::new(-w, 0.0F, -w),
    @core.Vec3::new(w, 0.0F, -w),
    @core.Vec3::new(w, 0.0F, w),
    @core.Vec3::new(-w, 0.0F, w),
    @core.Vec3::new(-w, -w, -w),
    @core.Vec3::new(w, -w, -w),
    @core.Vec3::new(w, -w, w),
    @core.Vec3::new(-w, -w, w),
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
  let ground = bodies.insert(@dynamics.RigidBodyBuilder3D::fixed().build())
  let mesh = @collision.ColliderBuilder3D::trimesh(vtx, idx).unwrap().build()
  let mesh_h = colliders.insert_with_parent(mesh, ground, bodies)

  // Falling ball.
  let ball = bodies.insert(
    @dynamics.RigidBodyBuilder3D::dynamic()
    .translation(@core.Vec3::new(0.0F, 5.0F, 0.0F))
    .build(),
  )
  let ball_h = colliders.insert_with_parent(
    @collision.ColliderBuilder3D::ball(0.25F).build(),
    ball,
    bodies,
  )
  let mut found_contact = false
  for _ in 0..<240 {
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    )
    if narrow_phase.contact_pair(ball_h, mesh_h) is Some(pair) &&
      pair.manifolds_len() > 0 {
      found_contact = true
    }
    if found_contact {
      break
    }
  }
  if !found_contact {
    return false
  }
  if bodies.get(ball) is Some(rb) {
    rb.translation().y > -10.0F
  } else {
    false
  }
}

///|
test "examples3d/debug_trimesh3.rs parity (ball-only headless)" {
  inspect(run_examples3d_debug_trimesh3_ball(), content="true")
}

///|
fn run_examples3d_trimesh3_balls_only() -> Bool {
  // Adapted from `rapier-reference/examples3d/trimesh3.rs` (headless; balls only).
  //
  // This validates a larger trimesh built from a heightfield-like grid.
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
  let nsubdivs = 20
  let size_x = 100.0F
  let size_z = 100.0F
  let dx = size_x / Float::from_int(nsubdivs)
  let dz = size_z / Float::from_int(nsubdivs)
  let origin_x = -size_x / 2.0F
  let origin_z = -size_z / 2.0F
  let vertices : Array[@core.Vec3] = []
  for i in 0..<(nsubdivs + 1) {
    for j in 0..<(nsubdivs + 1) {
      let x = origin_x + Float::from_int(i) * dx
      let z = origin_z + Float::from_int(j) * dz
      let y = if i == 0 || j == 0 || i == nsubdivs || j == nsubdivs {
        10.0F
      } else {
        Float::from_double(@math.sin(x.to_double())) +
        Float::from_double(@math.cos(z.to_double()))
      }
      vertices.push(@core.Vec3::new(x, y, z))
    }
  }
  fn vid(i : Int, j : Int, nsubdivs : Int) -> Int {
    i * (nsubdivs + 1) + j
  }

  let indices : Array[(Int, Int, Int)] = []
  for i in 0..<nsubdivs {
    for j in 0..<nsubdivs {
      let a = vid(i, j, nsubdivs)
      let b = vid(i + 1, j, nsubdivs)
      let c = vid(i + 1, j + 1, nsubdivs)
      let d = vid(i, j + 1, nsubdivs)
      indices.push((a, c, b))
      indices.push((a, d, c))
    }
  }
  let ground = bodies.insert(@dynamics.RigidBodyBuilder3D::fixed().build())
  let mesh = @collision.ColliderBuilder3D::trimesh(vertices, indices)
    .unwrap()
    .build()
  colliders.insert_with_parent(mesh, ground, bodies) |> ignore

  // Drop a few balls over the mesh.
  let rad = 1.0F
  let handles : Array[@dynamics.RigidBodyHandle] = []
  for i in 0..<5 {
    for j in 0..<5 {
      let x = -20.0F + Float::from_int(i) * 10.0F
      let z = -20.0F + Float::from_int(j) * 10.0F
      let h = bodies.insert(
        @dynamics.RigidBodyBuilder3D::dynamic()
        .translation(@core.Vec3::new(x, 30.0F, z))
        .build(),
      )
      handles.push(h)
      colliders.insert_with_parent(
        @collision.ColliderBuilder3D::ball(rad).build(),
        h,
        bodies,
      )
      |> ignore
    }
  }
  for _ in 0..<600 {
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    )
  }
  for h in handles {
    if bodies.get(h) is Some(rb) {
      let p = rb.translation()
      if !(p.x == p.x) || !(p.y == p.y) || !(p.z == p.z) {
        return false
      }
      if p.y < -100.0F {
        return false
      }
    } else {
      return false
    }
  }
  true
}

///|
test "examples3d/trimesh3.rs parity (balls-only headless)" {
  inspect(run_examples3d_trimesh3_balls_only(), content="true")
}
