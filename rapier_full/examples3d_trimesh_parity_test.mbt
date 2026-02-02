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
fn run_examples3d_debug_trimesh3() -> Bool {
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

  // Trimesh "cube shell" (12 triangles).
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

  // Dynamic box rigid-body.
  let body = bodies.insert(
    @dynamics.RigidBodyBuilder3D::dynamic()
    .translation(@core.Vec3::new(0.0F, 35.0F, 0.0F))
    .can_sleep(false)
    .build(),
  )
  let box_h = colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(1.0F, 2.0F, 1.0F).build(),
    body,
    bodies,
  )
  let mut found_contact = false
  for _ in 0..<480 {
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    )
    if narrow_phase.contact_pair(box_h, mesh_h) is Some(pair) &&
      pair.manifolds_len() > 0 {
      found_contact = true
      break
    }
  }
  if !found_contact {
    return false
  }
  if bodies.get(body) is Some(rb) {
    rb.translation().y > -10.0F
  } else {
    false
  }
}

///|
test "examples3d/debug_trimesh3.rs parity (headless)" {
  inspect(run_examples3d_debug_trimesh3(), content="true")
}

///|
fn run_examples3d_trimesh3() -> Bool {
  // Ported from `rapier-reference/examples3d/trimesh3.rs` (headless; reduced counts).
  let pipeline = @pipeline.PhysicsPipeline3DReal::new()
  let broad_phase = @collision.BroadPhase3D::new()
  let narrow_phase = @collision.NarrowPhase3D::new()
  let bodies = @dynamics.RigidBodySet3D::new()
  let colliders = @collision.ColliderSet3D::new()
  let islands = @dynamics.IslandManager3D::new()
  let gravity = @core.Vec3::new(0.0F, -9.81F, 0.0F)
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    8,
  )

  // Ground mesh built from a heightfield-like grid (same sin/cos pattern as rapier-reference).
  let nsubdivs = 10
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

  // Reduced stack.
  let num = 3
  let numy = 3
  let rad = 1.0F
  let shift = rad * 2.0F + rad
  let center = shift * Float::from_int(num / 2)
  let handles : Array[@dynamics.RigidBodyHandle] = []
  for j in 0..<numy {
    for i in 0..<num {
      for k in 0..<num {
        let x = Float::from_int(i) * shift - center
        let y = Float::from_int(j) * shift + shift / 2.0F + 3.0F
        let z = Float::from_int(k) * shift - center
        let h = bodies.insert(
          @dynamics.RigidBodyBuilder3D::dynamic()
          .translation(@core.Vec3::new(x, y, z))
          .build(),
        )
        handles.push(h)
        let m = j % 6
        if m == 0 {
          colliders.insert_with_parent(
            @collision.ColliderBuilder3D::cuboid(rad, rad, rad).build(),
            h,
            bodies,
          )
          |> ignore
        } else if m == 1 {
          colliders.insert_with_parent(
            @collision.ColliderBuilder3D::ball(rad).build(),
            h,
            bodies,
          )
          |> ignore
        } else if m == 2 {
          colliders.insert_with_parent(
            @collision.ColliderBuilder3D::round_cylinder(rad, rad, rad / 10.0F).build(),
            h,
            bodies,
          )
          |> ignore
        } else if m == 3 {
          colliders.insert_with_parent(
            @collision.ColliderBuilder3D::cone(rad, rad).build(),
            h,
            bodies,
          )
          |> ignore
        } else if m == 4 {
          colliders.insert_with_parent(
            @collision.ColliderBuilder3D::capsule_y(rad, rad).build(),
            h,
            bodies,
          )
          |> ignore
        } else {
          let compound = @collision.ColliderBuilder3D::compound([
              (
                @core.Isometry3::identity(),
                @collision.ColliderBuilder3D::cuboid(
                  rad,
                  rad / 2.0F,
                  rad / 2.0F,
                ),
              ),
              (
                @core.Isometry3::identity(),
                @collision.ColliderBuilder3D::cuboid(
                  rad / 2.0F,
                  rad,
                  rad / 2.0F,
                ).translation(@core.Vec3::new(rad, 0.0F, 0.0F)),
              ),
              (
                @core.Isometry3::identity(),
                @collision.ColliderBuilder3D::cuboid(
                  rad / 2.0F,
                  rad,
                  rad / 2.0F,
                ).translation(@core.Vec3::new(-rad, 0.0F, 0.0F)),
              ),
            ])
            .unwrap()
            .build()
          colliders.insert_with_parent(compound, h, bodies) |> ignore
        }
      }
    }
  }
  for _ in 0..<120 {
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
test "examples3d/trimesh3.rs parity (headless; reduced counts)" {
  inspect(run_examples3d_trimesh3(), content="true")
}

///|
test "examples3d/debug_thin_cube_on_mesh3.rs parity (headless; trimesh ground)" {
  // Ported from `rapier-reference/examples3d/debug_thin_cube_on_mesh3.rs` (headless).
  //
  // The upstream example uses a heightfield. For the real dim3 backend we use a flat trimesh
  // ground to validate thin cuboid stability with fast velocities.
  let pipeline = @pipeline.PhysicsPipeline3DReal::new()
  let broad_phase = @collision.BroadPhase3D::new()
  let narrow_phase = @collision.NarrowPhase3D::new()
  let bodies = @dynamics.RigidBodySet3D::new()
  let colliders = @collision.ColliderSet3D::new()
  let islands = @dynamics.IslandManager3D::new()
  let gravity = @core.Vec3::new(0.0F, -9.81F, 0.0F)
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    8,
  )

  // Flat heightfield (2x2) like the upstream example.
  let heights = [0.0F, 0.0F, 0.0F, 0.0F]
  let ground = bodies.insert(@dynamics.RigidBodyBuilder3D::fixed().build())
  let hf = @collision.ColliderBuilder3D::heightfield(
      heights,
      2,
      2,
      @core.Vec3::new(50.0F, 1.0F, 50.0F),
    )
    .unwrap()
    .build()
  let mesh_h = colliders.insert_with_parent(hf, ground, bodies)

  // Thin cuboid.
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
  let co = colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(5.0F, 0.015F, 5.0F).build(),
    body,
    bodies,
  )
  let mut saw_contact = false
  for _ in 0..<180 {
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    )
    if narrow_phase.contact_pair(co, mesh_h) is Some(pair) &&
      pair.manifolds_len() > 0 {
      saw_contact = true
    }
  }
  inspect(saw_contact, content="true")
  if bodies.get(body) is Some(rb) {
    let p = rb.translation()
    inspect(p.x == p.x && p.y == p.y && p.z == p.z, content="true")
    inspect(p.y > -5.0F, content="true")
  } else {
    inspect(false, content="true")
  }
}
