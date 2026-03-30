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
/// Parity tests for convex hull/decomposition examples against the real dim3 backend.

///|
test "examples3d/convex_polyhedron3.rs parity (3DReal headless)" {
  // Ported from `rapier-reference/examples3d/convex_polyhedron3.rs` (headless).
  fn rng_next(seed : Int) -> Int {
    (seed * 1103515245 + 12345) & 0x7fffffff
  }

  fn rng_float01(seed : Int) -> (@core.Real, Int) {
    let next = rng_next(seed)
    let value = Float::from_double(next.to_double() / 2147483647.0)
    (value, next)
  }

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
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder3D::fixed()
    .translation(@core.Vec3::new(0.0F, -0.1F, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(40.0F, 0.1F, 40.0F).build(),
    ground,
    bodies,
  )
  |> ignore
  let num = 4
  let scale = 2.0F
  let border_rad = 0.1F
  let shift = border_rad * 2.0F + scale
  let centerx = shift * Float::from_int(num / 2)
  let centery = shift / 2.0F
  let centerz = shift * Float::from_int(num / 2)
  let handles : Array[@dynamics.RigidBodyHandle] = []
  let mut seed = 0
  for j in 0..<6 {
    for i in 0..<num {
      for k in 0..<num {
        let x = Float::from_int(i) * shift - centerx
        let y = Float::from_int(j) * shift + centery + 3.0F
        let z = Float::from_int(k) * shift - centerz
        let points : Array[@core.Vec3] = []
        for _ in 0..<10 {
          let (rx, s1) = rng_float01(seed)
          let (ry, s2) = rng_float01(s1)
          let (rz, s3) = rng_float01(s2)
          seed = s3
          points.push(@core.Vec3::new(rx * scale, ry * scale, rz * scale))
        }
        let h = bodies.insert(
          @dynamics.RigidBodyBuilder3D::dynamic()
          .translation(@core.Vec3::new(x, y, z))
          .build(),
        )
        handles.push(h)
        if @collision.ColliderBuilder3D::round_convex_hull(points, border_rad)
          is Some(builder) {
          colliders.insert_with_parent(builder.build(), h, bodies) |> ignore
        } else {
          inspect(false, content="true")
        }
      }
    }
  }
  for _ in 0..<60 {
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    )
  }
  for h in handles {
    if bodies.get(h) is Some(rb) {
      let p = rb.translation()
      inspect(p.x == p.x && p.y == p.y && p.z == p.z, content="true")
      inspect(p.y > -100.0F, content="true")
    } else {
      inspect(false, content="true")
    }
  }
}

///|
test "examples3d/convex_decomposition3.rs parity (3DReal headless)" {
  // Ported from `rapier-reference/examples3d/convex_decomposition3.rs` (headless).
  //
  // The upstream example loads a number of OBJ meshes and applies convex decomposition.
  // This port runs a deterministic headless scenario exercising convex decomposition on a small
  // concave mesh and ensures the simulation stays stable.
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
  let ground = bodies.insert(@dynamics.RigidBodyBuilder3D::fixed().build())
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(50.0F, 0.1F, 50.0F)
    .translation(@core.Vec3::new(0.0F, -0.1F, 0.0F))
    .build(),
    ground,
    bodies,
  )
  |> ignore

  // A tiny concave "U" mesh in the XZ plane, extruded slightly in Y.
  let v : Array[@core.Vec3] = [
    @core.Vec3::new(-1.0F, 0.0F, -1.0F),
    @core.Vec3::new(1.0F, 0.0F, -1.0F),
    @core.Vec3::new(1.0F, 0.0F, 1.0F),
    @core.Vec3::new(0.4F, 0.0F, 1.0F),
    @core.Vec3::new(0.4F, 0.0F, -0.2F),
    @core.Vec3::new(-0.4F, 0.0F, -0.2F),
    @core.Vec3::new(-0.4F, 0.0F, 1.0F),
    @core.Vec3::new(-1.0F, 0.0F, 1.0F),
  ]
  let idx : Array[(Int, Int, Int)] = [
    (0, 1, 5),
    (1, 4, 5),
    (1, 2, 4),
    (2, 3, 4),
    (0, 6, 7),
    (0, 5, 6),
  ]
  let handles : Array[@dynamics.RigidBodyHandle] = []
  for i in 0..<8 {
    let x = Float::from_int(i % 4) * 3.0F - 4.5F
    let z = Float::from_int(i / 4) * 3.0F - 1.5F
    let h = bodies.insert(
      @dynamics.RigidBodyBuilder3D::dynamic()
      .translation(@core.Vec3::new(x, 4.0F, z))
      .can_sleep(false)
      .build(),
    )
    handles.push(h)
    if @collision.ColliderBuilder3D::convex_decomposition(v, idx)
      is Some(builder) {
      colliders.insert_with_parent(builder.build(), h, bodies) |> ignore
    } else {
      inspect(false, content="true")
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
      inspect(p.x == p.x && p.y == p.y && p.z == p.z, content="true")
      inspect(p.y > -100.0F, content="true")
    } else {
      inspect(false, content="true")
    }
  }
}
