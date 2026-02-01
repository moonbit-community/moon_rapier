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
/// Heightfield parity tests for the real dim3 backend.

///|
test "examples3d/heightfield3.rs parity (3DReal headless; reduced counts)" {
  // Ported from `rapier-reference/examples3d/heightfield3.rs` (headless; reduced counts).
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

  // Ground heightfield.
  let ground_size = @core.Vec3::new(100.0F, 1.0F, 100.0F)
  let nsubdivs = 10
  let heights : Array[@core.Real] = []
  for i in 0..<(nsubdivs + 1) {
    for j in 0..<(nsubdivs + 1) {
      let h = if i == 0 || i == nsubdivs || j == 0 || j == nsubdivs {
        10.0F
      } else {
        let x = Float::from_int(i) * ground_size.x / Float::from_int(nsubdivs)
        let z = Float::from_int(j) * ground_size.z / Float::from_int(nsubdivs)
        Float::from_double(@math.sin(x.to_double())) +
        Float::from_double(@math.cos(z.to_double()))
      }
      heights.push(h)
    }
  }
  let ground = bodies.insert(@dynamics.RigidBodyBuilder3D::fixed().build())
  let hf = @collision.ColliderBuilder3D::heightfield(
      heights,
      nsubdivs + 1,
      nsubdivs + 1,
      ground_size,
    )
    .unwrap()
    .build()
  colliders.insert_with_parent(hf, ground, bodies) |> ignore

  // Spawn a reduced stack with mixed shapes.
  let num = 3
  let numy = 5
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
          // Stand-in for `ColliderBuilder::compound`: multiple colliders attached to one body.
          let c1 = @collision.ColliderBuilder3D::cuboid(
            rad,
            rad / 2.0F,
            rad / 2.0F,
          ).build()
          let c2 = @collision.ColliderBuilder3D::cuboid(
              rad / 2.0F,
              rad,
              rad / 2.0F,
            )
            .translation(@core.Vec3::new(rad, 0.0F, 0.0F))
            .build()
          let c3 = @collision.ColliderBuilder3D::cuboid(
              rad / 2.0F,
              rad,
              rad / 2.0F,
            )
            .translation(@core.Vec3::new(-rad, 0.0F, 0.0F))
            .build()
          colliders.insert_with_parent(c1, h, bodies) |> ignore
          colliders.insert_with_parent(c2, h, bodies) |> ignore
          colliders.insert_with_parent(c3, h, bodies) |> ignore
        }
      }
    }
  }
  for _ in 0..<360 {
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
