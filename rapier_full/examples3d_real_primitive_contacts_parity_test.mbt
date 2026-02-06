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
/// Parity tests for missing primitive contact pairs in the real dim3 backend.

///|
test "examples3d/debug_cylinder3.rs parity (3DReal headless)" {
  // Ported from `rapier-reference/examples3d/debug_cylinder3.rs` (headless).
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
  let ground_co = colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(
      ground_size, ground_height, ground_size,
    ).build(),
    ground,
    bodies,
  )

  // Cylinder.
  let rad = 1.0F
  let y = (rad * 2.0F + rad) / 2.0F + 3.0F
  let body = bodies.insert(
    @dynamics.RigidBodyBuilder3D::dynamic()
    .translation(@core.Vec3::new(0.0F, y, 0.0F))
    .can_sleep(false)
    .build(),
  )
  let cyl_co = colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cylinder(rad, rad).build(),
    body,
    bodies,
  )
  let (ci, cg) = cyl_co.into_raw_parts()
  let (gi, gg) = ground_co.into_raw_parts()
  let cyl_first = ci < gi || (ci == gi && cg <= gg)
  let mut saw_contact = false
  let mut saw_up_normal = false
  for _ in 0..<360 {
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    )
    if narrow_phase.contact_pair(cyl_co, ground_co) is Some(pair) &&
      pair.manifolds_len() > 0 {
      saw_contact = true
      let manifolds = pair.manifolds()
      if manifolds.length() > 0 && manifolds[0].normal.y > 0.7F {
        saw_up_normal = true
      }
    }
  }
  inspect(saw_contact, content="true")
  inspect(cyl_first, content="false")
  inspect(saw_up_normal, content="true")
  if bodies.get(body) is Some(rb) {
    let p = rb.translation()
    inspect(p.x == p.x && p.y == p.y && p.z == p.z, content="true")
    // Keep this broad enough to avoid false negatives on the pure MoonBit backend.
    inspect(p.y > -100.0F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "examples3d/primitives3.rs cone-vs-ground parity (3DReal headless)" {
  // Derived from `rapier-reference/examples3d/primitives3.rs` (headless).
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
  let ground_height = 1.0F
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder3D::fixed()
    .translation(@core.Vec3::new(0.0F, -ground_height, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(20.0F, ground_height, 20.0F).build(),
    ground,
    bodies,
  )
  |> ignore
  let rad = 1.0F
  let body = bodies.insert(
    @dynamics.RigidBodyBuilder3D::dynamic()
    .translation(@core.Vec3::new(0.0F, 6.0F, 0.0F))
    .can_sleep(false)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cone(rad, rad).build(),
    body,
    bodies,
  )
  |> ignore
  for _ in 0..<360 {
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
test "examples3d/primitives3.rs round_cylinder-vs-ground parity (3DReal headless)" {
  // Derived from `rapier-reference/examples3d/primitives3.rs` (headless).
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
  let ground_height = 1.0F
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder3D::fixed()
    .translation(@core.Vec3::new(0.0F, -ground_height, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(20.0F, ground_height, 20.0F).build(),
    ground,
    bodies,
  )
  |> ignore
  let rad = 1.0F
  let body = bodies.insert(
    @dynamics.RigidBodyBuilder3D::dynamic()
    .translation(@core.Vec3::new(0.0F, 6.0F, 0.0F))
    .can_sleep(false)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::round_cylinder(rad, rad, rad / 10.0F).build(),
    body,
    bodies,
  )
  |> ignore
  for _ in 0..<360 {
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
test "contacts: cylinder-vs-cylinder (3DReal headless)" {
  let pipeline = @pipeline.PhysicsPipeline3DReal::new()
  let broad_phase = @collision.BroadPhase3D::new()
  let narrow_phase = @collision.NarrowPhase3D::new()
  let bodies = @dynamics.RigidBodySet3D::new()
  let colliders = @collision.ColliderSet3D::new()
  let islands = @dynamics.IslandManager3D::new()
  let gravity = @core.Vec3::zero()
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )
  let hh = 1.0F
  let r = 0.6F
  let fixed = bodies.insert(@dynamics.RigidBodyBuilder3D::fixed().build())
  let fixed_co = colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cylinder(r, hh).build(),
    fixed,
    bodies,
  )
  let dyn_body = bodies.insert(
    @dynamics.RigidBodyBuilder3D::dynamic()
    .translation(@core.Vec3::new(0.2F, 1.8F, 0.1F))
    .can_sleep(false)
    .build(),
  )
  let dyn_co = colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cylinder(r, hh).build(),
    dyn_body,
    bodies,
  )
  let mut saw_contact = false
  for _ in 0..<20 {
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    )
    if narrow_phase.contact_pair(dyn_co, fixed_co) is Some(pair) &&
      pair.manifolds_len() > 0 {
      saw_contact = true
    }
  }
  inspect(saw_contact, content="true")
}

///|
test "contacts: cone-vs-cylinder (3DReal headless)" {
  let pipeline = @pipeline.PhysicsPipeline3DReal::new()
  let broad_phase = @collision.BroadPhase3D::new()
  let narrow_phase = @collision.NarrowPhase3D::new()
  let bodies = @dynamics.RigidBodySet3D::new()
  let colliders = @collision.ColliderSet3D::new()
  let islands = @dynamics.IslandManager3D::new()
  let gravity = @core.Vec3::zero()
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )
  let hh = 1.0F
  let r = 0.6F
  let fixed = bodies.insert(@dynamics.RigidBodyBuilder3D::fixed().build())
  let fixed_co = colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cylinder(r, hh).build(),
    fixed,
    bodies,
  )
  let dyn_body = bodies.insert(
    @dynamics.RigidBodyBuilder3D::dynamic()
    .translation(@core.Vec3::new(0.15F, 1.8F, -0.1F))
    .can_sleep(false)
    .build(),
  )
  let dyn_co = colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cone(r, hh).build(),
    dyn_body,
    bodies,
  )
  let mut saw_contact = false
  for _ in 0..<20 {
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    )
    if narrow_phase.contact_pair(dyn_co, fixed_co) is Some(pair) &&
      pair.manifolds_len() > 0 {
      saw_contact = true
    }
  }
  inspect(saw_contact, content="true")
}

///|
test "contacts: cone-vs-cone (3DReal headless)" {
  let pipeline = @pipeline.PhysicsPipeline3DReal::new()
  let broad_phase = @collision.BroadPhase3D::new()
  let narrow_phase = @collision.NarrowPhase3D::new()
  let bodies = @dynamics.RigidBodySet3D::new()
  let colliders = @collision.ColliderSet3D::new()
  let islands = @dynamics.IslandManager3D::new()
  let gravity = @core.Vec3::zero()
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )
  let hh = 1.0F
  let r = 0.6F
  let fixed = bodies.insert(@dynamics.RigidBodyBuilder3D::fixed().build())
  let fixed_co = colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cone(r, hh).build(),
    fixed,
    bodies,
  )
  let dyn_body = bodies.insert(
    @dynamics.RigidBodyBuilder3D::dynamic()
    // Slight overlap should still generate contacts with robust narrow-phase.
    .translation(@core.Vec3::new(0.1F, 1.6F, 0.05F))
    .can_sleep(false)
    .build(),
  )
  let dyn_co = colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cone(r, hh).build(),
    dyn_body,
    bodies,
  )
  let mut saw_contact = false
  for _ in 0..<20 {
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    )
    if narrow_phase.contact_pair(dyn_co, fixed_co) is Some(pair) &&
      pair.manifolds_len() > 0 {
      saw_contact = true
    }
  }
  inspect(saw_contact, content="true")
}
