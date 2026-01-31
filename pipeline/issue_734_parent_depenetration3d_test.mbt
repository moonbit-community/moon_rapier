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
/// Ported from `rapier-reference/src/geometry/narrow_phase.rs` issue #734 test.
test "issue #734: collider_set parent depenetration (3D)" {
  let pipeline = PhysicsPipeline3DReal::new()
  let broad_phase = @collision.BroadPhase3D::new()
  let narrow_phase = @collision.NarrowPhase3D::new()
  let bodies = @dynamics.RigidBodySet3D::new()
  let colliders = @collision.ColliderSet3D::new()
  let gravity = @core.Vec3::zero()
  let parameters = @dynamics.IntegrationParameters::default().set_dt(
    1.0F / 60.0F,
  )
  let body1 = bodies.insert(
    @dynamics.RigidBodyBuilder3D::dynamic()
    .translation(@core.Vec3::zero())
    .build(),
  )
  let c1 = colliders.insert_with_parent(
    @collision.ColliderBuilder3D::ball(0.5F).build(),
    body1,
    bodies,
  )
  let c2 = colliders.insert_with_parent(
    @collision.ColliderBuilder3D::ball(0.5F).build(),
    body1,
    bodies,
  )
  let body2 = bodies.insert(@dynamics.RigidBodyBuilder3D::dynamic().build())

  // Initial step: same-parent pair exists but no manifolds.
  pipeline.step(
    gravity, parameters, broad_phase, narrow_phase, bodies, colliders,
  )
  if colliders.get(c1) is Some(co1) && colliders.get(c2) is Some(co2) {
    let d = co1.position().translation.sub(co2.position().translation).length()
    inspect(d < 0.5F, content="true")
  } else {
    inspect(false, content="true")
  }
  if narrow_phase.contact_pair(c1, c2) is Some(pair0) {
    inspect(pair0.manifolds_len() == 0, content="true")
  } else {
    inspect(false, content="true")
  }

  // Re-parent collider2 to body2 and ensure we now have a manifold.
  colliders.set_parent(c2, Some(body2), bodies)
  pipeline.step(
    gravity, parameters, broad_phase, narrow_phase, bodies, colliders,
  )
  if narrow_phase.contact_pair(c1, c2) is Some(pair1) {
    inspect(pair1.manifolds_len() == 1, content="true")
  } else {
    inspect(false, content="true")
  }

  // Let the solver depenetrate.
  for _ in 0..<200 {
    pipeline.step(
      gravity, parameters, broad_phase, narrow_phase, bodies, colliders,
    )
  }
  if colliders.get(c1) is Some(co1) && colliders.get(c2) is Some(co2) {
    let d = co1.position().translation.sub(co2.position().translation).length()
    inspect(d > 0.9F, content="true")
  } else {
    inspect(false, content="true")
  }
}
