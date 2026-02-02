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
/// Ported from `rapier-reference/src/geometry/narrow_phase.rs` issue #734 test, focusing on the
/// narrow-phase behavior:
/// colliders attached to the same body keep a contact pair but have no manifolds.
test "narrow phase 3d: same-parent colliders have empty manifolds; reparent generates contact" {
  let bodies = @dynamics.RigidBodySet3D::new()
  let colliders = ColliderSet3D::new()
  let body1 = bodies.insert(
    @dynamics.RigidBodyBuilder3D::dynamic()
    .translation(@core.Vec3::zero())
    .build(),
  )
  let ball = ColliderBuilder3D::ball(0.5F).build()
  let c1 = colliders.insert_with_parent(ball, body1, bodies)
  let c2 = colliders.insert_with_parent(
    ColliderBuilder3D::ball(0.5F).build(),
    body1,
    bodies,
  )
  let broad_phase = BroadPhase3D::new()
  broad_phase.update(0.0F, colliders)
  let nf = NarrowPhase3D::new()
  nf.update(broad_phase.pairs(), bodies, colliders)
  if nf.contact_pair(c1, c2) is Some(pair) {
    inspect(pair.manifolds_len() == 0, content="true")
  } else {
    inspect(false, content="true")
  }

  // Re-parent collider2 to a different body, and ensure we get a manifold.
  let body2 = bodies.insert(@dynamics.RigidBodyBuilder3D::dynamic().build())
  colliders.set_parent(c2, Some(body2), bodies)
  colliders.sync_with_bodies(bodies)
  broad_phase.update(0.0F, colliders)
  nf.update(broad_phase.pairs(), bodies, colliders)
  if nf.contact_pair(c1, c2) is Some(pair2) {
    inspect(pair2.manifolds_len() == 1, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "narrow phase 3d: collision groups filter disables contact manifolds" {
  let bodies = @dynamics.RigidBodySet3D::new()
  let colliders = ColliderSet3D::new()
  let b1 = bodies.insert(
    @dynamics.RigidBodyBuilder3D::dynamic()
    .translation(@core.Vec3::zero())
    .build(),
  )
  let b2 = bodies.insert(
    @dynamics.RigidBodyBuilder3D::dynamic()
    .translation(@core.Vec3::zero())
    .build(),
  )
  let g1 = @dynamics.InteractionGroups::new(
    @dynamics.Group::group_1(),
    @dynamics.Group::group_1(),
    @dynamics.InteractionTestMode::And,
  )
  let g2 = @dynamics.InteractionGroups::new(
    @dynamics.Group::group_2(),
    @dynamics.Group::group_2(),
    @dynamics.InteractionTestMode::And,
  )
  let c1 = colliders.insert_with_parent(
    ColliderBuilder3D::ball(0.5F).collision_groups(g1).build(),
    b1,
    bodies,
  )
  let c2 = colliders.insert_with_parent(
    ColliderBuilder3D::ball(0.5F).collision_groups(g2).build(),
    b2,
    bodies,
  )
  let broad_phase = BroadPhase3D::new()
  broad_phase.update(0.0F, colliders)
  let nf = NarrowPhase3D::new()
  nf.update(broad_phase.pairs(), bodies, colliders)
  if nf.contact_pair(c1, c2) is Some(pair) {
    inspect(pair.manifolds_len() == 0, content="true")
  } else {
    inspect(false, content="true")
  }
}
