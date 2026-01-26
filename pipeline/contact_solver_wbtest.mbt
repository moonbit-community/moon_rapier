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
fn setup_overlapping_balls() -> (
  @dynamics.RigidBodySet,
  @collision.ColliderSet,
  @dynamics.IslandManager,
  @collision.BroadPhaseBvh,
  @collision.NarrowPhase,
  @dynamics.RigidBodyHandle,
  @dynamics.RigidBodyHandle,
) {
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = @collision.ColliderSet::new()
  let islands = @dynamics.IslandManager::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let b1 = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(0.0F, 0.0F))
    .build(),
  )
  let b2 = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(0.0F, 1.5F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::ball(1.0F).build(),
    b1,
    bodies,
  )
  |> ignore
  colliders.insert_with_parent(
    @collision.ColliderBuilder::ball(1.0F).build(),
    b2,
    bodies,
  )
  |> ignore

  // The solver relies on non-zero mass properties.
  update_body_mass_properties_from_colliders(bodies, colliders, [b1, b2])
  let impulse_joints = @dynamics.ImpulseJointSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()
  CollisionPipeline::new().step(
    0.0F,
    islands,
    broad_phase,
    narrow_phase,
    bodies,
    colliders,
    impulse_joints,
    multibody_joints,
    PhysicsHooks::new(),
    EventHandler::new(),
  )
  (bodies, colliders, islands, broad_phase, narrow_phase, b1, b2)
}

///|
test "contact solver warmstart uses cached impulses" {
  let (
    bodies_cold,
    colliders_cold,
    _islands_cold,
    _broad_phase_cold,
    narrow_phase_cold,
    b1_cold,
    b2_cold,
  ) = setup_overlapping_balls()
  let pairs_cold = narrow_phase_cold.contact_pairs()
  inspect(pairs_cold.length() == 1, content="true")
  let pair_cold = pairs_cold[0]
  let manifolds_cold = pair_cold.2.manifolds()
  inspect(manifolds_cold.length() > 0, content="true")
  let points_cold = manifolds_cold[0].points()
  inspect(points_cold.length() > 0, content="true")
  let p_cold = points_cold[0]
  let cache_cold : Array[ContactImpulseCacheEntry] = []
  solve_contacts(
    @dynamics.IntegrationParameters::default(),
    bodies_cold,
    colliders_cold,
    narrow_phase_cold,
    0.0F,
    EventHandler::new(),
    cache_cold,
  )
  if bodies_cold.get(b1_cold) is Some(body1) {
    inspect(body1.linvel().dot(body1.linvel()) <= 1.0e-12F, content="true")
    inspect(@core.abs(body1.angvel()) <= 1.0e-12F, content="true")
  } else {
    inspect(false, content="true")
  }
  if bodies_cold.get(b2_cold) is Some(body2) {
    inspect(body2.linvel().dot(body2.linvel()) <= 1.0e-12F, content="true")
    inspect(@core.abs(body2.angvel()) <= 1.0e-12F, content="true")
  } else {
    inspect(false, content="true")
  }
  let (
    bodies_warm,
    colliders_warm,
    _islands_warm,
    _broad_phase_warm,
    narrow_phase_warm,
    b1_warm,
    b2_warm,
  ) = setup_overlapping_balls()
  let pairs_warm = narrow_phase_warm.contact_pairs()
  inspect(pairs_warm.length() == 1, content="true")
  let pair_warm = pairs_warm[0]
  let manifolds_warm = pair_warm.2.manifolds()
  inspect(manifolds_warm.length() > 0, content="true")
  let points_warm = manifolds_warm[0].points()
  inspect(points_warm.length() > 0, content="true")
  let p_warm = points_warm[0]
  let cache_warm : Array[ContactImpulseCacheEntry] = []
  cache_warm.push(ContactImpulseCacheEntry::{
    collider1: pair_warm.0,
    collider2: pair_warm.1,
    fid1: p_warm.fid1(),
    fid2: p_warm.fid2(),
    impulse_n: 2.0F,
    impulse_t: 0.0F,
  })
  solve_contacts(
    @dynamics.IntegrationParameters::default(),
    bodies_warm,
    colliders_warm,
    narrow_phase_warm,
    0.0F,
    EventHandler::new(),
    cache_warm,
  )
  if bodies_warm.get(b1_warm) is Some(body1) {
    inspect(body1.linvel().dot(body1.linvel()) > 1.0e-9F, content="true")
  } else {
    inspect(false, content="true")
  }
  if bodies_warm.get(b2_warm) is Some(body2) {
    inspect(body2.linvel().dot(body2.linvel()) > 1.0e-9F, content="true")
  } else {
    inspect(false, content="true")
  }

  // Keep the compiler honest about using stable contact ids.
  p_cold |> ignore
  p_warm |> ignore
}
