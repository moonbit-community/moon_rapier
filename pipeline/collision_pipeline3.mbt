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
/// 3D-flavored collision pipeline.
///
/// This is a thin wrapper around the existing 2D `CollisionPipeline` used to port the
/// rapier-reference dim3 unit tests.
pub struct CollisionPipeline3 {
  inner : CollisionPipeline

  fn new() -> CollisionPipeline3
}

///|
pub fn CollisionPipeline3::new() -> CollisionPipeline3 {
  { inner: CollisionPipeline::new() }
}

///|
pub fn CollisionPipeline3::step(
  self : CollisionPipeline3,
  prediction_distance : @core.Real,
  islands : @dynamics.IslandManager3,
  broad_phase : @collision.BroadPhaseBvh,
  narrow_phase : @collision.NarrowPhase,
  bodies : @dynamics.RigidBodySet3,
  colliders : @collision.ColliderSet3,
  hooks : PhysicsHooks,
  events : EventHandler,
) -> Unit {
  self |> ignore
  islands |> ignore
  events |> ignore
  broad_phase.update3(prediction_distance, bodies, colliders)
  let pairs = broad_phase.pairs()
  let filtered_pairs = []
  fn is_dynamic_collider(
    bodies : @dynamics.RigidBodySet3,
    collider : @collision.Collider,
  ) -> Bool {
    if collider.parent() is Some(parent) {
      if bodies.get(parent) is Some(body) {
        body.body_type() is @dynamics.RigidBodyType::Dynamic
      } else {
        false
      }
    } else {
      false
    }
  }

  for i in 0..<pairs.length() {
    let pair = pairs[i]
    let h1 = pair.0
    let h2 = pair.1
    let mut keep = true
    if colliders.get(h1) is Some(c1) {
      if colliders.get(h2) is Some(c2) {
        if c1.is_sensor() || c2.is_sensor() {
          let hook = @collision.ActiveHooks::filter_intersection_pair()
          if c1.active_hooks().contains(hook) ||
            c2.active_hooks().contains(hook) {
            keep = hooks.test_intersection_pair3(bodies, colliders, h1, h2)
          }
        } else {
          let hook = @collision.ActiveHooks::filter_contact_pairs()
          if !c1.active_hooks().contains(hook) &&
            !c2.active_hooks().contains(hook) {
            if !is_dynamic_collider(bodies, c1) &&
              !is_dynamic_collider(bodies, c2) {
              keep = false
            }
          }
          if c1.active_hooks().contains(hook) ||
            c2.active_hooks().contains(hook) {
            keep = hooks.test_contact_pair3(bodies, colliders, h1, h2)
          }
        }
      }
    }
    if keep {
      filtered_pairs.push(pair)
    }
  }
  narrow_phase.update_with_pairs3(
    bodies,
    colliders,
    prediction_distance,
    filtered_pairs,
    @dynamics.ImpulseJointSet3::new(),
    @dynamics.MultibodyJointSet3::new(),
  )
}
