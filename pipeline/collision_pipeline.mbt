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
pub struct CollisionPipeline {}

///|
pub fn CollisionPipeline::new() -> CollisionPipeline {
  CollisionPipeline::{  }
}

///|
pub fn CollisionPipeline::step(
  self : CollisionPipeline,
  prediction_distance : @core.Real,
  islands : @dynamics.IslandManager,
  broad_phase : @collision.BroadPhaseBvh,
  narrow_phase : @collision.NarrowPhase,
  bodies : @dynamics.RigidBodySet,
  colliders : @collision.ColliderSet,
  physics_hooks : PhysicsHooks,
  event_handler : EventHandler,
) -> Unit {
  self |> ignore
  islands |> ignore
  event_handler |> ignore
  broad_phase.update(prediction_distance, bodies, colliders)
  let pairs = broad_phase.pairs()
  let filtered_pairs = []
  fn is_dynamic_collider(
    bodies : @dynamics.RigidBodySet,
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
            keep = physics_hooks.test_intersection_pair(
              bodies, colliders, h1, h2,
            )
          }
        } else {
          let hook = @collision.ActiveHooks::filter_contact_pairs()
          // Match Rapier's default filtering: ignore contacts between two non-dynamic bodies
          // unless the pair filtering hook is enabled on at least one collider.
          if !c1.active_hooks().contains(hook) &&
            !c2.active_hooks().contains(hook) {
            if !is_dynamic_collider(bodies, c1) &&
              !is_dynamic_collider(bodies, c2) {
              keep = false
            }
          }
          if c1.active_hooks().contains(hook) ||
            c2.active_hooks().contains(hook) {
            keep = physics_hooks.test_contact_pair(bodies, colliders, h1, h2)
          }
        }
      }
    }
    if keep {
      filtered_pairs.push(pair)
    }
  }
  narrow_phase.update_with_pairs(bodies, colliders, filtered_pairs)
}
