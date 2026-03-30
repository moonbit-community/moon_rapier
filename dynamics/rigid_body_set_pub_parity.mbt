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
/// Pub-surface parity helpers for Rapier's `rigid_body_set` module.

///|
pub struct BodyPair {
  body1 : RigidBodyHandle
  body2 : RigidBodyHandle

  fn new(body1 : RigidBodyHandle, body2 : RigidBodyHandle) -> BodyPair
}

///|
pub fn BodyPair::new(
  body1 : RigidBodyHandle,
  body2 : RigidBodyHandle,
) -> BodyPair {
  { body1, body2 }
}

///|
/// Queue collider position propagation commands for all modified bodies whose position changed.
///
/// Because `dynamics` cannot depend on `collision`, this method records propagation commands
/// that can later be consumed by `collision` through
/// `RigidBodySet::take_pending_collider_position_updates`.
pub fn RigidBodySet::propagate_modified_body_positions_to_colliders(
  self : RigidBodySet,
) -> Unit {
  self.pending_collider_position_updates.clear()
  for i in 0..<self.modified_bodies.length() {
    let rb_handle = self.modified_bodies[i]
    if self.get(rb_handle) is Some(body) {
      if body.changes.contains(RIGID_BODY_CHANGES_POSITION) {
        let parent_pos = body.position()
        let attached = body.colliders()
        for j in 0..<attached.length() {
          let (collider_id, collider_generation) = attached[j]
          self.pending_collider_position_updates.push(
            (collider_id, collider_generation, parent_pos),
          )
        }
      }
    }
  }
}

///|
pub fn RigidBodySet::take_pending_collider_position_updates(
  self : RigidBodySet,
) -> Array[(Int, Int, @core.Isometry2)] {
  let out : Array[(Int, Int, @core.Isometry2)] = []
  for i in 0..<self.pending_collider_position_updates.length() {
    out.push(self.pending_collider_position_updates[i])
  }
  self.pending_collider_position_updates.clear()
  out
}
