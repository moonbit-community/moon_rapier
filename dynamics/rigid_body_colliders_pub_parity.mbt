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
/// Public-surface parity for Rapier's `RigidBodyColliders` component.
///
/// We store collider handles as `(id, generation)` raw parts so this package stays
/// independent from `collision::ColliderHandle` while preserving full handle identity.
pub struct RigidBodyColliders {
  mut colliders : Array[(Int, Int)]

  fn new() -> RigidBodyColliders
}

///|
pub fn RigidBodyColliders::new() -> RigidBodyColliders {
  { colliders: [] }
}

///|
pub fn RigidBodyColliders::len(self : RigidBodyColliders) -> Int {
  self.colliders.length()
}

///|
pub fn RigidBodyColliders::handles(
  self : RigidBodyColliders,
) -> Array[(Int, Int)] {
  let out : Array[(Int, Int)] = []
  for i in 0..<self.colliders.length() {
    out.push(self.colliders[i])
  }
  out
}

///|
fn rigid_body_collider_handle_equals(
  left : (Int, Int),
  id : Int,
  generation : Int,
) -> Bool {
  left.0 == id && left.1 == generation
}

///|
pub fn RigidBodyColliders::attach_collider_raw(
  self : RigidBodyColliders,
  id : Int,
  generation : Int,
) -> RigidBodyColliders {
  for i in 0..<self.colliders.length() {
    if rigid_body_collider_handle_equals(self.colliders[i], id, generation) {
      return self
    }
  }
  self.colliders.push((id, generation))
  self
}

///|
pub fn RigidBodyColliders::detach_collider_raw(
  self : RigidBodyColliders,
  id : Int,
  generation : Int,
) -> RigidBodyColliders {
  let out : Array[(Int, Int)] = []
  for i in 0..<self.colliders.length() {
    let current = self.colliders[i]
    if !rigid_body_collider_handle_equals(current, id, generation) {
      out.push(current)
    }
  }
  self.colliders = out
  self
}

///|
/// Backward-compatible helper that attaches a raw collider id with generation `0`.
pub fn RigidBodyColliders::attach_collider(
  self : RigidBodyColliders,
  handle : Int,
) -> RigidBodyColliders {
  self.attach_collider_raw(handle, 0)
}

///|
/// Backward-compatible helper that detaches a raw collider id with generation `0`.
pub fn RigidBodyColliders::detach_collider(
  self : RigidBodyColliders,
  handle : Int,
) -> RigidBodyColliders {
  self.detach_collider_raw(handle, 0)
}
