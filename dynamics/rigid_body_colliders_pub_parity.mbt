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
/// Public-surface placeholder for Rapier's `RigidBodyColliders` component.
///
/// In Rapier, this stores the set of colliders attached to a rigid-body. In this port,
/// collider parenting is handled by the `collision` package (which depends on `dynamics`),
/// so we expose a minimal record here without introducing a dependency cycle.
pub struct RigidBodyColliders {
  // Collider handles are represented opaquely in this port's public dynamics surface.
  colliders : Array[Int]
}

///|
pub fn RigidBodyColliders::new() -> RigidBodyColliders {
  { colliders: [] }
}

///|
pub fn RigidBodyColliders::attach_collider(
  self : RigidBodyColliders,
  handle : Int,
) -> RigidBodyColliders {
  let out = { colliders: [] }
  for i in 0..<self.colliders.length() {
    out.colliders.push(self.colliders[i])
  }
  out.colliders.push(handle)
  out
}

///|
pub fn RigidBodyColliders::detach_collider(
  self : RigidBodyColliders,
  handle : Int,
) -> RigidBodyColliders {
  let out = { colliders: [] }
  for i in 0..<self.colliders.length() {
    if self.colliders[i] != handle {
      out.colliders.push(self.colliders[i])
    }
  }
  out
}
