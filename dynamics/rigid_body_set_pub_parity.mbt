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
}

///|
pub fn BodyPair::new(
  body1 : RigidBodyHandle,
  body2 : RigidBodyHandle,
) -> BodyPair {
  { body1, body2 }
}

///|
/// Placeholder for pub parity with Rapier.
///
/// Rapier uses this to update attached colliders after user-driven rigid-body position changes.
/// In this port, collider parenting lives in the `collision` package (which depends on `dynamics`),
/// so we expose the method without introducing a dependency cycle.
pub fn RigidBodySet::propagate_modified_body_positions_to_colliders(
  self : RigidBodySet,
) -> Unit {
  self |> ignore
}
