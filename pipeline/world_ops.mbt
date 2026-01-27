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
/// Remove a rigid-body like Rapier's `RigidBodySet::remove`, but without introducing a dependency
/// from `dynamics` to `collision` (those packages depend on each other in this port).
pub fn remove_rigid_body(
  bodies : @dynamics.RigidBodySet,
  handle : @dynamics.RigidBodyHandle,
  islands : @dynamics.IslandManager,
  colliders : @collision.ColliderSet,
  impulse_joints : @dynamics.ImpulseJointSet,
  multibody_joints : @dynamics.MultibodyJointSet,
  remove_attached_colliders : Bool,
  wake_up : Bool,
) -> @dynamics.RigidBody? {
  if remove_attached_colliders {
    let attached = colliders.colliders_with_parent(handle)
    for i in 0..<attached.length() {
      colliders.remove(attached[i], islands, bodies, wake_up)
    }
  }
  bodies.remove(
    handle,
    islands,
    (),
    impulse_joints,
    multibody_joints,
    remove_attached_colliders,
  )
}
