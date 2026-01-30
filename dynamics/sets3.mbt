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
/// 3D-flavored island manager.
pub struct IslandManager3 {
  inner : IslandManager
}

///|
pub fn IslandManager3::new() -> IslandManager3 {
  { inner: IslandManager::new() }
}

///|
pub fn IslandManager3::as_2d(self : IslandManager3) -> IslandManager {
  self.inner
}

///|
/// 3D-flavored rigid-body set.
pub struct RigidBodySet3 {
  inner : RigidBodySet
}

///|
pub fn RigidBodySet3::new() -> RigidBodySet3 {
  { inner: RigidBodySet::new() }
}

///|
pub fn RigidBodySet3::as_2d(self : RigidBodySet3) -> RigidBodySet {
  self.inner
}

///|
pub fn RigidBodySet3::insert(
  self : RigidBodySet3,
  body : RigidBody,
) -> RigidBodyHandle {
  self.inner.insert(body)
}

///|
pub fn RigidBodySet3::remove(
  self : RigidBodySet3,
  handle : RigidBodyHandle,
  islands : IslandManager3,
  colliders : Unit,
  impulse_joints : ImpulseJointSet3,
  multibody_joints : MultibodyJointSet3,
  wake_up : Bool,
) -> RigidBody? {
  colliders |> ignore
  self.inner.remove(
    handle,
    islands.as_2d(),
    (),
    impulse_joints.as_2d(),
    multibody_joints.as_2d(),
    wake_up,
  )
}

///|
pub fn RigidBodySet3::get(
  self : RigidBodySet3,
  handle : RigidBodyHandle,
) -> RigidBody? {
  self.inner.get(handle)
}

///|
pub fn RigidBodySet3::get_mut(
  self : RigidBodySet3,
  handle : RigidBodyHandle,
) -> RigidBody? {
  self.inner.get_mut(handle)
}

///|
pub fn RigidBodySet3::translation(
  self : RigidBodySet3,
  handle : RigidBodyHandle,
) -> @core.Vec3 {
  if self.inner.get(handle) is Some(body) {
    let t2 = body.translation()
    @core.Vec3::new(t2.x, t2.y, 0.0F)
  } else {
    @core.Vec3::zero()
  }
}

///|
pub fn RigidBodySet3::rotation(
  self : RigidBodySet3,
  handle : RigidBodyHandle,
) -> @core.Quat {
  if self.inner.get(handle) is Some(body) {
    let r2 = body.rotation()
    // Map 2D angle to a quaternion around +Z.
    @core.rotation_from_scaled_axis(@core.Vec3::new(0.0F, 0.0F, r2.angle()))
  } else {
    @core.Quat::identity()
  }
}
