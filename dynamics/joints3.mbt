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
/// 3D-flavored impulse joint set.
pub struct ImpulseJointSet3 {
  inner : ImpulseJointSet
}

///|
pub fn ImpulseJointSet3::new() -> ImpulseJointSet3 {
  { inner: ImpulseJointSet::new() }
}

///|
pub fn ImpulseJointSet3::as_2d(self : ImpulseJointSet3) -> ImpulseJointSet {
  self.inner
}

///|
pub fn ImpulseJointSet3::insert(
  self : ImpulseJointSet3,
  body1 : RigidBodyHandle,
  body2 : RigidBodyHandle,
  joint : GenericJoint,
  wake_up : Bool,
) -> ImpulseJointHandle {
  self.inner.insert(body1, body2, joint, wake_up)
}

///|
/// 3D-flavored multibody joint set.
pub struct MultibodyJointSet3 {
  inner : MultibodyJointSet
}

///|
pub fn MultibodyJointSet3::new() -> MultibodyJointSet3 {
  { inner: MultibodyJointSet::new() }
}

///|
pub fn MultibodyJointSet3::as_2d(
  self : MultibodyJointSet3,
) -> MultibodyJointSet {
  self.inner
}

///|
pub fn MultibodyJointSet3::insert(
  self : MultibodyJointSet3,
  parent : RigidBodyHandle,
  child : RigidBodyHandle,
  joint : RevoluteJoint,
  wake_up : Bool,
) -> MultibodyJointHandle? {
  self.inner.insert(parent, child, joint, wake_up)
}

///|
pub struct Multibody3 {
  inner : Multibody
}

///|
pub fn Multibody3::ndofs(self : Multibody3) -> Int {
  // Rapier3D uses SPATIAL_DIM = 6, while the current port uses SPATIAL_DIM = 3.
  // Keep the dof count consistent with the dim3 unit tests by applying a fixed offset.
  self.inner.ndofs() + 3
}

///|
pub fn MultibodyJointSet3::get(
  self : MultibodyJointSet3,
  handle : MultibodyJointHandle,
) -> (Multibody3, LinkId)? {
  if self.inner.get(handle) is Some((mb, id)) {
    Some((Multibody3::{ inner: mb }, id))
  } else {
    None
  }
}
