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
  // Only the dynamic root contributes the extra 3 DOFs in 3D.
  if self.inner.root_is_dynamic {
    self.inner.ndofs() + 3
  } else {
    self.inner.ndofs()
  }
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

///|
pub fn MultibodyJointSet3::get_mut(
  self : MultibodyJointSet3,
  handle : MultibodyJointHandle,
) -> (Multibody3, LinkId)? {
  if self.inner.get_mut(handle) is Some((mb, id)) {
    Some((Multibody3::{ inner: mb }, id))
  } else {
    None
  }
}

///|
pub struct Jacobian3 {
  inner : Jacobian
}

///|
pub fn Jacobian3::zeros(size : Int) -> Jacobian3 {
  { inner: Jacobian::zeros(size) }
}

///|
pub fn Jacobian3::cols(self : Jacobian3) -> Int {
  self.inner.cols()
}

///|
pub fn Jacobian3::value_at(
  self : Jacobian3,
  row : Int,
  col : Int,
) -> @core.Real {
  // Map the 2D (3xN) jacobian to a 3D (6xN) one by zero-padding:
  // [lin.x, lin.y, lin.z=0, ang.x=0, ang.y=0, ang.z].
  if row == 0 {
    self.inner.value_at(0, col)
  } else if row == 1 {
    self.inner.value_at(1, col)
  } else if row == 5 {
    self.inner.value_at(2, col)
  } else {
    0.0F
  }
}

///|
fn iso3_from_iso2(iso : @core.Isometry2) -> @core.Isometry3 {
  @core.Isometry3::new(
    @core.Vec3::new(iso.translation.x, iso.translation.y, 0.0F),
    @core.rotation_from_scaled_axis(
      @core.Vec3::new(0.0F, 0.0F, iso.rotation.angle()),
    ),
  )
}

///|
pub fn Multibody3::forward_kinematics(
  self : Multibody3,
  bodies : RigidBodySet3,
  update_dofs : Bool,
) -> Unit {
  self.inner.forward_kinematics(bodies.as_2d(), update_dofs)
}

///|
pub fn Multibody3::apply_displacements(
  self : Multibody3,
  displacements : Array[@core.Real],
) -> Unit {
  self.inner.apply_displacements(displacements)
}

///|
pub fn Multibody3::link_pose(
  self : Multibody3,
  id : LinkId,
) -> @core.Isometry3? {
  if self.inner.link(id) is Some(link) {
    Some(iso3_from_iso2(link.local_to_world()))
  } else {
    None
  }
}

///|
pub fn Multibody3::body_jacobian(self : Multibody3, id : LinkId) -> Jacobian3 {
  { inner: self.inner.body_jacobian(id) }
}

///|
pub fn Multibody3::forward_kinematics_single_link(
  self : Multibody3,
  bodies : RigidBodySet3,
  id : LinkId,
  displacements : Array[@core.Real]?,
  jacobian : Jacobian3?,
) -> @core.Isometry3 {
  let jac2 : Jacobian? = if jacobian is Some(j) { Some(j.inner) } else { None }
  iso3_from_iso2(
    self.inner.forward_kinematics_single_link(
      bodies.as_2d(),
      id,
      displacements,
      jac2,
    ),
  )
}
