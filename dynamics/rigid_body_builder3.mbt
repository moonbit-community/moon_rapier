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
/// Source-aligned dim3 rigid-body builder alias.
pub type RigidBodyBuilder3 = RigidBodyBuilder3D

///|
/// Legacy dim3 compatibility builder that projects to the historical 2D wrapper body.
pub struct RigidBodyBuilder3Compat {
  inner : RigidBodyBuilder
}

///|
pub fn RigidBodyBuilder3Compat::fixed() -> RigidBodyBuilder3Compat {
  { inner: RigidBodyBuilder::fixed() }
}

///|
pub fn RigidBodyBuilder3Compat::dynamic() -> RigidBodyBuilder3Compat {
  { inner: RigidBodyBuilder::dynamic() }
}

///|
pub fn RigidBodyBuilder3Compat::kinematic_position_based() -> RigidBodyBuilder3Compat {
  { inner: RigidBodyBuilder::kinematic_position_based() }
}

///|
pub fn RigidBodyBuilder3Compat::kinematic_velocity_based() -> RigidBodyBuilder3Compat {
  { inner: RigidBodyBuilder::kinematic_velocity_based() }
}

///|
pub fn RigidBodyBuilder3Compat::translation(
  self : RigidBodyBuilder3Compat,
  translation : @core.Vec3,
) -> RigidBodyBuilder3Compat {
  {
    inner: self.inner.translation(@core.Vec2::new(translation.x, translation.y)),
  }
}

///|
pub fn RigidBodyBuilder3Compat::rotation(
  self : RigidBodyBuilder3Compat,
  rotation : @core.Quat,
) -> RigidBodyBuilder3Compat {
  // 2D compatibility: keep only the rotation around +Z (yaw).
  let axis = rotation.to_scaled_axis()
  { inner: self.inner.rotation(@core.Rot2::from_angle(axis.z)) }
}

///|
pub fn RigidBodyBuilder3Compat::rotation_scaled_axis(
  self : RigidBodyBuilder3Compat,
  scaled_axis : @core.Vec3,
) -> RigidBodyBuilder3Compat {
  self.rotation(@core.rotation_from_scaled_axis(scaled_axis))
}

///|
pub fn RigidBodyBuilder3Compat::linvel(
  self : RigidBodyBuilder3Compat,
  vel : @core.Vec3,
) -> RigidBodyBuilder3Compat {
  { inner: self.inner.linvel(@core.Vec2::new(vel.x, vel.y)) }
}

///|
pub fn RigidBodyBuilder3Compat::angvel(
  self : RigidBodyBuilder3Compat,
  vel : @core.Vec3,
) -> RigidBodyBuilder3Compat {
  // 2D compatibility: keep only the Z component.
  { inner: self.inner.angvel(vel.z) }
}

///|
pub fn RigidBodyBuilder3Compat::gravity_scale(
  self : RigidBodyBuilder3Compat,
  scale : @core.Real,
) -> RigidBodyBuilder3Compat {
  { inner: self.inner.gravity_scale(scale) }
}

///|
pub fn RigidBodyBuilder3Compat::linear_damping(
  self : RigidBodyBuilder3Compat,
  damping : @core.Real,
) -> RigidBodyBuilder3Compat {
  { inner: self.inner.linear_damping(damping) }
}

///|
pub fn RigidBodyBuilder3Compat::angular_damping(
  self : RigidBodyBuilder3Compat,
  damping : @core.Real,
) -> RigidBodyBuilder3Compat {
  { inner: self.inner.angular_damping(damping) }
}

///|
pub fn RigidBodyBuilder3Compat::additional_mass(
  self : RigidBodyBuilder3Compat,
  mass : @core.Real,
) -> RigidBodyBuilder3Compat {
  { inner: self.inner.additional_mass(mass) }
}

///|
pub fn RigidBodyBuilder3Compat::additional_solver_iterations(
  self : RigidBodyBuilder3Compat,
  additional_iterations : Int,
) -> RigidBodyBuilder3Compat {
  { inner: self.inner.additional_solver_iterations(additional_iterations) }
}

///|
pub fn RigidBodyBuilder3Compat::ccd_enabled(
  self : RigidBodyBuilder3Compat,
  enabled : Bool,
) -> RigidBodyBuilder3Compat {
  { inner: self.inner.ccd_enabled(enabled) }
}

///|
pub fn RigidBodyBuilder3Compat::soft_ccd_prediction(
  self : RigidBodyBuilder3Compat,
  distance : @core.Real,
) -> RigidBodyBuilder3Compat {
  { inner: self.inner.soft_ccd_prediction(distance) }
}

///|
pub fn RigidBodyBuilder3Compat::lock_rotations(
  self : RigidBodyBuilder3Compat,
) -> RigidBodyBuilder3Compat {
  { inner: self.inner.lock_rotations() }
}

///|
pub fn RigidBodyBuilder3Compat::lock_translations(
  self : RigidBodyBuilder3Compat,
) -> RigidBodyBuilder3Compat {
  { inner: self.inner.lock_translations() }
}

///|
pub fn RigidBodyBuilder3Compat::enabled_rotations(
  self : RigidBodyBuilder3Compat,
  _x : Bool,
  _y : Bool,
  z : Bool,
) -> RigidBodyBuilder3Compat {
  // 2D compatibility: only the Z rotation exists.
  if z {
    self
  } else {
    self.lock_rotations()
  }
}

///|
pub fn RigidBodyBuilder3Compat::gyroscopic_forces_enabled(
  self : RigidBodyBuilder3Compat,
  _enabled : Bool,
) -> RigidBodyBuilder3Compat {
  // 2D compatibility: no gyroscopic effect.
  self
}

///|
pub fn RigidBodyBuilder3Compat::can_sleep(
  self : RigidBodyBuilder3Compat,
  enabled : Bool,
) -> RigidBodyBuilder3Compat {
  { inner: self.inner.can_sleep(enabled) }
}

///|
pub fn RigidBodyBuilder3Compat::build(
  self : RigidBodyBuilder3Compat,
) -> RigidBody {
  self.inner.build()
}
