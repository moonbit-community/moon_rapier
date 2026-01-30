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
/// 3D-flavored rigid-body builder.
///
/// NOTE: This currently projects to the existing 2D builder by dropping the `z` coordinate and
/// representing orientation with the 2D rotation only.
pub struct RigidBodyBuilder3 {
  inner : RigidBodyBuilder
}

///|
pub fn RigidBodyBuilder3::fixed() -> RigidBodyBuilder3 {
  { inner: RigidBodyBuilder::fixed() }
}

///|
pub fn RigidBodyBuilder3::dynamic() -> RigidBodyBuilder3 {
  { inner: RigidBodyBuilder::dynamic() }
}

///|
pub fn RigidBodyBuilder3::kinematic_position_based() -> RigidBodyBuilder3 {
  { inner: RigidBodyBuilder::kinematic_position_based() }
}

///|
pub fn RigidBodyBuilder3::kinematic_velocity_based() -> RigidBodyBuilder3 {
  { inner: RigidBodyBuilder::kinematic_velocity_based() }
}

///|
pub fn RigidBodyBuilder3::translation(
  self : RigidBodyBuilder3,
  translation : @core.Vec3,
) -> RigidBodyBuilder3 {
  {
    inner: self.inner.translation(@core.Vec2::new(translation.x, translation.y)),
  }
}

///|
pub fn RigidBodyBuilder3::rotation(
  self : RigidBodyBuilder3,
  rotation : @core.Quat,
) -> RigidBodyBuilder3 {
  // 2D compatibility: keep only the rotation around +Z (yaw).
  let axis = rotation.to_scaled_axis()
  { inner: self.inner.rotation(@core.Rot2::from_angle(axis.z)) }
}

///|
pub fn RigidBodyBuilder3::rotation_scaled_axis(
  self : RigidBodyBuilder3,
  scaled_axis : @core.Vec3,
) -> RigidBodyBuilder3 {
  self.rotation(@core.rotation_from_scaled_axis(scaled_axis))
}

///|
pub fn RigidBodyBuilder3::linvel(
  self : RigidBodyBuilder3,
  vel : @core.Vec3,
) -> RigidBodyBuilder3 {
  { inner: self.inner.linvel(@core.Vec2::new(vel.x, vel.y)) }
}

///|
pub fn RigidBodyBuilder3::angvel(
  self : RigidBodyBuilder3,
  vel : @core.Vec3,
) -> RigidBodyBuilder3 {
  // 2D compatibility: keep only the Z component.
  { inner: self.inner.angvel(vel.z) }
}

///|
pub fn RigidBodyBuilder3::gravity_scale(
  self : RigidBodyBuilder3,
  scale : @core.Real,
) -> RigidBodyBuilder3 {
  { inner: self.inner.gravity_scale(scale) }
}

///|
pub fn RigidBodyBuilder3::linear_damping(
  self : RigidBodyBuilder3,
  damping : @core.Real,
) -> RigidBodyBuilder3 {
  { inner: self.inner.linear_damping(damping) }
}

///|
pub fn RigidBodyBuilder3::angular_damping(
  self : RigidBodyBuilder3,
  damping : @core.Real,
) -> RigidBodyBuilder3 {
  { inner: self.inner.angular_damping(damping) }
}

///|
pub fn RigidBodyBuilder3::additional_mass(
  self : RigidBodyBuilder3,
  mass : @core.Real,
) -> RigidBodyBuilder3 {
  { inner: self.inner.additional_mass(mass) }
}

///|
pub fn RigidBodyBuilder3::ccd_enabled(
  self : RigidBodyBuilder3,
  enabled : Bool,
) -> RigidBodyBuilder3 {
  { inner: self.inner.ccd_enabled(enabled) }
}

///|
pub fn RigidBodyBuilder3::soft_ccd_prediction(
  self : RigidBodyBuilder3,
  distance : @core.Real,
) -> RigidBodyBuilder3 {
  { inner: self.inner.soft_ccd_prediction(distance) }
}

///|
pub fn RigidBodyBuilder3::lock_rotations(
  self : RigidBodyBuilder3,
) -> RigidBodyBuilder3 {
  { inner: self.inner.lock_rotations() }
}

///|
pub fn RigidBodyBuilder3::lock_translations(
  self : RigidBodyBuilder3,
) -> RigidBodyBuilder3 {
  { inner: self.inner.lock_translations() }
}

///|
pub fn RigidBodyBuilder3::enabled_rotations(
  self : RigidBodyBuilder3,
  _x : Bool,
  _y : Bool,
  z : Bool,
) -> RigidBodyBuilder3 {
  // 2D compatibility: only the Z rotation exists.
  if z {
    self
  } else {
    self.lock_rotations()
  }
}

///|
pub fn RigidBodyBuilder3::gyroscopic_forces_enabled(
  self : RigidBodyBuilder3,
  _enabled : Bool,
) -> RigidBodyBuilder3 {
  // 2D compatibility: no gyroscopic effect.
  self
}

///|
pub fn RigidBodyBuilder3::can_sleep(
  self : RigidBodyBuilder3,
  enabled : Bool,
) -> RigidBodyBuilder3 {
  { inner: self.inner.can_sleep(enabled) }
}

///|
pub fn RigidBodyBuilder3::build(self : RigidBodyBuilder3) -> RigidBody {
  self.inner.build()
}
