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
/// 3D-flavored prismatic joint builder used by the rapier3d parity tests.
///
/// NOTE: This maps to the existing 2D prismatic joint builder by dropping the `z` coordinate.
pub struct PrismaticJointBuilder3 {
  inner : PrismaticJointBuilder

  fn new(axis : @core.Vec3) -> PrismaticJointBuilder3
}

///|
pub fn PrismaticJointBuilder3::new(axis : @core.Vec3) -> PrismaticJointBuilder3 {
  { inner: PrismaticJointBuilder::new(@core.Vec2::new(axis.x, axis.y)) }
}

///|
pub fn PrismaticJointBuilder3::local_anchor1(
  self : PrismaticJointBuilder3,
  anchor1 : @core.Vec3,
) -> PrismaticJointBuilder3 {
  { inner: self.inner.local_anchor1(@core.Vec2::new(anchor1.x, anchor1.y)) }
}

///|
pub fn PrismaticJointBuilder3::local_anchor2(
  self : PrismaticJointBuilder3,
  anchor2 : @core.Vec3,
) -> PrismaticJointBuilder3 {
  { inner: self.inner.local_anchor2(@core.Vec2::new(anchor2.x, anchor2.y)) }
}

///|
pub fn PrismaticJointBuilder3::softness(
  self : PrismaticJointBuilder3,
  softness : SpringCoefficients,
) -> PrismaticJointBuilder3 {
  { inner: self.inner.softness(softness) }
}

///|
pub fn PrismaticJointBuilder3::motor_model(
  self : PrismaticJointBuilder3,
  model : MotorModel,
) -> PrismaticJointBuilder3 {
  { inner: self.inner.motor_model(model) }
}

///|
pub fn PrismaticJointBuilder3::motor_velocity(
  self : PrismaticJointBuilder3,
  target_vel : @core.Real,
  factor : @core.Real,
) -> PrismaticJointBuilder3 {
  { inner: self.inner.motor_velocity(target_vel, factor) }
}

///|
pub fn PrismaticJointBuilder3::motor_position(
  self : PrismaticJointBuilder3,
  target_pos : @core.Real,
  stiffness : @core.Real,
  damping : @core.Real,
) -> PrismaticJointBuilder3 {
  { inner: self.inner.motor_position(target_pos, stiffness, damping) }
}

///|
pub fn PrismaticJointBuilder3::motor_max_force(
  self : PrismaticJointBuilder3,
  max_force : @core.Real,
) -> PrismaticJointBuilder3 {
  { inner: self.inner.motor_max_force(max_force) }
}

///|
pub fn PrismaticJointBuilder3::limits(
  self : PrismaticJointBuilder3,
  min : @core.Real,
  max : @core.Real,
) -> PrismaticJointBuilder3 {
  { inner: self.inner.limits(min, max) }
}

///|
pub fn PrismaticJointBuilder3::build(
  self : PrismaticJointBuilder3,
) -> PrismaticJoint {
  self.inner.build()
}
