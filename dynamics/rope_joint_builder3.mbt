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
/// 3D-flavored rope joint builder used by the rapier3d parity tests.
///
/// NOTE: This maps to the existing 2D rope joint builder by dropping the `z` coordinate.
pub struct RopeJointBuilder3 {
  inner : RopeJointBuilder

  fn new(max_length : @core.Real) -> RopeJointBuilder3
}

///|
pub fn RopeJointBuilder3::new(max_length : @core.Real) -> RopeJointBuilder3 {
  { inner: RopeJointBuilder::new(max_length) }
}

///|
pub fn RopeJointBuilder3::local_anchor1(
  self : RopeJointBuilder3,
  anchor : @core.Vec3,
) -> RopeJointBuilder3 {
  { inner: self.inner.local_anchor1(@core.Vec2::new(anchor.x, anchor.y)) }
}

///|
pub fn RopeJointBuilder3::local_anchor2(
  self : RopeJointBuilder3,
  anchor : @core.Vec3,
) -> RopeJointBuilder3 {
  { inner: self.inner.local_anchor2(@core.Vec2::new(anchor.x, anchor.y)) }
}

///|
pub fn RopeJointBuilder3::softness(
  self : RopeJointBuilder3,
  softness : SpringCoefficients,
) -> RopeJointBuilder3 {
  { inner: self.inner.softness(softness) }
}

///|
pub fn RopeJointBuilder3::motor_model(
  self : RopeJointBuilder3,
  model : MotorModel,
) -> RopeJointBuilder3 {
  { inner: self.inner.motor_model(model) }
}

///|
pub fn RopeJointBuilder3::motor_velocity(
  self : RopeJointBuilder3,
  target_vel : @core.Real,
  factor : @core.Real,
) -> RopeJointBuilder3 {
  { inner: self.inner.motor_velocity(target_vel, factor) }
}

///|
pub fn RopeJointBuilder3::motor_position(
  self : RopeJointBuilder3,
  target_pos : @core.Real,
  stiffness : @core.Real,
  damping : @core.Real,
) -> RopeJointBuilder3 {
  { inner: self.inner.motor_position(target_pos, stiffness, damping) }
}

///|
pub fn RopeJointBuilder3::motor_max_force(
  self : RopeJointBuilder3,
  max_force : @core.Real,
) -> RopeJointBuilder3 {
  { inner: self.inner.motor_max_force(max_force) }
}

///|
pub fn RopeJointBuilder3::max_distance(
  self : RopeJointBuilder3,
  max_distance : @core.Real,
) -> RopeJointBuilder3 {
  {
    inner: {
      max_length: max_distance,
      local_anchor1: self.inner.local_anchor1,
      local_anchor2: self.inner.local_anchor2,
      softness: self.inner.softness,
      motor: self.inner.motor,
      motor_enabled: self.inner.motor_enabled,
    },
  }
}

///|
pub fn RopeJointBuilder3::build(self : RopeJointBuilder3) -> RopeJoint {
  self.inner.build()
}
