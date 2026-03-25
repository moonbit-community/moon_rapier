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
/// 3D-flavored revolute joint builder used by the rapier3d parity tests.
pub struct RevoluteJointBuilder3 {
  inner : RevoluteJointBuilder
  axis : @core.Vec3
  local_anchor1_3 : @core.Vec3
  local_anchor2_3 : @core.Vec3

  fn new(axis : @core.Vec3) -> RevoluteJointBuilder3
}

///|
pub fn RevoluteJointBuilder3::new(axis : @core.Vec3) -> RevoluteJointBuilder3 {
  {
    inner: RevoluteJointBuilder::new(),
    axis: revolute_joint_axis3_or_default(axis),
    local_anchor1_3: @core.Vec3::zero(),
    local_anchor2_3: @core.Vec3::zero(),
  }
}

///|
pub fn RevoluteJointBuilder3::local_anchor1(
  self : RevoluteJointBuilder3,
  anchor1 : @core.Vec3,
) -> RevoluteJointBuilder3 {
  {
    inner: self.inner.local_anchor1(@core.Vec2::new(anchor1.x, anchor1.y)),
    axis: self.axis,
    local_anchor1_3: anchor1,
    local_anchor2_3: self.local_anchor2_3,
  }
}

///|
pub fn RevoluteJointBuilder3::local_anchor2(
  self : RevoluteJointBuilder3,
  anchor2 : @core.Vec3,
) -> RevoluteJointBuilder3 {
  {
    inner: self.inner.local_anchor2(@core.Vec2::new(anchor2.x, anchor2.y)),
    axis: self.axis,
    local_anchor1_3: self.local_anchor1_3,
    local_anchor2_3: anchor2,
  }
}

///|
pub fn RevoluteJointBuilder3::softness(
  self : RevoluteJointBuilder3,
  softness : SpringCoefficients,
) -> RevoluteJointBuilder3 {
  {
    inner: self.inner.softness(softness),
    axis: self.axis,
    local_anchor1_3: self.local_anchor1_3,
    local_anchor2_3: self.local_anchor2_3,
  }
}

///|
pub fn RevoluteJointBuilder3::motor_model(
  self : RevoluteJointBuilder3,
  model : MotorModel,
) -> RevoluteJointBuilder3 {
  {
    inner: self.inner.motor_model(model),
    axis: self.axis,
    local_anchor1_3: self.local_anchor1_3,
    local_anchor2_3: self.local_anchor2_3,
  }
}

///|
pub fn RevoluteJointBuilder3::motor_velocity(
  self : RevoluteJointBuilder3,
  target_vel : @core.Real,
  factor : @core.Real,
) -> RevoluteJointBuilder3 {
  {
    inner: self.inner.motor_velocity(target_vel, factor),
    axis: self.axis,
    local_anchor1_3: self.local_anchor1_3,
    local_anchor2_3: self.local_anchor2_3,
  }
}

///|
pub fn RevoluteJointBuilder3::motor_position(
  self : RevoluteJointBuilder3,
  target_pos : @core.Real,
  stiffness : @core.Real,
  damping : @core.Real,
) -> RevoluteJointBuilder3 {
  {
    inner: self.inner.motor_position(target_pos, stiffness, damping),
    axis: self.axis,
    local_anchor1_3: self.local_anchor1_3,
    local_anchor2_3: self.local_anchor2_3,
  }
}

///|
pub fn RevoluteJointBuilder3::motor_max_force(
  self : RevoluteJointBuilder3,
  max_force : @core.Real,
) -> RevoluteJointBuilder3 {
  {
    inner: self.inner.motor_max_force(max_force),
    axis: self.axis,
    local_anchor1_3: self.local_anchor1_3,
    local_anchor2_3: self.local_anchor2_3,
  }
}

///|
pub fn RevoluteJointBuilder3::limits(
  self : RevoluteJointBuilder3,
  min : @core.Real,
  max : @core.Real,
) -> RevoluteJointBuilder3 {
  {
    inner: self.inner.limits(min, max),
    axis: self.axis,
    local_anchor1_3: self.local_anchor1_3,
    local_anchor2_3: self.local_anchor2_3,
  }
}

///|
pub fn RevoluteJointBuilder3::build(
  self : RevoluteJointBuilder3,
) -> RevoluteJoint {
  let joint = self.inner.build()
  let frame = revolute_joint_frame_from_axis3(self.axis)
  {
    local_anchor1: joint.local_anchor1,
    local_anchor2: joint.local_anchor2,
    axis3: self.axis,
    local_anchor1_3: self.local_anchor1_3,
    local_anchor2_3: self.local_anchor2_3,
    local_frame1_rotation: frame,
    local_frame2_rotation: frame,
    is_spatial: true,
    softness: joint.softness,
    motor: joint.motor,
    motor_enabled: joint.motor_enabled,
    limits: joint.limits,
    limits_enabled: joint.limits_enabled,
    solver_lin_impulses: joint.solver_lin_impulses,
  }
}
