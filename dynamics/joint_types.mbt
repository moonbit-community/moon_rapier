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
const REAL_MAX : @core.Real = 1.0e30F

///|
pub(all) enum JointAxis {
  LinX
  LinY
  AngZ
}

///|
pub struct JointAxesMask {
  bits : Int
}

///|
pub fn JointAxesMask::empty() -> JointAxesMask {
  { bits: 0 }
}

///|
pub fn JointAxesMask::new(bits : Int) -> JointAxesMask {
  { bits, }
}

///|
pub fn JointAxesMask::bits(self : JointAxesMask) -> Int {
  self.bits
}

///|
pub fn JointAxesMask::contains(
  self : JointAxesMask,
  other : JointAxesMask,
) -> Bool {
  (self.bits & other.bits) == other.bits
}

///|
pub fn JointAxesMask::or(
  self : JointAxesMask,
  other : JointAxesMask,
) -> JointAxesMask {
  { bits: self.bits | other.bits }
}

///|
pub fn JointAxesMask::intersection(
  self : JointAxesMask,
  other : JointAxesMask,
) -> JointAxesMask {
  { bits: self.bits & other.bits }
}

///|
pub fn JointAxesMask::lin_x() -> JointAxesMask {
  { bits: 1 << 0 }
}

///|
pub fn JointAxesMask::lin_y() -> JointAxesMask {
  { bits: 1 << 1 }
}

///|
pub fn JointAxesMask::ang_z() -> JointAxesMask {
  { bits: 1 << 2 }
}

///|
pub fn JointAxesMask::lin_axes() -> JointAxesMask {
  JointAxesMask::lin_x().or(JointAxesMask::lin_y())
}

///|
pub fn JointAxesMask::ang_axes() -> JointAxesMask {
  JointAxesMask::ang_z()
}

///|
pub fn JointAxesMask::all() -> JointAxesMask {
  JointAxesMask::lin_axes().or(JointAxesMask::ang_axes())
}

///|
pub fn JointAxesMask::locked_revolute_axes() -> JointAxesMask {
  // 2D hinge: lock both linear axes; keep angular free.
  JointAxesMask::lin_axes()
}

///|
pub struct JointLimits {
  min : @core.Real
  max : @core.Real
  mut impulse : @core.Real
}

///|
pub fn JointLimits::default() -> JointLimits {
  { min: -REAL_MAX, max: REAL_MAX, impulse: 0.0F }
}

///|
pub fn JointLimits::new(min : @core.Real, max : @core.Real) -> JointLimits {
  { min, max, impulse: 0.0F }
}

///|
pub fn JointLimits::min(self : JointLimits) -> @core.Real {
  self.min
}

///|
pub fn JointLimits::max(self : JointLimits) -> @core.Real {
  self.max
}

///|
pub fn JointLimits::impulse(self : JointLimits) -> @core.Real {
  self.impulse
}

///|
pub fn JointLimits::set_min(
  self : JointLimits,
  min : @core.Real,
) -> JointLimits {
  JointLimits::{ min, max: self.max, impulse: self.impulse }
}

///|
pub fn JointLimits::set_max(
  self : JointLimits,
  max : @core.Real,
) -> JointLimits {
  JointLimits::{ min: self.min, max, impulse: self.impulse }
}

///|
pub fn JointLimits::clear_impulse(self : JointLimits) -> JointLimits {
  self.impulse = 0.0F
  self
}

///|
pub fn JointLimits::set_impulse(
  self : JointLimits,
  impulse : @core.Real,
) -> JointLimits {
  self.impulse = impulse
  self
}

///|
pub struct JointLimits3 {
  lin_x : JointLimits
  lin_y : JointLimits
  ang_z : JointLimits
}

///|
pub fn JointLimits3::default() -> JointLimits3 {
  {
    lin_x: JointLimits::default(),
    lin_y: JointLimits::default(),
    ang_z: JointLimits::default(),
  }
}

///|
pub(all) enum MotorModel {
  AccelerationBased
  ForceBased
}

///|
pub struct JointMotor {
  target_vel : @core.Real
  target_pos : @core.Real
  stiffness : @core.Real
  damping : @core.Real
  max_force : @core.Real
  mut impulse : @core.Real
  model : MotorModel
}

///|
pub fn JointMotor::default() -> JointMotor {
  {
    target_pos: 0.0F,
    target_vel: 0.0F,
    stiffness: 0.0F,
    damping: 0.0F,
    max_force: REAL_MAX,
    impulse: 0.0F,
    model: MotorModel::AccelerationBased,
  }
}

///|
pub fn JointMotor::set_target_pos(
  self : JointMotor,
  target_pos : @core.Real,
) -> JointMotor {
  JointMotor::{
    target_vel: self.target_vel,
    target_pos,
    stiffness: self.stiffness,
    damping: self.damping,
    max_force: self.max_force,
    impulse: self.impulse,
    model: self.model,
  }
}

///|
pub fn JointMotor::set_target_vel(
  self : JointMotor,
  target_vel : @core.Real,
) -> JointMotor {
  JointMotor::{
    target_vel,
    target_pos: self.target_pos,
    stiffness: self.stiffness,
    damping: self.damping,
    max_force: self.max_force,
    impulse: self.impulse,
    model: self.model,
  }
}

///|
pub fn JointMotor::set_stiffness(
  self : JointMotor,
  stiffness : @core.Real,
) -> JointMotor {
  JointMotor::{
    target_vel: self.target_vel,
    target_pos: self.target_pos,
    stiffness,
    damping: self.damping,
    max_force: self.max_force,
    impulse: self.impulse,
    model: self.model,
  }
}

///|
pub fn JointMotor::set_damping(
  self : JointMotor,
  damping : @core.Real,
) -> JointMotor {
  JointMotor::{
    target_vel: self.target_vel,
    target_pos: self.target_pos,
    stiffness: self.stiffness,
    damping,
    max_force: self.max_force,
    impulse: self.impulse,
    model: self.model,
  }
}

///|
pub fn JointMotor::set_max_force(
  self : JointMotor,
  max_force : @core.Real,
) -> JointMotor {
  JointMotor::{
    target_vel: self.target_vel,
    target_pos: self.target_pos,
    stiffness: self.stiffness,
    damping: self.damping,
    max_force,
    impulse: self.impulse,
    model: self.model,
  }
}

///|
pub fn JointMotor::set_model(
  self : JointMotor,
  model : MotorModel,
) -> JointMotor {
  JointMotor::{
    target_vel: self.target_vel,
    target_pos: self.target_pos,
    stiffness: self.stiffness,
    damping: self.damping,
    max_force: self.max_force,
    impulse: self.impulse,
    model,
  }
}

///|
pub fn JointMotor::clear_impulse(self : JointMotor) -> JointMotor {
  self.impulse = 0.0F
  self
}

///|
pub fn JointMotor::impulse(self : JointMotor) -> @core.Real {
  self.impulse
}

///|
pub fn JointMotor::set_impulse(
  self : JointMotor,
  impulse : @core.Real,
) -> JointMotor {
  self.impulse = impulse
  self
}

///|
pub struct JointMotors3 {
  lin_x : JointMotor
  lin_y : JointMotor
  ang_z : JointMotor
}

///|
pub fn JointMotors3::default() -> JointMotors3 {
  {
    lin_x: JointMotor::default(),
    lin_y: JointMotor::default(),
    ang_z: JointMotor::default(),
  }
}
