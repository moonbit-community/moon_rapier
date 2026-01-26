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
pub fn JointLimits::clear_impulse(self : JointLimits) -> JointLimits {
  self.impulse = 0.0F
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
pub fn JointMotor::clear_impulse(self : JointMotor) -> JointMotor {
  self.impulse = 0.0F
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
