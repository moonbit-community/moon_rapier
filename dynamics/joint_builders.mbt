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
struct FixedJointBuilder {
  mut local_anchor1 : @core.Vec2
  mut local_anchor2 : @core.Vec2
}

///|
pub fn FixedJointBuilder::new() -> FixedJointBuilder {
  { local_anchor1: @core.Vec2::zero(), local_anchor2: @core.Vec2::zero() }
}

///|
pub fn FixedJointBuilder::local_anchor1(
  self : FixedJointBuilder,
  anchor : @core.Vec2,
) -> FixedJointBuilder {
  self.local_anchor1 = anchor
  self
}

///|
pub fn FixedJointBuilder::local_anchor2(
  self : FixedJointBuilder,
  anchor : @core.Vec2,
) -> FixedJointBuilder {
  self.local_anchor2 = anchor
  self
}

///|
pub fn FixedJointBuilder::build(self : FixedJointBuilder) -> FixedJoint {
  { local_anchor1: self.local_anchor1, local_anchor2: self.local_anchor2 }
}

///|
pub struct FixedJoint {
  local_anchor1 : @core.Vec2
  local_anchor2 : @core.Vec2
}

///|
pub fn GenericJoint::from_fixed(joint : FixedJoint) -> GenericJoint {
  let frame1 = @core.Isometry2::from_translation(joint.local_anchor1)
  let frame2 = @core.Isometry2::from_translation(joint.local_anchor2)
  GenericJoint::default()
  .set_local_frame1(frame1)
  .set_local_frame2(frame2)
  .set_locked_axes(JointAxesMask::all())
}

///|
pub struct RopeJointBuilder {
  max_length : @core.Real
  mut local_anchor1 : @core.Vec2
  mut local_anchor2 : @core.Vec2
  mut local_anchor1_3 : @core.Vec3
  mut local_anchor2_3 : @core.Vec3
  is_spatial : Bool
  mut softness : SpringCoefficients
  mut motor : JointMotor
  mut motor_enabled : Bool

  fn new(max_length : @core.Real) -> RopeJointBuilder
}

///|
pub fn RopeJointBuilder::new(max_length : @core.Real) -> RopeJointBuilder {
  {
    max_length,
    local_anchor1: @core.Vec2::zero(),
    local_anchor2: @core.Vec2::zero(),
    local_anchor1_3: @core.Vec3::zero(),
    local_anchor2_3: @core.Vec3::zero(),
    is_spatial: false,
    softness: SpringCoefficients::joint_defaults(),
    motor: JointMotor::default(),
    motor_enabled: false,
  }
}

///|
pub fn RopeJointBuilder::local_anchor1(
  self : RopeJointBuilder,
  anchor : @core.Vec2,
) -> RopeJointBuilder {
  self.local_anchor1 = anchor
  self.local_anchor1_3 = @core.Vec3::new(anchor.x, anchor.y, 0.0F)
  self
}

///|
pub fn RopeJointBuilder::local_anchor2(
  self : RopeJointBuilder,
  anchor : @core.Vec2,
) -> RopeJointBuilder {
  self.local_anchor2 = anchor
  self.local_anchor2_3 = @core.Vec3::new(anchor.x, anchor.y, 0.0F)
  self
}

///|
pub fn RopeJointBuilder::softness(
  self : RopeJointBuilder,
  softness : SpringCoefficients,
) -> RopeJointBuilder {
  self.softness = softness
  self
}

///|
pub fn RopeJointBuilder::motor_model(
  self : RopeJointBuilder,
  model : MotorModel,
) -> RopeJointBuilder {
  self.motor = self.motor.set_model(model)
  self
}

///|
pub fn RopeJointBuilder::motor_velocity(
  self : RopeJointBuilder,
  target_vel : @core.Real,
  factor : @core.Real,
) -> RopeJointBuilder {
  self.motor_enabled = true
  self.motor = self.motor
    .set_target_vel(target_vel)
    .set_stiffness(0.0F)
    .set_damping(factor)
  self
}

///|
pub fn RopeJointBuilder::motor_position(
  self : RopeJointBuilder,
  target_pos : @core.Real,
  stiffness : @core.Real,
  damping : @core.Real,
) -> RopeJointBuilder {
  self.motor_enabled = true
  self.motor = self.motor
    .set_target_pos(target_pos)
    .set_target_vel(0.0F)
    .set_stiffness(stiffness)
    .set_damping(damping)
  self
}

///|
pub fn RopeJointBuilder::motor_max_force(
  self : RopeJointBuilder,
  max_force : @core.Real,
) -> RopeJointBuilder {
  self.motor = self.motor.set_max_force(max_force)
  self
}

///|
pub fn RopeJointBuilder::build(self : RopeJointBuilder) -> RopeJoint {
  {
    max_length: self.max_length,
    local_anchor1: self.local_anchor1,
    local_anchor2: self.local_anchor2,
    local_anchor1_3: self.local_anchor1_3,
    local_anchor2_3: self.local_anchor2_3,
    is_spatial: self.is_spatial,
    softness: self.softness,
    motor: self.motor,
    motor_enabled: self.motor_enabled,
  }
}

///|
pub struct RopeJoint {
  max_length : @core.Real
  local_anchor1 : @core.Vec2
  local_anchor2 : @core.Vec2
  local_anchor1_3 : @core.Vec3
  local_anchor2_3 : @core.Vec3
  is_spatial : Bool
  softness : SpringCoefficients
  motor : JointMotor
  motor_enabled : Bool
}

///|
pub fn RopeJoint::max_length(self : RopeJoint) -> @core.Real {
  self.max_length
}

///|
pub fn RopeJoint::max_distance(self : RopeJoint) -> @core.Real {
  self.max_length
}

///|
pub fn RopeJoint::softness(self : RopeJoint) -> SpringCoefficients {
  self.softness
}

///|
pub fn RopeJoint::motor(self : RopeJoint) -> JointMotor? {
  if self.motor_enabled {
    Some(self.motor)
  } else {
    None
  }
}

///|
pub fn RopeJoint::set_max_distance(
  self : RopeJoint,
  max_distance : @core.Real,
) -> RopeJoint {
  RopeJoint::{
    max_length: max_distance,
    local_anchor1: self.local_anchor1,
    local_anchor2: self.local_anchor2,
    local_anchor1_3: self.local_anchor1_3,
    local_anchor2_3: self.local_anchor2_3,
    is_spatial: self.is_spatial,
    softness: self.softness,
    motor: self.motor,
    motor_enabled: self.motor_enabled,
  }
}

///|
pub fn RopeJointBuilder::max_distance(self : RopeJointBuilder) -> @core.Real {
  self.max_length
}

///|
pub fn GenericJoint::from_rope(joint : RopeJoint) -> GenericJoint {
  let frame1 = @core.Isometry2::from_translation(joint.local_anchor1)
  let frame2 = @core.Isometry2::from_translation(joint.local_anchor2)
  let mut result = GenericJoint::{
    local_frame1: frame1,
    local_frame2: frame2,
    axis3: @core.Vec3::new(1.0F, 0.0F, 0.0F),
    local_anchor1_3: joint.local_anchor1_3,
    local_anchor2_3: joint.local_anchor2_3,
    local_frame1_rotation: @core.Quat::identity(),
    local_frame2_rotation: @core.Quat::identity(),
    locked_axes: JointAxesMask::empty(),
    limit_axes: JointAxesMask::empty(),
    motor_axes: JointAxesMask::empty(),
    coupled_axes: if joint.is_spatial {
      JointAxesMask::lin_axes3()
    } else {
      JointAxesMask::lin_axes()
    },
    limits: JointLimits3::default(),
    motors: JointMotors3::default(),
    softness: joint.softness,
    contacts_enabled: true,
    enabled: JointEnabled::Enabled,
    user_data: 0,
  }
  result = result.set_limits_axis(JointAxis::LinX, 0.0F, joint.max_length)
  if joint.motor_enabled {
    result = result
      .set_motor(
        JointAxis::LinX,
        joint.motor.target_pos,
        joint.motor.target_vel,
        joint.motor.stiffness,
        joint.motor.damping,
      )
      .set_motor_model(JointAxis::LinX, joint.motor.model)
      .set_motor_max_force(JointAxis::LinX, joint.motor.max_force)
  }
  result
}

///|
pub struct PinSlotJointBuilder {
  axis : @core.Vec2
  mut local_anchor1 : @core.Vec2
  mut local_anchor2 : @core.Vec2
  mut limits : JointLimits
  mut limits_enabled : Bool

  fn new(axis : @core.Vec2) -> PinSlotJointBuilder
}

///|
pub fn PinSlotJointBuilder::new(axis : @core.Vec2) -> PinSlotJointBuilder {
  let norm_axis = if axis.length_squared() <= 1.0e-12F {
    @core.Vec2::new(1.0F, 0.0F)
  } else {
    axis.normalize()
  }
  {
    axis: norm_axis,
    local_anchor1: @core.Vec2::zero(),
    local_anchor2: @core.Vec2::zero(),
    limits: JointLimits::default(),
    limits_enabled: false,
  }
}

///|
pub fn PinSlotJointBuilder::local_anchor1(
  self : PinSlotJointBuilder,
  anchor : @core.Vec2,
) -> PinSlotJointBuilder {
  self.local_anchor1 = anchor
  self
}

///|
pub fn PinSlotJointBuilder::local_anchor2(
  self : PinSlotJointBuilder,
  anchor : @core.Vec2,
) -> PinSlotJointBuilder {
  self.local_anchor2 = anchor
  self
}

///|
pub fn PinSlotJointBuilder::limits(
  self : PinSlotJointBuilder,
  min : @core.Real,
  max : @core.Real,
) -> PinSlotJointBuilder {
  self.limits_enabled = true
  self.limits = JointLimits::new(min, max)
  self
}

///|
pub fn PinSlotJointBuilder::build(self : PinSlotJointBuilder) -> PinSlotJoint {
  {
    axis: self.axis,
    local_anchor1: self.local_anchor1,
    local_anchor2: self.local_anchor2,
    limits: self.limits,
    limits_enabled: self.limits_enabled,
  }
}

///|
pub struct PinSlotJoint {
  axis : @core.Vec2
  local_anchor1 : @core.Vec2
  local_anchor2 : @core.Vec2
  limits : JointLimits
  limits_enabled : Bool
}

///|
pub fn PinSlotJoint::set_local_axis1(
  self : PinSlotJoint,
  axis : @core.Vec2,
) -> PinSlotJoint {
  PinSlotJoint::{
    axis,
    local_anchor1: self.local_anchor1,
    local_anchor2: self.local_anchor2,
    limits: self.limits,
    limits_enabled: self.limits_enabled,
  }
}

///|
pub fn PinSlotJoint::set_local_axis2(
  self : PinSlotJoint,
  axis : @core.Vec2,
) -> PinSlotJoint {
  // This 2D port stores a single joint axis, so we treat axis1/axis2 identically.
  self.set_local_axis1(axis)
}

///|
pub fn GenericJoint::from_pin_slot(joint : PinSlotJoint) -> GenericJoint {
  let angle = @core.atan2(joint.axis.y, joint.axis.x)
  let frame1 = @core.Isometry2::new(
    joint.local_anchor1,
    @core.Rot2::from_angle(angle),
  )
  let frame2 = @core.Isometry2::from_translation(joint.local_anchor2)
  let mut result = GenericJoint::default()
    .set_local_frame1(frame1)
    .set_local_frame2(frame2)
    .set_locked_axes(JointAxesMask::lin_y())
  if joint.limits_enabled {
    let limits = result.limits()
    result = result
      .set_limit_axes(JointAxesMask::lin_x())
      .set_limits(JointLimits3::{
        lin_x: joint.limits,
        lin_y: limits.lin_y,
        lin_z: limits.lin_z,
        ang_x: limits.ang_x,
        ang_y: limits.ang_y,
        ang_z: limits.ang_z,
      })
  }
  result
}

///|
struct PrismaticJointBuilder {
  axis : @core.Vec2
  axis3 : @core.Vec3
  mut local_anchor1 : @core.Vec2
  mut local_anchor2 : @core.Vec2
  mut local_anchor1_3 : @core.Vec3
  mut local_anchor2_3 : @core.Vec3
  local_frame1_rotation : @core.Quat
  local_frame2_rotation : @core.Quat
  is_spatial : Bool
  mut softness : SpringCoefficients
  mut motor : JointMotor
  mut motor_enabled : Bool
  mut limits : JointLimits
  mut limits_enabled : Bool
}

///|
pub fn PrismaticJointBuilder::new(axis : @core.Vec2) -> PrismaticJointBuilder {
  let norm_axis = if axis.length_squared() <= 1.0e-12F {
    @core.Vec2::new(1.0F, 0.0F)
  } else {
    axis.normalize()
  }
  {
    axis: norm_axis,
    axis3: @core.Vec3::new(norm_axis.x, norm_axis.y, 0.0F),
    local_anchor1: @core.Vec2::zero(),
    local_anchor2: @core.Vec2::zero(),
    local_anchor1_3: @core.Vec3::zero(),
    local_anchor2_3: @core.Vec3::zero(),
    local_frame1_rotation: @core.rotation_from_scaled_axis(
      @core.Vec3::new(0.0F, 0.0F, @core.atan2(norm_axis.y, norm_axis.x)),
    ),
    local_frame2_rotation: @core.rotation_from_scaled_axis(
      @core.Vec3::new(0.0F, 0.0F, @core.atan2(norm_axis.y, norm_axis.x)),
    ),
    is_spatial: false,
    softness: SpringCoefficients::joint_defaults(),
    motor: JointMotor::default(),
    motor_enabled: false,
    limits: JointLimits::default(),
    limits_enabled: false,
  }
}

///|
pub fn PrismaticJointBuilder::local_anchor1(
  self : PrismaticJointBuilder,
  anchor : @core.Vec2,
) -> PrismaticJointBuilder {
  self.local_anchor1 = anchor
  self.local_anchor1_3 = @core.Vec3::new(anchor.x, anchor.y, 0.0F)
  self
}

///|
pub fn PrismaticJointBuilder::local_anchor2(
  self : PrismaticJointBuilder,
  anchor : @core.Vec2,
) -> PrismaticJointBuilder {
  self.local_anchor2 = anchor
  self.local_anchor2_3 = @core.Vec3::new(anchor.x, anchor.y, 0.0F)
  self
}

///|
pub fn PrismaticJointBuilder::softness(
  self : PrismaticJointBuilder,
  softness : SpringCoefficients,
) -> PrismaticJointBuilder {
  self.softness = softness
  self
}

///|
pub fn PrismaticJointBuilder::motor_model(
  self : PrismaticJointBuilder,
  model : MotorModel,
) -> PrismaticJointBuilder {
  self.motor = self.motor.set_model(model)
  self
}

///|
pub fn PrismaticJointBuilder::motor_velocity(
  self : PrismaticJointBuilder,
  target_vel : @core.Real,
  factor : @core.Real,
) -> PrismaticJointBuilder {
  self.motor_enabled = true
  self.motor = self.motor
    .set_target_vel(target_vel)
    .set_stiffness(0.0F)
    .set_damping(factor)
  self
}

///|
pub fn PrismaticJointBuilder::motor_position(
  self : PrismaticJointBuilder,
  target_pos : @core.Real,
  stiffness : @core.Real,
  damping : @core.Real,
) -> PrismaticJointBuilder {
  self.motor_enabled = true
  self.motor = self.motor
    .set_target_pos(target_pos)
    .set_target_vel(0.0F)
    .set_stiffness(stiffness)
    .set_damping(damping)
  self
}

///|
pub fn PrismaticJointBuilder::motor_max_force(
  self : PrismaticJointBuilder,
  max_force : @core.Real,
) -> PrismaticJointBuilder {
  self.motor = self.motor.set_max_force(max_force)
  self
}

///|
pub fn PrismaticJointBuilder::limits(
  self : PrismaticJointBuilder,
  min : @core.Real,
  max : @core.Real,
) -> PrismaticJointBuilder {
  self.limits_enabled = true
  self.limits = JointLimits::new(min, max)
  self
}

///|
pub fn PrismaticJointBuilder::build(
  self : PrismaticJointBuilder,
) -> PrismaticJoint {
  {
    axis: self.axis,
    local_anchor1: self.local_anchor1,
    local_anchor2: self.local_anchor2,
    axis3: self.axis3,
    local_anchor1_3: self.local_anchor1_3,
    local_anchor2_3: self.local_anchor2_3,
    local_frame1_rotation: self.local_frame1_rotation,
    local_frame2_rotation: self.local_frame2_rotation,
    is_spatial: self.is_spatial,
    softness: self.softness,
    motor: self.motor,
    motor_enabled: self.motor_enabled,
    limits: self.limits,
    limits_enabled: self.limits_enabled,
  }
}

///|
pub struct PrismaticJoint {
  axis : @core.Vec2
  local_anchor1 : @core.Vec2
  local_anchor2 : @core.Vec2
  axis3 : @core.Vec3
  local_anchor1_3 : @core.Vec3
  local_anchor2_3 : @core.Vec3
  local_frame1_rotation : @core.Quat
  local_frame2_rotation : @core.Quat
  is_spatial : Bool
  softness : SpringCoefficients
  motor : JointMotor
  motor_enabled : Bool
  limits : JointLimits
  limits_enabled : Bool
}

///|
pub fn PrismaticJoint::set_local_axis1(
  self : PrismaticJoint,
  axis : @core.Vec2,
) -> PrismaticJoint {
  PrismaticJoint::{
    axis,
    local_anchor1: self.local_anchor1,
    local_anchor2: self.local_anchor2,
    axis3: @core.Vec3::new(axis.x, axis.y, self.axis3.z),
    local_anchor1_3: self.local_anchor1_3,
    local_anchor2_3: self.local_anchor2_3,
    local_frame1_rotation: @core.rotation_from_scaled_axis(
      @core.Vec3::new(0.0F, 0.0F, @core.atan2(axis.y, axis.x)),
    ),
    local_frame2_rotation: @core.rotation_from_scaled_axis(
      @core.Vec3::new(0.0F, 0.0F, @core.atan2(axis.y, axis.x)),
    ),
    is_spatial: self.is_spatial,
    softness: self.softness,
    motor: self.motor,
    motor_enabled: self.motor_enabled,
    limits: self.limits,
    limits_enabled: self.limits_enabled,
  }
}

///|
pub fn PrismaticJoint::set_local_axis2(
  self : PrismaticJoint,
  axis : @core.Vec2,
) -> PrismaticJoint {
  // This 2D port stores a single joint axis, so we treat axis1/axis2 identically.
  self.set_local_axis1(axis)
}

///|
pub fn PrismaticJoint::softness(self : PrismaticJoint) -> SpringCoefficients {
  self.softness
}

///|
pub fn PrismaticJoint::motor(self : PrismaticJoint) -> JointMotor? {
  if self.motor_enabled {
    Some(self.motor)
  } else {
    None
  }
}

///|
pub fn GenericJoint::from_prismatic(joint : PrismaticJoint) -> GenericJoint {
  let angle = @core.atan2(joint.axis.y, joint.axis.x)
  let frame1 = @core.Isometry2::new(
    joint.local_anchor1,
    @core.Rot2::from_angle(angle),
  )
  let frame2 = @core.Isometry2::from_translation(joint.local_anchor2)
  let mut result = GenericJoint::{
    local_frame1: frame1,
    local_frame2: frame2,
    axis3: joint.axis3,
    local_anchor1_3: joint.local_anchor1_3,
    local_anchor2_3: joint.local_anchor2_3,
    local_frame1_rotation: joint.local_frame1_rotation,
    local_frame2_rotation: joint.local_frame2_rotation,
    locked_axes: if joint.is_spatial {
      JointAxesMask::locked_prismatic_axes3()
    } else {
      JointAxesMask::locked_prismatic_axes()
    },
    limit_axes: JointAxesMask::empty(),
    motor_axes: JointAxesMask::empty(),
    coupled_axes: JointAxesMask::empty(),
    limits: JointLimits3::default(),
    motors: JointMotors3::default(),
    softness: joint.softness,
    contacts_enabled: true,
    enabled: JointEnabled::Enabled,
    user_data: 0,
  }
  if joint.motor_enabled {
    result = result
      .set_motor(
        JointAxis::LinX,
        joint.motor.target_pos,
        joint.motor.target_vel,
        joint.motor.stiffness,
        joint.motor.damping,
      )
      .set_motor_model(JointAxis::LinX, joint.motor.model)
      .set_motor_max_force(JointAxis::LinX, joint.motor.max_force)
  }
  if joint.limits_enabled {
    let limits = result.limits()
    result = result
      .set_limit_axes(JointAxesMask::lin_x())
      .set_limits(JointLimits3::{
        lin_x: joint.limits,
        lin_y: limits.lin_y,
        lin_z: limits.lin_z,
        ang_x: limits.ang_x,
        ang_y: limits.ang_y,
        ang_z: limits.ang_z,
      })
  }
  result
}
