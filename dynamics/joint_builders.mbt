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
}

///|
pub fn RopeJointBuilder::new(max_length : @core.Real) -> RopeJointBuilder {
  {
    max_length,
    local_anchor1: @core.Vec2::zero(),
    local_anchor2: @core.Vec2::zero(),
  }
}

///|
pub fn RopeJointBuilder::local_anchor1(
  self : RopeJointBuilder,
  anchor : @core.Vec2,
) -> RopeJointBuilder {
  self.local_anchor1 = anchor
  self
}

///|
pub fn RopeJointBuilder::local_anchor2(
  self : RopeJointBuilder,
  anchor : @core.Vec2,
) -> RopeJointBuilder {
  self.local_anchor2 = anchor
  self
}

///|
pub fn RopeJointBuilder::build(self : RopeJointBuilder) -> RopeJoint {
  {
    max_length: self.max_length,
    local_anchor1: self.local_anchor1,
    local_anchor2: self.local_anchor2,
  }
}

///|
pub struct RopeJoint {
  max_length : @core.Real
  local_anchor1 : @core.Vec2
  local_anchor2 : @core.Vec2
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
pub fn RopeJoint::set_max_distance(
  self : RopeJoint,
  max_distance : @core.Real,
) -> RopeJoint {
  RopeJoint::{
    max_length: max_distance,
    local_anchor1: self.local_anchor1,
    local_anchor2: self.local_anchor2,
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
  GenericJoint::default()
  .set_local_frame1(frame1)
  .set_local_frame2(frame2)
  .set_coupled_axes(JointAxesMask::lin_axes())
  // Rapier parity: rope joint is a coupled linear limit on the distance between anchors.
  // The coupled solver reads the limits stored on the first coupled axis (LinX).
  .set_limits_axis(JointAxis::LinX, 0.0F, joint.max_length)
}

///|
pub struct PinSlotJointBuilder {
  axis : @core.Vec2
  mut local_anchor1 : @core.Vec2
  mut local_anchor2 : @core.Vec2
  mut limits : JointLimits
  mut limits_enabled : Bool
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
        ang_z: limits.ang_z,
      })
  }
  result
}

///|
struct PrismaticJointBuilder {
  axis : @core.Vec2
  mut local_anchor1 : @core.Vec2
  mut local_anchor2 : @core.Vec2
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
    local_anchor1: @core.Vec2::zero(),
    local_anchor2: @core.Vec2::zero(),
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
  self
}

///|
pub fn PrismaticJointBuilder::local_anchor2(
  self : PrismaticJointBuilder,
  anchor : @core.Vec2,
) -> PrismaticJointBuilder {
  self.local_anchor2 = anchor
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
    limits: self.limits,
    limits_enabled: self.limits_enabled,
  }
}

///|
pub struct PrismaticJoint {
  axis : @core.Vec2
  local_anchor1 : @core.Vec2
  local_anchor2 : @core.Vec2
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
pub fn GenericJoint::from_prismatic(joint : PrismaticJoint) -> GenericJoint {
  let angle = @core.atan2(joint.axis.y, joint.axis.x)
  let frame1 = @core.Isometry2::new(
    joint.local_anchor1,
    @core.Rot2::from_angle(angle),
  )
  let frame2 = @core.Isometry2::from_translation(joint.local_anchor2)
  let mut result = GenericJoint::default()
    .set_local_frame1(frame1)
    .set_local_frame2(frame2)
    // 2D prismatic: keep translation along local x, lock translation along y and rotation.
    .set_locked_axes(JointAxesMask::lin_y().or(JointAxesMask::ang_z()))
  if joint.limits_enabled {
    let limits = result.limits()
    result = result
      .set_limit_axes(JointAxesMask::lin_x())
      .set_limits(JointLimits3::{
        lin_x: joint.limits,
        lin_y: limits.lin_y,
        ang_z: limits.ang_z,
      })
  }
  result
}
