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
/// Builder helpers for the real dim3 joint constraints.
pub(all) enum AxisState3DReal {
  Free
  Locked
  Limited(@core.Real, @core.Real)
}

///|
pub(all) enum MotorState3DReal {
  Disabled
  Velocity(@core.Real, @core.Real, @core.Real) // target_vel, damping, max_force
  Position(@core.Real, @core.Real, @core.Real, @core.Real) // target_pos, stiffness, damping, max_force
}

///|
fn axis_index_3d_real(axis : JointAxis3DReal) -> Int {
  match axis {
    JointAxis3DReal::LinX => 0
    JointAxis3DReal::LinY => 1
    JointAxis3DReal::LinZ => 2
    JointAxis3DReal::AngX => 3
    JointAxis3DReal::AngY => 4
    JointAxis3DReal::AngZ => 5
  }
}

///|
pub struct GenericJoint3DRealBuilder {
  locked_axes : JointAxesMask3DReal
  mut contacts_enabled : Bool
  mut coupled_axes : JointAxesMask3DReal
  mut local_anchor1 : @core.Vec3
  mut local_anchor2 : @core.Vec3
  // Joint frame rotation in body1 and body2 local-spaces (defaults to identity).
  mut local_basis1 : @core.Quat
  mut local_basis2 : @core.Quat
  axis_states : Array[AxisState3DReal]
  motors : Array[MotorState3DReal]
}

///|
pub fn GenericJoint3DRealBuilder::new(
  locked_axes : JointAxesMask3DReal,
) -> GenericJoint3DRealBuilder {
  let axis_states : Array[AxisState3DReal] = []
  let motors : Array[MotorState3DReal] = []
  for i in 0..<6 {
    i |> ignore
    axis_states.push(AxisState3DReal::Free)
    motors.push(MotorState3DReal::Disabled)
  }
  // Locked axes default to "Locked" unless overridden by limits.
  for i in 0..<6 {
    let bit = 1 << i
    if (locked_axes.bits() & bit) != 0 {
      axis_states[i] = AxisState3DReal::Locked
    }
  }
  {
    locked_axes,
    contacts_enabled: true,
    coupled_axes: JointAxesMask3DReal::empty(),
    local_anchor1: @core.Vec3::zero(),
    local_anchor2: @core.Vec3::zero(),
    local_basis1: @core.Quat::identity(),
    local_basis2: @core.Quat::identity(),
    axis_states,
    motors,
  }
}

///|
pub fn GenericJoint3DRealBuilder::contacts_enabled(
  self : GenericJoint3DRealBuilder,
  enabled : Bool,
) -> GenericJoint3DRealBuilder {
  self.contacts_enabled = enabled
  self
}

///|
pub fn GenericJoint3DRealBuilder::local_anchor1(
  self : GenericJoint3DRealBuilder,
  local_anchor1 : @core.Vec3,
) -> GenericJoint3DRealBuilder {
  self.local_anchor1 = local_anchor1
  self
}

///|
pub fn GenericJoint3DRealBuilder::local_anchor2(
  self : GenericJoint3DRealBuilder,
  local_anchor2 : @core.Vec3,
) -> GenericJoint3DRealBuilder {
  self.local_anchor2 = local_anchor2
  self
}

///|
pub fn GenericJoint3DRealBuilder::local_basis1(
  self : GenericJoint3DRealBuilder,
  local_basis1 : @core.Quat,
) -> GenericJoint3DRealBuilder {
  self.local_basis1 = local_basis1
  self
}

///|
pub fn GenericJoint3DRealBuilder::local_basis2(
  self : GenericJoint3DRealBuilder,
  local_basis2 : @core.Quat,
) -> GenericJoint3DRealBuilder {
  self.local_basis2 = local_basis2
  self
}

///|
pub fn GenericJoint3DRealBuilder::local_axis1(
  self : GenericJoint3DRealBuilder,
  axis : @core.Vec3,
) -> GenericJoint3DRealBuilder {
  let a = if axis.length_squared() <= 1.0e-12F {
    @core.Vec3::new(1.0F, 0.0F, 0.0F)
  } else {
    axis.normalize()
  }
  self.local_basis1 = @core.rotation_between(
    @core.Vec3::new(1.0F, 0.0F, 0.0F),
    a,
  )
  self
}

///|
pub fn GenericJoint3DRealBuilder::local_axis2(
  self : GenericJoint3DRealBuilder,
  axis : @core.Vec3,
) -> GenericJoint3DRealBuilder {
  let a = if axis.length_squared() <= 1.0e-12F {
    @core.Vec3::new(1.0F, 0.0F, 0.0F)
  } else {
    axis.normalize()
  }
  self.local_basis2 = @core.rotation_between(
    @core.Vec3::new(1.0F, 0.0F, 0.0F),
    a,
  )
  self
}

///|
pub fn GenericJoint3DRealBuilder::limits(
  self : GenericJoint3DRealBuilder,
  axis : JointAxis3DReal,
  min : @core.Real,
  max : @core.Real,
) -> GenericJoint3DRealBuilder {
  let i = axis_index_3d_real(axis)
  self.axis_states[i] = AxisState3DReal::Limited(min, max)
  self
}

///|
pub fn GenericJoint3DRealBuilder::motor_position(
  self : GenericJoint3DRealBuilder,
  axis : JointAxis3DReal,
  target_pos : @core.Real,
  stiffness : @core.Real,
  damping : @core.Real,
) -> GenericJoint3DRealBuilder {
  let i = axis_index_3d_real(axis)
  self.motors[i] = MotorState3DReal::Position(
    target_pos, stiffness, damping, 1.0e30F,
  )
  self
}

///|
pub fn GenericJoint3DRealBuilder::motor_velocity(
  self : GenericJoint3DRealBuilder,
  axis : JointAxis3DReal,
  target_vel : @core.Real,
  damping : @core.Real,
) -> GenericJoint3DRealBuilder {
  let i = axis_index_3d_real(axis)
  self.motors[i] = MotorState3DReal::Velocity(target_vel, damping, 1.0e30F)
  self
}

///|
pub fn GenericJoint3DRealBuilder::motor_max_force(
  self : GenericJoint3DRealBuilder,
  axis : JointAxis3DReal,
  max_force : @core.Real,
) -> GenericJoint3DRealBuilder {
  let i = axis_index_3d_real(axis)
  match self.motors[i] {
    MotorState3DReal::Disabled => ()
    MotorState3DReal::Velocity(target_vel, damping, _) =>
      self.motors[i] = MotorState3DReal::Velocity(
        target_vel, damping, max_force,
      )
    MotorState3DReal::Position(target_pos, stiffness, damping, _) =>
      self.motors[i] = MotorState3DReal::Position(
        target_pos, stiffness, damping, max_force,
      )
  }
  self
}

///|
pub fn GenericJoint3DRealBuilder::coupled_axes(
  self : GenericJoint3DRealBuilder,
  coupled_axes : JointAxesMask3DReal,
) -> GenericJoint3DRealBuilder {
  self.coupled_axes = coupled_axes
  self
}

///|
pub fn GenericJoint3DRealBuilder::build(
  self : GenericJoint3DRealBuilder,
) -> GenericJoint3DReal {
  GenericJoint3DReal::{
    local_frame1: @core.Isometry3::new(self.local_anchor1, self.local_basis1),
    local_frame2: @core.Isometry3::new(self.local_anchor2, self.local_basis2),
    axis_states: self.axis_states,
    motors: self.motors,
    coupled_axes: self.coupled_axes,
    contacts_enabled: self.contacts_enabled,
  }
}
