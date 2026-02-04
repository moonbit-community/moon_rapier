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
/// A spring joint that pulls/pushes two bodies toward a target distance.
///
/// This is a thin wrapper around `GenericJoint`, matching Rapier's public surface
/// (capability-wise) for default features.
pub struct SpringJoint {
  mut data : GenericJoint
}

///|
pub fn SpringJoint::new(
  rest_length : @core.Real,
  stiffness : @core.Real,
  damping : @core.Real,
) -> SpringJoint {
  let joint = GenericJoint::default()
    .set_coupled_axes(JointAxesMask::lin_axes())
    .set_motor_position(JointAxis::LinX, rest_length, stiffness, damping)
    .set_motor_model(JointAxis::LinX, MotorModel::ForceBased)
  { data: joint }
}

///|
pub fn SpringJoint::data(self : SpringJoint) -> GenericJoint {
  self.data
}

///|
pub fn SpringJoint::contacts_enabled(self : SpringJoint) -> Bool {
  self.data.contacts_enabled()
}

///|
pub fn SpringJoint::set_contacts_enabled(
  self : SpringJoint,
  enabled : Bool,
) -> SpringJoint {
  self.data = self.data.set_contacts_enabled(enabled)
  self
}

///|
pub fn SpringJoint::local_anchor1(self : SpringJoint) -> @core.Vec2 {
  self.data.local_anchor1()
}

///|
pub fn SpringJoint::set_local_anchor1(
  self : SpringJoint,
  anchor : @core.Vec2,
) -> SpringJoint {
  self.data = self.data.set_local_anchor1(anchor)
  self
}

///|
pub fn SpringJoint::local_anchor2(self : SpringJoint) -> @core.Vec2 {
  self.data.local_anchor2()
}

///|
pub fn SpringJoint::set_local_anchor2(
  self : SpringJoint,
  anchor : @core.Vec2,
) -> SpringJoint {
  self.data = self.data.set_local_anchor2(anchor)
  self
}

///|
pub fn SpringJoint::set_spring_model(
  self : SpringJoint,
  model : MotorModel,
) -> SpringJoint {
  self.data = self.data.set_motor_model(JointAxis::LinX, model)
  self
}

///|
pub struct SpringJointBuilder {
  mut inner : SpringJoint
}

///|
pub fn SpringJointBuilder::new(
  rest_length : @core.Real,
  stiffness : @core.Real,
  damping : @core.Real,
) -> SpringJointBuilder {
  { inner: SpringJoint::new(rest_length, stiffness, damping) }
}

///|
pub fn SpringJointBuilder::contacts_enabled(
  self : SpringJointBuilder,
  enabled : Bool,
) -> SpringJointBuilder {
  self.inner = self.inner.set_contacts_enabled(enabled)
  self
}

///|
pub fn SpringJointBuilder::local_anchor1(
  self : SpringJointBuilder,
  anchor : @core.Vec2,
) -> SpringJointBuilder {
  self.inner = self.inner.set_local_anchor1(anchor)
  self
}

///|
pub fn SpringJointBuilder::local_anchor2(
  self : SpringJointBuilder,
  anchor : @core.Vec2,
) -> SpringJointBuilder {
  self.inner = self.inner.set_local_anchor2(anchor)
  self
}

///|
pub fn SpringJointBuilder::spring_model(
  self : SpringJointBuilder,
  model : MotorModel,
) -> SpringJointBuilder {
  self.inner = self.inner.set_spring_model(model)
  self
}

///|
pub fn SpringJointBuilder::build(self : SpringJointBuilder) -> SpringJoint {
  self.inner
}
