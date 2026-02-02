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
/// Higher-level joint builders for the real dim3 backend.
///
/// These mirror Rapier's per-joint builders on top of `GenericJoint3DRealBuilder` by selecting a
/// conventional joint frame:
/// - The joint's principal axis is the local X axis.
/// - Prismatic joints translate along +X.
/// - Revolute joints rotate around +X.

///|
pub struct RevoluteJoint3DRealBuilder {
  inner : GenericJoint3DRealBuilder
}

///|
pub fn RevoluteJoint3DRealBuilder::new(
  axis : @core.Vec3,
) -> RevoluteJoint3DRealBuilder {
  let locked = JointAxesMask3DReal::lin_axes()
    .or(JointAxesMask3DReal::ang_y())
    .or(JointAxesMask3DReal::ang_z())
  let inner = GenericJoint3DRealBuilder::new(locked)
    .local_axis1(axis)
    .local_axis2(axis)
  { inner, }
}

///|
pub fn RevoluteJoint3DRealBuilder::local_anchor1(
  self : RevoluteJoint3DRealBuilder,
  a : @core.Vec3,
) -> RevoluteJoint3DRealBuilder {
  { inner: self.inner.local_anchor1(a) }
}

///|
pub fn RevoluteJoint3DRealBuilder::local_anchor2(
  self : RevoluteJoint3DRealBuilder,
  a : @core.Vec3,
) -> RevoluteJoint3DRealBuilder {
  { inner: self.inner.local_anchor2(a) }
}

///|
pub fn RevoluteJoint3DRealBuilder::limits(
  self : RevoluteJoint3DRealBuilder,
  min : @core.Real,
  max : @core.Real,
) -> RevoluteJoint3DRealBuilder {
  { inner: self.inner.limits(JointAxis3DReal::AngX, min, max) }
}

///|
pub fn RevoluteJoint3DRealBuilder::motor_position(
  self : RevoluteJoint3DRealBuilder,
  target_pos : @core.Real,
  stiffness : @core.Real,
  damping : @core.Real,
) -> RevoluteJoint3DRealBuilder {
  {
    inner: self.inner.motor_position(
      JointAxis3DReal::AngX,
      target_pos,
      stiffness,
      damping,
    ),
  }
}

///|
pub fn RevoluteJoint3DRealBuilder::motor_velocity(
  self : RevoluteJoint3DRealBuilder,
  target_vel : @core.Real,
  damping : @core.Real,
) -> RevoluteJoint3DRealBuilder {
  {
    inner: self.inner.motor_velocity(JointAxis3DReal::AngX, target_vel, damping),
  }
}

///|
pub fn RevoluteJoint3DRealBuilder::motor_max_force(
  self : RevoluteJoint3DRealBuilder,
  max_force : @core.Real,
) -> RevoluteJoint3DRealBuilder {
  { inner: self.inner.motor_max_force(JointAxis3DReal::AngX, max_force) }
}

///|
pub fn RevoluteJoint3DRealBuilder::build(
  self : RevoluteJoint3DRealBuilder,
) -> GenericJoint3DReal {
  self.inner.build()
}

///|
pub struct PrismaticJoint3DRealBuilder {
  inner : GenericJoint3DRealBuilder
}

///|
pub fn PrismaticJoint3DRealBuilder::new(
  axis : @core.Vec3,
) -> PrismaticJoint3DRealBuilder {
  let locked = JointAxesMask3DReal::lin_y()
    .or(JointAxesMask3DReal::lin_z())
    .or(JointAxesMask3DReal::ang_axes())
  let inner = GenericJoint3DRealBuilder::new(locked)
    .local_axis1(axis)
    .local_axis2(axis)
  { inner, }
}

///|
pub fn PrismaticJoint3DRealBuilder::local_anchor1(
  self : PrismaticJoint3DRealBuilder,
  a : @core.Vec3,
) -> PrismaticJoint3DRealBuilder {
  { inner: self.inner.local_anchor1(a) }
}

///|
pub fn PrismaticJoint3DRealBuilder::local_anchor2(
  self : PrismaticJoint3DRealBuilder,
  a : @core.Vec3,
) -> PrismaticJoint3DRealBuilder {
  { inner: self.inner.local_anchor2(a) }
}

///|
pub fn PrismaticJoint3DRealBuilder::limits(
  self : PrismaticJoint3DRealBuilder,
  min : @core.Real,
  max : @core.Real,
) -> PrismaticJoint3DRealBuilder {
  { inner: self.inner.limits(JointAxis3DReal::LinX, min, max) }
}

///|
pub fn PrismaticJoint3DRealBuilder::motor_position(
  self : PrismaticJoint3DRealBuilder,
  target_pos : @core.Real,
  stiffness : @core.Real,
  damping : @core.Real,
) -> PrismaticJoint3DRealBuilder {
  {
    inner: self.inner.motor_position(
      JointAxis3DReal::LinX,
      target_pos,
      stiffness,
      damping,
    ),
  }
}

///|
pub fn PrismaticJoint3DRealBuilder::motor_velocity(
  self : PrismaticJoint3DRealBuilder,
  target_vel : @core.Real,
  damping : @core.Real,
) -> PrismaticJoint3DRealBuilder {
  {
    inner: self.inner.motor_velocity(JointAxis3DReal::LinX, target_vel, damping),
  }
}

///|
pub fn PrismaticJoint3DRealBuilder::motor_max_force(
  self : PrismaticJoint3DRealBuilder,
  max_force : @core.Real,
) -> PrismaticJoint3DRealBuilder {
  { inner: self.inner.motor_max_force(JointAxis3DReal::LinX, max_force) }
}

///|
pub fn PrismaticJoint3DRealBuilder::build(
  self : PrismaticJoint3DRealBuilder,
) -> GenericJoint3DReal {
  self.inner.build()
}

///|
pub struct FixedJoint3DRealBuilder {
  inner : GenericJoint3DRealBuilder
}

///|
pub fn FixedJoint3DRealBuilder::new() -> FixedJoint3DRealBuilder {
  { inner: GenericJoint3DRealBuilder::new(JointAxesMask3DReal::all()) }
}

///|
pub fn FixedJoint3DRealBuilder::local_anchor1(
  self : FixedJoint3DRealBuilder,
  a : @core.Vec3,
) -> FixedJoint3DRealBuilder {
  { inner: self.inner.local_anchor1(a) }
}

///|
pub fn FixedJoint3DRealBuilder::local_anchor2(
  self : FixedJoint3DRealBuilder,
  a : @core.Vec3,
) -> FixedJoint3DRealBuilder {
  { inner: self.inner.local_anchor2(a) }
}

///|
pub fn FixedJoint3DRealBuilder::local_basis1(
  self : FixedJoint3DRealBuilder,
  q : @core.Quat,
) -> FixedJoint3DRealBuilder {
  { inner: self.inner.local_basis1(q) }
}

///|
pub fn FixedJoint3DRealBuilder::local_basis2(
  self : FixedJoint3DRealBuilder,
  q : @core.Quat,
) -> FixedJoint3DRealBuilder {
  { inner: self.inner.local_basis2(q) }
}

///|
pub fn FixedJoint3DRealBuilder::build(
  self : FixedJoint3DRealBuilder,
) -> GenericJoint3DReal {
  self.inner.build()
}

///|
pub struct SphericalJoint3DRealBuilder {
  inner : GenericJoint3DRealBuilder
}

///|
pub fn SphericalJoint3DRealBuilder::new() -> SphericalJoint3DRealBuilder {
  { inner: GenericJoint3DRealBuilder::new(JointAxesMask3DReal::lin_axes()) }
}

///|
pub fn SphericalJoint3DRealBuilder::local_anchor1(
  self : SphericalJoint3DRealBuilder,
  a : @core.Vec3,
) -> SphericalJoint3DRealBuilder {
  { inner: self.inner.local_anchor1(a) }
}

///|
pub fn SphericalJoint3DRealBuilder::local_anchor2(
  self : SphericalJoint3DRealBuilder,
  a : @core.Vec3,
) -> SphericalJoint3DRealBuilder {
  { inner: self.inner.local_anchor2(a) }
}

///|
pub fn SphericalJoint3DRealBuilder::limits(
  self : SphericalJoint3DRealBuilder,
  axis : JointAxis3DReal,
  min : @core.Real,
  max : @core.Real,
) -> SphericalJoint3DRealBuilder {
  { inner: self.inner.limits(axis, min, max) }
}

///|
pub fn SphericalJoint3DRealBuilder::motor_position(
  self : SphericalJoint3DRealBuilder,
  axis : JointAxis3DReal,
  target_pos : @core.Real,
  stiffness : @core.Real,
  damping : @core.Real,
) -> SphericalJoint3DRealBuilder {
  { inner: self.inner.motor_position(axis, target_pos, stiffness, damping) }
}

///|
pub fn SphericalJoint3DRealBuilder::motor_velocity(
  self : SphericalJoint3DRealBuilder,
  axis : JointAxis3DReal,
  target_vel : @core.Real,
  damping : @core.Real,
) -> SphericalJoint3DRealBuilder {
  { inner: self.inner.motor_velocity(axis, target_vel, damping) }
}

///|
pub fn SphericalJoint3DRealBuilder::motor_max_force(
  self : SphericalJoint3DRealBuilder,
  axis : JointAxis3DReal,
  max_force : @core.Real,
) -> SphericalJoint3DRealBuilder {
  { inner: self.inner.motor_max_force(axis, max_force) }
}

///|
pub fn SphericalJoint3DRealBuilder::build(
  self : SphericalJoint3DRealBuilder,
) -> GenericJoint3DReal {
  self.inner.build()
}
