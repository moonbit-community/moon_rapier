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
