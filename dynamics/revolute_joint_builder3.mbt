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
///
/// NOTE: This maps to the existing 2D revolute joint builder. The `axis` is ignored and the
/// z-component of anchors is dropped.
pub struct RevoluteJointBuilder3 {
  inner : RevoluteJointBuilder
}

///|
pub fn RevoluteJointBuilder3::new(_axis : @core.Vec3) -> RevoluteJointBuilder3 {
  { inner: RevoluteJointBuilder::new() }
}

///|
pub fn RevoluteJointBuilder3::local_anchor1(
  self : RevoluteJointBuilder3,
  anchor1 : @core.Vec3,
) -> RevoluteJointBuilder3 {
  { inner: self.inner.local_anchor1(@core.Vec2::new(anchor1.x, anchor1.y)) }
}

///|
pub fn RevoluteJointBuilder3::local_anchor2(
  self : RevoluteJointBuilder3,
  anchor2 : @core.Vec3,
) -> RevoluteJointBuilder3 {
  { inner: self.inner.local_anchor2(@core.Vec2::new(anchor2.x, anchor2.y)) }
}

///|
pub fn RevoluteJointBuilder3::build(
  self : RevoluteJointBuilder3,
) -> RevoluteJoint {
  self.inner.build()
}
