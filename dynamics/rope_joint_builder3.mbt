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
/// 3D-flavored rope joint builder used by the rapier3d parity tests.
///
/// NOTE: This maps to the existing 2D rope joint builder by dropping the `z` coordinate.
pub struct RopeJointBuilder3 {
  inner : RopeJointBuilder
}

///|
pub fn RopeJointBuilder3::new(max_length : @core.Real) -> RopeJointBuilder3 {
  { inner: RopeJointBuilder::new(max_length) }
}

///|
pub fn RopeJointBuilder3::local_anchor1(
  self : RopeJointBuilder3,
  anchor : @core.Vec3,
) -> RopeJointBuilder3 {
  { inner: self.inner.local_anchor1(@core.Vec2::new(anchor.x, anchor.y)) }
}

///|
pub fn RopeJointBuilder3::local_anchor2(
  self : RopeJointBuilder3,
  anchor : @core.Vec3,
) -> RopeJointBuilder3 {
  { inner: self.inner.local_anchor2(@core.Vec2::new(anchor.x, anchor.y)) }
}

///|
pub fn RopeJointBuilder3::build(self : RopeJointBuilder3) -> RopeJoint {
  self.inner.build()
}
