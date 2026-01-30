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
/// 3D revolute joint. This is the dim3 counterpart of the 2D `RevoluteJointBuilder`/`GenericJoint`
/// path used in the rapier2d-aligned API.
pub struct RevoluteJoint3 {
  axis : @core.Vec3
  local_frame1_rotation : @core.Quat
  local_frame2_rotation : @core.Quat
}

///|
pub fn RevoluteJoint3::new(axis : @core.Vec3) -> RevoluteJoint3 {
  let axis_n = axis.normalize()
  // Rapier expresses the free revolute DOF as AngX in joint-space. Build a frame
  // rotation whose local +X axis corresponds to the provided hinge axis.
  let joint_frame = @core.rotation_between(
    @core.Vec3::new(1.0F, 0.0F, 0.0F),
    axis_n,
  )
  RevoluteJoint3::{
    axis: axis_n,
    local_frame1_rotation: joint_frame,
    local_frame2_rotation: joint_frame,
  }
}

///|
/// The angle along the free degree of freedom of this revolute joint in `[-pi, pi]`.
pub fn RevoluteJoint3::angle(
  self : RevoluteJoint3,
  rb_rot1 : @core.Quat,
  rb_rot2 : @core.Quat,
) -> @core.Real {
  let joint_rot1 = rb_rot1.mul(self.local_frame1_rotation)
  let joint_rot2 = rb_rot2.mul(self.local_frame2_rotation)
  let ang_err = joint_rot1.inverse().mul(joint_rot2)
  let raw = @core.asin(@core.clamp(ang_err.x, -1.0F, 1.0F)) * 2.0F
  if joint_rot1.dot(joint_rot2) < 0.0F {
    -raw
  } else {
    raw
  }
}

///|
pub struct RevoluteJoint3Builder(RevoluteJoint3)

///|
pub fn RevoluteJoint3Builder::new(axis : @core.Vec3) -> RevoluteJoint3Builder {
  RevoluteJoint3Builder(RevoluteJoint3::new(axis))
}

///|
pub fn RevoluteJoint3Builder::build(
  self : RevoluteJoint3Builder,
) -> RevoluteJoint3 {
  let RevoluteJoint3Builder(j) = self
  j
}
