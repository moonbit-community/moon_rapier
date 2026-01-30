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
/// Pose error between two isometries, expressed at the center of mass.
pub struct PdErrors3 {
  linear : @core.Vec3
  angular : @core.Vec3
}

///|
pub struct RigidBodyPosition3 {
  position : @core.Isometry3
  next_position : @core.Isometry3
}

///|
pub fn RigidBodyPosition3::new(
  position : @core.Isometry3,
  next_position : @core.Isometry3,
) -> RigidBodyPosition3 {
  RigidBodyPosition3::{ position, next_position }
}

///|
/// Computes the difference between `next_position` and `position`, expressed at the center of mass.
pub fn RigidBodyPosition3::pose_errors(
  self : RigidBodyPosition3,
  local_com : @core.Vec3,
) -> PdErrors3 {
  let com = self.position.transform_point(local_com)
  let shift = @core.Isometry3::from_translation(com)
  let dpos = shift
    .inverse()
    .mul(self.next_position)
    .mul(self.position.inverse())
    .mul(shift)
  PdErrors3::{
    linear: dpos.translation,
    angular: dpos.rotation.to_scaled_axis(),
  }
}

///|
/// Computes the velocity needed to travel from `position` to `next_position` in a time `1.0 / inv_dt`.
pub fn RigidBodyPosition3::interpolate_velocity(
  self : RigidBodyPosition3,
  inv_dt : @core.Real,
  local_com : @core.Vec3,
) -> RigidBodyVelocity3 {
  let pose_err = self.pose_errors(local_com)
  RigidBodyVelocity3::{
    linvel: pose_err.linear.scale(inv_dt),
    angvel: pose_err.angular.scale(inv_dt),
  }
}

///|
pub struct RigidBodyVelocity3 {
  linvel : @core.Vec3
  angvel : @core.Vec3
}

///|
pub fn RigidBodyVelocity3::integrate(
  self : RigidBodyVelocity3,
  dt : @core.Real,
  curr_pos : @core.Isometry3,
  local_com : @core.Vec3,
) -> @core.Isometry3 {
  let com = curr_pos.transform_point(local_com)
  let shift = @core.Isometry3::from_translation(
    @core.Vec3::new(-com.x, -com.y, -com.z),
  )
  let rot = @core.Isometry3::new(
    @core.Vec3::zero(),
    @core.rotation_from_scaled_axis(self.angvel.scale(dt)),
  )
  let translation = self.linvel.scale(dt)
  let unshift = @core.Isometry3::from_translation(com.add(translation))
  unshift.mul(rot).mul(shift).mul(curr_pos)
}
