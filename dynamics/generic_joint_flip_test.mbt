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
test "generic joint flip respects coupled axes" {
  let mut joint = GenericJoint::default()
  joint = joint
    .set_coupled_axes(JointAxesMask::lin_axes())
    .set_limits_axis(JointAxis::LinX, 1.0F, 2.0F)
    .set_limits_axis(JointAxis::LinY, -3.0F, 4.0F)
    .set_limits_axis(JointAxis::AngZ, 5.0F, 6.0F)
  joint = joint
    .set_motor(JointAxis::LinX, 1.25F, 2.5F, 0.0F, 0.0F)
    .set_motor(JointAxis::LinY, -0.5F, 0.75F, 0.0F, 0.0F)
    .set_motor(JointAxis::AngZ, 0.1F, -0.2F, 0.0F, 0.0F)

  // Capture "before flip" values because `flip()` mutates the joint in-place.
  let lin_x0 = joint.limits_axis(JointAxis::LinX).unwrap()
  let lin_y0 = joint.limits_axis(JointAxis::LinY).unwrap()
  let ang_z0 = joint.limits_axis(JointAxis::AngZ).unwrap()
  let lin_x0_min = lin_x0.min()
  let lin_x0_max = lin_x0.max()
  let lin_y0_min = lin_y0.min()
  let lin_y0_max = lin_y0.max()
  let ang_z0_min = ang_z0.min()
  let ang_z0_max = ang_z0.max()
  let motors0 = joint.motors()
  let mx0_vel = motors0.lin_x.target_vel
  let mx0_pos = motors0.lin_x.target_pos
  let my0_vel = motors0.lin_y.target_vel
  let my0_pos = motors0.lin_y.target_pos
  let mz0_vel = motors0.ang_z.target_vel
  let mz0_pos = motors0.ang_z.target_pos
  let flipped = joint.flip()
  if joint.limits_axis(JointAxis::LinX) is Some(l0) &&
    flipped.limits_axis(JointAxis::LinX) is Some(l1) {
    l0 |> ignore
    inspect(l1.min() == lin_x0_min, content="true")
    inspect(l1.max() == lin_x0_max, content="true")
  } else {
    inspect(false, content="true")
  }
  if joint.limits_axis(JointAxis::LinY) is Some(l0) &&
    flipped.limits_axis(JointAxis::LinY) is Some(l1) {
    l0 |> ignore
    inspect(l1.min() == lin_y0_min, content="true")
    inspect(l1.max() == lin_y0_max, content="true")
  } else {
    inspect(false, content="true")
  }
  if joint.limits_axis(JointAxis::AngZ) is Some(l0) &&
    flipped.limits_axis(JointAxis::AngZ) is Some(l1) {
    l0 |> ignore
    inspect(l1.min() == -ang_z0_max, content="true")
    inspect(l1.max() == -ang_z0_min, content="true")
  } else {
    inspect(false, content="true")
  }
  let m1 = flipped.motors()
  inspect(m1.lin_x.target_vel == -mx0_vel, content="true")
  inspect(m1.lin_x.target_pos == -mx0_pos, content="true")
  inspect(m1.lin_y.target_vel == -my0_vel, content="true")
  inspect(m1.lin_y.target_pos == -my0_pos, content="true")
  inspect(m1.ang_z.target_vel == -mz0_vel, content="true")
  inspect(m1.ang_z.target_pos == -mz0_pos, content="true")
}
