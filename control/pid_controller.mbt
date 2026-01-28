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
pub(all) struct PdErrors {
  linear : @core.Vec2
  angular : @core.Real
}

///|
pub fn PdErrors::from_vels(vels : @dynamics.RigidBodyVelocity) -> PdErrors {
  { linear: vels.linvel(), angular: vels.angvel() }
}

///|
pub(all) struct PdController {
  lin_kp : @core.Vec2
  lin_kd : @core.Vec2
  ang_kp : @core.Real
  ang_kd : @core.Real
  axes : @dynamics.AxesMask
}

///|
pub fn PdController::default() -> PdController {
  PdController::new(60.0F, 0.8F, @dynamics.AxesMask::all())
}

///|
pub fn PdController::new(
  kp : @core.Real,
  kd : @core.Real,
  axes : @dynamics.AxesMask,
) -> PdController {
  {
    lin_kp: @core.Vec2::new(kp, kp),
    lin_kd: @core.Vec2::new(kd, kd),
    ang_kp: kp,
    ang_kd: kd,
    axes,
  }
}

///|
fn PdController::lin_mask(self : PdController) -> @core.Vec2 {
  let x = if self.axes.contains(@dynamics.AxesMask::lin_x()) {
    1.0F
  } else {
    0.0F
  }
  let y = if self.axes.contains(@dynamics.AxesMask::lin_y()) {
    1.0F
  } else {
    0.0F
  }
  @core.Vec2::new(x, y)
}

///|
fn PdController::ang_mask(self : PdController) -> @core.Real {
  if self.axes.contains(@dynamics.AxesMask::ang_z()) {
    1.0F
  } else {
    0.0F
  }
}

///|
fn PdController::with_axes(
  self : PdController,
  axes : @dynamics.AxesMask,
) -> PdController {
  {
    lin_kp: self.lin_kp,
    lin_kd: self.lin_kd,
    ang_kp: self.ang_kp,
    ang_kd: self.ang_kd,
    axes,
  }
}

///|
fn wrap_angle_pi(angle : @core.Real) -> @core.Real {
  let pi_value = @core.pi()
  let two_pi = @core.two_pi()
  let mut a = angle
  if a > pi_value {
    a = a - two_pi
  } else if a < -pi_value {
    a = a + two_pi
  }
  a
}

///|
fn pose_errors_at_com(
  current_pose : @core.Isometry2,
  target_pose : @core.Isometry2,
  local_com : @core.Vec2,
) -> PdErrors {
  let curr_com = current_pose.transform_point(local_com)
  let target_com = target_pose.transform_point(local_com)
  let linear = target_com.sub(curr_com)
  let curr_angle = current_pose.rotation.angle()
  let target_angle = target_pose.rotation.angle()
  let angular = wrap_angle_pi(target_angle - curr_angle)
  { linear, angular }
}

///|
pub fn PdController::linear_rigid_body_correction(
  self : PdController,
  rb : @dynamics.RigidBody,
  target_pos : @core.Vec2,
  target_linvel : @core.Vec2,
) -> @core.Vec2 {
  let angvel = rb.angvel()
  let vels = @dynamics.RigidBodyVelocity::new(target_linvel, angvel)
  self
  .rigid_body_correction(
    rb,
    @core.Isometry2::from_translation(target_pos),
    vels,
  )
  .linvel()
}

///|
pub fn PdController::angular_rigid_body_correction(
  self : PdController,
  rb : @dynamics.RigidBody,
  target_rot : @core.Rot2,
  target_angvel : @core.Real,
) -> @core.Real {
  let vels = @dynamics.RigidBodyVelocity::new(rb.linvel(), target_angvel)
  self
  .rigid_body_correction(
    rb,
    @core.Isometry2::new(@core.Vec2::zero(), target_rot),
    vels,
  )
  .angvel()
}

///|
pub fn PdController::rigid_body_correction(
  self : PdController,
  rb : @dynamics.RigidBody,
  target_pose : @core.Isometry2,
  target_vels : @dynamics.RigidBodyVelocity,
) -> @dynamics.RigidBodyVelocity {
  let pose_errors = pose_errors_at_com(
    rb.position(),
    target_pose,
    rb.center_of_mass(),
  )
  let curr_vels = rb.vels()
  let vel_errors = PdErrors::{
    linear: target_vels.linvel().sub(curr_vels.linvel()),
    angular: target_vels.angvel() - curr_vels.angvel(),
  }
  self.correction(pose_errors, vel_errors)
}

///|
pub fn PdController::correction(
  self : PdController,
  pose_errors : PdErrors,
  vel_errors : PdErrors,
) -> @dynamics.RigidBodyVelocity {
  let lin_mask = self.lin_mask()
  let ang_mask = self.ang_mask()
  let linvel = @core.Vec2::new(
    (pose_errors.linear.x * self.lin_kp.x + vel_errors.linear.x * self.lin_kd.x) *
    lin_mask.x,
    (pose_errors.linear.y * self.lin_kp.y + vel_errors.linear.y * self.lin_kd.y) *
    lin_mask.y,
  )
  let angvel = (
      pose_errors.angular * self.ang_kp + vel_errors.angular * self.ang_kd
    ) *
    ang_mask
  @dynamics.RigidBodyVelocity::new(linvel, angvel)
}

///|
pub(all) struct PidController {
  mut pd : PdController
  mut lin_integral : @core.Vec2
  mut ang_integral : @core.Real
  lin_ki : @core.Vec2
  ang_ki : @core.Real
}

///|
pub fn PidController::default() -> PidController {
  PidController::new(60.0F, 1.0F, 0.8F, @dynamics.AxesMask::all())
}

///|
pub fn PidController::new(
  kp : @core.Real,
  ki : @core.Real,
  kd : @core.Real,
  axes : @dynamics.AxesMask,
) -> PidController {
  {
    pd: PdController::new(kp, kd, axes),
    lin_integral: @core.Vec2::zero(),
    ang_integral: 0.0F,
    lin_ki: @core.Vec2::new(ki, ki),
    ang_ki: ki,
  }
}

///|
pub fn PidController::set_axes(
  self : PidController,
  axes : @dynamics.AxesMask,
) -> PidController {
  self.pd = self.pd.with_axes(axes)
  self
}

///|
pub fn PidController::axes(self : PidController) -> @dynamics.AxesMask {
  self.pd.axes
}

///|
pub fn PidController::reset_integrals(self : PidController) -> PidController {
  self.lin_integral = @core.Vec2::zero()
  self.ang_integral = 0.0F
  self
}

///|
pub fn PidController::linear_rigid_body_correction(
  self : PidController,
  dt : @core.Real,
  rb : @dynamics.RigidBody,
  target_pos : @core.Vec2,
  target_linvel : @core.Vec2,
) -> @core.Vec2 {
  let vels = @dynamics.RigidBodyVelocity::new(target_linvel, rb.angvel())
  self
  .rigid_body_correction(
    dt,
    rb,
    @core.Isometry2::from_translation(target_pos),
    vels,
  )
  .linvel()
}

///|
pub fn PidController::angular_rigid_body_correction(
  self : PidController,
  dt : @core.Real,
  rb : @dynamics.RigidBody,
  target_rot : @core.Rot2,
  target_angvel : @core.Real,
) -> @core.Real {
  let vels = @dynamics.RigidBodyVelocity::new(rb.linvel(), target_angvel)
  self
  .rigid_body_correction(
    dt,
    rb,
    @core.Isometry2::new(@core.Vec2::zero(), target_rot),
    vels,
  )
  .angvel()
}

///|
pub fn PidController::rigid_body_correction(
  self : PidController,
  dt : @core.Real,
  rb : @dynamics.RigidBody,
  target_pose : @core.Isometry2,
  target_vels : @dynamics.RigidBodyVelocity,
) -> @dynamics.RigidBodyVelocity {
  let pose_errors = pose_errors_at_com(
    rb.position(),
    target_pose,
    rb.center_of_mass(),
  )
  let curr_vels = rb.vels()
  let vel_errors = PdErrors::{
    linear: target_vels.linvel().sub(curr_vels.linvel()),
    angular: target_vels.angvel() - curr_vels.angvel(),
  }
  self.correction(dt, pose_errors, vel_errors)
}

///|
pub fn PidController::correction(
  self : PidController,
  dt : @core.Real,
  pose_errors : PdErrors,
  vel_errors : PdErrors,
) -> @dynamics.RigidBodyVelocity {
  self.lin_integral = self.lin_integral.add(
    @core.Vec2::new(pose_errors.linear.x * dt, pose_errors.linear.y * dt),
  )
  self.ang_integral = self.ang_integral + pose_errors.angular * dt
  let lin_mask = self.pd.lin_mask()
  let ang_mask = self.pd.ang_mask()
  let linvel = @core.Vec2::new(
    (
      pose_errors.linear.x * self.pd.lin_kp.x +
      vel_errors.linear.x * self.pd.lin_kd.x +
      self.lin_integral.x * self.lin_ki.x
    ) *
    lin_mask.x,
    (
      pose_errors.linear.y * self.pd.lin_kp.y +
      vel_errors.linear.y * self.pd.lin_kd.y +
      self.lin_integral.y * self.lin_ki.y
    ) *
    lin_mask.y,
  )
  let angvel = (
      pose_errors.angular * self.pd.ang_kp +
      vel_errors.angular * self.pd.ang_kd +
      self.ang_integral * self.ang_ki
    ) *
    ang_mask
  @dynamics.RigidBodyVelocity::new(linvel, angvel)
}
