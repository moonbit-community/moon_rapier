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
/// Joint constraints for the real dim3 backend.
///
/// This is intentionally much smaller than Rapier's impulse-based joint solver and is used to
/// migrate 3D scenes to the `PhysicsPipeline3DReal` backend incrementally.
pub struct SphericalJoint3DReal {
  body1 : RigidBodyHandle
  body2 : RigidBodyHandle
  local_anchor1 : @core.Vec3
  local_anchor2 : @core.Vec3
}

///|
pub struct PrismaticJoint3DReal {
  body1 : RigidBodyHandle
  body2 : RigidBodyHandle
  local_anchor1 : @core.Vec3
  local_anchor2 : @core.Vec3
  // Axis expressed in body1 local-space.
  local_axis1 : @core.Vec3
  motor_target : @core.Real
  motor_stiffness : @core.Real
  motor_damping : @core.Real
  motor_enabled : Bool
}

///|
pub struct SphericalMotor3DReal {
  body1 : RigidBodyHandle
  body2 : RigidBodyHandle
  // Target relative rotation: q_target = q1^{-1} * q2 (in body1 local-space).
  target_rel_rotation : @core.Quat
  stiffness : @core.Real
  damping : @core.Real
}

///|
/// A handle to a joint stored inside a `JointSet3DReal`.
///
/// This currently only references joints created via `insert_generic` and its helpers.
pub struct JointHandle3DReal {
  index : Int
}

///|
/// Generic 3D joint configuration used by the real dim3 backend.
///
/// The constraint is defined in a joint frame attached to each body:
/// `body.pos * local_frame1` and `body.pos * local_frame2`.
/// Axis limits/motors are expressed in the joint frame of body1.
pub struct GenericJoint3DReal {
  local_frame1 : @core.Isometry3
  local_frame2 : @core.Isometry3
  axis_states : Array[AxisState3DReal]
  motors : Array[MotorState3DReal]
  coupled_axes : JointAxesMask3DReal
  contacts_enabled : Bool
}

///|
fn generic_joint_axis_from_index(index : Int) -> JointAxis {
  match index {
    0 => JointAxis::LinX
    1 => JointAxis::LinY
    2 => JointAxis::LinZ
    3 => JointAxis::AngX
    4 => JointAxis::AngY
    _ => JointAxis::AngZ
  }
}

///|
fn generic_joint_mask_from_index(index : Int) -> JointAxesMask {
  JointAxesMask::new(1 << index)
}

///|
fn axis_state_from_generic_joint(
  joint : GenericJoint,
  index : Int,
) -> AxisState3DReal {
  let axis = generic_joint_axis_from_index(index)
  let mask = generic_joint_mask_from_index(index)
  if joint.locked_axes().contains(mask) {
    AxisState3DReal::Locked
  } else if joint.limit_axes().contains(mask) {
    let limits = joint_limits_for_axis(joint.limits(), axis)
    AxisState3DReal::Limited(limits.min(), limits.max())
  } else {
    AxisState3DReal::Free
  }
}

///|
fn motor_state_from_generic_joint(
  joint : GenericJoint,
  index : Int,
) -> MotorState3DReal {
  let axis = generic_joint_axis_from_index(index)
  let mask = generic_joint_mask_from_index(index)
  if !joint.motor_axes().contains(mask) {
    return MotorState3DReal::Disabled
  }
  let motor = joint_motor_for_axis(joint.motors(), axis)
  if motor.stiffness != 0.0F || motor.target_pos != 0.0F {
    MotorState3DReal::Position(
      motor.target_pos,
      motor.stiffness,
      motor.damping,
      motor.max_force,
    )
  } else {
    MotorState3DReal::Velocity(motor.target_vel, motor.damping, motor.max_force)
  }
}

///|
pub fn GenericJoint::to_3d_real(self : GenericJoint) -> GenericJoint3DReal {
  let axis_states : Array[AxisState3DReal] = []
  let motors : Array[MotorState3DReal] = []
  for index in 0..<6 {
    axis_states.push(axis_state_from_generic_joint(self, index))
    motors.push(motor_state_from_generic_joint(self, index))
  }
  GenericJoint3DReal::{
    local_frame1: self.local_frame1_3(),
    local_frame2: self.local_frame2_3(),
    axis_states,
    motors,
    coupled_axes: JointAxesMask3DReal::new(self.coupled_axes().bits()),
    contacts_enabled: self.contacts_enabled(),
  }
}

///|
pub fn GenericJoint::from_3d_real(joint : GenericJoint3DReal) -> GenericJoint {
  let mut out = GenericJoint::default()
    .set_local_frame1_3(joint.local_frame1)
    .set_local_frame2_3(joint.local_frame2)
    .set_coupled_axes(JointAxesMask::new(joint.coupled_axes.bits()))
    .set_contacts_enabled(joint.contacts_enabled)
  for index in 0..<6 {
    let axis = generic_joint_axis_from_index(index)
    let mask = generic_joint_mask_from_index(index)
    let state = if index < joint.axis_states.length() {
      joint.axis_states[index]
    } else {
      AxisState3DReal::Free
    }
    match state {
      AxisState3DReal::Free => ()
      AxisState3DReal::Locked =>
        out = out.set_locked_axes(out.locked_axes().or(mask))
      AxisState3DReal::Limited(min, max) =>
        out = out
          .set_limit_axes(out.limit_axes().or(mask))
          .set_limits_axis(axis, min, max)
    }
    let motor = if index < joint.motors.length() {
      joint.motors[index]
    } else {
      MotorState3DReal::Disabled
    }
    match motor {
      MotorState3DReal::Disabled => ()
      MotorState3DReal::Velocity(target_vel, damping, max_force) =>
        out = out
          .set_motor_velocity(axis, target_vel, damping)
          .set_motor_max_force(axis, max_force)
      MotorState3DReal::Position(target_pos, stiffness, damping, max_force) =>
        out = out
          .set_motor_position(axis, target_pos, stiffness, damping)
          .set_motor_max_force(axis, max_force)
    }
  }
  out
}

///|
pub fn GenericJoint3DReal::to_generic_joint(
  self : GenericJoint3DReal,
) -> GenericJoint {
  GenericJoint::from_3d_real(self)
}

///|
/// Internal storage type for generic constraints in `JointSet3DReal`.
pub struct GenericJointConstraint3DReal {
  body1 : RigidBodyHandle
  body2 : RigidBodyHandle
  data : GenericJoint3DReal
}

///|
pub struct SpringJoint3DReal {
  body1 : RigidBodyHandle
  body2 : RigidBodyHandle
  local_anchor1 : @core.Vec3
  local_anchor2 : @core.Vec3
  rest_length : @core.Real
  stiffness : @core.Real
  damping : @core.Real
}

///|
pub struct JointSet3DReal {
  spherical : Array[SphericalJoint3DReal]
  prismatic : Array[PrismaticJoint3DReal]
  spherical_motors : Array[SphericalMotor3DReal]
  generic : Array[GenericJointConstraint3DReal]
  springs : Array[SpringJoint3DReal]
  spherical_impulses : Array[@core.Vec3]
  prismatic_lock_impulses : Array[@core.Vec2]
  prismatic_motor_impulses : Array[@core.Real]
  prismatic_angular_impulses : Array[@core.Vec3]
  spherical_motor_impulses : Array[@core.Vec3]
  generic_lock_impulses : Array[Array[@core.Real]]
  generic_limit_impulses : Array[Array[@core.Real]]
  generic_motor_impulses : Array[Array[@core.Real]]
  generic_coupled_impulses : Array[@core.Real]
  spring_impulses : Array[@core.Real]

  fn new() -> JointSet3DReal
}

///|
pub fn JointSet3DReal::disabled_contact_pairs(
  self : JointSet3DReal,
) -> Array[(RigidBodyHandle, RigidBodyHandle)] {
  let out : Array[(RigidBodyHandle, RigidBodyHandle)] = []
  for g in self.generic {
    if !g.data.contacts_enabled {
      out.push((g.body1, g.body2))
    }
  }
  out
}

///|
pub fn JointSet3DReal::interaction_pairs(
  self : JointSet3DReal,
) -> Array[(RigidBodyHandle, RigidBodyHandle)] {
  let out : Array[(RigidBodyHandle, RigidBodyHandle)] = []
  for j in self.spherical {
    out.push((j.body1, j.body2))
  }
  for j in self.prismatic {
    out.push((j.body1, j.body2))
  }
  for j in self.spherical_motors {
    out.push((j.body1, j.body2))
  }
  for j in self.generic {
    out.push((j.body1, j.body2))
  }
  for j in self.springs {
    out.push((j.body1, j.body2))
  }
  out
}

///|
pub fn JointSet3DReal::new() -> JointSet3DReal {
  {
    spherical: [],
    prismatic: [],
    spherical_motors: [],
    generic: [],
    springs: [],
    spherical_impulses: [],
    prismatic_lock_impulses: [],
    prismatic_motor_impulses: [],
    prismatic_angular_impulses: [],
    spherical_motor_impulses: [],
    generic_lock_impulses: [],
    generic_limit_impulses: [],
    generic_motor_impulses: [],
    generic_coupled_impulses: [],
    spring_impulses: [],
  }
}

///|
pub fn JointSet3DReal::insert_spherical(
  self : JointSet3DReal,
  body1 : RigidBodyHandle,
  body2 : RigidBodyHandle,
  local_anchor1 : @core.Vec3,
  local_anchor2 : @core.Vec3,
) -> Unit {
  self.spherical.push({ body1, body2, local_anchor1, local_anchor2 })
  self.spherical_impulses.push(@core.Vec3::zero())
}

///|
pub fn JointSet3DReal::insert_prismatic_motor_position(
  self : JointSet3DReal,
  body1 : RigidBodyHandle,
  body2 : RigidBodyHandle,
  local_axis1 : @core.Vec3,
  local_anchor1 : @core.Vec3,
  local_anchor2 : @core.Vec3,
  target : @core.Real,
  stiffness : @core.Real,
  damping : @core.Real,
) -> Unit {
  self.prismatic.push({
    body1,
    body2,
    local_anchor1,
    local_anchor2,
    local_axis1,
    motor_target: target,
    motor_stiffness: stiffness,
    motor_damping: damping,
    motor_enabled: true,
  })
  self.prismatic_lock_impulses.push(@core.Vec2::zero())
  self.prismatic_motor_impulses.push(0.0F)
  self.prismatic_angular_impulses.push(@core.Vec3::zero())
}

///|
pub fn JointSet3DReal::insert_spherical_motor_rotation(
  self : JointSet3DReal,
  body1 : RigidBodyHandle,
  body2 : RigidBodyHandle,
  target_rel_rotation : @core.Quat,
  stiffness : @core.Real,
  damping : @core.Real,
) -> Unit {
  self.spherical_motors.push({
    body1,
    body2,
    target_rel_rotation,
    stiffness,
    damping,
  })
  self.spherical_motor_impulses.push(@core.Vec3::zero())
}

///|
pub fn JointSet3DReal::insert_generic(
  self : JointSet3DReal,
  body1 : RigidBodyHandle,
  body2 : RigidBodyHandle,
  data : GenericJoint3DReal,
  wake_up : Bool,
) -> JointHandle3DReal {
  wake_up |> ignore
  let idx = self.generic.length()
  self.generic.push({ body1, body2, data })
  let lock_impulses : Array[@core.Real] = []
  let limit_impulses : Array[@core.Real] = []
  let motor_impulses : Array[@core.Real] = []
  for _ in 0..<6 {
    lock_impulses.push(0.0F)
    limit_impulses.push(0.0F)
    motor_impulses.push(0.0F)
  }
  self.generic_lock_impulses.push(lock_impulses)
  self.generic_limit_impulses.push(limit_impulses)
  self.generic_motor_impulses.push(motor_impulses)
  self.generic_coupled_impulses.push(0.0F)
  { index: idx }
}

///|
pub fn JointSet3DReal::insert_revolute(
  self : JointSet3DReal,
  body1 : RigidBodyHandle,
  body2 : RigidBodyHandle,
  axis : @core.Vec3,
  local_anchor1 : @core.Vec3,
  local_anchor2 : @core.Vec3,
  wake_up : Bool,
) -> JointHandle3DReal {
  let a = normalized_or_default(axis)
  // Use the joint Z axis as the revolute axis.
  let basis = @core.rotation_between(@core.Vec3::new(0.0F, 0.0F, 1.0F), a)
  let locked = JointAxesMask3DReal::lin_axes()
    .or(JointAxesMask3DReal::ang_x())
    .or(JointAxesMask3DReal::ang_y())
  let joint = GenericJoint3DRealBuilder::new(locked)
    .local_anchor1(local_anchor1)
    .local_anchor2(local_anchor2)
    .local_basis1(basis)
    .local_basis2(basis)
    .build()
  JointSet3DReal::insert_generic(self, body1, body2, joint, wake_up)
}

///|
pub fn JointSet3DReal::insert_spring(
  self : JointSet3DReal,
  body1 : RigidBodyHandle,
  body2 : RigidBodyHandle,
  local_anchor1 : @core.Vec3,
  local_anchor2 : @core.Vec3,
  rest_length : @core.Real,
  stiffness : @core.Real,
  damping : @core.Real,
  wake_up : Bool,
) -> Unit {
  wake_up |> ignore
  self.springs.push({
    body1,
    body2,
    local_anchor1,
    local_anchor2,
    rest_length,
    stiffness,
    damping,
  })
  self.spring_impulses.push(0.0F)
}

///|
pub fn JointSet3DReal::set_motor_position(
  self : JointSet3DReal,
  handle : JointHandle3DReal,
  axis : JointAxis3DReal,
  target_pos : @core.Real,
  stiffness : @core.Real,
  damping : @core.Real,
  max_force : @core.Real,
) -> Unit {
  let i = axis_index_3d_real(axis)
  let idx = handle.index
  if idx < 0 || idx >= self.generic.length() {
    return
  }
  let j = self.generic[idx]
  j.data.motors[i] = MotorState3DReal::Position(
    target_pos, stiffness, damping, max_force,
  )
  self.generic[idx] = j
}

///|
pub fn JointSet3DReal::set_motor_velocity(
  self : JointSet3DReal,
  handle : JointHandle3DReal,
  axis : JointAxis3DReal,
  target_vel : @core.Real,
  damping : @core.Real,
  max_force : @core.Real,
) -> Unit {
  let i = axis_index_3d_real(axis)
  let idx = handle.index
  if idx < 0 || idx >= self.generic.length() {
    return
  }
  let j = self.generic[idx]
  j.data.motors[i] = MotorState3DReal::Velocity(target_vel, damping, max_force)
  self.generic[idx] = j
}

///|
const JOINT_SOLVER_BAUMGARTE_3D_REAL : @core.Real = 0.2F

///|
const JOINT_SOLVER_MAX_IMPULSE_3D_REAL : @core.Real = 1.0e30F

///|
const JOINT_SOLVER_EPSILON_3D_REAL : @core.Real = 1.0e-9F

///|
fn inv_mass3(rb : RigidBody3D) -> @core.Real {
  if rb.body_type().is_dynamic() {
    rb.mass_properties().inv_mass
  } else {
    0.0F
  }
}

///|
fn world_inv_inertia3(rb : RigidBody3D) -> @core.Mat3 {
  if !rb.body_type().is_dynamic() {
    return @core.Mat3::zero()
  }
  let rot = rb.rotation().to_mat3()
  let inv_local = rb.mass_properties().inv_inertia.into_mat3()
  rot.mul(inv_local).mul(rot.transpose())
}

///|
fn apply_body_impulse_delta3(
  body : RigidBody3D,
  linear_impulse : @core.Vec3,
  angular_impulse : @core.Vec3,
) -> Unit {
  let inv_m = inv_mass3(body)
  if inv_m <= 0.0F {
    return
  }
  if !body.is_active() &&
    (
      linear_impulse.length_squared() > 1.0e-12F ||
      angular_impulse.length_squared() > 1.0e-12F
    ) {
    body.wake_up()
  }
  if linear_impulse.length_squared() > 0.0F {
    body.set_linvel(body.linvel().add(linear_impulse.scale(inv_m)))
  }
  if angular_impulse.length_squared() > 0.0F {
    body.set_angvel(
      body.angvel().add(world_inv_inertia3(body).mul_vec3(angular_impulse)),
    )
  }
}

///|
fn normalized_or_default(v : @core.Vec3) -> @core.Vec3 {
  let n2 = v.length_squared()
  if n2 <= 1.0e-12F {
    @core.Vec3::new(1.0F, 0.0F, 0.0F)
  } else {
    v.scale(1.0F / Float::sqrt(n2))
  }
}

///|
fn apply_row_impulse3(
  rb1 : RigidBody3D,
  rb2 : RigidBody3D,
  lin_j1 : @core.Vec3,
  ang_j1 : @core.Vec3,
  lin_j2 : @core.Vec3,
  ang_j2 : @core.Vec3,
  impulse : @core.Real,
) -> Unit {
  if @core.abs(impulse) <= JOINT_SOLVER_EPSILON_3D_REAL {
    return
  }
  apply_body_impulse_delta3(rb1, lin_j1.scale(impulse), ang_j1.scale(impulse))
  apply_body_impulse_delta3(rb2, lin_j2.scale(impulse), ang_j2.scale(impulse))
}

///|
fn axis_value_from_vec3(v : @core.Vec3, axis_i : Int) -> @core.Real {
  if axis_i == 0 {
    v.x
  } else if axis_i == 1 {
    v.y
  } else {
    v.z
  }
}

///|
fn set_axis_value_vec3(
  value : @core.Vec3,
  axis_i : Int,
  component : @core.Real,
) -> @core.Vec3 {
  if axis_i == 0 {
    @core.Vec3::new(component, value.y, value.z)
  } else if axis_i == 1 {
    @core.Vec3::new(value.x, component, value.z)
  } else {
    @core.Vec3::new(value.x, value.y, component)
  }
}

///|
fn axis_value_from_vec2(v : @core.Vec2, axis_i : Int) -> @core.Real {
  if axis_i == 0 {
    v.x
  } else {
    v.y
  }
}

///|
fn set_axis_value_vec2(
  value : @core.Vec2,
  axis_i : Int,
  component : @core.Real,
) -> @core.Vec2 {
  if axis_i == 0 {
    @core.Vec2::new(component, value.y)
  } else {
    @core.Vec2::new(value.x, component)
  }
}

///|
fn axis_unit(axis_i : Int) -> @core.Vec3 {
  if axis_i == 0 {
    @core.Vec3::new(1.0F, 0.0F, 0.0F)
  } else if axis_i == 1 {
    @core.Vec3::new(0.0F, 1.0F, 0.0F)
  } else {
    @core.Vec3::new(0.0F, 0.0F, 1.0F)
  }
}

///|
fn orthonormal_basis3(n : @core.Vec3) -> (@core.Vec3, @core.Vec3) {
  let tangent = if @core.abs(n.x) < 0.57735026F {
    @core.Vec3::new(1.0F, 0.0F, 0.0F)
  } else {
    @core.Vec3::new(0.0F, 1.0F, 0.0F)
  }
  let t1 = normalized_or_default(n.cross(tangent))
  let t2 = normalized_or_default(n.cross(t1))
  (t1, t2)
}

///|
fn clamp_real(x : @core.Real, lo : @core.Real, hi : @core.Real) -> @core.Real {
  if x < lo {
    lo
  } else if x > hi {
    hi
  } else {
    x
  }
}

///|
fn shortest_angle_diff(curr : @core.Real, target : @core.Real) -> @core.Real {
  let mut diff = target - curr
  let pi = @core.pi()
  let two_pi = @core.two_pi()
  while diff > pi {
    diff = diff - two_pi
  }
  while diff < -pi {
    diff = diff + two_pi
  }
  diff
}

///|
fn shortest_quat(q : @core.Quat) -> @core.Quat {
  let qq = q.normalize()
  if qq.real() < 0.0F {
    qq.negated()
  } else {
    qq
  }
}

///|
fn baumgarte_bias3(
  dt : @core.Real,
  positional_error : @core.Real,
  scale : @core.Real,
) -> @core.Real {
  if dt <= JOINT_SOLVER_EPSILON_3D_REAL {
    0.0F
  } else {
    positional_error * JOINT_SOLVER_BAUMGARTE_3D_REAL * scale / dt
  }
}

///|
fn solve_row_impulse3(
  rb1 : RigidBody3D,
  rb2 : RigidBody3D,
  lin_j1 : @core.Vec3,
  ang_j1 : @core.Vec3,
  lin_j2 : @core.Vec3,
  ang_j2 : @core.Vec3,
  bias : @core.Real,
  target_vel : @core.Real,
  min_impulse : @core.Real,
  max_impulse : @core.Real,
  impulse : @core.Real,
) -> @core.Real {
  let inv_m1 = inv_mass3(rb1)
  let inv_m2 = inv_mass3(rb2)
  let inv_i1 = if inv_m1 > 0.0F {
    world_inv_inertia3(rb1)
  } else {
    @core.Mat3::zero()
  }
  let inv_i2 = if inv_m2 > 0.0F {
    world_inv_inertia3(rb2)
  } else {
    @core.Mat3::zero()
  }
  let denom = inv_m1 * lin_j1.length_squared() +
    ang_j1.dot(inv_i1.mul_vec3(ang_j1)) +
    inv_m2 * lin_j2.length_squared() +
    ang_j2.dot(inv_i2.mul_vec3(ang_j2))
  if denom <= JOINT_SOLVER_EPSILON_3D_REAL {
    return impulse
  }
  let jv = lin_j1.dot(rb1.linvel()) +
    ang_j1.dot(rb1.angvel()) +
    lin_j2.dot(rb2.linvel()) +
    ang_j2.dot(rb2.angvel())
  let delta = -(jv - target_vel + bias) / denom
  let total = clamp_real(impulse + delta, min_impulse, max_impulse)
  let applied = total - impulse
  if @core.abs(applied) > JOINT_SOLVER_EPSILON_3D_REAL {
    apply_row_impulse3(rb1, rb2, lin_j1, ang_j1, lin_j2, ang_j2, applied)
  }
  total
}

///|
fn joint_pair_in_island(
  body1 : RigidBodyHandle,
  body2 : RigidBodyHandle,
  island_stamps : Array[Int]?,
  island_stamp : Int,
) -> Bool {
  if island_stamps is None {
    return true
  }
  if island_stamp <= 0 {
    return false
  }
  if island_stamps is Some(stamps) {
    let id1 = body1.id
    let id2 = body2.id
    let s1 = if id1 >= 0 && id1 < stamps.length() { stamps[id1] } else { 0 }
    let s2 = if id2 >= 0 && id2 < stamps.length() { stamps[id2] } else { 0 }
    if s1 > 0 && s2 > 0 && s1 != s2 {
      return false
    }
    return s1 == island_stamp || s2 == island_stamp
  }
  false
}

///|
fn JointSet3DReal::ensure_solver_cache(self : JointSet3DReal) -> Unit {
  while self.spherical_impulses.length() < self.spherical.length() {
    self.spherical_impulses.push(@core.Vec3::zero())
  }
  while self.prismatic_lock_impulses.length() < self.prismatic.length() {
    self.prismatic_lock_impulses.push(@core.Vec2::zero())
  }
  while self.prismatic_motor_impulses.length() < self.prismatic.length() {
    self.prismatic_motor_impulses.push(0.0F)
  }
  while self.prismatic_angular_impulses.length() < self.prismatic.length() {
    self.prismatic_angular_impulses.push(@core.Vec3::zero())
  }
  while self.spherical_motor_impulses.length() < self.spherical_motors.length() {
    self.spherical_motor_impulses.push(@core.Vec3::zero())
  }
  while self.generic_lock_impulses.length() < self.generic.length() {
    let lock_impulses : Array[@core.Real] = []
    let limit_impulses : Array[@core.Real] = []
    let motor_impulses : Array[@core.Real] = []
    for _ in 0..<6 {
      lock_impulses.push(0.0F)
      limit_impulses.push(0.0F)
      motor_impulses.push(0.0F)
    }
    self.generic_lock_impulses.push(lock_impulses)
    self.generic_limit_impulses.push(limit_impulses)
    self.generic_motor_impulses.push(motor_impulses)
    self.generic_coupled_impulses.push(0.0F)
  }
  while self.spring_impulses.length() < self.springs.length() {
    self.spring_impulses.push(0.0F)
  }
}

///|
pub fn JointSet3DReal::solve(
  self : JointSet3DReal,
  bodies : RigidBodySet3D,
  dt : @core.Real,
  iterations : Int,
) -> Unit {
  JointSet3DReal::solve_with_island_stamp(self, bodies, dt, iterations, None, 0)
}

///|
pub fn JointSet3DReal::solve_with_island_stamp(
  self : JointSet3DReal,
  bodies : RigidBodySet3D,
  dt : @core.Real,
  iterations : Int,
  island_stamps : Array[Int]?,
  island_stamp : Int,
) -> Unit {
  self.ensure_solver_cache()
  if dt <= JOINT_SOLVER_EPSILON_3D_REAL {
    return
  }
  let iters = if iterations > 0 { iterations } else { 1 }
  // Warmstart from persistent cached impulses.
  for i in 0..<self.spherical.length() {
    let j = self.spherical[i]
    if !joint_pair_in_island(j.body1, j.body2, island_stamps, island_stamp) {
      continue
    }
    if bodies.get(j.body1) is Some(b1) && bodies.get(j.body2) is Some(b2) {
      let p1 = b1.position().transform_point(j.local_anchor1)
      let p2 = b2.position().transform_point(j.local_anchor2)
      let r1 = p1.sub(b1.world_com())
      let r2 = p2.sub(b2.world_com())
      let impulse = self.spherical_impulses[i]
      for axis_i in 0..<3 {
        let axis = axis_unit(axis_i)
        let warm = axis_value_from_vec3(impulse, axis_i)
        apply_row_impulse3(
          b1,
          b2,
          axis.scale(-1.0F),
          r1.cross(axis).scale(-1.0F),
          axis,
          r2.cross(axis),
          warm,
        )
      }
    }
  }

  for i in 0..<self.prismatic.length() {
    let j = self.prismatic[i]
    if !joint_pair_in_island(j.body1, j.body2, island_stamps, island_stamp) {
      continue
    }
    if bodies.get(j.body1) is Some(b1) && bodies.get(j.body2) is Some(b2) {
      let p1 = b1.position().transform_point(j.local_anchor1)
      let p2 = b2.position().transform_point(j.local_anchor2)
      let axis = normalized_or_default(b1.rotation().rotate_vec3(j.local_axis1))
      let (t1, t2) = orthonormal_basis3(axis)
      let r1 = p1.sub(b1.world_com())
      let r2 = p2.sub(b2.world_com())
      let lock_impulses = self.prismatic_lock_impulses[i]
      let lock_axes : Array[@core.Vec3] = [t1, t2]
      for axis_i in 0..<2 {
        let n = lock_axes[axis_i]
        let warm = axis_value_from_vec2(lock_impulses, axis_i)
        apply_row_impulse3(
          b1,
          b2,
          n.scale(-1.0F),
          r1.cross(n).scale(-1.0F),
          n,
          r2.cross(n),
          warm,
        )
      }
      let ang_impulses = self.prismatic_angular_impulses[i]
      for axis_i in 0..<3 {
        let n = b1.rotation().rotate_vec3(axis_unit(axis_i))
        let warm = axis_value_from_vec3(ang_impulses, axis_i)
        apply_row_impulse3(
          b1,
          b2,
          @core.Vec3::zero(),
          n.scale(-1.0F),
          @core.Vec3::zero(),
          n,
          warm,
        )
      }
      if j.motor_enabled {
        apply_row_impulse3(
          b1,
          b2,
          axis.scale(-1.0F),
          r1.cross(axis).scale(-1.0F),
          axis,
          r2.cross(axis),
          self.prismatic_motor_impulses[i],
        )
      }
    }
  }

  for i in 0..<self.spherical_motors.length() {
    let m = self.spherical_motors[i]
    if !joint_pair_in_island(m.body1, m.body2, island_stamps, island_stamp) {
      continue
    }
    if bodies.get(m.body1) is Some(b1) && bodies.get(m.body2) is Some(b2) {
      let warm = self.spherical_motor_impulses[i]
      for axis_i in 0..<3 {
        let n = b1.rotation().rotate_vec3(axis_unit(axis_i))
        let imp = axis_value_from_vec3(warm, axis_i)
        apply_row_impulse3(
          b1,
          b2,
          @core.Vec3::zero(),
          n.scale(-1.0F),
          @core.Vec3::zero(),
          n,
          imp,
        )
      }
    }
  }

  for i in 0..<self.generic.length() {
    let gj = self.generic[i]
    if !joint_pair_in_island(gj.body1, gj.body2, island_stamps, island_stamp) {
      continue
    }
    if bodies.get(gj.body1) is Some(rb1) && bodies.get(gj.body2) is Some(rb2) {
      let frame1 = rb1.position().mul(gj.data.local_frame1)
      let frame2 = rb2.position().mul(gj.data.local_frame2)
      let rot1 = frame1.rotation
      let r1 = frame1.translation.sub(rb1.world_com())
      let r2 = frame2.translation.sub(rb2.world_com())
      let lock_impulses = self.generic_lock_impulses[i]
      let limit_impulses = self.generic_limit_impulses[i]
      let motor_impulses = self.generic_motor_impulses[i]
      for axis_i in 0..<3 {
        let n = rot1.rotate_vec3(axis_unit(axis_i))
        apply_row_impulse3(
          rb1,
          rb2,
          n.scale(-1.0F),
          r1.cross(n).scale(-1.0F),
          n,
          r2.cross(n),
          lock_impulses[axis_i],
        )
        apply_row_impulse3(
          rb1,
          rb2,
          n.scale(-1.0F),
          r1.cross(n).scale(-1.0F),
          n,
          r2.cross(n),
          limit_impulses[axis_i],
        )
        apply_row_impulse3(
          rb1,
          rb2,
          n.scale(-1.0F),
          r1.cross(n).scale(-1.0F),
          n,
          r2.cross(n),
          motor_impulses[axis_i],
        )
      }
      for axis_i in 0..<3 {
        let n = rot1.rotate_vec3(axis_unit(axis_i))
        let idx = 3 + axis_i
        apply_row_impulse3(
          rb1,
          rb2,
          @core.Vec3::zero(),
          n.scale(-1.0F),
          @core.Vec3::zero(),
          n,
          lock_impulses[idx],
        )
        apply_row_impulse3(
          rb1,
          rb2,
          @core.Vec3::zero(),
          n.scale(-1.0F),
          @core.Vec3::zero(),
          n,
          limit_impulses[idx],
        )
        apply_row_impulse3(
          rb1,
          rb2,
          @core.Vec3::zero(),
          n.scale(-1.0F),
          @core.Vec3::zero(),
          n,
          motor_impulses[idx],
        )
      }
      if gj.data.coupled_axes.contains(JointAxesMask3DReal::lin_y()) &&
        gj.data.coupled_axes.contains(JointAxesMask3DReal::lin_z()) {
        let ay = rot1.rotate_vec3(@core.Vec3::new(0.0F, 1.0F, 0.0F))
        let az = rot1.rotate_vec3(@core.Vec3::new(0.0F, 0.0F, 1.0F))
        let n = ay.sub(az)
        apply_row_impulse3(
          rb1,
          rb2,
          n.scale(-1.0F),
          r1.cross(n).scale(-1.0F),
          n,
          r2.cross(n),
          self.generic_coupled_impulses[i],
        )
      }
    }
  }

  for i in 0..<self.springs.length() {
    let s = self.springs[i]
    if !joint_pair_in_island(s.body1, s.body2, island_stamps, island_stamp) {
      continue
    }
    if bodies.get(s.body1) is Some(b1) && bodies.get(s.body2) is Some(b2) {
      let p1 = b1.position().transform_point(s.local_anchor1)
      let p2 = b2.position().transform_point(s.local_anchor2)
      let delta = p2.sub(p1)
      if delta.length_squared() <= JOINT_SOLVER_EPSILON_3D_REAL {
        continue
      }
      let dir = normalized_or_default(delta)
      let r1 = p1.sub(b1.world_com())
      let r2 = p2.sub(b2.world_com())
      apply_row_impulse3(
        b1,
        b2,
        dir.scale(-1.0F),
        r1.cross(dir).scale(-1.0F),
        dir,
        r2.cross(dir),
        self.spring_impulses[i],
      )
    }
  }

  for _ in 0..<iters {
    for i in 0..<self.spherical.length() {
      let j = self.spherical[i]
      if !joint_pair_in_island(j.body1, j.body2, island_stamps, island_stamp) {
        continue
      }
      if bodies.get(j.body1) is Some(b1) && bodies.get(j.body2) is Some(b2) {
        let p1 = b1.position().transform_point(j.local_anchor1)
        let p2 = b2.position().transform_point(j.local_anchor2)
        let error = p2.sub(p1)
        let r1 = p1.sub(b1.world_com())
        let r2 = p2.sub(b2.world_com())
        let mut impulse = self.spherical_impulses[i]
        for axis_i in 0..<3 {
          let axis = axis_unit(axis_i)
          let bias = baumgarte_bias3(
            dt,
            axis_value_from_vec3(error, axis_i),
            1.0F,
          )
          let curr = axis_value_from_vec3(impulse, axis_i)
          let next = solve_row_impulse3(
            b1,
            b2,
            axis.scale(-1.0F),
            r1.cross(axis).scale(-1.0F),
            axis,
            r2.cross(axis),
            bias,
            0.0F,
            -JOINT_SOLVER_MAX_IMPULSE_3D_REAL,
            JOINT_SOLVER_MAX_IMPULSE_3D_REAL,
            curr,
          )
          impulse = set_axis_value_vec3(impulse, axis_i, next)
        }
        self.spherical_impulses[i] = impulse
      }
    }

    for i in 0..<self.prismatic.length() {
      let j = self.prismatic[i]
      if !joint_pair_in_island(j.body1, j.body2, island_stamps, island_stamp) {
        continue
      }
      if bodies.get(j.body1) is Some(b1) && bodies.get(j.body2) is Some(b2) {
        let p1 = b1.position().transform_point(j.local_anchor1)
        let p2 = b2.position().transform_point(j.local_anchor2)
        let axis = normalized_or_default(
          b1.rotation().rotate_vec3(j.local_axis1),
        )
        let diff = p2.sub(p1)
        let (t1, t2) = orthonormal_basis3(axis)
        let lock_axes : Array[@core.Vec3] = [t1, t2]
        let r1 = p1.sub(b1.world_com())
        let r2 = p2.sub(b2.world_com())
        let mut lock_impulses = self.prismatic_lock_impulses[i]
        for axis_i in 0..<2 {
          let n = lock_axes[axis_i]
          let bias = baumgarte_bias3(dt, diff.dot(n), 1.0F)
          let curr = axis_value_from_vec2(lock_impulses, axis_i)
          let next = solve_row_impulse3(
            b1,
            b2,
            n.scale(-1.0F),
            r1.cross(n).scale(-1.0F),
            n,
            r2.cross(n),
            bias,
            0.0F,
            -JOINT_SOLVER_MAX_IMPULSE_3D_REAL,
            JOINT_SOLVER_MAX_IMPULSE_3D_REAL,
            curr,
          )
          lock_impulses = set_axis_value_vec2(lock_impulses, axis_i, next)
        }
        self.prismatic_lock_impulses[i] = lock_impulses

        let rel_rot = shortest_quat(b1.rotation().inverse().mul(b2.rotation()))
        let ang_error_local = rel_rot.to_scaled_axis()
        let mut angular_impulses = self.prismatic_angular_impulses[i]
        for axis_i in 0..<3 {
          let n = b1.rotation().rotate_vec3(axis_unit(axis_i))
          let bias = baumgarte_bias3(
            dt,
            axis_value_from_vec3(ang_error_local, axis_i),
            1.0F,
          )
          let curr = axis_value_from_vec3(angular_impulses, axis_i)
          let next = solve_row_impulse3(
            b1,
            b2,
            @core.Vec3::zero(),
            n.scale(-1.0F),
            @core.Vec3::zero(),
            n,
            bias,
            0.0F,
            -JOINT_SOLVER_MAX_IMPULSE_3D_REAL,
            JOINT_SOLVER_MAX_IMPULSE_3D_REAL,
            curr,
          )
          angular_impulses = set_axis_value_vec3(angular_impulses, axis_i, next)
        }
        self.prismatic_angular_impulses[i] = angular_impulses

        if j.motor_enabled {
          let err = diff.dot(axis) - j.motor_target
          let stiffness_scale = clamp_real(
            j.motor_stiffness * 0.01F,
            0.0F,
            1.0F,
          )
          let damping_scale = clamp_real(j.motor_damping * 0.01F, 0.0F, 1.0F)
          let bias = baumgarte_bias3(dt, err, stiffness_scale)
          let target_vel = -damping_scale * err
          self.prismatic_motor_impulses[i] = solve_row_impulse3(
            b1,
            b2,
            axis.scale(-1.0F),
            r1.cross(axis).scale(-1.0F),
            axis,
            r2.cross(axis),
            bias,
            target_vel,
            -JOINT_SOLVER_MAX_IMPULSE_3D_REAL,
            JOINT_SOLVER_MAX_IMPULSE_3D_REAL,
            self.prismatic_motor_impulses[i],
          )
        } else {
          self.prismatic_motor_impulses[i] = 0.0F
        }
      }
    }

    for i in 0..<self.spherical_motors.length() {
      let m = self.spherical_motors[i]
      if !joint_pair_in_island(m.body1, m.body2, island_stamps, island_stamp) {
        continue
      }
      if bodies.get(m.body1) is Some(b1) && bodies.get(m.body2) is Some(b2) {
        let q1 = b1.rotation()
        let q2 = b2.rotation()
        let curr_rel = q1.inverse().mul(q2)
        let err_local = shortest_quat(
          m.target_rel_rotation.mul(curr_rel.inverse()),
        ).to_scaled_axis()
        let stiffness_scale = clamp_real(m.stiffness * 0.01F, 0.0F, 1.0F)
        let damping_scale = clamp_real(m.damping * 0.01F, 0.0F, 1.0F)
        let mut impulses = self.spherical_motor_impulses[i]
        for axis_i in 0..<3 {
          let n = q1.rotate_vec3(axis_unit(axis_i))
          let err = axis_value_from_vec3(err_local, axis_i)
          let bias = baumgarte_bias3(
            dt,
            err,
            clamp_real(stiffness_scale + damping_scale, 0.0F, 1.0F),
          )
          let curr = axis_value_from_vec3(impulses, axis_i)
          let next = solve_row_impulse3(
            b1,
            b2,
            @core.Vec3::zero(),
            n.scale(-1.0F),
            @core.Vec3::zero(),
            n,
            bias,
            0.0F,
            -JOINT_SOLVER_MAX_IMPULSE_3D_REAL,
            JOINT_SOLVER_MAX_IMPULSE_3D_REAL,
            curr,
          )
          impulses = set_axis_value_vec3(impulses, axis_i, next)
        }
        self.spherical_motor_impulses[i] = impulses
      }
    }

    for i in 0..<self.generic.length() {
      let gj = self.generic[i]
      if !joint_pair_in_island(gj.body1, gj.body2, island_stamps, island_stamp) {
        continue
      }
      if bodies.get(gj.body1) is Some(rb1) && bodies.get(gj.body2) is Some(rb2) {
        let frame1 = rb1.position().mul(gj.data.local_frame1)
        let frame2 = rb2.position().mul(gj.data.local_frame2)
        let rot1 = frame1.rotation
        let rot1_inv = rot1.inverse()
        let diff_world = frame2.translation.sub(frame1.translation)
        let diff_local = rot1_inv.rotate_vec3(diff_world)
        let r1 = frame1.translation.sub(rb1.world_com())
        let r2 = frame2.translation.sub(rb2.world_com())
        let lock_impulses = self.generic_lock_impulses[i]
        let limit_impulses = self.generic_limit_impulses[i]
        let motor_impulses = self.generic_motor_impulses[i]
        let coupled = gj.data.coupled_axes
        if coupled.contains(JointAxesMask3DReal::lin_y()) &&
          coupled.contains(JointAxesMask3DReal::lin_z()) {
          let err = diff_local.y - diff_local.z
          let ay = rot1.rotate_vec3(@core.Vec3::new(0.0F, 1.0F, 0.0F))
          let az = rot1.rotate_vec3(@core.Vec3::new(0.0F, 0.0F, 1.0F))
          let n = ay.sub(az)
          let bias = baumgarte_bias3(dt, err, 1.0F)
          self.generic_coupled_impulses[i] = solve_row_impulse3(
            rb1,
            rb2,
            n.scale(-1.0F),
            r1.cross(n).scale(-1.0F),
            n,
            r2.cross(n),
            bias,
            0.0F,
            -JOINT_SOLVER_MAX_IMPULSE_3D_REAL,
            JOINT_SOLVER_MAX_IMPULSE_3D_REAL,
            self.generic_coupled_impulses[i],
          )
        } else {
          self.generic_coupled_impulses[i] = 0.0F
        }

        for axis_i in 0..<3 {
          let state = gj.data.axis_states[axis_i]
          let comp = axis_value_from_vec3(diff_local, axis_i)
          let axis_w = rot1.rotate_vec3(axis_unit(axis_i))
          match state {
            AxisState3DReal::Locked =>
              lock_impulses[axis_i] = solve_row_impulse3(
                rb1,
                rb2,
                axis_w.scale(-1.0F),
                r1.cross(axis_w).scale(-1.0F),
                axis_w,
                r2.cross(axis_w),
                baumgarte_bias3(dt, comp, 1.0F),
                0.0F,
                -JOINT_SOLVER_MAX_IMPULSE_3D_REAL,
                JOINT_SOLVER_MAX_IMPULSE_3D_REAL,
                lock_impulses[axis_i],
              )
            AxisState3DReal::Limited(min, max) =>
              if comp < min - 1.0e-6F {
                limit_impulses[axis_i] = solve_row_impulse3(
                  rb1,
                  rb2,
                  axis_w.scale(-1.0F),
                  r1.cross(axis_w).scale(-1.0F),
                  axis_w,
                  r2.cross(axis_w),
                  baumgarte_bias3(dt, comp - min, 1.0F),
                  0.0F,
                  0.0F,
                  JOINT_SOLVER_MAX_IMPULSE_3D_REAL,
                  limit_impulses[axis_i],
                )
              } else if comp > max + 1.0e-6F {
                limit_impulses[axis_i] = solve_row_impulse3(
                  rb1,
                  rb2,
                  axis_w.scale(-1.0F),
                  r1.cross(axis_w).scale(-1.0F),
                  axis_w,
                  r2.cross(axis_w),
                  baumgarte_bias3(dt, comp - max, 1.0F),
                  0.0F,
                  -JOINT_SOLVER_MAX_IMPULSE_3D_REAL,
                  0.0F,
                  limit_impulses[axis_i],
                )
              } else {
                limit_impulses[axis_i] = 0.0F
              }
            AxisState3DReal::Free => lock_impulses[axis_i] = 0.0F
          }

          match gj.data.motors[axis_i] {
            MotorState3DReal::Disabled => motor_impulses[axis_i] = 0.0F
            MotorState3DReal::Position(target, stiffness, damping, max_force) => {
              let err = comp - target
              let stiffness_scale = clamp_real(stiffness * 0.01F, 0.0F, 1.0F)
              let damping_scale = clamp_real(damping * 0.01F, 0.0F, 1.0F)
              let max_impulse = if max_force > 0.0F {
                max_force * dt
              } else {
                JOINT_SOLVER_MAX_IMPULSE_3D_REAL
              }
              motor_impulses[axis_i] = solve_row_impulse3(
                rb1,
                rb2,
                axis_w.scale(-1.0F),
                r1.cross(axis_w).scale(-1.0F),
                axis_w,
                r2.cross(axis_w),
                baumgarte_bias3(
                  dt,
                  err,
                  clamp_real(stiffness_scale + damping_scale, 0.0F, 1.0F),
                ),
                0.0F,
                -max_impulse,
                max_impulse,
                motor_impulses[axis_i],
              )
            }
            MotorState3DReal::Velocity(target_vel, damping, max_force) => {
              let max_impulse = if max_force > 0.0F {
                max_force * dt
              } else {
                JOINT_SOLVER_MAX_IMPULSE_3D_REAL
              }
              motor_impulses[axis_i] = solve_row_impulse3(
                rb1,
                rb2,
                axis_w.scale(-1.0F),
                r1.cross(axis_w).scale(-1.0F),
                axis_w,
                r2.cross(axis_w),
                0.0F,
                target_vel * clamp_real(1.0F - damping * 0.01F, 0.0F, 1.0F),
                -max_impulse,
                max_impulse,
                motor_impulses[axis_i],
              )
            }
          }
        }

        let ang_local = shortest_quat(
          frame1.rotation.inverse().mul(frame2.rotation),
        ).to_scaled_axis()
        for axis_i in 0..<3 {
          let state = gj.data.axis_states[3 + axis_i]
          let idx = 3 + axis_i
          let comp = axis_value_from_vec3(ang_local, axis_i)
          let axis_w = rot1.rotate_vec3(axis_unit(axis_i))
          match state {
            AxisState3DReal::Locked =>
              lock_impulses[idx] = solve_row_impulse3(
                rb1,
                rb2,
                @core.Vec3::zero(),
                axis_w.scale(-1.0F),
                @core.Vec3::zero(),
                axis_w,
                baumgarte_bias3(dt, comp, 1.0F),
                0.0F,
                -JOINT_SOLVER_MAX_IMPULSE_3D_REAL,
                JOINT_SOLVER_MAX_IMPULSE_3D_REAL,
                lock_impulses[idx],
              )
            AxisState3DReal::Limited(min, max) =>
              if comp < min - 1.0e-6F {
                limit_impulses[idx] = solve_row_impulse3(
                  rb1,
                  rb2,
                  @core.Vec3::zero(),
                  axis_w.scale(-1.0F),
                  @core.Vec3::zero(),
                  axis_w,
                  baumgarte_bias3(dt, comp - min, 1.0F),
                  0.0F,
                  0.0F,
                  JOINT_SOLVER_MAX_IMPULSE_3D_REAL,
                  limit_impulses[idx],
                )
              } else if comp > max + 1.0e-6F {
                limit_impulses[idx] = solve_row_impulse3(
                  rb1,
                  rb2,
                  @core.Vec3::zero(),
                  axis_w.scale(-1.0F),
                  @core.Vec3::zero(),
                  axis_w,
                  baumgarte_bias3(dt, comp - max, 1.0F),
                  0.0F,
                  -JOINT_SOLVER_MAX_IMPULSE_3D_REAL,
                  0.0F,
                  limit_impulses[idx],
                )
              } else {
                limit_impulses[idx] = 0.0F
              }
            AxisState3DReal::Free => lock_impulses[idx] = 0.0F
          }
          match gj.data.motors[idx] {
            MotorState3DReal::Disabled => motor_impulses[idx] = 0.0F
            MotorState3DReal::Position(target, stiffness, damping, max_force) => {
              let diff = shortest_angle_diff(comp, target)
              let stiffness_scale = clamp_real(stiffness * 0.01F, 0.0F, 1.0F)
              let damping_scale = clamp_real(damping * 0.01F, 0.0F, 1.0F)
              let max_impulse = if max_force > 0.0F {
                max_force * dt
              } else {
                JOINT_SOLVER_MAX_IMPULSE_3D_REAL
              }
              motor_impulses[idx] = solve_row_impulse3(
                rb1,
                rb2,
                @core.Vec3::zero(),
                axis_w.scale(-1.0F),
                @core.Vec3::zero(),
                axis_w,
                baumgarte_bias3(
                  dt,
                  diff,
                  clamp_real(stiffness_scale + damping_scale, 0.0F, 1.0F),
                ),
                0.0F,
                -max_impulse,
                max_impulse,
                motor_impulses[idx],
              )
            }
            MotorState3DReal::Velocity(target_vel, damping, max_force) => {
              let max_impulse = if max_force > 0.0F {
                max_force * dt
              } else {
                JOINT_SOLVER_MAX_IMPULSE_3D_REAL
              }
              motor_impulses[idx] = solve_row_impulse3(
                rb1,
                rb2,
                @core.Vec3::zero(),
                axis_w.scale(-1.0F),
                @core.Vec3::zero(),
                axis_w,
                0.0F,
                target_vel * clamp_real(1.0F - damping * 0.01F, 0.0F, 1.0F),
                -max_impulse,
                max_impulse,
                motor_impulses[idx],
              )
            }
          }
        }
        self.generic_lock_impulses[i] = lock_impulses
        self.generic_limit_impulses[i] = limit_impulses
        self.generic_motor_impulses[i] = motor_impulses
      }
    }

    for i in 0..<self.springs.length() {
      let s = self.springs[i]
      if !joint_pair_in_island(s.body1, s.body2, island_stamps, island_stamp) {
        continue
      }
      if bodies.get(s.body1) is Some(b1) && bodies.get(s.body2) is Some(b2) {
        let p1 = b1.position().transform_point(s.local_anchor1)
        let p2 = b2.position().transform_point(s.local_anchor2)
        let delta = p2.sub(p1)
        let dist2 = delta.length_squared()
        if dist2 <= JOINT_SOLVER_EPSILON_3D_REAL {
          continue
        }
        let dist = Float::sqrt(dist2)
        let err = dist - s.rest_length
        let dir = delta.scale(1.0F / dist)
        let r1 = p1.sub(b1.world_com())
        let r2 = p2.sub(b2.world_com())
        let stiffness_scale = clamp_real(s.stiffness * 0.01F, 0.0F, 1.0F)
        let damping_scale = clamp_real(s.damping * 0.01F, 0.0F, 1.0F)
        self.spring_impulses[i] = solve_row_impulse3(
          b1,
          b2,
          dir.scale(-1.0F),
          r1.cross(dir).scale(-1.0F),
          dir,
          r2.cross(dir),
          baumgarte_bias3(
            dt,
            err,
            clamp_real(stiffness_scale + damping_scale, 0.0F, 1.0F),
          ),
          0.0F,
          -JOINT_SOLVER_MAX_IMPULSE_3D_REAL,
          JOINT_SOLVER_MAX_IMPULSE_3D_REAL,
          self.spring_impulses[i],
        )
      }
    }
  }
}
