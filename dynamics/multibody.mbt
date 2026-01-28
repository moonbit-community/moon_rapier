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
const SPATIAL_DIM : Int = 3

///|
struct RevoluteJointBuilder {
  mut local_anchor1 : @core.Vec2
  mut local_anchor2 : @core.Vec2
  mut softness : SpringCoefficients
  mut motor : JointMotor
  mut motor_enabled : Bool
  mut limits : JointLimits
  mut limits_enabled : Bool
}

///|
pub fn RevoluteJointBuilder::new() -> RevoluteJointBuilder {
  {
    local_anchor1: @core.Vec2::zero(),
    local_anchor2: @core.Vec2::zero(),
    softness: SpringCoefficients::joint_defaults(),
    motor: JointMotor::default(),
    motor_enabled: false,
    limits: JointLimits::default(),
    limits_enabled: false,
  }
}

///|
pub fn RevoluteJointBuilder::local_anchor1(
  self : RevoluteJointBuilder,
  anchor : @core.Vec2,
) -> RevoluteJointBuilder {
  self.local_anchor1 = anchor
  self
}

///|
pub fn RevoluteJointBuilder::local_anchor2(
  self : RevoluteJointBuilder,
  anchor : @core.Vec2,
) -> RevoluteJointBuilder {
  self.local_anchor2 = anchor
  self
}

///|
pub fn RevoluteJointBuilder::softness(
  self : RevoluteJointBuilder,
  softness : SpringCoefficients,
) -> RevoluteJointBuilder {
  self.softness = softness
  self
}

///|
pub fn RevoluteJointBuilder::motor_model(
  self : RevoluteJointBuilder,
  model : MotorModel,
) -> RevoluteJointBuilder {
  self.motor = self.motor.set_model(model)
  self
}

///|
pub fn RevoluteJointBuilder::motor_velocity(
  self : RevoluteJointBuilder,
  target_vel : @core.Real,
  factor : @core.Real,
) -> RevoluteJointBuilder {
  self.motor_enabled = true
  self.motor = self.motor
    .set_target_vel(target_vel)
    .set_stiffness(0.0F)
    .set_damping(factor)
  self
}

///|
pub fn RevoluteJointBuilder::motor_position(
  self : RevoluteJointBuilder,
  target_pos : @core.Real,
  stiffness : @core.Real,
  damping : @core.Real,
) -> RevoluteJointBuilder {
  self.motor_enabled = true
  self.motor = self.motor
    .set_target_pos(target_pos)
    .set_target_vel(0.0F)
    .set_stiffness(stiffness)
    .set_damping(damping)
  self
}

///|
pub fn RevoluteJointBuilder::motor_max_force(
  self : RevoluteJointBuilder,
  max_force : @core.Real,
) -> RevoluteJointBuilder {
  self.motor = self.motor.set_max_force(max_force)
  self
}

///|
pub fn RevoluteJointBuilder::limits(
  self : RevoluteJointBuilder,
  min : @core.Real,
  max : @core.Real,
) -> RevoluteJointBuilder {
  self.limits_enabled = true
  self.limits = JointLimits::new(min, max)
  self
}

///|
pub fn RevoluteJointBuilder::build(
  self : RevoluteJointBuilder,
) -> RevoluteJoint {
  {
    local_anchor1: self.local_anchor1,
    local_anchor2: self.local_anchor2,
    softness: self.softness,
    motor: self.motor,
    motor_enabled: self.motor_enabled,
    limits: self.limits,
    limits_enabled: self.limits_enabled,
    solver_lin_impulses: @core.Vec2::zero(),
  }
}

///|
struct RevoluteJoint {
  local_anchor1 : @core.Vec2
  local_anchor2 : @core.Vec2
  softness : SpringCoefficients
  mut motor : JointMotor
  motor_enabled : Bool
  mut limits : JointLimits
  limits_enabled : Bool
  // Solver warmstart cache for the (locked) linear DOFs of this revolute joint.
  // The motor/limit impulses are stored inside `motor` and `limits`.
  mut solver_lin_impulses : @core.Vec2
}

///|
pub fn RevoluteJoint::local_anchor1(self : RevoluteJoint) -> @core.Vec2 {
  self.local_anchor1
}

///|
pub fn RevoluteJoint::local_anchor2(self : RevoluteJoint) -> @core.Vec2 {
  self.local_anchor2
}

///|
pub fn RevoluteJoint::softness(self : RevoluteJoint) -> SpringCoefficients {
  self.softness
}

///|
pub fn RevoluteJoint::motor(self : RevoluteJoint) -> JointMotor? {
  if self.motor_enabled {
    Some(self.motor)
  } else {
    None
  }
}

///|
pub fn RevoluteJoint::limits(self : RevoluteJoint) -> JointLimits? {
  if self.limits_enabled {
    Some(self.limits)
  } else {
    None
  }
}

///|
pub fn RevoluteJoint::angle(
  self : RevoluteJoint,
  rot1 : @core.Rot2,
  rot2 : @core.Rot2,
) -> @core.Real {
  self.local_anchor1 |> ignore
  self.local_anchor2 |> ignore
  let delta = rot2.angle() - rot1.angle()
  let pi_value = @core.pi()
  let two_pi = @core.two_pi()
  let eps = 1.0e-4F
  if delta > pi_value {
    if delta - pi_value < eps {
      pi_value
    } else {
      delta - two_pi
    }
  } else if delta < -pi_value {
    if -pi_value - delta < eps {
      -pi_value
    } else {
      delta + two_pi
    }
  } else {
    delta
  }
}

///|
fn revolute_body_to_parent(
  joint : RevoluteJoint,
  angle : @core.Real,
) -> @core.Isometry2 {
  let t1 = @core.Isometry2::from_translation(joint.local_anchor1)
  let rot = @core.Isometry2::new(
    @core.Vec2::zero(),
    @core.Rot2::from_angle(angle),
  )
  let neg_anchor2 = @core.Vec2::new(
    -joint.local_anchor2.x,
    -joint.local_anchor2.y,
  )
  let t2 = @core.Isometry2::from_translation(neg_anchor2)
  t1.mul(rot).mul(t2)
}

///|
pub struct LinkId {
  value : Int
}

///|
fn LinkId::new(value : Int) -> LinkId {
  LinkId::{ value, }
}

///|
struct MultibodyLink {
  mut internal_id : Int
  mut parent_internal_id : Int
  mut assembly_id : Int
  rigid_body : RigidBodyHandle
  mut joint : RevoluteJoint
  mut joint_angle : @core.Real
  mut local_to_parent : @core.Isometry2
  mut local_to_world : @core.Isometry2
}

///|
pub fn MultibodyLink::local_to_world(self : MultibodyLink) -> @core.Isometry2 {
  self.local_to_world
}

///|
struct Multibody {
  links : Array[MultibodyLink]
  mut ndofs : Int
  mut root_is_dynamic : Bool
  mut root_translation : @core.Vec2
  mut root_angle : @core.Real
}

///|
fn Multibody::new_with_root(handle : RigidBodyHandle) -> Multibody {
  let joint = RevoluteJointBuilder::new().build()
  let link = MultibodyLink::{
    internal_id: 0,
    parent_internal_id: 0,
    assembly_id: 0,
    rigid_body: handle,
    joint,
    joint_angle: 0.0F,
    local_to_parent: @core.Isometry2::identity(),
    local_to_world: @core.Isometry2::identity(),
  }
  let links : Array[MultibodyLink] = [link]
  {
    links,
    ndofs: SPATIAL_DIM,
    root_is_dynamic: true,
    root_translation: @core.Vec2::zero(),
    root_angle: 0.0F,
  }
}

///|
fn Multibody::root_pose(
  self : Multibody,
  displacements : Array[@core.Real]?,
) -> @core.Isometry2 {
  let mut tx = self.root_translation.x
  let mut ty = self.root_translation.y
  let mut angle = self.root_angle
  if self.root_is_dynamic && displacements is Some(disp) {
    if disp.length() > 0 {
      tx = tx + disp[0]
    }
    if disp.length() > 1 {
      ty = ty + disp[1]
    }
    if disp.length() > 2 {
      angle = angle + disp[2]
    }
  }
  @core.Isometry2::new(@core.Vec2::new(tx, ty), @core.Rot2::from_angle(angle))
}

///|
fn Multibody::update_root_from_body(self : Multibody, body : RigidBody) -> Unit {
  let position = body.position()
  self.root_translation = position.translation
  self.root_angle = position.rotation.angle()
}

///|
fn Multibody::rebuild_indices(self : Multibody, root_dofs : Int) -> Unit {
  let mut assembly = root_dofs
  for i in 0..<self.links.length() {
    let link = self.links[i]
    link.internal_id = i
    if i == 0 {
      link.assembly_id = 0
    } else {
      link.assembly_id = assembly
      assembly = assembly + 1
    }
    self.links[i] = link
  }
}

///|
fn Multibody::recompute_ndofs(self : Multibody) -> Unit {
  let count = self.links.length()
  if count <= 0 {
    self.ndofs = 0
  } else {
    self.root_is_dynamic = true
    self.ndofs = SPATIAL_DIM + (count - 1)
    self.rebuild_indices(SPATIAL_DIM)
  }
}

///|
fn Multibody::update_ndofs_with_root(
  self : Multibody,
  bodies : RigidBodySet,
) -> Unit {
  if self.links.length() <= 0 {
    self.ndofs = 0
    return
  }
  let root = self.links[0]
  let is_dynamic = if bodies.get(root.rigid_body) is Some(body) {
    !(body.body_type is RigidBodyType::Fixed)
  } else {
    true
  }
  let root_dofs = if is_dynamic { SPATIAL_DIM } else { 0 }
  self.root_is_dynamic = is_dynamic
  self.ndofs = root_dofs + (self.links.length() - 1)
  self.rebuild_indices(root_dofs)
}

///|
pub fn Multibody::forward_kinematics(
  self : Multibody,
  bodies : RigidBodySet,
  update_positions : Bool,
) -> Unit {
  self.update_ndofs_with_root(bodies)
  if self.links.length() <= 0 {
    update_positions |> ignore
    return
  }
  if update_positions && bodies.get(self.links[0].rigid_body) is Some(body) {
    self.update_root_from_body(body)
  }
  let root_pose = self.root_pose(None)
  let root_link = self.links[0]
  root_link.local_to_parent = root_pose
  root_link.local_to_world = root_link.local_to_parent
  self.links[0] = root_link
  for i in 1..<self.links.length() {
    let link = self.links[i]
    let parent = self.links[link.parent_internal_id]
    let local_to_parent = revolute_body_to_parent(link.joint, link.joint_angle)
    link.local_to_parent = local_to_parent
    link.local_to_world = parent.local_to_world.mul(link.local_to_parent)
    self.links[i] = link
  }
}

///|
pub fn Multibody::update_rigid_bodies(
  self : Multibody,
  bodies : RigidBodySet,
) -> Unit {
  for i in 0..<self.links.length() {
    let link = self.links[i]
    let handle = link.rigid_body
    if bodies.get_mut_internal_with_modification_tracking(handle) is Some(body) {
      // Keep the rigid-body poses in sync with the multibody kinematics.
      body.set_position(link.local_to_world, false) |> ignore
    }
  }
}

///|
pub fn Multibody::ndofs(self : Multibody) -> Int {
  self.ndofs
}

///|
pub fn Multibody::link(self : Multibody, id : LinkId) -> MultibodyLink? {
  let idx = id.value
  if idx < 0 || idx >= self.links.length() {
    return None
  }
  let link = self.links[idx]
  if link.internal_id != idx {
    return None
  }
  Some(link)
}

///|
fn Multibody::branch_ids(self : Multibody, id : LinkId) -> Array[Int] {
  let branch : Array[Int] = []
  let mut current = id.value
  if current < 0 || current >= self.links.length() {
    return branch
  }
  while true {
    branch.push(current)
    if current == 0 {
      break
    }
    current = self.links[current].parent_internal_id
  }
  let reversed : Array[Int] = []
  let len = branch.length()
  for i in 0..<len {
    reversed.push(branch[len - 1 - i])
  }
  reversed
}

///|
fn Multibody::compute_link_pose(
  self : Multibody,
  id : LinkId,
  displacements : Array[@core.Real]?,
  out_jacobian : Jacobian?,
) -> @core.Isometry2 {
  let branch = self.branch_ids(id)
  if branch.length() == 0 {
    return @core.Isometry2::identity()
  }
  let root_pose = self.root_pose(displacements)
  let mut current_pose = root_pose
  let joint_positions : Array[@core.Vec2] = []
  let joint_cols : Array[Int] = []
  for i in 1..<branch.length() {
    let link_id = branch[i]
    let link = self.links[link_id]
    let joint_pos = current_pose.transform_point(link.joint.local_anchor1)
    joint_positions.push(joint_pos)
    joint_cols.push(link.assembly_id)
    let mut angle = link.joint_angle
    if displacements is Some(disp) && link.assembly_id < disp.length() {
      angle = angle + disp[link.assembly_id]
    }
    let local_to_parent = revolute_body_to_parent(link.joint, angle)
    current_pose = current_pose.mul(local_to_parent)
  }
  if out_jacobian is Some(jacobian) {
    jacobian.ensure_cols(self.ndofs)
    let link_pos = current_pose.translation
    let root_pos = root_pose.translation
    if self.root_is_dynamic && self.ndofs >= SPATIAL_DIM {
      jacobian.set_value(0, 0, 1.0F)
      jacobian.set_value(1, 0, 0.0F)
      jacobian.set_value(2, 0, 0.0F)
      jacobian.set_value(0, 1, 0.0F)
      jacobian.set_value(1, 1, 1.0F)
      jacobian.set_value(2, 1, 0.0F)
      let dx = link_pos.x - root_pos.x
      let dy = link_pos.y - root_pos.y
      jacobian.set_value(0, 2, -dy)
      jacobian.set_value(1, 2, dx)
      jacobian.set_value(2, 2, 1.0F)
    }
    for i in 0..<joint_positions.length() {
      let col = joint_cols[i]
      if col >= 0 && col < self.ndofs {
        let joint_pos = joint_positions[i]
        let dx = link_pos.x - joint_pos.x
        let dy = link_pos.y - joint_pos.y
        jacobian.set_value(0, col, -dy)
        jacobian.set_value(1, col, dx)
        jacobian.set_value(2, col, 1.0F)
      }
    }
  }
  current_pose
}

///|
pub fn Multibody::body_jacobian(self : Multibody, id : LinkId) -> Jacobian {
  let jacobian = Jacobian::zeros(self.ndofs)
  self.compute_link_pose(id, None, Some(jacobian)) |> ignore
  jacobian
}

///|
pub fn Multibody::forward_kinematics_single_link(
  self : Multibody,
  bodies : RigidBodySet,
  id : LinkId,
  displacements : Array[@core.Real]?,
  jacobian : Jacobian?,
) -> @core.Isometry2 {
  self.update_ndofs_with_root(bodies)
  self.compute_link_pose(id, displacements, jacobian)
}

///|
pub fn Multibody::apply_displacements(
  self : Multibody,
  displacements : Array[@core.Real],
) -> Unit {
  if self.root_is_dynamic {
    if displacements.length() > 0 {
      let tx = self.root_translation.x + displacements[0]
      self.root_translation = @core.Vec2::new(tx, self.root_translation.y)
    }
    if displacements.length() > 1 {
      let ty = self.root_translation.y + displacements[1]
      self.root_translation = @core.Vec2::new(self.root_translation.x, ty)
    }
    if displacements.length() > 2 {
      self.root_angle = self.root_angle + displacements[2]
    }
  }
  for i in 1..<self.links.length() {
    let link = self.links[i]
    let col = link.assembly_id
    if col >= 0 && col < displacements.length() {
      link.joint_angle = link.joint_angle + displacements[col]
      self.links[i] = link
    }
  }
}

///|
pub struct InverseKinematicsOption {
  mut damping : @core.Real
  mut max_iters : Int
  mut constrained_axes : JointAxesMask
  mut epsilon_linear : @core.Real
  mut epsilon_angular : @core.Real
}

///|
pub fn InverseKinematicsOption::default() -> InverseKinematicsOption {
  {
    damping: 1.0F,
    max_iters: 10,
    constrained_axes: JointAxesMask::all(),
    epsilon_linear: 1.0e-3F,
    epsilon_angular: 1.0e-3F,
  }
}

///|
pub fn InverseKinematicsOption::set_damping(
  self : InverseKinematicsOption,
  damping : @core.Real,
) -> InverseKinematicsOption {
  self.damping = damping
  self
}

///|
pub fn InverseKinematicsOption::set_max_iters(
  self : InverseKinematicsOption,
  max_iters : Int,
) -> InverseKinematicsOption {
  self.max_iters = max_iters
  self
}

///|
pub fn InverseKinematicsOption::set_constrained_axes(
  self : InverseKinematicsOption,
  constrained_axes : JointAxesMask,
) -> InverseKinematicsOption {
  self.constrained_axes = constrained_axes
  self
}

///|
pub fn InverseKinematicsOption::set_epsilon_linear(
  self : InverseKinematicsOption,
  epsilon_linear : @core.Real,
) -> InverseKinematicsOption {
  self.epsilon_linear = epsilon_linear
  self
}

///|
pub fn InverseKinematicsOption::set_epsilon_angular(
  self : InverseKinematicsOption,
  epsilon_angular : @core.Real,
) -> InverseKinematicsOption {
  self.epsilon_angular = epsilon_angular
  self
}

///|
fn ik_wrap_angle_pi(angle : @core.Real) -> @core.Real {
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
fn ik_mat3_inverse(
  a00 : @core.Real,
  a01 : @core.Real,
  a02 : @core.Real,
  a10 : @core.Real,
  a11 : @core.Real,
  a12 : @core.Real,
  a20 : @core.Real,
  a21 : @core.Real,
  a22 : @core.Real,
) -> (
  @core.Real,
  @core.Real,
  @core.Real,
  @core.Real,
  @core.Real,
  @core.Real,
  @core.Real,
  @core.Real,
  @core.Real,
)? {
  let c00 = a11 * a22 - a12 * a21
  let c01 = -(a10 * a22 - a12 * a20)
  let c02 = a10 * a21 - a11 * a20
  let det = a00 * c00 + a01 * c01 + a02 * c02
  if @core.abs(det) <= 1.0e-5F {
    return None
  }
  let inv_det = 1.0F / det
  // Adjugate (transpose of cofactor matrix) * 1/det.
  let inv00 = c00 * inv_det
  let inv01 = -(a01 * a22 - a02 * a21) * inv_det
  let inv02 = (a01 * a12 - a02 * a11) * inv_det
  let inv10 = c01 * inv_det
  let inv11 = (a00 * a22 - a02 * a20) * inv_det
  let inv12 = -(a00 * a12 - a02 * a10) * inv_det
  let inv20 = c02 * inv_det
  let inv21 = -(a00 * a21 - a01 * a20) * inv_det
  let inv22 = (a00 * a11 - a01 * a10) * inv_det
  Some((inv00, inv01, inv02, inv10, inv11, inv12, inv20, inv21, inv22))
}

///|
fn ik_inverse_kinematics_delta_with_jacobian(
  jacobian : Jacobian,
  dx : @core.Real,
  dy : @core.Real,
  da : @core.Real,
  damping : @core.Real,
  displacements : DVector,
) -> Unit {
  let cols = jacobian.cols()
  if displacements.len() < cols {
    return
  }

  // JJ = J * J^T + I * (damping^2), with J = 3 x cols.
  let mut a00 = 0.0F
  let mut a01 = 0.0F
  let mut a02 = 0.0F
  let mut a11 = 0.0F
  let mut a12 = 0.0F
  let mut a22 = 0.0F
  for c in 0..<cols {
    let j0 = jacobian.value_at(0, c)
    let j1 = jacobian.value_at(1, c)
    let j2 = jacobian.value_at(2, c)
    a00 = a00 + j0 * j0
    a01 = a01 + j0 * j1
    a02 = a02 + j0 * j2
    a11 = a11 + j1 * j1
    a12 = a12 + j1 * j2
    a22 = a22 + j2 * j2
  }
  let d2 = damping * damping
  a00 = a00 + d2
  a11 = a11 + d2
  a22 = a22 + d2
  let a10 = a01
  let a20 = a02
  let a21 = a12
  let inv = ik_mat3_inverse(a00, a01, a02, a10, a11, a12, a20, a21, a22)
  let (inv00, inv01, inv02, inv10, inv11, inv12, inv20, inv21, inv22) = if inv
    is Some(v) {
    v
  } else {
    (1.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 1.0F)
  }

  // u = inv(JJ) * desired_movement.
  let u0 = inv00 * dx + inv01 * dy + inv02 * da
  let u1 = inv10 * dx + inv11 * dy + inv12 * da
  let u2 = inv20 * dx + inv21 * dy + inv22 * da

  // displacements += J^T * u
  for c in 0..<cols {
    let j0 = jacobian.value_at(0, c)
    let j1 = jacobian.value_at(1, c)
    let j2 = jacobian.value_at(2, c)
    let delta_q = j0 * u0 + j1 * u1 + j2 * u2
    displacements.set_value(c, displacements.value_at(c) + delta_q)
  }
}

///|
pub fn Multibody::inverse_kinematics(
  self : Multibody,
  bodies : RigidBodySet,
  link_id : LinkId,
  options : InverseKinematicsOption,
  target_pose : @core.Isometry2,
  joint_can_move : (MultibodyLink) -> Bool,
  displacements : DVector,
) -> Unit {
  // Ensure ndofs/assembly indices match the current root kind (fixed/dynamic).
  self.update_ndofs_with_root(bodies)
  if displacements.len() < self.ndofs {
    return
  }
  let branch = self.branch_ids(link_id)
  let can_move : Array[Bool] = []
  for i in 0..<branch.length() {
    let id = branch[i]
    can_move.push(joint_can_move(self.links[id]))
  }
  let jacobian = Jacobian::zeros(0)
  for _ in 0..<options.max_iters {
    let pose = self.forward_kinematics_single_link(
      bodies,
      link_id,
      Some(displacements.data),
      Some(jacobian),
    )

    // Zero-out jacobian columns for joints that aren't allowed to move.
    for i in 0..<branch.length() {
      if !can_move[i] {
        let link = self.links[branch[i]]
        // Only one DOF per revolute joint link.
        let col = link.assembly_id
        if col >= 0 && col < self.ndofs {
          jacobian.set_value(0, col, 0.0F)
          jacobian.set_value(1, col, 0.0F)
          jacobian.set_value(2, col, 0.0F)
        }
      }
    }
    let mut dx = target_pose.translation.x - pose.translation.x
    let mut dy = target_pose.translation.y - pose.translation.y
    let mut da = ik_wrap_angle_pi(
      target_pose.rotation.mul(pose.rotation.inverse()).angle(),
    )
    if !options.constrained_axes.contains(JointAxesMask::lin_x()) {
      dx = 0.0F
    }
    if !options.constrained_axes.contains(JointAxesMask::lin_y()) {
      dy = 0.0F
    }
    if !options.constrained_axes.contains(JointAxesMask::ang_z()) {
      da = 0.0F
    }
    let lin_norm = Float::sqrt(dx * dx + dy * dy)
    let ang_norm = @core.abs(da)
    if lin_norm <= options.epsilon_linear && ang_norm <= options.epsilon_angular {
      break
    }
    ik_inverse_kinematics_delta_with_jacobian(
      jacobian,
      dx,
      dy,
      da,
      options.damping,
      displacements,
    )
  }
}

///|
struct MultibodyJointHandle {
  id : Int
}

///|
fn MultibodyJointHandle::new(id : Int) -> MultibodyJointHandle {
  MultibodyJointHandle::{ id, }
}

///|
pub fn MultibodyJointHandle::from_raw_parts(
  id : Int,
  generation : Int,
) -> MultibodyJointHandle {
  generation |> ignore
  MultibodyJointHandle::new(id)
}

///|
pub fn MultibodyJointHandle::into_raw_parts(
  self : MultibodyJointHandle,
) -> (Int, Int) {
  (self.id, 0)
}

///|
pub fn MultibodyJointHandle::invalid() -> MultibodyJointHandle {
  MultibodyJointHandle::new(-1)
}

///|
pub struct MultibodyIndex {
  value : Int
}

///|
fn MultibodyIndex::new(value : Int) -> MultibodyIndex {
  { value, }
}

///|
pub fn MultibodyIndex::invalid() -> MultibodyIndex {
  MultibodyIndex::new(-1)
}

///|
pub struct MultibodyLinkId {
  multibody : MultibodyIndex
  id : Int
}

///|
/// A lightweight joint view used by the physics pipeline to build solver constraints.
///
/// This intentionally exposes only what the pipeline needs (parent/child rigid-bodies and the
/// revolute joint parameters + warmstart cache). The canonical storage remains inside the
/// multibody.
pub struct MultibodyRevoluteJointDesc {
  link : MultibodyLinkId
  parent : RigidBodyHandle
  child : RigidBodyHandle
  joint : RevoluteJoint
  solver_lin_impulses : @core.Vec2
}

///|
priv struct LinkInfo {
  mb_index : Int
  link_id : LinkId
}

///|
struct MultibodyJointSet {
  multibodies : Array[Multibody]
  body_links : Array[LinkInfo?]
  to_wake_up : Array[RigidBodyHandle]
  to_join : Array[(RigidBodyHandle, RigidBodyHandle)]
}

///|
fn multibody_root_pose(
  multibody : Multibody,
  root_id : Int,
) -> (@core.Vec2, @core.Real) {
  if root_id < 0 || root_id >= multibody.links.length() {
    return (@core.Vec2::zero(), 0.0F)
  }
  let root_link = multibody.links[root_id]
  let pose = root_link.local_to_world
  (pose.translation, pose.rotation.angle())
}

///|
fn collect_subtree_indices(multibody : Multibody, root_id : Int) -> Array[Int] {
  if root_id < 0 || root_id >= multibody.links.length() {
    return []
  }
  let count = multibody.links.length()
  let children : Array[Array[Int]] = []
  for _ in 0..<count {
    children.push([])
  }
  for i in 0..<count {
    let link = multibody.links[i]
    let parent_id = link.parent_internal_id
    if parent_id >= 0 && parent_id < count && parent_id != i {
      children[parent_id].push(i)
    }
  }
  let visited : Array[Bool] = []
  for _ in 0..<count {
    visited.push(false)
  }
  let stack : Array[Int] = [root_id]
  let result : Array[Int] = []
  while stack.pop() is Some(id) {
    if id < 0 || id >= count {
      continue
    }
    if visited[id] {
      continue
    }
    visited[id] = true
    result.push(id)
    let child_list = children[id]
    for i in 0..<child_list.length() {
      stack.push(child_list[i])
    }
  }
  result
}

///|
fn build_multibody_from_indices(
  multibody : Multibody,
  indices : Array[Int],
  root_id : Int,
) -> Multibody {
  let total = multibody.links.length()
  let mut_order : Array[Int] = [root_id]
  for i in 0..<indices.length() {
    let id = indices[i]
    if id != root_id {
      mut_order.push(id)
    }
  }
  let map : Array[Int] = []
  for _ in 0..<total {
    map.push(-1)
  }
  let new_links : Array[MultibodyLink] = []
  for i in 0..<mut_order.length() {
    let old_id = mut_order[i]
    if old_id < 0 || old_id >= total {
      continue
    }
    let link = multibody.links[old_id]
    let new_id = new_links.length()
    map[old_id] = new_id
    link.internal_id = new_id
    let old_parent = link.parent_internal_id
    if old_id == root_id ||
      old_parent == old_id ||
      old_parent < 0 ||
      old_parent >= total ||
      map[old_parent] < 0 {
      link.parent_internal_id = new_id
    } else {
      link.parent_internal_id = map[old_parent]
    }
    new_links.push(link)
  }
  let (root_translation, root_angle) = multibody_root_pose(multibody, root_id)
  let new_multibody = Multibody::{
    links: new_links,
    ndofs: 0,
    root_is_dynamic: multibody.root_is_dynamic,
    root_translation,
    root_angle,
  }
  new_multibody.recompute_ndofs()
  new_multibody
}

///|
pub fn MultibodyJointSet::new() -> MultibodyJointSet {
  { multibodies: [], body_links: [], to_wake_up: [], to_join: [] }
}

///|
pub fn MultibodyJointSet::take_wake_up(
  self : MultibodyJointSet,
) -> Array[RigidBodyHandle] {
  let result : Array[RigidBodyHandle] = []
  for i in 0..<self.to_wake_up.length() {
    result.push(self.to_wake_up[i])
  }
  self.to_wake_up.clear()
  result
}

///|
pub fn MultibodyJointSet::take_to_join(
  self : MultibodyJointSet,
) -> Array[(RigidBodyHandle, RigidBodyHandle)] {
  let result : Array[(RigidBodyHandle, RigidBodyHandle)] = []
  for i in 0..<self.to_join.length() {
    result.push(self.to_join[i])
  }
  self.to_join.clear()
  result
}

///|
pub fn MultibodyJointSet::sync_rigid_bodies_from_multibodies(
  self : MultibodyJointSet,
  bodies : RigidBodySet,
) -> Unit {
  for i in 0..<self.multibodies.length() {
    let mb = self.multibodies[i]
    mb.forward_kinematics(bodies, true)
    mb.update_rigid_bodies(bodies)
    self.multibodies[i] = mb
  }
}

///|
pub fn MultibodyJointSet::revolute_joint_descriptors(
  self : MultibodyJointSet,
) -> Array[MultibodyRevoluteJointDesc] {
  let result : Array[MultibodyRevoluteJointDesc] = []
  for mb_index in 0..<self.multibodies.length() {
    let mb = self.multibodies[mb_index]
    for link_id in 1..<mb.links.length() {
      let link = mb.links[link_id]
      let parent_id = link.parent_internal_id
      if parent_id < 0 || parent_id >= mb.links.length() || parent_id == link_id {
        continue
      }
      let parent_link = mb.links[parent_id]
      result.push(MultibodyRevoluteJointDesc::{
        link: MultibodyLinkId::{
          multibody: MultibodyIndex::new(mb_index),
          id: link_id,
        },
        parent: parent_link.rigid_body,
        child: link.rigid_body,
        joint: link.joint,
        solver_lin_impulses: link.joint.solver_lin_impulses,
      })
    }
  }
  result
}

///|
pub fn MultibodyJointSet::set_revolute_joint_solver_impulses(
  self : MultibodyJointSet,
  link : MultibodyLinkId,
  lin_impulses : @core.Vec2,
  motor_impulse : @core.Real,
  limit_impulse : @core.Real,
) -> Unit {
  let mb_index = link.multibody.value
  if mb_index < 0 || mb_index >= self.multibodies.length() {
    return
  }
  let mb = self.multibodies[mb_index]
  if link.id <= 0 || link.id >= mb.links.length() {
    return
  }
  let lnk = mb.links[link.id]
  lnk.joint.solver_lin_impulses = lin_impulses
  lnk.joint.motor = lnk.joint.motor.set_impulse(motor_impulse)
  lnk.joint.limits = lnk.joint.limits.set_impulse(limit_impulse)
  mb.links[link.id] = lnk
  self.multibodies[mb_index] = mb
}

///|
pub fn MultibodyJointSet::iter(
  self : MultibodyJointSet,
) -> Array[(MultibodyJointHandle, MultibodyLinkId, Multibody, MultibodyLink)] {
  let result : Array[
    (MultibodyJointHandle, MultibodyLinkId, Multibody, MultibodyLink),
  ] = []
  for i in 0..<self.body_links.length() {
    if self.body_links[i] is Some(info) {
      if info.link_id.value > 0 &&
        info.mb_index >= 0 &&
        info.mb_index < self.multibodies.length() {
        let multibody = self.multibodies[info.mb_index]
        if info.link_id.value >= 0 &&
          info.link_id.value < multibody.links.length() {
          let link = multibody.links[info.link_id.value]
          let link_id = MultibodyLinkId::{
            multibody: MultibodyIndex::new(info.mb_index),
            id: info.link_id.value,
          }
          result.push((MultibodyJointHandle::new(i), link_id, multibody, link))
        }
      }
    }
  }
  result
}

///|
fn MultibodyJointSet::update_body_links_for_multibody(
  self : MultibodyJointSet,
  mb_index : Int,
  multibody : Multibody,
) -> Unit {
  for i in 0..<multibody.links.length() {
    let link = multibody.links[i]
    if link.rigid_body.id >= 0 {
      self.ensure_body_capacity(link.rigid_body.id)
      self.body_links[link.rigid_body.id] = Some(LinkInfo::{
        mb_index,
        link_id: LinkId::new(link.internal_id),
      })
    }
  }
}

///|
fn MultibodyJointSet::ensure_body_capacity(
  self : MultibodyJointSet,
  id : Int,
) -> Unit {
  while self.body_links.length() <= id {
    self.body_links.push(None)
  }
}

///|
fn MultibodyJointSet::remove_multibody_at(
  self : MultibodyJointSet,
  index : Int,
) -> Unit {
  if index < 0 || index >= self.multibodies.length() {
    return
  }
  let last_index = self.multibodies.length() - 1
  if index != last_index {
    let moved_multibody = self.multibodies[last_index]
    self.multibodies[index] = moved_multibody
    for i in 0..<moved_multibody.links.length() {
      let link = moved_multibody.links[i]
      if link.rigid_body.id >= 0 {
        self.ensure_body_capacity(link.rigid_body.id)
        self.body_links[link.rigid_body.id] = Some(LinkInfo::{
          mb_index: index,
          link_id: LinkId::new(i),
        })
      }
    }
  }
  self.multibodies.pop() |> ignore
}

///|
fn MultibodyJointSet::register_body(
  self : MultibodyJointSet,
  handle : RigidBodyHandle,
) -> LinkInfo {
  let mb = Multibody::new_with_root(handle)
  let index = self.multibodies.length()
  self.multibodies.push(mb)
  let info = LinkInfo::{ mb_index: index, link_id: LinkId::new(0) }
  self.ensure_body_capacity(handle.id)
  self.body_links[handle.id] = Some(info)
  info
}

///|
fn MultibodyJointSet::lookup_body(
  self : MultibodyJointSet,
  handle : RigidBodyHandle,
) -> LinkInfo {
  self.ensure_body_capacity(handle.id)
  if self.body_links[handle.id] is Some(info) {
    info
  } else {
    self.register_body(handle)
  }
}

///|
pub fn MultibodyJointSet::insert(
  self : MultibodyJointSet,
  parent : RigidBodyHandle,
  child : RigidBodyHandle,
  joint : RevoluteJoint,
  wake_up : Bool,
) -> MultibodyJointHandle? {
  let parent_info = self.lookup_body(parent)
  let child_info = self.lookup_body(child)
  if parent_info.mb_index == child_info.mb_index {
    return None
  }
  if child_info.link_id.value != 0 {
    return None
  }
  let parent_mb = self.multibodies[parent_info.mb_index]
  let child_mb = self.multibodies[child_info.mb_index]
  let offset = parent_mb.links.length()
  for i in 0..<child_mb.links.length() {
    let link = child_mb.links[i]
    if i == 0 {
      link.parent_internal_id = parent_info.link_id.value
      link.joint = joint
    } else {
      link.parent_internal_id = link.parent_internal_id + offset
    }
    link.internal_id = offset + i
    parent_mb.links.push(link)
  }
  parent_mb.recompute_ndofs()
  self.multibodies[parent_info.mb_index] = parent_mb
  for i in 0..<child_mb.links.length() {
    let link = child_mb.links[i]
    self.ensure_body_capacity(link.rigid_body.id)
    self.body_links[link.rigid_body.id] = Some(LinkInfo::{
      mb_index: parent_info.mb_index,
      link_id: LinkId::new(offset + i),
    })
  }
  self.remove_multibody_at(child_info.mb_index)
  if wake_up {
    push_unique_handle(self.to_wake_up, parent)
    push_unique_handle(self.to_wake_up, child)
  }
  push_unique_pair(self.to_join, (parent, child))
  Some(MultibodyJointHandle::new(child.id))
}

///|
pub fn MultibodyJointSet::insert_kinematic(
  self : MultibodyJointSet,
  parent : RigidBodyHandle,
  child : RigidBodyHandle,
  joint : RevoluteJoint,
  wake_up : Bool,
) -> MultibodyJointHandle? {
  self.insert(parent, child, joint, wake_up)
}

///|
pub fn MultibodyJointSet::remove(
  self : MultibodyJointSet,
  handle : MultibodyJointHandle,
  wake_up : Bool,
) -> Unit {
  if handle.id < 0 || handle.id >= self.body_links.length() {
    return
  }
  if self.body_links[handle.id] is Some(info) {
    if info.mb_index < 0 || info.mb_index >= self.multibodies.length() {
      self.body_links[handle.id] = None
      return
    }
    let link_id = info.link_id.value
    if link_id <= 0 {
      return
    }
    let multibody = self.multibodies[info.mb_index]
    let subtree = collect_subtree_indices(multibody, link_id)
    if subtree.length() == 0 || subtree.length() == multibody.links.length() {
      return
    }
    let remaining : Array[Int] = []
    for i in 0..<multibody.links.length() {
      let mut in_subtree = false
      for j in 0..<subtree.length() {
        if subtree[j] == i {
          in_subtree = true
          break
        }
      }
      if !in_subtree {
        remaining.push(i)
      }
    }
    if wake_up {
      let removed_link = multibody.links[link_id]
      push_unique_handle(self.to_wake_up, removed_link.rigid_body)
      let parent_id = removed_link.parent_internal_id
      if parent_id >= 0 && parent_id < multibody.links.length() {
        let parent_link = multibody.links[parent_id]
        push_unique_handle(self.to_wake_up, parent_link.rigid_body)
      }
    }
    let remaining_mb = build_multibody_from_indices(multibody, remaining, 0)
    let subtree_mb = build_multibody_from_indices(multibody, subtree, link_id)
    if remaining_mb.links.length() <= 1 {
      let link = remaining_mb.links[0]
      if link.rigid_body.id >= 0 {
        self.ensure_body_capacity(link.rigid_body.id)
        self.body_links[link.rigid_body.id] = None
      }
      self.remove_multibody_at(info.mb_index)
    } else {
      self.multibodies[info.mb_index] = remaining_mb
      self.update_body_links_for_multibody(info.mb_index, remaining_mb)
    }
    if subtree_mb.links.length() <= 1 {
      let link = subtree_mb.links[0]
      if link.rigid_body.id >= 0 {
        self.ensure_body_capacity(link.rigid_body.id)
        self.body_links[link.rigid_body.id] = None
      }
    } else {
      let new_index = self.multibodies.length()
      self.multibodies.push(subtree_mb)
      self.update_body_links_for_multibody(new_index, subtree_mb)
    }
  } else {
    self.body_links[handle.id] = None
  }
}

///|
pub fn MultibodyJointSet::remove_multibody_articulations(
  self : MultibodyJointSet,
  handle : RigidBodyHandle,
  wake_up : Bool,
) -> Unit {
  if handle.id < 0 || handle.id >= self.body_links.length() {
    return
  }
  if self.body_links[handle.id] is Some(info) {
    if info.mb_index < 0 || info.mb_index >= self.multibodies.length() {
      self.body_links[handle.id] = None
      return
    }
    let removed_multibody = self.multibodies[info.mb_index]
    for i in 0..<removed_multibody.links.length() {
      let link = removed_multibody.links[i]
      if link.rigid_body.id >= 0 {
        self.ensure_body_capacity(link.rigid_body.id)
        self.body_links[link.rigid_body.id] = None
      }
      if wake_up {
        push_unique_handle(self.to_wake_up, link.rigid_body)
      }
    }
    self.remove_multibody_at(info.mb_index)
  } else {
    self.body_links[handle.id] = None
  }
}

///|
pub fn MultibodyJointSet::rigid_body_link(
  self : MultibodyJointSet,
  handle : RigidBodyHandle,
) -> MultibodyLinkId? {
  if handle.id < 0 || handle.id >= self.body_links.length() {
    return None
  }
  if self.body_links[handle.id] is Some(info) {
    Some(MultibodyLinkId::{
      multibody: MultibodyIndex::new(info.mb_index),
      id: info.link_id.value,
    })
  } else {
    None
  }
}

///|
pub fn MultibodyJointSet::get_multibody(
  self : MultibodyJointSet,
  index : MultibodyIndex,
) -> Multibody? {
  if index.value < 0 || index.value >= self.multibodies.length() {
    None
  } else {
    Some(self.multibodies[index.value])
  }
}

///|
pub fn MultibodyJointSet::get_multibody_mut(
  self : MultibodyJointSet,
  index : MultibodyIndex,
) -> Multibody? {
  self.get_multibody(index)
}

///|
pub fn MultibodyJointSet::get_multibody_mut_internal(
  self : MultibodyJointSet,
  index : MultibodyIndex,
) -> Multibody? {
  self.get_multibody(index)
}

///|
pub fn MultibodyJointSet::multibodies(
  self : MultibodyJointSet,
) -> Array[Multibody] {
  let result : Array[Multibody] = []
  for i in 0..<self.multibodies.length() {
    result.push(self.multibodies[i])
  }
  result
}

///|
pub fn MultibodyJointSet::get(
  self : MultibodyJointSet,
  handle : MultibodyJointHandle,
) -> (Multibody, LinkId)? {
  if handle.id < 0 {
    return None
  }
  if handle.id >= self.body_links.length() {
    return None
  }
  if self.body_links[handle.id] is Some(info) {
    if info.mb_index < 0 || info.mb_index >= self.multibodies.length() {
      return None
    }
    Some((self.multibodies[info.mb_index], info.link_id))
  } else {
    None
  }
}

///|
pub fn MultibodyJointSet::get_mut(
  self : MultibodyJointSet,
  handle : MultibodyJointHandle,
) -> (Multibody, LinkId)? {
  self.get(handle)
}

///|
pub fn MultibodyJointSet::get_mut_internal(
  self : MultibodyJointSet,
  handle : MultibodyJointHandle,
) -> (Multibody, LinkId)? {
  self.get(handle)
}

///|
pub fn MultibodyJointSet::get_unknown_gen(
  self : MultibodyJointSet,
  id : Int,
) -> (Multibody, LinkId, MultibodyJointHandle)? {
  if id < 0 || id >= self.body_links.length() {
    return None
  }
  if self.body_links[id] is Some(info) {
    if info.mb_index < 0 || info.mb_index >= self.multibodies.length() {
      return None
    }
    Some(
      (
        self.multibodies[info.mb_index],
        info.link_id,
        MultibodyJointHandle::new(id),
      ),
    )
  } else {
    None
  }
}

///|
pub fn MultibodyJointSet::joint_between(
  self : MultibodyJointSet,
  body1 : RigidBodyHandle,
  body2 : RigidBodyHandle,
) -> (MultibodyJointHandle, Multibody, MultibodyLink)? {
  if body1.id < 0 || body2.id < 0 {
    return None
  }
  if body1.id >= self.body_links.length() ||
    body2.id >= self.body_links.length() {
    return None
  }
  if self.body_links[body1.id] is Some(info1) &&
    self.body_links[body2.id] is Some(info2) {
    if info1.mb_index != info2.mb_index {
      return None
    }
    if info1.mb_index < 0 || info1.mb_index >= self.multibodies.length() {
      return None
    }
    let multibody = self.multibodies[info1.mb_index]
    if info1.link_id.value < 0 ||
      info1.link_id.value >= multibody.links.length() ||
      info2.link_id.value < 0 ||
      info2.link_id.value >= multibody.links.length() {
      return None
    }
    let link1 = multibody.links[info1.link_id.value]
    let link2 = multibody.links[info2.link_id.value]
    if link1.parent_internal_id == info2.link_id.value {
      Some((MultibodyJointHandle::new(body1.id), multibody, link1))
    } else if link2.parent_internal_id == info1.link_id.value {
      Some((MultibodyJointHandle::new(body2.id), multibody, link2))
    } else {
      None
    }
  } else {
    None
  }
}

///|
pub fn MultibodyJointSet::attached_joints(
  self : MultibodyJointSet,
  handle : RigidBodyHandle,
) -> Array[(RigidBodyHandle, RigidBodyHandle, MultibodyJointHandle)] {
  if handle.id < 0 || handle.id >= self.body_links.length() {
    return []
  }
  if self.body_links[handle.id] is Some(info) {
    if info.mb_index < 0 || info.mb_index >= self.multibodies.length() {
      return []
    }
    let multibody = self.multibodies[info.mb_index]
    if info.link_id.value < 0 || info.link_id.value >= multibody.links.length() {
      return []
    }
    let result : Array[(RigidBodyHandle, RigidBodyHandle, MultibodyJointHandle)] = []
    let current_link = multibody.links[info.link_id.value]
    if current_link.parent_internal_id != current_link.internal_id {
      let parent_id = current_link.parent_internal_id
      if parent_id >= 0 && parent_id < multibody.links.length() {
        let parent = multibody.links[parent_id].rigid_body
        result.push((parent, handle, MultibodyJointHandle::new(handle.id)))
      }
    }
    for i in 0..<multibody.links.length() {
      let link = multibody.links[i]
      if link.parent_internal_id == info.link_id.value &&
        !RigidBodyHandle::equals(link.rigid_body, handle) {
        result.push(
          (
            handle,
            link.rigid_body,
            MultibodyJointHandle::new(link.rigid_body.id),
          ),
        )
      }
    }
    result
  } else {
    []
  }
}

///|
pub fn MultibodyJointSet::attached_bodies(
  self : MultibodyJointSet,
  handle : RigidBodyHandle,
) -> Array[RigidBodyHandle] {
  if handle.id < 0 || handle.id >= self.body_links.length() {
    return []
  }
  if self.body_links[handle.id] is Some(info) {
    if info.mb_index < 0 || info.mb_index >= self.multibodies.length() {
      return []
    }
    let multibody = self.multibodies[info.mb_index]
    let result : Array[RigidBodyHandle] = []
    for i in 0..<multibody.links.length() {
      let link = multibody.links[i]
      if !RigidBodyHandle::equals(link.rigid_body, handle) {
        result.push(link.rigid_body)
      }
    }
    result
  } else {
    []
  }
}

///|
pub fn MultibodyJointSet::bodies_attached_with_enabled_joint(
  self : MultibodyJointSet,
  handle : RigidBodyHandle,
) -> Array[RigidBodyHandle] {
  self.attached_bodies(handle)
}

///|
fn MultibodyJointSet::remove_joints_attached_to_rigid_body(
  self : MultibodyJointSet,
  handle : RigidBodyHandle,
) -> Unit {
  let attached = self.attached_joints(handle)
  if attached.length() == 0 {
    return
  }
  let to_remove : Array[MultibodyJointHandle] = []
  for i in 0..<attached.length() {
    let joint = attached[i]
    to_remove.push(joint.2)
  }
  for i in 0..<to_remove.length() {
    self.remove(to_remove[i], true)
  }
}
