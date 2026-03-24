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
fn revolute_joint_axis3_or_default(axis : @core.Vec3) -> @core.Vec3 {
  if axis.length_squared() > 1.0e-12F {
    axis.normalize()
  } else {
    @core.Vec3::new(0.0F, 0.0F, 1.0F)
  }
}

///|
fn revolute_joint_frame_from_axis3(axis : @core.Vec3) -> @core.Quat {
  @core.rotation_between(
    @core.Vec3::new(1.0F, 0.0F, 0.0F),
    revolute_joint_axis3_or_default(axis),
  )
}

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
  let axis3 = revolute_joint_axis3_or_default(@core.Vec3::new(0.0F, 0.0F, 1.0F))
  let frame3 = revolute_joint_frame_from_axis3(axis3)
  {
    local_anchor1: self.local_anchor1,
    local_anchor2: self.local_anchor2,
    axis3,
    local_anchor1_3: @core.Vec3::new(
      self.local_anchor1.x,
      self.local_anchor1.y,
      0.0F,
    ),
    local_anchor2_3: @core.Vec3::new(
      self.local_anchor2.x,
      self.local_anchor2.y,
      0.0F,
    ),
    local_frame1_rotation: frame3,
    local_frame2_rotation: frame3,
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
  axis3 : @core.Vec3
  local_anchor1_3 : @core.Vec3
  local_anchor2_3 : @core.Vec3
  local_frame1_rotation : @core.Quat
  local_frame2_rotation : @core.Quat
  softness : SpringCoefficients
  motor : JointMotor
  motor_enabled : Bool
  limits : JointLimits
  limits_enabled : Bool
  // Solver warmstart cache for the (locked) linear DOFs of this revolute joint.
  // The motor/limit impulses are stored inside `motor` and `limits`.
  solver_lin_impulses : @core.Vec2
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
fn multibody_joint_local_displacements(
  joint : MultibodyJoint,
  displacements : Array[@core.Real]?,
  assembly_id : Int,
) -> Array[@core.Real] {
  let result : Array[@core.Real] = []
  let ndofs = joint.ndofs()
  if displacements is Some(global) {
    for i in 0..<ndofs {
      let index = assembly_id + i
      if index >= 0 && index < global.length() {
        result.push(global[index])
      } else {
        result.push(0.0F)
      }
    }
  } else {
    for _ in 0..<ndofs {
      result.push(0.0F)
    }
  }
  result
}

///|
fn multibody_joint_with_local_displacements(
  joint : MultibodyJoint,
  displacements : Array[@core.Real]?,
  assembly_id : Int,
) -> MultibodyJoint {
  let result = MultibodyJoint::{
    data: joint.data,
    kinematic: joint.kinematic,
    coords: joint.coords,
    angle: joint.angle,
    joint_rot: joint.joint_rot,
    axis3: joint.axis3,
    local_anchor1_3: joint.local_anchor1_3,
    local_anchor2_3: joint.local_anchor2_3,
    local_frame1_rotation: joint.local_frame1_rotation,
    local_frame2_rotation: joint.local_frame2_rotation,
    solver_lin_impulses: joint.solver_lin_impulses,
  }
  let joint_disp = multibody_joint_local_displacements(
    joint, displacements, assembly_id,
  )
  result.apply_displacement(joint_disp)
  result
}

///|
pub struct LinkId {
  value : Int

  fn new(value : Int) -> LinkId
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
  mut joint : MultibodyJoint
  mut local_to_parent : @core.Isometry2
  mut local_to_world : @core.Isometry2
}

///|
pub fn MultibodyLink::local_to_world(self : MultibodyLink) -> @core.Isometry2 {
  self.local_to_world
}

///|
pub fn MultibodyLink::local_to_parent(self : MultibodyLink) -> @core.Isometry2 {
  self.local_to_parent
}

///|
pub fn MultibodyLink::is_root(self : MultibodyLink) -> Bool {
  self.internal_id == 0
}

///|
pub fn MultibodyLink::link_id(self : MultibodyLink) -> LinkId {
  { value: self.internal_id }
}

///|
pub fn MultibodyLink::parent_id(self : MultibodyLink) -> LinkId {
  { value: self.parent_internal_id }
}

///|
pub fn MultibodyLink::rigid_body_handle(
  self : MultibodyLink,
) -> RigidBodyHandle {
  self.rigid_body
}

///|
struct Multibody {
  links : Array[MultibodyLink]
  mut ndofs : Int
  mut root_is_dynamic : Bool
  mut root_translation : @core.Vec2
  mut root_angle : @core.Real
  mut root_translation3_z : @core.Real
  mut root_rotation3_extra : @core.Quat
  mut velocities : DVector
  mut damping : DVector
  mut accelerations : DVector
  mut inv_augmented_mass : @core.DMatrix
  solver_id : Int
  mut self_contacts_enabled : Bool
}

///|
fn Multibody::new_with_root(handle : RigidBodyHandle) -> Multibody {
  let joint = MultibodyJoint::new(GenericJoint::default(), false)
  let link = MultibodyLink::{
    internal_id: 0,
    parent_internal_id: 0,
    assembly_id: 0,
    rigid_body: handle,
    joint,
    local_to_parent: @core.Isometry2::identity(),
    local_to_world: @core.Isometry2::identity(),
  }
  let links : Array[MultibodyLink] = [link]
  let n = SPATIAL_DIM
  let zeros = DVector::from_fn(n, fn(_) { 0.0F })
  {
    links,
    ndofs: n,
    root_is_dynamic: true,
    root_translation: @core.Vec2::zero(),
    root_angle: 0.0F,
    root_translation3_z: 0.0F,
    root_rotation3_extra: @core.Quat::identity(),
    velocities: zeros,
    damping: DVector::from_fn(n, fn(_) { 0.0F }),
    accelerations: DVector::from_fn(n, fn(_) { 0.0F }),
    inv_augmented_mass: @core.DMatrix::identity(n),
    solver_id: 0,
    self_contacts_enabled: true,
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
      assembly = assembly + link.joint.ndofs()
    }
    self.links[i] = link
  }
}

///|
fn Multibody::ensure_buffers(self : Multibody) -> Unit {
  let mut n = self.ndofs
  if n < 0 {
    n = 0
  }
  fn copy_prefix(src : DVector, dst : DVector, n : Int) -> Unit {
    let limit = if src.len() < n { src.len() } else { n }
    for i in 0..<limit {
      dst.set_value(i, src.value_at(i))
    }
  }

  if self.velocities.len() != n || self.velocities.as_array().length() != n {
    let new_vels = DVector::from_fn(n, fn(_) { 0.0F })
    copy_prefix(self.velocities, new_vels, n)
    self.velocities = new_vels
  }
  if self.damping.len() != n || self.damping.as_array().length() != n {
    let new_damping = DVector::from_fn(n, fn(_) { 0.0F })
    copy_prefix(self.damping, new_damping, n)
    self.damping = new_damping
  }
  if self.accelerations.len() != n ||
    self.accelerations.as_array().length() != n {
    let new_acc = DVector::from_fn(n, fn(_) { 0.0F })
    copy_prefix(self.accelerations, new_acc, n)
    self.accelerations = new_acc
  }

  // This port currently doesn't compute the augmented mass; keep a deterministic placeholder.
  self.inv_augmented_mass = @core.DMatrix::identity(n)
}

///|
fn Multibody::recompute_ndofs(self : Multibody) -> Unit {
  if self.links.length() <= 0 {
    self.ndofs = 0
  } else {
    self.root_is_dynamic = true
    let mut joint_ndofs = 0
    for i in 1..<self.links.length() {
      joint_ndofs = joint_ndofs + self.links[i].joint.ndofs()
    }
    self.ndofs = SPATIAL_DIM + joint_ndofs
    self.rebuild_indices(SPATIAL_DIM)
  }
  self.ensure_buffers()
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
  let mut joint_ndofs = 0
  for i in 1..<self.links.length() {
    joint_ndofs = joint_ndofs + self.links[i].joint.ndofs()
  }
  self.root_is_dynamic = is_dynamic
  self.ndofs = root_dofs + joint_ndofs
  self.rebuild_indices(root_dofs)
  self.ensure_buffers()
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
    let local_to_parent = link.joint.body_to_parent()
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
pub fn Multibody::num_links(self : Multibody) -> Int {
  self.links.length()
}

///|
pub fn Multibody::root(self : Multibody) -> MultibodyLink? {
  self.link({ value: 0 })
}

///|
pub fn Multibody::root_mut(self : Multibody) -> MultibodyLink? {
  // MoonBit doesn't have Rust-like borrowing; we return the same link view as `root`.
  self.root()
}

///|
pub fn Multibody::links_mut(self : Multibody) -> Array[MultibodyLink] {
  self.links
}

///|
pub fn Multibody::link_mut(self : Multibody, id : LinkId) -> MultibodyLink? {
  self.link(id)
}

///|
pub fn Multibody::kinematic_branch(
  self : Multibody,
  link_id : Int,
) -> Array[Int] {
  self.branch_ids({ value: link_id })
}

///|
pub fn Multibody::self_contacts_enabled(self : Multibody) -> Bool {
  self.self_contacts_enabled
}

///|
pub fn Multibody::set_self_contacts_enabled(
  self : Multibody,
  enabled : Bool,
) -> Unit {
  self.self_contacts_enabled = enabled
}

///|
pub fn Multibody::generalized_velocity(self : Multibody) -> DVector {
  // Full view into the internal generalized velocity buffer.
  DVector::new_view(self.velocities.data, 0, self.ndofs)
}

///|
pub fn Multibody::generalized_velocity_mut(self : Multibody) -> DVector {
  // Same as `generalized_velocity`; the returned view can be mutated via DVector setters.
  self.generalized_velocity()
}

///|
pub fn Multibody::generalized_acceleration(self : Multibody) -> DVector {
  DVector::new_view(self.accelerations.data, 0, self.ndofs)
}

///|
pub fn Multibody::damping_mut(self : Multibody) -> DVector {
  DVector::new_view(self.damping.data, 0, self.ndofs)
}

///|
pub fn Multibody::inv_augmented_mass(self : Multibody) -> @core.DMatrix {
  self.inv_augmented_mass
}

///|
pub fn Multibody::joint_velocity(
  self : Multibody,
  link : MultibodyLink,
) -> DVector {
  if link.internal_id == 0 {
    let root_ndofs = if self.root_is_dynamic { SPATIAL_DIM } else { 0 }
    DVector::new_view(self.velocities.data, 0, root_ndofs)
  } else {
    DVector::new_view(
      self.velocities.data,
      link.assembly_id,
      link.joint.ndofs(),
    )
  }
}

///|
pub fn Multibody::forward_kinematics_single_branch(
  self : Multibody,
  bodies : RigidBodySet,
  branch : Array[Int],
  displacements : Array[@core.Real]?,
  out_jacobian : Jacobian?,
) -> @core.Isometry2 {
  self.update_ndofs_with_root(bodies)
  if branch.length() == 0 {
    out_jacobian |> ignore
    return @core.Isometry2::identity()
  }
  let first = branch[0]
  if first < 0 || first >= self.links.length() {
    out_jacobian |> ignore
    return @core.Isometry2::identity()
  }
  if out_jacobian is Some(jacobian) {
    jacobian.ensure_cols(self.ndofs)
  }

  // Start at root.
  let root_pose = self.root_pose(displacements)
  let mut current_pose = root_pose
  let mut last = first
  if first == 0 {
    let root_link = self.links[0]
    root_link.local_to_parent = root_pose
    root_link.local_to_world = root_pose
    self.links[0] = root_link
  }

  // Propagate along the branch.
  for i in 1..<branch.length() {
    let link_id = branch[i]
    if link_id < 0 || link_id >= self.links.length() {
      continue
    }
    let link = self.links[link_id]
    let displaced_joint = multibody_joint_with_local_displacements(
      link.joint,
      displacements,
      link.assembly_id,
    )
    let local_to_parent = displaced_joint.body_to_parent()
    current_pose = current_pose.mul(local_to_parent)
    link.local_to_parent = local_to_parent
    link.local_to_world = current_pose
    self.links[link_id] = link
    last = link_id
  }

  // If a jacobian is requested, compute it for the last link of the branch.
  if out_jacobian is Some(jacobian) {
    self.compute_link_pose({ value: last }, displacements, Some(jacobian))
    |> ignore
  }
  if last < 0 || last >= self.links.length() {
    @core.Isometry2::identity()
  } else {
    self.links[last].local_to_world
  }
}

///|
pub fn Multibody::inverse_kinematics_delta(
  self : Multibody,
  link_id : LinkId,
  dx : @core.Real,
  dy : @core.Real,
  da : @core.Real,
  damping : @core.Real,
  displacements : DVector,
) -> Unit {
  let jacobian = self.body_jacobian(link_id)
  Multibody::inverse_kinematics_delta_with_jacobian(
    jacobian, dx, dy, da, damping, displacements,
  )
}

///|
pub fn Multibody::inverse_kinematics_delta_with_jacobian(
  jacobian : Jacobian,
  dx : @core.Real,
  dy : @core.Real,
  da : @core.Real,
  damping : @core.Real,
  displacements : DVector,
) -> Unit {
  ik_inverse_kinematics_delta_with_jacobian(
    jacobian, dx, dy, da, damping, displacements,
  )
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
    let joint_pos = current_pose.transform_point(
      link.joint.data.local_anchor1(),
    )
    joint_positions.push(joint_pos)
    joint_cols.push(link.assembly_id)
    let displaced_joint = multibody_joint_with_local_displacements(
      link.joint,
      displacements,
      link.assembly_id,
    )
    let local_to_parent = displaced_joint.body_to_parent()
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
    let joint_disp = multibody_joint_local_displacements(
      link.joint,
      Some(displacements),
      link.assembly_id,
    )
    if joint_disp.length() > 0 {
      link.joint.apply_displacement(joint_disp)
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
  index : @data.Index
}

///|
fn MultibodyJointHandle::new(index : @data.Index) -> MultibodyJointHandle {
  MultibodyJointHandle::{ index, }
}

///|
fn MultibodyJointHandle::arena_index(
  self : MultibodyJointHandle,
) -> @data.Index {
  self.index
}

///|
pub fn MultibodyJointHandle::from_raw_parts(
  id : Int,
  generation : Int,
) -> MultibodyJointHandle {
  MultibodyJointHandle::new(@data.Index::from_raw_parts(id, generation))
}

///|
pub fn MultibodyJointHandle::into_raw_parts(
  self : MultibodyJointHandle,
) -> (Int, Int) {
  self.index.into_raw_parts()
}

///|
pub fn MultibodyJointHandle::invalid() -> MultibodyJointHandle {
  MultibodyJointHandle::from_raw_parts(-1, -1)
}

///|
struct MultibodyIndex {
  index : @data.Index
}

///|
fn MultibodyIndex::new(index : @data.Index) -> MultibodyIndex {
  MultibodyIndex::{ index, }
}

///|
fn MultibodyIndex::equals(
  self : MultibodyIndex,
  other : MultibodyIndex,
) -> Bool {
  self.index.into_raw_parts() == other.index.into_raw_parts()
}

///|
fn MultibodyIndex::arena_index(self : MultibodyIndex) -> @data.Index {
  self.index
}

///|
pub fn MultibodyIndex::invalid() -> MultibodyIndex {
  MultibodyIndex::new(@data.Index::from_raw_parts(-1, -1))
}

///|
struct MultibodyLinkId {
  mut graph_id : Int
  multibody : MultibodyIndex
  id : Int
}

///|
fn MultibodyLinkId::new(
  graph_id : Int,
  multibody : MultibodyIndex,
  id : Int,
) -> MultibodyLinkId {
  { graph_id, multibody, id }
}

///|
pub fn MultibodyLinkId::default() -> MultibodyLinkId {
  MultibodyLinkId::new(-1, MultibodyIndex::invalid(), 0)
}

///|
priv struct MultibodyGraphEdge {
  node1 : Int
  node2 : Int
  parent : RigidBodyHandle
  child : RigidBodyHandle
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
struct MultibodyJointSet {
  multibodies : @data.Arena[Multibody]
  rb2mb : @data.Coarena[MultibodyLinkId]
  graph_nodes : Array[RigidBodyHandle]
  graph_edges : Array[MultibodyGraphEdge]
  adjacency : Array[Array[Int]]
  to_wake_up : Array[RigidBodyHandle]
  to_join : Array[(RigidBodyHandle, RigidBodyHandle)]
}

///|
fn rb_handle_index(handle : RigidBodyHandle) -> @data.Index {
  @data.Index::from_raw_parts(handle.id, handle.generation)
}

///|
fn remove_graph_edge_index(values : Array[Int], index : Int) -> Unit {
  let mut found = -1
  for i in 0..<values.length() {
    if values[i] == index {
      found = i
      break
    }
  }
  if found < 0 {
    return
  }
  let last_index = values.length() - 1
  if found != last_index {
    values[found] = values[last_index]
  }
  values.pop() |> ignore
}

///|
fn replace_graph_edge_index(
  values : Array[Int],
  old_index : Int,
  new_index : Int,
) -> Unit {
  for i in 0..<values.length() {
    if values[i] == old_index {
      values[i] = new_index
      return
    }
  }
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
    root_translation3_z: multibody.root_translation3_z,
    root_rotation3_extra: multibody.root_rotation3_extra,
    velocities: DVector::from_fn(0, fn(_) { 0.0F }),
    damping: DVector::from_fn(0, fn(_) { 0.0F }),
    accelerations: DVector::from_fn(0, fn(_) { 0.0F }),
    inv_augmented_mass: @core.DMatrix::identity(0),
    solver_id: multibody.solver_id,
    self_contacts_enabled: multibody.self_contacts_enabled,
  }
  new_multibody.recompute_ndofs()
  new_multibody
}

///|
pub fn MultibodyJointSet::new() -> MultibodyJointSet {
  {
    multibodies: @data.Arena::new(),
    rb2mb: @data.Coarena::new(),
    graph_nodes: [],
    graph_edges: [],
    adjacency: [],
    to_wake_up: [],
    to_join: [],
  }
}

///|
fn MultibodyJointSet::set_link(
  self : MultibodyJointSet,
  handle : RigidBodyHandle,
  link : MultibodyLinkId,
) -> Unit {
  self.rb2mb.set(rb_handle_index(handle), link, MultibodyLinkId::default())
}

///|
fn MultibodyJointSet::graph_node_for(
  self : MultibodyJointSet,
  handle : RigidBodyHandle,
) -> Int? {
  if self.rb2mb.get(rb_handle_index(handle)) is Some(link) {
    let graph_id = link.graph_id
    if graph_id >= 0 &&
      graph_id < self.graph_nodes.length() &&
      RigidBodyHandle::equals(self.graph_nodes[graph_id], handle) {
      Some(graph_id)
    } else {
      None
    }
  } else {
    None
  }
}

///|
fn MultibodyJointSet::update_graph_id(
  self : MultibodyJointSet,
  handle : RigidBodyHandle,
  graph_id : Int,
) -> Unit {
  let key = rb_handle_index(handle)
  if self.rb2mb.get(key) is Some(link) {
    link.graph_id = graph_id
    self.rb2mb.set(key, link, MultibodyLinkId::default())
  }
}

///|
fn MultibodyJointSet::ensure_graph_node(
  self : MultibodyJointSet,
  handle : RigidBodyHandle,
) -> Int {
  if self.graph_node_for(handle) is Some(node_id) {
    node_id
  } else {
    let node_id = self.graph_nodes.length()
    self.graph_nodes.push(handle)
    self.adjacency.push([])
    node_id
  }
}

///|
fn MultibodyJointSet::insert_graph_edge(
  self : MultibodyJointSet,
  parent : RigidBodyHandle,
  child : RigidBodyHandle,
) -> Unit {
  let node1 = self.ensure_graph_node(parent)
  let node2 = self.ensure_graph_node(child)
  let edge_id = self.graph_edges.length()
  self.graph_edges.push(MultibodyGraphEdge::{ node1, node2, parent, child })
  self.adjacency[node1].push(edge_id)
  self.adjacency[node2].push(edge_id)
}

///|
fn MultibodyJointSet::remove_graph_edge(
  self : MultibodyJointSet,
  edge_id : Int,
) -> MultibodyGraphEdge? {
  if edge_id < 0 || edge_id >= self.graph_edges.length() {
    return None
  }
  let removed_edge = self.graph_edges[edge_id]
  remove_graph_edge_index(self.adjacency[removed_edge.node1], edge_id)
  if removed_edge.node2 != removed_edge.node1 {
    remove_graph_edge_index(self.adjacency[removed_edge.node2], edge_id)
  }
  let last_index = self.graph_edges.length() - 1
  if edge_id != last_index {
    let moved_edge = self.graph_edges[last_index]
    self.graph_edges[edge_id] = moved_edge
    replace_graph_edge_index(
      self.adjacency[moved_edge.node1],
      last_index,
      edge_id,
    )
    if moved_edge.node2 != moved_edge.node1 {
      replace_graph_edge_index(
        self.adjacency[moved_edge.node2],
        last_index,
        edge_id,
      )
    }
  }
  self.graph_edges.pop() |> ignore
  Some(removed_edge)
}

///|
fn MultibodyJointSet::remove_graph_node(
  self : MultibodyJointSet,
  node_id : Int,
) -> Unit {
  if node_id < 0 || node_id >= self.graph_nodes.length() {
    return
  }
  while self.adjacency[node_id].length() > 0 {
    let last_edge = self.adjacency[node_id].length() - 1
    self.remove_graph_edge(self.adjacency[node_id][last_edge]) |> ignore
  }
  let last_index = self.graph_nodes.length() - 1
  if node_id != last_index {
    let moved_handle = self.graph_nodes[last_index]
    self.graph_nodes[node_id] = moved_handle
    self.adjacency[node_id] = self.adjacency[last_index]
    self.update_graph_id(moved_handle, node_id)
    let edges = self.adjacency[node_id]
    for i in 0..<edges.length() {
      let edge_id = edges[i]
      if edge_id < 0 || edge_id >= self.graph_edges.length() {
        continue
      }
      let edge = self.graph_edges[edge_id]
      self.graph_edges[edge_id] = MultibodyGraphEdge::{
        node1: if edge.node1 == last_index {
          node_id
        } else {
          edge.node1
        },
        node2: if edge.node2 == last_index {
          node_id
        } else {
          edge.node2
        },
        parent: edge.parent,
        child: edge.child,
      }
    }
  }
  self.graph_nodes.pop() |> ignore
  self.adjacency.pop() |> ignore
}

///|
fn MultibodyJointSet::remove_graph_node_for_handle(
  self : MultibodyJointSet,
  handle : RigidBodyHandle,
) -> Unit {
  if self.graph_node_for(handle) is Some(node_id) {
    self.remove_graph_node(node_id)
  }
}

///|
fn MultibodyJointSet::find_graph_edge_by_child(
  self : MultibodyJointSet,
  child : RigidBodyHandle,
) -> Int? {
  if self.graph_node_for(child) is Some(node_id) {
    let edges = self.adjacency[node_id]
    for i in 0..<edges.length() {
      let edge_id = edges[i]
      if edge_id >= 0 && edge_id < self.graph_edges.length() {
        let edge = self.graph_edges[edge_id]
        if RigidBodyHandle::equals(edge.child, child) {
          return Some(edge_id)
        }
      }
    }
  }
  None
}

///|
fn mb_split_values(text : String, sep : String) -> Array[String] {
  let values : Array[String] = []
  for part in text.split(sep[:]) {
    let value = part.to_string()
    if value.length() > 0 {
      values.push(value)
    }
  }
  values
}

///|
fn mb_parse_int_value(text : String) -> Int {
  let result : Result[Int, @strconv.StrConvError] = try? @strconv.parse_int(
    text[:],
    base=10,
  )
  if result is Ok(value) {
    value
  } else {
    0
  }
}

///|
fn mb_parse_real_value(text : String) -> @core.Real {
  let result : Result[Double, @strconv.StrConvError] = try? @strconv.parse_double(
    text[:],
  )
  if result is Ok(value) {
    Float::from_double(value)
  } else {
    0.0F
  }
}

///|
fn mb_parse_bool_value(text : String) -> Bool {
  text == "1"
}

///|
fn mb_motor_model_from_int(value : Int) -> MotorModel {
  if value == 1 {
    MotorModel::ForceBased
  } else {
    MotorModel::AccelerationBased
  }
}

///|
fn write_link(sb : StringBuilder, link : MultibodyLink) -> Unit {
  let (rbid, rbgen) = link.rigid_body.into_raw_parts()
  sb.write_string("L^")
  sb.write_object(link.parent_internal_id)
  sb.write_char('^')
  sb.write_object(rbid)
  sb.write_char('^')
  sb.write_object(rbgen)
  sb.write_char('^')
  sb.write_object(if link.joint.kinematic { 1 } else { 0 })
  sb.write_char('^')
  write_generic_joint_fields(sb, link.joint.data)
  sb.write_char('^')
  sb.write_object(link.joint.solver_lin_impulses.x)
  sb.write_char('^')
  sb.write_object(link.joint.solver_lin_impulses.y)
  sb.write_char('^')
  sb.write_object(link.joint.angle)
}

///|
fn parse_link(text : String) -> MultibodyLink? {
  if text.strip_prefix("L^"[:]) is Some(view) {
    let fields = mb_split_values(view.to_string(), "^")
    if fields.length() < 24 {
      return None
    }
    let parent_id = mb_parse_int_value(fields[0])
    let rbid = mb_parse_int_value(fields[1])
    let rbgen = mb_parse_int_value(fields[2])
    if fields.length() >= 65 {
      let kinematic = mb_parse_bool_value(fields[3])
      let data = parse_generic_joint(fields, 4)
      let solver_lin_x = mb_parse_real_value(fields[62])
      let solver_lin_y = mb_parse_real_value(fields[63])
      let joint_angle = mb_parse_real_value(fields[64])
      let joint = MultibodyJoint::new(data, kinematic)
      joint.solver_lin_impulses = @core.Vec2::new(solver_lin_x, solver_lin_y)
      joint.angle = joint_angle
      joint.joint_rot = @core.Rot2::from_angle(joint_angle)
      let handle = RigidBodyHandle::from_raw_parts(rbid, rbgen)
      return Some(MultibodyLink::{
        internal_id: 0,
        parent_internal_id: parent_id,
        assembly_id: 0,
        rigid_body: handle,
        joint,
        local_to_parent: @core.Isometry2::identity(),
        local_to_world: @core.Isometry2::identity(),
      })
    }
    let a1x = mb_parse_real_value(fields[3])
    let a1y = mb_parse_real_value(fields[4])
    let a2x = mb_parse_real_value(fields[5])
    let a2y = mb_parse_real_value(fields[6])
    let snat = mb_parse_real_value(fields[7])
    let sdamp = mb_parse_real_value(fields[8])
    let motor_enabled = mb_parse_bool_value(fields[9])
    let motor_target_vel = mb_parse_real_value(fields[10])
    let motor_target_pos = mb_parse_real_value(fields[11])
    let motor_stiffness = mb_parse_real_value(fields[12])
    let motor_damping = mb_parse_real_value(fields[13])
    let motor_max_force = mb_parse_real_value(fields[14])
    let motor_impulse = mb_parse_real_value(fields[15])
    let motor_model = mb_motor_model_from_int(mb_parse_int_value(fields[16]))
    let limits_enabled = mb_parse_bool_value(fields[17])
    let limit_min = mb_parse_real_value(fields[18])
    let limit_max = mb_parse_real_value(fields[19])
    let limit_impulse = mb_parse_real_value(fields[20])
    let solver_lin_x = mb_parse_real_value(fields[21])
    let solver_lin_y = mb_parse_real_value(fields[22])
    let joint_angle = mb_parse_real_value(fields[23])
    let a1z = if fields.length() >= 25 {
      mb_parse_real_value(fields[24])
    } else {
      0.0F
    }
    let a2z = if fields.length() >= 26 {
      mb_parse_real_value(fields[25])
    } else {
      0.0F
    }
    let axis3 = if fields.length() >= 29 {
      revolute_joint_axis3_or_default(
        @core.Vec3::new(
          mb_parse_real_value(fields[26]),
          mb_parse_real_value(fields[27]),
          mb_parse_real_value(fields[28]),
        ),
      )
    } else {
      revolute_joint_axis3_or_default(@core.Vec3::new(0.0F, 0.0F, 1.0F))
    }
    let frame3 = revolute_joint_frame_from_axis3(axis3)
    let softness = SpringCoefficients::new(snat, sdamp)
    let motor = JointMotor::{
      target_vel: motor_target_vel,
      target_pos: motor_target_pos,
      stiffness: motor_stiffness,
      damping: motor_damping,
      max_force: motor_max_force,
      impulse: motor_impulse,
      model: motor_model,
    }
    let limits = JointLimits::new(limit_min, limit_max)
    limits.impulse = limit_impulse
    let revolute = RevoluteJoint::{
      local_anchor1: @core.Vec2::new(a1x, a1y),
      local_anchor2: @core.Vec2::new(a2x, a2y),
      axis3,
      local_anchor1_3: @core.Vec3::new(a1x, a1y, a1z),
      local_anchor2_3: @core.Vec3::new(a2x, a2y, a2z),
      local_frame1_rotation: frame3,
      local_frame2_rotation: frame3,
      softness,
      motor,
      motor_enabled,
      limits,
      limits_enabled,
      solver_lin_impulses: @core.Vec2::new(solver_lin_x, solver_lin_y),
    }
    let joint = MultibodyJoint::from_revolute(revolute, false)
    joint.angle = joint_angle
    joint.joint_rot = @core.Rot2::from_angle(joint_angle)
    let handle = RigidBodyHandle::from_raw_parts(rbid, rbgen)
    Some(MultibodyLink::{
      internal_id: 0,
      parent_internal_id: parent_id,
      assembly_id: 0,
      rigid_body: handle,
      joint,
      local_to_parent: @core.Isometry2::identity(),
      local_to_world: @core.Isometry2::identity(),
    })
  } else {
    None
  }
}

///|
fn MultibodyJointSet::update_body_links_for_multibody(
  self : MultibodyJointSet,
  mb_index : MultibodyIndex,
  multibody : Multibody,
) -> Unit {
  for i in 0..<multibody.links.length() {
    let link = multibody.links[i]
    let graph_id = if self.rb2mb.get(rb_handle_index(link.rigid_body))
      is Some(existing) {
      existing.graph_id
    } else {
      self.ensure_graph_node(link.rigid_body)
    }
    self.set_link(
      link.rigid_body,
      MultibodyLinkId::new(graph_id, mb_index, link.internal_id),
    )
  }
}

///|
fn MultibodyJointSet::insert_graph_edges_for_multibody(
  self : MultibodyJointSet,
  multibody : Multibody,
) -> Unit {
  for i in 1..<multibody.links.length() {
    let link = multibody.links[i]
    let parent_id = link.parent_internal_id
    if parent_id >= 0 && parent_id < multibody.links.length() && parent_id != i {
      let parent = multibody.links[parent_id].rigid_body
      self.insert_graph_edge(parent, link.rigid_body)
    }
  }
}

///|
pub fn MultibodyJointSet::serialize(self : MultibodyJointSet) -> String {
  let sb = StringBuilder::new()
  sb.write_string("mbs=")
  let items = self.multibodies.iter()
  for i in 0..<items.length() {
    if i > 0 {
      sb.write_char('@')
    }
    let mb = items[i].1
    sb.write_string("MB^")
    sb.write_object(mb.root_translation.x)
    sb.write_char('^')
    sb.write_object(mb.root_translation.y)
    sb.write_char('^')
    sb.write_object(mb.root_angle)
    for j in 0..<mb.links.length() {
      sb.write_char('|')
      write_link(sb, mb.links[j])
    }
  }
  sb.to_string()
}

///|
pub fn MultibodyJointSet::deserialize(data : String) -> MultibodyJointSet {
  let set = MultibodyJointSet::new()
  if data.strip_prefix("mbs="[:]) is Some(view) {
    let raw = view.to_string()
    if raw.length() > 0 {
      for mb_text in raw.split("@"[:]) {
        let mb_str = mb_text.to_string()
        if mb_str.length() == 0 {
          continue
        }
        let parts = mb_split_values(mb_str, "|")
        if parts.length() < 2 {
          continue
        }
        let header = parts[0]
        if header.strip_prefix("MB^"[:]) is None {
          continue
        }
        let header_fields = mb_split_values(header, "^")
        let root_tx = if header_fields.length() > 1 {
          mb_parse_real_value(header_fields[1])
        } else {
          0.0F
        }
        let root_ty = if header_fields.length() > 2 {
          mb_parse_real_value(header_fields[2])
        } else {
          0.0F
        }
        let root_angle = if header_fields.length() > 3 {
          mb_parse_real_value(header_fields[3])
        } else {
          0.0F
        }
        let links : Array[MultibodyLink] = []
        for i in 1..<parts.length() {
          if parse_link(parts[i]) is Some(link) {
            links.push(link)
          }
        }
        if links.length() == 0 {
          continue
        }
        for i in 0..<links.length() {
          let link = links[i]
          link.internal_id = i
          if i == 0 {
            link.parent_internal_id = 0
          }
          links[i] = link
        }
        let mb = Multibody::{
          links,
          ndofs: 0,
          root_is_dynamic: true,
          root_translation: @core.Vec2::new(root_tx, root_ty),
          root_angle,
          root_translation3_z: 0.0F,
          root_rotation3_extra: @core.Quat::identity(),
          velocities: DVector::from_fn(0, fn(_) { 0.0F }),
          damping: DVector::from_fn(0, fn(_) { 0.0F }),
          accelerations: DVector::from_fn(0, fn(_) { 0.0F }),
          inv_augmented_mass: @core.DMatrix::identity(0),
          solver_id: 0,
          self_contacts_enabled: true,
        }
        mb.recompute_ndofs()
        let mb_handle = set.multibodies.insert(mb)
        let mb_index = MultibodyIndex::new(mb_handle)
        set.update_body_links_for_multibody(mb_index, mb)
        set.insert_graph_edges_for_multibody(mb)
      }
    }
  }
  set
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
  let items = self.multibodies.iter()
  for i in 0..<items.length() {
    let (mb_handle, mb) = items[i]
    mb.forward_kinematics(bodies, true)
    mb.update_rigid_bodies(bodies)
    self.multibodies.set(mb_handle, mb) |> ignore
  }
}

///|
pub fn MultibodyJointSet::revolute_joint_descriptors(
  self : MultibodyJointSet,
) -> Array[MultibodyRevoluteJointDesc] {
  let result : Array[MultibodyRevoluteJointDesc] = []
  let items = self.multibodies.iter()
  for i in 0..<items.length() {
    let (mb_handle, mb) = items[i]
    let mb_index = MultibodyIndex::new(mb_handle)
    for link_id in 1..<mb.links.length() {
      let link = mb.links[link_id]
      let parent_id = link.parent_internal_id
      if parent_id < 0 || parent_id >= mb.links.length() || parent_id == link_id {
        continue
      }
      if !link.joint.is_revolute() {
        continue
      }
      let parent_link = mb.links[parent_id]
      let graph_id = if self.rb2mb.get(rb_handle_index(link.rigid_body))
        is Some(ids) {
        ids.graph_id
      } else {
        -1
      }
      result.push(MultibodyRevoluteJointDesc::{
        link: MultibodyLinkId::new(graph_id, mb_index, link_id),
        parent: parent_link.rigid_body,
        child: link.rigid_body,
        joint: link.joint.as_revolute_joint(),
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
  if self.multibodies.get(link.multibody.arena_index()) is Some(mb) {
    if link.id <= 0 || link.id >= mb.links.length() {
      return
    }
    let lnk = mb.links[link.id]
    lnk.joint.solver_lin_impulses = lin_impulses
    let motors = lnk.joint.data.motors
    lnk.joint.data.motors = JointMotors3::{
      lin_x: motors.lin_x,
      lin_y: motors.lin_y,
      ang_z: motors.ang_z.set_impulse(motor_impulse),
    }
    let limits = lnk.joint.data.limits
    lnk.joint.data.limits = JointLimits3::{
      lin_x: limits.lin_x,
      lin_y: limits.lin_y,
      ang_z: limits.ang_z.set_impulse(limit_impulse),
    }
    mb.links[link.id] = lnk
    self.multibodies.set(link.multibody.arena_index(), mb) |> ignore
  }
}

///|
pub fn MultibodyJointSet::iter(
  self : MultibodyJointSet,
) -> Array[(MultibodyJointHandle, MultibodyLinkId, Multibody, MultibodyLink)] {
  let result : Array[
    (MultibodyJointHandle, MultibodyLinkId, Multibody, MultibodyLink),
  ] = []
  let links = self.rb2mb.iter()
  for i in 0..<links.length() {
    let (joint_handle, link) = links[i]
    if link.id <= 0 {
      continue
    }
    if self.multibodies.get(link.multibody.arena_index()) is Some(multibody) {
      if link.id >= 0 && link.id < multibody.links.length() {
        result.push(
          (
            MultibodyJointHandle::new(joint_handle),
            link,
            multibody,
            multibody.links[link.id],
          ),
        )
      }
    }
  }
  result
}

///|
fn MultibodyJointSet::register_body(
  self : MultibodyJointSet,
  handle : RigidBodyHandle,
) -> MultibodyLinkId {
  let mb_handle = self.multibodies.insert(Multibody::new_with_root(handle))
  let graph_id = self.ensure_graph_node(handle)
  let link = MultibodyLinkId::new(graph_id, MultibodyIndex::new(mb_handle), 0)
  self.set_link(handle, link)
  link
}

///|
fn MultibodyJointSet::lookup_body(
  self : MultibodyJointSet,
  handle : RigidBodyHandle,
) -> MultibodyLinkId {
  if self.rb2mb.get(rb_handle_index(handle)) is Some(link) {
    link
  } else {
    self.register_body(handle)
  }
}

///|
pub fn MultibodyJointSet::insert(
  self : MultibodyJointSet,
  parent : RigidBodyHandle,
  child : RigidBodyHandle,
  joint : GenericJoint,
  wake_up : Bool,
) -> MultibodyJointHandle? {
  let multibody_joint = MultibodyJoint::new(joint, false)
  let parent_link = self.lookup_body(parent)
  let child_link = self.lookup_body(child)
  if parent_link.multibody.equals(child_link.multibody) || child_link.id != 0 {
    return None
  }
  if self.multibodies.get(parent_link.multibody.arena_index())
    is Some(parent_mb) {
    if self.multibodies.remove(child_link.multibody.arena_index())
      is Some(child_mb) {
      let offset = parent_mb.links.length()
      for i in 0..<child_mb.links.length() {
        let link = child_mb.links[i]
        if i == 0 {
          link.parent_internal_id = parent_link.id
          link.joint = multibody_joint
        } else {
          link.parent_internal_id = link.parent_internal_id + offset
        }
        link.internal_id = offset + i
        parent_mb.links.push(link)
      }
      parent_mb.recompute_ndofs()
      self.multibodies.set(parent_link.multibody.arena_index(), parent_mb)
      |> ignore
      self.insert_graph_edge(parent, child)
      for i in 0..<child_mb.links.length() {
        let link = child_mb.links[i]
        let graph_id = if self.rb2mb.get(rb_handle_index(link.rigid_body))
          is Some(existing) {
          existing.graph_id
        } else {
          self.ensure_graph_node(link.rigid_body)
        }
        self.set_link(
          link.rigid_body,
          MultibodyLinkId::new(graph_id, parent_link.multibody, offset + i),
        )
      }
      if wake_up {
        push_unique_handle(self.to_wake_up, parent)
        push_unique_handle(self.to_wake_up, child)
      }
      push_unique_pair(self.to_join, (parent, child))
      Some(MultibodyJointHandle::new(rb_handle_index(child)))
    } else {
      None
    }
  } else {
    None
  }
}

///|
pub fn MultibodyJointSet::insert_kinematic(
  self : MultibodyJointSet,
  parent : RigidBodyHandle,
  child : RigidBodyHandle,
  joint : GenericJoint,
  wake_up : Bool,
) -> MultibodyJointHandle? {
  let multibody_joint = MultibodyJoint::new(joint, true)
  let parent_link = self.lookup_body(parent)
  let child_link = self.lookup_body(child)
  if parent_link.multibody.equals(child_link.multibody) || child_link.id != 0 {
    return None
  }
  if self.multibodies.get(parent_link.multibody.arena_index())
    is Some(parent_mb) {
    if self.multibodies.remove(child_link.multibody.arena_index())
      is Some(child_mb) {
      let offset = parent_mb.links.length()
      for i in 0..<child_mb.links.length() {
        let link = child_mb.links[i]
        if i == 0 {
          link.parent_internal_id = parent_link.id
          link.joint = multibody_joint
        } else {
          link.parent_internal_id = link.parent_internal_id + offset
        }
        link.internal_id = offset + i
        parent_mb.links.push(link)
      }
      parent_mb.recompute_ndofs()
      self.multibodies.set(parent_link.multibody.arena_index(), parent_mb)
      |> ignore
      self.insert_graph_edge(parent, child)
      for i in 0..<child_mb.links.length() {
        let link = child_mb.links[i]
        let graph_id = if self.rb2mb.get(rb_handle_index(link.rigid_body))
          is Some(existing) {
          existing.graph_id
        } else {
          self.ensure_graph_node(link.rigid_body)
        }
        self.set_link(
          link.rigid_body,
          MultibodyLinkId::new(graph_id, parent_link.multibody, offset + i),
        )
      }
      if wake_up {
        push_unique_handle(self.to_wake_up, parent)
        push_unique_handle(self.to_wake_up, child)
      }
      push_unique_pair(self.to_join, (parent, child))
      Some(MultibodyJointHandle::new(rb_handle_index(child)))
    } else {
      None
    }
  } else {
    None
  }
}

///|
pub fn MultibodyJointSet::remove(
  self : MultibodyJointSet,
  handle : MultibodyJointHandle,
  wake_up : Bool,
) -> Unit {
  if self.rb2mb.get(handle.arena_index()) is Some(removed) {
    if removed.id <= 0 {
      return
    }
    if self.multibodies.remove(removed.multibody.arena_index())
      is Some(multibody) {
      let removed_link = multibody.links[removed.id]
      if self.find_graph_edge_by_child(removed_link.rigid_body) is Some(edge_id) {
        self.remove_graph_edge(edge_id) |> ignore
      }
      let subtree = collect_subtree_indices(multibody, removed.id)
      if subtree.length() == 0 || subtree.length() == multibody.links.length() {
        self.multibodies.insert(multibody) |> ignore
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
        push_unique_handle(self.to_wake_up, removed_link.rigid_body)
        let parent_id = removed_link.parent_internal_id
        if parent_id >= 0 && parent_id < multibody.links.length() {
          let parent_link = multibody.links[parent_id]
          push_unique_handle(self.to_wake_up, parent_link.rigid_body)
        }
      }
      let remaining_mb = build_multibody_from_indices(multibody, remaining, 0)
      let subtree_mb = build_multibody_from_indices(
        multibody,
        subtree,
        removed.id,
      )
      if remaining_mb.links.length() <= 1 {
        let link = remaining_mb.links[0]
        self.rb2mb.remove(
          rb_handle_index(link.rigid_body),
          MultibodyLinkId::default(),
        )
        |> ignore
        self.remove_graph_node_for_handle(link.rigid_body)
      } else {
        let remaining_handle = self.multibodies.insert(remaining_mb)
        self.update_body_links_for_multibody(
          MultibodyIndex::new(remaining_handle),
          remaining_mb,
        )
      }
      if subtree_mb.links.length() <= 1 {
        let link = subtree_mb.links[0]
        self.rb2mb.remove(
          rb_handle_index(link.rigid_body),
          MultibodyLinkId::default(),
        )
        |> ignore
        self.remove_graph_node_for_handle(link.rigid_body)
      } else {
        let subtree_handle = self.multibodies.insert(subtree_mb)
        self.update_body_links_for_multibody(
          MultibodyIndex::new(subtree_handle),
          subtree_mb,
        )
      }
    }
  }
}

///|
pub fn MultibodyJointSet::remove_multibody_articulations(
  self : MultibodyJointSet,
  handle : RigidBodyHandle,
  wake_up : Bool,
) -> Unit {
  if self.rb2mb.get(rb_handle_index(handle)) is Some(link) {
    if self.multibodies.remove(link.multibody.arena_index()) is Some(multibody) {
      for i in 0..<multibody.links.length() {
        let rb_handle = multibody.links[i].rigid_body
        if wake_up {
          push_unique_handle(self.to_wake_up, rb_handle)
        }
        let graph_id = if self.rb2mb.get(rb_handle_index(rb_handle))
          is Some(ids) {
          ids.graph_id
        } else {
          -1
        }
        self.rb2mb.remove(
          rb_handle_index(rb_handle),
          MultibodyLinkId::default(),
        )
        |> ignore
        if graph_id >= 0 {
          self.remove_graph_node(graph_id)
        }
      }
    }
  }
}

///|
pub fn MultibodyJointSet::rigid_body_link(
  self : MultibodyJointSet,
  handle : RigidBodyHandle,
) -> MultibodyLinkId? {
  self.rb2mb.get(rb_handle_index(handle))
}

///|
pub fn MultibodyJointSet::get_multibody(
  self : MultibodyJointSet,
  index : MultibodyIndex,
) -> Multibody? {
  self.multibodies.get(index.arena_index())
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
  let items = self.multibodies.iter()
  for i in 0..<items.length() {
    result.push(items[i].1)
  }
  result
}

///|
pub fn MultibodyJointSet::get(
  self : MultibodyJointSet,
  handle : MultibodyJointHandle,
) -> (Multibody, LinkId)? {
  if self.rb2mb.get(handle.arena_index()) is Some(link) {
    if self.multibodies.get(link.multibody.arena_index()) is Some(multibody) {
      Some((multibody, LinkId::new(link.id)))
    } else {
      None
    }
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
  if self.rb2mb.get_unknown_gen(id) is Some(link) {
    if self.multibodies.get(link.multibody.arena_index()) is Some(multibody) {
      if self.rb2mb.get_gen(id) is Some(generation) {
        Some(
          (
            multibody,
            LinkId::new(link.id),
            MultibodyJointHandle::from_raw_parts(id, generation),
          ),
        )
      } else {
        None
      }
    } else {
      None
    }
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
  if self.rb2mb.get(rb_handle_index(body1)) is Some(link1) &&
    self.rb2mb.get(rb_handle_index(body2)) is Some(link2) {
    if !link1.multibody.equals(link2.multibody) {
      return None
    }
    if self.multibodies.get(link1.multibody.arena_index()) is Some(multibody) {
      if link1.id < 0 ||
        link1.id >= multibody.links.length() ||
        link2.id < 0 ||
        link2.id >= multibody.links.length() {
        return None
      }
      let child1 = multibody.links[link1.id]
      let child2 = multibody.links[link2.id]
      if child1.parent_internal_id == link2.id {
        Some(
          (MultibodyJointHandle::new(rb_handle_index(body1)), multibody, child1),
        )
      } else if child2.parent_internal_id == link1.id {
        Some(
          (MultibodyJointHandle::new(rb_handle_index(body2)), multibody, child2),
        )
      } else {
        None
      }
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
  if self.graph_node_for(handle) is Some(node_id) {
    let result : Array[(RigidBodyHandle, RigidBodyHandle, MultibodyJointHandle)] = []
    let edges = self.adjacency[node_id]
    for i in 0..<edges.length() {
      let edge_id = edges[i]
      if edge_id < 0 || edge_id >= self.graph_edges.length() {
        continue
      }
      let edge = self.graph_edges[edge_id]
      result.push(
        (
          edge.parent,
          edge.child,
          MultibodyJointHandle::new(rb_handle_index(edge.child)),
        ),
      )
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
  if self.graph_node_for(handle) is Some(node_id) {
    let result : Array[RigidBodyHandle] = []
    let edges = self.adjacency[node_id]
    for i in 0..<edges.length() {
      let edge_id = edges[i]
      if edge_id < 0 || edge_id >= self.graph_edges.length() {
        continue
      }
      let edge = self.graph_edges[edge_id]
      if RigidBodyHandle::equals(edge.parent, handle) {
        result.push(edge.child)
      } else {
        result.push(edge.parent)
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
    to_remove.push(attached[i].2)
  }
  for i in 0..<to_remove.length() {
    self.remove(to_remove[i], true)
  }
}
