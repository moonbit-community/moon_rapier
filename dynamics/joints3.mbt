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
/// 3D-flavored impulse joint set.
pub struct ImpulseJointSet3 {
  inner : ImpulseJointSet

  fn new() -> ImpulseJointSet3
}

///|
pub fn ImpulseJointSet3::new() -> ImpulseJointSet3 {
  { inner: ImpulseJointSet::new() }
}

///|
pub fn ImpulseJointSet3::as_2d(self : ImpulseJointSet3) -> ImpulseJointSet {
  self.inner
}

///|
pub fn ImpulseJointSet3::insert(
  self : ImpulseJointSet3,
  body1 : RigidBodyHandle,
  body2 : RigidBodyHandle,
  joint : GenericJoint,
  wake_up : Bool,
) -> ImpulseJointHandle {
  self.inner.insert(body1, body2, joint, wake_up)
}

///|
/// 3D-flavored multibody joint set.
struct MultibodyJointSet3 {
  mut multibodies : @data.Arena[Multibody]
  mut rb2mb : @data.Coarena[MultibodyLinkId]
  mut graph_nodes : Array[RigidBodyHandle]
  mut graph_edges : Array[MultibodyGraphEdge]
  mut adjacency : Array[Array[Int]]
  mut to_wake_up : Array[RigidBodyHandle]
  mut to_join : Array[(RigidBodyHandle, RigidBodyHandle)]
}

///|
pub fn MultibodyJointSet3::new() -> MultibodyJointSet3 {
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
pub fn MultibodyJointSet3::as_2d(
  self : MultibodyJointSet3,
) -> MultibodyJointSet {
  MultibodyJointSet::{
    multibodies: self.multibodies,
    rb2mb: self.rb2mb,
    graph_nodes: self.graph_nodes,
    graph_edges: self.graph_edges,
    adjacency: self.adjacency,
    to_wake_up: self.to_wake_up,
    to_join: self.to_join,
  }
}

///|
fn MultibodyJointSet3::overwrite_from(
  self : MultibodyJointSet3,
  inner : MultibodyJointSet,
) -> Unit {
  self.multibodies = inner.multibodies
  self.rb2mb = inner.rb2mb
  self.graph_nodes = inner.graph_nodes
  self.graph_edges = inner.graph_edges
  self.adjacency = inner.adjacency
  self.to_wake_up = inner.to_wake_up
  self.to_join = inner.to_join
}

///|
pub fn MultibodyJointSet3::take_wake_up(
  self : MultibodyJointSet3,
) -> Array[RigidBodyHandle] {
  let inner = self.as_2d()
  let result = inner.take_wake_up()
  self.overwrite_from(inner)
  result
}

///|
pub fn MultibodyJointSet3::take_to_join(
  self : MultibodyJointSet3,
) -> Array[(RigidBodyHandle, RigidBodyHandle)] {
  let inner = self.as_2d()
  let result = inner.take_to_join()
  self.overwrite_from(inner)
  result
}

///|
pub fn MultibodyJointSet3::insert(
  self : MultibodyJointSet3,
  parent : RigidBodyHandle,
  child : RigidBodyHandle,
  joint : GenericJoint,
  wake_up : Bool,
) -> MultibodyJointHandle? {
  let inner = self.as_2d()
  let result = inner.insert(parent, child, joint, wake_up)
  self.overwrite_from(inner)
  result
}

///|
pub fn MultibodyJointSet3::insert_kinematic(
  self : MultibodyJointSet3,
  parent : RigidBodyHandle,
  child : RigidBodyHandle,
  joint : GenericJoint,
  wake_up : Bool,
) -> MultibodyJointHandle? {
  let inner = self.as_2d()
  let result = inner.insert_kinematic(parent, child, joint, wake_up)
  self.overwrite_from(inner)
  result
}

///|
pub struct Multibody3 {
  inner : Multibody
}

///|
pub fn Multibody3::ndofs(self : Multibody3) -> Int {
  mb3_logical_ndofs(self.inner)
}

///|
pub fn MultibodyJointSet3::get(
  self : MultibodyJointSet3,
  handle : MultibodyJointHandle,
) -> (Multibody3, LinkId)? {
  if self.as_2d().get(handle) is Some((multibody, link_id)) {
    Some((Multibody3::{ inner: multibody }, link_id))
  } else {
    None
  }
}

///|
pub fn MultibodyJointSet3::get_mut(
  self : MultibodyJointSet3,
  handle : MultibodyJointHandle,
) -> (Multibody3, LinkId)? {
  self.get(handle)
}

///|
pub fn MultibodyJointSet3::get_mut_internal(
  self : MultibodyJointSet3,
  handle : MultibodyJointHandle,
) -> (Multibody3, LinkId)? {
  self.get(handle)
}

///|
pub fn MultibodyJointSet3::remove(
  self : MultibodyJointSet3,
  handle : MultibodyJointHandle,
  wake_up : Bool,
) -> Unit {
  let inner = self.as_2d()
  inner.remove(handle, wake_up)
  self.overwrite_from(inner)
}

///|
pub fn MultibodyJointSet3::remove_multibody_articulations(
  self : MultibodyJointSet3,
  handle : RigidBodyHandle,
  wake_up : Bool,
) -> Unit {
  let inner = self.as_2d()
  inner.remove_multibody_articulations(handle, wake_up)
  self.overwrite_from(inner)
}

///|
pub fn MultibodyJointSet3::rigid_body_link(
  self : MultibodyJointSet3,
  handle : RigidBodyHandle,
) -> MultibodyLinkId? {
  self.as_2d().rigid_body_link(handle)
}

///|
pub fn MultibodyJointSet3::get_multibody(
  self : MultibodyJointSet3,
  index : MultibodyIndex,
) -> Multibody3? {
  if self.as_2d().get_multibody(index) is Some(multibody) {
    Some(Multibody3::{ inner: multibody })
  } else {
    None
  }
}

///|
pub fn MultibodyJointSet3::get_multibody_mut(
  self : MultibodyJointSet3,
  index : MultibodyIndex,
) -> Multibody3? {
  self.get_multibody(index)
}

///|
pub fn MultibodyJointSet3::get_multibody_mut_internal(
  self : MultibodyJointSet3,
  index : MultibodyIndex,
) -> Multibody3? {
  self.get_multibody(index)
}

///|
pub fn MultibodyJointSet3::multibodies(
  self : MultibodyJointSet3,
) -> Array[Multibody3] {
  let result : Array[Multibody3] = []
  let multibodies = self.as_2d().multibodies()
  for i in 0..<multibodies.length() {
    result.push(Multibody3::{ inner: multibodies[i] })
  }
  result
}

///|
pub fn MultibodyJointSet3::get_unknown_gen(
  self : MultibodyJointSet3,
  id : Int,
) -> (Multibody3, LinkId, MultibodyJointHandle)? {
  if self.as_2d().get_unknown_gen(id) is Some((multibody, link_id, handle)) {
    Some((Multibody3::{ inner: multibody }, link_id, handle))
  } else {
    None
  }
}

///|
pub fn MultibodyJointSet3::joint_between(
  self : MultibodyJointSet3,
  body1 : RigidBodyHandle,
  body2 : RigidBodyHandle,
) -> (MultibodyJointHandle, Multibody3, MultibodyLink)? {
  if self.as_2d().joint_between(body1, body2) is Some((handle, multibody, link)) {
    Some((handle, Multibody3::{ inner: multibody }, link))
  } else {
    None
  }
}

///|
pub fn MultibodyJointSet3::attached_joints(
  self : MultibodyJointSet3,
  handle : RigidBodyHandle,
) -> Array[(RigidBodyHandle, RigidBodyHandle, MultibodyJointHandle)] {
  self.as_2d().attached_joints(handle)
}

///|
pub fn MultibodyJointSet3::attached_bodies(
  self : MultibodyJointSet3,
  handle : RigidBodyHandle,
) -> Array[RigidBodyHandle] {
  self.as_2d().attached_bodies(handle)
}

///|
pub fn MultibodyJointSet3::bodies_attached_with_enabled_joint(
  self : MultibodyJointSet3,
  handle : RigidBodyHandle,
) -> Array[RigidBodyHandle] {
  self.as_2d().bodies_attached_with_enabled_joint(handle)
}

///|
pub fn MultibodyJointSet3::iter(
  self : MultibodyJointSet3,
) -> Array[(MultibodyJointHandle, MultibodyLinkId, Multibody3, MultibodyLink)] {
  let result : Array[
    (MultibodyJointHandle, MultibodyLinkId, Multibody3, MultibodyLink),
  ] = []
  let items = self.as_2d().iter()
  for i in 0..<items.length() {
    let (handle, link_id, multibody, link) = items[i]
    result.push((handle, link_id, Multibody3::{ inner: multibody }, link))
  }
  result
}

///|
fn MultibodyJointSet3::remove_joints_attached_to_rigid_body(
  self : MultibodyJointSet3,
  handle : RigidBodyHandle,
) -> Unit {
  let inner = self.as_2d()
  inner.remove_joints_attached_to_rigid_body(handle)
  self.overwrite_from(inner)
}

///|
pub struct Jacobian3 {
  mut cols_ : Int
  data : Array[@core.Real]
}

///|
pub fn Jacobian3::zeros(size : Int) -> Jacobian3 {
  let data : Array[@core.Real] = []
  for _ in 0..<(size * 6) {
    data.push(0.0F)
  }
  { cols_: size, data }
}

///|
pub fn Jacobian3::cols(self : Jacobian3) -> Int {
  self.cols_
}

///|
pub fn Jacobian3::value_at(
  self : Jacobian3,
  row : Int,
  col : Int,
) -> @core.Real {
  if row < 0 || row >= 6 || col < 0 || col >= self.cols_ {
    0.0F
  } else {
    self.data[col * 6 + row]
  }
}

///|
fn Jacobian3::ensure_cols(self : Jacobian3, cols : Int) -> Unit {
  if self.cols_ != cols {
    self.cols_ = cols
    self.data.clear()
    for _ in 0..<(cols * 6) {
      self.data.push(0.0F)
    }
  } else {
    for i in 0..<self.data.length() {
      self.data[i] = 0.0F
    }
  }
}

///|
fn Jacobian3::set_value(
  self : Jacobian3,
  row : Int,
  col : Int,
  value : @core.Real,
) -> Unit {
  if row >= 0 && row < 6 && col >= 0 && col < self.cols_ {
    self.data[col * 6 + row] = value
  }
}

///|
fn Jacobian3::clear_column(self : Jacobian3, col : Int) -> Unit {
  if col < 0 || col >= self.cols_ {
    return
  }
  let base = col * 6
  for row in 0..<6 {
    self.data[base + row] = 0.0F
  }
}

///|
fn mb3_logical_ndofs(multibody : Multibody) -> Int {
  if multibody.root_is_dynamic {
    multibody.ndofs() + 3
  } else {
    multibody.ndofs()
  }
}

///|
fn mb3_joint_logical_column(multibody : Multibody, assembly_id : Int) -> Int {
  if multibody.root_is_dynamic && assembly_id >= 3 {
    assembly_id + 3
  } else {
    assembly_id
  }
}

///|
fn mb3_set_spatial_column(
  jacobian : Jacobian3,
  col : Int,
  linear : @core.Vec3,
  angular : @core.Vec3,
) -> Unit {
  jacobian.set_value(0, col, linear.x)
  jacobian.set_value(1, col, linear.y)
  jacobian.set_value(2, col, linear.z)
  jacobian.set_value(3, col, angular.x)
  jacobian.set_value(4, col, angular.y)
  jacobian.set_value(5, col, angular.z)
}

///|
fn mb3_cached_root_pose(multibody : Multibody) -> @core.Isometry3 {
  if multibody.links.length() > 0 {
    let pose2 = multibody.links[0].local_to_world
    @core.Isometry3::new(
      @core.Vec3::new(
        pose2.translation.x,
        pose2.translation.y,
        multibody.root_translation3_z,
      ),
      multibody.root_rotation3_extra.mul(
        @core.rotation_from_scaled_axis(
          @core.Vec3::new(0.0F, 0.0F, pose2.rotation.angle()),
        ),
      ),
    )
  } else {
    @core.Isometry3::identity()
  }
}

///|
fn mb3_root_pose_from_bodies(
  multibody : Multibody,
  bodies : RigidBodySet3,
  displacements : Array[@core.Real]?,
) -> @core.Isometry3 {
  let mut pose = if multibody.links.length() > 0 {
    let root = multibody.links[0].rigid_body
    let body_translation = bodies.translation(root)
    @core.Isometry3::new(
      @core.Vec3::new(
        body_translation.x,
        body_translation.y,
        multibody.root_translation3_z,
      ),
      multibody.root_rotation3_extra.mul(bodies.rotation(root)),
    )
  } else {
    @core.Isometry3::identity()
  }

  if multibody.root_is_dynamic && displacements is Some(disp) {
    let dx = if disp.length() > 0 { disp[0] } else { 0.0F }
    let dy = if disp.length() > 1 { disp[1] } else { 0.0F }
    let dz = if disp.length() > 2 { disp[2] } else { 0.0F }
    let rot_delta = @core.rotation_from_scaled_axis(
      @core.Vec3::new(
        if disp.length() > 3 {
          disp[3]
        } else {
          0.0F
        },
        if disp.length() > 4 {
          disp[4]
        } else {
          0.0F
        },
        if disp.length() > 5 {
          disp[5]
        } else {
          0.0F
        },
      ),
    )
    pose = @core.Isometry3::new(
      pose.translation.add(@core.Vec3::new(dx, dy, dz)),
      rot_delta.mul(pose.rotation),
    )
  }

  pose
}

///|
fn mb3_multibody_body_to_parent(
  joint : MultibodyJoint,
  displacements : Array[@core.Real],
) -> @core.Isometry3 {
  let locked = joint.data.locked_axes().bits()
  let frame1 = @core.Isometry3::new(
    joint.local_anchor1_3,
    joint.local_frame1_rotation,
  )
  let frame2 = @core.Isometry3::new(
    joint.local_anchor2_3,
    joint.local_frame2_rotation,
  )
  let last_displacement = if displacements.length() > 0 {
    displacements[displacements.length() - 1]
  } else {
    0.0F
  }
  let angle = if (locked & (1 << 2)) == 0 {
    joint.angle + last_displacement
  } else {
    0.0F
  }
  let lin_x_disp = if displacements.length() > 0 {
    displacements[0]
  } else {
    0.0F
  }
  let lin_y_disp = if displacements.length() > 1 {
    displacements[1]
  } else if (locked & (1 << 0)) != 0 && displacements.length() > 0 {
    displacements[0]
  } else {
    0.0F
  }
  let mut transform = @core.Isometry3::new(
    @core.Vec3::zero(),
    @core.rotation_from_scaled_axis(@core.Vec3::new(angle, 0.0F, 0.0F)),
  ).mul(frame2.inverse())

  let lin_x = if (locked & (1 << 0)) == 0 {
    joint.coords.x + lin_x_disp
  } else {
    0.0F
  }
  let lin_y = if (locked & (1 << 1)) == 0 {
    joint.coords.y + lin_y_disp
  } else {
    0.0F
  }

  if (locked & (1 << 0)) == 0 {
    transform = @core.Isometry3::from_translation(
      @core.Vec3::new(lin_x, 0.0F, 0.0F),
    ).mul(transform)
  }
  if (locked & (1 << 1)) == 0 {
    transform = @core.Isometry3::from_translation(
      @core.Vec3::new(0.0F, lin_y, 0.0F),
    ).mul(transform)
  }

  frame1.mul(transform)
}

///|
fn mb3_compute_link_pose(
  multibody : Multibody,
  id : LinkId,
  root_pose : @core.Isometry3,
  displacements : Array[@core.Real]?,
  out_jacobian : Jacobian3?,
) -> @core.Isometry3 {
  let branch = multibody.branch_ids(id)
  if branch.length() <= 0 {
    return @core.Isometry3::identity()
  }

  if out_jacobian is Some(jacobian) {
    jacobian.ensure_cols(mb3_logical_ndofs(multibody))
  }

  let dof_positions : Array[@core.Vec3] = []
  let dof_axes : Array[@core.Vec3] = []
  let dof_cols : Array[Int] = []
  let dof_is_angular : Array[Bool] = []
  let root_position = root_pose.translation
  let mut current_pose = root_pose

  for i in 1..<branch.length() {
    let link = multibody.links[branch[i]]
    let joint_origin = current_pose.transform_point(link.joint.local_anchor1_3)
    let frame1_rot = current_pose.rotation.mul(link.joint.local_frame1_rotation)
    let locked = link.joint.data.locked_axes().bits()
    let joint_base_col = mb3_joint_logical_column(multibody, link.assembly_id)
    let joint_disp : Array[@core.Real] = []
    let mut local_col = 0
    if (locked & (1 << 0)) == 0 {
      let axis = frame1_rot.rotate_vec3(@core.Vec3::new(1.0F, 0.0F, 0.0F))
      dof_positions.push(joint_origin)
      dof_axes.push(axis)
      dof_cols.push(joint_base_col + local_col)
      dof_is_angular.push(false)
      if displacements is Some(disp) &&
        joint_base_col + local_col < disp.length() {
        joint_disp.push(disp[joint_base_col + local_col])
      } else {
        joint_disp.push(0.0F)
      }
      local_col = local_col + 1
    }
    if (locked & (1 << 1)) == 0 {
      let axis = frame1_rot.rotate_vec3(@core.Vec3::new(0.0F, 1.0F, 0.0F))
      dof_positions.push(joint_origin)
      dof_axes.push(axis)
      dof_cols.push(joint_base_col + local_col)
      dof_is_angular.push(false)
      if displacements is Some(disp) &&
        joint_base_col + local_col < disp.length() {
        joint_disp.push(disp[joint_base_col + local_col])
      } else {
        joint_disp.push(0.0F)
      }
      local_col = local_col + 1
    }
    if (locked & (1 << 2)) == 0 {
      let axis = current_pose.rotation.rotate_vec3(link.joint.axis3)
      dof_positions.push(joint_origin)
      dof_axes.push(axis)
      dof_cols.push(joint_base_col + local_col)
      dof_is_angular.push(true)
      if displacements is Some(disp) &&
        joint_base_col + local_col < disp.length() {
        joint_disp.push(disp[joint_base_col + local_col])
      } else {
        joint_disp.push(0.0F)
      }
    }

    current_pose = current_pose.mul(
      mb3_multibody_body_to_parent(link.joint, joint_disp),
    )
  }

  if out_jacobian is Some(jacobian) {
    let link_position = current_pose.translation
    if multibody.root_is_dynamic {
      mb3_set_spatial_column(
        jacobian,
        0,
        @core.Vec3::new(1.0F, 0.0F, 0.0F),
        @core.Vec3::zero(),
      )
      mb3_set_spatial_column(
        jacobian,
        1,
        @core.Vec3::new(0.0F, 1.0F, 0.0F),
        @core.Vec3::zero(),
      )
      mb3_set_spatial_column(
        jacobian,
        2,
        @core.Vec3::new(0.0F, 0.0F, 1.0F),
        @core.Vec3::zero(),
      )

      let root_axes : Array[@core.Vec3] = [
        root_pose.rotation.rotate_vec3(@core.Vec3::new(1.0F, 0.0F, 0.0F)),
        root_pose.rotation.rotate_vec3(@core.Vec3::new(0.0F, 1.0F, 0.0F)),
        root_pose.rotation.rotate_vec3(@core.Vec3::new(0.0F, 0.0F, 1.0F)),
      ]
      let rel = link_position.sub(root_position)
      for axis_index in 0..<3 {
        let axis = root_axes[axis_index]
        mb3_set_spatial_column(jacobian, 3 + axis_index, axis.cross(rel), axis)
      }
    }

    for i in 0..<dof_positions.length() {
      let axis = dof_axes[i]
      if dof_is_angular[i] {
        let joint_origin = dof_positions[i]
        let linear = axis.cross(link_position.sub(joint_origin))
        mb3_set_spatial_column(jacobian, dof_cols[i], linear, axis)
      } else {
        mb3_set_spatial_column(jacobian, dof_cols[i], axis, @core.Vec3::zero())
      }
    }
  }

  current_pose
}

///|
fn mb3_matrix_inverse_6x6(data : Array[@core.Real]) -> Array[@core.Real]? {
  if data.length() != 36 {
    return None
  }

  let augmented : Array[@core.Real] = []
  for row in 0..<6 {
    for col in 0..<6 {
      augmented.push(data[row * 6 + col])
    }
    for col in 0..<6 {
      augmented.push(if row == col { 1.0F } else { 0.0F })
    }
  }

  for pivot in 0..<6 {
    let mut best_row = pivot
    let mut best_abs = @core.abs(augmented[pivot * 12 + pivot])
    for row in (pivot + 1)..<6 {
      let candidate = @core.abs(augmented[row * 12 + pivot])
      if candidate > best_abs {
        best_abs = candidate
        best_row = row
      }
    }

    if best_abs <= 1.0e-8F {
      return None
    }

    if best_row != pivot {
      for col in 0..<12 {
        let temp = augmented[pivot * 12 + col]
        augmented[pivot * 12 + col] = augmented[best_row * 12 + col]
        augmented[best_row * 12 + col] = temp
      }
    }

    let pivot_value = augmented[pivot * 12 + pivot]
    for col in 0..<12 {
      augmented[pivot * 12 + col] = augmented[pivot * 12 + col] / pivot_value
    }

    for row in 0..<6 {
      if row == pivot {
        continue
      }
      let factor = augmented[row * 12 + pivot]
      if @core.abs(factor) <= 1.0e-12F {
        continue
      }
      for col in 0..<12 {
        augmented[row * 12 + col] = augmented[row * 12 + col] -
          factor * augmented[pivot * 12 + col]
      }
    }
  }

  let inverse : Array[@core.Real] = []
  for row in 0..<6 {
    for col in 0..<6 {
      inverse.push(augmented[row * 12 + 6 + col])
    }
  }
  Some(inverse)
}

///|
fn mb3_inverse_kinematics_delta_with_jacobian(
  jacobian : Jacobian3,
  desired_movement : Array[@core.Real],
  damping : @core.Real,
  displacements : DVector,
) -> Unit {
  let jj : Array[@core.Real] = []
  for _ in 0..<36 {
    jj.push(0.0F)
  }

  for row in 0..<6 {
    for col in 0..<6 {
      let mut value = 0.0F
      for j in 0..<jacobian.cols() {
        value = value + jacobian.value_at(row, j) * jacobian.value_at(col, j)
      }
      if row == col {
        value = value + damping * damping
      }
      jj[row * 6 + col] = value
    }
  }

  if mb3_matrix_inverse_6x6(jj) is Some(inv_jj) {
    let solved : Array[@core.Real] = []
    for row in 0..<6 {
      let mut value = 0.0F
      for col in 0..<6 {
        value = value + inv_jj[row * 6 + col] * desired_movement[col]
      }
      solved.push(value)
    }

    for col in 0..<jacobian.cols() {
      let mut delta = 0.0F
      for row in 0..<6 {
        delta = delta + jacobian.value_at(row, col) * solved[row]
      }
      if col < displacements.len() {
        displacements.set_value(col, displacements.value_at(col) + delta)
      }
    }
  }
}

///|
pub fn Multibody3::forward_kinematics(
  self : Multibody3,
  bodies : RigidBodySet3,
  update_dofs : Bool,
) -> Unit {
  self.inner.forward_kinematics(bodies.as_2d(), update_dofs)
}

///|
pub fn Multibody3::apply_displacements(
  self : Multibody3,
  displacements : Array[@core.Real],
) -> Unit {
  let projected : Array[@core.Real] = []
  if self.inner.root_is_dynamic {
    if displacements.length() > 0 {
      projected.push(displacements[0])
    }
    if displacements.length() > 1 {
      projected.push(displacements[1])
    }
    if displacements.length() > 2 {
      self.inner.root_translation3_z = self.inner.root_translation3_z +
        displacements[2]
    }
    let rot_delta = @core.rotation_from_scaled_axis(
      @core.Vec3::new(
        if displacements.length() > 3 {
          displacements[3]
        } else {
          0.0F
        },
        if displacements.length() > 4 {
          displacements[4]
        } else {
          0.0F
        },
        if displacements.length() > 5 {
          displacements[5]
        } else {
          0.0F
        },
      ),
    )
    self.inner.root_rotation3_extra = rot_delta.mul(
      self.inner.root_rotation3_extra,
    )
    if displacements.length() > 5 {
      projected.push(displacements[5])
    }
    for i in 6..<displacements.length() {
      projected.push(displacements[i])
    }
    self.inner.apply_displacements(projected)
  } else {
    self.inner.apply_displacements(displacements)
  }
}

///|
pub fn Multibody3::link_pose(
  self : Multibody3,
  id : LinkId,
) -> @core.Isometry3? {
  if self.inner.link(id) is Some(_) {
    Some(
      mb3_compute_link_pose(
        self.inner,
        id,
        mb3_cached_root_pose(self.inner),
        None,
        None,
      ),
    )
  } else {
    None
  }
}

///|
pub fn Multibody3::body_jacobian(self : Multibody3, id : LinkId) -> Jacobian3 {
  let jacobian = Jacobian3::zeros(0)
  mb3_compute_link_pose(
    self.inner,
    id,
    mb3_cached_root_pose(self.inner),
    None,
    Some(jacobian),
  )
  |> ignore
  jacobian
}

///|
pub fn Multibody3::forward_kinematics_single_link(
  self : Multibody3,
  bodies : RigidBodySet3,
  id : LinkId,
  displacements : Array[@core.Real]?,
  jacobian : Jacobian3?,
) -> @core.Isometry3 {
  self.inner.update_ndofs_with_root(bodies.as_2d())
  mb3_compute_link_pose(
    self.inner,
    id,
    mb3_root_pose_from_bodies(self.inner, bodies, displacements),
    displacements,
    jacobian,
  )
}

///|
pub fn Multibody3::inverse_kinematics(
  self : Multibody3,
  bodies : RigidBodySet3,
  id : LinkId,
  options : InverseKinematicsOption,
  target : @core.Isometry3,
  can_move : (MultibodyLink) -> Bool,
  displacements : DVector,
) -> Unit {
  let branch = self.inner.branch_ids(id)
  let can_move_branch : Array[Bool] = []
  for branch_id in branch {
    can_move_branch.push(can_move(self.inner.links[branch_id]))
  }

  let jacobian = Jacobian3::zeros(0)
  for _ in 0..<options.max_iters {
    let pose = self.forward_kinematics_single_link(
      bodies,
      id,
      Some(displacements.as_array()),
      Some(jacobian),
    )

    for i in 0..<branch.length() {
      if can_move_branch[i] {
        continue
      }

      let link_id = branch[i]
      let link = self.inner.links[link_id]
      if link_id == 0 && self.inner.root_is_dynamic {
        for col in 0..<6 {
          jacobian.clear_column(col)
        }
      } else {
        jacobian.clear_column(
          mb3_joint_logical_column(self.inner, link.assembly_id),
        )
      }
    }

    let delta_lin = target.translation.sub(pose.translation)
    let delta_ang = target.rotation
      .mul(pose.rotation.inverse())
      .to_scaled_axis()
    let desired : Array[@core.Real] = [
      delta_lin.x,
      delta_lin.y,
      delta_lin.z,
      delta_ang.x,
      delta_ang.y,
      delta_ang.z,
    ]

    if !options.constrained_axes.contains(JointAxesMask::lin_x()) {
      desired[0] = 0.0F
    }
    if !options.constrained_axes.contains(JointAxesMask::lin_y()) {
      desired[1] = 0.0F
    }
    if !options.constrained_axes.contains(JointAxesMask::ang_z()) {
      desired[5] = 0.0F
    }

    let linear_error = Float::sqrt(
      desired[0] * desired[0] +
      desired[1] * desired[1] +
      desired[2] * desired[2],
    )
    let angular_error = Float::sqrt(
      desired[3] * desired[3] +
      desired[4] * desired[4] +
      desired[5] * desired[5],
    )
    if linear_error <= options.epsilon_linear &&
      angular_error <= options.epsilon_angular {
      break
    }

    mb3_inverse_kinematics_delta_with_jacobian(
      jacobian,
      desired,
      options.damping,
      displacements,
    )
  }
}
