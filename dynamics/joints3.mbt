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
  multibodies : @data.Arena[Multibody]
  rb2mb : @data.Coarena[MultibodyLinkId]
  graph_nodes : Array[RigidBodyHandle]
  graph_edges : Array[MultibodyGraphEdge]
  adjacency : Array[Array[Int]]
  to_wake_up : Array[RigidBodyHandle]
  to_join : Array[(RigidBodyHandle, RigidBodyHandle)]
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
fn MultibodyJointSet3::set_link(
  self : MultibodyJointSet3,
  handle : RigidBodyHandle,
  link : MultibodyLinkId,
) -> Unit {
  self.rb2mb.set(rb_handle_index(handle), link, MultibodyLinkId::default())
}

///|
fn MultibodyJointSet3::graph_node_for(
  self : MultibodyJointSet3,
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
fn MultibodyJointSet3::update_graph_id(
  self : MultibodyJointSet3,
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
fn MultibodyJointSet3::ensure_graph_node(
  self : MultibodyJointSet3,
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
fn MultibodyJointSet3::insert_graph_edge(
  self : MultibodyJointSet3,
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
fn MultibodyJointSet3::remove_graph_edge(
  self : MultibodyJointSet3,
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
fn MultibodyJointSet3::remove_graph_node(
  self : MultibodyJointSet3,
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
fn MultibodyJointSet3::remove_graph_node_for_handle(
  self : MultibodyJointSet3,
  handle : RigidBodyHandle,
) -> Unit {
  if self.graph_node_for(handle) is Some(node_id) {
    self.remove_graph_node(node_id)
  }
}

///|
fn MultibodyJointSet3::find_graph_edge_by_child(
  self : MultibodyJointSet3,
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
fn MultibodyJointSet3::register_body(
  self : MultibodyJointSet3,
  handle : RigidBodyHandle,
) -> MultibodyLinkId {
  let mb_handle = self.multibodies.insert(Multibody::new_with_root(handle))
  let graph_id = self.ensure_graph_node(handle)
  let link = MultibodyLinkId::new(graph_id, MultibodyIndex::new(mb_handle), 0)
  self.set_link(handle, link)
  link
}

///|
fn MultibodyJointSet3::lookup_body(
  self : MultibodyJointSet3,
  handle : RigidBodyHandle,
) -> MultibodyLinkId {
  if self.rb2mb.get(rb_handle_index(handle)) is Some(link) {
    link
  } else {
    self.register_body(handle)
  }
}

///|
fn MultibodyJointSet3::update_body_links_for_multibody(
  self : MultibodyJointSet3,
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
pub fn MultibodyJointSet3::take_wake_up(
  self : MultibodyJointSet3,
) -> Array[RigidBodyHandle] {
  let result : Array[RigidBodyHandle] = []
  for i in 0..<self.to_wake_up.length() {
    result.push(self.to_wake_up[i])
  }
  self.to_wake_up.clear()
  result
}

///|
pub fn MultibodyJointSet3::take_to_join(
  self : MultibodyJointSet3,
) -> Array[(RigidBodyHandle, RigidBodyHandle)] {
  let result : Array[(RigidBodyHandle, RigidBodyHandle)] = []
  for i in 0..<self.to_join.length() {
    result.push(self.to_join[i])
  }
  self.to_join.clear()
  result
}

///|
pub fn MultibodyJointSet3::sync_rigid_bodies_from_multibodies(
  self : MultibodyJointSet3,
  bodies : RigidBodySet3,
) -> Unit {
  let items = self.multibodies.iter()
  for i in 0..<items.length() {
    let (mb_handle, mb) = items[i]
    mb3_forward_kinematics(mb, bodies, true)
    mb3_update_rigid_bodies(mb, bodies)
    self.multibodies.set(mb_handle, mb) |> ignore
  }
}

///|
pub fn MultibodyJointSet3::revolute_joint_descriptors(
  self : MultibodyJointSet3,
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
pub fn MultibodyJointSet3::set_revolute_joint_solver_impulses(
  self : MultibodyJointSet3,
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
pub fn MultibodyJointSet3::insert(
  self : MultibodyJointSet3,
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
pub fn MultibodyJointSet3::insert_kinematic(
  self : MultibodyJointSet3,
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
  if self.rb2mb.get(handle.arena_index()) is Some(link) {
    if self.multibodies.get(link.multibody.arena_index()) is Some(multibody) {
      Some((Multibody3::{ inner: multibody }, LinkId::new(link.id)))
    } else {
      None
    }
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
pub fn MultibodyJointSet3::remove_multibody_articulations(
  self : MultibodyJointSet3,
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
pub fn MultibodyJointSet3::rigid_body_link(
  self : MultibodyJointSet3,
  handle : RigidBodyHandle,
) -> MultibodyLinkId? {
  self.rb2mb.get(rb_handle_index(handle))
}

///|
pub fn MultibodyJointSet3::get_multibody(
  self : MultibodyJointSet3,
  index : MultibodyIndex,
) -> Multibody3? {
  if self.multibodies.get(index.arena_index()) is Some(multibody) {
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
  let items = self.multibodies.iter()
  for i in 0..<items.length() {
    result.push(Multibody3::{ inner: items[i].1 })
  }
  result
}

///|
pub fn MultibodyJointSet3::get_unknown_gen(
  self : MultibodyJointSet3,
  id : Int,
) -> (Multibody3, LinkId, MultibodyJointHandle)? {
  if self.rb2mb.get_unknown_gen(id) is Some(link) {
    if self.multibodies.get(link.multibody.arena_index()) is Some(multibody) {
      if self.rb2mb.get_gen(id) is Some(generation) {
        Some(
          (
            Multibody3::{ inner: multibody },
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
pub fn MultibodyJointSet3::joint_between(
  self : MultibodyJointSet3,
  body1 : RigidBodyHandle,
  body2 : RigidBodyHandle,
) -> (MultibodyJointHandle, Multibody3, MultibodyLink)? {
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
          (
            MultibodyJointHandle::new(rb_handle_index(body1)),
            Multibody3::{ inner: multibody },
            child1,
          ),
        )
      } else if child2.parent_internal_id == link1.id {
        Some(
          (
            MultibodyJointHandle::new(rb_handle_index(body2)),
            Multibody3::{ inner: multibody },
            child2,
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
pub fn MultibodyJointSet3::attached_joints(
  self : MultibodyJointSet3,
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
pub fn MultibodyJointSet3::attached_bodies(
  self : MultibodyJointSet3,
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
pub fn MultibodyJointSet3::bodies_attached_with_enabled_joint(
  self : MultibodyJointSet3,
  handle : RigidBodyHandle,
) -> Array[RigidBodyHandle] {
  self.attached_bodies(handle)
}

///|
pub fn MultibodyJointSet3::iter(
  self : MultibodyJointSet3,
) -> Array[(MultibodyJointHandle, MultibodyLinkId, Multibody3, MultibodyLink)] {
  let result : Array[
    (MultibodyJointHandle, MultibodyLinkId, Multibody3, MultibodyLink),
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
            Multibody3::{ inner: multibody },
            multibody.links[link.id],
          ),
        )
      }
    }
  }
  result
}

///|
fn MultibodyJointSet3::remove_joints_attached_to_rigid_body(
  self : MultibodyJointSet3,
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
fn mb3_project_pose_to_2d(pose : @core.Isometry3) -> @core.Isometry2 {
  let axis = pose.rotation.to_scaled_axis()
  @core.Isometry2::new(
    @core.Vec2::new(pose.translation.x, pose.translation.y),
    @core.Rot2::from_angle(axis.z),
  )
}

///|
fn mb3_cached_root_pose(multibody : Multibody) -> @core.Isometry3 {
  if multibody.links.length() > 0 {
    @core.Isometry3::new(
      @core.Vec3::new(
        multibody.root_translation.x,
        multibody.root_translation.y,
        multibody.root_translation3_z,
      ),
      multibody.root_rotation3_extra.mul(
        @core.rotation_from_scaled_axis(
          @core.Vec3::new(0.0F, 0.0F, multibody.root_angle),
        ),
      ),
    )
  } else {
    @core.Isometry3::identity()
  }
}

///|
fn mb3_update_root_from_bodies(
  multibody : Multibody,
  bodies : RigidBodySet3,
) -> Unit {
  if multibody.links.length() > 0 {
    let root = multibody.links[0].rigid_body
    let translation = bodies.translation(root)
    multibody.root_translation = @core.Vec2::new(translation.x, translation.y)
    multibody.root_angle = bodies.rotation(root).to_scaled_axis().z
  }
}

///|
fn mb3_update_ndofs_with_root(
  multibody : Multibody,
  bodies : RigidBodySet3,
) -> Unit {
  if multibody.links.length() <= 0 {
    multibody.ndofs = 0
    return
  }
  let root = multibody.links[0]
  let is_dynamic = if bodies.get(root.rigid_body) is Some(body) {
    !(body.body_type is RigidBodyType::Fixed)
  } else {
    true
  }
  let root_dofs = if is_dynamic { 6 } else { 0 }
  let mut joint_ndofs = 0
  for i in 1..<multibody.links.length() {
    joint_ndofs = joint_ndofs + multibody.links[i].joint.ndofs()
  }
  multibody.root_is_dynamic = is_dynamic
  multibody.ndofs = root_dofs + joint_ndofs
  multibody.rebuild_indices(root_dofs)
  multibody.ensure_buffers()
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
fn mb3_store_projected_link_poses(
  multibody : Multibody,
  root_pose : @core.Isometry3,
) -> Unit {
  if multibody.links.length() <= 0 {
    return
  }

  let world_poses : Array[@core.Isometry3] = []
  for _ in 0..<multibody.links.length() {
    world_poses.push(@core.Isometry3::identity())
  }
  world_poses[0] = root_pose

  let root_link = multibody.links[0]
  let root_pose2 = mb3_project_pose_to_2d(root_pose)
  root_link.local_to_parent = root_pose2
  root_link.local_to_world = root_pose2
  multibody.links[0] = root_link

  for i in 1..<multibody.links.length() {
    let link = multibody.links[i]
    let parent_pose = world_poses[link.parent_internal_id]
    let local_to_parent3 = mb3_multibody_body_to_parent(
      link.joint,
      multibody_joint_local_displacements(link.joint, None, link.assembly_id),
    )
    let world_pose = parent_pose.mul(local_to_parent3)
    world_poses[i] = world_pose
    link.local_to_parent = mb3_project_pose_to_2d(local_to_parent3)
    link.local_to_world = mb3_project_pose_to_2d(world_pose)
    multibody.links[i] = link
  }
}

///|
fn mb3_forward_kinematics(
  multibody : Multibody,
  bodies : RigidBodySet3,
  update_dofs : Bool,
) -> Unit {
  mb3_update_ndofs_with_root(multibody, bodies)
  if multibody.links.length() <= 0 {
    update_dofs |> ignore
    return
  }
  if update_dofs {
    mb3_update_root_from_bodies(multibody, bodies)
  }
  mb3_store_projected_link_poses(multibody, mb3_cached_root_pose(multibody))
}

///|
fn mb3_update_rigid_bodies(
  multibody : Multibody,
  bodies : RigidBodySet3,
) -> Unit {
  for i in 0..<multibody.links.length() {
    let link = multibody.links[i]
    if bodies.get_mut(link.rigid_body) is Some(body) {
      body.set_position(link.local_to_world, false) |> ignore
    }
  }
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
  mb3_forward_kinematics(self.inner, bodies, update_dofs)
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
  mb3_update_ndofs_with_root(self.inner, bodies)
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
