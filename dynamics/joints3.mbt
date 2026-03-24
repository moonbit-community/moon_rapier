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
  multibodies : Array[Multibody]
  body_links : Array[LinkInfo?]
  rb_graph_ids : Array[Int]
  rb_graph_generations : Array[Int]
  graph_nodes : Array[RigidBodyHandle]
  graph_edges : Array[MultibodyGraphEdge3]
  adjacency : Array[Array[Int]]
  to_wake_up : Array[RigidBodyHandle]
  to_join : Array[(RigidBodyHandle, RigidBodyHandle)]
}

///|
priv struct MultibodyGraphEdge3 {
  node1 : Int
  node2 : Int
  parent : RigidBodyHandle
  child : RigidBodyHandle
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
pub fn MultibodyJointSet3::new() -> MultibodyJointSet3 {
  {
    multibodies: [],
    body_links: [],
    rb_graph_ids: [],
    rb_graph_generations: [],
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
    body_links: self.body_links,
    to_wake_up: self.to_wake_up,
    to_join: self.to_join,
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
fn MultibodyJointSet3::ensure_body_capacity(
  self : MultibodyJointSet3,
  id : Int,
) -> Unit {
  while self.body_links.length() <= id {
    self.body_links.push(None)
  }
}

///|
fn MultibodyJointSet3::ensure_graph_mapping_capacity(
  self : MultibodyJointSet3,
  id : Int,
) -> Unit {
  if id < 0 {
    return
  }
  let mut index = self.rb_graph_ids.length()
  while index <= id {
    self.rb_graph_ids.push(-1)
    self.rb_graph_generations.push(0)
    index = index + 1
  }
}

///|
fn MultibodyJointSet3::graph_node_for(
  self : MultibodyJointSet3,
  handle : RigidBodyHandle,
) -> Int? {
  if handle.id < 0 || handle.id >= self.rb_graph_ids.length() {
    return None
  }
  let node_id = self.rb_graph_ids[handle.id]
  if node_id < 0 || node_id >= self.graph_nodes.length() {
    return None
  }
  if self.rb_graph_generations[handle.id] != handle.generation {
    return None
  }
  if !RigidBodyHandle::equals(self.graph_nodes[node_id], handle) {
    return None
  }
  Some(node_id)
}

///|
fn MultibodyJointSet3::ensure_graph_node(
  self : MultibodyJointSet3,
  handle : RigidBodyHandle,
) -> Int {
  self.ensure_graph_mapping_capacity(handle.id)
  if self.graph_node_for(handle) is Some(node_id) {
    node_id
  } else {
    let node_id = self.graph_nodes.length()
    self.graph_nodes.push(handle)
    self.adjacency.push([])
    self.rb_graph_ids[handle.id] = node_id
    self.rb_graph_generations[handle.id] = handle.generation
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
  self.graph_edges.push(MultibodyGraphEdge3::{ node1, node2, parent, child })
  self.adjacency[node1].push(edge_id)
  self.adjacency[node2].push(edge_id)
}

///|
fn MultibodyJointSet3::remove_graph_edge(
  self : MultibodyJointSet3,
  edge_id : Int,
) -> MultibodyGraphEdge3? {
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
  let removed_handle = self.graph_nodes[node_id]
  if removed_handle.id >= 0 && removed_handle.id < self.rb_graph_ids.length() {
    self.rb_graph_ids[removed_handle.id] = -1
  }
  let last_index = self.graph_nodes.length() - 1
  if node_id != last_index {
    let moved_handle = self.graph_nodes[last_index]
    self.graph_nodes[node_id] = moved_handle
    self.adjacency[node_id] = self.adjacency[last_index]
    if moved_handle.id >= 0 && moved_handle.id < self.rb_graph_ids.length() {
      self.rb_graph_ids[moved_handle.id] = node_id
      self.rb_graph_generations[moved_handle.id] = moved_handle.generation
    }
    let edges = self.adjacency[node_id]
    for i in 0..<edges.length() {
      let edge_id = edges[i]
      if edge_id >= 0 && edge_id < self.graph_edges.length() {
        let edge = self.graph_edges[edge_id]
        self.graph_edges[edge_id] = MultibodyGraphEdge3::{
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
fn MultibodyJointSet3::update_body_links_for_multibody(
  self : MultibodyJointSet3,
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
fn MultibodyJointSet3::remove_multibody_at(
  self : MultibodyJointSet3,
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
fn MultibodyJointSet3::register_body(
  self : MultibodyJointSet3,
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
fn MultibodyJointSet3::lookup_body(
  self : MultibodyJointSet3,
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
pub fn MultibodyJointSet3::insert(
  self : MultibodyJointSet3,
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
  self.insert_graph_edge(parent, child)
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
pub fn MultibodyJointSet3::insert_kinematic(
  self : MultibodyJointSet3,
  parent : RigidBodyHandle,
  child : RigidBodyHandle,
  joint : RevoluteJoint,
  wake_up : Bool,
) -> MultibodyJointHandle? {
  self.insert(parent, child, joint, wake_up)
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
  if handle.id < 0 || handle.id >= self.body_links.length() {
    return None
  }
  if self.body_links[handle.id] is Some(info) {
    if info.mb_index < 0 || info.mb_index >= self.multibodies.length() {
      return None
    }
    let mb = self.multibodies[info.mb_index]
    Some((Multibody3::{ inner: mb }, info.link_id))
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
    let removed_link = multibody.links[link_id]
    if self.find_graph_edge_by_child(removed_link.rigid_body) is Some(edge_id) {
      self.remove_graph_edge(edge_id) |> ignore
    }
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
        self.remove_graph_node_for_handle(link.rigid_body)
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
        self.remove_graph_node_for_handle(link.rigid_body)
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
pub fn MultibodyJointSet3::remove_multibody_articulations(
  self : MultibodyJointSet3,
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
    let removed_handles : Array[RigidBodyHandle] = []
    for i in 0..<removed_multibody.links.length() {
      let link = removed_multibody.links[i]
      removed_handles.push(link.rigid_body)
      if link.rigid_body.id >= 0 {
        self.ensure_body_capacity(link.rigid_body.id)
        self.body_links[link.rigid_body.id] = None
      }
      if wake_up {
        push_unique_handle(self.to_wake_up, link.rigid_body)
      }
    }
    self.remove_multibody_at(info.mb_index)
    for i in 0..<removed_handles.length() {
      self.remove_graph_node_for_handle(removed_handles[i])
    }
  } else {
    self.body_links[handle.id] = None
  }
}

///|
pub fn MultibodyJointSet3::rigid_body_link(
  self : MultibodyJointSet3,
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
pub fn MultibodyJointSet3::get_multibody(
  self : MultibodyJointSet3,
  index : MultibodyIndex,
) -> Multibody3? {
  if index.value < 0 || index.value >= self.multibodies.length() {
    None
  } else {
    Some(Multibody3::{ inner: self.multibodies[index.value] })
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
  for i in 0..<self.multibodies.length() {
    result.push(Multibody3::{ inner: self.multibodies[i] })
  }
  result
}

///|
pub fn MultibodyJointSet3::get_unknown_gen(
  self : MultibodyJointSet3,
  id : Int,
) -> (Multibody3, LinkId, MultibodyJointHandle)? {
  if id < 0 || id >= self.body_links.length() {
    return None
  }
  if self.body_links[id] is Some(info) {
    if info.mb_index < 0 || info.mb_index >= self.multibodies.length() {
      return None
    }
    Some(
      (
        Multibody3::{ inner: self.multibodies[info.mb_index] },
        info.link_id,
        MultibodyJointHandle::new(id),
      ),
    )
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
      Some(
        (
          MultibodyJointHandle::new(body1.id),
          Multibody3::{ inner: multibody },
          link1,
        ),
      )
    } else if link2.parent_internal_id == info1.link_id.value {
      Some(
        (
          MultibodyJointHandle::new(body2.id),
          Multibody3::{ inner: multibody },
          link2,
        ),
      )
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
          MultibodyJointHandle::from_raw_parts(
            edge.child.id,
            edge.child.generation,
          ),
        ),
      )
    }
    result
  } else {
    return []
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
  for i in 0..<self.body_links.length() {
    if self.body_links[i] is Some(info) &&
      info.link_id.value > 0 &&
      info.mb_index >= 0 &&
      info.mb_index < self.multibodies.length() {
      let multibody = self.multibodies[info.mb_index]
      let link = multibody.links[info.link_id.value]
      result.push(
        (
          MultibodyJointHandle::new(i),
          MultibodyLinkId::{
            multibody: MultibodyIndex::new(info.mb_index),
            id: info.link_id.value,
          },
          Multibody3::{ inner: multibody },
          link,
        ),
      )
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
    let joint = attached[i]
    to_remove.push(joint.2)
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
fn mb3_revolute_body_to_parent(
  joint : RevoluteJoint,
  angle : @core.Real,
) -> @core.Isometry3 {
  let frame1 = @core.Isometry3::new(
    joint.local_anchor1_3,
    joint.local_frame1_rotation,
  )
  let frame2 = @core.Isometry3::new(
    joint.local_anchor2_3,
    joint.local_frame2_rotation,
  )
  let free_axis_rotation = @core.Isometry3::new(
    @core.Vec3::zero(),
    @core.rotation_from_scaled_axis(@core.Vec3::new(angle, 0.0F, 0.0F)),
  )
  frame1.mul(free_axis_rotation).mul(frame2.inverse())
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

  let joint_positions : Array[@core.Vec3] = []
  let joint_axes : Array[@core.Vec3] = []
  let joint_cols : Array[Int] = []
  let root_position = root_pose.translation
  let mut current_pose = root_pose

  for i in 1..<branch.length() {
    let link = multibody.links[branch[i]]
    let joint_origin = current_pose.transform_point(link.joint.local_anchor1_3)
    joint_positions.push(joint_origin)
    joint_axes.push(current_pose.rotation.rotate_vec3(link.joint.axis3))
    joint_cols.push(mb3_joint_logical_column(multibody, link.assembly_id))

    let mut angle = link.joint_angle
    let disp_col = mb3_joint_logical_column(multibody, link.assembly_id)
    if displacements is Some(disp) && disp_col < disp.length() {
      angle = angle + disp[disp_col]
    }

    current_pose = current_pose.mul(
      mb3_revolute_body_to_parent(link.joint, angle),
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

    for i in 0..<joint_positions.length() {
      let joint_origin = joint_positions[i]
      let axis = joint_axes[i]
      let linear = axis.cross(link_position.sub(joint_origin))
      mb3_set_spatial_column(jacobian, joint_cols[i], linear, axis)
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
