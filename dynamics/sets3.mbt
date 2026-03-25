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
/// 3D-flavored island manager.
pub struct IslandManager3 {
  inner : IslandManager

  fn new() -> IslandManager3
}

///|
pub fn IslandManager3::new() -> IslandManager3 {
  { inner: IslandManager::new() }
}

///|
pub fn IslandManager3::as_2d(self : IslandManager3) -> IslandManager {
  self.inner
}

///|
pub fn IslandManager3::active_bodies(
  self : IslandManager3,
) -> Array[RigidBodyHandle] {
  self.inner.active_bodies()
}

///|
pub fn IslandManager3::wake_up(
  self : IslandManager3,
  bodies : RigidBodySet3,
  handle : RigidBodyHandle,
  strong : Bool,
) -> Unit {
  self.inner.wake_up(bodies.inner, handle, strong)
}

///|
pub fn IslandManager3::interaction_started_or_stopped(
  self : IslandManager3,
  bodies : RigidBodySet3,
  handle1 : RigidBodyHandle?,
  handle2 : RigidBodyHandle?,
  started : Bool,
  stop_on_same_island : Bool,
) -> Unit {
  self.inner.interaction_started_or_stopped(
    bodies.inner,
    handle1,
    handle2,
    started,
    stop_on_same_island,
  )
}

///|
pub fn IslandManager3::rigid_body_updated(
  self : IslandManager3,
  handle : RigidBodyHandle,
  bodies : RigidBodySet3,
) -> Unit {
  self.inner.rigid_body_updated(handle, bodies.inner)
}

///|
pub fn IslandManager3::rigid_body_removed_or_disabled(
  self : IslandManager3,
  handle : RigidBodyHandle,
  bodies : RigidBodySet3,
) -> Unit {
  self.inner.rigid_body_removed_or_disabled(handle, bodies.inner)
}

///|
fn IslandManager::push_linked_bodies3(
  self : IslandManager,
  bodies : RigidBodySet,
  impulse_joints : ImpulseJointSet3,
  multibody_joints : MultibodyJointSet3,
  handle : RigidBodyHandle,
) -> Unit {
  bodies |> ignore
  let attached = impulse_joints.attached_enabled_joints(handle)
  for i in 0..<attached.length() {
    let joint = attached[i]
    let other = if handle_equals(joint.0, handle) { joint.1 } else { joint.0 }
    self.stack.push(other)
  }
  let attached = multibody_joints.bodies_attached_with_enabled_joint(handle)
  for i in 0..<attached.length() {
    self.stack.push(attached[i])
  }
}

///|
fn IslandManager::extract_sleeping_island3(
  self : IslandManager,
  bodies : RigidBodySet,
  contacts : ContactGraph,
  impulse_joints : ImpulseJointSet3,
  multibody_joints : MultibodyJointSet3,
  sleep_root : RigidBodyHandle,
) -> Int {
  if sleep_root.id < 0 {
    return 0
  }
  if bodies.get(sleep_root) is None {
    return 0
  }
  if bodies.get_mut(sleep_root) is Some(root_body) {
    if !(root_body.activation.sleep_root_state
      is SleepRootState::TraversalPending) {
      return 0
    }
    root_body.activation.sleep_root_state = SleepRootState::Traversed
  } else {
    return 0
  }
  if self.island_id_for(sleep_root) is Some(island_id) {
    if self.islands[island_id] is Some(active_island) {
      if active_island.id_in_awake_list is None {
        return 0
      }
      let new_island_bodies : Array[RigidBodyHandle] = []
      self.stack.clear()
      self.stack.push(sleep_root)
      self.traversal_timestamp = self.traversal_timestamp + 1
      let mut visited_count = 0
      while self.stack.pop() is Some(handle) {
        if bodies.get_mut(handle) is Some(body) {
          if body.is_fixed() {
            continue
          }
          if body.ids.active_set_timestamp == self.traversal_timestamp {
            continue
          }
          body.ids.active_set_timestamp = self.traversal_timestamp
          visited_count = visited_count + 1
          if body.activation.is_eligible_for_sleep() {
            body.activation.sleep_root_state = SleepRootState::Traversed
          } else {
            self.stack.clear()
            return visited_count
          }
          self.push_contacting_bodies(bodies, contacts, handle)
          self.push_linked_bodies3(
            bodies, impulse_joints, multibody_joints, handle,
          )
          new_island_bodies.push(handle)
        }
      }
      if active_island.bodies.length() == new_island_bodies.length() {
        for i in 0..<active_island.bodies.length() {
          let handle = active_island.bodies[i]
          if bodies.get_mut(handle) is Some(body) {
            body.activation = body.activation.sleep()
            body.vels = RigidBodyVelocity::zero()
          }
        }
        self.remove_from_awake_list(island_id)
        self.islands[island_id] = Some(active_island)
      } else {
        visited_count = visited_count + new_island_bodies.length()
        self.extract_sub_island(bodies, island_id, new_island_bodies, true)
      }
      visited_count
    } else {
      0
    }
  } else {
    0
  }
}

///|
fn IslandManager::update_optimizer3(
  self : IslandManager,
  bodies : RigidBodySet,
  contacts : ContactGraph,
  impulse_joints : ImpulseJointSet3,
  multibody_joints : MultibodyJointSet3,
) -> Unit {
  if self.optimizer.mode % 2 == 0 {
    self.incremental_merge(bodies)
  } else {
    self.incremental_split3(bodies, contacts, impulse_joints, multibody_joints)
  }
  self.optimizer.mode = self.optimizer.mode + 1
}

///|
fn IslandManager::incremental_split3(
  self : IslandManager,
  bodies : RigidBodySet,
  contacts : ContactGraph,
  impulse_joints : ImpulseJointSet3,
  multibody_joints : MultibodyJointSet3,
) -> Unit {
  if self.optimizer.split_state.curr_awake_id >= self.awake_islands.length() {
    self.optimizer.split_state.curr_awake_id = 0
  }
  for
    awake_id in self.optimizer.split_state.curr_awake_id..<self.awake_islands.length() {
    let island_id = self.awake_islands[awake_id]
    if self.islands[island_id] is Some(island) {
      if island.bodies.length() > self.optimizer.max_island_size {
        self.stack.clear()
        let new_island_bodies : Array[RigidBodyHandle] = []
        self.traversal_timestamp = self.traversal_timestamp + 1
        for root_index in 0..<island.bodies.length() {
          self.stack.push(island.bodies[root_index])
          let len_before = new_island_bodies.length()
          while self.stack.pop() is Some(handle) {
            if bodies.get_mut(handle) is Some(body) {
              if body.is_fixed() {
                continue
              }
              if body.ids.active_set_timestamp == self.traversal_timestamp {
                continue
              }
              body.ids.active_set_timestamp = self.traversal_timestamp
              self.push_contacting_bodies(bodies, contacts, handle)
              self.push_linked_bodies3(
                bodies, impulse_joints, multibody_joints, handle,
              )
              new_island_bodies.push(handle)
              if new_island_bodies.length() > self.optimizer.max_island_size {
                truncate_handles(new_island_bodies, len_before)
                self.stack.clear()
                break
              }
            }
          }
          if new_island_bodies.length() == 0 {
            return
          } else if new_island_bodies.length() >= self.optimizer.min_island_size {
            self.extract_sub_island(bodies, island_id, new_island_bodies, false)
            return
          }
        }
      }
      self.optimizer.split_state.curr_awake_id = awake_id + 1
    }
  }
}

///|
pub fn IslandManager3::update_islands(
  self : IslandManager3,
  dt : @core.Real,
  length_unit : @core.Real,
  bodies : RigidBodySet3,
  contacts : ContactGraph,
  impulse_joints : ImpulseJointSet3,
  multibody_joints : MultibodyJointSet3,
) -> Unit {
  let bodies2 = bodies.inner
  for handle in self.inner.active_bodies() {
    if handle.id < 0 {
      continue
    }
    if bodies2.get_mut(handle) is Some(body) {
      self.inner.sync_body_state_generation(handle)
      let linvel = body.vels.linvel
      let sq_linvel = linvel.x * linvel.x + linvel.y * linvel.y
      let angvel = body.vels.angvel
      let sq_angvel = angvel * angvel
      body.activation = body.activation.update_energy(
        body.body_type,
        length_unit,
        sq_linvel,
        sq_angvel,
        dt,
      )
      let eligible = body.activation.is_eligible_for_sleep()
      if eligible {
        if body.activation.sleep_root_state is SleepRootState::Unknown {
          self.inner.traversal_candidates.push(SleepCandidate::{ handle, })
          body.activation.sleep_root_state = SleepRootState::TraversalPending
        }
      } else {
        body.activation.sleep_root_state = SleepRootState::Unknown
      }
    }
  }
  let mut cost = 0
  while true {
    if self.inner.pop_traversal_candidate() is Some(candidate) {
      cost = cost +
        self.inner.extract_sleeping_island3(
          bodies2,
          contacts,
          impulse_joints,
          multibody_joints,
          candidate.handle,
        )
      if cost > MAX_SLEEP_TRAVERSAL_COST {
        break
      }
    } else {
      break
    }
  }
  self.inner.update_optimizer3(
    bodies2, contacts, impulse_joints, multibody_joints,
  )
}

///|
/// 3D-flavored rigid-body set.
pub struct RigidBodySet3 {
  inner : RigidBodySet

  fn new() -> RigidBodySet3
}

///|
pub fn RigidBodySet3::new() -> RigidBodySet3 {
  { inner: RigidBodySet::new() }
}

///|
pub fn RigidBodySet3::as_2d(self : RigidBodySet3) -> RigidBodySet {
  self.inner
}

///|
pub fn RigidBodySet3::len(self : RigidBodySet3) -> Int {
  self.inner.len()
}

///|
pub fn RigidBodySet3::insert(
  self : RigidBodySet3,
  body : RigidBody,
) -> RigidBodyHandle {
  self.inner.insert(body)
}

///|
pub fn RigidBodySet3::remove(
  self : RigidBodySet3,
  handle : RigidBodyHandle,
  islands : IslandManager3,
  colliders : Unit,
  impulse_joints : ImpulseJointSet3,
  multibody_joints : MultibodyJointSet3,
  wake_up : Bool,
) -> RigidBody? {
  colliders |> ignore
  let empty_impulse_joints = ImpulseJointSet::new()
  let removed = self.inner.remove(
    handle,
    islands.inner,
    (),
    empty_impulse_joints,
    MultibodyJointSet::new(),
    wake_up,
  )
  if removed is Some(_) {
    impulse_joints.remove_joints_attached_to_rigid_body(handle) |> ignore
    multibody_joints.remove_joints_attached_to_rigid_body(handle)
  }
  removed
}

///|
pub fn RigidBodySet3::get(
  self : RigidBodySet3,
  handle : RigidBodyHandle,
) -> RigidBody? {
  self.inner.get(handle)
}

///|
pub fn RigidBodySet3::get_mut(
  self : RigidBodySet3,
  handle : RigidBodyHandle,
) -> RigidBody? {
  self.inner.get_mut(handle)
}

///|
pub fn RigidBodySet3::iter(
  self : RigidBodySet3,
) -> Array[(RigidBodyHandle, RigidBody)] {
  self.inner.iter()
}

///|
pub fn RigidBodySet3::iter_mut(
  self : RigidBodySet3,
) -> Array[(RigidBodyHandle, RigidBody)] {
  self.inner.iter_mut()
}

///|
pub fn RigidBodySet3::take_modified(
  self : RigidBodySet3,
) -> Array[RigidBodyHandle] {
  self.inner.take_modified()
}

///|
pub fn RigidBodySet3::translation(
  self : RigidBodySet3,
  handle : RigidBodyHandle,
) -> @core.Vec3 {
  if self.inner.get(handle) is Some(body) {
    let t2 = body.translation()
    @core.Vec3::new(t2.x, t2.y, 0.0F)
  } else {
    @core.Vec3::zero()
  }
}

///|
pub fn RigidBodySet3::rotation(
  self : RigidBodySet3,
  handle : RigidBodyHandle,
) -> @core.Quat {
  if self.inner.get(handle) is Some(body) {
    let r2 = body.rotation()
    // Map 2D angle to a quaternion around +Z.
    @core.rotation_from_scaled_axis(@core.Vec3::new(0.0F, 0.0F, r2.angle()))
  } else {
    @core.Quat::identity()
  }
}
