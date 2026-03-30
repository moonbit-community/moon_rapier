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
const MIN_ISLAND_SIZE_3D : Int = 1024

///|
const MAX_ISLAND_SIZE_3D : Int = 4096

///|
const SLEEP_STATE_UNKNOWN : Int = 0

///|
const SLEEP_STATE_PENDING : Int = 1

///|
const SLEEP_STATE_TRAVERSED : Int = 2

///|
struct Island3D {
  bodies : Array[RigidBodyHandle]
  mut id_in_awake_list : Int?
  mut additional_solver_iterations : Int
}

///|
struct IslandsOptimizer3D {
  mut min_island_size : Int
  mut max_island_size : Int
  mut mode : Int
  mut merge_curr_awake_id : Int
  mut split_curr_awake_id : Int
}

///|
fn IslandsOptimizer3D::new() -> IslandsOptimizer3D {
  {
    min_island_size: MIN_ISLAND_SIZE_3D,
    max_island_size: MAX_ISLAND_SIZE_3D,
    mode: 0,
    merge_curr_awake_id: 0,
    split_curr_awake_id: 0,
  }
}

///|
fn handle_equals3d(a : RigidBodyHandle, b : RigidBodyHandle) -> Bool {
  a.id == b.id && a.generation == b.generation
}

///|
fn max_non_negative(value : Int) -> Int {
  if value > 0 {
    value
  } else {
    0
  }
}

///|
fn truncate_handles3d(values : Array[RigidBodyHandle], len : Int) -> Unit {
  while values.length() > len {
    values.pop() |> ignore
  }
}

///|
/// Dim3 island manager aligned with Rapier's persistent island model:
/// - stable island ids
/// - awake island list
/// - incremental merge/split optimizer
/// - sleeping component extraction on the interaction graph
///
/// Public query methods (`active_islands`, `island_bodies`, `island_additional_solver_iterations`)
/// expose island-scoped data used by the solver pipeline.
pub struct IslandManager3D {
  // Public snapshots preserved for compatibility.
  active_bodies : Array[RigidBodyHandle]
  islands : Array[Array[RigidBodyHandle]]
  island_solver_iterations : Array[Int]
  // Internal persistent island storage.
  island_nodes : Array[Island3D?]
  awake_island_ids : Array[Int]
  free_island_ids : Array[Int]
  body_island_ids : Array[Int]
  body_island_generations : Array[Int]
  body_island_indices : Array[Int]
  body_sleep_state_generations : Array[Int]
  body_sleep_states : Array[Int]
  body_traversal_timestamps : Array[Int]
  stack : Array[RigidBodyHandle]
  optimizer : IslandsOptimizer3D
  mut traversal_timestamp : Int

  fn new() -> IslandManager3D
}

///|
pub fn IslandManager3D::new() -> IslandManager3D {
  let manager = {
    active_bodies: [],
    islands: [],
    island_solver_iterations: [],
    island_nodes: [],
    awake_island_ids: [],
    free_island_ids: [],
    body_island_ids: [],
    body_island_generations: [],
    body_island_indices: [],
    body_sleep_state_generations: [],
    body_sleep_states: [],
    body_traversal_timestamps: [],
    stack: [],
    optimizer: IslandsOptimizer3D::new(),
    traversal_timestamp: 0,
  }
  manager.set_island_size_limits(MIN_ISLAND_SIZE_3D, MAX_ISLAND_SIZE_3D)
  manager
}

///|
fn IslandManager3D::set_island_size_limits(
  self : IslandManager3D,
  min_island_size : Int,
  max_island_size : Int,
) -> Unit {
  let min_size = if min_island_size > 0 { min_island_size } else { 1 }
  let max_size = if max_island_size > min_size {
    max_island_size
  } else {
    min_size + 1
  }
  self.optimizer.min_island_size = min_size
  self.optimizer.max_island_size = max_size
}

///|
fn IslandManager3D::ensure_body_capacity(
  self : IslandManager3D,
  id : Int,
) -> Unit {
  if id < 0 {
    return
  }
  let mut i = self.body_island_ids.length()
  while i <= id {
    self.body_island_ids.push(-1)
    self.body_island_generations.push(0)
    self.body_island_indices.push(-1)
    self.body_sleep_state_generations.push(0)
    self.body_sleep_states.push(SLEEP_STATE_UNKNOWN)
    self.body_traversal_timestamps.push(0)
    i = i + 1
  }
}

///|
fn IslandManager3D::reset_body_state_for_generation(
  self : IslandManager3D,
  handle : RigidBodyHandle,
) -> Unit {
  if handle.id < 0 {
    return
  }
  self.ensure_body_capacity(handle.id)
  self.body_island_ids[handle.id] = -1
  self.body_island_generations[handle.id] = handle.generation
  self.body_island_indices[handle.id] = -1
  self.body_sleep_state_generations[handle.id] = handle.generation
  self.body_sleep_states[handle.id] = SLEEP_STATE_UNKNOWN
  self.body_traversal_timestamps[handle.id] = 0
}

///|
fn IslandManager3D::sync_body_generation(
  self : IslandManager3D,
  handle : RigidBodyHandle,
) -> Unit {
  if handle.id < 0 {
    return
  }
  self.ensure_body_capacity(handle.id)
  if self.body_island_generations[handle.id] != handle.generation {
    self.reset_body_state_for_generation(handle)
  }
}

///|
fn IslandManager3D::island_id_for(
  self : IslandManager3D,
  handle : RigidBodyHandle,
) -> Int? {
  if handle.id < 0 || handle.id >= self.body_island_ids.length() {
    return None
  }
  if self.body_island_generations[handle.id] != handle.generation {
    return None
  }
  let island_id = self.body_island_ids[handle.id]
  if island_id < 0 || island_id >= self.island_nodes.length() {
    return None
  }
  if self.island_nodes[island_id] is Some(_) {
    Some(island_id)
  } else {
    None
  }
}

///|
fn IslandManager3D::insert_island_node(
  self : IslandManager3D,
  island : Island3D,
) -> Int {
  if self.free_island_ids.pop() is Some(id) {
    self.island_nodes[id] = Some(island)
    id
  } else {
    let id = self.island_nodes.length()
    self.island_nodes.push(Some(island))
    id
  }
}

///|
fn IslandManager3D::remove_from_awake_list(
  self : IslandManager3D,
  island_id : Int,
) -> Unit {
  if island_id < 0 || island_id >= self.island_nodes.length() {
    return
  }
  if self.island_nodes[island_id] is Some(island) {
    if island.id_in_awake_list is Some(awake_index) {
      let last_index = self.awake_island_ids.length() - 1
      if awake_index != last_index {
        let moved_id = self.awake_island_ids[last_index]
        self.awake_island_ids[awake_index] = moved_id
        if self.island_nodes[moved_id] is Some(moved) {
          moved.id_in_awake_list = Some(awake_index)
          self.island_nodes[moved_id] = Some(moved)
        }
      }
      self.awake_island_ids.pop() |> ignore
      island.id_in_awake_list = None
      self.island_nodes[island_id] = Some(island)
    }
  }
}

///|
fn IslandManager3D::wake_up_body(
  self : IslandManager3D,
  bodies : RigidBodySet3D,
  handle : RigidBodyHandle,
) -> Unit {
  if handle.id < 0 {
    return
  }
  self.sync_body_generation(handle)
  if bodies.get_mut(handle) is Some(body) {
    if body.body_type().is_fixed() {
      return
    }
    if body.is_sleeping() {
      body.wake_up()
    }
  }
}

///|
fn IslandManager3D::wake_up_island(
  self : IslandManager3D,
  bodies : RigidBodySet3D,
  island_id : Int,
) -> Unit {
  if island_id < 0 || island_id >= self.island_nodes.length() {
    return
  }
  if self.island_nodes[island_id] is Some(island) {
    if island.id_in_awake_list is None {
      island.id_in_awake_list = Some(self.awake_island_ids.length())
      self.awake_island_ids.push(island_id)
    }
    for i in 0..<island.bodies.length() {
      self.wake_up_body(bodies, island.bodies[i])
    }
    self.island_nodes[island_id] = Some(island)
  }
}

///|
fn IslandManager3D::insert_body(
  self : IslandManager3D,
  bodies : RigidBodySet3D,
  handle : RigidBodyHandle,
  sleeping : Bool,
) -> Unit {
  self.reset_body_state_for_generation(handle)
  let mut additional_solver_iterations = 0
  if bodies.get(handle) is Some(body) {
    additional_solver_iterations = max_non_negative(
      body.additional_solver_iterations(),
    )
  }
  let island : Island3D = {
    bodies: [handle],
    id_in_awake_list: None,
    additional_solver_iterations,
  }
  if !sleeping {
    island.id_in_awake_list = Some(self.awake_island_ids.length())
  }
  let island_id = self.insert_island_node(island)
  if !sleeping {
    self.awake_island_ids.push(island_id)
  }
  self.ensure_body_capacity(handle.id)
  self.body_island_ids[handle.id] = island_id
  self.body_island_generations[handle.id] = handle.generation
  self.body_island_indices[handle.id] = 0
}

///|
fn IslandManager3D::remove_empty_island(
  self : IslandManager3D,
  island_id : Int,
) -> Unit {
  if island_id < 0 || island_id >= self.island_nodes.length() {
    return
  }
  self.remove_from_awake_list(island_id)
  self.island_nodes[island_id] = None
  self.free_island_ids.push(island_id)
}

///|
fn IslandManager3D::cleanup_invalid_bodies(
  self : IslandManager3D,
  bodies : RigidBodySet3D,
) -> Unit {
  for island_id in 0..<self.island_nodes.length() {
    if self.island_nodes[island_id] is Some(island) {
      let old = island.bodies.copy()
      island.bodies.clear()
      let mut max_additional = 0
      for i in 0..<old.length() {
        let handle = old[i]
        self.sync_body_generation(handle)
        if handle.id >= 0 &&
          handle.id < self.body_island_ids.length() &&
          self.body_island_generations[handle.id] == handle.generation {
          self.body_island_ids[handle.id] = -1
          self.body_island_indices[handle.id] = -1
        }
        if bodies.get(handle) is Some(body) && !body.body_type().is_fixed() {
          let idx = island.bodies.length()
          island.bodies.push(handle)
          if handle.id >= 0 && handle.id < self.body_island_ids.length() {
            self.body_island_ids[handle.id] = island_id
            self.body_island_generations[handle.id] = handle.generation
            self.body_island_indices[handle.id] = idx
          }
          let add_iters = max_non_negative(body.additional_solver_iterations())
          if add_iters > max_additional {
            max_additional = add_iters
          }
        }
      }
      if island.bodies.length() == 0 {
        self.remove_empty_island(island_id)
      } else {
        island.additional_solver_iterations = max_additional
        self.island_nodes[island_id] = Some(island)
      }
    }
  }
}

///|
fn IslandManager3D::merge_islands(
  self : IslandManager3D,
  bodies : RigidBodySet3D,
  island_id1 : Int,
  island_id2 : Int,
) -> Unit {
  if island_id1 == island_id2 {
    return
  }
  if island_id1 < 0 || island_id2 < 0 {
    return
  }
  if island_id1 >= self.island_nodes.length() ||
    island_id2 >= self.island_nodes.length() {
    return
  }
  if self.island_nodes[island_id1] is Some(island1) &&
    self.island_nodes[island_id2] is Some(island2) {
    let sleeping1 = island1.id_in_awake_list is None
    let sleeping2 = island2.id_in_awake_list is None
    if sleeping1 != sleeping2 {
      if sleeping1 {
        self.wake_up_island(bodies, island_id1)
      } else {
        self.wake_up_island(bodies, island_id2)
      }
    }

    let (keep_id, remove_id) = if island1.bodies.length() <
      island2.bodies.length() {
      (island_id2, island_id1)
    } else {
      (island_id1, island_id2)
    }
    if self.island_nodes[keep_id] is Some(keep) &&
      self.island_nodes[remove_id] is Some(remove) {
      let keep_sleeping = keep.id_in_awake_list is None
      for i in 0..<remove.bodies.length() {
        let handle = remove.bodies[i]
        self.sync_body_generation(handle)
        if !keep_sleeping {
          self.wake_up_body(bodies, handle)
        }
        let idx = keep.bodies.length()
        keep.bodies.push(handle)
        if handle.id >= 0 && handle.id < self.body_island_ids.length() {
          self.body_island_ids[handle.id] = keep_id
          self.body_island_generations[handle.id] = handle.generation
          self.body_island_indices[handle.id] = idx
        }
        if bodies.get(handle) is Some(body) {
          let add_iters = max_non_negative(body.additional_solver_iterations())
          if add_iters > keep.additional_solver_iterations {
            keep.additional_solver_iterations = add_iters
          }
        }
      }

      self.remove_from_awake_list(remove_id)
      self.island_nodes[remove_id] = None
      self.free_island_ids.push(remove_id)
      self.island_nodes[keep_id] = Some(keep)
    }
  }
}

///|
fn IslandManager3D::push_contacting_bodies(
  self : IslandManager3D,
  interactions : Array[(RigidBodyHandle, RigidBodyHandle)],
  handle : RigidBodyHandle,
) -> Unit {
  for i in 0..<interactions.length() {
    let (h1, h2) = interactions[i]
    if handle_equals3d(h1, handle) {
      self.stack.push(h2)
    } else if handle_equals3d(h2, handle) {
      self.stack.push(h1)
    }
  }
}

///|
fn IslandManager3D::extract_sub_island(
  self : IslandManager3D,
  bodies : RigidBodySet3D,
  source_island_id : Int,
  extracted : Array[RigidBodyHandle],
  sleeping : Bool,
) -> Unit {
  if source_island_id < 0 || source_island_id >= self.island_nodes.length() {
    return
  }
  if extracted.length() == 0 {
    return
  }
  if self.island_nodes[source_island_id] is Some(source) {
    fn contains_handle(
      handles : Array[RigidBodyHandle],
      target : RigidBodyHandle,
    ) -> Bool {
      for i in 0..<handles.length() {
        if handle_equals3d(handles[i], target) {
          return true
        }
      }
      false
    }

    let source_old = source.bodies.copy()
    let source_kept : Array[RigidBodyHandle] = []
    let moved : Array[RigidBodyHandle] = []
    for i in 0..<source_old.length() {
      let handle = source_old[i]
      if contains_handle(extracted, handle) {
        moved.push(handle)
      } else {
        source_kept.push(handle)
      }
    }

    let new_island : Island3D = {
      bodies: [],
      id_in_awake_list: None,
      additional_solver_iterations: 0,
    }
    let new_island_id = self.insert_island_node(new_island)
    let new_node = if self.island_nodes[new_island_id] is Some(n) {
      n
    } else {
      return
    }

    source.bodies.clear()
    source.additional_solver_iterations = 0
    for i in 0..<source_kept.length() {
      let handle = source_kept[i]
      source.bodies.push(handle)
      if handle.id >= 0 && handle.id < self.body_island_ids.length() {
        self.body_island_ids[handle.id] = source_island_id
        self.body_island_generations[handle.id] = handle.generation
        self.body_island_indices[handle.id] = i
      }
      if bodies.get(handle) is Some(body) {
        let add_iters = max_non_negative(body.additional_solver_iterations())
        if add_iters > source.additional_solver_iterations {
          source.additional_solver_iterations = add_iters
        }
      }
    }

    new_node.bodies.clear()
    new_node.additional_solver_iterations = 0
    for i in 0..<moved.length() {
      let handle = moved[i]
      new_node.bodies.push(handle)
      if handle.id >= 0 && handle.id < self.body_island_ids.length() {
        self.body_island_ids[handle.id] = new_island_id
        self.body_island_generations[handle.id] = handle.generation
        self.body_island_indices[handle.id] = i
      }
      if bodies.get_mut(handle) is Some(body) {
        if sleeping {
          body.set_linvel(@core.Vec3::zero())
          body.set_angvel(@core.Vec3::zero())
          body.activation.sleeping = true
          body.activation.time_since_can_sleep = body.activation.time_until_sleep
        }
        let add_iters = max_non_negative(body.additional_solver_iterations())
        if add_iters > new_node.additional_solver_iterations {
          new_node.additional_solver_iterations = add_iters
        }
      }
    }

    if source.bodies.length() == 0 {
      self.remove_empty_island(source_island_id)
    } else {
      self.island_nodes[source_island_id] = Some(source)
    }

    if sleeping {
      new_node.id_in_awake_list = None
    } else {
      new_node.id_in_awake_list = Some(self.awake_island_ids.length())
      self.awake_island_ids.push(new_island_id)
    }
    self.island_nodes[new_island_id] = Some(new_node)
  }
}

///|
fn IslandManager3D::extract_sleeping_components(
  self : IslandManager3D,
  bodies : RigidBodySet3D,
  interactions : Array[(RigidBodyHandle, RigidBodyHandle)],
) -> Unit {
  let mut changed = true
  while changed {
    changed = false
    for awake_i in 0..<self.awake_island_ids.length() {
      let island_id = self.awake_island_ids[awake_i]
      if island_id < 0 || island_id >= self.island_nodes.length() {
        continue
      }
      if self.island_nodes[island_id] is Some(island) {
        let island_bodies = island.bodies.copy()
        for root_i in 0..<island_bodies.length() {
          let root = island_bodies[root_i]
          self.sync_body_generation(root)
          if root.id < 0 || root.id >= self.body_sleep_states.length() {
            continue
          }
          let sleep_state = self.body_sleep_states[root.id]
          if sleep_state != SLEEP_STATE_PENDING &&
            sleep_state != SLEEP_STATE_UNKNOWN {
            continue
          }
          if bodies.get(root) is Some(root_body) {
            if root_body.body_type().is_fixed() || !root_body.is_sleeping() {
              self.body_sleep_states[root.id] = SLEEP_STATE_UNKNOWN
              continue
            }
          } else {
            continue
          }

          self.traversal_timestamp = self.traversal_timestamp + 1
          let stamp = self.traversal_timestamp
          self.stack.clear()
          self.stack.push(root)
          let extracted : Array[RigidBodyHandle] = []
          let mut can_extract = true
          while self.stack.pop() is Some(handle) {
            self.sync_body_generation(handle)
            if handle.id < 0 ||
              handle.id >= self.body_traversal_timestamps.length() {
              continue
            }
            if self.body_traversal_timestamps[handle.id] == stamp {
              continue
            }
            if self.island_id_for(handle) is Some(owner) {
              if owner != island_id {
                continue
              }
            } else {
              continue
            }
            self.body_traversal_timestamps[handle.id] = stamp
            if bodies.get(handle) is Some(body) {
              if body.body_type().is_fixed() {
                continue
              }
              if !body.is_sleeping() {
                can_extract = false
                self.body_sleep_states[handle.id] = SLEEP_STATE_UNKNOWN
                self.stack.clear()
                break
              }
              self.body_sleep_states[handle.id] = SLEEP_STATE_TRAVERSED
              extracted.push(handle)
              self.push_contacting_bodies(interactions, handle)
            }
          }

          if can_extract && extracted.length() > 0 {
            if extracted.length() == island_bodies.length() {
              self.remove_from_awake_list(island_id)
            } else {
              self.extract_sub_island(bodies, island_id, extracted, true)
            }
            changed = true
            break
          }
        }
      }
      if changed {
        break
      }
    }
  }
}

///|
fn IslandManager3D::incremental_merge(
  self : IslandManager3D,
  bodies : RigidBodySet3D,
) -> Unit {
  if self.optimizer.merge_curr_awake_id >= self.awake_island_ids.length() {
    self.optimizer.merge_curr_awake_id = 0
  }
  let mut first_awake_id = -1
  let mut first_island_id = -1
  let mut first_solver_iterations = 0
  for
    awake_id in self.optimizer.merge_curr_awake_id..<self.awake_island_ids.length() {
    let island_id = self.awake_island_ids[awake_id]
    if self.island_nodes[island_id] is Some(island) {
      if island.bodies.length() < self.optimizer.min_island_size {
        first_awake_id = awake_id
        first_island_id = island_id
        first_solver_iterations = island.additional_solver_iterations
        break
      }
    }
  }
  if first_awake_id < 0 {
    self.optimizer.merge_curr_awake_id = 0
    return
  }
  self.optimizer.merge_curr_awake_id = first_awake_id + 1
  let mut found_next = false
  for awake_id in (first_awake_id + 1)..<self.awake_island_ids.length() {
    let island_id = self.awake_island_ids[awake_id]
    if self.island_nodes[island_id] is Some(island) {
      if island.bodies.length() < self.optimizer.min_island_size {
        if island.additional_solver_iterations == first_solver_iterations {
          self.merge_islands(bodies, first_island_id, island_id)
          return
        } else if !found_next {
          self.optimizer.merge_curr_awake_id = awake_id
          found_next = true
        }
      }
    }
  }
}

///|
fn IslandManager3D::incremental_split(
  self : IslandManager3D,
  bodies : RigidBodySet3D,
  interactions : Array[(RigidBodyHandle, RigidBodyHandle)],
) -> Unit {
  if self.optimizer.split_curr_awake_id >= self.awake_island_ids.length() {
    self.optimizer.split_curr_awake_id = 0
  }
  for
    awake_id in self.optimizer.split_curr_awake_id..<self.awake_island_ids.length() {
    let island_id = self.awake_island_ids[awake_id]
    if self.island_nodes[island_id] is Some(island) {
      if island.bodies.length() > self.optimizer.max_island_size {
        self.stack.clear()
        let extracted : Array[RigidBodyHandle] = []
        self.traversal_timestamp = self.traversal_timestamp + 1
        let stamp = self.traversal_timestamp
        for root_i in 0..<island.bodies.length() {
          self.stack.push(island.bodies[root_i])
          let len_before = extracted.length()
          while self.stack.pop() is Some(handle) {
            self.sync_body_generation(handle)
            if handle.id < 0 ||
              handle.id >= self.body_traversal_timestamps.length() {
              continue
            }
            if self.body_traversal_timestamps[handle.id] == stamp {
              continue
            }
            self.body_traversal_timestamps[handle.id] = stamp
            if self.island_id_for(handle) is Some(owner) {
              if owner != island_id {
                continue
              }
            } else {
              continue
            }
            if bodies.get(handle) is Some(body) {
              if body.body_type().is_fixed() {
                continue
              }
              self.push_contacting_bodies(interactions, handle)
              extracted.push(handle)
              if extracted.length() > self.optimizer.max_island_size {
                truncate_handles3d(extracted, len_before)
                self.stack.clear()
                break
              }
            }
          }
          if extracted.length() == 0 {
            return
          } else if extracted.length() >= self.optimizer.min_island_size {
            self.extract_sub_island(bodies, island_id, extracted, false)
            return
          }
        }
      }
    }
  }
}

///|
fn IslandManager3D::update_optimizer(
  self : IslandManager3D,
  bodies : RigidBodySet3D,
  interactions : Array[(RigidBodyHandle, RigidBodyHandle)],
) -> Unit {
  if self.optimizer.mode % 2 == 0 {
    self.incremental_merge(bodies)
  } else {
    self.incremental_split(bodies, interactions)
  }
  self.optimizer.mode = self.optimizer.mode + 1
}

///|
fn IslandManager3D::rebuild_public_views(self : IslandManager3D) -> Unit {
  self.active_bodies.clear()
  self.islands.clear()
  self.island_solver_iterations.clear()
  for i in 0..<self.awake_island_ids.length() {
    let island_id = self.awake_island_ids[i]
    if island_id < 0 || island_id >= self.island_nodes.length() {
      continue
    }
    if self.island_nodes[island_id] is Some(island) {
      let island_copy : Array[RigidBodyHandle] = []
      for j in 0..<island.bodies.length() {
        let handle = island.bodies[j]
        island_copy.push(handle)
        self.active_bodies.push(handle)
      }
      self.islands.push(island_copy)
      self.island_solver_iterations.push(
        max_non_negative(island.additional_solver_iterations),
      )
    }
  }
}

///|
pub fn IslandManager3D::active_bodies(
  self : IslandManager3D,
) -> Array[RigidBodyHandle] {
  self.active_bodies
}

///|
pub fn IslandManager3D::active_islands(self : IslandManager3D) -> Array[Int] {
  let out : Array[Int] = []
  for i in 0..<self.awake_island_ids.length() {
    out.push(self.awake_island_ids[i])
  }
  out
}

///|
pub fn IslandManager3D::island_bodies(
  self : IslandManager3D,
  island_id : Int,
) -> Array[RigidBodyHandle] {
  if island_id < 0 || island_id >= self.island_nodes.length() {
    return []
  }
  if self.island_nodes[island_id] is Some(island) {
    let out : Array[RigidBodyHandle] = []
    for i in 0..<island.bodies.length() {
      out.push(island.bodies[i])
    }
    out
  } else {
    []
  }
}

///|
pub fn IslandManager3D::island_additional_solver_iterations(
  self : IslandManager3D,
  island_id : Int,
) -> Int {
  if island_id < 0 || island_id >= self.island_nodes.length() {
    return 0
  }
  if self.island_nodes[island_id] is Some(island) {
    max_non_negative(island.additional_solver_iterations)
  } else {
    0
  }
}

///|
pub fn IslandManager3D::update(
  self : IslandManager3D,
  bodies : RigidBodySet3D,
  interactions : Array[(RigidBodyHandle, RigidBodyHandle)],
) -> Unit {
  // Ensure each dynamic/kinematic body belongs to one persistent island.
  for id in 0..<bodies.bodies.length() {
    if bodies.bodies[id] is Some(body) {
      if body.body_type().is_fixed() {
        continue
      }
      let handle = RigidBodyHandle::from_raw_parts(id, bodies.generations[id])
      self.sync_body_generation(handle)
      if self.island_id_for(handle) is None {
        self.insert_body(bodies, handle, body.is_sleeping())
      } else if body.is_active() {
        if self.island_id_for(handle) is Some(island_id) {
          self.wake_up_island(bodies, island_id)
        }
      }
      if body.is_sleeping() {
        if handle.id >= 0 && handle.id < self.body_sleep_states.length() {
          if self.body_sleep_states[handle.id] == SLEEP_STATE_UNKNOWN {
            self.body_sleep_states[handle.id] = SLEEP_STATE_PENDING
          }
        }
      } else if handle.id >= 0 && handle.id < self.body_sleep_states.length() {
        self.body_sleep_states[handle.id] = SLEEP_STATE_UNKNOWN
      }
    }
  }

  self.cleanup_invalid_bodies(bodies)

  // Merge/wake through current interaction graph.
  for i in 0..<interactions.length() {
    let (h1, h2) = interactions[i]
    self.sync_body_generation(h1)
    self.sync_body_generation(h2)
    if bodies.get(h1) is Some(b1) && bodies.get(h2) is Some(b2) {
      if b1.body_type().is_fixed() || b2.body_type().is_fixed() {
        continue
      }
      if b1.is_active() && b2.is_sleeping() {
        self.wake_up_body(bodies, h2)
      } else if b2.is_active() && b1.is_sleeping() {
        self.wake_up_body(bodies, h1)
      }
      if self.island_id_for(h1) is None {
        self.insert_body(bodies, h1, b1.is_sleeping())
      }
      if self.island_id_for(h2) is None {
        self.insert_body(bodies, h2, b2.is_sleeping())
      }
      if self.island_id_for(h1) is Some(id1) &&
        self.island_id_for(h2) is Some(id2) &&
        id1 != id2 {
        self.merge_islands(bodies, id1, id2)
      }
    }
  }

  // Extract sleeping components from awake islands and keep awake-list coherent.
  self.extract_sleeping_components(bodies, interactions)

  // Incremental optimizer (merge/split) like upstream island manager.
  self.update_optimizer(bodies, interactions)

  self.rebuild_public_views()
}
