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
pub struct ColliderHandle3D {
  id : Int
  generation : Int
}

///|
fn ColliderHandle3D::new(id : Int, generation : Int) -> ColliderHandle3D {
  ColliderHandle3D::{ id, generation }
}

///|
pub fn ColliderHandle3D::equals(
  self : ColliderHandle3D,
  other : ColliderHandle3D,
) -> Bool {
  self.id == other.id && self.generation == other.generation
}

///|
pub fn ColliderHandle3D::from_raw_parts(
  id : Int,
  generation : Int,
) -> ColliderHandle3D {
  ColliderHandle3D::new(id, generation)
}

///|
pub fn ColliderHandle3D::into_raw_parts(self : ColliderHandle3D) -> (Int, Int) {
  (self.id, self.generation)
}

///|
pub fn ColliderHandle3D::invalid() -> ColliderHandle3D {
  ColliderHandle3D::new(-1, -1)
}

///|
pub struct ColliderSet3D {
  colliders : Array[Collider3D?]
  generations : Array[Int]
  free_list : Array[Int]
}

///|
pub fn ColliderSet3D::new() -> ColliderSet3D {
  { colliders: [], generations: [], free_list: [] }
}

///|
pub fn ColliderSet3D::len(self : ColliderSet3D) -> Int {
  let mut count = 0
  for i in 0..<self.colliders.length() {
    if self.colliders[i] is Some(_) {
      count = count + 1
    }
  }
  count
}

///|
pub fn ColliderSet3D::insert(
  self : ColliderSet3D,
  collider : Collider3D,
) -> ColliderHandle3D {
  collider.set_parent(None)
  collider.set_local_position(@core.Isometry3::identity())
  self.insert_raw(collider)
}

///|
fn ColliderSet3D::insert_raw(
  self : ColliderSet3D,
  collider : Collider3D,
) -> ColliderHandle3D {
  if self.free_list.pop() is Some(index) {
    self.colliders[index] = Some(collider)
    let generation = self.generations[index]
    ColliderHandle3D::new(index, generation)
  } else {
    let index = self.colliders.length()
    self.colliders.push(Some(collider))
    self.generations.push(0)
    ColliderHandle3D::new(index, 0)
  }
}

///|
pub fn ColliderSet3D::insert_with_parent(
  self : ColliderSet3D,
  collider : Collider3D,
  parent : @dynamics.RigidBodyHandle,
  bodies : @dynamics.RigidBodySet3D,
) -> ColliderHandle3D {
  if bodies.get(parent) is Some(body) {
    // Interpret `collider.position` as a transform local to the parent (Rapier-style).
    let local_pos = collider.position()
    collider.set_local_position(local_pos)
    collider.set_parent(Some(parent))
    collider.set_position(body.position().mul(local_pos))
  } else {
    collider.set_parent(Some(parent))
    collider.set_local_position(collider.position())
  }
  self.insert_raw(collider)
}

///|
pub fn ColliderSet3D::set_parent(
  self : ColliderSet3D,
  handle : ColliderHandle3D,
  parent : @dynamics.RigidBodyHandle?,
  bodies : @dynamics.RigidBodySet3D,
) -> Unit {
  if self.get_mut(handle) is Some(co) {
    match parent {
      None => {
        co.set_parent(None)
        co.set_local_position(@core.Isometry3::identity())
      }
      Some(p) =>
        if bodies.get(p) is Some(body) {
          let world = co.position()
          let local_pos = body.position().inverse().mul(world)
          co.set_local_position(local_pos)
          co.set_parent(Some(p))
          co.set_position(body.position().mul(local_pos))
        } else {
          co.set_parent(Some(p))
        }
    }
  }
}

///|
pub fn ColliderSet3D::sync_with_bodies(
  self : ColliderSet3D,
  bodies : @dynamics.RigidBodySet3D,
) -> Unit {
  for i in 0..<self.colliders.length() {
    if self.colliders[i] is Some(co) {
      if co.parent() is Some(p) {
        if bodies.get(p) is Some(body) {
          co.set_position(body.position().mul(co.local_position()))
        }
      }
    }
  }
}

///|
pub fn ColliderSet3D::remove(
  self : ColliderSet3D,
  handle : ColliderHandle3D,
) -> Collider3D? {
  let (id, gen) = handle.into_raw_parts()
  if id < 0 || id >= self.colliders.length() {
    return None
  }
  if self.generations[id] != gen {
    return None
  }
  let removed = self.colliders[id]
  self.colliders[id] = None
  self.free_list.push(id)
  self.generations[id] = self.generations[id] + 1
  removed
}

///|
pub fn ColliderSet3D::get(
  self : ColliderSet3D,
  handle : ColliderHandle3D,
) -> Collider3D? {
  let (id, gen) = handle.into_raw_parts()
  if id < 0 || id >= self.colliders.length() {
    return None
  }
  if self.generations[id] != gen {
    return None
  }
  self.colliders[id]
}

///|
pub fn ColliderSet3D::get_mut(
  self : ColliderSet3D,
  handle : ColliderHandle3D,
) -> Collider3D? {
  self.get(handle)
}
