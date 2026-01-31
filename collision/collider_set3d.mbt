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
