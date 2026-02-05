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
/// Dim3 island manager.
///
/// Rapier's island manager builds connected components of the interaction graph.
/// The real 3D backend will refine this further, but for now we maintain a list
/// of awake bodies that can be used by the pipeline.
pub struct IslandManager3D {
  active_bodies : Array[RigidBodyHandle]
}

///|
pub fn IslandManager3D::new() -> IslandManager3D {
  { active_bodies: [] }
}

///|
pub fn IslandManager3D::active_bodies(
  self : IslandManager3D,
) -> Array[RigidBodyHandle] {
  self.active_bodies
}

///|
pub fn IslandManager3D::update(
  self : IslandManager3D,
  bodies : RigidBodySet3D,
  interactions : Array[(RigidBodyHandle, RigidBodyHandle)],
) -> Unit {
  let n = bodies.bodies.length()
  let parent : Array[Int] = []
  let size : Array[Int] = []
  for _ in 0..<n {
    parent.push(-1)
    size.push(1)
  }
  for i in 0..<n {
    if bodies.bodies[i] is Some(body) && !body.body_type().is_fixed() {
      parent[i] = i
    }
  }
  fn uf_find(parent : Array[Int], x : Int) -> Int {
    let mut x = x
    while parent[x] != x {
      let p = parent[x]
      let gp = parent[p]
      if gp >= 0 {
        parent[x] = gp
      }
      x = p
    }
    x
  }

  fn uf_union(
    parent : Array[Int],
    size : Array[Int],
    a : Int,
    b : Int,
  ) -> Unit {
    let ra = uf_find(parent, a)
    let rb = uf_find(parent, b)
    if ra == rb {
      return
    }
    if size[ra] < size[rb] {
      parent[ra] = rb
      size[rb] = size[rb] + size[ra]
    } else {
      parent[rb] = ra
      size[ra] = size[ra] + size[rb]
    }
  }

  for i in 0..<interactions.length() {
    let (a, b) = interactions[i]
    let id1 = a.id
    let id2 = b.id
    if id1 < 0 || id2 < 0 || id1 >= n || id2 >= n {
      continue
    }
    if parent[id1] < 0 || parent[id2] < 0 {
      continue
    }
    if bodies.generations[id1] != a.generation ||
      bodies.generations[id2] != b.generation {
      continue
    }
    uf_union(parent, size, id1, id2)
  }
  let comp_active : Array[Bool] = []
  for _ in 0..<n {
    comp_active.push(false)
  }
  for i in 0..<n {
    if parent[i] >= 0 && bodies.bodies[i] is Some(body) {
      if body.is_active() {
        let root = uf_find(parent, i)
        comp_active[root] = true
      }
    }
  }
  for i in 0..<n {
    if parent[i] >= 0 && bodies.bodies[i] is Some(body) {
      let root = uf_find(parent, i)
      if comp_active[root] && body.is_sleeping() {
        body.wake_up()
      }
    }
  }
  self.active_bodies.clear()
  for i in 0..<n {
    if bodies.bodies[i] is Some(body) && body.is_active() {
      self.active_bodies.push(
        RigidBodyHandle::from_raw_parts(i, bodies.generations[i]),
      )
    }
  }
}
