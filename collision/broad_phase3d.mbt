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
pub struct ColliderPair3D {
  collider1 : ColliderHandle3D
  collider2 : ColliderHandle3D
}

///|
pub fn ColliderPair3D::new(
  collider1 : ColliderHandle3D,
  collider2 : ColliderHandle3D,
) -> ColliderPair3D {
  { collider1, collider2 }
}

///|
pub fn ColliderPair3D::collider1(self : ColliderPair3D) -> ColliderHandle3D {
  self.collider1
}

///|
pub fn ColliderPair3D::collider2(self : ColliderPair3D) -> ColliderHandle3D {
  self.collider2
}

///|
pub(all) enum BroadPhasePairEvent3D {
  AddPair(ColliderPair3D)
  DeletePair(ColliderPair3D)
}

///|
pub struct BroadPhase3D {
  pairs : Array[(ColliderHandle3D, ColliderHandle3D)]
  prev_pairs : Array[(ColliderHandle3D, ColliderHandle3D)]
  events : Array[BroadPhasePairEvent3D]
}

///|
pub fn BroadPhase3D::new() -> BroadPhase3D {
  { pairs: [], prev_pairs: [], events: [] }
}

///|
pub fn BroadPhase3D::pairs(
  self : BroadPhase3D,
) -> Array[(ColliderHandle3D, ColliderHandle3D)] {
  self.pairs
}

///|
pub fn BroadPhase3D::take_events(
  self : BroadPhase3D,
) -> Array[BroadPhasePairEvent3D] {
  let out : Array[BroadPhasePairEvent3D] = []
  for i in 0..<self.events.length() {
    out.push(self.events[i])
  }
  self.events.clear()
  out
}

///|
fn pair_key(a : ColliderHandle3D, b : ColliderHandle3D) -> (Int, Int, Int, Int) {
  let (ai, ag) = a.into_raw_parts()
  let (bi, bg) = b.into_raw_parts()
  if ai < bi || (ai == bi && ag <= bg) {
    (ai, ag, bi, bg)
  } else {
    (bi, bg, ai, ag)
  }
}

///|

///|
fn pair_less_3d(
  a : (ColliderHandle3D, ColliderHandle3D),
  b : (ColliderHandle3D, ColliderHandle3D),
) -> Bool {
  let ka = pair_key(a.0, a.1)
  let kb = pair_key(b.0, b.1)
  if ka.0 != kb.0 {
    return ka.0 < kb.0
  }
  if ka.1 != kb.1 {
    return ka.1 < kb.1
  }
  if ka.2 != kb.2 {
    return ka.2 < kb.2
  }
  ka.3 < kb.3
}

///|
fn handle_less_3d(a : ColliderHandle3D, b : ColliderHandle3D) -> Bool {
  let (ai, ag) = a.into_raw_parts()
  let (bi, bg) = b.into_raw_parts()
  if ai < bi {
    true
  } else if ai > bi {
    false
  } else {
    ag < bg
  }
}

///|
fn sorted_pair_3d(
  a : ColliderHandle3D,
  b : ColliderHandle3D,
) -> (ColliderHandle3D, ColliderHandle3D) {
  if handle_less_3d(a, b) {
    (a, b)
  } else {
    (b, a)
  }
}

///|
fn sort_pairs_3d(pairs : Array[(ColliderHandle3D, ColliderHandle3D)]) -> Unit {
  // Simple insertion sort (small N in tests; can be replaced with a faster sort later).
  for i in 1..<pairs.length() {
    let mut j = i
    while j > 0 && pair_less_3d(pairs[j], pairs[j - 1]) {
      let tmp = pairs[j - 1]
      pairs[j - 1] = pairs[j]
      pairs[j] = tmp
      j = j - 1
    }
  }
}

///|
priv struct BvhLeaf3D {
  handle : ColliderHandle3D
  aabb : @core.Aabb3
  center : @core.Vec3
}

///|
priv struct BvhNode3D {
  aabb : @core.Aabb3
  left : Int?
  right : Int?
  leaf : ColliderHandle3D?
}

///|
fn swap_leaves_3d(leaves : Array[BvhLeaf3D], i : Int, j : Int) -> Unit {
  if i == j {
    return
  }
  let tmp = leaves[i]
  leaves[i] = leaves[j]
  leaves[j] = tmp
}

///|
fn leaf_key_3d(leaf : BvhLeaf3D, axis : Int) -> @core.Real {
  match axis {
    0 => leaf.center.x
    1 => leaf.center.y
    _ => leaf.center.z
  }
}

///|
fn quicksort_leaves_3d(
  leaves : Array[BvhLeaf3D],
  start : Int,
  end : Int,
  axis : Int,
) -> Unit {
  if end - start <= 1 {
    return
  }
  let pivot = leaf_key_3d(leaves[(start + end) / 2], axis)
  let mut i = start
  let mut j = end - 1
  while i <= j {
    while leaf_key_3d(leaves[i], axis) < pivot {
      i = i + 1
    }
    while leaf_key_3d(leaves[j], axis) > pivot {
      j = j - 1
    }
    if i <= j {
      swap_leaves_3d(leaves, i, j)
      i = i + 1
      j = j - 1
    }
  }
  if start < j + 1 {
    quicksort_leaves_3d(leaves, start, j + 1, axis)
  }
  if i < end {
    quicksort_leaves_3d(leaves, i, end, axis)
  }
}

///|
fn build_bvh_3d(
  nodes : Array[BvhNode3D],
  leaves : Array[BvhLeaf3D],
  start : Int,
  end : Int,
) -> Int {
  let count = end - start
  if count == 1 {
    let leaf = leaves[start]
    let index = nodes.length()
    nodes.push(BvhNode3D::{
      aabb: leaf.aabb,
      left: None,
      right: None,
      leaf: Some(leaf.handle),
    })
    return index
  }

  // Choose split axis based on the spread of centers.
  let mut min_x = leaves[start].center.x
  let mut max_x = min_x
  let mut min_y = leaves[start].center.y
  let mut max_y = min_y
  let mut min_z = leaves[start].center.z
  let mut max_z = min_z
  for i in (start + 1)..<end {
    let c = leaves[i].center
    if c.x < min_x {
      min_x = c.x
    }
    if c.x > max_x {
      max_x = c.x
    }
    if c.y < min_y {
      min_y = c.y
    }
    if c.y > max_y {
      max_y = c.y
    }
    if c.z < min_z {
      min_z = c.z
    }
    if c.z > max_z {
      max_z = c.z
    }
  }
  let sx = max_x - min_x
  let sy = max_y - min_y
  let sz = max_z - min_z
  let axis = if sx >= sy && sx >= sz { 0 } else if sy >= sz { 1 } else { 2 }
  quicksort_leaves_3d(leaves, start, end, axis)
  let mid = start + count / 2
  let left = build_bvh_3d(nodes, leaves, start, mid)
  let right = build_bvh_3d(nodes, leaves, mid, end)
  let node_aabb = nodes[left].aabb.combine(nodes[right].aabb)
  let index = nodes.length()
  nodes.push(BvhNode3D::{
    aabb: node_aabb,
    left: Some(left),
    right: Some(right),
    leaf: None,
  })
  index
}

///|
fn collect_bvh_pairs_3d(
  nodes : Array[BvhNode3D],
  a : Int,
  b : Int,
  out : Array[(ColliderHandle3D, ColliderHandle3D)],
) -> Unit {
  let node_a = nodes[a]
  let node_b = nodes[b]
  if !node_a.aabb.intersects(node_b.aabb) {
    return
  }
  if node_a.leaf is Some(handle_a) && node_b.leaf is Some(handle_b) {
    if handle_a.equals(handle_b) {
      return
    }
    out.push(sorted_pair_3d(handle_a, handle_b))
    return
  }
  if a == b {
    if node_a.left is Some(left) && node_a.right is Some(right) {
      collect_bvh_pairs_3d(nodes, left, left, out)
      collect_bvh_pairs_3d(nodes, left, right, out)
      collect_bvh_pairs_3d(nodes, right, right, out)
    }
    return
  }
  if node_a.leaf is Some(_) {
    if node_b.left is Some(left) && node_b.right is Some(right) {
      collect_bvh_pairs_3d(nodes, a, left, out)
      collect_bvh_pairs_3d(nodes, a, right, out)
    }
    return
  }
  if node_b.leaf is Some(_) {
    if node_a.left is Some(left) && node_a.right is Some(right) {
      collect_bvh_pairs_3d(nodes, left, b, out)
      collect_bvh_pairs_3d(nodes, right, b, out)
    }
    return
  }
  if node_a.left is Some(al) &&
    node_a.right is Some(ar) &&
    node_b.left is Some(bl) &&
    node_b.right is Some(br) {
    collect_bvh_pairs_3d(nodes, al, bl, out)
    collect_bvh_pairs_3d(nodes, al, br, out)
    collect_bvh_pairs_3d(nodes, ar, bl, out)
    collect_bvh_pairs_3d(nodes, ar, br, out)
  }
}

///|
pub fn BroadPhase3D::update(
  self : BroadPhase3D,
  prediction_distance : @core.Real,
  colliders : ColliderSet3D,
) -> Unit {
  self.prev_pairs.clear()
  for i in 0..<self.pairs.length() {
    self.prev_pairs.push(self.pairs[i])
  }
  self.pairs.clear()
  self.events.clear()
  let handles : Array[ColliderHandle3D] = []
  for i in 0..<colliders.colliders.length() {
    if colliders.colliders[i] is Some(_) {
      handles.push(
        ColliderHandle3D::from_raw_parts(i, colliders.generations[i]),
      )
    }
  }

  // BVH broadphase (replaces the old naive O(n^2) scan).
  let leaves : Array[BvhLeaf3D] = []
  for i in 0..<handles.length() {
    let h = handles[i]
    if colliders.get(h) is Some(c) {
      if !c.enabled() {
        continue
      }
      let aabb = c.compute_collision_aabb(prediction_distance)
      let center = aabb.mins.add(aabb.maxs).scale(0.5F)
      leaves.push(BvhLeaf3D::{ handle: h, aabb, center })
    }
  }
  if leaves.length() > 1 {
    let nodes : Array[BvhNode3D] = []
    let root = build_bvh_3d(nodes, leaves, 0, leaves.length())
    collect_bvh_pairs_3d(nodes, root, root, self.pairs)
  }
  sort_pairs_3d(self.pairs)
  sort_pairs_3d(self.prev_pairs)

  // Diff old vs new.
  let mut i = 0
  let mut j = 0
  while i < self.prev_pairs.length() || j < self.pairs.length() {
    if i >= self.prev_pairs.length() {
      let p = self.pairs[j]
      self.events.push(
        BroadPhasePairEvent3D::AddPair(ColliderPair3D::new(p.0, p.1)),
      )
      j = j + 1
      continue
    }
    if j >= self.pairs.length() {
      let p = self.prev_pairs[i]
      self.events.push(
        BroadPhasePairEvent3D::DeletePair(ColliderPair3D::new(p.0, p.1)),
      )
      i = i + 1
      continue
    }
    let a = self.prev_pairs[i]
    let b = self.pairs[j]
    if pair_less_3d(a, b) {
      self.events.push(
        BroadPhasePairEvent3D::DeletePair(ColliderPair3D::new(a.0, a.1)),
      )
      i = i + 1
    } else if pair_less_3d(b, a) {
      self.events.push(
        BroadPhasePairEvent3D::AddPair(ColliderPair3D::new(b.0, b.1)),
      )
      j = j + 1
    } else {
      i = i + 1
      j = j + 1
    }
  }
}
