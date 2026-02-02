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
  if pairs.length() <= 1 {
    return
  }
  fn swap_pairs_3d(
    pairs : Array[(ColliderHandle3D, ColliderHandle3D)],
    i : Int,
    j : Int,
  ) -> Unit {
    if i == j {
      return
    }
    let tmp = pairs[i]
    pairs[i] = pairs[j]
    pairs[j] = tmp
  }

  fn quicksort_pairs_3d(
    pairs : Array[(ColliderHandle3D, ColliderHandle3D)],
    start : Int,
    end : Int,
  ) -> Unit {
    if end - start <= 1 {
      return
    }
    let pivot = pairs[(start + end) / 2]
    let mut i = start
    let mut j = end - 1
    while i <= j {
      while pair_less_3d(pairs[i], pivot) {
        i = i + 1
      }
      while pair_less_3d(pivot, pairs[j]) {
        j = j - 1
      }
      if i <= j {
        swap_pairs_3d(pairs, i, j)
        i = i + 1
        j = j - 1
      }
    }
    if start < j + 1 {
      quicksort_pairs_3d(pairs, start, j + 1)
    }
    if i < end {
      quicksort_pairs_3d(pairs, i, end)
    }
  }

  quicksort_pairs_3d(pairs, 0, pairs.length())
}

///|
fn dedup_pairs_3d(pairs : Array[(ColliderHandle3D, ColliderHandle3D)]) -> Unit {
  if pairs.length() <= 1 {
    return
  }
  let mut out = 1
  for i in 1..<pairs.length() {
    if pair_less_3d(pairs[out - 1], pairs[i]) ||
      pair_less_3d(pairs[i], pairs[out - 1]) {
      pairs[out] = pairs[i]
      out = out + 1
    }
  }
  while pairs.length() > out {
    pairs.pop() |> ignore
  }
}

///|
priv struct BvhLeaf3D {
  handle : ColliderHandle3D
  aabb : @core.Aabb3
  center : @core.Vec3
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

  // Sweep-and-prune along X.
  let regular_handles : Array[ColliderHandle3D] = []
  let infinite_handles : Array[ColliderHandle3D] = []
  for i in 0..<handles.length() {
    let h = handles[i]
    if colliders.get(h) is Some(c) {
      if !c.enabled() {
        continue
      }
      match c.shape() {
        Shape3D::HalfSpace(_) => infinite_handles.push(h)
        _ => regular_handles.push(h)
      }
    }
  }
  let leaves : Array[BvhLeaf3D] = []
  for i in 0..<regular_handles.length() {
    let h = regular_handles[i]
    if colliders.get(h) is Some(c) {
      let aabb = c.compute_collision_aabb(prediction_distance)
      // `center` is only used as a sort key in this sweep-and-prune path.
      leaves.push(BvhLeaf3D::{ handle: h, aabb, center: aabb.mins })
    }
  }
  if leaves.length() > 1 {
    quicksort_leaves_3d(leaves, 0, leaves.length(), 0)
    for i in 0..<leaves.length() {
      let a = leaves[i]
      for j in (i + 1)..<leaves.length() {
        let b = leaves[j]
        if b.aabb.mins.x > a.aabb.maxs.x {
          break
        }
        if a.aabb.intersects(b.aabb) {
          self.pairs.push(sorted_pair_3d(a.handle, b.handle))
        }
      }
    }
  }

  // Pair infinite colliders (half-spaces) with everything else.
  for i in 0..<infinite_handles.length() {
    let h = infinite_handles[i]
    for j in 0..<regular_handles.length() {
      let g = regular_handles[j]
      if !h.equals(g) {
        self.pairs.push(sorted_pair_3d(h, g))
      }
    }
    // Also pair between half-spaces (rare, but keeps the pair set consistent).
    for j in (i + 1)..<infinite_handles.length() {
      let g = infinite_handles[j]
      if !h.equals(g) {
        self.pairs.push(sorted_pair_3d(h, g))
      }
    }
  }
  sort_pairs_3d(self.pairs)
  dedup_pairs_3d(self.pairs)
  // `prev_pairs` is a verbatim copy of `self.pairs` from the previous update, which was already
  // sorted and deduplicated.

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
