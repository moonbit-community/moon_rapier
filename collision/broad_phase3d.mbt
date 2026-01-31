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

  // Naive O(n^2) broadphase for now. This is used only by the real dim3 backend.
  let handles : Array[ColliderHandle3D] = []
  for i in 0..<colliders.colliders.length() {
    if colliders.colliders[i] is Some(_) {
      handles.push(
        ColliderHandle3D::from_raw_parts(i, colliders.generations[i]),
      )
    }
  }
  for i in 0..<handles.length() {
    let hi = handles[i]
    if colliders.get(hi) is Some(ci) {
      if !ci.enabled() {
        continue
      }
      let ai = ci.compute_collision_aabb(prediction_distance)
      for j in (i + 1)..<handles.length() {
        let hj = handles[j]
        if colliders.get(hj) is Some(cj) {
          if !cj.enabled() {
            continue
          }
          let aj = cj.compute_collision_aabb(prediction_distance)
          if ai.intersects(aj) {
            self.pairs.push((hi, hj))
          }
        }
      }
    }
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
