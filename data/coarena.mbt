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
pub struct Coarena[T] {
  data : Array[(Int, T)]
}

///|
const INVALID_GENERATION : Int = -1

///|
pub fn[T] Coarena::new() -> Coarena[T] {
  { data: [] }
}

///|
pub fn[T] Coarena::len(self : Coarena[T]) -> Int {
  let mut n = 0
  for i in 0..<self.data.length() {
    if self.data[i].0 != INVALID_GENERATION {
      n = n + 1
    }
  }
  n
}

///|
pub fn[T] Coarena::reserve(self : Coarena[T], additional : Int) -> Unit {
  self.data.reserve_capacity(additional)
}

///|
pub fn[T] Coarena::iter(self : Coarena[T]) -> Array[(Index, T)] {
  let result : Array[(Index, T)] = []
  for i in 0..<self.data.length() {
    let (generation, value) = self.data[i]
    if generation != INVALID_GENERATION {
      result.push((Index::from_raw_parts(i, generation), value))
    }
  }
  result
}

///|
pub fn[T] Coarena::get_unknown_gen(self : Coarena[T], index : Int) -> T? {
  if index < 0 {
    return None
  }
  match self.data.get(index) {
    Some((_, value)) => Some(value)
    None => None
  }
}

///|
pub fn[T] Coarena::get_mut_unknown_gen(self : Coarena[T], index : Int) -> T? {
  // MoonBit doesn't expose a stable `&mut` reference, so this mirrors `get_unknown_gen`.
  self.get_unknown_gen(index)
}

///|
pub fn[T] Coarena::remove(
  self : Coarena[T],
  index : Index,
  removed_value : T,
) -> T? {
  let (i, g) = index.into_raw_parts()
  if i < 0 || i >= self.data.length() {
    return None
  }
  let (gen, value) = self.data[i]
  if gen == g {
    self.data[i] = (INVALID_GENERATION, removed_value)
    Some(value)
  } else {
    None
  }
}

///|
pub fn[T] Coarena::get(self : Coarena[T], index : Index) -> T? {
  let (i, g) = index.into_raw_parts()
  if i < 0 {
    return None
  }
  match self.data.get(i) {
    Some((gen, value)) if gen == g => Some(value)
    _ => None
  }
}

///|
pub fn[T] Coarena::get_mut(self : Coarena[T], index : Index) -> T? {
  // MoonBit doesn't expose a stable `&mut` reference, so this mirrors `get`.
  self.get(index)
}

///|
pub fn[T : Default] Coarena::insert(
  self : Coarena[T],
  index : Index,
  value : T,
) -> Unit {
  let (i, g) = index.into_raw_parts()
  if i < 0 {
    panic()
  }
  if self.data.length() <= i {
    self.data.resize(i + 1, (INVALID_GENERATION, T::default()))
  }
  self.data[i] = (g, value)
}

///|
pub fn[T] Coarena::ensure_element_exist(
  self : Coarena[T],
  index : Index,
  default : T,
) -> T {
  let (i, g) = index.into_raw_parts()
  if i < 0 {
    panic()
  }
  if self.data.length() <= i {
    self.data.resize(i + 1, (INVALID_GENERATION, default))
  }
  let (gen, value) = self.data[i]
  if gen != g {
    self.data[i] = (g, default)
    default
  } else {
    value
  }
}

///|
pub fn[T] Coarena::ensure_pair_exists(
  self : Coarena[T],
  index1 : Index,
  index2 : Index,
  default : T,
) -> (T, T) {
  let (i1, g1) = index1.into_raw_parts()
  let (i2, g2) = index2.into_raw_parts()
  if i1 == i2 {
    panic()
  }
  if i1 < 0 || i2 < 0 {
    panic()
  }
  let max_i = if i1 > i2 { i1 } else { i2 }
  if self.data.length() <= max_i {
    self.data.resize(max_i + 1, (INVALID_GENERATION, default))
  }
  let (gen1, _) = self.data[i1]
  if gen1 != g1 {
    self.data[i1] = (g1, default)
  }
  let (gen2, _) = self.data[i2]
  if gen2 != g2 {
    self.data[i2] = (g2, default)
  }
  (self.data[i1].1, self.data[i2].1)
}
