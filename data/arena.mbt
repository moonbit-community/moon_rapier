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
pub struct Index {
  index : Int
  generation : Int
}

///|
pub fn Index::default() -> Index {
  Index::from_raw_parts(-1, -1)
}

///|
pub fn Index::from_raw_parts(index : Int, generation : Int) -> Index {
  { index, generation }
}

///|
pub fn Index::into_raw_parts(self : Index) -> (Int, Int) {
  (self.index, self.generation)
}

///|
const DEFAULT_ARENA_CAPACITY : Int = 4

///|
enum Entry[T] {
  Free(Int?)
  Occupied(Int, T)
}

///|
pub struct Arena[T] {
  items : Array[Entry[T]]
  mut generation : Int
  mut free_list_head : Int?
  mut len : Int
}

///|
pub fn[T] Arena::new() -> Arena[T] {
  Arena::with_capacity(DEFAULT_ARENA_CAPACITY)
}

///|
pub fn[T] Arena::with_capacity(capacity : Int) -> Arena[T] {
  let mut cap = capacity
  if cap < 1 {
    cap = 1
  }
  let arena : Arena[T] = {
    items: [],
    generation: 0,
    free_list_head: None,
    len: 0,
  }
  arena.reserve(cap)
  arena
}

///|
pub fn[T] Arena::clear(self : Arena[T]) -> Unit {
  // Mirror rapier's behavior: keep allocation, but reinitialize the free list.
  let mut end = self.items.capacity()
  if end < 1 {
    end = 1
  }
  self.items.clear()
  for i in 0..<end {
    let next_free : Int? = if i == end - 1 { None } else { Some(i + 1) }
    self.items.push(Entry::Free(next_free))
  }
  self.free_list_head = Some(0)
  self.len = 0
}

///|
pub fn[T] Arena::len(self : Arena[T]) -> Int {
  self.len
}

///|
pub fn[T] Arena::is_empty(self : Arena[T]) -> Bool {
  self.len == 0
}

///|
pub fn[T] Arena::capacity(self : Arena[T]) -> Int {
  self.items.length()
}

///|
pub fn[T] Arena::reserve(self : Arena[T], additional_capacity : Int) -> Unit {
  if additional_capacity <= 0 {
    return
  }
  let start = self.items.length()
  let end = start + additional_capacity
  let old_head = self.free_list_head
  self.items.reserve_capacity(additional_capacity)
  for i in start..<end {
    if i == end - 1 {
      self.items.push(Entry::Free(old_head))
    } else {
      self.items.push(Entry::Free(Some(i + 1)))
    }
  }
  self.free_list_head = Some(start)
}

///|
fn[T] Arena::try_alloc_next_index(self : Arena[T]) -> Index? {
  match self.free_list_head {
    None => None
    Some(i) =>
      match self.items.get(i) {
        None => panic()
        Some(Entry::Occupied(_, _)) => panic()
        Some(Entry::Free(next_free)) => {
          self.free_list_head = next_free
          self.len = self.len + 1
          Some(Index::from_raw_parts(i, self.generation))
        }
      }
  }
}

///|
pub fn[T] Arena::try_insert(self : Arena[T], value : T) -> Result[Index, T] {
  match self.try_alloc_next_index() {
    None => Err(value)
    Some(index) => {
      let (i, _) = index.into_raw_parts()
      self.items[i] = Entry::Occupied(self.generation, value)
      Ok(index)
    }
  }
}

///|
pub fn[T] Arena::try_insert_with(
  self : Arena[T],
  create : (Index) -> T,
) -> Result[Index, (Index) -> T] {
  match self.try_alloc_next_index() {
    None => Err(create)
    Some(index) => {
      let (i, _) = index.into_raw_parts()
      self.items[i] = Entry::Occupied(self.generation, create(index))
      Ok(index)
    }
  }
}

///|
fn[T] Arena::insert_slow_path(self : Arena[T], value : T) -> Index {
  let additional = self.items.length()
  self.reserve(additional)
  match self.try_insert(value) {
    Ok(i) => i
    Err(_) => panic()
  }
}

///|
fn[T] Arena::insert_with_slow_path(
  self : Arena[T],
  create : (Index) -> T,
) -> Index {
  let additional = self.items.length()
  self.reserve(additional)
  match self.try_insert_with(create) {
    Ok(i) => i
    Err(_) => panic()
  }
}

///|
pub fn[T] Arena::insert(self : Arena[T], value : T) -> Index {
  match self.try_insert(value) {
    Ok(i) => i
    Err(value) => self.insert_slow_path(value)
  }
}

///|
pub fn[T] Arena::insert_with(self : Arena[T], create : (Index) -> T) -> Index {
  match self.try_insert_with(create) {
    Ok(i) => i
    Err(create) => self.insert_with_slow_path(create)
  }
}

///|
pub fn[T] Arena::remove(self : Arena[T], index : Index) -> T? {
  let (i, g) = index.into_raw_parts()
  if i < 0 || i >= self.items.length() {
    return None
  }
  match self.items[i] {
    Entry::Occupied(gen, value) =>
      if g == gen {
        self.items[i] = Entry::Free(self.free_list_head)
        self.generation = self.generation + 1
        self.free_list_head = Some(i)
        self.len = self.len - 1
        Some(value)
      } else {
        None
      }
    _ => None
  }
}

///|
pub fn[T] Arena::retain(
  self : Arena[T],
  predicate : (Index, T) -> Bool,
) -> Unit {
  for i in 0..<self.capacity() {
    match self.items[i] {
      Entry::Occupied(generation, value) => {
        let index = Index::from_raw_parts(i, generation)
        if !predicate(index, value) {
          self.remove(index) |> ignore
        }
      }
      _ => ()
    }
  }
}

///|
pub fn[T] Arena::contains(self : Arena[T], index : Index) -> Bool {
  self.get(index) is Some(_)
}

///|
pub fn[T] Arena::get(self : Arena[T], index : Index) -> T? {
  let (i, g) = index.into_raw_parts()
  if i < 0 {
    return None
  }
  match self.items.get(i) {
    Some(Entry::Occupied(gen, value)) if gen == g => Some(value)
    _ => None
  }
}

///|
pub fn[T] Arena::get_mut(self : Arena[T], index : Index) -> T? {
  // MoonBit doesn't expose a stable `&mut` reference, so this mirrors `get`.
  self.get(index)
}

///|
pub fn[T] Arena::get2_mut(
  self : Arena[T],
  index1 : Index,
  index2 : Index,
) -> (T?, T?) {
  let (i1, g1) = index1.into_raw_parts()
  let (i2, g2) = index2.into_raw_parts()
  if i1 == i2 {
    if g1 == g2 {
      panic()
    }
    if g1 > g2 {
      return (self.get_mut(index1), None)
    }
    return (None, self.get_mut(index2))
  }
  if i1 < 0 || i1 >= self.items.length() {
    return (None, self.get_mut(index2))
  }
  if i2 < 0 || i2 >= self.items.length() {
    return (self.get_mut(index1), None)
  }
  (
    match self.items[i1] {
      Entry::Occupied(gen, value) if gen == g1 => Some(value)
      _ => None
    },
    match self.items[i2] {
      Entry::Occupied(gen, value) if gen == g2 => Some(value)
      _ => None
    },
  )
}

///|
pub fn[T] Arena::iter(self : Arena[T]) -> Array[(Index, T)] {
  let result : Array[(Index, T)] = []
  for i in 0..<self.items.length() {
    match self.items[i] {
      Entry::Occupied(generation, value) =>
        result.push((Index::from_raw_parts(i, generation), value))
      _ => ()
    }
  }
  result
}

///|
pub fn[T] Arena::iter_mut(self : Arena[T]) -> Array[(Index, T)] {
  // MoonBit doesn't expose a stable `&mut` reference, so this mirrors `iter`.
  self.iter()
}

///|
/// Iterate over elements of the arena and remove them.
///
/// Note: this mirrors Rapier's `Arena::drain` semantics by emptying the internal
/// `items` array without rebuilding the free list.
pub fn[T] Arena::drain(self : Arena[T]) -> Drain[T] {
  let drained : Array[(Index, T)] = []
  for i in 0..<self.items.length() {
    match self.items[i] {
      Entry::Occupied(generation, value) =>
        drained.push((Index::from_raw_parts(i, generation), value))
      _ => ()
    }
  }
  self.items.clear()
  { items: drained, cursor: 0 }
}

///|
pub struct Drain[T] {
  items : Array[(Index, T)]
  mut cursor : Int
}

///|
pub fn[T] Drain::next(self : Drain[T]) -> (Index, T)? {
  match self.items.get(self.cursor) {
    None => None
    Some(v) => {
      self.cursor = self.cursor + 1
      Some(v)
    }
  }
}

///|
pub struct Iter[T] {
  items : Array[(Index, T)]
  mut cursor : Int
}

///|
pub fn[T] Iter::from_arena(arena : Arena[T]) -> Iter[T] {
  { items: arena.iter(), cursor: 0 }
}

///|
pub fn[T] Iter::next(self : Iter[T]) -> (Index, T)? {
  match self.items.get(self.cursor) {
    None => None
    Some(v) => {
      self.cursor = self.cursor + 1
      Some(v)
    }
  }
}

///|
pub struct IterMut[T] {
  items : Array[(Index, T)]
  mut cursor : Int
}

///|
pub fn[T] IterMut::from_arena(arena : Arena[T]) -> IterMut[T] {
  // MoonBit doesn't expose a stable `&mut` reference, so this mirrors `Iter`.
  { items: arena.iter(), cursor: 0 }
}

///|
pub fn[T] IterMut::next(self : IterMut[T]) -> (Index, T)? {
  match self.items.get(self.cursor) {
    None => None
    Some(v) => {
      self.cursor = self.cursor + 1
      Some(v)
    }
  }
}

///|
pub struct IntoIter[T] {
  items : Array[T]
  mut cursor : Int
}

///|
pub fn[T] IntoIter::from_arena(arena : Arena[T]) -> IntoIter[T] {
  let values : Array[T] = []
  for i in 0..<arena.items.length() {
    match arena.items[i] {
      Entry::Occupied(_, value) => values.push(value)
      _ => ()
    }
  }
  { items: values, cursor: 0 }
}

///|
pub fn[T] IntoIter::next(self : IntoIter[T]) -> T? {
  match self.items.get(self.cursor) {
    None => None
    Some(v) => {
      self.cursor = self.cursor + 1
      Some(v)
    }
  }
}
