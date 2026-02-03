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
test "arena insert/remove reuses slot with new generation" {
  let arena : Arena[Int] = Arena::with_capacity(2)
  let idx1 = arena.insert(10)
  inspect(arena.get(idx1) == Some(10), content="true")
  let removed = arena.remove(idx1)
  inspect(removed == Some(10), content="true")
  inspect(arena.get(idx1) is None, content="true")
  let idx2 = arena.insert(20)
  inspect(arena.get(idx2) == Some(20), content="true")
  let (i1, g1) = idx1.into_raw_parts()
  let (i2, g2) = idx2.into_raw_parts()
  inspect(i1 == i2, content="true")
  inspect(g1 != g2, content="true")
}

///|
test "arena try_insert respects fixed capacity" {
  let arena : Arena[Int] = Arena::with_capacity(1)
  inspect(arena.try_insert(1) is Ok(_), content="true")
  // No free slots left, `try_insert` must not allocate.
  inspect(arena.try_insert(2) is Err(_), content="true")
  // But `insert` will allocate and succeed.
  let _ = arena.insert(3)
  inspect(arena.len() == 2, content="true")
}

///|
test "arena drain removes all occupied elements" {
  let arena : Arena[String] = Arena::new()
  let idx1 = arena.insert("hello")
  let idx2 = arena.insert("world")
  inspect(arena.get(idx1) is Some(_), content="true")
  inspect(arena.get(idx2) is Some(_), content="true")
  let d = arena.drain()
  let got : Array[String] = []
  let it = d
  while true {
    match it.next() {
      None => break
      Some((_idx, v)) => got.push(v)
    }
  }
  got.sort()
  inspect(got, content="[\"hello\", \"world\"]")
  inspect(arena.get(idx1) is None, content="true")
  inspect(arena.get(idx2) is None, content="true")
  inspect(arena.capacity() == 0, content="true")
}
