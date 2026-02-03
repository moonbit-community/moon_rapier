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
test "ActiveEvents bitflags parity helpers" {
  let a = ActiveEvents::collision_events()
  let b = ActiveEvents::contact_force_events()
  let ab = a.union(b)
  inspect(ab.contains(a), content="true")
  inspect(ab.contains(b), content="true")
  inspect(ab.is_all(), content="true")
  inspect(a.complement().contains(b), content="true")
  inspect(ActiveEvents::from_bits(3) is Some(_), content="true")
  inspect(ActiveEvents::from_bits(4) is None, content="true")
  inspect(
    ActiveEvents::from_name("COLLISION_EVENTS") is Some(_),
    content="true",
  )
  inspect(ActiveEvents::from_name("NOPE") is None, content="true")
}

///|
test "ActiveHooks bitflags parity helpers" {
  let a = ActiveHooks::filter_contact_pairs()
  let b = ActiveHooks::modify_solver_contacts()
  let ab = a.union(b)
  inspect(ab.contains(a), content="true")
  inspect(ab.contains(b), content="true")
  inspect(ab.difference(a).contains(b), content="true")
  inspect(ActiveHooks::from_bits(0b1000) is None, content="true")
  inspect(
    ActiveHooks::from_name("FILTER_CONTACT_PAIRS") is Some(_),
    content="true",
  )
}

///|
test "QueryFilterFlags bitflags parity helpers" {
  let f = QueryFilterFlags::exclude_fixed().union(
    QueryFilterFlags::exclude_dynamic(),
  )
  inspect(f.bits == ((1 << 0) | (1 << 2)), content="true")
  let g = f.difference(QueryFilterFlags::exclude_fixed())
  inspect(g.bits == 1 << 2, content="true")
  inspect(QueryFilterFlags::from_bits(1 << 6) is None, content="true")
  inspect(
    QueryFilterFlags::from_name("ONLY_DYNAMIC") is Some(_),
    content="true",
  )
  inspect(
    QueryFilterFlags::from_name("EXCLUDE_SOLIDS") is Some(_),
    content="true",
  )
}
