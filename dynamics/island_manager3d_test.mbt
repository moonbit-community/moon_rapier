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
test "island_manager3d update propagates activation across interactions" {
  let islands = IslandManager3D::new()
  let bodies = RigidBodySet3D::new()
  let b1 = bodies.insert(RigidBodyBuilder3D::dynamic().build())
  let b2 = bodies.insert(RigidBodyBuilder3D::dynamic().build())

  // Put both bodies to sleep (default time_until_sleep is 2s).
  bodies.update_sleep_all(2.1F, 1.0F)
  if bodies.get(b1) is Some(rb1) {
    rb1.wake_up()
  } else {
    inspect(false, content="true")
  }
  islands.update(bodies, [(b1, b2)])
  if bodies.get(b2) is Some(rb2) {
    inspect(rb2.is_active(), content="true")
  } else {
    inspect(false, content="true")
  }
  let actives = islands.active_bodies()
  let mut found1 = false
  let mut found2 = false
  for h in actives {
    if h.equals(b1) {
      found1 = true
    }
    if h.equals(b2) {
      found2 = true
    }
  }
  inspect(found1 && found2, content="true")
}
