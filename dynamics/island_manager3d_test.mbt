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

///|
test "island_manager3d tracks per-island additional solver iterations" {
  let islands = IslandManager3D::new()
  let bodies = RigidBodySet3D::new()
  let b1 = bodies.insert(
    RigidBodyBuilder3D::dynamic().additional_solver_iterations(2).build(),
  )
  let b2 = bodies.insert(
    RigidBodyBuilder3D::dynamic().additional_solver_iterations(5).build(),
  )
  let b3 = bodies.insert(
    RigidBodyBuilder3D::dynamic().additional_solver_iterations(1).build(),
  )
  islands.update(bodies, [(b1, b2)])
  let active_islands = islands.active_islands()
  let mut max_iters = 0
  for i in 0..<active_islands.length() {
    let island_id = active_islands[i]
    let island_bodies = islands.island_bodies(island_id)
    if island_bodies.length() == 2 &&
      islands.island_additional_solver_iterations(island_id) > max_iters {
      max_iters = islands.island_additional_solver_iterations(island_id)
    }
  }
  inspect(max_iters == 5, content="true")
  // Body b3 is disconnected, so it either forms its own island or remains isolated.
  // In both cases, querying invalid ids must be safe.
  inspect(islands.island_additional_solver_iterations(-1) == 0, content="true")
  inspect(
    islands.island_additional_solver_iterations(active_islands.length() + 10) ==
    0,
    content="true",
  )
  // Keep handle referenced to avoid accidental "unused" behavior changes.
  b3 |> ignore
}
