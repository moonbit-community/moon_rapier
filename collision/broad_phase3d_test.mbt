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
test "broad phase 3d pairs basic" {
  let colliders = ColliderSet3D::new()
  let ch1 = colliders.insert(ColliderBuilder3D::ball(0.5F).build())
  let ch2 = colliders.insert(ColliderBuilder3D::ball(0.5F).build())
  let broad_phase = BroadPhase3D::new()
  broad_phase.update(0.0F, colliders)
  let pairs = broad_phase.pairs()
  inspect(pairs.length() == 1, content="true")
  let pair = pairs[0]
  let matches = pair.0.equals(ch1) && pair.1.equals(ch2)
  let matches_swapped = pair.0.equals(ch2) && pair.1.equals(ch1)
  inspect(matches || matches_swapped, content="true")
}

///|
test "broad phase 3d emits add/delete pair events" {
  let colliders = ColliderSet3D::new()
  let ch1 = colliders.insert(ColliderBuilder3D::ball(0.5F).build())
  let ch2 = colliders.insert(ColliderBuilder3D::ball(0.5F).build())
  let broad_phase = BroadPhase3D::new()
  broad_phase.update(0.0F, colliders)
  let ev1 = broad_phase.take_events()
  inspect(ev1.length() == 1, content="true")
  if ev1[0] is BroadPhasePairEvent3D::AddPair(pair1) {
    let a = pair1.collider1()
    let b = pair1.collider2()
    inspect(
      (a.equals(ch1) && b.equals(ch2)) || (a.equals(ch2) && b.equals(ch1)),
      content="true",
    )
  } else {
    inspect(false, content="true")
  }
  if colliders.get_mut(ch2) is Some(c2) {
    c2.set_enabled(false) |> ignore
  } else {
    inspect(false, content="true")
  }
  broad_phase.update(0.0F, colliders)
  let ev2 = broad_phase.take_events()
  inspect(ev2.length() == 1, content="true")
  if ev2[0] is BroadPhasePairEvent3D::DeletePair(pair2) {
    let a = pair2.collider1()
    let b = pair2.collider2()
    inspect(
      (a.equals(ch1) && b.equals(ch2)) || (a.equals(ch2) && b.equals(ch1)),
      content="true",
    )
  } else {
    inspect(false, content="true")
  }
}

///|
test "broad phase 3d skips disabled colliders" {
  let colliders = ColliderSet3D::new()
  colliders.insert(ColliderBuilder3D::ball(0.5F).build()) |> ignore
  colliders.insert(ColliderBuilder3D::ball(0.5F).enabled(false).build())
  |> ignore
  let broad_phase = BroadPhase3D::new()
  broad_phase.update(0.0F, colliders)
  inspect(broad_phase.pairs().length() == 0, content="true")
}

///|
test "broad phase 3d prediction distance expands aabbs" {
  let colliders = ColliderSet3D::new()
  colliders.insert(
    ColliderBuilder3D::ball(0.5F).translation(@core.Vec3::zero()).build(),
  )
  |> ignore
  colliders.insert(
    ColliderBuilder3D::ball(0.5F)
    .translation(@core.Vec3::new(1.01F, 0.0F, 0.0F))
    .build(),
  )
  |> ignore
  let broad_phase = BroadPhase3D::new()
  broad_phase.update(0.0F, colliders)
  inspect(broad_phase.pairs().length() == 0, content="true")
  broad_phase.update(0.01F, colliders)
  inspect(broad_phase.pairs().length() == 1, content="true")
}
