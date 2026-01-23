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
test "broad phase bvh pairs basic" {
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = ColliderSet::new()
  let body1 = @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::zero())
    .build()
  let body2 = @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::zero())
    .build()
  let h1 = bodies.insert(body1)
  let h2 = bodies.insert(body2)
  let c1 = ColliderBuilder::ball(0.5F).build()
  let c2 = ColliderBuilder::ball(0.5F).build()
  let ch1 = colliders.insert_with_parent(c1, h1, bodies)
  let ch2 = colliders.insert_with_parent(c2, h2, bodies)
  let broad_phase = BroadPhaseBvh::new()
  broad_phase.update(0.0F, bodies, colliders)
  let pairs = broad_phase.pairs()
  inspect(pairs.length() == 1, content="true")
  let pair = pairs[0]
  let matches = ColliderHandle::equals(pair.0, ch1) &&
    ColliderHandle::equals(pair.1, ch2)
  let matches_swapped = ColliderHandle::equals(pair.0, ch2) &&
    ColliderHandle::equals(pair.1, ch1)
  inspect(matches || matches_swapped, content="true")
}

///|
test "broad phase bvh skips disabled colliders" {
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = ColliderSet::new()
  let h1 = bodies.insert(@dynamics.RigidBodyBuilder::dynamic().build())
  let h2 = bodies.insert(@dynamics.RigidBodyBuilder::dynamic().build())
  let c1 = ColliderBuilder::ball(0.5F).build()
  let c2 = ColliderBuilder::ball(0.5F).enabled(false).build()
  colliders.insert_with_parent(c1, h1, bodies) |> ignore
  colliders.insert_with_parent(c2, h2, bodies) |> ignore
  let broad_phase = BroadPhaseBvh::new()
  broad_phase.update(0.0F, bodies, colliders)
  inspect(broad_phase.pairs().length() == 0, content="true")
}

///|
test "broad phase bvh prediction distance expands aabb pairs" {
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = ColliderSet::new()
  let h1 = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::zero())
    .build(),
  )
  let h2 = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(1.01F, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(ColliderBuilder::ball(0.5F).build(), h1, bodies)
  |> ignore
  colliders.insert_with_parent(ColliderBuilder::ball(0.5F).build(), h2, bodies)
  |> ignore
  let broad_phase = BroadPhaseBvh::new()
  broad_phase.update(0.0F, bodies, colliders)
  inspect(broad_phase.pairs().length() == 0, content="true")
  broad_phase.update(0.01F, bodies, colliders)
  inspect(broad_phase.pairs().length() == 1, content="true")
}
