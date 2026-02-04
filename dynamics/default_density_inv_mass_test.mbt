// Copyright 2026 International Digital Economy Academy
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
test "2d: attaching collider recomputes mass properties immediately (default density)" {
  let bodies = RigidBodySet::new()
  let colliders = @collision.ColliderSet::new()
  let rb = bodies.insert(RigidBodyBuilder::dynamic().build())

  // Simulate a world that already processed the initial insertion.
  bodies.take_modified() |> ignore

  // Attach a collider without calling `.density(...)` explicitly.
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(0.5F, 0.5F).build(),
    rb,
    bodies,
  )
  |> ignore

  // Mass properties should be immediately available for impulses.
  if bodies.get(rb) is Some(body) {
    inspect(body.inv_mass() > 0.0F, content="true")
    body.apply_impulse(@core.Vec2::new(1.0F, 0.0F), true) |> ignore
    inspect(body.linvel().x != 0.0F, content="true")
  } else {
    inspect(false, content="true")
  }
}
