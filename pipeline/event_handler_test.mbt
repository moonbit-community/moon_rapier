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
test "event handler collects contact force events and invokes callback" {
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = @collision.ColliderSet::new()
  let h1 = colliders.insert(@collision.ColliderBuilder::ball(0.5F).build())
  let h2 = colliders.insert(@collision.ColliderBuilder::ball(0.5F).build())
  let event = @collision.ContactForceEvent::new(
    h1,
    h2,
    @core.Vec2::new(2.0F, -1.0F),
    2.25F,
    @core.Vec2::new(1.0F, 0.0F),
    2.25F,
  )
  let seen : Array[@collision.ContactForceEvent] = []
  let seen_dt : Array[@core.Real] = []
  let handler = EventHandler::new().on_contact_force_event(fn(
    dt : @core.Real,
    _bodies : @dynamics.RigidBodySet,
    _colliders : @collision.ColliderSet,
    event : @collision.ContactForceEvent,
  ) -> Unit {
    seen_dt.push(dt)
    seen.push(event)
  })
  handler.handle_contact_force_event(0.1F, bodies, colliders, event)
  inspect(seen.length() == 1, content="true")
  inspect(seen_dt.length() == 1, content="true")
  inspect(@core.abs(seen_dt[0] - 0.1F) < 1.0e-6F, content="true")
  let collected = handler.take_contact_force_events()
  inspect(collected.length() == 1, content="true")
  inspect(
    @collision.ColliderHandle::equals(collected[0].collider1(), h1),
    content="true",
  )
  inspect(
    @collision.ColliderHandle::equals(collected[0].collider2(), h2),
    content="true",
  )
}
