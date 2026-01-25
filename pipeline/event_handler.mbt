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
pub struct EventHandler {
  collision_events : Array[@collision.CollisionEvent]
  mut collision_callback : ((
    @dynamics.RigidBodySet,
    @collision.ColliderSet,
    @collision.CollisionEvent,
  ) -> Unit)?
}

///|
pub fn EventHandler::new() -> EventHandler {
  { collision_events: [], collision_callback: None }
}

///|
pub fn EventHandler::on_collision_event(
  self : EventHandler,
  callback : (
    @dynamics.RigidBodySet,
    @collision.ColliderSet,
    @collision.CollisionEvent,
  ) -> Unit,
) -> EventHandler {
  self.collision_callback = Some(callback)
  self
}

///|
pub fn EventHandler::handle_collision_event(
  self : EventHandler,
  bodies : @dynamics.RigidBodySet,
  colliders : @collision.ColliderSet,
  event : @collision.CollisionEvent,
) -> Unit {
  if self.collision_callback is Some(callback) {
    callback(bodies, colliders, event)
  }
  self.collision_events.push(event)
}

///|
pub fn EventHandler::push_collision_event(
  self : EventHandler,
  event : @collision.CollisionEvent,
) -> Unit {
  self.collision_events.push(event)
}

///|
pub fn EventHandler::take_collision_events(
  self : EventHandler,
) -> Array[@collision.CollisionEvent] {
  let result : Array[@collision.CollisionEvent] = []
  for i in 0..<self.collision_events.length() {
    result.push(self.collision_events[i])
  }
  self.collision_events.clear()
  result
}
