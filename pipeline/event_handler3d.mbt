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
pub struct EventHandler3D {
  collision_events : Array[@collision.CollisionEvent3D]
}

///|
pub fn EventHandler3D::new() -> EventHandler3D {
  { collision_events: [] }
}

///|
pub fn EventHandler3D::push_collision_event(
  self : EventHandler3D,
  event : @collision.CollisionEvent3D,
) -> Unit {
  self.collision_events.push(event)
}

///|
pub fn EventHandler3D::take_collision_events(
  self : EventHandler3D,
) -> Array[@collision.CollisionEvent3D] {
  let out : Array[@collision.CollisionEvent3D] = []
  for i in 0..<self.collision_events.length() {
    out.push(self.collision_events[i])
  }
  self.collision_events.clear()
  out
}
