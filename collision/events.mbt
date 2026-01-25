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
const COLLISION_EVENT_FLAG_SENSOR : Int = 1 << 0

///|
const COLLISION_EVENT_FLAG_REMOVED : Int = 1 << 1

///|
pub struct CollisionEventFlags {
  mut bits : Int
}

///|
fn CollisionEventFlags::new(bits : Int) -> CollisionEventFlags {
  { bits, }
}

///|
pub fn CollisionEventFlags::empty() -> CollisionEventFlags {
  CollisionEventFlags::new(0)
}

///|
pub fn CollisionEventFlags::sensor() -> CollisionEventFlags {
  CollisionEventFlags::new(COLLISION_EVENT_FLAG_SENSOR)
}

///|
pub fn CollisionEventFlags::removed() -> CollisionEventFlags {
  CollisionEventFlags::new(COLLISION_EVENT_FLAG_REMOVED)
}

///|
pub fn CollisionEventFlags::contains(
  self : CollisionEventFlags,
  other : CollisionEventFlags,
) -> Bool {
  (self.bits & other.bits) == other.bits
}

///|
pub fn CollisionEventFlags::insert(
  self : CollisionEventFlags,
  other : CollisionEventFlags,
) -> CollisionEventFlags {
  self.bits = self.bits | other.bits
  self
}

///|
pub fn CollisionEvent::started(self : CollisionEvent) -> Bool {
  match self {
    CollisionEvent::Started(_, _, _) => true
    _ => false
  }
}

///|
pub fn CollisionEvent::stopped(self : CollisionEvent) -> Bool {
  match self {
    CollisionEvent::Stopped(_, _, _) => true
    _ => false
  }
}

///|
pub fn CollisionEvent::collider1(self : CollisionEvent) -> ColliderHandle {
  match self {
    CollisionEvent::Started(h, _, _) => h
    CollisionEvent::Stopped(h, _, _) => h
  }
}

///|
pub fn CollisionEvent::collider2(self : CollisionEvent) -> ColliderHandle {
  match self {
    CollisionEvent::Started(_, h, _) => h
    CollisionEvent::Stopped(_, h, _) => h
  }
}

///|
pub fn CollisionEvent::sensor(self : CollisionEvent) -> Bool {
  match self {
    CollisionEvent::Started(_, _, flags) =>
      flags.contains(CollisionEventFlags::sensor())
    CollisionEvent::Stopped(_, _, flags) =>
      flags.contains(CollisionEventFlags::sensor())
  }
}

///|
pub fn CollisionEvent::removed(self : CollisionEvent) -> Bool {
  match self {
    CollisionEvent::Started(_, _, flags) =>
      flags.contains(CollisionEventFlags::removed())
    CollisionEvent::Stopped(_, _, flags) =>
      flags.contains(CollisionEventFlags::removed())
  }
}
