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
const ACTIVE_EVENTS_COLLISION_EVENTS : Int = 1 << 0

///|
const ACTIVE_EVENTS_CONTACT_FORCE_EVENTS : Int = 1 << 1

///|
pub struct ActiveEvents {
  mut bits : Int
}

///|
fn ActiveEvents::new(bits : Int) -> ActiveEvents {
  { bits, }
}

///|
fn active_events_all_bits() -> Int {
  ACTIVE_EVENTS_COLLISION_EVENTS | ACTIVE_EVENTS_CONTACT_FORCE_EVENTS
}

///|
pub fn ActiveEvents::empty() -> ActiveEvents {
  ActiveEvents::new(0)
}

///|
pub fn ActiveEvents::collision_events() -> ActiveEvents {
  ActiveEvents::new(ACTIVE_EVENTS_COLLISION_EVENTS)
}

///|
pub fn ActiveEvents::contact_force_events() -> ActiveEvents {
  ActiveEvents::new(ACTIVE_EVENTS_CONTACT_FORCE_EVENTS)
}

///|
pub fn ActiveEvents::contains(
  self : ActiveEvents,
  other : ActiveEvents,
) -> Bool {
  (self.bits & other.bits) == other.bits
}

///|
pub fn ActiveEvents::insert(
  self : ActiveEvents,
  other : ActiveEvents,
) -> ActiveEvents {
  self.bits = self.bits | other.bits
  self
}

///|
pub fn ActiveEvents::union(
  self : ActiveEvents,
  other : ActiveEvents,
) -> ActiveEvents {
  ActiveEvents::{ bits: self.bits | other.bits }
}

///|
pub fn ActiveEvents::difference(
  self : ActiveEvents,
  other : ActiveEvents,
) -> ActiveEvents {
  ActiveEvents::{ bits: self.bits & Int::lnot(other.bits) }
}

///|
pub fn ActiveEvents::symmetric_difference(
  self : ActiveEvents,
  other : ActiveEvents,
) -> ActiveEvents {
  ActiveEvents::{ bits: self.bits ^ other.bits }
}

///|
pub fn ActiveEvents::toggle(
  self : ActiveEvents,
  other : ActiveEvents,
) -> ActiveEvents {
  self.symmetric_difference(other)
}

///|
pub fn ActiveEvents::complement(self : ActiveEvents) -> ActiveEvents {
  ActiveEvents::{ bits: active_events_all_bits() & Int::lnot(self.bits) }
}

///|
pub fn ActiveEvents::is_all(self : ActiveEvents) -> Bool {
  (self.bits & active_events_all_bits()) == active_events_all_bits()
}

///|
pub fn ActiveEvents::from_bits(bits : Int) -> ActiveEvents? {
  if (bits & Int::lnot(active_events_all_bits())) == 0 {
    Some(ActiveEvents::{ bits, })
  } else {
    None
  }
}

///|
pub fn ActiveEvents::from_bits_truncate(bits : Int) -> ActiveEvents {
  ActiveEvents::{ bits: bits & active_events_all_bits() }
}

///|
pub fn ActiveEvents::from_bits_retain(bits : Int) -> ActiveEvents {
  ActiveEvents::{ bits, }
}

///|
pub fn ActiveEvents::iter_names(self : ActiveEvents) -> Array[String] {
  let result : Array[String] = []
  if self.contains(ActiveEvents::collision_events()) {
    result.push("COLLISION_EVENTS")
  }
  if self.contains(ActiveEvents::contact_force_events()) {
    result.push("CONTACT_FORCE_EVENTS")
  }
  result
}

///|
pub fn ActiveEvents::from_name(name : String) -> ActiveEvents? {
  if name == "COLLISION_EVENTS" {
    Some(ActiveEvents::collision_events())
  } else if name == "CONTACT_FORCE_EVENTS" {
    Some(ActiveEvents::contact_force_events())
  } else {
    None
  }
}

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

///|
pub struct ContactForceEvent {
  collider1 : ColliderHandle
  collider2 : ColliderHandle
  total_force : @core.Vec2
  total_force_magnitude : @core.Real
  max_force_direction : @core.Vec2
  max_force_magnitude : @core.Real
}

///|
pub fn ContactForceEvent::new(
  collider1 : ColliderHandle,
  collider2 : ColliderHandle,
  total_force : @core.Vec2,
  total_force_magnitude : @core.Real,
  max_force_direction : @core.Vec2,
  max_force_magnitude : @core.Real,
) -> ContactForceEvent {
  {
    collider1,
    collider2,
    total_force,
    total_force_magnitude,
    max_force_direction,
    max_force_magnitude,
  }
}

///|
pub fn ContactForceEvent::collider1(self : ContactForceEvent) -> ColliderHandle {
  self.collider1
}

///|
pub fn ContactForceEvent::collider2(self : ContactForceEvent) -> ColliderHandle {
  self.collider2
}

///|
pub fn ContactForceEvent::total_force(self : ContactForceEvent) -> @core.Vec2 {
  self.total_force
}

///|
pub fn ContactForceEvent::total_force_magnitude(
  self : ContactForceEvent,
) -> @core.Real {
  self.total_force_magnitude
}

///|
pub fn ContactForceEvent::max_force_direction(
  self : ContactForceEvent,
) -> @core.Vec2 {
  self.max_force_direction
}

///|
pub fn ContactForceEvent::max_force_magnitude(
  self : ContactForceEvent,
) -> @core.Real {
  self.max_force_magnitude
}
