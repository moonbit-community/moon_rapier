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
pub struct PhysicsHooks {
  mut filter_contact_pair : ((
    @dynamics.RigidBodySet,
    @collision.ColliderSet,
    @collision.ColliderHandle,
    @collision.ColliderHandle,
  ) -> Bool)?
  mut filter_intersection_pair : ((
    @dynamics.RigidBodySet,
    @collision.ColliderSet,
    @collision.ColliderHandle,
    @collision.ColliderHandle,
  ) -> Bool)?
}

///|
pub fn PhysicsHooks::new() -> PhysicsHooks {
  { filter_contact_pair: None, filter_intersection_pair: None }
}

///|
pub fn PhysicsHooks::filter_contact_pair(
  self : PhysicsHooks,
  predicate : (
    @dynamics.RigidBodySet,
    @collision.ColliderSet,
    @collision.ColliderHandle,
    @collision.ColliderHandle,
  ) -> Bool,
) -> PhysicsHooks {
  self.filter_contact_pair = Some(predicate)
  self
}

///|
pub fn PhysicsHooks::filter_intersection_pair(
  self : PhysicsHooks,
  predicate : (
    @dynamics.RigidBodySet,
    @collision.ColliderSet,
    @collision.ColliderHandle,
    @collision.ColliderHandle,
  ) -> Bool,
) -> PhysicsHooks {
  self.filter_intersection_pair = Some(predicate)
  self
}

///|
fn PhysicsHooks::test_contact_pair(
  self : PhysicsHooks,
  bodies : @dynamics.RigidBodySet,
  colliders : @collision.ColliderSet,
  collider1 : @collision.ColliderHandle,
  collider2 : @collision.ColliderHandle,
) -> Bool {
  if self.filter_contact_pair is Some(predicate) {
    predicate(bodies, colliders, collider1, collider2)
  } else {
    true
  }
}

///|
fn PhysicsHooks::test_intersection_pair(
  self : PhysicsHooks,
  bodies : @dynamics.RigidBodySet,
  colliders : @collision.ColliderSet,
  collider1 : @collision.ColliderHandle,
  collider2 : @collision.ColliderHandle,
) -> Bool {
  if self.filter_intersection_pair is Some(predicate) {
    predicate(bodies, colliders, collider1, collider2)
  } else {
    true
  }
}
