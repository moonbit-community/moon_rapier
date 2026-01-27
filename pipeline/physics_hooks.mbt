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
  mut modify_solver_contacts : ((
    @dynamics.RigidBodySet,
    @collision.ColliderSet,
    @collision.ColliderHandle,
    @collision.ColliderHandle,
    @collision.ContactManifold,
  ) -> (Bool, @core.Vec2))?
}

///|
pub fn PhysicsHooks::new() -> PhysicsHooks {
  {
    filter_contact_pair: None,
    filter_intersection_pair: None,
    modify_solver_contacts: None,
  }
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
pub fn PhysicsHooks::modify_solver_contacts(
  self : PhysicsHooks,
  callback : (
    @dynamics.RigidBodySet,
    @collision.ColliderSet,
    @collision.ColliderHandle,
    @collision.ColliderHandle,
    @collision.ContactManifold,
  ) -> (Bool, @core.Vec2),
) -> PhysicsHooks {
  self.modify_solver_contacts = Some(callback)
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

///|
fn PhysicsHooks::modify_solver_contacts_internal(
  self : PhysicsHooks,
  bodies : @dynamics.RigidBodySet,
  colliders : @collision.ColliderSet,
  collider1 : @collision.ColliderHandle,
  collider2 : @collision.ColliderHandle,
  manifold : @collision.ContactManifold,
) -> (Bool, @core.Vec2) {
  if self.modify_solver_contacts is Some(callback) {
    callback(bodies, colliders, collider1, collider2, manifold)
  } else {
    (true, @core.Vec2::zero())
  }
}
