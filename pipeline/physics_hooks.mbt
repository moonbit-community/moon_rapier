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
pub struct PairFilterContext {
  bodies : @dynamics.RigidBodySet
  colliders : @collision.ColliderSet
  collider1 : @collision.ColliderHandle
  collider2 : @collision.ColliderHandle
  rigid_body1 : @dynamics.RigidBodyHandle?
  rigid_body2 : @dynamics.RigidBodyHandle?
}

///|
pub struct ContactModificationContext {
  bodies : @dynamics.RigidBodySet
  colliders : @collision.ColliderSet
  collider1 : @collision.ColliderHandle
  collider2 : @collision.ColliderHandle
  rigid_body1 : @dynamics.RigidBodyHandle?
  rigid_body2 : @dynamics.RigidBodyHandle?
  manifold : @collision.ContactManifold
  solver_contacts : Array[@collision.SolverContact]
  mut normal : @core.Vec2
  mut user_data : Int
}

///|
const CONTACT_CONFIGURATION_UNKNOWN : Int = 0

///|
const CONTACT_CURRENTLY_ALLOWED : Int = 1

///|
const CONTACT_CURRENTLY_FORBIDDEN : Int = 2

///|
pub fn ContactModificationContext::new(
  bodies : @dynamics.RigidBodySet,
  colliders : @collision.ColliderSet,
  collider1 : @collision.ColliderHandle,
  collider2 : @collision.ColliderHandle,
  rigid_body1 : @dynamics.RigidBodyHandle?,
  rigid_body2 : @dynamics.RigidBodyHandle?,
  manifold : @collision.ContactManifold,
  solver_contacts : Array[@collision.SolverContact],
  normal : @core.Vec2,
  user_data : Int,
) -> ContactModificationContext {
  {
    bodies,
    colliders,
    collider1,
    collider2,
    rigid_body1,
    rigid_body2,
    manifold,
    solver_contacts,
    normal,
    user_data,
  }
}

///|
pub fn ContactModificationContext::normal(
  self : ContactModificationContext,
) -> @core.Vec2 {
  self.normal
}

///|
pub fn ContactModificationContext::set_normal(
  self : ContactModificationContext,
  normal : @core.Vec2,
) -> Unit {
  self.normal = normal
}

///|
pub fn ContactModificationContext::set_user_data(
  self : ContactModificationContext,
  user_data : Int,
) -> Unit {
  self.user_data = user_data
}

///|
pub fn ContactModificationContext::update_as_oneway_platform(
  self : ContactModificationContext,
  allowed_local_n1 : @core.Vec2,
  allowed_angle : @core.Real,
) -> Unit {
  let cang = @core.cos(allowed_angle)
  let contact_is_ok = self.manifold.normal().dot(allowed_local_n1) >= cang
  fn all_contacts_separated(
    contacts : Array[@collision.SolverContact],
  ) -> Bool {
    for i in 0..<contacts.length() {
      if contacts[i].dist() <= 0.0F {
        return false
      }
    }
    true
  }

  if self.user_data == CONTACT_CONFIGURATION_UNKNOWN {
    if contact_is_ok {
      self.user_data = CONTACT_CURRENTLY_ALLOWED
    } else {
      self.solver_contacts.clear()
      // In rare cases the normal may be near zero; in that case we can't conclude.
      if self.manifold.normal().length_squared() > 0.1F {
        self.user_data = CONTACT_CURRENTLY_FORBIDDEN
      }
    }
  } else if self.user_data == CONTACT_CURRENTLY_FORBIDDEN {
    if contact_is_ok && all_contacts_separated(self.solver_contacts) {
      self.user_data = CONTACT_CURRENTLY_ALLOWED
    } else {
      self.solver_contacts.clear()
    }
  } else if self.user_data == CONTACT_CURRENTLY_ALLOWED {
    if self.solver_contacts.length() == 0 {
      self.user_data = CONTACT_CONFIGURATION_UNKNOWN
    }
  } else {
    self.user_data = CONTACT_CONFIGURATION_UNKNOWN
  }
}

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
  mut modify_solver_contacts_with_solver_contacts : ((
    @dynamics.RigidBodySet,
    @collision.ColliderSet,
    @collision.ColliderHandle,
    @collision.ColliderHandle,
    @collision.ContactManifold,
    Array[@collision.SolverContact],
    @collision.SolverFlags,
  ) -> @collision.SolverFlags)?
}

///|
pub fn PhysicsHooks::new() -> PhysicsHooks {
  {
    filter_contact_pair: None,
    filter_intersection_pair: None,
    modify_solver_contacts: None,
    modify_solver_contacts_with_solver_contacts: None,
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
pub fn PhysicsHooks::modify_solver_contacts_with_solver_contacts(
  self : PhysicsHooks,
  callback : (
    @dynamics.RigidBodySet,
    @collision.ColliderSet,
    @collision.ColliderHandle,
    @collision.ColliderHandle,
    @collision.ContactManifold,
    Array[@collision.SolverContact],
    @collision.SolverFlags,
  ) -> @collision.SolverFlags,
) -> PhysicsHooks {
  self.modify_solver_contacts_with_solver_contacts = Some(callback)
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

///|
fn PhysicsHooks::modify_solver_contacts_with_solver_contacts_internal(
  self : PhysicsHooks,
  bodies : @dynamics.RigidBodySet,
  colliders : @collision.ColliderSet,
  collider1 : @collision.ColliderHandle,
  collider2 : @collision.ColliderHandle,
  manifold : @collision.ContactManifold,
  solver_contacts : Array[@collision.SolverContact],
  solver_flags : @collision.SolverFlags,
) -> @collision.SolverFlags {
  if self.modify_solver_contacts_with_solver_contacts is Some(callback) {
    callback(
      bodies, colliders, collider1, collider2, manifold, solver_contacts, solver_flags,
    )
  } else {
    solver_flags
  }
}
