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
pub struct PhysicsHooks3D {
  mut filter_contact_pair : ((
    @dynamics.RigidBodySet3D,
    @collision.ColliderSet3D,
    @collision.ColliderHandle3D,
    @collision.ColliderHandle3D,
  ) -> Bool)?
  mut filter_intersection_pair : ((
    @dynamics.RigidBodySet3D,
    @collision.ColliderSet3D,
    @collision.ColliderHandle3D,
    @collision.ColliderHandle3D,
  ) -> Bool)?
  mut modify_solver_contacts : ((
    @dynamics.RigidBodySet3D,
    @collision.ColliderSet3D,
    @collision.ColliderHandle3D,
    @collision.ColliderHandle3D,
    @core.Vec3,
  ) -> (Bool, @core.Vec3?))?
}

///|
fn resolve_hook_user_data_3d(
  bodies : @dynamics.RigidBodySet3D,
  colliders : @collision.ColliderSet3D,
  collider1 : @collision.ColliderHandle3D,
  collider2 : @collision.ColliderHandle3D,
) -> (
  @core.UserData128,
  @core.UserData128,
  @dynamics.RigidBodyHandle?,
  @dynamics.RigidBodyHandle?,
  @core.UserData128?,
  @core.UserData128?,
) {
  let mut ud1 = @core.UserData128::zero()
  let mut ud2 = @core.UserData128::zero()
  let mut rb1 : @dynamics.RigidBodyHandle? = None
  let mut rb2 : @dynamics.RigidBodyHandle? = None
  let mut rb_ud1 : @core.UserData128? = None
  let mut rb_ud2 : @core.UserData128? = None
  if colliders.get(collider1) is Some(co1) {
    ud1 = co1.user_data128()
    rb1 = co1.parent()
    if rb1 is Some(h1) {
      if bodies.get(h1) is Some(b1) {
        rb_ud1 = Some(b1.user_data128())
      }
    }
  }
  if colliders.get(collider2) is Some(co2) {
    ud2 = co2.user_data128()
    rb2 = co2.parent()
    if rb2 is Some(h2) {
      if bodies.get(h2) is Some(b2) {
        rb_ud2 = Some(b2.user_data128())
      }
    }
  }
  (ud1, ud2, rb1, rb2, rb_ud1, rb_ud2)
}

///|
pub fn PhysicsHooks3D::new() -> PhysicsHooks3D {
  {
    filter_contact_pair: None,
    filter_intersection_pair: None,
    modify_solver_contacts: None,
  }
}

///|
pub fn PhysicsHooks3D::filter_contact_pair(
  self : PhysicsHooks3D,
  predicate : (
    @dynamics.RigidBodySet3D,
    @collision.ColliderSet3D,
    @collision.ColliderHandle3D,
    @collision.ColliderHandle3D,
  ) -> Bool,
) -> PhysicsHooks3D {
  self.filter_contact_pair = Some(predicate)
  self
}

///|
pub fn PhysicsHooks3D::filter_contact_pair_with_user_data(
  self : PhysicsHooks3D,
  predicate : (
    @dynamics.RigidBodySet3D,
    @collision.ColliderSet3D,
    @collision.ColliderHandle3D,
    @collision.ColliderHandle3D,
    @core.UserData128,
    @core.UserData128,
    @dynamics.RigidBodyHandle?,
    @dynamics.RigidBodyHandle?,
    @core.UserData128?,
    @core.UserData128?,
  ) -> Bool,
) -> PhysicsHooks3D {
  self.filter_contact_pair = Some(fn(
    bodies : @dynamics.RigidBodySet3D,
    colliders : @collision.ColliderSet3D,
    collider1 : @collision.ColliderHandle3D,
    collider2 : @collision.ColliderHandle3D,
  ) -> Bool {
    let (ud1, ud2, rb1, rb2, rb_ud1, rb_ud2) = resolve_hook_user_data_3d(
      bodies, colliders, collider1, collider2,
    )
    predicate(
      bodies, colliders, collider1, collider2, ud1, ud2, rb1, rb2, rb_ud1, rb_ud2,
    )
  })
  self
}

///|
pub fn PhysicsHooks3D::filter_intersection_pair(
  self : PhysicsHooks3D,
  predicate : (
    @dynamics.RigidBodySet3D,
    @collision.ColliderSet3D,
    @collision.ColliderHandle3D,
    @collision.ColliderHandle3D,
  ) -> Bool,
) -> PhysicsHooks3D {
  self.filter_intersection_pair = Some(predicate)
  self
}

///|
pub fn PhysicsHooks3D::filter_intersection_pair_with_user_data(
  self : PhysicsHooks3D,
  predicate : (
    @dynamics.RigidBodySet3D,
    @collision.ColliderSet3D,
    @collision.ColliderHandle3D,
    @collision.ColliderHandle3D,
    @core.UserData128,
    @core.UserData128,
    @dynamics.RigidBodyHandle?,
    @dynamics.RigidBodyHandle?,
    @core.UserData128?,
    @core.UserData128?,
  ) -> Bool,
) -> PhysicsHooks3D {
  self.filter_intersection_pair = Some(fn(
    bodies : @dynamics.RigidBodySet3D,
    colliders : @collision.ColliderSet3D,
    collider1 : @collision.ColliderHandle3D,
    collider2 : @collision.ColliderHandle3D,
  ) -> Bool {
    let (ud1, ud2, rb1, rb2, rb_ud1, rb_ud2) = resolve_hook_user_data_3d(
      bodies, colliders, collider1, collider2,
    )
    predicate(
      bodies, colliders, collider1, collider2, ud1, ud2, rb1, rb2, rb_ud1, rb_ud2,
    )
  })
  self
}

///|
pub fn PhysicsHooks3D::modify_solver_contacts(
  self : PhysicsHooks3D,
  callback : (
    @dynamics.RigidBodySet3D,
    @collision.ColliderSet3D,
    @collision.ColliderHandle3D,
    @collision.ColliderHandle3D,
    @core.Vec3,
  ) -> (Bool, @core.Vec3?),
) -> PhysicsHooks3D {
  self.modify_solver_contacts = Some(callback)
  self
}

///|
pub fn PhysicsHooks3D::modify_solver_contacts_with_user_data(
  self : PhysicsHooks3D,
  callback : (
    @dynamics.RigidBodySet3D,
    @collision.ColliderSet3D,
    @collision.ColliderHandle3D,
    @collision.ColliderHandle3D,
    @core.UserData128,
    @core.UserData128,
    @dynamics.RigidBodyHandle?,
    @dynamics.RigidBodyHandle?,
    @core.UserData128?,
    @core.UserData128?,
    @core.Vec3,
  ) -> (Bool, @core.Vec3?),
) -> PhysicsHooks3D {
  self.modify_solver_contacts = Some(fn(
    bodies : @dynamics.RigidBodySet3D,
    colliders : @collision.ColliderSet3D,
    collider1 : @collision.ColliderHandle3D,
    collider2 : @collision.ColliderHandle3D,
    normal : @core.Vec3,
  ) -> (Bool, @core.Vec3?) {
    let (ud1, ud2, rb1, rb2, rb_ud1, rb_ud2) = resolve_hook_user_data_3d(
      bodies, colliders, collider1, collider2,
    )
    callback(
      bodies, colliders, collider1, collider2, ud1, ud2, rb1, rb2, rb_ud1, rb_ud2,
      normal,
    )
  })
  self
}

///|
fn PhysicsHooks3D::test_contact_pair(
  self : PhysicsHooks3D,
  bodies : @dynamics.RigidBodySet3D,
  colliders : @collision.ColliderSet3D,
  collider1 : @collision.ColliderHandle3D,
  collider2 : @collision.ColliderHandle3D,
) -> Bool {
  if self.filter_contact_pair is Some(predicate) {
    predicate(bodies, colliders, collider1, collider2)
  } else {
    true
  }
}

///|
fn PhysicsHooks3D::test_intersection_pair(
  self : PhysicsHooks3D,
  bodies : @dynamics.RigidBodySet3D,
  colliders : @collision.ColliderSet3D,
  collider1 : @collision.ColliderHandle3D,
  collider2 : @collision.ColliderHandle3D,
) -> Bool {
  if self.filter_intersection_pair is Some(predicate) {
    predicate(bodies, colliders, collider1, collider2)
  } else {
    true
  }
}

///|
fn PhysicsHooks3D::modify_solver_contacts_internal(
  self : PhysicsHooks3D,
  bodies : @dynamics.RigidBodySet3D,
  colliders : @collision.ColliderSet3D,
  collider1 : @collision.ColliderHandle3D,
  collider2 : @collision.ColliderHandle3D,
  normal : @core.Vec3,
) -> (Bool, @core.Vec3?) {
  if self.modify_solver_contacts is Some(callback) {
    callback(bodies, colliders, collider1, collider2, normal)
  } else {
    (true, None)
  }
}
