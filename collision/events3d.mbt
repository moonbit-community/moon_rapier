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
pub(all) enum CollisionEvent3D {
  Started(ColliderHandle3D, ColliderHandle3D, CollisionEventFlags)
  Stopped(ColliderHandle3D, ColliderHandle3D, CollisionEventFlags)
}

///|
pub fn CollisionEvent3D::started(self : CollisionEvent3D) -> Bool {
  match self {
    CollisionEvent3D::Started(_, _, _) => true
    _ => false
  }
}

///|
pub fn CollisionEvent3D::stopped(self : CollisionEvent3D) -> Bool {
  match self {
    CollisionEvent3D::Stopped(_, _, _) => true
    _ => false
  }
}

///|
pub fn CollisionEvent3D::collider1(self : CollisionEvent3D) -> ColliderHandle3D {
  match self {
    CollisionEvent3D::Started(h, _, _) => h
    CollisionEvent3D::Stopped(h, _, _) => h
  }
}

///|
pub fn CollisionEvent3D::collider2(self : CollisionEvent3D) -> ColliderHandle3D {
  match self {
    CollisionEvent3D::Started(_, h, _) => h
    CollisionEvent3D::Stopped(_, h, _) => h
  }
}

///|
pub fn CollisionEvent3D::sensor(self : CollisionEvent3D) -> Bool {
  match self {
    CollisionEvent3D::Started(_, _, flags) =>
      flags.contains(CollisionEventFlags::sensor())
    CollisionEvent3D::Stopped(_, _, flags) =>
      flags.contains(CollisionEventFlags::sensor())
  }
}

///|
pub fn CollisionEvent3D::removed(self : CollisionEvent3D) -> Bool {
  match self {
    CollisionEvent3D::Started(_, _, flags) =>
      flags.contains(CollisionEventFlags::removed())
    CollisionEvent3D::Stopped(_, _, flags) =>
      flags.contains(CollisionEventFlags::removed())
  }
}

///|
pub struct IntersectionEvent3D {
  collider1 : ColliderHandle3D
  collider2 : ColliderHandle3D
  intersecting : Bool
}

///|
pub fn IntersectionEvent3D::new(
  collider1 : ColliderHandle3D,
  collider2 : ColliderHandle3D,
  intersecting : Bool,
) -> IntersectionEvent3D {
  { collider1, collider2, intersecting }
}

///|
pub fn IntersectionEvent3D::collider1(
  self : IntersectionEvent3D,
) -> ColliderHandle3D {
  self.collider1
}

///|
pub fn IntersectionEvent3D::collider2(
  self : IntersectionEvent3D,
) -> ColliderHandle3D {
  self.collider2
}

///|
pub fn IntersectionEvent3D::intersecting(self : IntersectionEvent3D) -> Bool {
  self.intersecting
}

///|
pub struct ContactForceEvent3D {
  collider1 : ColliderHandle3D
  collider2 : ColliderHandle3D
  total_force : @core.Vec3
  total_force_magnitude : @core.Real
  max_force_direction : @core.Vec3
  max_force_magnitude : @core.Real
}

///|
pub fn ContactForceEvent3D::new(
  collider1 : ColliderHandle3D,
  collider2 : ColliderHandle3D,
  total_force : @core.Vec3,
  total_force_magnitude : @core.Real,
  max_force_direction : @core.Vec3,
  max_force_magnitude : @core.Real,
) -> ContactForceEvent3D {
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
/// Build a contact force event from a contact pair.
///
/// Rapier's signature is `(dt, &ContactPair, total_force_magnitude)`.
/// This port's `ContactPair3D` does not store its collider handles, so we pass them explicitly.
pub fn ContactForceEvent3D::from_contact_pair(
  dt : @core.Real,
  collider1 : ColliderHandle3D,
  collider2 : ColliderHandle3D,
  pair : ContactPair3D,
  total_force_magnitude : @core.Real,
) -> ContactForceEvent3D {
  if dt <= 0.0F {
    return ContactForceEvent3D::new(
      collider1,
      collider2,
      @core.Vec3::zero(),
      0.0F,
      @core.Vec3::zero(),
      0.0F,
    )
  }
  let inv_dt = 1.0F / dt
  let total_force = pair.total_impulse().scale(inv_dt)
  let (max_impulse, max_dir) = pair.max_impulse()
  ContactForceEvent3D::new(
    collider1,
    collider2,
    total_force,
    total_force_magnitude,
    max_dir,
    max_impulse * inv_dt,
  )
}

///|
pub fn ContactForceEvent3D::collider1(
  self : ContactForceEvent3D,
) -> ColliderHandle3D {
  self.collider1
}

///|
pub fn ContactForceEvent3D::collider2(
  self : ContactForceEvent3D,
) -> ColliderHandle3D {
  self.collider2
}

///|
pub fn ContactForceEvent3D::total_force(
  self : ContactForceEvent3D,
) -> @core.Vec3 {
  self.total_force
}

///|
pub fn ContactForceEvent3D::total_force_magnitude(
  self : ContactForceEvent3D,
) -> @core.Real {
  self.total_force_magnitude
}

///|
pub fn ContactForceEvent3D::max_force_direction(
  self : ContactForceEvent3D,
) -> @core.Vec3 {
  self.max_force_direction
}

///|
pub fn ContactForceEvent3D::max_force_magnitude(
  self : ContactForceEvent3D,
) -> @core.Real {
  self.max_force_magnitude
}
