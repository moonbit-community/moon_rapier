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
/// 3D-flavored collider builder (dim3,f32) used by the rapier3d parity tests.
///
/// NOTE: This currently projects to the existing 2D collider builder by dropping the `z`
/// coordinate. It is a compatibility layer to enable strict porting of the rapier-reference
/// dim3 unit tests.
pub struct ColliderBuilder3 {
  inner : ColliderBuilder
}

///|
pub fn ColliderBuilder3::cuboid(
  hx : @core.Real,
  hy : @core.Real,
  _hz : @core.Real,
) -> ColliderBuilder3 {
  { inner: ColliderBuilder::cuboid(hx, hy) }
}

///|
pub fn ColliderBuilder3::ball(radius : @core.Real) -> ColliderBuilder3 {
  { inner: ColliderBuilder::ball(radius) }
}

///|
pub fn ColliderBuilder3::translation(
  self : ColliderBuilder3,
  translation : @core.Vec3,
) -> ColliderBuilder3 {
  {
    inner: self.inner.translation(@core.Vec2::new(translation.x, translation.y)),
  }
}

///|
pub fn ColliderBuilder3::active_collision_types(
  self : ColliderBuilder3,
  types : ActiveCollisionTypes,
) -> ColliderBuilder3 {
  { inner: self.inner.active_collision_types(types) }
}

///|
pub fn ColliderBuilder3::active_events(
  self : ColliderBuilder3,
  events : ActiveEvents,
) -> ColliderBuilder3 {
  { inner: self.inner.active_events(events) }
}

///|
pub fn ColliderBuilder3::active_hooks(
  self : ColliderBuilder3,
  hooks : ActiveHooks,
) -> ColliderBuilder3 {
  { inner: self.inner.active_hooks(hooks) }
}

///|
pub fn ColliderBuilder3::sensor(
  self : ColliderBuilder3,
  enabled : Bool,
) -> ColliderBuilder3 {
  { inner: self.inner.sensor(enabled) }
}

///|
pub fn ColliderBuilder3::build(self : ColliderBuilder3) -> Collider {
  self.inner.build()
}

///|
/// 3D-flavored collider set (dim3,f32).
pub struct ColliderSet3 {
  inner : ColliderSet
}

///|
pub fn ColliderSet3::new() -> ColliderSet3 {
  { inner: ColliderSet::new() }
}

///|
pub fn ColliderSet3::as_2d(self : ColliderSet3) -> ColliderSet {
  self.inner
}

///|
pub fn ColliderSet3::insert(
  self : ColliderSet3,
  collider : Collider,
) -> ColliderHandle {
  self.inner.insert(collider)
}

///|
pub fn ColliderSet3::insert_with_parent(
  self : ColliderSet3,
  collider : Collider,
  parent : @dynamics.RigidBodyHandle,
  bodies : @dynamics.RigidBodySet3,
) -> ColliderHandle {
  self.inner.insert_with_parent(collider, parent, bodies.as_2d())
}

///|
pub fn ColliderSet3::remove(
  self : ColliderSet3,
  handle : ColliderHandle,
  islands : @dynamics.IslandManager3,
  bodies : @dynamics.RigidBodySet3,
  wake_up : Bool,
) -> Unit {
  self.inner.remove(handle, islands.as_2d(), bodies.as_2d(), wake_up)
}
