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
/// Source-aligned dim3 collider builder alias.
pub type ColliderBuilder3 = ColliderBuilder3D

///|
/// Source-aligned dim3 compatibility alias.
/// This intentionally resolves to the real dim3 builder implementation.
pub type ColliderBuilder3Compat = ColliderBuilder3D

///|
pub fn Collider::translation3(self : Collider) -> @core.Vec3 {
  let t = self.translation()
  @core.Vec3::new(t.x, t.y, 0.0F)
}

///|
fn recompute_body_mass_properties_from_attached_colliders3(
  bodies : @dynamics.RigidBodySet3,
  colliders : ColliderSet3,
  body_handle : @dynamics.RigidBodyHandle,
) -> Unit {
  if bodies.get_mut(body_handle) is Some(body) {
    let attached = colliders.colliders_with_parent(body_handle)
    let mut mprops = @core.MassProperties::default()
    for j in 0..<attached.length() {
      let co_handle = attached[j]
      if colliders.get(co_handle) is Some(collider) {
        if collider.is_enabled() {
          let local_pos = @core.Isometry2::new(
            collider.local_translation,
            @core.Rot2::from_angle(collider.local_rotation),
          )
          mprops = mprops.add(
            collider.mass_properties().transform_by(local_pos),
          )
        }
      }
    }
    body.mark_local_mass_properties_changed() |> ignore
    body.set_local_mass_properties(mprops) |> ignore
    body.recompute_mass_properties_from_colliders() |> ignore
  }
}

///|
fn ColliderSet3::sync_with_bodies(
  self : ColliderSet3,
  bodies : @dynamics.RigidBodySet3,
) -> Unit {
  for i in 0..<self.inner.colliders.length() {
    if self.inner.colliders[i] is Some(collider) {
      if collider.parent is Some(parent) {
        if bodies.get(parent) is Some(body) {
          let body_pos = body.position()
          let local_pos = @core.Isometry2::new(
            collider.local_translation,
            @core.Rot2::from_angle(collider.local_rotation),
          )
          let world_pos = body_pos.mul(local_pos)
          collider.world_translation = world_pos.translation
          collider.world_rotation = world_pos.rotation.angle()
          continue
        }
      }
      collider.world_translation = collider.local_translation
      collider.world_rotation = collider.local_rotation
    }
  }
}

///|
/// 3D-flavored collider set (dim3,f32).
pub struct ColliderSet3 {
  inner : ColliderSet

  fn new() -> ColliderSet3
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
pub fn ColliderSet3::len(self : ColliderSet3) -> Int {
  self.inner.len()
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
  let handle = self.inner.allocate_handle()
  let updated = collider
  updated.changes = ColliderChanges::all()
  updated.parent = Some(parent)
  if bodies.get(parent) is Some(body) {
    let body_pos = body.position()
    let local_pos = @core.Isometry2::new(
      updated.local_translation,
      @core.Rot2::from_angle(updated.local_rotation),
    )
    let world_pos = body_pos.mul(local_pos)
    updated.world_translation = world_pos.translation
    updated.world_rotation = world_pos.rotation.angle()
  } else {
    updated.world_translation = updated.local_translation
    updated.world_rotation = updated.local_rotation
  }
  self.inner.colliders[handle.id] = Some(updated)
  if self.inner.colliders[handle.id] is Some(collider_ref) {
    self.inner.push_modified_unchecked(handle, collider_ref)
  }
  if bodies.get_mut(parent) is Some(body) {
    let (collider_id, collider_generation) = handle.into_raw_parts()
    body.attach_collider_raw(collider_id, collider_generation) |> ignore
  }
  recompute_body_mass_properties_from_attached_colliders3(bodies, self, parent)
  handle
}

///|
pub fn ColliderSet3::remove(
  self : ColliderSet3,
  handle : ColliderHandle,
  islands : @dynamics.IslandManager3,
  bodies : @dynamics.RigidBodySet3,
  wake_up : Bool,
) -> Unit {
  if handle.id < 0 || handle.id >= self.inner.colliders.length() {
    return
  }
  if self.inner.generations[handle.id] != handle.generation {
    return
  }
  if self.inner.colliders[handle.id] is Some(collider) {
    let parent = collider.parent
    let (collider_id, collider_generation) = handle.into_raw_parts()
    if parent is Some(parent_handle) {
      if bodies.get_mut(parent_handle) is Some(body) {
        body.detach_collider_raw(collider_id, collider_generation) |> ignore
        body.mark_local_mass_properties_changed() |> ignore
        if wake_up && body.is_dynamic_or_kinematic() {
          body.wake_up(true) |> ignore
        }
      }
    }
    self.inner.colliders[handle.id] = None
    self.inner.removed_colliders.push(handle)
    self.inner.generations[handle.id] = self.inner.generations[handle.id] + 1
    self.inner.free_list.push(handle.id)
    if parent is Some(parent_handle) {
      recompute_body_mass_properties_from_attached_colliders3(
        bodies, self, parent_handle,
      )
      if wake_up {
        islands.wake_up(bodies, parent_handle, true)
      }
    }
  }
}

///|
pub fn ColliderSet3::colliders_with_parent(
  self : ColliderSet3,
  parent : @dynamics.RigidBodyHandle,
) -> Array[ColliderHandle] {
  self.inner.colliders_with_parent(parent)
}

///|
pub fn ColliderSet3::get(
  self : ColliderSet3,
  handle : ColliderHandle,
) -> Collider? {
  self.inner.get(handle)
}

///|
pub fn ColliderSet3::set_parent(
  self : ColliderSet3,
  handle : ColliderHandle,
  parent : @dynamics.RigidBodyHandle?,
  bodies : @dynamics.RigidBodySet3,
) -> Unit {
  if self.inner.get_mut_internal_with_modification_tracking(handle)
    is Some(collider) {
    let old_parent = collider.parent
    let (collider_id, collider_generation) = handle.into_raw_parts()
    collider.parent = parent
    collider.changes = collider.changes.insert(COLLIDER_CHANGES_PARENT)
    if old_parent is Some(old_parent_handle) {
      if bodies.get_mut(old_parent_handle) is Some(body) {
        body.detach_collider_raw(collider_id, collider_generation) |> ignore
      }
    }
    if parent is Some(parent_handle) {
      if bodies.get_mut(parent_handle) is Some(body) {
        body.attach_collider_raw(collider_id, collider_generation) |> ignore
      }
      if bodies.get(parent_handle) is Some(body) {
        let body_pos = body.position()
        let local_pos = @core.Isometry2::new(
          collider.local_translation,
          @core.Rot2::from_angle(collider.local_rotation),
        )
        let world_pos = body_pos.mul(local_pos)
        collider.world_translation = world_pos.translation
        collider.world_rotation = world_pos.rotation.angle()
      } else {
        collider.world_translation = collider.local_translation
        collider.world_rotation = collider.local_rotation
      }
      recompute_body_mass_properties_from_attached_colliders3(
        bodies, self, parent_handle,
      )
    } else {
      collider.world_translation = collider.local_translation
      collider.world_rotation = collider.local_rotation
    }
    if old_parent is Some(old_parent_handle) {
      recompute_body_mass_properties_from_attached_colliders3(
        bodies, self, old_parent_handle,
      )
    }
  }
}

///|
pub fn ColliderSet3::take_modified(
  self : ColliderSet3,
) -> Array[ColliderHandle] {
  self.inner.take_modified()
}

///|
pub fn ColliderSet3::set_parent_enabled(
  self : ColliderSet3,
  handle : @dynamics.RigidBodyHandle,
  enabled : Bool,
) -> Unit {
  self.inner.set_parent_enabled(handle, enabled)
}

///|
pub fn ColliderSet3::clear_changes_for(
  self : ColliderSet3,
  modified_colliders : Array[ColliderHandle],
) -> Unit {
  self.inner.clear_changes_for(modified_colliders)
}
