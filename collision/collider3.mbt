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
pub fn ColliderBuilder3::compound(
  shapes : Array[(@core.Isometry3, SharedShape)],
) -> ColliderBuilder3 {
  let shapes2 : Array[(@core.Isometry2, Shape)] = []
  for i in 0..<shapes.length() {
    let (iso3, shared) = shapes[i]
    let axis = iso3.rotation.to_scaled_axis()
    let iso2 = @core.Isometry2::new(
      @core.Vec2::new(iso3.translation.x, iso3.translation.y),
      @core.Rot2::from_angle(axis.z),
    )
    shapes2.push((iso2, shared.as_shape()))
  }
  { inner: ColliderBuilder::compound(shapes2) }
}

///|
pub fn ColliderBuilder3::capsule_x(
  radius : @core.Real,
  half_height : @core.Real,
) -> ColliderBuilder3 {
  { inner: ColliderBuilder::capsule_x(radius, half_height) }
}

///|
pub fn ColliderBuilder3::capsule_y(
  radius : @core.Real,
  half_height : @core.Real,
) -> ColliderBuilder3 {
  { inner: ColliderBuilder::capsule_y(radius, half_height) }
}

///|
pub fn ColliderBuilder3::round_cylinder(
  radius : @core.Real,
  half_height : @core.Real,
  _border_radius : @core.Real,
) -> ColliderBuilder3 {
  // 2D compatibility: approximate a (rounded) cylinder with a capsule aligned with the Y axis.
  { inner: ColliderBuilder::capsule_y(radius, half_height) }
}

///|
pub fn ColliderBuilder3::cone(
  radius : @core.Real,
  half_height : @core.Real,
) -> ColliderBuilder3 {
  // 2D compatibility: approximate a cone with a cuboid of similar extents.
  { inner: ColliderBuilder::cuboid(radius, half_height) }
}

///|
pub fn ColliderBuilder3::convex_hull(
  points : Array[@core.Vec3],
) -> ColliderBuilder3? {
  let pts2 : Array[@core.Vec2] = []
  for i in 0..<points.length() {
    let p = points[i]
    pts2.push(@core.Vec2::new(p.x, p.y))
  }
  if ColliderBuilder::convex_hull(pts2) is Some(inner) {
    Some({ inner, })
  } else {
    None
  }
}

///|
pub fn ColliderBuilder3::round_convex_hull(
  points : Array[@core.Vec3],
  border_radius : @core.Real,
) -> ColliderBuilder3? {
  let pts2 : Array[@core.Vec2] = []
  for i in 0..<points.length() {
    let p = points[i]
    pts2.push(@core.Vec2::new(p.x, p.y))
  }
  if ColliderBuilder::round_convex_hull(pts2, border_radius) is Some(inner) {
    Some({ inner, })
  } else {
    None
  }
}

///|
pub fn ColliderBuilder3::trimesh(
  vertices : Array[@core.Vec3],
  indices : Array[(Int, Int, Int)],
) -> ColliderBuilder3? {
  let verts2 : Array[@core.Vec2] = []
  for i in 0..<vertices.length() {
    let v = vertices[i]
    verts2.push(@core.Vec2::new(v.x, v.y))
  }
  if ColliderBuilder::trimesh(verts2, indices) is Some(inner) {
    Some({ inner, })
  } else {
    None
  }
}

///|
pub fn ColliderBuilder3::heightfield(
  heights : Array[@core.Real],
  scale : @core.Vec3,
) -> ColliderBuilder3 {
  // 2D compatibility: interpret `scale.x` as the horizontal extent and `scale.y` as the vertical
  // scaling factor. The Z axis is ignored.
  {
    inner: ColliderBuilder::heightfield(
      heights,
      @core.Vec2::new(scale.x, scale.y),
    ),
  }
}

///|
pub fn ColliderBuilder3::segment(
  a : @core.Vec3,
  b : @core.Vec3,
) -> ColliderBuilder3 {
  {
    inner: ColliderBuilder::segment(
      @core.Vec2::new(a.x, a.y),
      @core.Vec2::new(b.x, b.y),
    ),
  }
}

///|
pub fn ColliderBuilder3::voxelized_mesh(
  vertices : Array[@core.Vec3],
  indices : Array[(Int, Int)],
  voxel_size : @core.Real,
  fill_mode : FillMode,
) -> ColliderBuilder3 {
  let verts2 : Array[@core.Vec2] = []
  for i in 0..<vertices.length() {
    let v = vertices[i]
    verts2.push(@core.Vec2::new(v.x, v.y))
  }
  {
    inner: ColliderBuilder::voxelized_mesh(
      verts2, indices, voxel_size, fill_mode,
    ),
  }
}

///|
pub fn ColliderBuilder3::voxels_from_points(
  voxel_size : @core.Vec3,
  points : Array[@core.Vec3],
) -> ColliderBuilder3 {
  let pts2 : Array[@core.Vec2] = []
  for i in 0..<points.length() {
    let p = points[i]
    pts2.push(@core.Vec2::new(p.x, p.y))
  }
  {
    inner: ColliderBuilder::voxels_from_points(
      @core.Vec2::new(voxel_size.x, voxel_size.y),
      pts2,
    ),
  }
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
pub fn ColliderBuilder3::rotation(
  self : ColliderBuilder3,
  rotation : @core.Quat,
) -> ColliderBuilder3 {
  // 2D compatibility: keep only the rotation around +Z (yaw).
  let axis = rotation.to_scaled_axis()
  { inner: self.inner.rotation(axis.z) }
}

///|
pub fn ColliderBuilder3::rotation_scaled_axis(
  self : ColliderBuilder3,
  scaled_axis : @core.Vec3,
) -> ColliderBuilder3 {
  { inner: self.inner.rotation(scaled_axis.z) }
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
pub fn ColliderBuilder3::collision_groups(
  self : ColliderBuilder3,
  groups : @dynamics.InteractionGroups,
) -> ColliderBuilder3 {
  { inner: self.inner.collision_groups(groups) }
}

///|
pub fn ColliderBuilder3::solver_groups(
  self : ColliderBuilder3,
  groups : @dynamics.InteractionGroups,
) -> ColliderBuilder3 {
  { inner: self.inner.solver_groups(groups) }
}

///|
pub fn ColliderBuilder3::restitution(
  self : ColliderBuilder3,
  restitution : @core.Real,
) -> ColliderBuilder3 {
  { inner: self.inner.restitution(restitution) }
}

///|
pub fn ColliderBuilder3::friction(
  self : ColliderBuilder3,
  friction : @core.Real,
) -> ColliderBuilder3 {
  { inner: self.inner.friction(friction) }
}

///|
pub fn ColliderBuilder3::density(
  self : ColliderBuilder3,
  density : @core.Real,
) -> ColliderBuilder3 {
  { inner: self.inner.density(density) }
}

///|
pub fn ColliderBuilder3::contact_skin(
  self : ColliderBuilder3,
  skin : @core.Real,
) -> ColliderBuilder3 {
  { inner: self.inner.contact_skin(skin) }
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
        let local_pos = @core.Isometry2::new(
          collider.local_translation,
          @core.Rot2::from_angle(collider.local_rotation),
        )
        mprops = mprops.add(collider.mass_properties().transform_by(local_pos))
      }
    }
    let added_mass = body.additional_mass()
    if added_mass != 0.0F {
      mprops = mprops.set_mass(mprops.mass + added_mass, true)
    }
    body.mark_local_mass_properties_changed() |> ignore
    body.set_local_mass_properties(mprops) |> ignore
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
    if parent is Some(parent_handle) {
      if bodies.get_mut(parent_handle) is Some(body) {
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
    collider.parent = parent
    collider.changes = collider.changes.insert(COLLIDER_CHANGES_PARENT)
    if parent is Some(parent_handle) {
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
