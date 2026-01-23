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
pub fn Ray::new(origin : @core.Vec2, dir : @core.Vec2) -> Ray {
  { origin, dir }
}

///|
const QUERY_FILTER_EXCLUDE_FIXED : Int = 1 << 0

///|
const QUERY_FILTER_EXCLUDE_KINEMATIC : Int = 1 << 1

///|
const QUERY_FILTER_EXCLUDE_DYNAMIC : Int = 1 << 2

///|
const QUERY_FILTER_EXCLUDE_SENSORS : Int = 1 << 3

///|
const QUERY_FILTER_EXCLUDE_SOLIDS : Int = 1 << 4

///|
const QUERY_FILTER_ONLY_DYNAMIC : Int = QUERY_FILTER_EXCLUDE_FIXED |
  QUERY_FILTER_EXCLUDE_KINEMATIC

///|
const QUERY_FILTER_ONLY_KINEMATIC : Int = QUERY_FILTER_EXCLUDE_DYNAMIC |
  QUERY_FILTER_EXCLUDE_FIXED

///|
const QUERY_FILTER_ONLY_FIXED : Int = QUERY_FILTER_EXCLUDE_DYNAMIC |
  QUERY_FILTER_EXCLUDE_KINEMATIC

///|
pub struct QueryFilterFlags {
  mut bits : Int
}

///|
pub fn QueryFilterFlags::new(bits : Int) -> QueryFilterFlags {
  { bits, }
}

///|
pub fn QueryFilterFlags::empty() -> QueryFilterFlags {
  QueryFilterFlags::new(0)
}

///|
pub fn QueryFilterFlags::contains(self : QueryFilterFlags, bits : Int) -> Bool {
  (self.bits & bits) == bits
}

///|
pub fn QueryFilterFlags::insert(
  self : QueryFilterFlags,
  bits : Int,
) -> QueryFilterFlags {
  self.bits = self.bits | bits
  self
}

///|
pub struct QueryFilter {
  excluded_body : @dynamics.RigidBodyHandle?
  excluded_collider : ColliderHandle?
  mut groups : @dynamics.InteractionGroups?
  mut flags : QueryFilterFlags
}

///|
pub fn QueryFilter::new() -> QueryFilter {
  {
    excluded_body: None,
    excluded_collider: None,
    groups: None,
    flags: QueryFilterFlags::empty(),
  }
}

///|
pub fn QueryFilter::exclude_rigid_body(
  self : QueryFilter,
  handle : @dynamics.RigidBodyHandle,
) -> QueryFilter {
  self |> ignore
  {
    excluded_body: Some(handle),
    excluded_collider: self.excluded_collider,
    groups: self.groups,
    flags: self.flags,
  }
}

///|
pub fn QueryFilter::exclude_collider(
  self : QueryFilter,
  handle : ColliderHandle,
) -> QueryFilter {
  self |> ignore
  {
    excluded_body: self.excluded_body,
    excluded_collider: Some(handle),
    groups: self.groups,
    flags: self.flags,
  }
}

///|
pub fn QueryFilter::groups(
  self : QueryFilter,
  groups : @dynamics.InteractionGroups,
) -> QueryFilter {
  self.groups = Some(groups)
  self
}

///|
pub fn QueryFilter::exclude_fixed(self : QueryFilter) -> QueryFilter {
  self.flags = self.flags.insert(QUERY_FILTER_EXCLUDE_FIXED)
  self
}

///|
pub fn QueryFilter::exclude_kinematic(self : QueryFilter) -> QueryFilter {
  self.flags = self.flags.insert(QUERY_FILTER_EXCLUDE_KINEMATIC)
  self
}

///|
pub fn QueryFilter::exclude_dynamic(self : QueryFilter) -> QueryFilter {
  self.flags = self.flags.insert(QUERY_FILTER_EXCLUDE_DYNAMIC)
  self
}

///|
pub fn QueryFilter::only_dynamic() -> QueryFilter {
  let filter = QueryFilter::new()
  filter.flags = QueryFilterFlags::new(QUERY_FILTER_ONLY_DYNAMIC)
  filter
}

///|
pub fn QueryFilter::only_kinematic() -> QueryFilter {
  let filter = QueryFilter::new()
  filter.flags = QueryFilterFlags::new(QUERY_FILTER_ONLY_KINEMATIC)
  filter
}

///|
pub fn QueryFilter::only_fixed() -> QueryFilter {
  let filter = QueryFilter::new()
  filter.flags = QueryFilterFlags::new(QUERY_FILTER_ONLY_FIXED)
  filter
}

///|
pub fn QueryFilter::exclude_sensors(self : QueryFilter) -> QueryFilter {
  self.flags = self.flags.insert(QUERY_FILTER_EXCLUDE_SENSORS)
  self
}

///|
pub fn QueryFilter::exclude_solids(self : QueryFilter) -> QueryFilter {
  self.flags = self.flags.insert(QUERY_FILTER_EXCLUDE_SOLIDS)
  self
}

///|
pub fn QueryFilter::groups_filter(
  self : QueryFilter,
) -> @dynamics.InteractionGroups? {
  self.groups
}

///|
fn query_filter_test(
  filter : QueryFilter,
  bodies : @dynamics.RigidBodySet,
  handle : ColliderHandle,
  collider : Collider,
) -> Bool {
  if filter.excluded_collider is Some(excluded) &&
    ColliderHandle::equals(excluded, handle) {
    return false
  }
  if filter.excluded_body is Some(excluded_body) {
    if collider.parent is Some(parent) &&
      @dynamics.RigidBodyHandle::equals(parent, excluded_body) {
      return false
    }
  }
  if filter.groups is Some(groups) {
    if !collider.collision_groups().test_groups(groups) {
      return false
    }
  }
  if filter.flags.contains(QUERY_FILTER_EXCLUDE_SENSORS) && collider.is_sensor() {
    return false
  }
  if filter.flags.contains(QUERY_FILTER_EXCLUDE_SOLIDS) && !collider.is_sensor() {
    return false
  }
  if filter.flags.contains(QUERY_FILTER_EXCLUDE_FIXED) {
    if collider.parent is None {
      return false
    }
  }
  if collider.parent is Some(parent) {
    if bodies.get(parent) is Some(body) {
      let body_type = body.body_type()
      if filter.flags.contains(QUERY_FILTER_EXCLUDE_FIXED) &&
        body_type.is_fixed() {
        return false
      }
      if filter.flags.contains(QUERY_FILTER_EXCLUDE_KINEMATIC) &&
        body_type.is_kinematic() {
        return false
      }
      if filter.flags.contains(QUERY_FILTER_EXCLUDE_DYNAMIC) &&
        body_type.is_dynamic() {
        return false
      }
    }
  }
  true
}

///|
pub struct QueryPipeline {
  filter : QueryFilter
  bodies : @dynamics.RigidBodySet
  colliders : ColliderSet
}

///|
pub fn QueryPipeline::new(
  filter : QueryFilter,
  bodies : @dynamics.RigidBodySet,
  colliders : ColliderSet,
) -> QueryPipeline {
  { filter, bodies, colliders }
}

///|
fn sqrt_real(value : @core.Real) -> @core.Real {
  if value <= 0.0F {
    0.0F
  } else {
    let mut x = value
    for _ in 0..<8 {
      x = 0.5F * (x + value / x)
    }
    x
  }
}

///|
fn ray_intersect_ball(
  ray : Ray,
  center : @core.Vec2,
  radius : @core.Real,
  max_toi : @core.Real,
  solid : Bool,
) -> @core.Real? {
  let d = ray.dir
  let m = ray.origin.sub(center)
  let a = d.dot(d)
  if a <= 1.0e-12F {
    return None
  }
  let b = m.dot(d)
  let c = m.dot(m) - radius * radius
  if c <= 0.0F {
    if solid {
      return Some(0.0F)
    }
    let disc = b * b - a * c
    if disc < 0.0F {
      return None
    }
    let t = (-b + sqrt_real(disc)) / a
    if t >= 0.0F && t <= max_toi {
      Some(t)
    } else {
      None
    }
  } else {
    let disc = b * b - a * c
    if disc < 0.0F {
      return None
    }
    let t = (-b - sqrt_real(disc)) / a
    let hit = if t >= 0.0F { t } else { (-b + sqrt_real(disc)) / a }
    if hit >= 0.0F && hit <= max_toi {
      Some(hit)
    } else {
      None
    }
  }
}

///|
fn ray_intersect_cuboid(
  ray : Ray,
  center : @core.Vec2,
  half_width : @core.Real,
  half_height : @core.Real,
  max_toi : @core.Real,
  solid : Bool,
) -> @core.Real? {
  let min_x = center.x - half_width
  let max_x = center.x + half_width
  let min_y = center.y - half_height
  let max_y = center.y + half_height
  let mut tmin = 0.0F
  let mut tmax = max_toi
  if @core.abs(ray.dir.x) <= 1.0e-12F {
    if ray.origin.x < min_x || ray.origin.x > max_x {
      return None
    }
  } else {
    let inv = 1.0F / ray.dir.x
    let mut t1 = (min_x - ray.origin.x) * inv
    let mut t2 = (max_x - ray.origin.x) * inv
    if t1 > t2 {
      let tmp = t1
      t1 = t2
      t2 = tmp
    }
    if t1 > tmin {
      tmin = t1
    }
    if t2 < tmax {
      tmax = t2
    }
    if tmax < tmin {
      return None
    }
  }
  if @core.abs(ray.dir.y) <= 1.0e-12F {
    if ray.origin.y < min_y || ray.origin.y > max_y {
      return None
    }
  } else {
    let inv = 1.0F / ray.dir.y
    let mut t1 = (min_y - ray.origin.y) * inv
    let mut t2 = (max_y - ray.origin.y) * inv
    if t1 > t2 {
      let tmp = t1
      t1 = t2
      t2 = tmp
    }
    if t1 > tmin {
      tmin = t1
    }
    if t2 < tmax {
      tmax = t2
    }
    if tmax < tmin {
      return None
    }
  }

  // For solid queries, a ray starting inside hits at toi = 0.
  if tmin <= 0.0F && tmax >= 0.0F {
    if solid {
      Some(0.0F)
    } else if tmax <= max_toi {
      Some(tmax)
    } else {
      None
    }
  } else if tmin >= 0.0F && tmin <= max_toi {
    Some(tmin)
  } else {
    None
  }
}

///|
fn ray_intersect_cuboid_oriented(
  ray : Ray,
  center : @core.Vec2,
  rotation : @core.Real,
  half_width : @core.Real,
  half_height : @core.Real,
  max_toi : @core.Real,
  solid : Bool,
) -> @core.Real? {
  let rot = @core.Rot2::from_angle(rotation)
  let inv = rot.inverse()
  let local_ray = Ray::new(
    inv.rotate_vec2(ray.origin.sub(center)),
    inv.rotate_vec2(ray.dir),
  )
  // Rotation does not affect the time-of-impact parameter, only the frame.
  ray_intersect_cuboid(
    local_ray,
    @core.Vec2::zero(),
    half_width,
    half_height,
    max_toi,
    solid,
  )
}

///|
pub fn QueryPipeline::cast_ray(
  self : QueryPipeline,
  ray : Ray,
  max_toi : @core.Real,
  solid : Bool,
) -> (ColliderHandle, @core.Real)? {
  let max_dist = if max_toi < 0.0F { 0.0F } else { max_toi }
  let mut best : (ColliderHandle, @core.Real)? = None
  for i in 0..<self.colliders.colliders.length() {
    if self.colliders.colliders[i] is Some(collider) && collider.is_enabled() {
      let handle = ColliderHandle::new(i, self.colliders.generations[i])
      if !query_filter_test(self.filter, self.bodies, handle, collider) {
        continue
      }
      let pos = collider.world_translation
      let rot = collider.world_rotation
      let toi = match collider.shape {
        Shape::Ball(radius) =>
          ray_intersect_ball(ray, pos, radius, max_dist, solid)
        Shape::Cuboid(hw, hh) =>
          ray_intersect_cuboid_oriented(ray, pos, rot, hw, hh, max_dist, solid)
      }
      if toi is Some(hit) {
        if best is Some(current) {
          if hit < current.1 {
            best = Some((handle, hit))
          }
        } else {
          best = Some((handle, hit))
        }
      }
    }
  }
  best
}

///|
fn point_in_shape(
  shape : Shape,
  center : @core.Vec2,
  rotation : @core.Real,
  point : @core.Vec2,
) -> Bool {
  match shape {
    Shape::Ball(radius) => point.sub(center).length_squared() <= radius * radius
    Shape::Cuboid(hw, hh) => {
      let rot = @core.Rot2::from_angle(rotation)
      let inv = rot.inverse()
      let local_point = inv.rotate_vec2(point.sub(center))
      @core.abs(local_point.x) <= hw && @core.abs(local_point.y) <= hh
    }
  }
}

///|
pub fn QueryPipeline::intersect_point(
  self : QueryPipeline,
  point : @core.Vec2,
) -> Array[ColliderHandle] {
  let result : Array[ColliderHandle] = []
  for i in 0..<self.colliders.colliders.length() {
    if self.colliders.colliders[i] is Some(collider) && collider.is_enabled() {
      let handle = ColliderHandle::new(i, self.colliders.generations[i])
      if !query_filter_test(self.filter, self.bodies, handle, collider) {
        continue
      }
      if point_in_shape(
          collider.shape,
          collider.world_translation,
          collider.world_rotation,
          point,
        ) {
        result.push(handle)
      }
    }
  }
  result
}
