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
  // Cached AABBs at the time this pipeline is created. This matches Rapier's
  // "conservative" AABB queries which do not recompute the latest collider AABB.
  aabbs : Array[@core.Aabb?]
}

///|
pub struct RayIntersection {
  toi : @core.Real
  normal : @core.Vec2
}

///|
pub fn RayIntersection::toi(self : RayIntersection) -> @core.Real {
  self.toi
}

///|
pub fn RayIntersection::normal(self : RayIntersection) -> @core.Vec2 {
  self.normal
}

///|
pub struct PointProjection {
  point : @core.Vec2
  is_inside : Bool
}

///|
pub fn PointProjection::point(self : PointProjection) -> @core.Vec2 {
  self.point
}

///|
pub fn PointProjection::is_inside(self : PointProjection) -> Bool {
  self.is_inside
}

///|
pub fn QueryPipeline::new(
  filter : QueryFilter,
  bodies : @dynamics.RigidBodySet,
  colliders : ColliderSet,
) -> QueryPipeline {
  let aabbs : Array[@core.Aabb?] = []
  for i in 0..<colliders.colliders.length() {
    if colliders.colliders[i] is Some(collider) && collider.is_enabled() {
      aabbs.push(Some(compute_collider_aabb(collider)))
    } else {
      aabbs.push(None)
    }
  }
  { filter, bodies, colliders, aabbs }
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
fn vec2_scale(v : @core.Vec2, s : @core.Real) -> @core.Vec2 {
  @core.Vec2::new(v.x * s, v.y * s)
}

///|
fn ray_point(ray : Ray, toi : @core.Real) -> @core.Vec2 {
  ray.origin.add(vec2_scale(ray.dir, toi))
}

///|
fn normalize_or_fallback(v : @core.Vec2, fallback : @core.Vec2) -> @core.Vec2 {
  let len2 = v.dot(v)
  if len2 <= 1.0e-12F {
    fallback
  } else {
    v.normalize()
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
fn ray_intersect_ball_and_get_normal(
  ray : Ray,
  center : @core.Vec2,
  radius : @core.Real,
  max_toi : @core.Real,
  solid : Bool,
) -> RayIntersection? {
  let d = ray.dir
  let m = ray.origin.sub(center)
  let a = d.dot(d)
  if a <= 1.0e-12F {
    return None
  }
  let c = m.dot(m) - radius * radius
  let inside = c <= 0.0F
  if ray_intersect_ball(ray, center, radius, max_toi, solid) is Some(hit) {
    let normal = if hit <= 1.0e-12F && inside && solid {
      normalize_or_fallback(
        @core.Vec2::new(-ray.dir.x, -ray.dir.y),
        @core.Vec2::new(1.0F, 0.0F),
      )
    } else {
      let p = ray_point(ray, hit)
      normalize_or_fallback(p.sub(center), @core.Vec2::new(1.0F, 0.0F))
    }
    Some(RayIntersection::{ toi: hit, normal })
  } else {
    None
  }
}

///|
fn ray_intersect_cuboid_local_and_get_normal(
  ray : Ray,
  half_width : @core.Real,
  half_height : @core.Real,
  max_toi : @core.Real,
  solid : Bool,
) -> RayIntersection? {
  let min_x = -half_width
  let max_x = half_width
  let min_y = -half_height
  let max_y = half_height
  let mut tmin = 0.0F
  let mut tmax = max_toi
  let mut enter_normal = @core.Vec2::new(0.0F, 0.0F)
  let mut exit_normal = @core.Vec2::new(0.0F, 0.0F)
  if @core.abs(ray.dir.x) <= 1.0e-12F {
    if ray.origin.x < min_x || ray.origin.x > max_x {
      return None
    }
  } else {
    let inv = 1.0F / ray.dir.x
    let mut t1 = (min_x - ray.origin.x) * inv
    let mut t2 = (max_x - ray.origin.x) * inv
    let near_normal = if ray.dir.x > 0.0F {
      @core.Vec2::new(-1.0F, 0.0F)
    } else {
      @core.Vec2::new(1.0F, 0.0F)
    }
    let far_normal = @core.Vec2::new(-near_normal.x, -near_normal.y)
    let mut axis_enter = near_normal
    let mut axis_exit = far_normal
    if t1 > t2 {
      let tmp = t1
      t1 = t2
      t2 = tmp
      axis_enter = far_normal
      axis_exit = near_normal
    }
    if t1 > tmin {
      tmin = t1
      enter_normal = axis_enter
    }
    if t2 < tmax {
      tmax = t2
      exit_normal = axis_exit
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
    let near_normal = if ray.dir.y > 0.0F {
      @core.Vec2::new(0.0F, -1.0F)
    } else {
      @core.Vec2::new(0.0F, 1.0F)
    }
    let far_normal = @core.Vec2::new(-near_normal.x, -near_normal.y)
    let mut axis_enter = near_normal
    let mut axis_exit = far_normal
    if t1 > t2 {
      let tmp = t1
      t1 = t2
      t2 = tmp
      axis_enter = far_normal
      axis_exit = near_normal
    }
    if t1 > tmin {
      tmin = t1
      enter_normal = axis_enter
    }
    if t2 < tmax {
      tmax = t2
      exit_normal = axis_exit
    }
    if tmax < tmin {
      return None
    }
  }
  if tmin <= 0.0F && tmax >= 0.0F {
    if solid {
      let normal = normalize_or_fallback(
        @core.Vec2::new(-ray.dir.x, -ray.dir.y),
        @core.Vec2::new(1.0F, 0.0F),
      )
      Some(RayIntersection::{ toi: 0.0F, normal })
    } else if tmax <= max_toi {
      Some(RayIntersection::{ toi: tmax, normal: exit_normal })
    } else {
      None
    }
  } else if tmin >= 0.0F && tmin <= max_toi {
    Some(RayIntersection::{ toi: tmin, normal: enter_normal })
  } else {
    None
  }
}

///|
fn ray_intersect_cuboid_oriented_and_get_normal(
  ray : Ray,
  center : @core.Vec2,
  rotation : @core.Real,
  half_width : @core.Real,
  half_height : @core.Real,
  max_toi : @core.Real,
  solid : Bool,
) -> RayIntersection? {
  let rot = @core.Rot2::from_angle(rotation)
  let inv = rot.inverse()
  let local_ray = Ray::new(
    inv.rotate_vec2(ray.origin.sub(center)),
    inv.rotate_vec2(ray.dir),
  )
  if ray_intersect_cuboid_local_and_get_normal(
      local_ray, half_width, half_height, max_toi, solid,
    )
    is Some(hit) {
    Some(RayIntersection::{ toi: hit.toi, normal: rot.rotate_vec2(hit.normal) })
  } else {
    None
  }
}

///|
pub fn QueryPipeline::cast_ray(
  self : QueryPipeline,
  ray : Ray,
  max_toi : @core.Real,
  solid : Bool,
) -> (ColliderHandle, @core.Real)? {
  if self.cast_ray_and_get_normal(ray, max_toi, solid) is Some(hit) {
    Some((hit.0, hit.1.toi))
  } else {
    None
  }
}

///|
pub fn QueryPipeline::cast_ray_and_get_normal(
  self : QueryPipeline,
  ray : Ray,
  max_toi : @core.Real,
  solid : Bool,
) -> (ColliderHandle, RayIntersection)? {
  let max_dist = if max_toi < 0.0F { 0.0F } else { max_toi }
  let mut best : (ColliderHandle, RayIntersection)? = None
  for i in 0..<self.colliders.colliders.length() {
    if self.colliders.colliders[i] is Some(collider) && collider.is_enabled() {
      let handle = ColliderHandle::new(i, self.colliders.generations[i])
      if !query_filter_test(self.filter, self.bodies, handle, collider) {
        continue
      }
      let pos = collider.world_translation
      let rot = collider.world_rotation
      let hit = match collider.shape {
        Shape::Ball(radius) =>
          ray_intersect_ball_and_get_normal(ray, pos, radius, max_dist, solid)
        Shape::Cuboid(hw, hh) =>
          ray_intersect_cuboid_oriented_and_get_normal(
            ray, pos, rot, hw, hh, max_dist, solid,
          )
      }
      if hit is Some(intersection) {
        if best is Some(current) {
          if intersection.toi < current.1.toi {
            best = Some((handle, intersection))
          }
        } else {
          best = Some((handle, intersection))
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
fn compute_collider_aabb(collider : Collider) -> @core.Aabb {
  match collider.shape {
    Shape::Ball(radius) => {
      let c = collider.world_translation
      @core.Aabb::new(
        @core.Vec2::new(c.x - radius, c.y - radius),
        @core.Vec2::new(c.x + radius, c.y + radius),
      )
    }
    Shape::Cuboid(half_width, half_height) => {
      let rot = @core.Rot2::from_angle(collider.world_rotation)
      let abs_cos = @core.abs(rot.cos)
      let abs_sin = @core.abs(rot.sin)
      let hw = abs_cos * half_width + abs_sin * half_height
      let hh = abs_sin * half_width + abs_cos * half_height
      let c = collider.world_translation
      @core.Aabb::new(
        @core.Vec2::new(c.x - hw, c.y - hh),
        @core.Vec2::new(c.x + hw, c.y + hh),
      )
    }
  }
}

///|
pub fn QueryPipeline::intersect_aabb_conservative(
  self : QueryPipeline,
  aabb : @core.Aabb,
) -> Array[ColliderHandle] {
  let result : Array[ColliderHandle] = []
  for i in 0..<self.colliders.colliders.length() {
    if self.colliders.colliders[i] is Some(collider) && collider.is_enabled() {
      let handle = ColliderHandle::new(i, self.colliders.generations[i])
      if !query_filter_test(self.filter, self.bodies, handle, collider) {
        continue
      }
      // Use the cached AABB from pipeline creation time (Rapier parity).
      if i < self.aabbs.length() &&
        self.aabbs[i] is Some(co_aabb) &&
        co_aabb.intersects(aabb) {
        result.push(handle)
      }
    }
  }
  result
}

///|
fn project_point_on_ball(
  center : @core.Vec2,
  radius : @core.Real,
  point : @core.Vec2,
  solid : Bool,
) -> PointProjection {
  let v = point.sub(center)
  let dist2 = v.dot(v)
  let inside = dist2 <= radius * radius
  if inside && solid {
    PointProjection::{ point, is_inside: true }
  } else if dist2 <= 1.0e-12F {
    PointProjection::{
      point: center.add(@core.Vec2::new(radius, 0.0F)),
      is_inside: inside,
    }
  } else {
    let dir = v.normalize()
    PointProjection::{
      point: center.add(vec2_scale(dir, radius)),
      is_inside: inside,
    }
  }
}

///|
fn project_point_on_cuboid_oriented(
  center : @core.Vec2,
  rotation : @core.Real,
  half_width : @core.Real,
  half_height : @core.Real,
  point : @core.Vec2,
  solid : Bool,
) -> PointProjection {
  let rot = @core.Rot2::from_angle(rotation)
  let inv = rot.inverse()
  let local_point = inv.rotate_vec2(point.sub(center))
  let inside = @core.abs(local_point.x) <= half_width &&
    @core.abs(local_point.y) <= half_height
  if inside && solid {
    return PointProjection::{ point, is_inside: true }
  }
  let projected_local = if inside {
    let dx = half_width - @core.abs(local_point.x)
    let dy = half_height - @core.abs(local_point.y)
    if dx < dy {
      let sign = if local_point.x < 0.0F { -1.0F } else { 1.0F }
      @core.Vec2::new(sign * half_width, local_point.y)
    } else {
      let sign = if local_point.y < 0.0F { -1.0F } else { 1.0F }
      @core.Vec2::new(local_point.x, sign * half_height)
    }
  } else {
    @core.Vec2::new(
      clamp_value(local_point.x, -half_width, half_width),
      clamp_value(local_point.y, -half_height, half_height),
    )
  }
  let projected = center.add(rot.rotate_vec2(projected_local))
  PointProjection::{ point: projected, is_inside: inside }
}

///|
pub fn QueryPipeline::project_point(
  self : QueryPipeline,
  point : @core.Vec2,
  max_dist : @core.Real,
  solid : Bool,
) -> (ColliderHandle, PointProjection)? {
  // Rapier keeps this parameter for API consistency, but doesn't use it.
  max_dist |> ignore
  let mut best : (ColliderHandle, PointProjection, @core.Real)? = None
  for i in 0..<self.colliders.colliders.length() {
    if self.colliders.colliders[i] is Some(collider) && collider.is_enabled() {
      let handle = ColliderHandle::new(i, self.colliders.generations[i])
      if !query_filter_test(self.filter, self.bodies, handle, collider) {
        continue
      }
      let projection = match collider.shape {
        Shape::Ball(radius) =>
          project_point_on_ball(
            collider.world_translation,
            radius,
            point,
            solid,
          )
        Shape::Cuboid(hw, hh) =>
          project_point_on_cuboid_oriented(
            collider.world_translation,
            collider.world_rotation,
            hw,
            hh,
            point,
            solid,
          )
      }
      let delta = point.sub(projection.point)
      let dist2 = delta.dot(delta)
      if best is Some(current) {
        if dist2 < current.2 {
          best = Some((handle, projection, dist2))
        }
      } else {
        best = Some((handle, projection, dist2))
      }
    }
  }
  if best is Some(hit) {
    Some((hit.0, hit.1))
  } else {
    None
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
