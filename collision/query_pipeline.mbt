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
pub struct ShapeCastOptions {
  max_toi : @core.Real
  stop_at_penetration : Bool
}

///|
pub fn ShapeCastOptions::new(
  max_toi : @core.Real,
  stop_at_penetration : Bool,
) -> ShapeCastOptions {
  { max_toi, stop_at_penetration }
}

///|
pub fn ShapeCastOptions::max_toi(self : ShapeCastOptions) -> @core.Real {
  self.max_toi
}

///|
pub fn ShapeCastOptions::stop_at_penetration(self : ShapeCastOptions) -> Bool {
  self.stop_at_penetration
}

///|
pub struct ShapeCastHit {
  toi : @core.Real
  point : @core.Vec2
  normal : @core.Vec2
}

///|
pub fn ShapeCastHit::toi(self : ShapeCastHit) -> @core.Real {
  self.toi
}

///|
pub fn ShapeCastHit::point(self : ShapeCastHit) -> @core.Vec2 {
  self.point
}

///|
pub fn ShapeCastHit::normal(self : ShapeCastHit) -> @core.Vec2 {
  self.normal
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
    let mut axis_enter = @core.Vec2::new(-1.0F, 0.0F)
    let mut axis_exit = @core.Vec2::new(1.0F, 0.0F)
    if t1 > t2 {
      let tmp = t1
      t1 = t2
      t2 = tmp
      let tmp_n = axis_enter
      axis_enter = axis_exit
      axis_exit = tmp_n
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
    let mut axis_enter = @core.Vec2::new(0.0F, -1.0F)
    let mut axis_exit = @core.Vec2::new(0.0F, 1.0F)
    if t1 > t2 {
      let tmp = t1
      t1 = t2
      t2 = tmp
      let tmp_n = axis_enter
      axis_enter = axis_exit
      axis_exit = tmp_n
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
fn ray_intersect_capsule_oriented_and_get_normal(
  ray : Ray,
  center : @core.Vec2,
  rotation : @core.Real,
  axis : @core.Vec2,
  half_height : @core.Real,
  radius : @core.Real,
  core_half_width : @core.Real,
  core_half_height : @core.Real,
  max_toi : @core.Real,
  solid : Bool,
) -> RayIntersection? {
  let rot = @core.Rot2::from_angle(rotation)
  let offset = rot.rotate_vec2(
    @core.Vec2::new(axis.x * half_height, axis.y * half_height),
  )
  let a = center.add(offset)
  let b = center.sub(offset)
  let mut best : RayIntersection? = None
  let hit_a = ray_intersect_ball_and_get_normal(ray, a, radius, max_toi, solid)
  if hit_a is Some(h) {
    best = Some(h)
  }
  let hit_b = ray_intersect_ball_and_get_normal(ray, b, radius, max_toi, solid)
  if hit_b is Some(h) {
    if best is Some(cur) {
      if h.toi < cur.toi {
        best = Some(h)
      }
    } else {
      best = Some(h)
    }
  }
  let hit_core = ray_intersect_cuboid_oriented_and_get_normal(
    ray, center, rotation, core_half_width, core_half_height, max_toi, solid,
  )
  if hit_core is Some(h) {
    if best is Some(cur) {
      if h.toi < cur.toi {
        best = Some(h)
      }
    } else {
      best = Some(h)
    }
  }
  best
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
        Shape::CapsuleX(half_height, radius) =>
          ray_intersect_capsule_oriented_and_get_normal(
            ray,
            pos,
            rot,
            @core.Vec2::new(1.0F, 0.0F),
            half_height,
            radius,
            half_height,
            radius,
            max_dist,
            solid,
          )
        Shape::CapsuleY(half_height, radius) =>
          ray_intersect_capsule_oriented_and_get_normal(
            ray,
            pos,
            rot,
            @core.Vec2::new(0.0F, 1.0F),
            half_height,
            radius,
            radius,
            half_height,
            max_dist,
            solid,
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
fn shape_cast_ball_ball(
  start : @core.Vec2,
  vel : @core.Vec2,
  radius1 : @core.Real,
  center2 : @core.Vec2,
  radius2 : @core.Real,
  max_toi : @core.Real,
  stop_at_penetration : Bool,
) -> ShapeCastHit? {
  let r = radius1 + radius2
  let m = start.sub(center2)
  let a = vel.dot(vel)
  let c = m.dot(m) - r * r
  if c <= 0.0F && stop_at_penetration {
    let normal = normalize_or_fallback(
      @core.Vec2::new(-vel.x, -vel.y),
      @core.Vec2::new(1.0F, 0.0F),
    )
    let point = center2.add(vec2_scale(normal, radius2))
    return Some(ShapeCastHit::{ toi: 0.0F, point, normal })
  }
  if a <= 1.0e-12F {
    return None
  }
  let b = m.dot(vel)
  let disc = b * b - a * c
  if disc < 0.0F {
    return None
  }
  let root = sqrt_real(disc)
  let mut t = (-b - root) / a
  if t < 0.0F {
    t = (-b + root) / a
  }
  if t < 0.0F || t > max_toi {
    return None
  }
  let center1 = start.add(vec2_scale(vel, t))
  let normal = normalize_or_fallback(
    center1.sub(center2),
    @core.Vec2::new(1.0F, 0.0F),
  )
  let point = center2.add(vec2_scale(normal, radius2))
  Some(ShapeCastHit::{ toi: t, point, normal })
}

///|
fn shape_cast_ball_cuboid(
  start : @core.Vec2,
  vel : @core.Vec2,
  radius : @core.Real,
  cuboid_center : @core.Vec2,
  cuboid_rotation : @core.Real,
  half_width : @core.Real,
  half_height : @core.Real,
  max_toi : @core.Real,
  stop_at_penetration : Bool,
) -> ShapeCastHit? {
  let ray = Ray::new(start, vel)
  let expanded_hw = half_width + radius
  let expanded_hh = half_height + radius
  if ray_intersect_cuboid_oriented_and_get_normal(
      ray, cuboid_center, cuboid_rotation, expanded_hw, expanded_hh, max_toi, stop_at_penetration,
    )
    is Some(hit) {
    let center = ray_point(ray, hit.toi)
    let point = center.sub(vec2_scale(hit.normal, radius))
    Some(ShapeCastHit::{ toi: hit.toi, point, normal: hit.normal })
  } else {
    None
  }
}

///|
fn shape_cast_support_point_cuboid(
  center : @core.Vec2,
  rotation : @core.Real,
  half_width : @core.Real,
  half_height : @core.Real,
  dir : @core.Vec2,
) -> @core.Vec2 {
  let (axis_x, axis_y) = cuboid_axes(rotation)
  let sx = if dir.dot(axis_x) >= 0.0F { 1.0F } else { -1.0F }
  let sy = if dir.dot(axis_y) >= 0.0F { 1.0F } else { -1.0F }
  center
  .add(vec2_scale(axis_x, sx * half_width))
  .add(vec2_scale(axis_y, sy * half_height))
}

///|
fn shape_cast_cuboid_ball(
  cuboid_center : @core.Vec2,
  cuboid_rotation : @core.Real,
  half_width : @core.Real,
  half_height : @core.Real,
  vel : @core.Vec2,
  ball_center : @core.Vec2,
  ball_radius : @core.Real,
  max_toi : @core.Real,
  stop_at_penetration : Bool,
) -> ShapeCastHit? {
  let inv_vel = @core.Vec2::new(-vel.x, -vel.y)
  if shape_cast_ball_cuboid(
      ball_center, inv_vel, ball_radius, cuboid_center, cuboid_rotation, half_width,
      half_height, max_toi, stop_at_penetration,
    )
    is Some(hit) {
    let normal = @core.Vec2::new(-hit.normal.x, -hit.normal.y)
    let point = ball_center.add(vec2_scale(normal, ball_radius))
    Some(ShapeCastHit::{ toi: hit.toi, point, normal })
  } else {
    None
  }
}

///|
fn shape_cast_project_interval(
  center : @core.Vec2,
  rotation : @core.Real,
  half_width : @core.Real,
  half_height : @core.Real,
  axis : @core.Vec2,
) -> (@core.Real, @core.Real) {
  let (axis_x, axis_y) = cuboid_axes(rotation)
  let c = center.dot(axis)
  let r = @core.abs(axis.dot(axis_x)) * half_width +
    @core.abs(axis.dot(axis_y)) * half_height
  (c - r, c + r)
}

///|
fn shape_cast_cuboid_cuboid(
  center1 : @core.Vec2,
  rotation1 : @core.Real,
  half_width1 : @core.Real,
  half_height1 : @core.Real,
  vel : @core.Vec2,
  center2 : @core.Vec2,
  rotation2 : @core.Real,
  half_width2 : @core.Real,
  half_height2 : @core.Real,
  max_toi : @core.Real,
  stop_at_penetration : Bool,
) -> ShapeCastHit? {
  let (a_x, a_y) = cuboid_axes(rotation1)
  let (b_x, b_y) = cuboid_axes(rotation2)
  let axes = [a_x, a_y, b_x, b_y]
  let mut best_entry = -1.0e30F
  let mut best_exit = 1.0e30F
  let mut best_axis = @core.Vec2::new(1.0F, 0.0F)
  let mut best_vel_axis = 0.0F
  for axis in axes {
    let (min1, max1) = shape_cast_project_interval(
      center1, rotation1, half_width1, half_height1, axis,
    )
    let (min2, max2) = shape_cast_project_interval(
      center2, rotation2, half_width2, half_height2, axis,
    )
    let vel_axis = vel.dot(axis)
    if @core.abs(vel_axis) <= 1.0e-12F {
      if max1 < min2 || min1 > max2 {
        return None
      }
      continue
    }
    let t1 = (min2 - max1) / vel_axis
    let t2 = (max2 - min1) / vel_axis
    let entry = if t1 < t2 { t1 } else { t2 }
    let exit = if t1 > t2 { t1 } else { t2 }
    if entry > best_entry {
      best_entry = entry
      best_axis = axis
      best_vel_axis = vel_axis
    }
    if exit < best_exit {
      best_exit = exit
    }
    if best_entry > best_exit {
      return None
    }
  }
  if best_exit < 0.0F {
    return None
  }
  let toi = if best_entry <= 0.0F {
    if stop_at_penetration {
      0.0F
    } else {
      0.0F
    }
  } else {
    best_entry
  }
  if toi > max_toi {
    return None
  }
  let normal = if best_vel_axis > 1.0e-12F {
    @core.Vec2::new(-best_axis.x, -best_axis.y)
  } else if best_vel_axis < -1.0e-12F {
    best_axis
  } else {
    normalize_or_fallback(
      @core.Vec2::new(-vel.x, -vel.y),
      @core.Vec2::new(1.0F, 0.0F),
    )
  }
  let point = shape_cast_support_point_cuboid(
    center2, rotation2, half_width2, half_height2, normal,
  )
  Some(ShapeCastHit::{ toi, point, normal })
}

///|
fn capsule_components_static(
  center : @core.Vec2,
  rotation : @core.Real,
  axis : @core.Vec2,
  half_height : @core.Real,
  radius : @core.Real,
  core_half_width : @core.Real,
  core_half_height : @core.Real,
) -> Array[(Shape, @core.Vec2, @core.Real)] {
  let rot = @core.Rot2::from_angle(rotation)
  let offset = rot.rotate_vec2(
    @core.Vec2::new(axis.x * half_height, axis.y * half_height),
  )
  let a = center.add(offset)
  let b = center.sub(offset)
  [
    (Shape::Ball(radius), a, 0.0F),
    (Shape::Ball(radius), b, 0.0F),
    (Shape::Cuboid(core_half_width, core_half_height), center, rotation),
  ]
}

///|
fn capsule_components_moving(
  shape_pos : @core.Isometry2,
  axis : @core.Vec2,
  half_height : @core.Real,
  radius : @core.Real,
  core_half_width : @core.Real,
  core_half_height : @core.Real,
) -> Array[(Shape, @core.Isometry2)] {
  let rot = shape_pos.rotation
  let offset = rot.rotate_vec2(
    @core.Vec2::new(axis.x * half_height, axis.y * half_height),
  )
  let a = shape_pos.translation.add(offset)
  let b = shape_pos.translation.sub(offset)
  [
    (Shape::Ball(radius), @core.Isometry2::from_translation(a)),
    (Shape::Ball(radius), @core.Isometry2::from_translation(b)),
    (Shape::Cuboid(core_half_width, core_half_height), shape_pos),
  ]
}

///|
fn shape_cast_basic(
  moving_pos : @core.Isometry2,
  vel : @core.Vec2,
  moving_shape : Shape,
  static_center : @core.Vec2,
  static_rotation : @core.Real,
  static_shape : Shape,
  max_toi : @core.Real,
  stop_at_penetration : Bool,
) -> ShapeCastHit? {
  match (moving_shape, static_shape) {
    (Shape::Ball(r1), Shape::Ball(r2)) =>
      shape_cast_ball_ball(
        moving_pos.translation,
        vel,
        r1,
        static_center,
        r2,
        max_toi,
        stop_at_penetration,
      )
    (Shape::Ball(r), Shape::Cuboid(hw, hh)) =>
      shape_cast_ball_cuboid(
        moving_pos.translation,
        vel,
        r,
        static_center,
        static_rotation,
        hw,
        hh,
        max_toi,
        stop_at_penetration,
      )
    (Shape::Cuboid(hw, hh), Shape::Ball(r)) =>
      shape_cast_cuboid_ball(
        moving_pos.translation,
        moving_pos.rotation.angle(),
        hw,
        hh,
        vel,
        static_center,
        r,
        max_toi,
        stop_at_penetration,
      )
    (Shape::Cuboid(hw1, hh1), Shape::Cuboid(hw2, hh2)) =>
      shape_cast_cuboid_cuboid(
        moving_pos.translation,
        moving_pos.rotation.angle(),
        hw1,
        hh1,
        vel,
        static_center,
        static_rotation,
        hw2,
        hh2,
        max_toi,
        stop_at_penetration,
      )
    _ => None
  }
}

///|
fn shape_cast_against_capsule(
  moving_pos : @core.Isometry2,
  vel : @core.Vec2,
  moving_shape : Shape,
  capsule_center : @core.Vec2,
  capsule_rotation : @core.Real,
  axis : @core.Vec2,
  half_height : @core.Real,
  radius : @core.Real,
  core_half_width : @core.Real,
  core_half_height : @core.Real,
  max_toi : @core.Real,
  stop_at_penetration : Bool,
) -> ShapeCastHit? {
  let components = capsule_components_static(
    capsule_center, capsule_rotation, axis, half_height, radius, core_half_width,
    core_half_height,
  )
  let mut best : ShapeCastHit? = None
  for i in 0..<components.length() {
    let c = components[i]
    if shape_cast_basic(
        moving_pos,
        vel,
        moving_shape,
        c.1,
        c.2,
        c.0,
        max_toi,
        stop_at_penetration,
      )
      is Some(hit) {
      if best is Some(current) {
        if hit.toi < current.toi {
          best = Some(hit)
        }
      } else {
        best = Some(hit)
      }
    }
  }
  best
}

///|
pub fn QueryPipeline::cast_shape(
  self : QueryPipeline,
  shape_pos : @core.Isometry2,
  shape_vel : @core.Vec2,
  shape : Shape,
  options : ShapeCastOptions,
) -> (ColliderHandle, ShapeCastHit)? {
  let max_toi = if options.max_toi < 0.0F { 0.0F } else { options.max_toi }
  let mut best : (ColliderHandle, ShapeCastHit)? = None
  for i in 0..<self.colliders.colliders.length() {
    if self.colliders.colliders[i] is Some(collider) && collider.is_enabled() {
      let handle = ColliderHandle::new(i, self.colliders.generations[i])
      if !query_filter_test(self.filter, self.bodies, handle, collider) {
        continue
      }
      let hit = match (shape, collider.shape) {
        (Shape::Ball(_), Shape::Ball(_))
        | (Shape::Ball(_), Shape::Cuboid(_, _))
        | (Shape::Cuboid(_, _), Shape::Ball(_))
        | (Shape::Cuboid(_, _), Shape::Cuboid(_, _)) =>
          shape_cast_basic(
            shape_pos,
            shape_vel,
            shape,
            collider.world_translation,
            collider.world_rotation,
            collider.shape,
            max_toi,
            options.stop_at_penetration,
          )
        (Shape::Ball(_), Shape::CapsuleX(h, r)) =>
          shape_cast_against_capsule(
            shape_pos,
            shape_vel,
            shape,
            collider.world_translation,
            collider.world_rotation,
            @core.Vec2::new(1.0F, 0.0F),
            h,
            r,
            h,
            r,
            max_toi,
            options.stop_at_penetration,
          )
        (Shape::Ball(_), Shape::CapsuleY(h, r)) =>
          shape_cast_against_capsule(
            shape_pos,
            shape_vel,
            shape,
            collider.world_translation,
            collider.world_rotation,
            @core.Vec2::new(0.0F, 1.0F),
            h,
            r,
            r,
            h,
            max_toi,
            options.stop_at_penetration,
          )
        (Shape::Cuboid(_, _), Shape::CapsuleX(h, r)) =>
          shape_cast_against_capsule(
            shape_pos,
            shape_vel,
            shape,
            collider.world_translation,
            collider.world_rotation,
            @core.Vec2::new(1.0F, 0.0F),
            h,
            r,
            h,
            r,
            max_toi,
            options.stop_at_penetration,
          )
        (Shape::Cuboid(_, _), Shape::CapsuleY(h, r)) =>
          shape_cast_against_capsule(
            shape_pos,
            shape_vel,
            shape,
            collider.world_translation,
            collider.world_rotation,
            @core.Vec2::new(0.0F, 1.0F),
            h,
            r,
            r,
            h,
            max_toi,
            options.stop_at_penetration,
          )
        (Shape::CapsuleX(h, r), target) => {
          let moving_components = capsule_components_moving(
            shape_pos,
            @core.Vec2::new(1.0F, 0.0F),
            h,
            r,
            h,
            r,
          )
          let mut best_hit : ShapeCastHit? = None
          for ci in 0..<moving_components.length() {
            let comp = moving_components[ci]
            let comp_shape = comp.0
            let comp_pos = comp.1
            let h2 = match target {
              Shape::CapsuleX(th, tr) =>
                shape_cast_against_capsule(
                  comp_pos,
                  shape_vel,
                  comp_shape,
                  collider.world_translation,
                  collider.world_rotation,
                  @core.Vec2::new(1.0F, 0.0F),
                  th,
                  tr,
                  th,
                  tr,
                  max_toi,
                  options.stop_at_penetration,
                )
              Shape::CapsuleY(th, tr) =>
                shape_cast_against_capsule(
                  comp_pos,
                  shape_vel,
                  comp_shape,
                  collider.world_translation,
                  collider.world_rotation,
                  @core.Vec2::new(0.0F, 1.0F),
                  th,
                  tr,
                  tr,
                  th,
                  max_toi,
                  options.stop_at_penetration,
                )
              _ =>
                shape_cast_basic(
                  comp_pos,
                  shape_vel,
                  comp_shape,
                  collider.world_translation,
                  collider.world_rotation,
                  target,
                  max_toi,
                  options.stop_at_penetration,
                )
            }
            if h2 is Some(hit2) {
              if best_hit is Some(cur) {
                if hit2.toi < cur.toi {
                  best_hit = Some(hit2)
                }
              } else {
                best_hit = Some(hit2)
              }
            }
          }
          best_hit
        }
        (Shape::CapsuleY(h, r), target) => {
          let moving_components = capsule_components_moving(
            shape_pos,
            @core.Vec2::new(0.0F, 1.0F),
            h,
            r,
            r,
            h,
          )
          let mut best_hit : ShapeCastHit? = None
          for ci in 0..<moving_components.length() {
            let comp = moving_components[ci]
            let comp_shape = comp.0
            let comp_pos = comp.1
            let h2 = match target {
              Shape::CapsuleX(th, tr) =>
                shape_cast_against_capsule(
                  comp_pos,
                  shape_vel,
                  comp_shape,
                  collider.world_translation,
                  collider.world_rotation,
                  @core.Vec2::new(1.0F, 0.0F),
                  th,
                  tr,
                  th,
                  tr,
                  max_toi,
                  options.stop_at_penetration,
                )
              Shape::CapsuleY(th, tr) =>
                shape_cast_against_capsule(
                  comp_pos,
                  shape_vel,
                  comp_shape,
                  collider.world_translation,
                  collider.world_rotation,
                  @core.Vec2::new(0.0F, 1.0F),
                  th,
                  tr,
                  tr,
                  th,
                  max_toi,
                  options.stop_at_penetration,
                )
              _ =>
                shape_cast_basic(
                  comp_pos,
                  shape_vel,
                  comp_shape,
                  collider.world_translation,
                  collider.world_rotation,
                  target,
                  max_toi,
                  options.stop_at_penetration,
                )
            }
            if h2 is Some(hit2) {
              if best_hit is Some(cur) {
                if hit2.toi < cur.toi {
                  best_hit = Some(hit2)
                }
              } else {
                best_hit = Some(hit2)
              }
            }
          }
          best_hit
        }
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
pub fn QueryPipeline::collider(
  self : QueryPipeline,
  handle : ColliderHandle,
) -> Collider? {
  self.colliders.get(handle)
}

///|
pub fn QueryPipeline::rigid_body(
  self : QueryPipeline,
  handle : @dynamics.RigidBodyHandle,
) -> @dynamics.RigidBody? {
  self.bodies.get(handle)
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
    Shape::CapsuleX(half_height, radius) => {
      let rot = @core.Rot2::from_angle(rotation)
      let inv = rot.inverse()
      let local_point = inv.rotate_vec2(point.sub(center))
      let cx = clamp_value(local_point.x, -half_height, half_height)
      let delta = @core.Vec2::new(local_point.x - cx, local_point.y)
      delta.dot(delta) <= radius * radius
    }
    Shape::CapsuleY(half_height, radius) => {
      let rot = @core.Rot2::from_angle(rotation)
      let inv = rot.inverse()
      let local_point = inv.rotate_vec2(point.sub(center))
      let cy = clamp_value(local_point.y, -half_height, half_height)
      let delta = @core.Vec2::new(local_point.x, local_point.y - cy)
      delta.dot(delta) <= radius * radius
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
    Shape::CapsuleX(half_height, radius) => {
      let rot = @core.Rot2::from_angle(collider.world_rotation)
      let offset = rot.rotate_vec2(@core.Vec2::new(half_height, 0.0F))
      let c = collider.world_translation
      let a = c.add(offset)
      let b = c.sub(offset)
      let min_x = if a.x < b.x { a.x } else { b.x }
      let max_x = if a.x > b.x { a.x } else { b.x }
      let min_y = if a.y < b.y { a.y } else { b.y }
      let max_y = if a.y > b.y { a.y } else { b.y }
      @core.Aabb::new(
        @core.Vec2::new(min_x - radius, min_y - radius),
        @core.Vec2::new(max_x + radius, max_y + radius),
      )
    }
    Shape::CapsuleY(half_height, radius) => {
      let rot = @core.Rot2::from_angle(collider.world_rotation)
      let offset = rot.rotate_vec2(@core.Vec2::new(0.0F, half_height))
      let c = collider.world_translation
      let a = c.add(offset)
      let b = c.sub(offset)
      let min_x = if a.x < b.x { a.x } else { b.x }
      let max_x = if a.x > b.x { a.x } else { b.x }
      let min_y = if a.y < b.y { a.y } else { b.y }
      let max_y = if a.y > b.y { a.y } else { b.y }
      @core.Aabb::new(
        @core.Vec2::new(min_x - radius, min_y - radius),
        @core.Vec2::new(max_x + radius, max_y + radius),
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
fn project_point_on_capsule_oriented(
  center : @core.Vec2,
  rotation : @core.Real,
  axis : @core.Vec2,
  half_height : @core.Real,
  radius : @core.Real,
  point : @core.Vec2,
  solid : Bool,
) -> PointProjection {
  let rot = @core.Rot2::from_angle(rotation)
  let inv = rot.inverse()
  let local_point = inv.rotate_vec2(point.sub(center))
  let closest_line_point = if @core.abs(axis.x) > @core.abs(axis.y) {
    let cx = clamp_value(local_point.x, -half_height, half_height)
    @core.Vec2::new(cx, 0.0F)
  } else {
    let cy = clamp_value(local_point.y, -half_height, half_height)
    @core.Vec2::new(0.0F, cy)
  }
  let delta = local_point.sub(closest_line_point)
  let dist2 = delta.dot(delta)
  let inside = dist2 <= radius * radius
  if inside && solid {
    return PointProjection::{ point, is_inside: true }
  }
  let projected_local = if dist2 <= 1.0e-12F {
    // Arbitrary direction (consistent with project_point_on_ball fallback).
    closest_line_point.add(@core.Vec2::new(radius, 0.0F))
  } else {
    closest_line_point.add(vec2_scale(delta, radius / Float::sqrt(dist2)))
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
        Shape::CapsuleX(half_height, radius) =>
          project_point_on_capsule_oriented(
            collider.world_translation,
            collider.world_rotation,
            @core.Vec2::new(1.0F, 0.0F),
            half_height,
            radius,
            point,
            solid,
          )
        Shape::CapsuleY(half_height, radius) =>
          project_point_on_capsule_oriented(
            collider.world_translation,
            collider.world_rotation,
            @core.Vec2::new(0.0F, 1.0F),
            half_height,
            radius,
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
