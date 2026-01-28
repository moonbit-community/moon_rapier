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
  mut predicate : ((ColliderHandle, Collider) -> Bool)?
}

///|
pub fn QueryFilter::new() -> QueryFilter {
  {
    excluded_body: None,
    excluded_collider: None,
    groups: None,
    flags: QueryFilterFlags::empty(),
    predicate: None,
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
    predicate: self.predicate,
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
    predicate: self.predicate,
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
pub fn QueryFilter::predicate(
  self : QueryFilter,
  predicate : (ColliderHandle, Collider) -> Bool,
) -> QueryFilter {
  self.predicate = Some(predicate)
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
  if filter.predicate is Some(pred) {
    if !pred(handle, collider) {
      return false
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
pub(all) enum FeatureId {
  Unknown
  Vertex(Int)
  Edge(Int)
  Face(Int)
}

///|
pub struct ShapeCastOptions {
  max_toi : @core.Real
  target_distance : @core.Real
  stop_at_penetration : Bool
}

///|
pub fn ShapeCastOptions::new(
  max_toi : @core.Real,
  stop_at_penetration : Bool,
) -> ShapeCastOptions {
  { max_toi, target_distance: 0.0F, stop_at_penetration }
}

///|
pub fn ShapeCastOptions::max_toi(self : ShapeCastOptions) -> @core.Real {
  self.max_toi
}

///|
pub fn ShapeCastOptions::target_distance(self : ShapeCastOptions) -> @core.Real {
  self.target_distance
}

///|
pub fn ShapeCastOptions::stop_at_penetration(self : ShapeCastOptions) -> Bool {
  self.stop_at_penetration
}

///|
pub fn ShapeCastOptions::with_target_distance(
  self : ShapeCastOptions,
  target_distance : @core.Real,
) -> ShapeCastOptions {
  let td = if target_distance < 0.0F { 0.0F } else { target_distance }
  {
    max_toi: self.max_toi,
    target_distance: td,
    stop_at_penetration: self.stop_at_penetration,
  }
}

///|
pub struct NonlinearRigidMotion {
  start_pos : @core.Isometry2
  linvel : @core.Vec2
  angvel : @core.Real
}

///|
pub fn NonlinearRigidMotion::identity() -> NonlinearRigidMotion {
  {
    start_pos: @core.Isometry2::identity(),
    linvel: @core.Vec2::zero(),
    angvel: 0.0F,
  }
}

///|
pub fn NonlinearRigidMotion::new(
  start_pos : @core.Isometry2,
  linvel : @core.Vec2,
  angvel : @core.Real,
) -> NonlinearRigidMotion {
  { start_pos, linvel, angvel }
}

///|
fn NonlinearRigidMotion::pose_at_time(
  self : NonlinearRigidMotion,
  time : @core.Real,
) -> @core.Isometry2 {
  let translation = self.start_pos.translation.add(
    vec2_scale(self.linvel, time),
  )
  let rot = self.start_pos.rotation.mul(
    @core.Rot2::from_angle(self.angvel * time),
  )
  @core.Isometry2::new(translation, rot)
}

///|
fn NonlinearRigidMotion::point_velocity_at_time(
  self : NonlinearRigidMotion,
  time : @core.Real,
  point : @core.Vec2,
) -> @core.Vec2 {
  let pose = self.pose_at_time(time)
  let r = point.sub(pose.translation)
  let rot_vel = @core.Vec2::new(-self.angvel * r.y, self.angvel * r.x)
  self.linvel.add(rot_vel)
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
pub fn QueryPipeline::with_filter(
  self : QueryPipeline,
  filter : QueryFilter,
) -> QueryPipeline {
  { filter, bodies: self.bodies, colliders: self.colliders, aabbs: self.aabbs }
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
fn cross2(a : @core.Vec2, b : @core.Vec2) -> @core.Real {
  a.x * b.y - a.y * b.x
}

///|
fn atan2_real(y : @core.Real, x : @core.Real) -> @core.Real {
  Float::from_double(@math.atan2(y.to_double(), x.to_double()))
}

///|
fn closest_point_on_segment(
  a : @core.Vec2,
  b : @core.Vec2,
  p : @core.Vec2,
) -> @core.Vec2 {
  let ab = b.sub(a)
  let denom = ab.dot(ab)
  if denom <= 1.0e-12F {
    a
  } else {
    let t = clamp_value(p.sub(a).dot(ab) / denom, 0.0F, 1.0F)
    a.add(vec2_scale(ab, t))
  }
}

///|
fn segment_world_endpoints(
  center : @core.Vec2,
  rotation : @core.Real,
  a : @core.Vec2,
  b : @core.Vec2,
) -> (@core.Vec2, @core.Vec2) {
  let rot = @core.Rot2::from_angle(rotation)
  (center.add(rot.rotate_vec2(a)), center.add(rot.rotate_vec2(b)))
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
fn ray_intersect_segment_and_get_normal(
  ray : Ray,
  center : @core.Vec2,
  rotation : @core.Real,
  a : @core.Vec2,
  b : @core.Vec2,
  max_toi : @core.Real,
  solid : Bool,
) -> RayIntersection? {
  let (sa, sb) = segment_world_endpoints(center, rotation, a, b)
  let s = sb.sub(sa)
  let denom = cross2(ray.dir, s)

  // Handle rays starting on the segment for solid casts (Rapier-like behavior).
  let q0 = closest_point_on_segment(sa, sb, ray.origin)
  if solid && ray.origin.sub(q0).dot(ray.origin.sub(q0)) <= 1.0e-12F {
    let normal = normalize_or_fallback(
      @core.Vec2::new(-ray.dir.x, -ray.dir.y),
      @core.Vec2::new(1.0F, 0.0F),
    )
    return Some(RayIntersection::{ toi: 0.0F, normal })
  }
  if @core.abs(denom) <= 1.0e-12F {
    return None
  }
  let qp = sa.sub(ray.origin)
  let t = cross2(qp, s) / denom
  let u = cross2(qp, ray.dir) / denom
  if t < 0.0F || t > max_toi || u < 0.0F || u > 1.0F {
    return None
  }
  let mut normal = normalize_or_fallback(
    @core.Vec2::new(-s.y, s.x),
    @core.Vec2::new(0.0F, 1.0F),
  )
  if normal.dot(ray.dir) > 0.0F {
    normal = @core.Vec2::new(-normal.x, -normal.y)
  }
  Some(RayIntersection::{ toi: t, normal })
}

///|
fn ray_intersect_convex_polygon_local_and_get_normal(
  ray : Ray,
  vertices : Array[@core.Vec2],
  max_toi : @core.Real,
  solid : Bool,
) -> RayIntersection? {
  if vertices.length() < 3 {
    return None
  }
  // Half-space intersection (Liang–Barsky style) against a CCW convex polygon.
  let mut tmin = 0.0F
  let mut tmax = max_toi
  let mut enter_normal = @core.Vec2::new(1.0F, 0.0F)
  let mut exit_normal = @core.Vec2::new(1.0F, 0.0F)
  for i in 0..<vertices.length() {
    let a = vertices[i]
    let b = vertices[(i + 1) % vertices.length()]
    let e = b.sub(a)
    // Outward normal for CCW polygon.
    let n = normalize_or_fallback(
      @core.Vec2::new(e.y, -e.x),
      @core.Vec2::new(1.0F, 0.0F),
    )
    let num = n.dot(ray.origin.sub(a))
    let den = n.dot(ray.dir)
    if @core.abs(den) <= 1.0e-12F {
      if num > 0.0F {
        return None
      }
      continue
    }
    let t = -num / den
    if den > 0.0F {
      // Exiting the half-space.
      if t < tmax {
        tmax = t
        exit_normal = n
      }
      // Entering the half-space.
    } else if t > tmin {
      tmin = t
      enter_normal = n
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
fn ray_intersect_convex_polygon_oriented_and_get_normal(
  ray : Ray,
  center : @core.Vec2,
  rotation : @core.Real,
  vertices : Array[@core.Vec2],
  max_toi : @core.Real,
  solid : Bool,
) -> RayIntersection? {
  let rot = @core.Rot2::from_angle(rotation)
  let inv = rot.inverse()
  let local_ray = Ray::new(
    inv.rotate_vec2(ray.origin.sub(center)),
    inv.rotate_vec2(ray.dir),
  )
  if ray_intersect_convex_polygon_local_and_get_normal(
      local_ray, vertices, max_toi, solid,
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
      let hit = ray_intersect_shape_and_get_normal(
        ray,
        pos,
        rot,
        collider.shape,
        max_dist,
        solid,
      )
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
pub fn QueryPipeline::intersect_ray(
  self : QueryPipeline,
  ray : Ray,
  max_toi : @core.Real,
  solid : Bool,
) -> Array[(ColliderHandle, RayIntersection)] {
  let max_dist = if max_toi < 0.0F { 0.0F } else { max_toi }
  let results : Array[(ColliderHandle, RayIntersection)] = []
  for i in 0..<self.colliders.colliders.length() {
    if self.colliders.colliders[i] is Some(collider) && collider.is_enabled() {
      let handle = ColliderHandle::new(i, self.colliders.generations[i])
      if !query_filter_test(self.filter, self.bodies, handle, collider) {
        continue
      }
      let pos = collider.world_translation
      let rot = collider.world_rotation
      let hit = ray_intersect_shape_and_get_normal(
        ray,
        pos,
        rot,
        collider.shape,
        max_dist,
        solid,
      )
      if hit is Some(intersection) {
        results.push((handle, intersection))
      }
    }
  }
  results
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
  let radius = radius1 + radius2
  let ray = Ray::new(start, vel)
  if ray_intersect_ball(ray, center2, radius, max_toi, true) is Some(toi) {
    if toi > max_toi {
      return None
    }
    let center1 = start.add(vec2_scale(vel, toi))
    let normal = normalize_or_fallback(
      center1.sub(center2),
      normalize_or_fallback(
        @core.Vec2::new(-vel.x, -vel.y),
        @core.Vec2::new(1.0F, 0.0F),
      ),
    )
    // Match parry/rapier: discard initial penetration when we are separating.
    if !stop_at_penetration && toi < 1.0e-5F && normal.dot(vel) >= 0.0F {
      return None
    }
    let point = center2.add(vec2_scale(normal, radius2))
    Some(ShapeCastHit::{ toi, point, normal })
  } else {
    None
  }
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
  let rot = @core.Rot2::from_angle(cuboid_rotation)
  let inv = rot.inverse()
  let local_p = inv.rotate_vec2(start.sub(cuboid_center))
  let inside = @core.abs(local_p.x) <= expanded_hw &&
    @core.abs(local_p.y) <= expanded_hh
  if ray_intersect_cuboid_oriented_and_get_normal(
      ray, cuboid_center, cuboid_rotation, expanded_hw, expanded_hh, max_toi, true,
    )
    is Some(hit) {
    let mut normal = hit.normal
    if hit.toi < 1.0e-5F && inside {
      // Find a stable normal based on the closest cuboid face (instead of -vel).
      let dx = expanded_hw - @core.abs(local_p.x)
      let dy = expanded_hh - @core.abs(local_p.y)
      let vel_local = inv.rotate_vec2(vel)
      if dx < dy {
        let mut sx = if local_p.x >= 0.0F { 1.0F } else { -1.0F }
        if @core.abs(local_p.x) <= 1.0e-12F {
          sx = if -vel_local.x >= 0.0F { 1.0F } else { -1.0F }
        }
        normal = rot.rotate_vec2(@core.Vec2::new(sx, 0.0F))
      } else {
        let mut sy = if local_p.y >= 0.0F { 1.0F } else { -1.0F }
        if @core.abs(local_p.y) <= 1.0e-12F {
          sy = if -vel_local.y >= 0.0F { 1.0F } else { -1.0F }
        }
        normal = rot.rotate_vec2(@core.Vec2::new(0.0F, sy))
      }
    }
    if !stop_at_penetration && hit.toi < 1.0e-5F && normal.dot(vel) >= 0.0F {
      return None
    }
    let center = ray_point(ray, hit.toi)
    let point = center.sub(vec2_scale(normal, radius))
    Some(ShapeCastHit::{ toi: hit.toi, point, normal })
  } else {
    None
  }
}

///|
fn shape_cast_ball_segment(
  start : @core.Vec2,
  vel : @core.Vec2,
  ball_radius : @core.Real,
  seg_a : @core.Vec2,
  seg_b : @core.Vec2,
  seg_radius : @core.Real,
  max_toi : @core.Real,
  stop_at_penetration : Bool,
) -> ShapeCastHit? {
  let dir = seg_b.sub(seg_a)
  let len = dir.length()
  if len <= 1.0e-6F {
    // Degenerate segment: treat as a point at seg_a with radius = seg_radius.
    return shape_cast_ball_ball(
      start, vel, ball_radius, seg_a, seg_radius, max_toi, stop_at_penetration,
    )
  }
  let angle = atan2_real(dir.y, dir.x)
  let mid = @core.Vec2::new(
    0.5F * (seg_a.x + seg_b.x),
    0.5F * (seg_a.y + seg_b.y),
  )
  let ray = Ray::new(start, vel)
  let mut best : ShapeCastHit? = None

  // Endpoint caps: circle hits against endpoints expanded by seg_radius.
  let hit_a = shape_cast_ball_ball(
    start, vel, ball_radius, seg_a, seg_radius, max_toi, stop_at_penetration,
  )
  if hit_a is Some(h) {
    best = Some(h)
  }
  let hit_b = shape_cast_ball_ball(
    start, vel, ball_radius, seg_b, seg_radius, max_toi, stop_at_penetration,
  )
  if hit_b is Some(h) {
    if best is Some(cur) {
      if h.toi < cur.toi {
        best = Some(h)
      }
    } else {
      best = Some(h)
    }
  }

  // Cylinder part: ray vs oriented rectangle expanded only along the normal direction.
  // (No tangential expansion: this matches the segment's Minkowski expansion into a capsule.)
  let expanded_hh = ball_radius + seg_radius
  if ray_intersect_cuboid_oriented_and_get_normal(
      ray,
      mid,
      angle,
      0.5F * len,
      expanded_hh,
      max_toi,
      true,
    )
    is Some(hit) {
    let mut normal = hit.normal
    if hit.toi < 1.0e-5F {
      // Determine if the ball center starts inside the expanded rectangle.
      let rect_rot = @core.Rot2::from_angle(angle)
      let rect_inv = rect_rot.inverse()
      let local_p = rect_inv.rotate_vec2(start.sub(mid))
      let inside_rect = @core.abs(local_p.x) <= 0.5F * len &&
        @core.abs(local_p.y) <= expanded_hh
      if inside_rect {
        let q = closest_point_on_segment(seg_a, seg_b, start)
        let delta = start.sub(q)
        if delta.length_squared() > 1.0e-12F {
          normal = delta.normalize()
        } else {
          let mut seg_n = normalize_or_fallback(
            @core.Vec2::new(-dir.y, dir.x),
            @core.Vec2::new(0.0F, 1.0F),
          )
          if seg_n.dot(vel) > 0.0F {
            seg_n = @core.Vec2::new(-seg_n.x, -seg_n.y)
          }
          normal = seg_n
        }
      }
    }
    if !stop_at_penetration && hit.toi < 1.0e-5F && normal.dot(vel) >= 0.0F {
      // Discard initial penetration when separating.
      ()
    } else {
      let center = ray_point(ray, hit.toi)
      let point = center.sub(vec2_scale(normal, ball_radius))
      let h = ShapeCastHit::{ toi: hit.toi, point, normal }
      if best is Some(cur) {
        if h.toi < cur.toi {
          best = Some(h)
        }
      } else {
        best = Some(h)
      }
    }
  }
  best
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
  let penetration = best_entry <= 0.0F
  let toi = if penetration { 0.0F } else { best_entry }
  if toi > max_toi {
    return None
  }
  let normal = if penetration {
    // Choose a stable penetration normal using the smallest overlap axis (SAT-style).
    let mut best_overlap = 1.0e30F
    let mut axis_best = @core.Vec2::new(1.0F, 0.0F)
    for axis in axes {
      let (min1, max1) = shape_cast_project_interval(
        center1, rotation1, half_width1, half_height1, axis,
      )
      let (min2, max2) = shape_cast_project_interval(
        center2, rotation2, half_width2, half_height2, axis,
      )
      let overlap = min_value(max1, max2) - max_value(min1, min2)
      if overlap < best_overlap {
        best_overlap = overlap
        axis_best = axis
      }
    }
    let sign = if center1.sub(center2).dot(axis_best) >= 0.0F {
      1.0F
    } else {
      -1.0F
    }
    @core.Vec2::new(axis_best.x * sign, axis_best.y * sign)
  } else if best_vel_axis > 1.0e-12F {
    @core.Vec2::new(-best_axis.x, -best_axis.y)
  } else if best_vel_axis < -1.0e-12F {
    best_axis
  } else {
    normalize_or_fallback(
      @core.Vec2::new(-vel.x, -vel.y),
      @core.Vec2::new(1.0F, 0.0F),
    )
  }
  if !stop_at_penetration && toi < 1.0e-5F && normal.dot(vel) >= 0.0F {
    return None
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
  let target_distance = if options.target_distance < 0.0F {
    0.0F
  } else {
    options.target_distance
  }
  // Match parry/rapier semantics: `target_distance` is an outward offset of the casted (moving)
  // shape (like RoundShapeRef), not an expansion of the target collider.
  let moving_shape = match shape {
    Shape::Ball(r) => Shape::Ball(r + target_distance)
    Shape::Cuboid(hw, hh) =>
      Shape::Cuboid(hw + target_distance, hh + target_distance)
    Shape::CapsuleX(h, r) => Shape::CapsuleX(h, r + target_distance)
    Shape::CapsuleY(h, r) => Shape::CapsuleY(h, r + target_distance)
    Shape::Segment(_, _) => shape
    Shape::Polyline(_, _) => shape
    Shape::ConvexPolygon(_) => shape
    Shape::TriMesh(_, _) => shape
    Shape::Compound(_) => shape
  }
  let mut best : (ColliderHandle, ShapeCastHit)? = None
  for i in 0..<self.colliders.colliders.length() {
    if self.colliders.colliders[i] is Some(collider) && collider.is_enabled() {
      let handle = ColliderHandle::new(i, self.colliders.generations[i])
      if !query_filter_test(self.filter, self.bodies, handle, collider) {
        continue
      }
      fn best_hit(
        current : ShapeCastHit?,
        candidate : ShapeCastHit?,
      ) -> ShapeCastHit? {
        if candidate is None {
          return current
        }
        if current is None {
          return candidate
        }
        let c = current.unwrap()
        let n = candidate.unwrap()
        if n.toi < c.toi {
          candidate
        } else {
          current
        }
      }

      fn cast_against_target(
        target_center : @core.Vec2,
        target_rotation : @core.Real,
        target_shape : Shape,
      ) -> ShapeCastHit? {
        match target_shape {
          Shape::Compound(parts) => {
            if parts.length() == 0 {
              return None
            }
            let parent_iso = @core.Isometry2::new(
              target_center,
              @core.Rot2::from_angle(target_rotation),
            )
            let mut best : ShapeCastHit? = None
            for i in 0..<parts.length() {
              let (pose, part_shape) = parts[i]
              let w = parent_iso.mul(pose)
              let p = isometry2_translation(w)
              let r = isometry2_rotation_angle(w)
              best = best_hit(best, cast_against_target(p, r, part_shape))
            }
            best
          }
          Shape::Segment(a, b) => {
            let (sa, sb) = segment_world_endpoints(
              target_center, target_rotation, a, b,
            )
            let seg_dir = sb.sub(sa)
            let seg_len = seg_dir.length()
            let seg_radius = 0.0F

            // Model the segment as a capsule with radius 0.
            let mut segment_components : Array[(Shape, @core.Vec2, @core.Real)] = []
            if seg_len <= 1.0e-6F {
              segment_components = [(Shape::Ball(seg_radius), sa, 0.0F)]
            } else {
              let angle = atan2_real(seg_dir.y, seg_dir.x)
              let mid = @core.Vec2::new(
                0.5F * (sa.x + sb.x),
                0.5F * (sa.y + sb.y),
              )
              segment_components = [
                (Shape::Ball(seg_radius), sa, 0.0F),
                (Shape::Ball(seg_radius), sb, 0.0F),
                (Shape::Cuboid(0.5F * seg_len, seg_radius), mid, angle),
              ]
            }

            // Cast each moving component against the segment capsule components.
            let mut moving_components : Array[(Shape, @core.Isometry2)] = [
              (moving_shape, shape_pos),
            ]
            match moving_shape {
              Shape::CapsuleX(h, r) =>
                moving_components = capsule_components_moving(
                  shape_pos,
                  @core.Vec2::new(1.0F, 0.0F),
                  h,
                  r,
                  h,
                  r,
                )
              Shape::CapsuleY(h, r) =>
                moving_components = capsule_components_moving(
                  shape_pos,
                  @core.Vec2::new(0.0F, 1.0F),
                  h,
                  r,
                  r,
                  h,
                )
              _ => ()
            }
            let mut best : ShapeCastHit? = None
            for ci in 0..<moving_components.length() {
              let comp = moving_components[ci]
              match comp.0 {
                Shape::Ball(r) =>
                  best = best_hit(
                    best,
                    shape_cast_ball_segment(
                      comp.1.translation,
                      shape_vel,
                      r,
                      sa,
                      sb,
                      seg_radius,
                      max_toi,
                      options.stop_at_penetration,
                    ),
                  )
                _ =>
                  for sj in 0..<segment_components.length() {
                    let seg_comp = segment_components[sj]
                    best = best_hit(
                      best,
                      shape_cast_basic(
                        comp.1,
                        shape_vel,
                        comp.0,
                        seg_comp.1,
                        seg_comp.2,
                        seg_comp.0,
                        max_toi,
                        options.stop_at_penetration,
                      ),
                    )
                  }
              }
            }
            best
          }
          target =>
            match (moving_shape, target) {
              (Shape::Ball(_), Shape::Ball(_))
              | (Shape::Ball(_), Shape::Cuboid(_, _))
              | (Shape::Cuboid(_, _), Shape::Ball(_))
              | (Shape::Cuboid(_, _), Shape::Cuboid(_, _)) =>
                shape_cast_basic(
                  shape_pos,
                  shape_vel,
                  moving_shape,
                  target_center,
                  target_rotation,
                  target,
                  max_toi,
                  options.stop_at_penetration,
                )
              (Shape::Ball(_), Shape::CapsuleX(h, r)) =>
                shape_cast_against_capsule(
                  shape_pos,
                  shape_vel,
                  moving_shape,
                  target_center,
                  target_rotation,
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
                  moving_shape,
                  target_center,
                  target_rotation,
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
                  moving_shape,
                  target_center,
                  target_rotation,
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
                  moving_shape,
                  target_center,
                  target_rotation,
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
                let mut best_hit2 : ShapeCastHit? = None
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
                        target_center,
                        target_rotation,
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
                        target_center,
                        target_rotation,
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
                        target_center,
                        target_rotation,
                        target,
                        max_toi,
                        options.stop_at_penetration,
                      )
                  }
                  best_hit2 = best_hit(best_hit2, h2)
                }
                best_hit2
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
                let mut best_hit2 : ShapeCastHit? = None
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
                        target_center,
                        target_rotation,
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
                        target_center,
                        target_rotation,
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
                        target_center,
                        target_rotation,
                        target,
                        max_toi,
                        options.stop_at_penetration,
                      )
                  }
                  best_hit2 = best_hit(best_hit2, h2)
                }
                best_hit2
              }
              _ => None
            }
        }
      }

      let hit = cast_against_target(
        collider.world_translation,
        collider.world_rotation,
        collider.shape,
      )
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
pub fn QueryPipeline::cast_shape_nonlinear(
  self : QueryPipeline,
  shape_motion : NonlinearRigidMotion,
  shape : Shape,
  start_time : @core.Real,
  end_time : @core.Real,
  stop_at_penetration : Bool,
) -> (ColliderHandle, ShapeCastHit)? {
  if end_time <= start_time {
    return None
  }
  fn shape_bounding_radius(shape : Shape) -> @core.Real {
    match shape {
      Shape::Ball(r) => r
      Shape::Cuboid(hw, hh) => sqrt_real(hw * hw + hh * hh)
      Shape::CapsuleX(h, r) => h + r
      Shape::CapsuleY(h, r) => h + r
      Shape::Segment(a, b) => {
        let la = a.length()
        let lb = b.length()
        if la > lb {
          la
        } else {
          lb
        }
      }
      Shape::Polyline(vertices, _) => {
        let mut best = 0.0F
        for i in 0..<vertices.length() {
          let d = vertices[i].length()
          if d > best {
            best = d
          }
        }
        best
      }
      Shape::ConvexPolygon(vertices) => {
        let mut best = 0.0F
        for i in 0..<vertices.length() {
          let d = vertices[i].length()
          if d > best {
            best = d
          }
        }
        best
      }
      Shape::TriMesh(vertices, _) => {
        let mut best = 0.0F
        for i in 0..<vertices.length() {
          let d = vertices[i].length()
          if d > best {
            best = d
          }
        }
        best
      }
      Shape::Compound(parts) => {
        let mut best = 0.0F
        for i in 0..<parts.length() {
          let (pose, part_shape) = parts[i]
          let t = isometry2_translation(pose)
          let d = t.length() + shape_bounding_radius(part_shape)
          if d > best {
            best = d
          }
        }
        best
      }
    }
  }

  fn swept_center_aabb(
    c0 : @core.Vec2,
    c1 : @core.Vec2,
    radius : @core.Real,
  ) -> @core.Aabb {
    @core.Aabb::new(
      @core.Vec2::new(
        min_value(c0.x, c1.x) - radius,
        min_value(c0.y, c1.y) - radius,
      ),
      @core.Vec2::new(
        max_value(c0.x, c1.x) + radius,
        max_value(c0.y, c1.y) + radius,
      ),
    )
  }

  fn project_point_on_shape_boundary(
    shape : Shape,
    center : @core.Vec2,
    rotation : @core.Real,
    point : @core.Vec2,
  ) -> @core.Vec2 {
    match shape {
      Shape::Ball(r) => project_point_on_ball(center, r, point, false).point
      Shape::Cuboid(hw, hh) =>
        project_point_on_cuboid_oriented(center, rotation, hw, hh, point, false).point
      Shape::CapsuleX(h, r) =>
        project_point_on_capsule_oriented(
          center,
          rotation,
          @core.Vec2::new(1.0F, 0.0F),
          h,
          r,
          point,
          false,
        ).point
      Shape::CapsuleY(h, r) =>
        project_point_on_capsule_oriented(
          center,
          rotation,
          @core.Vec2::new(0.0F, 1.0F),
          h,
          r,
          point,
          false,
        ).point
      Shape::Segment(a, b) =>
        project_point_on_segment(center, rotation, a, b, point).point
      Shape::Polyline(vertices, indices) =>
        project_point_on_polyline(center, rotation, vertices, indices, point).point
      Shape::ConvexPolygon(vertices) =>
        project_point_on_convex_polygon_oriented(
          center, rotation, vertices, point, false,
        ).point
      Shape::TriMesh(vertices, indices) =>
        project_point_on_shape(
          center,
          rotation,
          Shape::TriMesh(vertices, indices),
          point,
          false,
        ).point
      Shape::Compound(parts) =>
        project_point_on_shape(
          center,
          rotation,
          Shape::Compound(parts),
          point,
          false,
        ).point
    }
  }

  fn contact_at_time(
    time : @core.Real,
    target_shape : Shape,
    target_center : @core.Vec2,
    target_rot : @core.Real,
  ) -> (@core.Vec2, @core.Vec2, @core.Vec2) {
    let pose = shape_motion.pose_at_time(time)
    let moving_center = pose.translation
    let moving_rot = pose.rotation.angle()
    let point_on_target = project_point_on_shape_boundary(
      target_shape, target_center, target_rot, moving_center,
    )
    let point_on_moving = project_point_on_shape_boundary(
      shape, moving_center, moving_rot, point_on_target,
    )
    let vel = shape_motion.point_velocity_at_time(time, point_on_moving)
    let fallback = @core.Vec2::new(-vel.x, -vel.y)
    let normal = normalize_or_fallback(
      point_on_moving.sub(point_on_target),
      normalize_or_fallback(fallback, @core.Vec2::new(1.0F, 0.0F)),
    )
    (point_on_target, point_on_moving, normal)
  }

  fn intersects_at_time(
    time : @core.Real,
    target_shape : Shape,
    target_center : @core.Vec2,
    target_rot : @core.Real,
  ) -> Bool {
    let pose = shape_motion.pose_at_time(time)
    shapes_intersect(
      shape,
      pose.translation,
      pose.rotation.angle(),
      target_shape,
      target_center,
      target_rot,
    )
  }

  fn compute_steps(
    duration : @core.Real,
    c0 : @core.Vec2,
    c1 : @core.Vec2,
    radius : @core.Real,
    angvel : @core.Real,
  ) -> Int {
    let dist = c1.sub(c0).length()
    let trans_scale = radius * 0.25F + 0.05F
    let trans_steps = if trans_scale <= 1.0e-6F {
      64
    } else {
      (dist / trans_scale).to_int() + 1
    }
    let rot_steps = (@core.abs(angvel) * duration / 0.1F).to_int() + 1
    let mut steps = if trans_steps > rot_steps {
      trans_steps
    } else {
      rot_steps
    }
    if steps < 8 {
      steps = 8
    }
    if steps > 256 {
      steps = 256
    }
    steps
  }

  fn bisect_transition(
    lo : @core.Real,
    hi : @core.Real,
    target_shape : Shape,
    target_center : @core.Vec2,
    target_rot : @core.Real,
    want_intersection_at_hi : Bool,
  ) -> @core.Real {
    let mut left = lo
    let mut right = hi
    for _ in 0..<25 {
      let mid = 0.5F * (left + right)
      let mid_state = intersects_at_time(
        mid, target_shape, target_center, target_rot,
      )
      if mid_state == want_intersection_at_hi {
        right = mid
      } else {
        left = mid
      }
    }
    right
  }

  fn find_first_entry_time(
    scan_start : @core.Real,
    scan_end : @core.Real,
    target_shape : Shape,
    target_center : @core.Vec2,
    target_rot : @core.Real,
    radius : @core.Real,
  ) -> @core.Real? {
    let duration = scan_end - scan_start
    if duration <= 0.0F {
      return None
    }
    let c0 = shape_motion.pose_at_time(scan_start).translation
    let c1 = shape_motion.pose_at_time(scan_end).translation
    let steps = compute_steps(duration, c0, c1, radius, shape_motion.angvel)
    let dt = duration / Float::from_double(steps.to_double())
    let mut prev_t = scan_start
    for si in 1..<(steps + 1) {
      let mut t = scan_start + dt * Float::from_double(si.to_double())
      if t > scan_end {
        t = scan_end
      }
      if intersects_at_time(t, target_shape, target_center, target_rot) {
        return Some(
          bisect_transition(
            prev_t, t, target_shape, target_center, target_rot, true,
          ),
        )
      }
      prev_t = t
    }
    None
  }

  fn find_first_exit_time(
    scan_start : @core.Real,
    scan_end : @core.Real,
    target_shape : Shape,
    target_center : @core.Vec2,
    target_rot : @core.Real,
    radius : @core.Real,
  ) -> @core.Real? {
    let duration = scan_end - scan_start
    if duration <= 0.0F {
      return None
    }
    let c0 = shape_motion.pose_at_time(scan_start).translation
    let c1 = shape_motion.pose_at_time(scan_end).translation
    let steps = compute_steps(duration, c0, c1, radius, shape_motion.angvel)
    let dt = duration / Float::from_double(steps.to_double())
    let mut prev_t = scan_start
    for si in 1..<(steps + 1) {
      let mut t = scan_start + dt * Float::from_double(si.to_double())
      if t > scan_end {
        t = scan_end
      }
      if !intersects_at_time(t, target_shape, target_center, target_rot) {
        return Some(
          bisect_transition(
            prev_t, t, target_shape, target_center, target_rot, false,
          ),
        )
      }
      prev_t = t
    }
    None
  }

  let radius = shape_bounding_radius(shape)
  let c0 = shape_motion.pose_at_time(start_time).translation
  let c1 = shape_motion.pose_at_time(end_time).translation
  let sweep_aabb = swept_center_aabb(c0, c1, radius)
  let mut best : (ColliderHandle, ShapeCastHit)? = None
  for i in 0..<self.colliders.colliders.length() {
    if self.colliders.colliders[i] is Some(collider) && collider.is_enabled() {
      let handle = ColliderHandle::new(i, self.colliders.generations[i])
      if !query_filter_test(self.filter, self.bodies, handle, collider) {
        continue
      }
      // AABB prefilter using the cached collider AABB (Rapier parity).
      if i < self.aabbs.length() && self.aabbs[i] is Some(co_aabb) {
        if !co_aabb.intersects(sweep_aabb) {
          continue
        }
      }
      let target_shape = collider.shape
      let target_center = collider.world_translation
      let target_rot = collider.world_rotation
      let mut scan_start = start_time
      if intersects_at_time(scan_start, target_shape, target_center, target_rot) {
        let (point_on_target, point_on_moving, normal) = contact_at_time(
          scan_start, target_shape, target_center, target_rot,
        )
        let vel = shape_motion.point_velocity_at_time(
          scan_start, point_on_moving,
        )
        let separating = normal.dot(vel) >= 0.0F
        if stop_at_penetration || !separating {
          let hit = ShapeCastHit::{
            toi: scan_start,
            point: point_on_target,
            normal,
          }
          if best is Some(current) {
            if hit.toi < current.1.toi {
              best = Some((handle, hit))
            }
          } else {
            best = Some((handle, hit))
          }
          continue
        }

        // Separating from an initial penetration: skip until we exit the overlap, then
        // look for a later time-of-impact.
        if find_first_exit_time(
            scan_start, end_time, target_shape, target_center, target_rot, radius,
          )
          is Some(exit_time) {
          scan_start = exit_time
        } else {
          continue
        }
      }
      if find_first_entry_time(
          scan_start, end_time, target_shape, target_center, target_rot, radius,
        )
        is Some(toi) {
        let (point_on_target, _, normal) = contact_at_time(
          toi, target_shape, target_center, target_rot,
        )
        let hit = ShapeCastHit::{ toi, point: point_on_target, normal }
        if best is Some(current) {
          if hit.toi < current.1.toi {
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
    Shape::Segment(a, b) => {
      let (wa, wb) = segment_world_endpoints(center, rotation, a, b)
      let q = closest_point_on_segment(wa, wb, point)
      point.sub(q).dot(point.sub(q)) <= 1.0e-12F
    }
    Shape::Polyline(vertices, indices) => {
      let proj = project_point_on_polyline(
        center, rotation, vertices, indices, point,
      )
      let delta = point.sub(proj.point)
      delta.dot(delta) <= 1.0e-12F
    }
    Shape::ConvexPolygon(vertices) => {
      let rot = @core.Rot2::from_angle(rotation)
      let inv = rot.inverse()
      let local_point = inv.rotate_vec2(point.sub(center))
      point_in_convex_polygon_local(vertices, local_point)
    }
    Shape::TriMesh(vertices, indices) => {
      let rot = @core.Rot2::from_angle(rotation)
      let inv = rot.inverse()
      let local_point = inv.rotate_vec2(point.sub(center))
      for i in 0..<indices.length() {
        let (i0, i1, i2) = indices[i]
        if i0 < 0 || i1 < 0 || i2 < 0 {
          continue
        }
        if i0 >= vertices.length() ||
          i1 >= vertices.length() ||
          i2 >= vertices.length() {
          continue
        }
        let a = vertices[i0]
        let b0 = vertices[i1]
        let c0 = vertices[i2]
        let ab = b0.sub(a)
        let ac = c0.sub(a)
        let area2 = ab.x * ac.y - ab.y * ac.x
        let b = if area2 >= 0.0F { b0 } else { c0 }
        let c = if area2 >= 0.0F { c0 } else { b0 }
        if point_in_convex_polygon_local([a, b, c], local_point) {
          return true
        }
      }
      false
    }
    Shape::Compound(parts) => {
      if parts.length() == 0 {
        return false
      }
      let parent_iso = @core.Isometry2::new(
        center,
        @core.Rot2::from_angle(rotation),
      )
      for i in 0..<parts.length() {
        let (pose, part_shape) = parts[i]
        let w = parent_iso.mul(pose)
        let p = isometry2_translation(w)
        let r = isometry2_rotation_angle(w)
        if point_in_shape(part_shape, p, r, point) {
          return true
        }
      }
      false
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
    Shape::Segment(a, b) => {
      let (wa, wb) = segment_world_endpoints(
        collider.world_translation,
        collider.world_rotation,
        a,
        b,
      )
      let min_x = if wa.x < wb.x { wa.x } else { wb.x }
      let max_x = if wa.x > wb.x { wa.x } else { wb.x }
      let min_y = if wa.y < wb.y { wa.y } else { wb.y }
      let max_y = if wa.y > wb.y { wa.y } else { wb.y }
      @core.Aabb::new(
        @core.Vec2::new(min_x, min_y),
        @core.Vec2::new(max_x, max_y),
      )
    }
    Shape::Polyline(vertices, _) =>
      if vertices.length() == 0 {
        @core.Aabb::new(collider.world_translation, collider.world_translation)
      } else {
        let rot = @core.Rot2::from_angle(collider.world_rotation)
        let c = collider.world_translation
        let mut min_x = 1.0e30F
        let mut max_x = -1.0e30F
        let mut min_y = 1.0e30F
        let mut max_y = -1.0e30F
        for i in 0..<vertices.length() {
          let wp = c.add(rot.rotate_vec2(vertices[i]))
          if wp.x < min_x {
            min_x = wp.x
          }
          if wp.x > max_x {
            max_x = wp.x
          }
          if wp.y < min_y {
            min_y = wp.y
          }
          if wp.y > max_y {
            max_y = wp.y
          }
        }
        @core.Aabb::new(
          @core.Vec2::new(min_x, min_y),
          @core.Vec2::new(max_x, max_y),
        )
      }
    Shape::ConvexPolygon(vertices) =>
      if vertices.length() == 0 {
        @core.Aabb::new(collider.world_translation, collider.world_translation)
      } else {
        let rot = @core.Rot2::from_angle(collider.world_rotation)
        let c = collider.world_translation
        let mut min_x = 1.0e30F
        let mut max_x = -1.0e30F
        let mut min_y = 1.0e30F
        let mut max_y = -1.0e30F
        for i in 0..<vertices.length() {
          let wp = c.add(rot.rotate_vec2(vertices[i]))
          if wp.x < min_x {
            min_x = wp.x
          }
          if wp.x > max_x {
            max_x = wp.x
          }
          if wp.y < min_y {
            min_y = wp.y
          }
          if wp.y > max_y {
            max_y = wp.y
          }
        }
        @core.Aabb::new(
          @core.Vec2::new(min_x, min_y),
          @core.Vec2::new(max_x, max_y),
        )
      }
    Shape::TriMesh(vertices, _) =>
      if vertices.length() == 0 {
        @core.Aabb::new(collider.world_translation, collider.world_translation)
      } else {
        let rot = @core.Rot2::from_angle(collider.world_rotation)
        let c = collider.world_translation
        let mut min_x = 1.0e30F
        let mut max_x = -1.0e30F
        let mut min_y = 1.0e30F
        let mut max_y = -1.0e30F
        for i in 0..<vertices.length() {
          let wp = c.add(rot.rotate_vec2(vertices[i]))
          if wp.x < min_x {
            min_x = wp.x
          }
          if wp.x > max_x {
            max_x = wp.x
          }
          if wp.y < min_y {
            min_y = wp.y
          }
          if wp.y > max_y {
            max_y = wp.y
          }
        }
        @core.Aabb::new(
          @core.Vec2::new(min_x, min_y),
          @core.Vec2::new(max_x, max_y),
        )
      }
    Shape::Compound(parts) =>
      if parts.length() == 0 {
        @core.Aabb::new(collider.world_translation, collider.world_translation)
      } else {
        let base_iso = @core.Isometry2::new(
          collider.world_translation,
          @core.Rot2::from_angle(collider.world_rotation),
        )
        let mut acc : @core.Aabb? = None
        for i in 0..<parts.length() {
          let (pose, part_shape) = parts[i]
          let w = base_iso.mul(pose)
          let part_collider = collider
          part_collider.shape = part_shape
          part_collider.world_translation = isometry2_translation(w)
          part_collider.world_rotation = isometry2_rotation_angle(w)
          let part_aabb = compute_collider_aabb(part_collider)
          if acc is Some(existing) {
            acc = Some(existing.combine(part_aabb))
          } else {
            acc = Some(part_aabb)
          }
        }
        if acc is Some(aabb) {
          aabb
        } else {
          @core.Aabb::new(
            collider.world_translation,
            collider.world_translation,
          )
        }
      }
  }
}

///|
fn segment_world_endpoints_rot(
  center : @core.Vec2,
  rotation : @core.Rot2,
  a : @core.Vec2,
  b : @core.Vec2,
) -> (@core.Vec2, @core.Vec2) {
  (center.add(rotation.rotate_vec2(a)), center.add(rotation.rotate_vec2(b)))
}

///|
fn compute_query_shape_aabb(
  shape_pos : @core.Isometry2,
  shape : Shape,
) -> @core.Aabb {
  let center = shape_pos.translation
  let rot = shape_pos.rotation
  match shape {
    Shape::Ball(radius) =>
      @core.Aabb::new(
        @core.Vec2::new(center.x - radius, center.y - radius),
        @core.Vec2::new(center.x + radius, center.y + radius),
      )
    Shape::Cuboid(half_width, half_height) => {
      let abs_cos = @core.abs(rot.cos)
      let abs_sin = @core.abs(rot.sin)
      let hw = abs_cos * half_width + abs_sin * half_height
      let hh = abs_sin * half_width + abs_cos * half_height
      @core.Aabb::new(
        @core.Vec2::new(center.x - hw, center.y - hh),
        @core.Vec2::new(center.x + hw, center.y + hh),
      )
    }
    Shape::CapsuleX(half_height, radius) => {
      let offset = rot.rotate_vec2(@core.Vec2::new(half_height, 0.0F))
      let a = center.add(offset)
      let b = center.sub(offset)
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
      let offset = rot.rotate_vec2(@core.Vec2::new(0.0F, half_height))
      let a = center.add(offset)
      let b = center.sub(offset)
      let min_x = if a.x < b.x { a.x } else { b.x }
      let max_x = if a.x > b.x { a.x } else { b.x }
      let min_y = if a.y < b.y { a.y } else { b.y }
      let max_y = if a.y > b.y { a.y } else { b.y }
      @core.Aabb::new(
        @core.Vec2::new(min_x - radius, min_y - radius),
        @core.Vec2::new(max_x + radius, max_y + radius),
      )
    }
    Shape::Segment(a, b) => {
      let (wa, wb) = segment_world_endpoints_rot(center, rot, a, b)
      let min_x = if wa.x < wb.x { wa.x } else { wb.x }
      let max_x = if wa.x > wb.x { wa.x } else { wb.x }
      let min_y = if wa.y < wb.y { wa.y } else { wb.y }
      let max_y = if wa.y > wb.y { wa.y } else { wb.y }
      @core.Aabb::new(
        @core.Vec2::new(min_x, min_y),
        @core.Vec2::new(max_x, max_y),
      )
    }
    Shape::Polyline(vertices, _) =>
      if vertices.length() == 0 {
        @core.Aabb::new(center, center)
      } else {
        let mut min_x = 1.0e30F
        let mut max_x = -1.0e30F
        let mut min_y = 1.0e30F
        let mut max_y = -1.0e30F
        for i in 0..<vertices.length() {
          let wp = center.add(rot.rotate_vec2(vertices[i]))
          if wp.x < min_x {
            min_x = wp.x
          }
          if wp.x > max_x {
            max_x = wp.x
          }
          if wp.y < min_y {
            min_y = wp.y
          }
          if wp.y > max_y {
            max_y = wp.y
          }
        }
        @core.Aabb::new(
          @core.Vec2::new(min_x, min_y),
          @core.Vec2::new(max_x, max_y),
        )
      }
    Shape::ConvexPolygon(vertices) =>
      if vertices.length() == 0 {
        @core.Aabb::new(center, center)
      } else {
        let mut min_x = 1.0e30F
        let mut max_x = -1.0e30F
        let mut min_y = 1.0e30F
        let mut max_y = -1.0e30F
        for i in 0..<vertices.length() {
          let wp = center.add(rot.rotate_vec2(vertices[i]))
          if wp.x < min_x {
            min_x = wp.x
          }
          if wp.x > max_x {
            max_x = wp.x
          }
          if wp.y < min_y {
            min_y = wp.y
          }
          if wp.y > max_y {
            max_y = wp.y
          }
        }
        @core.Aabb::new(
          @core.Vec2::new(min_x, min_y),
          @core.Vec2::new(max_x, max_y),
        )
      }
    Shape::TriMesh(vertices, _) =>
      if vertices.length() == 0 {
        @core.Aabb::new(center, center)
      } else {
        let mut min_x = 1.0e30F
        let mut max_x = -1.0e30F
        let mut min_y = 1.0e30F
        let mut max_y = -1.0e30F
        for i in 0..<vertices.length() {
          let wp = center.add(rot.rotate_vec2(vertices[i]))
          if wp.x < min_x {
            min_x = wp.x
          }
          if wp.x > max_x {
            max_x = wp.x
          }
          if wp.y < min_y {
            min_y = wp.y
          }
          if wp.y > max_y {
            max_y = wp.y
          }
        }
        @core.Aabb::new(
          @core.Vec2::new(min_x, min_y),
          @core.Vec2::new(max_x, max_y),
        )
      }
    Shape::Compound(parts) =>
      if parts.length() == 0 {
        @core.Aabb::new(center, center)
      } else {
        let mut acc : @core.Aabb? = None
        for i in 0..<parts.length() {
          let (pose, part_shape) = parts[i]
          let part_pos = shape_pos.mul(pose)
          let part_aabb = compute_query_shape_aabb(part_pos, part_shape)
          if acc is Some(existing) {
            acc = Some(existing.combine(part_aabb))
          } else {
            acc = Some(part_aabb)
          }
        }
        if acc is Some(aabb) {
          aabb
        } else {
          @core.Aabb::new(center, center)
        }
      }
  }
}

///|
pub fn QueryPipeline::intersect_shape(
  self : QueryPipeline,
  shape_pos : @core.Isometry2,
  shape : Shape,
) -> Array[ColliderHandle] {
  let shape_aabb = compute_query_shape_aabb(shape_pos, shape)
  let results : Array[ColliderHandle] = []
  for i in 0..<self.colliders.colliders.length() {
    if self.colliders.colliders[i] is Some(collider) && collider.is_enabled() {
      let handle = ColliderHandle::new(i, self.colliders.generations[i])
      if !query_filter_test(self.filter, self.bodies, handle, collider) {
        continue
      }
      if i < self.aabbs.length() && self.aabbs[i] is Some(co_aabb) {
        if !co_aabb.intersects(shape_aabb) {
          continue
        }
      }
      if shapes_intersect(
          shape,
          shape_pos.translation,
          shape_pos.rotation.angle(),
          collider.shape,
          collider.world_translation,
          collider.world_rotation,
        ) {
        results.push(handle)
      }
    }
  }
  results
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
fn project_point_on_ball_and_get_feature(
  center : @core.Vec2,
  radius : @core.Real,
  point : @core.Vec2,
  solid : Bool,
) -> (PointProjection, FeatureId) {
  let projection = project_point_on_ball(center, radius, point, solid)
  let feature = if projection.is_inside && solid {
    FeatureId::Unknown
  } else {
    FeatureId::Face(0)
  }
  (projection, feature)
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
fn project_point_on_cuboid_oriented_and_get_feature(
  center : @core.Vec2,
  rotation : @core.Real,
  half_width : @core.Real,
  half_height : @core.Real,
  point : @core.Vec2,
  solid : Bool,
) -> (PointProjection, FeatureId) {
  let rot = @core.Rot2::from_angle(rotation)
  let inv = rot.inverse()
  let local_point = inv.rotate_vec2(point.sub(center))
  let inside = @core.abs(local_point.x) <= half_width &&
    @core.abs(local_point.y) <= half_height
  if inside && solid {
    return (PointProjection::{ point, is_inside: true }, FeatureId::Unknown)
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
  let projection = PointProjection::{ point: projected, is_inside: inside }

  // Feature indices:
  // - Edges: 0 = x=+hw, 1 = x=-hw, 2 = y=+hh, 3 = y=-hh
  // - Vertices: 0 = (+,+), 1 = (-,+), 2 = (-,-), 3 = (+,-)
  let on_x = @core.abs(@core.abs(projected_local.x) - half_width) <= 1.0e-6F
  let on_y = @core.abs(@core.abs(projected_local.y) - half_height) <= 1.0e-6F
  let feature = if on_x && on_y {
    let vx = if projected_local.x >= 0.0F { 1 } else { 0 }
    let vy = if projected_local.y >= 0.0F { 1 } else { 0 }
    let vid = match (vx, vy) {
      (1, 1) => 0
      (0, 1) => 1
      (0, 0) => 2
      (1, 0) => 3
      _ => 0
    }
    FeatureId::Vertex(vid)
  } else if on_x {
    FeatureId::Edge(if projected_local.x >= 0.0F { 0 } else { 1 })
  } else {
    FeatureId::Edge(if projected_local.y >= 0.0F { 2 } else { 3 })
  }
  (projection, feature)
}

///|
fn point_in_convex_polygon_local(
  vertices : Array[@core.Vec2],
  point : @core.Vec2,
) -> Bool {
  if vertices.length() < 3 {
    return false
  }
  // CCW polygon: point is inside iff it is never strictly to the right of an edge.
  let eps = 1.0e-6F
  for i in 0..<vertices.length() {
    let a = vertices[i]
    let b = vertices[(i + 1) % vertices.length()]
    let e = b.sub(a)
    if cross2(e, point.sub(a)) < -eps {
      return false
    }
  }
  true
}

///|
fn project_point_on_convex_polygon_oriented(
  center : @core.Vec2,
  rotation : @core.Real,
  vertices : Array[@core.Vec2],
  point : @core.Vec2,
  solid : Bool,
) -> PointProjection {
  let rot = @core.Rot2::from_angle(rotation)
  let inv = rot.inverse()
  let local_point = inv.rotate_vec2(point.sub(center))
  let inside = point_in_convex_polygon_local(vertices, local_point)
  if inside && solid {
    return PointProjection::{ point, is_inside: true }
  }
  let mut best_dist2 = 1.0e30F
  let mut best_local = local_point
  for i in 0..<vertices.length() {
    let a = vertices[i]
    let b = vertices[(i + 1) % vertices.length()]
    let ab = b.sub(a)
    let denom = ab.dot(ab)
    let t = if denom <= 1.0e-12F {
      0.0F
    } else {
      clamp_value(local_point.sub(a).dot(ab) / denom, 0.0F, 1.0F)
    }
    let proj = a.add(vec2_scale(ab, t))
    let delta = local_point.sub(proj)
    let dist2 = delta.dot(delta)
    if dist2 < best_dist2 {
      best_dist2 = dist2
      best_local = proj
    }
  }
  let projected = center.add(rot.rotate_vec2(best_local))
  PointProjection::{ point: projected, is_inside: inside }
}

///|
fn project_point_on_convex_polygon_oriented_and_get_feature(
  center : @core.Vec2,
  rotation : @core.Real,
  vertices : Array[@core.Vec2],
  point : @core.Vec2,
  solid : Bool,
) -> (PointProjection, FeatureId) {
  let rot = @core.Rot2::from_angle(rotation)
  let inv = rot.inverse()
  let local_point = inv.rotate_vec2(point.sub(center))
  let inside = point_in_convex_polygon_local(vertices, local_point)
  if inside && solid {
    return (PointProjection::{ point, is_inside: true }, FeatureId::Unknown)
  }
  let mut best_dist2 = 1.0e30F
  let mut best_local = local_point
  let mut best_feature = FeatureId::Unknown
  let eps = 1.0e-6F
  for i in 0..<vertices.length() {
    let a = vertices[i]
    let b = vertices[(i + 1) % vertices.length()]
    let ab = b.sub(a)
    let denom = ab.dot(ab)
    let t = if denom <= 1.0e-12F {
      0.0F
    } else {
      clamp_value(local_point.sub(a).dot(ab) / denom, 0.0F, 1.0F)
    }
    let proj = a.add(vec2_scale(ab, t))
    let delta = local_point.sub(proj)
    let dist2 = delta.dot(delta)
    if dist2 < best_dist2 {
      best_dist2 = dist2
      best_local = proj
      best_feature = if t <= eps {
        FeatureId::Vertex(i)
      } else if t >= 1.0F - eps {
        FeatureId::Vertex((i + 1) % vertices.length())
      } else {
        FeatureId::Edge(i)
      }
    }
  }
  let projected = center.add(rot.rotate_vec2(best_local))
  (PointProjection::{ point: projected, is_inside: inside }, best_feature)
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
fn project_point_on_capsule_oriented_and_get_feature(
  center : @core.Vec2,
  rotation : @core.Real,
  axis : @core.Vec2,
  half_height : @core.Real,
  radius : @core.Real,
  point : @core.Vec2,
  solid : Bool,
) -> (PointProjection, FeatureId) {
  let rot = @core.Rot2::from_angle(rotation)
  let inv = rot.inverse()
  let local_point = inv.rotate_vec2(point.sub(center))
  let is_axis_x = @core.abs(axis.x) > @core.abs(axis.y)
  let (clamped, cap_id) = if is_axis_x {
    let cx = clamp_value(local_point.x, -half_height, half_height)
    let id = if cx >= half_height - 1.0e-6F {
      1
    } else if cx <= -half_height + 1.0e-6F {
      0
    } else {
      -1
    }
    (@core.Vec2::new(cx, 0.0F), id)
  } else {
    let cy = clamp_value(local_point.y, -half_height, half_height)
    let id = if cy >= half_height - 1.0e-6F {
      1
    } else if cy <= -half_height + 1.0e-6F {
      0
    } else {
      -1
    }
    (@core.Vec2::new(0.0F, cy), id)
  }
  let delta = local_point.sub(clamped)
  let dist2 = delta.dot(delta)
  let inside = dist2 <= radius * radius
  if inside && solid {
    return (PointProjection::{ point, is_inside: true }, FeatureId::Unknown)
  }
  let projected_local = if dist2 <= 1.0e-12F {
    clamped.add(@core.Vec2::new(radius, 0.0F))
  } else {
    clamped.add(vec2_scale(delta, radius / Float::sqrt(dist2)))
  }
  let projected = center.add(rot.rotate_vec2(projected_local))
  let projection = PointProjection::{ point: projected, is_inside: inside }
  let feature = if cap_id >= 0 {
    FeatureId::Vertex(cap_id)
  } else {
    FeatureId::Edge(0)
  }
  (projection, feature)
}

///|
fn project_point_on_segment(
  center : @core.Vec2,
  rotation : @core.Real,
  a : @core.Vec2,
  b : @core.Vec2,
  point : @core.Vec2,
) -> PointProjection {
  let (wa, wb) = segment_world_endpoints(center, rotation, a, b)
  let q = closest_point_on_segment(wa, wb, point)
  PointProjection::{ point: q, is_inside: false }
}

///|
fn project_point_on_segment_and_get_feature(
  center : @core.Vec2,
  rotation : @core.Real,
  a : @core.Vec2,
  b : @core.Vec2,
  point : @core.Vec2,
) -> (PointProjection, FeatureId) {
  let (wa, wb) = segment_world_endpoints(center, rotation, a, b)
  let ab = wb.sub(wa)
  let denom = ab.dot(ab)
  let mut t = 0.0F
  if denom > 1.0e-12F {
    t = clamp_value(point.sub(wa).dot(ab) / denom, 0.0F, 1.0F)
  }
  let q = wa.add(@core.Vec2::new(ab.x * t, ab.y * t))
  let feature = if t <= 1.0e-6F {
    FeatureId::Vertex(0)
  } else if t >= 1.0F - 1.0e-6F {
    FeatureId::Vertex(1)
  } else {
    FeatureId::Edge(0)
  }
  (PointProjection::{ point: q, is_inside: false }, feature)
}

///|
fn polyline_for_each_segment(
  vertices : Array[@core.Vec2],
  indices : Array[(Int, Int)]?,
  callback : (Int, @core.Vec2, @core.Vec2) -> Unit,
) -> Unit {
  if indices is Some(idxs) {
    for i in 0..<idxs.length() {
      let (i0, i1) = idxs[i]
      if i0 < 0 || i1 < 0 {
        continue
      }
      if i0 >= vertices.length() || i1 >= vertices.length() {
        continue
      }
      callback(i, vertices[i0], vertices[i1])
    }
  } else {
    if vertices.length() < 2 {
      return
    }
    for i in 0..<(vertices.length() - 1) {
      callback(i, vertices[i], vertices[i + 1])
    }
  }
}

///|
fn project_point_on_polyline(
  center : @core.Vec2,
  rotation : @core.Real,
  vertices : Array[@core.Vec2],
  indices : Array[(Int, Int)]?,
  point : @core.Vec2,
) -> PointProjection {
  let mut best : PointProjection? = None
  let mut best_dist2 = 1.0e30F
  polyline_for_each_segment(vertices, indices, fn(
    _seg_index : Int,
    a : @core.Vec2,
    b : @core.Vec2,
  ) -> Unit {
    let proj = project_point_on_segment(center, rotation, a, b, point)
    let delta = point.sub(proj.point)
    let dist2 = delta.dot(delta)
    if dist2 < best_dist2 {
      best_dist2 = dist2
      best = Some(proj)
    }
  })
  if best is Some(p) {
    p
  } else {
    PointProjection::{ point, is_inside: false }
  }
}

///|
fn project_point_on_polyline_and_get_feature(
  center : @core.Vec2,
  rotation : @core.Real,
  vertices : Array[@core.Vec2],
  indices : Array[(Int, Int)]?,
  point : @core.Vec2,
) -> (PointProjection, FeatureId) {
  let mut best : (PointProjection, FeatureId)? = None
  let mut best_dist2 = 1.0e30F
  polyline_for_each_segment(vertices, indices, fn(
    seg_index : Int,
    a : @core.Vec2,
    b : @core.Vec2,
  ) -> Unit {
    let proj = project_point_on_segment(center, rotation, a, b, point)
    let delta = point.sub(proj.point)
    let dist2 = delta.dot(delta)
    if dist2 < best_dist2 {
      best_dist2 = dist2
      best = Some((proj, FeatureId::Edge(seg_index)))
    }
  })
  if best is Some(r) {
    r
  } else {
    (PointProjection::{ point, is_inside: false }, FeatureId::Unknown)
  }
}

///|
fn ray_intersect_polyline_and_get_normal(
  ray : Ray,
  center : @core.Vec2,
  rotation : @core.Real,
  vertices : Array[@core.Vec2],
  indices : Array[(Int, Int)]?,
  max_dist : @core.Real,
  solid : Bool,
) -> RayIntersection? {
  let mut best : RayIntersection? = None
  polyline_for_each_segment(vertices, indices, fn(
    _seg_index : Int,
    a : @core.Vec2,
    b : @core.Vec2,
  ) -> Unit {
    let hit = ray_intersect_segment_and_get_normal(
      ray, center, rotation, a, b, max_dist, solid,
    )
    if hit is Some(h) {
      if best is Some(cur) {
        if h.toi < cur.toi {
          best = Some(h)
        }
      } else {
        best = Some(h)
      }
    }
  })
  best
}

///|
fn ray_intersect_shape_and_get_normal(
  ray : Ray,
  center : @core.Vec2,
  rotation : @core.Real,
  shape : Shape,
  max_dist : @core.Real,
  solid : Bool,
) -> RayIntersection? {
  match shape {
    Shape::Ball(radius) =>
      ray_intersect_ball_and_get_normal(ray, center, radius, max_dist, solid)
    Shape::Cuboid(hw, hh) =>
      ray_intersect_cuboid_oriented_and_get_normal(
        ray, center, rotation, hw, hh, max_dist, solid,
      )
    Shape::CapsuleX(half_height, radius) =>
      ray_intersect_capsule_oriented_and_get_normal(
        ray,
        center,
        rotation,
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
        center,
        rotation,
        @core.Vec2::new(0.0F, 1.0F),
        half_height,
        radius,
        radius,
        half_height,
        max_dist,
        solid,
      )
    Shape::Segment(a, b) =>
      ray_intersect_segment_and_get_normal(
        ray, center, rotation, a, b, max_dist, solid,
      )
    Shape::Polyline(vertices, indices) =>
      ray_intersect_polyline_and_get_normal(
        ray, center, rotation, vertices, indices, max_dist, solid,
      )
    Shape::ConvexPolygon(vertices) =>
      ray_intersect_convex_polygon_oriented_and_get_normal(
        ray, center, rotation, vertices, max_dist, solid,
      )
    Shape::TriMesh(vertices, indices) => {
      let mut best : RayIntersection? = None
      for i in 0..<indices.length() {
        let (i0, i1, i2) = indices[i]
        if i0 < 0 || i1 < 0 || i2 < 0 {
          continue
        }
        if i0 >= vertices.length() ||
          i1 >= vertices.length() ||
          i2 >= vertices.length() {
          continue
        }
        let a = vertices[i0]
        let b0 = vertices[i1]
        let c0 = vertices[i2]
        let ab = b0.sub(a)
        let ac = c0.sub(a)
        let area2 = ab.x * ac.y - ab.y * ac.x
        let b = if area2 >= 0.0F { b0 } else { c0 }
        let c = if area2 >= 0.0F { c0 } else { b0 }
        let hit = ray_intersect_convex_polygon_oriented_and_get_normal(
          ray,
          center,
          rotation,
          [a, b, c],
          max_dist,
          solid,
        )
        if hit is Some(h) {
          if best is Some(cur) {
            if h.toi < cur.toi {
              best = Some(h)
            }
          } else {
            best = Some(h)
          }
        }
      }
      best
    }
    Shape::Compound(parts) => {
      if parts.length() == 0 {
        return None
      }
      let parent_iso = @core.Isometry2::new(
        center,
        @core.Rot2::from_angle(rotation),
      )
      let mut best : RayIntersection? = None
      for i in 0..<parts.length() {
        let (pose, part_shape) = parts[i]
        let w = parent_iso.mul(pose)
        let p = isometry2_translation(w)
        let r = isometry2_rotation_angle(w)
        let hit = ray_intersect_shape_and_get_normal(
          ray, p, r, part_shape, max_dist, solid,
        )
        if hit is Some(h) {
          if best is Some(cur) {
            if h.toi < cur.toi {
              best = Some(h)
            }
          } else {
            best = Some(h)
          }
        }
      }
      best
    }
  }
}

///|
fn project_point_on_shape(
  center : @core.Vec2,
  rotation : @core.Real,
  shape : Shape,
  point : @core.Vec2,
  solid : Bool,
) -> PointProjection {
  match shape {
    Shape::Ball(radius) => project_point_on_ball(center, radius, point, solid)
    Shape::Cuboid(hw, hh) =>
      project_point_on_cuboid_oriented(center, rotation, hw, hh, point, solid)
    Shape::CapsuleX(half_height, radius) =>
      project_point_on_capsule_oriented(
        center,
        rotation,
        @core.Vec2::new(1.0F, 0.0F),
        half_height,
        radius,
        point,
        solid,
      )
    Shape::CapsuleY(half_height, radius) =>
      project_point_on_capsule_oriented(
        center,
        rotation,
        @core.Vec2::new(0.0F, 1.0F),
        half_height,
        radius,
        point,
        solid,
      )
    Shape::Segment(a, b) =>
      project_point_on_segment(center, rotation, a, b, point)
    Shape::Polyline(vertices, indices) =>
      project_point_on_polyline(center, rotation, vertices, indices, point)
    Shape::ConvexPolygon(vertices) =>
      project_point_on_convex_polygon_oriented(
        center, rotation, vertices, point, solid,
      )
    Shape::TriMesh(vertices, indices) => {
      let mut best : PointProjection? = None
      let mut best_dist2 = 1.0e30F
      for i in 0..<indices.length() {
        let (i0, i1, i2) = indices[i]
        if i0 < 0 || i1 < 0 || i2 < 0 {
          continue
        }
        if i0 >= vertices.length() ||
          i1 >= vertices.length() ||
          i2 >= vertices.length() {
          continue
        }
        let a = vertices[i0]
        let b0 = vertices[i1]
        let c0 = vertices[i2]
        let ab = b0.sub(a)
        let ac = c0.sub(a)
        let area2 = ab.x * ac.y - ab.y * ac.x
        let b = if area2 >= 0.0F { b0 } else { c0 }
        let c = if area2 >= 0.0F { c0 } else { b0 }
        let proj = project_point_on_convex_polygon_oriented(
          center,
          rotation,
          [a, b, c],
          point,
          solid,
        )
        let delta = point.sub(proj.point)
        let dist2 = delta.dot(delta)
        if dist2 < best_dist2 {
          best_dist2 = dist2
          best = Some(proj)
        }
      }
      if best is Some(p) {
        p
      } else {
        PointProjection::{ point, is_inside: false }
      }
    }
    Shape::Compound(parts) => {
      if parts.length() == 0 {
        return PointProjection::{ point, is_inside: false }
      }
      let parent_iso = @core.Isometry2::new(
        center,
        @core.Rot2::from_angle(rotation),
      )
      let mut best : PointProjection? = None
      let mut best_dist2 = 1.0e30F
      for i in 0..<parts.length() {
        let (pose, part_shape) = parts[i]
        let w = parent_iso.mul(pose)
        let p = isometry2_translation(w)
        let r = isometry2_rotation_angle(w)
        let proj = project_point_on_shape(p, r, part_shape, point, solid)
        let delta = point.sub(proj.point)
        let dist2 = delta.dot(delta)
        if dist2 < best_dist2 {
          best_dist2 = dist2
          best = Some(proj)
        }
      }
      if best is Some(p) {
        p
      } else {
        PointProjection::{ point, is_inside: false }
      }
    }
  }
}

///|
fn project_point_on_shape_and_get_feature(
  center : @core.Vec2,
  rotation : @core.Real,
  shape : Shape,
  point : @core.Vec2,
  solid : Bool,
) -> (PointProjection, FeatureId) {
  fn pack_feature(prefix : Int, fid : FeatureId) -> FeatureId {
    if prefix == 0 {
      return fid
    }
    let base = prefix * 1000000
    match fid {
      FeatureId::Unknown => FeatureId::Face(base)
      FeatureId::Vertex(id) => FeatureId::Vertex(base + id)
      FeatureId::Edge(id) => FeatureId::Edge(base + id)
      FeatureId::Face(id) => FeatureId::Face(base + id)
    }
  }

  match shape {
    Shape::Ball(radius) =>
      project_point_on_ball_and_get_feature(center, radius, point, solid)
    Shape::Cuboid(hw, hh) =>
      project_point_on_cuboid_oriented_and_get_feature(
        center, rotation, hw, hh, point, solid,
      )
    Shape::CapsuleX(half_height, radius) =>
      project_point_on_capsule_oriented_and_get_feature(
        center,
        rotation,
        @core.Vec2::new(1.0F, 0.0F),
        half_height,
        radius,
        point,
        solid,
      )
    Shape::CapsuleY(half_height, radius) =>
      project_point_on_capsule_oriented_and_get_feature(
        center,
        rotation,
        @core.Vec2::new(0.0F, 1.0F),
        half_height,
        radius,
        point,
        solid,
      )
    Shape::Segment(a, b) =>
      project_point_on_segment_and_get_feature(center, rotation, a, b, point)
    Shape::Polyline(vertices, indices) =>
      project_point_on_polyline_and_get_feature(
        center, rotation, vertices, indices, point,
      )
    Shape::ConvexPolygon(vertices) =>
      project_point_on_convex_polygon_oriented_and_get_feature(
        center, rotation, vertices, point, solid,
      )
    Shape::TriMesh(vertices, indices) => {
      let mut best : (PointProjection, FeatureId)? = None
      let mut best_dist2 = 1.0e30F
      for i in 0..<indices.length() {
        let (i0, i1, i2) = indices[i]
        if i0 < 0 || i1 < 0 || i2 < 0 {
          continue
        }
        if i0 >= vertices.length() ||
          i1 >= vertices.length() ||
          i2 >= vertices.length() {
          continue
        }
        let a = vertices[i0]
        let b0 = vertices[i1]
        let c0 = vertices[i2]
        let ab = b0.sub(a)
        let ac = c0.sub(a)
        let area2 = ab.x * ac.y - ab.y * ac.x
        let b = if area2 >= 0.0F { b0 } else { c0 }
        let c = if area2 >= 0.0F { c0 } else { b0 }
        let (proj, feature) = project_point_on_convex_polygon_oriented_and_get_feature(
          center,
          rotation,
          [a, b, c],
          point,
          solid,
        )
        let delta = point.sub(proj.point)
        let dist2 = delta.dot(delta)
        if dist2 < best_dist2 {
          best_dist2 = dist2
          best = Some((proj, pack_feature(i + 1, feature)))
        }
      }
      if best is Some(r) {
        r
      } else {
        (PointProjection::{ point, is_inside: false }, FeatureId::Unknown)
      }
    }
    Shape::Compound(parts) => {
      if parts.length() == 0 {
        return (
          PointProjection::{ point, is_inside: false },
          FeatureId::Unknown,
        )
      }
      let parent_iso = @core.Isometry2::new(
        center,
        @core.Rot2::from_angle(rotation),
      )
      let mut best : (PointProjection, FeatureId)? = None
      let mut best_dist2 = 1.0e30F
      for i in 0..<parts.length() {
        let (pose, part_shape) = parts[i]
        let w = parent_iso.mul(pose)
        let p = isometry2_translation(w)
        let r = isometry2_rotation_angle(w)
        let (proj, feature) = project_point_on_shape_and_get_feature(
          p, r, part_shape, point, solid,
        )
        let delta = point.sub(proj.point)
        let dist2 = delta.dot(delta)
        if dist2 < best_dist2 {
          best_dist2 = dist2
          best = Some((proj, pack_feature(i + 1, feature)))
        }
      }
      if best is Some(result) {
        result
      } else {
        (PointProjection::{ point, is_inside: false }, FeatureId::Unknown)
      }
    }
  }
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
      let projection = project_point_on_shape(
        collider.world_translation,
        collider.world_rotation,
        collider.shape,
        point,
        solid,
      )
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
pub fn QueryPipeline::project_point_and_get_feature(
  self : QueryPipeline,
  point : @core.Vec2,
  max_dist : @core.Real,
  solid : Bool,
) -> (ColliderHandle, PointProjection, FeatureId)? {
  // Rapier keeps this parameter for API consistency, but doesn't use it.
  max_dist |> ignore
  let mut best : (ColliderHandle, PointProjection, FeatureId, @core.Real)? = None
  for i in 0..<self.colliders.colliders.length() {
    if self.colliders.colliders[i] is Some(collider) && collider.is_enabled() {
      let handle = ColliderHandle::new(i, self.colliders.generations[i])
      if !query_filter_test(self.filter, self.bodies, handle, collider) {
        continue
      }
      let (projection, feature) = project_point_on_shape_and_get_feature(
        collider.world_translation,
        collider.world_rotation,
        collider.shape,
        point,
        solid,
      )
      let delta = point.sub(projection.point)
      let dist2 = delta.dot(delta)
      if best is Some(current) {
        if dist2 < current.3 {
          best = Some((handle, projection, feature, dist2))
        }
      } else {
        best = Some((handle, projection, feature, dist2))
      }
    }
  }
  if best is Some(hit) {
    Some((hit.0, hit.1, hit.2))
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
