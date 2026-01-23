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
pub struct QueryFilter {
  excluded_body : @dynamics.RigidBodyHandle?
  excluded_collider : ColliderHandle?
  mut groups : @dynamics.InteractionGroups?
}

///|
pub fn QueryFilter::new() -> QueryFilter {
  { excluded_body: None, excluded_collider: None, groups: None }
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
  bodies |> ignore
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
    return collider.collision_groups().test_groups(groups)
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
      let toi = match collider.shape {
        Shape::Ball(radius) =>
          ray_intersect_ball(ray, pos, radius, max_dist, solid)
        Shape::Cuboid(hw, hh) =>
          ray_intersect_cuboid(ray, pos, hw, hh, max_dist, solid)
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
  point : @core.Vec2,
) -> Bool {
  match shape {
    Shape::Ball(radius) => point.sub(center).length_squared() <= radius * radius
    Shape::Cuboid(hw, hh) =>
      @core.abs(point.x - center.x) <= hw && @core.abs(point.y - center.y) <= hh
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
      if point_in_shape(collider.shape, collider.world_translation, point) {
        result.push(handle)
      }
    }
  }
  result
}
