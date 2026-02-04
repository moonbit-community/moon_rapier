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
/// Real 3D query pipeline (brute-force).
///
/// This query pipeline intentionally iterates all colliders (no BVH yet), but aims to match Rapier's
/// query API surface and semantics for common operations (ray casts, point projections, shape casts).
pub struct Ray3 {
  origin : @core.Vec3
  dir : @core.Vec3
}

///|
pub fn Ray3::new(origin : @core.Vec3, dir : @core.Vec3) -> Ray3 {
  { origin, dir }
}

///|
pub struct RayIntersection3 {
  toi : @core.Real
  normal : @core.Vec3
}

///|
pub fn RayIntersection3::toi(self : RayIntersection3) -> @core.Real {
  self.toi
}

///|
pub fn RayIntersection3::normal(self : RayIntersection3) -> @core.Vec3 {
  self.normal
}

///|
pub struct RayIntersection3Feature {
  toi : @core.Real
  normal : @core.Vec3
  feature_id : Int
}

///|
pub fn RayIntersection3Feature::toi(
  self : RayIntersection3Feature,
) -> @core.Real {
  self.toi
}

///|
pub fn RayIntersection3Feature::normal(
  self : RayIntersection3Feature,
) -> @core.Vec3 {
  self.normal
}

///|
pub fn RayIntersection3Feature::feature_id(
  self : RayIntersection3Feature,
) -> Int {
  self.feature_id
}

///|
pub struct PointProjection3 {
  point : @core.Vec3
  is_inside : Bool
}

///|
pub fn PointProjection3::point(self : PointProjection3) -> @core.Vec3 {
  self.point
}

///|
pub fn PointProjection3::is_inside(self : PointProjection3) -> Bool {
  self.is_inside
}

///|
pub struct ShapeCastHit3 {
  toi : @core.Real
  point : @core.Vec3
  normal : @core.Vec3
}

///|
pub fn ShapeCastHit3::toi(self : ShapeCastHit3) -> @core.Real {
  self.toi
}

///|
pub fn ShapeCastHit3::point(self : ShapeCastHit3) -> @core.Vec3 {
  self.point
}

///|
pub fn ShapeCastHit3::normal(self : ShapeCastHit3) -> @core.Vec3 {
  self.normal
}

///|
pub struct ShapeCastOptions3 {
  max_toi : @core.Real
  target_distance : @core.Real
  stop_at_penetration : Bool
}

///|
pub fn ShapeCastOptions3::new(
  max_toi : @core.Real,
  stop_at_penetration : Bool,
) -> ShapeCastOptions3 {
  { max_toi, target_distance: 0.0F, stop_at_penetration }
}

///|
pub fn ShapeCastOptions3::max_toi(self : ShapeCastOptions3) -> @core.Real {
  self.max_toi
}

///|
pub fn ShapeCastOptions3::target_distance(
  self : ShapeCastOptions3,
) -> @core.Real {
  self.target_distance
}

///|
pub fn ShapeCastOptions3::stop_at_penetration(self : ShapeCastOptions3) -> Bool {
  self.stop_at_penetration
}

///|
pub fn ShapeCastOptions3::with_target_distance(
  self : ShapeCastOptions3,
  target_distance : @core.Real,
) -> ShapeCastOptions3 {
  let td = if target_distance < 0.0F { 0.0F } else { target_distance }
  {
    max_toi: self.max_toi,
    target_distance: td,
    stop_at_penetration: self.stop_at_penetration,
  }
}

///|
const QUERY_FILTER_3D_EXCLUDE_FIXED : Int = 1 << 0

///|
const QUERY_FILTER_3D_EXCLUDE_KINEMATIC : Int = 1 << 1

///|
const QUERY_FILTER_3D_EXCLUDE_DYNAMIC : Int = 1 << 2

///|
const QUERY_FILTER_3D_EXCLUDE_SENSORS : Int = 1 << 3

///|
const QUERY_FILTER_3D_EXCLUDE_SOLIDS : Int = 1 << 4

///|
const QUERY_FILTER_3D_ONLY_DYNAMIC : Int = QUERY_FILTER_3D_EXCLUDE_FIXED |
  QUERY_FILTER_3D_EXCLUDE_KINEMATIC

///|
const QUERY_FILTER_3D_ONLY_KINEMATIC : Int = QUERY_FILTER_3D_EXCLUDE_DYNAMIC |
  QUERY_FILTER_3D_EXCLUDE_FIXED

///|
const QUERY_FILTER_3D_ONLY_FIXED : Int = QUERY_FILTER_3D_EXCLUDE_DYNAMIC |
  QUERY_FILTER_3D_EXCLUDE_KINEMATIC

///|
pub struct QueryFilter3DReal {
  excluded_body : @dynamics.RigidBodyHandle?
  excluded_collider : ColliderHandle3D?
  mut groups : @dynamics.InteractionGroups?
  mut flags : QueryFilterFlags
  mut predicate : ((ColliderHandle3D, Collider3D) -> Bool)?
}

///|
pub fn QueryFilter3DReal::new() -> QueryFilter3DReal {
  {
    excluded_body: None,
    excluded_collider: None,
    groups: None,
    flags: QueryFilterFlags::empty(),
    predicate: None,
  }
}

///|
pub fn QueryFilter3DReal::exclude_rigid_body(
  self : QueryFilter3DReal,
  handle : @dynamics.RigidBodyHandle,
) -> QueryFilter3DReal {
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
pub fn QueryFilter3DReal::exclude_collider(
  self : QueryFilter3DReal,
  handle : ColliderHandle3D,
) -> QueryFilter3DReal {
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
pub fn QueryFilter3DReal::groups(
  self : QueryFilter3DReal,
  groups : @dynamics.InteractionGroups,
) -> QueryFilter3DReal {
  self.groups = Some(groups)
  self
}

///|
pub fn QueryFilter3DReal::predicate(
  self : QueryFilter3DReal,
  predicate : (ColliderHandle3D, Collider3D) -> Bool,
) -> QueryFilter3DReal {
  self.predicate = Some(predicate)
  self
}

///|
pub fn QueryFilter3DReal::exclude_fixed(
  self : QueryFilter3DReal,
) -> QueryFilter3DReal {
  self.flags = self.flags.insert(QUERY_FILTER_3D_EXCLUDE_FIXED)
  self
}

///|
pub fn QueryFilter3DReal::exclude_kinematic(
  self : QueryFilter3DReal,
) -> QueryFilter3DReal {
  self.flags = self.flags.insert(QUERY_FILTER_3D_EXCLUDE_KINEMATIC)
  self
}

///|
pub fn QueryFilter3DReal::exclude_dynamic(
  self : QueryFilter3DReal,
) -> QueryFilter3DReal {
  self.flags = self.flags.insert(QUERY_FILTER_3D_EXCLUDE_DYNAMIC)
  self
}

///|
pub fn QueryFilter3DReal::exclude_sensors(
  self : QueryFilter3DReal,
) -> QueryFilter3DReal {
  self.flags = self.flags.insert(QUERY_FILTER_3D_EXCLUDE_SENSORS)
  self
}

///|
pub fn QueryFilter3DReal::exclude_solids(
  self : QueryFilter3DReal,
) -> QueryFilter3DReal {
  self.flags = self.flags.insert(QUERY_FILTER_3D_EXCLUDE_SOLIDS)
  self
}

///|
pub fn QueryFilter3DReal::only_dynamic() -> QueryFilter3DReal {
  let filter = QueryFilter3DReal::new()
  filter.flags = QueryFilterFlags::new(QUERY_FILTER_3D_ONLY_DYNAMIC)
  filter
}

///|
pub fn QueryFilter3DReal::only_kinematic() -> QueryFilter3DReal {
  let filter = QueryFilter3DReal::new()
  filter.flags = QueryFilterFlags::new(QUERY_FILTER_3D_ONLY_KINEMATIC)
  filter
}

///|
pub fn QueryFilter3DReal::only_fixed() -> QueryFilter3DReal {
  let filter = QueryFilter3DReal::new()
  filter.flags = QueryFilterFlags::new(QUERY_FILTER_3D_ONLY_FIXED)
  filter
}

///|
fn filter_pass_3d_real(
  filter : QueryFilter3DReal,
  bodies : @dynamics.RigidBodySet3D,
  handle : ColliderHandle3D,
  collider : Collider3D,
) -> Bool {
  if !collider.enabled() {
    return false
  }
  if filter.excluded_collider is Some(excluded) && excluded.equals(handle) {
    return false
  }
  if filter.excluded_body is Some(excluded_body) {
    if collider.parent() is Some(parent) && parent.equals(excluded_body) {
      return false
    }
  }
  if filter.groups is Some(groups) {
    if !collider.collision_groups().test_groups(groups) {
      return false
    }
  }
  if filter.flags.contains(QUERY_FILTER_3D_EXCLUDE_SENSORS) &&
    collider.is_sensor() {
    return false
  }
  if filter.flags.contains(QUERY_FILTER_3D_EXCLUDE_SOLIDS) &&
    !collider.is_sensor() {
    return false
  }
  if filter.flags.contains(QUERY_FILTER_3D_EXCLUDE_FIXED) {
    if collider.parent() is None {
      return false
    }
  }
  if collider.parent() is Some(parent) {
    if bodies.get(parent) is Some(body) {
      let body_type = body.body_type()
      if filter.flags.contains(QUERY_FILTER_3D_EXCLUDE_FIXED) &&
        body_type.is_fixed() {
        return false
      }
      if filter.flags.contains(QUERY_FILTER_3D_EXCLUDE_KINEMATIC) &&
        body_type.is_kinematic() {
        return false
      }
      if filter.flags.contains(QUERY_FILTER_3D_EXCLUDE_DYNAMIC) &&
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
pub struct QueryPipeline3DReal {
  filter : QueryFilter3DReal
  // Cached world-space AABBs to accelerate repeated queries (e.g. character controller).
  mut cached_aabbs : Array[@core.Aabb3?]
}

///|
pub struct QueryPipelineMut3DReal {
  pipeline : QueryPipeline3DReal
}

///|
pub fn QueryPipeline3DReal::new(
  filter : QueryFilter3DReal,
  bodies : @dynamics.RigidBodySet3D,
  colliders : ColliderSet3D,
) -> QueryPipeline3DReal {
  let qp = { filter, cached_aabbs: [] }
  qp.update(bodies, colliders)
  qp
}

///|
pub fn QueryPipeline3DReal::update(
  self : QueryPipeline3DReal,
  bodies : @dynamics.RigidBodySet3D,
  colliders : ColliderSet3D,
) -> Unit {
  colliders.sync_with_bodies(bodies)
  // Cache AABBs for all colliders (including fixed ones) to speed up repeated queries.
  let n = colliders.colliders.length()
  if self.cached_aabbs.length() != n {
    self.cached_aabbs = []
    for _ in 0..<n {
      self.cached_aabbs.push(None)
    }
  } else {
    for i in 0..<n {
      self.cached_aabbs[i] = None
    }
  }
  for i in 0..<n {
    if colliders.colliders[i] is Some(co) {
      self.cached_aabbs[i] = Some(co.compute_aabb())
    }
  }
}

///|
pub fn QueryPipeline3DReal::with_filter(
  self : QueryPipeline3DReal,
  filter : QueryFilter3DReal,
) -> QueryPipeline3DReal {
  { filter, cached_aabbs: self.cached_aabbs }
}

///|
pub fn QueryPipelineMut3DReal::new(
  filter : QueryFilter3DReal,
  bodies : @dynamics.RigidBodySet3D,
  colliders : ColliderSet3D,
) -> QueryPipelineMut3DReal {
  { pipeline: QueryPipeline3DReal::new(filter, bodies, colliders) }
}

///|
pub fn QueryPipelineMut3DReal::as_ref(
  self : QueryPipelineMut3DReal,
) -> QueryPipeline3DReal {
  self.pipeline
}

///|
pub fn QueryPipelineMut3DReal::update(
  self : QueryPipelineMut3DReal,
  bodies : @dynamics.RigidBodySet3D,
  colliders : ColliderSet3D,
) -> Unit {
  self.pipeline.update(bodies, colliders)
}

///|
pub fn QueryPipeline3DReal::rigid_body(
  self : QueryPipeline3DReal,
  bodies : @dynamics.RigidBodySet3D,
  handle : @dynamics.RigidBodyHandle,
) -> @dynamics.RigidBody3D? {
  self |> ignore
  bodies.get(handle)
}

///|
pub fn QueryPipeline3DReal::rigid_body_mut(
  self : QueryPipeline3DReal,
  bodies : @dynamics.RigidBodySet3D,
  handle : @dynamics.RigidBodyHandle,
) -> @dynamics.RigidBody3D? {
  self |> ignore
  bodies.get_mut(handle)
}

///|
pub fn QueryPipeline3DReal::collider(
  self : QueryPipeline3DReal,
  colliders : ColliderSet3D,
  handle : ColliderHandle3D,
) -> Collider3D? {
  self |> ignore
  colliders.get(handle)
}

///|
fn qp3d_real_sqrt(x : @core.Real) -> @core.Real {
  if x <= 0.0F {
    0.0F
  } else {
    Float::sqrt(x)
  }
}

///|
fn qp3d_real_normalized_or_default(v : @core.Vec3) -> @core.Vec3 {
  let n2 = v.length_squared()
  if n2 <= 1.0e-12F {
    @core.Vec3::new(1.0F, 0.0F, 0.0F)
  } else {
    v.scale(1.0F / Float::sqrt(n2))
  }
}

///|
fn ray_sphere_toi_normal(
  ray : Ray3,
  center : @core.Vec3,
  radius : @core.Real,
  max_toi : @core.Real,
) -> RayIntersection3? {
  let oc = ray.origin.sub(center)
  let a = ray.dir.length_squared()
  if a <= 1.0e-12F {
    return None
  }
  let b = oc.dot(ray.dir)
  let c = oc.length_squared() - radius * radius
  let disc = b * b - a * c
  if disc < 0.0F {
    return None
  }
  let s = qp3d_real_sqrt(disc)
  // Solve for the smallest positive root.
  let mut t = (-b - s) / a
  if t < 0.0F {
    t = (-b + s) / a
  }
  if t < 0.0F || t > max_toi {
    return None
  }
  let hit = ray.origin.add(ray.dir.scale(t))
  let n = qp3d_real_normalized_or_default(hit.sub(center))
  Some({ toi: t, normal: n })
}

///|
fn ray_obb_toi_normal(
  ray : Ray3,
  pos : @core.Isometry3,
  he : @core.Vec3,
  max_toi : @core.Real,
) -> RayIntersection3? {
  // Ray vs OBB via slab test in local-space.
  let inv = pos.inverse()
  let o = inv.transform_point(ray.origin)
  let d = pos.rotation.inverse().rotate_vec3(ray.dir)
  let mut tmin = 0.0F
  let mut tmax = max_toi
  let mut hit_normal = @core.Vec3::zero()
  fn update_slab(
    p0 : @core.Real,
    v : @core.Real,
    he : @core.Real,
    axis : @core.Vec3,
    tmin : @core.Real,
    tmax : @core.Real,
    hit_normal : @core.Vec3,
  ) -> (@core.Real, @core.Real, @core.Vec3)? {
    let mut tmin = tmin
    let mut tmax = tmax
    let mut hit_normal = hit_normal
    if @core.abs(v) <= 1.0e-12F {
      if p0 < -he || p0 > he {
        return None
      }
      return Some((tmin, tmax, hit_normal))
    }
    let inv_v = 1.0F / v
    let t1 = (-he - p0) * inv_v
    let t2 = (he - p0) * inv_v
    let (lo, hi, n_lo) = if t1 < t2 {
      (t1, t2, axis.scale(-1.0F))
    } else {
      (t2, t1, axis)
    }
    if lo > tmin {
      tmin = lo
      hit_normal = n_lo
    }
    if hi < tmax {
      tmax = hi
    }
    if tmin > tmax {
      None
    } else {
      Some((tmin, tmax, hit_normal))
    }
  }

  if update_slab(
      o.x,
      d.x,
      he.x,
      @core.Vec3::new(1.0F, 0.0F, 0.0F),
      tmin,
      tmax,
      hit_normal,
    )
    is Some(result) {
    tmin = result.0
    tmax = result.1
    hit_normal = result.2
  } else {
    return None
  }
  if update_slab(
      o.y,
      d.y,
      he.y,
      @core.Vec3::new(0.0F, 1.0F, 0.0F),
      tmin,
      tmax,
      hit_normal,
    )
    is Some(result) {
    tmin = result.0
    tmax = result.1
    hit_normal = result.2
  } else {
    return None
  }
  if update_slab(
      o.z,
      d.z,
      he.z,
      @core.Vec3::new(0.0F, 0.0F, 1.0F),
      tmin,
      tmax,
      hit_normal,
    )
    is Some(result) {
    tmin = result.0
    tmax = result.1
    hit_normal = result.2
  } else {
    return None
  }
  if tmax < 0.0F || tmin < 0.0F || tmin > max_toi {
    return None
  }
  // Transform normal back to world.
  let n = pos.rotation.rotate_vec3(hit_normal)
  Some({ toi: tmin, normal: qp3d_real_normalized_or_default(n) })
}

///|
fn orient_against_ray(n : @core.Vec3, dir : @core.Vec3) -> @core.Vec3 {
  if n.dot(dir) > 0.0F {
    n.scale(-1.0F)
  } else {
    n
  }
}

///|
fn ray_triangle_toi_normal(
  ray : Ray3,
  a : @core.Vec3,
  b : @core.Vec3,
  c : @core.Vec3,
  max_toi : @core.Real,
) -> RayIntersection3? {
  // Moller-Trumbore.
  let eps = 1.0e-9F
  let ab = b.sub(a)
  let ac = c.sub(a)
  let pvec = ray.dir.cross(ac)
  let det = ab.dot(pvec)
  if @core.abs(det) < eps {
    return None
  }
  let inv_det = 1.0F / det
  let tvec = ray.origin.sub(a)
  let u = tvec.dot(pvec) * inv_det
  if u < 0.0F || u > 1.0F {
    return None
  }
  let qvec = tvec.cross(ab)
  let v = ray.dir.dot(qvec) * inv_det
  if v < 0.0F || u + v > 1.0F {
    return None
  }
  let t = ac.dot(qvec) * inv_det
  if t < 0.0F || t > max_toi {
    return None
  }
  let n = qp3d_real_normalized_or_default(ab.cross(ac))
  Some({ toi: t, normal: orient_against_ray(n, ray.dir) })
}

///|
fn ray_plane_toi_normal(
  ray : Ray3,
  point_on_plane : @core.Vec3,
  normal : @core.Vec3,
  max_toi : @core.Real,
) -> RayIntersection3? {
  let eps = 1.0e-9F
  let denom = normal.dot(ray.dir)
  if @core.abs(denom) < eps {
    return None
  }
  let t = -normal.dot(ray.origin.sub(point_on_plane)) / denom
  if t < 0.0F || t > max_toi {
    return None
  }
  Some({ toi: t, normal: orient_against_ray(normal, ray.dir) })
}

///|
fn qp3d_real_hit_shape(
  ray : Ray3,
  pos : @core.Isometry3,
  shape : Shape3D,
  max_toi : @core.Real,
) -> RayIntersection3Feature? {
  match shape {
    Shape3D::Ball(r) =>
      if ray_sphere_toi_normal(ray, pos.translation, r, max_toi) is Some(it) {
        Some({ toi: it.toi, normal: it.normal, feature_id: -1 })
      } else {
        None
      }
    Shape3D::Cuboid(he) =>
      if ray_obb_toi_normal(ray, pos, he, max_toi) is Some(it) {
        Some({ toi: it.toi, normal: it.normal, feature_id: -1 })
      } else {
        None
      }
    Shape3D::CapsuleY(r, hh) =>
      if ray_capsule_y_toi_normal(ray, pos, r, hh, max_toi) is Some(it) {
        Some({ toi: it.toi, normal: it.normal, feature_id: -1 })
      } else {
        None
      }
    Shape3D::Cylinder(r, hh) =>
      if ray_cylinder_y_toi_normal(ray, pos, r, hh, max_toi) is Some(it) {
        Some({ toi: it.toi, normal: it.normal, feature_id: -1 })
      } else {
        None
      }
    Shape3D::RoundCylinder(r, hh, br) =>
      if ray_cylinder_y_toi_normal(ray, pos, r + br, hh + br, max_toi)
        is Some(it) {
        Some({ toi: it.toi, normal: it.normal, feature_id: -1 })
      } else {
        None
      }
    Shape3D::Cone(r, hh) =>
      if ray_cone_y_toi_normal(ray, pos, r, hh, max_toi) is Some(it) {
        Some({ toi: it.toi, normal: it.normal, feature_id: -1 })
      } else {
        None
      }
    Shape3D::HalfSpace(n_local) => {
      let n = qp3d_real_normalized_or_default(pos.rotation.rotate_vec3(n_local))
      if ray_plane_toi_normal(ray, pos.translation, n, max_toi) is Some(it) {
        Some({ toi: it.toi, normal: it.normal, feature_id: -1 })
      } else {
        None
      }
    }
    Shape3D::Triangle(a, b, c) => {
      let wa = pos.transform_point(a)
      let wb = pos.transform_point(b)
      let wc = pos.transform_point(c)
      if ray_triangle_toi_normal(ray, wa, wb, wc, max_toi) is Some(it) {
        Some({ toi: it.toi, normal: it.normal, feature_id: 0 })
      } else {
        None
      }
    }
    Shape3D::ConvexHull(points, br) => {
      if points.length() == 0 {
        return None
      }
      let shape0 = Shape3D::ConvexHull(points, br)
      let eps = 1.0e-4F
      let mut t = 0.0F
      let mut t_prev = 0.0F
      for _ in 0..<64 {
        if t < 0.0F {
          t = 0.0F
        }
        if t > max_toi {
          break
        }
        let p = ray.origin.add(ray.dir.scale(t))
        let (_, d2) = qp3d_real_project_point_on_shape(pos, shape0, p, false)
        let dist = Float::sqrt(d2)
        if dist <= eps {
          // Refine via bisection on [t_prev, t].
          let mut lo = t_prev
          let mut hi = t
          for _ in 0..<20 {
            let mid = (lo + hi) * 0.5F
            let pm = ray.origin.add(ray.dir.scale(mid))
            let (_, d2m) = qp3d_real_project_point_on_shape(
              pos, shape0, pm, false,
            )
            if Float::sqrt(d2m) <= eps {
              hi = mid
            } else {
              lo = mid
            }
          }
          let toi = hi
          let hit_p = ray.origin.add(ray.dir.scale(toi))
          let (proj2, _) = qp3d_real_project_point_on_shape(
            pos, shape0, hit_p, false,
          )
          let n0 = qp3d_real_normalized_or_default(hit_p.sub(proj2.point))
          let n = orient_against_ray(n0, ray.dir)
          return Some({ toi, normal: n, feature_id: -1 })
        }
        t_prev = t
        t = t + dist
      }
      None
    }
    Shape3D::Voxels(v) => {
      let sx = v.voxel_size.x
      let sy = v.voxel_size.y
      let sz = v.voxel_size.z
      if sx <= 0.0F || sy <= 0.0F || sz <= 0.0F {
        return None
      }
      let inv = pos.inverse()
      let local_ray = Ray3::new(
        inv.transform_point(ray.origin),
        pos.rotation.inverse().rotate_vec3(ray.dir),
      )
      let (ix0, iy0, iz0) = v.min_key
      let (ix1, iy1, iz1) = v.max_key
      let min = @core.Vec3::new(
        Float::from_int(ix0) * sx,
        Float::from_int(iy0) * sy,
        Float::from_int(iz0) * sz,
      )
      let max = @core.Vec3::new(
        Float::from_int(ix1 + 1) * sx,
        Float::from_int(iy1 + 1) * sy,
        Float::from_int(iz1 + 1) * sz,
      )

      // Slab intersection with the voxel bounds.
      let mut t_min = 0.0F
      let mut t_max = max_toi
      fn slab(
        o : @core.Real,
        d : @core.Real,
        lo : @core.Real,
        hi : @core.Real,
        t_min : @core.Real,
        t_max : @core.Real,
      ) -> (@core.Real, @core.Real, Bool) {
        if @core.abs(d) <= 1.0e-12F {
          if o < lo || o > hi {
            (t_min, t_max, false)
          } else {
            (t_min, t_max, true)
          }
        } else {
          let t1 = (lo - o) / d
          let t2 = (hi - o) / d
          let a = if t1 < t2 { t1 } else { t2 }
          let b = if t1 > t2 { t1 } else { t2 }
          let tmin2 = if a > t_min { a } else { t_min }
          let tmax2 = if b < t_max { b } else { t_max }
          (tmin2, tmax2, tmin2 <= tmax2)
        }
      }

      let (t_min2, t_max2, okx) = slab(
        local_ray.origin.x,
        local_ray.dir.x,
        min.x,
        max.x,
        t_min,
        t_max,
      )
      if !okx {
        return None
      }
      t_min = t_min2
      t_max = t_max2
      let (t_min3, t_max3, oky) = slab(
        local_ray.origin.y,
        local_ray.dir.y,
        min.y,
        max.y,
        t_min,
        t_max,
      )
      if !oky {
        return None
      }
      t_min = t_min3
      t_max = t_max3
      let (t_min4, t_max4, okz) = slab(
        local_ray.origin.z,
        local_ray.dir.z,
        min.z,
        max.z,
        t_min,
        t_max,
      )
      if !okz {
        return None
      }
      t_min = t_min4
      t_max = t_max4
      if t_max < 0.0F {
        return None
      }
      let t0 = if t_min < 0.0F { 0.0F } else { t_min }
      fn floor_div_to_int(value : @core.Real, denom : @core.Real) -> Int {
        if denom == 0.0F {
          0
        } else {
          @math.floor((value / denom).to_double()).to_int()
        }
      }

      // Fast-path for (almost) vertical rays: scan the (ix, iz) column directly.
      // This avoids edge-case failures of the generic DDA traversal on degenerate directions.
      let eps_dir = 1.0e-12F
      if @core.abs(local_ray.dir.x) <= eps_dir &&
        @core.abs(local_ray.dir.z) <= eps_dir &&
        @core.abs(local_ray.dir.y) > eps_dir {
        let ixv = floor_div_to_int(local_ray.origin.x, sx)
        let izv = floor_div_to_int(local_ray.origin.z, sz)
        if ixv < ix0 || ixv > ix1 || izv < iz0 || izv > iz1 {
          return None
        }
        if v.column_ys(ixv, izv) is Some(ys) {
          let mut best_t = max_toi + 1.0F
          let mut best_key : (Int, Int, Int)? = None
          for i in 0..<ys.length() {
            let iyv = ys[i]
            let face_y = if local_ray.dir.y < 0.0F {
              Float::from_int(iyv + 1) * sy
            } else {
              Float::from_int(iyv) * sy
            }
            let t_hit = (face_y - local_ray.origin.y) / local_ray.dir.y
            if t_hit < 0.0F || t_hit < t0 || t_hit > t_max || t_hit > max_toi {
              continue
            }
            if t_hit < best_t {
              best_t = t_hit
              best_key = Some((ixv, iyv, izv))
            }
          }
          if best_key is Some(key) {
            let local_n = if local_ray.dir.y < 0.0F {
              @core.Vec3::new(0.0F, 1.0F, 0.0F)
            } else {
              @core.Vec3::new(0.0F, -1.0F, 0.0F)
            }
            let n_world = qp3d_real_normalized_or_default(
              pos.rotation.rotate_vec3(local_n),
            )
            let fid = if v.key_to_id.get(key) is Some(id) { id } else { -1 }
            return Some({
              toi: best_t,
              normal: orient_against_ray(n_world, ray.dir),
              feature_id: fid,
            })
          }
        }
      }

      // Start slightly inside to avoid boundary ambiguity.
      let p0 = local_ray.origin.add(local_ray.dir.scale(t0 + 1.0e-6F))
      let mut ix = floor_div_to_int(p0.x, sx)
      let mut iy = floor_div_to_int(p0.y, sy)
      let mut iz = floor_div_to_int(p0.z, sz)
      let step_x = if local_ray.dir.x > 0.0F {
        1
      } else if local_ray.dir.x < 0.0F {
        -1
      } else {
        0
      }
      let step_y = if local_ray.dir.y > 0.0F {
        1
      } else if local_ray.dir.y < 0.0F {
        -1
      } else {
        0
      }
      let step_z = if local_ray.dir.z > 0.0F {
        1
      } else if local_ray.dir.z < 0.0F {
        -1
      } else {
        0
      }
      let inf = 1.0e30F
      let mut t_max_x = inf
      let mut t_max_y = inf
      let mut t_max_z = inf
      let mut t_delta_x = inf
      let mut t_delta_y = inf
      let mut t_delta_z = inf
      if step_x != 0 {
        let next_x = if step_x > 0 {
          Float::from_int(ix + 1) * sx
        } else {
          Float::from_int(ix) * sx
        }
        t_max_x = (next_x - local_ray.origin.x) / local_ray.dir.x
        t_delta_x = sx / @core.abs(local_ray.dir.x)
      }
      if step_y != 0 {
        let next_y = if step_y > 0 {
          Float::from_int(iy + 1) * sy
        } else {
          Float::from_int(iy) * sy
        }
        t_max_y = (next_y - local_ray.origin.y) / local_ray.dir.y
        t_delta_y = sy / @core.abs(local_ray.dir.y)
      }
      if step_z != 0 {
        let next_z = if step_z > 0 {
          Float::from_int(iz + 1) * sz
        } else {
          Float::from_int(iz) * sz
        }
        t_max_z = (next_z - local_ray.origin.z) / local_ray.dir.z
        t_delta_z = sz / @core.abs(local_ray.dir.z)
      }
      let mut t = t0
      let mut last_axis = -1
      let mut last_step = 0
      for _ in 0..<131072 {
        if t > t_max || t > max_toi {
          break
        }
        if ix < ix0 || ix > ix1 || iy < iy0 || iy > iy1 || iz < iz0 || iz > iz1 {
          break
        }
        let mut hit = false
        if v.column_ys(ix, iz) is Some(ys) {
          for i in 0..<ys.length() {
            if ys[i] == iy {
              hit = true
              break
            }
          }
        }
        if hit {
          let key = (ix, iy, iz)
          let local_n = if last_axis == 0 {
            @core.Vec3::new(Float::from_int(-last_step), 0.0F, 0.0F)
          } else if last_axis == 1 {
            @core.Vec3::new(0.0F, Float::from_int(-last_step), 0.0F)
          } else if last_axis == 2 {
            @core.Vec3::new(0.0F, 0.0F, Float::from_int(-last_step))
          } else {
            qp3d_real_normalized_or_default(local_ray.dir).scale(-1.0F)
          }
          let n_world = qp3d_real_normalized_or_default(
            pos.rotation.rotate_vec3(local_n),
          )
          let fid = if v.key_to_id.get(key) is Some(id) { id } else { -1 }
          return Some({
            toi: t,
            normal: orient_against_ray(n_world, ray.dir),
            feature_id: fid,
          })
        }
        if t_max_x <= t_max_y && t_max_x <= t_max_z {
          ix = ix + step_x
          t = t_max_x
          t_max_x = t_max_x + t_delta_x
          last_axis = 0
          last_step = step_x
        } else if t_max_y <= t_max_z {
          iy = iy + step_y
          t = t_max_y
          t_max_y = t_max_y + t_delta_y
          last_axis = 1
          last_step = step_y
        } else {
          iz = iz + step_z
          t = t_max_z
          t_max_z = t_max_z + t_delta_z
          last_axis = 2
          last_step = step_z
        }
      }
      None
    }
    Shape3D::Compound(parts) => {
      let mut best : RayIntersection3Feature? = None
      let mut best_t = max_toi + 1.0F
      for i in 0..<parts.length() {
        let (iso, sh) = parts[i]
        let child_pos = pos.mul(iso)
        if qp3d_real_hit_shape(ray, child_pos, sh, max_toi) is Some(it) &&
          it.toi < best_t {
          best_t = it.toi
          best = Some(it)
        }
      }
      best
    }
    Shape3D::Heightfield(vertices, _, nrows, ncols) => {
      if nrows <= 1 || ncols <= 1 || vertices.length() != nrows * ncols {
        return None
      }
      let inv = pos.inverse()
      let local_ray = Ray3::new(
        inv.transform_point(ray.origin),
        pos.rotation.inverse().rotate_vec3(ray.dir),
      )
      let origin_x = vertices[0].x
      let origin_z = vertices[0].z
      let dx0 = vertices[ncols].x - origin_x
      let dz0 = vertices[1].z - origin_z
      let dx = if dx0 < 0.0F { -dx0 } else { dx0 }
      let dz = if dz0 < 0.0F { -dz0 } else { dz0 }
      if dx <= 1.0e-12F || dz <= 1.0e-12F {
        return None
      }
      let max_x = origin_x + Float::from_int(nrows - 1) * dx
      let max_z = origin_z + Float::from_int(ncols - 1) * dz
      let min_x = if origin_x < max_x { origin_x } else { max_x }
      let max_x2 = if origin_x > max_x { origin_x } else { max_x }
      let min_z = if origin_z < max_z { origin_z } else { max_z }
      let max_z2 = if origin_z > max_z { origin_z } else { max_z }

      // Intersect with the heightfield xz bounds to find the segment of the ray that
      // can possibly hit the terrain.
      let eps = 1.0e-12F
      let mut t_min = 0.0F
      let mut t_max = max_toi
      if @core.abs(local_ray.dir.x) <= eps {
        if local_ray.origin.x < min_x || local_ray.origin.x > max_x2 {
          return None
        }
      } else {
        let tx1 = (min_x - local_ray.origin.x) / local_ray.dir.x
        let tx2 = (max_x2 - local_ray.origin.x) / local_ray.dir.x
        let t0 = if tx1 < tx2 { tx1 } else { tx2 }
        let t1 = if tx1 > tx2 { tx1 } else { tx2 }
        if t0 > t_min {
          t_min = t0
        }
        if t1 < t_max {
          t_max = t1
        }
      }
      if @core.abs(local_ray.dir.z) <= eps {
        if local_ray.origin.z < min_z || local_ray.origin.z > max_z2 {
          return None
        }
      } else {
        let tz1 = (min_z - local_ray.origin.z) / local_ray.dir.z
        let tz2 = (max_z2 - local_ray.origin.z) / local_ray.dir.z
        let t0 = if tz1 < tz2 { tz1 } else { tz2 }
        let t1 = if tz1 > tz2 { tz1 } else { tz2 }
        if t0 > t_min {
          t_min = t0
        }
        if t1 < t_max {
          t_max = t1
        }
      }
      if t_min > t_max || t_max < 0.0F {
        return None
      }
      let t_enter = if t_min > 0.0F { t_min } else { 0.0F }
      if t_enter > max_toi {
        return None
      }

      // Determine initial cell at the ray entry point.
      let p_enter = local_ray.origin.add(local_ray.dir.scale(t_enter))
      fn floor_to_int(x : @core.Real) -> Int {
        let i = x.to_int()
        if Float::from_int(i) > x {
          i - 1
        } else {
          i
        }
      }

      fn clamp_int(v : Int, lo : Int, hi : Int) -> Int {
        if v < lo {
          lo
        } else if v > hi {
          hi
        } else {
          v
        }
      }

      let cells_x = nrows - 1
      let cells_z = ncols - 1
      let mut i = clamp_int(
        floor_to_int((p_enter.x - origin_x) / dx),
        0,
        cells_x - 1,
      )
      let mut j = clamp_int(
        floor_to_int((p_enter.z - origin_z) / dz),
        0,
        cells_z - 1,
      )
      let mut best : RayIntersection3? = None
      let mut best_t = max_toi + 1.0F
      let mut best_feature = -1
      let inf = 1.0e30F
      let mut step_x = 0
      let mut t_max_x = inf
      let mut t_delta_x = inf
      if @core.abs(local_ray.dir.x) > eps {
        if local_ray.dir.x > 0.0F {
          step_x = 1
          let boundary = origin_x + Float::from_int(i + 1) * dx
          t_max_x = (boundary - local_ray.origin.x) / local_ray.dir.x
        } else {
          step_x = -1
          let boundary = origin_x + Float::from_int(i) * dx
          t_max_x = (boundary - local_ray.origin.x) / local_ray.dir.x
        }
        t_delta_x = dx / @core.abs(local_ray.dir.x)
        while t_max_x < t_enter {
          t_max_x = t_max_x + t_delta_x
        }
      }
      let mut step_z = 0
      let mut t_max_z = inf
      let mut t_delta_z = inf
      if @core.abs(local_ray.dir.z) > eps {
        if local_ray.dir.z > 0.0F {
          step_z = 1
          let boundary = origin_z + Float::from_int(j + 1) * dz
          t_max_z = (boundary - local_ray.origin.z) / local_ray.dir.z
        } else {
          step_z = -1
          let boundary = origin_z + Float::from_int(j) * dz
          t_max_z = (boundary - local_ray.origin.z) / local_ray.dir.z
        }
        t_delta_z = dz / @core.abs(local_ray.dir.z)
        while t_max_z < t_enter {
          t_max_z = t_max_z + t_delta_z
        }
      }
      let mut t_limit = if t_max < max_toi { t_max } else { max_toi }
      if t_limit < t_enter {
        t_limit = t_enter
      }

      // Traverse cells in increasing ray parameter order (2D DDA over the grid).
      let mut t_cur = t_enter
      while t_cur <= t_limit && i >= 0 && i < cells_x && j >= 0 && j < cells_z {
        let next_boundary = {
          let nx = if t_max_x < t_max_z { t_max_x } else { t_max_z }
          if nx < t_limit {
            nx
          } else {
            t_limit
          }
        }
        let tri_max = if best_t < next_boundary {
          best_t
        } else {
          next_boundary
        }

        // Test the two triangles of this cell in local-space.
        let a = vertices[i * ncols + j]
        let b = vertices[(i + 1) * ncols + j]
        let c = vertices[(i + 1) * ncols + (j + 1)]
        let d = vertices[i * ncols + (j + 1)]
        if ray_triangle_toi_normal(local_ray, a, c, b, tri_max) is Some(hit0) &&
          hit0.toi + 1.0e-9F >= t_cur &&
          hit0.toi < best_t {
          best_t = hit0.toi
          best = Some(hit0)
          best_feature = (i * (ncols - 1) + j) * 2
        }
        if ray_triangle_toi_normal(local_ray, a, d, c, tri_max) is Some(hit1) &&
          hit1.toi + 1.0e-9F >= t_cur &&
          hit1.toi < best_t {
          best_t = hit1.toi
          best = Some(hit1)
          best_feature = (i * (ncols - 1) + j) * 2 + 1
        }
        if best_t <= next_boundary {
          break
        }

        // Advance to the next cell.
        t_cur = next_boundary
        if t_max_x < t_max_z {
          i = i + step_x
          t_max_x = t_max_x + t_delta_x
        } else {
          j = j + step_z
          t_max_z = t_max_z + t_delta_z
        }
        if next_boundary >= t_limit {
          break
        }
      }
      if best is Some(hit) && best_t <= max_toi {
        Some({
          toi: best_t,
          normal: pos.rotation.rotate_vec3(hit.normal),
          feature_id: best_feature,
        })
      } else {
        None
      }
    }
    Shape3D::TriMesh(vertices, indices) => {
      let mut best_local : RayIntersection3? = None
      let mut best_local_t = max_toi + 1.0F
      let mut best_tri = -1
      for j in 0..<indices.length() {
        let (i0, i1, i2) = indices[j]
        let a = pos.transform_point(vertices[i0])
        let b = pos.transform_point(vertices[i1])
        let c = pos.transform_point(vertices[i2])
        if ray_triangle_toi_normal(ray, a, b, c, max_toi) is Some(it) {
          if it.toi < best_local_t {
            best_local_t = it.toi
            best_local = Some(it)
            best_tri = j
          }
        }
      }
      if best_local is Some(it) {
        Some({ toi: it.toi, normal: it.normal, feature_id: best_tri })
      } else {
        None
      }
    }
  }
}

///|
fn ray_capsule_y_toi_normal(
  ray : Ray3,
  pos : @core.Isometry3,
  radius : @core.Real,
  half_height : @core.Real,
  max_toi : @core.Real,
) -> RayIntersection3? {
  let inv = pos.inverse()
  let o = inv.transform_point(ray.origin)
  let d = pos.rotation.inverse().rotate_vec3(ray.dir)
  let hh = half_height
  let r = radius
  let eps = 1.0e-9F
  let mut best : RayIntersection3? = None
  let mut best_t = max_toi + 1.0F

  // Side cylinder.
  let a = d.x * d.x + d.z * d.z
  if a > eps {
    let b = o.x * d.x + o.z * d.z
    let c = o.x * o.x + o.z * o.z - r * r
    let disc = b * b - a * c
    if disc >= 0.0F {
      let s = qp3d_real_sqrt(disc)
      let mut t = (-b - s) / a
      if t < 0.0F {
        t = (-b + s) / a
      }
      if t >= 0.0F && t <= max_toi {
        let y = o.y + d.y * t
        if y >= -hh && y <= hh {
          let hit = @core.Vec3::new(o.x + d.x * t, y, o.z + d.z * t)
          let n_local = qp3d_real_normalized_or_default(
            @core.Vec3::new(hit.x, 0.0F, hit.z),
          )
          let n_world = pos.rotation.rotate_vec3(n_local)
          best_t = t
          best = Some({ toi: t, normal: orient_against_ray(n_world, ray.dir) })
        }
      }
    }
  }

  // Caps as spheres.
  fn best_sphere(
    ray : Ray3,
    pos : @core.Isometry3,
    o : @core.Vec3,
    d : @core.Vec3,
    center_local : @core.Vec3,
    r : @core.Real,
    max_toi : @core.Real,
    best_t : @core.Real,
    best : RayIntersection3?,
  ) -> (@core.Real, RayIntersection3?) {
    let ray_local = Ray3::new(o, d)
    if ray_sphere_toi_normal(ray_local, center_local, r, max_toi) is Some(it) {
      if it.toi < best_t {
        let hit = o.add(d.scale(it.toi))
        let n_local = qp3d_real_normalized_or_default(hit.sub(center_local))
        let n_world = pos.rotation.rotate_vec3(n_local)
        (
          it.toi,
          Some({ toi: it.toi, normal: orient_against_ray(n_world, ray.dir) }),
        )
      } else {
        (best_t, best)
      }
    } else {
      (best_t, best)
    }
  }

  let (t1, b1) = best_sphere(
    ray,
    pos,
    o,
    d,
    @core.Vec3::new(0.0F, hh, 0.0F),
    r,
    max_toi,
    best_t,
    best,
  )
  best_t = t1
  best = b1
  let (t2, b2) = best_sphere(
    ray,
    pos,
    o,
    d,
    @core.Vec3::new(0.0F, -hh, 0.0F),
    r,
    max_toi,
    best_t,
    best,
  )
  best_t = t2
  best = b2
  best
}

///|
fn ray_cylinder_y_toi_normal(
  ray : Ray3,
  pos : @core.Isometry3,
  radius : @core.Real,
  half_height : @core.Real,
  max_toi : @core.Real,
) -> RayIntersection3? {
  let inv = pos.inverse()
  let o = inv.transform_point(ray.origin)
  let d = pos.rotation.inverse().rotate_vec3(ray.dir)
  let hh = half_height
  let r = radius
  let eps = 1.0e-9F
  let mut best : RayIntersection3? = None
  let mut best_t = max_toi + 1.0F

  // Side cylinder.
  let a = d.x * d.x + d.z * d.z
  if a > eps {
    let b = o.x * d.x + o.z * d.z
    let c = o.x * o.x + o.z * o.z - r * r
    let disc = b * b - a * c
    if disc >= 0.0F {
      let s = qp3d_real_sqrt(disc)
      let mut t = (-b - s) / a
      if t < 0.0F {
        t = (-b + s) / a
      }
      if t >= 0.0F && t <= max_toi {
        let y = o.y + d.y * t
        if y >= -hh && y <= hh {
          let hit = @core.Vec3::new(o.x + d.x * t, y, o.z + d.z * t)
          let n_local = qp3d_real_normalized_or_default(
            @core.Vec3::new(hit.x, 0.0F, hit.z),
          )
          let n_world = pos.rotation.rotate_vec3(n_local)
          best_t = t
          best = Some({ toi: t, normal: orient_against_ray(n_world, ray.dir) })
        }
      }
    }
  }

  // Caps (disks).
  if @core.abs(d.y) > eps {
    for cap_y in [-hh, hh] {
      let t = (cap_y - o.y) / d.y
      if t >= 0.0F && t <= max_toi && t < best_t {
        let x = o.x + d.x * t
        let z = o.z + d.z * t
        if x * x + z * z <= r * r + 1.0e-6F {
          let n_local = @core.Vec3::new(
            0.0F,
            if cap_y > 0.0F {
              1.0F
            } else {
              -1.0F
            },
            0.0F,
          )
          let n_world = pos.rotation.rotate_vec3(n_local)
          best_t = t
          best = Some({ toi: t, normal: orient_against_ray(n_world, ray.dir) })
        }
      }
    }
  }
  best
}

///|
fn ray_cone_y_toi_normal(
  ray : Ray3,
  pos : @core.Isometry3,
  radius : @core.Real,
  half_height : @core.Real,
  max_toi : @core.Real,
) -> RayIntersection3? {
  // Cone with apex at y=+hh and base at y=-hh, aligned with local Y.
  let inv = pos.inverse()
  let o = inv.transform_point(ray.origin)
  let d = pos.rotation.inverse().rotate_vec3(ray.dir)
  let hh = half_height
  let r = radius
  let eps = 1.0e-9F
  if hh <= 1.0e-6F {
    return None
  }
  let k = r / (2.0F * hh) // radius per unit of (hh - y)
  let k2 = k * k

  // Solve (x^2+z^2) - k^2*(hh - y)^2 = 0.
  let ox = o.x
  let oy = o.y
  let oz = o.z
  let dx = d.x
  let dy = d.y
  let dz = d.z
  let fy0 = hh - oy
  let a = dx * dx + dz * dz - k2 * (dy * dy)
  let b = ox * dx + oz * dz + k2 * fy0 * dy
  let c = ox * ox + oz * oz - k2 * (fy0 * fy0)
  let mut best : RayIntersection3? = None
  let mut best_t = max_toi + 1.0F

  // Side intersection.
  if @core.abs(a) > eps {
    let disc = b * b - a * c
    if disc >= 0.0F {
      let s = qp3d_real_sqrt(disc)
      let mut t = (-b - s) / a
      if t < 0.0F {
        t = (-b + s) / a
      }
      if t >= 0.0F && t <= max_toi {
        let y = oy + dy * t
        if y >= -hh && y <= hh {
          let x = ox + dx * t
          let z = oz + dz * t
          let fy = hh - y
          if fy >= 0.0F {
            let n_local = qp3d_real_normalized_or_default(
              @core.Vec3::new(x, k2 * fy, z),
            )
            let n_world = pos.rotation.rotate_vec3(n_local)
            best_t = t
            best = Some({ toi: t, normal: orient_against_ray(n_world, ray.dir) })
          }
        }
      }
    }
  }

  // Base disk at y=-hh.
  if @core.abs(dy) > eps {
    let t = (-hh - oy) / dy
    if t >= 0.0F && t <= max_toi && t < best_t {
      let x = ox + dx * t
      let z = oz + dz * t
      if x * x + z * z <= r * r + 1.0e-6F {
        let n_local = @core.Vec3::new(0.0F, -1.0F, 0.0F)
        let n_world = pos.rotation.rotate_vec3(n_local)
        best_t = t
        best = Some({ toi: t, normal: orient_against_ray(n_world, ray.dir) })
      }
    }
  }
  best
}

///|
pub fn QueryPipeline3DReal::cast_ray_and_get_normal(
  self : QueryPipeline3DReal,
  bodies : @dynamics.RigidBodySet3D,
  colliders : ColliderSet3D,
  ray : Ray3,
  max_toi : @core.Real,
  solid : Bool,
) -> (ColliderHandle3D, RayIntersection3)? {
  if QueryPipeline3DReal::cast_ray_and_get_normal_and_feature(
      self, bodies, colliders, ray, max_toi, solid,
    )
    is Some((h, it)) {
    Some((h, { toi: it.toi, normal: it.normal }))
  } else {
    None
  }
}

///|
pub fn QueryPipeline3DReal::cast_ray(
  self : QueryPipeline3DReal,
  bodies : @dynamics.RigidBodySet3D,
  colliders : ColliderSet3D,
  ray : Ray3,
  max_toi : @core.Real,
  solid : Bool,
) -> (ColliderHandle3D, @core.Real)? {
  if QueryPipeline3DReal::cast_ray_and_get_normal_and_feature(
      self, bodies, colliders, ray, max_toi, solid,
    )
    is Some((h, it)) {
    Some((h, it.toi))
  } else {
    None
  }
}

///|
pub fn QueryPipeline3DReal::intersect_ray(
  self : QueryPipeline3DReal,
  bodies : @dynamics.RigidBodySet3D,
  colliders : ColliderSet3D,
  ray : Ray3,
  max_toi : @core.Real,
  solid : Bool,
) -> Array[(ColliderHandle3D, RayIntersection3Feature)] {
  solid |> ignore
  let max_dist = if max_toi < 0.0F { 0.0F } else { max_toi }
  let results : Array[(ColliderHandle3D, RayIntersection3Feature)] = []
  for h in colliders.all_handles() {
    if colliders.get(h) is Some(co) {
      if !filter_pass_3d_real(self.filter, bodies, h, co) {
        continue
      }
      if qp3d_real_hit_shape(ray, co.position(), co.shape(), max_dist)
        is Some(it) {
        results.push((h, it))
      }
    }
  }
  results
}

///|
pub fn QueryPipeline3DReal::cast_ray_and_get_normal_and_feature(
  self : QueryPipeline3DReal,
  bodies : @dynamics.RigidBodySet3D,
  colliders : ColliderSet3D,
  ray : Ray3,
  max_toi : @core.Real,
  solid : Bool,
) -> (ColliderHandle3D, RayIntersection3Feature)? {
  solid |> ignore
  let handles = colliders.all_handles()
  let mut best_t = max_toi + 1.0F
  let mut best : (ColliderHandle3D, RayIntersection3Feature)? = None
  for i in 0..<handles.length() {
    let h = handles[i]
    if colliders.get(h) is Some(co) {
      if !filter_pass_3d_real(self.filter, bodies, h, co) {
        continue
      }
      let pos = co.position()
      let hit = qp3d_real_hit_shape(ray, pos, co.shape(), max_toi)
      if hit is Some(it) && it.toi < best_t {
        best_t = it.toi
        best = Some((h, it))
      }
    }
  }
  best
}

///|
pub fn QueryPipeline3DReal::cast_ray_and_get_voxel_key(
  self : QueryPipeline3DReal,
  bodies : @dynamics.RigidBodySet3D,
  colliders : ColliderSet3D,
  ray : Ray3,
  max_toi : @core.Real,
  solid : Bool,
) -> (ColliderHandle3D, RayIntersection3, (Int, Int, Int))? {
  if QueryPipeline3DReal::cast_ray_and_get_normal_and_feature(
      self, bodies, colliders, ray, max_toi, solid,
    )
    is Some((h, it)) {
    if it.feature_id >= 0 && colliders.get(h) is Some(co) {
      if co.voxel_key_for_triangle(it.feature_id) is Some(key) {
        return Some((h, { toi: it.toi, normal: it.normal }, key))
      }
    }
    None
  } else {
    None
  }
}

///|
pub fn QueryPipeline3DReal::intersect_aabb_conservative(
  self : QueryPipeline3DReal,
  bodies : @dynamics.RigidBodySet3D,
  colliders : ColliderSet3D,
  aabb : @core.Aabb3,
) -> Array[ColliderHandle3D] {
  let out : Array[ColliderHandle3D] = []
  for h in colliders.all_handles() {
    if colliders.get(h) is Some(co) {
      if !filter_pass_3d_real(self.filter, bodies, h, co) {
        continue
      }
      // Conservative query: uses current collider AABB.
      if co.compute_aabb().intersects(aabb) {
        out.push(h)
      }
    }
  }
  out
}

///|
priv struct Qp3dRealGjkSupportPoint {
  p : @core.Vec3
  sup_a : @core.Vec3
}

///|
fn qp3d_real_support_world_for_convex(
  pos : @core.Isometry3,
  points : Array[@core.Vec3],
  border_radius : @core.Real,
  dir : @core.Vec3,
) -> @core.Vec3 {
  if points.length() == 0 {
    return pos.translation
  }
  let local_dir = pos.rotation.inverse().rotate_vec3(dir)
  let mut best_local = points[0]
  let mut best_d = best_local.dot(local_dir)
  for i in 1..<points.length() {
    let p = points[i]
    let d = p.dot(local_dir)
    if d > best_d {
      best_d = d
      best_local = p
    }
  }
  let mut best = pos.transform_point(best_local)
  if border_radius > 0.0F {
    let d2 = dir.length_squared()
    let offset = if d2 <= 1.0e-12F {
      @core.Vec3::new(border_radius, 0.0F, 0.0F)
    } else {
      dir.scale(border_radius / Float::sqrt(d2))
    }
    best = best.add(offset)
  }
  best
}

///|
fn qp3d_real_gjk_reduce_simplex_closest_to_origin(
  simplex : Array[Qp3dRealGjkSupportPoint],
) -> (@core.Vec3, Array[@core.Real], Bool) {
  // Returns: closest point on simplex to the origin, barycentric weights for each simplex vertex
  // (in the same order), and whether the origin is inside the simplex.
  if simplex.length() == 0 {
    return (@core.Vec3::zero(), [], false)
  }
  if simplex.length() == 1 {
    return (simplex[0].p, [1.0F], false)
  }
  if simplex.length() == 2 {
    let a = simplex[1].p
    let b = simplex[0].p
    let ab = b.sub(a)
    let denom = ab.length_squared()
    if denom <= 1.0e-18F {
      simplex[0] = simplex[1]
      simplex.pop() |> ignore
      return (a, [1.0F], false)
    }
    let t = @core.clamp(-a.dot(ab) / denom, 0.0F, 1.0F)
    let closest = a.add(ab.scale(t))
    if t <= 1.0e-6F {
      simplex[0] = simplex[1]
      simplex.pop() |> ignore
      return (a, [1.0F], false)
    }
    if t >= 1.0F - 1.0e-6F {
      simplex.pop() |> ignore
      return (b, [1.0F], false)
    }
    // simplex order is [b, a] (index 0, 1) with weights [t, 1-t].
    (closest, [t, 1.0F - t], false)
  } else if simplex.length() == 3 {
    // Triangle region test (Ericson).
    let a_v = simplex[2].p
    let b_v = simplex[1].p
    let c_v = simplex[0].p
    let ab = b_v.sub(a_v)
    let ac = c_v.sub(a_v)
    let ao = a_v.scale(-1.0F)
    let d1 = ab.dot(ao)
    let d2 = ac.dot(ao)
    if d1 <= 0.0F && d2 <= 0.0F {
      // Closest to A.
      simplex[0] = simplex[2]
      simplex.pop() |> ignore
      simplex.pop() |> ignore
      return (a_v, [1.0F], false)
    }
    let bo = b_v.scale(-1.0F)
    let d3 = ab.dot(bo)
    let d4 = ac.dot(bo)
    if d3 >= 0.0F && d4 <= d3 {
      // Closest to B.
      simplex[0] = simplex[1]
      simplex.pop() |> ignore
      return (b_v, [1.0F], false)
    }
    let vc = d1 * d4 - d3 * d2
    if vc <= 0.0F && d1 >= 0.0F && d3 <= 0.0F {
      // Closest on AB.
      let v = d1 / (d1 - d3)
      let closest = a_v.add(ab.scale(v))
      simplex[0] = simplex[1]
      simplex[1] = simplex[2]
      simplex.pop() |> ignore
      return (closest, [v, 1.0F - v], false)
    }
    let co = c_v.scale(-1.0F)
    let d5 = ab.dot(co)
    let d6 = ac.dot(co)
    if d6 >= 0.0F && d5 <= d6 {
      // Closest to C.
      simplex[1] = simplex[0]
      simplex[0] = simplex[2]
      simplex.pop() |> ignore
      simplex.pop() |> ignore
      return (c_v, [1.0F], false)
    }
    let vb = d5 * d2 - d1 * d6
    if vb <= 0.0F && d2 >= 0.0F && d6 <= 0.0F {
      // Closest on AC.
      let w = d2 / (d2 - d6)
      let closest = a_v.add(ac.scale(w))
      simplex[1] = simplex[2]
      simplex.pop() |> ignore
      return (closest, [w, 1.0F - w], false)
    }
    let va = d3 * d6 - d5 * d4
    if va <= 0.0F && d4 - d3 >= 0.0F && d5 - d6 >= 0.0F {
      // Closest on BC.
      let bc = c_v.sub(b_v)
      let w = (d4 - d3) / (d4 - d3 + (d5 - d6))
      let closest = b_v.add(bc.scale(w))
      simplex[2] = simplex[1]
      simplex[1] = simplex[0]
      simplex.pop() |> ignore
      return (closest, [1.0F - w, w], false)
    }
    // Closest inside face.
    let denom = 1.0F / (va + vb + vc)
    let v = vb * denom
    let w = vc * denom
    let u = 1.0F - v - w
    let closest = a_v.scale(u).add(b_v.scale(v)).add(c_v.scale(w))
    // Keep full triangle with order [c, b, a] (0,1,2) => weights [w, v, u]
    (closest, [w, v, u], false)
  } else {
    // Tetrahedron: check if the origin is outside any face; otherwise it's inside.
    // simplex order is [d, c, b, a] (0..3), with a the newest point.
    let faces = [
      (3, 2, 1, 0), // (a,b,c) opposite d
      (3, 1, 0, 2), // (a,c,d) opposite b
      (3, 0, 2, 1), // (a,d,b) opposite c
      (2, 0, 1, 3),
    ] // (b,d,c) opposite a
    let mut best_d2 = 1.0e30F
    let mut best_simplex : Array[Qp3dRealGjkSupportPoint] = []
    let mut best_weights : Array[@core.Real] = []
    let mut best_closest = @core.Vec3::zero()
    for fi in 0..<faces.length() {
      let (ia, ib, ic, iopp) = faces[fi]
      let a = simplex[ia].p
      let b = simplex[ib].p
      let c = simplex[ic].p
      let opp = simplex[iopp].p
      let ab = b.sub(a)
      let ac = c.sub(a)
      let mut n = ab.cross(ac)
      let n2 = n.length_squared()
      if n2 <= 1.0e-18F {
        continue
      }
      // Ensure n points outward (away from opposite vertex).
      if n.dot(opp.sub(a)) > 0.0F {
        n = n.scale(-1.0F)
      }
      if n.dot(a.scale(-1.0F)) > 0.0F {
        // Origin outside this face: compute closest point on the triangle.
        let tri_simplex : Array[Qp3dRealGjkSupportPoint] = [
          simplex[ic],
          simplex[ib],
          simplex[ia],
        ]
        let (closest, w, _) = qp3d_real_gjk_reduce_simplex_closest_to_origin(
          tri_simplex,
        )
        let d2 = closest.length_squared()
        if d2 < best_d2 {
          best_d2 = d2
          best_simplex = tri_simplex
          best_weights = w
          best_closest = closest
        }
      }
    }
    if best_simplex.length() > 0 {
      simplex.clear()
      for i in 0..<best_simplex.length() {
        simplex.push(best_simplex[i])
      }
      (best_closest, best_weights, false)
    } else {
      // Origin inside tetrahedron.
      (@core.Vec3::zero(), [0.0F], true)
    }
  }
}

///|
fn qp3d_real_project_point_on_convex(
  pos : @core.Isometry3,
  points : Array[@core.Vec3],
  border_radius : @core.Real,
  point : @core.Vec3,
  solid : Bool,
) -> (PointProjection3, @core.Real) {
  if points.length() == 0 {
    let d2 = pos.translation.sub(point).length_squared()
    return ({ point: pos.translation, is_inside: false }, d2)
  }
  let eps2 = 1.0e-10F
  let mut dir = pos.translation.sub(point)
  if dir.length_squared() <= 1.0e-12F {
    dir = @core.Vec3::new(1.0F, 0.0F, 0.0F)
  }
  let simplex : Array[Qp3dRealGjkSupportPoint] = []
  let sup_a0 = qp3d_real_support_world_for_convex(
    pos, points, border_radius, dir,
  )
  simplex.push({ p: sup_a0.sub(point), sup_a: sup_a0 })
  let mut closest = simplex[0].p
  dir = closest.scale(-1.0F)
  let mut weights : Array[@core.Real] = [1.0F]
  for _ in 0..<32 {
    let d2 = dir.length_squared()
    if d2 <= eps2 {
      if solid {
        return ({ point, is_inside: true }, 0.0F)
      }
      let proj = simplex[0].sup_a
      return ({ point: proj, is_inside: true }, 0.0F)
    }
    let sup_a = qp3d_real_support_world_for_convex(
      pos, points, border_radius, dir,
    )
    let p = sup_a.sub(point)
    if p.dot(dir) >= closest.dot(dir) - 1.0e-7F {
      break
    }
    simplex.push({ p, sup_a })
    let (new_closest, w, inside) = qp3d_real_gjk_reduce_simplex_closest_to_origin(
      simplex,
    )
    closest = new_closest
    weights = w
    if inside {
      if solid {
        return ({ point, is_inside: true }, 0.0F)
      }
      let proj = simplex[0].sup_a
      return ({ point: proj, is_inside: true }, 0.0F)
    }
    dir = closest.scale(-1.0F)
  }
  let mut proj = simplex[0].sup_a.scale(weights[0])
  for i in 1..<simplex.length() {
    proj = proj.add(simplex[i].sup_a.scale(weights[i]))
  }
  let d2 = proj.sub(point).length_squared()
  ({ point: proj, is_inside: false }, d2)
}

///|
fn qp3d_real_project_point_on_shape(
  pos : @core.Isometry3,
  shape : Shape3D,
  point : @core.Vec3,
  solid : Bool,
) -> (PointProjection3, @core.Real) {
  match shape {
    Shape3D::Ball(r) => {
      let center = pos.translation
      let delta = point.sub(center)
      let dist2 = delta.length_squared()
      let inside = dist2 <= r * r
      if inside && solid {
        ({ point, is_inside: true }, 0.0F)
      } else {
        let dir = if dist2 <= 1.0e-12F {
          @core.Vec3::new(1.0F, 0.0F, 0.0F)
        } else {
          delta.scale(1.0F / Float::sqrt(dist2))
        }
        let proj = center.add(dir.scale(r))
        ({ point: proj, is_inside: inside }, proj.sub(point).length_squared())
      }
    }
    Shape3D::Cuboid(he) => {
      let inv = pos.inverse()
      let lp = inv.transform_point(point)
      let inside = @core.abs(lp.x) <= he.x &&
        @core.abs(lp.y) <= he.y &&
        @core.abs(lp.z) <= he.z
      if inside && solid {
        ({ point, is_inside: true }, 0.0F)
      } else {
        let mut p = @core.Vec3::new(
          @core.clamp(lp.x, -he.x, he.x),
          @core.clamp(lp.y, -he.y, he.y),
          @core.clamp(lp.z, -he.z, he.z),
        )
        if inside {
          // Project to the closest face.
          let dx = he.x - @core.abs(lp.x)
          let dy = he.y - @core.abs(lp.y)
          let dz = he.z - @core.abs(lp.z)
          if dx <= dy && dx <= dz {
            p = @core.Vec3::new(
              if lp.x >= 0.0F {
                he.x
              } else {
                -he.x
              },
              p.y,
              p.z,
            )
          } else if dy <= dz {
            p = @core.Vec3::new(
              p.x,
              if lp.y >= 0.0F {
                he.y
              } else {
                -he.y
              },
              p.z,
            )
          } else {
            p = @core.Vec3::new(
              p.x,
              p.y,
              if lp.z >= 0.0F {
                he.z
              } else {
                -he.z
              },
            )
          }
        }
        let proj = pos.transform_point(p)
        ({ point: proj, is_inside: inside }, proj.sub(point).length_squared())
      }
    }
    Shape3D::CapsuleY(r, hh) => {
      let inv = pos.inverse()
      let p = inv.transform_point(point)
      let axis_pt = @core.Vec3::new(0.0F, @core.clamp(p.y, -hh, hh), 0.0F)
      let delta = p.sub(axis_pt)
      let dist2 = delta.length_squared()
      let inside = dist2 <= r * r
      if inside && solid {
        ({ point, is_inside: true }, 0.0F)
      } else {
        let dir = if dist2 <= 1.0e-12F {
          @core.Vec3::new(1.0F, 0.0F, 0.0F)
        } else {
          delta.scale(1.0F / Float::sqrt(dist2))
        }
        let proj_local = axis_pt.add(dir.scale(r))
        let proj = pos.transform_point(proj_local)
        ({ point: proj, is_inside: inside }, proj.sub(point).length_squared())
      }
    }
    Shape3D::Cylinder(r, hh) => {
      let inv = pos.inverse()
      let p = inv.transform_point(point)
      let radial = @core.Vec3::new(p.x, 0.0F, p.z)
      let rl2 = radial.length_squared()
      let rl = if rl2 <= 1.0e-12F { 0.0F } else { Float::sqrt(rl2) }
      let inside = @core.abs(p.y) <= hh && rl <= r
      if inside && solid {
        ({ point, is_inside: true }, 0.0F)
      } else {
        let mut y = @core.clamp(p.y, -hh, hh)
        let mut x = p.x
        let mut z = p.z
        if rl > r {
          let inv_rl = 1.0F / rl
          x = x * inv_rl * r
          z = z * inv_rl * r
        }
        if inside {
          // Snap to nearest of side/caps.
          let dist_side = r - rl
          let dist_cap = hh - @core.abs(p.y)
          if dist_cap < dist_side {
            y = if p.y >= 0.0F { hh } else { -hh }
          } else if rl <= 1.0e-12F {
            x = r
            z = 0.0F
          }
        }
        let proj = pos.transform_point(@core.Vec3::new(x, y, z))
        ({ point: proj, is_inside: inside }, proj.sub(point).length_squared())
      }
    }
    Shape3D::RoundCylinder(r, hh, br) =>
      qp3d_real_project_point_on_shape(
        pos,
        Shape3D::CapsuleY(r + br, hh + br),
        point,
        solid,
      )
    Shape3D::Cone(r, hh) => {
      // Conservative: project on the local AABB of the cone.
      let inv = pos.inverse()
      let p = inv.transform_point(point)
      let he = @core.Vec3::new(r, hh, r)
      let inside = @core.abs(p.x) <= he.x &&
        @core.abs(p.y) <= he.y &&
        @core.abs(p.z) <= he.z
      if inside && solid {
        ({ point, is_inside: true }, 0.0F)
      } else {
        let q = @core.Vec3::new(
          @core.clamp(p.x, -he.x, he.x),
          @core.clamp(p.y, -he.y, he.y),
          @core.clamp(p.z, -he.z, he.z),
        )
        let proj = pos.transform_point(q)
        ({ point: proj, is_inside: inside }, proj.sub(point).length_squared())
      }
    }
    Shape3D::HalfSpace(n_local) => {
      let n = qp3d_real_normalized_or_default(pos.rotation.rotate_vec3(n_local))
      let d = n.dot(point.sub(pos.translation))
      let inside = d <= 0.0F
      if inside && solid {
        ({ point, is_inside: true }, 0.0F)
      } else {
        let proj = point.sub(n.scale(d))
        ({ point: proj, is_inside: inside }, d * d)
      }
    }
    Shape3D::Triangle(a, b, c) => {
      let vtx = [a, b, c]
      let idx = [(0, 1, 2)]
      if trimesh_closest_point_query(point, pos, vtx, idx) is Some((q, _, d2)) {
        ({ point: q, is_inside: false }, d2)
      } else {
        ({ point, is_inside: false }, 1.0e30F)
      }
    }
    Shape3D::ConvexHull(points, br) =>
      qp3d_real_project_point_on_convex(pos, points, br, point, solid)
    Shape3D::Voxels(v) => {
      let sx = v.voxel_size.x
      let sy = v.voxel_size.y
      let sz = v.voxel_size.z
      if sx <= 0.0F || sy <= 0.0F || sz <= 0.0F {
        (
          { point: pos.translation, is_inside: false },
          pos.translation.sub(point).length_squared(),
        )
      } else {
        let inv = pos.inverse()
        let lp = inv.transform_point(point)
        fn floor_div_to_int(value : @core.Real, denom : @core.Real) -> Int {
          if denom == 0.0F {
            0
          } else {
            @math.floor((value / denom).to_double()).to_int()
          }
        }

        let ix = floor_div_to_int(lp.x, sx)
        let iy = floor_div_to_int(lp.y, sy)
        let iz = floor_div_to_int(lp.z, sz)
        let mut best_q = lp
        let mut best_d2 = 1.0e30F
        let mut inside = false
        let y_lo = iy - 1
        let y_hi = iy + 1
        for ddx in 0..<3 {
          let x = ix + ddx - 1
          for ddz in 0..<3 {
            let z = iz + ddz - 1
            if v.column_ys(x, z) is Some(ys) {
              for i in 0..<ys.length() {
                let y = ys[i]
                if y < y_lo || y > y_hi {
                  continue
                }
                let x0 = Float::from_int(x) * sx
                let y0 = Float::from_int(y) * sy
                let z0 = Float::from_int(z) * sz
                let x1 = x0 + sx
                let y1 = y0 + sy
                let z1 = z0 + sz
                let q = @core.Vec3::new(
                  @core.clamp(lp.x, x0, x1),
                  @core.clamp(lp.y, y0, y1),
                  @core.clamp(lp.z, z0, z1),
                )
                let d2 = q.sub(lp).length_squared()
                if d2 <= 1.0e-12F {
                  inside = true
                }
                if d2 < best_d2 {
                  best_d2 = d2
                  best_q = q
                }
              }
            }
          }
        }
        if inside && solid {
          ({ point, is_inside: true }, 0.0F)
        } else {
          let proj = pos.transform_point(best_q)
          ({ point: proj, is_inside: inside }, proj.sub(point).length_squared())
        }
      }
    }
    Shape3D::Compound(parts) => {
      let mut best_proj : PointProjection3 = {
        point: pos.translation,
        is_inside: false,
      }
      let mut best_d2 = 1.0e30F
      let mut any_inside = false
      for i in 0..<parts.length() {
        let (iso, sh) = parts[i]
        let (proj, d2) = qp3d_real_project_point_on_shape(
          pos.mul(iso),
          sh,
          point,
          solid,
        )
        if proj.is_inside {
          any_inside = true
        }
        if d2 < best_d2 {
          best_d2 = d2
          best_proj = proj
        }
      }
      if any_inside && solid {
        ({ point, is_inside: true }, 0.0F)
      } else {
        ({ point: best_proj.point, is_inside: any_inside }, best_d2)
      }
    }
    Shape3D::Heightfield(vertices, _, rows, cols) =>
      if heightfield_closest_point_query(point, pos, vertices, rows, cols)
        is Some((q, _, d2)) {
        ({ point: q, is_inside: false }, d2)
      } else {
        ({ point, is_inside: false }, 1.0e30F)
      }
    Shape3D::TriMesh(vertices, indices) =>
      if trimesh_closest_point_query(point, pos, vertices, indices)
        is Some((q, _, d2)) {
        ({ point: q, is_inside: false }, d2)
      } else {
        ({ point, is_inside: false }, 1.0e30F)
      }
  }
}

///|
pub fn QueryPipeline3DReal::project_point(
  self : QueryPipeline3DReal,
  bodies : @dynamics.RigidBodySet3D,
  colliders : ColliderSet3D,
  point : @core.Vec3,
  max_dist : @core.Real,
  solid : Bool,
) -> (ColliderHandle3D, PointProjection3)? {
  let max_d = if max_dist < 0.0F { 0.0F } else { max_dist }
  let max_d2 = max_d * max_d
  let mut best : (ColliderHandle3D, PointProjection3)? = None
  let mut best_d2 = max_d2 + 1.0F
  for h in colliders.all_handles() {
    if colliders.get(h) is Some(co) {
      if !filter_pass_3d_real(self.filter, bodies, h, co) {
        continue
      }
      let (proj, d2) = qp3d_real_project_point_on_shape(
        co.position(),
        co.shape(),
        point,
        solid,
      )
      if d2 <= max_d2 && d2 < best_d2 {
        best_d2 = d2
        best = Some((h, proj))
      }
    }
  }
  best
}

///|
fn qp3d_real_contains_point(
  pos : @core.Isometry3,
  shape : Shape3D,
  point : @core.Vec3,
) -> Bool {
  match shape {
    Shape3D::Ball(r) => point.sub(pos.translation).length_squared() <= r * r
    Shape3D::Cuboid(he) => {
      let p = pos.inverse().transform_point(point)
      @core.abs(p.x) <= he.x && @core.abs(p.y) <= he.y && @core.abs(p.z) <= he.z
    }
    Shape3D::CapsuleY(r, hh) => {
      let p = pos.inverse().transform_point(point)
      let axis_pt = @core.Vec3::new(0.0F, @core.clamp(p.y, -hh, hh), 0.0F)
      p.sub(axis_pt).length_squared() <= r * r
    }
    Shape3D::Cylinder(r, hh) => {
      let p = pos.inverse().transform_point(point)
      let rl2 = p.x * p.x + p.z * p.z
      @core.abs(p.y) <= hh && rl2 <= r * r
    }
    Shape3D::RoundCylinder(r, hh, br) =>
      qp3d_real_contains_point(pos, Shape3D::CapsuleY(r + br, hh + br), point)
    Shape3D::Cone(r, hh) => {
      let p = pos.inverse().transform_point(point)
      // Conservative: use AABB containment.
      @core.abs(p.x) <= r && @core.abs(p.y) <= hh && @core.abs(p.z) <= r
    }
    Shape3D::HalfSpace(n_local) => {
      let n = qp3d_real_normalized_or_default(pos.rotation.rotate_vec3(n_local))
      n.dot(point.sub(pos.translation)) <= 0.0F
    }
    Shape3D::Compound(parts) => {
      for i in 0..<parts.length() {
        let (iso, sh) = parts[i]
        if qp3d_real_contains_point(pos.mul(iso), sh, point) {
          return true
        }
      }
      false
    }
    _ => false
  }
}

///|
pub fn QueryPipeline3DReal::intersect_point(
  self : QueryPipeline3DReal,
  bodies : @dynamics.RigidBodySet3D,
  colliders : ColliderSet3D,
  point : @core.Vec3,
) -> Array[ColliderHandle3D] {
  let out : Array[ColliderHandle3D] = []
  for h in colliders.all_handles() {
    if colliders.get(h) is Some(co) {
      if !filter_pass_3d_real(self.filter, bodies, h, co) {
        continue
      }
      if qp3d_real_contains_point(co.position(), co.shape(), point) {
        out.push(h)
      }
    }
  }
  out
}

///|
fn qp3d_real_cast_ball_against_shape(
  center : @core.Vec3,
  vel : @core.Vec3,
  effective_radius : @core.Real,
  pos2 : @core.Isometry3,
  shape2 : Shape3D,
  max_toi : @core.Real,
) -> ShapeCastHit3? {
  let ray = Ray3::new(center, vel)
  match shape2 {
    Shape3D::Ball(r2) =>
      if ray_sphere_toi_normal(
          ray,
          pos2.translation,
          r2 + effective_radius,
          max_toi,
        )
        is Some(hit) {
        let point = pos2.translation.add(hit.normal.scale(r2))
        Some({ toi: hit.toi, point, normal: hit.normal })
      } else {
        None
      }
    Shape3D::Cuboid(he) =>
      if ray_obb_toi_normal(
          ray,
          pos2,
          he.add(
            @core.Vec3::new(
              effective_radius, effective_radius, effective_radius,
            ),
          ),
          max_toi,
        )
        is Some(hit) {
        let center_at_hit = center.add(vel.scale(hit.toi))
        let point = center_at_hit.sub(hit.normal.scale(effective_radius))
        Some({ toi: hit.toi, point, normal: hit.normal })
      } else {
        None
      }
    Shape3D::CapsuleY(r2, hh2) =>
      if ray_capsule_y_toi_normal(
          ray,
          pos2,
          r2 + effective_radius,
          hh2,
          max_toi,
        )
        is Some(hit) {
        let center_at_hit = center.add(vel.scale(hit.toi))
        let point = center_at_hit.sub(hit.normal.scale(effective_radius))
        Some({ toi: hit.toi, point, normal: hit.normal })
      } else {
        None
      }
    Shape3D::Cylinder(r2, hh2) =>
      if ray_cylinder_y_toi_normal(
          ray,
          pos2,
          r2 + effective_radius,
          hh2 + effective_radius,
          max_toi,
        )
        is Some(hit) {
        let center_at_hit = center.add(vel.scale(hit.toi))
        let point = center_at_hit.sub(hit.normal.scale(effective_radius))
        Some({ toi: hit.toi, point, normal: hit.normal })
      } else {
        None
      }
    Shape3D::RoundCylinder(r2, hh2, br2) =>
      qp3d_real_cast_ball_against_shape(
        center,
        vel,
        effective_radius,
        pos2,
        Shape3D::Cylinder(r2 + br2, hh2 + br2),
        max_toi,
      )
    Shape3D::Cone(r2, hh2) =>
      if ray_cone_y_toi_normal(
          ray,
          pos2,
          r2 + effective_radius,
          hh2 + effective_radius,
          max_toi,
        )
        is Some(hit) {
        let center_at_hit = center.add(vel.scale(hit.toi))
        let point = center_at_hit.sub(hit.normal.scale(effective_radius))
        Some({ toi: hit.toi, point, normal: hit.normal })
      } else {
        None
      }
    Shape3D::HalfSpace(n_local) => {
      let n = qp3d_real_normalized_or_default(
        pos2.rotation.rotate_vec3(n_local),
      )
      let d0 = n.dot(center.sub(pos2.translation))
      let denom = n.dot(vel)
      if d0 <= effective_radius {
        Some({
          toi: 0.0F,
          point: center.sub(n.scale(effective_radius)),
          normal: n,
        })
      } else if denom >= -1.0e-12F {
        None
      } else {
        let t = (effective_radius - d0) / denom
        if t < 0.0F || t > max_toi {
          None
        } else {
          let c = center.add(vel.scale(t))
          Some({ toi: t, point: c.sub(n.scale(effective_radius)), normal: n })
        }
      }
    }
    Shape3D::Triangle(a, b, c) => {
      let vtx = [a, b, c]
      let idx = [(0, 1, 2)]
      qp3d_real_cast_ball_against_shape(
        center,
        vel,
        effective_radius,
        pos2,
        Shape3D::TriMesh(vtx, idx),
        max_toi,
      )
    }
    Shape3D::Compound(parts) => {
      let mut best : ShapeCastHit3? = None
      let mut best_t = max_toi + 1.0F
      for i in 0..<parts.length() {
        let (iso, sh) = parts[i]
        let child_pos = pos2.mul(iso)
        if qp3d_real_cast_ball_against_shape(
            center, vel, effective_radius, child_pos, sh, max_toi,
          )
          is Some(hit) {
          if hit.toi < best_t {
            best_t = hit.toi
            best = Some(hit)
          }
        }
      }
      best
    }
    Shape3D::Heightfield(vertices, _, rows, cols) => {
      // Conservative advancement using heightfield distance queries.
      let mut t = 0.0F
      let mut prev_d = 1.0e30F
      for _ in 0..<32 {
        if t > max_toi {
          return None
        }
        let c = center.add(vel.scale(t))
        if heightfield_closest_point_query(c, pos2, vertices, rows, cols)
          is Some((q, n0, d2)) {
          let dist = Float::sqrt(d2)
          let delta = c.sub(q)
          let n = if d2 <= 1.0e-12F {
            qp3d_real_normalized_or_default(n0)
          } else {
            delta.scale(1.0F / dist)
          }
          if dist <= effective_radius {
            return Some({ toi: t, point: q, normal: n })
          }
          let approach = -n.dot(vel)
          if approach <= 1.0e-12F {
            return None
          }
          let d = dist - effective_radius
          if @core.abs(d - prev_d) <= 1.0e-6F {
            return None
          }
          prev_d = d
          t = t + d / approach
        } else {
          return None
        }
      }
      None
    }
    Shape3D::TriMesh(vertices, indices) => {
      // Conservative advancement using mesh distance queries.
      let mut t = 0.0F
      let mut prev_d = 1.0e30F
      for _ in 0..<32 {
        if t > max_toi {
          return None
        }
        let c = center.add(vel.scale(t))
        if trimesh_closest_point_query(c, pos2, vertices, indices)
          is Some((q, n0, d2)) {
          let dist = Float::sqrt(d2)
          let delta = c.sub(q)
          let n = if d2 <= 1.0e-12F {
            qp3d_real_normalized_or_default(n0)
          } else {
            delta.scale(1.0F / dist)
          }
          if dist <= effective_radius {
            return Some({ toi: t, point: q, normal: n })
          }
          let approach = -n.dot(vel)
          if approach <= 1.0e-12F {
            return None
          }
          let d = dist - effective_radius
          if @core.abs(d - prev_d) <= 1.0e-6F {
            // Stalled.
            return None
          }
          prev_d = d
          t = t + d / approach
        } else {
          return None
        }
      }
      None
    }
    Shape3D::Voxels(v) => {
      let sx = v.voxel_size.x
      let sy = v.voxel_size.y
      let sz = v.voxel_size.z
      if sx <= 0.0F || sy <= 0.0F || sz <= 0.0F {
        return None
      }
      let inv = pos2.inverse()
      fn overlap_at_time(
        center : @core.Vec3,
        effective_radius : @core.Real,
        pos2 : @core.Isometry3,
        inv : @core.Isometry3,
        v : Voxels3DReal,
      ) -> ShapeCastHit3? {
        let lp = inv.transform_point(center)
        fn floor_div_to_int(value : @core.Real, denom : @core.Real) -> Int {
          if denom == 0.0F {
            0
          } else {
            @math.floor((value / denom).to_double()).to_int()
          }
        }

        let ix0 = floor_div_to_int(lp.x - effective_radius, v.voxel_size.x)
        let ix1 = floor_div_to_int(lp.x + effective_radius, v.voxel_size.x)
        let iy0 = floor_div_to_int(lp.y - effective_radius, v.voxel_size.y)
        let iy1 = floor_div_to_int(lp.y + effective_radius, v.voxel_size.y)
        let iz0 = floor_div_to_int(lp.z - effective_radius, v.voxel_size.z)
        let iz1 = floor_div_to_int(lp.z + effective_radius, v.voxel_size.z)
        let mut best : ShapeCastHit3? = None
        let mut best_d2 = 1.0e30F
        // Scan a small neighborhood around the center in voxel coordinates.
        for x in ix0..<(ix1 + 1) {
          for z in iz0..<(iz1 + 1) {
            if v.column_ys(x, z) is Some(ys) {
              for i in 0..<ys.length() {
                let y = ys[i]
                if y < iy0 || y > iy1 {
                  continue
                }
                let x0 = Float::from_int(x) * v.voxel_size.x
                let y0 = Float::from_int(y) * v.voxel_size.y
                let z0 = Float::from_int(z) * v.voxel_size.z
                let x1 = x0 + v.voxel_size.x
                let y1 = y0 + v.voxel_size.y
                let z1 = z0 + v.voxel_size.z
                let q_local = @core.Vec3::new(
                  @core.clamp(lp.x, x0, x1),
                  @core.clamp(lp.y, y0, y1),
                  @core.clamp(lp.z, z0, z1),
                )
                let d2 = q_local.sub(lp).length_squared()
                if d2 < best_d2 {
                  best_d2 = d2
                  let q_world = pos2.transform_point(q_local)
                  let dist = Float::sqrt(d2)
                  let n_local = if d2 <= 1.0e-12F {
                    @core.Vec3::new(0.0F, 1.0F, 0.0F)
                  } else {
                    lp.sub(q_local).scale(1.0F / dist)
                  }
                  let n_world = qp3d_real_normalized_or_default(
                    pos2.rotation.rotate_vec3(n_local),
                  )
                  best = Some({ toi: 0.0F, point: q_world, normal: n_world })
                }
              }
            }
          }
        }
        if best is Some(hit) {
          if Float::sqrt(best_d2) <= effective_radius {
            Some(hit)
          } else {
            None
          }
        } else {
          None
        }
      }

      let mut t = 0.0F
      let mut t_prev = 0.0F
      let mxy = if sx < sy { sx } else { sy }
      let step_len = (if mxy < sz { mxy } else { sz }) * 0.5F
      let dt = if step_len <= 1.0e-6F { 0.05F } else { step_len }
      let steps = (max_toi / dt).to_int() + 2
      let max_steps = if steps > 1024 { 1024 } else { steps }
      for si in 0..<max_steps {
        si |> ignore
        if t > max_toi {
          break
        }
        let c = center.add(vel.scale(t))
        if overlap_at_time(c, effective_radius, pos2, inv, v) is Some(_) {
          // Bisection refine.
          let mut lo = t_prev
          let mut hi = t
          for _ in 0..<20 {
            let mid = (lo + hi) * 0.5F
            let cm = center.add(vel.scale(mid))
            if overlap_at_time(cm, effective_radius, pos2, inv, v) is Some(_) {
              hi = mid
            } else {
              lo = mid
            }
          }
          let toi = hi
          let ch = center.add(vel.scale(toi))
          if overlap_at_time(ch, effective_radius, pos2, inv, v) is Some(hit0) {
            return Some({ toi, point: hit0.point, normal: hit0.normal })
          } else {
            return None
          }
        }
        t_prev = t
        t = t + dt
      }
      None
    }
    _ => None
  }
}

///|
fn qp3d_real_shape_bounding_radius(shape : Shape3D) -> @core.Real {
  let aabb = shape.local_aabb()
  let he = aabb.maxs.sub(aabb.mins).scale(0.5F)
  Float::sqrt(he.length_squared())
}

///|
fn qp3d_real_inflate_shape(shape : Shape3D, margin : @core.Real) -> Shape3D {
  if margin <= 0.0F {
    return shape
  }
  match shape {
    Shape3D::Ball(r) => Shape3D::Ball(r + margin)
    Shape3D::Cuboid(he) =>
      Shape3D::Cuboid(he.add(@core.Vec3::new(margin, margin, margin)))
    Shape3D::CapsuleY(r, hh) => Shape3D::CapsuleY(r + margin, hh)
    Shape3D::Cylinder(r, hh) => Shape3D::Cylinder(r + margin, hh + margin)
    Shape3D::RoundCylinder(r, hh, br) =>
      Shape3D::RoundCylinder(r, hh, br + margin)
    Shape3D::Cone(r, hh) => Shape3D::Cone(r + margin, hh + margin)
    Shape3D::HalfSpace(n) => Shape3D::HalfSpace(n)
    Shape3D::Triangle(a, b, c) => Shape3D::Triangle(a, b, c)
    Shape3D::ConvexHull(points, br) => Shape3D::ConvexHull(points, br + margin)
    Shape3D::Compound(parts) => {
      let out : Array[(@core.Isometry3, Shape3D)] = []
      for i in 0..<parts.length() {
        let (iso, sh) = parts[i]
        out.push((iso, qp3d_real_inflate_shape(sh, margin)))
      }
      Shape3D::Compound(out)
    }
    Shape3D::Voxels(v) => Shape3D::Voxels(v)
    Shape3D::Heightfield(vertices, indices, rows, cols) =>
      Shape3D::Heightfield(vertices, indices, rows, cols)
    Shape3D::TriMesh(vertices, indices) => Shape3D::TriMesh(vertices, indices)
  }
}

///|
fn qp3d_real_flip_contact(cp : ContactPoint3D) -> ContactPoint3D {
  {
    point1: cp.point2,
    point2: cp.point1,
    normal: cp.normal.scale(-1.0F),
    penetration: cp.penetration,
  }
}

///|
fn qp3d_real_best_contact_update(
  best : ContactPoint3D?,
  cp : ContactPoint3D,
) -> ContactPoint3D? {
  if best is Some(b) {
    if cp.penetration > b.penetration {
      Some(cp)
    } else {
      best
    }
  } else {
    Some(cp)
  }
}

///|
fn qp3d_real_best_contact_for_pair(
  p1 : @core.Isometry3,
  s1 : Shape3D,
  p2 : @core.Isometry3,
  s2 : Shape3D,
) -> ContactPoint3D? {
  match (s1, s2) {
    (Shape3D::Compound(parts), other) => {
      let mut best : ContactPoint3D? = None
      for i in 0..<parts.length() {
        let (iso, sh) = parts[i]
        if qp3d_real_best_contact_for_pair(p1.mul(iso), sh, p2, other)
          is Some(cp) {
          best = qp3d_real_best_contact_update(best, cp)
        }
      }
      best
    }
    (other, Shape3D::Compound(parts)) => {
      let mut best : ContactPoint3D? = None
      for i in 0..<parts.length() {
        let (iso, sh) = parts[i]
        if qp3d_real_best_contact_for_pair(p1, other, p2.mul(iso), sh)
          is Some(cp) {
          best = qp3d_real_best_contact_update(best, cp)
        }
      }
      best
    }
    _ => {
      let mut best : ContactPoint3D? = None
      match (s1, s2) {
        (Shape3D::HalfSpace(n), other) =>
          if compute_halfspace_contact(p1, n, p2, other) is Some(cp) {
            best = qp3d_real_best_contact_update(best, cp)
          }
        (other, Shape3D::HalfSpace(n)) =>
          if compute_halfspace_contact(p2, n, p1, other) is Some(cp) {
            best = qp3d_real_best_contact_update(
              best,
              qp3d_real_flip_contact(cp),
            )
          }
        (Shape3D::Ball(r1), Shape3D::Ball(r2)) =>
          if compute_ball_ball_contact(p1.translation, r1, p2.translation, r2)
            is Some(cp) {
            best = qp3d_real_best_contact_update(best, cp)
          }
        (Shape3D::Ball(r), Shape3D::Cuboid(he)) =>
          if compute_ball_cuboid_contact(p1.translation, r, p2, he) is Some(cp) {
            best = qp3d_real_best_contact_update(best, cp)
          }
        (Shape3D::Cuboid(he), Shape3D::Ball(r)) =>
          if compute_ball_cuboid_contact(p2.translation, r, p1, he) is Some(cp) {
            best = qp3d_real_best_contact_update(
              best,
              qp3d_real_flip_contact(cp),
            )
          }
        (Shape3D::Cuboid(he1), Shape3D::Cuboid(he2)) =>
          if compute_cuboid_cuboid_contact(p1, he1, p2, he2) is Some(cp) {
            best = qp3d_real_best_contact_update(best, cp)
          }
        (Shape3D::Ball(r), Shape3D::CapsuleY(cr, ch)) =>
          if compute_ball_capsule_contact(p1.translation, r, p2, cr, ch)
            is Some(cp) {
            best = qp3d_real_best_contact_update(best, cp)
          }
        (Shape3D::CapsuleY(cr, ch), Shape3D::Ball(r)) =>
          if compute_ball_capsule_contact(p2.translation, r, p1, cr, ch)
            is Some(cp) {
            best = qp3d_real_best_contact_update(
              best,
              qp3d_real_flip_contact(cp),
            )
          }
        (Shape3D::CapsuleY(r1, h1), Shape3D::CapsuleY(r2, h2)) =>
          if compute_capsule_capsule_contact(p1, r1, h1, p2, r2, h2) is Some(cp) {
            best = qp3d_real_best_contact_update(best, cp)
          }
        (Shape3D::CapsuleY(r, h), Shape3D::Cuboid(he)) =>
          if compute_capsule_cuboid_contact(p1, r, h, p2, he) is Some(cp) {
            best = qp3d_real_best_contact_update(best, cp)
          }
        (Shape3D::Cuboid(he), Shape3D::CapsuleY(r, h)) =>
          if compute_capsule_cuboid_contact(p2, r, h, p1, he) is Some(cp) {
            best = qp3d_real_best_contact_update(
              best,
              qp3d_real_flip_contact(cp),
            )
          }
        (Shape3D::Cylinder(cr, ch), Shape3D::Cuboid(he)) =>
          if compute_cylinder_cuboid_contact(p1, cr, ch, p2, he) is Some(cp) {
            best = qp3d_real_best_contact_update(best, cp)
          }
        (Shape3D::Cuboid(he), Shape3D::Cylinder(cr, ch)) =>
          if compute_cylinder_cuboid_contact(p2, cr, ch, p1, he) is Some(cp) {
            best = qp3d_real_best_contact_update(
              best,
              qp3d_real_flip_contact(cp),
            )
          }
        (Shape3D::RoundCylinder(cr, ch, br), Shape3D::Cuboid(he)) =>
          if compute_cylinder_cuboid_contact(p1, cr + br, ch + br, p2, he)
            is Some(cp) {
            best = qp3d_real_best_contact_update(best, cp)
          }
        (Shape3D::Cuboid(he), Shape3D::RoundCylinder(cr, ch, br)) =>
          if compute_cylinder_cuboid_contact(p2, cr + br, ch + br, p1, he)
            is Some(cp) {
            best = qp3d_real_best_contact_update(
              best,
              qp3d_real_flip_contact(cp),
            )
          }
        (Shape3D::Cylinder(r1, h1), Shape3D::Cylinder(r2, h2)) =>
          if compute_cylinder_cylinder_contact(p1, r1, h1, p2, r2, h2)
            is Some(cp) {
            best = qp3d_real_best_contact_update(best, cp)
          }
        (Shape3D::Cylinder(r1, h1), Shape3D::RoundCylinder(r2, h2, br2)) =>
          if compute_cylinder_cylinder_contact(
              p1,
              r1,
              h1,
              p2,
              r2 + br2,
              h2 + br2,
            )
            is Some(cp) {
            best = qp3d_real_best_contact_update(best, cp)
          }
        (Shape3D::RoundCylinder(r1, h1, br1), Shape3D::Cylinder(r2, h2)) =>
          if compute_cylinder_cylinder_contact(
              p1,
              r1 + br1,
              h1 + br1,
              p2,
              r2,
              h2,
            )
            is Some(cp) {
            best = qp3d_real_best_contact_update(best, cp)
          }
        (
          Shape3D::RoundCylinder(r1, h1, br1),
          Shape3D::RoundCylinder(r2, h2, br2),
        ) =>
          if compute_cylinder_cylinder_contact(
              p1,
              r1 + br1,
              h1 + br1,
              p2,
              r2 + br2,
              h2 + br2,
            )
            is Some(cp) {
            best = qp3d_real_best_contact_update(best, cp)
          }
        (Shape3D::Cone(cr, ch), Shape3D::Cuboid(he)) =>
          if compute_cone_cuboid_contact(p1, cr, ch, p2, he) is Some(cp) {
            best = qp3d_real_best_contact_update(best, cp)
          }
        (Shape3D::Cuboid(he), Shape3D::Cone(cr, ch)) =>
          if compute_cone_cuboid_contact(p2, cr, ch, p1, he) is Some(cp) {
            best = qp3d_real_best_contact_update(
              best,
              qp3d_real_flip_contact(cp),
            )
          }
        (Shape3D::Cone(r1, h1), Shape3D::Cylinder(r2, h2)) =>
          if compute_cone_cylinder_contact(p1, r1, h1, p2, r2, h2) is Some(cp) {
            best = qp3d_real_best_contact_update(best, cp)
          }
        (Shape3D::Cone(r1, h1), Shape3D::RoundCylinder(r2, h2, br2)) =>
          if compute_cone_cylinder_contact(p1, r1, h1, p2, r2 + br2, h2 + br2)
            is Some(cp) {
            best = qp3d_real_best_contact_update(best, cp)
          }
        (Shape3D::Cylinder(r1, h1), Shape3D::Cone(r2, h2)) =>
          if compute_cone_cylinder_contact(p2, r2, h2, p1, r1, h1) is Some(cp) {
            best = qp3d_real_best_contact_update(
              best,
              qp3d_real_flip_contact(cp),
            )
          }
        (Shape3D::RoundCylinder(r1, h1, br1), Shape3D::Cone(r2, h2)) =>
          if compute_cone_cylinder_contact(p2, r2, h2, p1, r1 + br1, h1 + br1)
            is Some(cp) {
            best = qp3d_real_best_contact_update(
              best,
              qp3d_real_flip_contact(cp),
            )
          }
        (Shape3D::Cone(r1, h1), Shape3D::Cone(r2, h2)) =>
          if compute_cone_cone_contact(p1, r1, h1, p2, r2, h2) is Some(cp) {
            best = qp3d_real_best_contact_update(best, cp)
          }
        (Shape3D::Ball(r), Shape3D::Cylinder(cr, ch)) =>
          if compute_ball_cylinder_contact(p1.translation, r, p2, cr, ch)
            is Some(cp) {
            best = qp3d_real_best_contact_update(best, cp)
          }
        (Shape3D::Cylinder(cr, ch), Shape3D::Ball(r)) =>
          if compute_ball_cylinder_contact(p2.translation, r, p1, cr, ch)
            is Some(cp) {
            best = qp3d_real_best_contact_update(
              best,
              qp3d_real_flip_contact(cp),
            )
          }
        (Shape3D::Ball(r), Shape3D::RoundCylinder(cr, ch, br)) =>
          if compute_ball_cylinder_contact(
              p1.translation,
              r,
              p2,
              cr + br,
              ch + br,
            )
            is Some(cp) {
            best = qp3d_real_best_contact_update(best, cp)
          }
        (Shape3D::RoundCylinder(cr, ch, br), Shape3D::Ball(r)) =>
          if compute_ball_cylinder_contact(
              p2.translation,
              r,
              p1,
              cr + br,
              ch + br,
            )
            is Some(cp) {
            best = qp3d_real_best_contact_update(
              best,
              qp3d_real_flip_contact(cp),
            )
          }
        (Shape3D::Ball(r), Shape3D::Cone(cr, ch)) =>
          if compute_ball_cone_contact(p1.translation, r, p2, cr, ch)
            is Some(cp) {
            best = qp3d_real_best_contact_update(best, cp)
          }
        (Shape3D::Cone(cr, ch), Shape3D::Ball(r)) =>
          if compute_ball_cone_contact(p2.translation, r, p1, cr, ch)
            is Some(cp) {
            best = qp3d_real_best_contact_update(
              best,
              qp3d_real_flip_contact(cp),
            )
          }
        (Shape3D::Ball(r), Shape3D::TriMesh(vertices, indices)) =>
          if compute_ball_trimesh_contact(
              p1.translation,
              r,
              p2,
              vertices,
              indices,
            )
            is Some(cp) {
            best = qp3d_real_best_contact_update(best, cp)
          }
        (Shape3D::Ball(r), Shape3D::Triangle(a, b, c)) => {
          let vtx = [a, b, c]
          let idx = [(0, 1, 2)]
          if compute_ball_trimesh_contact(p1.translation, r, p2, vtx, idx)
            is Some(cp) {
            best = qp3d_real_best_contact_update(best, cp)
          }
        }
        (Shape3D::Ball(r), Shape3D::Heightfield(vertices, _, rows, cols)) =>
          if compute_ball_heightfield_contact(
              p1.translation,
              r,
              p2,
              vertices,
              rows,
              cols,
            )
            is Some(cp) {
            best = qp3d_real_best_contact_update(best, cp)
          }
        (Shape3D::Ball(r), Shape3D::Voxels(v)) =>
          if compute_ball_voxels_contact(p1.translation, r, p2, v) is Some(cp) {
            best = qp3d_real_best_contact_update(best, cp)
          }
        (Shape3D::TriMesh(vertices, indices), Shape3D::Ball(r)) =>
          if compute_ball_trimesh_contact(
              p2.translation,
              r,
              p1,
              vertices,
              indices,
            )
            is Some(cp) {
            best = qp3d_real_best_contact_update(
              best,
              qp3d_real_flip_contact(cp),
            )
          }
        (Shape3D::Triangle(a, b, c), Shape3D::Ball(r)) => {
          let vtx = [a, b, c]
          let idx = [(0, 1, 2)]
          if compute_ball_trimesh_contact(p2.translation, r, p1, vtx, idx)
            is Some(cp) {
            best = qp3d_real_best_contact_update(
              best,
              qp3d_real_flip_contact(cp),
            )
          }
        }
        (Shape3D::Heightfield(vertices, _, rows, cols), Shape3D::Ball(r)) =>
          if compute_ball_heightfield_contact(
              p2.translation,
              r,
              p1,
              vertices,
              rows,
              cols,
            )
            is Some(cp) {
            best = qp3d_real_best_contact_update(
              best,
              qp3d_real_flip_contact(cp),
            )
          }
        (Shape3D::Voxels(v), Shape3D::Ball(r)) =>
          if compute_ball_voxels_contact(p2.translation, r, p1, v) is Some(cp) {
            best = qp3d_real_best_contact_update(
              best,
              qp3d_real_flip_contact(cp),
            )
          }
        (Shape3D::Cuboid(he), Shape3D::TriMesh(vertices, indices)) =>
          if compute_cuboid_trimesh_contact(p1, he, p2, vertices, indices)
            is Some(cp) {
            best = qp3d_real_best_contact_update(best, cp)
          }
        (Shape3D::Cuboid(he), Shape3D::Triangle(a, b, c)) => {
          let vtx = [a, b, c]
          let idx = [(0, 1, 2)]
          if compute_cuboid_trimesh_contact(p1, he, p2, vtx, idx) is Some(cp) {
            best = qp3d_real_best_contact_update(best, cp)
          }
        }
        (Shape3D::Cuboid(he), Shape3D::Heightfield(vertices, _, rows, cols)) =>
          if compute_cuboid_heightfield_contact(
              p1, he, p2, vertices, rows, cols,
            )
            is Some(cp) {
            best = qp3d_real_best_contact_update(best, cp)
          }
        (Shape3D::Cuboid(he), Shape3D::Voxels(v)) =>
          if compute_cuboid_voxels_contact(p1, he, p2, v) is Some(cp) {
            best = qp3d_real_best_contact_update(best, cp)
          }
        (Shape3D::TriMesh(vertices, indices), Shape3D::Cuboid(he)) =>
          if compute_cuboid_trimesh_contact(p2, he, p1, vertices, indices)
            is Some(cp) {
            best = qp3d_real_best_contact_update(
              best,
              qp3d_real_flip_contact(cp),
            )
          }
        (Shape3D::Triangle(a, b, c), Shape3D::Cuboid(he)) => {
          let vtx = [a, b, c]
          let idx = [(0, 1, 2)]
          if compute_cuboid_trimesh_contact(p2, he, p1, vtx, idx) is Some(cp) {
            best = qp3d_real_best_contact_update(
              best,
              qp3d_real_flip_contact(cp),
            )
          }
        }
        (Shape3D::Heightfield(vertices, _, rows, cols), Shape3D::Cuboid(he)) =>
          if compute_cuboid_heightfield_contact(
              p2, he, p1, vertices, rows, cols,
            )
            is Some(cp) {
            best = qp3d_real_best_contact_update(
              best,
              qp3d_real_flip_contact(cp),
            )
          }
        (Shape3D::Voxels(v), Shape3D::Cuboid(he)) =>
          if compute_cuboid_voxels_contact(p2, he, p1, v) is Some(cp) {
            best = qp3d_real_best_contact_update(
              best,
              qp3d_real_flip_contact(cp),
            )
          }
        (Shape3D::CapsuleY(r, hh), Shape3D::TriMesh(vertices, indices)) =>
          if compute_capsule_trimesh_contact(p1, r, hh, p2, vertices, indices)
            is Some(cp) {
            best = qp3d_real_best_contact_update(best, cp)
          }
        (Shape3D::CapsuleY(r, hh), Shape3D::Triangle(a, b, c)) => {
          let vtx = [a, b, c]
          let idx = [(0, 1, 2)]
          if compute_capsule_trimesh_contact(p1, r, hh, p2, vtx, idx)
            is Some(cp) {
            best = qp3d_real_best_contact_update(best, cp)
          }
        }
        (Shape3D::CapsuleY(r, hh), Shape3D::Heightfield(vertices, _, rows, cols)
        ) =>
          if compute_capsule_heightfield_contact(
              p1, r, hh, p2, vertices, rows, cols,
            )
            is Some(cp) {
            best = qp3d_real_best_contact_update(best, cp)
          }
        (Shape3D::CapsuleY(r, hh), Shape3D::Voxels(v)) =>
          if compute_capsule_voxels_contact(p1, r, hh, p2, v) is Some(cp) {
            best = qp3d_real_best_contact_update(best, cp)
          }
        (Shape3D::TriMesh(vertices, indices), Shape3D::CapsuleY(r, hh)) =>
          if compute_capsule_trimesh_contact(p2, r, hh, p1, vertices, indices)
            is Some(cp) {
            best = qp3d_real_best_contact_update(
              best,
              qp3d_real_flip_contact(cp),
            )
          }
        (Shape3D::Triangle(a, b, c), Shape3D::CapsuleY(r, hh)) => {
          let vtx = [a, b, c]
          let idx = [(0, 1, 2)]
          if compute_capsule_trimesh_contact(p2, r, hh, p1, vtx, idx)
            is Some(cp) {
            best = qp3d_real_best_contact_update(
              best,
              qp3d_real_flip_contact(cp),
            )
          }
        }
        (Shape3D::Heightfield(vertices, _, rows, cols), Shape3D::CapsuleY(r, hh)
        ) =>
          if compute_capsule_heightfield_contact(
              p2, r, hh, p1, vertices, rows, cols,
            )
            is Some(cp) {
            best = qp3d_real_best_contact_update(
              best,
              qp3d_real_flip_contact(cp),
            )
          }
        (Shape3D::Voxels(v), Shape3D::CapsuleY(r, hh)) =>
          if compute_capsule_voxels_contact(p2, r, hh, p1, v) is Some(cp) {
            best = qp3d_real_best_contact_update(
              best,
              qp3d_real_flip_contact(cp),
            )
          }
        (Shape3D::Cylinder(r, hh), Shape3D::TriMesh(vertices, indices)) =>
          if compute_cylinder_trimesh_contact(p1, r, hh, p2, vertices, indices)
            is Some(cp) {
            best = qp3d_real_best_contact_update(best, cp)
          }
        (Shape3D::Cylinder(r, hh), Shape3D::Triangle(a, b, c)) => {
          let vtx = [a, b, c]
          let idx = [(0, 1, 2)]
          if compute_cylinder_trimesh_contact(p1, r, hh, p2, vtx, idx)
            is Some(cp) {
            best = qp3d_real_best_contact_update(best, cp)
          }
        }
        (Shape3D::Cylinder(r, hh), Shape3D::Heightfield(vertices, _, rows, cols)
        ) =>
          if compute_cylinder_heightfield_contact(
              p1, r, hh, p2, vertices, rows, cols,
            )
            is Some(cp) {
            best = qp3d_real_best_contact_update(best, cp)
          }
        (Shape3D::Cylinder(r, hh), Shape3D::Voxels(v)) =>
          if compute_cylinder_voxels_contact(p1, r, hh, p2, v) is Some(cp) {
            best = qp3d_real_best_contact_update(best, cp)
          }
        (Shape3D::TriMesh(vertices, indices), Shape3D::Cylinder(r, hh)) =>
          if compute_cylinder_trimesh_contact(p2, r, hh, p1, vertices, indices)
            is Some(cp) {
            best = qp3d_real_best_contact_update(
              best,
              qp3d_real_flip_contact(cp),
            )
          }
        (Shape3D::Triangle(a, b, c), Shape3D::Cylinder(r, hh)) => {
          let vtx = [a, b, c]
          let idx = [(0, 1, 2)]
          if compute_cylinder_trimesh_contact(p2, r, hh, p1, vtx, idx)
            is Some(cp) {
            best = qp3d_real_best_contact_update(
              best,
              qp3d_real_flip_contact(cp),
            )
          }
        }
        (Shape3D::Heightfield(vertices, _, rows, cols), Shape3D::Cylinder(r, hh)
        ) =>
          if compute_cylinder_heightfield_contact(
              p2, r, hh, p1, vertices, rows, cols,
            )
            is Some(cp) {
            best = qp3d_real_best_contact_update(
              best,
              qp3d_real_flip_contact(cp),
            )
          }
        (Shape3D::Voxels(v), Shape3D::Cylinder(r, hh)) =>
          if compute_cylinder_voxels_contact(p2, r, hh, p1, v) is Some(cp) {
            best = qp3d_real_best_contact_update(
              best,
              qp3d_real_flip_contact(cp),
            )
          }
        (Shape3D::RoundCylinder(r, hh, br), Shape3D::TriMesh(vertices, indices)) =>
          if compute_round_cylinder_trimesh_contact(
              p1, r, hh, br, p2, vertices, indices,
            )
            is Some(cp) {
            best = qp3d_real_best_contact_update(best, cp)
          }
        (Shape3D::RoundCylinder(r, hh, br), Shape3D::Triangle(a, b, c)) => {
          let vtx = [a, b, c]
          let idx = [(0, 1, 2)]
          if compute_round_cylinder_trimesh_contact(p1, r, hh, br, p2, vtx, idx)
            is Some(cp) {
            best = qp3d_real_best_contact_update(best, cp)
          }
        }
        (
          Shape3D::RoundCylinder(r, hh, br),
          Shape3D::Heightfield(vertices, _, rows, cols),
        ) =>
          if compute_round_cylinder_heightfield_contact(
              p1, r, hh, br, p2, vertices, rows, cols,
            )
            is Some(cp) {
            best = qp3d_real_best_contact_update(best, cp)
          }
        (Shape3D::RoundCylinder(r, hh, br), Shape3D::Voxels(v)) =>
          if compute_cylinder_voxels_contact(p1, r + br, hh + br, p2, v)
            is Some(cp) {
            best = qp3d_real_best_contact_update(best, cp)
          }
        (Shape3D::TriMesh(vertices, indices), Shape3D::RoundCylinder(r, hh, br)) =>
          if compute_round_cylinder_trimesh_contact(
              p2, r, hh, br, p1, vertices, indices,
            )
            is Some(cp) {
            best = qp3d_real_best_contact_update(
              best,
              qp3d_real_flip_contact(cp),
            )
          }
        (Shape3D::Triangle(a, b, c), Shape3D::RoundCylinder(r, hh, br)) => {
          let vtx = [a, b, c]
          let idx = [(0, 1, 2)]
          if compute_round_cylinder_trimesh_contact(p2, r, hh, br, p1, vtx, idx)
            is Some(cp) {
            best = qp3d_real_best_contact_update(
              best,
              qp3d_real_flip_contact(cp),
            )
          }
        }
        (
          Shape3D::Heightfield(vertices, _, rows, cols),
          Shape3D::RoundCylinder(r, hh, br),
        ) =>
          if compute_round_cylinder_heightfield_contact(
              p2, r, hh, br, p1, vertices, rows, cols,
            )
            is Some(cp) {
            best = qp3d_real_best_contact_update(
              best,
              qp3d_real_flip_contact(cp),
            )
          }
        (Shape3D::Voxels(v), Shape3D::RoundCylinder(r, hh, br)) =>
          if compute_cylinder_voxels_contact(p2, r + br, hh + br, p1, v)
            is Some(cp) {
            best = qp3d_real_best_contact_update(
              best,
              qp3d_real_flip_contact(cp),
            )
          }
        (Shape3D::Cone(r, hh), Shape3D::TriMesh(vertices, indices)) =>
          if compute_cone_trimesh_contact(p1, r, hh, p2, vertices, indices)
            is Some(cp) {
            best = qp3d_real_best_contact_update(best, cp)
          }
        (Shape3D::Cone(r, hh), Shape3D::Triangle(a, b, c)) => {
          let vtx = [a, b, c]
          let idx = [(0, 1, 2)]
          if compute_cone_trimesh_contact(p1, r, hh, p2, vtx, idx) is Some(cp) {
            best = qp3d_real_best_contact_update(best, cp)
          }
        }
        (Shape3D::Cone(r, hh), Shape3D::Heightfield(vertices, _, rows, cols)) =>
          if compute_cone_heightfield_contact(
              p1, r, hh, p2, vertices, rows, cols,
            )
            is Some(cp) {
            best = qp3d_real_best_contact_update(best, cp)
          }
        (Shape3D::Cone(r, hh), Shape3D::Voxels(v)) =>
          if compute_cone_voxels_contact(p1, r, hh, p2, v) is Some(cp) {
            best = qp3d_real_best_contact_update(best, cp)
          }
        (Shape3D::TriMesh(vertices, indices), Shape3D::Cone(r, hh)) =>
          if compute_cone_trimesh_contact(p2, r, hh, p1, vertices, indices)
            is Some(cp) {
            best = qp3d_real_best_contact_update(
              best,
              qp3d_real_flip_contact(cp),
            )
          }
        (Shape3D::Triangle(a, b, c), Shape3D::Cone(r, hh)) => {
          let vtx = [a, b, c]
          let idx = [(0, 1, 2)]
          if compute_cone_trimesh_contact(p2, r, hh, p1, vtx, idx) is Some(cp) {
            best = qp3d_real_best_contact_update(
              best,
              qp3d_real_flip_contact(cp),
            )
          }
        }
        (Shape3D::Heightfield(vertices, _, rows, cols), Shape3D::Cone(r, hh)) =>
          if compute_cone_heightfield_contact(
              p2, r, hh, p1, vertices, rows, cols,
            )
            is Some(cp) {
            best = qp3d_real_best_contact_update(
              best,
              qp3d_real_flip_contact(cp),
            )
          }
        (Shape3D::Voxels(v), Shape3D::Cone(r, hh)) =>
          if compute_cone_voxels_contact(p2, r, hh, p1, v) is Some(cp) {
            best = qp3d_real_best_contact_update(
              best,
              qp3d_real_flip_contact(cp),
            )
          }
        _ =>
          if compute_convex_contact(p1, s1, p2, s2) is Some(cp) {
            best = qp3d_real_best_contact_update(best, cp)
          }
      }
      best
    }
  }
}

///|
fn qp3d_real_cast_compute_steps(
  p0 : @core.Vec3,
  p1 : @core.Vec3,
  radius : @core.Real,
) -> Int {
  let dist = p1.sub(p0).length()
  let trans_scale = radius * 0.25F + 0.05F
  let trans_steps = if trans_scale <= 1.0e-6F {
    64
  } else {
    (dist / trans_scale).to_int() + 1
  }
  let mut steps = trans_steps
  if steps < 8 {
    steps = 8
  }
  if steps > 256 {
    steps = 256
  }
  steps
}

///|
fn qp3d_real_cast_moving_shape(
  shape_pos : @core.Isometry3,
  shape_vel : @core.Vec3,
  shape : Shape3D,
  target_pos : @core.Isometry3,
  target_shape : Shape3D,
  options : ShapeCastOptions3,
) -> ShapeCastHit3? {
  let max_toi = options.max_toi
  if max_toi < 0.0F {
    return None
  }
  let target_distance = options.target_distance
  let stop_at_penetration = options.stop_at_penetration
  match shape {
    // Keep the efficient/specialized implementation for ball casts.
    Shape3D::Ball(r) => {
      let eff_r = r + target_distance
      qp3d_real_cast_ball_against_shape(
        shape_pos.translation,
        shape_vel,
        eff_r,
        target_pos,
        target_shape,
        max_toi,
      )
    }
    _ => {
      // Generic conservative shape cast: scan for the first time the moving (inflated) shape
      // starts intersecting the target shape, then bisect for a tight TOI.
      let moving = qp3d_real_inflate_shape(shape, target_distance)
      let duration = max_toi
      let vel2 = shape_vel.length_squared()
      let radius = qp3d_real_shape_bounding_radius(moving)
      fn pose_at_time(
        p : @core.Isometry3,
        v : @core.Vec3,
        t : @core.Real,
      ) -> @core.Isometry3 {
        @core.Isometry3::new(p.translation.add(v.scale(t)), p.rotation)
      }

      fn contact_at_time(
        t : @core.Real,
        shape_pos : @core.Isometry3,
        shape_vel : @core.Vec3,
        moving : Shape3D,
        target_pos : @core.Isometry3,
        target_shape : Shape3D,
      ) -> ContactPoint3D? {
        let p = pose_at_time(shape_pos, shape_vel, t)
        qp3d_real_best_contact_for_pair(p, moving, target_pos, target_shape)
      }

      fn intersects_at_time(
        t : @core.Real,
        shape_pos : @core.Isometry3,
        shape_vel : @core.Vec3,
        moving : Shape3D,
        target_pos : @core.Isometry3,
        target_shape : Shape3D,
      ) -> Bool {
        contact_at_time(
          t, shape_pos, shape_vel, moving, target_pos, target_shape,
        )
        is Some(_)
      }

      fn bisect_transition(
        lo : @core.Real,
        hi : @core.Real,
        want_intersection_at_hi : Bool,
        shape_pos : @core.Isometry3,
        shape_vel : @core.Vec3,
        moving : Shape3D,
        target_pos : @core.Isometry3,
        target_shape : Shape3D,
      ) -> @core.Real {
        let mut left = lo
        let mut right = hi
        for _ in 0..<25 {
          let mid = 0.5F * (left + right)
          let mid_state = intersects_at_time(
            mid, shape_pos, shape_vel, moving, target_pos, target_shape,
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
        radius : @core.Real,
        shape_pos : @core.Isometry3,
        shape_vel : @core.Vec3,
        moving : Shape3D,
        target_pos : @core.Isometry3,
        target_shape : Shape3D,
      ) -> @core.Real? {
        let duration = scan_end - scan_start
        if duration <= 0.0F {
          return None
        }
        let p0 = pose_at_time(shape_pos, shape_vel, scan_start).translation
        let p1 = pose_at_time(shape_pos, shape_vel, scan_end).translation
        let steps = qp3d_real_cast_compute_steps(p0, p1, radius)
        let dt = duration / Float::from_double(steps.to_double())
        let mut prev_t = scan_start
        for si in 1..<(steps + 1) {
          let mut t = scan_start + dt * Float::from_double(si.to_double())
          if t > scan_end {
            t = scan_end
          }
          if intersects_at_time(
              t, shape_pos, shape_vel, moving, target_pos, target_shape,
            ) {
            return Some(
              bisect_transition(
                prev_t, t, true, shape_pos, shape_vel, moving, target_pos, target_shape,
              ),
            )
          }
          prev_t = t
        }
        None
      }

      if intersects_at_time(
          0.0F, shape_pos, shape_vel, moving, target_pos, target_shape,
        ) {
        if contact_at_time(
            0.0F, shape_pos, shape_vel, moving, target_pos, target_shape,
          )
          is Some(cp) {
          let n = cp.normal.scale(-1.0F) // target -> moving
          if !stop_at_penetration && n.dot(shape_vel) >= 0.0F {
            // Separating from an initial penetration: ignore this hit (linear motion can't re-enter).
            return None
          }
          return Some({ toi: 0.0F, point: cp.point2, normal: n })
        } else {
          return None
        }
      }
      if vel2 <= 1.0e-12F {
        return None
      }
      if find_first_entry_time(
          0.0F, duration, radius, shape_pos, shape_vel, moving, target_pos, target_shape,
        )
        is Some(toi) {
        if contact_at_time(
            toi, shape_pos, shape_vel, moving, target_pos, target_shape,
          )
          is Some(cp) {
          let n = cp.normal.scale(-1.0F) // target -> moving
          let hit : ShapeCastHit3 = { toi, point: cp.point2, normal: n }
          if !stop_at_penetration &&
            hit.toi < 1.0e-5F &&
            hit.normal.dot(shape_vel) >= 0.0F {
            None
          } else {
            Some(hit)
          }
        } else {
          None
        }
      } else {
        None
      }
    }
  }
}

///|
pub fn QueryPipeline3DReal::cast_shape(
  self : QueryPipeline3DReal,
  bodies : @dynamics.RigidBodySet3D,
  colliders : ColliderSet3D,
  shape_pos : @core.Isometry3,
  shape_vel : @core.Vec3,
  shape : Shape3D,
  options : ShapeCastOptions3,
) -> (ColliderHandle3D, ShapeCastHit3)? {
  let max_toi = options.max_toi()
  let td = options.target_distance()
  // Broad-phase culling: compute a conservative swept AABB for the moving shape.
  fn abs_mat3(m : @core.Mat3) -> @core.Mat3 {
    @core.Mat3::new(
      @core.abs(m.m00),
      @core.abs(m.m01),
      @core.abs(m.m02),
      @core.abs(m.m10),
      @core.abs(m.m11),
      @core.abs(m.m12),
      @core.abs(m.m20),
      @core.abs(m.m21),
      @core.abs(m.m22),
    )
  }

  fn aabb3_transform(iso : @core.Isometry3, aabb : @core.Aabb3) -> @core.Aabb3 {
    let center = aabb.mins.add(aabb.maxs).scale(0.5F)
    let half = aabb.maxs.sub(aabb.mins).scale(0.5F)
    let rot = abs_mat3(iso.rotation.to_mat3())
    let ext = rot.mul_vec3(half)
    let world_center = iso.transform_point(center)
    @core.Aabb3::new(world_center.sub(ext), world_center.add(ext))
  }

  let local_aabb = shape.local_aabb()
  let start_aabb = aabb3_transform(shape_pos, local_aabb)
  let end_pos = @core.Isometry3::new(
    shape_pos.translation.add(shape_vel.scale(max_toi)),
    shape_pos.rotation,
  )
  let end_aabb = aabb3_transform(end_pos, local_aabb)
  let sweep_aabb = start_aabb.combine(end_aabb).dilated(td)
  let mut best : (ColliderHandle3D, ShapeCastHit3)? = None
  let mut best_t = max_toi + 1.0F
  let n = colliders.colliders.length()
  for i in 0..<n {
    if colliders.colliders[i] is Some(co) {
      let h = ColliderHandle3D::from_raw_parts(i, colliders.generations[i])
      if !filter_pass_3d_real(self.filter, bodies, h, co) {
        continue
      }
      if self.cached_aabbs[i] is Some(aabb) && !sweep_aabb.intersects(aabb) {
        continue
      }
      if qp3d_real_cast_moving_shape(
          shape_pos,
          shape_vel,
          shape,
          co.position(),
          co.shape(),
          options,
        )
        is Some(hit) {
        if hit.toi < best_t {
          best_t = hit.toi
          best = Some((h, hit))
        }
      }
    }
  }
  best
}
