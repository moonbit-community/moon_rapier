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
/// Minimal real 3D query pipeline.
///
/// This is intentionally brute-force (iterate all colliders) and currently focuses on ray casting
/// for the subset of shapes exercised by migrated examples (vehicle controller, character controller).
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
pub struct QueryFilter3DReal {
  exclude_rigid_body : @dynamics.RigidBodyHandle?
  exclude_collider : ColliderHandle3D?
  groups : @dynamics.InteractionGroups
}

///|
pub fn QueryFilter3DReal::new() -> QueryFilter3DReal {
  {
    exclude_rigid_body: None,
    exclude_collider: None,
    groups: @dynamics.InteractionGroups::all(),
  }
}

///|
pub fn QueryFilter3DReal::exclude_rigid_body(
  self : QueryFilter3DReal,
  handle : @dynamics.RigidBodyHandle,
) -> QueryFilter3DReal {
  {
    exclude_rigid_body: Some(handle),
    exclude_collider: self.exclude_collider,
    groups: self.groups,
  }
}

///|
pub fn QueryFilter3DReal::exclude_collider(
  self : QueryFilter3DReal,
  handle : ColliderHandle3D,
) -> QueryFilter3DReal {
  {
    exclude_rigid_body: self.exclude_rigid_body,
    exclude_collider: Some(handle),
    groups: self.groups,
  }
}

///|
pub fn QueryFilter3DReal::groups(
  self : QueryFilter3DReal,
  groups : @dynamics.InteractionGroups,
) -> QueryFilter3DReal {
  {
    exclude_rigid_body: self.exclude_rigid_body,
    exclude_collider: self.exclude_collider,
    groups,
  }
}

///|
fn filter_pass_3d_real(
  filter : QueryFilter3DReal,
  handle : ColliderHandle3D,
  collider : Collider3D,
) -> Bool {
  if !collider.enabled() {
    return false
  }
  if filter.exclude_collider is Some(h) && h.equals(handle) {
    return false
  }
  if filter.exclude_rigid_body is Some(bh) &&
    collider.parent() is Some(p) &&
    p.equals(bh) {
    return false
  }
  if !filter.groups.test_groups(collider.collision_groups()) {
    return false
  }
  true
}

///|
pub struct QueryPipeline3DReal {
  filter : QueryFilter3DReal
  bodies : @dynamics.RigidBodySet3D
  colliders : ColliderSet3D
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
  { filter, bodies, colliders }
}

///|
pub fn QueryPipeline3DReal::update(self : QueryPipeline3DReal) -> Unit {
  // Brute-force queries always read the current collider poses.
  self |> ignore
}

///|
pub fn QueryPipeline3DReal::with_filter(
  self : QueryPipeline3DReal,
  filter : QueryFilter3DReal,
) -> QueryPipeline3DReal {
  { filter, bodies: self.bodies, colliders: self.colliders }
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
pub fn QueryPipelineMut3DReal::update(self : QueryPipelineMut3DReal) -> Unit {
  self.pipeline.update()
}

///|
pub fn QueryPipeline3DReal::rigid_body(
  self : QueryPipeline3DReal,
  handle : @dynamics.RigidBodyHandle,
) -> @dynamics.RigidBody3D? {
  self.bodies.get(handle)
}

///|
pub fn QueryPipeline3DReal::rigid_body_mut(
  self : QueryPipeline3DReal,
  handle : @dynamics.RigidBodyHandle,
) -> @dynamics.RigidBody3D? {
  self.bodies.get_mut(handle)
}

///|
pub fn QueryPipeline3DReal::collider(
  self : QueryPipeline3DReal,
  handle : ColliderHandle3D,
) -> Collider3D? {
  self.colliders.get(handle)
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
  ray : Ray3,
  max_toi : @core.Real,
  solid : Bool,
) -> (ColliderHandle3D, RayIntersection3)? {
  if QueryPipeline3DReal::cast_ray_and_get_normal_and_feature(
      self, ray, max_toi, solid,
    )
    is Some((h, it)) {
    Some((h, { toi: it.toi, normal: it.normal }))
  } else {
    None
  }
}

///|
pub fn QueryPipeline3DReal::cast_ray_and_get_normal_and_feature(
  self : QueryPipeline3DReal,
  ray : Ray3,
  max_toi : @core.Real,
  solid : Bool,
) -> (ColliderHandle3D, RayIntersection3Feature)? {
  solid |> ignore
  let handles = self.colliders.all_handles()
  let mut best_t = max_toi + 1.0F
  let mut best : (ColliderHandle3D, RayIntersection3Feature)? = None
  fn hit_shape(
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
      Shape3D::ConvexHull(_, _) => None
      Shape3D::Compound(parts) => {
        let mut best : RayIntersection3Feature? = None
        let mut best_t = max_toi + 1.0F
        for i in 0..<parts.length() {
          let (iso, sh) = parts[i]
          let child_pos = pos.mul(iso)
          if hit_shape(ray, child_pos, sh, max_toi) is Some(it) &&
            it.toi < best_t {
            best_t = it.toi
            best = Some(it)
          }
        }
        best
      }
      Shape3D::Heightfield(vertices, indices, _, _) => {
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

  for i in 0..<handles.length() {
    let h = handles[i]
    if self.colliders.get(h) is Some(co) {
      if !filter_pass_3d_real(self.filter, h, co) {
        continue
      }
      let pos = co.position()
      let hit = hit_shape(ray, pos, co.shape(), max_toi)
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
  ray : Ray3,
  max_toi : @core.Real,
  solid : Bool,
) -> (ColliderHandle3D, RayIntersection3, (Int, Int, Int))? {
  if QueryPipeline3DReal::cast_ray_and_get_normal_and_feature(
      self, ray, max_toi, solid,
    )
    is Some((h, it)) {
    if it.feature_id >= 0 && self.colliders.get(h) is Some(co) {
      if co.voxel_key_for_triangle(it.feature_id) is Some(key) {
        return Some((h, { toi: it.toi, normal: it.normal }, key))
      }
    }
    None
  } else {
    None
  }
}
