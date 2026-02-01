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
pub struct ContactPoint3D {
  point1 : @core.Vec3
  point2 : @core.Vec3
  normal : @core.Vec3
  penetration : @core.Real
}

///|
priv enum CuboidBestAxis {
  A(Int)
  B(Int)
  Cross(Int, Int)
}

///|
fn local_sqrt(value : @core.Real) -> @core.Real {
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
fn local_max(a : @core.Real, b : @core.Real) -> @core.Real {
  if a > b {
    a
  } else {
    b
  }
}

///|
fn local_min(a : @core.Real, b : @core.Real) -> @core.Real {
  if a < b {
    a
  } else {
    b
  }
}

///|
fn closest_point_on_triangle3d(
  p : @core.Vec3,
  a : @core.Vec3,
  b : @core.Vec3,
  c : @core.Vec3,
) -> @core.Vec3 {
  // Real-Time Collision Detection (Christer Ericson), 5.1.5.
  let ab = b.sub(a)
  let ac = c.sub(a)
  let ap = p.sub(a)
  let d1 = ab.dot(ap)
  let d2 = ac.dot(ap)
  if d1 <= 0.0F && d2 <= 0.0F {
    return a
  }
  let bp = p.sub(b)
  let d3 = ab.dot(bp)
  let d4 = ac.dot(bp)
  if d3 >= 0.0F && d4 <= d3 {
    return b
  }
  let vc = d1 * d4 - d3 * d2
  if vc <= 0.0F && d1 >= 0.0F && d3 <= 0.0F {
    let v = d1 / (d1 - d3)
    return a.add(ab.scale(v))
  }
  let cp = p.sub(c)
  let d5 = ab.dot(cp)
  let d6 = ac.dot(cp)
  if d6 >= 0.0F && d5 <= d6 {
    return c
  }
  let vb = d5 * d2 - d1 * d6
  if vb <= 0.0F && d2 >= 0.0F && d6 <= 0.0F {
    let w = d2 / (d2 - d6)
    return a.add(ac.scale(w))
  }
  let va = d3 * d6 - d5 * d4
  if va <= 0.0F && d4 - d3 >= 0.0F && d5 - d6 >= 0.0F {
    let w = (d4 - d3) / (d4 - d3 + (d5 - d6))
    return b.add(c.sub(b).scale(w))
  }
  let denom = 1.0F / (va + vb + vc)
  let v = vb * denom
  let w = vc * denom
  a.add(ab.scale(v)).add(ac.scale(w))
}

///|
fn trimesh_closest_point(
  p : @core.Vec3,
  mesh_pos : @core.Isometry3,
  vertices : Array[@core.Vec3],
  indices : Array[(Int, Int, Int)],
) -> (@core.Vec3, @core.Vec3, @core.Real, Bool) {
  if indices.length() == 0 || vertices.length() == 0 {
    return (@core.Vec3::zero(), @core.Vec3::new(0.0F, 1.0F, 0.0F), 0.0F, false)
  }
  let mut best_dist2 = 1.0e30F
  let mut best_point = @core.Vec3::zero()
  let mut best_normal = @core.Vec3::new(0.0F, 1.0F, 0.0F)
  for i in 0..<indices.length() {
    let (i0, i1, i2) = indices[i]
    if i0 < 0 ||
      i1 < 0 ||
      i2 < 0 ||
      i0 >= vertices.length() ||
      i1 >= vertices.length() ||
      i2 >= vertices.length() {
      continue
    }
    let a = mesh_pos.transform_point(vertices[i0])
    let b = mesh_pos.transform_point(vertices[i1])
    let c = mesh_pos.transform_point(vertices[i2])
    let q = closest_point_on_triangle3d(p, a, b, c)
    let d2 = q.sub(p).length_squared()
    if d2 < best_dist2 {
      best_dist2 = d2
      best_point = q
      let n = b.sub(a).cross(c.sub(a))
      if n.length_squared() > 1.0e-12F {
        best_normal = n.normalize()
      }
    }
  }
  (best_point, best_normal, best_dist2, true)
}

///|
fn compute_point_trimesh_contact(
  point1 : @core.Vec3,
  radius : @core.Real,
  mesh_pos : @core.Isometry3,
  vertices : Array[@core.Vec3],
  indices : Array[(Int, Int, Int)],
) -> ContactPoint3D? {
  let (best_point, best_normal, best_dist2, ok) = trimesh_closest_point(
    point1, mesh_pos, vertices, indices,
  )
  if !ok {
    return None
  }
  let r2 = radius * radius
  if best_dist2 > r2 {
    return None
  }
  let dist = local_sqrt(best_dist2)
  let normal0 = if dist > 1.0e-6F {
    best_point.sub(point1).scale(1.0F / dist)
  } else {
    best_normal
  }
  // For the current real dim3 backend, trimeshes are primarily used as terrains/grounds.
  // Flip upward-facing normals so penetrations are pushed back above the surface instead of
  // potentially getting "stuck" below due to unsigned distance queries.
  let normal = if normal0.y > 0.0F { normal0.scale(-1.0F) } else { normal0 }
  Some({ point1, point2: best_point, normal, penetration: radius - dist })
}

///|
pub fn compute_ball_trimesh_contact(
  ball_center : @core.Vec3,
  ball_radius : @core.Real,
  mesh_pos : @core.Isometry3,
  vertices : Array[@core.Vec3],
  indices : Array[(Int, Int, Int)],
) -> ContactPoint3D? {
  let (best_point, best_normal, best_dist2, ok) = trimesh_closest_point(
    ball_center, mesh_pos, vertices, indices,
  )
  if !ok {
    return None
  }
  let r2 = ball_radius * ball_radius
  if best_dist2 > r2 {
    return None
  }
  let dist = local_sqrt(best_dist2)
  let normal = if dist > 1.0e-6F {
    // Pointing from the ball center toward the mesh surface.
    best_point.sub(ball_center).scale(1.0F / dist)
  } else {
    // Fallback to triangle normal.
    best_normal
  }
  let point1 = ball_center.add(normal.scale(ball_radius))
  let point2 = best_point
  Some({ point1, point2, normal, penetration: ball_radius - dist })
}

///|
pub fn compute_cuboid_trimesh_contact(
  cuboid_pos : @core.Isometry3,
  cuboid_half_extents : @core.Vec3,
  mesh_pos : @core.Isometry3,
  vertices : Array[@core.Vec3],
  indices : Array[(Int, Int, Int)],
) -> ContactPoint3D? {
  // Approximation: sample points on the cuboid surface and treat them as small spheres.
  let skin = 0.05F
  let hx = cuboid_half_extents.x
  let hy = cuboid_half_extents.y
  let hz = cuboid_half_extents.z
  let points = [
    // 8 corners.
    @core.Vec3::new(-hx, -hy, -hz),
    @core.Vec3::new(hx, -hy, -hz),
    @core.Vec3::new(-hx, hy, -hz),
    @core.Vec3::new(hx, hy, -hz),
    @core.Vec3::new(-hx, -hy, hz),
    @core.Vec3::new(hx, -hy, hz),
    @core.Vec3::new(-hx, hy, hz),
    @core.Vec3::new(hx, hy, hz),
    // 6 face centers.
    @core.Vec3::new(0.0F, -hy, 0.0F),
    @core.Vec3::new(0.0F, hy, 0.0F),
    @core.Vec3::new(-hx, 0.0F, 0.0F),
    @core.Vec3::new(hx, 0.0F, 0.0F),
    @core.Vec3::new(0.0F, 0.0F, -hz),
    @core.Vec3::new(0.0F, 0.0F, hz),
    // 12 edge midpoints.
    @core.Vec3::new(0.0F, -hy, -hz),
    @core.Vec3::new(0.0F, -hy, hz),
    @core.Vec3::new(0.0F, hy, -hz),
    @core.Vec3::new(0.0F, hy, hz),
    @core.Vec3::new(-hx, 0.0F, -hz),
    @core.Vec3::new(-hx, 0.0F, hz),
    @core.Vec3::new(hx, 0.0F, -hz),
    @core.Vec3::new(hx, 0.0F, hz),
    @core.Vec3::new(-hx, -hy, 0.0F),
    @core.Vec3::new(hx, -hy, 0.0F),
    @core.Vec3::new(-hx, hy, 0.0F),
    @core.Vec3::new(hx, hy, 0.0F),
  ]
  let mut best : ContactPoint3D? = None
  let mut best_pen = -1.0F
  for i in 0..<points.length() {
    let p = cuboid_pos.transform_point(points[i])
    if compute_point_trimesh_contact(p, skin, mesh_pos, vertices, indices)
      is Some(cp) {
      if cp.penetration > best_pen {
        best_pen = cp.penetration
        best = Some(cp)
      }
    }
  }
  best
}

///|
pub fn compute_capsule_trimesh_contact(
  capsule_pos : @core.Isometry3,
  capsule_radius : @core.Real,
  capsule_half_height : @core.Real,
  mesh_pos : @core.Isometry3,
  vertices : Array[@core.Vec3],
  indices : Array[(Int, Int, Int)],
) -> ContactPoint3D? {
  let (a, b) = capsule_segment_endpoints(capsule_pos, capsule_half_height)
  let centers = [
    a,
    b,
    a.add(b).scale(0.5F),
    a.scale(0.75F).add(b.scale(0.25F)),
    a.scale(0.25F).add(b.scale(0.75F)),
  ]
  let mut best : ContactPoint3D? = None
  let mut best_pen = -1.0F
  for i in 0..<centers.length() {
    if compute_ball_trimesh_contact(
        centers[i],
        capsule_radius,
        mesh_pos,
        vertices,
        indices,
      )
      is Some(cp) {
      if cp.penetration > best_pen {
        best_pen = cp.penetration
        best = Some(cp)
      }
    }
  }
  best
}

///|
pub fn compute_cylinder_trimesh_contact(
  cyl_pos : @core.Isometry3,
  cyl_radius : @core.Real,
  cyl_half_height : @core.Real,
  mesh_pos : @core.Isometry3,
  vertices : Array[@core.Vec3],
  indices : Array[(Int, Int, Int)],
) -> ContactPoint3D? {
  // Approximation: sample points on the cylinder surface and treat them as small spheres.
  let skin = 0.05F
  let r = cyl_radius
  let h = cyl_half_height
  let a = 0.70710677F // sqrt(0.5)
  let ring = [
    @core.Vec3::new(r, 0.0F, 0.0F),
    @core.Vec3::new(-r, 0.0F, 0.0F),
    @core.Vec3::new(0.0F, 0.0F, r),
    @core.Vec3::new(0.0F, 0.0F, -r),
    @core.Vec3::new(r * a, 0.0F, r * a),
    @core.Vec3::new(r * a, 0.0F, -r * a),
    @core.Vec3::new(-r * a, 0.0F, r * a),
    @core.Vec3::new(-r * a, 0.0F, -r * a),
  ]
  let mut best : ContactPoint3D? = None
  let mut best_pen = -1.0F
  for i in 0..<ring.length() {
    let p0 = cyl_pos.transform_point(
      ring[i].add(@core.Vec3::new(0.0F, -h, 0.0F)),
    )
    if compute_point_trimesh_contact(p0, skin, mesh_pos, vertices, indices)
      is Some(cp) {
      if cp.penetration > best_pen {
        best_pen = cp.penetration
        best = Some(cp)
      }
    }
    let p1 = cyl_pos.transform_point(
      ring[i].add(@core.Vec3::new(0.0F, h, 0.0F)),
    )
    if compute_point_trimesh_contact(p1, skin, mesh_pos, vertices, indices)
      is Some(cp) {
      if cp.penetration > best_pen {
        best_pen = cp.penetration
        best = Some(cp)
      }
    }
    let pm = cyl_pos.transform_point(ring[i])
    if compute_point_trimesh_contact(pm, skin, mesh_pos, vertices, indices)
      is Some(cp) {
      if cp.penetration > best_pen {
        best_pen = cp.penetration
        best = Some(cp)
      }
    }
  }
  best
}

///|
pub fn compute_round_cylinder_trimesh_contact(
  cyl_pos : @core.Isometry3,
  cyl_radius : @core.Real,
  cyl_half_height : @core.Real,
  border_radius : @core.Real,
  mesh_pos : @core.Isometry3,
  vertices : Array[@core.Vec3],
  indices : Array[(Int, Int, Int)],
) -> ContactPoint3D? {
  compute_capsule_trimesh_contact(
    cyl_pos,
    cyl_radius + border_radius,
    cyl_half_height + border_radius,
    mesh_pos,
    vertices,
    indices,
  )
}

///|
pub fn compute_cone_trimesh_contact(
  cone_pos : @core.Isometry3,
  cone_radius : @core.Real,
  cone_half_height : @core.Real,
  mesh_pos : @core.Isometry3,
  vertices : Array[@core.Vec3],
  indices : Array[(Int, Int, Int)],
) -> ContactPoint3D? {
  // Approximation: sample points on the cone base rim and on a few rings along the slope.
  let skin = 0.05F
  let r = cone_radius
  let hh = cone_half_height
  let a = 0.70710677F
  let ring_dirs = [
    @core.Vec3::new(1.0F, 0.0F, 0.0F),
    @core.Vec3::new(-1.0F, 0.0F, 0.0F),
    @core.Vec3::new(0.0F, 0.0F, 1.0F),
    @core.Vec3::new(0.0F, 0.0F, -1.0F),
    @core.Vec3::new(a, 0.0F, a),
    @core.Vec3::new(a, 0.0F, -a),
    @core.Vec3::new(-a, 0.0F, a),
    @core.Vec3::new(-a, 0.0F, -a),
  ]
  let slices = [1.0F, 0.66F, 0.33F]
  let mut best : ContactPoint3D? = None
  let mut best_pen = -1.0F
  // Base rim is at y = -hh, tip is at y = +hh.
  for s in slices {
    let y = -hh + (1.0F - s) * (2.0F * hh)
    let rr = r * s
    for i in 0..<ring_dirs.length() {
      let pt_local = ring_dirs[i].scale(rr).add(@core.Vec3::new(0.0F, y, 0.0F))
      let p = cone_pos.transform_point(pt_local)
      if compute_point_trimesh_contact(p, skin, mesh_pos, vertices, indices)
        is Some(cp) {
        if cp.penetration > best_pen {
          best_pen = cp.penetration
          best = Some(cp)
        }
      }
    }
  }
  best
}

///|
pub fn compute_ball_ball_contact(
  center1 : @core.Vec3,
  radius1 : @core.Real,
  center2 : @core.Vec3,
  radius2 : @core.Real,
) -> ContactPoint3D? {
  let delta = center2.sub(center1)
  let dist2 = delta.length_squared()
  let r = radius1 + radius2
  let r2 = r * r
  if dist2 > r2 {
    return None
  }
  let dist = local_sqrt(dist2)
  let normal = if dist <= 1.0e-6F {
    // Deterministic fallback to avoid NaNs.
    @core.Vec3::new(1.0F, 0.0F, 0.0F)
  } else {
    delta.scale(1.0F / dist)
  }
  let point1 = center1.add(normal.scale(radius1))
  let point2 = center2.sub(normal.scale(radius2))
  Some({ point1, point2, normal, penetration: r - dist })
}

///|
pub fn compute_ball_cuboid_contact(
  ball_center : @core.Vec3,
  ball_radius : @core.Real,
  cuboid_pos : @core.Isometry3,
  cuboid_half_extents : @core.Vec3,
) -> ContactPoint3D? {
  let inv = cuboid_pos.inverse()
  let local_center = inv.transform_point(ball_center)
  let he = cuboid_half_extents
  let clamped = @core.Vec3::new(
    @core.clamp(local_center.x, -he.x, he.x),
    @core.clamp(local_center.y, -he.y, he.y),
    @core.clamp(local_center.z, -he.z, he.z),
  )
  let delta = local_center.sub(clamped)
  let dist2 = delta.length_squared()
  if dist2 > ball_radius * ball_radius {
    // Sphere center is outside the box and too far away.
    // Note: when the center is inside, dist2 == 0 and this check doesn't trigger.
    return None
  }
  let dist = local_sqrt(dist2)
  let normal_local = if dist > 1.0e-6F {
    // Outside: normal points from the sphere to the box (ball is shape1).
    delta.scale(-1.0F / dist)
  } else {
    // Inside (or extremely close): pick the nearest face direction.
    let dx = he.x - @core.abs(local_center.x)
    let dy = he.y - @core.abs(local_center.y)
    let dz = he.z - @core.abs(local_center.z)
    if dx <= dy && dx <= dz {
      @core.Vec3::new(
        if local_center.x >= 0.0F {
          1.0F
        } else {
          -1.0F
        },
        0.0F,
        0.0F,
      )
    } else if dy <= dz {
      @core.Vec3::new(
        0.0F,
        if local_center.y >= 0.0F {
          1.0F
        } else {
          -1.0F
        },
        0.0F,
      )
    } else {
      @core.Vec3::new(
        0.0F,
        0.0F,
        if local_center.z >= 0.0F {
          1.0F
        } else {
          -1.0F
        },
      )
    }
  }
  let normal = cuboid_pos.rotation.rotate_vec3(normal_local)
  let point2_local = if dist > 1.0e-6F {
    clamped
  } else if @core.abs(normal_local.x) > 0.0F {
    @core.Vec3::new(normal_local.x * he.x, local_center.y, local_center.z)
  } else if @core.abs(normal_local.y) > 0.0F {
    @core.Vec3::new(local_center.x, normal_local.y * he.y, local_center.z)
  } else {
    @core.Vec3::new(local_center.x, local_center.y, normal_local.z * he.z)
  }
  let point2 = cuboid_pos.transform_point(point2_local)
  let point1 = ball_center.add(normal.scale(ball_radius))
  let penetration = if dist > 1.0e-6F {
    ball_radius - dist
  } else {
    let dx = he.x - @core.abs(local_center.x)
    let dy = he.y - @core.abs(local_center.y)
    let dz = he.z - @core.abs(local_center.z)
    let face_dist = if dx <= dy && dx <= dz {
      dx
    } else if dy <= dz {
      dy
    } else {
      dz
    }
    ball_radius + face_dist
  }
  Some({ point1, point2, normal, penetration })
}

///|
fn closest_point_on_segment3d(
  p : @core.Vec3,
  a : @core.Vec3,
  b : @core.Vec3,
) -> @core.Vec3 {
  let ab = b.sub(a)
  let denom = ab.length_squared()
  if denom <= 1.0e-12F {
    return a
  }
  let t = @core.clamp(p.sub(a).dot(ab) / denom, 0.0F, 1.0F)
  a.add(ab.scale(t))
}

///|
fn capsule_segment_endpoints(
  pos : @core.Isometry3,
  half_height : @core.Real,
) -> (@core.Vec3, @core.Vec3) {
  let a = pos.transform_point(@core.Vec3::new(0.0F, -half_height, 0.0F))
  let b = pos.transform_point(@core.Vec3::new(0.0F, half_height, 0.0F))
  (a, b)
}

///|
fn axis_interval(
  p0 : @core.Real,
  d : @core.Real,
  he : @core.Real,
  state : Int,
) -> (@core.Real, @core.Real, Bool) {
  // state: 0=inside, 1=below, 2=above
  let neg_inf = -1.0e30F
  let pos_inf = 1.0e30F
  if @core.abs(d) <= 1.0e-12F {
    match state {
      0 =>
        if p0 >= -he && p0 <= he {
          (neg_inf, pos_inf, true)
        } else {
          (0.0F, 0.0F, false)
        }
      1 =>
        if p0 <= -he {
          (neg_inf, pos_inf, true)
        } else {
          (0.0F, 0.0F, false)
        }
      _ => if p0 >= he { (neg_inf, pos_inf, true) } else { (0.0F, 0.0F, false) }
    }
  } else {
    let t_neg = (-he - p0) / d
    let t_pos = (he - p0) / d
    let t_min = if t_neg < t_pos { t_neg } else { t_pos }
    let t_max = if t_neg < t_pos { t_pos } else { t_neg }
    match state {
      0 => (t_min, t_max, true)
      1 =>
        if d > 0.0F {
          (neg_inf, t_neg, true)
        } else {
          (t_neg, pos_inf, true)
        }
      _ =>
        if d > 0.0F {
          (t_pos, pos_inf, true)
        } else {
          (neg_inf, t_pos, true)
        }
    }
  }
}

///|
fn segment_aabb_closest_points(
  a : @core.Vec3,
  b : @core.Vec3,
  he : @core.Vec3,
) -> (@core.Vec3, @core.Vec3) {
  let p0 = a
  let d = b.sub(a)
  let mut best_t = 0.0F
  let mut best_dist2 = 1.0e30F
  let mut best_p = a
  let mut best_q = @core.Vec3::new(
    @core.clamp(a.x, -he.x, he.x),
    @core.clamp(a.y, -he.y, he.y),
    @core.clamp(a.z, -he.z, he.z),
  )
  for sx in 0..<3 {
    let (x_lo, x_hi, x_ok) = axis_interval(p0.x, d.x, he.x, sx)
    if !x_ok {
      continue
    }
    for sy in 0..<3 {
      let (y_lo, y_hi, y_ok) = axis_interval(p0.y, d.y, he.y, sy)
      if !y_ok {
        continue
      }
      for sz in 0..<3 {
        let (z_lo, z_hi, z_ok) = axis_interval(p0.z, d.z, he.z, sz)
        if !z_ok {
          continue
        }
        let lo = local_max(local_max(local_max(x_lo, y_lo), z_lo), 0.0F)
        let hi = local_min(local_min(local_min(x_hi, y_hi), z_hi), 1.0F)
        if lo > hi {
          continue
        }
        let mut a2 = 0.0F
        let mut b2 = 0.0F
        let mut c2 = 0.0F
        if sx != 0 {
          let bound = if sx == 1 { -he.x } else { he.x }
          let k0 = p0.x - bound
          a2 = a2 + d.x * d.x
          b2 = b2 + 2.0F * d.x * k0
          c2 = c2 + k0 * k0
        }
        if sy != 0 {
          let bound = if sy == 1 { -he.y } else { he.y }
          let k0 = p0.y - bound
          a2 = a2 + d.y * d.y
          b2 = b2 + 2.0F * d.y * k0
          c2 = c2 + k0 * k0
        }
        if sz != 0 {
          let bound = if sz == 1 { -he.z } else { he.z }
          let k0 = p0.z - bound
          a2 = a2 + d.z * d.z
          b2 = b2 + 2.0F * d.z * k0
          c2 = c2 + k0 * k0
        }
        let mut t = lo
        if a2 > 1.0e-12F {
          t = @core.clamp(-b2 / (2.0F * a2), lo, hi)
        } else if hi > lo {
          let f_lo = b2 * lo + c2
          let f_hi = b2 * hi + c2
          t = if f_hi < f_lo { hi } else { lo }
        }
        let p = p0.add(d.scale(t))
        let q = @core.Vec3::new(
          @core.clamp(p.x, -he.x, he.x),
          @core.clamp(p.y, -he.y, he.y),
          @core.clamp(p.z, -he.z, he.z),
        )
        let dist2 = p.sub(q).length_squared()
        if dist2 < best_dist2 - 1.0e-12F ||
          (@core.abs(dist2 - best_dist2) <= 1.0e-12F && t < best_t) {
          best_dist2 = dist2
          best_t = t
          best_p = p
          best_q = q
        }
      }
    }
  }
  (best_p, best_q)
}

///|
pub fn compute_ball_capsule_contact(
  ball_center : @core.Vec3,
  ball_radius : @core.Real,
  capsule_pos : @core.Isometry3,
  capsule_radius : @core.Real,
  capsule_half_height : @core.Real,
) -> ContactPoint3D? {
  let (a, b) = capsule_segment_endpoints(capsule_pos, capsule_half_height)
  let axis_pt = closest_point_on_segment3d(ball_center, a, b)
  let delta = axis_pt.sub(ball_center)
  let dist2 = delta.length_squared()
  let r = ball_radius + capsule_radius
  if dist2 > r * r {
    return None
  }
  let dist = local_sqrt(dist2)
  let normal = if dist <= 1.0e-6F {
    @core.Vec3::new(1.0F, 0.0F, 0.0F)
  } else {
    delta.scale(1.0F / dist)
  }
  let point1 = ball_center.add(normal.scale(ball_radius))
  let point2 = axis_pt.sub(normal.scale(capsule_radius))
  Some({ point1, point2, normal, penetration: r - dist })
}

///|
fn closest_points_segment_segment3d(
  p1 : @core.Vec3,
  q1 : @core.Vec3,
  p2 : @core.Vec3,
  q2 : @core.Vec3,
) -> (@core.Vec3, @core.Vec3) {
  let d1 = q1.sub(p1)
  let d2 = q2.sub(p2)
  let r = p1.sub(p2)
  let a = d1.dot(d1)
  let e = d2.dot(d2)
  let f = d2.dot(r)
  let eps = 1.0e-12F
  let mut s = 0.0F
  let mut t = 0.0F
  if a <= eps && e <= eps {
    return (p1, p2)
  }
  if a <= eps {
    s = 0.0F
    t = @core.clamp(f / e, 0.0F, 1.0F)
  } else {
    let c = d1.dot(r)
    if e <= eps {
      t = 0.0F
      s = @core.clamp(-c / a, 0.0F, 1.0F)
    } else {
      let b = d1.dot(d2)
      let denom = a * e - b * b
      if denom != 0.0F {
        s = @core.clamp((b * f - c * e) / denom, 0.0F, 1.0F)
      } else {
        s = 0.0F
      }
      t = (b * s + f) / e
      if t < 0.0F {
        t = 0.0F
        s = @core.clamp(-c / a, 0.0F, 1.0F)
      } else if t > 1.0F {
        t = 1.0F
        s = @core.clamp((b - c) / a, 0.0F, 1.0F)
      }
    }
  }
  (p1.add(d1.scale(s)), p2.add(d2.scale(t)))
}

///|
pub fn compute_capsule_capsule_contact(
  pos1 : @core.Isometry3,
  r1 : @core.Real,
  hh1 : @core.Real,
  pos2 : @core.Isometry3,
  r2 : @core.Real,
  hh2 : @core.Real,
) -> ContactPoint3D? {
  let (a1, b1) = capsule_segment_endpoints(pos1, hh1)
  let (a2, b2) = capsule_segment_endpoints(pos2, hh2)
  let (p1, p2) = closest_points_segment_segment3d(a1, b1, a2, b2)
  let delta = p2.sub(p1)
  let dist2 = delta.length_squared()
  let r = r1 + r2
  if dist2 > r * r {
    return None
  }
  let dist = local_sqrt(dist2)
  let normal = if dist <= 1.0e-6F {
    @core.Vec3::new(1.0F, 0.0F, 0.0F)
  } else {
    delta.scale(1.0F / dist)
  }
  let point1 = p1.add(normal.scale(r1))
  let point2 = p2.sub(normal.scale(r2))
  Some({ point1, point2, normal, penetration: r - dist })
}

///|
pub fn compute_capsule_cuboid_contact(
  capsule_pos : @core.Isometry3,
  capsule_radius : @core.Real,
  capsule_half_height : @core.Real,
  cuboid_pos : @core.Isometry3,
  cuboid_half_extents : @core.Vec3,
) -> ContactPoint3D? {
  let (a_world, b_world) = capsule_segment_endpoints(
    capsule_pos, capsule_half_height,
  )
  let inv = cuboid_pos.inverse()
  let a_local = inv.transform_point(a_world)
  let b_local = inv.transform_point(b_world)
  let he = cuboid_half_extents
  let (axis_local, box_local) = segment_aabb_closest_points(
    a_local, b_local, he,
  )
  let delta = box_local.sub(axis_local)
  let dist2 = delta.length_squared()
  if dist2 > capsule_radius * capsule_radius {
    return None
  }
  let dist = local_sqrt(dist2)
  let mut normal_local = @core.Vec3::zero()
  let mut point2_local = box_local
  let penetration = if dist > 1.0e-6F {
    normal_local = delta.scale(1.0F / dist)
    capsule_radius - dist
  } else {
    // Segment passes through (or extremely close to) the box: pick a deterministic face normal.
    let dx = he.x - @core.abs(axis_local.x)
    let dy = he.y - @core.abs(axis_local.y)
    let dz = he.z - @core.abs(axis_local.z)
    if dx <= dy && dx <= dz {
      normal_local = @core.Vec3::new(
        if axis_local.x >= 0.0F {
          1.0F
        } else {
          -1.0F
        },
        0.0F,
        0.0F,
      )
      point2_local = @core.Vec3::new(
        normal_local.x * he.x,
        axis_local.y,
        axis_local.z,
      )
      capsule_radius + dx
    } else if dy <= dz {
      normal_local = @core.Vec3::new(
        0.0F,
        if axis_local.y >= 0.0F {
          1.0F
        } else {
          -1.0F
        },
        0.0F,
      )
      point2_local = @core.Vec3::new(
        axis_local.x,
        normal_local.y * he.y,
        axis_local.z,
      )
      capsule_radius + dy
    } else {
      normal_local = @core.Vec3::new(
        0.0F,
        0.0F,
        if axis_local.z >= 0.0F {
          1.0F
        } else {
          -1.0F
        },
      )
      point2_local = @core.Vec3::new(
        axis_local.x,
        axis_local.y,
        normal_local.z * he.z,
      )
      capsule_radius + dz
    }
  }
  let normal = cuboid_pos.rotation.rotate_vec3(normal_local)
  let axis_world = cuboid_pos.transform_point(axis_local)
  let point2 = cuboid_pos.transform_point(point2_local)
  let point1 = axis_world.add(normal.scale(capsule_radius))
  Some({ point1, point2, normal, penetration })
}

///|
pub fn compute_ball_cylinder_contact(
  ball_center : @core.Vec3,
  ball_radius : @core.Real,
  cyl_pos : @core.Isometry3,
  cyl_radius : @core.Real,
  cyl_half_height : @core.Real,
) -> ContactPoint3D? {
  let inv = cyl_pos.inverse()
  let local_center = inv.transform_point(ball_center)
  let hh = cyl_half_height
  let r = cyl_radius
  let y = @core.clamp(local_center.y, -hh, hh)
  let mut x = local_center.x
  let mut z = local_center.z
  let s2 = x * x + z * z
  if s2 > 1.0e-12F {
    let s = local_sqrt(s2)
    if s > r {
      let k = r / s
      x = x * k
      z = z * k
    }
  } else {
    x = 0.0F
    z = 0.0F
  }
  let point2_local = @core.Vec3::new(x, y, z)
  let delta = local_center.sub(point2_local)
  let dist2 = delta.length_squared()
  if dist2 > ball_radius * ball_radius {
    return None
  }
  let dist = local_sqrt(dist2)
  let normal_local = if dist > 1.0e-6F {
    delta.scale(-1.0F / dist)
  } else {
    // Inside: choose the nearest surface.
    let radial = local_sqrt(s2)
    let dr = r - radial
    let dy = hh - @core.abs(local_center.y)
    if dy <= dr {
      @core.Vec3::new(
        0.0F,
        if local_center.y >= 0.0F {
          1.0F
        } else {
          -1.0F
        },
        0.0F,
      )
    } else if radial > 1.0e-6F {
      @core.Vec3::new(local_center.x / radial, 0.0F, local_center.z / radial)
    } else {
      @core.Vec3::new(1.0F, 0.0F, 0.0F)
    }
  }
  let normal = cyl_pos.rotation.rotate_vec3(normal_local)
  let point2 = cyl_pos.transform_point(point2_local)
  let point1 = ball_center.add(normal.scale(ball_radius))
  let penetration = if dist > 1.0e-6F {
    ball_radius - dist
  } else {
    let radial = local_sqrt(s2)
    let dr = r - radial
    let dy = hh - @core.abs(local_center.y)
    let min_dist = if dy <= dr { dy } else { dr }
    ball_radius + min_dist
  }
  Some({ point1, point2, normal, penetration })
}

///|
pub fn compute_ball_cone_contact(
  ball_center : @core.Vec3,
  ball_radius : @core.Real,
  cone_pos : @core.Isometry3,
  cone_radius : @core.Real,
  cone_half_height : @core.Real,
) -> ContactPoint3D? {
  let inv = cone_pos.inverse()
  let local_center = inv.transform_point(ball_center)
  let hh = cone_half_height
  let r = cone_radius
  let s2 = local_center.x * local_center.x + local_center.z * local_center.z
  let s = local_sqrt(s2)
  // Reduce to 2D in (s, y).
  let ax = 0.0F
  let ay = hh
  let bx = r
  let by = -hh
  let px = s
  let py = local_center.y
  let abx = bx - ax
  let aby = by - ay
  let denom = abx * abx + aby * aby
  let mut t = 0.0F
  if denom > 1.0e-12F {
    t = @core.clamp(((px - ax) * abx + (py - ay) * aby) / denom, 0.0F, 1.0F)
  }
  let qx = ax + abx * t
  let qy = ay + aby * t
  let base_x = if px > r { r } else { px }
  let base_y = -hh
  let dist2_lateral = (px - qx) * (px - qx) + (py - qy) * (py - qy)
  let dist2_base = (px - base_x) * (px - base_x) + (py - base_y) * (py - base_y)
  let use_base = dist2_base < dist2_lateral
  let target_x = if use_base { base_x } else { qx }
  let target_y = if use_base { base_y } else { qy }
  // Map back to 3D.
  let (tx, tz) = if s > 1.0e-6F {
    let k = target_x / s
    (local_center.x * k, local_center.z * k)
  } else {
    (target_x, 0.0F)
  }
  let point2_local = @core.Vec3::new(tx, target_y, tz)
  let delta = local_center.sub(point2_local)
  let dist2 = delta.length_squared()
  if dist2 > ball_radius * ball_radius {
    return None
  }
  let dist = local_sqrt(dist2)
  let normal_local = if dist > 1.0e-6F {
    delta.scale(-1.0F / dist)
  } else {
    // Inside: nearest between base and lateral surface (approx).
    let dist_base = local_center.y + hh
    let slope = r / (2.0F * hh)
    let radius_y = (hh - local_center.y) * slope
    let dist_side = radius_y - s
    if dist_base <= dist_side {
      @core.Vec3::new(0.0F, -1.0F, 0.0F)
    } else if s > 1.0e-6F {
      @core.Vec3::new(local_center.x / s, slope, local_center.z / s).normalize()
    } else {
      @core.Vec3::new(1.0F, 0.0F, 0.0F)
    }
  }
  let normal = cone_pos.rotation.rotate_vec3(normal_local)
  let point2 = cone_pos.transform_point(point2_local)
  let point1 = ball_center.add(normal.scale(ball_radius))
  let penetration = if dist > 1.0e-6F {
    ball_radius - dist
  } else {
    let dist_base = local_center.y + hh
    let slope = r / (2.0F * hh)
    let radius_y = (hh - local_center.y) * slope
    let dist_side = radius_y - s
    let min_dist = if dist_base <= dist_side { dist_base } else { dist_side }
    ball_radius + min_dist
  }
  Some({ point1, point2, normal, penetration })
}

///|
pub fn compute_cuboid_cuboid_contact(
  pos1 : @core.Isometry3,
  he1 : @core.Vec3,
  pos2 : @core.Isometry3,
  he2 : @core.Vec3,
) -> ContactPoint3D? {
  let a = [he1.x, he1.y, he1.z]
  let b = [he2.x, he2.y, he2.z]
  let ra = pos1.rotation.to_mat3()
  let rb = pos2.rotation.to_mat3()
  fn col(m : @core.Mat3, i : Int) -> @core.Vec3 {
    match i {
      0 => @core.Vec3::new(m.m00, m.m10, m.m20)
      1 => @core.Vec3::new(m.m01, m.m11, m.m21)
      _ => @core.Vec3::new(m.m02, m.m12, m.m22)
    }
  }

  let a_axis = [col(ra, 0), col(ra, 1), col(ra, 2)]
  let b_axis = [col(rb, 0), col(rb, 1), col(rb, 2)]
  let d = pos2.translation.sub(pos1.translation)
  let t = [d.dot(a_axis[0]), d.dot(a_axis[1]), d.dot(a_axis[2])]

  // R = A^T B (in A's frame).
  let r : Array[@core.Real] = []
  let abs_r : Array[@core.Real] = []
  let eps = 1.0e-6F
  for i in 0..<3 {
    for j in 0..<3 {
      let rij = a_axis[i].dot(b_axis[j])
      r.push(rij)
      abs_r.push(@core.abs(rij) + eps)
    }
  }
  fn r_at(r : Array[@core.Real], i : Int, j : Int) -> @core.Real {
    r[i * 3 + j]
  }

  fn abs_r_at(r : Array[@core.Real], i : Int, j : Int) -> @core.Real {
    r[i * 3 + j]
  }

  // Track the best (minimum) penetration axis.
  let mut best = CuboidBestAxis::A(0)
  let mut min_overlap = 1.0e30F

  // Test L = A_i.
  for i in 0..<3 {
    let ra_i = a[i]
    let rb_i = b[0] * abs_r_at(abs_r, i, 0) +
      b[1] * abs_r_at(abs_r, i, 1) +
      b[2] * abs_r_at(abs_r, i, 2)
    let dist = @core.abs(t[i])
    let overlap = ra_i + rb_i - dist
    if overlap < 0.0F {
      return None
    }
    if overlap < min_overlap {
      min_overlap = overlap
      best = CuboidBestAxis::A(i)
    }
  }

  // Test L = B_j.
  for j in 0..<3 {
    let ra_j = a[0] * abs_r_at(abs_r, 0, j) +
      a[1] * abs_r_at(abs_r, 1, j) +
      a[2] * abs_r_at(abs_r, 2, j)
    let rb_j = b[j]
    let proj = t[0] * r_at(r, 0, j) +
      t[1] * r_at(r, 1, j) +
      t[2] * r_at(r, 2, j)
    let dist = @core.abs(proj)
    let overlap = ra_j + rb_j - dist
    if overlap < 0.0F {
      return None
    }
    if overlap < min_overlap {
      min_overlap = overlap
      best = CuboidBestAxis::B(j)
    }
  }

  // Test L = A_i x B_j.
  for i in 0..<3 {
    let i1 = (i + 1) % 3
    let i2 = (i + 2) % 3
    for j in 0..<3 {
      let j1 = (j + 1) % 3
      let j2 = (j + 2) % 3
      let ra_c = a[i1] * abs_r_at(abs_r, i2, j) + a[i2] * abs_r_at(abs_r, i1, j)
      let rb_c = b[j1] * abs_r_at(abs_r, i, j2) + b[j2] * abs_r_at(abs_r, i, j1)
      let dist = @core.abs(t[i2] * r_at(r, i1, j) - t[i1] * r_at(r, i2, j))
      let overlap = ra_c + rb_c - dist
      if overlap < 0.0F {
        return None
      }
      // Skip near-parallel axes when choosing the best axis.
      let axis = a_axis[i].cross(b_axis[j])
      if axis.length_squared() > 1.0e-12F {
        if overlap < min_overlap {
          min_overlap = overlap
          best = CuboidBestAxis::Cross(i, j)
        }
      }
    }
  }

  // Contact normal pointing from box1 to box2.
  let mut normal = match best {
    CuboidBestAxis::A(i) => {
      let s = if t[i] < 0.0F { -1.0F } else { 1.0F }
      a_axis[i].scale(s)
    }
    CuboidBestAxis::B(j) => {
      let proj = t[0] * r_at(r, 0, j) +
        t[1] * r_at(r, 1, j) +
        t[2] * r_at(r, 2, j)
      let s = if proj < 0.0F { -1.0F } else { 1.0F }
      b_axis[j].scale(s)
    }
    CuboidBestAxis::Cross(i, j) => {
      let axis = a_axis[i].cross(b_axis[j])
      let len = local_sqrt(axis.length_squared())
      if len <= 1.0e-6F {
        @core.Vec3::new(1.0F, 0.0F, 0.0F)
      } else {
        axis.scale(1.0F / len)
      }
    }
  }
  if normal.dot(d) < 0.0F {
    normal = normal.scale(-1.0F)
  }
  fn support(
    pos : @core.Isometry3,
    he : @core.Vec3,
    dir : @core.Vec3,
  ) -> @core.Vec3 {
    let local_dir = pos.rotation.inverse().rotate_vec3(dir)
    let sx = if local_dir.x >= 0.0F { he.x } else { -he.x }
    let sy = if local_dir.y >= 0.0F { he.y } else { -he.y }
    let sz = if local_dir.z >= 0.0F { he.z } else { -he.z }
    pos.transform_point(@core.Vec3::new(sx, sy, sz))
  }

  let point1 = support(pos1, he1, normal)
  let point2 = support(pos2, he2, normal.scale(-1.0F))
  Some({ point1, point2, normal, penetration: min_overlap })
}
