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
  let mut best_normal_sum = @core.Vec3::zero()
  let mut best_count = 0
  let eps_d2 = 1.0e-6F
  let eps_q2 = 1.0e-8F
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
    let n = b.sub(a).cross(c.sub(a))
    let n_norm = if n.length_squared() > 1.0e-12F {
      n.normalize()
    } else {
      @core.Vec3::new(0.0F, 1.0F, 0.0F)
    }
    if d2 + eps_d2 < best_dist2 {
      best_dist2 = d2
      best_point = q
      best_normal_sum = n_norm
      best_count = 1
    } else if @core.abs(d2 - best_dist2) <= eps_d2 &&
      q.sub(best_point).length_squared() <= eps_q2 {
      // FIX_INTERNAL_EDGES-style smoothing: if multiple triangles are equally close at the same
      // projection point (common for coplanar splits), average their normals.
      best_normal_sum = best_normal_sum.add(n_norm)
      best_count = best_count + 1
    }
  }
  if best_count > 0 && best_normal_sum.length_squared() > 1.0e-12F {
    best_normal = best_normal_sum.normalize()
  }
  (best_point, best_normal, best_dist2, true)
}

///|
/// Heightfield-specific closest-point query.
///
/// Unlike `trimesh_closest_point`, this exploits the regular grid structure:
/// it performs a best-first search over the heightfield cells, using each cell's AABB
/// as a lower-bound. This keeps performance reasonable for large heightfields while
/// still returning the exact closest point.
fn heightfield_closest_point(
  p : @core.Vec3,
  mesh_pos : @core.Isometry3,
  vertices : Array[@core.Vec3],
  nrows : Int,
  ncols : Int,
  flags : Int,
) -> (@core.Vec3, @core.Vec3, @core.Real, Bool) {
  if nrows <= 1 || ncols <= 1 || vertices.length() != nrows * ncols {
    return (@core.Vec3::zero(), @core.Vec3::new(0.0F, 1.0F, 0.0F), 0.0F, false)
  }
  // Work in the heightfield local-space to avoid per-triangle transforms.
  let inv = mesh_pos.inverse()
  let p_local = inv.transform_point(p)
  let origin_x = vertices[0].x
  let origin_z = vertices[0].z
  let dx0 = if nrows >= 2 { vertices[ncols].x - origin_x } else { 1.0F }
  let dz0 = if ncols >= 2 { vertices[1].z - origin_z } else { 1.0F }
  let dx = if dx0 < 0.0F { -dx0 } else { dx0 }
  let dz = if dz0 < 0.0F { -dz0 } else { dz0 }
  if dx <= 1.0e-12F || dz <= 1.0e-12F {
    return (@core.Vec3::zero(), @core.Vec3::new(0.0F, 1.0F, 0.0F), 0.0F, false)
  }
  let cells_x = nrows - 1
  let cells_z = ncols - 1
  fn clamp_int(v : Int, lo : Int, hi : Int) -> Int {
    if v < lo {
      lo
    } else if v > hi {
      hi
    } else {
      v
    }
  }

  fn floor_to_int(x : @core.Real) -> Int {
    // Matches Rust's `floor()` behavior for negative values.
    let i = x.to_int()
    if Float::from_int(i) > x {
      i - 1
    } else {
      i
    }
  }

  let fx = (p_local.x - origin_x) / dx
  let fz = (p_local.z - origin_z) / dz
  let start_i = clamp_int(floor_to_int(fx), 0, cells_x - 1)
  let start_j = clamp_int(floor_to_int(fz), 0, cells_z - 1)
  fn cell_id(i : Int, j : Int, cells_z : Int) -> Int {
    i * cells_z + j
  }

  fn point_aabb_dist2(
    p : @core.Vec3,
    min : @core.Vec3,
    max : @core.Vec3,
  ) -> @core.Real {
    let dx = if p.x < min.x {
      min.x - p.x
    } else if p.x > max.x {
      p.x - max.x
    } else {
      0.0F
    }
    let dy = if p.y < min.y {
      min.y - p.y
    } else if p.y > max.y {
      p.y - max.y
    } else {
      0.0F
    }
    let dz = if p.z < min.z {
      min.z - p.z
    } else if p.z > max.z {
      p.z - max.z
    } else {
      0.0F
    }
    dx * dx + dy * dy + dz * dz
  }

  fn cell_aabb(
    vertices : Array[@core.Vec3],
    ncols : Int,
    origin_x : @core.Real,
    origin_z : @core.Real,
    dx : @core.Real,
    dz : @core.Real,
    i : Int,
    j : Int,
  ) -> (@core.Vec3, @core.Vec3) {
    let x0 = origin_x + Float::from_int(i) * dx
    let x1 = origin_x + Float::from_int(i + 1) * dx
    let z0 = origin_z + Float::from_int(j) * dz
    let z1 = origin_z + Float::from_int(j + 1) * dz
    let minx = if x0 < x1 { x0 } else { x1 }
    let maxx = if x0 > x1 { x0 } else { x1 }
    let minz = if z0 < z1 { z0 } else { z1 }
    let maxz = if z0 > z1 { z0 } else { z1 }
    let a = vertices[i * ncols + j]
    let b = vertices[(i + 1) * ncols + j]
    let c = vertices[(i + 1) * ncols + (j + 1)]
    let d = vertices[i * ncols + (j + 1)]
    let mut miny = a.y
    let mut maxy = a.y
    for v in [b, c, d] {
      if v.y < miny {
        miny = v.y
      }
      if v.y > maxy {
        maxy = v.y
      }
    }
    (@core.Vec3::new(minx, miny, minz), @core.Vec3::new(maxx, maxy, maxz))
  }

  // Min-heap of (lower_bound_dist2, i, j).
  let heap : Array[(@core.Real, Int, Int)] = []
  fn heap_push(
    heap : Array[(@core.Real, Int, Int)],
    item : (@core.Real, Int, Int),
  ) {
    heap.push(item)
    let mut k = heap.length() - 1
    while k > 0 {
      let p = (k - 1) / 2
      if heap[p].0 <= heap[k].0 {
        break
      }
      let tmp = heap[p]
      heap[p] = heap[k]
      heap[k] = tmp
      k = p
    }
  }

  fn heap_pop_min(
    heap : Array[(@core.Real, Int, Int)],
  ) -> (@core.Real, Int, Int)? {
    if heap.length() == 0 {
      return None
    }
    let out = heap[0]
    let last_opt = heap.pop()
    if heap.length() == 0 {
      return Some(out)
    }
    if last_opt is Some(last) {
      heap[0] = last
    } else {
      return Some(out)
    }
    let mut k = 0
    while true {
      let l = 2 * k + 1
      if l >= heap.length() {
        break
      }
      let r = l + 1
      let mut m = l
      if r < heap.length() && heap[r].0 < heap[l].0 {
        m = r
      }
      if heap[k].0 <= heap[m].0 {
        break
      }
      let tmp = heap[k]
      heap[k] = heap[m]
      heap[m] = tmp
      k = m
    }
    Some(out)
  }

  let visited : Array[Bool] = []
  for _ in 0..<(cells_x * cells_z) {
    visited.push(false)
  }
  let (bbmin0, bbmax0) = cell_aabb(
    vertices, ncols, origin_x, origin_z, dx, dz, start_i, start_j,
  )
  heap_push(heap, (point_aabb_dist2(p_local, bbmin0, bbmax0), start_i, start_j))
  let mut best_dist2 = 1.0e30F
  let mut best_point_local = @core.Vec3::zero()
  let mut best_normal_sum = @core.Vec3::zero()
  let mut best_count = 0
  let fix_internal_edges = (flags & 1) != 0
  let eps_d2 = 1.0e-6F
  let eps_q2 = 1.0e-8F
  while heap_pop_min(heap) is Some((lb, i, j)) {
    if lb > best_dist2 {
      break
    }
    let id = cell_id(i, j, cells_z)
    if visited[id] {
      continue
    }
    visited[id] = true

    // Test the two triangles of this cell in local-space.
    let a = vertices[i * ncols + j]
    let b = vertices[(i + 1) * ncols + j]
    let c = vertices[(i + 1) * ncols + (j + 1)]
    let d = vertices[i * ncols + (j + 1)]
    fn update_best_for_triangle(
      p_local : @core.Vec3,
      a : @core.Vec3,
      b : @core.Vec3,
      c : @core.Vec3,
      best_dist2 : @core.Real,
      best_point : @core.Vec3,
      best_normal_sum : @core.Vec3,
      best_count : Int,
      fix_internal_edges : Bool,
      eps_d2 : @core.Real,
      eps_q2 : @core.Real,
    ) -> (@core.Real, @core.Vec3, @core.Vec3, Int) {
      let q = closest_point_on_triangle3d(p_local, a, b, c)
      let d2 = q.sub(p_local).length_squared()
      let n = b.sub(a).cross(c.sub(a))
      let n_norm = if n.length_squared() > 1.0e-12F {
        n.normalize()
      } else {
        @core.Vec3::new(0.0F, 1.0F, 0.0F)
      }
      if d2 + eps_d2 < best_dist2 {
        (d2, q, n_norm, 1)
      } else if @core.abs(d2 - best_dist2) <= eps_d2 &&
        q.sub(best_point).length_squared() <= eps_q2 {
        if fix_internal_edges {
          (best_dist2, best_point, best_normal_sum.add(n_norm), best_count + 1)
        } else {
          (best_dist2, best_point, best_normal_sum, best_count)
        }
      } else {
        (best_dist2, best_point, best_normal_sum, best_count)
      }
    }

    let (d2_0, q0, ns0, c0) = update_best_for_triangle(
      p_local, a, c, b, best_dist2, best_point_local, best_normal_sum, best_count,
      fix_internal_edges, eps_d2, eps_q2,
    )
    best_dist2 = d2_0
    best_point_local = q0
    best_normal_sum = ns0
    best_count = c0
    let (d2_1, q1, ns1, c1) = update_best_for_triangle(
      p_local, a, d, c, best_dist2, best_point_local, best_normal_sum, best_count,
      fix_internal_edges, eps_d2, eps_q2,
    )
    best_dist2 = d2_1
    best_point_local = q1
    best_normal_sum = ns1
    best_count = c1

    // Enqueue neighbors (4-neighborhood).
    if i > 0 {
      let ni = i - 1
      let nj = j
      let (bbmin, bbmax) = cell_aabb(
        vertices, ncols, origin_x, origin_z, dx, dz, ni, nj,
      )
      heap_push(heap, (point_aabb_dist2(p_local, bbmin, bbmax), ni, nj))
    }
    if i + 1 < cells_x {
      let ni = i + 1
      let nj = j
      let (bbmin, bbmax) = cell_aabb(
        vertices, ncols, origin_x, origin_z, dx, dz, ni, nj,
      )
      heap_push(heap, (point_aabb_dist2(p_local, bbmin, bbmax), ni, nj))
    }
    if j > 0 {
      let ni = i
      let nj = j - 1
      let (bbmin, bbmax) = cell_aabb(
        vertices, ncols, origin_x, origin_z, dx, dz, ni, nj,
      )
      heap_push(heap, (point_aabb_dist2(p_local, bbmin, bbmax), ni, nj))
    }
    if j + 1 < cells_z {
      let ni = i
      let nj = j + 1
      let (bbmin, bbmax) = cell_aabb(
        vertices, ncols, origin_x, origin_z, dx, dz, ni, nj,
      )
      heap_push(heap, (point_aabb_dist2(p_local, bbmin, bbmax), ni, nj))
    }
  }
  if best_count == 0 {
    return (@core.Vec3::zero(), @core.Vec3::new(0.0F, 1.0F, 0.0F), 0.0F, false)
  }
  let best_normal_local = if best_normal_sum.length_squared() > 1.0e-12F {
    best_normal_sum.normalize()
  } else {
    @core.Vec3::new(0.0F, 1.0F, 0.0F)
  }
  let best_point = mesh_pos.transform_point(best_point_local)
  let best_normal = mesh_pos.rotation.rotate_vec3(best_normal_local)
  (best_point, best_normal, best_dist2, true)
}

///|
/// Fast heightfield closest-point query for cases where we only care about hits within
/// a known maximum distance (typical for contact generation). This avoids allocating
/// a visited map / priority queue and instead scans only the cells whose xz-range
/// could contain a closest point within `max_dist`.
fn heightfield_closest_point_bounded(
  p : @core.Vec3,
  mesh_pos : @core.Isometry3,
  vertices : Array[@core.Vec3],
  nrows : Int,
  ncols : Int,
  flags : Int,
  max_dist : @core.Real,
) -> (@core.Vec3, @core.Vec3, @core.Real, Bool) {
  if nrows <= 1 || ncols <= 1 || vertices.length() != nrows * ncols {
    return (@core.Vec3::zero(), @core.Vec3::new(0.0F, 1.0F, 0.0F), 0.0F, false)
  }
  let inv = mesh_pos.inverse()
  let p_local = inv.transform_point(p)
  let origin_x = vertices[0].x
  let origin_z = vertices[0].z
  let dx0 = vertices[ncols].x - origin_x
  let dz0 = vertices[1].z - origin_z
  let dx = if dx0 < 0.0F { -dx0 } else { dx0 }
  let dz = if dz0 < 0.0F { -dz0 } else { dz0 }
  if dx <= 1.0e-12F || dz <= 1.0e-12F {
    return (@core.Vec3::zero(), @core.Vec3::new(0.0F, 1.0F, 0.0F), 0.0F, false)
  }
  let cells_x = nrows - 1
  let cells_z = ncols - 1
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

  // Any point within `max_dist` implies |dx| <= max_dist and |dz| <= max_dist.
  let r = if max_dist < 0.0F { 0.0F } else { max_dist }
  let min_i = clamp_int(
    floor_to_int((p_local.x - r - origin_x) / dx),
    0,
    cells_x - 1,
  )
  let max_i = clamp_int(
    floor_to_int((p_local.x + r - origin_x) / dx),
    0,
    cells_x - 1,
  )
  let min_j = clamp_int(
    floor_to_int((p_local.z - r - origin_z) / dz),
    0,
    cells_z - 1,
  )
  let max_j = clamp_int(
    floor_to_int((p_local.z + r - origin_z) / dz),
    0,
    cells_z - 1,
  )
  let mut best_dist2 = 1.0e30F
  let mut best_point_local = @core.Vec3::zero()
  let mut best_normal_sum = @core.Vec3::zero()
  let mut best_count = 0
  let fix_internal_edges = (flags & 1) != 0
  let eps_d2 = 1.0e-6F
  let eps_q2 = 1.0e-8F
  for i in min_i..<(max_i + 1) {
    for j in min_j..<(max_j + 1) {
      let a = vertices[i * ncols + j]
      let b = vertices[(i + 1) * ncols + j]
      let c = vertices[(i + 1) * ncols + (j + 1)]
      let d = vertices[i * ncols + (j + 1)]
      fn update_best_for_triangle(
        p_local : @core.Vec3,
        a : @core.Vec3,
        b : @core.Vec3,
        c : @core.Vec3,
        best_dist2 : @core.Real,
        best_point : @core.Vec3,
        best_normal_sum : @core.Vec3,
        best_count : Int,
        fix_internal_edges : Bool,
        eps_d2 : @core.Real,
        eps_q2 : @core.Real,
      ) -> (@core.Real, @core.Vec3, @core.Vec3, Int) {
        let q = closest_point_on_triangle3d(p_local, a, b, c)
        let d2 = q.sub(p_local).length_squared()
        let n = b.sub(a).cross(c.sub(a))
        let n_norm = if n.length_squared() > 1.0e-12F {
          n.normalize()
        } else {
          @core.Vec3::new(0.0F, 1.0F, 0.0F)
        }
        if d2 + eps_d2 < best_dist2 {
          (d2, q, n_norm, 1)
        } else if @core.abs(d2 - best_dist2) <= eps_d2 &&
          q.sub(best_point).length_squared() <= eps_q2 {
          if fix_internal_edges {
            (
              best_dist2,
              best_point,
              best_normal_sum.add(n_norm),
              best_count + 1,
            )
          } else {
            (best_dist2, best_point, best_normal_sum, best_count)
          }
        } else {
          (best_dist2, best_point, best_normal_sum, best_count)
        }
      }

      let (d2_0, q0, ns0, c0) = update_best_for_triangle(
        p_local, a, c, b, best_dist2, best_point_local, best_normal_sum, best_count,
        fix_internal_edges, eps_d2, eps_q2,
      )
      best_dist2 = d2_0
      best_point_local = q0
      best_normal_sum = ns0
      best_count = c0
      let (d2_1, q1, ns1, c1) = update_best_for_triangle(
        p_local, a, d, c, best_dist2, best_point_local, best_normal_sum, best_count,
        fix_internal_edges, eps_d2, eps_q2,
      )
      best_dist2 = d2_1
      best_point_local = q1
      best_normal_sum = ns1
      best_count = c1
    }
  }
  if best_count == 0 {
    return (@core.Vec3::zero(), @core.Vec3::new(0.0F, 1.0F, 0.0F), 0.0F, false)
  }
  let best_normal_local = if best_normal_sum.length_squared() > 1.0e-12F {
    best_normal_sum.normalize()
  } else {
    @core.Vec3::new(0.0F, 1.0F, 0.0F)
  }
  (
    mesh_pos.transform_point(best_point_local),
    mesh_pos.rotation.rotate_vec3(best_normal_local),
    best_dist2,
    true,
  )
}

///|
/// Returns the closest point on a triangle mesh to `p` (in world-space), along with the triangle
/// normal (world-space) and the squared distance.
///
/// This is a building block for query pipeline operations (point projection, swept-sphere
/// conservative advancement against meshes).
pub fn trimesh_closest_point_query(
  p : @core.Vec3,
  mesh_pos : @core.Isometry3,
  vertices : Array[@core.Vec3],
  indices : Array[(Int, Int, Int)],
) -> (@core.Vec3, @core.Vec3, @core.Real)? {
  let (best_point, best_normal, best_dist2, ok) = trimesh_closest_point(
    p, mesh_pos, vertices, indices,
  )
  if ok {
    Some((best_point, best_normal, best_dist2))
  } else {
    None
  }
}

///|
/// Heightfield variant of `trimesh_closest_point_query`.
pub fn heightfield_closest_point_query(
  p : @core.Vec3,
  mesh_pos : @core.Isometry3,
  vertices : Array[@core.Vec3],
  nrows : Int,
  ncols : Int,
) -> (@core.Vec3, @core.Vec3, @core.Real)? {
  let (best_point, best_normal, best_dist2, ok) = heightfield_closest_point(
    p, mesh_pos, vertices, nrows, ncols, 0,
  )
  if ok {
    Some((best_point, best_normal, best_dist2))
  } else {
    None
  }
}

///|
/// Heightfield variant of `trimesh_closest_point_query` with explicit HeightFieldFlags bits.
pub fn heightfield_closest_point_query_with_flags(
  p : @core.Vec3,
  mesh_pos : @core.Isometry3,
  vertices : Array[@core.Vec3],
  nrows : Int,
  ncols : Int,
  flags : Int,
) -> (@core.Vec3, @core.Vec3, @core.Real)? {
  let (best_point, best_normal, best_dist2, ok) = heightfield_closest_point(
    p, mesh_pos, vertices, nrows, ncols, flags,
  )
  if ok {
    Some((best_point, best_normal, best_dist2))
  } else {
    None
  }
}

///|
fn voxels_floor_div_to_int(value : @core.Real, denom : @core.Real) -> Int {
  if denom == 0.0F {
    0
  } else {
    @math.floor((value / denom).to_double()).to_int()
  }
}

///|
fn voxels_scan_range_local(
  voxels : Voxels3DReal,
  center_local : @core.Vec3,
  radius : @core.Real,
) -> (Int, Int, Int, Int, Int, Int) {
  let sx = voxels.voxel_size.x
  let sy = voxels.voxel_size.y
  let sz = voxels.voxel_size.z
  let x0 = center_local.x - radius
  let x1 = center_local.x + radius
  let y0 = center_local.y - radius
  let y1 = center_local.y + radius
  let z0 = center_local.z - radius
  let z1 = center_local.z + radius
  (
    voxels_floor_div_to_int(x0, sx),
    voxels_floor_div_to_int(x1, sx),
    voxels_floor_div_to_int(y0, sy),
    voxels_floor_div_to_int(y1, sy),
    voxels_floor_div_to_int(z0, sz),
    voxels_floor_div_to_int(z1, sz),
  )
}

///|
fn voxel_cell_aabb_local(
  voxels : Voxels3DReal,
  key : (Int, Int, Int),
) -> (@core.Vec3, @core.Vec3) {
  let sx = voxels.voxel_size.x
  let sy = voxels.voxel_size.y
  let sz = voxels.voxel_size.z
  let mins = @core.Vec3::new(
    Float::from_int(key.0) * sx,
    Float::from_int(key.1) * sy,
    Float::from_int(key.2) * sz,
  )
  let maxs = mins.add(@core.Vec3::new(sx, sy, sz))
  (mins, maxs)
}

///|
fn compute_sphere_voxel_contact_local(
  center_world : @core.Vec3,
  center_local : @core.Vec3,
  radius : @core.Real,
  voxels_pos : @core.Isometry3,
  voxels : Voxels3DReal,
  key : (Int, Int, Int),
) -> ContactPoint3D? {
  let (mins, maxs) = voxel_cell_aabb_local(voxels, key)
  let clamped = @core.Vec3::new(
    @core.clamp(center_local.x, mins.x, maxs.x),
    @core.clamp(center_local.y, mins.y, maxs.y),
    @core.clamp(center_local.z, mins.z, maxs.z),
  )
  let delta = center_local.sub(clamped)
  let dist2 = delta.length_squared()
  let r2 = radius * radius
  if dist2 > r2 {
    return None
  }
  let dist = local_sqrt(dist2)
  let (normal_local, point2_local, penetration) = if dist > 1.0e-6F {
    (delta.scale(-1.0F / dist), clamped, radius - dist)
  } else {
    let dx0 = center_local.x - mins.x
    let dx1 = maxs.x - center_local.x
    let dy0 = center_local.y - mins.y
    let dy1 = maxs.y - center_local.y
    let dz0 = center_local.z - mins.z
    let dz1 = maxs.z - center_local.z
    let mut best_d = dx0
    let mut normal = @core.Vec3::new(-1.0F, 0.0F, 0.0F)
    let mut point2 = @core.Vec3::new(mins.x, center_local.y, center_local.z)
    if dx1 < best_d {
      best_d = dx1
      normal = @core.Vec3::new(1.0F, 0.0F, 0.0F)
      point2 = @core.Vec3::new(maxs.x, center_local.y, center_local.z)
    }
    if dy0 < best_d {
      best_d = dy0
      normal = @core.Vec3::new(0.0F, -1.0F, 0.0F)
      point2 = @core.Vec3::new(center_local.x, mins.y, center_local.z)
    }
    if dy1 < best_d {
      best_d = dy1
      normal = @core.Vec3::new(0.0F, 1.0F, 0.0F)
      point2 = @core.Vec3::new(center_local.x, maxs.y, center_local.z)
    }
    if dz0 < best_d {
      best_d = dz0
      normal = @core.Vec3::new(0.0F, 0.0F, -1.0F)
      point2 = @core.Vec3::new(center_local.x, center_local.y, mins.z)
    }
    if dz1 < best_d {
      best_d = dz1
      normal = @core.Vec3::new(0.0F, 0.0F, 1.0F)
      point2 = @core.Vec3::new(center_local.x, center_local.y, maxs.z)
    }
    (normal, point2, radius + best_d)
  }
  let normal_world = voxels_pos.rotation.rotate_vec3(normal_local)
  let point2_world = voxels_pos.transform_point(point2_local)
  let point1_world = center_world.add(normal_world.scale(radius))
  Some({
    point1: point1_world,
    point2: point2_world,
    normal: normal_world,
    penetration,
  })
}

///|
fn compute_sphere_voxels_contact(
  center_world : @core.Vec3,
  center_local : @core.Vec3,
  radius : @core.Real,
  voxels_pos : @core.Isometry3,
  voxels : Voxels3DReal,
) -> ContactPoint3D? {
  let (ix0, ix1, iy0, iy1, iz0, iz1) = voxels_scan_range_local(
    voxels, center_local, radius,
  )
  let mut best : ContactPoint3D? = None
  let mut best_pen = -1.0F
  for ix in ix0..<(ix1 + 1) {
    for iz in iz0..<(iz1 + 1) {
      if voxels.column_ys(ix, iz) is Some(ys) {
        let mut start = 0
        while start < ys.length() && ys[start] < iy0 {
          start = start + 1
        }
        for i in start..<ys.length() {
          let iy = ys[i]
          if iy > iy1 {
            break
          }
          let key = (ix, iy, iz)
          if compute_sphere_voxel_contact_local(
              center_world, center_local, radius, voxels_pos, voxels, key,
            )
            is Some(cp) {
            if cp.penetration > best_pen {
              best_pen = cp.penetration
              best = Some(cp)
            }
          }
        }
      }
    }
  }
  best
}

///|
pub fn compute_ball_voxels_contact(
  ball_center : @core.Vec3,
  ball_radius : @core.Real,
  voxels_pos : @core.Isometry3,
  voxels : Voxels3DReal,
) -> ContactPoint3D? {
  let inv = voxels_pos.inverse()
  let center_local = inv.transform_point(ball_center)
  compute_sphere_voxels_contact(
    ball_center, center_local, ball_radius, voxels_pos, voxels,
  )
}

///|
pub fn compute_cuboid_voxels_contact(
  cuboid_pos : @core.Isometry3,
  cuboid_half_extents : @core.Vec3,
  voxels_pos : @core.Isometry3,
  voxels : Voxels3DReal,
) -> ContactPoint3D? {
  let inv = voxels_pos.inverse()
  let mut best : ContactPoint3D? = None
  let mut best_pen = -1.0F
  let hx = cuboid_half_extents.x
  let hy = cuboid_half_extents.y
  let hz = cuboid_half_extents.z
  let skin = 0.05F
  let samples = [
    @core.Vec3::new(-hx, -hy, -hz),
    @core.Vec3::new(-hx, -hy, hz),
    @core.Vec3::new(hx, -hy, -hz),
    @core.Vec3::new(hx, -hy, hz),
    @core.Vec3::new(0.0F, -hy, 0.0F),
  ]
  for i in 0..<samples.length() {
    let pw = cuboid_pos.transform_point(samples[i])
    let pl = inv.transform_point(pw)
    if compute_sphere_voxels_contact(pw, pl, skin, voxels_pos, voxels)
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
pub fn compute_capsule_voxels_contact(
  capsule_pos : @core.Isometry3,
  capsule_radius : @core.Real,
  capsule_half_height : @core.Real,
  voxels_pos : @core.Isometry3,
  voxels : Voxels3DReal,
) -> ContactPoint3D? {
  let inv = voxels_pos.inverse()
  let mut best : ContactPoint3D? = None
  let mut best_pen = -1.0F
  let hh = capsule_half_height
  let centers_local = [
    @core.Vec3::new(0.0F, -hh, 0.0F),
    @core.Vec3::new(0.0F, -hh * 0.5F, 0.0F),
    @core.Vec3::new(0.0F, 0.0F, 0.0F),
    @core.Vec3::new(0.0F, hh * 0.5F, 0.0F),
    @core.Vec3::new(0.0F, hh, 0.0F),
  ]
  for i in 0..<centers_local.length() {
    let pw = capsule_pos.transform_point(centers_local[i])
    let pl = inv.transform_point(pw)
    if compute_sphere_voxels_contact(pw, pl, capsule_radius, voxels_pos, voxels)
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
pub fn compute_cylinder_voxels_contact(
  cyl_pos : @core.Isometry3,
  cyl_radius : @core.Real,
  cyl_half_height : @core.Real,
  voxels_pos : @core.Isometry3,
  voxels : Voxels3DReal,
) -> ContactPoint3D? {
  let inv = voxels_pos.inverse()
  let mut best : ContactPoint3D? = None
  let mut best_pen = -1.0F
  let skin = 0.05F
  let hh = cyl_half_height
  let r = cyl_radius
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
  // Bottom cap ring + center.
  for i in 0..<ring_dirs.length() {
    let pt_local = ring_dirs[i].scale(r).add(@core.Vec3::new(0.0F, -hh, 0.0F))
    let pw = cyl_pos.transform_point(pt_local)
    let pl = inv.transform_point(pw)
    if compute_sphere_voxels_contact(pw, pl, skin, voxels_pos, voxels)
      is Some(cp) {
      if cp.penetration > best_pen {
        best_pen = cp.penetration
        best = Some(cp)
      }
    }
  }
  let pw0 = cyl_pos.transform_point(@core.Vec3::new(0.0F, -hh, 0.0F))
  let pl0 = inv.transform_point(pw0)
  if compute_sphere_voxels_contact(pw0, pl0, skin, voxels_pos, voxels)
    is Some(cp0) {
    if cp0.penetration > best_pen {
      best_pen = cp0.penetration
      best = Some(cp0)
    }
  }
  best
}

///|
pub fn compute_cone_voxels_contact(
  cone_pos : @core.Isometry3,
  cone_radius : @core.Real,
  cone_half_height : @core.Real,
  voxels_pos : @core.Isometry3,
  voxels : Voxels3DReal,
) -> ContactPoint3D? {
  let inv = voxels_pos.inverse()
  let mut best : ContactPoint3D? = None
  let mut best_pen = -1.0F
  let skin = 0.05F
  let hh = cone_half_height
  let r = cone_radius
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
  // Base ring.
  for i in 0..<ring_dirs.length() {
    let pt_local = ring_dirs[i].scale(r).add(@core.Vec3::new(0.0F, -hh, 0.0F))
    let pw = cone_pos.transform_point(pt_local)
    let pl = inv.transform_point(pw)
    if compute_sphere_voxels_contact(pw, pl, skin, voxels_pos, voxels)
      is Some(cp) {
      if cp.penetration > best_pen {
        best_pen = cp.penetration
        best = Some(cp)
      }
    }
  }
  // Tip.
  let pw_tip = cone_pos.transform_point(@core.Vec3::new(0.0F, hh, 0.0F))
  let pl_tip = inv.transform_point(pw_tip)
  if compute_sphere_voxels_contact(pw_tip, pl_tip, skin, voxels_pos, voxels)
    is Some(cp_tip) {
    if cp_tip.penetration > best_pen {
      best_pen = cp_tip.penetration
      best = Some(cp_tip)
    }
  }
  best
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
fn compute_convex_trimesh_contact(
  convex_pos : @core.Isometry3,
  convex_shape : Shape3D,
  mesh_pos : @core.Isometry3,
  vertices : Array[@core.Vec3],
  indices : Array[(Int, Int, Int)],
) -> ContactPoint3D? {
  let mut best : ContactPoint3D? = None
  let mut best_pen = -1.0F
  for ti in 0..<indices.length() {
    let (i0, i1, i2) = indices[ti]
    let tri = Shape3D::Triangle(vertices[i0], vertices[i1], vertices[i2])
    if compute_convex_contact(convex_pos, convex_shape, mesh_pos, tri)
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
pub fn compute_cuboid_trimesh_contact(
  cuboid_pos : @core.Isometry3,
  cuboid_half_extents : @core.Vec3,
  mesh_pos : @core.Isometry3,
  vertices : Array[@core.Vec3],
  indices : Array[(Int, Int, Int)],
) -> ContactPoint3D? {
  compute_convex_trimesh_contact(
    cuboid_pos,
    Shape3D::Cuboid(cuboid_half_extents),
    mesh_pos,
    vertices,
    indices,
  )
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
  compute_convex_trimesh_contact(
    capsule_pos,
    Shape3D::CapsuleY(capsule_radius, capsule_half_height),
    mesh_pos,
    vertices,
    indices,
  )
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
  compute_convex_trimesh_contact(
    cyl_pos,
    Shape3D::Cylinder(cyl_radius, cyl_half_height),
    mesh_pos,
    vertices,
    indices,
  )
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
  compute_convex_trimesh_contact(
    cone_pos,
    Shape3D::Cone(cone_radius, cone_half_height),
    mesh_pos,
    vertices,
    indices,
  )
}

///|
fn compute_point_heightfield_contact_with_flags(
  point1 : @core.Vec3,
  radius : @core.Real,
  heightfield_pos : @core.Isometry3,
  vertices : Array[@core.Vec3],
  nrows : Int,
  ncols : Int,
  flags : Int,
) -> ContactPoint3D? {
  let (best_point, best_normal, best_dist2, ok) = heightfield_closest_point_bounded(
    point1, heightfield_pos, vertices, nrows, ncols, flags, radius,
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
  // Keep the same terrain-oriented convention as trimeshes.
  let normal = if normal0.y > 0.0F { normal0.scale(-1.0F) } else { normal0 }
  Some({ point1, point2: best_point, normal, penetration: radius - dist })
}

///|
pub fn compute_ball_heightfield_contact(
  ball_center : @core.Vec3,
  ball_radius : @core.Real,
  heightfield_pos : @core.Isometry3,
  vertices : Array[@core.Vec3],
  nrows : Int,
  ncols : Int,
) -> ContactPoint3D? {
  compute_ball_heightfield_contact_with_flags(
    ball_center, ball_radius, heightfield_pos, vertices, nrows, ncols, 0,
  )
}

///|
pub fn compute_ball_heightfield_contact_with_flags(
  ball_center : @core.Vec3,
  ball_radius : @core.Real,
  heightfield_pos : @core.Isometry3,
  vertices : Array[@core.Vec3],
  nrows : Int,
  ncols : Int,
  flags : Int,
) -> ContactPoint3D? {
  let (best_point, best_normal, best_dist2, ok) = heightfield_closest_point_bounded(
    ball_center, heightfield_pos, vertices, nrows, ncols, flags, ball_radius,
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
    best_point.sub(ball_center).scale(1.0F / dist)
  } else {
    best_normal
  }
  let point1 = ball_center.add(normal.scale(ball_radius))
  let point2 = best_point
  Some({ point1, point2, normal, penetration: ball_radius - dist })
}

///|
pub fn compute_cuboid_heightfield_contact(
  cuboid_pos : @core.Isometry3,
  cuboid_half_extents : @core.Vec3,
  heightfield_pos : @core.Isometry3,
  vertices : Array[@core.Vec3],
  nrows : Int,
  ncols : Int,
) -> ContactPoint3D? {
  compute_cuboid_heightfield_contact_with_flags(
    cuboid_pos, cuboid_half_extents, heightfield_pos, vertices, nrows, ncols, 0,
  )
}

///|
pub fn compute_cuboid_heightfield_contact_with_flags(
  cuboid_pos : @core.Isometry3,
  cuboid_half_extents : @core.Vec3,
  heightfield_pos : @core.Isometry3,
  vertices : Array[@core.Vec3],
  nrows : Int,
  ncols : Int,
  flags : Int,
) -> ContactPoint3D? {
  let skin = 0.05F
  let hx = cuboid_half_extents.x
  let hy = cuboid_half_extents.y
  let hz = cuboid_half_extents.z
  let points = [
    @core.Vec3::new(-hx, -hy, -hz),
    @core.Vec3::new(hx, -hy, -hz),
    @core.Vec3::new(-hx, hy, -hz),
    @core.Vec3::new(hx, hy, -hz),
    @core.Vec3::new(-hx, -hy, hz),
    @core.Vec3::new(hx, -hy, hz),
    @core.Vec3::new(-hx, hy, hz),
    @core.Vec3::new(hx, hy, hz),
    @core.Vec3::new(0.0F, -hy, 0.0F),
    @core.Vec3::new(0.0F, hy, 0.0F),
    @core.Vec3::new(-hx, 0.0F, 0.0F),
    @core.Vec3::new(hx, 0.0F, 0.0F),
    @core.Vec3::new(0.0F, 0.0F, -hz),
    @core.Vec3::new(0.0F, 0.0F, hz),
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
    if compute_point_heightfield_contact_with_flags(
        p, skin, heightfield_pos, vertices, nrows, ncols, flags,
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
pub fn compute_capsule_heightfield_contact(
  capsule_pos : @core.Isometry3,
  capsule_radius : @core.Real,
  capsule_half_height : @core.Real,
  heightfield_pos : @core.Isometry3,
  vertices : Array[@core.Vec3],
  nrows : Int,
  ncols : Int,
) -> ContactPoint3D? {
  compute_capsule_heightfield_contact_with_flags(
    capsule_pos, capsule_radius, capsule_half_height, heightfield_pos, vertices,
    nrows, ncols, 0,
  )
}

///|
pub fn compute_capsule_heightfield_contact_with_flags(
  capsule_pos : @core.Isometry3,
  capsule_radius : @core.Real,
  capsule_half_height : @core.Real,
  heightfield_pos : @core.Isometry3,
  vertices : Array[@core.Vec3],
  nrows : Int,
  ncols : Int,
  flags : Int,
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
    if compute_ball_heightfield_contact_with_flags(
        centers[i],
        capsule_radius,
        heightfield_pos,
        vertices,
        nrows,
        ncols,
        flags,
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
pub fn compute_cylinder_heightfield_contact(
  cyl_pos : @core.Isometry3,
  cyl_radius : @core.Real,
  cyl_half_height : @core.Real,
  heightfield_pos : @core.Isometry3,
  vertices : Array[@core.Vec3],
  nrows : Int,
  ncols : Int,
) -> ContactPoint3D? {
  compute_cylinder_heightfield_contact_with_flags(
    cyl_pos, cyl_radius, cyl_half_height, heightfield_pos, vertices, nrows, ncols,
    0,
  )
}

///|
pub fn compute_cylinder_heightfield_contact_with_flags(
  cyl_pos : @core.Isometry3,
  cyl_radius : @core.Real,
  cyl_half_height : @core.Real,
  heightfield_pos : @core.Isometry3,
  vertices : Array[@core.Vec3],
  nrows : Int,
  ncols : Int,
  flags : Int,
) -> ContactPoint3D? {
  let skin = 0.05F
  let r = cyl_radius
  let h = cyl_half_height
  let a = 0.70710677F
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
    if compute_point_heightfield_contact_with_flags(
        p0, skin, heightfield_pos, vertices, nrows, ncols, flags,
      )
      is Some(cp) {
      if cp.penetration > best_pen {
        best_pen = cp.penetration
        best = Some(cp)
      }
    }
    let p1 = cyl_pos.transform_point(
      ring[i].add(@core.Vec3::new(0.0F, h, 0.0F)),
    )
    if compute_point_heightfield_contact_with_flags(
        p1, skin, heightfield_pos, vertices, nrows, ncols, flags,
      )
      is Some(cp) {
      if cp.penetration > best_pen {
        best_pen = cp.penetration
        best = Some(cp)
      }
    }
    let pm = cyl_pos.transform_point(ring[i])
    if compute_point_heightfield_contact_with_flags(
        pm, skin, heightfield_pos, vertices, nrows, ncols, flags,
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
pub fn compute_round_cylinder_heightfield_contact(
  cyl_pos : @core.Isometry3,
  cyl_radius : @core.Real,
  cyl_half_height : @core.Real,
  border_radius : @core.Real,
  heightfield_pos : @core.Isometry3,
  vertices : Array[@core.Vec3],
  nrows : Int,
  ncols : Int,
) -> ContactPoint3D? {
  compute_round_cylinder_heightfield_contact_with_flags(
    cyl_pos, cyl_radius, cyl_half_height, border_radius, heightfield_pos, vertices,
    nrows, ncols, 0,
  )
}

///|
pub fn compute_round_cylinder_heightfield_contact_with_flags(
  cyl_pos : @core.Isometry3,
  cyl_radius : @core.Real,
  cyl_half_height : @core.Real,
  border_radius : @core.Real,
  heightfield_pos : @core.Isometry3,
  vertices : Array[@core.Vec3],
  nrows : Int,
  ncols : Int,
  flags : Int,
) -> ContactPoint3D? {
  compute_capsule_heightfield_contact_with_flags(
    cyl_pos,
    cyl_radius + border_radius,
    cyl_half_height + border_radius,
    heightfield_pos,
    vertices,
    nrows,
    ncols,
    flags,
  )
}

///|
pub fn compute_cone_heightfield_contact(
  cone_pos : @core.Isometry3,
  cone_radius : @core.Real,
  cone_half_height : @core.Real,
  heightfield_pos : @core.Isometry3,
  vertices : Array[@core.Vec3],
  nrows : Int,
  ncols : Int,
) -> ContactPoint3D? {
  compute_cone_heightfield_contact_with_flags(
    cone_pos, cone_radius, cone_half_height, heightfield_pos, vertices, nrows, ncols,
    0,
  )
}

///|
pub fn compute_cone_heightfield_contact_with_flags(
  cone_pos : @core.Isometry3,
  cone_radius : @core.Real,
  cone_half_height : @core.Real,
  heightfield_pos : @core.Isometry3,
  vertices : Array[@core.Vec3],
  nrows : Int,
  ncols : Int,
  flags : Int,
) -> ContactPoint3D? {
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
  for s in slices {
    let y = -hh + (1.0F - s) * (2.0F * hh)
    let rr = r * s
    for i in 0..<ring_dirs.length() {
      let pt_local = ring_dirs[i].scale(rr).add(@core.Vec3::new(0.0F, y, 0.0F))
      let p = cone_pos.transform_point(pt_local)
      if compute_point_heightfield_contact_with_flags(
          p, skin, heightfield_pos, vertices, nrows, ncols, flags,
        )
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
pub fn compute_halfspace_contact(
  halfspace_pos : @core.Isometry3,
  halfspace_normal_local : @core.Vec3,
  other_pos : @core.Isometry3,
  other_shape : Shape3D,
) -> ContactPoint3D? {
  fn normalize_or_default(v : @core.Vec3) -> @core.Vec3 {
    let n2 = v.length_squared()
    if n2 <= 1.0e-12F {
      @core.Vec3::new(0.0F, 1.0F, 0.0F)
    } else {
      v.scale(1.0F / local_sqrt(n2))
    }
  }

  // Support point of `shape` in world-space along `dir`.
  fn support_world(
    p : @core.Isometry3,
    shape : Shape3D,
    dir : @core.Vec3,
  ) -> @core.Vec3? {
    match shape {
      Shape3D::Ball(r) => Some(support_world_for_ball(p.translation, r, dir))
      Shape3D::Cuboid(he) => Some(support_world_for_cuboid(p, he, dir))
      Shape3D::CapsuleY(r, hh) =>
        Some(support_world_for_capsule_y(p, r, hh, dir))
      Shape3D::Cylinder(r, hh) =>
        Some(support_world_for_cylinder_y(p, r, hh, dir))
      Shape3D::RoundCylinder(r, hh, br) =>
        Some(support_world_for_cylinder_y(p, r + br, hh + br, dir))
      Shape3D::Cone(r, hh) => Some(support_world_for_cone_y(p, r, hh, dir))
      Shape3D::Triangle(a, b, c) => {
        let wa = p.transform_point(a)
        let wb = p.transform_point(b)
        let wc = p.transform_point(c)
        let mut best = wa
        let mut best_d = wa.dot(dir)
        let db = wb.dot(dir)
        if db > best_d {
          best = wb
          best_d = db
        }
        let dc = wc.dot(dir)
        if dc > best_d {
          best = wc
        }
        Some(best)
      }
      Shape3D::ConvexHull(points, br) =>
        Some(support_world_for_convex(p, points, br, dir))
      Shape3D::Compound(parts) => {
        let mut best : @core.Vec3? = None
        let mut best_d = -1.0e30F
        for i in 0..<parts.length() {
          let (iso, sh) = parts[i]
          if support_world(p.mul(iso), sh, dir) is Some(pt) {
            let d = pt.dot(dir)
            if d > best_d {
              best_d = d
              best = Some(pt)
            }
          }
        }
        best
      }
      Shape3D::Voxels(v) => {
        let (ix0, iy0, iz0) = v.min_key
        let (ix1, iy1, iz1) = v.max_key
        let sx = v.voxel_size.x
        let sy = v.voxel_size.y
        let sz = v.voxel_size.z
        let local_dir = p.rotation.inverse().rotate_vec3(dir)
        let x0 = Float::from_int(ix0) * sx
        let y0 = Float::from_int(iy0) * sy
        let z0 = Float::from_int(iz0) * sz
        let x1 = Float::from_int(ix1 + 1) * sx
        let y1 = Float::from_int(iy1 + 1) * sy
        let z1 = Float::from_int(iz1 + 1) * sz
        let corner = @core.Vec3::new(
          if local_dir.x >= 0.0F {
            x1
          } else {
            x0
          },
          if local_dir.y >= 0.0F {
            y1
          } else {
            y0
          },
          if local_dir.z >= 0.0F {
            z1
          } else {
            z0
          },
        )
        Some(p.transform_point(corner))
      }
      Shape3D::TriMesh(vertices, _) =>
        if vertices.length() == 0 {
          None
        } else {
          let mut best = p.transform_point(vertices[0])
          let mut best_d = best.dot(dir)
          for i in 1..<vertices.length() {
            let pt = p.transform_point(vertices[i])
            let d = pt.dot(dir)
            if d > best_d {
              best_d = d
              best = pt
            }
          }
          Some(best)
        }
      Shape3D::Heightfield(vertices, _, _, _, _) =>
        if vertices.length() == 0 {
          None
        } else {
          let mut best = p.transform_point(vertices[0])
          let mut best_d = best.dot(dir)
          for i in 1..<vertices.length() {
            let pt = p.transform_point(vertices[i])
            let d = pt.dot(dir)
            if d > best_d {
              best_d = d
              best = pt
            }
          }
          Some(best)
        }
      Shape3D::HalfSpace(_) => None
    }
  }

  let normal = normalize_or_default(
    halfspace_pos.rotation.rotate_vec3(halfspace_normal_local),
  )
  // Choose the point with minimum projection on the plane normal.
  if support_world(other_pos, other_shape, normal.scale(-1.0F)) is Some(p2) {
    let plane_origin = halfspace_pos.translation
    let dist = normal.dot(p2.sub(plane_origin))
    if dist >= 0.0F {
      return None
    }
    // Point on the plane directly "below" the witness point.
    let p1 = p2.sub(normal.scale(dist))
    Some({ point1: p1, point2: p2, normal, penetration: -dist })
  } else {
    None
  }
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
pub fn compute_cylinder_cuboid_contact(
  cyl_pos : @core.Isometry3,
  cyl_radius : @core.Real,
  cyl_half_height : @core.Real,
  cuboid_pos : @core.Isometry3,
  cuboid_half_extents : @core.Vec3,
) -> ContactPoint3D? {
  compute_convex_contact(
    cyl_pos,
    Shape3D::Cylinder(cyl_radius, cyl_half_height),
    cuboid_pos,
    Shape3D::Cuboid(cuboid_half_extents),
  )
}

///|
pub fn compute_cylinder_cylinder_contact(
  cyl1_pos : @core.Isometry3,
  cyl1_radius : @core.Real,
  cyl1_half_height : @core.Real,
  cyl2_pos : @core.Isometry3,
  cyl2_radius : @core.Real,
  cyl2_half_height : @core.Real,
) -> ContactPoint3D? {
  compute_convex_contact(
    cyl1_pos,
    Shape3D::Cylinder(cyl1_radius, cyl1_half_height),
    cyl2_pos,
    Shape3D::Cylinder(cyl2_radius, cyl2_half_height),
  )
}

///|
pub fn compute_cone_cylinder_contact(
  cone_pos : @core.Isometry3,
  cone_radius : @core.Real,
  cone_half_height : @core.Real,
  cyl_pos : @core.Isometry3,
  cyl_radius : @core.Real,
  cyl_half_height : @core.Real,
) -> ContactPoint3D? {
  compute_convex_contact(
    cone_pos,
    Shape3D::Cone(cone_radius, cone_half_height),
    cyl_pos,
    Shape3D::Cylinder(cyl_radius, cyl_half_height),
  )
}

///|
pub fn compute_cone_cone_contact(
  cone1_pos : @core.Isometry3,
  cone1_radius : @core.Real,
  cone1_half_height : @core.Real,
  cone2_pos : @core.Isometry3,
  cone2_radius : @core.Real,
  cone2_half_height : @core.Real,
) -> ContactPoint3D? {
  compute_convex_contact(
    cone1_pos,
    Shape3D::Cone(cone1_radius, cone1_half_height),
    cone2_pos,
    Shape3D::Cone(cone2_radius, cone2_half_height),
  )
}

///|
pub fn compute_cone_cuboid_contact(
  cone_pos : @core.Isometry3,
  cone_radius : @core.Real,
  cone_half_height : @core.Real,
  cuboid_pos : @core.Isometry3,
  cuboid_half_extents : @core.Vec3,
) -> ContactPoint3D? {
  compute_convex_contact(
    cone_pos,
    Shape3D::Cone(cone_radius, cone_half_height),
    cuboid_pos,
    Shape3D::Cuboid(cuboid_half_extents),
  )
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

///|
/// Generate a contact manifold (up to 4 points) for a cuboid-cuboid contact.
///
/// This is a simplified BoxBox manifold generator:
/// - For face-face contacts (best axis is a face normal), it computes the incident face corners,
///   clips them against the reference face side planes, and keeps up to 4 extreme points.
/// - For edge-edge contacts (best axis is a cross product), it falls back to a single support-point
///   contact.
fn compute_cuboid_cuboid_contacts(
  pos1 : @core.Isometry3,
  he1 : @core.Vec3,
  pos2 : @core.Isometry3,
  he2 : @core.Vec3,
) -> Array[ContactPoint3D] {
  let out : Array[ContactPoint3D] = []
  let a0 = he1.x
  let a1 = he1.y
  let a2 = he1.z
  let b0 = he2.x
  let b1 = he2.y
  let b2 = he2.z
  let ra = pos1.rotation.to_mat3()
  let rb = pos2.rotation.to_mat3()
  fn col(m : @core.Mat3, i : Int) -> @core.Vec3 {
    match i {
      0 => @core.Vec3::new(m.m00, m.m10, m.m20)
      1 => @core.Vec3::new(m.m01, m.m11, m.m21)
      _ => @core.Vec3::new(m.m02, m.m12, m.m22)
    }
  }

  let a_axis0 = col(ra, 0)
  let a_axis1 = col(ra, 1)
  let a_axis2 = col(ra, 2)
  let b_axis0 = col(rb, 0)
  let b_axis1 = col(rb, 1)
  let b_axis2 = col(rb, 2)
  let d = pos2.translation.sub(pos1.translation)
  let t0 = d.dot(a_axis0)
  let t1 = d.dot(a_axis1)
  let t2 = d.dot(a_axis2)
  let eps = 1.0e-6F
  let r00 = a_axis0.dot(b_axis0)
  let r01 = a_axis0.dot(b_axis1)
  let r02 = a_axis0.dot(b_axis2)
  let r10 = a_axis1.dot(b_axis0)
  let r11 = a_axis1.dot(b_axis1)
  let r12 = a_axis1.dot(b_axis2)
  let r20 = a_axis2.dot(b_axis0)
  let r21 = a_axis2.dot(b_axis1)
  let r22 = a_axis2.dot(b_axis2)
  let abs_r00 = @core.abs(r00) + eps
  let abs_r01 = @core.abs(r01) + eps
  let abs_r02 = @core.abs(r02) + eps
  let abs_r10 = @core.abs(r10) + eps
  let abs_r11 = @core.abs(r11) + eps
  let abs_r12 = @core.abs(r12) + eps
  let abs_r20 = @core.abs(r20) + eps
  let abs_r21 = @core.abs(r21) + eps
  let abs_r22 = @core.abs(r22) + eps
  fn a_extent_at(
    a0 : @core.Real,
    a1 : @core.Real,
    a2 : @core.Real,
    i : Int,
  ) -> @core.Real {
    match i {
      0 => a0
      1 => a1
      _ => a2
    }
  }

  fn b_extent_at(
    b0 : @core.Real,
    b1 : @core.Real,
    b2 : @core.Real,
    i : Int,
  ) -> @core.Real {
    match i {
      0 => b0
      1 => b1
      _ => b2
    }
  }

  fn a_axis_at(
    a0 : @core.Vec3,
    a1 : @core.Vec3,
    a2 : @core.Vec3,
    i : Int,
  ) -> @core.Vec3 {
    match i {
      0 => a0
      1 => a1
      _ => a2
    }
  }

  fn b_axis_at(
    b0 : @core.Vec3,
    b1 : @core.Vec3,
    b2 : @core.Vec3,
    i : Int,
  ) -> @core.Vec3 {
    match i {
      0 => b0
      1 => b1
      _ => b2
    }
  }

  fn t_at(
    t0 : @core.Real,
    t1 : @core.Real,
    t2 : @core.Real,
    i : Int,
  ) -> @core.Real {
    match i {
      0 => t0
      1 => t1
      _ => t2
    }
  }

  fn r_at(
    r00 : @core.Real,
    r01 : @core.Real,
    r02 : @core.Real,
    r10 : @core.Real,
    r11 : @core.Real,
    r12 : @core.Real,
    r20 : @core.Real,
    r21 : @core.Real,
    r22 : @core.Real,
    i : Int,
    j : Int,
  ) -> @core.Real {
    match (i, j) {
      (0, 0) => r00
      (0, 1) => r01
      (0, 2) => r02
      (1, 0) => r10
      (1, 1) => r11
      (1, 2) => r12
      (2, 0) => r20
      (2, 1) => r21
      _ => r22
    }
  }

  fn abs_r_at(
    r00 : @core.Real,
    r01 : @core.Real,
    r02 : @core.Real,
    r10 : @core.Real,
    r11 : @core.Real,
    r12 : @core.Real,
    r20 : @core.Real,
    r21 : @core.Real,
    r22 : @core.Real,
    i : Int,
    j : Int,
  ) -> @core.Real {
    match (i, j) {
      (0, 0) => r00
      (0, 1) => r01
      (0, 2) => r02
      (1, 0) => r10
      (1, 1) => r11
      (1, 2) => r12
      (2, 0) => r20
      (2, 1) => r21
      _ => r22
    }
  }

  let mut best = CuboidBestAxis::A(0)
  let mut min_overlap = 1.0e30F

  // Test L = A_i.
  for i in 0..<3 {
    let ra_i = a_extent_at(a0, a1, a2, i)
    let rb_i = b0 *
      abs_r_at(
        abs_r00, abs_r01, abs_r02, abs_r10, abs_r11, abs_r12, abs_r20, abs_r21, abs_r22,
        i, 0,
      ) +
      b1 *
      abs_r_at(
        abs_r00, abs_r01, abs_r02, abs_r10, abs_r11, abs_r12, abs_r20, abs_r21, abs_r22,
        i, 1,
      ) +
      b2 *
      abs_r_at(
        abs_r00, abs_r01, abs_r02, abs_r10, abs_r11, abs_r12, abs_r20, abs_r21, abs_r22,
        i, 2,
      )
    let dist = @core.abs(t_at(t0, t1, t2, i))
    let overlap = ra_i + rb_i - dist
    if overlap < 0.0F {
      return out
    }
    if overlap < min_overlap {
      min_overlap = overlap
      best = CuboidBestAxis::A(i)
    }
  }

  // Test L = B_j.
  for j in 0..<3 {
    let ra_j = a0 *
      abs_r_at(
        abs_r00, abs_r01, abs_r02, abs_r10, abs_r11, abs_r12, abs_r20, abs_r21, abs_r22,
        0, j,
      ) +
      a1 *
      abs_r_at(
        abs_r00, abs_r01, abs_r02, abs_r10, abs_r11, abs_r12, abs_r20, abs_r21, abs_r22,
        1, j,
      ) +
      a2 *
      abs_r_at(
        abs_r00, abs_r01, abs_r02, abs_r10, abs_r11, abs_r12, abs_r20, abs_r21, abs_r22,
        2, j,
      )
    let rb_j = b_extent_at(b0, b1, b2, j)
    let proj = t0 * r_at(r00, r01, r02, r10, r11, r12, r20, r21, r22, 0, j) +
      t1 * r_at(r00, r01, r02, r10, r11, r12, r20, r21, r22, 1, j) +
      t2 * r_at(r00, r01, r02, r10, r11, r12, r20, r21, r22, 2, j)
    let dist = @core.abs(proj)
    let overlap = ra_j + rb_j - dist
    if overlap < 0.0F {
      return out
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
      let ra_c = a_extent_at(a0, a1, a2, i1) *
        abs_r_at(
          abs_r00, abs_r01, abs_r02, abs_r10, abs_r11, abs_r12, abs_r20, abs_r21,
          abs_r22, i2, j,
        ) +
        a_extent_at(a0, a1, a2, i2) *
        abs_r_at(
          abs_r00, abs_r01, abs_r02, abs_r10, abs_r11, abs_r12, abs_r20, abs_r21,
          abs_r22, i1, j,
        )
      let rb_c = b_extent_at(b0, b1, b2, j1) *
        abs_r_at(
          abs_r00, abs_r01, abs_r02, abs_r10, abs_r11, abs_r12, abs_r20, abs_r21,
          abs_r22, i, j2,
        ) +
        b_extent_at(b0, b1, b2, j2) *
        abs_r_at(
          abs_r00, abs_r01, abs_r02, abs_r10, abs_r11, abs_r12, abs_r20, abs_r21,
          abs_r22, i, j1,
        )
      let dist = @core.abs(
        t_at(t0, t1, t2, i2) *
        r_at(r00, r01, r02, r10, r11, r12, r20, r21, r22, i1, j) -
        t_at(t0, t1, t2, i1) *
        r_at(r00, r01, r02, r10, r11, r12, r20, r21, r22, i2, j),
      )
      let overlap = ra_c + rb_c - dist
      if overlap < 0.0F {
        return out
      }
      let axis = a_axis_at(a_axis0, a_axis1, a_axis2, i).cross(
        b_axis_at(b_axis0, b_axis1, b_axis2, j),
      )
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
      let s = if t_at(t0, t1, t2, i) < 0.0F { -1.0F } else { 1.0F }
      a_axis_at(a_axis0, a_axis1, a_axis2, i).scale(s)
    }
    CuboidBestAxis::B(j) => {
      let proj = t0 * r_at(r00, r01, r02, r10, r11, r12, r20, r21, r22, 0, j) +
        t1 * r_at(r00, r01, r02, r10, r11, r12, r20, r21, r22, 1, j) +
        t2 * r_at(r00, r01, r02, r10, r11, r12, r20, r21, r22, 2, j)
      let s = if proj < 0.0F { -1.0F } else { 1.0F }
      b_axis_at(b_axis0, b_axis1, b_axis2, j).scale(s)
    }
    CuboidBestAxis::Cross(i, j) => {
      let axis = a_axis_at(a_axis0, a_axis1, a_axis2, i).cross(
        b_axis_at(b_axis0, b_axis1, b_axis2, j),
      )
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

  // Edge-edge contact: fall back to a single point.
  match best {
    CuboidBestAxis::Cross(_, _) => {
      let point1 = support(pos1, he1, normal)
      let point2 = support(pos2, he2, normal.scale(-1.0F))
      out.push({ point1, point2, normal, penetration: min_overlap })
      return out
    }
    _ => ()
  }
  fn incident_face_corners(
    pos : @core.Isometry3,
    axis0 : @core.Vec3,
    axis1 : @core.Vec3,
    axis2 : @core.Vec3,
    e0 : @core.Real,
    e1 : @core.Real,
    e2 : @core.Real,
    ref_normal : @core.Vec3,
  ) -> Array[@core.Vec3] {
    // Choose the face normal most anti-parallel to `ref_normal`.
    let mut best_k = 0
    let mut best_abs = @core.abs(axis0.dot(ref_normal))
    let abs1 = @core.abs(axis1.dot(ref_normal))
    if abs1 > best_abs {
      best_abs = abs1
      best_k = 1
    }
    let abs2 = @core.abs(axis2.dot(ref_normal))
    if abs2 > best_abs {
      best_abs = abs2
      best_k = 2
    }
    let (best_axis, best_extent, u_axis, u_extent, v_axis, v_extent) = match
      best_k {
      0 => (axis0, e0, axis1, e1, axis2, e2)
      1 => (axis1, e1, axis2, e2, axis0, e0)
      _ => (axis2, e2, axis0, e0, axis1, e1)
    }
    let sign = if best_axis.dot(ref_normal) > 0.0F { -1.0F } else { 1.0F }
    let face_center = pos.translation.add(best_axis.scale(sign * best_extent))
    let au = u_axis.scale(u_extent)
    let av = v_axis.scale(v_extent)
    [
      face_center.add(au).add(av),
      face_center.add(au).sub(av),
      face_center.sub(au).add(av),
      face_center.sub(au).sub(av),
    ]
  }

  fn clip_polygon_plane(
    poly : Array[@core.Vec3],
    plane_point : @core.Vec3,
    plane_normal : @core.Vec3,
  ) -> Array[@core.Vec3] {
    if poly.length() == 0 {
      return []
    }
    let out : Array[@core.Vec3] = []
    let mut prev = poly[poly.length() - 1]
    let mut prev_d = prev.sub(plane_point).dot(plane_normal)
    for i in 0..<poly.length() {
      let cur = poly[i]
      let cur_d = cur.sub(plane_point).dot(plane_normal)
      let prev_in = prev_d <= 0.0F
      let cur_in = cur_d <= 0.0F
      if prev_in && cur_in {
        out.push(cur)
      } else if prev_in && !cur_in {
        let denom = prev_d - cur_d
        if @core.abs(denom) > 1.0e-12F {
          let t = prev_d / denom
          out.push(prev.add(cur.sub(prev).scale(t)))
        }
      } else if !prev_in && cur_in {
        let denom = prev_d - cur_d
        if @core.abs(denom) > 1.0e-12F {
          let t = prev_d / denom
          out.push(prev.add(cur.sub(prev).scale(t)))
        }
        out.push(cur)
      }
      prev = cur
      prev_d = cur_d
    }
    out
  }

  fn reduce_face_contacts(
    candidates : Array[@core.Vec3],
    origin : @core.Vec3,
    u_axis : @core.Vec3,
    v_axis : @core.Vec3,
  ) -> Array[@core.Vec3] {
    if candidates.length() <= 4 {
      return candidates
    }
    let mut min_u_i = 0
    let mut max_u_i = 0
    let mut min_v_i = 0
    let mut max_v_i = 0
    let mut min_u = 1.0e30F
    let mut max_u = -1.0e30F
    let mut min_v = 1.0e30F
    let mut max_v = -1.0e30F
    for i in 0..<candidates.length() {
      let rel = candidates[i].sub(origin)
      let u = rel.dot(u_axis)
      let v = rel.dot(v_axis)
      if u < min_u {
        min_u = u
        min_u_i = i
      }
      if u > max_u {
        max_u = u
        max_u_i = i
      }
      if v < min_v {
        min_v = v
        min_v_i = i
      }
      if v > max_v {
        max_v = v
        max_v_i = i
      }
    }
    let idxs = [min_u_i, max_u_i, min_v_i, max_v_i]
    let out : Array[@core.Vec3] = []
    let eps2 = 1.0e-8F
    for k in 0..<idxs.length() {
      let p = candidates[idxs[k]]
      let mut dup = false
      for j in 0..<out.length() {
        if out[j].sub(p).length_squared() <= eps2 {
          dup = true
          break
        }
      }
      if !dup {
        out.push(p)
      }
    }
    out
  }

  fn face_axes(i : Int) -> (Int, Int) {
    ((i + 1) % 3, (i + 2) % 3)
  }

  match best {
    CuboidBestAxis::A(i) => {
      let (u_i, v_i) = face_axes(i)
      let u_axis = a_axis_at(a_axis0, a_axis1, a_axis2, u_i)
      let v_axis = a_axis_at(a_axis0, a_axis1, a_axis2, v_i)
      let ref_center = pos1.translation.add(
        normal.scale(a_extent_at(a0, a1, a2, i)),
      )
      // Incident face is on box2.
      let corners = incident_face_corners(
        pos2, b_axis0, b_axis1, b_axis2, b0, b1, b2, normal,
      )
      let mut poly = corners
      // Clip against the 4 side planes of the reference face.
      poly = clip_polygon_plane(
        poly,
        ref_center.add(u_axis.scale(a_extent_at(a0, a1, a2, u_i))),
        u_axis,
      )
      poly = clip_polygon_plane(
        poly,
        ref_center.sub(u_axis.scale(a_extent_at(a0, a1, a2, u_i))),
        u_axis.scale(-1.0F),
      )
      poly = clip_polygon_plane(
        poly,
        ref_center.add(v_axis.scale(a_extent_at(a0, a1, a2, v_i))),
        v_axis,
      )
      poly = clip_polygon_plane(
        poly,
        ref_center.sub(v_axis.scale(a_extent_at(a0, a1, a2, v_i))),
        v_axis.scale(-1.0F),
      )
      let pts = reduce_face_contacts(poly, ref_center, u_axis, v_axis)
      for p2 in pts {
        let p1 = p2.sub(normal.scale(min_overlap))
        out.push({ point1: p1, point2: p2, normal, penetration: min_overlap })
      }
      out
    }
    CuboidBestAxis::B(j) => {
      let (u_j, v_j) = face_axes(j)
      let u_axis = b_axis_at(b_axis0, b_axis1, b_axis2, u_j)
      let v_axis = b_axis_at(b_axis0, b_axis1, b_axis2, v_j)
      let ref_normal = normal.scale(-1.0F)
      let ref_center = pos2.translation.add(
        ref_normal.scale(b_extent_at(b0, b1, b2, j)),
      )
      // Incident face is on box1.
      let corners = incident_face_corners(
        pos1, a_axis0, a_axis1, a_axis2, a0, a1, a2, ref_normal,
      )
      let mut poly = corners
      poly = clip_polygon_plane(
        poly,
        ref_center.add(u_axis.scale(b_extent_at(b0, b1, b2, u_j))),
        u_axis,
      )
      poly = clip_polygon_plane(
        poly,
        ref_center.sub(u_axis.scale(b_extent_at(b0, b1, b2, u_j))),
        u_axis.scale(-1.0F),
      )
      poly = clip_polygon_plane(
        poly,
        ref_center.add(v_axis.scale(b_extent_at(b0, b1, b2, v_j))),
        v_axis,
      )
      poly = clip_polygon_plane(
        poly,
        ref_center.sub(v_axis.scale(b_extent_at(b0, b1, b2, v_j))),
        v_axis.scale(-1.0F),
      )
      let pts = reduce_face_contacts(poly, ref_center, u_axis, v_axis)
      for p1 in pts {
        let p2 = p1.add(normal.scale(min_overlap))
        out.push({ point1: p1, point2: p2, normal, penetration: min_overlap })
      }
      out
    }
    _ => out
  }
}

///|
priv struct SupportVertex3D {
  p : @core.Vec3 // Minkowski point (support1 - support2)
  sup1 : @core.Vec3
  sup2 : @core.Vec3
}

///|
fn support_world_for_convex(
  pos : @core.Isometry3,
  points : Array[@core.Vec3],
  border_radius : @core.Real,
  dir : @core.Vec3,
) -> @core.Vec3 {
  if points.length() == 0 {
    return pos.translation
  }
  let local_dir = pos.rotation.inverse().rotate_vec3(dir)
  let mut best = points[0]
  let mut best_dot = best.dot(local_dir)
  for i in 1..<points.length() {
    let p = points[i]
    let d = p.dot(local_dir)
    if d > best_dot {
      best_dot = d
      best = p
    }
  }
  let mut out = pos.transform_point(best)
  let d2 = dir.length_squared()
  if border_radius > 0.0F && d2 > 1.0e-12F {
    out = out.add(dir.scale(border_radius / local_sqrt(d2)))
  }
  out
}

///|
fn support_world_for_ball(
  center : @core.Vec3,
  radius : @core.Real,
  dir : @core.Vec3,
) -> @core.Vec3 {
  let d2 = dir.length_squared()
  if d2 <= 1.0e-12F {
    center
  } else {
    center.add(dir.scale(radius / local_sqrt(d2)))
  }
}

///|
fn support_world_for_cuboid(
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

///|
fn support_world_for_cylinder_y(
  pos : @core.Isometry3,
  radius : @core.Real,
  half_height : @core.Real,
  dir : @core.Vec3,
) -> @core.Vec3 {
  let local_dir = pos.rotation.inverse().rotate_vec3(dir)
  let y = if local_dir.y >= 0.0F { half_height } else { -half_height }
  let radial = @core.Vec3::new(local_dir.x, 0.0F, local_dir.z)
  let rl2 = radial.length_squared()
  let (x, z) = if rl2 <= 1.0e-12F {
    // Any point on the cap is valid when the direction is purely along Y.
    // Pick the center to avoid introducing a spurious tangential component
    // that can cause GJK/EPA degeneracy.
    (0.0F, 0.0F)
  } else {
    let inv = 1.0F / local_sqrt(rl2)
    (radial.x * inv * radius, radial.z * inv * radius)
  }
  pos.transform_point(@core.Vec3::new(x, y, z))
}

///|
fn support_world_for_capsule_y(
  pos : @core.Isometry3,
  radius : @core.Real,
  half_height : @core.Real,
  dir : @core.Vec3,
) -> @core.Vec3 {
  let local_dir = pos.rotation.inverse().rotate_vec3(dir)
  let cap_y = if local_dir.y >= 0.0F { half_height } else { -half_height }
  let base = @core.Vec3::new(0.0F, cap_y, 0.0F)
  let d2 = local_dir.length_squared()
  let offset = if d2 <= 1.0e-12F {
    @core.Vec3::new(radius, 0.0F, 0.0F)
  } else {
    local_dir.scale(radius / local_sqrt(d2))
  }
  pos.transform_point(base.add(offset))
}

///|
fn support_world_for_cone_y(
  pos : @core.Isometry3,
  radius : @core.Real,
  half_height : @core.Real,
  dir : @core.Vec3,
) -> @core.Vec3 {
  let local_dir = pos.rotation.inverse().rotate_vec3(dir)
  let apex = @core.Vec3::new(0.0F, half_height, 0.0F)
  let radial = @core.Vec3::new(local_dir.x, 0.0F, local_dir.z)
  let rl2 = radial.length_squared()
  let base_pt = if rl2 <= 1.0e-12F {
    // When the direction is purely along Y, any point on the base disk is valid.
    // Pick the center to keep support points symmetric.
    @core.Vec3::new(0.0F, -half_height, 0.0F)
  } else {
    let inv = 1.0F / local_sqrt(rl2)
    @core.Vec3::new(
      radial.x * inv * radius,
      -half_height,
      radial.z * inv * radius,
    )
  }
  let da = apex.dot(local_dir)
  let db = base_pt.dot(local_dir)
  pos.transform_point(if da >= db { apex } else { base_pt })
}

///|
fn support_minkowski(
  pos1 : @core.Isometry3,
  s1 : Shape3D,
  pos2 : @core.Isometry3,
  s2 : Shape3D,
  dir : @core.Vec3,
) -> SupportVertex3D? {
  let sup1 = match s1 {
    Shape3D::ConvexHull(points, br) =>
      support_world_for_convex(pos1, points, br, dir)
    Shape3D::Cuboid(he) => support_world_for_cuboid(pos1, he, dir)
    Shape3D::Ball(r) => support_world_for_ball(pos1.translation, r, dir)
    Shape3D::CapsuleY(r, hh) => support_world_for_capsule_y(pos1, r, hh, dir)
    Shape3D::Cylinder(r, hh) => support_world_for_cylinder_y(pos1, r, hh, dir)
    Shape3D::RoundCylinder(r, hh, br) =>
      support_world_for_cylinder_y(pos1, r + br, hh + br, dir)
    Shape3D::Cone(r, hh) => support_world_for_cone_y(pos1, r, hh, dir)
    _ => return None
  }
  let neg_dir = dir.scale(-1.0F)
  let sup2 = match s2 {
    Shape3D::ConvexHull(points, br) =>
      support_world_for_convex(pos2, points, br, neg_dir)
    Shape3D::Cuboid(he) => support_world_for_cuboid(pos2, he, neg_dir)
    Shape3D::Ball(r) => support_world_for_ball(pos2.translation, r, neg_dir)
    Shape3D::CapsuleY(r, hh) =>
      support_world_for_capsule_y(pos2, r, hh, neg_dir)
    Shape3D::Cylinder(r, hh) =>
      support_world_for_cylinder_y(pos2, r, hh, neg_dir)
    Shape3D::RoundCylinder(r, hh, br) =>
      support_world_for_cylinder_y(pos2, r + br, hh + br, neg_dir)
    Shape3D::Cone(r, hh) => support_world_for_cone_y(pos2, r, hh, neg_dir)
    _ => return None
  }
  Some({ p: sup1.sub(sup2), sup1, sup2 })
}

///|
fn gjk_reduce_simplex_closest_to_origin(
  simplex : Array[SupportVertex3D],
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
    let mut best_simplex : Array[SupportVertex3D] = []
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
        let tri_simplex : Array[SupportVertex3D] = [
          simplex[ic],
          simplex[ib],
          simplex[ia],
        ]
        let (closest, w, _) = gjk_reduce_simplex_closest_to_origin(tri_simplex)
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
fn gjk_intersect(
  pos1 : @core.Isometry3,
  s1 : Shape3D,
  pos2 : @core.Isometry3,
  s2 : Shape3D,
) -> Array[SupportVertex3D]? {
  let mut dir = pos2.translation.sub(pos1.translation)
  if dir.length_squared() <= 1.0e-12F {
    dir = @core.Vec3::new(1.0F, 0.0F, 0.0F)
  }
  let simplex : Array[SupportVertex3D] = []
  if support_minkowski(pos1, s1, pos2, s2, dir) is Some(v0) {
    simplex.push(v0)
    dir = v0.p.scale(-1.0F)
  } else {
    return None
  }
  let eps2 = 1.0e-18F
  let mut closest = simplex[0].p
  for _ in 0..<64 {
    if dir.length_squared() <= eps2 {
      if simplex.length() >= 4 {
        return Some(simplex)
      }
      // Try to expand the simplex in a new direction to avoid low-dimensional degeneracy.
      if simplex.length() == 1 {
        dir = @core.Vec3::new(0.0F, 1.0F, 0.0F)
      } else if simplex.length() == 2 {
        let a = simplex[1].p
        let b = simplex[0].p
        let ab = b.sub(a)
        let axis = if @core.abs(ab.x) < 0.9F {
          @core.Vec3::new(1.0F, 0.0F, 0.0F)
        } else {
          @core.Vec3::new(0.0F, 1.0F, 0.0F)
        }
        dir = ab.cross(axis)
        if dir.length_squared() <= eps2 {
          dir = ab.cross(@core.Vec3::new(0.0F, 0.0F, 1.0F))
        }
      } else if simplex.length() == 3 {
        let a = simplex[2].p
        let b = simplex[1].p
        let c = simplex[0].p
        dir = b.sub(a).cross(c.sub(a))
        if dir.length_squared() <= eps2 {
          dir = @core.Vec3::new(1.0F, 0.0F, 0.0F)
        }
      } else {
        dir = @core.Vec3::new(1.0F, 0.0F, 0.0F)
      }
    }
    if support_minkowski(pos1, s1, pos2, s2, dir) is Some(a) {
      if a.p.dot(dir) < 0.0F {
        return None
      }
      simplex.push(a)
      let (new_closest, _, inside) = gjk_reduce_simplex_closest_to_origin(
        simplex,
      )
      closest = new_closest
      if inside {
        return Some(simplex)
      }
      dir = closest.scale(-1.0F)
    } else {
      return None
    }
  }
  None
}

///|
priv struct EpaFace {
  i0 : Int
  i1 : Int
  i2 : Int
  normal : @core.Vec3
  dist : @core.Real
}

///|
fn epa_make_face(
  verts : Array[SupportVertex3D],
  i0 : Int,
  i1 : Int,
  i2 : Int,
) -> EpaFace? {
  let a = verts[i0].p
  let b = verts[i1].p
  let c = verts[i2].p
  let n0 = b.sub(a).cross(c.sub(a))
  let n2 = n0.length_squared()
  if n2 <= 1.0e-18F {
    return None
  }
  let inv = 1.0F / local_sqrt(n2)
  let mut normal = n0.scale(inv)
  let mut dist = normal.dot(a)
  let mut i1 = i1
  let mut i2 = i2
  if dist < 0.0F {
    normal = normal.scale(-1.0F)
    dist = -dist
    let tmp = i1
    i1 = i2
    i2 = tmp
  }
  Some({ i0, i1, i2, normal, dist })
}

///|
fn epa_make_face_with_opposite(
  verts : Array[SupportVertex3D],
  i0 : Int,
  i1 : Int,
  i2 : Int,
  i_opp : Int,
) -> EpaFace? {
  let a = verts[i0].p
  let b = verts[i1].p
  let c = verts[i2].p
  let opp = verts[i_opp].p
  let mut n0 = b.sub(a).cross(c.sub(a))
  let n2 = n0.length_squared()
  if n2 <= 1.0e-18F {
    return None
  }
  let mut i1 = i1
  let mut i2 = i2
  // Ensure the normal points away from the opposite vertex (outward for tetrahedra).
  if n0.dot(opp.sub(a)) > 0.0F {
    n0 = n0.scale(-1.0F)
    let tmp = i1
    i1 = i2
    i2 = tmp
  }
  let inv = 1.0F / local_sqrt(n2)
  let mut normal = n0.scale(inv)
  let mut dist = normal.dot(a)
  if dist < 0.0F {
    normal = normal.scale(-1.0F)
    dist = -dist
    let tmp = i1
    i1 = i2
    i2 = tmp
  }
  Some({ i0, i1, i2, normal, dist })
}

///|
fn barycentric_on_triangle(
  a : @core.Vec3,
  b : @core.Vec3,
  c : @core.Vec3,
  p : @core.Vec3,
) -> (@core.Real, @core.Real, @core.Real) {
  let v0 = b.sub(a)
  let v1 = c.sub(a)
  let v2 = p.sub(a)
  let d00 = v0.dot(v0)
  let d01 = v0.dot(v1)
  let d11 = v1.dot(v1)
  let d20 = v2.dot(v0)
  let d21 = v2.dot(v1)
  let denom = d00 * d11 - d01 * d01
  if @core.abs(denom) <= 1.0e-18F {
    return (1.0F, 0.0F, 0.0F)
  }
  let inv = 1.0F / denom
  let v = (d11 * d20 - d01 * d21) * inv
  let w = (d00 * d21 - d01 * d20) * inv
  let u = 1.0F - v - w
  (u, v, w)
}

///|
fn epa_contact(
  pos1 : @core.Isometry3,
  s1 : Shape3D,
  pos2 : @core.Isometry3,
  s2 : Shape3D,
  simplex : Array[SupportVertex3D],
) -> ContactPoint3D? {
  // Gather vertices.
  let verts : Array[SupportVertex3D] = []
  for i in 0..<simplex.length() {
    verts.push(simplex[i])
  }
  // If we don't have a tetrahedron, try to grow it along a fixed direction.
  while verts.length() < 4 {
    let dir = if verts.length() == 1 {
      @core.Vec3::new(1.0F, 0.0F, 0.0F)
    } else {
      verts[verts.length() - 1].p.scale(-1.0F)
    }
    if support_minkowski(pos1, s1, pos2, s2, dir) is Some(v) {
      verts.push(v)
    } else {
      break
    }
  }
  if verts.length() < 4 {
    return None
  }
  let faces : Array[EpaFace] = []
  let initial = [(0, 1, 2, 3), (0, 3, 1, 2), (0, 2, 3, 1), (1, 3, 2, 0)]
  for i in 0..<initial.length() {
    let (i0, i1, i2, i_opp) = initial[i]
    if epa_make_face_with_opposite(verts, i0, i1, i2, i_opp) is Some(f) {
      faces.push(f)
    }
  }
  if faces.length() == 0 {
    return None
  }
  let tol = 1.0e-4F
  for _ in 0..<64 {
    // Find closest face.
    let mut best_i = 0
    let mut best_d = faces[0].dist
    for i in 1..<faces.length() {
      if faces[i].dist < best_d {
        best_d = faces[i].dist
        best_i = i
      }
    }
    let best = faces[best_i]
    let dir = best.normal
    if support_minkowski(pos1, s1, pos2, s2, dir) is Some(v) {
      let d = v.p.dot(dir)
      if d - best.dist < tol {
        // Converged.
        let p = dir.scale(best.dist)
        let a = verts[best.i0]
        let b = verts[best.i1]
        let c = verts[best.i2]
        let (u, vv, w) = barycentric_on_triangle(a.p, b.p, c.p, p)
        let cp1 = a.sup1.scale(u).add(b.sup1.scale(vv)).add(c.sup1.scale(w))
        let cp2 = a.sup2.scale(u).add(b.sup2.scale(vv)).add(c.sup2.scale(w))
        // Keep the convention consistent with the analytic contact generators and the solver:
        // `normal` points from shape1 to shape2 (collider1 → collider2).
        //
        // Prefer the witness-point direction for robustness on smooth shapes; fall back to the
        // polytope face normal if the witness direction is degenerate.
        let nvec = cp1.sub(cp2)
        let n2 = nvec.length_squared()
        let mut normal = if n2 > 1.0e-18F {
          nvec.scale(1.0F / local_sqrt(n2))
        } else {
          dir
        }
        let delta = pos2.translation.sub(pos1.translation)
        if delta.length_squared() > 1.0e-12F && normal.dot(delta) < 0.0F {
          normal = normal.scale(-1.0F)
        }
        return Some({ point1: cp1, point2: cp2, normal, penetration: best.dist })
      }
      // Expand polytope.
      let new_index = verts.length()
      verts.push(v)
      let edges : Array[(Int, Int)] = []
      fn add_edge(edges : Array[(Int, Int)], a : Int, b : Int) -> Unit {
        for i in 0..<edges.length() {
          let e = edges[i]
          if e.0 == b && e.1 == a {
            edges[i] = edges[edges.length() - 1]
            edges.pop() |> ignore
            return
          }
        }
        edges.push((a, b))
      }

      let mut fi = 0
      while fi < faces.length() {
        let f = faces[fi]
        // Visible if the new point is outside this face.
        if f.normal.dot(verts[new_index].p) - f.dist > 1.0e-6F {
          add_edge(edges, f.i0, f.i1)
          add_edge(edges, f.i1, f.i2)
          add_edge(edges, f.i2, f.i0)
          faces[fi] = faces[faces.length() - 1]
          faces.pop() |> ignore
          continue
        }
        fi = fi + 1
      }
      for i in 0..<edges.length() {
        let (a, b) = edges[i]
        if epa_make_face(verts, a, b, new_index) is Some(f) {
          faces.push(f)
        }
      }
      if faces.length() == 0 {
        return None
      }
    } else {
      return None
    }
  }
  None
}

///|
pub fn compute_convex_contact(
  pos1 : @core.Isometry3,
  s1 : Shape3D,
  pos2 : @core.Isometry3,
  s2 : Shape3D,
) -> ContactPoint3D? {
  // Compound handling.
  match (s1, s2) {
    (Shape3D::Compound(parts), other) => {
      let mut best : ContactPoint3D? = None
      let mut best_pen = -1.0F
      for i in 0..<parts.length() {
        let (iso, shape) = parts[i]
        if compute_convex_contact(pos1.mul(iso), shape, pos2, other) is Some(cp) {
          if cp.penetration > best_pen {
            best_pen = cp.penetration
            best = Some(cp)
          }
        }
      }
      best
    }
    (other, Shape3D::Compound(parts)) =>
      if compute_convex_contact(pos2, Shape3D::Compound(parts), pos1, other)
        is Some(cp) {
        Some({
          point1: cp.point2,
          point2: cp.point1,
          normal: cp.normal.scale(-1.0F),
          penetration: cp.penetration,
        })
      } else {
        None
      }
    _ =>
      // Supported convex pairs: ConvexHull/Cuboid/Ball.
      if gjk_intersect(pos1, s1, pos2, s2) is Some(simplex) {
        epa_contact(pos1, s1, pos2, s2, simplex)
      } else {
        None
      }
  }
}
