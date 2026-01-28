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
pub(all) enum FillMode {
  /// Only keep voxels intersecting the input polyline segments.
  SurfaceOnly
  /// Keep surface voxels and fill the interior using a flood-fill from the volume boundary.
  ///
  /// NOTE: `detect_self_intersections` is accepted for API parity with Parry/Rapier, but this 2D
  /// port currently ignores it.
  FloodFill(Bool, Bool)
}

///|
pub fn FillMode::default() -> FillMode {
  FillMode::FloodFill(false, false)
}

///|
pub fn FillMode::surface_only() -> FillMode {
  FillMode::SurfaceOnly
}

///|
fn max_real(a : @core.Real, b : @core.Real) -> @core.Real {
  if a > b {
    a
  } else {
    b
  }
}

///|
fn min_real(a : @core.Real, b : @core.Real) -> @core.Real {
  if a < b {
    a
  } else {
    b
  }
}

///|
fn max_int(a : Int, b : Int) -> Int {
  if a > b {
    a
  } else {
    b
  }
}

///|
fn min_int(a : Int, b : Int) -> Int {
  if a < b {
    a
  } else {
    b
  }
}

///|
fn floor_div_to_int(value : @core.Real, denom : @core.Real) -> Int {
  if denom == 0.0F {
    0
  } else {
    @math.floor((value / denom).to_double()).to_int()
  }
}

///|
fn ceil_to_int(value : @core.Real) -> Int {
  @math.ceil(value.to_double()).to_int()
}

///|
fn segment_intersects_aabb(
  p0 : @core.Vec2,
  p1 : @core.Vec2,
  mins : @core.Vec2,
  maxs : @core.Vec2,
) -> Bool {
  let d = p1.sub(p0)
  let eps = 1.0e-12F
  let mut tmin = 0.0F
  let mut tmax = 1.0F
  fn update_slab(
    p0a : @core.Real,
    da : @core.Real,
    mina : @core.Real,
    maxa : @core.Real,
    tmin : @core.Real,
    tmax : @core.Real,
    eps : @core.Real,
  ) -> (@core.Real, @core.Real, Bool) {
    let mut tmin2 = tmin
    let mut tmax2 = tmax
    if @core.abs(da) < eps {
      if p0a < mina || p0a > maxa {
        (tmin2, tmax2, false)
      } else {
        (tmin2, tmax2, true)
      }
    } else {
      let inv = 1.0F / da
      let t1 = (mina - p0a) * inv
      let t2 = (maxa - p0a) * inv
      let ta = min_real(t1, t2)
      let tb = max_real(t1, t2)
      if ta > tmin2 {
        tmin2 = ta
      }
      if tb < tmax2 {
        tmax2 = tb
      }
      (tmin2, tmax2, tmin2 <= tmax2)
    }
  }

  let (tmin_x, tmax_x, ok_x) = update_slab(
    p0.x,
    d.x,
    mins.x,
    maxs.x,
    tmin,
    tmax,
    eps,
  )
  if !ok_x {
    return false
  }
  tmin = tmin_x
  tmax = tmax_x
  let (tmin_y, tmax_y, ok_y) = update_slab(
    p0.y,
    d.y,
    mins.y,
    maxs.y,
    tmin,
    tmax,
    eps,
  )
  if !ok_y {
    return false
  }
  tmin = tmin_y
  tmax = tmax_y
  tmin <= tmax
}

///|
fn collider_builder_with_shape(shape : Shape) -> ColliderBuilder {
  {
    shape,
    translation: @core.Vec2::zero(),
    rotation: 0.0F,
    active_collision_types: ActiveCollisionTypes::default(),
    sensor: false,
    active_events: ActiveEvents::empty(),
    active_hooks: ActiveHooks::empty(),
    collision_groups: @dynamics.InteractionGroups::all(),
    solver_groups: @dynamics.InteractionGroups::all(),
    density: 1.0F,
    friction: 0.5F,
    friction_combine_rule: @dynamics.CoefficientCombineRule::Average,
    restitution: 0.0F,
    restitution_combine_rule: @dynamics.CoefficientCombineRule::Average,
    contact_skin: 0.0F,
    contact_force_event_threshold: 0.0F,
    user_data: 0,
    enabled: true,
  }
}

///|
pub fn ColliderBuilder::new(shape : Shape) -> ColliderBuilder {
  collider_builder_with_shape(shape)
}

///|
fn voxel_rectangles_from_points(
  voxel_size : @core.Vec2,
  points : Array[@core.Vec2],
) -> Array[(Int, Int, Int, Int)] {
  if points.length() == 0 {
    return []
  }
  if voxel_size.x == 0.0F || voxel_size.y == 0.0F {
    return []
  }
  let coords : Array[(Int, Int)] = []
  let mut min_x = 0
  let mut max_x = 0
  let mut min_y = 0
  let mut max_y = 0
  let mut first = true
  for i in 0..<points.length() {
    let p = points[i]
    let gx = floor_div_to_int(p.x, voxel_size.x)
    let gy = floor_div_to_int(p.y, voxel_size.y)
    coords.push((gx, gy))
    if first {
      min_x = gx
      max_x = gx
      min_y = gy
      max_y = gy
      first = false
    } else {
      min_x = min_int(min_x, gx)
      max_x = max_int(max_x, gx)
      min_y = min_int(min_y, gy)
      max_y = max_int(max_y, gy)
    }
  }
  let width = max_x - min_x + 1
  let height = max_y - min_y + 1
  if width <= 0 || height <= 0 {
    return []
  }
  let occ : Array[Bool] = []
  let visited : Array[Bool] = []
  let n = width * height
  for _ in 0..<n {
    occ.push(false)
    visited.push(false)
  }
  for i in 0..<coords.length() {
    let (gx, gy) = coords[i]
    let x = gx - min_x
    let y = gy - min_y
    if x < 0 || y < 0 || x >= width || y >= height {
      continue
    }
    occ[x + y * width] = true
  }
  let rects : Array[(Int, Int, Int, Int)] = []
  for y in 0..<height {
    for x in 0..<width {
      let idx0 = x + y * width
      if !occ[idx0] || visited[idx0] {
        continue
      }
      let mut w = 1
      while x + w < width {
        let idx = x + w + y * width
        if !occ[idx] || visited[idx] {
          break
        }
        w = w + 1
      }
      let mut h = 1
      while y + h < height {
        let mut ok = true
        for xx in x..<(x + w) {
          let idx = xx + (y + h) * width
          if !occ[idx] || visited[idx] {
            ok = false
            break
          }
        }
        if !ok {
          break
        }
        h = h + 1
      }
      for yy in y..<(y + h) {
        for xx in x..<(x + w) {
          visited[xx + yy * width] = true
        }
      }
      rects.push((min_x + x, min_y + y, w, h))
    }
  }
  rects
}

///|
fn voxels_shape_from_points(
  voxel_size : @core.Vec2,
  points : Array[@core.Vec2],
) -> Shape {
  let rects = voxel_rectangles_from_points(voxel_size, points)
  let parts : Array[(@core.Isometry2, Shape)] = []
  for i in 0..<rects.length() {
    let (gx0, gy0, w, h) = rects[i]
    let cx = (
        Float::from_double(gx0.to_double()) +
        Float::from_double(w.to_double()) / 2.0F
      ) *
      voxel_size.x
    let cy = (
        Float::from_double(gy0.to_double()) +
        Float::from_double(h.to_double()) / 2.0F
      ) *
      voxel_size.y
    let hx = Float::from_double(w.to_double()) * voxel_size.x / 2.0F
    let hy = Float::from_double(h.to_double()) * voxel_size.y / 2.0F
    parts.push(
      (
        @core.Isometry2::from_translation(@core.Vec2::new(cx, cy)),
        Shape::Cuboid(hx, hy),
      ),
    )
  }
  Shape::Compound(parts)
}

///|
pub fn ColliderBuilder::voxels_from_points(
  voxel_size : @core.Vec2,
  points : Array[@core.Vec2],
) -> ColliderBuilder {
  ColliderBuilder::new(voxels_shape_from_points(voxel_size, points))
}

///|
pub fn Shape::voxelized_mesh(
  vertices : Array[@core.Vec2],
  indices : Array[(Int, Int)],
  voxel_size : @core.Real,
  fill_mode : FillMode,
) -> Shape {
  if vertices.length() == 0 || indices.length() == 0 || voxel_size <= 0.0F {
    return Shape::Compound([])
  }

  // Compute local AABB of the input polyline.
  let mut min_pt = vertices[0]
  let mut max_pt = vertices[0]
  for i in 1..<vertices.length() {
    let p = vertices[i]
    min_pt = @core.Vec2::new(min_real(min_pt.x, p.x), min_real(min_pt.y, p.y))
    max_pt = @core.Vec2::new(max_real(max_pt.x, p.x), max_real(max_pt.y, p.y))
  }
  let origin = min_pt
  let ext_x = (max_pt.x - min_pt.x) / voxel_size
  let ext_y = (max_pt.y - min_pt.y) / voxel_size
  let res_x = max_int(ceil_to_int(ext_x), 2) + 1
  let res_y = max_int(ceil_to_int(ext_y), 2) + 1
  if res_x <= 0 || res_y <= 0 {
    return Shape::Compound([])
  }

  // Dense grid of voxel states:
  // 0 = undefined, 1 = on-surface, 2 = outside (after flood fill).
  let states : Array[Int] = []
  for _ in 0..<(res_x * res_y) {
    states.push(0)
  }

  // Mark surface voxels by rasterizing each segment.
  let inv_scale = 1.0F / voxel_size
  for e in 0..<indices.length() {
    let (ia, ib) = indices[e]
    if ia < 0 || ib < 0 || ia >= vertices.length() || ib >= vertices.length() {
      continue
    }
    let a0 = vertices[ia]
    let b0 = vertices[ib]
    let da = a0.sub(origin)
    let db = b0.sub(origin)
    let a = @core.Vec2::new(da.x * inv_scale, da.y * inv_scale)
    let b = @core.Vec2::new(db.x * inv_scale, db.y * inv_scale)
    let iax = (a.x + 0.5F).to_int()
    let iay = (a.y + 0.5F).to_int()
    let ibx = (b.x + 0.5F).to_int()
    let iby = (b.y + 0.5F).to_int()
    let mut i0 = min_int(iax, ibx)
    let mut j0 = min_int(iay, iby)
    let mut i1 = max_int(iax, ibx)
    let mut j1 = max_int(iay, iby)
    i0 = max_int(i0 - 1, 0)
    j0 = max_int(j0 - 1, 0)
    i1 = min_int(i1 + 1, res_x)
    j1 = min_int(j1 + 1, res_y)
    for i in i0..<i1 {
      for j in j0..<j1 {
        let mins = @core.Vec2::new(
          Float::from_double(i.to_double()) - 0.5F,
          Float::from_double(j.to_double()) - 0.5F,
        )
        let maxs = @core.Vec2::new(
          Float::from_double(i.to_double()) + 0.5F,
          Float::from_double(j.to_double()) + 0.5F,
        )
        if segment_intersects_aabb(a, b, mins, maxs) {
          states[i + j * res_x] = 1
        }
      }
    }
  }
  match fill_mode {
    FillMode::SurfaceOnly => {
      // Keep only the on-surface voxels.
      let centers : Array[@core.Vec2] = []
      for j in 0..<res_y {
        for i in 0..<res_x {
          if states[i + j * res_x] == 1 {
            centers.push(
              origin.add(
                @core.Vec2::new(
                  Float::from_double(i.to_double()) * voxel_size,
                  Float::from_double(j.to_double()) * voxel_size,
                ),
              ),
            )
          }
        }
      }
      voxels_shape_from_points(@core.Vec2::new(voxel_size, voxel_size), centers)
    }
    FillMode::FloodFill(_, _) => {
      // Flood fill outside voxels starting from the volume boundary.
      let queue : Array[(Int, Int)] = []
      let mut qh = 0
      fn enqueue_if_undef(
        states : Array[Int],
        queue : Array[(Int, Int)],
        i : Int,
        j : Int,
        res_x : Int,
        res_y : Int,
      ) -> Unit {
        if i < 0 || j < 0 || i >= res_x || j >= res_y {
          return
        }
        let idx = i + j * res_x
        if states[idx] == 0 {
          states[idx] = 2
          queue.push((i, j))
        }
      }

      // Mark boundary undefined voxels as outside seeds.
      for i in 0..<res_x {
        enqueue_if_undef(states, queue, i, 0, res_x, res_y)
        enqueue_if_undef(states, queue, i, res_y - 1, res_x, res_y)
      }
      for j in 0..<res_y {
        enqueue_if_undef(states, queue, 0, j, res_x, res_y)
        enqueue_if_undef(states, queue, res_x - 1, j, res_x, res_y)
      }
      while qh < queue.length() {
        let (ci, cj) = queue[qh]
        qh = qh + 1
        enqueue_if_undef(states, queue, ci + 1, cj, res_x, res_y)
        enqueue_if_undef(states, queue, ci - 1, cj, res_x, res_y)
        enqueue_if_undef(states, queue, ci, cj + 1, res_x, res_y)
        enqueue_if_undef(states, queue, ci, cj - 1, res_x, res_y)
      }
      let centers : Array[@core.Vec2] = []
      for j in 0..<res_y {
        for i in 0..<res_x {
          let v = states[i + j * res_x]
          if v == 1 || v == 0 {
            centers.push(
              origin.add(
                @core.Vec2::new(
                  Float::from_double(i.to_double()) * voxel_size,
                  Float::from_double(j.to_double()) * voxel_size,
                ),
              ),
            )
          }
        }
      }
      voxels_shape_from_points(@core.Vec2::new(voxel_size, voxel_size), centers)
    }
  }
}

///|
pub fn ColliderBuilder::voxelized_mesh(
  vertices : Array[@core.Vec2],
  indices : Array[(Int, Int)],
  voxel_size : @core.Real,
  fill_mode : FillMode,
) -> ColliderBuilder {
  ColliderBuilder::new(
    Shape::voxelized_mesh(vertices, indices, voxel_size, fill_mode),
  )
}
