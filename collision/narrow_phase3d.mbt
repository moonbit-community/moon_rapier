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
