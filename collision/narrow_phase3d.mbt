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
pub fn compute_ball_ball_contact(
  center1 : @core.Vec3,
  radius1 : @core.Real,
  center2 : @core.Vec3,
  radius2 : @core.Real,
) -> ContactPoint3D? {
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
