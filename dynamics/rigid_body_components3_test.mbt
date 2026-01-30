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
/// Ported from rapier/src/dynamics/rigid_body_components.rs (test_interpolate_velocity) for dim3.
fn rb3_lcg_next(seed : Int) -> Int {
  let modulus = 65536
  let next = (seed * 25173 + 13849) % modulus
  if next < 0 {
    next + modulus
  } else {
    next
  }
}

///|
fn rb3_rand_real(seed : Int) -> (Int, @core.Real) {
  let next = rb3_lcg_next(seed)
  let value = Float::from_double(next.to_double() / 65536.0)
  (next, value)
}

///|
fn quat_max_abs_diff_sign_invariant(
  left : @core.Quat,
  right : @core.Quat,
) -> @core.Real {
  let mut r = right
  if left.dot(right) < 0.0F {
    r = @core.Quat::new(-right.x, -right.y, -right.z, -right.w)
  }
  let dx = @core.abs(left.x - r.x)
  let dy = @core.abs(left.y - r.y)
  let dz = @core.abs(left.z - r.z)
  let dw = @core.abs(left.w - r.w)
  let mut max_diff = dx
  if dy > max_diff {
    max_diff = dy
  }
  if dz > max_diff {
    max_diff = dz
  }
  if dw > max_diff {
    max_diff = dw
  }
  max_diff
}

///|
test "interpolate velocity (dim3)" {
  let dt = 0.016F
  let mut seed = 0
  for offset in 0..<21 {
    let multiplier = Float::from_double((offset - 10).to_double())
    let (seed1, local_x) = rb3_rand_real(seed)
    let (seed2, local_y) = rb3_rand_real(seed1)
    let (seed3, local_z) = rb3_rand_real(seed2)
    let (seed4, curr_x) = rb3_rand_real(seed3)
    let (seed5, curr_y) = rb3_rand_real(seed4)
    let (seed6, curr_z) = rb3_rand_real(seed5)
    let (seed7, curr_rx) = rb3_rand_real(seed6)
    let (seed8, curr_ry) = rb3_rand_real(seed7)
    let (seed9, curr_rz) = rb3_rand_real(seed8)
    let (seed10, next_x) = rb3_rand_real(seed9)
    let (seed11, next_y) = rb3_rand_real(seed10)
    let (seed12, next_z) = rb3_rand_real(seed11)
    let (seed13, next_rx) = rb3_rand_real(seed12)
    let (seed14, next_ry) = rb3_rand_real(seed13)
    let (seed15, next_rz) = rb3_rand_real(seed14)
    seed = seed15
    let local_com = @core.Vec3::new(local_x, local_y, local_z)
    let curr_pos = @core.Isometry3::new(
      @core.Vec3::new(
        curr_x * multiplier,
        curr_y * multiplier,
        curr_z * multiplier,
      ),
      @core.rotation_from_scaled_axis(
        @core.Vec3::new(curr_rx, curr_ry, curr_rz),
      ),
    )
    let next_pos = @core.Isometry3::new(
      @core.Vec3::new(
        next_x * multiplier,
        next_y * multiplier,
        next_z * multiplier,
      ),
      @core.rotation_from_scaled_axis(
        @core.Vec3::new(next_rx, next_ry, next_rz),
      ),
    )
    let rb_pos = RigidBodyPosition3::new(curr_pos, next_pos)
    let vel = rb_pos.interpolate_velocity(1.0F / dt, local_com)
    let interp_pos = vel.integrate(dt, curr_pos, local_com)
    let dx = @core.abs(interp_pos.translation.x - next_pos.translation.x)
    let dy = @core.abs(interp_pos.translation.y - next_pos.translation.y)
    let dz = @core.abs(interp_pos.translation.z - next_pos.translation.z)
    let drot = quat_max_abs_diff_sign_invariant(
      interp_pos.rotation,
      next_pos.rotation,
    )
    inspect(dx < 1.0e-5F, content="true")
    inspect(dy < 1.0e-5F, content="true")
    inspect(dz < 1.0e-5F, content="true")
    inspect(drot < 1.0e-5F, content="true")
  }
}
