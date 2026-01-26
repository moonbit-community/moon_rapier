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
/// Ported from rapier/src/dynamics/rigid_body_components.rs (test_interpolate_velocity).
fn rb_lcg_next(seed : Int) -> Int {
  let modulus = 65536
  let next = (seed * 25173 + 13849) % modulus
  if next < 0 {
    next + modulus
  } else {
    next
  }
}

///|
fn rb_rand_real(seed : Int) -> (Int, @core.Real) {
  let next = rb_lcg_next(seed)
  let value = Float::from_double(next.to_double() / 65536.0)
  (next, value)
}

///|
test "interpolate velocity" {
  let dt = 0.016F
  let mut seed = 0
  for offset in 0..<21 {
    let multiplier = Float::from_double((offset - 10).to_double())
    let (seed1, local_x) = rb_rand_real(seed)
    let (seed2, local_y) = rb_rand_real(seed1)
    let (seed3, curr_x) = rb_rand_real(seed2)
    let (seed4, curr_y) = rb_rand_real(seed3)
    let (seed5, curr_angle) = rb_rand_real(seed4)
    let (seed6, next_x) = rb_rand_real(seed5)
    let (seed7, next_y) = rb_rand_real(seed6)
    let (seed8, next_angle) = rb_rand_real(seed7)
    seed = seed8
    let local_com = @core.Vec2::new(local_x, local_y)
    let curr_pos = @core.Isometry2::new(
      @core.Vec2::new(curr_x * multiplier, curr_y * multiplier),
      @core.Rot2::from_angle(curr_angle),
    )
    let next_pos = @core.Isometry2::new(
      @core.Vec2::new(next_x * multiplier, next_y * multiplier),
      @core.Rot2::from_angle(next_angle),
    )
    let rb_pos = RigidBodyPosition::new(curr_pos, next_pos)
    let vel = rb_pos.interpolate_velocity(1.0F / dt, local_com)
    let interp_pos = vel.integrate(dt, curr_pos, local_com)
    let dx = @core.abs(interp_pos.translation.x - next_pos.translation.x)
    let dy = @core.abs(interp_pos.translation.y - next_pos.translation.y)
    let dsin = @core.abs(interp_pos.rotation.sin - next_pos.rotation.sin)
    let dcos = @core.abs(interp_pos.rotation.cos - next_pos.rotation.cos)
    inspect(dx < 1.0e-5F, content="true")
    inspect(dy < 1.0e-5F, content="true")
    inspect(dsin < 1.0e-5F, content="true")
    inspect(dcos < 1.0e-5F, content="true")
  }
}
