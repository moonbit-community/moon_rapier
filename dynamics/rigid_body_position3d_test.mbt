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
/// Ported from `rapier-reference/src/dynamics/rigid_body_components.rs::test_interpolate_velocity`
/// (cfg dim3).
test "rigid_body_position3d interpolate_velocity roundtrip" {
  // Deterministic xorshift RNG.
  let mut state : Int = 0x12345678
  fn next_u32() -> Int {
    state = state ^ (state << 13)
    state = state ^ (state >> 17)
    state = state ^ (state << 5)
    state
  }

  fn next_f() -> @core.Real {
    // Map to [0, 1).
    // 24-bit mantissa scale to keep determinism.
    let masked = next_u32() & 0x00ff_ffff
    Float::from_int(masked) / Float::from_int(0x0100_0000)
  }

  fn rand_vec3() -> @core.Vec3 {
    @core.Vec3::new(next_f(), next_f(), next_f())
  }

  for i in -10..<=10 {
    let mult = Float::from_int(i)
    let local_com = rand_vec3()
    let curr_pos = @core.Isometry3::new(
      rand_vec3().scale(mult),
      @core.rotation_from_scaled_axis(rand_vec3()),
    )
    let next_pos = @core.Isometry3::new(
      rand_vec3().scale(mult),
      @core.rotation_from_scaled_axis(rand_vec3()),
    )
    let dt = 0.016F
    let inv_dt = 1.0F / dt
    let rb_pos = RigidBodyPosition3D::new(curr_pos, next_pos)
    let vel = rb_pos.interpolate_velocity(inv_dt, local_com)
    let interp_pos = vel.integrate(dt, curr_pos, local_com)
    inspect(
      @core.abs(interp_pos.translation.x - next_pos.translation.x) < 1.0e-5F,
      content="true",
    )
    inspect(
      @core.abs(interp_pos.translation.y - next_pos.translation.y) < 1.0e-5F,
      content="true",
    )
    inspect(
      @core.abs(interp_pos.translation.z - next_pos.translation.z) < 1.0e-5F,
      content="true",
    )

    // Rotation equality up to sign.
    let d = interp_pos.rotation.dot(next_pos.rotation)
    inspect(@core.abs(@core.abs(d) - 1.0F) < 1.0e-4F, content="true")
  }
}
