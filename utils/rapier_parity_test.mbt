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
test "utils dim constants + copy_sign_to" {
  inspect(DIM_MINUS_ONE2 == 1, content="true")
  inspect(DIM_MINUS_ONE3 == 2, content="true")
  inspect(copy_sign_to(-2.0F, 1.0F) == -1.0F, content="true")
  inspect(copy_sign_to(2.0F, -3.0F) == 3.0F, content="true")
}

///|
test "utils smallest_abs_diff_between_angles chooses shortest path" {
  let a = 0.1F
  let b = 6.2F
  let d = smallest_abs_diff_between_angles(a, b)
  // Should wrap near 2*pi.
  inspect(@core.abs(d - 0.1831853F) < 1.0e-3F, content="true")
}

///|
test "utils try_normalize_and_get_length2/3" {
  let v2 = @core.Vec2::new(3.0F, 4.0F)
  match try_normalize_and_get_length2(v2, 0.1F) {
    None => inspect(false, content="true")
    Some((n, len)) => {
      inspect(@core.abs(len - 5.0F) < 1.0e-6F, content="true")
      inspect(@core.abs(n.length() - 1.0F) < 1.0e-6F, content="true")
    }
  }
  let v3 = @core.Vec3::new(0.0F, 3.0F, 4.0F)
  match try_normalize_and_get_length3(v3, 0.1F) {
    None => inspect(false, content="true")
    Some((n, len)) => {
      inspect(@core.abs(len - 5.0F) < 1.0e-6F, content="true")
      inspect(@core.abs(n.length() - 1.0F) < 1.0e-6F, content="true")
    }
  }
}

///|
test "utils orthonormal_basis3 stable case" {
  let v = @core.Vec3::new(0.0F, 0.0F, 1.0F)
  let b = orthonormal_basis3(v)
  inspect(b.length() == 2, content="true")
  inspect(b[0].x == 1.0F && b[0].y == 0.0F && b[0].z == 0.0F, content="true")
  inspect(b[1].x == 0.0F && b[1].y == 1.0F && b[1].z == 0.0F, content="true")
}

///|
test "utils gcross_matrix3 layout" {
  let v = @core.Vec3::new(1.0F, 2.0F, 3.0F)
  let m = gcross_matrix3(v)
  inspect(m.m00 == 0.0F && m.m01 == -3.0F && m.m02 == 2.0F, content="true")
  inspect(m.m10 == 3.0F && m.m11 == 0.0F && m.m12 == -1.0F, content="true")
  inspect(m.m20 == -2.0F && m.m21 == 1.0F && m.m22 == 0.0F, content="true")
}

///|
test "utils quat diff_conj1_2 transpose matches" {
  let q1 = @core.Quat::identity()
  let q2 = @core.rotation_from_scaled_axis(@core.Vec3::new(0.1F, 0.2F, 0.3F))
  let a = quat_diff_conj1_2(q1, q2)
  let b = quat_diff_conj1_2_tr(q1, q2)
  inspect(
    a.transpose().m00 == b.m00 && a.transpose().m12 == b.m12,
    content="true",
  )
}
