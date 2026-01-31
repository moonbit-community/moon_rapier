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
test "mat3 identity mul_vec3" {
  let m = Mat3::identity()
  let v = Vec3::new(1.0F, -2.0F, 3.5F)
  let r = m.mul_vec3(v)
  inspect(abs(r.x - v.x) < 1.0e-6F, content="true")
  inspect(abs(r.y - v.y) < 1.0e-6F, content="true")
  inspect(abs(r.z - v.z) < 1.0e-6F, content="true")
}

///|
test "mat3 inverse roundtrip" {
  let m = Mat3::new(2.0F, 1.0F, 0.0F, -1.0F, 3.0F, 1.0F, 0.5F, 0.0F, 4.0F)
  let inv = m.inverse()
  let id = m.mul(inv)
  inspect(abs(id.m00 - 1.0F) < 1.0e-4F, content="true")
  inspect(abs(id.m11 - 1.0F) < 1.0e-4F, content="true")
  inspect(abs(id.m22 - 1.0F) < 1.0e-4F, content="true")
  inspect(abs(id.m01) < 1.0e-4F, content="true")
  inspect(abs(id.m02) < 1.0e-4F, content="true")
  inspect(abs(id.m10) < 1.0e-4F, content="true")
  inspect(abs(id.m12) < 1.0e-4F, content="true")
  inspect(abs(id.m20) < 1.0e-4F, content="true")
  inspect(abs(id.m21) < 1.0e-4F, content="true")
}

///|
test "sdpmat3 inverse diagonal" {
  let i = SdpMat3::from_diagonal(Vec3::new(2.0F, 3.0F, 4.0F))
  let inv = i.inverse()
  let v = Vec3::new(6.0F, 12.0F, 8.0F)
  let r = inv.transform_vec3(v)
  inspect(abs(r.x - 3.0F) < 1.0e-6F, content="true")
  inspect(abs(r.y - 4.0F) < 1.0e-6F, content="true")
  inspect(abs(r.z - 2.0F) < 1.0e-6F, content="true")
}

///|
test "quat to_mat3 consistent with rotate_vec3" {
  let q = rotation_from_scaled_axis(Vec3::new(0.3F, -0.7F, 0.1F))
  let m = q.to_mat3()
  let v = Vec3::new(1.5F, 0.25F, -2.0F)
  let r0 = q.rotate_vec3(v)
  let r1 = m.mul_vec3(v)
  inspect(abs(r0.x - r1.x) < 1.0e-5F, content="true")
  inspect(abs(r0.y - r1.y) < 1.0e-5F, content="true")
  inspect(abs(r0.z - r1.z) < 1.0e-5F, content="true")
}
