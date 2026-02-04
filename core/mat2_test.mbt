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
test "mat2 mul_vec2" {
  let m = Mat2::new(1.0F, 2.0F, 3.0F, 4.0F)
  let v = Vec2::new(5.0F, 6.0F)
  let out = m.mul_vec2(v)
  inspect(out.x.to_int(), content="17")
  inspect(out.y.to_int(), content="39")
}

///|
test "mat2 inverse roundtrip" {
  let m = Mat2::new(2.0F, 1.0F, 3.0F, 4.0F)
  let inv = m.inverse()
  let prod = m.mul(inv)
  inspect(abs(prod.m00 - 1.0F) < 1.0e-5F, content="true")
  inspect(abs(prod.m01 - 0.0F) < 1.0e-5F, content="true")
  inspect(abs(prod.m10 - 0.0F) < 1.0e-5F, content="true")
  inspect(abs(prod.m11 - 1.0F) < 1.0e-5F, content="true")
}
