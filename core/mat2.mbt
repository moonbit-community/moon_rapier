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
pub struct Mat2 {
  m00 : Real
  m01 : Real
  m10 : Real
  m11 : Real
}

///|
pub fn Mat2::new(m00 : Real, m01 : Real, m10 : Real, m11 : Real) -> Mat2 {
  Mat2::{ m00, m01, m10, m11 }
}

///|
pub fn Mat2::zero() -> Mat2 {
  Mat2::new(0.0F, 0.0F, 0.0F, 0.0F)
}

///|
pub fn Mat2::identity() -> Mat2 {
  Mat2::new(1.0F, 0.0F, 0.0F, 1.0F)
}

///|
pub fn Mat2::add(self : Mat2, other : Mat2) -> Mat2 {
  Mat2::new(
    self.m00 + other.m00,
    self.m01 + other.m01,
    self.m10 + other.m10,
    self.m11 + other.m11,
  )
}

///|
pub fn Mat2::sub(self : Mat2, other : Mat2) -> Mat2 {
  Mat2::new(
    self.m00 - other.m00,
    self.m01 - other.m01,
    self.m10 - other.m10,
    self.m11 - other.m11,
  )
}

///|
pub fn Mat2::mul(self : Mat2, other : Mat2) -> Mat2 {
  Mat2::new(
    self.m00 * other.m00 + self.m01 * other.m10,
    self.m00 * other.m01 + self.m01 * other.m11,
    self.m10 * other.m00 + self.m11 * other.m10,
    self.m10 * other.m01 + self.m11 * other.m11,
  )
}

///|
pub fn Mat2::mul_vec2(self : Mat2, v : Vec2) -> Vec2 {
  Vec2::new(self.m00 * v.x + self.m01 * v.y, self.m10 * v.x + self.m11 * v.y)
}

///|
pub fn Mat2::transpose(self : Mat2) -> Mat2 {
  Mat2::new(self.m00, self.m10, self.m01, self.m11)
}

///|
pub fn Mat2::determinant(self : Mat2) -> Real {
  self.m00 * self.m11 - self.m01 * self.m10
}

///|
pub fn Mat2::inverse(self : Mat2) -> Mat2 {
  let det = self.determinant()
  if abs(det) <= 0.0F {
    Mat2::zero()
  } else {
    let inv = 1.0F / det
    Mat2::new(self.m11 * inv, -self.m01 * inv, -self.m10 * inv, self.m00 * inv)
  }
}
