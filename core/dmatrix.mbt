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
/// A minimal dynamic matrix type used for Rapier public-surface parity.
///
/// This is not a full nalgebra replacement; it is a small, deterministic,
/// allocation-backed matrix for internal use and for parity mapping.
pub struct DMatrix {
  rows : Int
  cols : Int
  data : Array[Real]
}

///|
fn index(rows : Int, cols : Int, r : Int, c : Int) -> Int? {
  if r < 0 || c < 0 || r >= rows || c >= cols {
    None
  } else {
    Some(r * cols + c)
  }
}

///|
pub fn DMatrix::new(rows : Int, cols : Int, init : Real) -> DMatrix {
  let data : Array[Real] = []
  let n = if rows < 0 || cols < 0 { 0 } else { rows * cols }
  for _ in 0..<n {
    data.push(init)
  }
  { rows, cols, data }
}

///|
pub fn DMatrix::zeros(rows : Int, cols : Int) -> DMatrix {
  DMatrix::new(rows, cols, 0.0F)
}

///|
pub fn DMatrix::identity(n : Int) -> DMatrix {
  let mut m = DMatrix::zeros(n, n)
  for i in 0..<n {
    m = m.set(i, i, 1.0F)
  }
  m
}

///|
pub fn DMatrix::rows(self : DMatrix) -> Int {
  self.rows
}

///|
pub fn DMatrix::cols(self : DMatrix) -> Int {
  self.cols
}

///|
pub fn DMatrix::get(self : DMatrix, r : Int, c : Int) -> Real? {
  if index(self.rows, self.cols, r, c) is Some(i) {
    Some(self.data[i])
  } else {
    None
  }
}

///|
pub fn DMatrix::set(self : DMatrix, r : Int, c : Int, v : Real) -> DMatrix {
  if index(self.rows, self.cols, r, c) is Some(i) {
    self.data[i] = v
  }
  self
}
