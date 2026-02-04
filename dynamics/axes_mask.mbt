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
pub struct AxesMask {
  bits : Int
}

///|
pub fn AxesMask::empty() -> AxesMask {
  { bits: 0 }
}

///|
pub fn AxesMask::all() -> AxesMask {
  AxesMask::lin_axes().insert(AxesMask::ang_axes())
}

///|
pub fn AxesMask::new(bits : Int) -> AxesMask {
  { bits, }
}

///|
pub fn AxesMask::bits(self : AxesMask) -> Int {
  self.bits
}

///|
pub fn AxesMask::contains(self : AxesMask, other : AxesMask) -> Bool {
  (self.bits & other.bits) == other.bits
}

///|
pub fn AxesMask::insert(self : AxesMask, other : AxesMask) -> AxesMask {
  AxesMask::new(self.bits | other.bits)
}

///|
pub fn AxesMask::remove(self : AxesMask, other : AxesMask) -> AxesMask {
  AxesMask::new(self.bits & Int::lnot(other.bits))
}

///|
pub fn AxesMask::lin_x() -> AxesMask {
  AxesMask::new(1 << 0)
}

///|
pub fn AxesMask::lin_y() -> AxesMask {
  AxesMask::new(1 << 1)
}

///|
pub fn AxesMask::lin_z() -> AxesMask {
  AxesMask::new(1 << 2)
}

///|
pub fn AxesMask::ang_x() -> AxesMask {
  AxesMask::new(1 << 3)
}

///|
pub fn AxesMask::ang_y() -> AxesMask {
  AxesMask::new(1 << 4)
}

///|
pub fn AxesMask::ang_z() -> AxesMask {
  AxesMask::new(1 << 5)
}

///|
pub fn AxesMask::lin_axes() -> AxesMask {
  AxesMask::lin_x().insert(AxesMask::lin_y()).insert(AxesMask::lin_z())
}

///|
pub fn AxesMask::ang_axes() -> AxesMask {
  AxesMask::ang_x().insert(AxesMask::ang_y()).insert(AxesMask::ang_z())
}
