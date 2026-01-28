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
  // MoonBit doesn't have a bitwise-not operator, and we only need the
  // dim2 subset for now.
  let mask_values = [
    AxesMask::lin_x().bits(),
    AxesMask::lin_y().bits(),
    AxesMask::ang_z().bits(),
  ]
  let mut bits = 0
  for i in 0..<mask_values.length() {
    let bit = mask_values[i]
    if (self.bits & bit) != 0 && (other.bits & bit) == 0 {
      bits = bits | bit
    }
  }
  AxesMask::new(bits)
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
pub fn AxesMask::ang_z() -> AxesMask {
  AxesMask::new(1 << 5)
}

///|
pub fn AxesMask::lin_axes() -> AxesMask {
  AxesMask::lin_x().insert(AxesMask::lin_y())
}

///|
pub fn AxesMask::ang_axes() -> AxesMask {
  AxesMask::ang_z()
}
