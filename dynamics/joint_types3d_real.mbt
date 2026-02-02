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
/// Joint axis types for the real dim3 backend.

///|
pub(all) enum JointAxis3DReal {
  LinX
  LinY
  LinZ
  AngX
  AngY
  AngZ
}

///|
pub struct JointAxesMask3DReal {
  bits : Int
}

///|
pub fn JointAxesMask3DReal::empty() -> JointAxesMask3DReal {
  { bits: 0 }
}

///|
pub fn JointAxesMask3DReal::new(bits : Int) -> JointAxesMask3DReal {
  { bits, }
}

///|
pub fn JointAxesMask3DReal::bits(self : JointAxesMask3DReal) -> Int {
  self.bits
}

///|
pub fn JointAxesMask3DReal::contains(
  self : JointAxesMask3DReal,
  other : JointAxesMask3DReal,
) -> Bool {
  (self.bits & other.bits) == other.bits
}

///|
pub fn JointAxesMask3DReal::or(
  self : JointAxesMask3DReal,
  other : JointAxesMask3DReal,
) -> JointAxesMask3DReal {
  { bits: self.bits | other.bits }
}

///|
pub fn JointAxesMask3DReal::lin_x() -> JointAxesMask3DReal {
  { bits: 1 << 0 }
}

///|
pub fn JointAxesMask3DReal::lin_y() -> JointAxesMask3DReal {
  { bits: 1 << 1 }
}

///|
pub fn JointAxesMask3DReal::lin_z() -> JointAxesMask3DReal {
  { bits: 1 << 2 }
}

///|
pub fn JointAxesMask3DReal::ang_x() -> JointAxesMask3DReal {
  { bits: 1 << 3 }
}

///|
pub fn JointAxesMask3DReal::ang_y() -> JointAxesMask3DReal {
  { bits: 1 << 4 }
}

///|
pub fn JointAxesMask3DReal::ang_z() -> JointAxesMask3DReal {
  { bits: 1 << 5 }
}

///|
pub fn JointAxesMask3DReal::lin_axes() -> JointAxesMask3DReal {
  JointAxesMask3DReal::lin_x()
  .or(JointAxesMask3DReal::lin_y())
  .or(JointAxesMask3DReal::lin_z())
}

///|
pub fn JointAxesMask3DReal::ang_axes() -> JointAxesMask3DReal {
  JointAxesMask3DReal::ang_x()
  .or(JointAxesMask3DReal::ang_y())
  .or(JointAxesMask3DReal::ang_z())
}

///|
pub fn JointAxesMask3DReal::all() -> JointAxesMask3DReal {
  JointAxesMask3DReal::lin_axes().or(JointAxesMask3DReal::ang_axes())
}
