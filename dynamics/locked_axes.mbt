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
/// Leaf-name shims for Rapier's `LockedAxes` bitflags.
///
/// This port's 2D rigid-body implementation currently uses coarse lock flags
/// (`locked_translations` / `locked_rotations`). We still expose `LockedAxes`
/// so the public surface matches Rapier's default-feature crates.
pub struct LockedAxes {
  bits : Int
}

///|
pub fn LockedAxes::new(bits : Int) -> LockedAxes {
  { bits, }
}

///|
pub fn LockedAxes::empty() -> LockedAxes {
  LockedAxes::new(0)
}

///|
pub fn LockedAxes::bits(self : LockedAxes) -> Int {
  self.bits
}

///|
pub fn LockedAxes::contains(self : LockedAxes, other : LockedAxes) -> Bool {
  (self.bits & other.bits) == other.bits
}

///|
pub fn LockedAxes::insert(self : LockedAxes, other : LockedAxes) -> LockedAxes {
  LockedAxes::new(self.bits | other.bits)
}

///|
pub fn LockedAxes::remove(self : LockedAxes, other : LockedAxes) -> LockedAxes {
  LockedAxes::new(self.bits & Int::lnot(other.bits))
}

///|
const LOCKED_TRANSLATION_X : Int = 1 << 0

///|
const LOCKED_TRANSLATION_Y : Int = 1 << 1

///|
const LOCKED_TRANSLATION_Z : Int = 1 << 2

///|
const LOCKED_ROTATION_X : Int = 1 << 3

///|
const LOCKED_ROTATION_Y : Int = 1 << 4

///|
const LOCKED_ROTATION_Z : Int = 1 << 5

///|
pub fn LockedAxes::translation_locked_x() -> LockedAxes {
  LockedAxes::new(LOCKED_TRANSLATION_X)
}

///|
pub fn LockedAxes::translation_locked_y() -> LockedAxes {
  LockedAxes::new(LOCKED_TRANSLATION_Y)
}

///|
pub fn LockedAxes::translation_locked_z() -> LockedAxes {
  LockedAxes::new(LOCKED_TRANSLATION_Z)
}

///|
pub fn LockedAxes::rotation_locked_x() -> LockedAxes {
  LockedAxes::new(LOCKED_ROTATION_X)
}

///|
pub fn LockedAxes::rotation_locked_y() -> LockedAxes {
  LockedAxes::new(LOCKED_ROTATION_Y)
}

///|
pub fn LockedAxes::rotation_locked_z() -> LockedAxes {
  LockedAxes::new(LOCKED_ROTATION_Z)
}

///|
pub fn LockedAxes::translation_locked() -> LockedAxes {
  LockedAxes::translation_locked_x()
  .insert(LockedAxes::translation_locked_y())
  .insert(LockedAxes::translation_locked_z())
}

///|
pub fn LockedAxes::rotation_locked() -> LockedAxes {
  LockedAxes::rotation_locked_x()
  .insert(LockedAxes::rotation_locked_y())
  .insert(LockedAxes::rotation_locked_z())
}
