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
/// A Rapier-compatible `u128`-equivalent payload used by `RigidBody`/`Collider` user data.
///
/// Representation is `(hi, lo)` where the numeric value is `(hi << 64) | lo`.
pub struct UserData128 {
  hi : UInt64
  lo : UInt64
}

///|
pub fn UserData128::zero() -> UserData128 {
  { hi: UInt64::default(), lo: UInt64::default() }
}

///|
pub fn UserData128::from_parts(hi : UInt64, lo : UInt64) -> UserData128 {
  { hi, lo }
}

///|
pub fn UserData128::from_u64(value : UInt64) -> UserData128 {
  { hi: UInt64::default(), lo: value }
}

///|
/// Back-compat helper: interpret the given `Int` as an unsigned payload truncated to 64 bits,
/// stored in the low 64 bits of this `UserData128`.
pub fn UserData128::from_int_truncate(value : Int) -> UserData128 {
  let lo = UInt64::extend_uint(value.reinterpret_as_uint())
  UserData128::from_u64(lo)
}

///|
/// Back-compat helper: return the low 64 bits truncated to `Int`.
pub fn UserData128::to_int_truncate(self : UserData128) -> Int {
  self.lo.to_int()
}

///|
pub fn UserData128::hi(self : UserData128) -> UInt64 {
  self.hi
}

///|
pub fn UserData128::lo(self : UserData128) -> UInt64 {
  self.lo
}

///|
pub fn UserData128::to_parts(self : UserData128) -> (UInt64, UInt64) {
  (self.hi, self.lo)
}

///|
pub fn UserData128::equals(self : UserData128, other : UserData128) -> Bool {
  UInt64::equal(self.hi, other.hi) && UInt64::equal(self.lo, other.lo)
}
