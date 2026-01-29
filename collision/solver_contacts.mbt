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
pub struct SolverFlags {
  mut value : Int
}

///|
const SOLVER_FLAGS_COMPUTE_IMPULSES : Int = 1 << 0

///|
pub fn SolverFlags::empty() -> SolverFlags {
  { value: 0 }
}

///|
pub fn SolverFlags::compute_impulses() -> SolverFlags {
  { value: SOLVER_FLAGS_COMPUTE_IMPULSES }
}

///|
pub fn SolverFlags::default() -> SolverFlags {
  SolverFlags::compute_impulses()
}

///|
pub fn SolverFlags::contains(self : SolverFlags, other : SolverFlags) -> Bool {
  (self.value & other.value) == other.value
}

///|
pub fn SolverFlags::insert(
  self : SolverFlags,
  other : SolverFlags,
) -> SolverFlags {
  self.value = self.value | other.value
  self
}

///|
pub fn SolverFlags::remove(
  self : SolverFlags,
  other : SolverFlags,
) -> SolverFlags {
  self.value = self.value & Int::lnot(other.value)
  self
}

///|
pub struct SolverContact {
  mut point : @core.Vec2
  mut dist : @core.Real
  mut friction : @core.Real
  mut restitution : @core.Real
  mut tangent_velocity : @core.Vec2
}

///|
pub fn SolverContact::new(
  point : @core.Vec2,
  dist : @core.Real,
) -> SolverContact {
  {
    point,
    dist,
    friction: 0.0F,
    restitution: 0.0F,
    tangent_velocity: @core.Vec2::zero(),
  }
}

///|
pub fn SolverContact::point(self : SolverContact) -> @core.Vec2 {
  self.point
}

///|
pub fn SolverContact::set_point(
  self : SolverContact,
  point : @core.Vec2,
) -> Unit {
  self.point = point
}

///|
pub fn SolverContact::dist(self : SolverContact) -> @core.Real {
  self.dist
}

///|
pub fn SolverContact::set_dist(self : SolverContact, dist : @core.Real) -> Unit {
  self.dist = dist
}

///|
pub fn SolverContact::friction(self : SolverContact) -> @core.Real {
  self.friction
}

///|
pub fn SolverContact::set_friction(
  self : SolverContact,
  friction : @core.Real,
) -> Unit {
  self.friction = friction
}

///|
pub fn SolverContact::restitution(self : SolverContact) -> @core.Real {
  self.restitution
}

///|
pub fn SolverContact::set_restitution(
  self : SolverContact,
  restitution : @core.Real,
) -> Unit {
  self.restitution = restitution
}

///|
pub fn SolverContact::tangent_velocity(self : SolverContact) -> @core.Vec2 {
  self.tangent_velocity
}

///|
pub fn SolverContact::set_tangent_velocity(
  self : SolverContact,
  tangent_velocity : @core.Vec2,
) -> Unit {
  self.tangent_velocity = tangent_velocity
}
