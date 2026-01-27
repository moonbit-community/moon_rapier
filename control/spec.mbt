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
pub(all) enum CharacterLength {
  Relative(@core.Real)
  Absolute(@core.Real)
}

///|
#declaration_only
pub fn CharacterLength::relative(value : @core.Real) -> CharacterLength {
  ...
}

///|
#declaration_only
pub fn CharacterLength::absolute(value : @core.Real) -> CharacterLength {
  ...
}

///|
#declaration_only
pub type CharacterCollision

///|
#declaration_only
pub fn CharacterCollision::handle(
  self : CharacterCollision,
) -> @collision.ColliderHandle {
  ...
}

///|
#declaration_only
pub fn CharacterCollision::hit(
  self : CharacterCollision,
) -> @collision.ShapeCastHit {
  ...
}

///|
#declaration_only
pub fn CharacterCollision::translation_applied(
  self : CharacterCollision,
) -> @core.Vec2 {
  ...
}

///|
#declaration_only
pub fn CharacterCollision::translation_remaining(
  self : CharacterCollision,
) -> @core.Vec2 {
  ...
}

///|
#declaration_only
pub fn CharacterCollision::character_pos(
  self : CharacterCollision,
) -> @core.Isometry2 {
  ...
}

///|
#declaration_only
pub type EffectiveMovement

///|
#declaration_only
pub fn EffectiveMovement::translation(self : EffectiveMovement) -> @core.Vec2 {
  ...
}

///|
#declaration_only
pub fn EffectiveMovement::is_grounded(self : EffectiveMovement) -> Bool {
  ...
}

///|
#declaration_only
pub type CharacterAutostep

///|
#declaration_only
pub fn CharacterAutostep::new(
  max_height : CharacterLength,
  min_width : CharacterLength,
) -> CharacterAutostep {
  ...
}

///|
#declaration_only
pub fn CharacterAutostep::max_height(
  self : CharacterAutostep,
) -> CharacterLength {
  ...
}

///|
#declaration_only
pub fn CharacterAutostep::min_width(
  self : CharacterAutostep,
) -> CharacterLength {
  ...
}

///|
#declaration_only
pub type KinematicCharacterController

///|
#declaration_only
pub fn KinematicCharacterController::new() -> KinematicCharacterController {
  ...
}

///|
#declaration_only
pub fn KinematicCharacterController::with_max_slope_climb_angle(
  self : KinematicCharacterController,
  angle : @core.Real,
) -> KinematicCharacterController {
  ...
}

///|
#declaration_only
pub fn KinematicCharacterController::with_snap_to_ground(
  self : KinematicCharacterController,
  snap : CharacterLength?,
) -> KinematicCharacterController {
  ...
}

///|
#declaration_only
pub fn KinematicCharacterController::with_autostep(
  self : KinematicCharacterController,
  autostep : CharacterAutostep?,
) -> KinematicCharacterController {
  ...
}

///|
#declaration_only
pub fn KinematicCharacterController::move_shape(
  self : KinematicCharacterController,
  dt : @core.Real,
  query : @collision.QueryPipeline,
  shape : @collision.Shape,
  position : @core.Isometry2,
  translation : @core.Vec2,
  handle_collision : (CharacterCollision) -> Unit,
) -> EffectiveMovement {
  ...
}

///|
#declaration_only
pub fn KinematicCharacterController::solve_character_collision_impulses(
  self : KinematicCharacterController,
  dt : @core.Real,
  query : @collision.QueryPipeline,
  character_shape : @collision.Shape,
  character_mass : @core.Real,
  collisions : Array[CharacterCollision],
) -> Unit {
  ...
}
