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
pub fn CharacterLength::relative(value : @core.Real) -> CharacterLength {
  CharacterLength::Relative(value)
}

///|
pub fn CharacterLength::absolute(value : @core.Real) -> CharacterLength {
  CharacterLength::Absolute(value)
}

///|
fn character_length_eval(
  length : CharacterLength,
  reference : @core.Real,
) -> @core.Real {
  match length {
    CharacterLength::Relative(value) => value * reference
    CharacterLength::Absolute(value) => value
  }
}

///|
pub struct CharacterCollision {}

///|
pub struct EffectiveMovement {
  translation : @core.Vec2
  grounded : Bool
}

///|
pub fn EffectiveMovement::translation(self : EffectiveMovement) -> @core.Vec2 {
  self.translation
}

///|
pub fn EffectiveMovement::is_grounded(self : EffectiveMovement) -> Bool {
  self.grounded
}

///|
pub struct KinematicCharacterController {
  max_slope_climb_angle : @core.Real
  snap_to_ground : CharacterLength?
}

///|
pub fn KinematicCharacterController::new() -> KinematicCharacterController {
  { max_slope_climb_angle: 0.78F, snap_to_ground: None }
}

///|
pub fn KinematicCharacterController::with_max_slope_climb_angle(
  self : KinematicCharacterController,
  angle : @core.Real,
) -> KinematicCharacterController {
  { max_slope_climb_angle: angle, snap_to_ground: self.snap_to_ground }
}

///|
pub fn KinematicCharacterController::with_snap_to_ground(
  self : KinematicCharacterController,
  snap : CharacterLength?,
) -> KinematicCharacterController {
  { max_slope_climb_angle: self.max_slope_climb_angle, snap_to_ground: snap }
}

///|
fn vec2_scale(v : @core.Vec2, s : @core.Real) -> @core.Vec2 {
  @core.Vec2::new(v.x * s, v.y * s)
}

///|
fn try_normalize_and_get_length(
  v : @core.Vec2,
  eps : @core.Real,
) -> (@core.Vec2, @core.Real)? {
  let len = v.length()
  if len <= eps {
    None
  } else {
    Some((vec2_scale(v, 1.0F / len), len))
  }
}

///|
fn cos_real(angle : @core.Real) -> @core.Real {
  Float::from_double(@math.cos(angle.to_double()))
}

///|
fn compute_dims(shape : @collision.Shape) -> (@core.Real, @core.Real) {
  match shape {
    @collision.Shape::Ball(r) => (r, r)
    @collision.Shape::Cuboid(hw, hh) => (hw, hh)
  }
}

///|
fn detect_grounded(
  query : @collision.QueryPipeline,
  shape : @collision.Shape,
  position : @core.Isometry2,
  max_slope_climb_angle : @core.Real,
) -> Bool {
  let down = @core.Vec2::new(0.0F, -1.0F)
  let options = @collision.ShapeCastOptions::new(0.06F, true)
  if query.cast_shape(position, down, shape, options) is Some(hit) {
    hit.1.normal().y >= cos_real(max_slope_climb_angle)
  } else {
    false
  }
}

///|
pub fn KinematicCharacterController::move_shape(
  self : KinematicCharacterController,
  dt : @core.Real,
  query : @collision.QueryPipeline,
  shape : @collision.Shape,
  position : @core.Isometry2,
  translation : @core.Vec2,
  handle_collision : (CharacterCollision) -> Unit,
) -> EffectiveMovement {
  dt |> ignore
  let up = @core.Vec2::new(0.0F, 1.0F)
  let skin = 1.0e-3F
  let cos_max_slope = cos_real(self.max_slope_climb_angle)
  let grounded_at_start = detect_grounded(
    query,
    shape,
    position,
    self.max_slope_climb_angle,
  )
  let mut result_translation = @core.Vec2::zero()
  let mut remaining = translation
  let mut grounded = false
  let mut iters = 0
  while try_normalize_and_get_length(remaining, 1.0e-5F) is Some(step) {
    if iters >= 20 {
      break
    }
    iters = iters + 1
    let dir = step.0
    let dist = step.1
    let current_pos = @core.Isometry2::new(
      position.translation.add(result_translation),
      position.rotation,
    )
    let options = @collision.ShapeCastOptions::new(dist + skin, true)
    if query.cast_shape(current_pos, dir, shape, options) is Some(hit) {
      let toi = hit.1.toi()
      let allowed_dist = if toi > skin { toi - skin } else { 0.0F }
      let applied = vec2_scale(dir, allowed_dist)
      result_translation = result_translation.add(applied)
      remaining = remaining.sub(applied)
      handle_collision(CharacterCollision::{  })
      let normal = hit.1.normal()
      if normal.y >= cos_max_slope {
        grounded = true
      }
      let dot = remaining.dot(normal)
      if dot < 0.0F {
        remaining = remaining.sub(vec2_scale(normal, dot))
      }

      // Prevent climbing slopes steeper than `max_slope_climb_angle`.
      if normal.y > 0.0F && normal.y < cos_max_slope && remaining.y > 0.0F {
        remaining = @core.Vec2::new(remaining.x, 0.0F)
      }

      // Small nudge to avoid getting stuck on surfaces.
      result_translation = result_translation.add(vec2_scale(normal, skin))
    } else {
      result_translation = result_translation.add(remaining)
      remaining = @core.Vec2::zero()
      break
    }
    grounded = grounded ||
      detect_grounded(
        query,
        shape,
        @core.Isometry2::new(
          position.translation.add(result_translation),
          position.rotation,
        ),
        self.max_slope_climb_angle,
      )
  }
  if iters == 0 {
    grounded = detect_grounded(
      query,
      shape,
      position,
      self.max_slope_climb_angle,
    )
  }
  if grounded_at_start && !grounded && self.snap_to_ground is Some(snap_length) {
    let (_, up_extent) = compute_dims(shape)
    let snap_dist = character_length_eval(snap_length, up_extent)
    if result_translation.dot(up) < -1.0e-5F && snap_dist > 0.0F {
      let current_pos = @core.Isometry2::new(
        position.translation.add(result_translation),
        position.rotation,
      )
      let down = @core.Vec2::new(-up.x, -up.y)
      let options = @collision.ShapeCastOptions::new(snap_dist, true)
      if query.cast_shape(current_pos, down, shape, options) is Some(hit) {
        result_translation = result_translation.add(
          vec2_scale(down, hit.1.toi()),
        )
        grounded = true
      }
    }
  }
  { translation: result_translation, grounded }
}
