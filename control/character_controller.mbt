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
pub struct CharacterCollision {
  handle : @collision.ColliderHandle
  hit : @collision.ShapeCastHit
  translation_applied : @core.Vec2
  translation_remaining : @core.Vec2
  character_pos : @core.Isometry2
}

///|
pub fn CharacterCollision::handle(
  self : CharacterCollision,
) -> @collision.ColliderHandle {
  self.handle
}

///|
pub fn CharacterCollision::hit(
  self : CharacterCollision,
) -> @collision.ShapeCastHit {
  self.hit
}

///|
pub fn CharacterCollision::translation_applied(
  self : CharacterCollision,
) -> @core.Vec2 {
  self.translation_applied
}

///|
pub fn CharacterCollision::translation_remaining(
  self : CharacterCollision,
) -> @core.Vec2 {
  self.translation_remaining
}

///|
pub fn CharacterCollision::character_pos(
  self : CharacterCollision,
) -> @core.Isometry2 {
  self.character_pos
}

///|
pub struct CharacterAutostep {
  max_height : CharacterLength
  min_width : CharacterLength
}

///|
pub fn CharacterAutostep::new(
  max_height : CharacterLength,
  min_width : CharacterLength,
) -> CharacterAutostep {
  { max_height, min_width }
}

///|
pub fn CharacterAutostep::max_height(
  self : CharacterAutostep,
) -> CharacterLength {
  self.max_height
}

///|
pub fn CharacterAutostep::min_width(
  self : CharacterAutostep,
) -> CharacterLength {
  self.min_width
}

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
  autostep : CharacterAutostep?
}

///|
pub fn KinematicCharacterController::new() -> KinematicCharacterController {
  { max_slope_climb_angle: 0.78F, snap_to_ground: None, autostep: None }
}

///|
pub fn KinematicCharacterController::with_max_slope_climb_angle(
  self : KinematicCharacterController,
  angle : @core.Real,
) -> KinematicCharacterController {
  {
    max_slope_climb_angle: angle,
    snap_to_ground: self.snap_to_ground,
    autostep: self.autostep,
  }
}

///|
pub fn KinematicCharacterController::with_snap_to_ground(
  self : KinematicCharacterController,
  snap : CharacterLength?,
) -> KinematicCharacterController {
  {
    max_slope_climb_angle: self.max_slope_climb_angle,
    snap_to_ground: snap,
    autostep: self.autostep,
  }
}

///|
pub fn KinematicCharacterController::with_autostep(
  self : KinematicCharacterController,
  autostep : CharacterAutostep?,
) -> KinematicCharacterController {
  {
    max_slope_climb_angle: self.max_slope_climb_angle,
    snap_to_ground: self.snap_to_ground,
    autostep,
  }
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
fn min_real(a : @core.Real, b : @core.Real) -> @core.Real {
  if a < b {
    a
  } else {
    b
  }
}

///|
fn is_wall_normal(normal : @core.Vec2, cos_max_slope : @core.Real) -> Bool {
  @core.abs(normal.x) >= 0.5F && @core.abs(normal.y) <= cos_max_slope
}

///|
fn attempt_autostep(
  query : @collision.QueryPipeline,
  shape : @collision.Shape,
  start_pos : @core.Isometry2,
  remaining : @core.Vec2,
  cos_max_slope : @core.Real,
  skin : @core.Real,
  autostep : CharacterAutostep,
) -> (@core.Vec2, @core.Vec2, Bool)? {
  let up = @core.Vec2::new(0.0F, 1.0F)
  let down = @core.Vec2::new(0.0F, -1.0F)
  if @core.abs(remaining.x) <= 1.0e-6F {
    return None
  }
  let dir_x = @core.Vec2::new(remaining.x, 0.0F)
  let step_dir = if try_normalize_and_get_length(dir_x, 1.0e-6F) is Some(step) {
    step.0
  } else {
    return None
  }
  let dist_x = @core.abs(remaining.x)
  let (side_extent, up_extent) = compute_dims(shape)
  let max_height = character_length_eval(autostep.max_height, up_extent) + skin
  let min_width = character_length_eval(autostep.min_width, side_extent) + skin
  if max_height <= 0.0F || min_width <= 0.0F {
    return None
  }
  let up_free = if query.cast_shape(
      start_pos,
      up,
      shape,
      @collision.ShapeCastOptions::new(max_height, false),
    )
    is Some(hit) {
    if hit.1.toi() > skin {
      hit.1.toi() - skin
    } else {
      max_height
    }
  } else {
    max_height
  }
  if up_free <= 0.0F {
    return None
  }
  let pos_up = @core.Isometry2::new(
    start_pos.translation.add(vec2_scale(up, up_free)),
    start_pos.rotation,
  )
  let forward = min_real(dist_x, min_width)
  if query.cast_shape(
      pos_up,
      step_dir,
      shape,
      @collision.ShapeCastOptions::new(forward, false),
    )
    is Some(_) {
    return None
  }
  let pos_fwd = @core.Isometry2::new(
    pos_up.translation.add(vec2_scale(step_dir, forward)),
    pos_up.rotation,
  )
  if query.cast_shape(
      pos_fwd,
      down,
      shape,
      @collision.ShapeCastOptions::new(max_height + skin, false),
    )
    is Some(hit_down) {
    let n = hit_down.1.normal()
    if n.y < cos_max_slope {
      return None
    }
    let step_translation = vec2_scale(up, up_free)
      .add(vec2_scale(step_dir, forward))
      .add(vec2_scale(down, hit_down.1.toi()))
    let new_remaining = @core.Vec2::new(
      remaining.x - step_dir.x * forward,
      remaining.y,
    )
    Some((step_translation, new_remaining, true))
  } else {
    None
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
    let options = @collision.ShapeCastOptions::new(dist + skin, false)
    if query.cast_shape(current_pos, dir, shape, options) is Some(hit) {
      let toi = hit.1.toi()
      let allowed_dist = if toi > skin { toi - skin } else { 0.0F }
      let applied = vec2_scale(dir, allowed_dist)
      result_translation = result_translation.add(applied)
      remaining = remaining.sub(applied)
      let event_pos = @core.Isometry2::new(
        position.translation.add(result_translation),
        position.rotation,
      )
      handle_collision(CharacterCollision::{
        handle: hit.0,
        hit: hit.1,
        translation_applied: result_translation,
        translation_remaining: remaining,
        character_pos: event_pos,
      })
      let normal = hit.1.normal()
      if normal.y >= cos_max_slope {
        grounded = true
      }
      if self.autostep is Some(autostep) &&
        is_wall_normal(normal, cos_max_slope) {
        if attempt_autostep(
            query, shape, event_pos, remaining, cos_max_slope, skin, autostep,
          )
          is Some(step) {
          result_translation = result_translation.add(step.0)
          remaining = step.1
          grounded = grounded || step.2
          continue
        }
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
