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
  include_dynamic_bodies : Bool
}

///|
pub fn CharacterAutostep::new(
  max_height : CharacterLength,
  min_width : CharacterLength,
) -> CharacterAutostep {
  // Rapier parity: include dynamic bodies by default.
  { max_height, min_width, include_dynamic_bodies: true }
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
pub fn CharacterAutostep::include_dynamic_bodies(
  self : CharacterAutostep,
) -> Bool {
  self.include_dynamic_bodies
}

///|
pub fn CharacterAutostep::with_include_dynamic_bodies(
  self : CharacterAutostep,
  include_dynamic_bodies : Bool,
) -> CharacterAutostep {
  {
    max_height: self.max_height,
    min_width: self.min_width,
    include_dynamic_bodies,
  }
}

///|
pub struct EffectiveMovement {
  translation : @core.Vec2
  grounded : Bool
  is_sliding_down_slope : Bool
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
pub fn EffectiveMovement::is_sliding_down_slope(
  self : EffectiveMovement,
) -> Bool {
  self.is_sliding_down_slope
}

///|
pub struct KinematicCharacterController {
  up : @core.Vec2
  offset : CharacterLength
  slide : Bool
  max_slope_climb_angle : @core.Real
  min_slope_slide_angle : @core.Real
  snap_to_ground : CharacterLength?
  autostep : CharacterAutostep?
  normal_nudge_factor : @core.Real
}

///|
pub fn KinematicCharacterController::new() -> KinematicCharacterController {
  {
    up: @core.Vec2::new(0.0F, 1.0F),
    offset: CharacterLength::relative(0.01F),
    slide: true,
    max_slope_climb_angle: 0.785398163F,
    min_slope_slide_angle: 0.785398163F,
    snap_to_ground: Some(CharacterLength::relative(0.2F)),
    autostep: None,
    normal_nudge_factor: 1.0e-4F,
  }
}

///|
pub fn KinematicCharacterController::with_up(
  self : KinematicCharacterController,
  up : @core.Vec2,
) -> KinematicCharacterController {
  {
    up,
    offset: self.offset,
    slide: self.slide,
    max_slope_climb_angle: self.max_slope_climb_angle,
    min_slope_slide_angle: self.min_slope_slide_angle,
    snap_to_ground: self.snap_to_ground,
    autostep: self.autostep,
    normal_nudge_factor: self.normal_nudge_factor,
  }
}

///|
pub fn KinematicCharacterController::with_offset(
  self : KinematicCharacterController,
  offset : CharacterLength,
) -> KinematicCharacterController {
  {
    up: self.up,
    offset,
    slide: self.slide,
    max_slope_climb_angle: self.max_slope_climb_angle,
    min_slope_slide_angle: self.min_slope_slide_angle,
    snap_to_ground: self.snap_to_ground,
    autostep: self.autostep,
    normal_nudge_factor: self.normal_nudge_factor,
  }
}

///|
pub fn KinematicCharacterController::with_slide(
  self : KinematicCharacterController,
  slide : Bool,
) -> KinematicCharacterController {
  {
    up: self.up,
    offset: self.offset,
    slide,
    max_slope_climb_angle: self.max_slope_climb_angle,
    min_slope_slide_angle: self.min_slope_slide_angle,
    snap_to_ground: self.snap_to_ground,
    autostep: self.autostep,
    normal_nudge_factor: self.normal_nudge_factor,
  }
}

///|
pub fn KinematicCharacterController::with_max_slope_climb_angle(
  self : KinematicCharacterController,
  angle : @core.Real,
) -> KinematicCharacterController {
  {
    up: self.up,
    offset: self.offset,
    slide: self.slide,
    max_slope_climb_angle: angle,
    min_slope_slide_angle: self.min_slope_slide_angle,
    snap_to_ground: self.snap_to_ground,
    autostep: self.autostep,
    normal_nudge_factor: self.normal_nudge_factor,
  }
}

///|
pub fn KinematicCharacterController::with_min_slope_slide_angle(
  self : KinematicCharacterController,
  angle : @core.Real,
) -> KinematicCharacterController {
  {
    up: self.up,
    offset: self.offset,
    slide: self.slide,
    max_slope_climb_angle: self.max_slope_climb_angle,
    min_slope_slide_angle: angle,
    snap_to_ground: self.snap_to_ground,
    autostep: self.autostep,
    normal_nudge_factor: self.normal_nudge_factor,
  }
}

///|
pub fn KinematicCharacterController::with_snap_to_ground(
  self : KinematicCharacterController,
  snap : CharacterLength?,
) -> KinematicCharacterController {
  {
    up: self.up,
    offset: self.offset,
    slide: self.slide,
    max_slope_climb_angle: self.max_slope_climb_angle,
    min_slope_slide_angle: self.min_slope_slide_angle,
    snap_to_ground: snap,
    autostep: self.autostep,
    normal_nudge_factor: self.normal_nudge_factor,
  }
}

///|
pub fn KinematicCharacterController::with_autostep(
  self : KinematicCharacterController,
  autostep : CharacterAutostep?,
) -> KinematicCharacterController {
  {
    up: self.up,
    offset: self.offset,
    slide: self.slide,
    max_slope_climb_angle: self.max_slope_climb_angle,
    min_slope_slide_angle: self.min_slope_slide_angle,
    snap_to_ground: self.snap_to_ground,
    autostep,
    normal_nudge_factor: self.normal_nudge_factor,
  }
}

///|
pub fn KinematicCharacterController::with_normal_nudge_factor(
  self : KinematicCharacterController,
  normal_nudge_factor : @core.Real,
) -> KinematicCharacterController {
  {
    up: self.up,
    offset: self.offset,
    slide: self.slide,
    max_slope_climb_angle: self.max_slope_climb_angle,
    min_slope_slide_angle: self.min_slope_slide_angle,
    snap_to_ground: self.snap_to_ground,
    autostep: self.autostep,
    normal_nudge_factor,
  }
}

///|
fn vec2_scale(v : @core.Vec2, s : @core.Real) -> @core.Vec2 {
  @core.Vec2::new(v.x * s, v.y * s)
}

///|
fn vec2_neg(v : @core.Vec2) -> @core.Vec2 {
  @core.Vec2::new(-v.x, -v.y)
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
fn compute_extents(shape : @collision.Shape) -> (@core.Real, @core.Real) {
  match shape {
    @collision.Shape::Ball(r) => (r, r)
    @collision.Shape::Cuboid(hw, hh) => (hw, hh)
    @collision.Shape::CapsuleX(half_height, radius) =>
      (half_height + radius, radius)
    @collision.Shape::CapsuleY(half_height, radius) =>
      (radius, half_height + radius)
    @collision.Shape::Segment(a, b) => {
      let hw = max_real(@core.abs(a.x), @core.abs(b.x))
      let hh = max_real(@core.abs(a.y), @core.abs(b.y))
      (hw, hh)
    }
    @collision.Shape::Polyline(vertices, _) => {
      let mut hw = 0.0F
      let mut hh = 0.0F
      for i in 0..<vertices.length() {
        let p = vertices[i]
        let ax = @core.abs(p.x)
        let ay = @core.abs(p.y)
        if ax > hw {
          hw = ax
        }
        if ay > hh {
          hh = ay
        }
      }
      (hw, hh)
    }
    @collision.Shape::ConvexPolygon(vertices) => {
      let mut hw = 0.0F
      let mut hh = 0.0F
      for i in 0..<vertices.length() {
        let p = vertices[i]
        let ax = @core.abs(p.x)
        let ay = @core.abs(p.y)
        if ax > hw {
          hw = ax
        }
        if ay > hh {
          hh = ay
        }
      }
      (hw, hh)
    }
    @collision.Shape::TriMesh(vertices, _) => {
      let mut hw = 0.0F
      let mut hh = 0.0F
      for i in 0..<vertices.length() {
        let p = vertices[i]
        let ax = @core.abs(p.x)
        let ay = @core.abs(p.y)
        if ax > hw {
          hw = ax
        }
        if ay > hh {
          hh = ay
        }
      }
      (hw, hh)
    }
    @collision.Shape::Compound(parts) => {
      let mut hw = 0.0F
      let mut hh = 0.0F
      for i in 0..<parts.length() {
        let (pose, part_shape) = parts[i]
        let t = pose.transform_point(@core.Vec2::zero())
        let (phw, phh) = compute_extents(part_shape)
        let cand_hw = @core.abs(t.x) + phw
        let cand_hh = @core.abs(t.y) + phh
        if cand_hw > hw {
          hw = cand_hw
        }
        if cand_hh > hh {
          hh = cand_hh
        }
      }
      (hw, hh)
    }
  }
}

///|
fn compute_dims_with_up(
  shape : @collision.Shape,
  up : @core.Vec2,
) -> (@core.Real, @core.Real) {
  let (hw, hh) = compute_extents(shape)
  let extents = @core.Vec2::new(hw, hh)
  let up_abs = @core.Vec2::new(@core.abs(up.x), @core.abs(up.y))
  let up_extent = extents.dot(up_abs)
  let side_vec = extents.sub(vec2_scale(up_abs, up_extent))
  let side_extent = side_vec.length()
  (side_extent, up_extent)
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
fn max_real(a : @core.Real, b : @core.Real) -> @core.Real {
  if a > b {
    a
  } else {
    b
  }
}

///|
fn angle_between(u : @core.Vec2, v : @core.Vec2) -> @core.Real {
  // Robust 2D angle computation without acos: angle = atan2(|u×v|, u·v).
  let cross = u.cross(v)
  let dot = u.dot(v)
  @core.abs(@core.atan2(cross, dot))
}

///|
fn split_into_components(
  up : @core.Vec2,
  translation : @core.Vec2,
) -> (@core.Vec2, @core.Vec2) {
  let vertical = vec2_scale(up, up.dot(translation))
  let horizontal = translation.sub(vertical)
  (vertical, horizontal)
}

///|
fn subtract_hit(
  translation : @core.Vec2,
  hit : @collision.ShapeCastHit,
) -> @core.Vec2 {
  let n = hit.normal()
  let surface_correction = max_real(vec2_neg(translation).dot(n), 0.0F)
  // Match Rapier's small multiplier to avoid tunneling through walls.
  let surface_correction = surface_correction * (1.0F + 1.0e-5F)
  translation.add(vec2_scale(n, surface_correction))
}

///|
priv struct HitInfo {
  toi : @collision.ShapeCastHit
  is_wall : Bool
  is_nonslip_slope : Bool
}

///|
priv struct HitDecomposition {
  normal_part : @core.Vec2
  horizontal_tangent : @core.Vec2
  vertical_tangent : @core.Vec2
}

///|
fn HitDecomposition::unconstrained_slide_part(
  self : HitDecomposition,
) -> @core.Vec2 {
  self.normal_part.add(self.horizontal_tangent).add(self.vertical_tangent)
}

///|
fn compute_hit_info(
  controller : KinematicCharacterController,
  toi : @collision.ShapeCastHit,
) -> HitInfo {
  let up = controller.up
  let angle_with_floor = angle_between(up, toi.normal())
  let is_ceiling = up.dot(toi.normal()) < 0.0F
  let is_wall = angle_with_floor >= controller.max_slope_climb_angle &&
    !is_ceiling
  let is_nonslip_slope = angle_with_floor <= controller.min_slope_slide_angle
  { toi, is_wall, is_nonslip_slope }
}

///|
fn decompose_hit(
  controller : KinematicCharacterController,
  translation : @core.Vec2,
  hit : @collision.ShapeCastHit,
) -> HitDecomposition {
  let n = hit.normal()
  let dist_to_surface = translation.dot(n)
  let (normal_part, penetration_part) = if dist_to_surface < 0.0F {
    (@core.Vec2::zero(), vec2_scale(n, dist_to_surface))
  } else {
    (vec2_scale(n, dist_to_surface), @core.Vec2::zero())
  }
  let tangent = translation.sub(normal_part).sub(penetration_part)
  // Rapier dim2 parity: the tangent space is 1D, so horizontal_tangent is always zero.
  controller |> ignore
  {
    normal_part,
    horizontal_tangent: @core.Vec2::zero(),
    vertical_tangent: tangent,
  }
}

///|
fn handle_slopes(
  controller : KinematicCharacterController,
  hit : HitInfo,
  movement_input : @core.Vec2,
  translation_remaining : @core.Vec2,
) -> (@core.Vec2, Bool) {
  let up = controller.up
  let (vertical_input, horizontal_input) = split_into_components(
    up, movement_input,
  )
  let horiz_input_decomp = decompose_hit(controller, horizontal_input, hit.toi)
  let decomp = decompose_hit(controller, translation_remaining, hit.toi)

  // An object is trying to slip if the tangential movement induced by its horizontal movement points downward.
  let slipping_intent = up.dot(horiz_input_decomp.vertical_tangent) < 0.0F
  // An object is slipping if its tangential movement points downward.
  let slipping = up.dot(decomp.vertical_tangent) < 0.0F

  // An object is trying to climb if its vertical input motion points upward.
  let climbing_intent = up.dot(vertical_input) > 0.0F
  // An object is climbing if the tangential movement induced by its movement points upward.
  let climbing = up.dot(decomp.vertical_tangent) > 0.0F
  let mut is_sliding = false
  let allowed = if hit.is_wall && climbing && !climbing_intent {
    decomp.horizontal_tangent.add(decomp.normal_part)
  } else if hit.is_nonslip_slope && slipping && !slipping_intent {
    decomp.horizontal_tangent.add(decomp.normal_part)
  } else {
    is_sliding = true
    decomp.unconstrained_slide_part()
  }
  let nudged = allowed.add(
    vec2_scale(hit.toi.normal(), controller.normal_nudge_factor),
  )
  (nudged, is_sliding)
}

///|
fn detect_ground_contact(
  query : @collision.QueryPipeline,
  shape : @collision.Shape,
  position : @core.Isometry2,
  up : @core.Vec2,
  prediction : @core.Real,
  offset : @core.Real,
) -> (@collision.ColliderHandle, @core.Vec2, @core.Vec2)? {
  // Rapier parity: grounded detection uses a conservative prediction based on offset.
  let down = vec2_neg(up)
  let options = @collision.ShapeCastOptions::new(prediction, true).with_target_distance(
    offset,
  )
  if query.cast_shape(position, down, shape, options) is Some(hit) {
    let normal = hit.1.normal()
    // For the controller to be grounded, the angle between the contact normal and the up vector
    // has to be smaller than acos(1.0e-3) ≈ 89.94 degrees.
    if normal.dot(up) >= 1.0e-3F {
      Some((hit.0, normal, hit.1.point()))
    } else {
      None
    }
  } else {
    None
  }
}

///|
fn attempt_autostep(
  query : @collision.QueryPipeline,
  shape : @collision.Shape,
  start_pos : @core.Isometry2,
  remaining : @core.Vec2,
  controller : KinematicCharacterController,
  dims : (@core.Real, @core.Real),
  stair_handle : @collision.ColliderHandle,
  autostep : CharacterAutostep,
) -> (@core.Vec2, @core.Vec2)? {
  let up = controller.up
  let down = vec2_neg(up)
  let (side_extent, up_extent) = dims
  let offset = character_length_eval(controller.offset, up_extent)
  let min_width = character_length_eval(autostep.min_width, side_extent) +
    offset
  let max_height = character_length_eval(autostep.max_height, up_extent) +
    offset
  if max_height <= 0.0F || min_width <= 0.0F {
    return None
  }
  if !autostep.include_dynamic_bodies() {
    // If the obstacle is dynamic, ignore it (Rapier parity).
    if query.collider(stair_handle) is Some(co) &&
      co.parent() is Some(parent) &&
      query.rigid_body(parent) is Some(body) &&
      body.is_dynamic() {
      return None
    }
  }
  let shifted_pos = @core.Isometry2::new(
    start_pos.translation.add(vec2_scale(up, max_height)),
    start_pos.rotation,
  )
  let horiz = remaining.sub(vec2_scale(up, up.dot(remaining)))
  let horizontal_dir = if try_normalize_and_get_length(horiz, 1.0e-6F)
    is Some(step) {
    step.0
  } else {
    return None
  }

  // Can we go up?
  if query.cast_shape(
      start_pos,
      up,
      shape,
      @collision.ShapeCastOptions::new(max_height, false).with_target_distance(
        offset,
      ),
    )
    is Some(_) {
    return None
  }

  // Is there enough room on the stair after stepping?
  if query.cast_shape(
      shifted_pos,
      horizontal_dir,
      shape,
      @collision.ShapeCastOptions::new(min_width, false).with_target_distance(
        offset,
      ),
    )
    is Some(_) {
    return None
  }

  // Check the ramp we would land on isn't too steep.
  let pos_on_stair = @core.Isometry2::new(
    shifted_pos.translation.add(vec2_scale(horizontal_dir, min_width)),
    shifted_pos.rotation,
  )
  if query.cast_shape(
      pos_on_stair,
      down,
      shape,
      @collision.ShapeCastOptions::new(max_height, false).with_target_distance(
        offset,
      ),
    )
    is Some(hit_down) {
    let (vertical_input, horizontal_input) = split_into_components(
      up, remaining,
    )
    let vertical_slope = subtract_hit(vertical_input, hit_down.1)
    let horizontal_slope = subtract_hit(horizontal_input, hit_down.1)
    let slope_translation = horizontal_slope.add(vertical_slope)
    let angle = angle_between(up, hit_down.1.normal())
    let climbing = up.dot(slope_translation) >= 0.0F
    if climbing && angle > controller.max_slope_climb_angle {
      return None
    }
  }

  // Find actual step height.
  let toi_down = if query.cast_shape(
      pos_on_stair,
      down,
      shape,
      @collision.ShapeCastOptions::new(max_height, false).with_target_distance(
        offset,
      ),
    )
    is Some(hit2) {
    hit2.1.toi()
  } else {
    max_height
  }
  let step_height = max_height - toi_down
  let step = vec2_scale(up, step_height)
  let remaining1 = remaining.sub(step)
  let nudge_amt = min_real(horizontal_dir.dot(remaining1), min_width)
  let horizontal_nudge = vec2_scale(horizontal_dir, nudge_amt)
  let remaining2 = remaining1.sub(horizontal_nudge)
  let delta = step.add(horizontal_nudge)
  Some((remaining2, delta))
}

///|
fn detect_grounded(
  query : @collision.QueryPipeline,
  shape : @collision.Shape,
  position : @core.Isometry2,
  up : @core.Vec2,
  prediction : @core.Real,
  offset : @core.Real,
) -> Bool {
  detect_ground_contact(query, shape, position, up, prediction, offset)
  is Some(_)
}

///|
fn grounded_info(
  query : @collision.QueryPipeline,
  shape : @collision.Shape,
  position : @core.Isometry2,
  up : @core.Vec2,
  prediction : @core.Real,
  offset : @core.Real,
) -> (Bool, @core.Vec2) {
  if detect_ground_contact(query, shape, position, up, prediction, offset)
    is Some(hit) {
    let handle = hit.0
    if query.collider(handle) is Some(co) &&
      co.parent is Some(parent) &&
      query.rigid_body(parent) is Some(body) &&
      body.is_kinematic() {
      (true, body.linvel())
    } else {
      (true, @core.Vec2::zero())
    }
  } else {
    (false, @core.Vec2::zero())
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
  let up = self.up
  let (side_extent, up_extent) = compute_dims_with_up(shape, up)
  let offset = character_length_eval(self.offset, up_extent)
  let prediction = offset + 0.05F
  let dt_value = if dt < 0.0F { 0.0F } else { dt }
  let mut result_translation = @core.Vec2::zero()
  let mut sliding_down_slope = false

  // Rapier parity: `check_and_fix_penetrations` is currently a stub upstream.
  let grounded_at_start = grounded_info(
      query,
      shape,
      @core.Isometry2::new(
        position.translation.add(result_translation),
        position.rotation,
      ),
      up,
      prediction,
      offset,
    ).0
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
    let options = @collision.ShapeCastOptions::new(dist, false).with_target_distance(
      offset,
    )
    if query.cast_shape(current_pos, dir, shape, options) is Some(hit) {
      let toi = hit.1.toi()
      let allowed_dist = toi
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
      let hit_info = compute_hit_info(self, hit.1)
      if self.autostep is Some(autostep) && hit_info.is_wall {
        if attempt_autostep(
            query,
            shape,
            event_pos,
            remaining,
            self,
            (side_extent, up_extent),
            hit.0,
            autostep,
          )
          is Some(step) {
          remaining = step.0
          result_translation = result_translation.add(step.1)
          continue
        }
      }
      let (new_remaining, slid) = handle_slopes(
        self, hit_info, translation, remaining,
      )
      remaining = new_remaining
      sliding_down_slope = sliding_down_slope || slid
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
        up,
        prediction,
        offset,
      )
    if !self.slide {
      break
    }
  }
  if iters == 0 {
    grounded = grounded_info(query, shape, position, up, prediction, offset).0
  }
  if grounded_at_start && !grounded && self.snap_to_ground is Some(snap_length) {
    let snap_dist = character_length_eval(snap_length, up_extent)
    if result_translation.dot(up) < -1.0e-5F && snap_dist > 0.0F {
      let current_pos = @core.Isometry2::new(
        position.translation.add(result_translation),
        position.rotation,
      )
      let down = vec2_neg(up)
      let options = @collision.ShapeCastOptions::new(snap_dist, false).with_target_distance(
        offset,
      )
      if query.cast_shape(current_pos, down, shape, options) is Some(hit) {
        result_translation = result_translation.add(
          vec2_scale(down, hit.1.toi()),
        )
        grounded = true
      }
    }
  }
  let final_pos = @core.Isometry2::new(
    position.translation.add(result_translation),
    position.rotation,
  )
  let end_info = grounded_info(query, shape, final_pos, up, prediction, offset)
  if end_info.0 {
    grounded = true
    result_translation = result_translation.add(
      vec2_scale(end_info.1, dt_value),
    )
  }
  {
    translation: result_translation,
    grounded,
    is_sliding_down_slope: sliding_down_slope,
  }
}

///|
pub fn KinematicCharacterController::solve_character_collision_impulses(
  self : KinematicCharacterController,
  dt : @core.Real,
  query : @collision.QueryPipeline,
  character_shape : @collision.Shape,
  character_mass : @core.Real,
  collisions : Array[CharacterCollision],
) -> Unit {
  self |> ignore
  character_shape |> ignore
  if dt <= 0.0F {
    return
  }
  // Simplified parity with Rapier: transfer a portion of the character's remaining movement
  // along the collision normal as an impulse applied to the dynamic body hit by the shape-cast.
  for i in 0..<collisions.length() {
    let collision = collisions[i]
    let hit = collision.hit()
    let remaining = collision.translation_remaining()
    let normal0 = hit.normal()
    let proj = remaining.dot(normal0)
    // Ensure the normal points in the direction of motion we want to transfer.
    let normal = if proj < 0.0F {
      @core.Vec2::new(-normal0.x, -normal0.y)
    } else {
      normal0
    }
    let to_transfer = vec2_scale(normal, @core.abs(proj))
    if to_transfer.length_squared() <= 1.0e-12F {
      continue
    }
    let vel_to_transfer = vec2_scale(to_transfer, 1.0F / dt)
    let point = hit.point()
    let handle = collision.handle()
    if query.collider(handle) is Some(co) &&
      co.parent() is Some(parent) &&
      query.bodies.get_mut(parent) is Some(body) {
      if !body.is_dynamic() || !body.is_enabled() {
        continue
      }
      let r = point.sub(body.world_com())
      let body_vel_at_point = body
        .linvel()
        .add(@core.Vec2::new(-body.angvel() * r.y, body.angvel() * r.x))
      let delta_v = vel_to_transfer.sub(body_vel_at_point).dot(normal)
      if delta_v <= 0.0F {
        continue
      }
      let m2 = body.mass()
      let denom = m2 + character_mass
      if denom <= 1.0e-12F {
        continue
      }
      let mass_ratio = m2 * character_mass / denom
      let impulse = vec2_scale(normal, delta_v * mass_ratio)
      body.apply_impulse_at_point(impulse, point, true) |> ignore
    }
  }
}
