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
/// Real 3D kinematic character controller (3DReal backend).
///
/// This is a direct analogue of the 2D `KinematicCharacterController` for the 3DReal backend query
/// types (`QueryPipeline3DReal` / `Shape3D` / `ShapeCastHit3`).

///|
pub struct EffectiveMovement3DReal {
  translation : @core.Vec3
  grounded : Bool
  is_sliding_down_slope : Bool
}

///|
pub fn EffectiveMovement3DReal::translation(
  self : EffectiveMovement3DReal,
) -> @core.Vec3 {
  self.translation
}

///|
pub fn EffectiveMovement3DReal::is_grounded(
  self : EffectiveMovement3DReal,
) -> Bool {
  self.grounded
}

///|
pub fn EffectiveMovement3DReal::is_sliding_down_slope(
  self : EffectiveMovement3DReal,
) -> Bool {
  self.is_sliding_down_slope
}

///|
pub struct CharacterCollision3DReal {
  handle : @collision.ColliderHandle3D
  hit : @collision.ShapeCastHit3
  translation_applied : @core.Vec3
  translation_remaining : @core.Vec3
  character_pos : @core.Isometry3
}

///|
pub fn CharacterCollision3DReal::handle(
  self : CharacterCollision3DReal,
) -> @collision.ColliderHandle3D {
  self.handle
}

///|
pub fn CharacterCollision3DReal::hit(
  self : CharacterCollision3DReal,
) -> @collision.ShapeCastHit3 {
  self.hit
}

///|
pub fn CharacterCollision3DReal::translation_applied(
  self : CharacterCollision3DReal,
) -> @core.Vec3 {
  self.translation_applied
}

///|
pub fn CharacterCollision3DReal::translation_remaining(
  self : CharacterCollision3DReal,
) -> @core.Vec3 {
  self.translation_remaining
}

///|
pub fn CharacterCollision3DReal::character_pos(
  self : CharacterCollision3DReal,
) -> @core.Isometry3 {
  self.character_pos
}

///|
pub struct KinematicCharacterController3DReal {
  up : @core.Vec3
  offset : CharacterLength
  slide : Bool
  autostep : CharacterAutostep?
  max_slope_climb_angle : @core.Real
  min_slope_slide_angle : @core.Real
  snap_to_ground : CharacterLength?
  normal_nudge_factor : @core.Real
}

///|
pub fn KinematicCharacterController3DReal::new() -> KinematicCharacterController3DReal {
  {
    up: @core.Vec3::new(0.0F, 1.0F, 0.0F),
    offset: CharacterLength::relative(0.01F),
    slide: true,
    autostep: None,
    max_slope_climb_angle: @core.pi() * 0.25F,
    min_slope_slide_angle: @core.pi() * 0.25F,
    snap_to_ground: Some(CharacterLength::relative(0.2F)),
    normal_nudge_factor: 1.0e-4F,
  }
}

///|
pub fn KinematicCharacterController3DReal::default() -> KinematicCharacterController3DReal {
  KinematicCharacterController3DReal::new()
}

///|
pub fn KinematicCharacterController3DReal::with_up(
  self : KinematicCharacterController3DReal,
  up : @core.Vec3,
) -> KinematicCharacterController3DReal {
  {
    up,
    offset: self.offset,
    slide: self.slide,
    autostep: self.autostep,
    max_slope_climb_angle: self.max_slope_climb_angle,
    min_slope_slide_angle: self.min_slope_slide_angle,
    snap_to_ground: self.snap_to_ground,
    normal_nudge_factor: self.normal_nudge_factor,
  }
}

///|
pub fn KinematicCharacterController3DReal::with_offset(
  self : KinematicCharacterController3DReal,
  offset : CharacterLength,
) -> KinematicCharacterController3DReal {
  {
    up: self.up,
    offset,
    slide: self.slide,
    autostep: self.autostep,
    max_slope_climb_angle: self.max_slope_climb_angle,
    min_slope_slide_angle: self.min_slope_slide_angle,
    snap_to_ground: self.snap_to_ground,
    normal_nudge_factor: self.normal_nudge_factor,
  }
}

///|
pub fn KinematicCharacterController3DReal::with_slide(
  self : KinematicCharacterController3DReal,
  slide : Bool,
) -> KinematicCharacterController3DReal {
  {
    up: self.up,
    offset: self.offset,
    slide,
    autostep: self.autostep,
    max_slope_climb_angle: self.max_slope_climb_angle,
    min_slope_slide_angle: self.min_slope_slide_angle,
    snap_to_ground: self.snap_to_ground,
    normal_nudge_factor: self.normal_nudge_factor,
  }
}

///|
pub fn KinematicCharacterController3DReal::with_autostep(
  self : KinematicCharacterController3DReal,
  autostep : CharacterAutostep?,
) -> KinematicCharacterController3DReal {
  {
    up: self.up,
    offset: self.offset,
    slide: self.slide,
    autostep,
    max_slope_climb_angle: self.max_slope_climb_angle,
    min_slope_slide_angle: self.min_slope_slide_angle,
    snap_to_ground: self.snap_to_ground,
    normal_nudge_factor: self.normal_nudge_factor,
  }
}

///|
pub fn KinematicCharacterController3DReal::with_max_slope_climb_angle(
  self : KinematicCharacterController3DReal,
  angle : @core.Real,
) -> KinematicCharacterController3DReal {
  {
    up: self.up,
    offset: self.offset,
    slide: self.slide,
    autostep: self.autostep,
    max_slope_climb_angle: angle,
    min_slope_slide_angle: self.min_slope_slide_angle,
    snap_to_ground: self.snap_to_ground,
    normal_nudge_factor: self.normal_nudge_factor,
  }
}

///|
pub fn KinematicCharacterController3DReal::with_min_slope_slide_angle(
  self : KinematicCharacterController3DReal,
  angle : @core.Real,
) -> KinematicCharacterController3DReal {
  {
    up: self.up,
    offset: self.offset,
    slide: self.slide,
    autostep: self.autostep,
    max_slope_climb_angle: self.max_slope_climb_angle,
    min_slope_slide_angle: angle,
    snap_to_ground: self.snap_to_ground,
    normal_nudge_factor: self.normal_nudge_factor,
  }
}

///|
pub fn KinematicCharacterController3DReal::with_snap_to_ground(
  self : KinematicCharacterController3DReal,
  snap : CharacterLength?,
) -> KinematicCharacterController3DReal {
  {
    up: self.up,
    offset: self.offset,
    slide: self.slide,
    autostep: self.autostep,
    max_slope_climb_angle: self.max_slope_climb_angle,
    min_slope_slide_angle: self.min_slope_slide_angle,
    snap_to_ground: snap,
    normal_nudge_factor: self.normal_nudge_factor,
  }
}

///|
pub fn KinematicCharacterController3DReal::with_normal_nudge_factor(
  self : KinematicCharacterController3DReal,
  nudge : @core.Real,
) -> KinematicCharacterController3DReal {
  {
    up: self.up,
    offset: self.offset,
    slide: self.slide,
    autostep: self.autostep,
    max_slope_climb_angle: self.max_slope_climb_angle,
    min_slope_slide_angle: self.min_slope_slide_angle,
    snap_to_ground: self.snap_to_ground,
    normal_nudge_factor: nudge,
  }
}

///|
fn cc3d_real_character_length_eval(
  length : CharacterLength,
  reference : @core.Real,
) -> @core.Real {
  match length {
    CharacterLength::Relative(value) => value * reference
    CharacterLength::Absolute(value) => value
  }
}

///|
fn cc3d_real_min(a : @core.Real, b : @core.Real) -> @core.Real {
  if a < b {
    a
  } else {
    b
  }
}

///|
fn cc3d_real_max(a : @core.Real, b : @core.Real) -> @core.Real {
  if a > b {
    a
  } else {
    b
  }
}

///|
fn cc3d_real_try_normalize_and_get_length(
  v : @core.Vec3,
  eps : @core.Real,
) -> (@core.Vec3, @core.Real)? {
  let len2 = v.length_squared()
  if len2 <= eps * eps {
    None
  } else {
    let len = Float::sqrt(len2)
    Some((v.scale(1.0F / len), len))
  }
}

///|
fn cc3d_real_angle_between(u : @core.Vec3, v : @core.Vec3) -> @core.Real {
  // Robust 3D angle computation without acos: angle = atan2(|u×v|, u·v).
  let cross_len = u.cross(v).length()
  let dot = u.dot(v)
  @core.abs(@core.atan2(cross_len, dot))
}

///|
fn cc3d_real_split_into_components(
  up : @core.Vec3,
  translation : @core.Vec3,
) -> (@core.Vec3, @core.Vec3) {
  let vertical = up.scale(up.dot(translation))
  let horizontal = translation.sub(vertical)
  (vertical, horizontal)
}

///|
fn cc3d_real_subtract_hit(
  translation : @core.Vec3,
  hit : @collision.ShapeCastHit3,
) -> @core.Vec3 {
  let n = hit.normal()
  let surface_correction = cc3d_real_max(translation.scale(-1.0F).dot(n), 0.0F)
  // Match Rapier's small multiplier to avoid tunneling through walls.
  let surface_correction = surface_correction * (1.0F + 1.0e-5F)
  translation.add(n.scale(surface_correction))
}

///|
fn cc3d_real_compute_dims_with_up(
  shape : @collision.Shape3D,
  up : @core.Vec3,
) -> (@core.Real, @core.Real) {
  let aabb = shape.local_aabb()
  let extents = aabb.maxs.sub(aabb.mins).scale(0.5F)
  let up_abs = @core.Vec3::new(
    @core.abs(up.x),
    @core.abs(up.y),
    @core.abs(up.z),
  )
  let up_extent = extents.dot(up_abs)
  let side_vec = extents.sub(up_abs.scale(up_extent))
  let side_extent = side_vec.length()
  (side_extent, up_extent)
}

///|
priv struct HitInfo3DReal {
  toi : @collision.ShapeCastHit3
  is_wall : Bool
  is_nonslip_slope : Bool
}

///|
priv struct HitDecomposition3DReal {
  normal_part : @core.Vec3
  horizontal_tangent : @core.Vec3
  vertical_tangent : @core.Vec3
}

///|
fn HitDecomposition3DReal::unconstrained_slide_part(
  self : HitDecomposition3DReal,
) -> @core.Vec3 {
  self.normal_part.add(self.horizontal_tangent).add(self.vertical_tangent)
}

///|
fn cc3d_real_compute_hit_info(
  controller : KinematicCharacterController3DReal,
  toi : @collision.ShapeCastHit3,
) -> HitInfo3DReal {
  let up = controller.up
  let angle_with_floor = cc3d_real_angle_between(up, toi.normal())
  let is_ceiling = up.dot(toi.normal()) < 0.0F
  let is_wall = angle_with_floor >= controller.max_slope_climb_angle &&
    !is_ceiling
  let is_nonslip_slope = angle_with_floor <= controller.min_slope_slide_angle
  { toi, is_wall, is_nonslip_slope }
}

///|
fn cc3d_real_decompose_hit(
  controller : KinematicCharacterController3DReal,
  translation : @core.Vec3,
  hit : @collision.ShapeCastHit3,
) -> HitDecomposition3DReal {
  let n = hit.normal()
  let dist_to_surface = translation.dot(n)
  let (normal_part, penetration_part) = if dist_to_surface < 0.0F {
    (@core.Vec3::zero(), n.scale(dist_to_surface))
  } else {
    (n.scale(dist_to_surface), @core.Vec3::zero())
  }
  let tangent = translation.sub(normal_part).sub(penetration_part)
  let (vertical_tangent, horizontal_tangent) = cc3d_real_split_into_components(
    controller.up,
    tangent,
  )
  { normal_part, horizontal_tangent, vertical_tangent }
}

///|
fn cc3d_real_handle_slopes(
  controller : KinematicCharacterController3DReal,
  hit : HitInfo3DReal,
  movement_input : @core.Vec3,
  translation_remaining : @core.Vec3,
) -> (@core.Vec3, Bool) {
  let up = controller.up
  let (vertical_input, horizontal_input) = cc3d_real_split_into_components(
    up, movement_input,
  )
  let horiz_input_decomp = cc3d_real_decompose_hit(
    controller,
    horizontal_input,
    hit.toi,
  )
  let decomp = cc3d_real_decompose_hit(
    controller,
    translation_remaining,
    hit.toi,
  )

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
    hit.toi.normal().scale(controller.normal_nudge_factor),
  )
  (nudged, is_sliding)
}

///|
fn cc3d_real_detect_ground_contact(
  query : @collision.QueryPipeline3DReal,
  shape : @collision.Shape3D,
  position : @core.Isometry3,
  up : @core.Vec3,
  prediction : @core.Real,
  offset : @core.Real,
) -> (@collision.ColliderHandle3D, @core.Vec3, @core.Vec3)? {
  let down = up.scale(-1.0F)
  let options = @collision.ShapeCastOptions3::new(prediction, true).with_target_distance(
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
fn cc3d_real_detect_grounded(
  query : @collision.QueryPipeline3DReal,
  shape : @collision.Shape3D,
  position : @core.Isometry3,
  up : @core.Vec3,
  prediction : @core.Real,
  offset : @core.Real,
) -> Bool {
  cc3d_real_detect_ground_contact(
    query, shape, position, up, prediction, offset,
  )
  is Some(_)
}

///|
fn cc3d_real_grounded_info(
  query : @collision.QueryPipeline3DReal,
  shape : @collision.Shape3D,
  position : @core.Isometry3,
  up : @core.Vec3,
  prediction : @core.Real,
  offset : @core.Real,
) -> (Bool, @core.Vec3) {
  if cc3d_real_detect_ground_contact(
      query, shape, position, up, prediction, offset,
    )
    is Some(hit) {
    let handle = hit.0
    if query.collider(handle) is Some(co) &&
      co.parent is Some(parent) &&
      query.rigid_body(parent) is Some(body) &&
      body.body_type().is_kinematic() {
      (true, body.linvel())
    } else {
      (true, @core.Vec3::zero())
    }
  } else {
    (false, @core.Vec3::zero())
  }
}

///|
fn cc3d_real_attempt_autostep(
  query : @collision.QueryPipeline3DReal,
  shape : @collision.Shape3D,
  start_pos : @core.Isometry3,
  remaining : @core.Vec3,
  controller : KinematicCharacterController3DReal,
  dims : (@core.Real, @core.Real),
  stair_handle : @collision.ColliderHandle3D,
  autostep : CharacterAutostep,
) -> (@core.Vec3, @core.Vec3)? {
  let up = controller.up
  let down = up.scale(-1.0F)
  let (side_extent, up_extent) = dims
  let offset = cc3d_real_character_length_eval(controller.offset, up_extent)
  let min_width = cc3d_real_character_length_eval(
      autostep.min_width(),
      side_extent,
    ) +
    offset
  let max_height = cc3d_real_character_length_eval(
      autostep.max_height(),
      up_extent,
    ) +
    offset
  if max_height <= 0.0F || min_width <= 0.0F {
    return None
  }
  if !autostep.include_dynamic_bodies() {
    // If the obstacle is dynamic, ignore it (Rapier parity).
    if query.collider(stair_handle) is Some(co) &&
      co.parent() is Some(parent) &&
      query.rigid_body(parent) is Some(body) &&
      body.body_type().is_dynamic() {
      return None
    }
  }
  let shifted_pos = @core.Isometry3::new(
    start_pos.translation.add(up.scale(max_height)),
    start_pos.rotation,
  )
  let horiz = remaining.sub(up.scale(up.dot(remaining)))
  let horizontal_dir = if cc3d_real_try_normalize_and_get_length(horiz, 1.0e-6F)
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
      @collision.ShapeCastOptions3::new(max_height, false).with_target_distance(
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
      @collision.ShapeCastOptions3::new(min_width, false).with_target_distance(
        offset,
      ),
    )
    is Some(_) {
    return None
  }

  // Check the ramp we would land on isn't too steep.
  let pos_on_stair = @core.Isometry3::new(
    shifted_pos.translation.add(horizontal_dir.scale(min_width)),
    shifted_pos.rotation,
  )
  if query.cast_shape(
      pos_on_stair,
      down,
      shape,
      @collision.ShapeCastOptions3::new(max_height, false).with_target_distance(
        offset,
      ),
    )
    is Some(hit_down) {
    let (vertical_input, horizontal_input) = cc3d_real_split_into_components(
      up, remaining,
    )
    let vertical_slope = cc3d_real_subtract_hit(vertical_input, hit_down.1)
    let horizontal_slope = cc3d_real_subtract_hit(horizontal_input, hit_down.1)
    let slope_translation = horizontal_slope.add(vertical_slope)
    let angle = cc3d_real_angle_between(up, hit_down.1.normal())
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
      @collision.ShapeCastOptions3::new(max_height, false).with_target_distance(
        offset,
      ),
    )
    is Some(hit2) {
    hit2.1.toi()
  } else {
    max_height
  }
  let step_height = max_height - toi_down
  let step = up.scale(step_height)
  let remaining1 = remaining.sub(step)
  let nudge_amt = cc3d_real_min(horizontal_dir.dot(remaining1), min_width)
  let horizontal_nudge = horizontal_dir.scale(nudge_amt)
  let remaining2 = remaining1.sub(horizontal_nudge)
  let delta = step.add(horizontal_nudge)
  Some((remaining2, delta))
}

///|
pub fn KinematicCharacterController3DReal::move_shape(
  self : KinematicCharacterController3DReal,
  dt : @core.Real,
  query : @collision.QueryPipeline3DReal,
  shape : @collision.Shape3D,
  position : @core.Isometry3,
  translation : @core.Vec3,
  handle_collision : (CharacterCollision3DReal) -> Unit,
) -> EffectiveMovement3DReal {
  let up = if cc3d_real_try_normalize_and_get_length(self.up, 1.0e-6F)
    is Some(step) {
    step.0
  } else {
    @core.Vec3::new(0.0F, 1.0F, 0.0F)
  }
  let controller = self.with_up(up)
  let (side_extent, up_extent) = cc3d_real_compute_dims_with_up(shape, up)
  let offset = cc3d_real_character_length_eval(controller.offset, up_extent)
  let prediction = offset + 0.05F
  let dt_value = if dt < 0.0F { 0.0F } else { dt }
  let mut result_translation = @core.Vec3::zero()
  let mut sliding_down_slope = false

  // Rapier parity: `check_and_fix_penetrations` is currently a stub upstream.
  let grounded_at_start = cc3d_real_grounded_info(
      query,
      shape,
      @core.Isometry3::new(
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
  while cc3d_real_try_normalize_and_get_length(remaining, 1.0e-5F) is Some(step) {
    if iters >= 20 {
      break
    }
    iters = iters + 1
    let dir = step.0
    let dist = step.1
    let current_pos = @core.Isometry3::new(
      position.translation.add(result_translation),
      position.rotation,
    )
    let options = @collision.ShapeCastOptions3::new(dist, false).with_target_distance(
      offset,
    )
    if query.cast_shape(current_pos, dir, shape, options) is Some(hit) {
      let toi = hit.1.toi()
      let allowed_dist = toi
      let applied = dir.scale(allowed_dist)
      result_translation = result_translation.add(applied)
      remaining = remaining.sub(applied)
      let event_pos = @core.Isometry3::new(
        position.translation.add(result_translation),
        position.rotation,
      )
      handle_collision(CharacterCollision3DReal::{
        handle: hit.0,
        hit: hit.1,
        translation_applied: result_translation,
        translation_remaining: remaining,
        character_pos: event_pos,
      })
      let hit_info = cc3d_real_compute_hit_info(controller, hit.1)
      if controller.autostep is Some(autostep) && hit_info.is_wall {
        if cc3d_real_attempt_autostep(
            query,
            shape,
            event_pos,
            remaining,
            controller,
            (side_extent, up_extent),
            hit.0,
            autostep,
          )
          is Some(step2) {
          remaining = step2.0
          result_translation = result_translation.add(step2.1)
          continue
        }
      }
      let (new_remaining, slid) = cc3d_real_handle_slopes(
        controller, hit_info, translation, remaining,
      )
      remaining = new_remaining
      sliding_down_slope = sliding_down_slope || slid
    } else {
      result_translation = result_translation.add(remaining)
      remaining = @core.Vec3::zero()
      break
    }
    grounded = grounded ||
      cc3d_real_detect_grounded(
        query,
        shape,
        @core.Isometry3::new(
          position.translation.add(result_translation),
          position.rotation,
        ),
        up,
        prediction,
        offset,
      )
    if !controller.slide {
      break
    }
  }
  if iters == 0 {
    grounded = cc3d_real_grounded_info(
        query, shape, position, up, prediction, offset,
      ).0
  }
  if grounded_at_start &&
    !grounded &&
    controller.snap_to_ground is Some(snap_length) {
    let snap_dist = cc3d_real_character_length_eval(snap_length, up_extent)
    if result_translation.dot(up) < -1.0e-5F && snap_dist > 0.0F {
      let current_pos = @core.Isometry3::new(
        position.translation.add(result_translation),
        position.rotation,
      )
      let down = up.scale(-1.0F)
      let options = @collision.ShapeCastOptions3::new(snap_dist, false).with_target_distance(
        offset,
      )
      if query.cast_shape(current_pos, down, shape, options) is Some(hit) {
        result_translation = result_translation.add(down.scale(hit.1.toi()))
        grounded = true
      }
    }
  }
  let final_pos = @core.Isometry3::new(
    position.translation.add(result_translation),
    position.rotation,
  )
  let end_info = cc3d_real_grounded_info(
    query, shape, final_pos, up, prediction, offset,
  )
  if end_info.0 {
    grounded = true
    result_translation = result_translation.add(end_info.1.scale(dt_value))
  }
  {
    translation: result_translation,
    grounded,
    is_sliding_down_slope: sliding_down_slope,
  }
}

///|
pub fn KinematicCharacterController3DReal::solve_character_collision_impulses(
  self : KinematicCharacterController3DReal,
  dt : @core.Real,
  query : @collision.QueryPipeline3DReal,
  character_shape : @collision.Shape3D,
  character_mass : @core.Real,
  collisions : Array[CharacterCollision3DReal],
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
    let normal = if proj < 0.0F { normal0.scale(-1.0F) } else { normal0 }
    let to_transfer = normal.scale(@core.abs(proj))
    if to_transfer.length_squared() <= 1.0e-12F {
      continue
    }
    let vel_to_transfer = to_transfer.scale(1.0F / dt)
    let point = hit.point()
    let handle = collision.handle()
    if query.collider(handle) is Some(co) &&
      co.parent() is Some(parent) &&
      query.rigid_body_mut(parent) is Some(body) {
      if !body.body_type().is_dynamic() {
        continue
      }
      let r = point.sub(body.world_com())
      let body_vel_at_point = body.linvel().add(body.angvel().cross(r))
      let delta_v = vel_to_transfer.sub(body_vel_at_point).dot(normal)
      if delta_v <= 0.0F {
        continue
      }
      let m2 = body.mass_properties().mass
      let denom = m2 + character_mass
      if denom <= 1.0e-12F {
        continue
      }
      let mass_ratio = m2 * character_mass / denom
      let impulse = normal.scale(delta_v * mass_ratio)
      body.apply_impulse_at_point(impulse, point, true)
    }
  }
}
