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
fn max_real(a : @core.Real, b : @core.Real) -> @core.Real {
  if a > b {
    a
  } else {
    b
  }
}

///|
fn clamp_value(
  value : @core.Real,
  min_value : @core.Real,
  max_value : @core.Real,
) -> @core.Real {
  max_real(min_value, min_real(max_value, value))
}

///|
fn is_wall_normal(normal : @core.Vec2, cos_max_slope : @core.Real) -> Bool {
  @core.abs(normal.x) >= 0.5F && @core.abs(normal.y) <= cos_max_slope
}

///|
fn compute_shape_aabb(
  shape_pos : @core.Isometry2,
  shape : @collision.Shape,
  prediction : @core.Real,
) -> @core.Aabb {
  let pd = if prediction < 0.0F { 0.0F } else { prediction }
  match shape {
    @collision.Shape::Ball(radius) => {
      let r = radius + pd
      let c = shape_pos.translation
      @core.Aabb::new(
        @core.Vec2::new(c.x - r, c.y - r),
        @core.Vec2::new(c.x + r, c.y + r),
      )
    }
    @collision.Shape::Cuboid(half_width, half_height) => {
      let rot = shape_pos.rotation
      let abs_cos = @core.abs(rot.cos)
      let abs_sin = @core.abs(rot.sin)
      let hw = abs_cos * half_width + abs_sin * half_height + pd
      let hh = abs_sin * half_width + abs_cos * half_height + pd
      let c = shape_pos.translation
      @core.Aabb::new(
        @core.Vec2::new(c.x - hw, c.y - hh),
        @core.Vec2::new(c.x + hw, c.y + hh),
      )
    }
  }
}

///|

///|
fn contact_ball_ball(
  center1 : @core.Vec2,
  radius1 : @core.Real,
  center2 : @core.Vec2,
  radius2 : @core.Real,
) -> (@core.Real, @core.Vec2, @core.Vec2) {
  let delta = center1.sub(center2)
  let dist = delta.length()
  let normal = if dist <= 1.0e-6F {
    @core.Vec2::new(0.0F, 1.0F)
  } else {
    vec2_scale(delta, 1.0F / dist)
  }
  let point2 = center2.add(vec2_scale(normal, radius2))
  (dist - (radius1 + radius2), normal, point2)
}

///|
fn contact_ball_cuboid(
  center : @core.Vec2,
  radius : @core.Real,
  cuboid_center : @core.Vec2,
  cuboid_rotation : @core.Real,
  half_width : @core.Real,
  half_height : @core.Real,
) -> (@core.Real, @core.Vec2, @core.Vec2) {
  let rot = @core.Rot2::from_angle(cuboid_rotation)
  let inv = rot.inverse()
  let local_point = inv.rotate_vec2(center.sub(cuboid_center))
  let clamped = @core.Vec2::new(
    clamp_value(local_point.x, -half_width, half_width),
    clamp_value(local_point.y, -half_height, half_height),
  )
  let delta_local = local_point.sub(clamped)
  let dist = delta_local.length()
  let normal_local = if dist <= 1.0e-6F {
    let dx = half_width - @core.abs(local_point.x)
    let dy = half_height - @core.abs(local_point.y)
    if dx < dy {
      let sign = if local_point.x < 0.0F { -1.0F } else { 1.0F }
      @core.Vec2::new(sign, 0.0F)
    } else {
      let sign = if local_point.y < 0.0F { -1.0F } else { 1.0F }
      @core.Vec2::new(0.0F, sign)
    }
  } else {
    vec2_scale(delta_local, 1.0F / dist)
  }
  let normal = rot.rotate_vec2(normal_local)
  let point2 = cuboid_center.add(rot.rotate_vec2(clamped))
  (dist - radius, normal, point2)
}

///|
fn contact_cuboid_ball(
  cuboid_center : @core.Vec2,
  cuboid_rot : @core.Rot2,
  half_width : @core.Real,
  half_height : @core.Real,
  ball_center : @core.Vec2,
  ball_radius : @core.Real,
) -> (@core.Real, @core.Vec2, @core.Vec2) {
  let inv = cuboid_rot.inverse()
  let local_point = inv.rotate_vec2(ball_center.sub(cuboid_center))
  let inside = @core.abs(local_point.x) <= half_width &&
    @core.abs(local_point.y) <= half_height
  if inside {
    // When the ball center is inside the cuboid, treat this as a penetration.
    // For grounded prediction we mostly care about the direction (up).
    let dx = half_width - @core.abs(local_point.x)
    let dy = half_height - @core.abs(local_point.y)
    let (normal_local, _surface_local) = if dx < dy {
      let sign = if local_point.x < 0.0F { -1.0F } else { 1.0F }
      (
        @core.Vec2::new(sign, 0.0F),
        @core.Vec2::new(sign * half_width, local_point.y),
      )
    } else {
      let sign = if local_point.y < 0.0F { -1.0F } else { 1.0F }
      (
        @core.Vec2::new(0.0F, sign),
        @core.Vec2::new(local_point.x, sign * half_height),
      )
    }
    let normal = cuboid_rot.rotate_vec2(normal_local)
    let point2 = ball_center.add(vec2_scale(normal, ball_radius))
    (-ball_radius, normal, point2)
  } else {
    let clamped = @core.Vec2::new(
      clamp_value(local_point.x, -half_width, half_width),
      clamp_value(local_point.y, -half_height, half_height),
    )
    let cuboid_point = cuboid_center.add(cuboid_rot.rotate_vec2(clamped))
    let delta = cuboid_point.sub(ball_center)
    let dist = delta.length()
    let normal = if dist <= 1.0e-6F {
      @core.Vec2::new(0.0F, 1.0F)
    } else {
      vec2_scale(delta, 1.0F / dist)
    }
    let point2 = ball_center.add(vec2_scale(normal, ball_radius))
    (dist - ball_radius, normal, point2)
  }
}

///|
fn cuboid_support_point(
  center : @core.Vec2,
  rot : @core.Rot2,
  half_width : @core.Real,
  half_height : @core.Real,
  dir : @core.Vec2,
) -> @core.Vec2 {
  let inv = rot.inverse()
  let local_dir = inv.rotate_vec2(dir)
  let x = if local_dir.x < 0.0F { -half_width } else { half_width }
  let y = if local_dir.y < 0.0F { -half_height } else { half_height }
  center.add(rot.rotate_vec2(@core.Vec2::new(x, y)))
}

///|
fn cuboid_vertices(
  center : @core.Vec2,
  rot : @core.Rot2,
  half_width : @core.Real,
  half_height : @core.Real,
) -> Array[@core.Vec2] {
  [
    center.add(rot.rotate_vec2(@core.Vec2::new(-half_width, -half_height))),
    center.add(rot.rotate_vec2(@core.Vec2::new(half_width, -half_height))),
    center.add(rot.rotate_vec2(@core.Vec2::new(half_width, half_height))),
    center.add(rot.rotate_vec2(@core.Vec2::new(-half_width, half_height))),
  ]
}

///|
fn closest_point_on_segment(
  a : @core.Vec2,
  b : @core.Vec2,
  p : @core.Vec2,
) -> @core.Vec2 {
  let ab = b.sub(a)
  let denom = ab.dot(ab)
  if denom <= 1.0e-12F {
    a
  } else {
    let t = clamp_value(p.sub(a).dot(ab) / denom, 0.0F, 1.0F)
    a.add(vec2_scale(ab, t))
  }
}

///|
fn contact_cuboid_cuboid(
  center1 : @core.Vec2,
  rot1 : @core.Rot2,
  half_width1 : @core.Real,
  half_height1 : @core.Real,
  center2 : @core.Vec2,
  rot2 : @core.Rot2,
  half_width2 : @core.Real,
  half_height2 : @core.Real,
) -> (@core.Real, @core.Vec2, @core.Vec2) {
  let ux1 = rot1.rotate_vec2(@core.Vec2::new(1.0F, 0.0F))
  let uy1 = rot1.rotate_vec2(@core.Vec2::new(0.0F, 1.0F))
  let ux2 = rot2.rotate_vec2(@core.Vec2::new(1.0F, 0.0F))
  let uy2 = rot2.rotate_vec2(@core.Vec2::new(0.0F, 1.0F))
  let delta_c = center1.sub(center2)

  // SAT overlap check (2D rectangles: axes = face normals of both cuboids).
  let axes : Array[@core.Vec2] = [ux1, uy1, ux2, uy2]
  let mut separated = false
  let mut min_overlap = 1.0e30F
  let mut min_axis = @core.Vec2::new(0.0F, 1.0F)
  for axis in axes {
    let d = @core.abs(delta_c.dot(axis))
    let r1 = half_width1 * @core.abs(axis.dot(ux1)) +
      half_height1 * @core.abs(axis.dot(uy1))
    let r2 = half_width2 * @core.abs(axis.dot(ux2)) +
      half_height2 * @core.abs(axis.dot(uy2))
    let overlap = r1 + r2 - d
    if overlap <= 0.0F {
      separated = true
      break
    }
    if overlap < min_overlap {
      min_overlap = overlap
      let s = if delta_c.dot(axis) < 0.0F { -1.0F } else { 1.0F }
      min_axis = vec2_scale(axis, s)
    }
  }
  if !separated {
    let normal = min_axis
    let point2 = cuboid_support_point(
      center2, rot2, half_width2, half_height2, normal,
    )
    (-min_overlap, normal, point2)
  } else {
    // Compute the closest feature pair between the two cuboids.
    let vs1 = cuboid_vertices(center1, rot1, half_width1, half_height1)
    let vs2 = cuboid_vertices(center2, rot2, half_width2, half_height2)
    let mut best_dist2 = 1.0e30F
    let mut best_normal = @core.Vec2::new(0.0F, 1.0F)
    let mut best_point2 = center2
    for v in vs1 {
      for i in 0..<4 {
        let a = vs2[i]
        let b = vs2[(i + 1) % 4]
        let q = closest_point_on_segment(a, b, v)
        let d = v.sub(q)
        let dist2 = d.dot(d)
        if dist2 < best_dist2 {
          best_dist2 = dist2
          let dist = Float::sqrt(dist2)
          best_normal = if dist <= 1.0e-6F {
            @core.Vec2::new(0.0F, 1.0F)
          } else {
            vec2_scale(d, 1.0F / dist)
          }
          best_point2 = q
        }
      }
    }
    for v in vs2 {
      for i in 0..<4 {
        let a = vs1[i]
        let b = vs1[(i + 1) % 4]
        let q = closest_point_on_segment(a, b, v)
        let d = q.sub(v)
        let dist2 = d.dot(d)
        if dist2 < best_dist2 {
          best_dist2 = dist2
          let dist = Float::sqrt(dist2)
          best_normal = if dist <= 1.0e-6F {
            @core.Vec2::new(0.0F, 1.0F)
          } else {
            vec2_scale(d, 1.0F / dist)
          }
          best_point2 = v
        }
      }
    }
    (Float::sqrt(best_dist2), best_normal, best_point2)
  }
}

///|
fn detect_ground_contact(
  query : @collision.QueryPipeline,
  shape : @collision.Shape,
  position : @core.Isometry2,
  cos_max_slope : @core.Real,
) -> (@collision.ColliderHandle, @core.Vec2, @core.Vec2)? {
  let prediction = 0.06F
  let aabb = compute_shape_aabb(position, shape, prediction)
  let mut best : (@collision.ColliderHandle, @core.Vec2, @core.Vec2, @core.Real)? = None
  for handle in query.intersect_aabb_conservative(aabb) {
    if query.collider(handle) is Some(co) {
      let (dist_to_surface, normal, point2) = match (shape, co.shape) {
        (@collision.Shape::Ball(r1), @collision.Shape::Ball(r2)) =>
          contact_ball_ball(position.translation, r1, co.world_translation, r2)
        (@collision.Shape::Ball(r), @collision.Shape::Cuboid(hw, hh)) =>
          contact_ball_cuboid(
            position.translation,
            r,
            co.world_translation,
            co.world_rotation,
            hw,
            hh,
          )
        (@collision.Shape::Cuboid(hw, hh), @collision.Shape::Ball(r)) =>
          contact_cuboid_ball(
            position.translation,
            position.rotation,
            hw,
            hh,
            co.world_translation,
            r,
          )
        (@collision.Shape::Cuboid(hw1, hh1), @collision.Shape::Cuboid(hw2, hh2)) =>
          contact_cuboid_cuboid(
            position.translation,
            position.rotation,
            hw1,
            hh1,
            co.world_translation,
            @core.Rot2::from_angle(co.world_rotation),
            hw2,
            hh2,
          )
      }
      if dist_to_surface <= prediction && normal.y >= cos_max_slope {
        if best is Some(current) {
          if dist_to_surface < current.3 {
            best = Some((handle, normal, point2, dist_to_surface))
          }
        } else {
          best = Some((handle, normal, point2, dist_to_surface))
        }
      }
    }
  }
  if best is Some(hit) {
    Some((hit.0, hit.1, hit.2))
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
  let cos_max_slope = cos_real(max_slope_climb_angle)
  detect_ground_contact(query, shape, position, cos_max_slope) is Some(_)
}

///|
fn grounded_info(
  query : @collision.QueryPipeline,
  shape : @collision.Shape,
  position : @core.Isometry2,
  max_slope_climb_angle : @core.Real,
) -> (Bool, @core.Vec2) {
  let cos_max_slope = cos_real(max_slope_climb_angle)
  if detect_ground_contact(query, shape, position, cos_max_slope) is Some(hit) {
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
  let up = @core.Vec2::new(0.0F, 1.0F)
  let skin = 1.0e-3F
  let cos_max_slope = cos_real(self.max_slope_climb_angle)
  let grounded_at_start = grounded_info(
      query,
      shape,
      position,
      self.max_slope_climb_angle,
    ).0
  let dt_value = if dt < 0.0F { 0.0F } else { dt }
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
    grounded = grounded_info(query, shape, position, self.max_slope_climb_angle).0
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
  let final_pos = @core.Isometry2::new(
    position.translation.add(result_translation),
    position.rotation,
  )
  let end_info = grounded_info(
    query,
    shape,
    final_pos,
    self.max_slope_climb_angle,
  )
  grounded = end_info.0
  if grounded {
    result_translation = result_translation.add(
      vec2_scale(end_info.1, dt_value),
    )
  }
  { translation: result_translation, grounded }
}
