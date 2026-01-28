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
  {
    max_slope_climb_angle: 0.785398163F,
    snap_to_ground: Some(CharacterLength::relative(0.2F)),
    autostep: None,
  }
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
        let (phw, phh) = compute_dims(part_shape)
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
fn is_wall_normal(normal : @core.Vec2, cos_max_slope : @core.Real) -> Bool {
  @core.abs(normal.x) >= 0.5F && @core.abs(normal.y) <= cos_max_slope
}

///|
fn detect_ground_contact(
  query : @collision.QueryPipeline,
  shape : @collision.Shape,
  position : @core.Isometry2,
  cos_max_slope : @core.Real,
) -> (@collision.ColliderHandle, @core.Vec2, @core.Vec2)? {
  // Use a downward shape-cast for grounded detection. This matches Rapier more closely than our
  // previous contact approximation, and avoids occasional false negatives on slopes.
  let prediction = 0.06F
  let down = @core.Vec2::new(0.0F, -1.0F)
  let options = @collision.ShapeCastOptions::new(prediction, true)
  if query.cast_shape(position, down, shape, options) is Some(hit) {
    let normal = hit.1.normal()
    if normal.y >= cos_max_slope {
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
  // Rapier parity: grounded detection is independent from the climb angle. We only require the
  // contact normal to have a non-negligible upward component (acos(1.0e-3) ≈ 89.94 degrees).
  max_slope_climb_angle |> ignore
  detect_ground_contact(query, shape, position, 1.0e-3F) is Some(_)
}

///|
fn grounded_info(
  query : @collision.QueryPipeline,
  shape : @collision.Shape,
  position : @core.Isometry2,
  max_slope_climb_angle : @core.Real,
) -> (Bool, @core.Vec2) {
  max_slope_climb_angle |> ignore
  if detect_ground_contact(query, shape, position, 1.0e-3F) is Some(hit) {
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
  // Match Rapier's default normal-nudge factor to avoid accumulating visible drift.
  let normal_nudge = 1.0e-4F
  let cos_max_slope = cos_real(self.max_slope_climb_angle)
  let dt_value = if dt < 0.0F { 0.0F } else { dt }
  let mut result_translation = @core.Vec2::zero()

  // Approximate Rapier's "check_and_fix_penetrations": if we start penetrating, push out
  // along the impact normal so subsequent casts don't get stuck at toi=0.
  let mut fix_iters = 0
  while fix_iters < 8 {
    fix_iters = fix_iters + 1
    let pos0 = @core.Isometry2::new(
      position.translation.add(result_translation),
      position.rotation,
    )
    let opts = @collision.ShapeCastOptions::new(0.0F, true)
    if query.cast_shape(pos0, up, shape, opts) is Some(hit) {
      if hit.1.toi() < 1.0e-5F {
        result_translation = result_translation.add(
          vec2_scale(hit.1.normal(), skin),
        )
        continue
      }
    }
    break
  }
  let grounded_at_start = grounded_info(
      query,
      shape,
      @core.Isometry2::new(
        position.translation.add(result_translation),
        position.rotation,
      ),
      self.max_slope_climb_angle,
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
      skin,
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
      result_translation = result_translation.add(
        vec2_scale(normal, normal_nudge),
      )
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
