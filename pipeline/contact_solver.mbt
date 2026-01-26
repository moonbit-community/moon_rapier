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
/// A minimal sequential-impulse contact solver (2D) aligned with Rapier's concepts.
/// This is not feature-complete (no warmstarting/manifold caching yet), but it uses
/// impulse-based resolution with restitution + friction and an optional position correction.
const SOLVER_BAUMGARTE : @core.Real = 0.2F
// Keep position correction strict for now so deep initial overlaps are resolved
// in one step (matches existing pipeline tests and avoids persistent intersection).

///|
const SOLVER_PENETRATION_SLOP : @core.Real = 0.0F

///|
const SOLVER_POSITION_CORRECTION_PERCENT : @core.Real = 1.0F

///|
const SOLVER_ITERATIONS : Int = 8

///|
fn solver_vec2_scale(v : @core.Vec2, s : @core.Real) -> @core.Vec2 {
  @core.Vec2::new(v.x * s, v.y * s)
}

///|
fn cross_z_scalar(w : @core.Real, v : @core.Vec2) -> @core.Vec2 {
  @core.Vec2::new(-w * v.y, w * v.x)
}

///|
fn cross_vec2(a : @core.Vec2, b : @core.Vec2) -> @core.Real {
  a.x * b.y - a.y * b.x
}

///|
fn body_inv_mass(body : @dynamics.RigidBody) -> @core.Real {
  // This repo does not yet compute mass properties from colliders, so we fall back to a
  // unit mass (scaled by `additional_mass`) for impulse computations.
  let m = 1.0F + body.additional_mass()
  if m <= 1.0e-12F {
    0.0F
  } else {
    1.0F / m
  }
}

///|
fn body_inv_inertia(body : @dynamics.RigidBody) -> @core.Real {
  let i = 1.0F + body.additional_mass()
  if i <= 1.0e-12F {
    0.0F
  } else {
    1.0F / i
  }
}

///|
fn solver_clamp_value(
  value : @core.Real,
  min_val : @core.Real,
  max_val : @core.Real,
) -> @core.Real {
  if value < min_val {
    min_val
  } else if value > max_val {
    max_val
  } else {
    value
  }
}

///|
fn normalize_or_fallback(v : @core.Vec2, fallback : @core.Vec2) -> @core.Vec2 {
  let len2 = v.dot(v)
  if len2 <= 1.0e-12F {
    fallback
  } else {
    v.normalize()
  }
}

///|
fn shape_support_point(
  shape : @collision.Shape,
  center : @core.Vec2,
  rotation : @core.Real,
  dir : @core.Vec2,
) -> @core.Vec2 {
  let d = normalize_or_fallback(dir, @core.Vec2::new(1.0F, 0.0F))
  match shape {
    @collision.Shape::Ball(radius) => center.add(solver_vec2_scale(d, radius))
    @collision.Shape::Cuboid(hw, hh) => {
      let rot = @core.Rot2::from_angle(rotation)
      let ax = @core.Vec2::new(rot.cos, rot.sin)
      let ay = @core.Vec2::new(-rot.sin, rot.cos)
      let sx = if d.dot(ax) >= 0.0F { 1.0F } else { -1.0F }
      let sy = if d.dot(ay) >= 0.0F { 1.0F } else { -1.0F }
      center
      .add(solver_vec2_scale(ax, hw * sx))
      .add(solver_vec2_scale(ay, hh * sy))
    }
    @collision.Shape::CapsuleX(half_height, radius) => {
      let rot = @core.Rot2::from_angle(rotation)
      let axis = @core.Vec2::new(rot.cos, rot.sin)
      let end = if d.dot(axis) >= 0.0F {
        center.add(solver_vec2_scale(axis, half_height))
      } else {
        center.add(solver_vec2_scale(axis, -half_height))
      }
      end.add(solver_vec2_scale(d, radius))
    }
    @collision.Shape::CapsuleY(half_height, radius) => {
      let rot = @core.Rot2::from_angle(rotation)
      let axis = @core.Vec2::new(-rot.sin, rot.cos)
      let end = if d.dot(axis) >= 0.0F {
        center.add(solver_vec2_scale(axis, half_height))
      } else {
        center.add(solver_vec2_scale(axis, -half_height))
      }
      end.add(solver_vec2_scale(d, radius))
    }
    @collision.Shape::Segment(a, b) => {
      let rot = @core.Rot2::from_angle(rotation)
      let wa = center.add(rot.rotate_vec2(a))
      let wb = center.add(rot.rotate_vec2(b))
      if wa.dot(d) >= wb.dot(d) {
        wa
      } else {
        wb
      }
    }
  }
}

///|
priv struct ContactConstraint {
  collider1 : @collision.ColliderHandle
  collider2 : @collision.ColliderHandle
  body1 : @dynamics.RigidBodyHandle?
  body2 : @dynamics.RigidBodyHandle?
  normal : @core.Vec2
  contact_point : @core.Vec2
  penetration : @core.Real
  restitution : @core.Real
  friction : @core.Real
  inv_mass_n : @core.Real
  inv_mass_t : @core.Real
  bias : @core.Real
  mut impulse_n : @core.Real
  mut impulse_t : @core.Real
}

///|
fn build_contact_constraint(
  bodies : @dynamics.RigidBodySet,
  colliders : @collision.ColliderSet,
  collider1 : @collision.ColliderHandle,
  collider2 : @collision.ColliderHandle,
  dt : @core.Real,
) -> ContactConstraint? {
  if colliders.get(collider1) is Some(c1) &&
    colliders.get(collider2) is Some(c2) {
    if c1.is_sensor() || c2.is_sensor() {
      return None
    }
    if !c1.solver_groups().test_groups(c2.solver_groups()) {
      return None
    }
    // Compute penetration and a separating normal using the existing separation logic.
    // The returned vector points from c2 towards c1 and its length is the penetration depth.
    let sep = separation_vector(
      c1.shape(),
      c1.translation(),
      c1.rotation(),
      c2.shape(),
      c2.translation(),
      c2.rotation(),
    )
    if sep is None {
      return None
    }
    let sep_vec = sep.unwrap()
    let penetration = sep_vec.length()
    let normal = normalize_or_fallback(sep_vec, @core.Vec2::new(1.0F, 0.0F))
    let p1 = shape_support_point(
      c1.shape(),
      c1.translation(),
      c1.rotation(),
      @core.Vec2::new(-normal.x, -normal.y),
    )
    let p2 = shape_support_point(
      c2.shape(),
      c2.translation(),
      c2.rotation(),
      normal,
    )
    let contact_point = @core.Vec2::new(
      (p1.x + p2.x) * 0.5F,
      (p1.y + p2.y) * 0.5F,
    )
    let mut inv_mass1 = 0.0F
    let mut inv_inertia1 = 0.0F
    let mut inv_mass2 = 0.0F
    let mut inv_inertia2 = 0.0F
    let body1 = c1.parent()
    let body2 = c2.parent()
    if body1 is Some(h1) {
      if bodies.get(h1) is Some(rb1) {
        if rb1.is_dynamic() && rb1.is_enabled() {
          inv_mass1 = body_inv_mass(rb1)
          inv_inertia1 = body_inv_inertia(rb1)
        }
      }
    }
    if body2 is Some(h2) {
      if bodies.get(h2) is Some(rb2) {
        if rb2.is_dynamic() && rb2.is_enabled() {
          inv_mass2 = body_inv_mass(rb2)
          inv_inertia2 = body_inv_inertia(rb2)
        }
      }
    }
    let tangent = @core.Vec2::new(-normal.y, normal.x)
    let mut denom_n = inv_mass1 + inv_mass2
    let mut denom_t = inv_mass1 + inv_mass2
    if body1 is Some(h1) {
      if bodies.get(h1) is Some(rb1) {
        let r1 = contact_point.sub(rb1.world_com())
        let rn = cross_vec2(r1, normal)
        let rt = cross_vec2(r1, tangent)
        denom_n = denom_n + inv_inertia1 * rn * rn
        denom_t = denom_t + inv_inertia1 * rt * rt
      }
    }
    if body2 is Some(h2) {
      if bodies.get(h2) is Some(rb2) {
        let r2 = contact_point.sub(rb2.world_com())
        let rn = cross_vec2(r2, normal)
        let rt = cross_vec2(r2, tangent)
        denom_n = denom_n + inv_inertia2 * rn * rn
        denom_t = denom_t + inv_inertia2 * rt * rt
      }
    }
    let inv_mass_n = if denom_n <= 1.0e-12F { 0.0F } else { 1.0F / denom_n }
    let inv_mass_t = if denom_t <= 1.0e-12F { 0.0F } else { 1.0F / denom_t }

    // Baumgarte bias is negative so `-(vn + bias)` yields a positive impulse when penetrating.
    let bias = if dt > 0.0F {
      -(SOLVER_BAUMGARTE / dt) *
      (if penetration > SOLVER_PENETRATION_SLOP {
        penetration - SOLVER_PENETRATION_SLOP
      } else {
        0.0F
      })
    } else {
      0.0F
    }
    let friction = @dynamics.CoefficientCombineRule::combine(
      c1.friction(),
      c2.friction(),
      c1.friction_combine_rule(),
      c2.friction_combine_rule(),
    )
    let restitution = @dynamics.CoefficientCombineRule::combine(
      c1.restitution(),
      c2.restitution(),
      c1.restitution_combine_rule(),
      c2.restitution_combine_rule(),
    )
    Some(ContactConstraint::{
      collider1,
      collider2,
      body1,
      body2,
      normal,
      contact_point,
      penetration,
      restitution,
      friction,
      inv_mass_n,
      inv_mass_t,
      bias,
      impulse_n: 0.0F,
      impulse_t: 0.0F,
    })
  } else {
    None
  }
}

///|
fn apply_impulse_delta(
  bodies : @dynamics.RigidBodySet,
  handle : @dynamics.RigidBodyHandle?,
  impulse : @core.Vec2,
  point : @core.Vec2,
) -> Unit {
  if handle is Some(h) {
    if bodies.get_mut(h) is Some(body) {
      if body.is_dynamic() && body.is_enabled() {
        let inv_m = body_inv_mass(body)
        let inv_i = body_inv_inertia(body)
        let new_linvel = body.linvel().add(solver_vec2_scale(impulse, inv_m))
        body.set_linvel(new_linvel, true) |> ignore
        let r = point.sub(body.world_com())
        let torque_impulse = cross_vec2(r, impulse)
        body.set_angvel(body.angvel() + torque_impulse * inv_i, true) |> ignore
      }
    }
  }
}

///|
fn relative_velocity_at_point(
  body : @dynamics.RigidBody,
  point : @core.Vec2,
) -> @core.Vec2 {
  let r = point.sub(body.world_com())
  body.linvel().add(cross_z_scalar(body.angvel(), r))
}

///|
fn solve_contact_constraints(
  bodies : @dynamics.RigidBodySet,
  colliders : @collision.ColliderSet,
  dt : @core.Real,
  event_handler : EventHandler,
  constraints : Array[ContactConstraint],
) -> Unit {
  if dt <= 0.0F {
    return
  }
  for _ in 0..<SOLVER_ITERATIONS {
    for i in 0..<constraints.length() {
      let c = constraints[i]
      let tangent = @core.Vec2::new(-c.normal.y, c.normal.x)
      let mut v1 = @core.Vec2::zero()
      let mut w1 = 0.0F
      let mut com1 = @core.Vec2::zero()
      if c.body1 is Some(h1) {
        if bodies.get(h1) is Some(rb1) {
          v1 = rb1.linvel()
          w1 = rb1.angvel()
          com1 = rb1.world_com()
        }
      }
      let mut v2 = @core.Vec2::zero()
      let mut w2 = 0.0F
      let mut com2 = @core.Vec2::zero()
      if c.body2 is Some(h2) {
        if bodies.get(h2) is Some(rb2) {
          v2 = rb2.linvel()
          w2 = rb2.angvel()
          com2 = rb2.world_com()
        }
      }
      let r1 = c.contact_point.sub(com1)
      let r2 = c.contact_point.sub(com2)
      let vel1 = v1.add(cross_z_scalar(w1, r1))
      let vel2 = v2.add(cross_z_scalar(w2, r2))
      let rv = vel1.sub(vel2)

      // Normal impulse (with restitution + bias).
      let vn = rv.dot(c.normal)
      let restitution_term = if vn < -1.0e-3F {
        c.restitution * vn
      } else {
        0.0F
      }
      let lambda_n = -(vn + c.bias + restitution_term) * c.inv_mass_n
      let new_impulse_n = if c.impulse_n + lambda_n > 0.0F {
        c.impulse_n + lambda_n
      } else {
        0.0F
      }
      let delta_n = new_impulse_n - c.impulse_n
      constraints[i].impulse_n = new_impulse_n
      let impulse_n_vec = solver_vec2_scale(c.normal, delta_n)
      apply_impulse_delta(bodies, c.body1, impulse_n_vec, c.contact_point)
      apply_impulse_delta(
        bodies,
        c.body2,
        @core.Vec2::new(-impulse_n_vec.x, -impulse_n_vec.y),
        c.contact_point,
      )

      // Recompute relative velocity after normal impulse for friction.
      let mut vel1_after = @core.Vec2::zero()
      if c.body1 is Some(h1a) {
        if bodies.get(h1a) is Some(rb1a) {
          vel1_after = relative_velocity_at_point(rb1a, c.contact_point)
        }
      }
      let mut vel2_after = @core.Vec2::zero()
      if c.body2 is Some(h2a) {
        if bodies.get(h2a) is Some(rb2a) {
          vel2_after = relative_velocity_at_point(rb2a, c.contact_point)
        }
      }
      let rv_after = vel1_after.sub(vel2_after)
      let vt = rv_after.dot(tangent)
      let lambda_t = -vt * c.inv_mass_t
      let max_friction = c.friction * constraints[i].impulse_n
      let new_impulse_t = solver_clamp_value(
        constraints[i].impulse_t + lambda_t,
        -max_friction,
        max_friction,
      )
      let delta_t = new_impulse_t - constraints[i].impulse_t
      constraints[i].impulse_t = new_impulse_t
      let impulse_t_vec = solver_vec2_scale(tangent, delta_t)
      apply_impulse_delta(bodies, c.body1, impulse_t_vec, c.contact_point)
      apply_impulse_delta(
        bodies,
        c.body2,
        @core.Vec2::new(-impulse_t_vec.x, -impulse_t_vec.y),
        c.contact_point,
      )
    }
  }

  // Generate contact force events based on the solved normal impulse magnitude.
  for i in 0..<constraints.length() {
    let c = constraints[i]
    if colliders.get(c.collider1) is Some(co1) &&
      colliders.get(c.collider2) is Some(co2) {
      let flag = @collision.ActiveEvents::contact_force_events()
      let max_threshold = 1.0e30F
      let t1 = if co1.active_events().contains(flag) {
        co1.contact_force_event_threshold()
      } else {
        max_threshold
      }
      let t2 = if co2.active_events().contains(flag) {
        co2.contact_force_event_threshold()
      } else {
        max_threshold
      }
      let threshold = if t1 < t2 { t1 } else { t2 }
      if threshold < max_threshold {
        let magnitude = c.impulse_n / dt
        if magnitude > threshold {
          let total_force = solver_vec2_scale(c.normal, magnitude)
          event_handler.handle_contact_force_event(
            dt,
            bodies,
            colliders,
            @collision.ContactForceEvent::new(
              c.collider1,
              c.collider2,
              total_force,
              magnitude,
              c.normal,
              magnitude,
            ),
          )
        }
      }
    }
  }
}

///|
fn position_correction(
  bodies : @dynamics.RigidBodySet,
  colliders : @collision.ColliderSet,
  constraints : Array[ContactConstraint],
) -> Unit {
  colliders |> ignore
  for i in 0..<constraints.length() {
    let c = constraints[i]
    if c.penetration <= SOLVER_PENETRATION_SLOP {
      continue
    }
    let correction_mag = (c.penetration - SOLVER_PENETRATION_SLOP) *
      SOLVER_POSITION_CORRECTION_PERCENT
    let correction = solver_vec2_scale(c.normal, correction_mag)
    let mut inv_m1 = 0.0F
    if c.body1 is Some(h1) {
      if bodies.get(h1) is Some(rb1) {
        if rb1.is_dynamic() && rb1.is_enabled() {
          inv_m1 = body_inv_mass(rb1)
        }
      }
    }
    let mut inv_m2 = 0.0F
    if c.body2 is Some(h2) {
      if bodies.get(h2) is Some(rb2) {
        if rb2.is_dynamic() && rb2.is_enabled() {
          inv_m2 = body_inv_mass(rb2)
        }
      }
    }
    let denom = inv_m1 + inv_m2
    if denom <= 1.0e-12F {
      continue
    }
    let ratio1 = inv_m1 / denom
    let ratio2 = inv_m2 / denom
    if c.body1 is Some(h1m) {
      if bodies.get_mut(h1m) is Some(b1) {
        let pos = b1.translation()
        b1.set_translation(pos.add(solver_vec2_scale(correction, ratio1)), true)
        |> ignore
      }
    }
    if c.body2 is Some(h2m) {
      if bodies.get_mut(h2m) is Some(b2) {
        let pos = b2.translation()
        b2.set_translation(pos.sub(solver_vec2_scale(correction, ratio2)), true)
        |> ignore
      }
    }
  }
}

///|
pub fn solve_contacts(
  bodies : @dynamics.RigidBodySet,
  colliders : @collision.ColliderSet,
  narrow_phase : @collision.NarrowPhase,
  dt : @core.Real,
  event_handler : EventHandler,
) -> Unit {
  let pairs = narrow_phase.contact_pairs()
  let constraints : Array[ContactConstraint] = []
  for i in 0..<pairs.length() {
    let pair = pairs[i]
    if build_contact_constraint(bodies, colliders, pair.0, pair.1, dt)
      is Some(c) {
      constraints.push(c)
    }
  }
  solve_contact_constraints(bodies, colliders, dt, event_handler, constraints)
  position_correction(bodies, colliders, constraints)
}
