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
/// A Rapier-style sequential impulse contact solver (2D) built from NarrowPhase contact manifolds.
/// It supports:
/// - multiple contact points per manifold (e.g. cuboid-cuboid clipping)
/// - restitution + Coulomb friction
/// - warmstarting through a per-pipeline cache keyed by collider handles + feature ids
///
/// This is still simplified compared to Rapier (no island-parallelism, no contact skin / prediction,
/// no manifold caching inside NarrowPhase), but it matches the "constraint from manifold" design.

///|
pub struct ContactImpulseCacheEntry {
  collider1 : @collision.ColliderHandle
  collider2 : @collision.ColliderHandle
  fid1 : @collision.FeatureId
  fid2 : @collision.FeatureId
  impulse_n : @core.Real
  impulse_t : @core.Real
}

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
  if v.dot(v) <= 1.0e-12F {
    fallback
  } else {
    v.normalize()
  }
}

///|
fn solver_collider_handle_equals(
  a : @collision.ColliderHandle,
  b : @collision.ColliderHandle,
) -> Bool {
  @collision.ColliderHandle::equals(a, b)
}

///|
fn feature_id_equals(
  a : @collision.FeatureId,
  b : @collision.FeatureId,
) -> Bool {
  match (a, b) {
    (@collision.FeatureId::Unknown, @collision.FeatureId::Unknown) => true
    (@collision.FeatureId::Vertex(i), @collision.FeatureId::Vertex(j)) => i == j
    (@collision.FeatureId::Edge(i), @collision.FeatureId::Edge(j)) => i == j
    (@collision.FeatureId::Face(i), @collision.FeatureId::Face(j)) => i == j
    _ => false
  }
}

///|
fn cache_key_equals(
  entry : ContactImpulseCacheEntry,
  collider1 : @collision.ColliderHandle,
  collider2 : @collision.ColliderHandle,
  fid1 : @collision.FeatureId,
  fid2 : @collision.FeatureId,
) -> Bool {
  solver_collider_handle_equals(entry.collider1, collider1) &&
  solver_collider_handle_equals(entry.collider2, collider2) &&
  feature_id_equals(entry.fid1, fid1) &&
  feature_id_equals(entry.fid2, fid2)
}

///|
fn cache_find(
  cache : Array[ContactImpulseCacheEntry],
  collider1 : @collision.ColliderHandle,
  collider2 : @collision.ColliderHandle,
  fid1 : @collision.FeatureId,
  fid2 : @collision.FeatureId,
) -> ContactImpulseCacheEntry? {
  for i in 0..<cache.length() {
    let entry = cache[i]
    if cache_key_equals(entry, collider1, collider2, fid1, fid2) {
      return Some(entry)
    }
  }
  None
}

///|
fn apply_impulse_delta_dynamic(
  bodies : @dynamics.RigidBodySet,
  handle : @dynamics.RigidBodyHandle?,
  impulse : @core.Vec2,
  point : @core.Vec2,
) -> Unit {
  if handle is Some(h) {
    if bodies.get_mut(h) is Some(body) {
      if body.is_dynamic() && body.is_enabled() {
        let inv_m = if body.locked_translations {
          0.0F
        } else {
          body.inv_mass()
        }
        let inv_i = if body.locked_rotations {
          0.0F
        } else {
          body.inv_inertia()
        }
        body.set_linvel(
          body.linvel().add(solver_vec2_scale(impulse, inv_m)),
          true,
        )
        |> ignore
        let r = point.sub(body.world_com())
        let torque_impulse = cross_vec2(r, impulse)
        body.set_angvel(body.angvel() + torque_impulse * inv_i, true) |> ignore
      }
    }
  }
}

///|
priv struct SolverConstraint {
  collider1 : @collision.ColliderHandle
  collider2 : @collision.ColliderHandle
  fid1 : @collision.FeatureId
  fid2 : @collision.FeatureId
  body1 : @dynamics.RigidBodyHandle?
  body2 : @dynamics.RigidBodyHandle?
  // Direction of the force/impulse applied to body1 (Rapier: -manifold.normal).
  dir1 : @core.Vec2
  tangent1 : @core.Vec2
  // Target relative velocity along `tangent1` (Rapier's SolverContact.tangent_velocity projected).
  tangent_velocity : @core.Real
  contact_point : @core.Vec2
  dist : @core.Real
  restitution_vel : @core.Real
  friction : @core.Real
  inv_mass_n : @core.Real
  inv_mass_n_nocfm : @core.Real
  inv_mass_t : @core.Real
  rhs_bias : @core.Real
  mut impulse_n : @core.Real
  mut impulse_t : @core.Real
}

///|
fn build_constraints(
  parameters : IntegrationParameters,
  bodies : @dynamics.RigidBodySet,
  colliders : @collision.ColliderSet,
  hooks : PhysicsHooks,
  narrow_phase : @collision.NarrowPhase,
  dt : @core.Real,
  cache : Array[ContactImpulseCacheEntry],
  warmstart_coefficient : @core.Real,
) -> Array[SolverConstraint] {
  let allowed_error = parameters.allowed_linear_error()
  let max_corrective_velocity = parameters.max_corrective_velocity()
  let erp_inv_dt = parameters.contact_softness.erp_inv_dt(dt)
  let cfm_coeff = parameters.contact_softness.cfm_coeff(dt)
  let pairs = narrow_phase.contact_pairs()
  let constraints : Array[SolverConstraint] = []
  for i in 0..<pairs.length() {
    let pair = pairs[i]
    let h1 = pair.0
    let h2 = pair.1
    if colliders.get(h1) is Some(c1) && colliders.get(h2) is Some(c2) {
      if c1.is_sensor() || c2.is_sensor() {
        continue
      }
      if !c1.solver_groups().test_groups(c2.solver_groups()) {
        continue
      }
      let body1 = c1.parent()
      let body2 = c2.parent()

      // Like Rapier: only solve if at least one body is dynamic.
      let mut any_dynamic = false
      if body1 is Some(bh1) {
        if bodies.get(bh1) is Some(rb1) {
          if rb1.is_dynamic() && rb1.is_enabled() {
            any_dynamic = true
          }
        }
      }
      if body2 is Some(bh2) {
        if bodies.get(bh2) is Some(rb2) {
          if rb2.is_dynamic() && rb2.is_enabled() {
            any_dynamic = true
          }
        }
      }
      if !any_dynamic {
        continue
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

      // For some manifolds (segment-* and capsule-*), we may not have a negative distance for
      // penetrations. Use the separation depth as a fallback to preserve depenetration behavior.
      let sep = separation_vector(
        c1.shape(),
        c1.translation(),
        c1.rotation(),
        c2.shape(),
        c2.translation(),
        c2.rotation(),
      )
      let sep_vec = if sep is Some(v) { v } else { @core.Vec2::zero() }
      let sep_depth = sep_vec.length()
      let sep_dir1 = if sep_depth > 1.0e-12F {
        // sep points from collider2 toward collider1, so it matches dir1 convention.
        sep_vec.normalize()
      } else {
        @core.Vec2::zero()
      }
      let manifolds = pair.2.manifolds()
      for mi in 0..<manifolds.length() {
        let manifold = manifolds[mi]
        // Rapier: dir1 = -manifold.normal (normal points from collider1 to collider2).
        let dir1_from_manifold = normalize_or_fallback(
          @core.Vec2::new(-manifold.normal().x, -manifold.normal().y),
          @core.Vec2::new(1.0F, 0.0F),
        )
        let dir1 = if sep_depth > 0.0F { sep_dir1 } else { dir1_from_manifold }
        let tangent1 = @core.Vec2::new(-dir1.y, dir1.x)
        let mut tangent_velocity = @core.Vec2::zero()
        if c1
          .active_hooks()
          .contains(@collision.ActiveHooks::modify_solver_contacts()) ||
          c2
          .active_hooks()
          .contains(@collision.ActiveHooks::modify_solver_contacts()) {
          let (accept, tv) = hooks.modify_solver_contacts_internal(
            bodies, colliders, h1, h2, manifold,
          )
          if !accept {
            continue
          }
          tangent_velocity = tv
        }
        let tangent_velocity_proj = tangent_velocity.dot(tangent1)
        let has_segment = c1.shape() is @collision.Shape::Segment(_, _) ||
          c2.shape() is @collision.Shape::Segment(_, _) ||
          c1.shape() is @collision.Shape::Polyline(_, _) ||
          c2.shape() is @collision.Shape::Polyline(_, _)
        fn push_constraint(
          fid1 : @collision.FeatureId,
          fid2 : @collision.FeatureId,
          contact_point : @core.Vec2,
          dist_in : @core.Real,
        ) -> Unit {
          let mut dist = dist_in
          // Some manifold generators (notably segment-*) only provide a non-negative distance.
          // When we have an MTV from `separation_vector`, use it to recover a penetration depth.
          if has_segment && sep_depth > 0.0F && dist >= 0.0F {
            dist = -sep_depth
          }
          // A small velocity-level bias is needed so overlapping objects separate even when their
          // relative velocity is zero (this also drives contact-force events).
          let rhs_bias0 = solver_clamp_value(
            erp_inv_dt * (dist + allowed_error),
            -max_corrective_velocity,
            0.0F,
          )
          let mut inv_mass1 = 0.0F
          let mut inv_inertia1 = 0.0F
          let mut v1 = @core.Vec2::zero()
          let mut w1 = 0.0F
          let mut com1 = @core.Vec2::zero()
          if body1 is Some(bh1) {
            if bodies.get(bh1) is Some(rb1) {
              v1 = rb1.linvel()
              w1 = rb1.angvel()
              com1 = rb1.world_com()
              if rb1.is_dynamic() && rb1.is_enabled() {
                inv_mass1 = if rb1.locked_translations {
                  0.0F
                } else {
                  rb1.inv_mass()
                }
                inv_inertia1 = if rb1.locked_rotations {
                  0.0F
                } else {
                  rb1.inv_inertia()
                }
              }
            }
          }
          let mut inv_mass2 = 0.0F
          let mut inv_inertia2 = 0.0F
          let mut v2 = @core.Vec2::zero()
          let mut w2 = 0.0F
          let mut com2 = @core.Vec2::zero()
          if body2 is Some(bh2) {
            if bodies.get(bh2) is Some(rb2) {
              v2 = rb2.linvel()
              w2 = rb2.angvel()
              com2 = rb2.world_com()
              if rb2.is_dynamic() && rb2.is_enabled() {
                inv_mass2 = if rb2.locked_translations {
                  0.0F
                } else {
                  rb2.inv_mass()
                }
                inv_inertia2 = if rb2.locked_rotations {
                  0.0F
                } else {
                  rb2.inv_inertia()
                }
              }
            }
          }

          // Effective mass denominators (2D).
          let r1 = contact_point.sub(com1)
          let r2 = contact_point.sub(com2)
          let rn1 = cross_vec2(r1, dir1)
          let rt1 = cross_vec2(r1, tangent1)
          let rn2 = cross_vec2(r2, dir1)
          let rt2 = cross_vec2(r2, tangent1)
          let denom_n = inv_mass1 +
            inv_mass2 +
            inv_inertia1 * rn1 * rn1 +
            inv_inertia2 * rn2 * rn2
          let denom_t = inv_mass1 +
            inv_mass2 +
            inv_inertia1 * rt1 * rt1 +
            inv_inertia2 * rt2 * rt2
          let inv_mass_n_nocfm = if denom_n <= 1.0e-12F {
            0.0F
          } else {
            1.0F / denom_n
          }
          let inv_mass_n = if denom_n + cfm_coeff <= 1.0e-12F {
            0.0F
          } else {
            1.0F / (denom_n + cfm_coeff)
          }
          let inv_mass_t = if denom_t <= 1.0e-12F {
            0.0F
          } else {
            1.0F / denom_t
          }
          let cached = cache_find(cache, h1, h2, fid1, fid2)
          let impulse_n = if cached is Some(e) {
            e.impulse_n * warmstart_coefficient
          } else {
            0.0F
          }
          let impulse_t = if cached is Some(e) {
            e.impulse_t * warmstart_coefficient
          } else {
            0.0F
          }
          // Restitution is applied as a velocity target based on the pre-solve normal speed,
          // like Rapier (use the initial impact speed, not the iterative vn which changes).
          let vel1 = v1.add(cross_z_scalar(w1, r1))
          let vel2 = v2.add(cross_z_scalar(w2, r2))
          let rv = vel1.sub(vel2)
          let vn0 = rv.dot(dir1)
          let restitution_vel = if dist <= 0.0F && vn0 < -1.0e-3F {
            -restitution * vn0
          } else {
            0.0F
          }
          // We keep the bias smaller than Rapier's default spring model because we also apply
          // explicit position corrections; a full split-impulse solver would be ideal, but this
          // balance avoids the high-mass-ratio "launch" instability while still resolving overlaps.
          let rhs_bias = rhs_bias0 * 0.2F
          constraints.push(SolverConstraint::{
            collider1: h1,
            collider2: h2,
            fid1,
            fid2,
            body1,
            body2,
            dir1,
            tangent1,
            tangent_velocity: tangent_velocity_proj,
            contact_point,
            dist,
            restitution_vel,
            friction,
            inv_mass_n,
            inv_mass_n_nocfm,
            inv_mass_t,
            rhs_bias,
            impulse_n,
            impulse_t,
          })
          // Precomputed vels are not stored, but this keeps the constraint build close to Rapier.
          v1 |> ignore
          w1 |> ignore
          v2 |> ignore
          w2 |> ignore
        }

        let points = manifold.points()
        if points.length() == 0 {
          continue
        }
        if has_segment {
          for pi in 0..<points.length() {
            let p = points[pi]
            let world_p1 = c1.position().transform_point(p.local_p1())
            let world_p2 = c2.position().transform_point(p.local_p2())
            let contact_point = @core.Vec2::new(
              (world_p1.x + world_p2.x) * 0.5F,
              (world_p1.y + world_p2.y) * 0.5F,
            )
            push_constraint(p.fid1(), p.fid2(), contact_point, p.dist())
          }
        } else {
          // Rapier solves up to two points per manifold with a block solver. Our simplified
          // implementation converges poorly for tall stacks if we treat each point independently,
          // because it introduces spurious torques. Use the averaged contact point instead.
          let mut world_p1_sum = @core.Vec2::zero()
          let mut world_p2_sum = @core.Vec2::zero()
          let mut dist_sum = 0.0F
          let p0 = points[0]
          for pi in 0..<points.length() {
            let p = points[pi]
            let wp1 = c1.position().transform_point(p.local_p1())
            let wp2 = c2.position().transform_point(p.local_p2())
            world_p1_sum = world_p1_sum.add(wp1)
            world_p2_sum = world_p2_sum.add(wp2)
            dist_sum = dist_sum + p.dist()
          }
          let inv_n = 1.0F / Float::from_double(points.length().to_double())
          let world_p1 = solver_vec2_scale(world_p1_sum, inv_n)
          let world_p2 = solver_vec2_scale(world_p2_sum, inv_n)
          let contact_point = @core.Vec2::new(
            (world_p1.x + world_p2.x) * 0.5F,
            (world_p1.y + world_p2.y) * 0.5F,
          )
          push_constraint(p0.fid1(), p0.fid2(), contact_point, dist_sum * inv_n)
        }
      }
    }
  }
  constraints
}

///|
fn warmstart(
  bodies : @dynamics.RigidBodySet,
  constraints : Array[SolverConstraint],
) -> Unit {
  for i in 0..<constraints.length() {
    let c = constraints[i]
    let impulse = solver_vec2_scale(c.dir1, c.impulse_n).add(
      solver_vec2_scale(c.tangent1, c.impulse_t),
    )
    apply_impulse_delta_dynamic(bodies, c.body1, impulse, c.contact_point)
    apply_impulse_delta_dynamic(
      bodies,
      c.body2,
      @core.Vec2::new(-impulse.x, -impulse.y),
      c.contact_point,
    )
  }
}

///|
fn get_body_kinematic_state(
  bodies : @dynamics.RigidBodySet,
  handle : @dynamics.RigidBodyHandle?,
) -> (@core.Vec2, @core.Real, @core.Vec2) {
  if handle is Some(h) {
    if bodies.get(h) is Some(rb) {
      (rb.linvel(), rb.angvel(), rb.world_com())
    } else {
      (@core.Vec2::zero(), 0.0F, @core.Vec2::zero())
    }
  } else {
    (@core.Vec2::zero(), 0.0F, @core.Vec2::zero())
  }
}

///|
fn solve_velocity_constraints(
  bodies : @dynamics.RigidBodySet,
  dt : @core.Real,
  constraints : Array[SolverConstraint],
  iterations : Int,
) -> Unit {
  if dt <= 0.0F {
    return
  }
  let inv_dt = 1.0F / dt
  let iters = if iterations <= 0 { 1 } else { iterations }
  for _ in 0..<iters {
    // Solve all normal constraints first (better convergence for stacks).
    for i in 0..<constraints.length() {
      let c = constraints[i]
      let (v1, w1, com1) = get_body_kinematic_state(bodies, c.body1)
      let (v2, w2, com2) = get_body_kinematic_state(bodies, c.body2)
      let r1 = c.contact_point.sub(com1)
      let r2 = c.contact_point.sub(com2)
      let vel1 = v1.add(cross_z_scalar(w1, r1))
      let vel2 = v2.add(cross_z_scalar(w2, r2))
      let rv = vel1.sub(vel2)
      let vn = rv.dot(c.dir1)
      let sep_term = if c.dist > 0.0F { c.dist * inv_dt } else { 0.0F }
      let lambda_n = -(vn + sep_term + c.rhs_bias) * c.inv_mass_n
      let new_impulse_n = if constraints[i].impulse_n + lambda_n > 0.0F {
        constraints[i].impulse_n + lambda_n
      } else {
        0.0F
      }
      let delta_n = new_impulse_n - constraints[i].impulse_n
      constraints[i].impulse_n = new_impulse_n
      let impulse_n_vec = solver_vec2_scale(c.dir1, delta_n)
      apply_impulse_delta_dynamic(
        bodies,
        c.body1,
        impulse_n_vec,
        c.contact_point,
      )
      apply_impulse_delta_dynamic(
        bodies,
        c.body2,
        @core.Vec2::new(-impulse_n_vec.x, -impulse_n_vec.y),
        c.contact_point,
      )
    }

    // Then solve all friction constraints using the updated normal impulses.
    for i in 0..<constraints.length() {
      let c = constraints[i]
      let (v1, w1, com1) = get_body_kinematic_state(bodies, c.body1)
      let (v2, w2, com2) = get_body_kinematic_state(bodies, c.body2)
      let r1 = c.contact_point.sub(com1)
      let r2 = c.contact_point.sub(com2)
      let vel1 = v1.add(cross_z_scalar(w1, r1))
      let vel2 = v2.add(cross_z_scalar(w2, r2))
      let rv = vel1.sub(vel2)
      let vt_err = rv.dot(c.tangent1) - c.tangent_velocity
      let lambda_t = -vt_err * c.inv_mass_t
      let max_friction = c.friction * constraints[i].impulse_n
      let new_impulse_t = solver_clamp_value(
        constraints[i].impulse_t + lambda_t,
        -max_friction,
        max_friction,
      )
      let delta_t = new_impulse_t - constraints[i].impulse_t
      constraints[i].impulse_t = new_impulse_t
      let impulse_t_vec = solver_vec2_scale(c.tangent1, delta_t)
      apply_impulse_delta_dynamic(
        bodies,
        c.body1,
        impulse_t_vec,
        c.contact_point,
      )
      apply_impulse_delta_dynamic(
        bodies,
        c.body2,
        @core.Vec2::new(-impulse_t_vec.x, -impulse_t_vec.y),
        c.contact_point,
      )
    }
  }
}

///|
fn solve_restitution_constraints(
  bodies : @dynamics.RigidBodySet,
  constraints : Array[SolverConstraint],
) -> Unit {
  for i in 0..<constraints.length() {
    let c = constraints[i]
    if c.restitution_vel == 0.0F {
      continue
    }
    let (v1, w1, com1) = get_body_kinematic_state(bodies, c.body1)
    let (v2, w2, com2) = get_body_kinematic_state(bodies, c.body2)
    let r1 = c.contact_point.sub(com1)
    let r2 = c.contact_point.sub(com2)
    let vel1 = v1.add(cross_z_scalar(w1, r1))
    let vel2 = v2.add(cross_z_scalar(w2, r2))
    let rv = vel1.sub(vel2)
    let vn = rv.dot(c.dir1)
    let lambda_n = -(vn - c.restitution_vel) * c.inv_mass_n_nocfm
    let new_impulse_n = if constraints[i].impulse_n + lambda_n > 0.0F {
      constraints[i].impulse_n + lambda_n
    } else {
      0.0F
    }
    let delta_n = new_impulse_n - constraints[i].impulse_n
    constraints[i].impulse_n = new_impulse_n
    let impulse_n_vec = solver_vec2_scale(c.dir1, delta_n)
    apply_impulse_delta_dynamic(bodies, c.body1, impulse_n_vec, c.contact_point)
    apply_impulse_delta_dynamic(
      bodies,
      c.body2,
      @core.Vec2::new(-impulse_n_vec.x, -impulse_n_vec.y),
      c.contact_point,
    )
  }
}

///|
fn position_correction(
  bodies : @dynamics.RigidBodySet,
  colliders : @collision.ColliderSet,
  constraints : Array[SolverConstraint],
  allowed_error : @core.Real,
) -> Unit {
  for i in 0..<constraints.length() {
    let c = constraints[i]
    // Position correction (split-impulse style):
    // - We keep this separate from velocity solving to avoid injecting energy.
    // - For segments/polylines we prefer an MTV (separation_vector), because manifold
    //   point distances are often too coarse for degenerate geometry.
    let mut has_segment = false
    if colliders.get(c.collider1) is Some(co1) {
      has_segment = has_segment ||
        co1.shape() is @collision.Shape::Segment(_, _) ||
        co1.shape() is @collision.Shape::Polyline(_, _)
    }
    if colliders.get(c.collider2) is Some(co2) {
      has_segment = has_segment ||
        co2.shape() is @collision.Shape::Segment(_, _) ||
        co2.shape() is @collision.Shape::Polyline(_, _)
    }
    let mut has_convex = false
    if colliders.get(c.collider1) is Some(co1) {
      has_convex = has_convex ||
        co1.shape() is @collision.Shape::ConvexPolygon(_)
    }
    if colliders.get(c.collider2) is Some(co2) {
      has_convex = has_convex ||
        co2.shape() is @collision.Shape::ConvexPolygon(_)
    }
    if c.dist >= -allowed_error {
      continue
    }
    let depth = -c.dist - allowed_error
    // For non-segment contacts, keep the correction small to avoid injecting visible jitter
    // into resting stacks (Rapier relies on a dedicated stabilization stage).
    let factor = if has_segment || has_convex { 1.0F } else { 0.2F }
    let mut correction = solver_vec2_scale(c.dir1, depth * factor + 1.0e-3F)
    if has_segment &&
      colliders.get(c.collider1) is Some(co1) &&
      colliders.get(c.collider2) is Some(co2) {
      if separation_vector(
          co1.shape(),
          co1.translation(),
          co1.rotation(),
          co2.shape(),
          co2.translation(),
          co2.rotation(),
        )
        is Some(sep) {
        let depth = sep.length()
        if depth > 1.0e-6F {
          let dir = sep.normalize()
          correction = solver_vec2_scale(dir, depth + 1.0e-3F)
        }
      }
    }
    let mut inv_m1 = 0.0F
    if c.body1 is Some(h1) {
      if bodies.get(h1) is Some(rb1) {
        if rb1.is_dynamic() && rb1.is_enabled() {
          inv_m1 = if rb1.locked_translations { 0.0F } else { rb1.inv_mass() }
        }
      }
    }
    let mut inv_m2 = 0.0F
    if c.body2 is Some(h2) {
      if bodies.get(h2) is Some(rb2) {
        if rb2.is_dynamic() && rb2.is_enabled() {
          inv_m2 = if rb2.locked_translations { 0.0F } else { rb2.inv_mass() }
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
        if b1.is_dynamic() && b1.is_enabled() && !b1.locked_translations {
          b1.set_translation(
            b1.translation().add(solver_vec2_scale(correction, ratio1)),
            true,
          )
          |> ignore
        }
      }
    }
    if c.body2 is Some(h2m) {
      if bodies.get_mut(h2m) is Some(b2) {
        if b2.is_dynamic() && b2.is_enabled() && !b2.locked_translations {
          b2.set_translation(
            b2.translation().sub(solver_vec2_scale(correction, ratio2)),
            true,
          )
          |> ignore
        }
      }
    }
  }
}

///|
const CONTACT_POSITION_CORRECTION_BAUMGARTE : @core.Real = 0.2F

///|
fn position_correction_all(
  bodies : @dynamics.RigidBodySet,
  colliders : @collision.ColliderSet,
  constraints : Array[SolverConstraint],
  allowed_error : @core.Real,
) -> Unit {
  for i in 0..<constraints.length() {
    let c = constraints[i]
    // Segment penetrations are handled by `position_correction` (stronger depenetration).
    if colliders.get(c.collider1) is Some(co1) &&
      (
        co1.shape() is @collision.Shape::Segment(_, _) ||
        co1.shape() is @collision.Shape::Polyline(_, _)
      ) {
      continue
    }
    if colliders.get(c.collider2) is Some(co2) &&
      (
        co2.shape() is @collision.Shape::Segment(_, _) ||
        co2.shape() is @collision.Shape::Polyline(_, _)
      ) {
      continue
    }
    if c.dist >= -allowed_error {
      continue
    }
    // Apply only a fraction of the penetration depth to reduce energy injection.
    let correction_mag = -c.dist * CONTACT_POSITION_CORRECTION_BAUMGARTE
    let correction = solver_vec2_scale(c.dir1, correction_mag)
    let mut inv_m1 = 0.0F
    if c.body1 is Some(h1) {
      if bodies.get(h1) is Some(rb1) {
        if rb1.is_dynamic() && rb1.is_enabled() {
          inv_m1 = if rb1.locked_translations { 0.0F } else { rb1.inv_mass() }
        }
      }
    }
    let mut inv_m2 = 0.0F
    if c.body2 is Some(h2) {
      if bodies.get(h2) is Some(rb2) {
        if rb2.is_dynamic() && rb2.is_enabled() {
          inv_m2 = if rb2.locked_translations { 0.0F } else { rb2.inv_mass() }
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
        if b1.is_dynamic() && b1.is_enabled() && !b1.locked_translations {
          b1.set_translation(
            b1.translation().add(solver_vec2_scale(correction, ratio1)),
            true,
          )
          |> ignore
        }
      }
    }
    if c.body2 is Some(h2m) {
      if bodies.get_mut(h2m) is Some(b2) {
        if b2.is_dynamic() && b2.is_enabled() && !b2.locked_translations {
          b2.set_translation(
            b2.translation().sub(solver_vec2_scale(correction, ratio2)),
            true,
          )
          |> ignore
        }
      }
    }
  }
}

///|
fn emit_contact_force_events(
  bodies : @dynamics.RigidBodySet,
  colliders : @collision.ColliderSet,
  dt : @core.Real,
  event_handler : EventHandler,
  constraints : Array[SolverConstraint],
) -> Unit {
  if dt <= 0.0F {
    return
  }
  // Rapier parity:
  // - total_force = sum(manifold_normal * total_manifold_impulse) / dt
  // - total_force_magnitude = sum(total_manifold_impulse) / dt (not |total_force|)
  // - max_force_{direction,magnitude} comes from the strongest individual contact impulse.
  let pairs : Array[
    (
      @collision.ColliderHandle,
      @collision.ColliderHandle,
      @core.Vec2,
      @core.Real,
      @core.Vec2,
      @core.Real,
    ),
  ] = []
  for i in 0..<constraints.length() {
    let c = constraints[i]
    let normal = @core.Vec2::new(-c.dir1.x, -c.dir1.y)
    let impulse = c.impulse_n
    let impulse_vec = solver_vec2_scale(normal, impulse)
    let mut found = false
    for j in 0..<pairs.length() {
      let p = pairs[j]
      if solver_collider_handle_equals(p.0, c.collider1) &&
        solver_collider_handle_equals(p.1, c.collider2) {
        let max_dir = if impulse > p.5 { normal } else { p.4 }
        let max_impulse = if impulse > p.5 { impulse } else { p.5 }
        pairs[j] = (
          p.0,
          p.1,
          p.2.add(impulse_vec),
          p.3 + impulse,
          max_dir,
          max_impulse,
        )
        found = true
        break
      }
    }
    if !found {
      pairs.push(
        (c.collider1, c.collider2, impulse_vec, impulse, normal, impulse),
      )
    }
  }
  for i in 0..<pairs.length() {
    let p = pairs[i]
    let h1 = p.0
    let h2 = p.1
    if colliders.get(h1) is Some(co1) && colliders.get(h2) is Some(co2) {
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
        let inv_dt = 1.0F / dt
        let total_force_magnitude = p.3 * inv_dt
        if total_force_magnitude > threshold {
          let total_force = solver_vec2_scale(p.2, inv_dt)
          event_handler.handle_contact_force_event(
            dt,
            bodies,
            colliders,
            @collision.ContactForceEvent::new(
              h1,
              h2,
              total_force,
              total_force_magnitude,
              p.4,
              p.5 * inv_dt,
            ),
          )
        }
      }
    }
  }
}

///|
fn update_cache(
  cache : Array[ContactImpulseCacheEntry],
  constraints : Array[SolverConstraint],
) -> Unit {
  cache.clear()
  for i in 0..<constraints.length() {
    let c = constraints[i]
    cache.push(ContactImpulseCacheEntry::{
      collider1: c.collider1,
      collider2: c.collider2,
      fid1: c.fid1,
      fid2: c.fid2,
      impulse_n: c.impulse_n,
      impulse_t: c.impulse_t,
    })
  }
}

///|
pub fn solve_contacts(
  parameters : IntegrationParameters,
  bodies : @dynamics.RigidBodySet,
  colliders : @collision.ColliderSet,
  hooks : PhysicsHooks,
  narrow_phase : @collision.NarrowPhase,
  dt : @core.Real,
  event_handler : EventHandler,
  cache : Array[ContactImpulseCacheEntry],
) -> Unit {
  let warmstart_coefficient = if parameters.warmstart_coefficient < 0.0F {
    0.0F
  } else {
    parameters.warmstart_coefficient
  }
  let constraints = build_constraints(
    parameters, bodies, colliders, hooks, narrow_phase, dt, cache, warmstart_coefficient,
  )
  warmstart(bodies, constraints)
  solve_velocity_constraints(
    bodies,
    dt,
    constraints,
    parameters.num_solver_iterations,
  )
  solve_restitution_constraints(bodies, constraints)
  position_correction(
    bodies,
    colliders,
    constraints,
    parameters.allowed_linear_error(),
  )
  emit_contact_force_events(bodies, colliders, dt, event_handler, constraints)
  update_cache(cache, constraints)
}
