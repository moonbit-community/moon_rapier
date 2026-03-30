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
pub struct ContactImpulse3D {
  normal : @core.Real
  tangent1 : @core.Real
  tangent2 : @core.Real
}

///|
pub fn ContactImpulse3D::zero() -> ContactImpulse3D {
  { normal: 0.0F, tangent1: 0.0F, tangent2: 0.0F }
}

///|
pub fn ContactImpulse3D::scaled(
  self : ContactImpulse3D,
  coeff : @core.Real,
) -> ContactImpulse3D {
  {
    normal: self.normal * coeff,
    tangent1: self.tangent1 * coeff,
    tangent2: self.tangent2 * coeff,
  }
}

///|
pub struct ContactSolverCache3D {
  // For each collider pair, store the per-contact impulse accumulators with associated
  // contact points (world-space) so we can match them approximately next step.
  //
  // Performance-critical: use a hash map keyed by canonical raw handle parts.
  entries : @hashmap.HashMap[
    (Int, Int, Int, Int),
    Array[(@core.Vec3, @core.Vec3, ContactImpulse3D)],
  ]

  fn new() -> ContactSolverCache3D
}

///|
pub fn ContactSolverCache3D::new() -> ContactSolverCache3D {
  { entries: @hashmap.new() }
}

///|
pub fn ContactSolverCache3D::clear(self : ContactSolverCache3D) -> Unit {
  self.entries.clear()
}

///|
fn cache_key(
  a : @collision.ColliderHandle3D,
  b : @collision.ColliderHandle3D,
) -> (Int, Int, Int, Int) {
  let (ai, ag) = a.into_raw_parts()
  let (bi, bg) = b.into_raw_parts()
  if ai < bi || (ai == bi && ag <= bg) {
    (ai, ag, bi, bg)
  } else {
    (bi, bg, ai, ag)
  }
}

///|
fn cache_get(
  cache : ContactSolverCache3D,
  a : @collision.ColliderHandle3D,
  b : @collision.ColliderHandle3D,
) -> Array[(@core.Vec3, @core.Vec3, ContactImpulse3D)]? {
  cache.entries.get(cache_key(a, b))
}

///|
fn cache_set(
  cache : ContactSolverCache3D,
  a : @collision.ColliderHandle3D,
  b : @collision.ColliderHandle3D,
  contacts : Array[(@core.Vec3, @core.Vec3, ContactImpulse3D)],
) -> Unit {
  cache.entries.set(cache_key(a, b), contacts)
}

///|
fn rigid_body_stamp_for_island_filter(
  island_stamps : Array[Int]?,
  handle : @dynamics.RigidBodyHandle?,
) -> Int {
  if island_stamps is Some(stamps) && handle is Some(h) {
    let id = h.id
    if id >= 0 && id < stamps.length() {
      return stamps[id]
    }
  }
  0
}

///|
fn contact_pair_matches_island(
  island_stamps : Array[Int]?,
  island_stamp : Int,
  body1 : @dynamics.RigidBodyHandle?,
  body2 : @dynamics.RigidBodyHandle?,
) -> Bool {
  if island_stamps is None {
    return true
  }
  if island_stamp <= 0 {
    return false
  }
  let s1 = rigid_body_stamp_for_island_filter(island_stamps, body1)
  let s2 = rigid_body_stamp_for_island_filter(island_stamps, body2)
  if s1 > 0 && s2 > 0 && s1 != s2 {
    return false
  }
  s1 == island_stamp || s2 == island_stamp
}

///|
priv struct ContactSolveEntry3D {
  rb1 : @dynamics.RigidBody3D?
  rb2 : @dynamics.RigidBody3D?
  points : Array[ContactSolvePoint3D]
  cached : Array[(@core.Vec3, @core.Vec3, ContactImpulse3D)]
  friction : @core.Real
  friction_model : @dynamics.FrictionModel
  friction_normal : @core.Vec3
  friction_center : @core.Vec3
  friction_t1 : @core.Vec3
  friction_t2 : @core.Vec3
  friction_r0 : @core.Real
  friction_r1 : @core.Real
  friction_r01 : @core.Real
  twist_r : @core.Real
  twist_dists : Array[@core.Real]
  surface_vel : @core.Vec3
}

///|
priv struct ContactSolvePoint3D {
  point1 : @core.Vec3
  point2 : @core.Vec3
  normal : @core.Vec3
  t1 : @core.Vec3
  t2 : @core.Vec3
  kn : @core.Real
  kt1 : @core.Real
  kt2 : @core.Real
  target_vn : @core.Real
  bias : @core.Real
  penetration : @core.Real
}

///|

///|
fn orthonormal_basis(n : @core.Vec3) -> (@core.Vec3, @core.Vec3) {
  let fallback = if @core.abs(n.x) < 0.9F {
    @core.Vec3::new(1.0F, 0.0F, 0.0F)
  } else {
    @core.Vec3::new(0.0F, 1.0F, 0.0F)
  }
  let t1 = n.cross(fallback).normalize()
  let t2 = n.cross(t1)
  (t1, t2)
}

///|
fn local_sqrt(value : @core.Real) -> @core.Real {
  if value <= 0.0F {
    0.0F
  } else {
    let mut x = value
    for _ in 0..<8 {
      x = 0.5F * (x + value / x)
    }
    x
  }
}

///|
fn world_inv_inertia(body : @dynamics.RigidBody3D) -> @core.Mat3 {
  let rot = body.rotation().to_mat3()
  let inv_local = body.mass_properties().inv_inertia.into_mat3()
  rot.mul(inv_local).mul(rot.transpose())
}

///|
fn apply_impulse(
  body : @dynamics.RigidBody3D,
  impulse : @core.Vec3,
  point : @core.Vec3,
) -> Unit {
  let mprops = body.mass_properties()
  if mprops.inv_mass <= 0.0F {
    return
  }
  // Any non-trivial impulse should wake up a sleeping dynamic body.
  if !body.is_active() && impulse.length_squared() > 1.0e-12F {
    body.wake_up()
  }
  body.set_linvel(body.linvel().add(impulse.scale(mprops.inv_mass)))
  let r = point.sub(body.world_com())
  let ang_imp = r.cross(impulse)
  let dw = world_inv_inertia(body).mul_vec3(ang_imp)
  body.set_angvel(body.angvel().add(dw))
}

///|
fn apply_angular_impulse(
  body : @dynamics.RigidBody3D,
  ang_impulse : @core.Vec3,
) -> Unit {
  let mprops = body.mass_properties()
  if mprops.inv_mass <= 0.0F {
    return
  }
  if !body.is_active() && ang_impulse.length_squared() > 1.0e-12F {
    body.wake_up()
  }
  let dw = world_inv_inertia(body).mul_vec3(ang_impulse)
  body.set_angvel(body.angvel().add(dw))
}

///|
fn apply_translation_correction(
  body : @dynamics.RigidBody3D,
  correction : @core.Vec3,
) -> Unit {
  let curr = body.translation()
  let locked_axes = body.locked_axes()
  let next = @core.Vec3::new(
    if locked_axes.contains(@dynamics.LockedAxes::translation_locked_x()) {
      curr.x
    } else {
      curr.x + correction.x
    },
    if locked_axes.contains(@dynamics.LockedAxes::translation_locked_y()) {
      curr.y
    } else {
      curr.y + correction.y
    },
    if locked_axes.contains(@dynamics.LockedAxes::translation_locked_z()) {
      curr.z
    } else {
      curr.z + correction.z
    },
  )
  body.set_translation(next)
}

///|
fn midpoint3(a : @core.Vec3, b : @core.Vec3) -> @core.Vec3 {
  a.add(b).scale(0.5F)
}

///|
fn tangent_contact_directions(
  n : @core.Vec3,
  linvel1 : @core.Vec3,
  linvel2 : @core.Vec3,
) -> (@core.Vec3, @core.Vec3) {
  let tangent_vel = linvel2
    .sub(linvel1)
    .sub(n.scale(linvel2.sub(linvel1).dot(n)))
  if tangent_vel.length_squared() > 1.0e-12F {
    let t1 = tangent_vel.normalize()
    let t2 = n.cross(t1)
    (t1, t2)
  } else {
    orthonormal_basis(n)
  }
}

///|
fn solve_contacts_3d_impl(
  parameters : @dynamics.IntegrationParameters,
  bodies : @dynamics.RigidBodySet3D,
  colliders : @collision.ColliderSet3D,
  narrow_phase : @collision.NarrowPhase3D,
  cache_in : ContactSolverCache3D,
  cache_out : ContactSolverCache3D,
  twist_cache_in : @hashmap.HashMap[(Int, Int, Int, Int), @core.Real],
  twist_cache_out : @hashmap.HashMap[(Int, Int, Int, Int), @core.Real],
  physics_hooks : PhysicsHooks3D?,
  event_handler : EventHandler3D?,
  island_stamps : Array[Int]?,
  island_stamp : Int,
  reset_outputs : Bool,
) -> Unit {
  if reset_outputs {
    cache_out.clear()
    twist_cache_out.clear()
  }
  fn hook_normal_or_fallback(
    hook_normal : @core.Vec3?,
    fallback : @core.Vec3,
  ) -> @core.Vec3 {
    if hook_normal is Some(n) {
      if n.length_squared() > 1.0e-12F {
        n.normalize()
      } else {
        fallback
      }
    } else {
      fallback
    }
  }

  fn resolve_modify_hook(
    physics_hooks : PhysicsHooks3D?,
    bodies : @dynamics.RigidBodySet3D,
    colliders : @collision.ColliderSet3D,
    collider1 : @collision.ColliderHandle3D,
    collider2 : @collision.ColliderHandle3D,
    fallback_normal : @core.Vec3,
  ) -> (Bool, @core.Vec3?) {
    if physics_hooks is Some(hooks) {
      if colliders.get(collider1) is Some(c1) &&
        colliders.get(collider2) is Some(c2) {
        let hook = @collision.ActiveHooks::modify_solver_contacts()
        if c1.active_hooks().contains(hook) || c2.active_hooks().contains(hook) {
          hooks.modify_solver_contacts_internal(
            bodies, colliders, collider1, collider2, fallback_normal,
          )
        } else {
          (true, None)
        }
      } else {
        (true, None)
      }
    } else {
      (true, None)
    }
  }

  fn accept_one_way(
    platform : @collision.Collider3D,
    other : @collision.Collider3D,
  ) -> Bool {
    if platform.one_way() is Some((offset, dir)) {
      let dy = other.position().translation.y -
        platform.position().translation.y
      if dir > 0 {
        dy > offset
      } else {
        dy < -offset
      }
    } else {
      true
    }
  }

  // Warmstart.
  let warmstart_coeff = parameters.warmstart_coefficient
  let all_pairs = narrow_phase.all_contact_pairs()
  for i in 0..<all_pairs.length() {
    let entry = all_pairs[i]
    let pair = entry.0
    let cp = entry.1
    if cp.manifolds_len() == 0 {
      continue
    }
    let a = pair.0
    let b = pair.1
    if colliders.get(a) is Some(co1) && colliders.get(b) is Some(co2) {
      let p1 = co1.parent()
      let p2 = co2.parent()
      if !contact_pair_matches_island(island_stamps, island_stamp, p1, p2) {
        continue
      }
      if co1.is_sensor() || co2.is_sensor() {
        continue
      }
      if !accept_one_way(co1, co2) || !accept_one_way(co2, co1) {
        continue
      }
      // Skip warmstarting contacts between sleeping dynamic bodies.
      let mut dyn_sleeping1 = false
      let mut dyn_sleeping2 = false
      let mut fixed_or_none1 = p1 is None
      let mut fixed_or_none2 = p2 is None
      if p1 is Some(h1) {
        if bodies.get(h1) is Some(rb1) && rb1.body_type().is_dynamic() {
          dyn_sleeping1 = !rb1.is_active()
          fixed_or_none1 = false
        } else if bodies.get(h1) is Some(rb1) && rb1.body_type().is_fixed() {
          fixed_or_none1 = true
        }
      }
      if p2 is Some(h2) {
        if bodies.get(h2) is Some(rb2) && rb2.body_type().is_dynamic() {
          dyn_sleeping2 = !rb2.is_active()
          fixed_or_none2 = false
        } else if bodies.get(h2) is Some(rb2) && rb2.body_type().is_fixed() {
          fixed_or_none2 = true
        }
      }
      let surface_vel = co2.surface_velocity().sub(co1.surface_velocity())
      let stationary = surface_vel.length_squared() <= 1.0e-12F
      // A sleeping dynamic resting on a fixed/static collider doesn't need warmstarting.
      if stationary &&
        ((dyn_sleeping1 && fixed_or_none2) || (dyn_sleeping2 && fixed_or_none1)) {
        continue
      }
      let prev = if cache_get(cache_in, a, b) is Some(list) { list } else { [] }
      fn match_impulse(
        prev : Array[(@core.Vec3, @core.Vec3, ContactImpulse3D)],
        p1 : @core.Vec3,
        p2 : @core.Vec3,
      ) -> ContactImpulse3D {
        if prev.length() == 0 {
          return ContactImpulse3D::zero()
        }
        let mut best = ContactImpulse3D::zero()
        let mut best_d2 = 1.0e30F
        for i in 0..<prev.length() {
          let e = prev[i]
          let d2 = e.0.sub(p1).length_squared() + e.1.sub(p2).length_squared()
          if d2 < best_d2 {
            best_d2 = d2
            best = e.2
          }
        }
        // Contacts drift slightly between frames; tolerate small motion.
        if best_d2 <= 1.0e-4F {
          best
        } else {
          ContactImpulse3D::zero()
        }
      }

      let contacts = cp.manifolds()
      if contacts.length() == 0 {
        continue
      }
      let friction_model = parameters.friction_model
      let (accept, hook_normal) = resolve_modify_hook(
        physics_hooks,
        bodies,
        colliders,
        a,
        b,
        contacts[0].normal,
      )
      if !accept {
        continue
      }
      let contact_normal = hook_normal_or_fallback(
        hook_normal,
        contacts[0].normal,
      )
      let current : Array[(@core.Vec3, @core.Vec3, ContactImpulse3D)] = []
      for k in 0..<contacts.length() {
        let c = contacts[k]
        let imp = match_impulse(prev, c.point1, c.point2)
        current.push((c.point1, c.point2, imp))
        if !(dyn_sleeping1 && dyn_sleeping2) {
          let normal = contact_normal
          let ws_normal = normal.scale(imp.normal * warmstart_coeff)
          if p1 is Some(h1) {
            if bodies.get(h1) is Some(rb1) && rb1.body_type().is_dynamic() {
              apply_impulse(rb1, ws_normal.scale(-1.0F), c.point1)
            }
          }
          if p2 is Some(h2) {
            if bodies.get(h2) is Some(rb2) && rb2.body_type().is_dynamic() {
              apply_impulse(rb2, ws_normal, c.point2)
            }
          }
          if friction_model is @dynamics.FrictionModel::Coulomb {
            let ws = imp.scaled(warmstart_coeff)
            let (t1, t2) = orthonormal_basis(normal)
            let tangent_total = t1.scale(ws.tangent1).add(t2.scale(ws.tangent2))
            if p1 is Some(h1) {
              if bodies.get(h1) is Some(rb1) && rb1.body_type().is_dynamic() {
                apply_impulse(rb1, tangent_total.scale(-1.0F), c.point1)
              }
            }
            if p2 is Some(h2) {
              if bodies.get(h2) is Some(rb2) && rb2.body_type().is_dynamic() {
                apply_impulse(rb2, tangent_total, c.point2)
              }
            }
          }
        }
      }
      // Initialize the per-contact impulse accumulators for this solve step.
      cache_set(cache_out, a, b, current)
    }
  }

  // Solve.
  let dt = parameters.dt
  let extra_pgs = if parameters.num_internal_pgs_iterations < 0 {
    0
  } else {
    parameters.num_internal_pgs_iterations
  }
  let iters = parameters.num_solver_iterations * (extra_pgs + 1)
  let allowed_error = parameters.allowed_linear_error()
  let max_corr = parameters.max_corrective_velocity()
  let erp_inv_dt = parameters.contact_softness.erp_inv_dt(dt)
  let cfm_factor = parameters.contact_softness.cfm_factor(dt)
  fn vel_at(body : @dynamics.RigidBody3D, point : @core.Vec3) -> @core.Vec3 {
    let r = point.sub(body.world_com())
    body.linvel().add(body.angvel().cross(r))
  }

  fn k_term(
    inv_i : @core.Mat3,
    r : @core.Vec3,
    dir : @core.Vec3,
  ) -> @core.Real {
    let c = r.cross(dir)
    let v = inv_i.mul_vec3(c)
    dir.dot(v.cross(r))
  }

  // Small impacts don't bounce (matches Rapier's restitution velocity threshold).
  let restitution_threshold = 1.0F
  let solve_entries : Array[ContactSolveEntry3D] = []
  let pair_friction_impulses : Array[(@core.Real, @core.Real, @core.Real)] = []
  let pair_keys : Array[(Int, Int, Int, Int)] = []
  for i in 0..<all_pairs.length() {
    let entry = all_pairs[i]
    let pair = entry.0
    let cp = entry.1
    if cp.manifolds_len() == 0 {
      continue
    }
    let a = pair.0
    let b = pair.1
    if colliders.get(a) is Some(co1) && colliders.get(b) is Some(co2) {
      let p1 = co1.parent()
      let p2 = co2.parent()
      if !contact_pair_matches_island(island_stamps, island_stamp, p1, p2) {
        continue
      }
      if co1.is_sensor() || co2.is_sensor() {
        continue
      }
      if !accept_one_way(co1, co2) || !accept_one_way(co2, co1) {
        continue
      }
      let contacts = cp.manifolds()
      if contacts.length() == 0 {
        continue
      }
      let (accept, hook_normal) = resolve_modify_hook(
        physics_hooks,
        bodies,
        colliders,
        a,
        b,
        contacts[0].normal,
      )
      if !accept {
        continue
      }
      let shared_normal = hook_normal_or_fallback(
        hook_normal,
        contacts[0].normal,
      )
      let mut rb1 : @dynamics.RigidBody3D? = None
      let mut rb2 : @dynamics.RigidBody3D? = None
      if p1 is Some(h1) {
        rb1 = bodies.get(h1)
      }
      if p2 is Some(h2) {
        rb2 = bodies.get(h2)
      }
      // Skip contacts between sleeping dynamic bodies.
      if rb1 is Some(b1) &&
        rb2 is Some(b2) &&
        b1.body_type().is_dynamic() &&
        b2.body_type().is_dynamic() &&
        !b1.is_active() &&
        !b2.is_active() {
        continue
      }
      let surface_vel = co2.surface_velocity().sub(co1.surface_velocity())
      let stationary = surface_vel.length_squared() <= 1.0e-12F
      // Skip solver work for a sleeping dynamic resting on a fixed/static collider when the
      // surface is stationary.
      if stationary {
        if rb1 is Some(b1) &&
          b1.body_type().is_dynamic() &&
          !b1.is_active() &&
          (rb2 is None || (rb2 is Some(b2) && b2.body_type().is_fixed())) {
          continue
        }
        if rb2 is Some(b2) &&
          b2.body_type().is_dynamic() &&
          !b2.is_active() &&
          (rb1 is None || (rb1 is Some(b1) && b1.body_type().is_fixed())) {
          continue
        }
      }
      let friction = @dynamics.CoefficientCombineRule::combine(
        co1.friction(),
        co2.friction(),
        co1.friction_combine_rule(),
        co2.friction_combine_rule(),
      )
      let inv_m1 = if rb1 is Some(b1) && b1.body_type().is_dynamic() {
        b1.mass_properties().inv_mass
      } else {
        0.0F
      }
      let inv_m2 = if rb2 is Some(b2) && b2.body_type().is_dynamic() {
        b2.mass_properties().inv_mass
      } else {
        0.0F
      }
      let i1 = if rb1 is Some(b1) && b1.body_type().is_dynamic() {
        world_inv_inertia(b1)
      } else {
        @core.Mat3::zero()
      }
      let i2 = if rb2 is Some(b2) && b2.body_type().is_dynamic() {
        world_inv_inertia(b2)
      } else {
        @core.Mat3::zero()
      }
      let cached = if cache_get(cache_out, a, b) is Some(existing) {
        existing
      } else {
        continue
      }
      if cached.length() != contacts.length() {
        cached.clear()
        for k in 0..<contacts.length() {
          let c = contacts[k]
          cached.push((c.point1, c.point2, ContactImpulse3D::zero()))
        }
        cache_set(cache_out, a, b, cached)
      }
      let restitution = @dynamics.CoefficientCombineRule::combine(
        co1.restitution(),
        co2.restitution(),
        co1.restitution_combine_rule(),
        co2.restitution_combine_rule(),
      )
      let friction_model = parameters.friction_model
      let points : Array[ContactSolvePoint3D] = []
      for k in 0..<contacts.length() {
        let c0 = contacts[k]
        let n = shared_normal
        let (t1, t2) = orthonormal_basis(n)
        let r1 = if rb1 is Some(b1) {
          c0.point1.sub(b1.world_com())
        } else {
          @core.Vec3::zero()
        }
        let r2 = if rb2 is Some(b2) {
          c0.point2.sub(b2.world_com())
        } else {
          @core.Vec3::zero()
        }
        let kn = inv_m1 + inv_m2 + k_term(i1, r1, n) + k_term(i2, r2, n)
        let kt1 = inv_m1 + inv_m2 + k_term(i1, r1, t1) + k_term(i2, r2, t1)
        let kt2 = inv_m1 + inv_m2 + k_term(i1, r1, t2) + k_term(i2, r2, t2)
        let v1 = if rb1 is Some(b1) {
          vel_at(b1, c0.point1)
        } else {
          @core.Vec3::zero()
        }
        let v2 = if rb2 is Some(b2) {
          vel_at(b2, c0.point2)
        } else {
          @core.Vec3::zero()
        }
        let vn0 = v2.sub(v1).dot(n)
        let target_vn = if restitution > 0.0F && vn0 <= -restitution_threshold {
          -restitution * vn0
        } else {
          0.0F
        }
        let bias = if dt <= 0.0F {
          0.0F
        } else {
          let raw = erp_inv_dt * (allowed_error - c0.penetration)
          if raw < -max_corr {
            -max_corr
          } else if raw > 0.0F {
            0.0F
          } else {
            raw
          }
        }
        points.push({
          point1: c0.point1,
          point2: c0.point2,
          normal: n,
          t1,
          t2,
          kn,
          kt1,
          kt2,
          target_vn,
          bias,
          penetration: c0.penetration,
        })
      }
      let friction_center = if points.length() == 0 {
        @core.Vec3::zero()
      } else {
        let mut sum = @core.Vec3::zero()
        for k in 0..<points.length() {
          let p = points[k]
          sum = sum.add(midpoint3(p.point1, p.point2))
        }
        sum.scale(1.0F / Float::from_double(points.length().to_double()))
      }
      let linvel1 = if rb1 is Some(b1) {
        b1.linvel()
      } else {
        @core.Vec3::zero()
      }
      let linvel2 = if rb2 is Some(b2) {
        b2.linvel()
      } else {
        @core.Vec3::zero()
      }
      let friction_normal = if points.length() > 0 {
        points[0].normal
      } else {
        @core.Vec3::new(0.0F, 1.0F, 0.0F)
      }
      let (friction_t1, friction_t2) = tangent_contact_directions(
        friction_normal,
        linvel1,
        linvel2.add(surface_vel),
      )
      let dp1 = if rb1 is Some(b1) {
        friction_center.sub(b1.world_com())
      } else {
        @core.Vec3::zero()
      }
      let dp2 = if rb2 is Some(b2) {
        friction_center.sub(b2.world_com())
      } else {
        @core.Vec3::zero()
      }
      let friction_r0 = inv_m1 +
        inv_m2 +
        k_term(i1, dp1, friction_t1) +
        k_term(i2, dp2, friction_t1)
      let friction_r1 = inv_m1 +
        inv_m2 +
        k_term(i1, dp1, friction_t2) +
        k_term(i2, dp2, friction_t2)
      let c11 = dp1.cross(friction_t1)
      let c12 = dp1.cross(friction_t2)
      let c21 = dp2.cross(friction_t1)
      let c22 = dp2.cross(friction_t2)
      let friction_r01 = i1.mul_vec3(c11).dot(c12) + i2.mul_vec3(c21).dot(c22)
      let twist_r = if points.length() > 1 {
        friction_normal.dot(i1.mul_vec3(friction_normal)) +
        friction_normal.dot(i2.mul_vec3(friction_normal))
      } else {
        0.0F
      }
      let twist_dists : Array[@core.Real] = []
      for k in 0..<points.length() {
        let p = points[k]
        twist_dists.push(
          midpoint3(p.point1, p.point2).sub(friction_center).length(),
        )
      }
      let mut warm_t1 = 0.0F
      let mut warm_t2 = 0.0F
      let warm_twist = if twist_cache_in.get(cache_key(a, b)) is Some(value) {
        value
      } else {
        0.0F
      }
      if cached.length() > 0 {
        for k in 0..<cached.length() {
          warm_t1 = warm_t1 + cached[k].2.tangent1
          warm_t2 = warm_t2 + cached[k].2.tangent2
        }
        let inv_cached_len = 1.0F /
          Float::from_double(cached.length().to_double())
        warm_t1 = warm_t1 * inv_cached_len
        warm_t2 = warm_t2 * inv_cached_len
      }
      pair_friction_impulses.push((warm_t1, warm_t2, warm_twist))
      pair_keys.push(cache_key(a, b))
      solve_entries.push({
        rb1,
        rb2,
        points,
        cached,
        friction,
        friction_model,
        friction_normal,
        friction_center,
        friction_t1,
        friction_t2,
        friction_r0,
        friction_r1,
        friction_r01,
        twist_r,
        twist_dists,
        surface_vel,
      })
    }
  }
  for i in 0..<solve_entries.length() {
    let e = solve_entries[i]
    if e.friction_model is @dynamics.FrictionModel::Simplified {
      let (warm_t1, warm_t2, warm_twist) = pair_friction_impulses[i]
      let warm_tangent = e.friction_t1
        .scale(warm_t1 * warmstart_coeff)
        .add(e.friction_t2.scale(warm_t2 * warmstart_coeff))
      if warm_tangent.length_squared() > 1.0e-12F {
        if e.rb1 is Some(b1) && b1.body_type().is_dynamic() {
          apply_impulse(b1, warm_tangent.scale(-1.0F), e.friction_center)
        }
        if e.rb2 is Some(b2) && b2.body_type().is_dynamic() {
          apply_impulse(b2, warm_tangent, e.friction_center)
        }
      }
      if warm_twist != 0.0F {
        let twist_impulse = e.friction_normal.scale(
          warm_twist * warmstart_coeff,
        )
        if e.rb1 is Some(b1) && b1.body_type().is_dynamic() {
          apply_angular_impulse(b1, twist_impulse.scale(-1.0F))
        }
        if e.rb2 is Some(b2) && b2.body_type().is_dynamic() {
          apply_angular_impulse(b2, twist_impulse)
        }
      }
    }
  }
  for _ in 0..<iters {
    for i in 0..<solve_entries.length() {
      let e = solve_entries[i]
      let points = e.points
      let cached = e.cached
      let friction = e.friction
      let friction_model = e.friction_model
      let surface_vel = e.surface_vel
      let rb1 = e.rb1
      let rb2 = e.rb2
      for k in 0..<points.length() {
        let p0 = points[k]
        let n = p0.normal
        let t1 = p0.t1
        let t2 = p0.t2
        let target_vn = p0.target_vn
        let bias = p0.bias
        let kn = p0.kn
        let kt1 = p0.kt1
        let kt2 = p0.kt2
        let mut acc = cached[k].2
        let v1 = if rb1 is Some(b1) {
          vel_at(b1, p0.point1)
        } else {
          @core.Vec3::zero()
        }
        let v2 = if rb2 is Some(b2) {
          vel_at(b2, p0.point2)
        } else {
          @core.Vec3::zero()
        }
        let rv = v2.sub(v1)
        let vn = rv.dot(n)
        if kn > 0.0F {
          let prev_n = acc.normal
          let cand_n = prev_n - (vn - target_vn + bias) / kn
          let next_n = if cand_n < 0.0F { 0.0F } else { cfm_factor * cand_n }
          let delta_n = next_n - prev_n
          if delta_n != 0.0F {
            let p = n.scale(delta_n)
            if rb1 is Some(b1) && b1.body_type().is_dynamic() {
              apply_impulse(b1, p.scale(-1.0F), p0.point1)
            }
            if rb2 is Some(b2) && b2.body_type().is_dynamic() {
              apply_impulse(b2, p, p0.point2)
            }
          }
          acc = {
            normal: next_n,
            tangent1: acc.tangent1,
            tangent2: acc.tangent2,
          }
        }
        if friction_model is @dynamics.FrictionModel::Coulomb &&
          friction > 0.0F &&
          acc.normal > 0.0F {
          // Recompute after normal impulse.
          let v1b = if rb1 is Some(b1) {
            vel_at(b1, p0.point1)
          } else {
            @core.Vec3::zero()
          }
          let v2b = if rb2 is Some(b2) {
            vel_at(b2, p0.point2)
          } else {
            @core.Vec3::zero()
          }
          let rvb = v2b.sub(v1b).add(surface_vel)
          let vt1 = rvb.dot(t1)
          let vt2 = rvb.dot(t2)
          if kt1 > 0.0F && kt2 > 0.0F {
            let prev_t1 = acc.tangent1
            let prev_t2 = acc.tangent2
            let cand_t1 = prev_t1 + -vt1 / kt1
            let cand_t2 = prev_t2 + -vt2 / kt2
            let max_t = friction * acc.normal
            let mag2 = cand_t1 * cand_t1 + cand_t2 * cand_t2
            let max2 = max_t * max_t
            let (next_t1, next_t2) = if mag2 > max2 && mag2 > 1.0e-12F {
              let kk = max_t / local_sqrt(mag2)
              (cand_t1 * kk, cand_t2 * kk)
            } else {
              (cand_t1, cand_t2)
            }
            let delta_t1 = next_t1 - prev_t1
            let delta_t2 = next_t2 - prev_t2
            if delta_t1 != 0.0F || delta_t2 != 0.0F {
              let p = t1.scale(delta_t1).add(t2.scale(delta_t2))
              if rb1 is Some(b1) && b1.body_type().is_dynamic() {
                apply_impulse(b1, p.scale(-1.0F), p0.point1)
              }
              if rb2 is Some(b2) && b2.body_type().is_dynamic() {
                apply_impulse(b2, p, p0.point2)
              }
            }
            acc = { normal: acc.normal, tangent1: next_t1, tangent2: next_t2 }
          }
        }
        cached[k] = (p0.point1, p0.point2, acc)
      }
      if friction_model is @dynamics.FrictionModel::Simplified &&
        friction > 0.0F {
        let mut tangent_limit = 0.0F
        let mut twist_limit = 0.0F
        for k in 0..<points.length() {
          let normal_impulse = cached[k].2.normal
          tangent_limit = tangent_limit + normal_impulse
          if k < e.twist_dists.length() {
            twist_limit = twist_limit + normal_impulse * e.twist_dists[k]
          }
        }
        tangent_limit = tangent_limit * friction
        twist_limit = twist_limit * friction
        let (prev_t1, prev_t2, prev_twist) = pair_friction_impulses[i]
        let mut next_t1 = prev_t1
        let mut next_t2 = prev_t2
        if tangent_limit > 0.0F &&
          (e.friction_r0 > 0.0F || e.friction_r1 > 0.0F) {
          let v1f = if rb1 is Some(b1) {
            vel_at(b1, e.friction_center)
          } else {
            @core.Vec3::zero()
          }
          let v2f = if rb2 is Some(b2) {
            vel_at(b2, e.friction_center)
          } else {
            @core.Vec3::zero()
          }
          let rvf = v2f.sub(v1f).add(surface_vel)
          let dvel0 = rvf.dot(e.friction_t1)
          let dvel1 = rvf.dot(e.friction_t2)
          let dvel00 = dvel0 * dvel0
          let dvel11 = dvel1 * dvel1
          let dvel01 = dvel0 * dvel1
          let lhs = dvel00 * e.friction_r0 +
            dvel11 * e.friction_r1 +
            dvel01 * (2.0F * e.friction_r01)
          let inv_lhs = if lhs > 1.0e-12F {
            (dvel00 + dvel11) / lhs
          } else {
            0.0F
          }
          let cand_t1 = prev_t1 - inv_lhs * dvel0
          let cand_t2 = prev_t2 - inv_lhs * dvel1
          let cand_mag2 = cand_t1 * cand_t1 + cand_t2 * cand_t2
          let max_mag2 = tangent_limit * tangent_limit
          if cand_mag2 > max_mag2 && cand_mag2 > 1.0e-12F {
            let scale = tangent_limit / local_sqrt(cand_mag2)
            next_t1 = cand_t1 * scale
            next_t2 = cand_t2 * scale
          } else {
            next_t1 = cand_t1
            next_t2 = cand_t2
          }
          let delta_t = e.friction_t1
            .scale(next_t1 - prev_t1)
            .add(e.friction_t2.scale(next_t2 - prev_t2))
          if delta_t.length_squared() > 1.0e-12F {
            if rb1 is Some(b1) && b1.body_type().is_dynamic() {
              apply_impulse(b1, delta_t.scale(-1.0F), e.friction_center)
            }
            if rb2 is Some(b2) && b2.body_type().is_dynamic() {
              apply_impulse(b2, delta_t, e.friction_center)
            }
          }
        }
        let mut next_twist = prev_twist
        if twist_limit > 0.0F && e.twist_r > 1.0e-12F {
          let w1 = if rb1 is Some(b1) {
            b1.angvel()
          } else {
            @core.Vec3::zero()
          }
          let w2 = if rb2 is Some(b2) {
            b2.angvel()
          } else {
            @core.Vec3::zero()
          }
          let rel_twist = w2.sub(w1).dot(e.friction_normal)
          let cand_twist = prev_twist - rel_twist / e.twist_r
          if cand_twist < -twist_limit {
            next_twist = -twist_limit
          } else if cand_twist > twist_limit {
            next_twist = twist_limit
          } else {
            next_twist = cand_twist
          }
          let delta_twist = next_twist - prev_twist
          if delta_twist != 0.0F {
            let twist_impulse = e.friction_normal.scale(delta_twist)
            if rb1 is Some(b1) && b1.body_type().is_dynamic() {
              apply_angular_impulse(b1, twist_impulse.scale(-1.0F))
            }
            if rb2 is Some(b2) && b2.body_type().is_dynamic() {
              apply_angular_impulse(b2, twist_impulse)
            }
          }
        }
        pair_friction_impulses[i] = (next_t1, next_t2, next_twist)
        for k in 0..<points.length() {
          let prev_cached = cached[k].2
          cached[k] = (
            points[k].point1,
            points[k].point2,
            { normal: prev_cached.normal, tangent1: next_t1, tangent2: next_t2 },
          )
        }
      }
    }
  }
  for i in 0..<solve_entries.length() {
    let e = solve_entries[i]
    if e.friction_model is @dynamics.FrictionModel::Simplified {
      twist_cache_out.set(pair_keys[i], pair_friction_impulses[i].2)
    }
  }

  // Positional stabilization for resting contacts.
  let pos_slop = 1.0e-3F
  let pos_beta = 0.35F
  for i in 0..<solve_entries.length() {
    let e = solve_entries[i]
    let rb1 = e.rb1
    let rb2 = e.rb2
    let mut inv_m1 = 0.0F
    let mut inv_m2 = 0.0F
    if rb1 is Some(b1) && b1.body_type().is_dynamic() {
      inv_m1 = b1.mass_properties().inv_mass
    }
    if rb2 is Some(b2) && b2.body_type().is_dynamic() {
      inv_m2 = b2.mass_properties().inv_mass
    }
    let denom = inv_m1 + inv_m2
    if denom <= 1.0e-12F {
      continue
    }
    let mut best_depth = 0.0F
    let mut best_normal = @core.Vec3::zero()
    for k in 0..<e.points.length() {
      let p = e.points[k]
      if p.penetration > best_depth {
        best_depth = p.penetration
        best_normal = p.normal
      }
    }
    if best_depth <= pos_slop {
      continue
    }
    let correction = best_normal.scale((best_depth - pos_slop) * pos_beta)
    let ratio1 = inv_m1 / denom
    let ratio2 = inv_m2 / denom
    if rb1 is Some(b1) && b1.body_type().is_dynamic() {
      apply_translation_correction(b1, correction.scale(-ratio1))
    }
    if rb2 is Some(b2) && b2.body_type().is_dynamic() {
      apply_translation_correction(b2, correction.scale(ratio2))
    }
  }

  // Write back the normal impulses for pub-surface parity, and emit contact-force events.
  for i in 0..<all_pairs.length() {
    let entry = all_pairs[i]
    let pair = entry.0
    let cp = entry.1
    if cp.manifolds_len() == 0 {
      continue
    }
    let a = pair.0
    let b = pair.1
    if colliders.get(a) is Some(co1) && colliders.get(b) is Some(co2) {
      let p1 = co1.parent()
      let p2 = co2.parent()
      if !contact_pair_matches_island(island_stamps, island_stamp, p1, p2) {
        continue
      }
      if co1.is_sensor() || co2.is_sensor() {
        continue
      }
      let cached = if cache_get(cache_out, a, b) is Some(list) {
        list
      } else {
        []
      }
      let contacts = cp.manifolds()
      if contacts.length() == 0 {
        continue
      }
      let hook_result = resolve_modify_hook(
        physics_hooks,
        bodies,
        colliders,
        a,
        b,
        contacts[0].normal,
      )
      let hook_normal = hook_result.1
      let n = contacts.length()
      let m = cached.length()
      let k = if n < m { n } else { m }
      let normal_impulses : Array[@core.Real] = []
      for _ in 0..<n {
        normal_impulses.push(0.0F)
      }
      let mut total_impulse = 0.0F
      let mut total_vec = @core.Vec3::zero()
      let mut best_impulse = 0.0F
      let mut best_dir = @core.Vec3::zero()
      for j in 0..<k {
        let imp = cached[j].2.normal
        normal_impulses[j] = imp
        total_impulse = total_impulse + imp
        let nrm = hook_normal_or_fallback(hook_normal, contacts[j].normal)
        total_vec = total_vec.add(nrm.scale(imp))
        if imp > best_impulse {
          best_impulse = imp
          best_dir = nrm
        }
      }
      narrow_phase.set_contact_pair_normal_impulses(a, b, normal_impulses)
      if event_handler is Some(handler) {
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
        if threshold < max_threshold && dt > 0.0F {
          let inv_dt = 1.0F / dt
          let total_force_magnitude = total_impulse * inv_dt
          if total_force_magnitude > threshold {
            handler.push_contact_force_event(
              @collision.ContactForceEvent3D::new(
                a,
                b,
                total_vec.scale(inv_dt),
                total_force_magnitude,
                best_dir,
                best_impulse * inv_dt,
              ),
            )
          }
        }
      }
    }
  }
}

///|
pub fn solve_contacts_3d(
  parameters : @dynamics.IntegrationParameters,
  bodies : @dynamics.RigidBodySet3D,
  colliders : @collision.ColliderSet3D,
  narrow_phase : @collision.NarrowPhase3D,
  cache_in : ContactSolverCache3D,
  cache_out : ContactSolverCache3D,
  physics_hooks : PhysicsHooks3D?,
  event_handler : EventHandler3D?,
) -> Unit {
  let twist_cache_in : @hashmap.HashMap[(Int, Int, Int, Int), @core.Real] = @hashmap.new()
  let twist_cache_out : @hashmap.HashMap[(Int, Int, Int, Int), @core.Real] = @hashmap.new()
  solve_contacts_3d_impl(
    parameters,
    bodies,
    colliders,
    narrow_phase,
    cache_in,
    cache_out,
    twist_cache_in,
    twist_cache_out,
    physics_hooks,
    event_handler,
    None,
    0,
    true,
  )
}
