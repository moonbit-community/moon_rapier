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
priv struct ContactSolveEntry3D {
  rb1 : @dynamics.RigidBody3D?
  rb2 : @dynamics.RigidBody3D?
  points : Array[ContactSolvePoint3D]
  cached : Array[(@core.Vec3, @core.Vec3, ContactImpulse3D)]
  friction : @core.Real
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
  cache_out.clear()
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
      hooks.modify_solver_contacts_internal(
        bodies, colliders, collider1, collider2, fallback_normal,
      )
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
      if co1.is_sensor() || co2.is_sensor() {
        continue
      }
      if !accept_one_way(co1, co2) || !accept_one_way(co2, co1) {
        continue
      }
      let p1 = co1.parent()
      let p2 = co2.parent()
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
      let current : Array[(@core.Vec3, @core.Vec3, ContactImpulse3D)] = []
      for k in 0..<contacts.length() {
        let c = contacts[k]
        let imp = match_impulse(prev, c.point1, c.point2)
        current.push((c.point1, c.point2, imp))
        if !(dyn_sleeping1 && dyn_sleeping2) {
          let ws = imp.scaled(warmstart_coeff)
          let normal = hook_normal_or_fallback(hook_normal, c.normal)
          let (t1, t2) = orthonormal_basis(normal)
          let total = normal
            .scale(ws.normal)
            .add(t1.scale(ws.tangent1))
            .add(t2.scale(ws.tangent2))
          if p1 is Some(h1) {
            if bodies.get(h1) is Some(rb1) && rb1.body_type().is_dynamic() {
              apply_impulse(rb1, total.scale(-1.0F), c.point1)
            }
          }
          if p2 is Some(h2) {
            if bodies.get(h2) is Some(rb2) && rb2.body_type().is_dynamic() {
              apply_impulse(rb2, total, c.point2)
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
  let iters = parameters.num_solver_iterations
  let slop = 1.0e-3F
  let beta = 0.2F
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
      let mut rb1 : @dynamics.RigidBody3D? = None
      let mut rb2 : @dynamics.RigidBody3D? = None
      if co1.parent() is Some(h1) {
        rb1 = bodies.get(h1)
      }
      if co2.parent() is Some(h2) {
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
      let friction = 0.5F * (co1.friction() + co2.friction())
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
      let restitution = 0.5F * (co1.restitution() + co2.restitution())
      let max_corr = parameters.max_corrective_velocity()
      let points : Array[ContactSolvePoint3D] = []
      for k in 0..<contacts.length() {
        let c0 = contacts[k]
        let n = hook_normal_or_fallback(hook_normal, c0.normal)
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
          let pen = c0.penetration - slop
          if pen > 0.0F {
            let raw = -beta * pen / dt
            if raw < -max_corr {
              -max_corr
            } else {
              raw
            }
          } else {
            0.0F
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
        })
      }
      solve_entries.push({ rb1, rb2, points, cached, friction, surface_vel })
    }
  }
  for _ in 0..<iters {
    for i in 0..<solve_entries.length() {
      let e = solve_entries[i]
      let points = e.points
      let cached = e.cached
      let friction = e.friction
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
          let jn = -(vn - target_vn + bias) / kn
          let prev_n = acc.normal
          let next_n = if prev_n + jn < 0.0F { 0.0F } else { prev_n + jn }
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
        if friction > 0.0F && acc.normal > 0.0F {
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
      let (_, hook_normal) = resolve_modify_hook(
        physics_hooks,
        bodies,
        colliders,
        a,
        b,
        contacts[0].normal,
      )
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
