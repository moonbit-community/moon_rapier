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
  entries : Array[
    (
      (@collision.ColliderHandle3D, @collision.ColliderHandle3D),
      ContactImpulse3D,
    ),
  ]
}

///|
pub fn ContactSolverCache3D::new() -> ContactSolverCache3D {
  { entries: [] }
}

///|
pub fn ContactSolverCache3D::clear(self : ContactSolverCache3D) -> Unit {
  self.entries.clear()
}

///|
fn canonical_pair(
  a : @collision.ColliderHandle3D,
  b : @collision.ColliderHandle3D,
) -> (@collision.ColliderHandle3D, @collision.ColliderHandle3D) {
  let (ai, ag) = a.into_raw_parts()
  let (bi, bg) = b.into_raw_parts()
  if ai < bi || (ai == bi && ag <= bg) {
    (a, b)
  } else {
    (b, a)
  }
}

///|
fn cache_get(
  cache : ContactSolverCache3D,
  a : @collision.ColliderHandle3D,
  b : @collision.ColliderHandle3D,
) -> ContactImpulse3D? {
  let key = canonical_pair(a, b)
  for i in 0..<cache.entries.length() {
    let entry = cache.entries[i]
    if entry.0.0.equals(key.0) && entry.0.1.equals(key.1) {
      return Some(entry.1)
    }
  }
  None
}

///|
fn cache_set(
  cache : ContactSolverCache3D,
  a : @collision.ColliderHandle3D,
  b : @collision.ColliderHandle3D,
  impulse : ContactImpulse3D,
) -> Unit {
  let key = canonical_pair(a, b)
  for i in 0..<cache.entries.length() {
    let entry = cache.entries[i]
    if entry.0.0.equals(key.0) && entry.0.1.equals(key.1) {
      cache.entries[i] = (key, impulse)
      return
    }
  }
  cache.entries.push((key, impulse))
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
) -> Unit {
  cache_out.clear()

  // Warmstart.
  let warmstart_coeff = parameters.warmstart_coefficient
  for i in 0..<narrow_phase.all_contact_pairs().length() {
    let entry = narrow_phase.all_contact_pairs()[i]
    let pair = entry.0
    let cp = entry.1
    if cp.manifolds_len() == 0 {
      continue
    }
    let a = pair.0
    let b = pair.1
    if cache_get(cache_in, a, b) is Some(imp) {
      let contacts = cp.manifolds()
      let c0 = contacts[0]
      if colliders.get(a) is Some(co1) && colliders.get(b) is Some(co2) {
        if co1.is_sensor() || co2.is_sensor() {
          continue
        }
        let p1 = co1.parent()
        let p2 = co2.parent()
        // Skip warmstarting contacts between sleeping dynamic bodies.
        let mut dyn_sleeping1 = false
        let mut dyn_sleeping2 = false
        if p1 is Some(h1) {
          if bodies.get(h1) is Some(rb1) && rb1.body_type().is_dynamic() {
            dyn_sleeping1 = !rb1.is_active()
          }
        }
        if p2 is Some(h2) {
          if bodies.get(h2) is Some(rb2) && rb2.body_type().is_dynamic() {
            dyn_sleeping2 = !rb2.is_active()
          }
        }
        if dyn_sleeping1 && dyn_sleeping2 {
          continue
        }
        let ws = imp.scaled(warmstart_coeff)
        let (t1, t2) = orthonormal_basis(c0.normal)
        let total = c0.normal
          .scale(ws.normal)
          .add(t1.scale(ws.tangent1))
          .add(t2.scale(ws.tangent2))
        cache_set(cache_out, a, b, ws)
        if p1 is Some(h1) {
          if bodies.get(h1) is Some(rb1) && rb1.body_type().is_dynamic() {
            apply_impulse(rb1, total.scale(-1.0F), c0.point1)
          }
        }
        if p2 is Some(h2) {
          if bodies.get(h2) is Some(rb2) && rb2.body_type().is_dynamic() {
            apply_impulse(rb2, total, c0.point2)
          }
        }
      }
    }
  }

  // Precompute restitution target velocity along the normal (after warmstart).
  //
  // We keep this constant during the solver iterations; otherwise the solver will progressively
  // damp the bounce back to zero relative normal velocity.
  let restitution_targets : Array[
    ((@collision.ColliderHandle3D, @collision.ColliderHandle3D), @core.Real),
  ] = []
  fn restitution_target_get(
    targets : Array[
      ((@collision.ColliderHandle3D, @collision.ColliderHandle3D), @core.Real),
    ],
    a : @collision.ColliderHandle3D,
    b : @collision.ColliderHandle3D,
  ) -> @core.Real {
    let key = canonical_pair(a, b)
    for i in 0..<targets.length() {
      let entry = targets[i]
      if entry.0.0.equals(key.0) && entry.0.1.equals(key.1) {
        return entry.1
      }
    }
    0.0F
  }
  // Small impacts don't bounce (matches Rapier's restitution velocity threshold).
  let restitution_threshold = 1.0F
  for i in 0..<narrow_phase.all_contact_pairs().length() {
    let entry = narrow_phase.all_contact_pairs()[i]
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
      let c0 = cp.manifolds()[0]

      // Gather bodies.
      let mut rb1 : @dynamics.RigidBody3D? = None
      let mut rb2 : @dynamics.RigidBody3D? = None
      if co1.parent() is Some(h1) {
        rb1 = bodies.get(h1)
      }
      if co2.parent() is Some(h2) {
        rb2 = bodies.get(h2)
      }
      if rb1 is Some(b1) &&
        rb2 is Some(b2) &&
        b1.body_type().is_dynamic() &&
        b2.body_type().is_dynamic() &&
        !b1.is_active() &&
        !b2.is_active() {
        continue
      }
      fn vel_at(
        body : @dynamics.RigidBody3D,
        point : @core.Vec3,
      ) -> @core.Vec3 {
        let r = point.sub(body.world_com())
        body.linvel().add(body.angvel().cross(r))
      }

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
      let vn0 = v2.sub(v1).dot(c0.normal)
      let restitution = 0.5F * (co1.restitution() + co2.restitution())
      let target_vn = if restitution > 0.0F && vn0 <= -restitution_threshold {
        -restitution * vn0
      } else {
        0.0F
      }
      restitution_targets.push((canonical_pair(a, b), target_vn))
    }
  }

  // Solve.
  let dt = parameters.dt
  let iters = parameters.num_solver_iterations
  let slop = 1.0e-3F
  let beta = 0.2F
  for _ in 0..<iters {
    let all = narrow_phase.all_contact_pairs()
    for i in 0..<all.length() {
      let entry = all[i]
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
        let c0 = cp.manifolds()[0]
        let n = c0.normal
        let (t1, t2) = orthonormal_basis(n)

        // Gather bodies.
        let mut rb1 : @dynamics.RigidBody3D? = None
        let mut rb2 : @dynamics.RigidBody3D? = None
        if co1.parent() is Some(h1) {
          rb1 = bodies.get(h1)
        }
        if co2.parent() is Some(h2) {
          rb2 = bodies.get(h2)
        }
        // Don't waste solver work on contacts between sleeping dynamic bodies.
        if rb1 is Some(b1) &&
          rb2 is Some(b2) &&
          b1.body_type().is_dynamic() &&
          b2.body_type().is_dynamic() &&
          !b1.is_active() &&
          !b2.is_active() {
          continue
        }
        let mut acc = if cache_get(cache_out, a, b) is Some(existing) {
          existing
        } else {
          ContactImpulse3D::zero()
        }
        let friction = 0.5F * (co1.friction() + co2.friction())
        let target_vn = restitution_target_get(restitution_targets, a, b)
        fn vel_at(
          body : @dynamics.RigidBody3D,
          point : @core.Vec3,
        ) -> @core.Vec3 {
          let r = point.sub(body.world_com())
          body.linvel().add(body.angvel().cross(r))
        }

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
        let rv = v2.sub(v1)
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
        fn k_term(
          inv_i : @core.Mat3,
          r : @core.Vec3,
          dir : @core.Vec3,
        ) -> @core.Real {
          let c = r.cross(dir)
          let v = inv_i.mul_vec3(c)
          dir.dot(v.cross(r))
        }

        let vn = rv.dot(n)
        let bias = if dt <= 0.0F {
          0.0F
        } else {
          let pen = c0.penetration - slop
          if pen > 0.0F {
            -beta * pen / dt
          } else {
            0.0F
          }
        }
        let kn = inv_m1 + inv_m2 + k_term(i1, r1, n) + k_term(i2, r2, n)
        if kn > 0.0F {
          let jn = -(vn - target_vn + bias) / kn
          let prev = acc.normal
          let next = if prev + jn < 0.0F { 0.0F } else { prev + jn }
          let delta = next - prev
          if delta != 0.0F {
            let p = n.scale(delta)
            if rb1 is Some(b1) && b1.body_type().is_dynamic() {
              apply_impulse(b1, p.scale(-1.0F), c0.point1)
            }
            if rb2 is Some(b2) && b2.body_type().is_dynamic() {
              apply_impulse(b2, p, c0.point2)
            }
          }
          acc = { normal: next, tangent1: acc.tangent1, tangent2: acc.tangent2 }
        }
        if friction > 0.0F && acc.normal > 0.0F {
          // Recompute after normal impulse.
          let v1b = if rb1 is Some(b1) {
            vel_at(b1, c0.point1)
          } else {
            @core.Vec3::zero()
          }
          let v2b = if rb2 is Some(b2) {
            vel_at(b2, c0.point2)
          } else {
            @core.Vec3::zero()
          }
          let rvb = v2b.sub(v1b)
          let vt1 = rvb.dot(t1)
          let vt2 = rvb.dot(t2)
          let kt1 = inv_m1 + inv_m2 + k_term(i1, r1, t1) + k_term(i2, r2, t1)
          let kt2 = inv_m1 + inv_m2 + k_term(i1, r1, t2) + k_term(i2, r2, t2)
          if kt1 > 0.0F && kt2 > 0.0F {
            let prev_t1 = acc.tangent1
            let prev_t2 = acc.tangent2
            let cand_t1 = prev_t1 + -vt1 / kt1
            let cand_t2 = prev_t2 + -vt2 / kt2
            let max_t = friction * acc.normal
            let mag2 = cand_t1 * cand_t1 + cand_t2 * cand_t2
            let max2 = max_t * max_t
            let (next_t1, next_t2) = if mag2 > max2 && mag2 > 1.0e-12F {
              let k = max_t / local_sqrt(mag2)
              (cand_t1 * k, cand_t2 * k)
            } else {
              (cand_t1, cand_t2)
            }
            let delta_t1 = next_t1 - prev_t1
            let delta_t2 = next_t2 - prev_t2
            if delta_t1 != 0.0F || delta_t2 != 0.0F {
              let p = t1.scale(delta_t1).add(t2.scale(delta_t2))
              if rb1 is Some(b1) && b1.body_type().is_dynamic() {
                apply_impulse(b1, p.scale(-1.0F), c0.point1)
              }
              if rb2 is Some(b2) && b2.body_type().is_dynamic() {
                apply_impulse(b2, p, c0.point2)
              }
            }
            acc = { normal: acc.normal, tangent1: next_t1, tangent2: next_t2 }
          }
        }
        cache_set(cache_out, a, b, acc)
      }
    }
  }
}
