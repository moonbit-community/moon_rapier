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
/// Real dim3 physics pipeline (broadphase + narrowphase + contact solver).
///
/// This is an incremental replacement for the current dim3 compatibility layer.
pub struct PhysicsPipeline3DReal {
  mut contact_cache : ContactSolverCache3D
  sensor_pairs : Array[
    (@collision.ColliderHandle3D, @collision.ColliderHandle3D),
  ]
}

///|
pub fn PhysicsPipeline3DReal::new() -> PhysicsPipeline3DReal {
  { contact_cache: ContactSolverCache3D::new(), sensor_pairs: [] }
}

///|
fn ccd_toi_ball_ball(
  center1 : @core.Vec3,
  radius1 : @core.Real,
  vel1 : @core.Vec3,
  center2 : @core.Vec3,
  radius2 : @core.Real,
  max_toi : @core.Real,
) -> @core.Real? {
  let r = radius1 + radius2
  let p = center1.sub(center2)
  let c = p.length_squared() - r * r
  // Already intersecting.
  if c <= 0.0F {
    return Some(0.0F)
  }
  let a = vel1.length_squared()
  if a <= 1.0e-12F {
    return None
  }
  let b = p.dot(vel1)
  let disc = b * b - a * c
  if disc < 0.0F {
    return None
  }
  let t = (-b - Float::sqrt(disc)) / a
  if t >= 0.0F && t <= max_toi {
    Some(t)
  } else {
    None
  }
}

///|
fn ccd_toi_ball_cuboid(
  ball_center : @core.Vec3,
  ball_radius : @core.Real,
  ball_vel : @core.Vec3,
  cuboid_pos : @core.Isometry3,
  cuboid_half_extents : @core.Vec3,
  max_toi : @core.Real,
) -> @core.Real? {
  // Sphere vs OBB TOI, approximated as point vs expanded box in the box local-space.
  let inv = cuboid_pos.inverse()
  let local_center = inv.transform_point(ball_center)
  let rot_inv = cuboid_pos.rotation.inverse()
  let local_vel = rot_inv.rotate_vec3(ball_vel)
  let he = cuboid_half_extents.add(
    @core.Vec3::new(ball_radius, ball_radius, ball_radius),
  )
  let mut tmin = 0.0F
  let mut tmax = max_toi
  fn axis_slab(
    p0 : @core.Real,
    v : @core.Real,
    he : @core.Real,
    tmin : @core.Real,
    tmax : @core.Real,
  ) -> (@core.Real, @core.Real)? {
    let mut tmin = tmin
    let mut tmax = tmax
    if @core.abs(v) <= 1.0e-12F {
      if p0 < -he || p0 > he {
        return None
      }
      return Some((tmin, tmax))
    }
    let inv_v = 1.0F / v
    let t1 = (-he - p0) * inv_v
    let t2 = (he - p0) * inv_v
    let lo = if t1 < t2 { t1 } else { t2 }
    let hi = if t1 < t2 { t2 } else { t1 }
    if lo > tmin {
      tmin = lo
    }
    if hi < tmax {
      tmax = hi
    }
    if tmin > tmax {
      None
    } else {
      Some((tmin, tmax))
    }
  }

  if axis_slab(local_center.x, local_vel.x, he.x, tmin, tmax) is Some(pair) {
    tmin = pair.0
    tmax = pair.1
  } else {
    return None
  }
  if axis_slab(local_center.y, local_vel.y, he.y, tmin, tmax) is Some(pair) {
    tmin = pair.0
    tmax = pair.1
  } else {
    return None
  }
  if axis_slab(local_center.z, local_vel.z, he.z, tmin, tmax) is Some(pair) {
    tmin = pair.0
    tmax = pair.1
  } else {
    return None
  }
  if tmax < 0.0F {
    None
  } else if tmin < 0.0F {
    Some(0.0F)
  } else {
    Some(tmin)
  }
}

///|
fn ccd_min_toi_ball_3d(
  bodies : @dynamics.RigidBodySet3D,
  colliders : @collision.ColliderSet3D,
  max_toi : @core.Real,
) -> @core.Real? {
  let handles = colliders.all_handles()
  let mut best : @core.Real? = None
  for i in 0..<handles.length() {
    let h1 = handles[i]
    if colliders.get(h1) is Some(c1) {
      if c1.parent() is Some(bh1) && bodies.get(bh1) is Some(rb1) {
        if !rb1.body_type().is_dynamic() ||
          !rb1.is_active() ||
          !rb1.is_ccd_active() {
          continue
        }
        match c1.shape() {
          @collision.Shape3D::Ball(r1) => {
            let p1 = c1.position().translation
            let v1 = rb1.linvel()
            for j in 0..<handles.length() {
              let h2 = handles[j]
              if h1.equals(h2) {
                continue
              }
              if colliders.get(h2) is Some(c2) {
                if c2.parent() is Some(bh2) && bh2.equals(bh1) {
                  continue
                }
                if !c1.collision_groups().test_groups(c2.collision_groups()) {
                  continue
                }
                let v2 = if c2.parent() is Some(bh2) &&
                  bodies.get(bh2) is Some(rb2) &&
                  rb2.body_type().is_dynamic() &&
                  rb2.is_active() {
                  rb2.linvel()
                } else {
                  @core.Vec3::zero()
                }
                let rel_v = v1.sub(v2)
                if rel_v.length_squared() <= 1.0e-12F {
                  continue
                }
                let hit : @core.Real? = match c2.shape() {
                  @collision.Shape3D::Ball(r2) =>
                    ccd_toi_ball_ball(
                      p1,
                      r1,
                      rel_v,
                      c2.position().translation,
                      r2,
                      max_toi,
                    )
                  @collision.Shape3D::Cuboid(he) =>
                    ccd_toi_ball_cuboid(
                      p1,
                      r1,
                      rel_v,
                      c2.position(),
                      he,
                      max_toi,
                    )
                  _ => None
                }
                if hit is Some(t) {
                  if best is Some(curr) {
                    if t < curr {
                      best = Some(t)
                    }
                  } else {
                    best = Some(t)
                  }
                }
              }
            }
          }
          _ => ()
        }
      }
    }
  }
  best
}

///|
fn pair_key_3d(
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
fn canonical_pair_3d(
  a : @collision.ColliderHandle3D,
  b : @collision.ColliderHandle3D,
) -> (@collision.ColliderHandle3D, @collision.ColliderHandle3D) {
  let (ai, ag, bi, bg) = pair_key_3d(a, b)
  (
    @collision.ColliderHandle3D::from_raw_parts(ai, ag),
    @collision.ColliderHandle3D::from_raw_parts(bi, bg),
  )
}

///|
fn pair_less_3d(
  a : (@collision.ColliderHandle3D, @collision.ColliderHandle3D),
  b : (@collision.ColliderHandle3D, @collision.ColliderHandle3D),
) -> Bool {
  let ka = pair_key_3d(a.0, a.1)
  let kb = pair_key_3d(b.0, b.1)
  if ka.0 != kb.0 {
    return ka.0 < kb.0
  }
  if ka.1 != kb.1 {
    return ka.1 < kb.1
  }
  if ka.2 != kb.2 {
    return ka.2 < kb.2
  }
  ka.3 < kb.3
}

///|
fn sort_pairs_3d(
  pairs : Array[(@collision.ColliderHandle3D, @collision.ColliderHandle3D)],
) -> Unit {
  // Insertion sort (small N in tests; replace with faster sort later).
  for i in 1..<pairs.length() {
    let mut j = i
    while j > 0 && pair_less_3d(pairs[j], pairs[j - 1]) {
      let tmp = pairs[j - 1]
      pairs[j - 1] = pairs[j]
      pairs[j] = tmp
      j = j - 1
    }
  }
}

///|
fn collect_sensor_pairs(
  narrow_phase : @collision.NarrowPhase3D,
  colliders : @collision.ColliderSet3D,
) -> Array[(@collision.ColliderHandle3D, @collision.ColliderHandle3D)] {
  let out : Array[(@collision.ColliderHandle3D, @collision.ColliderHandle3D)] = []
  let all = narrow_phase.all_contact_pairs()
  for i in 0..<all.length() {
    let pair = all[i].0
    let cp = all[i].1
    if cp.manifolds_len() == 0 {
      continue
    }
    let (a, b) = canonical_pair_3d(pair.0, pair.1)
    if colliders.get(a) is Some(ca) && colliders.get(b) is Some(cb) {
      let sensor = ca.is_sensor() || cb.is_sensor()
      if !sensor {
        continue
      }
      let wants = ca
        .active_events()
        .contains(@collision.ActiveEvents::collision_events()) ||
        cb.active_events().contains(@collision.ActiveEvents::collision_events())
      if !wants {
        continue
      }
      out.push((a, b))
    }
  }
  sort_pairs_3d(out)
  out
}

///|
pub fn PhysicsPipeline3DReal::step(
  self : PhysicsPipeline3DReal,
  gravity : @core.Vec3,
  parameters : @dynamics.IntegrationParameters,
  islands : @dynamics.IslandManager3D,
  broad_phase : @collision.BroadPhase3D,
  narrow_phase : @collision.NarrowPhase3D,
  bodies : @dynamics.RigidBodySet3D,
  colliders : @collision.ColliderSet3D,
) -> Unit {
  PhysicsPipeline3DReal::step_impl(
    self,
    gravity,
    parameters,
    islands,
    broad_phase,
    narrow_phase,
    bodies,
    colliders,
    None,
    None,
    None,
  )
}

///|
pub fn PhysicsPipeline3DReal::step_with_events(
  self : PhysicsPipeline3DReal,
  gravity : @core.Vec3,
  parameters : @dynamics.IntegrationParameters,
  islands : @dynamics.IslandManager3D,
  broad_phase : @collision.BroadPhase3D,
  narrow_phase : @collision.NarrowPhase3D,
  bodies : @dynamics.RigidBodySet3D,
  colliders : @collision.ColliderSet3D,
  events : EventHandler3D,
) -> Unit {
  PhysicsPipeline3DReal::step_impl(
    self,
    gravity,
    parameters,
    islands,
    broad_phase,
    narrow_phase,
    bodies,
    colliders,
    Some(events),
    None,
    None,
  )
}

///|
pub fn PhysicsPipeline3DReal::step_with_rope_joints(
  self : PhysicsPipeline3DReal,
  gravity : @core.Vec3,
  parameters : @dynamics.IntegrationParameters,
  islands : @dynamics.IslandManager3D,
  broad_phase : @collision.BroadPhase3D,
  narrow_phase : @collision.NarrowPhase3D,
  bodies : @dynamics.RigidBodySet3D,
  colliders : @collision.ColliderSet3D,
  rope_joints : @dynamics.RopeJointSet3DReal,
) -> Unit {
  PhysicsPipeline3DReal::step_impl(
    self,
    gravity,
    parameters,
    islands,
    broad_phase,
    narrow_phase,
    bodies,
    colliders,
    None,
    Some(rope_joints),
    None,
  )
}

///|
pub fn PhysicsPipeline3DReal::step_with_joints(
  self : PhysicsPipeline3DReal,
  gravity : @core.Vec3,
  parameters : @dynamics.IntegrationParameters,
  islands : @dynamics.IslandManager3D,
  broad_phase : @collision.BroadPhase3D,
  narrow_phase : @collision.NarrowPhase3D,
  bodies : @dynamics.RigidBodySet3D,
  colliders : @collision.ColliderSet3D,
  joints : @dynamics.JointSet3DReal,
) -> Unit {
  PhysicsPipeline3DReal::step_impl(
    self,
    gravity,
    parameters,
    islands,
    broad_phase,
    narrow_phase,
    bodies,
    colliders,
    None,
    None,
    Some(joints),
  )
}

///|
fn PhysicsPipeline3DReal::step_impl(
  self : PhysicsPipeline3DReal,
  gravity : @core.Vec3,
  parameters : @dynamics.IntegrationParameters,
  islands : @dynamics.IslandManager3D,
  broad_phase : @collision.BroadPhase3D,
  narrow_phase : @collision.NarrowPhase3D,
  bodies : @dynamics.RigidBodySet3D,
  colliders : @collision.ColliderSet3D,
  events : EventHandler3D?,
  rope_joints : @dynamics.RopeJointSet3DReal?,
  joints : @dynamics.JointSet3DReal?,
) -> Unit {
  fn ceil_pos(x : @core.Real) -> Int {
    let i = x.to_int()
    if Float::from_int(i) == x {
      i
    } else {
      i + 1
    }
  }

  let dt = parameters.dt
  if dt <= 0.0F {
    return
  }
  // Substep to reduce tunneling in the minimal dim3 backend.
  //
  // This is NOT a full TOI-based CCD, but it is sufficient for the current parity tests and
  // keeps behavior deterministic.
  let mut substeps = 1
  let max_speed = bodies.max_linvel_len()
  // Keep displacement small enough to avoid crossing thin shapes within a single substep.
  let max_disp = 0.03F * parameters.length_unit
  if max_disp > 0.0F && max_speed * dt > max_disp {
    substeps = ceil_pos(max_speed * dt / max_disp)
  }
  if bodies.has_ccd_enabled() {
    let max_disp_ccd = 0.01F * parameters.length_unit
    if max_disp_ccd > 0.0F && max_speed * dt > max_disp_ccd {
      let needed = ceil_pos(max_speed * dt / max_disp_ccd)
      if needed > substeps {
        substeps = needed
      }
    }
  }
  if substeps > 64 {
    substeps = 64
  }
  let sub_dt = dt / Float::from_int(substeps)
  let sub_params = parameters.with_dt(sub_dt)
  for _ in 0..<substeps {
    // Minimal TOI-based CCD for fast moving balls.
    if bodies.has_ccd_enabled() &&
      ccd_min_toi_ball_3d(bodies, colliders, sub_dt) is Some(toi) &&
      toi > 1.0e-6F &&
      toi < sub_dt {
      let p1 = parameters.with_dt(toi)
      PhysicsPipeline3DReal::step_impl_one(
        self, gravity, p1, islands, broad_phase, narrow_phase, bodies, colliders,
        events, rope_joints, joints,
      )
      let rem = sub_dt - toi
      if rem > 0.0F {
        let p2 = parameters.with_dt(rem)
        PhysicsPipeline3DReal::step_impl_one(
          self, gravity, p2, islands, broad_phase, narrow_phase, bodies, colliders,
          events, rope_joints, joints,
        )
      }
    } else {
      PhysicsPipeline3DReal::step_impl_one(
        self, gravity, sub_params, islands, broad_phase, narrow_phase, bodies, colliders,
        events, rope_joints, joints,
      )
    }
  }
}

///|
fn PhysicsPipeline3DReal::step_impl_one(
  self : PhysicsPipeline3DReal,
  gravity : @core.Vec3,
  parameters : @dynamics.IntegrationParameters,
  islands : @dynamics.IslandManager3D,
  broad_phase : @collision.BroadPhase3D,
  narrow_phase : @collision.NarrowPhase3D,
  bodies : @dynamics.RigidBodySet3D,
  colliders : @collision.ColliderSet3D,
  events : EventHandler3D?,
  rope_joints : @dynamics.RopeJointSet3DReal?,
  joints : @dynamics.JointSet3DReal?,
) -> Unit {
  let dt = parameters.dt
  bodies.apply_gravity_all(gravity, dt)
  bodies.apply_damping_all(dt)
  bodies.apply_gyroscopic_forces_all(dt)
  bodies.update_kinematic_velocities_all(dt)

  // Sync colliders to body motion before detecting contacts.
  colliders.sync_with_bodies(bodies)
  let prediction_distance = parameters.prediction_distance()
  broad_phase.update(prediction_distance, colliders)
  narrow_phase.update(broad_phase.pairs(), bodies, colliders)
  let cache_mid = ContactSolverCache3D::new()
  solve_contacts_3d(
    parameters,
    bodies,
    colliders,
    narrow_phase,
    self.contact_cache,
    cache_mid,
  )
  bodies.advance_positions_all(dt)
  colliders.sync_with_bodies(bodies)

  // Joint constraints are applied after integrating positions so they can clamp fast motion
  // deterministically (sufficient for the current parity tests).
  let mut applied_constraints = false
  if rope_joints is Some(rj) {
    rj.solve(bodies, dt)
    applied_constraints = true
  }
  if joints is Some(js) {
    js.solve(bodies, dt)
    applied_constraints = true
  }
  if applied_constraints {
    colliders.sync_with_bodies(bodies)
  }

  // Update the collision state after integrating positions so queries and sensor events reflect
  // the end-of-step configuration.
  broad_phase.update(prediction_distance, colliders)
  narrow_phase.update(broad_phase.pairs(), bodies, colliders)

  // Solve contacts once more using the end-of-step contact state so fast-moving bodies don't
  // wait an extra frame to respond to newly created overlaps.
  let cache_out = ContactSolverCache3D::new()
  solve_contacts_3d(
    parameters, bodies, colliders, narrow_phase, cache_mid, cache_out,
  )
  self.contact_cache = cache_out

  // Sensor collision events (Started/Stopped) are based on the end-of-step contact pairs.
  let next_sensor_pairs = collect_sensor_pairs(narrow_phase, colliders)
  if events is Some(handler) {
    let prev_pairs = self.sensor_pairs.copy()
    sort_pairs_3d(prev_pairs)
    let mut i = 0
    let mut j = 0
    while i < prev_pairs.length() || j < next_sensor_pairs.length() {
      if i >= prev_pairs.length() {
        let p = next_sensor_pairs[j]
        handler.push_collision_event(
          @collision.CollisionEvent3D::Started(
            p.0,
            p.1,
            @collision.CollisionEventFlags::sensor(),
          ),
        )
        j = j + 1
        continue
      }
      if j >= next_sensor_pairs.length() {
        let p = prev_pairs[i]
        handler.push_collision_event(
          @collision.CollisionEvent3D::Stopped(
            p.0,
            p.1,
            @collision.CollisionEventFlags::sensor(),
          ),
        )
        i = i + 1
        continue
      }
      let a = prev_pairs[i]
      let b = next_sensor_pairs[j]
      if pair_less_3d(a, b) {
        handler.push_collision_event(
          @collision.CollisionEvent3D::Stopped(
            a.0,
            a.1,
            @collision.CollisionEventFlags::sensor(),
          ),
        )
        i = i + 1
      } else if pair_less_3d(b, a) {
        handler.push_collision_event(
          @collision.CollisionEvent3D::Started(
            b.0,
            b.1,
            @collision.CollisionEventFlags::sensor(),
          ),
        )
        j = j + 1
      } else {
        i = i + 1
        j = j + 1
      }
    }
  }
  self.sensor_pairs.clear()
  self.sensor_pairs.append(next_sensor_pairs[:])
  bodies.update_sleep_all(dt, parameters.length_unit)
  islands.update(bodies)
}
