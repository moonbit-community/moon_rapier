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
  priv mut contact_twist_cache : @hashmap.HashMap[
    (Int, Int, Int, Int),
    @core.Real,
  ]
  sensor_pairs : Array[
    (@collision.ColliderHandle3D, @collision.ColliderHandle3D),
  ]
  contact_pairs : Array[
    (@collision.ColliderHandle3D, @collision.ColliderHandle3D),
  ]
  intersection_pairs : Array[
    (@collision.ColliderHandle3D, @collision.ColliderHandle3D),
  ]

  fn new() -> PhysicsPipeline3DReal
}

///|
pub fn PhysicsPipeline3DReal::new() -> PhysicsPipeline3DReal {
  {
    contact_cache: ContactSolverCache3D::new(),
    contact_twist_cache: @hashmap.new(),
    sensor_pairs: [],
    contact_pairs: [],
    intersection_pairs: [],
  }
}

///|
fn integrate_lin_ang_3d_real(
  dt : @core.Real,
  curr_pos : @core.Isometry3,
  local_com : @core.Vec3,
  linvel : @core.Vec3,
  angvel : @core.Vec3,
) -> @core.Isometry3 {
  let com = curr_pos.transform_point(local_com)
  let shift = @core.Isometry3::from_translation(
    @core.Vec3::new(-com.x, -com.y, -com.z),
  )
  let rot = @core.Isometry3::new(
    @core.Vec3::zero(),
    @core.rotation_from_scaled_axis(angvel.scale(dt)),
  )
  let translation = linvel.scale(dt)
  let unshift = @core.Isometry3::from_translation(com.add(translation))
  unshift.mul(rot).mul(shift).mul(curr_pos)
}

///|
fn shape_bounding_radius_3d_real(shape : @collision.Shape3D) -> @core.Real {
  match shape {
    @collision.Shape3D::HalfSpace(_) => 0.0F
    _ => {
      let aabb = shape.local_aabb()
      let he = aabb.maxs.sub(aabb.mins).scale(0.5F)
      Float::sqrt(he.length_squared())
    }
  }
}

///|
fn shape_ccd_thickness_3d_real(shape : @collision.Shape3D) -> @core.Real {
  match shape {
    @collision.Shape3D::HalfSpace(_) => 0.0F
    @collision.Shape3D::Triangle(_, _, _) => 0.0F
    @collision.Shape3D::Heightfield(_, _, _, _, _) => 0.0F
    @collision.Shape3D::TriMesh(_, _) => 0.0F
    @collision.Shape3D::Compound(parts) => {
      if parts.length() == 0 {
        return 0.0F
      }
      let mut best = 1.0e30F
      for i in 0..<parts.length() {
        let t = shape_ccd_thickness_3d_real(parts[i].1)
        if t < best {
          best = t
        }
      }
      if best >= 1.0e29F {
        0.0F
      } else {
        best
      }
    }
    _ => {
      let aabb = shape.local_aabb()
      let he = aabb.maxs.sub(aabb.mins).scale(0.5F)
      let mut t = he.x
      if he.y < t {
        t = he.y
      }
      if he.z < t {
        t = he.z
      }
      if t > 0.0F {
        t
      } else {
        0.0F
      }
    }
  }
}

///|
fn body_ccd_metrics_3d_real(
  handle : @dynamics.RigidBodyHandle,
  colliders : @collision.ColliderSet3D,
) -> (@core.Real, @core.Real, Bool) {
  let mut ccd_thickness = 1.0e30F
  let mut ccd_max_dist = 0.0F
  let mut found = false
  let collider_handles = colliders.all_handles()
  for i in 0..<collider_handles.length() {
    let ch = collider_handles[i]
    if colliders.get(ch) is Some(co) &&
      co.parent() is Some(parent) &&
      parent.equals(handle) {
      found = true
      let thickness = shape_ccd_thickness_3d_real(co.shape())
      if thickness < ccd_thickness {
        ccd_thickness = thickness
      }
      let radius = shape_bounding_radius_3d_real(co.shape())
      let dist = co.local_position().translation.length() + radius
      if dist > ccd_max_dist {
        ccd_max_dist = dist
      }
    }
  }
  if !found {
    (0.0F, 0.0F, false)
  } else if ccd_thickness >= 1.0e29F {
    (0.0F, ccd_max_dist, true)
  } else {
    (ccd_thickness, ccd_max_dist, true)
  }
}

///|
fn update_ccd_active_flags_3d_real(
  dt : @core.Real,
  bodies : @dynamics.RigidBodySet3D,
  colliders : @collision.ColliderSet3D,
  _include_forces : Bool,
) -> Bool {
  if dt <= 0.0F {
    return false
  }
  let mut any_active = false
  let body_handles = bodies.all_handles()
  for i in 0..<body_handles.length() {
    let bh = body_handles[i]
    if bodies.get_mut(bh) is Some(rb) {
      if !rb.is_ccd_enabled() ||
        !rb.body_type().is_dynamic_or_kinematic() ||
        !rb.is_active() {
        rb.set_ccd_active(false)
        continue
      }
      let (ccd_thickness, ccd_max_dist, has_attached_collider) = body_ccd_metrics_3d_real(
        bh, colliders,
      )
      if !has_attached_collider {
        rb.set_ccd_active(false)
        continue
      }
      let max_point_velocity = rb.linvel().length() +
        rb.angvel().length() * ccd_max_dist
      let threshold = ccd_thickness / 10.0F
      let moving_fast = max_point_velocity * dt > threshold
      rb.set_ccd_active(moving_fast)
      if moving_fast {
        any_active = true
      }
    }
  }
  any_active
}

///|
fn find_first_impact_3d_real(
  dt : @core.Real,
  bodies : @dynamics.RigidBodySet3D,
  colliders : @collision.ColliderSet3D,
) -> @core.Real? {
  if dt <= 0.0F {
    return None
  }
  let base_query = @collision.QueryPipeline3DReal::new(
    @collision.QueryFilter3DReal::new(),
    bodies,
    colliders,
  )
  let mut best : @core.Real? = None
  let collider_handles = colliders.all_handles()
  for i in 0..<collider_handles.length() {
    let ch = collider_handles[i]
    if colliders.get(ch) is Some(co) &&
      co.enabled() &&
      !co.is_sensor() &&
      co.parent() is Some(bh) &&
      bodies.get(bh) is Some(rb) {
      if !rb.is_ccd_active() ||
        !rb.body_type().is_dynamic_or_kinematic() ||
        !rb.is_active() {
        continue
      }
      let start_pos = co.position()
      let end_body_pos = integrate_lin_ang_3d_real(
        dt,
        rb.position(),
        rb.mass_properties().center_of_mass,
        rb.linvel(),
        rb.angvel(),
      )
      let end_pos = end_body_pos.mul(co.local_position())
      let delta = end_pos.translation.sub(start_pos.translation)
      if delta.length_squared() <= 1.0e-12F {
        continue
      }
      let shape_vel = delta.scale(1.0F / dt)
      let query = base_query.with_filter(
        @collision.QueryFilter3DReal::new()
        .exclude_rigid_body(bh)
        .groups(co.collision_groups())
        .exclude_sensors(),
      )
      let options = @collision.ShapeCastOptions3::new(dt, true)
      if query.cast_shape(
          bodies,
          colliders,
          start_pos,
          shape_vel,
          co.shape(),
          options,
        )
        is Some((_, hit)) {
        let toi = hit.toi()
        if toi >= 0.0F && toi < dt {
          if best is Some(curr) {
            if toi < curr {
              best = Some(toi)
            }
          } else {
            best = Some(toi)
          }
        }
      }
    }
  }
  best
}

///|
fn clamp_fast_ccd_body_motions_3d_real(
  dt : @core.Real,
  bodies : @dynamics.RigidBodySet3D,
  colliders : @collision.ColliderSet3D,
  require_ccd_active : Bool,
) -> Unit {
  if dt <= 0.0F {
    return
  }
  let base_query = @collision.QueryPipeline3DReal::new(
    @collision.QueryFilter3DReal::new(),
    bodies,
    colliders,
  )
  let best_toi_by_body : @hashmap.HashMap[(Int, Int), @core.Real] = @hashmap.new()
  let collider_handles = colliders.all_handles()
  for i in 0..<collider_handles.length() {
    let ch = collider_handles[i]
    if colliders.get(ch) is Some(co) &&
      co.enabled() &&
      !co.is_sensor() &&
      co.parent() is Some(bh) &&
      bodies.get(bh) is Some(rb) {
      if !rb.body_type().is_dynamic_or_kinematic() || !rb.is_active() {
        continue
      }
      if require_ccd_active && !rb.is_ccd_active() {
        continue
      }
      let v = rb.linvel()
      let w = rb.angvel()
      if v.length_squared() <= 1.0e-12F && w.length_squared() <= 1.0e-12F {
        continue
      }
      let start_pos = co.position()
      let end_body_pos = integrate_lin_ang_3d_real(
        dt,
        rb.position(),
        rb.mass_properties().center_of_mass,
        v,
        w,
      )
      let end_pos = end_body_pos.mul(co.local_position())
      let delta = end_pos.translation.sub(start_pos.translation)
      if delta.length_squared() <= 1.0e-12F {
        continue
      }
      let shape_vel = delta.scale(1.0F / dt)
      let query = base_query.with_filter(
        @collision.QueryFilter3DReal::new()
        .exclude_rigid_body(bh)
        .groups(co.collision_groups())
        .exclude_sensors(),
      )
      let options = @collision.ShapeCastOptions3::new(dt, true)
      if query.cast_shape(
          bodies,
          colliders,
          start_pos,
          shape_vel,
          co.shape(),
          options,
        )
        is Some((_, hit)) {
        let toi = hit.toi()
        if toi >= 0.0F && toi < dt {
          let key = bh.into_raw_parts()
          if best_toi_by_body.get(key) is Some(best) {
            if toi < best {
              best_toi_by_body.set(key, toi)
            }
          } else {
            best_toi_by_body.set(key, toi)
          }
        }
      }
    }
  }
  let body_handles = bodies.all_handles()
  for i in 0..<body_handles.length() {
    let bh = body_handles[i]
    let key = bh.into_raw_parts()
    if best_toi_by_body.get(key) is Some(toi) && bodies.get_mut(bh) is Some(rb) {
      let mut effective_toi = toi
      if require_ccd_active {
        let (ccd_thickness, ccd_max_dist, _) = body_ccd_metrics_3d_real(
          bh, colliders,
        )
        let max_point_velocity = rb.linvel().length() +
          rb.angvel().length() * ccd_max_dist
        let min_toi = if max_point_velocity > 0.0F {
          let value = ccd_thickness * 0.15F / max_point_velocity
          if value < dt {
            value
          } else {
            dt
          }
        } else {
          dt
        }
        effective_toi = if toi > min_toi { toi } else { min_toi }
      }
      let s = effective_toi / dt
      if s < 1.0F {
        rb.set_linvel(rb.linvel().scale(s))
        rb.set_angvel(rb.angvel().scale(s))
      }
    }
  }
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
  let all = narrow_phase.all_intersection_pairs()
  for i in 0..<all.length() {
    let pair = all[i].0
    let ip = all[i].1
    if !ip.intersecting() {
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
fn collect_intersection_pairs(
  narrow_phase : @collision.NarrowPhase3D,
  colliders : @collision.ColliderSet3D,
) -> Array[(@collision.ColliderHandle3D, @collision.ColliderHandle3D)] {
  let out : Array[(@collision.ColliderHandle3D, @collision.ColliderHandle3D)] = []
  let all = narrow_phase.all_intersection_pairs()
  for i in 0..<all.length() {
    let pair = all[i].0
    let ip = all[i].1
    if !ip.intersecting() {
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
        .contains(@collision.ActiveEvents::intersection_events()) ||
        cb
        .active_events()
        .contains(@collision.ActiveEvents::intersection_events())
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
fn collect_contact_pairs(
  narrow_phase : @collision.NarrowPhase3D,
  colliders : @collision.ColliderSet3D,
) -> Array[(@collision.ColliderHandle3D, @collision.ColliderHandle3D)] {
  let out : Array[(@collision.ColliderHandle3D, @collision.ColliderHandle3D)] = []
  let all = narrow_phase.all_contact_pairs()
  for i in 0..<all.length() {
    let pair = all[i].0
    let cp = all[i].1
    if cp.manifolds_len() <= 0 {
      continue
    }
    let (a, b) = canonical_pair_3d(pair.0, pair.1)
    if colliders.get(a) is Some(ca) && colliders.get(b) is Some(cb) {
      let sensor = ca.is_sensor() || cb.is_sensor()
      if sensor {
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
/// Restore contact-pair impulse data from the solver cache after a narrow-phase refresh.
///
/// `PhysicsPipeline3DReal::step_impl_one` refreshes the narrow-phase after integrating positions
/// in order to emit collision/intersection events based on the end-of-step configuration.
/// This refresh clears the impulse data written by the contact solver, so we reapply it here to
/// keep `ContactPair3D::{total_impulse,*}` usable after `step_with_events`.
fn restore_contact_pair_impulses_from_cache(
  narrow_phase : @collision.NarrowPhase3D,
  cache : ContactSolverCache3D,
) -> Unit {
  let all = narrow_phase.all_contact_pairs()
  for i in 0..<all.length() {
    let pair = all[i].0
    let cp = all[i].1
    if cp.manifolds_len() <= 0 {
      continue
    }
    let impulses : Array[@core.Real] = []
    for _ in 0..<cp.manifolds_len() {
      impulses.push(0.0F)
    }
    if cache.entries.get(pair_key_3d(pair.0, pair.1)) is Some(list) {
      let n = impulses.length()
      let m = list.length()
      let k = if n < m { n } else { m }
      for j in 0..<k {
        impulses[j] = list[j].2.normal
      }
    }
    narrow_phase.set_contact_pair_normal_impulses(pair.0, pair.1, impulses)
  }
}

///|
fn emit_collision_events_for_pair_diffs(
  handler : EventHandler3D,
  colliders : @collision.ColliderSet3D,
  prev_pairs : Array[(@collision.ColliderHandle3D, @collision.ColliderHandle3D)],
  next_pairs : Array[(@collision.ColliderHandle3D, @collision.ColliderHandle3D)],
  flags : @collision.CollisionEventFlags,
) -> Unit {
  fn stopped_flags(
    colliders : @collision.ColliderSet3D,
    a : @collision.ColliderHandle3D,
    b : @collision.ColliderHandle3D,
    flags : @collision.CollisionEventFlags,
  ) -> @collision.CollisionEventFlags {
    let mut out = flags
    if colliders.get(a) is None || colliders.get(b) is None {
      out = out.insert(@collision.CollisionEventFlags::removed())
    }
    out
  }

  sort_pairs_3d(prev_pairs)
  let mut i = 0
  let mut j = 0
  while i < prev_pairs.length() || j < next_pairs.length() {
    if i >= prev_pairs.length() {
      let p = next_pairs[j]
      handler.push_collision_event(
        @collision.CollisionEvent3D::Started(p.0, p.1, flags),
      )
      j = j + 1
      continue
    }
    if j >= next_pairs.length() {
      let p = prev_pairs[i]
      let event_flags = stopped_flags(colliders, p.0, p.1, flags)
      handler.push_collision_event(
        @collision.CollisionEvent3D::Stopped(p.0, p.1, event_flags),
      )
      i = i + 1
      continue
    }
    let a = prev_pairs[i]
    let b = next_pairs[j]
    if pair_less_3d(a, b) {
      let event_flags = stopped_flags(colliders, a.0, a.1, flags)
      handler.push_collision_event(
        @collision.CollisionEvent3D::Stopped(a.0, a.1, event_flags),
      )
      i = i + 1
    } else if pair_less_3d(b, a) {
      handler.push_collision_event(
        @collision.CollisionEvent3D::Started(b.0, b.1, flags),
      )
      j = j + 1
    } else {
      i = i + 1
      j = j + 1
    }
  }
}

///|
fn emit_intersection_events_for_pair_diffs(
  handler : EventHandler3D,
  prev_pairs : Array[(@collision.ColliderHandle3D, @collision.ColliderHandle3D)],
  next_pairs : Array[(@collision.ColliderHandle3D, @collision.ColliderHandle3D)],
) -> Unit {
  sort_pairs_3d(prev_pairs)
  let mut i = 0
  let mut j = 0
  while i < prev_pairs.length() || j < next_pairs.length() {
    if i >= prev_pairs.length() {
      let p = next_pairs[j]
      handler.push_intersection_event(
        @collision.IntersectionEvent3D::new(p.0, p.1, true),
      )
      j = j + 1
      continue
    }
    if j >= next_pairs.length() {
      let p = prev_pairs[i]
      handler.push_intersection_event(
        @collision.IntersectionEvent3D::new(p.0, p.1, false),
      )
      i = i + 1
      continue
    }
    let a = prev_pairs[i]
    let b = next_pairs[j]
    if pair_less_3d(a, b) {
      handler.push_intersection_event(
        @collision.IntersectionEvent3D::new(a.0, a.1, false),
      )
      i = i + 1
    } else if pair_less_3d(b, a) {
      handler.push_intersection_event(
        @collision.IntersectionEvent3D::new(b.0, b.1, true),
      )
      j = j + 1
    } else {
      i = i + 1
      j = j + 1
    }
  }
}

///|
fn rb_pair_key_3d(
  a : @dynamics.RigidBodyHandle,
  b : @dynamics.RigidBodyHandle,
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
fn filter_broad_phase_pairs_3d(
  pairs : Array[(@collision.ColliderHandle3D, @collision.ColliderHandle3D)],
  bodies : @dynamics.RigidBodySet3D,
  colliders : @collision.ColliderSet3D,
  joints : @dynamics.JointSet3DReal?,
  multibody_joints : @dynamics.MultibodyJointSet3DReal?,
  hooks : PhysicsHooks3D?,
) -> Array[(@collision.ColliderHandle3D, @collision.ColliderHandle3D)] {
  let mut disabled : @hashmap.HashMap[(Int, Int, Int, Int), Bool]? = None
  let disabled_body_pairs : Array[
    (@dynamics.RigidBodyHandle, @dynamics.RigidBodyHandle),
  ] = []
  if joints is Some(js) {
    disabled_body_pairs.append(js.disabled_contact_pairs()[:])
  }
  if multibody_joints is Some(multibody) {
    disabled_body_pairs.append(multibody.disabled_contact_pairs()[:])
  }
  if disabled_body_pairs.length() > 0 {
    let out : @hashmap.HashMap[(Int, Int, Int, Int), Bool] = @hashmap.new(
      capacity=disabled_body_pairs.length() * 2,
    )
    for p in disabled_body_pairs {
      out.set(rb_pair_key_3d(p.0, p.1), true)
    }
    disabled = Some(out)
  }
  let filtered : Array[
    (@collision.ColliderHandle3D, @collision.ColliderHandle3D),
  ] = []
  for pair in pairs {
    let c1 = pair.0
    let c2 = pair.1
    if colliders.get(c1) is Some(co1) && colliders.get(c2) is Some(co2) {
      if disabled is Some(set) &&
        co1.parent() is Some(p1) &&
        co2.parent() is Some(p2) &&
        set.get(rb_pair_key_3d(p1, p2)) is Some(true) {
        continue
      }
      if hooks is Some(physics_hooks) {
        let keep = if co1.is_sensor() || co2.is_sensor() {
          let hook = @collision.ActiveHooks::filter_intersection_pair()
          if co1.active_hooks().contains(hook) ||
            co2.active_hooks().contains(hook) {
            physics_hooks.test_intersection_pair(bodies, colliders, c1, c2)
          } else {
            true
          }
        } else {
          let hook = @collision.ActiveHooks::filter_contact_pairs()
          if co1.active_hooks().contains(hook) ||
            co2.active_hooks().contains(hook) {
            physics_hooks.test_contact_pair(bodies, colliders, c1, c2)
          } else {
            true
          }
        }
        if !keep {
          continue
        }
      }
      filtered.push((c1, c2))
    }
  }
  filtered
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
    None,
    None,
  )
}

///|
pub fn PhysicsPipeline3DReal::step_with_hooks(
  self : PhysicsPipeline3DReal,
  gravity : @core.Vec3,
  parameters : @dynamics.IntegrationParameters,
  islands : @dynamics.IslandManager3D,
  broad_phase : @collision.BroadPhase3D,
  narrow_phase : @collision.NarrowPhase3D,
  bodies : @dynamics.RigidBodySet3D,
  colliders : @collision.ColliderSet3D,
  hooks : PhysicsHooks3D,
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
    None,
    Some(hooks),
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
    None,
    None,
  )
}

///|
pub fn PhysicsPipeline3DReal::step_with_events_and_hooks(
  self : PhysicsPipeline3DReal,
  gravity : @core.Vec3,
  parameters : @dynamics.IntegrationParameters,
  islands : @dynamics.IslandManager3D,
  broad_phase : @collision.BroadPhase3D,
  narrow_phase : @collision.NarrowPhase3D,
  bodies : @dynamics.RigidBodySet3D,
  colliders : @collision.ColliderSet3D,
  events : EventHandler3D,
  hooks : PhysicsHooks3D,
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
    None,
    Some(hooks),
  )
}

///|
pub fn PhysicsPipeline3DReal::step_with_events_and_joints(
  self : PhysicsPipeline3DReal,
  gravity : @core.Vec3,
  parameters : @dynamics.IntegrationParameters,
  islands : @dynamics.IslandManager3D,
  broad_phase : @collision.BroadPhase3D,
  narrow_phase : @collision.NarrowPhase3D,
  bodies : @dynamics.RigidBodySet3D,
  colliders : @collision.ColliderSet3D,
  events : EventHandler3D,
  rope_joints : @dynamics.RopeJointSet3DReal,
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
    Some(events),
    Some(rope_joints),
    Some(joints),
    None,
    None,
  )
}

///|
pub fn PhysicsPipeline3DReal::step_with_events_joints_and_hooks(
  self : PhysicsPipeline3DReal,
  gravity : @core.Vec3,
  parameters : @dynamics.IntegrationParameters,
  islands : @dynamics.IslandManager3D,
  broad_phase : @collision.BroadPhase3D,
  narrow_phase : @collision.NarrowPhase3D,
  bodies : @dynamics.RigidBodySet3D,
  colliders : @collision.ColliderSet3D,
  events : EventHandler3D,
  rope_joints : @dynamics.RopeJointSet3DReal,
  joints : @dynamics.JointSet3DReal,
  hooks : PhysicsHooks3D,
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
    Some(rope_joints),
    Some(joints),
    None,
    Some(hooks),
  )
}

///|
pub fn PhysicsPipeline3DReal::step_with_events_and_joint_sets(
  self : PhysicsPipeline3DReal,
  gravity : @core.Vec3,
  parameters : @dynamics.IntegrationParameters,
  islands : @dynamics.IslandManager3D,
  broad_phase : @collision.BroadPhase3D,
  narrow_phase : @collision.NarrowPhase3D,
  bodies : @dynamics.RigidBodySet3D,
  colliders : @collision.ColliderSet3D,
  events : EventHandler3D,
  rope_joints : @dynamics.RopeJointSet3DReal,
  joints : @dynamics.JointSet3DReal,
  multibody_joints : @dynamics.MultibodyJointSet3DReal,
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
    Some(rope_joints),
    Some(joints),
    Some(multibody_joints),
    None,
  )
}

///|
pub fn PhysicsPipeline3DReal::step_with_events_joint_sets_and_hooks(
  self : PhysicsPipeline3DReal,
  gravity : @core.Vec3,
  parameters : @dynamics.IntegrationParameters,
  islands : @dynamics.IslandManager3D,
  broad_phase : @collision.BroadPhase3D,
  narrow_phase : @collision.NarrowPhase3D,
  bodies : @dynamics.RigidBodySet3D,
  colliders : @collision.ColliderSet3D,
  events : EventHandler3D,
  rope_joints : @dynamics.RopeJointSet3DReal,
  joints : @dynamics.JointSet3DReal,
  multibody_joints : @dynamics.MultibodyJointSet3DReal,
  hooks : PhysicsHooks3D,
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
    Some(rope_joints),
    Some(joints),
    Some(multibody_joints),
    Some(hooks),
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
    None,
    None,
  )
}

///|
pub fn PhysicsPipeline3DReal::step_with_rope_joints_and_hooks(
  self : PhysicsPipeline3DReal,
  gravity : @core.Vec3,
  parameters : @dynamics.IntegrationParameters,
  islands : @dynamics.IslandManager3D,
  broad_phase : @collision.BroadPhase3D,
  narrow_phase : @collision.NarrowPhase3D,
  bodies : @dynamics.RigidBodySet3D,
  colliders : @collision.ColliderSet3D,
  rope_joints : @dynamics.RopeJointSet3DReal,
  hooks : PhysicsHooks3D,
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
    None,
    Some(hooks),
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
    None,
    None,
  )
}

///|
pub fn PhysicsPipeline3DReal::step_with_joints_and_hooks(
  self : PhysicsPipeline3DReal,
  gravity : @core.Vec3,
  parameters : @dynamics.IntegrationParameters,
  islands : @dynamics.IslandManager3D,
  broad_phase : @collision.BroadPhase3D,
  narrow_phase : @collision.NarrowPhase3D,
  bodies : @dynamics.RigidBodySet3D,
  colliders : @collision.ColliderSet3D,
  joints : @dynamics.JointSet3DReal,
  hooks : PhysicsHooks3D,
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
    None,
    Some(hooks),
  )
}

///|
pub fn PhysicsPipeline3DReal::step_with_multibody_joints(
  self : PhysicsPipeline3DReal,
  gravity : @core.Vec3,
  parameters : @dynamics.IntegrationParameters,
  islands : @dynamics.IslandManager3D,
  broad_phase : @collision.BroadPhase3D,
  narrow_phase : @collision.NarrowPhase3D,
  bodies : @dynamics.RigidBodySet3D,
  colliders : @collision.ColliderSet3D,
  multibody_joints : @dynamics.MultibodyJointSet3DReal,
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
    Some(multibody_joints),
    None,
  )
}

///|
pub fn PhysicsPipeline3DReal::step_with_multibody_joints_and_hooks(
  self : PhysicsPipeline3DReal,
  gravity : @core.Vec3,
  parameters : @dynamics.IntegrationParameters,
  islands : @dynamics.IslandManager3D,
  broad_phase : @collision.BroadPhase3D,
  narrow_phase : @collision.NarrowPhase3D,
  bodies : @dynamics.RigidBodySet3D,
  colliders : @collision.ColliderSet3D,
  multibody_joints : @dynamics.MultibodyJointSet3DReal,
  hooks : PhysicsHooks3D,
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
    Some(multibody_joints),
    Some(hooks),
  )
}

///|
pub fn PhysicsPipeline3DReal::step_with_events_and_multibody_joints(
  self : PhysicsPipeline3DReal,
  gravity : @core.Vec3,
  parameters : @dynamics.IntegrationParameters,
  islands : @dynamics.IslandManager3D,
  broad_phase : @collision.BroadPhase3D,
  narrow_phase : @collision.NarrowPhase3D,
  bodies : @dynamics.RigidBodySet3D,
  colliders : @collision.ColliderSet3D,
  events : EventHandler3D,
  multibody_joints : @dynamics.MultibodyJointSet3DReal,
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
    Some(multibody_joints),
    None,
  )
}

///|
pub fn PhysicsPipeline3DReal::step_with_events_multibody_joints_and_hooks(
  self : PhysicsPipeline3DReal,
  gravity : @core.Vec3,
  parameters : @dynamics.IntegrationParameters,
  islands : @dynamics.IslandManager3D,
  broad_phase : @collision.BroadPhase3D,
  narrow_phase : @collision.NarrowPhase3D,
  bodies : @dynamics.RigidBodySet3D,
  colliders : @collision.ColliderSet3D,
  events : EventHandler3D,
  multibody_joints : @dynamics.MultibodyJointSet3DReal,
  hooks : PhysicsHooks3D,
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
    Some(multibody_joints),
    Some(hooks),
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
  multibody_joints : @dynamics.MultibodyJointSet3DReal?,
  hooks : PhysicsHooks3D?,
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
  if multibody_joints is Some(multibody) {
    let to_wake_up = multibody.take_wake_up()
    for handle in to_wake_up {
      if bodies.get(handle) is Some(body) {
        body.wake_up()
      }
    }
    multibody.take_to_join() |> ignore
    multibody.sync_rigid_bodies_from_multibodies(bodies)
    colliders.sync_with_bodies(bodies)
  }
  let has_ccd_bodies = bodies.has_ccd_enabled()
  let ccd_enabled = has_ccd_bodies && parameters.max_ccd_substeps > 0
  let max_speed = bodies.max_linvel_len()
  let non_ccd_motion_clamping = !has_ccd_bodies &&
    dt <= 0.1F &&
    max_speed > 8.0F
  let motion_clamping_enabled = ccd_enabled || non_ccd_motion_clamping
  let mut non_ccd_substeps = 1
  if !ccd_enabled && !has_ccd_bodies {
    // Fallback stabilization for scenes without hard-CCD-enabled bodies.
    let max_disp = 0.03F * parameters.length_unit
    if max_disp > 0.0F && max_speed * dt > max_disp {
      non_ccd_substeps = ceil_pos(max_speed * dt / max_disp)
    }
    if non_ccd_substeps > 64 {
      non_ccd_substeps = 64
    }
  }
  let mut remaining_time = dt
  let mut remaining_substeps = if ccd_enabled {
    parameters.max_ccd_substeps
  } else {
    non_ccd_substeps
  }
  while remaining_substeps > 0 && remaining_time > 0.0F {
    let mut sub_dt = remaining_time
    if ccd_enabled && remaining_substeps > 1 {
      let ccd_active = update_ccd_active_flags_3d_real(
        remaining_time, bodies, colliders, true,
      )
      let first_impact = if ccd_active {
        find_first_impact_3d_real(remaining_time, bodies, colliders)
      } else {
        None
      }
      if first_impact is Some(toi) {
        let interval = remaining_time / Float::from_int(remaining_substeps)
        sub_dt = if toi < interval {
          interval
        } else {
          toi + (remaining_time - toi) / Float::from_int(remaining_substeps)
        }
        remaining_substeps = remaining_substeps - 1
      } else {
        sub_dt = remaining_time
        remaining_substeps = 0
      }
      remaining_time = remaining_time - sub_dt
      if remaining_time <= parameters.min_ccd_dt {
        sub_dt = sub_dt + remaining_time
        remaining_time = 0.0F
        remaining_substeps = 0
      }
    } else if !ccd_enabled && remaining_substeps > 1 {
      sub_dt = remaining_time / Float::from_int(remaining_substeps)
      remaining_time = remaining_time - sub_dt
      remaining_substeps = remaining_substeps - 1
    } else {
      remaining_time = 0.0F
      remaining_substeps = 0
    }
    if sub_dt > 0.0F {
      let sub_params = parameters.with_dt(sub_dt)
      PhysicsPipeline3DReal::step_impl_one(
        self, gravity, sub_params, islands, broad_phase, narrow_phase, bodies, colliders,
        events, rope_joints, joints, multibody_joints, hooks, ccd_enabled, motion_clamping_enabled,
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
  multibody_joints : @dynamics.MultibodyJointSet3DReal?,
  hooks : PhysicsHooks3D?,
  ccd_enabled : Bool,
  motion_clamping_enabled : Bool,
) -> Unit {
  let dt = parameters.dt
  bodies.apply_gravity_all(gravity, dt)
  bodies.apply_damping_all(dt)
  bodies.apply_gyroscopic_forces_all(dt)
  bodies.update_kinematic_velocities_all(dt)
  if multibody_joints is Some(multibody) {
    multibody.sync_rigid_bodies_from_multibodies(bodies)
  }

  // Sync colliders to body motion before detecting contacts.
  colliders.sync_with_bodies(bodies)
  let prediction_distance = parameters.prediction_distance()
  broad_phase.update(prediction_distance, colliders)
  let pairs = filter_broad_phase_pairs_3d(
    broad_phase.pairs(),
    bodies,
    colliders,
    joints,
    multibody_joints,
    hooks,
  )
  narrow_phase.update(pairs, bodies, colliders)
  let cache_out = ContactSolverCache3D::new()
  let twist_cache_out : @hashmap.HashMap[(Int, Int, Int, Int), @core.Real] = @hashmap.new()
  solve_contacts_3d_impl(
    parameters,
    bodies,
    colliders,
    narrow_phase,
    self.contact_cache,
    cache_out,
    self.contact_twist_cache,
    twist_cache_out,
    hooks,
    events,
  )
  self.contact_cache = cache_out
  self.contact_twist_cache = twist_cache_out
  let mut motion_clamped = false
  if ccd_enabled {
    let ccd_active = update_ccd_active_flags_3d_real(
      dt, bodies, colliders, false,
    )
    if ccd_active {
      clamp_fast_ccd_body_motions_3d_real(dt, bodies, colliders, true)
      motion_clamped = true
    }
  }
  if motion_clamping_enabled && !motion_clamped {
    clamp_fast_ccd_body_motions_3d_real(dt, bodies, colliders, false)
  }
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
  if multibody_joints is Some(multibody) {
    multibody.sync_rigid_bodies_from_multibodies(bodies)
    applied_constraints = true
  }
  if applied_constraints {
    colliders.sync_with_bodies(bodies)
  }

  // Collision events (Started/Stopped) are based on the end-of-step collision state.
  // - sensor pairs: from intersection pairs, flagged with CollisionEventFlags::sensor().
  // - contact pairs: from contact pairs, flagged with CollisionEventFlags::empty().
  if events is Some(handler) {
    // Update the collision state after integrating positions so queries and sensor events reflect
    // the end-of-step configuration.
    broad_phase.update(prediction_distance, colliders)
    let pairs = filter_broad_phase_pairs_3d(
      broad_phase.pairs(),
      bodies,
      colliders,
      joints,
      multibody_joints,
      hooks,
    )
    narrow_phase.update(pairs, bodies, colliders)
    restore_contact_pair_impulses_from_cache(narrow_phase, self.contact_cache)
    let next_sensor_pairs = collect_sensor_pairs(narrow_phase, colliders)
    let next_intersection_pairs = collect_intersection_pairs(
      narrow_phase, colliders,
    )
    let next_contact_pairs = collect_contact_pairs(narrow_phase, colliders)
    let prev_sensor_pairs = self.sensor_pairs.copy()
    emit_collision_events_for_pair_diffs(
      handler,
      colliders,
      prev_sensor_pairs,
      next_sensor_pairs,
      @collision.CollisionEventFlags::sensor(),
    )
    let prev_intersection_pairs = self.intersection_pairs.copy()
    emit_intersection_events_for_pair_diffs(
      handler, prev_intersection_pairs, next_intersection_pairs,
    )
    let prev_contact_pairs = self.contact_pairs.copy()
    emit_collision_events_for_pair_diffs(
      handler,
      colliders,
      prev_contact_pairs,
      next_contact_pairs,
      @collision.CollisionEventFlags::empty(),
    )
    self.sensor_pairs.clear()
    self.sensor_pairs.append(next_sensor_pairs[:])
    self.intersection_pairs.clear()
    self.intersection_pairs.append(next_intersection_pairs[:])
    self.contact_pairs.clear()
    self.contact_pairs.append(next_contact_pairs[:])
  }
  bodies.update_sleep_all(dt, parameters.length_unit)
  let interactions : Array[
    (@dynamics.RigidBodyHandle, @dynamics.RigidBodyHandle),
  ] = []
  let interaction_keys : @hashmap.HashMap[(Int, Int, Int, Int), Bool] = @hashmap.new()
  fn push_unique_interaction(
    interactions : Array[(@dynamics.RigidBodyHandle, @dynamics.RigidBodyHandle)],
    interaction_keys : @hashmap.HashMap[(Int, Int, Int, Int), Bool],
    body1 : @dynamics.RigidBodyHandle,
    body2 : @dynamics.RigidBodyHandle,
  ) -> Unit {
    let key = rb_pair_key_3d(body1, body2)
    if interaction_keys.get(key) is Some(true) {
      return
    }
    interaction_keys.set(key, true)
    interactions.push((body1, body2))
  }

  let pairs = narrow_phase.all_contact_pairs()
  for i in 0..<pairs.length() {
    let ((c1h, c2h), pair) = pairs[i]
    if pair.manifolds_len() <= 0 {
      continue
    }
    if colliders.get(c1h) is Some(c1) &&
      colliders.get(c2h) is Some(c2) &&
      c1.parent() is Some(b1) &&
      c2.parent() is Some(b2) {
      push_unique_interaction(interactions, interaction_keys, b1, b2)
    }
  }
  if rope_joints is Some(rope) {
    for pair in rope.interaction_pairs() {
      push_unique_interaction(interactions, interaction_keys, pair.0, pair.1)
    }
  }
  if joints is Some(generic) {
    for pair in generic.interaction_pairs() {
      push_unique_interaction(interactions, interaction_keys, pair.0, pair.1)
    }
  }
  if multibody_joints is Some(multibody) {
    for pair in multibody.interaction_pairs() {
      push_unique_interaction(interactions, interaction_keys, pair.0, pair.1)
    }
  }
  islands.update(bodies, interactions)
}
