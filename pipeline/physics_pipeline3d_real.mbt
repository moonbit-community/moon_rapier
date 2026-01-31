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
) -> Unit {
  let dt = parameters.dt
  bodies.apply_gravity_all(gravity, dt)
  bodies.apply_damping_all(dt)

  // Sync colliders to body motion before detecting contacts.
  colliders.sync_with_bodies(bodies)
  let prediction_distance = 0.0F
  broad_phase.update(prediction_distance, colliders)
  narrow_phase.update(broad_phase.pairs(), bodies, colliders)

  // Sensor collision events (Started/Stopped) are based on the contact pairs.
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
  let cache_out = ContactSolverCache3D::new()
  solve_contacts_3d(
    parameters,
    bodies,
    colliders,
    narrow_phase,
    self.contact_cache,
    cache_out,
  )
  self.contact_cache = cache_out
  bodies.advance_positions_all(dt)
  colliders.sync_with_bodies(bodies)
  bodies.update_sleep_all(dt, parameters.length_unit)
  islands.update(bodies)
}
