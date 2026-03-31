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
pub struct CCDSolver {}

///|
pub fn CCDSolver::new() -> CCDSolver {
  CCDSolver::{  }
}

///|
pub fn CCDSolver::clamp_motions(
  self : CCDSolver,
  dt : @core.Real,
  bodies : @dynamics.RigidBodySet,
  impacts : @dynamics.PredictedImpacts,
) -> Unit {
  self |> ignore
  if impacts is @dynamics.PredictedImpacts::Impacts(tois) {
    for i in 0..<tois.length() {
      let pair = tois[i]
      let handle = pair.0
      let toi = pair.1
      if bodies.get_mut_internal(handle) is Some(body) {
        let ccd = body.ccd()
        let max_vel = ccd.max_point_velocity(body.vels())
        let min_toi = if max_vel > 0.0F {
          let value = ccd.ccd_thickness * 0.15F / max_vel
          if value < dt {
            value
          } else {
            dt
          }
        } else {
          dt
        }
        let effective_toi = if toi > min_toi { toi } else { min_toi }
        let local_com = body.mass_props().local_mprops.center_of_mass
        let new_pos = body
          .vels()
          .integrate(effective_toi, body.position(), local_com)
        body.set_next_position_internal(new_pos) |> ignore
      }
    }
  }
}

///|
pub fn CCDSolver::clamp_motions3(
  self : CCDSolver,
  dt : @core.Real,
  bodies : @dynamics.RigidBodySet3,
  impacts : @dynamics.PredictedImpacts,
) -> Unit {
  self.clamp_motions(dt, bodies.inner, impacts)
}

///|
pub fn CCDSolver::update_ccd_active_flags(
  self : CCDSolver,
  islands : @dynamics.IslandManager,
  bodies : @dynamics.RigidBodySet,
  dt : @core.Real,
  include_forces : Bool,
) -> Bool {
  self |> ignore
  let mut any_active = false
  for handle in islands.active_bodies() {
    if bodies.get_mut_internal(handle) is Some(body) {
      let ccd = body.ccd()
      let active = if ccd.ccd_enabled {
        let forces = if include_forces { Some(body.forces()) } else { None }
        ccd.is_moving_fast(dt, body.vels(), forces)
      } else {
        false
      }
      body.set_ccd_active(active) |> ignore
      if active {
        any_active = true
      }
    }
  }
  any_active
}

///|
pub fn CCDSolver::update_ccd_active_flags3(
  self : CCDSolver,
  islands : @dynamics.IslandManager3,
  bodies : @dynamics.RigidBodySet3,
  dt : @core.Real,
  include_forces : Bool,
) -> Bool {
  self.update_ccd_active_flags(islands.inner, bodies.inner, dt, include_forces)
}

///|
fn predicted_body_position_for_first_impact(
  body : @dynamics.RigidBody,
  dt : @core.Real,
) -> @core.Isometry2 {
  match body.body_type() {
    @dynamics.RigidBodyType::Dynamic =>
      @dynamics.RigidBodyPosition::new(body.position(), body.next_position()).integrate_forces_and_velocities(
        dt,
        body.forces(),
        body.vels(),
        body.mass_props(),
      )
    @dynamics.RigidBodyType::KinematicVelocityBased =>
      body.vels().integrate(dt, body.position(), body.center_of_mass())
    _ => body.next_position()
  }
}

///|
fn predict_body_collider_toi(
  query_pipeline : @collision.QueryPipeline,
  collider : @collision.Collider,
  start_pos : @core.Isometry2,
  end_pos : @core.Isometry2,
  dt : @core.Real,
  max_toi : @core.Real,
  bodies : @dynamics.RigidBodySet,
  colliders : @collision.ColliderSet,
) -> @core.Real? {
  if dt <= 0.0F {
    return None
  }
  let linvel = end_pos.translation.sub(start_pos.translation)
  let vel = @core.Vec2::new(linvel.x / dt, linvel.y / dt)
  let angvel = (end_pos.rotation.angle() - start_pos.rotation.angle()) / dt
  let motion = @collision.NonlinearRigidMotion::new(start_pos, vel, angvel)
  if query_pipeline.cast_shape_nonlinear(
      bodies,
      colliders,
      motion,
      collider.shape(),
      0.0F,
      max_toi,
      true,
    )
    is Some((_, hit)) {
    Some(hit.toi())
  } else {
    None
  }
}

///|
pub fn CCDSolver::predict_impacts_at_next_positions(
  self : CCDSolver,
  dt : @core.Real,
  islands : @dynamics.IslandManager,
  bodies : @dynamics.RigidBodySet,
  colliders : @collision.ColliderSet,
  broad_phase : @collision.BroadPhaseBvh,
  narrow_phase : @collision.NarrowPhase,
) -> @dynamics.PredictedImpacts {
  self |> ignore
  narrow_phase |> ignore
  if dt <= 0.0F {
    return @dynamics.PredictedImpacts::NoImpacts
  }
  let impacts : Array[(@dynamics.RigidBodyHandle, @core.Real)] = []
  let active = islands.active_bodies()
  for i in 0..<active.length() {
    let handle = active[i]
    if bodies.get(handle) is Some(body) && body.is_ccd_active() {
      let attached = colliders.colliders_with_parent(handle)
      let mut best_toi : @core.Real? = None
      for j in 0..<attached.length() {
        let co_handle = attached[j]
        if colliders.get(co_handle) is Some(co) &&
          co.is_enabled() &&
          !co.is_sensor() {
          let start_pos = co.position()
          let local_pos = @core.Isometry2::new(
            co.local_translation,
            @core.Rot2::from_angle(co.local_rotation),
          )
          let end_pos = body.next_position().mul(local_pos)
          let filter = @collision.QueryFilter::new()
            .exclude_rigid_body(handle)
            .groups(co.collision_groups())
            .exclude_sensors()
          let query_pipeline = broad_phase.as_query_pipeline(
            bodies, colliders, filter,
          )
          if predict_body_collider_toi(
              query_pipeline, co, start_pos, end_pos, dt, dt, bodies, colliders,
            )
            is Some(toi) {
            best_toi = match best_toi {
              None => Some(toi)
              Some(best) => Some(if toi < best { toi } else { best })
            }
          }
        }
      }
      if best_toi is Some(toi) && toi <= dt {
        impacts.push((handle, toi))
      }
    }
  }
  if impacts.length() == 0 {
    @dynamics.PredictedImpacts::NoImpacts
  } else {
    @dynamics.PredictedImpacts::Impacts(impacts)
  }
}

///|
pub fn CCDSolver::find_first_impact(
  self : CCDSolver,
  dt : @core.Real,
  islands : @dynamics.IslandManager,
  bodies : @dynamics.RigidBodySet,
  colliders : @collision.ColliderSet,
  broad_phase : @collision.BroadPhaseBvh,
  narrow_phase : @collision.NarrowPhase,
) -> @core.Real? {
  self |> ignore
  narrow_phase |> ignore
  if dt <= 0.0F {
    return None
  }
  let active = islands.active_bodies()
  let mut min_toi = dt
  let mut found = false
  for i in 0..<active.length() {
    let handle = active[i]
    if bodies.get(handle) is Some(body) && body.is_ccd_active() {
      let predicted_body_pos = predicted_body_position_for_first_impact(
        body, dt,
      )
      let attached = colliders.colliders_with_parent(handle)
      for j in 0..<attached.length() {
        let co_handle = attached[j]
        if colliders.get(co_handle) is Some(co) &&
          co.is_enabled() &&
          !co.is_sensor() {
          let start_pos = co.position()
          let local_pos = @core.Isometry2::new(
            co.local_translation,
            @core.Rot2::from_angle(co.local_rotation),
          )
          let end_pos = predicted_body_pos.mul(local_pos)
          let filter = @collision.QueryFilter::new()
            .exclude_rigid_body(handle)
            .groups(co.collision_groups())
            .exclude_sensors()
          let query_pipeline = broad_phase.as_query_pipeline(
            bodies, colliders, filter,
          )
          if predict_body_collider_toi(
              query_pipeline, co, start_pos, end_pos, dt, min_toi, bodies, colliders,
            )
            is Some(toi) {
            if toi < min_toi {
              min_toi = toi
              found = true
            }
          }
        }
      }
    }
  }
  if found {
    Some(min_toi)
  } else {
    None
  }
}

///|
pub fn CCDSolver::predict_impacts_at_next_positions3(
  self : CCDSolver,
  dt : @core.Real,
  islands : @dynamics.IslandManager3,
  bodies : @dynamics.RigidBodySet3,
  colliders : @collision.ColliderSet3,
  broad_phase : @collision.BroadPhaseBvh,
  narrow_phase : @collision.NarrowPhase,
) -> @dynamics.PredictedImpacts {
  self.predict_impacts_at_next_positions(
    dt,
    islands.inner,
    bodies.inner,
    colliders.inner,
    broad_phase,
    narrow_phase,
  )
}

///|
pub fn CCDSolver::find_first_impact3(
  self : CCDSolver,
  dt : @core.Real,
  islands : @dynamics.IslandManager3,
  bodies : @dynamics.RigidBodySet3,
  colliders : @collision.ColliderSet3,
  broad_phase : @collision.BroadPhaseBvh,
  narrow_phase : @collision.NarrowPhase,
) -> @core.Real? {
  self.find_first_impact(
    dt,
    islands.inner,
    bodies.inner,
    colliders.inner,
    broad_phase,
    narrow_phase,
  )
}
