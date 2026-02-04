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
  bodies : RigidBodySet,
  impacts : PredictedImpacts,
) -> Unit {
  self |> ignore
  if impacts is PredictedImpacts::Impacts(tois) {
    for i in 0..<tois.length() {
      let pair = tois[i]
      let handle = pair.0
      let toi = pair.1
      if bodies.get_mut_internal(handle) is Some(body) {
        let ccd = body.ccd
        let max_vel = ccd.max_point_velocity(body.vels)
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
        let local_com = body.mass_props.local_mprops.center_of_mass
        body.position.next_position = body.vels.integrate(
          effective_toi,
          body.position.position,
          local_com,
        )
      }
    }
  }
}

///|
pub fn CCDSolver::update_ccd_active_flags(
  self : CCDSolver,
  islands : IslandManager,
  bodies : RigidBodySet,
  dt : @core.Real,
  include_forces : Bool,
) -> Bool {
  self |> ignore
  let mut any_active = false
  for handle in islands.active_bodies() {
    if bodies.get_mut_internal(handle) is Some(body) {
      let ccd = body.ccd
      let active = if ccd.ccd_enabled {
        let forces = if include_forces { Some(body.forces) } else { None }
        ccd.is_moving_fast(dt, body.vels, forces)
      } else {
        false
      }
      let updated = RigidBodyCcd::{
        ccd_thickness: ccd.ccd_thickness,
        ccd_max_dist: ccd.ccd_max_dist,
        ccd_active: active,
        ccd_enabled: ccd.ccd_enabled,
        soft_ccd_prediction: ccd.soft_ccd_prediction,
      }
      body.ccd = updated
      if active {
        any_active = true
      }
    }
  }
  any_active
}

///|
/// Predicts potential impacts if all bodies moved to their `next_position`.
///
/// This port currently does not perform CCD shape-casts against colliders here, so it returns
/// `NoImpacts`. It is still exposed for pub parity.
pub fn CCDSolver::predict_impacts_at_next_positions(
  self : CCDSolver,
  _bodies : RigidBodySet,
  _dt : @core.Real,
) -> PredictedImpacts {
  self |> ignore
  PredictedImpacts::NoImpacts
}

///|
/// Find the first time-of-impact among the predicted impacts.
///
/// This port currently does not compute CCD impacts, so this always returns `None`.
pub fn CCDSolver::find_first_impact(
  self : CCDSolver,
  impacts : PredictedImpacts,
) -> (RigidBodyHandle, @core.Real)? {
  self |> ignore
  match impacts {
    PredictedImpacts::Impacts(tois) =>
      if tois.length() == 0 {
        None
      } else {
        // Keep the earliest toi.
        let mut best = tois[0]
        for i in 1..<tois.length() {
          let cand = tois[i]
          if cand.1 < best.1 {
            best = cand
          }
        }
        Some(best)
      }
    _ => None
  }
}
