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
fn shape_ccd_thickness(shape : @collision.Shape) -> @core.Real {
  match shape {
    @collision.Shape::Ball(radius) => radius
    @collision.Shape::Cuboid(hw, hh) => if hw < hh { hw } else { hh }
    @collision.Shape::CapsuleX(_, radius) => radius
    @collision.Shape::CapsuleY(_, radius) => radius
    @collision.Shape::Segment(_, _) => 0.0F
  }
}

///|
fn shape_bounding_radius(shape : @collision.Shape) -> @core.Real {
  match shape {
    @collision.Shape::Ball(radius) => radius
    @collision.Shape::Cuboid(hw, hh) => Float::sqrt(hw * hw + hh * hh)
    @collision.Shape::CapsuleX(half_height, radius)
    | @collision.Shape::CapsuleY(half_height, radius) => half_height + radius
    @collision.Shape::Segment(a, b) => {
      let la = a.length()
      let lb = b.length()
      if la > lb {
        la
      } else {
        lb
      }
    }
  }
}

///|
fn predict_ccd_impacts_at_next_positions(
  dt : @core.Real,
  islands : @dynamics.IslandManager,
  bodies : @dynamics.RigidBodySet,
  colliders : @collision.ColliderSet,
) -> @dynamics.PredictedImpacts {
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
          let linvel = end_pos.translation.sub(start_pos.translation)
          let vel = @core.Vec2::new(linvel.x / dt, linvel.y / dt)
          let angvel = (end_pos.rotation.angle() - start_pos.rotation.angle()) /
            dt
          let motion = @collision.NonlinearRigidMotion::new(
            start_pos, vel, angvel,
          )
          let filter = @collision.QueryFilter::new()
            .exclude_rigid_body(handle)
            .groups(co.collision_groups())
            .exclude_sensors()
          let pipeline = @collision.QueryPipeline::new(
            filter, bodies, colliders,
          )
          if pipeline.cast_shape_nonlinear(motion, co.shape, 0.0F, dt, true)
            is Some((_, hit)) {
            let toi = hit.toi()
            best_toi = match best_toi {
              None => Some(toi)
              Some(best) => Some(if toi < best { toi } else { best })
            }
          }
        }
      }
      if best_toi is Some(t) && t <= dt {
        impacts.push((handle, t))
      }
    }
  }
  if impacts.length() == 0 {
    @dynamics.PredictedImpacts::NoImpacts
  } else {
    @dynamics.PredictedImpacts::Impacts(impacts)
  }
}
