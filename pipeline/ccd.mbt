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

///|
fn find_first_ccd_impact(
  gravity : @core.Vec2,
  dt : @core.Real,
  islands : @dynamics.IslandManager,
  bodies : @dynamics.RigidBodySet,
  colliders : @collision.ColliderSet,
) -> @core.Real? {
  if dt <= 0.0F {
    return None
  }
  let active = islands.active_bodies()
  let mut min_toi = dt
  let mut found = false
  for i in 0..<active.length() {
    let handle = active[i]
    if bodies.get(handle) is Some(body) && body.is_ccd_active() {
      let predicted_body_pos = match body.body_type() {
        @dynamics.RigidBodyType::Dynamic => {
          let scale = body.gravity_scale()
          let gravity_step = @core.Vec2::new(
            gravity.x * dt * scale,
            gravity.y * dt * scale,
          )
          let lin_scale = 1.0F / (1.0F + dt * body.linear_damping())
          let ang_scale = 1.0F / (1.0F + dt * body.angular_damping())
          let linvel = body.linvel().add(gravity_step)
          let linvel = @core.Vec2::new(
            linvel.x * lin_scale,
            linvel.y * lin_scale,
          )
          let angvel = body.angvel() * ang_scale
          let curr_pos = body.position()
          let com = curr_pos.transform_point(body.center_of_mass())
          let shift = @core.Isometry2::from_translation(
            @core.Vec2::new(-com.x, -com.y),
          )
          let rot = @core.Isometry2::new(
            @core.Vec2::zero(),
            @core.Rot2::from_angle(angvel * dt),
          )
          let translation = @core.Vec2::new(linvel.x * dt, linvel.y * dt)
          let unshift = @core.Isometry2::from_translation(com.add(translation))
          unshift.mul(rot).mul(shift).mul(curr_pos)
        }
        @dynamics.RigidBodyType::KinematicVelocityBased =>
          body.vels().integrate(dt, body.position(), body.center_of_mass())
        _ => body.next_position()
      }
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
          if pipeline.cast_shape_nonlinear(
              motion,
              co.shape,
              0.0F,
              min_toi,
              true,
            )
            is Some((_, hit)) {
            let toi = hit.toi()
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
