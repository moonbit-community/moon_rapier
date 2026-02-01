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
/// Minimal real 3D ray-cast vehicle controller.
///
/// This is an incremental port of Rapier's `DynamicRayCastVehicleController` for the 3DReal backend.
/// It currently supports:
/// - Wheel suspension raycasts (against QueryPipeline3DReal)
/// - Engine/brake forces along the chassis forward direction
/// - Steering by rotating the forward direction for front wheels
pub struct Wheel3DReal {
  connection_point_cs : @core.Vec3
  direction_cs : @core.Vec3
  axle_cs : @core.Vec3
  suspension_rest_length : @core.Real
  radius : @core.Real
  tuning : WheelTuning
  mut engine_force : @core.Real
  mut brake : @core.Real
  mut steering : @core.Real

  // State updated each step.
  mut is_in_contact : Bool
  mut contact_point_ws : @core.Vec3
  mut contact_normal_ws : @core.Vec3
  mut suspension_length : @core.Real
}

///|
pub struct DynamicRayCastVehicleController3DReal {
  chassis : @dynamics.RigidBodyHandle
  wheels : Array[Wheel3DReal]
}

///|
pub fn DynamicRayCastVehicleController3DReal::new(
  chassis : @dynamics.RigidBodyHandle,
) -> DynamicRayCastVehicleController3DReal {
  { chassis, wheels: [] }
}

///|
pub fn DynamicRayCastVehicleController3DReal::add_wheel(
  self : DynamicRayCastVehicleController3DReal,
  connection_point_cs : @core.Vec3,
  direction_cs : @core.Vec3,
  axle_cs : @core.Vec3,
  suspension_rest_length : @core.Real,
  radius : @core.Real,
  tuning : WheelTuning,
) -> Int {
  let w = {
    connection_point_cs,
    direction_cs,
    axle_cs,
    suspension_rest_length,
    radius,
    tuning,
    engine_force: 0.0F,
    brake: 0.0F,
    steering: 0.0F,
    is_in_contact: false,
    contact_point_ws: @core.Vec3::zero(),
    contact_normal_ws: @core.Vec3::new(0.0F, 1.0F, 0.0F),
    suspension_length: suspension_rest_length,
  }
  self.wheels.push(w)
  self.wheels.length() - 1
}

///|
pub fn DynamicRayCastVehicleController3DReal::set_engine_force(
  self : DynamicRayCastVehicleController3DReal,
  wheel : Int,
  value : @core.Real,
) -> Unit {
  if wheel >= 0 && wheel < self.wheels.length() {
    self.wheels[wheel].engine_force = value
  }
}

///|
pub fn DynamicRayCastVehicleController3DReal::set_brake(
  self : DynamicRayCastVehicleController3DReal,
  wheel : Int,
  value : @core.Real,
) -> Unit {
  if wheel >= 0 && wheel < self.wheels.length() {
    self.wheels[wheel].brake = value
  }
}

///|
pub fn DynamicRayCastVehicleController3DReal::set_steering_value(
  self : DynamicRayCastVehicleController3DReal,
  wheel : Int,
  value : @core.Real,
) -> Unit {
  if wheel >= 0 && wheel < self.wheels.length() {
    self.wheels[wheel].steering = value
  }
}

///|
pub fn DynamicRayCastVehicleController3DReal::wheels(
  self : DynamicRayCastVehicleController3DReal,
) -> Array[Wheel3DReal] {
  self.wheels
}

///|
fn normalized_or_default(v : @core.Vec3) -> @core.Vec3 {
  let n2 = v.length_squared()
  if n2 <= 1.0e-12F {
    @core.Vec3::new(1.0F, 0.0F, 0.0F)
  } else {
    v.scale(1.0F / Float::sqrt(n2))
  }
}

///|
pub fn DynamicRayCastVehicleController3DReal::update_vehicle(
  self : DynamicRayCastVehicleController3DReal,
  dt : @core.Real,
  queries : @collision.QueryPipelineMut3DReal,
) -> Unit {
  queries.update()
  let qp = queries.as_ref()
  let chassis = match qp.rigid_body_mut(self.chassis) {
    Some(rb) => rb
    None => return
  }
  if !chassis.body_type().is_dynamic() {
    return
  }
  let chassis_pos = chassis.position()
  let chassis_rot = chassis.rotation()
  let fwd = normalized_or_default(
    chassis_rot.rotate_vec3(@core.Vec3::new(1.0F, 0.0F, 0.0F)),
  )
  let up = normalized_or_default(
    chassis_rot.rotate_vec3(@core.Vec3::new(0.0F, 1.0F, 0.0F)),
  )

  // Suspension ray casts and suspension impulses.
  for i in 0..<self.wheels.length() {
    let w = self.wheels[i]
    let hard_point_ws = chassis_pos.transform_point(w.connection_point_cs)
    let dir_ws = normalized_or_default(chassis_rot.rotate_vec3(w.direction_cs))
    let ray_len = w.suspension_rest_length + w.radius
    let ray = @collision.Ray3::new(hard_point_ws, dir_ws)
    let filter = @collision.QueryFilter3DReal::new().exclude_rigid_body(
      self.chassis,
    )
    let qp2 = qp.with_filter(filter)
    if qp2.cast_ray_and_get_normal(ray, ray_len, true) is Some((_h, hit)) {
      let toi = hit.toi()
      let contact_point = hard_point_ws.add(dir_ws.scale(toi))
      let normal = hit.normal()
      w.is_in_contact = true
      w.contact_point_ws = contact_point
      w.contact_normal_ws = normal
      let susp_len = toi - w.radius
      w.suspension_length = if susp_len < 0.0F { 0.0F } else { susp_len }

      // Simple spring-damper along the contact normal.
      let err = w.suspension_rest_length - w.suspension_length
      let k = w.tuning.suspension_stiffness
      let c = w.tuning.suspension_damping
      let rel_v = chassis.linvel().dot(normal)
      let force = k * err - c * rel_v
      if force > 0.0F {
        chassis.apply_impulse_at_point(
          normal.scale(force * dt),
          contact_point,
          true,
        )
      }
    } else {
      w.is_in_contact = false
      w.contact_point_ws = hard_point_ws.add(dir_ws.scale(ray_len))
      w.contact_normal_ws = up
      w.suspension_length = w.suspension_rest_length
    }
    self.wheels[i] = w
  }

  // Engine/brake impulses.
  for i in 0..<self.wheels.length() {
    let w = self.wheels[i]
    if !w.is_in_contact {
      continue
    }
    let steer = w.steering
    // Steering by rotating the forward direction around chassis up axis.
    let (sinv, cosv) = (
      @math.sin(steer.to_double()),
      @math.cos(steer.to_double()),
    )
    let sinv = Float::from_double(sinv)
    let cosv = Float::from_double(cosv)
    let fwd2 = normalized_or_default(
      fwd.scale(cosv).add(up.cross(fwd).scale(sinv)),
    )
    if w.engine_force != 0.0F {
      chassis.apply_impulse_at_point(
        fwd2.scale(w.engine_force * dt),
        w.contact_point_ws,
        true,
      )
    }
    if w.brake != 0.0F {
      // Oppose motion along forward axis.
      let v = chassis.linvel().dot(fwd2)
      let imp = fwd2.scale(-v * w.brake * dt)
      chassis.apply_impulse_at_point(imp, w.contact_point_ws, true)
    }
  }
}
