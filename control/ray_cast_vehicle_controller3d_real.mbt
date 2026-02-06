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
/// Real dim3 ray-cast vehicle controller.
///
/// Ported from `rapier-reference/src/control/ray_cast_vehicle_controller.rs` (v0.32.0, `dim3`).
/// This controller uses ray-casts for suspension contacts and applies impulses on the chassis.
pub struct RayCastInfo3DReal {
  mut contact_normal_ws : @core.Vec3
  mut contact_point_ws : @core.Vec3
  mut suspension_length : @core.Real
  mut hard_point_ws : @core.Vec3
  mut is_in_contact : Bool
  mut ground_object : @collision.ColliderHandle3D?
}

///|
pub fn RayCastInfo3DReal::default() -> RayCastInfo3DReal {
  {
    contact_normal_ws: @core.Vec3::zero(),
    contact_point_ws: @core.Vec3::zero(),
    suspension_length: 0.0F,
    hard_point_ws: @core.Vec3::zero(),
    is_in_contact: false,
    ground_object: None,
  }
}

///|
pub struct Wheel3DReal {
  raycast_info : RayCastInfo3DReal
  mut center : @core.Vec3
  mut wheel_direction_ws : @core.Vec3
  mut wheel_axle_ws : @core.Vec3

  // Inputs / configuration.
  chassis_connection_point_cs : @core.Vec3
  direction_cs : @core.Vec3
  axle_cs : @core.Vec3
  suspension_rest_length : @core.Real
  max_suspension_travel : @core.Real
  radius : @core.Real
  suspension_stiffness : @core.Real
  damping_compression : @core.Real
  damping_relaxation : @core.Real
  friction_slip : @core.Real
  side_friction_stiffness : @core.Real
  max_suspension_force : @core.Real

  // State.
  mut rotation : @core.Real
  mut delta_rotation : @core.Real
  roll_influence : @core.Real
  mut clipped_inv_contact_dot_suspension : @core.Real
  mut suspension_relative_velocity : @core.Real
  mut wheel_suspension_force : @core.Real
  mut skid_info : @core.Real

  // Outputs.
  mut forward_impulse : @core.Real
  mut side_impulse : @core.Real

  // Control.
  mut steering : @core.Real
  mut engine_force : @core.Real
  mut brake : @core.Real
}

///|
fn wheel3d_real_new(
  chassis_connection_cs : @core.Vec3,
  direction_cs : @core.Vec3,
  axle_cs : @core.Vec3,
  suspension_rest_length : @core.Real,
  radius : @core.Real,
  tuning : WheelTuning,
) -> Wheel3DReal {
  {
    raycast_info: RayCastInfo3DReal::default(),
    center: @core.Vec3::zero(),
    wheel_direction_ws: direction_cs,
    wheel_axle_ws: axle_cs,
    chassis_connection_point_cs: chassis_connection_cs,
    direction_cs,
    axle_cs,
    suspension_rest_length,
    max_suspension_travel: tuning.max_suspension_travel,
    radius,
    suspension_stiffness: tuning.suspension_stiffness,
    damping_compression: tuning.suspension_compression,
    damping_relaxation: tuning.suspension_damping,
    friction_slip: tuning.friction_slip,
    side_friction_stiffness: tuning.side_friction_stiffness,
    max_suspension_force: tuning.max_suspension_force,
    rotation: 0.0F,
    delta_rotation: 0.0F,
    brake: 0.0F,
    steering: 0.0F,
    engine_force: 0.0F,
    roll_influence: 0.1F,
    clipped_inv_contact_dot_suspension: 0.0F,
    suspension_relative_velocity: 0.0F,
    wheel_suspension_force: 0.0F,
    skid_info: 0.0F,
    forward_impulse: 0.0F,
    side_impulse: 0.0F,
  }
}

///|
pub fn Wheel3DReal::raycast_info(self : Wheel3DReal) -> RayCastInfo3DReal {
  self.raycast_info
}

///|
pub fn Wheel3DReal::center(self : Wheel3DReal) -> @core.Vec3 {
  self.center
}

///|
pub fn Wheel3DReal::suspension(self : Wheel3DReal) -> @core.Vec3 {
  self.wheel_direction_ws
}

///|
pub fn Wheel3DReal::axle(self : Wheel3DReal) -> @core.Vec3 {
  self.wheel_axle_ws
}

///|
pub struct DynamicRayCastVehicleController3DReal {
  wheels : Array[Wheel3DReal]
  forward_ws : Array[@core.Vec3]
  axle_ws : Array[@core.Vec3]
  /// The current forward speed of the vehicle.
  mut current_vehicle_speed : @core.Real
  /// Handle of the vehicle chassis.
  chassis : @dynamics.RigidBodyHandle
  /// The chassis local up direction (`0 = x, 1 = y, 2 = z`).
  index_up_axis : Int
  /// The chassis local forward direction (`0 = x, 1 = y, 2 = z`).
  index_forward_axis : Int
}

///|
pub fn DynamicRayCastVehicleController3DReal::new(
  chassis : @dynamics.RigidBodyHandle,
) -> DynamicRayCastVehicleController3DReal {
  {
    wheels: [],
    forward_ws: [],
    axle_ws: [],
    current_vehicle_speed: 0.0F,
    chassis,
    index_up_axis: 1,
    index_forward_axis: 0,
  }
}

///|
pub fn DynamicRayCastVehicleController3DReal::add_wheel(
  self : DynamicRayCastVehicleController3DReal,
  chassis_connection_cs : @core.Vec3,
  direction_cs : @core.Vec3,
  axle_cs : @core.Vec3,
  suspension_rest_length : @core.Real,
  radius : @core.Real,
  tuning : WheelTuning,
) -> Int {
  let wheel_id = self.wheels.length()
  self.wheels.push(
    wheel3d_real_new(
      chassis_connection_cs, direction_cs, axle_cs, suspension_rest_length, radius,
      tuning,
    ),
  )
  wheel_id
}

///|
pub fn DynamicRayCastVehicleController3DReal::set_engine_force(
  self : DynamicRayCastVehicleController3DReal,
  wheel_id : Int,
  force : @core.Real,
) -> Unit {
  if wheel_id < 0 || wheel_id >= self.wheels.length() {
    return
  }
  let wheel = self.wheels[wheel_id]
  wheel.engine_force = force
  self.wheels[wheel_id] = wheel
}

///|
pub fn DynamicRayCastVehicleController3DReal::set_brake(
  self : DynamicRayCastVehicleController3DReal,
  wheel_id : Int,
  brake : @core.Real,
) -> Unit {
  if wheel_id < 0 || wheel_id >= self.wheels.length() {
    return
  }
  let wheel = self.wheels[wheel_id]
  wheel.brake = brake
  self.wheels[wheel_id] = wheel
}

///|
pub fn DynamicRayCastVehicleController3DReal::set_steering_value(
  self : DynamicRayCastVehicleController3DReal,
  wheel_id : Int,
  steering : @core.Real,
) -> Unit {
  if wheel_id < 0 || wheel_id >= self.wheels.length() {
    return
  }
  let wheel = self.wheels[wheel_id]
  wheel.steering = steering
  self.wheels[wheel_id] = wheel
}

///|
pub fn DynamicRayCastVehicleController3DReal::wheels(
  self : DynamicRayCastVehicleController3DReal,
) -> Array[Wheel3DReal] {
  self.wheels
}

///|
pub fn DynamicRayCastVehicleController3DReal::wheels_mut(
  self : DynamicRayCastVehicleController3DReal,
) -> Array[Wheel3DReal] {
  self.wheels
}

///|
fn veh3d_real_axis_vec3(axis : Int) -> @core.Vec3 {
  if axis == 0 {
    @core.Vec3::new(1.0F, 0.0F, 0.0F)
  } else if axis == 1 {
    @core.Vec3::new(0.0F, 1.0F, 0.0F)
  } else {
    @core.Vec3::new(0.0F, 0.0F, 1.0F)
  }
}

///|
fn veh3d_real_normalize_or_zero(v : @core.Vec3) -> @core.Vec3 {
  let lsq = v.length_squared()
  if lsq <= 1.0e-12F {
    @core.Vec3::zero()
  } else {
    v.scale(1.0F / Float::sqrt(lsq))
  }
}

///|
fn veh3d_real_body_mass(body : @dynamics.RigidBody3D) -> @core.Real {
  let inv_m = body.mass_properties().inv_mass
  if inv_m <= 0.0F {
    0.0F
  } else {
    1.0F / inv_m
  }
}

///|
fn veh3d_real_velocity_at_point(
  body : @dynamics.RigidBody3D,
  point : @core.Vec3,
) -> @core.Vec3 {
  let r = point.sub(body.world_com())
  body.linvel().add(body.angvel().cross(r))
}

///|
fn veh3d_real_impulse_denominator(
  body : @dynamics.RigidBody3D,
  point : @core.Vec3,
  dir : @core.Vec3,
) -> @core.Real {
  let inv_m = body.mass_properties().inv_mass
  let r = point.sub(body.world_com())
  let gcross = r.cross(dir)
  let rot = body.rotation().to_mat3()
  let inv_i_local = body.mass_properties().inv_inertia.into_mat3()
  let inv_i_world = rot.mul(inv_i_local).mul(rot.transpose())
  let v = inv_i_world.mul_vec3(gcross).cross(r)
  inv_m + dir.dot(v)
}

///|
fn veh3d_real_clamp_real(
  x : @core.Real,
  minv : @core.Real,
  maxv : @core.Real,
) -> @core.Real {
  if x < minv {
    minv
  } else if x > maxv {
    maxv
  } else {
    x
  }
}

///|
fn veh3d_real_resolve_single_unilateral(
  body1 : @dynamics.RigidBody3D,
  pt1 : @core.Vec3,
  normal : @core.Vec3,
) -> @core.Real {
  let vel1 = veh3d_real_velocity_at_point(body1, pt1)
  let dvel = vel1
  let denom = veh3d_real_impulse_denominator(body1, pt1, normal)
  let jac_inv = if denom <= 1.0e-12F { 0.0F } else { 1.0F / denom }
  let rel_vel = normal.dot(dvel)
  let contact_damping = 0.2F
  -contact_damping * rel_vel * jac_inv
}

///|
fn veh3d_real_resolve_single_bilateral(
  body1 : @dynamics.RigidBody3D,
  pt1 : @core.Vec3,
  body2 : @dynamics.RigidBody3D,
  pt2 : @core.Vec3,
  normal : @core.Vec3,
) -> @core.Real {
  let vel1 = veh3d_real_velocity_at_point(body1, pt1)
  let vel2 = veh3d_real_velocity_at_point(body2, pt2)
  let dvel = vel1.sub(vel2)
  let denom = veh3d_real_impulse_denominator(body1, pt1, normal) +
    veh3d_real_impulse_denominator(body2, pt2, normal)
  let jac_inv = if denom <= 1.0e-12F { 0.0F } else { 1.0F / denom }
  let rel_vel = normal.dot(dvel)
  let contact_damping = 0.2F
  -contact_damping * rel_vel * jac_inv
}

///|
fn veh3d_real_calc_rolling_friction_impulse(
  body0 : @dynamics.RigidBody3D,
  body1 : @dynamics.RigidBody3D?,
  contact_pos : @core.Vec3,
  dir : @core.Vec3,
  max_impulse : @core.Real,
  num_wheels_on_ground : Int,
) -> @core.Real {
  let denom0 = veh3d_real_impulse_denominator(body0, contact_pos, dir)
  let denom1 = if body1 is Some(b1) {
    veh3d_real_impulse_denominator(b1, contact_pos, dir)
  } else {
    0.0F
  }
  let jac_inv = if denom0 + denom1 <= 1.0e-12F {
    0.0F
  } else {
    1.0F / (denom0 + denom1)
  }
  let v0 = veh3d_real_velocity_at_point(body0, contact_pos)
  let v1 = if body1 is Some(b1) {
    veh3d_real_velocity_at_point(b1, contact_pos)
  } else {
    @core.Vec3::zero()
  }
  let vrel = dir.dot(v0.sub(v1))
  let denom_wheels = if num_wheels_on_ground <= 0 {
    1.0F
  } else {
    Float::from_int(num_wheels_on_ground)
  }
  veh3d_real_clamp_real(
    -vrel * jac_inv / denom_wheels,
    -max_impulse,
    max_impulse,
  )
}

///|
fn veh3d_real_update_wheel_transforms_ws(
  controller : DynamicRayCastVehicleController3DReal,
  chassis : @dynamics.RigidBody3D,
  wheel : Wheel3DReal,
) -> Wheel3DReal {
  let out = wheel
  out.raycast_info.is_in_contact = false
  out.raycast_info.ground_object = None
  let pos = chassis.position()
  out.raycast_info.hard_point_ws = pos.transform_point(
    out.chassis_connection_point_cs,
  )
  out.wheel_direction_ws = veh3d_real_normalize_or_zero(
    pos.rotation.rotate_vec3(out.direction_cs),
  )
  out.wheel_axle_ws = veh3d_real_normalize_or_zero(
    pos.rotation.rotate_vec3(out.axle_cs),
  )

  // Steering (dim3): rotate the axle around the wheel direction.
  let steering_rot = @core.rotation_from_scaled_axis(
    out.wheel_direction_ws.scale(-out.steering),
  )
  out.wheel_axle_ws = veh3d_real_normalize_or_zero(
    steering_rot.rotate_vec3(out.wheel_axle_ws),
  )
  out.center = out.raycast_info.hard_point_ws.add(
    out.wheel_direction_ws.scale(out.raycast_info.suspension_length),
  )
  controller |> ignore
  out
}

///|
fn veh3d_real_ray_cast(
  queries : @collision.QueryPipeline3DReal,
  bodies : @dynamics.RigidBodySet3D,
  colliders : @collision.ColliderSet3D,
  chassis : @dynamics.RigidBody3D,
  wheel : Wheel3DReal,
) -> Wheel3DReal {
  let out = wheel
  let raylen = out.suspension_rest_length + out.radius
  let rayvector = out.wheel_direction_ws.scale(raylen)
  let source_ws = out.raycast_info.hard_point_ws
  out.raycast_info.contact_point_ws = source_ws.add(rayvector)
  let ray = @collision.Ray3::new(source_ws, rayvector)
  out.raycast_info.ground_object = None
  if queries.cast_ray_and_get_normal(bodies, colliders, ray, 1.0F, true)
    is Some(hit) {
    let collider_hit = hit.0
    let intersection = hit.1
    let mut normal_ws = intersection.normal()
    if intersection.toi() == 0.0F && normal_ws.length_squared() <= 1.0e-12F {
      normal_ws = out.wheel_direction_ws.scale(-1.0F)
    }
    if normal_ws.length_squared() <= 1.0e-12F {
      normal_ws = out.wheel_direction_ws.scale(-1.0F)
    }
    out.raycast_info.contact_normal_ws = veh3d_real_normalize_or_zero(normal_ws)
    out.raycast_info.is_in_contact = true
    out.raycast_info.ground_object = Some(collider_hit)
    let hit_distance = intersection.toi() * raylen
    out.raycast_info.suspension_length = hit_distance - out.radius

    // Clamp on max suspension travel.
    let min_len = out.suspension_rest_length - out.max_suspension_travel
    let max_len = out.suspension_rest_length + out.max_suspension_travel
    out.raycast_info.suspension_length = veh3d_real_clamp_real(
      out.raycast_info.suspension_length,
      min_len,
      max_len,
    )
    out.raycast_info.contact_point_ws = source_ws.add(
      rayvector.scale(intersection.toi()),
    )
    let denom = out.raycast_info.contact_normal_ws.dot(out.wheel_direction_ws)
    let chassis_vel_at_contact = veh3d_real_velocity_at_point(
      chassis,
      out.raycast_info.contact_point_ws,
    )
    let proj_vel = out.raycast_info.contact_normal_ws.dot(
      chassis_vel_at_contact,
    )
    if denom >= -0.1F {
      out.suspension_relative_velocity = 0.0F
      out.clipped_inv_contact_dot_suspension = 1.0F / 0.1F
    } else {
      let inv = -1.0F / denom
      out.suspension_relative_velocity = proj_vel * inv
      out.clipped_inv_contact_dot_suspension = inv
    }
  } else {
    out.raycast_info.suspension_length = out.suspension_rest_length
    out.suspension_relative_velocity = 0.0F
    out.raycast_info.contact_normal_ws = out.wheel_direction_ws.scale(-1.0F)
    out.clipped_inv_contact_dot_suspension = 1.0F
  }
  out.center = out.raycast_info.hard_point_ws.add(
    out.wheel_direction_ws.scale(out.raycast_info.suspension_length),
  )
  out
}

///|
fn veh3d_real_update_suspension(
  wheel : Wheel3DReal,
  chassis_mass : @core.Real,
) -> Wheel3DReal {
  let out = wheel
  if out.raycast_info.is_in_contact {
    let rest_length = out.suspension_rest_length
    let current_length = out.raycast_info.suspension_length
    let length_diff = rest_length - current_length
    let mut force = out.suspension_stiffness *
      length_diff *
      out.clipped_inv_contact_dot_suspension
    let projected_rel_vel = out.suspension_relative_velocity
    let susp_damping = if projected_rel_vel < 0.0F {
      out.damping_compression
    } else {
      out.damping_relaxation
    }
    force = force - susp_damping * projected_rel_vel
    out.wheel_suspension_force = if force * chassis_mass > 0.0F {
      force * chassis_mass
    } else {
      0.0F
    }
  } else {
    out.wheel_suspension_force = 0.0F
  }
  out
}

///|
fn veh3d_real_update_friction(
  controller : DynamicRayCastVehicleController3DReal,
  bodies : @dynamics.RigidBodySet3D,
  colliders : @collision.ColliderSet3D,
  dt : @core.Real,
) -> Unit {
  let num_wheels = controller.wheels.length()
  if num_wheels == 0 {
    return
  }
  while controller.forward_ws.length() < num_wheels {
    controller.forward_ws.push(@core.Vec3::zero())
  }
  while controller.axle_ws.length() < num_wheels {
    controller.axle_ws.push(@core.Vec3::zero())
  }
  let mut num_wheels_on_ground = 0
  for i in 0..<num_wheels {
    let wheel = controller.wheels[i]
    if wheel.raycast_info.ground_object is Some(_) {
      num_wheels_on_ground = num_wheels_on_ground + 1
    }
    wheel.side_impulse = 0.0F
    wheel.forward_impulse = 0.0F
    controller.wheels[i] = wheel
  }

  // Compute axle/forward directions and side impulses.
  for i in 0..<num_wheels {
    let wheel0 = controller.wheels[i]
    if wheel0.raycast_info.ground_object is None {
      controller.wheels[i] = wheel0
      continue
    }
    let surf_normal = wheel0.raycast_info.contact_normal_ws
    let mut axle = wheel0.wheel_axle_ws
    axle = axle.sub(surf_normal.scale(axle.dot(surf_normal)))
    axle = veh3d_real_normalize_or_zero(axle)
    controller.axle_ws[i] = axle
    controller.forward_ws[i] = veh3d_real_normalize_or_zero(
      surf_normal.cross(axle),
    )
    let wheel = wheel0
    let chassis_body = if bodies.get(controller.chassis) is Some(rb) {
      rb
    } else {
      controller.wheels[i] = wheel
      continue
    }
    let ground_body = if wheel.raycast_info.ground_object is Some(coh) {
      if colliders.get(coh) is Some(co) && co.parent() is Some(parent) {
        if bodies.get(parent) is Some(b) && b.body_type().is_dynamic() {
          Some(b)
        } else {
          None
        }
      } else {
        None
      }
    } else {
      None
    }
    let n = controller.axle_ws[i]
    let pt = wheel.raycast_info.contact_point_ws
    wheel.side_impulse = if ground_body is Some(gb) {
      veh3d_real_resolve_single_bilateral(chassis_body, pt, gb, pt, n)
    } else {
      veh3d_real_resolve_single_unilateral(chassis_body, pt, n)
    }
    wheel.side_impulse = wheel.side_impulse * wheel.side_friction_stiffness
    controller.wheels[i] = wheel
  }
  let side_factor = 1.0F
  let fwd_factor = 0.5F
  let mut sliding = false
  for wheel_id in 0..<num_wheels {
    let wheel0 = controller.wheels[wheel_id]
    let ground_object = wheel0.raycast_info.ground_object
    let mut rolling_friction = 0.0F
    if ground_object is Some(_) {
      if wheel0.engine_force != 0.0F {
        rolling_friction = wheel0.engine_force * dt
      } else {
        let max_impulse = if wheel0.brake != 0.0F { wheel0.brake } else { 0.0F }
        if bodies.get(controller.chassis) is Some(b0) {
          let body1 = if ground_object is Some(h) {
            if colliders.get(h) is Some(co) {
              if co.parent() is Some(parent) {
                bodies.get(parent)
              } else {
                None
              }
            } else {
              None
            }
          } else {
            None
          }
          rolling_friction = veh3d_real_calc_rolling_friction_impulse(
            b0,
            body1,
            wheel0.raycast_info.contact_point_ws,
            controller.forward_ws[wheel_id],
            max_impulse,
            num_wheels_on_ground,
          )
        }
      }
    }
    let wheel = wheel0
    wheel.forward_impulse = 0.0F
    wheel.skid_info = 1.0F
    if ground_object is Some(_) {
      let max_imp = wheel.wheel_suspension_force * dt * wheel.friction_slip
      let max_imp_squared = max_imp * max_imp
      wheel.forward_impulse = rolling_friction
      let x = wheel.forward_impulse * fwd_factor
      let y = wheel.side_impulse * side_factor
      let impulse_squared = x * x + y * y
      if impulse_squared > max_imp_squared && max_imp_squared >= 0.0F {
        sliding = true
        let factor = max_imp / Float::sqrt(impulse_squared)
        wheel.skid_info = wheel.skid_info * factor
      }
    }
    controller.wheels[wheel_id] = wheel
  }
  if sliding {
    for wheel_id in 0..<num_wheels {
      let wheel = controller.wheels[wheel_id]
      if wheel.side_impulse != 0.0F && wheel.skid_info < 1.0F {
        wheel.forward_impulse = wheel.forward_impulse * wheel.skid_info
        wheel.side_impulse = wheel.side_impulse * wheel.skid_info
      }
      controller.wheels[wheel_id] = wheel
    }
  }

  // Apply impulses to the chassis.
  if bodies.get_mut(controller.chassis) is Some(chassis) {
    for wheel_id in 0..<num_wheels {
      let wheel = controller.wheels[wheel_id]
      let mut impulse_point = wheel.raycast_info.contact_point_ws
      if wheel.forward_impulse != 0.0F {
        chassis.apply_impulse_at_point(
          controller.forward_ws[wheel_id].scale(wheel.forward_impulse),
          impulse_point,
          false,
        )
      }
      if wheel.side_impulse != 0.0F {
        let side_impulse = controller.axle_ws[wheel_id].scale(
          wheel.side_impulse,
        )
        let v_up = chassis
          .rotation()
          .rotate_vec3(veh3d_real_axis_vec3(controller.index_up_axis))
        let com = chassis.world_com()
        impulse_point = impulse_point.sub(
          v_up.scale(
            v_up.dot(impulse_point.sub(com)) * (1.0F - wheel.roll_influence),
          ),
        )
        chassis.apply_impulse_at_point(side_impulse, impulse_point, false)
      }
    }
  }
}

///|
pub fn DynamicRayCastVehicleController3DReal::update_vehicle(
  self : DynamicRayCastVehicleController3DReal,
  dt : @core.Real,
  queries : @collision.QueryPipelineMut3DReal,
  bodies : @dynamics.RigidBodySet3D,
  colliders : @collision.ColliderSet3D,
) -> Unit {
  queries.update(bodies, colliders)
  let qp = queries.as_ref()
  let chassis = match qp.rigid_body(bodies, self.chassis) {
    Some(rb) => rb
    None => return
  }
  let num_wheels = self.wheels.length()
  for i in 0..<num_wheels {
    let w = self.wheels[i]
    self.wheels[i] = veh3d_real_update_wheel_transforms_ws(self, chassis, w)
  }

  // Current speed sign.
  let forward_w = chassis
    .rotation()
    .rotate_vec3(veh3d_real_axis_vec3(self.index_forward_axis))
  let linvel = chassis.linvel()
  self.current_vehicle_speed = linvel.length()
  if forward_w.dot(linvel) < 0.0F {
    self.current_vehicle_speed = -self.current_vehicle_speed
  }

  // Suspension ray casts.
  for i in 0..<num_wheels {
    let w = self.wheels[i]
    self.wheels[i] = veh3d_real_ray_cast(
      qp.with_filter(
        @collision.QueryFilter3DReal::new().exclude_rigid_body(self.chassis),
      ),
      bodies,
      colliders,
      chassis,
      w,
    )
  }

  // Update suspension forces.
  let chassis_mass = veh3d_real_body_mass(chassis)
  for i in 0..<num_wheels {
    let w = self.wheels[i]
    self.wheels[i] = veh3d_real_update_suspension(w, chassis_mass)
  }

  // Apply suspension impulses.
  if qp.rigid_body_mut(bodies, self.chassis) is Some(rb) {
    for i in 0..<num_wheels {
      let w = self.wheels[i]
      if w.engine_force > 0.0F {
        rb.wake_up()
      }
      let mut suspension_force = w.wheel_suspension_force
      if suspension_force > w.max_suspension_force {
        suspension_force = w.max_suspension_force
      }
      let impulse = w.raycast_info.contact_normal_ws.scale(
        suspension_force * dt,
      )
      rb.apply_impulse_at_point(impulse, w.raycast_info.contact_point_ws, false)
    }
  }
  veh3d_real_update_friction(self, bodies, colliders, dt)

  // Update wheel rotation.
  if qp.rigid_body_mut(bodies, self.chassis) is Some(rb) {
    let fwd_axis = rb
      .rotation()
      .rotate_vec3(veh3d_real_axis_vec3(self.index_forward_axis))
    for i in 0..<num_wheels {
      let w = self.wheels[i]
      let vel = veh3d_real_velocity_at_point(rb, w.raycast_info.hard_point_ws)
      if w.raycast_info.is_in_contact {
        let mut fwd = fwd_axis
        let proj = fwd.dot(w.raycast_info.contact_normal_ws)
        fwd = fwd.sub(w.raycast_info.contact_normal_ws.scale(proj))
        let proj2 = fwd.dot(vel)
        w.delta_rotation = proj2 * dt / w.radius
        w.rotation = w.rotation + w.delta_rotation
      } else {
        w.rotation = w.rotation + w.delta_rotation
      }
      w.delta_rotation = w.delta_rotation * 0.99F
      self.wheels[i] = w
    }
  }
}
