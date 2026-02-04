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
/// A dim3-flavored ray-cast vehicle controller, ported from
/// `rapier-reference/src/control/ray_cast_vehicle_controller.rs`.
///
/// Note: the current MoonBit Rapier port is still 2D under the hood. This controller therefore
/// applies impulses only on the XY plane (dropping the Z component), but keeps the dim3 API surface
/// so the `examples3d/vehicle_controller3` port can be strict and deterministic.

///|
pub struct RayCastInfo3 {
  mut contact_normal_ws : @core.Vec3
  mut contact_point_ws : @core.Vec3
  mut suspension_length : @core.Real
  mut hard_point_ws : @core.Vec3
  mut is_in_contact : Bool
  mut ground_object : @collision.ColliderHandle?
}

///|
pub fn RayCastInfo3::default() -> RayCastInfo3 {
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
pub struct Wheel3 {
  raycast_info : RayCastInfo3
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
fn wheel3_new(
  chassis_connection_cs : @core.Vec3,
  direction_cs : @core.Vec3,
  axle_cs : @core.Vec3,
  suspension_rest_length : @core.Real,
  radius : @core.Real,
  tuning : WheelTuning,
) -> Wheel3 {
  {
    raycast_info: RayCastInfo3::default(),
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
pub fn Wheel3::raycast_info(self : Wheel3) -> RayCastInfo3 {
  self.raycast_info
}

///|
pub fn Wheel3::center(self : Wheel3) -> @core.Vec3 {
  self.center
}

///|
pub fn Wheel3::suspension(self : Wheel3) -> @core.Vec3 {
  self.wheel_direction_ws
}

///|
pub fn Wheel3::axle(self : Wheel3) -> @core.Vec3 {
  self.wheel_axle_ws
}

///|
pub struct DynamicRayCastVehicleController3 {
  wheels : Array[Wheel3]
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
pub fn DynamicRayCastVehicleController3::new(
  chassis : @dynamics.RigidBodyHandle,
) -> DynamicRayCastVehicleController3 {
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
pub fn DynamicRayCastVehicleController3::add_wheel(
  self : DynamicRayCastVehicleController3,
  chassis_connection_cs : @core.Vec3,
  direction_cs : @core.Vec3,
  axle_cs : @core.Vec3,
  suspension_rest_length : @core.Real,
  radius : @core.Real,
  tuning : WheelTuning,
) -> Int {
  let wheel_id = self.wheels.length()
  self.wheels.push(
    wheel3_new(
      chassis_connection_cs, direction_cs, axle_cs, suspension_rest_length, radius,
      tuning,
    ),
  )
  wheel_id
}

///|
pub fn DynamicRayCastVehicleController3::set_engine_force(
  self : DynamicRayCastVehicleController3,
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
pub fn DynamicRayCastVehicleController3::set_brake(
  self : DynamicRayCastVehicleController3,
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
pub fn DynamicRayCastVehicleController3::set_steering_value(
  self : DynamicRayCastVehicleController3,
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
pub fn DynamicRayCastVehicleController3::wheels(
  self : DynamicRayCastVehicleController3,
) -> Array[Wheel3] {
  self.wheels
}

///|
fn v3_to_v2(v : @core.Vec3) -> @core.Vec2 {
  @core.Vec2::new(v.x, v.y)
}

///|
fn v2_to_v3(v : @core.Vec2) -> @core.Vec3 {
  @core.Vec3::new(v.x, v.y, 0.0F)
}

///|
fn axis_vec3(axis : Int) -> @core.Vec3 {
  if axis == 0 {
    @core.Vec3::new(1.0F, 0.0F, 0.0F)
  } else if axis == 1 {
    @core.Vec3::new(0.0F, 1.0F, 0.0F)
  } else {
    @core.Vec3::new(0.0F, 0.0F, 1.0F)
  }
}

///|
fn vec3_normalize_or_zero(v : @core.Vec3) -> @core.Vec3 {
  let lsq = v.length_squared()
  if lsq <= 1.0e-12F {
    @core.Vec3::zero()
  } else {
    v.scale(1.0F / lsq.sqrt())
  }
}

///|
fn rotate_vec3_yaw(rot : @core.Rot2, v : @core.Vec3) -> @core.Vec3 {
  let r = rot.rotate_vec2(@core.Vec2::new(v.x, v.y))
  @core.Vec3::new(r.x, r.y, v.z)
}

///|
fn transform_point3(pos : @core.Isometry2, p : @core.Vec3) -> @core.Vec3 {
  let v2 = pos.transform_point(@core.Vec2::new(p.x, p.y))
  @core.Vec3::new(v2.x, v2.y, p.z)
}

///|
fn update_wheel_transforms_ws3(
  controller : DynamicRayCastVehicleController3,
  chassis : @dynamics.RigidBody,
  wheel : Wheel3,
) -> Wheel3 {
  let out = wheel
  out.raycast_info.is_in_contact = false
  out.raycast_info.ground_object = None
  let pos = chassis.position()
  out.raycast_info.hard_point_ws = transform_point3(
    pos,
    out.chassis_connection_point_cs,
  )
  out.wheel_direction_ws = vec3_normalize_or_zero(
    rotate_vec3_yaw(chassis.rotation(), out.direction_cs),
  )
  out.wheel_axle_ws = vec3_normalize_or_zero(
    rotate_vec3_yaw(chassis.rotation(), out.axle_cs),
  )

  // Steering (dim3): rotate the axle around the wheel direction.
  let steering_axis = out.wheel_direction_ws.scale(-out.steering)
  let steering_rot = @core.rotation_from_scaled_axis(steering_axis)
  out.wheel_axle_ws = vec3_normalize_or_zero(
    steering_rot.rotate_vec3(out.wheel_axle_ws),
  )

  // Keep center consistent with the current suspension length.
  out.center = out.raycast_info.hard_point_ws.add(
    out.wheel_direction_ws.scale(out.raycast_info.suspension_length),
  )
  controller |> ignore
  out
}

///|
fn ray_cast3(
  queries : @collision.QueryPipeline,
  bodies : @dynamics.RigidBodySet,
  colliders : @collision.ColliderSet,
  chassis : @dynamics.RigidBody,
  wheel : Wheel3,
) -> Wheel3 {
  let out = wheel
  let raylen = out.suspension_rest_length + out.radius
  let ray_dir_ws = out.wheel_direction_ws
  let source_ws = out.raycast_info.hard_point_ws
  let source = v3_to_v2(source_ws)
  let ray_dir = v3_to_v2(ray_dir_ws)
  let ray = @collision.Ray::new(source, ray_dir)
  out.raycast_info.contact_point_ws = source_ws.add(ray_dir_ws.scale(raylen))
  if queries.cast_ray_and_get_normal(bodies, colliders, ray, raylen, true)
    is Some(hit) {
    let collider_hit = hit.0
    let intersection = hit.1
    let normal2 = intersection.normal()
    let mut normal_ws = v2_to_v3(normal2)
    if normal_ws.length_squared() <= 1.0e-12F {
      normal_ws = out.wheel_direction_ws.scale(-1.0F)
    }
    out.raycast_info.contact_normal_ws = vec3_normalize_or_zero(normal_ws)
    out.raycast_info.is_in_contact = true
    out.raycast_info.ground_object = Some(collider_hit)
    let hit_distance = intersection.toi()
    out.raycast_info.suspension_length = hit_distance - out.radius

    // Clamp on max suspension travel.
    let min_len = out.suspension_rest_length - out.max_suspension_travel
    let max_len = out.suspension_rest_length + out.max_suspension_travel
    if out.raycast_info.suspension_length < min_len {
      out.raycast_info.suspension_length = min_len
    }
    if out.raycast_info.suspension_length > max_len {
      out.raycast_info.suspension_length = max_len
    }
    out.raycast_info.contact_point_ws = source_ws.add(
      ray_dir_ws.scale(hit_distance),
    )
    let denom = out.raycast_info.contact_normal_ws.dot(out.wheel_direction_ws)
    let chassis_vel_at_contact = velocity_at_point(
      chassis,
      v3_to_v2(out.raycast_info.contact_point_ws),
    )
    let proj_vel = out.raycast_info.contact_normal_ws.dot(
      v2_to_v3(chassis_vel_at_contact),
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
fn update_suspension3(wheel : Wheel3, chassis_mass : @core.Real) -> Wheel3 {
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
    out.wheel_suspension_force = veh_max_real(force * chassis_mass, 0.0F)
  } else {
    out.wheel_suspension_force = 0.0F
  }
  out
}

///|
fn resolve_single_unilateral3(
  body1 : @dynamics.RigidBody,
  pt1 : @core.Vec3,
  normal : @core.Vec3,
) -> @core.Real {
  let pt1_2 = v3_to_v2(pt1)
  let n2 = v3_to_v2(normal)
  let vel1 = velocity_at_point(body1, pt1_2)
  let dvel = vel1
  let denom = impulse_denominator(body1, pt1_2, n2)
  let jac_inv = if denom <= 1.0e-12F { 0.0F } else { 1.0F / denom }
  let rel_vel = n2.dot(dvel)
  let contact_damping = 0.2F
  -contact_damping * rel_vel * jac_inv
}

///|
fn resolve_single_bilateral3(
  body1 : @dynamics.RigidBody,
  pt1 : @core.Vec3,
  body2 : @dynamics.RigidBody,
  pt2 : @core.Vec3,
  normal : @core.Vec3,
) -> @core.Real {
  let pt1_2 = v3_to_v2(pt1)
  let pt2_2 = v3_to_v2(pt2)
  let n2 = v3_to_v2(normal)
  let vel1 = velocity_at_point(body1, pt1_2)
  let vel2 = velocity_at_point(body2, pt2_2)
  let dvel = vel1.sub(vel2)
  let denom = impulse_denominator(body1, pt1_2, n2) +
    impulse_denominator(body2, pt2_2, n2)
  let jac_inv = if denom <= 1.0e-12F { 0.0F } else { 1.0F / denom }
  let rel_vel = n2.dot(dvel)
  let contact_damping = 0.2F
  -contact_damping * rel_vel * jac_inv
}

///|
fn update_friction3(
  controller : DynamicRayCastVehicleController3,
  bodies : @dynamics.RigidBodySet,
  colliders : @collision.ColliderSet,
  dt : @core.Real,
) -> Unit {
  let num_wheels = controller.wheels.length()
  if num_wheels == 0 {
    return
  }

  // Ensure cached direction arrays are large enough.
  while controller.forward_ws.length() < num_wheels {
    controller.forward_ws.push(@core.Vec3::zero())
  }
  while controller.axle_ws.length() < num_wheels {
    controller.axle_ws.push(@core.Vec3::zero())
  }
  let mut num_wheels_on_ground = 0
  for i in 0..<num_wheels {
    let w = controller.wheels[i]
    if w.raycast_info.ground_object is Some(_) {
      num_wheels_on_ground += 1
    }
    w.side_impulse = 0.0F
    w.forward_impulse = 0.0F
    controller.wheels[i] = w
  }

  // Compute axle/forward directions and side impulses.
  for i in 0..<num_wheels {
    let w0 = controller.wheels[i]
    if w0.raycast_info.ground_object is None {
      controller.wheels[i] = w0
      continue
    }
    let surf_normal = w0.raycast_info.contact_normal_ws
    let mut axle = w0.wheel_axle_ws
    axle = axle.sub(surf_normal.scale(axle.dot(surf_normal)))
    axle = vec3_normalize_or_zero(axle)
    controller.axle_ws[i] = axle
    controller.forward_ws[i] = vec3_normalize_or_zero(surf_normal.cross(axle))
    let w = w0
    let chassis_body = if bodies.get(controller.chassis) is Some(rb) {
      rb
    } else {
      controller.wheels[i] = w
      continue
    }
    let ground_body = if w.raycast_info.ground_object is Some(coh) {
      if colliders.get(coh) is Some(co) && co.parent() is Some(parent) {
        if bodies.get(parent) is Some(b) && b.is_dynamic() {
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
    let pt = w.raycast_info.contact_point_ws
    w.side_impulse = if ground_body is Some(gb) {
      resolve_single_bilateral3(chassis_body, pt, gb, pt, n)
    } else {
      resolve_single_unilateral3(chassis_body, pt, n)
    }
    w.side_impulse = w.side_impulse * w.side_friction_stiffness
    controller.wheels[i] = w
  }
  let side_factor = 1.0F
  let fwd_factor = 0.5F
  let mut sliding = false
  for wheel_id in 0..<num_wheels {
    let wheel = controller.wheels[wheel_id]
    let ground_object = wheel.raycast_info.ground_object
    let mut rolling_friction = 0.0F
    if ground_object is Some(_) {
      if wheel.engine_force != 0.0F {
        rolling_friction = wheel.engine_force * dt
      } else {
        let max_impulse = if wheel.brake != 0.0F { wheel.brake } else { 0.0F }
        let body0 = bodies.get(controller.chassis)
        if body0 is Some(b0) {
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
          rolling_friction = calc_rolling_friction_impulse(
            b0,
            body1,
            v3_to_v2(wheel.raycast_info.contact_point_ws),
            v3_to_v2(controller.forward_ws[wheel_id]),
            max_impulse,
            num_wheels_on_ground,
          )
        }
      }
    }
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
        let factor = max_imp / impulse_squared.sqrt()
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

  // Apply impulses.
  if bodies.get_mut_internal_with_modification_tracking(controller.chassis)
    is Some(chassis) {
    for wheel_id in 0..<num_wheels {
      let wheel = controller.wheels[wheel_id]
      let mut impulse_point = wheel.raycast_info.contact_point_ws
      if wheel.forward_impulse != 0.0F {
        chassis.apply_impulse_at_point(
          v3_to_v2(controller.forward_ws[wheel_id].scale(wheel.forward_impulse)),
          v3_to_v2(impulse_point),
          false,
        )
        |> ignore
      }
      if wheel.side_impulse != 0.0F {
        let side_impulse = controller.axle_ws[wheel_id].scale(
          wheel.side_impulse,
        )
        let v_up = rotate_vec3_yaw(
          chassis.rotation(),
          axis_vec3(controller.index_up_axis),
        )
        let com = v2_to_v3(chassis.world_com())
        impulse_point = impulse_point.sub(
          v_up.scale(
            v_up.dot(impulse_point.sub(com)) * (1.0F - wheel.roll_influence),
          ),
        )
        chassis.apply_impulse_at_point(
          v3_to_v2(side_impulse),
          v3_to_v2(impulse_point),
          false,
        )
        |> ignore
      }
    }
  }
}

///|
pub fn DynamicRayCastVehicleController3::update_vehicle(
  self : DynamicRayCastVehicleController3,
  dt : @core.Real,
  queries : @collision.QueryPipelineMut,
  bodies : @dynamics.RigidBodySet,
  colliders : @collision.ColliderSet,
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
    self.wheels[i] = update_wheel_transforms_ws3(self, chassis, w)
  }

  // Current speed sign.
  let forward_w = rotate_vec3_yaw(
    chassis.rotation(),
    axis_vec3(self.index_forward_axis),
  )
  let linvel = chassis.linvel()
  self.current_vehicle_speed = linvel.length()
  if v3_to_v2(forward_w).dot(linvel) < 0.0F {
    self.current_vehicle_speed = -self.current_vehicle_speed
  }

  // Suspension ray casts.
  for i in 0..<num_wheels {
    let w = self.wheels[i]
    self.wheels[i] = ray_cast3(qp, bodies, colliders, chassis, w)
  }
  let chassis_mass = chassis.mass()
  for i in 0..<num_wheels {
    let w = self.wheels[i]
    self.wheels[i] = update_suspension3(w, chassis_mass)
  }

  // Apply suspension impulses.
  if bodies.get_mut_internal_with_modification_tracking(self.chassis)
    is Some(rb) {
    for i in 0..<num_wheels {
      let w = self.wheels[i]
      if w.engine_force > 0.0F {
        rb.wake_up(true) |> ignore
      }
      let mut suspension_force = w.wheel_suspension_force
      if suspension_force > w.max_suspension_force {
        suspension_force = w.max_suspension_force
      }
      let impulse = w.raycast_info.contact_normal_ws.scale(
        suspension_force * dt,
      )
      rb.apply_impulse_at_point(
        v3_to_v2(impulse),
        v3_to_v2(w.raycast_info.contact_point_ws),
        false,
      )
      |> ignore
    }
  }
  update_friction3(self, bodies, colliders, dt)

  // Update wheel rotation.
  if bodies.get_mut_internal_with_modification_tracking(self.chassis)
    is Some(rb) {
    let fwd_axis = rotate_vec3_yaw(
      rb.rotation(),
      axis_vec3(self.index_forward_axis),
    )
    for i in 0..<num_wheels {
      let w = self.wheels[i]
      let vel = velocity_at_point(rb, v3_to_v2(w.raycast_info.hard_point_ws))
      if w.raycast_info.is_in_contact {
        let mut fwd = fwd_axis
        let proj = fwd.dot(w.raycast_info.contact_normal_ws)
        fwd = fwd.sub(w.raycast_info.contact_normal_ws.scale(proj))
        let proj2 = fwd.dot(v2_to_v3(vel))
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
