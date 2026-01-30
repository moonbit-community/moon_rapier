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
/// A 2D ray-cast vehicle controller inspired by Rapier's `DynamicRayCastVehicleController`.
///
/// Notes:
/// - Upstream Rapier implements this controller only for `dim3`.
/// - This port provides a deterministic 2D **side-scroller**-style controller:
///   suspension + traction/braking along the contact tangent.
/// - Lateral axle slip (a 3D-only concept) is intentionally not simulated.

///|
pub struct WheelTuning {
  suspension_stiffness : @core.Real
  suspension_compression : @core.Real
  suspension_damping : @core.Real
  max_suspension_travel : @core.Real
  side_friction_stiffness : @core.Real
  friction_slip : @core.Real
  max_suspension_force : @core.Real
}

///|
pub fn WheelTuning::default() -> WheelTuning {
  {
    suspension_stiffness: 5.88F,
    suspension_compression: 0.83F,
    suspension_damping: 0.88F,
    max_suspension_travel: 5.0F,
    side_friction_stiffness: 1.0F,
    friction_slip: 10.5F,
    max_suspension_force: 6000.0F,
  }
}

///|
pub fn WheelTuning::with_suspension_stiffness(
  self : WheelTuning,
  suspension_stiffness : @core.Real,
) -> WheelTuning {
  {
    suspension_stiffness,
    suspension_compression: self.suspension_compression,
    suspension_damping: self.suspension_damping,
    max_suspension_travel: self.max_suspension_travel,
    side_friction_stiffness: self.side_friction_stiffness,
    friction_slip: self.friction_slip,
    max_suspension_force: self.max_suspension_force,
  }
}

///|
pub fn WheelTuning::with_suspension_compression(
  self : WheelTuning,
  suspension_compression : @core.Real,
) -> WheelTuning {
  {
    suspension_stiffness: self.suspension_stiffness,
    suspension_compression,
    suspension_damping: self.suspension_damping,
    max_suspension_travel: self.max_suspension_travel,
    side_friction_stiffness: self.side_friction_stiffness,
    friction_slip: self.friction_slip,
    max_suspension_force: self.max_suspension_force,
  }
}

///|
pub fn WheelTuning::with_suspension_damping(
  self : WheelTuning,
  suspension_damping : @core.Real,
) -> WheelTuning {
  {
    suspension_stiffness: self.suspension_stiffness,
    suspension_compression: self.suspension_compression,
    suspension_damping,
    max_suspension_travel: self.max_suspension_travel,
    side_friction_stiffness: self.side_friction_stiffness,
    friction_slip: self.friction_slip,
    max_suspension_force: self.max_suspension_force,
  }
}

///|
pub fn WheelTuning::with_max_suspension_travel(
  self : WheelTuning,
  max_suspension_travel : @core.Real,
) -> WheelTuning {
  {
    suspension_stiffness: self.suspension_stiffness,
    suspension_compression: self.suspension_compression,
    suspension_damping: self.suspension_damping,
    max_suspension_travel,
    side_friction_stiffness: self.side_friction_stiffness,
    friction_slip: self.friction_slip,
    max_suspension_force: self.max_suspension_force,
  }
}

///|
pub fn WheelTuning::with_side_friction_stiffness(
  self : WheelTuning,
  side_friction_stiffness : @core.Real,
) -> WheelTuning {
  {
    suspension_stiffness: self.suspension_stiffness,
    suspension_compression: self.suspension_compression,
    suspension_damping: self.suspension_damping,
    max_suspension_travel: self.max_suspension_travel,
    side_friction_stiffness,
    friction_slip: self.friction_slip,
    max_suspension_force: self.max_suspension_force,
  }
}

///|
pub fn WheelTuning::with_friction_slip(
  self : WheelTuning,
  friction_slip : @core.Real,
) -> WheelTuning {
  {
    suspension_stiffness: self.suspension_stiffness,
    suspension_compression: self.suspension_compression,
    suspension_damping: self.suspension_damping,
    max_suspension_travel: self.max_suspension_travel,
    side_friction_stiffness: self.side_friction_stiffness,
    friction_slip,
    max_suspension_force: self.max_suspension_force,
  }
}

///|
pub fn WheelTuning::with_max_suspension_force(
  self : WheelTuning,
  max_suspension_force : @core.Real,
) -> WheelTuning {
  {
    suspension_stiffness: self.suspension_stiffness,
    suspension_compression: self.suspension_compression,
    suspension_damping: self.suspension_damping,
    max_suspension_travel: self.max_suspension_travel,
    side_friction_stiffness: self.side_friction_stiffness,
    friction_slip: self.friction_slip,
    max_suspension_force,
  }
}

///|
pub struct RayCastInfo {
  mut contact_normal_ws : @core.Vec2
  mut contact_point_ws : @core.Vec2
  mut suspension_length : @core.Real
  mut hard_point_ws : @core.Vec2
  mut is_in_contact : Bool
  mut ground_object : @collision.ColliderHandle?
}

///|
pub fn RayCastInfo::default() -> RayCastInfo {
  {
    contact_normal_ws: @core.Vec2::zero(),
    contact_point_ws: @core.Vec2::zero(),
    suspension_length: 0.0F,
    hard_point_ws: @core.Vec2::zero(),
    is_in_contact: false,
    ground_object: None,
  }
}

///|
pub struct Wheel {
  raycast_info : RayCastInfo
  mut center : @core.Vec2
  mut wheel_direction_ws : @core.Vec2

  // Inputs / configuration.
  chassis_connection_point_cs : @core.Vec2
  direction_cs : @core.Vec2
  suspension_rest_length : @core.Real
  max_suspension_travel : @core.Real
  radius : @core.Real
  suspension_stiffness : @core.Real
  damping_compression : @core.Real
  damping_relaxation : @core.Real
  friction_slip : @core.Real
  max_suspension_force : @core.Real

  // State.
  mut rotation : @core.Real
  mut delta_rotation : @core.Real
  mut clipped_inv_contact_dot_suspension : @core.Real
  mut suspension_relative_velocity : @core.Real
  mut wheel_suspension_force : @core.Real

  // Outputs.
  mut forward_impulse : @core.Real

  // Control.
  mut engine_force : @core.Real
  mut brake : @core.Real
}

///|
fn wheel_new(
  chassis_connection_cs : @core.Vec2,
  direction_cs : @core.Vec2,
  suspension_rest_length : @core.Real,
  radius : @core.Real,
  tuning : WheelTuning,
) -> Wheel {
  {
    raycast_info: RayCastInfo::default(),
    center: @core.Vec2::zero(),
    wheel_direction_ws: direction_cs,
    chassis_connection_point_cs: chassis_connection_cs,
    direction_cs,
    suspension_rest_length,
    max_suspension_travel: tuning.max_suspension_travel,
    radius,
    suspension_stiffness: tuning.suspension_stiffness,
    damping_compression: tuning.suspension_compression,
    damping_relaxation: tuning.suspension_damping,
    friction_slip: tuning.friction_slip,
    max_suspension_force: tuning.max_suspension_force,
    rotation: 0.0F,
    delta_rotation: 0.0F,
    clipped_inv_contact_dot_suspension: 0.0F,
    suspension_relative_velocity: 0.0F,
    wheel_suspension_force: 0.0F,
    forward_impulse: 0.0F,
    engine_force: 0.0F,
    brake: 0.0F,
  }
}

///|
pub fn Wheel::raycast_info(self : Wheel) -> RayCastInfo {
  self.raycast_info
}

///|
pub fn Wheel::center(self : Wheel) -> @core.Vec2 {
  self.center
}

///|
pub fn Wheel::suspension(self : Wheel) -> @core.Vec2 {
  self.wheel_direction_ws
}

///|
pub struct DynamicRayCastVehicleController {
  wheels : Array[Wheel]
  chassis : @dynamics.RigidBodyHandle
  index_up_axis : Int
  index_forward_axis : Int
}

///|
pub fn DynamicRayCastVehicleController::new(
  chassis : @dynamics.RigidBodyHandle,
) -> DynamicRayCastVehicleController {
  { wheels: [], chassis, index_up_axis: 1, index_forward_axis: 0 }
}

///|
pub fn DynamicRayCastVehicleController::add_wheel(
  self : DynamicRayCastVehicleController,
  chassis_connection_cs : @core.Vec2,
  direction_cs : @core.Vec2,
  suspension_rest_length : @core.Real,
  radius : @core.Real,
  tuning : WheelTuning,
) -> Unit {
  self.wheels.push(
    wheel_new(
      chassis_connection_cs, direction_cs, suspension_rest_length, radius, tuning,
    ),
  )
}

///|
pub fn DynamicRayCastVehicleController::set_engine_force(
  self : DynamicRayCastVehicleController,
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
pub fn DynamicRayCastVehicleController::set_brake(
  self : DynamicRayCastVehicleController,
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
fn axis_vec2(axis : Int) -> @core.Vec2 {
  if axis == 0 {
    @core.Vec2::new(1.0F, 0.0F)
  } else {
    @core.Vec2::new(0.0F, 1.0F)
  }
}

///|
fn veh_vec2_scale(v : @core.Vec2, s : @core.Real) -> @core.Vec2 {
  @core.Vec2::new(v.x * s, v.y * s)
}

///|
fn veh_vec2_neg(v : @core.Vec2) -> @core.Vec2 {
  @core.Vec2::new(-v.x, -v.y)
}

///|
fn veh_normalize_or_zero(v : @core.Vec2) -> @core.Vec2 {
  let lsq = v.length_squared()
  if lsq <= 1.0e-12F {
    @core.Vec2::zero()
  } else {
    veh_vec2_scale(v, 1.0F / lsq.sqrt())
  }
}

///|
fn velocity_at_point(
  rb : @dynamics.RigidBody,
  world_pt : @core.Vec2,
) -> @core.Vec2 {
  let linvel = rb.linvel()
  let w = rb.angvel()
  let com = rb.world_com()
  let r = world_pt.sub(com)
  // 2D: w x r = (-w * r.y, w * r.x)
  let ang = @core.Vec2::new(-w * r.y, w * r.x)
  linvel.add(ang)
}

///|
fn effective_inv_mass_dir(
  mprops : @dynamics.RigidBodyMassProps,
  dir : @core.Vec2,
) -> @core.Real {
  // effective_inv_mass is diagonal (per-axis) for locked axes, so:
  // inv_mass(dir) = dir.x^2 * inv_mass_x + dir.y^2 * inv_mass_y
  let invm = mprops.effective_inv_mass
  dir.x * dir.x * invm.x + dir.y * dir.y * invm.y
}

///|
fn impulse_denominator(
  rb : @dynamics.RigidBody,
  pos : @core.Vec2,
  dir : @core.Vec2,
) -> @core.Real {
  let mprops = rb.mass_props()
  let inv_mass_dir = effective_inv_mass_dir(mprops, dir)
  let com = rb.world_com()
  let r = pos.sub(com)
  let rcross = r.cross(dir)
  let inv_inertia = mprops.effective_world_inv_inertia
  inv_mass_dir + rcross * rcross * inv_inertia
}

///|
fn update_wheel_transforms_ws(
  chassis : @dynamics.RigidBody,
  wheel : Wheel,
) -> Wheel {
  let out = wheel
  out.raycast_info.is_in_contact = false
  out.raycast_info.ground_object = None
  let pos = chassis.position()
  out.raycast_info.hard_point_ws = pos.transform_point(
    out.chassis_connection_point_cs,
  )
  out.wheel_direction_ws = veh_normalize_or_zero(
    chassis.rotation().rotate_vec2(out.direction_cs),
  )
  out
}

///|
fn ray_cast(
  queries : @collision.QueryPipeline,
  chassis : @dynamics.RigidBody,
  wheel : Wheel,
) -> Wheel {
  let out = wheel
  let raylen = out.suspension_rest_length + out.radius
  let ray_dir = out.wheel_direction_ws
  let source = out.raycast_info.hard_point_ws
  let ray = @collision.Ray::new(source, ray_dir)
  out.raycast_info.contact_point_ws = source.add(
    veh_vec2_scale(ray_dir, raylen),
  )
  if queries.cast_ray_and_get_normal(ray, raylen, true) is Some(hit) {
    let collider_hit = hit.0
    let intersection = hit.1
    out.raycast_info.contact_normal_ws = intersection.normal()
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
    out.raycast_info.contact_point_ws = source.add(
      veh_vec2_scale(ray_dir, hit_distance),
    )
    let denom = out.raycast_info.contact_normal_ws.dot(out.wheel_direction_ws)
    let chassis_vel_at_contact = velocity_at_point(
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
    out.raycast_info.contact_normal_ws = veh_vec2_neg(out.wheel_direction_ws)
    out.clipped_inv_contact_dot_suspension = 1.0F
  }
  out.center = out.raycast_info.hard_point_ws.add(
    veh_vec2_scale(out.wheel_direction_ws, out.raycast_info.suspension_length),
  )
  out
}

///|
fn update_suspension(wheel : Wheel, chassis_mass : @core.Real) -> Wheel {
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
fn veh_max_real(a : @core.Real, b : @core.Real) -> @core.Real {
  if a > b {
    a
  } else {
    b
  }
}

///|
fn veh_min_real(a : @core.Real, b : @core.Real) -> @core.Real {
  if a < b {
    a
  } else {
    b
  }
}

///|
fn clamp_real(
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
fn calc_rolling_friction_impulse(
  body0 : @dynamics.RigidBody,
  body1 : @dynamics.RigidBody?,
  contact_pos : @core.Vec2,
  dir : @core.Vec2,
  max_impulse : @core.Real,
  num_wheels_on_ground : Int,
) -> @core.Real {
  let denom0 = impulse_denominator(body0, contact_pos, dir)
  let denom1 = if body1 is Some(b1) {
    impulse_denominator(b1, contact_pos, dir)
  } else {
    0.0F
  }
  let jac_inv = if denom0 + denom1 <= 1.0e-12F {
    0.0F
  } else {
    1.0F / (denom0 + denom1)
  }
  let v0 = velocity_at_point(body0, contact_pos)
  let v1 = if body1 is Some(b1) {
    velocity_at_point(b1, contact_pos)
  } else {
    @core.Vec2::zero()
  }
  let vrel = dir.dot(v0.sub(v1))
  let denom_wheels = if num_wheels_on_ground <= 0 {
    1.0F
  } else {
    Float::from_double(num_wheels_on_ground.to_double())
  }
  clamp_real(-vrel * jac_inv / denom_wheels, -max_impulse, max_impulse)
}

///|
/// Updates wheel contacts and applies suspension/traction impulses to the chassis.
///
/// Expected usage:
/// - Build/update a `QueryPipelineMut` with a filter excluding the chassis' colliders.
/// - Call this *before* the physics pipeline step so the impulses affect the solver.
pub fn DynamicRayCastVehicleController::update_vehicle(
  self : DynamicRayCastVehicleController,
  dt : @core.Real,
  queries : @collision.QueryPipelineMut,
) -> Unit {
  queries.update()
  let qp = queries.as_ref()
  let chassis = match qp.rigid_body(self.chassis) {
    Some(rb) => rb
    None => return
  }

  // Wheel transforms + ray casts.
  for i in 0..<self.wheels.length() {
    let mut w = self.wheels[i]
    w = update_wheel_transforms_ws(chassis, w)
    w = ray_cast(qp, chassis, w)
    self.wheels[i] = w
  }
  let chassis_mass = chassis.mass()
  for i in 0..<self.wheels.length() {
    let mut w = self.wheels[i]
    w = update_suspension(w, chassis_mass)
    self.wheels[i] = w
  }

  // Apply suspension impulses.
  // Need a mutable body for impulses.
  if qp.bodies.get_mut_internal_with_modification_tracking(self.chassis)
    is Some(rb) {
    for i in 0..<self.wheels.length() {
      let w = self.wheels[i]
      if w.engine_force > 0.0F {
        rb.wake_up(true) |> ignore
      }
      let mut suspension_force = w.wheel_suspension_force
      suspension_force = veh_min_real(suspension_force, w.max_suspension_force)
      let impulse = veh_vec2_scale(
        w.raycast_info.contact_normal_ws,
        suspension_force * dt,
      )
      rb.apply_impulse_at_point(impulse, w.raycast_info.contact_point_ws, false)
      |> ignore
    }
  }

  // Traction / braking.
  let mut wheels_on_ground = 0
  for i in 0..<self.wheels.length() {
    if self.wheels[i].raycast_info.is_in_contact {
      wheels_on_ground = wheels_on_ground + 1
    }
    let w = self.wheels[i]
    w.forward_impulse = 0.0F
    self.wheels[i] = w
  }
  if wheels_on_ground == 0 {
    return
  }
  if qp.bodies.get_mut_internal_with_modification_tracking(self.chassis)
    is Some(rb) {
    let fwd_axis_ws = rb
      .rotation()
      .rotate_vec2(axis_vec2(self.index_forward_axis))
    for i in 0..<self.wheels.length() {
      let w = self.wheels[i]
      if !w.raycast_info.is_in_contact {
        self.wheels[i] = w
        continue
      }
      let normal = w.raycast_info.contact_normal_ws
      // Tangent direction along the surface.
      let mut fwd = fwd_axis_ws.sub(
        veh_vec2_scale(normal, fwd_axis_ws.dot(normal)),
      )
      fwd = veh_normalize_or_zero(fwd)
      if fwd.length_squared() <= 1.0e-12F {
        // Fallback: pick any tangent.
        fwd = veh_normalize_or_zero(@core.Vec2::new(-normal.y, normal.x))
      }
      // Choose tangent sign to align with the chassis forward axis.
      if fwd.dot(fwd_axis_ws) < 0.0F {
        fwd = veh_vec2_neg(fwd)
      }
      let mut rolling = 0.0F
      if w.engine_force != 0.0F {
        rolling = w.engine_force * dt
      } else {
        let max_impulse = if w.brake != 0.0F { w.brake } else { 0.0F }
        let mut ground_body : @dynamics.RigidBody? = None
        if w.raycast_info.ground_object is Some(h) {
          if qp.collider(h) is Some(c) {
            if c.parent() is Some(parent) {
              ground_body = qp.rigid_body(parent)
            }
          }
        }
        rolling = calc_rolling_friction_impulse(
          rb,
          ground_body,
          w.raycast_info.contact_point_ws,
          fwd,
          max_impulse,
          wheels_on_ground,
        )
      }

      // Clamp traction to available friction.
      let max_imp = w.wheel_suspension_force * dt * w.friction_slip
      rolling = clamp_real(rolling, -max_imp, max_imp)
      w.forward_impulse = rolling
      if rolling != 0.0F {
        rb.apply_impulse_at_point(
          veh_vec2_scale(fwd, rolling),
          w.raycast_info.contact_point_ws,
          false,
        )
        |> ignore
      }
      self.wheels[i] = w
    }
  }

  // Wheel rotation update.
  if qp.rigid_body(self.chassis) is Some(rb2) {
    let fwd_axis_ws = rb2
      .rotation()
      .rotate_vec2(axis_vec2(self.index_forward_axis))
    for i in 0..<self.wheels.length() {
      let w = self.wheels[i]
      let vel = velocity_at_point(rb2, w.raycast_info.hard_point_ws)
      let mut fwd = fwd_axis_ws
      if w.raycast_info.is_in_contact {
        let normal = w.raycast_info.contact_normal_ws
        fwd = veh_normalize_or_zero(
          fwd.sub(veh_vec2_scale(normal, fwd.dot(normal))),
        )
        if fwd.length_squared() <= 1.0e-12F {
          fwd = veh_normalize_or_zero(@core.Vec2::new(-normal.y, normal.x))
        }
        if fwd.dot(fwd_axis_ws) < 0.0F {
          fwd = veh_vec2_neg(fwd)
        }
      } else {
        fwd = veh_normalize_or_zero(fwd)
      }
      let proj = fwd.dot(vel)
      w.delta_rotation = if w.radius <= 1.0e-6F {
        0.0F
      } else {
        proj * dt / w.radius
      }
      w.rotation = w.rotation + w.delta_rotation
      w.delta_rotation = w.delta_rotation * 0.99F
      self.wheels[i] = w
    }
  }
}
