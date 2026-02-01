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
/// 3D rigid-body velocity: linear and angular.
pub struct RigidBodyVelocity3D {
  linvel : @core.Vec3
  angvel : @core.Vec3
}

///|
pub fn RigidBodyVelocity3D::linvel(self : RigidBodyVelocity3D) -> @core.Vec3 {
  self.linvel
}

///|
pub fn RigidBodyVelocity3D::angvel(self : RigidBodyVelocity3D) -> @core.Vec3 {
  self.angvel
}

///|
pub fn RigidBodyVelocity3D::zero() -> RigidBodyVelocity3D {
  { linvel: @core.Vec3::zero(), angvel: @core.Vec3::zero() }
}

///|
pub fn RigidBodyVelocity3D::apply_damping(
  self : RigidBodyVelocity3D,
  dt : @core.Real,
  linear_damping : @core.Real,
  angular_damping : @core.Real,
) -> RigidBodyVelocity3D {
  let lin_scale = 1.0F / (1.0F + dt * linear_damping)
  let ang_scale = 1.0F / (1.0F + dt * angular_damping)
  { linvel: self.linvel.scale(lin_scale), angvel: self.angvel.scale(ang_scale) }
}

///|
pub fn RigidBodyVelocity3D::integrate(
  self : RigidBodyVelocity3D,
  dt : @core.Real,
  curr_pos : @core.Isometry3,
  local_com : @core.Vec3,
) -> @core.Isometry3 {
  let com = curr_pos.transform_point(local_com)
  let shift = @core.Isometry3::from_translation(
    @core.Vec3::new(-com.x, -com.y, -com.z),
  )
  let rot = @core.Isometry3::new(
    @core.Vec3::zero(),
    @core.rotation_from_scaled_axis(self.angvel.scale(dt)),
  )
  let translation = self.linvel.scale(dt)
  let unshift = @core.Isometry3::from_translation(com.add(translation))
  unshift.mul(rot).mul(shift).mul(curr_pos)
}

///|
/// 3D rigid-body position state with interpolation support.
pub struct RigidBodyPosition3D {
  position : @core.Isometry3
  next_position : @core.Isometry3
}

///|
pub fn RigidBodyPosition3D::new(
  position : @core.Isometry3,
  next_position : @core.Isometry3,
) -> RigidBodyPosition3D {
  { position, next_position }
}

///|
pub fn RigidBodyPosition3D::interpolate_velocity(
  self : RigidBodyPosition3D,
  inv_dt : @core.Real,
  local_com : @core.Vec3,
) -> RigidBodyVelocity3D {
  let curr_com = self.position.transform_point(local_com)
  let next_com = self.next_position.transform_point(local_com)
  let linvel = next_com.sub(curr_com).scale(inv_dt)
  let curr_rot = self.position.rotation
  let next_rot = self.next_position.rotation
  let delta_rot = next_rot.mul(curr_rot.inverse())
  let angvel = delta_rot.to_scaled_axis().scale(inv_dt)
  { linvel, angvel }
}

///|
const DEFAULT_SLEEP_LINEAR_THRESHOLD_3D : @core.Real = 0.4F

///|
const DEFAULT_SLEEP_ANGULAR_THRESHOLD_3D : @core.Real = 0.5F

///|
const DEFAULT_TIME_UNTIL_SLEEP_3D : @core.Real = 2.0F

///|
pub struct RigidBodyActivation3D {
  normalized_linear_threshold : @core.Real
  angular_threshold : @core.Real
  time_until_sleep : @core.Real
  mut time_since_can_sleep : @core.Real
  mut sleeping : Bool
}

///|
pub fn RigidBodyActivation3D::active() -> RigidBodyActivation3D {
  {
    normalized_linear_threshold: DEFAULT_SLEEP_LINEAR_THRESHOLD_3D,
    angular_threshold: DEFAULT_SLEEP_ANGULAR_THRESHOLD_3D,
    time_until_sleep: DEFAULT_TIME_UNTIL_SLEEP_3D,
    time_since_can_sleep: 0.0F,
    sleeping: false,
  }
}

///|
pub fn RigidBodyActivation3D::is_sleeping(self : RigidBodyActivation3D) -> Bool {
  self.sleeping
}

///|
pub fn RigidBodyActivation3D::wake_up(
  self : RigidBodyActivation3D,
) -> RigidBodyActivation3D {
  self.sleeping = false
  self.time_since_can_sleep = 0.0F
  self
}

///|
pub fn RigidBodyActivation3D::update(
  self : RigidBodyActivation3D,
  dt : @core.Real,
  length_unit : @core.Real,
  body_type : RigidBodyType,
  vels : RigidBodyVelocity3D,
) -> RigidBodyActivation3D {
  if self.sleeping {
    return self
  }
  let sq_linvel = vels.linvel().length_squared()
  let sq_angvel = vels.angvel().length_squared()
  let can_sleep = match body_type {
    RigidBodyType::Dynamic => {
      let linear_threshold = self.normalized_linear_threshold * length_unit
      sq_linvel < linear_threshold * linear_threshold &&
      sq_angvel < self.angular_threshold * self.angular_threshold
    }
    RigidBodyType::KinematicPositionBased =>
      sq_linvel == 0.0F && sq_angvel == 0.0F
    RigidBodyType::KinematicVelocityBased =>
      sq_linvel == 0.0F && sq_angvel == 0.0F
    RigidBodyType::Fixed => true
  }
  if can_sleep {
    self.time_since_can_sleep = self.time_since_can_sleep + dt
    if self.time_since_can_sleep >= self.time_until_sleep {
      self.sleeping = true
    }
  } else {
    self.time_since_can_sleep = 0.0F
  }
  self
}

///|
pub struct RigidBody3D {
  body_type : RigidBodyType
  mut position : @core.Isometry3
  // Target pose used by kinematic position-based bodies.
  mut next_position : @core.Isometry3
  mut velocity : RigidBodyVelocity3D
  // Locks/flags (subset of Rapier's LockedAxes API).
  locked_translations : Bool
  locked_rotations : Bool
  rot_x_enabled : Bool
  rot_y_enabled : Bool
  rot_z_enabled : Bool
  // CCD/solver-related flags.
  ccd_enabled : Bool
  soft_ccd_prediction : @core.Real
  gyroscopic_forces_enabled : Bool
  linear_damping : @core.Real
  angular_damping : @core.Real
  mut gravity_scale : @core.Real
  mass_properties : @core.MassProperties3
  mut activation : RigidBodyActivation3D
  can_sleep : Bool
}

///|
pub fn RigidBody3D::translation(self : RigidBody3D) -> @core.Vec3 {
  self.position.translation
}

///|
pub fn RigidBody3D::rotation(self : RigidBody3D) -> @core.Quat {
  self.position.rotation
}

///|
pub fn RigidBody3D::position(self : RigidBody3D) -> @core.Isometry3 {
  self.position
}

///|
pub fn RigidBody3D::next_position(self : RigidBody3D) -> @core.Isometry3 {
  self.next_position
}

///|
pub fn RigidBody3D::linvel(self : RigidBody3D) -> @core.Vec3 {
  self.velocity.linvel
}

///|
pub fn RigidBody3D::angvel(self : RigidBody3D) -> @core.Vec3 {
  self.velocity.angvel
}

///|
pub fn RigidBody3D::set_linvel(self : RigidBody3D, v : @core.Vec3) -> Unit {
  let vv = if self.locked_translations { @core.Vec3::zero() } else { v }
  self.velocity = { linvel: vv, angvel: self.velocity.angvel }
}

///|
pub fn RigidBody3D::set_angvel(self : RigidBody3D, w : @core.Vec3) -> Unit {
  let ww = if self.locked_rotations {
    @core.Vec3::zero()
  } else {
    @core.Vec3::new(
      if self.rot_x_enabled {
        w.x
      } else {
        0.0F
      },
      if self.rot_y_enabled {
        w.y
      } else {
        0.0F
      },
      if self.rot_z_enabled {
        w.z
      } else {
        0.0F
      },
    )
  }
  self.velocity = { linvel: self.velocity.linvel, angvel: ww }
}

///|
pub fn RigidBody3D::body_type(self : RigidBody3D) -> RigidBodyType {
  self.body_type
}

///|
pub fn RigidBody3D::locked_translations(self : RigidBody3D) -> Bool {
  self.locked_translations
}

///|
pub fn RigidBody3D::locked_rotations(self : RigidBody3D) -> Bool {
  self.locked_rotations
}

///|
pub fn RigidBody3D::is_ccd_active(self : RigidBody3D) -> Bool {
  self.ccd_enabled
}

///|
pub fn RigidBody3D::soft_ccd_prediction(self : RigidBody3D) -> @core.Real {
  self.soft_ccd_prediction
}

///|
pub fn RigidBody3D::gyroscopic_forces_enabled(self : RigidBody3D) -> Bool {
  self.gyroscopic_forces_enabled
}

///|
pub fn RigidBody3D::gravity_scale(self : RigidBody3D) -> @core.Real {
  self.gravity_scale
}

///|
pub fn RigidBody3D::set_gravity_scale(
  self : RigidBody3D,
  scale : @core.Real,
  wake_up : Bool,
) -> Unit {
  self.gravity_scale = scale
  if wake_up {
    self.wake_up()
  }
}

///|
pub fn RigidBody3D::mass_properties(
  self : RigidBody3D,
) -> @core.MassProperties3 {
  self.mass_properties
}

///|
pub fn RigidBody3D::world_com(self : RigidBody3D) -> @core.Vec3 {
  self.position.transform_point(self.mass_properties.center_of_mass)
}

///|
pub fn RigidBody3D::is_active(self : RigidBody3D) -> Bool {
  !self.activation.is_sleeping()
}

///|
pub fn RigidBody3D::wake_up(self : RigidBody3D) -> Unit {
  self.activation = self.activation.wake_up()
}

///|
pub fn RigidBody3D::update_sleep(
  self : RigidBody3D,
  dt : @core.Real,
  length_unit : @core.Real,
) -> Unit {
  if !self.can_sleep {
    return
  }
  self.activation = self.activation.update(
    dt,
    length_unit,
    self.body_type,
    self.velocity,
  )
}

///|
pub fn RigidBody3D::set_translation(self : RigidBody3D, t : @core.Vec3) -> Unit {
  self.position = @core.Isometry3::new(t, self.position.rotation)
}

///|
pub fn RigidBody3D::set_rotation(self : RigidBody3D, r : @core.Quat) -> Unit {
  self.position = @core.Isometry3::new(self.position.translation, r)
}

///|
pub fn RigidBody3D::set_next_kinematic_translation(
  self : RigidBody3D,
  t : @core.Vec3,
) -> Unit {
  self.next_position = @core.Isometry3::new(t, self.next_position.rotation)
}

///|
pub fn RigidBody3D::set_next_kinematic_rotation(
  self : RigidBody3D,
  r : @core.Quat,
) -> Unit {
  self.next_position = @core.Isometry3::new(self.next_position.translation, r)
}

///|
pub fn RigidBody3D::update_kinematic_velocities(
  self : RigidBody3D,
  dt : @core.Real,
) -> Unit {
  if self.body_type is RigidBodyType::KinematicPositionBased {
    if dt <= 0.0F {
      self.set_linvel(@core.Vec3::zero())
      self.set_angvel(@core.Vec3::zero())
      return
    }
    let inv_dt = 1.0F / dt
    let linvel = self.next_position.translation
      .sub(self.position.translation)
      .scale(inv_dt)
    let delta_rot = self.next_position.rotation.mul(
      self.position.rotation.inverse(),
    )
    let angvel = delta_rot.to_scaled_axis().scale(inv_dt)
    self.set_linvel(linvel)
    self.set_angvel(angvel)
  }
}

///|
pub fn RigidBody3D::apply_gravity(
  self : RigidBody3D,
  gravity : @core.Vec3,
  dt : @core.Real,
) -> Unit {
  if !self.body_type.is_dynamic() {
    return
  }
  if self.activation.is_sleeping() {
    return
  }
  if self.locked_translations {
    self.set_linvel(@core.Vec3::zero())
    return
  }
  self.velocity = {
    linvel: self.velocity.linvel.add(gravity.scale(self.gravity_scale * dt)),
    angvel: self.velocity.angvel,
  }
}

///|
pub fn RigidBody3D::apply_damping(self : RigidBody3D, dt : @core.Real) -> Unit {
  if !self.body_type.is_dynamic() {
    return
  }
  if self.activation.is_sleeping() {
    return
  }
  self.velocity = self.velocity.apply_damping(
    dt,
    self.linear_damping,
    self.angular_damping,
  )
  // Enforce locks after damping.
  if self.locked_translations {
    self.set_linvel(@core.Vec3::zero())
  }
  self.set_angvel(self.velocity.angvel())
}

///|
pub fn RigidBody3D::apply_gyroscopic_forces(
  self : RigidBody3D,
  dt : @core.Real,
) -> Unit {
  if !self.body_type.is_dynamic() {
    return
  }
  if self.activation.is_sleeping() {
    return
  }
  if !self.gyroscopic_forces_enabled {
    return
  }
  if self.locked_rotations {
    self.set_angvel(@core.Vec3::zero())
    return
  }
  let w_world = self.angvel()
  if w_world.length_squared() <= 1.0e-12F {
    return
  }
  let rot = self.rotation().to_mat3()
  let w_local = rot.transpose().mul_vec3(w_world)
  let inv_i_local = self.mass_properties.inv_inertia.into_mat3()
  let i_local = inv_i_local.inverse()
  // Singular inertia: no meaningful gyroscopic effects.
  if i_local.determinant() == 0.0F {
    return
  }
  let l_local = i_local.mul_vec3(w_local)
  // Euler equation with no external torque: dw/dt = -I^{-1}(w x (I w)).
  let gyro = w_local.cross(l_local)
  let dw_local = inv_i_local.mul_vec3(gyro).scale(-dt)
  let w_local2 = w_local.add(dw_local)
  let w_world2 = rot.mul_vec3(w_local2)
  self.set_angvel(w_world2)
}

///|
pub fn RigidBody3D::advance_position(
  self : RigidBody3D,
  dt : @core.Real,
) -> Unit {
  match self.body_type {
    RigidBodyType::Dynamic => {
      if self.activation.is_sleeping() {
        return
      }
      let linvel = if self.locked_translations {
        @core.Vec3::zero()
      } else {
        self.velocity.linvel()
      }
      let angvel = if self.locked_rotations {
        @core.Vec3::zero()
      } else {
        self.velocity.angvel()
      }
      let vels : RigidBodyVelocity3D = { linvel, angvel }
      self.position = vels.integrate(
        dt,
        self.position,
        self.mass_properties.center_of_mass,
      )
    }
    RigidBodyType::KinematicVelocityBased => {
      let linvel = if self.locked_translations {
        @core.Vec3::zero()
      } else {
        self.velocity.linvel()
      }
      let angvel = if self.locked_rotations {
        @core.Vec3::zero()
      } else {
        self.velocity.angvel()
      }
      let vels : RigidBodyVelocity3D = { linvel, angvel }
      self.position = vels.integrate(
        dt,
        self.position,
        self.mass_properties.center_of_mass,
      )
    }
    RigidBodyType::KinematicPositionBased => {
      let mut next = self.next_position
      if self.locked_translations {
        next = @core.Isometry3::new(self.position.translation, next.rotation)
      }
      if self.locked_rotations {
        next = @core.Isometry3::new(next.translation, self.position.rotation)
      }
      self.position = next
    }
    RigidBodyType::Fixed => ()
  }
}

///|
pub fn RigidBody3D::integrate(
  self : RigidBody3D,
  gravity : @core.Vec3,
  dt : @core.Real,
) -> Unit {
  self.apply_gravity(gravity, dt)
  self.apply_damping(dt)
  self.apply_gyroscopic_forces(dt)
  self.advance_position(dt)
}

///|
pub struct RigidBodyBuilder3D {
  mut body_type : RigidBodyType
  mut translation : @core.Vec3
  mut rotation : @core.Quat
  mut linvel : @core.Vec3
  mut angvel : @core.Vec3
  mut locked_translations : Bool
  mut locked_rotations : Bool
  mut rot_x_enabled : Bool
  mut rot_y_enabled : Bool
  mut rot_z_enabled : Bool
  mut ccd_enabled : Bool
  mut soft_ccd_prediction : @core.Real
  mut gyroscopic_forces_enabled : Bool
  mut linear_damping : @core.Real
  mut angular_damping : @core.Real
  mut gravity_scale : @core.Real
  mut mass_properties : @core.MassProperties3
  mut can_sleep : Bool
}

///|
pub fn RigidBodyBuilder3D::dynamic() -> RigidBodyBuilder3D {
  {
    body_type: RigidBodyType::Dynamic,
    translation: @core.Vec3::zero(),
    rotation: @core.Quat::identity(),
    linvel: @core.Vec3::zero(),
    angvel: @core.Vec3::zero(),
    locked_translations: false,
    locked_rotations: false,
    rot_x_enabled: true,
    rot_y_enabled: true,
    rot_z_enabled: true,
    ccd_enabled: false,
    soft_ccd_prediction: 0.0F,
    gyroscopic_forces_enabled: false,
    linear_damping: 0.0F,
    angular_damping: 0.0F,
    gravity_scale: 1.0F,
    mass_properties: @core.MassProperties3::new(
      1.0F,
      @core.SdpMat3::from_diagonal(@core.Vec3::new(1.0F, 1.0F, 1.0F)),
      @core.Vec3::zero(),
    ),
    can_sleep: true,
  }
}

///|
pub fn RigidBodyBuilder3D::fixed() -> RigidBodyBuilder3D {
  let b = RigidBodyBuilder3D::dynamic()
  b.body_type = RigidBodyType::Fixed
  b.mass_properties = @core.MassProperties3::default()
  b.can_sleep = false
  b
}

///|
pub fn RigidBodyBuilder3D::kinematic_position_based() -> RigidBodyBuilder3D {
  let b = RigidBodyBuilder3D::dynamic()
  b.body_type = RigidBodyType::KinematicPositionBased
  b.mass_properties = @core.MassProperties3::default()
  b.can_sleep = false
  b
}

///|
pub fn RigidBodyBuilder3D::kinematic_velocity_based() -> RigidBodyBuilder3D {
  let b = RigidBodyBuilder3D::dynamic()
  b.body_type = RigidBodyType::KinematicVelocityBased
  b.mass_properties = @core.MassProperties3::default()
  b.can_sleep = false
  b
}

///|
pub fn RigidBodyBuilder3D::can_sleep(
  self : RigidBodyBuilder3D,
  can_sleep : Bool,
) -> RigidBodyBuilder3D {
  self.can_sleep = can_sleep
  self
}

///|
pub fn RigidBodyBuilder3D::translation(
  self : RigidBodyBuilder3D,
  t : @core.Vec3,
) -> RigidBodyBuilder3D {
  self.translation = t
  self
}

///|
pub fn RigidBodyBuilder3D::rotation(
  self : RigidBodyBuilder3D,
  r : @core.Quat,
) -> RigidBodyBuilder3D {
  self.rotation = r
  self
}

///|
pub fn RigidBodyBuilder3D::rotation_scaled_axis(
  self : RigidBodyBuilder3D,
  scaled_axis : @core.Vec3,
) -> RigidBodyBuilder3D {
  self.rotation = @core.rotation_from_scaled_axis(scaled_axis)
  self
}

///|
pub fn RigidBodyBuilder3D::linvel(
  self : RigidBodyBuilder3D,
  v : @core.Vec3,
) -> RigidBodyBuilder3D {
  self.linvel = v
  self
}

///|
pub fn RigidBodyBuilder3D::angvel(
  self : RigidBodyBuilder3D,
  w : @core.Vec3,
) -> RigidBodyBuilder3D {
  self.angvel = w
  self
}

///|
pub fn RigidBodyBuilder3D::linear_damping(
  self : RigidBodyBuilder3D,
  damping : @core.Real,
) -> RigidBodyBuilder3D {
  self.linear_damping = damping
  self
}

///|
pub fn RigidBodyBuilder3D::angular_damping(
  self : RigidBodyBuilder3D,
  damping : @core.Real,
) -> RigidBodyBuilder3D {
  self.angular_damping = damping
  self
}

///|
pub fn RigidBodyBuilder3D::gravity_scale(
  self : RigidBodyBuilder3D,
  scale : @core.Real,
) -> RigidBodyBuilder3D {
  self.gravity_scale = scale
  self
}

///|
pub fn RigidBodyBuilder3D::lock_translations(
  self : RigidBodyBuilder3D,
) -> RigidBodyBuilder3D {
  self.locked_translations = true
  self
}

///|
pub fn RigidBodyBuilder3D::lock_rotations(
  self : RigidBodyBuilder3D,
) -> RigidBodyBuilder3D {
  self.locked_rotations = true
  self.rot_x_enabled = false
  self.rot_y_enabled = false
  self.rot_z_enabled = false
  self
}

///|
pub fn RigidBodyBuilder3D::enabled_rotations(
  self : RigidBodyBuilder3D,
  x : Bool,
  y : Bool,
  z : Bool,
) -> RigidBodyBuilder3D {
  self.locked_rotations = false
  self.rot_x_enabled = x
  self.rot_y_enabled = y
  self.rot_z_enabled = z
  self
}

///|
pub fn RigidBodyBuilder3D::ccd_enabled(
  self : RigidBodyBuilder3D,
  enabled : Bool,
) -> RigidBodyBuilder3D {
  self.ccd_enabled = enabled
  self
}

///|
pub fn RigidBodyBuilder3D::soft_ccd_prediction(
  self : RigidBodyBuilder3D,
  prediction : @core.Real,
) -> RigidBodyBuilder3D {
  self.soft_ccd_prediction = prediction
  self
}

///|
pub fn RigidBodyBuilder3D::gyroscopic_forces_enabled(
  self : RigidBodyBuilder3D,
  enabled : Bool,
) -> RigidBodyBuilder3D {
  self.gyroscopic_forces_enabled = enabled
  self
}

///|
pub fn RigidBodyBuilder3D::mass_properties(
  self : RigidBodyBuilder3D,
  props : @core.MassProperties3,
) -> RigidBodyBuilder3D {
  self.mass_properties = props
  self
}

///|
pub fn RigidBodyBuilder3D::build(self : RigidBodyBuilder3D) -> RigidBody3D {
  let pos = @core.Isometry3::new(self.translation, self.rotation)
  {
    body_type: self.body_type,
    position: pos,
    next_position: pos,
    velocity: { linvel: self.linvel, angvel: self.angvel },
    locked_translations: self.locked_translations,
    locked_rotations: self.locked_rotations,
    rot_x_enabled: self.rot_x_enabled,
    rot_y_enabled: self.rot_y_enabled,
    rot_z_enabled: self.rot_z_enabled,
    ccd_enabled: self.ccd_enabled,
    soft_ccd_prediction: self.soft_ccd_prediction,
    gyroscopic_forces_enabled: self.gyroscopic_forces_enabled,
    linear_damping: self.linear_damping,
    angular_damping: self.angular_damping,
    gravity_scale: self.gravity_scale,
    mass_properties: self.mass_properties,
    activation: RigidBodyActivation3D::active(),
    can_sleep: self.can_sleep,
  }
}

///|
pub struct RigidBodySet3D {
  bodies : Array[RigidBody3D?]
  generations : Array[Int]
  free_list : Array[Int]
}

///|
pub fn RigidBodySet3D::new() -> RigidBodySet3D {
  { bodies: [], generations: [], free_list: [] }
}

///|
pub fn RigidBodySet3D::len(self : RigidBodySet3D) -> Int {
  let mut count = 0
  for i in 0..<self.bodies.length() {
    if self.bodies[i] is Some(_) {
      count = count + 1
    }
  }
  count
}

///|
pub fn RigidBodySet3D::insert(
  self : RigidBodySet3D,
  body : RigidBody3D,
) -> RigidBodyHandle {
  if self.free_list.pop() is Some(index) {
    self.bodies[index] = Some(body)
    let generation = self.generations[index]
    RigidBodyHandle::from_raw_parts(index, generation)
  } else {
    let index = self.bodies.length()
    self.bodies.push(Some(body))
    self.generations.push(0)
    RigidBodyHandle::from_raw_parts(index, 0)
  }
}

///|
pub fn RigidBodySet3D::all_handles(
  self : RigidBodySet3D,
) -> Array[RigidBodyHandle] {
  let out : Array[RigidBodyHandle] = []
  for i in 0..<self.bodies.length() {
    if self.bodies[i] is Some(_) {
      out.push(RigidBodyHandle::from_raw_parts(i, self.generations[i]))
    }
  }
  out
}

///|
pub fn RigidBodySet3D::remove(
  self : RigidBodySet3D,
  handle : RigidBodyHandle,
) -> RigidBody3D? {
  let (id, gen) = handle.into_raw_parts()
  if id < 0 || id >= self.bodies.length() {
    return None
  }
  if self.generations[id] != gen {
    return None
  }
  let removed = self.bodies[id]
  if removed is Some(_) {
    self.bodies[id] = None
    self.free_list.push(id)
    self.generations[id] = self.generations[id] + 1
  }
  removed
}

///|
pub fn RigidBodySet3D::get(
  self : RigidBodySet3D,
  handle : RigidBodyHandle,
) -> RigidBody3D? {
  let (id, gen) = handle.into_raw_parts()
  if id < 0 || id >= self.bodies.length() {
    return None
  }
  if self.generations[id] != gen {
    return None
  }
  self.bodies[id]
}

///|
pub fn RigidBodySet3D::get_mut(
  self : RigidBodySet3D,
  handle : RigidBodyHandle,
) -> RigidBody3D? {
  self.get(handle)
}

///|
pub fn RigidBodySet3D::integrate_all(
  self : RigidBodySet3D,
  gravity : @core.Vec3,
  dt : @core.Real,
) -> Unit {
  for i in 0..<self.bodies.length() {
    if self.bodies[i] is Some(body) {
      body.integrate(gravity, dt)
    }
  }
}

///|
pub fn RigidBodySet3D::apply_gravity_all(
  self : RigidBodySet3D,
  gravity : @core.Vec3,
  dt : @core.Real,
) -> Unit {
  for i in 0..<self.bodies.length() {
    if self.bodies[i] is Some(body) {
      body.apply_gravity(gravity, dt)
    }
  }
}

///|
pub fn RigidBodySet3D::apply_damping_all(
  self : RigidBodySet3D,
  dt : @core.Real,
) -> Unit {
  for i in 0..<self.bodies.length() {
    if self.bodies[i] is Some(body) {
      body.apply_damping(dt)
    }
  }
}

///|
pub fn RigidBodySet3D::apply_gyroscopic_forces_all(
  self : RigidBodySet3D,
  dt : @core.Real,
) -> Unit {
  for i in 0..<self.bodies.length() {
    if self.bodies[i] is Some(body) {
      body.apply_gyroscopic_forces(dt)
    }
  }
}

///|
pub fn RigidBodySet3D::update_kinematic_velocities_all(
  self : RigidBodySet3D,
  dt : @core.Real,
) -> Unit {
  for i in 0..<self.bodies.length() {
    if self.bodies[i] is Some(body) {
      body.update_kinematic_velocities(dt)
    }
  }
}

///|
pub fn RigidBodySet3D::has_ccd_enabled(self : RigidBodySet3D) -> Bool {
  for i in 0..<self.bodies.length() {
    if self.bodies[i] is Some(body) && body.is_ccd_active() {
      return true
    }
  }
  false
}

///|
pub fn RigidBodySet3D::max_linvel_len(self : RigidBodySet3D) -> @core.Real {
  let mut best2 = 0.0F
  for i in 0..<self.bodies.length() {
    if self.bodies[i] is Some(body) {
      let v2 = body.linvel().length_squared()
      if v2 > best2 {
        best2 = v2
      }
    }
  }
  if best2 <= 0.0F {
    0.0F
  } else {
    Float::sqrt(best2)
  }
}

///|
pub fn RigidBodySet3D::advance_positions_all(
  self : RigidBodySet3D,
  dt : @core.Real,
) -> Unit {
  for i in 0..<self.bodies.length() {
    if self.bodies[i] is Some(body) {
      body.advance_position(dt)
    }
  }
}

///|
pub fn RigidBodySet3D::update_sleep_all(
  self : RigidBodySet3D,
  dt : @core.Real,
  length_unit : @core.Real,
) -> Unit {
  for i in 0..<self.bodies.length() {
    if self.bodies[i] is Some(body) {
      body.update_sleep(dt, length_unit)
    }
  }
}
