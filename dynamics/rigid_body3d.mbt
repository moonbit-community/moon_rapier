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
pub struct RigidBody3D {
  body_type : RigidBodyType
  mut position : @core.Isometry3
  mut velocity : RigidBodyVelocity3D
  linear_damping : @core.Real
  angular_damping : @core.Real
  gravity_scale : @core.Real
  mass_properties : @core.MassProperties3
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
pub fn RigidBody3D::linvel(self : RigidBody3D) -> @core.Vec3 {
  self.velocity.linvel
}

///|
pub fn RigidBody3D::angvel(self : RigidBody3D) -> @core.Vec3 {
  self.velocity.angvel
}

///|
pub fn RigidBody3D::set_linvel(self : RigidBody3D, v : @core.Vec3) -> Unit {
  self.velocity = { linvel: v, angvel: self.velocity.angvel }
}

///|
pub fn RigidBody3D::set_angvel(self : RigidBody3D, w : @core.Vec3) -> Unit {
  self.velocity = { linvel: self.velocity.linvel, angvel: w }
}

///|
pub fn RigidBody3D::body_type(self : RigidBody3D) -> RigidBodyType {
  self.body_type
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
pub fn RigidBody3D::set_translation(self : RigidBody3D, t : @core.Vec3) -> Unit {
  self.position = @core.Isometry3::new(t, self.position.rotation)
}

///|
pub fn RigidBody3D::set_rotation(self : RigidBody3D, r : @core.Quat) -> Unit {
  self.position = @core.Isometry3::new(self.position.translation, r)
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
  self.velocity = self.velocity.apply_damping(
    dt,
    self.linear_damping,
    self.angular_damping,
  )
}

///|
pub fn RigidBody3D::advance_position(
  self : RigidBody3D,
  dt : @core.Real,
) -> Unit {
  if !self.body_type.is_dynamic() {
    return
  }
  self.position = self.velocity.integrate(
    dt,
    self.position,
    self.mass_properties.center_of_mass,
  )
}

///|
pub fn RigidBody3D::integrate(
  self : RigidBody3D,
  gravity : @core.Vec3,
  dt : @core.Real,
) -> Unit {
  self.apply_gravity(gravity, dt)
  self.apply_damping(dt)
  self.advance_position(dt)
}

///|
pub struct RigidBodyBuilder3D {
  mut body_type : RigidBodyType
  mut translation : @core.Vec3
  mut rotation : @core.Quat
  mut linvel : @core.Vec3
  mut angvel : @core.Vec3
  mut linear_damping : @core.Real
  mut angular_damping : @core.Real
  mut gravity_scale : @core.Real
  mut mass_properties : @core.MassProperties3
}

///|
pub fn RigidBodyBuilder3D::dynamic() -> RigidBodyBuilder3D {
  {
    body_type: RigidBodyType::Dynamic,
    translation: @core.Vec3::zero(),
    rotation: @core.Quat::identity(),
    linvel: @core.Vec3::zero(),
    angvel: @core.Vec3::zero(),
    linear_damping: 0.0F,
    angular_damping: 0.0F,
    gravity_scale: 1.0F,
    mass_properties: @core.MassProperties3::new(
      1.0F,
      @core.SdpMat3::from_diagonal(@core.Vec3::new(1.0F, 1.0F, 1.0F)),
      @core.Vec3::zero(),
    ),
  }
}

///|
pub fn RigidBodyBuilder3D::fixed() -> RigidBodyBuilder3D {
  let b = RigidBodyBuilder3D::dynamic()
  b.body_type = RigidBodyType::Fixed
  b.mass_properties = @core.MassProperties3::default()
  b
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
pub fn RigidBodyBuilder3D::mass_properties(
  self : RigidBodyBuilder3D,
  props : @core.MassProperties3,
) -> RigidBodyBuilder3D {
  self.mass_properties = props
  self
}

///|
pub fn RigidBodyBuilder3D::build(self : RigidBodyBuilder3D) -> RigidBody3D {
  {
    body_type: self.body_type,
    position: @core.Isometry3::new(self.translation, self.rotation),
    velocity: { linvel: self.linvel, angvel: self.angvel },
    linear_damping: self.linear_damping,
    angular_damping: self.angular_damping,
    gravity_scale: self.gravity_scale,
    mass_properties: self.mass_properties,
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
