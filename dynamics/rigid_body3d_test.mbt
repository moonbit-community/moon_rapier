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
test "rigid_body3d integrates gravity (semi-implicit Euler)" {
  let gravity = @core.Vec3::new(0.0F, -9.81F, 0.0F)
  let rb = RigidBodyBuilder3D::dynamic()
    .translation(@core.Vec3::new(0.0F, 0.0F, 0.0F))
    .linvel(@core.Vec3::zero())
    .build()
  rb.integrate(gravity, 1.0F)
  inspect(@core.abs(rb.linvel().y - -9.81F) < 1.0e-6F, content="true")
  inspect(@core.abs(rb.translation().y - -9.81F) < 1.0e-6F, content="true")
}

///|
test "rigid_body3d integrates angular velocity into quaternion" {
  let rb = RigidBodyBuilder3D::dynamic()
    .rotation(@core.Quat::identity())
    .angvel(@core.Vec3::new(0.0F, @core.pi(), 0.0F))
    .build()
  rb.integrate(@core.Vec3::zero(), 1.0F)
  let v = @core.Vec3::new(1.0F, 0.0F, 0.0F)
  let r = rb.rotation().rotate_vec3(v)
  inspect(@core.abs(r.x - -1.0F) < 1.0e-5F, content="true")
  inspect(@core.abs(r.y - 0.0F) < 1.0e-5F, content="true")
  inspect(@core.abs(r.z - 0.0F) < 1.0e-5F, content="true")
}

///|
test "rigid_body3d dominance and locked axes APIs" {
  let locked_axes = LockedAxes::translation_locked_x().insert(
    LockedAxes::rotation_locked_y(),
  )
  let rb = RigidBodyBuilder3D::dynamic()
    .dominance_group(7)
    .locked_axes(locked_axes)
    .linvel(@core.Vec3::new(1.0F, 2.0F, 3.0F))
    .angvel(@core.Vec3::new(4.0F, 5.0F, 6.0F))
    .build()
  inspect(rb.dominance_group() == 7, content="true")
  inspect(rb.effective_dominance_group() == 7, content="true")
  inspect(
    rb.locked_axes().contains(LockedAxes::translation_locked_x()),
    content="true",
  )
  inspect(
    rb.locked_axes().contains(LockedAxes::rotation_locked_y()),
    content="true",
  )
  inspect(rb.linvel().x == 0.0F, content="true")
  inspect(rb.linvel().y == 2.0F && rb.linvel().z == 3.0F, content="true")
  inspect(rb.angvel().x == 4.0F && rb.angvel().z == 6.0F, content="true")
  inspect(rb.angvel().y == 0.0F, content="true")

  rb.set_enabled_translations(false, true, true, true)
  rb.set_linvel(@core.Vec3::new(9.0F, 8.0F, 7.0F))
  inspect(rb.linvel().x == 0.0F, content="true")
  inspect(rb.linvel().y == 8.0F && rb.linvel().z == 7.0F, content="true")

  rb.set_enabled_rotations(true, false, true, true)
  rb.set_angvel(@core.Vec3::new(3.0F, 2.0F, 1.0F))
  inspect(rb.angvel().x == 3.0F && rb.angvel().z == 1.0F, content="true")
  inspect(rb.angvel().y == 0.0F, content="true")

  rb.set_dominance_group(2)
  inspect(rb.dominance_group() == 2, content="true")

  let fixed = RigidBodyBuilder3D::fixed().dominance_group(5).build()
  inspect(fixed.effective_dominance_group() == 128, content="true")
}

///|
test "rigid_body3d builder additional mass updates mass properties" {
  let rb = RigidBodyBuilder3D::dynamic().additional_mass(3.5F).build()
  inspect(@core.abs(rb.mass_properties().mass - 3.5F) < 1.0e-6F, content="true")
  inspect(
    @core.abs(rb.mass_properties().inv_mass - 1.0F / 3.5F) < 1.0e-6F,
    content="true",
  )
}

///|
test "rigid_body3d builder pose aliases and sleeping flag" {
  let pose = @core.Isometry3::new(
    @core.Vec3::new(1.0F, 2.0F, 3.0F),
    @core.rotation_from_scaled_axis(@core.Vec3::new(0.1F, 0.2F, 0.3F)),
  )
  let props = @core.MassProperties3::new(
    2.0F,
    @core.SdpMat3::from_diagonal(@core.Vec3::new(4.0F, 5.0F, 6.0F)),
    @core.Vec3::new(0.1F, 0.0F, -0.2F),
  )
  let rb = RigidBodyBuilder3D::new_kinematic_position_based()
    .pose(pose)
    .additional_mass_properties(props)
    .sleeping(true)
    .build()
  inspect(
    rb.translation().sub(pose.translation).length() < 1.0e-6F,
    content="true",
  )
  inspect(
    rb.rotation().rotate_vec3(@core.Vec3::new(1.0F, 0.0F, 0.0F)).x != 1.0F,
    content="true",
  )
  inspect(rb.is_sleeping(), content="true")
  inspect(@core.abs(rb.mass_properties().mass - 2.0F) < 1.0e-6F, content="true")

  let fixed = RigidBodyBuilder3D::new_static().build()
  inspect(fixed.body_type() is RigidBodyType::Fixed, content="true")

  let vel_based = RigidBodyBuilder3D::new_kinematic_velocity_based()
    .position(
      @core.Isometry3::from_translation(@core.Vec3::new(0.0F, 1.0F, 2.0F)),
    )
    .build()
  inspect(
    vel_based.translation().sub(@core.Vec3::new(0.0F, 1.0F, 2.0F)).length() <
    1.0e-6F,
    content="true",
  )
}

///|
test "rigid_body_builder3 aliases the real dim3 builder" {
  let rotation = @core.rotation_from_scaled_axis(
    @core.Vec3::new(0.2F, 0.3F, 0.4F),
  )
  let rb = RigidBodyBuilder3::dynamic()
    .translation(@core.Vec3::new(1.0F, 2.0F, 3.0F))
    .rotation(rotation)
    .angvel(@core.Vec3::new(4.0F, 5.0F, 6.0F))
    .enabled_translations(true, false, true)
    .build()
  inspect(
    rb.translation().sub(@core.Vec3::new(1.0F, 2.0F, 3.0F)).length() < 1.0e-6F,
    content="true",
  )
  inspect(
    rb
    .rotation()
    .rotate_vec3(@core.Vec3::new(0.0F, 1.0F, 0.0F))
    .sub(rotation.rotate_vec3(@core.Vec3::new(0.0F, 1.0F, 0.0F)))
    .length() <
    1.0e-6F,
    content="true",
  )
  inspect(rb.linvel().y == 0.0F, content="true")
  inspect(
    rb.angvel().sub(@core.Vec3::new(4.0F, 5.0F, 6.0F)).length() < 1.0e-6F,
    content="true",
  )
}
