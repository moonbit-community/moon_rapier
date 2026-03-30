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
test "revolute joint builder3 exposes motor model and softness" {
  let softness = SpringCoefficients::new(5.0F, 0.25F)
  let joint = GenericJoint::from_revolute(
    RevoluteJointBuilder3::new(@core.Vec3::new(0.0F, 0.0F, 1.0F))
    .softness(softness)
    .motor_model(MotorModel::ForceBased)
    .motor_position(1.0F, 2.0F, 3.0F)
    .motor_max_force(4.0F)
    .build(),
  )
  inspect(
    @core.abs(joint.softness().natural_frequency - 5.0F) < 1.0e-6F,
    content="true",
  )
  inspect(
    @core.abs(joint.softness().damping_ratio - 0.25F) < 1.0e-6F,
    content="true",
  )
  inspect(joint.motors.ang_x.model is MotorModel::ForceBased, content="true")
  inspect(
    @core.abs(joint.motors.ang_x.max_force - 4.0F) < 1.0e-6F,
    content="true",
  )
  inspect(
    joint.locked_axes().bits() == JointAxesMask::locked_revolute_axes3().bits(),
    content="true",
  )
}

///|
test "prismatic joint builder3 exposes motor model and softness" {
  let softness = SpringCoefficients::new(7.0F, 0.1F)
  let joint = GenericJoint::from_prismatic(
    PrismaticJointBuilder3::new(@core.Vec3::new(1.0F, 0.0F, 0.0F))
    .softness(softness)
    .motor_model(MotorModel::ForceBased)
    .motor_velocity(3.0F, 0.5F)
    .motor_max_force(6.0F)
    .build(),
  )
  inspect(
    @core.abs(joint.softness().natural_frequency - 7.0F) < 1.0e-6F,
    content="true",
  )
  inspect(
    @core.abs(joint.softness().damping_ratio - 0.1F) < 1.0e-6F,
    content="true",
  )
  inspect(joint.motors.lin_x.model is MotorModel::ForceBased, content="true")
  inspect(
    @core.abs(joint.motors.lin_x.target_vel - 3.0F) < 1.0e-6F,
    content="true",
  )
  inspect(
    @core.abs(joint.motors.lin_x.damping - 0.5F) < 1.0e-6F,
    content="true",
  )
  inspect(
    @core.abs(joint.motors.lin_x.max_force - 6.0F) < 1.0e-6F,
    content="true",
  )
  inspect(
    joint.locked_axes().bits() == JointAxesMask::locked_prismatic_axes3().bits(),
    content="true",
  )
}

///|
test "rope joint builder3 exposes motor model and softness" {
  let softness = SpringCoefficients::new(9.0F, 0.2F)
  let joint = GenericJoint::from_rope(
    RopeJointBuilder3::new(10.0F)
    .softness(softness)
    .motor_model(MotorModel::ForceBased)
    .motor_position(2.0F, 4.0F, 5.0F)
    .motor_max_force(8.0F)
    .build(),
  )
  inspect(
    @core.abs(joint.softness().natural_frequency - 9.0F) < 1.0e-6F,
    content="true",
  )
  inspect(
    @core.abs(joint.softness().damping_ratio - 0.2F) < 1.0e-6F,
    content="true",
  )
  inspect(joint.motors.lin_x.model is MotorModel::ForceBased, content="true")
  inspect(
    @core.abs(joint.motors.lin_x.target_pos - 2.0F) < 1.0e-6F,
    content="true",
  )
  inspect(
    @core.abs(joint.motors.lin_x.stiffness - 4.0F) < 1.0e-6F,
    content="true",
  )
  inspect(
    @core.abs(joint.motors.lin_x.max_force - 8.0F) < 1.0e-6F,
    content="true",
  )
  inspect(
    joint.coupled_axes().bits() == JointAxesMask::lin_axes3().bits(),
    content="true",
  )
}

///|
test "generic joint 3d setters expose public anchor and basis state" {
  let frame1 = @core.Isometry3::new(
    @core.Vec3::new(1.5F, -2.0F, 3.25F),
    @core.rotation_from_scaled_axis(@core.Vec3::new(0.3F, -0.2F, 0.4F)),
  )
  let frame2 = @core.Isometry3::new(
    @core.Vec3::new(-0.75F, 0.5F, 1.25F),
    @core.rotation_from_scaled_axis(@core.Vec3::new(-0.1F, 0.6F, 0.2F)),
  )
  let joint = GenericJoint::default()
    .set_local_frame1_3(frame1)
    .set_local_frame2_3(frame2)
  inspect(
    @core.abs(joint.local_anchor1_3().z - 3.25F) < 1.0e-6F,
    content="true",
  )
  inspect(
    @core.abs(joint.local_anchor2_3().z - 1.25F) < 1.0e-6F,
    content="true",
  )
  inspect(@core.abs(joint.local_anchor1().x - 1.5F) < 1.0e-6F, content="true")
  inspect(
    @core.abs(
      joint.local_frame1_rotation().to_scaled_axis().x -
      frame1.rotation.to_scaled_axis().x,
    ) <
    1.0e-6F,
    content="true",
  )
  inspect(
    @core.abs(
      joint.local_frame2_3().rotation.to_scaled_axis().y -
      frame2.rotation.to_scaled_axis().y,
    ) <
    1.0e-6F,
    content="true",
  )
}

///|
test "generic joint converts losslessly with generic joint 3d real" {
  let joint3 = SphericalJoint3DRealBuilder::new()
    .local_anchor1(@core.Vec3::new(0.0F, 0.5F, -1.0F))
    .local_anchor2(@core.Vec3::new(0.0F, -0.5F, 2.0F))
    .limits(JointAxis3DReal::AngY, -0.3F, 0.7F)
    .motor_position(JointAxis3DReal::AngX, 1.2F, 2.5F, 0.4F)
    .motor_max_force(JointAxis3DReal::AngX, 9.0F)
    .build()
  let generic = GenericJoint::from_3d_real(joint3)
  inspect(
    generic
    .locked_axes()
    .contains(
      JointAxesMask::lin_x()
      .or(JointAxesMask::lin_y())
      .or(JointAxesMask::lin_z()),
    ),
    content="true",
  )
  if generic.limits_axis(JointAxis::AngY) is Some(limits) {
    inspect(@core.abs(limits.min() + 0.3F) < 1.0e-6F, content="true")
    inspect(@core.abs(limits.max() - 0.7F) < 1.0e-6F, content="true")
  } else {
    inspect(false, content="true")
  }
  if generic.motor(JointAxis::AngX) is Some(motor) {
    inspect(@core.abs(motor.target_pos - 1.2F) < 1.0e-6F, content="true")
    inspect(@core.abs(motor.max_force - 9.0F) < 1.0e-6F, content="true")
  } else {
    inspect(false, content="true")
  }
  let roundtrip = generic.to_3d_real()
  inspect(
    @core.abs(roundtrip.local_frame1.translation.z + 1.0F) < 1.0e-6F,
    content="true",
  )
  inspect(
    roundtrip.coupled_axes.bits() == joint3.coupled_axes.bits(),
    content="true",
  )
}

///|
test "generic joint from spring exposes spring semantics" {
  let spring = SpringJoint::new(2.0F, 5.0F, 0.25F)
    .set_local_anchor1(@core.Vec2::new(1.0F, -2.0F))
    .set_local_anchor2(@core.Vec2::new(-3.0F, 4.0F))
  let joint = GenericJoint::from_spring(spring)
  inspect(
    joint.coupled_axes().bits() == JointAxesMask::lin_axes().bits(),
    content="true",
  )
  if joint.motor(JointAxis::LinX) is Some(motor) {
    inspect(@core.abs(motor.target_pos - 2.0F) < 1.0e-6F, content="true")
    inspect(motor.model is MotorModel::ForceBased, content="true")
  } else {
    inspect(false, content="true")
  }
}
