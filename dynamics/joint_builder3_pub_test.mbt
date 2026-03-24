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
  inspect(joint.motors.ang_z.model is MotorModel::ForceBased, content="true")
  inspect(
    @core.abs(joint.motors.ang_z.max_force - 4.0F) < 1.0e-6F,
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
}
