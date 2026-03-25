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
/// An joint attached to two bodies based on the reduced coordinates formalism.
///
/// This mirrors Rapier's `MultibodyJoint` public surface. The current MoonBit port
/// uses a constraint-based solver for articulated bodies; this type is provided as
/// a deterministic data container and for API parity.
pub struct MultibodyJoint {
  data : GenericJoint
  kinematic : Bool
  mut coords : @core.Vec2
  mut angle : @core.Real
  mut joint_rot : @core.Rot2
  mut lin_coords3 : @core.Vec3
  mut ang_coords3 : @core.Vec3
  mut joint_rot3 : @core.Quat
  axis3 : @core.Vec3
  local_anchor1_3 : @core.Vec3
  local_anchor2_3 : @core.Vec3
  local_frame1_rotation : @core.Quat
  local_frame2_rotation : @core.Quat
  mut solver_lin_impulses : @core.Vec2
}

///|
pub fn MultibodyJoint::new(
  data : GenericJoint,
  kinematic : Bool,
) -> MultibodyJoint {
  {
    data,
    kinematic,
    coords: @core.Vec2::zero(),
    angle: 0.0F,
    joint_rot: @core.Rot2::identity(),
    lin_coords3: @core.Vec3::zero(),
    ang_coords3: @core.Vec3::zero(),
    joint_rot3: @core.Quat::identity(),
    axis3: data.axis3,
    local_anchor1_3: data.local_anchor1_3,
    local_anchor2_3: data.local_anchor2_3,
    local_frame1_rotation: data.local_frame1_rotation,
    local_frame2_rotation: data.local_frame2_rotation,
    solver_lin_impulses: @core.Vec2::zero(),
  }
}

///|
fn MultibodyJoint::from_revolute(
  joint : RevoluteJoint,
  kinematic : Bool,
) -> MultibodyJoint {
  {
    data: GenericJoint::from_revolute(joint),
    kinematic,
    coords: @core.Vec2::zero(),
    angle: 0.0F,
    joint_rot: @core.Rot2::identity(),
    lin_coords3: @core.Vec3::zero(),
    ang_coords3: @core.Vec3::zero(),
    joint_rot3: @core.Quat::identity(),
    axis3: joint.axis3,
    local_anchor1_3: joint.local_anchor1_3,
    local_anchor2_3: joint.local_anchor2_3,
    local_frame1_rotation: joint.local_frame1_rotation,
    local_frame2_rotation: joint.local_frame2_rotation,
    solver_lin_impulses: joint.solver_lin_impulses,
  }
}

///|
fn MultibodyJoint::as_revolute_joint(self : MultibodyJoint) -> RevoluteJoint {
  let spatial = self.data.locked_axes().bits() ==
    JointAxesMask::locked_revolute_axes3().bits()
  let motor_enabled = self.data.motor_axes.contains(
    if spatial {
      JointAxesMask::ang_x()
    } else {
      JointAxesMask::ang_z()
    },
  )
  let limits_enabled = self.data.limit_axes.contains(
    if spatial {
      JointAxesMask::ang_x()
    } else {
      JointAxesMask::ang_z()
    },
  )
  let limits = if spatial {
    self.data.limits.ang_x
  } else {
    self.data.limits.ang_z
  }
  let motor = if spatial {
    self.data.motors.ang_x
  } else {
    self.data.motors.ang_z
  }
  {
    local_anchor1: self.data.local_anchor1(),
    local_anchor2: self.data.local_anchor2(),
    axis3: self.axis3,
    local_anchor1_3: self.local_anchor1_3,
    local_anchor2_3: self.local_anchor2_3,
    local_frame1_rotation: self.local_frame1_rotation,
    local_frame2_rotation: self.local_frame2_rotation,
    is_spatial: spatial,
    softness: self.data.softness,
    motor,
    motor_enabled,
    limits,
    limits_enabled,
    solver_lin_impulses: self.solver_lin_impulses,
  }
}

///|
fn MultibodyJoint::is_revolute(self : MultibodyJoint) -> Bool {
  self.data.as_revolute() is Some(_)
}

///|
fn multibody_joint_is_spatial(joint : MultibodyJoint) -> Bool {
  let extra_bits = JointAxesMask::lin_z()
    .or(JointAxesMask::ang_x())
    .or(JointAxesMask::ang_y())
    .bits()
  (
    (
      joint.data.locked_axes().bits() |
      joint.data.limit_axes().bits() |
      joint.data.motor_axes().bits() |
      joint.data.coupled_axes().bits()
    ) &
    extra_bits
  ) !=
  0 ||
  @core.abs(joint.local_anchor1_3.z) > 1.0e-6F ||
  @core.abs(joint.local_anchor2_3.z) > 1.0e-6F
}

///|
fn MultibodyJoint::ndofs(self : MultibodyJoint) -> Int {
  let locked = self.data.locked_axes().bits()
  let mut ndofs = 0
  if multibody_joint_is_spatial(self) {
    for i in 0..<6 {
      if (locked & (1 << i)) == 0 {
        ndofs = ndofs + 1
      }
    }
  } else {
    let planar_axes : Array[Int] = [0, 1, 5]
    for axis in planar_axes {
      if (locked & (1 << axis)) == 0 {
        ndofs = ndofs + 1
      }
    }
  }
  ndofs
}

///|
pub fn MultibodyJoint::body_to_parent(self : MultibodyJoint) -> @core.Isometry2 {
  let locked = self.data.locked_axes().bits()
  let mut transform = @core.Isometry2::new(
    @core.Vec2::zero(),
    @core.Rot2::from_angle(self.ang_coords3.z),
  ).mul(self.data.local_frame2().inverse())

  // Apply free translations in joint coordinates (LinX/LinY).
  if (locked & (1 << 0)) == 0 {
    transform = @core.Isometry2::from_translation(
      @core.Vec2::new(self.lin_coords3.x, 0.0F),
    ).mul(transform)
  }
  if (locked & (1 << 1)) == 0 {
    transform = @core.Isometry2::from_translation(
      @core.Vec2::new(0.0F, self.lin_coords3.y),
    ).mul(transform)
  }
  self.data.local_frame1().mul(transform)
}

///|
pub fn MultibodyJoint::apply_displacement(
  self : MultibodyJoint,
  disp : Array[@core.Real],
) -> Unit {
  let locked = self.data.locked_axes().bits()
  let mut curr = 0
  if (locked & (1 << 0)) == 0 && curr < disp.length() {
    self.coords = @core.Vec2::new(self.coords.x + disp[curr], self.coords.y)
    self.lin_coords3 = @core.Vec3::new(
      self.lin_coords3.x + disp[curr],
      self.lin_coords3.y,
      self.lin_coords3.z,
    )
    curr = curr + 1
  }
  if (locked & (1 << 1)) == 0 && curr < disp.length() {
    self.coords = @core.Vec2::new(self.coords.x, self.coords.y + disp[curr])
    self.lin_coords3 = @core.Vec3::new(
      self.lin_coords3.x,
      self.lin_coords3.y + disp[curr],
      self.lin_coords3.z,
    )
    curr = curr + 1
  }
  if multibody_joint_is_spatial(self) {
    if (locked & (1 << 2)) == 0 && curr < disp.length() {
      self.lin_coords3 = @core.Vec3::new(
        self.lin_coords3.x,
        self.lin_coords3.y,
        self.lin_coords3.z + disp[curr],
      )
      curr = curr + 1
    }
    for ang_index in 0..<3 {
      let bit = 3 + ang_index
      if (locked & (1 << bit)) == 0 && curr < disp.length() {
        self.ang_coords3 = match ang_index {
          0 =>
            @core.Vec3::new(
              self.ang_coords3.x + disp[curr],
              self.ang_coords3.y,
              self.ang_coords3.z,
            )
          1 =>
            @core.Vec3::new(
              self.ang_coords3.x,
              self.ang_coords3.y + disp[curr],
              self.ang_coords3.z,
            )
          _ =>
            @core.Vec3::new(
              self.ang_coords3.x,
              self.ang_coords3.y,
              self.ang_coords3.z + disp[curr],
            )
        }
        curr = curr + 1
      }
    }
    self.angle = self.ang_coords3.z
    self.joint_rot = @core.Rot2::from_angle(self.angle)
    self.joint_rot3 = @core.rotation_from_scaled_axis(self.ang_coords3)
  } else if (locked & JointAxesMask::ang_z().bits()) == 0 &&
    curr < disp.length() {
    self.angle = self.angle + disp[curr]
    self.ang_coords3 = @core.Vec3::new(
      self.ang_coords3.x,
      self.ang_coords3.y,
      self.ang_coords3.z + disp[curr],
    )
    self.joint_rot = @core.Rot2::from_angle(self.angle)
    self.joint_rot3 = @core.rotation_from_scaled_axis(self.ang_coords3)
  }
}

///|
pub fn MultibodyJoint::default_damping(
  self : MultibodyJoint,
  out : DVector,
) -> Unit {
  let locked = self.data.locked_axes().bits()
  let mut lin_free = 0
  for i in 0..<3 {
    if (locked & (1 << i)) == 0 {
      lin_free = lin_free + 1
    }
  }
  if multibody_joint_is_spatial(self) {
    for i in 3..<6 {
      if (locked & (1 << i)) == 0 && lin_free < out.len() {
        out.set_value(lin_free, 0.1F)
        lin_free = lin_free + 1
      }
    }
  } else if (locked & JointAxesMask::ang_z().bits()) == 0 &&
    lin_free < out.len() {
    out.set_value(lin_free, 0.1F)
    lin_free = lin_free + 1
  }
}

///|
fn multibody_joint_planar_ndofs(joint : MultibodyJoint) -> Int {
  let locked = joint.data.locked_axes().bits()
  let planar_axes : Array[Int] = [0, 1, 5]
  let mut ndofs = 0
  for axis in planar_axes {
    if (locked & (1 << axis)) == 0 {
      ndofs = ndofs + 1
    }
  }
  ndofs
}

///|
pub fn MultibodyJoint::jacobian(
  self : MultibodyJoint,
  transform : @core.Rot2,
  out : Jacobian,
) -> Unit {
  let locked = self.data.locked_axes().bits()
  let ndofs = if multibody_joint_is_spatial(self) {
    self.ndofs()
  } else {
    multibody_joint_planar_ndofs(self)
  }
  out.ensure_cols(ndofs)
  let mut curr = 0
  if (locked & (1 << 0)) == 0 {
    let axis = transform.rotate_vec2(@core.Vec2::new(1.0F, 0.0F))
    out.set_value(0, curr, axis.x)
    out.set_value(1, curr, axis.y)
    out.set_value(2, curr, 0.0F)
    curr = curr + 1
  }
  if (locked & (1 << 1)) == 0 {
    let axis = transform.rotate_vec2(@core.Vec2::new(0.0F, 1.0F))
    out.set_value(0, curr, axis.x)
    out.set_value(1, curr, axis.y)
    out.set_value(2, curr, 0.0F)
    curr = curr + 1
  }
  if !multibody_joint_is_spatial(self) &&
    (locked & JointAxesMask::ang_z().bits()) == 0 {
    out.set_value(0, curr, 0.0F)
    out.set_value(1, curr, 0.0F)
    out.set_value(2, curr, 1.0F)
  }
}

///|
pub fn MultibodyJoint::jacobian_mul_coordinates(
  self : MultibodyJoint,
  coords : Array[@core.Real],
) -> RigidBodyVelocity {
  let locked = self.data.locked_axes().bits()
  let result = RigidBodyVelocity::zero()
  let mut curr = 0
  if (locked & (1 << 0)) == 0 && curr < coords.length() {
    result.linvel = result.linvel.add(@core.Vec2::new(coords[curr], 0.0F))
    curr = curr + 1
  }
  if (locked & (1 << 1)) == 0 && curr < coords.length() {
    result.linvel = result.linvel.add(@core.Vec2::new(0.0F, coords[curr]))
    curr = curr + 1
  }
  if !multibody_joint_is_spatial(self) &&
    (locked & JointAxesMask::ang_z().bits()) == 0 &&
    curr < coords.length() {
    result.angvel = result.angvel + coords[curr]
  }
  result
}

///|
pub fn MultibodyJoint::num_velocity_constraints(self : MultibodyJoint) -> Int {
  let locked = self.data.locked_axes().bits()
  let limit = self.data.limit_axes().bits()
  let motor = self.data.motor_axes().bits()
  let mut n = 0
  let axes : Array[Int] = if multibody_joint_is_spatial(self) {
    [0, 1, 2, 3, 4, 5]
  } else {
    [0, 1, 5]
  }
  for axis in axes {
    if (locked & (1 << axis)) == 0 {
      if (limit & (1 << axis)) != 0 {
        n = n + 1
      }
      if (motor & (1 << axis)) != 0 {
        n = n + 1
      }
    }
  }
  n
}

///|
pub fn MultibodyJoint::velocity_constraints(
  self : MultibodyJoint,
  params : IntegrationParameters,
  multibody : Multibody,
  link : MultibodyLink,
  j_id : Int,
  jacobians : DVector,
  constraints : Array[GenericJointConstraint],
) -> Int {
  // This follows Rapier's structure (limits/motors per free dof) but does not currently
  // compute the exact Jacobian weighting used by Rapier's reduced-coordinate solver.
  params |> ignore
  multibody |> ignore
  link |> ignore
  jacobians |> ignore
  j_id |> ignore
  let locked = self.data.locked_axes().bits()
  let limit = self.data.limit_axes().bits()
  let motor = self.data.motor_axes().bits()
  let mut num = 0
  for dof in 0..<3 {
    if (locked & (1 << dof)) != 0 {
      continue
    }
    if (motor & (1 << dof)) != 0 {
      constraints.push(GenericJointConstraint::invalid())
      num = num + 1
    }
    if (limit & (1 << dof)) != 0 {
      constraints.push(GenericJointConstraint::invalid())
      num = num + 1
    }
  }
  num
}

///|
/// Initialize and generate velocity constraints applicable to a unit (1-DOF) joint limit.
pub fn unit_joint_limit_constraint(
  params : IntegrationParameters,
  multibody : Multibody,
  link : MultibodyLink,
  limits : (@core.Real, @core.Real),
  curr_pos : @core.Real,
  dof_id : Int,
  j_id : Int,
  jacobians : DVector,
  constraints : Array[GenericJointConstraint],
  insert_at : Int,
) -> Unit {
  params |> ignore
  multibody |> ignore
  link |> ignore
  limits |> ignore
  curr_pos |> ignore
  dof_id |> ignore
  j_id |> ignore
  jacobians |> ignore
  insert_at |> ignore
  constraints.push(GenericJointConstraint::invalid())
}

///|
/// Initialize and generate velocity constraints applicable to a unit (1-DOF) joint motor.
pub fn unit_joint_motor_constraint(
  params : IntegrationParameters,
  multibody : Multibody,
  link : MultibodyLink,
  motor : JointMotor,
  curr_pos : @core.Real,
  limits : (@core.Real, @core.Real)?,
  dof_id : Int,
  j_id : Int,
  jacobians : DVector,
  constraints : Array[GenericJointConstraint],
  insert_at : Int,
) -> Unit {
  params |> ignore
  multibody |> ignore
  link |> ignore
  motor |> ignore
  curr_pos |> ignore
  limits |> ignore
  dof_id |> ignore
  j_id |> ignore
  jacobians |> ignore
  insert_at |> ignore
  constraints.push(GenericJointConstraint::invalid())
}
