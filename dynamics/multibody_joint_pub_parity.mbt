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
  }
}

///|
pub fn MultibodyJoint::body_to_parent(self : MultibodyJoint) -> @core.Isometry2 {
  let locked = self.data.locked_axes().bits()
  // rot * local_frame2^{-1}
  let mut transform = @core.Isometry2::new(@core.Vec2::zero(), self.joint_rot).mul(
    self.data.local_frame2().inverse(),
  )

  // Apply free translations in joint coordinates (LinX/LinY).
  if (locked & (1 << 0)) == 0 {
    transform = @core.Isometry2::from_translation(
      @core.Vec2::new(self.coords.x, 0.0F),
    ).mul(transform)
  }
  if (locked & (1 << 1)) == 0 {
    transform = @core.Isometry2::from_translation(
      @core.Vec2::new(0.0F, self.coords.y),
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
    curr = curr + 1
  }
  if (locked & (1 << 1)) == 0 && curr < disp.length() {
    self.coords = @core.Vec2::new(self.coords.x, self.coords.y + disp[curr])
    curr = curr + 1
  }
  if (locked & (1 << 2)) == 0 && curr < disp.length() {
    self.angle = self.angle + disp[curr]
    self.joint_rot = @core.Rot2::from_angle(self.angle)
  }
}

///|
pub fn MultibodyJoint::default_damping(
  self : MultibodyJoint,
  out : DVector,
) -> Unit {
  let locked = self.data.locked_axes().bits()
  let mut lin_free = 0
  if (locked & (1 << 0)) == 0 {
    lin_free = lin_free + 1
  }
  if (locked & (1 << 1)) == 0 {
    lin_free = lin_free + 1
  }
  if (locked & (1 << 2)) == 0 && lin_free < out.len() {
    out.set_value(lin_free, 0.1F)
  }
}

///|
pub fn MultibodyJoint::jacobian(
  self : MultibodyJoint,
  transform : @core.Rot2,
  out : Jacobian,
) -> Unit {
  let locked = self.data.locked_axes().bits()
  // Number of free dofs.
  let mut ndofs = 0
  if (locked & (1 << 0)) == 0 {
    ndofs = ndofs + 1
  }
  if (locked & (1 << 1)) == 0 {
    ndofs = ndofs + 1
  }
  if (locked & (1 << 2)) == 0 {
    ndofs = ndofs + 1
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
  if (locked & (1 << 2)) == 0 {
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
  if (locked & (1 << 2)) == 0 && curr < coords.length() {
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
  for i in 0..<3 {
    if (locked & (1 << i)) == 0 {
      if (limit & (1 << i)) != 0 {
        n = n + 1
      }
      if (motor & (1 << i)) != 0 {
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
