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
/// A reference to either a multibody link or a rigid-body solver slot.
///
/// This is part of Rapier's advanced joint-solver public surface. The MoonBit port
/// keeps it as a lightweight enum to satisfy pub parity, without tying it to a
/// particular solver implementation.
pub(all) enum LinkOrBodyRef {
  Link(MultibodyLinkId)
  Body(Int)
  Fixed
}

///|
/// Identifies the kind of impulse to write back for a joint constraint.
pub(all) enum WritebackId {
  Dof(Int)
  Limit(Int)
  Motor(Int)
}

///|
/// A small "solver-body" view used by advanced joint-constraint builders.
pub struct JointSolverBody {
  im : @core.Vec2
  ii : @core.Real
  world_com : @core.Vec2
  solver_vel : Int
}

///|
pub fn JointSolverBody::invalid() -> JointSolverBody {
  {
    im: @core.Vec2::zero(),
    ii: 0.0F,
    world_com: @core.Vec2::zero(),
    solver_vel: -1,
  }
}

///|
/// A single scalar constraint used by Rapier's generic joint constraint machinery.
///
/// This is exposed for pub parity and as a stable data container. The actual solver
/// implementation in this repository may not use this type directly.
pub struct GenericJointConstraint {
  is_rigid_body1 : Bool
  is_rigid_body2 : Bool
  solver_vel1 : Int
  solver_vel2 : Int
  ndofs1 : Int
  j_id1 : Int
  ndofs2 : Int
  j_id2 : Int
  joint_id : Int
  impulse : @core.Real
  impulse_bounds : (@core.Real, @core.Real)
  inv_lhs : @core.Real
  rhs : @core.Real
  rhs_wo_bias : @core.Real
  cfm_coeff : @core.Real
  cfm_gain : @core.Real
  writeback_id : WritebackId
}

///|
pub fn GenericJointConstraint::invalid() -> GenericJointConstraint {
  {
    is_rigid_body1: false,
    is_rigid_body2: false,
    solver_vel1: -1,
    solver_vel2: -1,
    ndofs1: 0,
    j_id1: 0,
    ndofs2: 0,
    j_id2: 0,
    joint_id: -1,
    impulse: 0.0F,
    impulse_bounds: (-1.0e30F, 1.0e30F),
    inv_lhs: 0.0F,
    rhs: 0.0F,
    rhs_wo_bias: 0.0F,
    cfm_coeff: 0.0F,
    cfm_gain: 0.0F,
    writeback_id: WritebackId::Dof(0),
  }
}
