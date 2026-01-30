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
test "joint solver motor applies angular impulse (wbtest)" {
  let bodies = @dynamics.RigidBodySet::new()
  let impulse_joints = @dynamics.ImpulseJointSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()
  let fixed = bodies.insert(@dynamics.RigidBodyBuilder::fixed().build())
  let dynamic = bodies.insert(@dynamics.RigidBodyBuilder::dynamic().build())
  if bodies.get_mut_internal(dynamic) is Some(rb) {
    rb.set_local_mass_properties(
      @core.MassProperties::new(1.0F, 0.1F, @core.Vec2::zero()),
    )
    |> ignore
  }
  let target = @core.pi() / 2.0F
  let joint = @dynamics.GenericJoint::from_revolute(
    @dynamics.RevoluteJointBuilder::new()
    .motor_position(target, 20.0F, 1.0F)
    .motor_max_force(1000.0F)
    .build(),
  )
  impulse_joints.insert(fixed, dynamic, joint, true) |> ignore
  let dt = 1.0F / 60.0F
  let params = IntegrationParameters::default()
    .set_dt(dt)
    .set_warmstart_coefficient(0.0F)
    .set_num_solver_iterations(1)
  let constraints = build_joint_constraints(
    params,
    bodies,
    impulse_joints,
    multibody_joints,
    dt,
    None,
    0,
  )
  inspect(constraints.length() == 1, content="true")
  let solver_bodies = SolverBodies::default()
  let n = bodies.bodies.length()
  solver_bodies.resize(n)
  for i in 0..<n {
    if bodies.bodies[i] is Some(rb) {
      solver_bodies.copy_from(i, rb)
    }
  }
  solve_joint_velocity_constraints(bodies, solver_bodies, constraints, dt, 1)
  solver_bodies.writeback_vels(bodies)
  if bodies.get(dynamic) is Some(rb) {
    inspect(@core.abs(rb.angvel()) > 1.0e-6F, content="true")
  } else {
    inspect(false, content="true")
  }
}
