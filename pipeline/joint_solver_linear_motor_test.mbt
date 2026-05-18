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
test "joint solver linear motor (prismatic lin_x velocity)" {
  let pipeline = PhysicsPipeline::PhysicsPipeline()
  let broad_phase = @collision.BroadPhaseBvh()
  let narrow_phase = @collision.NarrowPhase()
  let bodies = @dynamics.RigidBodySet()
  let colliders = @collision.ColliderSet()
  let impulse_joints = @dynamics.ImpulseJointSet()
  let multibody_joints = @dynamics.MultibodyJointSet()
  let islands = @dynamics.IslandManager()
  let hooks = PhysicsHooks::PhysicsHooks()
  let events = EventHandler::EventHandler()
  let ccd_solver = @dynamics_ccd.CCDSolver()
  let gravity = @core.Vec2::zero()
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )
  let ground = bodies.insert(@dynamics.RigidBodyBuilder::fixed().build())
  let body = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::zero())
    .can_sleep(false)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::ball(0.1F).build(),
    body,
    bodies,
  )
  |> ignore
  let mut joint = @dynamics.GenericJoint::from_prismatic(
    @dynamics.PrismaticJointBuilder(Vec2(1.0F, 0.0F)).build(),
  )
  joint = joint
    .set_motor_velocity(LinX, 2.0F, 0.0F)
    .set_motor_max_force(LinX, 1.0e6F)
  impulse_joints.insert(ground, body, joint, true) |> ignore
  for _ in 0..<120 {
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
      impulse_joints, multibody_joints, ccd_solver, hooks, events,
    )
  }
  if bodies.get(body) is Some(rb) {
    inspect(rb.translation().x > 2.0F * 0.5F, content="true")
    inspect(@core.abs(rb.translation().y) < 1.0e-2F, content="true")
  } else {
    inspect(false, content="true")
  }
}
