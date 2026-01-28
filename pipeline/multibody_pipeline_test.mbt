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
test "physics_pipeline syncs multibody rigid bodies" {
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = @collision.ColliderSet::new()
  let impulse_joints = @dynamics.ImpulseJointSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()
  let pipeline = PhysicsPipeline::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let islands = @dynamics.IslandManager::new()
  let ccd_solver = @dynamics.CCDSolver::new()
  let hooks = PhysicsHooks::new()
  let events = EventHandler::new()
  let parent = bodies.insert(@dynamics.RigidBodyBuilder::fixed().build())
  let child = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(10.0F, 10.0F))
    .build(),
  )
  let joint = @dynamics.RevoluteJointBuilder::new()
    .local_anchor1(@core.Vec2::zero())
    .local_anchor2(@core.Vec2::new(1.0F, 0.0F))
    .build()
  multibody_joints.insert(parent, child, joint, true) |> ignore
  let parameters = @dynamics.IntegrationParameters::default().set_dt(0.0F)
  pipeline.step(
    @core.Vec2::zero(),
    parameters,
    islands,
    broad_phase,
    narrow_phase,
    bodies,
    colliders,
    impulse_joints,
    multibody_joints,
    ccd_solver,
    hooks,
    events,
  )
  if bodies.get(child) is Some(body) {
    let t = body.translation()
    inspect(t.x == -1.0F, content="true")
    inspect(t.y == 0.0F, content="true")
  } else {
    inspect(false, content="true")
  }
}
