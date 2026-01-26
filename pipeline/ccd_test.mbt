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
test "ccd clamps fast-moving body to avoid tunneling (disabled vs enabled)" {
  fn run(ccd_enabled : Bool, max_ccd_substeps : Int) -> @core.Real {
    let pipeline = PhysicsPipeline::new()
    let broad_phase = @collision.BroadPhaseBvh::new()
    let narrow_phase = @collision.NarrowPhase::new()
    let bodies = @dynamics.RigidBodySet::new()
    let colliders = @collision.ColliderSet::new()
    let islands = @dynamics.IslandManager::new()
    let impulse_joints = @dynamics.ImpulseJointSet::new()
    let multibody_joints = @dynamics.MultibodyJointSet::new()

    // Static wall centered at x = 0.
    colliders.insert(
      @collision.ColliderBuilder::cuboid(0.1F, 10.0F)
      .translation(@core.Vec2::zero())
      .build(),
    )
    |> ignore
    let ball = bodies.insert(
      @dynamics.RigidBodyBuilder::dynamic()
      .translation(@core.Vec2::new(-10.0F, 0.0F))
      .ccd_enabled(ccd_enabled)
      .build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder::ball(1.0F).build(),
      ball,
      bodies,
    )
    |> ignore
    if bodies.get_mut_internal(ball) is Some(rb) {
      rb.set_linvel(@core.Vec2::new(100.0F, 0.0F), false) |> ignore
    }
    pipeline.step(
      @core.Vec2::zero(),
      @dynamics.IntegrationParameters::default()
      .set_dt(1.0F)
      .set_max_ccd_substeps(max_ccd_substeps),
      islands,
      broad_phase,
      narrow_phase,
      bodies,
      colliders,
      impulse_joints,
      multibody_joints,
      @dynamics.CCDSolver::new(),
      PhysicsHooks::new(),
      EventHandler::new(),
    )
    if bodies.get(ball) is Some(rb) {
      rb.translation().x
    } else {
      0.0F
    }
  }

  let no_ccd_x = run(false, 1)
  inspect(no_ccd_x > 0.0F, content="true")
  let ccd_x = run(true, 1)
  inspect(ccd_x < 0.0F, content="true")
  let ccd_disabled_by_params_x = run(true, 0)
  inspect(ccd_disabled_by_params_x > 0.0F, content="true")
  let ccd_substeps_x = run(true, 2)
  inspect(ccd_substeps_x < 0.0F, content="true")
}
