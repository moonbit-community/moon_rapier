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
fn step_pipeline_2d_n_smoke(
  pipeline : @pipeline.PhysicsPipeline,
  gravity : @core.Vec2,
  parameters : @dynamics.IntegrationParameters,
  islands : @dynamics.IslandManager,
  broad_phase : @collision.BroadPhaseBvh,
  narrow_phase : @collision.NarrowPhase,
  bodies : @dynamics.RigidBodySet,
  colliders : @collision.ColliderSet,
  impulse_joints : @dynamics.ImpulseJointSet,
  multibody_joints : @dynamics.MultibodyJointSet,
  ccd_solver : @dynamics.CCDSolver,
  hooks : @pipeline.PhysicsHooks,
  events : @pipeline.EventHandler,
  nsteps : Int,
) -> Unit {
  for _ in 0..<nsteps {
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
      impulse_joints, multibody_joints, ccd_solver, hooks, events,
    )
  }
}

///|
test "voxels2 smoke (voxelized_mesh + voxels_from_points)" {
  let pipeline = @pipeline.PhysicsPipeline::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = @collision.ColliderSet::new()
  let impulse_joints = @dynamics.ImpulseJointSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()
  let islands = @dynamics.IslandManager::new()
  let hooks = @pipeline.PhysicsHooks::new()
  let events = @pipeline.EventHandler::new()
  let ccd_solver = @dynamics.CCDSolver::new()
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )
  let gravity = @core.Vec2::new(0.0F, -9.81F)

  // Voxelized polyline mesh.
  let polyline = [
    @core.Vec2::new(0.0F, 0.0F),
    @core.Vec2::new(0.0F, 2.0F),
    @core.Vec2::new(2.0F, 2.0F),
    @core.Vec2::new(2.0F, 0.0F),
  ]
  let poly_idx = [(0, 1), (1, 2), (2, 3), (3, 0)]
  let mesh_body = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(-5.0F, -2.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::voxelized_mesh(
      polyline,
      poly_idx,
      0.2F,
      @collision.FillMode::default(),
    ).build(),
    mesh_body,
    bodies,
  )
  |> ignore

  // Voxel floor from points.
  let voxel_size = @core.Vec2::new(1.0F, 1.0F)
  let voxels : Array[@core.Vec2] = []
  for i in 0..<60 {
    let t = Float::from_double(i.to_double()) / 10.0F
    let mut y = Float::from_double(@math.sin(t.to_double()))
    if y < -0.5F {
      y = -0.5F
    } else if y > 0.5F {
      y = 0.5F
    }
    y = y * 4.0F
    let x = (Float::from_double(i.to_double()) - 25.0F) * voxel_size.x / 2.0F
    voxels.push(@core.Vec2::new(x, y * voxel_size.y))
  }
  colliders.insert(
    @collision.ColliderBuilder::voxels_from_points(voxel_size, voxels).build(),
  )
  |> ignore

  // A few falling bodies.
  let dyn_handles : Array[@dynamics.RigidBodyHandle] = []
  for i in 0..<12 {
    let x = Float::from_double(i.to_double()) * 1.0F - 6.0F
    let rb = bodies.insert(
      @dynamics.RigidBodyBuilder::dynamic()
      .translation(@core.Vec2::new(x, 10.0F))
      .build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder::ball(0.5F).build(),
      rb,
      bodies,
    )
    |> ignore
    dyn_handles.push(rb)
  }
  step_pipeline_2d_n_smoke(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 120,
  )
  for i in 0..<dyn_handles.length() {
    if bodies.get(dyn_handles[i]) is Some(body) {
      let p = body.translation()
      inspect(p.x == p.x, content="true")
      inspect(p.y == p.y, content="true")
    } else {
      inspect(false, content="true")
    }
  }
}
