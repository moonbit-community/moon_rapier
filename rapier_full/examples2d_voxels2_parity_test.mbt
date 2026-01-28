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
fn step_pipeline_2d_n_voxels2(
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
/// Ported from `rapier-reference/examples2d/voxels2.rs` (testbed settings fixed to defaults).
test "examples2d/voxels2.rs parity" {
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

  // Defaults from the testbed UI:
  // - Falling objects = Mixed (3)
  // - Voxel size y = 1.0
  // - CCD disabled
  let falling_objects_default = 3
  let voxel_size_y = 1.0F
  let voxel_size = @core.Vec2::new(1.0F, voxel_size_y)

  // Create dynamic objects to fall on voxels.
  let nx = 50
  let ball_radius = 0.5F
  let dyn_handles : Array[@dynamics.RigidBodyHandle] = []
  for i in 0..<nx {
    for j in 0..<10 {
      let x = Float::from_double(i.to_double()) * 2.0F -
        Float::from_double(nx.to_double()) / 2.0F
      let y = 20.0F + Float::from_double(j.to_double()) * 2.0F
      let rb_handle = bodies.insert(
        @dynamics.RigidBodyBuilder::dynamic()
        .translation(@core.Vec2::new(x, y))
        .build(),
      )
      let falling_objects = if falling_objects_default == 3 {
        j % 3
      } else {
        falling_objects_default
      }
      let co = if falling_objects == 0 {
        @collision.ColliderBuilder::ball(ball_radius)
      } else if falling_objects == 1 {
        @collision.ColliderBuilder::cuboid(ball_radius, ball_radius)
      } else {
        @collision.ColliderBuilder::capsule_y(ball_radius, ball_radius)
      }
      colliders.insert_with_parent(co.build(), rb_handle, bodies) |> ignore
      dyn_handles.push(rb_handle)
    }
  }

  // Voxelization of a polyline mesh.
  let polyline = [
    @core.Vec2::new(0.0F, 0.0F),
    @core.Vec2::new(0.0F, 10.0F),
    @core.Vec2::new(7.0F, 4.0F),
    @core.Vec2::new(14.0F, 10.0F),
    @core.Vec2::new(14.0F, 0.0F),
    @core.Vec2::new(13.0F, 7.0F),
    @core.Vec2::new(7.0F, 2.0F),
    @core.Vec2::new(1.0F, 7.0F),
  ]
  let poly_idx : Array[(Int, Int)] = []
  for i in 0..<polyline.length() {
    poly_idx.push((i, (i + 1) % polyline.length()))
  }
  let mesh_body = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(-20.0F, -10.0F))
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

  // A voxel wavy floor (`ColliderBuilder::voxels_from_points`).
  let voxels : Array[@core.Vec2] = []
  for i in 0..<300 {
    let t = Float::from_double(i.to_double()) / 20.0F
    let mut y = Float::from_double(@math.sin(t.to_double()))
    if y < -0.5F {
      y = -0.5F
    } else if y > 0.5F {
      y = 0.5F
    }
    y = y * 20.0F
    let x = (Float::from_double(i.to_double()) - 125.0F) * voxel_size.x / 2.0F
    voxels.push(@core.Vec2::new(x, y * voxel_size.y))
  }
  colliders.insert(
    @collision.ColliderBuilder::voxels_from_points(voxel_size, voxels).build(),
  )
  |> ignore
  step_pipeline_2d_n_voxels2(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 240,
  )

  // Sanity checks (no NaNs, at least some bodies fell toward the floor).
  let mut min_y = 1.0e30F
  for i in 0..<dyn_handles.length() {
    if bodies.get(dyn_handles[i]) is Some(body) {
      let p = body.translation()
      inspect(p.x == p.x, content="true")
      inspect(p.y == p.y, content="true")
      if p.y < min_y {
        min_y = p.y
      }
    } else {
      inspect(false, content="true")
    }
  }
  inspect(min_y < 5.0F, content="true")
  inspect(min_y > -1000.0F, content="true")
}
