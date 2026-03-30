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
fn step_pipeline_2d_n_s2d_pyramid(
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
fn run_examples2d_s2d_pyramid(heavy : Bool) -> Bool {
  // Ported from `rapier-reference/examples2d/s2d_pyramid.rs`.
  // Keep default mode tractable; heavy mode moves toward upstream scene scale.
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
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let parameters = @dynamics.IntegrationParameters::default()

  // Ground.
  let ground_handle = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, -1.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(100.0F, 1.0F).friction(0.6F).build(),
    ground_handle,
    bodies,
  )
  |> ignore
  let base_count = if heavy { 45 } else { 30 }
  let nsteps = if heavy { 120 } else { 80 }
  let h = 0.5F
  let shift = 1.0F * h
  let mut dyn_count = 0
  for i in 0..<base_count {
    let fi = Float::from_double(i.to_double())
    let y = (2.0F * fi + 1.0F) * shift
    for j in i..<base_count {
      let fj = Float::from_double(j.to_double())
      let x = (fi + 1.0F) * shift +
        2.0F * (fj - fi) * shift -
        h * Float::from_double(base_count.to_double())
      let rb = bodies.insert(
        @dynamics.RigidBodyBuilder::dynamic()
        .translation(@core.Vec2::new(x, y))
        .build(),
      )
      colliders.insert_with_parent(
        @collision.ColliderBuilder::cuboid(h, h).friction(0.6F).build(),
        rb,
        bodies,
      )
      |> ignore
      dyn_count = dyn_count + 1
    }
  }
  if dyn_count <= 0 {
    return false
  }
  step_pipeline_2d_n_s2d_pyramid(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, nsteps,
  )

  // Sanity: no NaNs/explosions, pyramid stays in a reasonable region.
  let mut min_y = 1.0e30F
  let mut max_d2 = 0.0F
  let it = bodies.iter()
  for i in 0..<it.length() {
    let (_, b) = it[i]
    let p = b.translation()
    if !(p.x == p.x) || !(p.y == p.y) {
      return false
    }
    if p.y < min_y {
      min_y = p.y
    }
    let d2 = p.length_squared()
    if d2 > max_d2 {
      max_d2 = d2
    }
  }
  min_y > -5.0F && max_d2 < 1.0e8F
}

///|
test "examples2d/s2d_pyramid.rs parity" {
  inspect(run_examples2d_s2d_pyramid(false), content="true")
}

///|
#skip("heavy parity profile; run with --include-skipped")
test "HEAVY examples2d/s2d_pyramid.rs parity" {
  inspect(run_examples2d_s2d_pyramid(true), content="true")
}
