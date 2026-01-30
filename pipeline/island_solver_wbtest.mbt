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
test "island solver filtering matches unfiltered solve (wbtest)" {
  fn setup_world() -> (
    @dynamics.IslandManager,
    @dynamics.RigidBodySet,
    @collision.ColliderSet,
    @collision.BroadPhaseBvh,
    @collision.NarrowPhase,
    @dynamics.ImpulseJointSet,
    @dynamics.MultibodyJointSet,
    (@dynamics.RigidBodyHandle, @dynamics.RigidBodyHandle),
  ) {
    let islands = @dynamics.IslandManager::new()
    let bodies = @dynamics.RigidBodySet::new()
    let colliders = @collision.ColliderSet::new()
    let broad_phase = @collision.BroadPhaseBvh::new()
    let narrow_phase = @collision.NarrowPhase::new()
    let impulse_joints = @dynamics.ImpulseJointSet::new()
    let multibody_joints = @dynamics.MultibodyJointSet::new()
    let ground = bodies.insert(
      @dynamics.RigidBodyBuilder::fixed()
      .translation(@core.Vec2::new(0.0F, -0.5F))
      .build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder::cuboid(50.0F, 0.5F).build(),
      ground,
      bodies,
    )
    |> ignore
    let b1 = bodies.insert(
      @dynamics.RigidBodyBuilder::dynamic()
      .translation(@core.Vec2::new(-10.0F, 0.3F))
      .build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder::ball(0.5F).build(),
      b1,
      bodies,
    )
    |> ignore
    let b2 = bodies.insert(
      @dynamics.RigidBodyBuilder::dynamic()
      .translation(@core.Vec2::new(10.0F, 0.3F))
      .build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder::ball(0.5F).build(),
      b2,
      bodies,
    )
    |> ignore

    // Ensure island manager has these bodies registered as awake.
    islands.rigid_body_updated(b1, bodies)
    islands.rigid_body_updated(b2, bodies)
    islands.rigid_body_updated(ground, bodies)
    (
      islands,
      bodies,
      colliders,
      broad_phase,
      narrow_phase,
      impulse_joints,
      multibody_joints,
      (b1, b2),
    )
  }

  let (
    islands_a,
    bodies_a,
    colliders_a,
    broad_a,
    narrow_a,
    impulse_a,
    multibody_a,
    (a1, a2),
  ) = setup_world()
  let (
    islands_b,
    bodies_b,
    colliders_b,
    broad_b,
    narrow_b,
    impulse_b,
    multibody_b,
    (b1, b2),
  ) = setup_world()
  let hooks = PhysicsHooks::new()
  let events = EventHandler::new()
  let collision_pipeline = CollisionPipeline::new()
  let params = IntegrationParameters::default().set_num_solver_iterations(4)
  let dt = 1.0F / 60.0F
  let cache_a : Array[ContactImpulseCacheEntry] = []
  let cache_b : Array[ContactImpulseCacheEntry] = []
  collision_pipeline.step(
    0.0F, islands_a, broad_a, narrow_a, bodies_a, colliders_a, impulse_a, multibody_a,
    hooks, events,
  )
  collision_pipeline.step(
    0.0F, islands_b, broad_b, narrow_b, bodies_b, colliders_b, impulse_b, multibody_b,
    hooks, events,
  )

  // Unfiltered solve (single call over all interactions).
  let solver_bodies_a = SolverBodies::default()
  solver_bodies_a.resize(bodies_a.bodies.length())
  for i in 0..<bodies_a.bodies.length() {
    if bodies_a.bodies[i] is Some(rb) {
      solver_bodies_a.copy_from(i, rb)
    }
  }
  solve_joints_and_contacts_interleaved(
    params,
    bodies_a,
    colliders_a,
    hooks,
    narrow_a,
    impulse_a,
    multibody_a,
    solver_bodies_a,
    None,
    0,
    dt,
    events,
    cache_a,
  )
  solver_bodies_a.writeback_vels(bodies_a)

  // Filtered solve (one solve per awake island).
  let island_solver = IslandSolver::new()
  island_solver.solve(
    params, islands_b, bodies_b, colliders_b, hooks, narrow_b, impulse_b, multibody_b,
    dt, events, cache_b,
  )
  let p_a1 = bodies_a.get(a1).unwrap().translation()
  let p_a2 = bodies_a.get(a2).unwrap().translation()
  let p_b1 = bodies_b.get(b1).unwrap().translation()
  let p_b2 = bodies_b.get(b2).unwrap().translation()

  // Both solvers should perform equivalent depenetration of the two independent islands.
  inspect(@core.abs(p_a1.y - p_b1.y) < 1.0e-6F, content="true")
  inspect(@core.abs(p_a2.y - p_b2.y) < 1.0e-6F, content="true")
}
