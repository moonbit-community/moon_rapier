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
/// Island-based solver facade.
///
/// Port target:
/// - `rapier-reference/src/dynamics/solver/island_solver.rs`
///
/// This port currently keeps the existing constraint-based solvers, but introduces an explicit
/// `IslandSolver` entry point so we can progressively align the pipeline with Rapier's solver
/// architecture (solver bodies + constraint sets + per-island solving).

///|
priv struct IslandSolver {
  velocity_solver : VelocitySolver
}

///|
fn IslandSolver::new() -> IslandSolver {
  { velocity_solver: VelocitySolver::new() }
}

///|
fn IslandSolver::solve(
  self : IslandSolver,
  parameters : IntegrationParameters,
  islands : @dynamics.IslandManager,
  bodies : @dynamics.RigidBodySet,
  colliders : @collision.ColliderSet,
  physics_hooks : PhysicsHooks,
  narrow_phase : @collision.NarrowPhase,
  impulse_joints : @dynamics.ImpulseJointSet,
  multibody_joints : @dynamics.MultibodyJointSet,
  dt : @core.Real,
  event_handler : EventHandler,
  contact_cache_in : Array[ContactImpulseCacheEntry],
  contact_cache_out : Array[ContactImpulseCacheEntry],
) -> Unit {
  // Keep a solver-side snapshot of body poses/vels. This is currently used only as a
  // compatibility scaffold: later steps will build constraints from this cache.
  let n = bodies.bodies.length()
  self.velocity_solver.solver_bodies.clear()
  self.velocity_solver.solver_bodies.resize(n)
  while self.velocity_solver.island_stamps.length() < n {
    self.velocity_solver.island_stamps.push(0)
  }
  for i in 0..<n {
    if bodies.bodies[i] is Some(rb) {
      self.velocity_solver.solver_bodies.copy_from(i, rb)
    }
  }
  let active_islands = islands.active_islands()
  for ai in 0..<active_islands.length() {
    let island_id = active_islands[ai]
    let island_bodies = islands.island_bodies(island_id)
    if island_bodies.length() == 0 {
      continue
    }
    // Stamp bodies in this island so constraint generation can filter interactions.
    self.velocity_solver.stamp = self.velocity_solver.stamp + 1
    let stamp = self.velocity_solver.stamp
    for i in 0..<island_bodies.length() {
      let h = island_bodies[i]
      let id = h.id
      if id >= 0 && id < self.velocity_solver.island_stamps.length() {
        self.velocity_solver.island_stamps[id] = stamp
      }
    }
    let extra_iters = islands.island_additional_solver_iterations(island_id)
    solve_joints_and_contacts_interleaved(
      parameters,
      bodies,
      colliders,
      physics_hooks,
      narrow_phase,
      impulse_joints,
      multibody_joints,
      self.velocity_solver.solver_bodies,
      Some(self.velocity_solver.island_stamps),
      stamp,
      extra_iters,
      dt,
      event_handler,
      contact_cache_in,
      contact_cache_out,
    )
  }
  // Scatter updated velocities back into the rigid-body set for integration.
  self.velocity_solver.solver_bodies.writeback_vels(bodies)
}
