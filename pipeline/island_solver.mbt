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
  bodies : @dynamics.RigidBodySet,
  colliders : @collision.ColliderSet,
  physics_hooks : PhysicsHooks,
  narrow_phase : @collision.NarrowPhase,
  impulse_joints : @dynamics.ImpulseJointSet,
  multibody_joints : @dynamics.MultibodyJointSet,
  dt : @core.Real,
  event_handler : EventHandler,
  contact_cache : Array[ContactImpulseCacheEntry],
) -> Unit {
  // Keep a solver-side snapshot of body poses/vels. This is currently used only as a
  // compatibility scaffold: later steps will build constraints from this cache.
  let n = bodies.bodies.length()
  self.velocity_solver.solver_bodies.clear()
  self.velocity_solver.solver_bodies.resize(n)
  for i in 0..<n {
    if bodies.bodies[i] is Some(rb) {
      self.velocity_solver.solver_bodies.copy_from(i, rb)
    }
  }
  solve_joints_and_contacts_interleaved(
    parameters,
    bodies,
    colliders,
    physics_hooks,
    narrow_phase,
    impulse_joints,
    multibody_joints,
    self.velocity_solver.solver_bodies,
    dt,
    event_handler,
    contact_cache,
  )
  // Scatter updated velocities back into the rigid-body set for integration.
  self.velocity_solver.solver_bodies.writeback_vels(bodies)
}
