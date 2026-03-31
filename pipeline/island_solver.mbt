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

///|
priv struct IslandSolver {
  contact_constraints : ContactConstraintsSet
  joint_constraints : JointConstraintsSet
  velocity_solver : VelocitySolver
}

///|
fn IslandSolver::new() -> IslandSolver {
  {
    contact_constraints: ContactConstraintsSet::new(),
    joint_constraints: JointConstraintsSet::new(),
    velocity_solver: VelocitySolver::new(),
  }
}

///|
fn IslandSolver::init_and_solve(
  self : IslandSolver,
  island_id : Int,
  base_parameters : IntegrationParameters,
  islands : @dynamics.IslandManager,
  bodies : @dynamics.RigidBodySet,
  colliders : @collision.ColliderSet,
  physics_hooks : PhysicsHooks,
  narrow_phase : @collision.NarrowPhase,
  impulse_joints : @dynamics.ImpulseJointSet,
  multibody_joints : @dynamics.MultibodyJointSet,
  event_handler : EventHandler,
  contact_cache_in : Array[ContactImpulseCacheEntry],
  contact_cache_out : Array[ContactImpulseCacheEntry],
) -> Unit {
  let extra_iters = islands.island_additional_solver_iterations(island_id)
  let base_iters = if base_parameters.num_solver_iterations <= 0 {
    1
  } else {
    base_parameters.num_solver_iterations
  }
  let extra_iters_clamped = if extra_iters > 0 { extra_iters } else { 0 }
  let num_solver_iterations = base_iters + extra_iters_clamped
  let solver_parameters = base_parameters.set_num_internal_pgs_iterations(
    num_solver_iterations,
  )

  let island_stamp = self.velocity_solver.stamp_island_bodies(
    islands,
    island_id,
    bodies.bodies.length(),
  )
  self.velocity_solver.init_solver_velocities_and_solver_bodies(
    base_parameters.dt,
    solver_parameters,
    island_id,
    islands,
    bodies,
    multibody_joints,
  )
  self.velocity_solver.init_constraints(
    solver_parameters,
    bodies,
    colliders,
    physics_hooks,
    narrow_phase,
    impulse_joints,
    multibody_joints,
    self.contact_constraints,
    self.joint_constraints,
    contact_cache_in,
    island_stamp,
  )
  self.velocity_solver.solve_constraints(
    solver_parameters,
    1,
    bodies,
    colliders,
    self.joint_constraints,
    self.contact_constraints,
    island_stamp,
  )
  self.joint_constraints.writeback_impulses(impulse_joints, multibody_joints)
  self.contact_constraints.writeback_impulses(contact_cache_out)
  self.contact_constraints.emit_contact_force_events(
    bodies,
    colliders,
    base_parameters.dt,
    event_handler,
  )
  self.velocity_solver.writeback_bodies(
    base_parameters, islands, island_id, bodies, multibody_joints,
  )
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
  let step_parameters = parameters.set_dt(dt)
  let active_islands = islands.active_islands()
  for ai in 0..<active_islands.length() {
    let island_id = active_islands[ai]
    if islands.island_bodies(island_id).length() == 0 {
      continue
    }
    self.init_and_solve(
      island_id, step_parameters, islands, bodies, colliders, physics_hooks, narrow_phase,
      impulse_joints, multibody_joints, event_handler, contact_cache_in, contact_cache_out,
    )
  }
}

///|
fn solve_joints_and_contacts_interleaved3(
  parameters : IntegrationParameters,
  bodies : @dynamics.RigidBodySet3,
  colliders : @collision.ColliderSet3,
  physics_hooks : PhysicsHooks,
  narrow_phase : @collision.NarrowPhase,
  impulse_joints : @dynamics.ImpulseJointSet3,
  multibody_joints : @dynamics.MultibodyJointSet3,
  solver_bodies : SolverBodies,
  island_stamps : Array[Int]?,
  stamp : Int,
  additional_solver_iterations : Int,
  dt : @core.Real,
  event_handler : EventHandler,
  contact_cache_in : Array[ContactImpulseCacheEntry],
  contact_cache_out : Array[ContactImpulseCacheEntry],
) -> Unit {
  let joint_constraints = JointConstraintsSet::new()
  joint_constraints.init3(
    parameters, bodies, impulse_joints, multibody_joints, dt, island_stamps, stamp,
  )
  let contact_constraints = ContactConstraintsSet::new()
  contact_constraints.init(
    parameters,
    bodies.inner,
    colliders.inner,
    physics_hooks,
    narrow_phase,
    dt,
    contact_cache_in,
    parameters.warmstart_coefficient,
    island_stamps,
    stamp,
  )
  joint_constraints.warmstart(bodies.inner, solver_bodies)
  if contact_constraints.constraints.length() > 0 {
    contact_constraints.warmstart(bodies.inner, solver_bodies)
  }
  let base_iters0 = if parameters.num_solver_iterations <= 0 {
    1
  } else {
    parameters.num_solver_iterations
  }
  let extra_iters = if additional_solver_iterations <= 0 {
    0
  } else {
    additional_solver_iterations
  }
  let base_iters = base_iters0 + extra_iters
  let joint_iters = base_iters
  let contact_iters = base_iters
  let iters = if contact_iters > joint_iters {
    contact_iters
  } else {
    joint_iters
  }
  for i in 0..<iters {
    if joint_constraints.constraints.length() > 0 && i < joint_iters {
      joint_constraints.solve_velocity_constraints(
        bodies.inner,
        solver_bodies,
        dt,
        1,
      )
    }
    if contact_constraints.constraints.length() > 0 {
      contact_constraints.solve_velocity_constraints(
        bodies.inner,
        solver_bodies,
        dt,
        1,
      )
    }
  }
  if contact_constraints.constraints.length() > 0 {
    contact_constraints.solve_restitution_constraints(
      bodies.inner,
      solver_bodies,
    )
    contact_constraints.position_correction(
      bodies.inner,
      colliders.inner,
      parameters.allowed_linear_error(),
    )
  }
  joint_constraints.position_correction(bodies.inner)
  joint_constraints.writeback_impulses3(impulse_joints, multibody_joints)
  if contact_constraints.constraints.length() > 0 {
    contact_constraints.emit_contact_force_events(
      bodies.inner,
      colliders.inner,
      dt,
      event_handler,
    )
    contact_constraints.append_cache_entries(contact_cache_out)
  }
}

///|
fn IslandSolver::solve3(
  self : IslandSolver,
  parameters : IntegrationParameters,
  islands : @dynamics.IslandManager3,
  bodies : @dynamics.RigidBodySet3,
  colliders : @collision.ColliderSet3,
  physics_hooks : PhysicsHooks,
  narrow_phase : @collision.NarrowPhase,
  impulse_joints : @dynamics.ImpulseJointSet3,
  multibody_joints : @dynamics.MultibodyJointSet3,
  dt : @core.Real,
  event_handler : EventHandler,
  contact_cache_in : Array[ContactImpulseCacheEntry],
  contact_cache_out : Array[ContactImpulseCacheEntry],
) -> Unit {
  let n = bodies.inner.bodies.length()
  self.velocity_solver.solver_bodies.clear()
  self.velocity_solver.solver_bodies.resize(n)
  while self.velocity_solver.island_stamps.length() < n {
    self.velocity_solver.island_stamps.push(0)
  }
  for i in 0..<n {
    if bodies.inner.bodies[i] is Some(rb) {
      self.velocity_solver.solver_bodies.copy_from(i, rb)
    }
  }
  let active_islands = islands.inner.active_islands()
  for ai in 0..<active_islands.length() {
    let island_id = active_islands[ai]
    let island_bodies = islands.inner.island_bodies(island_id)
    if island_bodies.length() == 0 {
      continue
    }
    self.velocity_solver.stamp = self.velocity_solver.stamp + 1
    let stamp = self.velocity_solver.stamp
    for i in 0..<island_bodies.length() {
      let h = island_bodies[i]
      let id = h.id
      if id >= 0 && id < self.velocity_solver.island_stamps.length() {
        self.velocity_solver.island_stamps[id] = stamp
      }
    }
    let extra_iters = islands.inner.island_additional_solver_iterations(
      island_id,
    )
    solve_joints_and_contacts_interleaved3(
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
  self.velocity_solver.solver_bodies.writeback_vels(bodies.inner)
}
