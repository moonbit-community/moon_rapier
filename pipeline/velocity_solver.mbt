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
/// Island-scoped staged velocity solver (2D).
///
/// Port target:
/// - `rapier-reference/src/dynamics/solver/velocity_solver.rs`
///
/// The implementation keeps the upstream phase model (`init_solver_velocities_and_solver_bodies`,
/// `init_constraints`, `solve_constraints`, `integrate_positions`, `writeback_bodies`) while
/// staying scalar/single-threaded for MoonBit.

///|
priv struct VelocitySolver {
  solver_bodies : SolverBodies
  solver_vels_increment : Array[SolverVel]
  generic_solver_vels : Array[@core.Real]
  generic_solver_vels_increment : Array[@core.Real]
  multibody_roots : Array[@dynamics.MultibodyLinkId]
  island_stamps : Array[Int]
  mut stamp : Int
}

///|
fn VelocitySolver::new() -> VelocitySolver {
  {
    solver_bodies: SolverBodies::default(),
    solver_vels_increment: [],
    generic_solver_vels: [],
    generic_solver_vels_increment: [],
    multibody_roots: [],
    island_stamps: [],
    stamp: 0,
  }
}

///|
fn velocity_solver_vec2_scale(v : @core.Vec2, s : @core.Real) -> @core.Vec2 {
  @core.Vec2::new(v.x * s, v.y * s)
}

///|
fn VelocitySolver::stamp_island_bodies(
  self : VelocitySolver,
  islands : @dynamics.IslandManager,
  island_id : Int,
  total_bodies : Int,
) -> Int {
  while self.island_stamps.length() < total_bodies {
    self.island_stamps.push(0)
  }
  self.stamp = self.stamp + 1
  let stamp = self.stamp
  let island_bodies = islands.island_bodies(island_id)
  for i in 0..<island_bodies.length() {
    let id = island_bodies[i].id
    if id >= 0 && id < self.island_stamps.length() {
      self.island_stamps[id] = stamp
    }
  }
  stamp
}

///|
fn VelocitySolver::init_solver_velocities_and_solver_bodies(
  self : VelocitySolver,
  total_step_dt : @core.Real,
  parameters : IntegrationParameters,
  island_id : Int,
  islands : @dynamics.IslandManager,
  bodies : @dynamics.RigidBodySet,
  multibody_joints : @dynamics.MultibodyJointSet,
) -> Unit {
  total_step_dt |> ignore
  multibody_joints |> ignore

  self.multibody_roots.clear()
  self.generic_solver_vels.clear()
  self.generic_solver_vels_increment.clear()

  let body_count = bodies.bodies.length()
  self.solver_bodies.clear()
  self.solver_bodies.resize(body_count)
  self.solver_vels_increment.clear()
  for _ in 0..<body_count {
    self.solver_vels_increment.push(SolverVel::zero())
  }

  let island_bodies = islands.island_bodies(island_id)
  for i in 0..<island_bodies.length() {
    let handle = island_bodies[i]
    let id = handle.id
    if id < 0 || id >= body_count {
      continue
    }
    if bodies.bodies[id] is Some(rb) {
      self.solver_bodies.copy_from(id, rb)
      let incr = self.solver_vels_increment[id]
      if rb.is_dynamic() && rb.is_enabled() && !rb.is_sleeping() {
        let forces = rb.forces()
        let inv_m = if rb.locked_translations {
          0.0F
        } else {
          rb.mass_props().effective_inv_mass.x
        }
        let inv_i = if rb.locked_rotations {
          0.0F
        } else {
          rb.mass_props().effective_world_inv_inertia
        }
        incr.linear = velocity_solver_vec2_scale(
          forces.force,
          inv_m * parameters.dt,
        )
        incr.angular = forces.torque * inv_i * parameters.dt
      } else {
        incr.linear = @core.Vec2::zero()
        incr.angular = 0.0F
      }
      self.solver_vels_increment[id] = incr
    }
  }
}

///|
fn VelocitySolver::init_constraints(
  self : VelocitySolver,
  parameters : IntegrationParameters,
  bodies : @dynamics.RigidBodySet,
  colliders : @collision.ColliderSet,
  physics_hooks : PhysicsHooks,
  narrow_phase : @collision.NarrowPhase,
  impulse_joints : @dynamics.ImpulseJointSet,
  multibody_joints : @dynamics.MultibodyJointSet,
  contact_constraints : ContactConstraintsSet,
  joint_constraints : JointConstraintsSet,
  contact_cache : Array[ContactImpulseCacheEntry],
  island_stamp : Int,
) -> Unit {
  joint_constraints.init(
    parameters,
    bodies,
    impulse_joints,
    multibody_joints,
    parameters.dt,
    Some(self.island_stamps),
    island_stamp,
  )
  contact_constraints.init(
    parameters,
    bodies,
    colliders,
    physics_hooks,
    narrow_phase,
    parameters.dt,
    contact_cache,
    parameters.warmstart_coefficient,
    Some(self.island_stamps),
    island_stamp,
  )
}

///|
fn VelocitySolver::solve_constraints(
  self : VelocitySolver,
  parameters : IntegrationParameters,
  num_substeps : Int,
  bodies : @dynamics.RigidBodySet,
  colliders : @collision.ColliderSet,
  joint_constraints : JointConstraintsSet,
  contact_constraints : ContactConstraintsSet,
  island_stamp : Int,
) -> Unit {
  let substeps = if num_substeps <= 0 { 1 } else { num_substeps }
  let pgs_iters = if parameters.num_internal_pgs_iterations <= 0 {
    1
  } else {
    parameters.num_internal_pgs_iterations
  }
  let stabilization_iters = 0
  parameters.num_internal_stabilization_iterations |> ignore

  for substep in 0..<substeps {
    for i in 0..<self.solver_bodies.vels.length() {
      if i >= self.island_stamps.length() || self.island_stamps[i] != island_stamp {
        continue
      }
      let vel = self.solver_bodies.vels[i]
      let incr = self.solver_vels_increment[i]
      vel.linear = vel.linear.add(incr.linear)
      vel.angular = vel.angular + incr.angular
      self.solver_bodies.vels[i] = vel
    }

    joint_constraints.update(parameters, bodies, self.solver_bodies)
    contact_constraints.update(
      parameters,
      substep,
      bodies,
      self.solver_bodies,
    )

    if substep == 0 {
      joint_constraints.warmstart(bodies, self.solver_bodies)
      if parameters.warmstart_coefficient != 0.0F {
        contact_constraints.warmstart(bodies, self.solver_bodies)
      }
    }

    for _ in 0..<pgs_iters {
      joint_constraints.solve(
        bodies,
        self.solver_bodies,
        parameters.dt,
      )
      contact_constraints.solve(
        bodies,
        self.solver_bodies,
        parameters.dt,
      )
    }

    contact_constraints.solve_restitution_constraints(
      bodies,
      self.solver_bodies,
    )

    let is_last_substep = substep == substeps - 1
    self.integrate_positions(parameters, is_last_substep, bodies, island_stamp)
    if is_last_substep {
      for i in 0..<self.solver_bodies.poses.length() {
        if i >= self.island_stamps.length() || self.island_stamps[i] != island_stamp {
          continue
        }
        if bodies.bodies[i] is Some(rb) {
          if rb.is_ccd_enabled() {
            continue
          }
          let pose = self.solver_bodies.poses[i]
          let body_translation = pose.translation.sub(
            pose.rotation.rotate_vec2(rb.local_center_of_mass()),
          )
          rb.set_position(
            @core.Isometry2::new(body_translation, pose.rotation),
            false,
          )
          |> ignore
        }
      }
      contact_constraints.position_correction(
        bodies,
        colliders,
        parameters.allowed_linear_error(),
      )
      contact_constraints.position_correction_all(
        bodies,
        colliders,
        parameters.allowed_linear_error(),
      )
      joint_constraints.position_correction(bodies)
      for i in 0..<self.solver_bodies.poses.length() {
        if i >= self.island_stamps.length() || self.island_stamps[i] != island_stamp {
          continue
        }
        if bodies.bodies[i] is Some(rb) {
          if rb.is_ccd_enabled() {
            continue
          }
          let pose = self.solver_bodies.poses[i]
          pose.translation = rb.world_com()
          pose.rotation = rb.rotation()
          self.solver_bodies.poses[i] = pose
        }
      }
    }

    for _ in 0..<stabilization_iters {
      joint_constraints.solve_wo_bias(
        bodies,
        self.solver_bodies,
        parameters.dt,
      )
      contact_constraints.solve_wo_bias(
        bodies,
        self.solver_bodies,
        parameters.dt,
      )
    }
  }
}

///|
fn VelocitySolver::integrate_positions(
  self : VelocitySolver,
  parameters : IntegrationParameters,
  is_last_substep : Bool,
  bodies : @dynamics.RigidBodySet,
  island_stamp : Int,
) -> Unit {
  is_last_substep |> ignore
  if parameters.dt <= 0.0F {
    return
  }
  for i in 0..<self.solver_bodies.poses.length() {
    if i >= self.island_stamps.length() || self.island_stamps[i] != island_stamp {
      continue
    }
    if bodies.bodies[i] is Some(rb) {
      if !rb.is_enabled() || rb.is_sleeping() {
        continue
      }
      let pose = self.solver_bodies.poses[i]
      let vel = self.solver_bodies.vels[i]
      pose.translation = pose.translation.add(
        velocity_solver_vec2_scale(vel.linear, parameters.dt),
      )
      pose.rotation = pose.rotation.mul(
        @core.Rot2::from_angle(vel.angular * parameters.dt),
      )
      self.solver_bodies.poses[i] = pose
    }
  }
}

///|
fn VelocitySolver::writeback_bodies(
  self : VelocitySolver,
  base_parameters : IntegrationParameters,
  islands : @dynamics.IslandManager,
  island_id : Int,
  bodies : @dynamics.RigidBodySet,
  multibody_joints : @dynamics.MultibodyJointSet,
) -> Unit {
  multibody_joints |> ignore
  let island_bodies = islands.island_bodies(island_id)
  for i in 0..<island_bodies.length() {
    let handle = island_bodies[i]
    let id = handle.id
    if id < 0 || id >= self.solver_bodies.vels.length() {
      continue
    }
    if bodies.get(handle) is Some(rb) {
      if !rb.is_enabled() || rb.is_sleeping() {
        continue
      }
      let solver_vel = self.solver_bodies.vels[id]
      let solver_pose = self.solver_bodies.poses[id]
      let mut body_vel = @dynamics.RigidBodyVelocity::new(
        solver_vel.linear,
        solver_vel.angular,
      )
      body_vel = body_vel.apply_damping(base_parameters.dt, rb.damping())
      let linvel = if rb.locked_translations {
        @core.Vec2::zero()
      } else {
        body_vel.linvel()
      }
      let angvel = if rb.locked_rotations { 0.0F } else { body_vel.angvel() }
      let wake_up = rb.is_dynamic()
      rb.set_linvel(linvel, wake_up) |> ignore
      rb.set_angvel(angvel, wake_up) |> ignore

      if rb.body_type is @dynamics.RigidBodyType::KinematicPositionBased {
        continue
      }
      let body_translation = solver_pose.translation.sub(
        solver_pose.rotation.rotate_vec2(rb.local_center_of_mass()),
      )
      rb.set_next_position_internal(
        @core.Isometry2::new(body_translation, solver_pose.rotation),
      )
      |> ignore
    }
  }
}
