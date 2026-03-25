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
/// 3D-flavored physics pipeline.
///
/// This is a compatibility wrapper used to port rapier-reference dim3 tests.
pub struct PhysicsPipeline3 {
  inner : PhysicsPipeline

  fn new() -> PhysicsPipeline3
}

///|
pub fn PhysicsPipeline3::new() -> PhysicsPipeline3 {
  { inner: PhysicsPipeline::new() }
}

///|
fn handle_user_changes_to_bodies3(
  islands : @dynamics.IslandManager,
  bodies : @dynamics.RigidBodySet,
  colliders : @collision.ColliderSet,
  impulse_joints : @dynamics.ImpulseJointSet3,
  multibody_joints : @dynamics.MultibodyJointSet3,
  modified_bodies : Array[@dynamics.RigidBodyHandle],
) -> Unit {
  multibody_joints |> ignore
  for i in 0..<modified_bodies.length() {
    let handle = modified_bodies[i]
    if bodies.get(handle) is Some(body) {
      if body.is_enabled() {
        islands.rigid_body_updated(handle, bodies)
      } else {
        islands.rigid_body_removed_or_disabled(handle, bodies)
      }
      colliders.set_parent_enabled(handle, body.is_enabled())
      impulse_joints.map_attached_joints_mut(handle, (
        parent,
        child,
        joint_handle,
        joint,
      ) => {
        parent |> ignore
        child |> ignore
        joint_handle |> ignore
        joint.set_enabled_by_attached_body(body.is_enabled()) |> ignore
      })
    }
  }
}

///|
pub fn PhysicsPipeline3::step(
  self : PhysicsPipeline3,
  gravity : @core.Vec3,
  parameters : @dynamics.IntegrationParameters,
  islands : @dynamics.IslandManager3,
  broad_phase : @collision.BroadPhaseBvh,
  narrow_phase : @collision.NarrowPhase,
  bodies : @dynamics.RigidBodySet3,
  colliders : @collision.ColliderSet3,
  impulse_joints : @dynamics.ImpulseJointSet3,
  multibody_joints : @dynamics.MultibodyJointSet3,
  ccd_solver : @dynamics.CCDSolver,
  hooks : PhysicsHooks,
  events : EventHandler,
) -> Unit {
  let gravity2 = @core.Vec2::new(gravity.x, gravity.y)
  let islands2 = islands.as_2d()
  let bodies2 = bodies.as_2d()
  let colliders2 = colliders.as_2d()
  let empty_multibody_joints = @dynamics.MultibodyJointSet::new()
  self.inner.counters.reset()
  self.inner.counters.step_started()

  let wake_from_impulse = impulse_joints.take_wake_up()
  for i in 0..<wake_from_impulse.length() {
    islands2.wake_up(bodies2, wake_from_impulse[i], true)
  }
  let wake_from_multibody = multibody_joints.take_wake_up()
  for i in 0..<wake_from_multibody.length() {
    islands2.wake_up(bodies2, wake_from_multibody[i], true)
  }

  let modified_colliders_before_step = colliders2.take_modified()
  @collision.handle_user_changes_to_colliders(
    bodies2, colliders2, modified_colliders_before_step,
  )
  let modified_bodies_before_step = bodies2.take_modified()
  handle_user_changes_to_bodies3(
    islands2, bodies2, colliders2, impulse_joints, multibody_joints, modified_bodies_before_step,
  )
  update_body_mass_properties_from_colliders(
    bodies2, colliders2, modified_bodies_before_step,
  )

  let join_from_impulse = impulse_joints.take_to_join()
  for i in 0..<join_from_impulse.length() {
    let pair = join_from_impulse[i]
    islands2.interaction_started_or_stopped(
      bodies2,
      Some(pair.0),
      Some(pair.1),
      true,
      false,
    )
  }
  let join_from_multibody = multibody_joints.take_to_join()
  for i in 0..<join_from_multibody.length() {
    let pair = join_from_multibody[i]
    islands2.interaction_started_or_stopped(
      bodies2,
      Some(pair.0),
      Some(pair.1),
      true,
      false,
    )
  }

  multibody_joints.sync_rigid_bodies_from_multibodies(bodies)
  let prediction_distance = parameters.prediction_distance()
  let mut remaining_time = parameters.dt
  let mut remaining_substeps = parameters.max_ccd_substeps
  let ccd_is_enabled = remaining_substeps > 0
  if !ccd_is_enabled {
    remaining_substeps = 1
  }
  let min_ccd_dt = parameters.min_ccd_dt
  let collision_pipeline = CollisionPipeline::new()
  while remaining_substeps > 0 && remaining_time > 0.0F {
    let mut dt = remaining_time
    if ccd_is_enabled && remaining_substeps > 1 {
      for i in 0..<bodies2.bodies.length() {
        if bodies2.bodies[i] is Some(body) {
          if !body.enabled || body.is_sleeping() {
            continue
          }
          if body.body_type is @dynamics.RigidBodyType::Dynamic {
            body.update_effective_force_and_torque(gravity2) |> ignore
          }
        }
      }
      let ccd_active = ccd_solver.update_ccd_active_flags(
        islands2, bodies2, remaining_time, true,
      )
      let first_impact = if ccd_active {
        find_first_ccd_impact(
          gravity2, remaining_time, islands2, bodies2, colliders2,
        )
      } else {
        None
      }
      if first_impact is Some(toi) {
        let steps_real = Float::from_double(remaining_substeps.to_double())
        dt = {
          let original_interval = remaining_time / steps_real
          if toi < original_interval {
            original_interval
          } else {
            toi + (remaining_time - toi) / steps_real
          }
        }
        remaining_substeps = remaining_substeps - 1
      } else {
        dt = remaining_time
        remaining_substeps = 0
      }
      remaining_time = remaining_time - dt
      if remaining_time <= min_ccd_dt {
        dt = dt + remaining_time
        remaining_time = 0.0F
        remaining_substeps = 0
      }
    } else {
      dt = remaining_time
      remaining_time = 0.0F
      remaining_substeps = 0
    }

    collision_pipeline.step(
      prediction_distance,
      islands2,
      broad_phase,
      narrow_phase,
      bodies2,
      colliders2,
      impulse_joints.as_2d(),
      empty_multibody_joints,
      hooks,
      events,
    )
    for i in 0..<bodies2.bodies.length() {
      if bodies2.bodies[i] is Some(body) {
        if !body.enabled || body.is_sleeping() {
          continue
        }
        match body.body_type {
          @dynamics.RigidBodyType::Dynamic => {
            body.update_effective_force_and_torque(gravity2) |> ignore
            let forces = body.forces()
            let raw_vels = forces.integrate(dt, body.vels(), body.mass_props())
            let damped_vels = raw_vels.apply_damping(dt, body.damping())
            let linvel = if body.locked_translations {
              @core.Vec2::zero()
            } else {
              damped_vels.linvel
            }
            let angvel = if body.locked_rotations {
              0.0F
            } else {
              damped_vels.angvel
            }
            body.set_linvel(linvel, false) |> ignore
            body.set_angvel(angvel, false) |> ignore
          }
          @dynamics.RigidBodyType::KinematicPositionBased => {
            let inv_dt = 1.0F / dt
            let derived_vels = @dynamics.RigidBodyPosition::new(
              body.position(),
              body.next_position(),
            ).interpolate_velocity(inv_dt, body.center_of_mass())
            body.set_kinematic_position_based_vels(derived_vels) |> ignore
          }
          @dynamics.RigidBodyType::KinematicVelocityBased => ()
          _ => ()
        }
      }
    }
    let island_solver = IslandSolver::new()
    let contact_cache_prev : Array[ContactImpulseCacheEntry] = []
    for i in 0..<self.inner.contact_solver_cache.length() {
      contact_cache_prev.push(self.inner.contact_solver_cache[i])
    }
    self.inner.contact_solver_cache.clear()
    island_solver.solve3(
      parameters,
      islands2,
      bodies2,
      colliders2,
      hooks,
      narrow_phase,
      impulse_joints,
      multibody_joints,
      dt,
      events,
      contact_cache_prev,
      self.inner.contact_solver_cache,
    )

    broad_phase.update(prediction_distance, bodies2, colliders2)
    for i in 0..<bodies2.bodies.length() {
      if bodies2.bodies[i] is Some(body) {
        if !body.enabled || body.is_sleeping() {
          continue
        }
        match body.body_type {
          @dynamics.RigidBodyType::Dynamic
          | @dynamics.RigidBodyType::KinematicVelocityBased => {
            let next_pos = body
              .vels()
              .integrate(dt, body.position(), body.center_of_mass())
            body.set_next_position_internal(next_pos) |> ignore
          }
          _ => ()
        }
      }
    }
    if ccd_is_enabled {
      let ccd_active = ccd_solver.update_ccd_active_flags(
        islands2, bodies2, dt, false,
      )
      if ccd_active {
        let impacts = predict_ccd_impacts_at_next_positions(
          dt, islands2, bodies2, colliders2,
        )
        ccd_solver.clamp_motions(dt, bodies2, impacts)
      }
    }

    for i in 0..<bodies2.bodies.length() {
      if bodies2.bodies[i] is Some(body) {
        if !body.enabled || body.is_sleeping() {
          continue
        }
        if body.body_type.is_dynamic_or_kinematic() {
          body.set_position(body.next_position(), false) |> ignore
        }
      }
    }

    collision_pipeline.step(
      prediction_distance,
      islands2,
      broad_phase,
      narrow_phase,
      bodies2,
      colliders2,
      impulse_joints.as_2d(),
      empty_multibody_joints,
      hooks,
      events,
    )
    let (post_constraints, post_manifolds) = build_constraints(
      parameters,
      bodies2,
      colliders2,
      hooks,
      narrow_phase,
      dt,
      self.inner.contact_solver_cache,
      0.0F,
      None,
      0,
    )
    if post_constraints.length() > 0 {
      position_correction(
        bodies2,
        colliders2,
        post_constraints,
        post_manifolds,
        parameters.allowed_linear_error(),
      )
      position_correction_all(
        bodies2,
        colliders2,
        post_constraints,
        post_manifolds,
        parameters.allowed_linear_error(),
      )
    }
    let post_joint_constraints = build_joint_constraints3(
      parameters,
      bodies2,
      impulse_joints,
      multibody_joints,
      dt,
      None,
      0,
    )
    if post_joint_constraints.length() > 0 {
      position_correction_joints(bodies2, post_joint_constraints)
    }
    collision_pipeline.step(
      prediction_distance,
      islands2,
      broad_phase,
      narrow_phase,
      bodies2,
      colliders2,
      impulse_joints.as_2d(),
      empty_multibody_joints,
      hooks,
      events,
    )
  }

  populate_contact_impulses_from_cache(
    narrow_phase,
    self.inner.contact_solver_cache,
  )
  PhysicsPipeline::emit_collision_events(
    self.inner,
    bodies2,
    colliders2,
    narrow_phase,
    events,
  )
  PhysicsPipeline::emit_intersection_events(
    self.inner,
    bodies2,
    colliders2,
    narrow_phase,
    events,
  )
  let contacts = @dynamics.ContactGraph::new()
  for i in 0..<narrow_phase.contacts.length() {
    let contact = narrow_phase.contacts[i]
    if colliders2.get(contact.0) is Some(collider1) &&
      colliders2.get(contact.1) is Some(collider2) {
      if collider1.parent is Some(parent1) && collider2.parent is Some(parent2) {
        if !@dynamics.RigidBodyHandle::equals(parent1, parent2) {
          contacts.add_pair(parent1, parent2)
        }
      }
    }
  }
  islands.update_islands(
    parameters.dt,
    1.0F,
    bodies,
    contacts,
    impulse_joints,
    multibody_joints,
  )
  let modified_colliders_after_step = colliders2.take_modified()
  let modified_bodies_after_step = bodies2.take_modified()
  clear_modified_colliders(colliders2, modified_colliders_before_step)
  clear_modified_colliders(colliders2, modified_colliders_after_step)
  clear_modified_bodies(bodies2, modified_bodies_before_step)
  clear_modified_bodies(bodies2, modified_bodies_after_step)
}
