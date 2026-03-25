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
/// This keeps the dim3 entrypoint wired through native 3D sets/pipelines while reusing the
/// shared solver core.
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
  islands : @dynamics.IslandManager3,
  bodies : @dynamics.RigidBodySet3,
  colliders : @collision.ColliderSet3,
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
fn update_body_mass_properties_from_colliders3(
  bodies : @dynamics.RigidBodySet3,
  colliders : @collision.ColliderSet3,
  modified_bodies : Array[@dynamics.RigidBodyHandle],
) -> Unit {
  for i in 0..<modified_bodies.length() {
    let handle = modified_bodies[i]
    if bodies.get_mut(handle) is Some(body) {
      let attached = colliders.colliders_with_parent(handle)
      let mut mprops = @core.MassProperties::default()
      let default_ccd = @dynamics.RigidBodyCcd::default()
      let mut ccd_thickness = default_ccd.ccd_thickness
      let mut ccd_max_dist = default_ccd.ccd_max_dist
      for j in 0..<attached.length() {
        let co_handle = attached[j]
        if colliders.get(co_handle) is Some(collider) {
          let local_pos = @core.Isometry2::new(
            collider.local_translation,
            @core.Rot2::from_angle(collider.local_rotation),
          )
          mprops = mprops.add(
            collider.mass_properties().transform_by(local_pos),
          )
          let thickness = shape_ccd_thickness(collider.shape)
          if thickness < ccd_thickness {
            ccd_thickness = thickness
          }
          let radius = shape_bounding_radius(collider.shape)
          let dist = collider.local_translation.length() + radius
          if dist > ccd_max_dist {
            ccd_max_dist = dist
          }
        }
      }
      let added_mass = body.additional_mass()
      if added_mass != 0.0F {
        mprops = mprops.set_mass(mprops.mass + added_mass, true)
      }
      body.set_local_mass_properties(mprops) |> ignore
      body.set_ccd_properties(ccd_thickness, ccd_max_dist) |> ignore
    }
  }
}

///|
fn clear_modified_bodies3(
  bodies : @dynamics.RigidBodySet3,
  modified_bodies : Array[@dynamics.RigidBodyHandle],
) -> Unit {
  for i in 0..<modified_bodies.length() {
    let handle = modified_bodies[i]
    if bodies.get_mut(handle) is Some(body) {
      body.clear_changes() |> ignore
    }
  }
}

///|
fn clear_modified_colliders3(
  colliders : @collision.ColliderSet3,
  modified_colliders : Array[@collision.ColliderHandle],
) -> Unit {
  colliders.clear_changes_for(modified_colliders)
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
  self.inner.counters.reset()
  self.inner.counters.step_started()

  let wake_from_impulse = impulse_joints.take_wake_up()
  for i in 0..<wake_from_impulse.length() {
    islands.wake_up(bodies, wake_from_impulse[i], true)
  }
  let wake_from_multibody = multibody_joints.take_wake_up()
  for i in 0..<wake_from_multibody.length() {
    islands.wake_up(bodies, wake_from_multibody[i], true)
  }

  let modified_colliders_before_step = colliders.take_modified()
  @collision.handle_user_changes_to_colliders3(
    bodies, colliders, modified_colliders_before_step,
  )
  let modified_bodies_before_step = bodies.take_modified()
  handle_user_changes_to_bodies3(
    islands, bodies, colliders, impulse_joints, multibody_joints, modified_bodies_before_step,
  )
  update_body_mass_properties_from_colliders3(
    bodies, colliders, modified_bodies_before_step,
  )

  let join_from_impulse = impulse_joints.take_to_join()
  for i in 0..<join_from_impulse.length() {
    let pair = join_from_impulse[i]
    islands.interaction_started_or_stopped(
      bodies,
      Some(pair.0),
      Some(pair.1),
      true,
      false,
    )
  }
  let join_from_multibody = multibody_joints.take_to_join()
  for i in 0..<join_from_multibody.length() {
    let pair = join_from_multibody[i]
    islands.interaction_started_or_stopped(
      bodies,
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
  let collision_pipeline = CollisionPipeline3::new()
  while remaining_substeps > 0 && remaining_time > 0.0F {
    let mut dt = remaining_time
    if ccd_is_enabled && remaining_substeps > 1 {
      let live_bodies = bodies.iter_mut()
      for i in 0..<live_bodies.length() {
        let (_, body) = live_bodies[i]
        if !body.enabled || body.is_sleeping() {
          continue
        }
        if body.body_type is @dynamics.RigidBodyType::Dynamic {
          body.update_effective_force_and_torque(gravity2) |> ignore
        }
      }
      let ccd_active = ccd_solver.update_ccd_active_flags3(
        islands, bodies, remaining_time, true,
      )
      let first_impact = if ccd_active {
        find_first_ccd_impact3(
          gravity, remaining_time, islands, bodies, colliders,
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
      prediction_distance, islands, broad_phase, narrow_phase, bodies, colliders,
      hooks, events,
    )
    let integrating_bodies = bodies.iter_mut()
    for i in 0..<integrating_bodies.length() {
      let (_, body) = integrating_bodies[i]
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
    let island_solver = IslandSolver::new()
    let contact_cache_prev : Array[ContactImpulseCacheEntry] = []
    for i in 0..<self.inner.contact_solver_cache.length() {
      contact_cache_prev.push(self.inner.contact_solver_cache[i])
    }
    self.inner.contact_solver_cache.clear()
    island_solver.solve3(
      parameters,
      islands,
      bodies,
      colliders,
      hooks,
      narrow_phase,
      impulse_joints,
      multibody_joints,
      dt,
      events,
      contact_cache_prev,
      self.inner.contact_solver_cache,
    )

    broad_phase.update3(prediction_distance, bodies, colliders)
    let advancing_bodies = bodies.iter_mut()
    for i in 0..<advancing_bodies.length() {
      let (_, body) = advancing_bodies[i]
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
    if ccd_is_enabled {
      let ccd_active = ccd_solver.update_ccd_active_flags3(
        islands, bodies, dt, false,
      )
      if ccd_active {
        let impacts = predict_ccd_impacts_at_next_positions3(
          dt, islands, bodies, colliders,
        )
        ccd_solver.clamp_motions3(dt, bodies, impacts)
      }
    }

    let syncing_bodies = bodies.iter_mut()
    for i in 0..<syncing_bodies.length() {
      let (_, body) = syncing_bodies[i]
      if !body.enabled || body.is_sleeping() {
        continue
      }
      if body.body_type.is_dynamic_or_kinematic() {
        body.set_position(body.next_position(), false) |> ignore
      }
    }

    collision_pipeline.step(
      prediction_distance, islands, broad_phase, narrow_phase, bodies, colliders,
      hooks, events,
    )
    let (post_constraints, post_manifolds) = build_constraints3(
      parameters,
      bodies,
      colliders,
      hooks,
      narrow_phase,
      dt,
      self.inner.contact_solver_cache,
      0.0F,
      None,
      0,
    )
    if post_constraints.length() > 0 {
      position_correction3(
        bodies,
        colliders,
        post_constraints,
        post_manifolds,
        parameters.allowed_linear_error(),
      )
      position_correction_all3(
        bodies,
        colliders,
        post_constraints,
        post_manifolds,
        parameters.allowed_linear_error(),
      )
    }
    let post_joint_constraints = build_joint_constraints3(
      parameters,
      bodies,
      impulse_joints,
      multibody_joints,
      dt,
      None,
      0,
    )
    if post_joint_constraints.length() > 0 {
      position_correction_joints3(bodies, post_joint_constraints)
    }
    collision_pipeline.step(
      prediction_distance, islands, broad_phase, narrow_phase, bodies, colliders,
      hooks, events,
    )
  }

  populate_contact_impulses_from_cache3(
    narrow_phase,
    self.inner.contact_solver_cache,
  )
  PhysicsPipeline::emit_collision_events3(
    self.inner,
    bodies,
    colliders,
    narrow_phase,
    events,
  )
  PhysicsPipeline::emit_intersection_events3(
    self.inner,
    bodies,
    colliders,
    narrow_phase,
    events,
  )
  let contacts = @dynamics.ContactGraph::new()
  for i in 0..<narrow_phase.contacts.length() {
    let contact = narrow_phase.contacts[i]
    if colliders.get(contact.0) is Some(collider1) &&
      colliders.get(contact.1) is Some(collider2) {
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
  let modified_colliders_after_step = colliders.take_modified()
  let modified_bodies_after_step = bodies.take_modified()
  clear_modified_colliders3(colliders, modified_colliders_before_step)
  clear_modified_colliders3(colliders, modified_colliders_after_step)
  clear_modified_bodies3(bodies, modified_bodies_before_step)
  clear_modified_bodies3(bodies, modified_bodies_after_step)
}
