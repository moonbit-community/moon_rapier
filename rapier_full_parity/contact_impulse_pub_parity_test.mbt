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
test "ContactPair impulse/force pub parity (2D)" {
  let mut parameters = @dynamics.IntegrationParameters::default()
  parameters = parameters
    .set_num_internal_stabilization_iterations(0)
    .set_num_internal_pgs_iterations(1)
    .set_num_solver_iterations(4)
    .set_dt(1.0F / 60.0F)
  let gravity = @core.Vec2::zero()
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = @collision.ColliderSet::new()
  let islands = @dynamics.IslandManager::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let impulse_joints = @dynamics.ImpulseJointSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()
  let ccd = @dynamics.CCDSolver::new()
  let hooks = PhysicsHooks::new()
  let events = EventHandler::new()
  let pipeline = PhysicsPipeline::new()
  let ground = @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, -0.5F))
    .build()
  let ground_handle = bodies.insert(ground)
  let ground_collider = @collision.ColliderBuilder::cuboid(5.0F, 0.5F).build()
  let ground_collider_handle = colliders.insert_with_parent(
    ground_collider, ground_handle, bodies,
  )

  // Overlap slightly so we get a non-zero bias impulse even with zero gravity/velocities.
  let dynamic = @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(0.0F, 0.4F))
    .build()
  let dynamic_handle = bodies.insert(dynamic)
  let dynamic_collider = @collision.ColliderBuilder::cuboid(0.5F, 0.5F).build()
  let dynamic_collider_handle = colliders.insert_with_parent(
    dynamic_collider, dynamic_handle, bodies,
  )
  pipeline.step(
    gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders, impulse_joints,
    multibody_joints, ccd, hooks, events,
  )
  if narrow_phase.contact_pair(dynamic_collider_handle, ground_collider_handle)
    is Some(pair) {
    let total_impulse_mag = pair.total_impulse_magnitude()
    inspect(total_impulse_mag > 0.0F, content="true")
    let total_impulse = pair.total_impulse()
    inspect(total_impulse.length_squared() > 0.0F, content="true")
    let force_mag = total_impulse_mag / parameters.dt
    let event = @collision.ContactForceEvent::from_contact_pair(
      parameters.dt,
      dynamic_collider_handle,
      ground_collider_handle,
      pair,
      force_mag,
    )
    inspect(
      @core.abs(event.total_force_magnitude() - force_mag) < 1.0e-4F,
      content="true",
    )
    let expected_force = @core.Vec2::new(
      total_impulse.x / parameters.dt,
      total_impulse.y / parameters.dt,
    )
    let df = event.total_force().sub(expected_force)
    inspect(df.length_squared() < 1.0e-4F, content="true")
  } else {
    inspect(false, content="true")
  }
}
