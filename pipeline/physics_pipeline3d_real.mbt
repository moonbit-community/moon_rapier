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
/// Real dim3 physics pipeline (broadphase + narrowphase + contact solver).
///
/// This is an incremental replacement for the current dim3 compatibility layer.
pub struct PhysicsPipeline3DReal {
  mut contact_cache : ContactSolverCache3D
}

///|
pub fn PhysicsPipeline3DReal::new() -> PhysicsPipeline3DReal {
  { contact_cache: ContactSolverCache3D::new() }
}

///|
pub fn PhysicsPipeline3DReal::step(
  self : PhysicsPipeline3DReal,
  gravity : @core.Vec3,
  parameters : @dynamics.IntegrationParameters,
  islands : @dynamics.IslandManager3D,
  broad_phase : @collision.BroadPhase3D,
  narrow_phase : @collision.NarrowPhase3D,
  bodies : @dynamics.RigidBodySet3D,
  colliders : @collision.ColliderSet3D,
) -> Unit {
  let dt = parameters.dt
  bodies.apply_gravity_all(gravity, dt)
  bodies.apply_damping_all(dt)

  // Sync colliders to body motion before detecting contacts.
  colliders.sync_with_bodies(bodies)
  let prediction_distance = 0.0F
  broad_phase.update(prediction_distance, colliders)
  narrow_phase.update(broad_phase.pairs(), bodies, colliders)
  let cache_out = ContactSolverCache3D::new()
  solve_contacts_3d(
    parameters,
    bodies,
    colliders,
    narrow_phase,
    self.contact_cache,
    cache_out,
  )
  self.contact_cache = cache_out
  bodies.advance_positions_all(dt)
  colliders.sync_with_bodies(bodies)
  bodies.update_sleep_all(dt, parameters.length_unit)
  islands.update(bodies)
}
