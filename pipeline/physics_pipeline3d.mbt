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
/// This is the public 3D pipeline entrypoint. Internally it delegates to the
/// incremental 3D backend implementation (`PhysicsPipeline3DReal`).
pub struct PhysicsPipeline3D {
  inner : PhysicsPipeline3DReal
}

///|
pub fn PhysicsPipeline3D::new() -> PhysicsPipeline3D {
  { inner: PhysicsPipeline3DReal::new() }
}

///|
pub fn PhysicsPipeline3D::step(
  self : PhysicsPipeline3D,
  gravity : @core.Vec3,
  parameters : @dynamics.IntegrationParameters,
  islands : @dynamics.IslandManager3D,
  broad_phase : @collision.BroadPhase3D,
  narrow_phase : @collision.NarrowPhase3D,
  bodies : @dynamics.RigidBodySet3D,
  colliders : @collision.ColliderSet3D,
) -> Unit {
  self.inner.step(
    gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
  )
}

///|
pub fn PhysicsPipeline3D::step_with_hooks(
  self : PhysicsPipeline3D,
  gravity : @core.Vec3,
  parameters : @dynamics.IntegrationParameters,
  islands : @dynamics.IslandManager3D,
  broad_phase : @collision.BroadPhase3D,
  narrow_phase : @collision.NarrowPhase3D,
  bodies : @dynamics.RigidBodySet3D,
  colliders : @collision.ColliderSet3D,
  hooks : PhysicsHooks3D,
) -> Unit {
  self.inner.step_with_hooks(
    gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders, hooks,
  )
}

///|
pub fn PhysicsPipeline3D::step_with_events(
  self : PhysicsPipeline3D,
  gravity : @core.Vec3,
  parameters : @dynamics.IntegrationParameters,
  islands : @dynamics.IslandManager3D,
  broad_phase : @collision.BroadPhase3D,
  narrow_phase : @collision.NarrowPhase3D,
  bodies : @dynamics.RigidBodySet3D,
  colliders : @collision.ColliderSet3D,
  events : EventHandler3D,
) -> Unit {
  self.inner.step_with_events(
    gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders, events,
  )
}

///|
pub fn PhysicsPipeline3D::step_with_events_and_hooks(
  self : PhysicsPipeline3D,
  gravity : @core.Vec3,
  parameters : @dynamics.IntegrationParameters,
  islands : @dynamics.IslandManager3D,
  broad_phase : @collision.BroadPhase3D,
  narrow_phase : @collision.NarrowPhase3D,
  bodies : @dynamics.RigidBodySet3D,
  colliders : @collision.ColliderSet3D,
  events : EventHandler3D,
  hooks : PhysicsHooks3D,
) -> Unit {
  self.inner.step_with_events_and_hooks(
    gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders, events,
    hooks,
  )
}

///|
pub fn PhysicsPipeline3D::step_with_rope_joints(
  self : PhysicsPipeline3D,
  gravity : @core.Vec3,
  parameters : @dynamics.IntegrationParameters,
  islands : @dynamics.IslandManager3D,
  broad_phase : @collision.BroadPhase3D,
  narrow_phase : @collision.NarrowPhase3D,
  bodies : @dynamics.RigidBodySet3D,
  colliders : @collision.ColliderSet3D,
  rope_joints : @dynamics.RopeJointSet3DReal,
) -> Unit {
  self.inner.step_with_rope_joints(
    gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders, rope_joints,
  )
}

///|
pub fn PhysicsPipeline3D::step_with_rope_joints_and_hooks(
  self : PhysicsPipeline3D,
  gravity : @core.Vec3,
  parameters : @dynamics.IntegrationParameters,
  islands : @dynamics.IslandManager3D,
  broad_phase : @collision.BroadPhase3D,
  narrow_phase : @collision.NarrowPhase3D,
  bodies : @dynamics.RigidBodySet3D,
  colliders : @collision.ColliderSet3D,
  rope_joints : @dynamics.RopeJointSet3DReal,
  hooks : PhysicsHooks3D,
) -> Unit {
  self.inner.step_with_rope_joints_and_hooks(
    gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders, rope_joints,
    hooks,
  )
}

///|
pub fn PhysicsPipeline3D::step_with_joints(
  self : PhysicsPipeline3D,
  gravity : @core.Vec3,
  parameters : @dynamics.IntegrationParameters,
  islands : @dynamics.IslandManager3D,
  broad_phase : @collision.BroadPhase3D,
  narrow_phase : @collision.NarrowPhase3D,
  bodies : @dynamics.RigidBodySet3D,
  colliders : @collision.ColliderSet3D,
  joints : @dynamics.JointSet3DReal,
) -> Unit {
  self.inner.step_with_joints(
    gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders, joints,
  )
}

///|
pub fn PhysicsPipeline3D::step_with_joints_and_hooks(
  self : PhysicsPipeline3D,
  gravity : @core.Vec3,
  parameters : @dynamics.IntegrationParameters,
  islands : @dynamics.IslandManager3D,
  broad_phase : @collision.BroadPhase3D,
  narrow_phase : @collision.NarrowPhase3D,
  bodies : @dynamics.RigidBodySet3D,
  colliders : @collision.ColliderSet3D,
  joints : @dynamics.JointSet3DReal,
  hooks : PhysicsHooks3D,
) -> Unit {
  self.inner.step_with_joints_and_hooks(
    gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders, joints,
    hooks,
  )
}

///|
/// Integration-only step retained as a lightweight helper.
pub fn PhysicsPipeline3D::step_integration_only(
  self : PhysicsPipeline3D,
  gravity : @core.Vec3,
  parameters : @dynamics.IntegrationParameters,
  bodies : @dynamics.RigidBodySet3D,
) -> Unit {
  self |> ignore
  bodies.integrate_all(gravity, parameters.dt)
}
