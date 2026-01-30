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
}

///|
pub fn PhysicsPipeline3::new() -> PhysicsPipeline3 {
  { inner: PhysicsPipeline::new() }
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
  self.inner.step(
    @core.Vec2::new(gravity.x, gravity.y),
    parameters,
    islands.as_2d(),
    broad_phase,
    narrow_phase,
    bodies.as_2d(),
    colliders.as_2d(),
    impulse_joints.as_2d(),
    multibody_joints.as_2d(),
    ccd_solver,
    hooks,
    events,
  )
}
