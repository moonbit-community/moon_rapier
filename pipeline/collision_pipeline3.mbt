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
/// 3D-flavored collision pipeline.
///
/// This is a thin wrapper around the existing 2D `CollisionPipeline` used to port the
/// rapier-reference dim3 unit tests.
pub struct CollisionPipeline3 {
  inner : CollisionPipeline
}

///|
pub fn CollisionPipeline3::new() -> CollisionPipeline3 {
  { inner: CollisionPipeline::new() }
}

///|
pub fn CollisionPipeline3::step(
  self : CollisionPipeline3,
  prediction_distance : @core.Real,
  islands : @dynamics.IslandManager3,
  broad_phase : @collision.BroadPhaseBvh,
  narrow_phase : @collision.NarrowPhase,
  bodies : @dynamics.RigidBodySet3,
  colliders : @collision.ColliderSet3,
  hooks : PhysicsHooks,
  events : EventHandler,
) -> Unit {
  self.inner.step(
    prediction_distance,
    islands.as_2d(),
    broad_phase,
    narrow_phase,
    bodies.as_2d(),
    colliders.as_2d(),
    @dynamics.ImpulseJointSet::new(),
    @dynamics.MultibodyJointSet::new(),
    hooks,
    events,
  )
}
