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
pub type IntegrationParameters = @dynamics.IntegrationParameters

///|
#declaration_only
pub type EventHandler

///|
#declaration_only
pub fn EventHandler::new() -> EventHandler {
  ...
}

///|
#declaration_only
pub fn EventHandler::on_collision_event(
  self : EventHandler,
  callback : (
    @dynamics.RigidBodySet,
    @collision.ColliderSet,
    @collision.CollisionEvent,
  ) -> Unit,
) -> EventHandler {
  ...
}

///|
#declaration_only
pub fn EventHandler::handle_collision_event(
  self : EventHandler,
  bodies : @dynamics.RigidBodySet,
  colliders : @collision.ColliderSet,
  event : @collision.CollisionEvent,
) -> Unit {
  ...
}

///|
#declaration_only
pub fn EventHandler::on_contact_force_event(
  self : EventHandler,
  callback : (
    @core.Real,
    @dynamics.RigidBodySet,
    @collision.ColliderSet,
    @collision.ContactForceEvent,
  ) -> Unit,
) -> EventHandler {
  ...
}

///|
#declaration_only
pub fn EventHandler::handle_contact_force_event(
  self : EventHandler,
  dt : @core.Real,
  bodies : @dynamics.RigidBodySet,
  colliders : @collision.ColliderSet,
  event : @collision.ContactForceEvent,
) -> Unit {
  ...
}

///|
#declaration_only
pub fn EventHandler::push_collision_event(
  self : EventHandler,
  event : @collision.CollisionEvent,
) -> Unit {
  ...
}

///|
#declaration_only
pub fn EventHandler::push_contact_force_event(
  self : EventHandler,
  event : @collision.ContactForceEvent,
) -> Unit {
  ...
}

///|
#declaration_only
pub fn EventHandler::take_collision_events(
  self : EventHandler,
) -> Array[@collision.CollisionEvent] {
  ...
}

///|
#declaration_only
pub fn EventHandler::take_contact_force_events(
  self : EventHandler,
) -> Array[@collision.ContactForceEvent] {
  ...
}

///|
#declaration_only
pub type PhysicsHooks

///|
#declaration_only
pub fn PhysicsHooks::new() -> PhysicsHooks {
  ...
}

///|
#declaration_only
pub fn PhysicsHooks::filter_contact_pair(
  self : PhysicsHooks,
  predicate : (
    @dynamics.RigidBodySet,
    @collision.ColliderSet,
    @collision.ColliderHandle,
    @collision.ColliderHandle,
  ) -> Bool,
) -> PhysicsHooks {
  ...
}

///|
#declaration_only
pub fn PhysicsHooks::filter_intersection_pair(
  self : PhysicsHooks,
  predicate : (
    @dynamics.RigidBodySet,
    @collision.ColliderSet,
    @collision.ColliderHandle,
    @collision.ColliderHandle,
  ) -> Bool,
) -> PhysicsHooks {
  ...
}

///|
#declaration_only
pub type CollisionPipeline

///|
#declaration_only
pub fn CollisionPipeline::new() -> CollisionPipeline {
  ...
}

///|
#declaration_only
pub fn CollisionPipeline::step(
  self : CollisionPipeline,
  prediction_distance : @core.Real,
  islands : @dynamics.IslandManager,
  broad_phase : @collision.BroadPhaseBvh,
  narrow_phase : @collision.NarrowPhase,
  bodies : @dynamics.RigidBodySet,
  colliders : @collision.ColliderSet,
  physics_hooks : PhysicsHooks,
  event_handler : EventHandler,
) -> Unit {
  ...
}

///|
#declaration_only
pub type PhysicsPipeline

///|
#declaration_only
pub fn PhysicsPipeline::new() -> PhysicsPipeline {
  ...
}

///|
#declaration_only
pub fn PhysicsPipeline::step(
  self : PhysicsPipeline,
  gravity : @core.Vec2,
  parameters : IntegrationParameters,
  islands : @dynamics.IslandManager,
  broad_phase : @collision.BroadPhaseBvh,
  narrow_phase : @collision.NarrowPhase,
  bodies : @dynamics.RigidBodySet,
  colliders : @collision.ColliderSet,
  impulse_joints : @dynamics.ImpulseJointSet,
  multibody_joints : @dynamics.MultibodyJointSet,
  ccd_solver : @dynamics.CCDSolver,
  physics_hooks : PhysicsHooks,
  event_handler : EventHandler,
) -> Unit {
  ...
}
