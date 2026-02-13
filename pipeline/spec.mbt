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
pub fn EventHandler::on_intersection_event(
  self : EventHandler,
  callback : (
    @dynamics.RigidBodySet,
    @collision.ColliderSet,
    @collision.IntersectionEvent,
  ) -> Unit,
) -> EventHandler {
  ...
}

///|
#declaration_only
pub fn EventHandler::handle_intersection_event(
  self : EventHandler,
  bodies : @dynamics.RigidBodySet,
  colliders : @collision.ColliderSet,
  event : @collision.IntersectionEvent,
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
pub fn EventHandler::push_intersection_event(
  self : EventHandler,
  event : @collision.IntersectionEvent,
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
pub fn EventHandler::take_intersection_events(
  self : EventHandler,
) -> Array[@collision.IntersectionEvent] {
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
pub fn PhysicsHooks::modify_solver_contacts(
  self : PhysicsHooks,
  callback : (
    @dynamics.RigidBodySet,
    @collision.ColliderSet,
    @collision.ColliderHandle,
    @collision.ColliderHandle,
    @collision.ContactManifold,
  ) -> (Bool, @core.Vec2),
) -> PhysicsHooks {
  ...
}

///|
#declaration_only
pub fn PhysicsHooks::modify_solver_contacts_with_solver_contacts(
  self : PhysicsHooks,
  callback : (
    @dynamics.RigidBodySet,
    @collision.ColliderSet,
    @collision.ColliderHandle,
    @collision.ColliderHandle,
    @collision.ContactManifold,
    Array[@collision.SolverContact],
    @collision.SolverFlags,
  ) -> @collision.SolverFlags,
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
  impulse_joints : @dynamics.ImpulseJointSet,
  multibody_joints : @dynamics.MultibodyJointSet,
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

///|
#declaration_only
pub fn remove_rigid_body(
  bodies : @dynamics.RigidBodySet,
  handle : @dynamics.RigidBodyHandle,
  islands : @dynamics.IslandManager,
  colliders : @collision.ColliderSet,
  impulse_joints : @dynamics.ImpulseJointSet,
  multibody_joints : @dynamics.MultibodyJointSet,
  remove_attached_colliders : Bool,
  wake_up : Bool,
) -> @dynamics.RigidBody? {
  ...
}

///|
#declaration_only
pub type DebugColor

///|
#declaration_only
pub fn DebugColor::new(
  h : @core.Real,
  s : @core.Real,
  l : @core.Real,
  a : @core.Real,
) -> DebugColor {
  ...
}

///|
#declaration_only
pub type DebugRenderStyle

///|
#declaration_only
pub fn DebugRenderStyle::default() -> DebugRenderStyle {
  ...
}

///|
#declaration_only
pub type DebugRenderMode

///|
#declaration_only
pub fn DebugRenderMode::empty() -> DebugRenderMode {
  ...
}

///|
#declaration_only
pub fn DebugRenderMode::all() -> DebugRenderMode {
  ...
}

///|
#declaration_only
pub fn DebugRenderMode::collider_shapes() -> DebugRenderMode {
  ...
}

///|
#declaration_only
pub fn DebugRenderMode::rigid_body_axes() -> DebugRenderMode {
  ...
}

///|
#declaration_only
pub fn DebugRenderMode::multibody_joints() -> DebugRenderMode {
  ...
}

///|
#declaration_only
pub fn DebugRenderMode::impulse_joints() -> DebugRenderMode {
  ...
}

///|
#declaration_only
pub fn DebugRenderMode::joints() -> DebugRenderMode {
  ...
}

///|
#declaration_only
pub fn DebugRenderMode::solver_contacts() -> DebugRenderMode {
  ...
}

///|
#declaration_only
pub fn DebugRenderMode::contacts() -> DebugRenderMode {
  ...
}

///|
#declaration_only
pub fn DebugRenderMode::collider_aabbs() -> DebugRenderMode {
  ...
}

///|
#declaration_only
pub fn DebugRenderMode::default() -> DebugRenderMode {
  ...
}

///|
#declaration_only
pub fn DebugRenderMode::contains(
  self : DebugRenderMode,
  other : DebugRenderMode,
) -> Bool {
  ...
}

///|
#declaration_only
pub fn DebugRenderMode::insert(
  self : DebugRenderMode,
  other : DebugRenderMode,
) -> DebugRenderMode {
  ...
}

///|
#declaration_only
pub fn DebugRenderMode::remove(
  self : DebugRenderMode,
  other : DebugRenderMode,
) -> DebugRenderMode {
  ...
}

///|
pub(all) enum DebugRenderObject {
  RigidBody(@dynamics.RigidBodyHandle)
  Collider(@collision.ColliderHandle)
  ColliderAabb(@collision.ColliderHandle)
  ImpulseJoint(@dynamics.ImpulseJointHandle)
  MultibodyJoint(@dynamics.MultibodyJointHandle)
  ContactPair(@collision.ColliderHandle, @collision.ColliderHandle)
}

///|
#declaration_only
pub type DebugRenderLine

///|
#declaration_only
pub type DebugRenderBackend

///|
#declaration_only
pub fn DebugRenderBackend::new() -> DebugRenderBackend {
  ...
}

///|
#declaration_only
pub fn DebugRenderBackend::draw_line(
  self : DebugRenderBackend,
  object : DebugRenderObject,
  a : @core.Vec2,
  b : @core.Vec2,
  color : DebugColor,
) -> Unit {
  ...
}

///|
#declaration_only
pub fn DebugRenderBackend::filter_object(
  self : DebugRenderBackend,
  object : DebugRenderObject,
) -> Bool {
  ...
}

///|
#declaration_only
pub fn DebugRenderBackend::draw_polyline(
  self : DebugRenderBackend,
  object : DebugRenderObject,
  vertices : Array[@core.Vec2],
  color : DebugColor,
  closed : Bool,
) -> Unit {
  ...
}

///|
#declaration_only
pub fn DebugRenderBackend::draw_line_strip(
  self : DebugRenderBackend,
  object : DebugRenderObject,
  vertices : Array[@core.Vec2],
  color : DebugColor,
  closed : Bool,
) -> Unit {
  ...
}

///|
#declaration_only
pub fn DebugRenderBackend::lines(
  self : DebugRenderBackend,
) -> Array[DebugRenderLine] {
  ...
}

///|
#declaration_only
pub fn DebugRenderBackend::serialize_lines(self : DebugRenderBackend) -> String {
  ...
}

///|
#declaration_only
pub type DebugRenderPipeline

///|
#declaration_only
pub fn DebugRenderPipeline::new(
  style : DebugRenderStyle,
  mode : DebugRenderMode,
) -> DebugRenderPipeline {
  ...
}

///|
#declaration_only
pub fn DebugRenderPipeline::render_all(
  style : DebugRenderStyle,
) -> DebugRenderPipeline {
  ...
}

///|
#declaration_only
pub fn DebugRenderPipeline::default() -> DebugRenderPipeline {
  ...
}

///|
#declaration_only
pub fn DebugRenderPipeline::render_rigid_bodies(
  self : DebugRenderPipeline,
  backend : DebugRenderBackend,
  bodies : @dynamics.RigidBodySet,
) -> Unit {
  ...
}

///|
#declaration_only
pub fn DebugRenderPipeline::render_colliders(
  self : DebugRenderPipeline,
  backend : DebugRenderBackend,
  colliders : @collision.ColliderSet,
) -> Unit {
  ...
}

///|
#declaration_only
pub fn DebugRenderPipeline::render_joints(
  self : DebugRenderPipeline,
  backend : DebugRenderBackend,
  bodies : @dynamics.RigidBodySet,
  impulse_joints : @dynamics.ImpulseJointSet,
  multibody_joints : @dynamics.MultibodyJointSet,
) -> Unit {
  ...
}

///|
#declaration_only
pub fn DebugRenderPipeline::render_contacts(
  self : DebugRenderPipeline,
  backend : DebugRenderBackend,
  colliders : @collision.ColliderSet,
  narrow_phase : @collision.NarrowPhase,
) -> Unit {
  ...
}

///|
#declaration_only
pub fn DebugRenderPipeline::render(
  self : DebugRenderPipeline,
  backend : DebugRenderBackend,
  bodies : @dynamics.RigidBodySet,
  colliders : @collision.ColliderSet,
  impulse_joints : @dynamics.ImpulseJointSet,
  multibody_joints : @dynamics.MultibodyJointSet,
  narrow_phase : @collision.NarrowPhase,
) -> Unit {
  ...
}

///|
#declaration_only
pub fn parallel_enabled() -> Bool {
  ...
}

///|
#declaration_only
pub fn simd_enabled() -> Bool {
  ...
}

///|
#declaration_only
pub fn parallel_strategy() -> String {
  ...
}

///|
#declaration_only
pub fn simd_strategy() -> String {
  ...
}
