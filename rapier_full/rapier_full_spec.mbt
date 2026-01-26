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
/// Rapier full-API spec scaffold (2D).
/// This file defines the public contract that we will expand toward
/// rapier-reference parity. Implementation lives in rapier_full.mbt.
pub type RigidBodySet = @dynamics.RigidBodySet

///|
pub type ColliderSet = @collision.ColliderSet

///|
pub type ImpulseJointSet = @dynamics.ImpulseJointSet

///|
pub type MultibodyJointSet = @dynamics.MultibodyJointSet

///|
pub type IslandManager = @dynamics.IslandManager

///|
pub type BroadPhaseBvh = @collision.BroadPhaseBvh

///|
pub type NarrowPhase = @collision.NarrowPhase

///|
pub type PhysicsPipeline = @pipeline.PhysicsPipeline

///|
pub type CCDSolver = @dynamics.CCDSolver

///|
pub type IntegrationParameters = @dynamics.IntegrationParameters

///|
pub type PhysicsHooks = @pipeline.PhysicsHooks

///|
pub type EventHandler = @pipeline.EventHandler

///|
pub type QueryPipeline = @collision.QueryPipeline

///|
pub type QueryFilter = @collision.QueryFilter

///|
pub type Ray = @collision.Ray

///|
pub type Shape = @collision.Shape

///|
pub type ColliderBuilder = @collision.ColliderBuilder

///|
pub type RigidBodyBuilder = @dynamics.RigidBodyBuilder

///|
pub type KinematicCharacterController = @control.KinematicCharacterController

///|
#declaration_only
pub fn smoke_scene() -> Bool {
  ...
}

///|
#declaration_only
pub fn smoke_query() -> Bool {
  ...
}
