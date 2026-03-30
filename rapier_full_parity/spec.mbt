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
/// Aggregate "prelude-style" aliases for portability when porting Rapier examples/tests.
/// This package is intentionally focused and grows with parity tests.
pub type Real = @core.Real

///|
pub type Vec2 = @core.Vec2

///|
pub type Rot2 = @core.Rot2

///|
pub type Isometry2 = @core.Isometry2

///|
pub type ColliderSet = @collision.ColliderSet

///|
pub type ColliderBuilder = @collision.ColliderBuilder

///|
pub type RigidBodySet = @dynamics.RigidBodySet

///|
pub type RigidBodyBuilder = @dynamics.RigidBodyBuilder

///|
pub type ImpulseJointSet = @dynamics.ImpulseJointSet

///|
pub type MultibodyJointSet = @dynamics.MultibodyJointSet

///|
pub type GenericJoint = @dynamics.GenericJoint

///|
pub type RevoluteJointBuilder = @dynamics.RevoluteJointBuilder

///|
pub type IntegrationParameters = @dynamics.IntegrationParameters

///|
pub type PhysicsPipeline = @pipeline.PhysicsPipeline

///|
pub type PhysicsHooks = @pipeline.PhysicsHooks

///|
pub type EventHandler = @pipeline.EventHandler
