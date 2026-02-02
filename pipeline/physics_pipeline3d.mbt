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
/// Real dim3 physics pipeline (integration only for now).
///
/// This is the first building block for the true 3D backend. Collision
/// detection and constraint solving will be added incrementally to match
/// rapier-reference (cfg dim3).
pub struct PhysicsPipeline3D {}

///|
pub fn PhysicsPipeline3D::new() -> PhysicsPipeline3D {
  PhysicsPipeline3D::{  }
}

///|
pub fn PhysicsPipeline3D::step(
  self : PhysicsPipeline3D,
  gravity : @core.Vec3,
  parameters : @dynamics.IntegrationParameters,
  bodies : @dynamics.RigidBodySet3D,
) -> Unit {
  self |> ignore
  let dt = parameters.dt
  bodies.integrate_all(gravity, dt)
}
