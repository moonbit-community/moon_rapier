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
test "physics_pipeline3d step integrates bodies" {
  let pipeline = PhysicsPipeline3D::new()
  let gravity = @core.Vec3::new(0.0F, -9.81F, 0.0F)
  let parameters = @dynamics.IntegrationParameters::default().set_dt(0.1F)
  let bodies = @dynamics.RigidBodySet3D::new()
  let handle = bodies.insert(
    @dynamics.RigidBodyBuilder3D::dynamic()
    .translation(@core.Vec3::new(0.0F, 1.0F, 2.0F))
    .build(),
  )
  pipeline.step(gravity, parameters, bodies)
  if bodies.get(handle) is Some(rb) {
    inspect(rb.translation().y < 1.0F, content="true")
    inspect(rb.translation().z == 2.0F, content="true")
  } else {
    inspect(false, content="true")
  }
}
