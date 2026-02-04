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
test "joint solver 3dreal: prismatic motor position moves body along axis" {
  let pipeline = PhysicsPipeline3DReal::new()
  let broad_phase = @collision.BroadPhase3D::new()
  let narrow_phase = @collision.NarrowPhase3D::new()
  let islands = @dynamics.IslandManager3D::new()
  let bodies = @dynamics.RigidBodySet3D::new()
  let colliders = @collision.ColliderSet3D::new()
  let joints = @dynamics.JointSet3DReal::new()
  let gravity = @core.Vec3::zero()
  let parameters = @dynamics.IntegrationParameters::default().set_dt(
    1.0F / 60.0F,
  )
  let ground = bodies.insert(@dynamics.RigidBodyBuilder3D::fixed().build())
  let body = bodies.insert(
    @dynamics.RigidBodyBuilder3D::dynamic()
    .translation(@core.Vec3::zero())
    .can_sleep(false)
    .build(),
  )
  // Give the dynamic body a collider so broad/narrow-phase stay exercised.
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::ball(0.2F).build(),
    body,
    bodies,
  )
  |> ignore
  joints.insert_prismatic_motor_position(
    ground,
    body,
    @core.Vec3::new(1.0F, 0.0F, 0.0F),
    @core.Vec3::zero(),
    @core.Vec3::zero(),
    2.0F,
    100.0F,
    10.0F,
  )
  for _ in 0..<240 {
    pipeline.step_with_joints(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
      joints,
    )
  }
  if bodies.get(body) is Some(rb) {
    // Should have moved in +X direction toward the target.
    inspect(rb.translation().x > 1.0F, content="true")
    inspect(@core.abs(rb.translation().y) < 1.0e-2F, content="true")
    inspect(@core.abs(rb.translation().z) < 1.0e-2F, content="true")
  } else {
    inspect(false, content="true")
  }
}
