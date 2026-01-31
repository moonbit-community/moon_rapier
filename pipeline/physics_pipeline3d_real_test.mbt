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
test "physics_pipeline3d_real: ball rests on ground" {
  let pipeline = PhysicsPipeline3DReal::new()
  let broad_phase = @collision.BroadPhase3D::new()
  let narrow_phase = @collision.NarrowPhase3D::new()
  let bodies = @dynamics.RigidBodySet3D::new()
  let colliders = @collision.ColliderSet3D::new()
  let gravity = @core.Vec3::new(0.0F, -9.81F, 0.0F)
  let parameters = @dynamics.IntegrationParameters::default().set_dt(
    1.0F / 60.0F,
  )
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder3D::fixed()
    .translation(@core.Vec3::new(0.0F, -1.0F, 0.0F))
    .build(),
  )
  let ground_collider = colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(10.0F, 1.0F, 10.0F).build(),
    ground,
    bodies,
  )
  let ball = bodies.insert(
    @dynamics.RigidBodyBuilder3D::dynamic()
    .translation(@core.Vec3::new(0.0F, 5.0F, 0.0F))
    .build(),
  )
  let ball_collider = colliders.insert_with_parent(
    @collision.ColliderBuilder3D::ball(0.5F).build(),
    ball,
    bodies,
  )
  for _ in 0..<240 {
    pipeline.step(
      gravity, parameters, broad_phase, narrow_phase, bodies, colliders,
    )
  }

  // Ball should not tunnel far below the ground top face (y=0).
  if bodies.get(ball) is Some(rb) {
    inspect(rb.translation().y > -0.2F, content="true")
  } else {
    inspect(false, content="true")
  }

  // We should have a contact pair between ball and ground at the end.
  if narrow_phase.contact_pair(ball_collider, ground_collider) is Some(pair) {
    inspect(pair.manifolds_len() > 0, content="true")
  } else {
    // The simulation is expected to reach the ground; treat missing pair as failure.
    inspect(false, content="true")
  }
}
