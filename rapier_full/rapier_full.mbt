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
pub fn smoke_scene() -> Bool {
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = @collision.ColliderSet::new()
  let impulse_joints = @dynamics.ImpulseJointSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()
  let islands = @dynamics.IslandManager::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let pipeline = @pipeline.PhysicsPipeline::new()
  let ccd = @dynamics.CCDSolver::new()
  let hooks = @pipeline.PhysicsHooks::new()
  let events = @pipeline.EventHandler::new()
  let ground = bodies.insert(@dynamics.RigidBodyBuilder::fixed().build())
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(10.0F, 0.1F).build(),
    ground,
    bodies,
  )
  |> ignore
  let ball = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(0.0F, 1.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::ball(0.5F).build(),
    ball,
    bodies,
  )
  |> ignore
  pipeline.step(
    @core.Vec2::new(0.0F, -9.81F),
    @dynamics.IntegrationParameters::default(),
    islands,
    broad_phase,
    narrow_phase,
    bodies,
    colliders,
    impulse_joints,
    multibody_joints,
    ccd,
    hooks,
    events,
  )
  if bodies.get(ball) is Some(body) {
    body.translation().y < 1.0F
  } else {
    false
  }
}

///|
pub fn smoke_query() -> Bool {
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = @collision.ColliderSet::new()
  let rb = bodies.insert(@dynamics.RigidBodyBuilder::fixed().build())
  colliders.insert_with_parent(
    @collision.ColliderBuilder::ball(0.5F).build(),
    rb,
    bodies,
  )
  |> ignore
  let query = @collision.QueryPipeline::new(
    @collision.QueryFilter::new(),
    bodies,
    colliders,
  )
  let ray = @collision.Ray::new(
    @core.Vec2::new(-1.0F, 0.0F),
    @core.Vec2::new(1.0F, 0.0F),
  )
  query.cast_ray(ray, 10.0F, true) is Some(_)
}
