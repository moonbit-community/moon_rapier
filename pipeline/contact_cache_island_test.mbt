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
test "contact solver cache accumulates across islands" {
  fn cache_contains_pair(
    cache : Array[ContactImpulseCacheEntry],
    a : @collision.ColliderHandle,
    b : @collision.ColliderHandle,
  ) -> Bool {
    for i in 0..<cache.length() {
      let e = cache[i]
      let ab = @collision.ColliderHandle::equals(e.collider1, a) &&
        @collision.ColliderHandle::equals(e.collider2, b)
      let ba = @collision.ColliderHandle::equals(e.collider1, b) &&
        @collision.ColliderHandle::equals(e.collider2, a)
      if ab || ba {
        return true
      }
    }
    false
  }

  let pipeline = PhysicsPipeline::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = @collision.ColliderSet::new()
  let islands = @dynamics.IslandManager::new()
  let impulse_joints = @dynamics.ImpulseJointSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()
  let ground1 = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(-10.0F, -0.5F))
    .build(),
  )
  let ground2 = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(10.0F, -0.5F))
    .build(),
  )
  let ground_co1 = colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(5.0F, 0.5F).build(),
    ground1,
    bodies,
  )
  let ground_co2 = colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(5.0F, 0.5F).build(),
    ground2,
    bodies,
  )
  let ball1 = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(-10.0F, 0.25F))
    .build(),
  )
  let ball2 = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(10.0F, 0.25F))
    .build(),
  )
  let ball_co1 = colliders.insert_with_parent(
    @collision.ColliderBuilder::ball(0.5F).build(),
    ball1,
    bodies,
  )
  let ball_co2 = colliders.insert_with_parent(
    @collision.ColliderBuilder::ball(0.5F).build(),
    ball2,
    bodies,
  )
  inspect(ground_co1.id >= 0, content="true")
  inspect(ground_co2.id >= 0, content="true")
  inspect(ball_co1.id >= 0, content="true")
  inspect(ball_co2.id >= 0, content="true")
  pipeline.step(
    @core.Vec2::zero(),
    IntegrationParameters::default(),
    islands,
    broad_phase,
    narrow_phase,
    bodies,
    colliders,
    impulse_joints,
    multibody_joints,
    @dynamics.CCDSolver::new(),
    PhysicsHooks::new(),
    EventHandler::new(),
  )

  // The solver cache should contain entries for both disjoint islands, not just the last one.
  let cache = pipeline.contact_solver_cache
  inspect(cache_contains_pair(cache, ground_co1, ball_co1), content="true")
  inspect(cache_contains_pair(cache, ground_co2, ball_co2), content="true")
}
