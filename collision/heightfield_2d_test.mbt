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
/// Regression test for Issue #15: heightfield must be a true primitive, not a polyline alias.
test "heightfield builder produces Shape::HeightField" {
  let heights = [0.0F, 1.0F, 0.0F]
  let scale = @core.Vec2::new(10.0F, 2.0F)
  let collider = ColliderBuilder::heightfield(heights, scale).build()
  match collider.shape() {
    Shape::HeightField(hs, sc) => {
      inspect(hs.length() == heights.length(), content="true")
      inspect(@core.abs(sc.x - 10.0F) < 1.0e-6F, content="true")
      inspect(@core.abs(sc.y - 2.0F) < 1.0e-6F, content="true")
    }
    _ => inspect(false, content="true")
  }
}

///|
test "heightfield participates in contacts and ray queries" {
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = ColliderSet::new()
  let narrow_phase = NarrowPhase::new()
  let impulse_joints = @dynamics.ImpulseJointSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()
  let ground_body = bodies.insert(@dynamics.RigidBodyBuilder::fixed().build())
  let ground = colliders.insert_with_parent(
    ColliderBuilder::heightfield([0.0F, 0.0F], @core.Vec2::new(10.0F, 1.0F)).build(),
    ground_body,
    bodies,
  )
  let ball_body = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(0.0F, 0.5F))
    .build(),
  )
  let ball = colliders.insert_with_parent(
    ColliderBuilder::ball(1.0F).build(),
    ball_body,
    bodies,
  )
  narrow_phase.update_with_pairs(
    bodies,
    colliders,
    0.0F,
    [(ground, ball)],
    impulse_joints,
    multibody_joints,
  )
  if narrow_phase.contact_pair(ground, ball) is Some(pair) {
    inspect(pair.manifold_count() > 0, content="true")
  } else {
    inspect(false, content="true")
  }
  let query_pipeline = BroadPhaseBvh::new().as_query_pipeline(
    bodies,
    colliders,
    QueryFilter::new(),
  )
  // Cast far enough from the ball so the first hit is the terrain.
  let ray = Ray::new(@core.Vec2::new(4.0F, 10.0F), @core.Vec2::new(0.0F, -1.0F))
  if query_pipeline.cast_ray_and_get_normal(
      bodies, colliders, ray, 100.0F, true,
    )
    is Some(hit) {
    inspect(ColliderHandle::equals(hit.0, ground), content="true")
    inspect(@core.abs(hit.1.toi() - 10.0F) < 1.0e-4F, content="true")
  } else {
    inspect(false, content="true")
  }
}
