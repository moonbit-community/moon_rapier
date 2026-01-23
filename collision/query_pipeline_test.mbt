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
test "query pipeline cast_ray hits nearest collider" {
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = ColliderSet::new()
  let h1 = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(0.0F, 0.0F))
    .build(),
  )
  let h2 = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(5.0F, 0.0F))
    .build(),
  )
  let c1 = ColliderBuilder::ball(0.5F).build()
  let c2 = ColliderBuilder::ball(0.5F).build()
  let ch1 = colliders.insert_with_parent(c1, h1, bodies)
  let ch2 = colliders.insert_with_parent(c2, h2, bodies)
  let broad_phase = BroadPhaseBvh::new()
  let query_pipeline = broad_phase.as_query_pipeline(
    bodies,
    colliders,
    QueryFilter::new(),
  )
  let ray = Ray::new(@core.Vec2::new(-10.0F, 0.0F), @core.Vec2::new(1.0F, 0.0F))
  if query_pipeline.cast_ray(ray, 100.0F, true) is Some(hit) {
    inspect(ColliderHandle::equals(hit.0, ch1), content="true")
    inspect(!ColliderHandle::equals(hit.0, ch2), content="true")
    inspect(@core.abs(hit.1 - 9.5F) < 1.0e-4F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "query pipeline cast_ray solid vs hollow" {
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = ColliderSet::new()
  let h1 = bodies.insert(@dynamics.RigidBodyBuilder::dynamic().build())
  let ch1 = colliders.insert_with_parent(
    ColliderBuilder::ball(1.0F).build(),
    h1,
    bodies,
  )
  let broad_phase = BroadPhaseBvh::new()
  let query_pipeline = broad_phase.as_query_pipeline(
    bodies,
    colliders,
    QueryFilter::new(),
  )
  let ray = Ray::new(@core.Vec2::new(0.0F, 0.0F), @core.Vec2::new(1.0F, 0.0F))
  if query_pipeline.cast_ray(ray, 10.0F, true) is Some(hit_solid) {
    inspect(ColliderHandle::equals(hit_solid.0, ch1), content="true")
    inspect(@core.abs(hit_solid.1) < 1.0e-6F, content="true")
  } else {
    inspect(false, content="true")
  }
  if query_pipeline.cast_ray(ray, 10.0F, false) is Some(hit_hollow) {
    inspect(ColliderHandle::equals(hit_hollow.0, ch1), content="true")
    inspect(@core.abs(hit_hollow.1 - 1.0F) < 1.0e-4F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "query pipeline filter exclude rigid body" {
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = ColliderSet::new()
  let body1 = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(0.0F, 0.0F))
    .build(),
  )
  let body2 = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(5.0F, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    ColliderBuilder::ball(0.5F).build(),
    body1,
    bodies,
  )
  |> ignore
  let ch2 = colliders.insert_with_parent(
    ColliderBuilder::ball(0.5F).build(),
    body2,
    bodies,
  )
  let filter = QueryFilter::new().exclude_rigid_body(body1)
  let broad_phase = BroadPhaseBvh::new()
  let query_pipeline = broad_phase.as_query_pipeline(bodies, colliders, filter)
  let ray = Ray::new(@core.Vec2::new(-10.0F, 0.0F), @core.Vec2::new(1.0F, 0.0F))
  if query_pipeline.cast_ray(ray, 100.0F, true) is Some(hit) {
    inspect(ColliderHandle::equals(hit.0, ch2), content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "query pipeline filter collision groups" {
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = ColliderSet::new()
  let h1 = bodies.insert(@dynamics.RigidBodyBuilder::dynamic().build())
  let groups = @dynamics.InteractionGroups::new(
    @dynamics.Group::group_1(),
    @dynamics.Group::group_1(),
    @dynamics.InteractionTestMode::And,
  )
  let collider = ColliderBuilder::ball(1.0F).collision_groups(groups).build()
  colliders.insert_with_parent(collider, h1, bodies) |> ignore
  let filter_groups = @dynamics.InteractionGroups::new(
    @dynamics.Group::group_2(),
    @dynamics.Group::group_2(),
    @dynamics.InteractionTestMode::And,
  )
  let filter = QueryFilter::new().groups(filter_groups)
  let broad_phase = BroadPhaseBvh::new()
  let query_pipeline = broad_phase.as_query_pipeline(bodies, colliders, filter)
  let ray = Ray::new(@core.Vec2::new(-10.0F, 0.0F), @core.Vec2::new(1.0F, 0.0F))
  inspect(query_pipeline.cast_ray(ray, 100.0F, true) is None, content="true")
}

///|
test "query pipeline intersect_point" {
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = ColliderSet::new()
  let h1 = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(1.0F, 0.0F))
    .build(),
  )
  let h2 = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(10.0F, 0.0F))
    .build(),
  )
  let ch1 = colliders.insert_with_parent(
    ColliderBuilder::ball(1.0F).build(),
    h1,
    bodies,
  )
  colliders.insert_with_parent(ColliderBuilder::ball(1.0F).build(), h2, bodies)
  |> ignore
  let broad_phase = BroadPhaseBvh::new()
  let query_pipeline = broad_phase.as_query_pipeline(
    bodies,
    colliders,
    QueryFilter::new(),
  )
  let results = query_pipeline.intersect_point(@core.Vec2::new(1.0F, 0.0F))
  inspect(results.length() == 1, content="true")
  inspect(ColliderHandle::equals(results[0], ch1), content="true")
}
