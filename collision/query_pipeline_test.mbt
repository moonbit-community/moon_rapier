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

///|
test "query pipeline cast_ray respects cuboid rotation" {
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = ColliderSet::new()
  let body = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .rotation(@core.Rot2::from_angle(@core.pi() * 0.5F))
    .build(),
  )
  let cuboid = ColliderBuilder::cuboid(2.0F, 0.5F).build()
  let handle = colliders.insert_with_parent(cuboid, body, bodies)
  let broad_phase = BroadPhaseBvh::new()
  let query_pipeline = broad_phase.as_query_pipeline(
    bodies,
    colliders,
    QueryFilter::new(),
  )
  let ray = Ray::new(@core.Vec2::new(-10.0F, 0.0F), @core.Vec2::new(1.0F, 0.0F))
  if query_pipeline.cast_ray(ray, 100.0F, true) is Some(hit) {
    inspect(ColliderHandle::equals(hit.0, handle), content="true")
    // Rotated cuboid has world half-extent 0.5 along x, so it hits at x = -0.5.
    inspect(@core.abs(hit.1 - 9.5F) < 1.0e-4F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "query pipeline intersect_point respects cuboid rotation" {
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = ColliderSet::new()
  let body = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .rotation(@core.Rot2::from_angle(@core.pi() * 0.5F))
    .build(),
  )
  colliders.insert_with_parent(
    ColliderBuilder::cuboid(2.0F, 0.5F).build(),
    body,
    bodies,
  )
  |> ignore
  let broad_phase = BroadPhaseBvh::new()
  let query_pipeline = broad_phase.as_query_pipeline(
    bodies,
    colliders,
    QueryFilter::new(),
  )
  // This point is inside the AABB of the cuboid but outside the rotated cuboid itself.
  let results = query_pipeline.intersect_point(@core.Vec2::new(1.2F, 0.0F))
  inspect(results.length() == 0, content="true")
}

///|
test "query filter exclude sensors vs solids" {
  let ray = Ray::new(@core.Vec2::new(-10.0F, 0.0F), @core.Vec2::new(1.0F, 0.0F))

  // exclude_sensors: should hit the solid collider, not the sensor.
  {
    let bodies = @dynamics.RigidBodySet::new()
    let colliders = ColliderSet::new()
    let b1 = bodies.insert(@dynamics.RigidBodyBuilder::dynamic().build())
    let b2 = bodies.insert(
      @dynamics.RigidBodyBuilder::dynamic()
      .translation(@core.Vec2::new(5.0F, 0.0F))
      .build(),
    )
    let sensor = ColliderBuilder::ball(0.5F).sensor(true).build()
    let solid = ColliderBuilder::ball(0.5F).sensor(false).build()
    colliders.insert_with_parent(sensor, b1, bodies) |> ignore
    let solid_handle = colliders.insert_with_parent(solid, b2, bodies)
    let pipeline = BroadPhaseBvh::new().as_query_pipeline(
      bodies,
      colliders,
      QueryFilter::new().exclude_sensors(),
    )
    if pipeline.cast_ray(ray, 100.0F, true) is Some(hit) {
      inspect(ColliderHandle::equals(hit.0, solid_handle), content="true")
    } else {
      inspect(false, content="true")
    }
  }

  // exclude_solids: should hit the sensor collider, not the solid.
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = ColliderSet::new()
  let b1 = bodies.insert(@dynamics.RigidBodyBuilder::dynamic().build())
  let b2 = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(5.0F, 0.0F))
    .build(),
  )
  let sensor = ColliderBuilder::ball(0.5F).sensor(true).build()
  let solid = ColliderBuilder::ball(0.5F).sensor(false).build()
  let sensor_handle = colliders.insert_with_parent(sensor, b1, bodies)
  colliders.insert_with_parent(solid, b2, bodies) |> ignore
  let pipeline = BroadPhaseBvh::new().as_query_pipeline(
    bodies,
    colliders,
    QueryFilter::new().exclude_solids(),
  )
  if pipeline.cast_ray(ray, 100.0F, true) is Some(hit) {
    inspect(ColliderHandle::equals(hit.0, sensor_handle), content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "query filter only_dynamic ignores fixed colliders" {
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = ColliderSet::new()
  let fixed_body = bodies.insert(@dynamics.RigidBodyBuilder::fixed().build())
  let dynamic_body = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(5.0F, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    ColliderBuilder::ball(0.5F).build(),
    fixed_body,
    bodies,
  )
  |> ignore
  let dynamic_collider = colliders.insert_with_parent(
    ColliderBuilder::ball(0.5F).build(),
    dynamic_body,
    bodies,
  )
  let query_pipeline = BroadPhaseBvh::new().as_query_pipeline(
    bodies,
    colliders,
    QueryFilter::only_dynamic(),
  )
  let ray = Ray::new(@core.Vec2::new(-10.0F, 0.0F), @core.Vec2::new(1.0F, 0.0F))
  if query_pipeline.cast_ray(ray, 100.0F, true) is Some(hit) {
    inspect(ColliderHandle::equals(hit.0, dynamic_collider), content="true")
  } else {
    inspect(false, content="true")
  }
}
