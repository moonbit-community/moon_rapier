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
test "query pipeline cast_ray_and_get_normal returns toi and outward normal" {
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = ColliderSet::new()
  let body = bodies.insert(@dynamics.RigidBodyBuilder::dynamic().build())
  let ball = ColliderBuilder::ball(1.0F).build()
  let handle = colliders.insert_with_parent(ball, body, bodies)
  let query_pipeline = BroadPhaseBvh::new().as_query_pipeline(
    bodies,
    colliders,
    QueryFilter::new(),
  )
  let ray = Ray::new(@core.Vec2::new(-10.0F, 0.0F), @core.Vec2::new(1.0F, 0.0F))
  if query_pipeline.cast_ray_and_get_normal(ray, 100.0F, true) is Some(hit) {
    inspect(ColliderHandle::equals(hit.0, handle), content="true")
    inspect(@core.abs(hit.1.toi() - 9.0F) < 1.0e-4F, content="true")
    let n = hit.1.normal()
    inspect(@core.abs(n.x + 1.0F) < 1.0e-4F, content="true")
    inspect(@core.abs(n.y) < 1.0e-4F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "query pipeline cast_ray_and_get_normal inside solid vs hollow" {
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = ColliderSet::new()
  let body = bodies.insert(@dynamics.RigidBodyBuilder::dynamic().build())
  colliders.insert_with_parent(
    ColliderBuilder::ball(1.0F).build(),
    body,
    bodies,
  )
  |> ignore
  let query_pipeline = BroadPhaseBvh::new().as_query_pipeline(
    bodies,
    colliders,
    QueryFilter::new(),
  )
  let ray = Ray::new(@core.Vec2::new(0.0F, 0.0F), @core.Vec2::new(1.0F, 0.0F))
  if query_pipeline.cast_ray_and_get_normal(ray, 100.0F, true)
    is Some(hit_solid) {
    inspect(@core.abs(hit_solid.1.toi()) < 1.0e-6F, content="true")
    let n = hit_solid.1.normal()
    // For a solid ray cast starting inside, Rapier returns a normal opposing the ray direction.
    inspect(@core.abs(n.x + 1.0F) < 1.0e-4F, content="true")
    inspect(@core.abs(n.y) < 1.0e-4F, content="true")
  } else {
    inspect(false, content="true")
  }
  if query_pipeline.cast_ray_and_get_normal(ray, 100.0F, false)
    is Some(hit_hollow) {
    inspect(@core.abs(hit_hollow.1.toi() - 1.0F) < 1.0e-4F, content="true")
    let n = hit_hollow.1.normal()
    inspect(@core.abs(n.x - 1.0F) < 1.0e-4F, content="true")
    inspect(@core.abs(n.y) < 1.0e-4F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "query pipeline cast_shape ball hits ball" {
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = ColliderSet::new()
  let body = bodies.insert(@dynamics.RigidBodyBuilder::dynamic().build())
  let handle = colliders.insert_with_parent(
    ColliderBuilder::ball(1.0F).build(),
    body,
    bodies,
  )
  let query_pipeline = BroadPhaseBvh::new().as_query_pipeline(
    bodies,
    colliders,
    QueryFilter::new(),
  )
  let shape_pos = @core.Isometry2::from_translation(
    @core.Vec2::new(-10.0F, 0.0F),
  )
  let vel = @core.Vec2::new(1.0F, 0.0F)
  let options = ShapeCastOptions::new(100.0F, true)
  if query_pipeline.cast_shape(shape_pos, vel, Shape::Ball(1.0F), options)
    is Some(hit) {
    inspect(ColliderHandle::equals(hit.0, handle), content="true")
    inspect(@core.abs(hit.1.toi() - 8.0F) < 1.0e-4F, content="true")
    let n = hit.1.normal()
    inspect(@core.abs(n.x + 1.0F) < 1.0e-4F, content="true")
    inspect(@core.abs(n.y) < 1.0e-4F, content="true")
    let p = hit.1.point()
    inspect(@core.abs(p.x + 1.0F) < 1.0e-4F, content="true")
    inspect(@core.abs(p.y) < 1.0e-4F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "query pipeline cast_shape ball hits rotated cuboid" {
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = ColliderSet::new()
  let body = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .rotation(@core.Rot2::from_angle(@core.pi() * 0.5F))
    .build(),
  )
  let handle = colliders.insert_with_parent(
    ColliderBuilder::cuboid(2.0F, 0.5F).build(),
    body,
    bodies,
  )
  let query_pipeline = BroadPhaseBvh::new().as_query_pipeline(
    bodies,
    colliders,
    QueryFilter::new(),
  )
  let shape_pos = @core.Isometry2::from_translation(
    @core.Vec2::new(-10.0F, 0.0F),
  )
  let vel = @core.Vec2::new(1.0F, 0.0F)
  let options = ShapeCastOptions::new(100.0F, true)
  // Ball radius 0.5 against a 90deg-rotated cuboid with world x half-extent 0.5:
  // expanded boundary is at x = -(0.5 + 0.5) = -1.0, so toi = 9.0.
  if query_pipeline.cast_shape(shape_pos, vel, Shape::Ball(0.5F), options)
    is Some(hit) {
    inspect(ColliderHandle::equals(hit.0, handle), content="true")
    inspect(@core.abs(hit.1.toi() - 9.0F) < 1.0e-4F, content="true")
    let n = hit.1.normal()
    inspect(@core.abs(n.x + 1.0F) < 1.0e-4F, content="true")
    inspect(@core.abs(n.y) < 1.0e-4F, content="true")
    let p = hit.1.point()
    inspect(@core.abs(p.x + 0.5F) < 1.0e-4F, content="true")
    inspect(@core.abs(p.y) < 1.0e-4F, content="true")
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
test "query pipeline cast_ray hits capsule" {
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = ColliderSet::new()
  let body = bodies.insert(@dynamics.RigidBodyBuilder::dynamic().build())
  let capsule = ColliderBuilder::capsule_y(1.0F, 0.5F).build()
  let handle = colliders.insert_with_parent(capsule, body, bodies)
  let query_pipeline = BroadPhaseBvh::new().as_query_pipeline(
    bodies,
    colliders,
    QueryFilter::new(),
  )
  let ray = Ray::new(@core.Vec2::new(-10.0F, 0.0F), @core.Vec2::new(1.0F, 0.0F))
  if query_pipeline.cast_ray_and_get_normal(ray, 100.0F, true) is Some(hit) {
    inspect(ColliderHandle::equals(hit.0, handle), content="true")
    inspect(@core.abs(hit.1.toi() - 9.5F) < 1.0e-4F, content="true")
    let n = hit.1.normal()
    inspect(@core.abs(n.x + 1.0F) < 1.0e-4F, content="true")
    inspect(@core.abs(n.y) < 1.0e-4F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "query pipeline cast_shape capsule hits ball" {
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = ColliderSet::new()
  let body = bodies.insert(@dynamics.RigidBodyBuilder::dynamic().build())
  let handle = colliders.insert_with_parent(
    ColliderBuilder::ball(0.5F).build(),
    body,
    bodies,
  )
  let query_pipeline = BroadPhaseBvh::new().as_query_pipeline(
    bodies,
    colliders,
    QueryFilter::new(),
  )
  let shape_pos = @core.Isometry2::from_translation(
    @core.Vec2::new(-10.0F, 0.0F),
  )
  let vel = @core.Vec2::new(1.0F, 0.0F)
  let options = ShapeCastOptions::new(100.0F, true)
  if query_pipeline.cast_shape(
      shape_pos,
      vel,
      Shape::CapsuleY(1.0F, 0.5F),
      options,
    )
    is Some(hit) {
    inspect(ColliderHandle::equals(hit.0, handle), content="true")
    inspect(@core.abs(hit.1.toi() - 9.0F) < 1.0e-4F, content="true")
    let n = hit.1.normal()
    inspect(@core.abs(n.x + 1.0F) < 1.0e-4F, content="true")
    inspect(@core.abs(n.y) < 1.0e-4F, content="true")
    let p = hit.1.point()
    inspect(@core.abs(p.x + 0.5F) < 1.0e-4F, content="true")
    inspect(@core.abs(p.y) < 1.0e-4F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "query pipeline cast_ray hits segment" {
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = ColliderSet::new()
  let body = bodies.insert(@dynamics.RigidBodyBuilder::dynamic().build())
  let seg = ColliderBuilder::segment(
    @core.Vec2::new(-1.0F, 0.0F),
    @core.Vec2::new(1.0F, 0.0F),
  ).build()
  let handle = colliders.insert_with_parent(seg, body, bodies)
  let query_pipeline = BroadPhaseBvh::new().as_query_pipeline(
    bodies,
    colliders,
    QueryFilter::new(),
  )
  let ray = Ray::new(@core.Vec2::new(0.0F, -1.0F), @core.Vec2::new(0.0F, 1.0F))
  if query_pipeline.cast_ray_and_get_normal(ray, 10.0F, true) is Some(hit) {
    inspect(ColliderHandle::equals(hit.0, handle), content="true")
    inspect(@core.abs(hit.1.toi() - 1.0F) < 1.0e-4F, content="true")
    let n = hit.1.normal()
    inspect(@core.abs(n.x) < 1.0e-4F, content="true")
    inspect(n.y < -0.9F, content="true")
  } else {
    inspect(false, content="true")
  }
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
test "query pipeline project_point ignores max_dist and matches solid semantics" {
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = ColliderSet::new()
  let body = bodies.insert(@dynamics.RigidBodyBuilder::dynamic().build())
  let handle = colliders.insert_with_parent(
    ColliderBuilder::ball(1.0F).build(),
    body,
    bodies,
  )
  let query_pipeline = BroadPhaseBvh::new().as_query_pipeline(
    bodies,
    colliders,
    QueryFilter::new(),
  )

  // Outside point; max_dist is ignored.
  if query_pipeline.project_point(@core.Vec2::new(2.0F, 0.0F), 0.01F, true)
    is Some(hit) {
    inspect(ColliderHandle::equals(hit.0, handle), content="true")
    inspect(@core.abs(hit.1.point().x - 1.0F) < 1.0e-4F, content="true")
    inspect(@core.abs(hit.1.point().y) < 1.0e-4F, content="true")
    inspect(hit.1.is_inside() == false, content="true")
  } else {
    inspect(false, content="true")
  }

  // Inside point with solid=true projects to itself.
  if query_pipeline.project_point(@core.Vec2::new(0.2F, 0.0F), 0.01F, true)
    is Some(hit) {
    inspect(@core.abs(hit.1.point().x - 0.2F) < 1.0e-4F, content="true")
    inspect(hit.1.is_inside() == true, content="true")
  } else {
    inspect(false, content="true")
  }

  // Inside point with solid=false projects to the boundary.
  if query_pipeline.project_point(@core.Vec2::new(0.2F, 0.0F), 0.01F, false)
    is Some(hit) {
    inspect(@core.abs(hit.1.point().x - 1.0F) < 1.0e-4F, content="true")
    inspect(hit.1.is_inside() == true, content="true")
  } else {
    inspect(false, content="true")
  }
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
test "query pipeline intersect_aabb_conservative uses cached aabbs" {
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = ColliderSet::new()
  let body1 = bodies.insert(@dynamics.RigidBodyBuilder::dynamic().build())
  let body2 = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(10.0F, 0.0F))
    .build(),
  )
  let h1 = colliders.insert_with_parent(
    ColliderBuilder::ball(1.0F).build(),
    body1,
    bodies,
  )
  colliders.insert_with_parent(
    ColliderBuilder::ball(1.0F).build(),
    body2,
    bodies,
  )
  |> ignore
  let broad_phase = BroadPhaseBvh::new()
  let pipeline = broad_phase.as_query_pipeline(
    bodies,
    colliders,
    QueryFilter::new(),
  )
  let query = @core.Aabb::new(
    @core.Vec2::new(-2.0F, -2.0F),
    @core.Vec2::new(2.0F, 2.0F),
  )
  let results = pipeline.intersect_aabb_conservative(query)
  inspect(results.length() == 1, content="true")
  inspect(ColliderHandle::equals(results[0], h1), content="true")

  // Move the collider far away without recreating the pipeline. Conservative AABB queries
  // must still use the cached AABBs from the pipeline's creation.
  if bodies.get_mut(body1) is Some(b) {
    b.set_translation(@core.Vec2::new(100.0F, 0.0F), true) |> ignore
  }
  broad_phase.update(0.0F, bodies, pipeline.colliders)
  let results_after = pipeline.intersect_aabb_conservative(query)
  inspect(results_after.length() == 1, content="true")
  inspect(ColliderHandle::equals(results_after[0], h1), content="true")
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
