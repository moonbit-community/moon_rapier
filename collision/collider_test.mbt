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
/// Ported from rapier collider position setters.
test "collider position setters" {
  let collider = ColliderBuilder::ball(1.0F).build()
  collider.set_translation(@core.Vec2::new(1.0F, -2.0F)) |> ignore
  collider.set_rotation(1.5F) |> ignore
  inspect(collider.translation().x == 1.0F, content="true")
  inspect(collider.translation().y == -2.0F, content="true")
  inspect(collider.rotation() == 1.5F, content="true")
  let position = collider.position()
  inspect(position.translation.x == 1.0F, content="true")
  inspect(position.translation.y == -2.0F, content="true")
  inspect(position.rotation.angle() == 1.5F, content="true")
}

///|
/// Ported from rapier collider shape setters.
test "collider shape setters" {
  let collider = ColliderBuilder::ball(1.0F).build()
  let collider = collider.set_shape(Shape::Cuboid(0.5F, 1.25F))
  if collider.shape() is Shape::Cuboid(half_width, half_height) {
    inspect(half_width == 0.5F, content="true")
    inspect(half_height == 1.25F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
/// Ported from rapier interaction groups.
test "interaction groups behavior" {
  let group_a = @dynamics.InteractionGroups::new(
    @dynamics.Group::group_1(),
    @dynamics.Group::group_2(),
    @dynamics.InteractionTestMode::And,
  )
  let group_b = @dynamics.InteractionGroups::new(
    @dynamics.Group::group_2(),
    @dynamics.Group::group_1(),
    @dynamics.InteractionTestMode::And,
  )
  inspect(group_a.test_groups(group_b), content="true")
  let group_c = @dynamics.InteractionGroups::new(
    @dynamics.Group::group_1(),
    @dynamics.Group::group_2(),
    @dynamics.InteractionTestMode::Or,
  )
  let group_d = @dynamics.InteractionGroups::new(
    @dynamics.Group::group_3(),
    @dynamics.Group::group_1(),
    @dynamics.InteractionTestMode::Or,
  )
  inspect(group_c.test_groups(group_d), content="true")
  let customized = @dynamics.InteractionGroups::default()
    .with_memberships(@dynamics.Group::group_4())
    .with_filter(@dynamics.Group::group_5())
  inspect(
    customized.memberships().equals(@dynamics.Group::group_4()),
    content="true",
  )
  inspect(
    customized.filter().equals(@dynamics.Group::group_5()),
    content="true",
  )
}

///|
/// Ported from rapier collider group setters.
test "collider groups accessors" {
  let collision_groups = @dynamics.InteractionGroups::new(
    @dynamics.Group::group_1(),
    @dynamics.Group::group_2(),
    @dynamics.InteractionTestMode::And,
  )
  let solver_groups = @dynamics.InteractionGroups::none()
  let collider = ColliderBuilder::ball(1.0F)
    .collision_groups(collision_groups)
    .solver_groups(solver_groups)
    .build()
  let collision = collider.collision_groups()
  inspect(
    collision.memberships().equals(@dynamics.Group::group_1()),
    content="true",
  )
  inspect(collision.filter().equals(@dynamics.Group::group_2()), content="true")
  let solver = collider.solver_groups()
  inspect(solver.memberships().equals(@dynamics.Group::none()), content="true")
  let collider = collider.set_collision_groups(
    @dynamics.InteractionGroups::all(),
  )
  inspect(
    collider.collision_groups().test_mode()
    is @dynamics.InteractionTestMode::And,
    content="true",
  )
  let collider = collider.set_solver_groups(
    @dynamics.InteractionGroups::default().with_filter(
      @dynamics.Group::group_3(),
    ),
  )
  inspect(
    collider.solver_groups().filter().equals(@dynamics.Group::group_3()),
    content="true",
  )
}

///|
test "collider builder: triangle + round_triangle" {
  let a = @core.Vec2::new(0.0F, 0.0F)
  let b = @core.Vec2::new(2.0F, 0.0F)
  let c = @core.Vec2::new(0.0F, 1.0F)
  let tri = ColliderBuilder::triangle(a, b, c).build()
  match tri.shape() {
    Shape::ConvexPolygon(vertices) =>
      inspect(vertices.length() >= 3, content="true")
    Shape::Polyline(_, _) => inspect(true, content="true")
    Shape::Segment(_, _) => inspect(true, content="true")
    _ => inspect(false, content="true")
  }
  let round = ColliderBuilder::round_triangle(a, b, c, 0.2F).build()
  inspect(round.contact_skin() == 0.0F, content="true")
  match round.shape() {
    Shape::Round(_, r) => inspect(@core.abs(r - 0.2F) < 1.0e-6F, content="true")
    _ => inspect(false, content="true")
  }
}

///|
test "collider builder: halfspace approximation" {
  let ground = ColliderBuilder::halfspace(@core.Vec2::new(0.0F, 1.0F)).build()
  if ground.shape() is Shape::Cuboid(hw, hh) {
    inspect(hw > 1.0F, content="true")
    inspect(hh > 1.0F, content="true")
    // The approximation shifts the cuboid so the supporting face is near the origin.
    inspect(ground.translation().y < 0.0F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "collider builder: compound + round_convex_hull" {
  let parts = [
    (@core.Isometry2::identity(), Shape::Cuboid(1.0F, 0.2F)),
    (
      @core.Isometry2::from_translation(@core.Vec2::new(2.0F, 0.0F)),
      Shape::Cuboid(0.5F, 0.5F),
    ),
  ]
  let co = ColliderBuilder::compound(parts).build()
  if co.shape() is Shape::Compound(compound_parts) {
    inspect(compound_parts.length() == parts.length(), content="true")
    let p1 = compound_parts[1].0.transform_point(@core.Vec2::zero())
    inspect(@core.abs(p1.x - 2.0F) < 1.0e-6F, content="true")
    inspect(@core.abs(p1.y - 0.0F) < 1.0e-6F, content="true")
    match compound_parts[0].1 {
      Shape::Cuboid(_, _) => inspect(true, content="true")
      _ => inspect(false, content="true")
    }
  } else {
    inspect(false, content="true")
  }
  let pts = [
    @core.Vec2::new(0.0F, 0.0F),
    @core.Vec2::new(2.0F, 0.0F),
    @core.Vec2::new(2.0F, 1.0F),
    @core.Vec2::new(0.0F, 1.0F),
  ]
  if ColliderBuilder::round_convex_hull(pts, 0.3F) is Some(builder) {
    let collider = builder.build()
    inspect(collider.contact_skin() == 0.0F, content="true")
    match collider.shape() {
      Shape::Round(_, r) =>
        inspect(@core.abs(r - 0.3F) < 1.0e-6F, content="true")
      _ => inspect(false, content="true")
    }
  } else {
    inspect(false, content="true")
  }
}

///|
/// Ported from rapier collider material and event accessors.
test "collider material and event accessors" {
  let collider = ColliderBuilder::ball(1.0F)
    .friction(0.75F)
    .friction_combine_rule(@dynamics.CoefficientCombineRule::Max)
    .restitution(0.25F)
    .restitution_combine_rule(@dynamics.CoefficientCombineRule::Min)
    .contact_skin(0.1F)
    .contact_force_event_threshold(3.0F)
    .user_data(7)
    .active_hooks(ActiveHooks::filter_contact_pairs())
    .active_events(ActiveEvents::collision_events())
    .build()
  inspect(collider.friction() == 0.75F, content="true")
  inspect(
    collider.friction_combine_rule() is @dynamics.CoefficientCombineRule::Max,
    content="true",
  )
  inspect(collider.restitution() == 0.25F, content="true")
  inspect(
    collider.restitution_combine_rule() is @dynamics.CoefficientCombineRule::Min,
    content="true",
  )
  inspect(collider.contact_skin() == 0.1F, content="true")
  inspect(collider.contact_force_event_threshold() == 3.0F, content="true")
  inspect(collider.user_data() == 7, content="true")
  inspect(
    collider.active_hooks().contains(ActiveHooks::filter_contact_pairs()),
    content="true",
  )
  inspect(
    collider.active_events().contains(ActiveEvents::collision_events()),
    content="true",
  )
  inspect(
    collider.active_collision_types().equals(ActiveCollisionTypes::default()),
    content="true",
  )
  let collider = collider.set_restitution(0.5F)
  inspect(collider.restitution() == 0.5F, content="true")
  let collider = collider.set_friction(0.25F)
  inspect(collider.friction() == 0.25F, content="true")
  let collider = collider.set_restitution_combine_rule(
    @dynamics.CoefficientCombineRule::Max,
  )
  inspect(
    collider.restitution_combine_rule() is @dynamics.CoefficientCombineRule::Max,
    content="true",
  )
  let collider = collider.set_friction_combine_rule(
    @dynamics.CoefficientCombineRule::Average,
  )
  inspect(
    collider.friction_combine_rule()
    is @dynamics.CoefficientCombineRule::Average,
    content="true",
  )
  let collider = collider.set_contact_skin(0.2F)
  inspect(collider.contact_skin() == 0.2F, content="true")
  let collider = collider.set_contact_force_event_threshold(5.0F)
  inspect(collider.contact_force_event_threshold() == 5.0F, content="true")
  let collider = collider.set_user_data(11)
  inspect(collider.user_data() == 11, content="true")
  let collider = collider.set_active_hooks(
    ActiveHooks::filter_contact_pairs().insert(
      ActiveHooks::modify_solver_contacts(),
    ),
  )
  inspect(
    collider.active_hooks().contains(ActiveHooks::modify_solver_contacts()),
    content="true",
  )
  let collider = collider.set_active_events(ActiveEvents::empty())
  inspect(
    !collider.active_events().contains(ActiveEvents::collision_events()),
    content="true",
  )
  inspect(
    !collider.active_events().contains(ActiveEvents::contact_force_events()),
    content="true",
  )
  let collider = collider.set_active_events(
    ActiveEvents::contact_force_events(),
  )
  inspect(
    collider.active_events().contains(ActiveEvents::contact_force_events()),
    content="true",
  )
}

///|
test "collider user_data128 roundtrip through ColliderSet serialize/deserialize" {
  let colliders = ColliderSet::new()
  let data = @core.UserData128::from_parts(1UL, 2UL)
  let h = colliders.insert(
    ColliderBuilder::ball(0.5F).user_data128(data).build(),
  )
  let snapshot = colliders.serialize()
  let colliders2 = ColliderSet::deserialize(snapshot)
  if colliders2.get(h) is Some(c) {
    inspect(c.user_data128().equals(data), content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "convex polygon contacts cuboid" {
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = ColliderSet::new()
  let impulse_joints = @dynamics.ImpulseJointSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()
  let narrow_phase = NarrowPhase::new()
  let ground_body = bodies.insert(@dynamics.RigidBodyBuilder::fixed().build())
  let ground = colliders.insert_with_parent(
    ColliderBuilder::cuboid(10.0F, 1.2F).build(),
    ground_body,
    bodies,
  )
  let poly_body = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(0.0F, 1.0F))
    .build(),
  )
  let points = [
    @core.Vec2::new(0.0F, 0.0F),
    @core.Vec2::new(4.0F, 0.0F),
    @core.Vec2::new(4.0F, 4.0F),
    @core.Vec2::new(0.0F, 4.0F),
  ]
  if ColliderBuilder::convex_hull(points) is Some(builder) {
    let poly = colliders.insert_with_parent(builder.build(), poly_body, bodies)
    narrow_phase.update_with_pairs(
      bodies,
      colliders,
      0.0F,
      [(ground, poly)],
      impulse_joints,
      multibody_joints,
    )
    if narrow_phase.contact_pair(ground, poly) is Some(pair) {
      inspect(pair.manifold_count() > 0, content="true")
      let manifolds = pair.manifolds()
      inspect(manifolds.length() > 0, content="true")
      let points = manifolds[0].points()
      inspect(points.length() > 0, content="true")
      inspect(points[0].dist() == points[0].dist(), content="true")
      inspect(points[0].dist() < 0.0F, content="true")
    } else {
      inspect(false, content="true")
    }
  } else {
    inspect(false, content="true")
  }
}

///|
test "convex polygon cuboid manifold keeps contacts for random hulls" {
  fn rng_next(seed : Int) -> Int {
    (seed * 1103515245 + 12345) & 0x7fffffff
  }

  fn rng_float01(seed : Int) -> (@core.Real, Int) {
    let next = rng_next(seed)
    let value = Float::from_double(next.to_double() / 2147483647.0)
    (value, next)
  }

  let bodies = @dynamics.RigidBodySet::new()
  let colliders = ColliderSet::new()
  let impulse_joints = @dynamics.ImpulseJointSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()
  let narrow_phase = NarrowPhase::new()
  let ground_body = bodies.insert(@dynamics.RigidBodyBuilder::fixed().build())
  let ground = colliders.insert_with_parent(
    ColliderBuilder::cuboid(50.0F, 1.2F).build(),
    ground_body,
    bodies,
  )
  let pairs : Array[(ColliderHandle, ColliderHandle)] = []
  let mut seed = 42
  for i in 0..<200 {
    let handle = bodies.insert(
      @dynamics.RigidBodyBuilder::dynamic()
      // Ensure all random hulls intersect the ground AABB:
      // local y ∈ [0, 4) => world y ∈ [-2.8, 1.2) overlaps the ground cuboid.
      .translation(
        @core.Vec2::new(
          0.0F,
          -2.8F + Float::from_double(i.to_double()) * 0.0001F,
        ),
      )
      .build(),
    )
    let points : Array[@core.Vec2] = []
    for _ in 0..<10 {
      let (rx, s1) = rng_float01(seed)
      let (ry, s2) = rng_float01(s1)
      seed = s2
      points.push(@core.Vec2::new(rx * 4.0F, ry * 4.0F))
    }
    if ColliderBuilder::convex_hull(points) is Some(builder) {
      let poly = colliders.insert_with_parent(builder.build(), handle, bodies)
      pairs.push((ground, poly))
    } else {
      inspect(false, content="true")
    }
  }
  narrow_phase.update_with_pairs(
    bodies, colliders, 0.0F, pairs, impulse_joints, multibody_joints,
  )
  for i in 0..<pairs.length() {
    let (h1, h2) = pairs[i]
    if narrow_phase.contact_pair(h1, h2) is Some(pair) {
      inspect(pair.manifold_count() > 0, content="true")
      let manifolds = pair.manifolds()
      inspect(manifolds.length() > 0, content="true")
      let pts = manifolds[0].points()
      inspect(pts.length() > 0, content="true")
      inspect(pts[0].dist() <= 0.0F, content="true")
    } else {
      inspect(false, content="true")
    }
  }
}

///|
/// Ported from rapier active collision types checks.
test "active collision types test" {
  let defaults = ActiveCollisionTypes::default()
  inspect(
    defaults.test_body_types(
      @dynamics.RigidBodyType::Dynamic,
      @dynamics.RigidBodyType::Dynamic,
    ),
    content="true",
  )
  inspect(
    defaults.test_body_types(
      @dynamics.RigidBodyType::Dynamic,
      @dynamics.RigidBodyType::Fixed,
    ),
    content="true",
  )
  inspect(
    defaults.test_body_types(
      @dynamics.RigidBodyType::Dynamic,
      @dynamics.RigidBodyType::KinematicPositionBased,
    ),
    content="true",
  )
  inspect(
    defaults.test_body_types(
      @dynamics.RigidBodyType::KinematicPositionBased,
      @dynamics.RigidBodyType::KinematicVelocityBased,
    ),
    content="false",
  )
  inspect(
    defaults.test_body_types(
      @dynamics.RigidBodyType::Fixed,
      @dynamics.RigidBodyType::Fixed,
    ),
    content="false",
  )
  let all_types = ActiveCollisionTypes::all()
  inspect(
    all_types.test_body_types(
      @dynamics.RigidBodyType::Fixed,
      @dynamics.RigidBodyType::Fixed,
    ),
    content="true",
  )
}

///|
/// Ported from rapier collider parent offset setters.
test "collider parent offset setters" {
  let bodies = @dynamics.RigidBodySet::new()
  let body = @dynamics.RigidBodyBuilder::dynamic().build()
  let body_handle = bodies.insert(body)
  let colliders = ColliderSet::new()
  let collider = ColliderBuilder::ball(0.5F).enabled(false).build()
  let handle = colliders.insert_with_parent(collider, body_handle, bodies)
  if colliders.get_mut(handle) is Some(owned) {
    owned.set_translation_wrt_parent(@core.Vec2::new(0.5F, 1.0F)) |> ignore
    owned.set_rotation_wrt_parent(0.25F) |> ignore
  }
  if colliders.get(handle) is Some(stored) {
    inspect(!stored.is_enabled(), content="true")
    if stored.parent() is Some(parent) {
      inspect(
        @dynamics.RigidBodyHandle::equals(parent, body_handle),
        content="true",
      )
    } else {
      inspect(false, content="true")
    }
    if stored.position_wrt_parent() is Some(position) {
      inspect(position.translation.x == 0.5F, content="true")
      inspect(position.translation.y == 1.0F, content="true")
      inspect(position.rotation.angle() == 0.25F, content="true")
    } else {
      inspect(false, content="true")
    }
  } else {
    inspect(false, content="true")
  }
}

///|
/// Ported from rapier: collider world pose = body pose * local pose.
test "collider world pose respects parent rotation" {
  let bodies = @dynamics.RigidBodySet::new()
  let body = @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(1.0F, 2.0F))
    .rotation(@core.Rot2::from_angle(@core.pi() * 0.5F))
    .build()
  let body_handle = bodies.insert(body)
  let colliders = ColliderSet::new()
  let collider = ColliderBuilder::ball(0.5F)
    .translation(@core.Vec2::new(1.0F, 0.0F))
    .rotation(0.25F)
    .build()
  let handle = colliders.insert_with_parent(collider, body_handle, bodies)
  if colliders.get(handle) is Some(stored) {
    let tol = 1.0e-5F
    inspect(@core.abs(stored.translation().x - 1.0F) < tol, content="true")
    inspect(@core.abs(stored.translation().y - 3.0F) < tol, content="true")
    inspect(
      @core.abs(stored.rotation() - (@core.pi() * 0.5F + 0.25F)) < tol,
      content="true",
    )
  } else {
    inspect(false, content="true")
  }
}
