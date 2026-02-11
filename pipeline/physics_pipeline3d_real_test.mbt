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
  let islands = @dynamics.IslandManager3D::new()
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
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
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

///|
test "physics_pipeline3d_real: sleeping and wake-up" {
  let pipeline = PhysicsPipeline3DReal::new()
  let broad_phase = @collision.BroadPhase3D::new()
  let narrow_phase = @collision.NarrowPhase3D::new()
  let islands = @dynamics.IslandManager3D::new()
  let bodies = @dynamics.RigidBodySet3D::new()
  let colliders = @collision.ColliderSet3D::new()
  let gravity = @core.Vec3::zero()
  let parameters = @dynamics.IntegrationParameters::default().set_dt(
    1.0F / 60.0F,
  )
  let ball1 = bodies.insert(
    @dynamics.RigidBodyBuilder3D::dynamic()
    .translation(@core.Vec3::new(0.0F, 0.0F, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::ball(0.5F).build(),
    ball1,
    bodies,
  )
  |> ignore

  // Let ball1 go to sleep (default time_until_sleep is 2s).
  for _ in 0..<240 {
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    )
  }
  if bodies.get(ball1) is Some(rb1) {
    inspect(!rb1.is_active(), content="true")
  } else {
    inspect(false, content="true")
  }

  // A moving ball collides with ball1 and should wake it up.
  let ball2 = bodies.insert(
    @dynamics.RigidBodyBuilder3D::dynamic()
    .translation(@core.Vec3::new(-3.0F, 0.0F, 0.0F))
    .linvel(@core.Vec3::new(10.0F, 0.0F, 0.0F))
    .can_sleep(false)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::ball(0.5F).build(),
    ball2,
    bodies,
  )
  |> ignore
  for _ in 0..<60 {
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    )
  }
  if bodies.get(ball1) is Some(rb1b) {
    inspect(rb1b.is_active(), content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "physics_pipeline3d_real: collision events for non-sensor contacts" {
  let pipeline = PhysicsPipeline3DReal::new()
  let broad_phase = @collision.BroadPhase3D::new()
  let narrow_phase = @collision.NarrowPhase3D::new()
  let islands = @dynamics.IslandManager3D::new()
  let bodies = @dynamics.RigidBodySet3D::new()
  let colliders = @collision.ColliderSet3D::new()
  let gravity = @core.Vec3::zero()
  let parameters = @dynamics.IntegrationParameters::default().set_dt(
    1.0F / 60.0F,
  )
  let handler = EventHandler3D::new()
  fn has_event(
    events : Array[@collision.CollisionEvent3D],
    started : Bool,
    sensor : Bool,
    a : @collision.ColliderHandle3D,
    b : @collision.ColliderHandle3D,
  ) -> Bool {
    for e in events {
      if started {
        if !e.started() {
          continue
        }
      } else if !e.stopped() {
        continue
      }
      if e.sensor() != sensor {
        continue
      }
      let c1 = e.collider1()
      let c2 = e.collider2()
      if (c1.equals(a) && c2.equals(b)) || (c1.equals(b) && c2.equals(a)) {
        return true
      }
    }
    false
  }

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
    @dynamics.RigidBodyBuilder3D::kinematic_position_based()
    .translation(@core.Vec3::new(0.0F, 2.0F, 0.0F))
    .build(),
  )
  let ball_collider = colliders.insert_with_parent(
    @collision.ColliderBuilder3D::ball(0.5F)
    .active_events(@collision.ActiveEvents::collision_events())
    .active_hooks(@collision.ActiveHooks::filter_contact_pairs())
    .build(),
    ball,
    bodies,
  )

  // Step once: no contact, no events.
  pipeline.step_with_events(
    gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders, handler,
  )
  let ev0 = handler.take_collision_events()
  inspect(ev0.length(), content="0")

  // Move into contact: should emit Started with non-sensor flags.
  if bodies.get(ball) is Some(rb) {
    rb.set_next_kinematic_translation(@core.Vec3::new(0.0F, 0.4F, 0.0F))
  } else {
    inspect(false, content="true")
  }
  pipeline.step_with_events(
    gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders, handler,
  )
  let ev1 = handler.take_collision_events()
  inspect(
    has_event(ev1, true, false, ball_collider, ground_collider),
    content="true",
  )

  // Move out of contact: should emit Stopped with non-sensor flags.
  if bodies.get(ball) is Some(rb2) {
    rb2.set_next_kinematic_translation(@core.Vec3::new(0.0F, 2.0F, 0.0F))
  } else {
    inspect(false, content="true")
  }
  pipeline.step_with_events(
    gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders, handler,
  )
  let ev2 = handler.take_collision_events()
  inspect(
    has_event(ev2, false, false, ball_collider, ground_collider),
    content="true",
  )
}

///|
test "physics_pipeline3d_real: collision events for sensor intersections" {
  let pipeline = PhysicsPipeline3DReal::new()
  let broad_phase = @collision.BroadPhase3D::new()
  let narrow_phase = @collision.NarrowPhase3D::new()
  let islands = @dynamics.IslandManager3D::new()
  let bodies = @dynamics.RigidBodySet3D::new()
  let colliders = @collision.ColliderSet3D::new()
  let gravity = @core.Vec3::zero()
  let parameters = @dynamics.IntegrationParameters::default().set_dt(
    1.0F / 60.0F,
  )
  let handler = EventHandler3D::new()
  fn has_event(
    events : Array[@collision.CollisionEvent3D],
    started : Bool,
    sensor : Bool,
    a : @collision.ColliderHandle3D,
    b : @collision.ColliderHandle3D,
  ) -> Bool {
    for e in events {
      if started {
        if !e.started() {
          continue
        }
      } else if !e.stopped() {
        continue
      }
      if e.sensor() != sensor {
        continue
      }
      let c1 = e.collider1()
      let c2 = e.collider2()
      if (c1.equals(a) && c2.equals(b)) || (c1.equals(b) && c2.equals(a)) {
        return true
      }
    }
    false
  }

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
    @dynamics.RigidBodyBuilder3D::kinematic_position_based()
    .translation(@core.Vec3::new(0.0F, 2.0F, 0.0F))
    .build(),
  )
  let ball_collider = colliders.insert_with_parent(
    @collision.ColliderBuilder3D::ball(0.5F)
    .sensor(true)
    .active_events(@collision.ActiveEvents::collision_events())
    .build(),
    ball,
    bodies,
  )

  // Step once: no intersection, no events.
  pipeline.step_with_events(
    gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders, handler,
  )
  let ev0 = handler.take_collision_events()
  inspect(ev0.length(), content="0")

  // Move into intersection: should emit Started with sensor flags.
  if bodies.get(ball) is Some(rb) {
    rb.set_next_kinematic_translation(@core.Vec3::new(0.0F, 0.4F, 0.0F))
  } else {
    inspect(false, content="true")
  }
  pipeline.step_with_events(
    gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders, handler,
  )
  let ev1 = handler.take_collision_events()
  inspect(
    has_event(ev1, true, true, ball_collider, ground_collider),
    content="true",
  )

  // Move out of intersection: should emit Stopped with sensor flags.
  if bodies.get(ball) is Some(rb2) {
    rb2.set_next_kinematic_translation(@core.Vec3::new(0.0F, 2.0F, 0.0F))
  } else {
    inspect(false, content="true")
  }
  pipeline.step_with_events(
    gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders, handler,
  )
  let ev2 = handler.take_collision_events()
  inspect(
    has_event(ev2, false, true, ball_collider, ground_collider),
    content="true",
  )
}

///|
test "physics_pipeline3d_real: CCD prevents tunneling for fast balls (disabled vs enabled)" {
  fn run(ccd_enabled : Bool) -> @core.Real {
    let pipeline = PhysicsPipeline3DReal::new()
    let broad_phase = @collision.BroadPhase3D::new()
    let narrow_phase = @collision.NarrowPhase3D::new()
    let islands = @dynamics.IslandManager3D::new()
    let bodies = @dynamics.RigidBodySet3D::new()
    let colliders = @collision.ColliderSet3D::new()
    let gravity = @core.Vec3::zero()
    let parameters = @dynamics.IntegrationParameters::default().set_dt(1.0F)
    let wall = bodies.insert(@dynamics.RigidBodyBuilder3D::fixed().build())
    colliders.insert_with_parent(
      @collision.ColliderBuilder3D::cuboid(0.1F, 10.0F, 10.0F).build(),
      wall,
      bodies,
    )
    |> ignore
    let ball = bodies.insert(
      @dynamics.RigidBodyBuilder3D::dynamic()
      .translation(@core.Vec3::new(-10.0F, 0.0F, 0.0F))
      .ccd_enabled(ccd_enabled)
      .build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder3D::ball(1.0F).build(),
      ball,
      bodies,
    )
    |> ignore
    if bodies.get_mut(ball) is Some(rb) {
      rb.set_linvel(@core.Vec3::new(1000.0F, 0.0F, 0.0F))
      rb.wake_up()
    }
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    )
    if bodies.get(ball) is Some(rb2) {
      rb2.translation().x
    } else {
      0.0F
    }
  }

  let no_ccd_x = run(false)
  inspect(no_ccd_x > 0.0F, content="true")
  let ccd_x = run(true)
  inspect(ccd_x < 0.0F, content="true")
}

///|
test "physics_pipeline3d_real: collision events do not repeat while contact persists" {
  let pipeline = PhysicsPipeline3DReal::new()
  let broad_phase = @collision.BroadPhase3D::new()
  let narrow_phase = @collision.NarrowPhase3D::new()
  let islands = @dynamics.IslandManager3D::new()
  let bodies = @dynamics.RigidBodySet3D::new()
  let colliders = @collision.ColliderSet3D::new()
  let gravity = @core.Vec3::zero()
  let parameters = @dynamics.IntegrationParameters::default().set_dt(
    1.0F / 60.0F,
  )
  let handler = EventHandler3D::new()
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder3D::fixed()
    .translation(@core.Vec3::new(0.0F, -1.0F, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(10.0F, 1.0F, 10.0F).build(),
    ground,
    bodies,
  )
  |> ignore
  let ball = bodies.insert(
    @dynamics.RigidBodyBuilder3D::kinematic_position_based()
    .translation(@core.Vec3::new(0.0F, 2.0F, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::ball(0.5F)
    .active_events(@collision.ActiveEvents::collision_events())
    .build(),
    ball,
    bodies,
  )
  |> ignore

  // Step into contact: expect exactly one Started.
  if bodies.get(ball) is Some(rb) {
    rb.set_next_kinematic_translation(@core.Vec3::new(0.0F, 0.4F, 0.0F))
  } else {
    inspect(false, content="true")
  }
  pipeline.step_with_events(
    gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders, handler,
  )
  let ev1 = handler.take_collision_events()
  inspect(ev1.length() > 0, content="true")

  // Keep contact: should emit no further events.
  pipeline.step_with_events(
    gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders, handler,
  )
  let ev2 = handler.take_collision_events()
  inspect(ev2.length(), content="0")
}

///|
test "physics_pipeline3d_real: collision events do not repeat while sensor intersection persists" {
  let pipeline = PhysicsPipeline3DReal::new()
  let broad_phase = @collision.BroadPhase3D::new()
  let narrow_phase = @collision.NarrowPhase3D::new()
  let islands = @dynamics.IslandManager3D::new()
  let bodies = @dynamics.RigidBodySet3D::new()
  let colliders = @collision.ColliderSet3D::new()
  let gravity = @core.Vec3::zero()
  let parameters = @dynamics.IntegrationParameters::default().set_dt(
    1.0F / 60.0F,
  )
  let handler = EventHandler3D::new()
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder3D::fixed()
    .translation(@core.Vec3::new(0.0F, -1.0F, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(10.0F, 1.0F, 10.0F).build(),
    ground,
    bodies,
  )
  |> ignore
  let ball = bodies.insert(
    @dynamics.RigidBodyBuilder3D::kinematic_position_based()
    .translation(@core.Vec3::new(0.0F, 2.0F, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::ball(0.5F)
    .sensor(true)
    .active_events(@collision.ActiveEvents::collision_events())
    .build(),
    ball,
    bodies,
  )
  |> ignore

  // Step into intersection: expect exactly one Started.
  if bodies.get(ball) is Some(rb) {
    rb.set_next_kinematic_translation(@core.Vec3::new(0.0F, 0.4F, 0.0F))
  } else {
    inspect(false, content="true")
  }
  pipeline.step_with_events(
    gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders, handler,
  )
  let ev1 = handler.take_collision_events()
  inspect(ev1.length() > 0, content="true")

  // Keep intersection: should emit no further events.
  pipeline.step_with_events(
    gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders, handler,
  )
  let ev2 = handler.take_collision_events()
  inspect(ev2.length(), content="0")
}

///|
test "physics_pipeline3d_real: intersection events for sensor overlaps (without collision events)" {
  let pipeline = PhysicsPipeline3DReal::new()
  let broad_phase = @collision.BroadPhase3D::new()
  let narrow_phase = @collision.NarrowPhase3D::new()
  let islands = @dynamics.IslandManager3D::new()
  let bodies = @dynamics.RigidBodySet3D::new()
  let colliders = @collision.ColliderSet3D::new()
  let gravity = @core.Vec3::zero()
  let parameters = @dynamics.IntegrationParameters::default().set_dt(
    1.0F / 60.0F,
  )
  let handler = EventHandler3D::new()
  fn has_event(
    events : Array[@collision.IntersectionEvent3D],
    intersecting : Bool,
    a : @collision.ColliderHandle3D,
    b : @collision.ColliderHandle3D,
  ) -> Bool {
    for e in events {
      if e.intersecting() != intersecting {
        continue
      }
      let c1 = e.collider1()
      let c2 = e.collider2()
      if (c1.equals(a) && c2.equals(b)) || (c1.equals(b) && c2.equals(a)) {
        return true
      }
    }
    false
  }

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
    @dynamics.RigidBodyBuilder3D::kinematic_position_based()
    .translation(@core.Vec3::new(0.0F, 2.0F, 0.0F))
    .build(),
  )
  let sensor_ball = colliders.insert_with_parent(
    @collision.ColliderBuilder3D::ball(0.5F)
    .sensor(true)
    .active_events(@collision.ActiveEvents::intersection_events())
    .build(),
    ball,
    bodies,
  )

  // Step once: no intersection, no events.
  pipeline.step_with_events(
    gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders, handler,
  )
  inspect(handler.take_collision_events().length(), content="0")
  inspect(handler.take_intersection_events().length(), content="0")

  // Step into intersection: expect intersecting=true, and no collision events.
  if bodies.get(ball) is Some(rb) {
    rb.set_next_kinematic_translation(@core.Vec3::new(0.0F, 0.4F, 0.0F))
  } else {
    inspect(false, content="true")
  }
  pipeline.step_with_events(
    gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders, handler,
  )
  inspect(handler.take_collision_events().length(), content="0")
  let ev1 = handler.take_intersection_events()
  inspect(has_event(ev1, true, sensor_ball, ground_collider), content="true")

  // Keep intersection: should emit no further events.
  pipeline.step_with_events(
    gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders, handler,
  )
  inspect(handler.take_intersection_events().length(), content="0")

  // Step out of intersection: expect intersecting=false.
  if bodies.get(ball) is Some(rb2) {
    rb2.set_next_kinematic_translation(@core.Vec3::new(0.0F, 2.0F, 0.0F))
  } else {
    inspect(false, content="true")
  }
  pipeline.step_with_events(
    gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders, handler,
  )
  let ev2 = handler.take_intersection_events()
  inspect(has_event(ev2, false, sensor_ball, ground_collider), content="true")
}

///|
test "physics_pipeline3d_real: contact force events and contact impulses are populated" {
  let pipeline = PhysicsPipeline3DReal::new()
  let broad_phase = @collision.BroadPhase3D::new()
  let narrow_phase = @collision.NarrowPhase3D::new()
  let islands = @dynamics.IslandManager3D::new()
  let bodies = @dynamics.RigidBodySet3D::new()
  let colliders = @collision.ColliderSet3D::new()
  let gravity = @core.Vec3::new(0.0F, -9.81F, 0.0F)
  let parameters = @dynamics.IntegrationParameters::default().set_dt(
    1.0F / 60.0F,
  )
  let handler = EventHandler3D::new()
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
    // Start slightly penetrating the ground so the solver must apply impulses immediately.
    .translation(@core.Vec3::new(0.0F, 0.4F, 0.0F))
    .build(),
  )
  let ball_collider = colliders.insert_with_parent(
    @collision.ColliderBuilder3D::ball(0.5F)
    .active_events(@collision.ActiveEvents::contact_force_events())
    .contact_force_event_threshold(0.0F)
    .build(),
    ball,
    bodies,
  )
  pipeline.step_with_events(
    gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders, handler,
  )
  let forces = handler.take_contact_force_events()
  inspect(forces.length() > 0, content="true")
  if narrow_phase.contact_pair(ball_collider, ground_collider) is Some(pair) {
    inspect(pair.total_impulse_magnitude() > 0.0F, content="true")
    let impulse = pair.total_impulse()
    inspect(impulse.y > 0.0F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "physics_pipeline3d_real: hooks can filter contact pairs" {
  let pipeline = PhysicsPipeline3DReal::new()
  let broad_phase = @collision.BroadPhase3D::new()
  let narrow_phase = @collision.NarrowPhase3D::new()
  let islands = @dynamics.IslandManager3D::new()
  let bodies = @dynamics.RigidBodySet3D::new()
  let colliders = @collision.ColliderSet3D::new()
  let gravity = @core.Vec3::zero()
  let parameters = @dynamics.IntegrationParameters::default().set_dt(
    1.0F / 60.0F,
  )
  let handler = EventHandler3D::new()
  let hooks = PhysicsHooks3D::new().filter_contact_pair(fn(
    _bodies : @dynamics.RigidBodySet3D,
    _colliders : @collision.ColliderSet3D,
    _c1 : @collision.ColliderHandle3D,
    _c2 : @collision.ColliderHandle3D,
  ) -> Bool {
    false
  })
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
    @dynamics.RigidBodyBuilder3D::kinematic_position_based()
    .translation(@core.Vec3::new(0.0F, 2.0F, 0.0F))
    .build(),
  )
  let ball_collider = colliders.insert_with_parent(
    @collision.ColliderBuilder3D::ball(0.5F)
    .active_events(@collision.ActiveEvents::collision_events())
    .active_hooks(@collision.ActiveHooks::filter_contact_pairs())
    .build(),
    ball,
    bodies,
  )
  if bodies.get(ball) is Some(rb) {
    rb.set_next_kinematic_translation(@core.Vec3::new(0.0F, 0.4F, 0.0F))
  } else {
    inspect(false, content="true")
  }
  pipeline.step_with_events_and_hooks(
    gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders, handler,
    hooks,
  )
  inspect(handler.take_collision_events().length(), content="0")
  inspect(
    narrow_phase.contact_pair(ball_collider, ground_collider) is None,
    content="true",
  )
}

///|
test "physics_pipeline3d_real: contact filter hook requires ActiveHooks flag" {
  let pipeline = PhysicsPipeline3DReal::new()
  let broad_phase = @collision.BroadPhase3D::new()
  let narrow_phase = @collision.NarrowPhase3D::new()
  let islands = @dynamics.IslandManager3D::new()
  let bodies = @dynamics.RigidBodySet3D::new()
  let colliders = @collision.ColliderSet3D::new()
  let gravity = @core.Vec3::zero()
  let parameters = @dynamics.IntegrationParameters::default().set_dt(
    1.0F / 60.0F,
  )
  let hooks = PhysicsHooks3D::new().filter_contact_pair(fn(
    _bodies : @dynamics.RigidBodySet3D,
    _colliders : @collision.ColliderSet3D,
    _c1 : @collision.ColliderHandle3D,
    _c2 : @collision.ColliderHandle3D,
  ) -> Bool {
    false
  })
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
    .translation(@core.Vec3::new(0.0F, 0.4F, 0.0F))
    .build(),
  )
  let ball_collider = colliders.insert_with_parent(
    @collision.ColliderBuilder3D::ball(0.5F).build(),
    ball,
    bodies,
  )
  pipeline.step_with_hooks(
    gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders, hooks,
  )
  inspect(
    narrow_phase.contact_pair(ball_collider, ground_collider) is Some(_),
    content="true",
  )
}

///|
test "physics_pipeline3d_real: hooks can override solver normal" {
  let pipeline = PhysicsPipeline3DReal::new()
  let broad_phase = @collision.BroadPhase3D::new()
  let narrow_phase = @collision.NarrowPhase3D::new()
  let islands = @dynamics.IslandManager3D::new()
  let bodies = @dynamics.RigidBodySet3D::new()
  let colliders = @collision.ColliderSet3D::new()
  let gravity = @core.Vec3::zero()
  let parameters = @dynamics.IntegrationParameters::default().set_dt(
    1.0F / 60.0F,
  )
  let hooks = PhysicsHooks3D::new().modify_solver_contacts(fn(
    _bodies : @dynamics.RigidBodySet3D,
    _colliders : @collision.ColliderSet3D,
    _c1 : @collision.ColliderHandle3D,
    _c2 : @collision.ColliderHandle3D,
    _normal : @core.Vec3,
  ) -> (Bool, @core.Vec3?) {
    (true, Some(@core.Vec3::new(1.0F, 0.0F, 0.0F)))
  })
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder3D::fixed()
    .translation(@core.Vec3::new(0.0F, -1.0F, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(10.0F, 1.0F, 10.0F).build(),
    ground,
    bodies,
  )
  |> ignore
  let ball = bodies.insert(
    @dynamics.RigidBodyBuilder3D::dynamic()
    // Start penetrating so the solver applies a corrective impulse immediately.
    .translation(@core.Vec3::new(0.0F, 0.4F, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::ball(0.5F)
    .active_hooks(@collision.ActiveHooks::modify_solver_contacts())
    .build(),
    ball,
    bodies,
  )
  |> ignore
  pipeline.step_with_hooks(
    gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders, hooks,
  )
  if bodies.get(ball) is Some(rb) {
    inspect(@core.abs(rb.linvel().x) > 1.0e-6F, content="true")
  } else {
    inspect(false, content="true")
  }
}
