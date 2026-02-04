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
