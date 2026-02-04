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
/// Ported from rapier/src/control/character_controller.rs (character_controller_climb_test) for 2D.
test "character controller climb" {
  let colliders = @collision.ColliderSet::new()
  let impulse_joints = @dynamics.ImpulseJointSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()
  let pipeline = @pipeline.PhysicsPipeline::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let islands = @dynamics.IslandManager::new()
  let bodies = @dynamics.RigidBodySet::new()
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let ground_size = 100.0F
  let ground_height = 0.1F
  let rigid_body = @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, -ground_height))
    .build()
  let floor_handle = bodies.insert(rigid_body)
  let collider = @collision.ColliderBuilder::cuboid(ground_size, ground_height).build()
  colliders.insert_with_parent(collider, floor_handle, bodies) |> ignore
  let slope_angle = 0.2F
  let slope_size = 2.0F
  let collider = @collision.ColliderBuilder::cuboid(slope_size, ground_height)
    .translation(@core.Vec2::new(0.1F + slope_size, -ground_height + 0.4F))
    .rotation(slope_angle)
    .build()
  colliders.insert(collider) |> ignore
  let impossible_slope_angle = 0.6F
  let impossible_slope_size = 2.0F
  let collider = @collision.ColliderBuilder::cuboid(slope_size, ground_height)
    .translation(
      @core.Vec2::new(
        0.1F + slope_size * 2.0F + impossible_slope_size - 0.9F,
        -ground_height + 1.7F,
      ),
    )
    .rotation(impossible_slope_angle)
    .build()
  colliders.insert(collider) |> ignore
  let integration_parameters = @dynamics.IntegrationParameters::default()
  let character_body_can_climb = @dynamics.RigidBodyBuilder::kinematic_position_based()
    .additional_mass(1.0F)
    .build()
  character_body_can_climb.set_translation(@core.Vec2::new(0.6F, 0.5F), false)
  |> ignore
  let character_handle_can_climb = bodies.insert(character_body_can_climb)
  let collider = @collision.ColliderBuilder::ball(0.5F).build()
  colliders.insert_with_parent(collider, character_handle_can_climb, bodies)
  |> ignore
  let character_body_cannot_climb = @dynamics.RigidBodyBuilder::kinematic_position_based()
    .additional_mass(1.0F)
    .build()
  character_body_cannot_climb.set_translation(
    @core.Vec2::new(-0.6F, 0.5F),
    false,
  )
  |> ignore
  let character_handle_cannot_climb = bodies.insert(character_body_cannot_climb)
  let collider = @collision.ColliderBuilder::ball(0.5F).build()
  let character_shape = collider.shape()
  colliders.insert_with_parent(collider, character_handle_cannot_climb, bodies)
  |> ignore
  for _ in 0..<200 {
    pipeline.step(
      gravity,
      integration_parameters,
      islands,
      broad_phase,
      narrow_phase,
      bodies,
      colliders,
      impulse_joints,
      multibody_joints,
      @dynamics.CCDSolver::new(),
      @pipeline.PhysicsHooks::new(),
      @pipeline.EventHandler::new(),
    )
    let update_character_controller = fn(
      controller : KinematicCharacterController,
      handle : @dynamics.RigidBodyHandle,
    ) -> Bool {
      if bodies.get(handle) is Some(character_body) {
        let filter = @collision.QueryFilter::new().exclude_rigid_body(handle)
        let query_pipeline = broad_phase.as_query_pipeline(
          bodies, colliders, filter,
        )
        let effective_movement = controller.move_shape(
          integration_parameters.dt,
          query_pipeline,
          bodies,
          colliders,
          character_shape,
          character_body.position(),
          @core.Vec2::new(0.1F, -0.1F),
          fn(_collision) {  },
        )
        let is_grounded = effective_movement.is_grounded()
        let translation = character_body.translation()
        character_body.set_next_kinematic_translation(
          translation.add(effective_movement.translation()),
        )
        |> ignore
        return is_grounded
      }
      return false
    }
    let controller_cannot = KinematicCharacterController::new().with_max_slope_climb_angle(
      impossible_slope_angle - 0.001F,
    )
    let controller_can = KinematicCharacterController::new().with_max_slope_climb_angle(
      impossible_slope_angle + 0.001F,
    )
    inspect(
      update_character_controller(
        controller_cannot, character_handle_cannot_climb,
      ),
      content="true",
    )
    inspect(
      update_character_controller(controller_can, character_handle_can_climb),
      content="true",
    )
  }
}

///|
test "character controller utilities: CharacterLength map_* and defaults" {
  let a = CharacterLength::absolute(1.0F)
  let a2 = a.map_absolute(fn(v) { v + 1.0F })
  if a2 is CharacterLength::Absolute(v) {
    inspect(@core.abs(v - 2.0F) < 1.0e-6F, content="true")
  } else {
    inspect(false, content="true")
  }
  // map_absolute should not affect Relative.
  let r = CharacterLength::relative(0.5F)
  let r2 = r.map_absolute(fn(v) { v + 1.0F })
  if r2 is CharacterLength::Relative(v) {
    inspect(@core.abs(v - 0.5F) < 1.0e-6F, content="true")
  } else {
    inspect(false, content="true")
  }
  let r3 = r.map_relative(fn(v) { v * 2.0F })
  if r3 is CharacterLength::Relative(v) {
    inspect(@core.abs(v - 1.0F) < 1.0e-6F, content="true")
  } else {
    inspect(false, content="true")
  }
  let auto = CharacterAutostep::default()
  if auto.max_height() is CharacterLength::Relative(v) {
    inspect(@core.abs(v - 0.25F) < 1.0e-6F, content="true")
  } else {
    inspect(false, content="true")
  }
  if auto.min_width() is CharacterLength::Relative(v) {
    inspect(@core.abs(v - 0.5F) < 1.0e-6F, content="true")
  } else {
    inspect(false, content="true")
  }
  inspect(auto.include_dynamic_bodies(), content="true")
  let c = KinematicCharacterController::default()
  inspect(
    @core.abs(c.max_slope_climb_angle - 0.785398163F) < 1.0e-6F,
    content="true",
  )
  inspect(
    @core.abs(c.min_slope_slide_angle - 0.785398163F) < 1.0e-6F,
    content="true",
  )
  if c.offset is CharacterLength::Relative(v) {
    inspect(@core.abs(v - 0.01F) < 1.0e-6F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
/// Ported from rapier-reference/examples2d/character_controller2.rs (moving platform callback) for 2D.
test "character controller moving platform keeps grounded and follows platform" {
  let colliders = @collision.ColliderSet::new()
  let impulse_joints = @dynamics.ImpulseJointSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()
  let pipeline = @pipeline.PhysicsPipeline::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let islands = @dynamics.IslandManager::new()
  let bodies = @dynamics.RigidBodySet::new()
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let integration_parameters = @dynamics.IntegrationParameters::default()
  let inv_dt = integration_parameters.inv_dt()
  let dt = if inv_dt == 0.0F { 0.0F } else { 1.0F / inv_dt }

  // Flat ground.
  let ground_height = 0.1F
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, -ground_height))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(50.0F, ground_height).build(),
    ground,
    bodies,
  )
  |> ignore

  // Moving platform (kinematic velocity based).
  let platform = bodies.insert(
    @dynamics.RigidBodyBuilder::kinematic_velocity_based()
    .translation(@core.Vec2::new(-8.0F, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(2.0F, ground_height).build(),
    platform,
    bodies,
  )
  |> ignore

  // Character (kinematic position based).
  let character = bodies.insert(
    @dynamics.RigidBodyBuilder::kinematic_position_based()
    // Ball radius is 0.5, platform top is at y=0.1, so grounded center is y=0.6.
    .translation(@core.Vec2::new(-8.0F, 0.6F))
    .build(),
  )
  let character_collider = @collision.ColliderBuilder::ball(0.5F).build()
  let character_shape = character_collider.shape()
  colliders.insert_with_parent(character_collider, character, bodies) |> ignore
  let controller = KinematicCharacterController::new()
  let mut last_grounded = true
  for _ in 0..<240 {
    // Move platform horizontally.
    if bodies.get_mut(platform) is Some(rb) {
      rb.set_linvel(@core.Vec2::new(2.0F, 0.0F), true) |> ignore
    }
    pipeline.step(
      gravity,
      integration_parameters,
      islands,
      broad_phase,
      narrow_phase,
      bodies,
      colliders,
      impulse_joints,
      multibody_joints,
      @dynamics.CCDSolver::new(),
      @pipeline.PhysicsHooks::new(),
      @pipeline.EventHandler::new(),
    )
    if bodies.get(character) is Some(character_body) {
      let filter_character_controller = @collision.QueryFilter::new().exclude_rigid_body(
        character,
      )
      let query_pipeline = broad_phase.as_query_pipeline(
        bodies, colliders, filter_character_controller,
      )
      let movement = controller.move_shape(
        dt,
        query_pipeline,
        bodies,
        colliders,
        character_shape,
        character_body.position(),
        @core.Vec2::new(0.0F, -0.1F),
        fn(_collision) {  },
      )
      last_grounded = movement.is_grounded()
      inspect(movement.is_grounded(), content="true")
      // Keep the character following the controller output.
      let t = character_body.translation()
      if bodies.get_mut(character) is Some(rb) {
        rb.set_next_kinematic_translation(t.add(movement.translation()))
        |> ignore
      }
    } else {
      inspect(false, content="true")
    }
  }
  inspect(last_grounded, content="true")
  if bodies.get(character) is Some(character_body) {
    let p = character_body.translation()
    // The platform moves at +2 m/s for 4 seconds; expect the character to have advanced too.
    inspect(p.x > -2.0F, content="true")
    inspect(p.y > -10.0F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
/// Ported from rapier-reference/examples2d/character_controller2.rs (stairs) for 2D.
test "character controller autostep climbs stairs" {
  let colliders = @collision.ColliderSet::new()
  let impulse_joints = @dynamics.ImpulseJointSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()
  let pipeline = @pipeline.PhysicsPipeline::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let islands = @dynamics.IslandManager::new()
  let bodies = @dynamics.RigidBodySet::new()
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let integration_parameters = @dynamics.IntegrationParameters::default()
  let inv_dt = integration_parameters.inv_dt()
  let dt = if inv_dt == 0.0F { 0.0F } else { 1.0F / inv_dt }

  // Ground.
  let ground_height = 0.1F
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, -ground_height))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(50.0F, ground_height).build(),
    ground,
    bodies,
  )
  |> ignore

  // Stairs.
  let stair_width = 1.0F
  let stair_height = 0.1F
  for i in 0..<10 {
    let x = Float::from_double(i.to_double()) * stair_width / 2.0F
    let y = Float::from_double(i.to_double()) * stair_height * 1.5F + 0.2F
    colliders.insert(
      @collision.ColliderBuilder::cuboid(
        stair_width / 2.0F,
        stair_height / 2.0F,
      )
      .translation(@core.Vec2::new(x, y))
      .build(),
    )
    |> ignore
  }

  // Character.
  let character = bodies.insert(
    @dynamics.RigidBodyBuilder::kinematic_position_based()
    .translation(@core.Vec2::new(-2.0F, 1.0F))
    .build(),
  )
  let character_collider = @collision.ColliderBuilder::ball(0.5F).build()
  let character_shape = character_collider.shape()
  colliders.insert_with_parent(character_collider, character, bodies) |> ignore
  // The stairs in this scenario have a relatively large step height, so use
  // an explicit autostep configuration.
  let autostep = CharacterAutostep::new(
    CharacterLength::absolute(0.25F),
    CharacterLength::absolute(0.2F),
  )
  let controller = KinematicCharacterController::new().with_autostep(
    Some(autostep),
  )
  let mut max_y = -1.0F
  for _ in 0..<300 {
    pipeline.step(
      gravity,
      integration_parameters,
      islands,
      broad_phase,
      narrow_phase,
      bodies,
      colliders,
      impulse_joints,
      multibody_joints,
      @dynamics.CCDSolver::new(),
      @pipeline.PhysicsHooks::new(),
      @pipeline.EventHandler::new(),
    )
    if bodies.get(character) is Some(character_body) {
      let filter_character_controller = @collision.QueryFilter::new().exclude_rigid_body(
        character,
      )
      let query_pipeline = broad_phase.as_query_pipeline(
        bodies, colliders, filter_character_controller,
      )
      let movement = controller.move_shape(
        dt,
        query_pipeline,
        bodies,
        colliders,
        character_shape,
        character_body.position(),
        @core.Vec2::new(0.08F, -0.1F),
        fn(_collision) {  },
      )
      let t = character_body.translation()
      let next_pos = t.add(movement.translation())
      if next_pos.y > max_y {
        max_y = next_pos.y
      }
      if bodies.get_mut(character) is Some(rb) {
        rb.set_next_kinematic_translation(next_pos) |> ignore
      }
    }
  }
  if bodies.get(character) is Some(character_body) {
    let p = character_body.translation()
    inspect(p.x > 1.0F, content="true")
    // Must have climbed at least a few steps.
    inspect(max_y > 0.9F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
/// Derived from rapier-reference/examples2d/character_controller2.rs controller setup.
test "character controller reports sliding on steep slope but not on flat ground" {
  let colliders = @collision.ColliderSet::new()
  let impulse_joints = @dynamics.ImpulseJointSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()
  let pipeline = @pipeline.PhysicsPipeline::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let islands = @dynamics.IslandManager::new()
  let bodies = @dynamics.RigidBodySet::new()
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let integration_parameters = @dynamics.IntegrationParameters::default()
  let inv_dt = integration_parameters.inv_dt()
  let dt = if inv_dt == 0.0F { 0.0F } else { 1.0F / inv_dt }

  // Ground + a steep slope.
  let ground_height = 0.1F
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, -ground_height))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(50.0F, ground_height).build(),
    ground,
    bodies,
  )
  |> ignore
  colliders.insert(
    @collision.ColliderBuilder::cuboid(2.0F, ground_height)
    .translation(@core.Vec2::new(2.0F, 0.6F))
    .rotation(0.7F)
    .build(),
  )
  |> ignore
  let character = bodies.insert(
    @dynamics.RigidBodyBuilder::kinematic_position_based()
    .translation(@core.Vec2::new(0.0F, 1.0F))
    .build(),
  )
  let character_collider = @collision.ColliderBuilder::ball(0.5F).build()
  let character_shape = character_collider.shape()
  colliders.insert_with_parent(character_collider, character, bodies) |> ignore
  let controller_flat = KinematicCharacterController::new()
  let controller_steep = KinematicCharacterController::new()
    .with_min_slope_slide_angle(0.1F)
    .with_max_slope_climb_angle(1.2F)

  // Flat ground move should not report sliding.
  pipeline.step(
    gravity,
    integration_parameters,
    islands,
    broad_phase,
    narrow_phase,
    bodies,
    colliders,
    impulse_joints,
    multibody_joints,
    @dynamics.CCDSolver::new(),
    @pipeline.PhysicsHooks::new(),
    @pipeline.EventHandler::new(),
  )
  if bodies.get(character) is Some(character_body) {
    let filter_character_controller = @collision.QueryFilter::new().exclude_rigid_body(
      character,
    )
    let query_pipeline = broad_phase.as_query_pipeline(
      bodies, colliders, filter_character_controller,
    )
    let m = controller_flat.move_shape(
      dt,
      query_pipeline,
      bodies,
      colliders,
      character_shape,
      character_body.position(),
      @core.Vec2::new(0.05F, 0.0F),
      fn(_collision) {  },
    )
    inspect(!m.is_sliding_down_slope(), content="true")
  }

  // Move onto the steep slope.
  if bodies.get_mut(character) is Some(rb) {
    rb.set_translation(@core.Vec2::new(1.0F, 1.0F), false) |> ignore
  }
  let mut saw_sliding = false
  for _ in 0..<20 {
    pipeline.step(
      gravity,
      integration_parameters,
      islands,
      broad_phase,
      narrow_phase,
      bodies,
      colliders,
      impulse_joints,
      multibody_joints,
      @dynamics.CCDSolver::new(),
      @pipeline.PhysicsHooks::new(),
      @pipeline.EventHandler::new(),
    )
    let filter_character_controller = @collision.QueryFilter::new().exclude_rigid_body(
      character,
    )
    let query_pipeline = broad_phase.as_query_pipeline(
      bodies, colliders, filter_character_controller,
    )
    if bodies.get(character) is Some(character_body) {
      let m = controller_steep.move_shape(
        dt,
        query_pipeline,
        bodies,
        colliders,
        character_shape,
        character_body.position(),
        @core.Vec2::new(0.1F, -0.1F),
        fn(_collision) {  },
      )
      saw_sliding = saw_sliding || m.is_sliding_down_slope()
      let t = character_body.translation()
      if bodies.get_mut(character) is Some(rb) {
        rb.set_next_kinematic_translation(t.add(m.translation())) |> ignore
      }
    }
  }
  inspect(saw_sliding, content="true")
}

///|
test "character controller slides along wall" {
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = @collision.ColliderSet::new()
  let wall_body = bodies.insert(@dynamics.RigidBodyBuilder::fixed().build())
  let wall = @collision.ColliderBuilder::cuboid(0.5F, 10.0F).build()
  colliders.insert_with_parent(wall, wall_body, bodies) |> ignore
  let broad_phase = @collision.BroadPhaseBvh::new()
  broad_phase.update(0.0F, bodies, colliders)
  let query_pipeline = broad_phase.as_query_pipeline(
    bodies,
    colliders,
    @collision.QueryFilter::new(),
  )
  let controller = KinematicCharacterController::new()
  let shape = @collision.Shape::Ball(0.5F)
  let start_pos = @core.Isometry2::from_translation(
    @core.Vec2::new(-2.0F, 0.0F),
  )
  let desired = @core.Vec2::new(2.0F, 1.0F)
  let movement = controller.move_shape(
    0.016F,
    query_pipeline,
    bodies,
    colliders,
    shape,
    start_pos,
    desired,
    fn(_collision) {  },
  )
  let applied = movement.translation()
  inspect(@core.abs(applied.x - 1.0F) < 1.0e-2F, content="true")
  inspect(@core.abs(applied.y - 1.0F) < 1.0e-2F, content="true")
  inspect(movement.is_grounded() == false, content="true")
}

///|
test "character controller collision event includes hit info" {
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = @collision.ColliderSet::new()
  let wall_body = bodies.insert(@dynamics.RigidBodyBuilder::fixed().build())
  let wall = @collision.ColliderBuilder::cuboid(0.5F, 10.0F).build()
  let wall_handle = colliders.insert_with_parent(wall, wall_body, bodies)
  let broad_phase = @collision.BroadPhaseBvh::new()
  broad_phase.update(0.0F, bodies, colliders)
  let query_pipeline = broad_phase.as_query_pipeline(
    bodies,
    colliders,
    @collision.QueryFilter::new(),
  )
  let collisions : Array[CharacterCollision] = []
  let controller = KinematicCharacterController::new()
  let shape = @collision.Shape::Ball(0.5F)
  let start_pos = @core.Isometry2::from_translation(
    @core.Vec2::new(-2.0F, 0.0F),
  )
  let movement = controller.move_shape(
    0.016F,
    query_pipeline,
    bodies,
    colliders,
    shape,
    start_pos,
    @core.Vec2::new(2.0F, 0.0F),
    fn(collision) { collisions.push(collision) },
  )
  inspect(movement.is_grounded() == false, content="true")
  inspect(collisions.length() >= 1, content="true")
  inspect(
    @collision.ColliderHandle::equals(collisions[0].handle(), wall_handle),
    content="true",
  )
  let n = collisions[0].hit().normal()
  inspect(@core.abs(n.x + 1.0F) < 1.0e-3F, content="true")
  inspect(@core.abs(n.y) < 1.0e-3F, content="true")
  let applied = collisions[0].translation_applied()
  inspect(@core.abs(applied.x - 1.0F) < 1.0e-2F, content="true")
}

///|
test "character controller follows kinematic platform velocity" {
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = @collision.ColliderSet::new()
  let platform_body = bodies.insert(
    @dynamics.RigidBodyBuilder::kinematic_velocity_based()
    .linvel(@core.Vec2::new(1.0F, 0.0F))
    .build(),
  )
  let platform = @collision.ColliderBuilder::cuboid(10.0F, 0.5F)
    .translation(@core.Vec2::new(0.0F, -0.5F))
    .build()
  colliders.insert_with_parent(platform, platform_body, bodies) |> ignore
  let broad_phase = @collision.BroadPhaseBvh::new()
  broad_phase.update(0.0F, bodies, colliders)
  let query_pipeline = broad_phase.as_query_pipeline(
    bodies,
    colliders,
    @collision.QueryFilter::new(),
  )
  let controller = KinematicCharacterController::new()
  let shape = @collision.Shape::Ball(0.5F)
  let start_pos = @core.Isometry2::from_translation(@core.Vec2::new(0.0F, 0.5F))
  let movement = controller.move_shape(
    1.0F,
    query_pipeline,
    bodies,
    colliders,
    shape,
    start_pos,
    @core.Vec2::zero(),
    fn(_collision) {  },
  )
  inspect(movement.is_grounded() == true, content="true")
  inspect(@core.abs(movement.translation().x - 1.0F) < 1.0e-3F, content="true")
}

///|
test "character controller autostep moves past obstacle" {
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = @collision.ColliderSet::new()
  let ground_body = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, -0.5F))
    .build(),
  )
  let ground = @collision.ColliderBuilder::cuboid(50.0F, 0.5F).build()
  colliders.insert_with_parent(ground, ground_body, bodies) |> ignore
  let obstacle = @collision.ColliderBuilder::cuboid(0.5F, 0.2F)
    .translation(@core.Vec2::new(0.0F, 0.2F))
    .build()
  colliders.insert(obstacle) |> ignore
  let broad_phase = @collision.BroadPhaseBvh::new()
  broad_phase.update(0.0F, bodies, colliders)
  let query_pipeline = broad_phase.as_query_pipeline(
    bodies,
    colliders,
    @collision.QueryFilter::new(),
  )
  let shape = @collision.Shape::Ball(0.5F)
  let start_pos = @core.Isometry2::from_translation(
    @core.Vec2::new(-2.0F, 0.5F),
  )
  let desired = @core.Vec2::new(4.0F, 0.0F)
  let controller_no_step = KinematicCharacterController::new()
  let no_step = controller_no_step.move_shape(
    0.016F,
    query_pipeline,
    bodies,
    colliders,
    shape,
    start_pos,
    desired,
    fn(_collision) {  },
  )
  inspect(no_step.translation().x < 2.0F, content="true")
  let controller_step = KinematicCharacterController::new().with_autostep(
    Some(
      CharacterAutostep::new(
        CharacterLength::absolute(0.6F),
        CharacterLength::absolute(2.2F),
      ),
    ),
  )
  let stepped = controller_step.move_shape(
    0.016F,
    query_pipeline,
    bodies,
    colliders,
    shape,
    start_pos,
    desired,
    fn(_collision) {  },
  )
  inspect(stepped.translation().x > 3.5F, content="true")
}

///|
/// Ported from rapier/src/control/character_controller.rs (character_controller_ground_detection) for 2D.
test "character controller ground detection" {
  let colliders = @collision.ColliderSet::new()
  let impulse_joints = @dynamics.ImpulseJointSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()
  let pipeline = @pipeline.PhysicsPipeline::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let islands = @dynamics.IslandManager::new()
  let bodies = @dynamics.RigidBodySet::new()
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let ground_size = 100.0F
  let ground_height = 1.0F
  let rigid_body = @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, -ground_height / 2.0F))
    .build()
  let floor_handle = bodies.insert(rigid_body)
  let collider = @collision.ColliderBuilder::cuboid(ground_size, ground_height).build()
  colliders.insert_with_parent(collider, floor_handle, bodies) |> ignore
  let integration_parameters = @dynamics.IntegrationParameters::default()
  let character_controller_snap = KinematicCharacterController::new().with_snap_to_ground(
    Some(CharacterLength::relative(0.2F)),
  )
  let character_body_snap = @dynamics.RigidBodyBuilder::kinematic_position_based()
    .additional_mass(1.0F)
    .build()
  character_body_snap.set_translation(@core.Vec2::new(0.6F, 0.5F), false)
  |> ignore
  let character_handle_snap = bodies.insert(character_body_snap)
  let collider = @collision.ColliderBuilder::ball(0.5F).build()
  colliders.insert_with_parent(collider, character_handle_snap, bodies)
  |> ignore
  let character_controller_no_snap = KinematicCharacterController::new().with_snap_to_ground(
    None,
  )
  let character_body_no_snap = @dynamics.RigidBodyBuilder::kinematic_position_based()
    .additional_mass(1.0F)
    .build()
  character_body_no_snap.set_translation(@core.Vec2::new(-0.6F, 0.5F), false)
  |> ignore
  let character_handle_no_snap = bodies.insert(character_body_no_snap)
  let collider = @collision.ColliderBuilder::ball(0.5F).build()
  let character_shape = collider.shape()
  colliders.insert_with_parent(collider, character_handle_no_snap, bodies)
  |> ignore
  for _ in 0..<500 {
    pipeline.step(
      gravity,
      integration_parameters,
      islands,
      broad_phase,
      narrow_phase,
      bodies,
      colliders,
      impulse_joints,
      multibody_joints,
      @dynamics.CCDSolver::new(),
      @pipeline.PhysicsHooks::new(),
      @pipeline.EventHandler::new(),
    )
    let update_character_controller = fn(
      controller : KinematicCharacterController,
      handle : @dynamics.RigidBodyHandle,
    ) -> Bool {
      if bodies.get(handle) is Some(character_body) {
        let filter = @collision.QueryFilter::new().exclude_rigid_body(handle)
        let query_pipeline = broad_phase.as_query_pipeline(
          bodies, colliders, filter,
        )
        let effective_movement = controller.move_shape(
          integration_parameters.dt,
          query_pipeline,
          bodies,
          colliders,
          character_shape,
          character_body.position(),
          @core.Vec2::new(0.1F, -0.1F),
          fn(_collision) {  },
        )
        let is_grounded = effective_movement.is_grounded()
        let translation = character_body.translation()
        character_body.set_next_kinematic_translation(
          translation.add(effective_movement.translation()),
        )
        |> ignore
        return is_grounded
      }
      return false
    }
    inspect(
      update_character_controller(
        character_controller_no_snap, character_handle_no_snap,
      ),
      content="true",
    )
    inspect(
      update_character_controller(
        character_controller_snap, character_handle_snap,
      ),
      content="true",
    )
  }
}

///|
test "character controller grounded on rotated cuboid (cuboid character)" {
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = @collision.ColliderSet::new()
  let angle = 0.3F
  let ground = @collision.ColliderBuilder::cuboid(5.0F, 0.2F)
    .translation(@core.Vec2::new(0.0F, -1.5F))
    .rotation(angle)
    .build()
  colliders.insert(ground) |> ignore
  let broad_phase = @collision.BroadPhaseBvh::new()
  broad_phase.update(0.0F, bodies, colliders)
  let query_pipeline = broad_phase.as_query_pipeline(
    bodies,
    colliders,
    @collision.QueryFilter::new(),
  )
  let controller = KinematicCharacterController::new()
  let shape = @collision.Shape::Cuboid(0.5F, 0.5F)
  let start_pos = @core.Isometry2::from_translation(@core.Vec2::new(0.0F, 2.0F))
  let movement = controller.move_shape(
    0.016F,
    query_pipeline,
    bodies,
    colliders,
    shape,
    start_pos,
    @core.Vec2::new(0.0F, -4.0F),
    fn(_collision) {  },
  )
  inspect(movement.is_grounded() == true, content="true")
}

///|
test "character controller grounded on rotated cuboid (capsule character)" {
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = @collision.ColliderSet::new()
  let angle = 0.3F
  let ground = @collision.ColliderBuilder::cuboid(5.0F, 0.2F)
    .translation(@core.Vec2::new(0.0F, -1.5F))
    .rotation(angle)
    .build()
  colliders.insert(ground) |> ignore
  let broad_phase = @collision.BroadPhaseBvh::new()
  broad_phase.update(0.0F, bodies, colliders)
  let query_pipeline = broad_phase.as_query_pipeline(
    bodies,
    colliders,
    @collision.QueryFilter::new(),
  )
  let controller = KinematicCharacterController::new()
  let shape = @collision.Shape::CapsuleY(0.5F, 0.3F)
  let start_pos = @core.Isometry2::from_translation(@core.Vec2::new(0.0F, 2.0F))
  let movement = controller.move_shape(
    0.016F,
    query_pipeline,
    bodies,
    colliders,
    shape,
    start_pos,
    @core.Vec2::new(0.0F, -4.0F),
    fn(_collision) {  },
  )
  inspect(movement.is_grounded() == true, content="true")
}

///|
test "character controller grounded on segment" {
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = @collision.ColliderSet::new()
  let ground = @collision.ColliderBuilder::segment(
      @core.Vec2::new(-10.0F, 0.0F),
      @core.Vec2::new(10.0F, 0.0F),
    )
    .translation(@core.Vec2::new(0.0F, -1.0F))
    .build()
  colliders.insert(ground) |> ignore
  let broad_phase = @collision.BroadPhaseBvh::new()
  broad_phase.update(0.0F, bodies, colliders)
  let query_pipeline = broad_phase.as_query_pipeline(
    bodies,
    colliders,
    @collision.QueryFilter::new(),
  )
  let controller = KinematicCharacterController::new()
  let shape = @collision.Shape::Ball(0.5F)
  let start_pos = @core.Isometry2::from_translation(@core.Vec2::new(0.0F, 2.0F))
  let movement = controller.move_shape(
    0.016F,
    query_pipeline,
    bodies,
    colliders,
    shape,
    start_pos,
    @core.Vec2::new(0.0F, -4.0F),
    fn(_collision) {  },
  )
  inspect(movement.is_grounded() == true, content="true")
}

///|
test "character controller transfers impulse to dynamic body" {
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = @collision.ColliderSet::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let islands = @dynamics.IslandManager::new()
  let impulse_joints = @dynamics.ImpulseJointSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()
  let ccd_solver = @dynamics.CCDSolver::new()
  let hooks = @pipeline.PhysicsHooks::new()
  let events = @pipeline.EventHandler::new()
  let pipeline = @pipeline.PhysicsPipeline::new()

  // Static floor so the pushed body doesn't fall away.
  let floor = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, -1.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(20.0F, 1.0F).build(),
    floor,
    bodies,
  )
  |> ignore

  // Dynamic box we will push using `solve_character_collision_impulses`.
  let box_handle = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(0.0F, 0.5F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(0.5F, 0.5F).build(),
    box_handle,
    bodies,
  )
  |> ignore

  // Ensure mass-properties are computed from colliders so impulses affect velocities.
  pipeline.step(
    @core.Vec2::zero(),
    @dynamics.IntegrationParameters::default().set_dt(0.0F),
    islands,
    broad_phase,
    narrow_phase,
    bodies,
    colliders,
    impulse_joints,
    multibody_joints,
    ccd_solver,
    hooks,
    events,
  )
  let query_pipeline = broad_phase.as_query_pipeline(
    bodies,
    colliders,
    @collision.QueryFilter::new(),
  )
  let controller = KinematicCharacterController::new()
  let dt = 0.016F
  let character_shape = @collision.Shape::Ball(0.5F)
  let start_pos = @core.Isometry2::from_translation(
    @core.Vec2::new(-2.0F, 0.5F),
  )
  let collisions : Array[CharacterCollision] = []
  let movement = controller.move_shape(
    dt,
    query_pipeline,
    bodies,
    colliders,
    character_shape,
    start_pos,
    @core.Vec2::new(4.0F, 0.0F),
    fn(c) { collisions.push(c) },
  )
  // We should have hit the box.
  inspect(movement.translation().x > 0.0F, content="true")
  inspect(collisions.length() > 0, content="true")
  controller.solve_character_collision_impulses(
    dt, query_pipeline, bodies, colliders, character_shape, 1.0F, collisions,
  )
  if bodies.get(box_handle) is Some(body) {
    inspect(body.linvel().x > 0.0F, content="true")
  } else {
    inspect(false, content="true")
  }
}
