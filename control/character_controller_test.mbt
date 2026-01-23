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
/// Ported from rapier/src/control/character_controller.rs (character_controller_climb_test) adapted for 2D.
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
  let integration_parameters = @pipeline.IntegrationParameters::default()
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
      (),
      (),
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
    shape,
    start_pos,
    desired,
    fn(_collision) {  },
  )
  inspect(stepped.translation().x > 3.5F, content="true")
}

///|
/// Ported from rapier/src/control/character_controller.rs (character_controller_ground_detection) adapted for 2D.
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
  let integration_parameters = @pipeline.IntegrationParameters::default()
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
      (),
      (),
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
