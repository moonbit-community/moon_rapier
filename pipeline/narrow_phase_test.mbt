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
/// Ported from rapier/src/geometry/narrow_phase.rs (collider_set_parent_depenetration) adapted for 2D.
test "collider set parent depenetration" {
  let rigid_body_set = @dynamics.RigidBodySet::new()
  let collider_set = @collision.ColliderSet::new()
  let collider = @collision.ColliderBuilder::ball(0.5F)
  let rigid_body_1 = @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::zero())
    .build()
  let body_1_handle = rigid_body_set.insert(rigid_body_1)
  let collider_1_handle = collider_set.insert_with_parent(
    collider.build(),
    body_1_handle,
    rigid_body_set,
  )
  let collider_2_handle = collider_set.insert_with_parent(
    collider.build(),
    body_1_handle,
    rigid_body_set,
  )
  let rigid_body_2 = @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::zero())
    .build()
  let body_2_handle = rigid_body_set.insert(rigid_body_2)
  let gravity = @core.Vec2::zero()
  let integration_parameters = IntegrationParameters::default()
  let physics_pipeline = PhysicsPipeline::new()
  let island_manager = @dynamics.IslandManager::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let impulse_joint_set = @dynamics.ImpulseJointSet::new()
  let multibody_joint_set = @dynamics.MultibodyJointSet::new()
  let ccd_solver = @dynamics.CCDSolver::new()
  physics_pipeline.step(
    gravity,
    integration_parameters,
    island_manager,
    broad_phase,
    narrow_phase,
    rigid_body_set,
    collider_set,
    impulse_joint_set,
    multibody_joint_set,
    ccd_solver,
    (),
    EventHandler::new(),
  )
  if collider_set.get(collider_1_handle) is Some(c1) {
    if collider_set.get(collider_2_handle) is Some(c2) {
      let dist = c1.translation().sub(c2.translation()).length()
      inspect(dist < 0.5F, content="true")
    }
  }
  let contact_pair = narrow_phase.contact_pair(
    collider_1_handle, collider_2_handle,
  )
  if contact_pair is Some(pair) {
    inspect(pair.manifold_count() == 0, content="true")
  }
  let intersection_pair = narrow_phase.intersection_pair(
    collider_1_handle, collider_2_handle,
  )
  inspect(intersection_pair is None, content="true")
  collider_set.set_parent(
    collider_2_handle,
    Some(body_2_handle),
    rigid_body_set,
  )
  physics_pipeline.step(
    gravity,
    integration_parameters,
    island_manager,
    broad_phase,
    narrow_phase,
    rigid_body_set,
    collider_set,
    impulse_joint_set,
    multibody_joint_set,
    ccd_solver,
    (),
    EventHandler::new(),
  )
  let contact_pair2 = narrow_phase.contact_pair(
    collider_1_handle, collider_2_handle,
  )
  if contact_pair2 is Some(pair) {
    inspect(pair.manifold_count() == 1, content="true")
  }
  for _ in 0..<200 {
    physics_pipeline.step(
      gravity,
      integration_parameters,
      island_manager,
      broad_phase,
      narrow_phase,
      rigid_body_set,
      collider_set,
      impulse_joint_set,
      multibody_joint_set,
      ccd_solver,
      (),
      EventHandler::new(),
    )
  }
  if collider_set.get(collider_1_handle) is Some(c1) {
    if collider_set.get(collider_2_handle) is Some(c2) {
      let dist = c1.translation().sub(c2.translation()).length()
      inspect(dist >= 0.5F, content="true")
    }
  }
}

///|
/// Ported from rapier/src/geometry/narrow_phase.rs (collider_set_parent_no_self_intersection) adapted for 2D.
test "collider set parent no self intersection" {
  let rigid_body_set = @dynamics.RigidBodySet::new()
  let collider_set = @collision.ColliderSet::new()
  let collider = @collision.ColliderBuilder::ball(0.5F)
  let rigid_body_1 = @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::zero())
    .build()
  let body_1_handle = rigid_body_set.insert(rigid_body_1)
  let collider_1_handle = collider_set.insert_with_parent(
    collider.build(),
    body_1_handle,
    rigid_body_set,
  )
  let rigid_body_2 = @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::zero())
    .build()
  let body_2_handle = rigid_body_set.insert(rigid_body_2)
  let collider_2_handle = collider_set.insert_with_parent(
    collider.build(),
    body_2_handle,
    rigid_body_set,
  )
  let gravity = @core.Vec2::zero()
  let integration_parameters = IntegrationParameters::default()
  let physics_pipeline = PhysicsPipeline::new()
  let island_manager = @dynamics.IslandManager::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let impulse_joint_set = @dynamics.ImpulseJointSet::new()
  let multibody_joint_set = @dynamics.MultibodyJointSet::new()
  let ccd_solver = @dynamics.CCDSolver::new()
  physics_pipeline.step(
    gravity,
    integration_parameters,
    island_manager,
    broad_phase,
    narrow_phase,
    rigid_body_set,
    collider_set,
    impulse_joint_set,
    multibody_joint_set,
    ccd_solver,
    (),
    EventHandler::new(),
  )
  let contact_pair = narrow_phase.contact_pair(
    collider_1_handle, collider_2_handle,
  )
  if contact_pair is Some(pair) {
    inspect(pair.manifold_count() == 1, content="true")
  }
  collider_set.set_parent(
    collider_2_handle,
    Some(body_1_handle),
    rigid_body_set,
  )
  physics_pipeline.step(
    gravity,
    integration_parameters,
    island_manager,
    broad_phase,
    narrow_phase,
    rigid_body_set,
    collider_set,
    impulse_joint_set,
    multibody_joint_set,
    ccd_solver,
    (),
    EventHandler::new(),
  )
  let contact_pair2 = narrow_phase.contact_pair(
    collider_1_handle, collider_2_handle,
  )
  if contact_pair2 is Some(pair) {
    inspect(pair.manifold_count() == 0, content="true")
  }
  collider_set.set_parent(
    collider_2_handle,
    Some(body_2_handle),
    rigid_body_set,
  )
  physics_pipeline.step(
    gravity,
    integration_parameters,
    island_manager,
    broad_phase,
    narrow_phase,
    rigid_body_set,
    collider_set,
    impulse_joint_set,
    multibody_joint_set,
    ccd_solver,
    (),
    EventHandler::new(),
  )
  let contact_pair3 = narrow_phase.contact_pair(
    collider_1_handle, collider_2_handle,
  )
  if contact_pair3 is Some(pair) {
    inspect(pair.manifold_count() == 1, content="true")
  }
}

///|
/// Ported from rapier narrow-phase collision filtering (collision groups).
test "narrow phase collision groups filtering" {
  let rigid_body_set = @dynamics.RigidBodySet::new()
  let collider_set = @collision.ColliderSet::new()
  let body1 = @dynamics.RigidBodyBuilder::dynamic().build()
  let body2 = @dynamics.RigidBodyBuilder::dynamic().build()
  let body1_handle = rigid_body_set.insert(body1)
  let body2_handle = rigid_body_set.insert(body2)
  let groups1 = @dynamics.InteractionGroups::new(
    @dynamics.Group::group_1(),
    @dynamics.Group::group_1(),
    @dynamics.InteractionTestMode::And,
  )
  let groups2 = @dynamics.InteractionGroups::new(
    @dynamics.Group::group_2(),
    @dynamics.Group::group_2(),
    @dynamics.InteractionTestMode::And,
  )
  let collider1 = @collision.ColliderBuilder::ball(0.5F)
    .collision_groups(groups1)
    .build()
  let collider2 = @collision.ColliderBuilder::ball(0.5F)
    .collision_groups(groups2)
    .build()
  let collider1_handle = collider_set.insert_with_parent(
    collider1, body1_handle, rigid_body_set,
  )
  let collider2_handle = collider_set.insert_with_parent(
    collider2, body2_handle, rigid_body_set,
  )
  let physics_pipeline = PhysicsPipeline::new()
  let island_manager = @dynamics.IslandManager::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let impulse_joint_set = @dynamics.ImpulseJointSet::new()
  let multibody_joint_set = @dynamics.MultibodyJointSet::new()
  let ccd_solver = @dynamics.CCDSolver::new()
  physics_pipeline.step(
    @core.Vec2::zero(),
    IntegrationParameters::default(),
    island_manager,
    broad_phase,
    narrow_phase,
    rigid_body_set,
    collider_set,
    impulse_joint_set,
    multibody_joint_set,
    ccd_solver,
    (),
    EventHandler::new(),
  )
  inspect(
    narrow_phase.contact_pair(collider1_handle, collider2_handle) is None,
    content="true",
  )
  inspect(
    narrow_phase.intersection_pair(collider1_handle, collider2_handle) is None,
    content="true",
  )
}

///|
/// Ported from rapier narrow-phase collision filtering (active collision types).
test "narrow phase active collision types filtering" {
  let rigid_body_set = @dynamics.RigidBodySet::new()
  let collider_set = @collision.ColliderSet::new()
  let fixed = @dynamics.RigidBodyBuilder::fixed().build()
  let kinematic = @dynamics.RigidBodyBuilder::kinematic_position_based().build()
  let fixed_handle = rigid_body_set.insert(fixed)
  let kinematic_handle = rigid_body_set.insert(kinematic)
  let collider_fixed = @collision.ColliderBuilder::ball(0.5F).build()
  let collider_kinematic = @collision.ColliderBuilder::ball(0.5F).build()
  let collider_fixed_handle = collider_set.insert_with_parent(
    collider_fixed, fixed_handle, rigid_body_set,
  )
  let collider_kinematic_handle = collider_set.insert_with_parent(
    collider_kinematic, kinematic_handle, rigid_body_set,
  )
  let physics_pipeline = PhysicsPipeline::new()
  let island_manager = @dynamics.IslandManager::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let impulse_joint_set = @dynamics.ImpulseJointSet::new()
  let multibody_joint_set = @dynamics.MultibodyJointSet::new()
  let ccd_solver = @dynamics.CCDSolver::new()
  physics_pipeline.step(
    @core.Vec2::zero(),
    IntegrationParameters::default(),
    island_manager,
    broad_phase,
    narrow_phase,
    rigid_body_set,
    collider_set,
    impulse_joint_set,
    multibody_joint_set,
    ccd_solver,
    (),
    EventHandler::new(),
  )
  inspect(
    narrow_phase.contact_pair(collider_fixed_handle, collider_kinematic_handle)
    is None,
    content="true",
  )
}
