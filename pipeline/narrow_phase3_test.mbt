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
/// Ported from rapier/src/geometry/narrow_phase.rs (dim3,f32 collider_set_parent_depenetration).
test "collider set parent depenetration (dim3)" {
  let rigid_body_set = @dynamics.RigidBodySet3::new()
  let collider_set = @collision.ColliderSet3::new()
  let collider = @collision.ColliderBuilder3::ball(0.5F)
  let body_1_handle = rigid_body_set.insert(
    @dynamics.RigidBodyBuilder3::dynamic()
    .translation(@core.Vec3::zero())
    .build(),
  )
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
  let body_2_handle = rigid_body_set.insert(
    @dynamics.RigidBodyBuilder3::dynamic()
    .translation(@core.Vec3::zero())
    .build(),
  )
  let gravity = @core.Vec3::zero()
  let integration_parameters = @dynamics.IntegrationParameters::default()
  let physics_pipeline = PhysicsPipeline3::new()
  let island_manager = @dynamics.IslandManager3::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let impulse_joint_set = @dynamics.ImpulseJointSet3::new()
  let multibody_joint_set = @dynamics.MultibodyJointSet3::new()
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
    PhysicsHooks::new(),
    EventHandler::new(),
  )
  if collider_set.get(collider_1_handle) is Some(c1) {
    if collider_set.get(collider_2_handle) is Some(c2) {
      let dist = c1.translation3().sub(c2.translation3()).length()
      inspect(dist < 0.5F, content="true")
    }
  }
  let contact_pair = narrow_phase.contact_pair(
    collider_1_handle, collider_2_handle,
  )
  if contact_pair is Some(pair) {
    inspect(pair.manifold_count() == 0, content="true")
  }
  inspect(
    narrow_phase.intersection_pair(collider_1_handle, collider_2_handle) is None,
    content="true",
  )
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
    PhysicsHooks::new(),
    EventHandler::new(),
  )
  let contact_pair2 = narrow_phase.contact_pair(
    collider_1_handle, collider_2_handle,
  )
  if contact_pair2 is Some(pair) {
    inspect(pair.manifold_count() == 1, content="true")
  }
  inspect(
    narrow_phase.intersection_pair(collider_1_handle, collider_2_handle) is None,
    content="true",
  )
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
      PhysicsHooks::new(),
      EventHandler::new(),
    )
  }
  if collider_set.get(collider_1_handle) is Some(c1) {
    if collider_set.get(collider_2_handle) is Some(c2) {
      let dist = c1.translation3().sub(c2.translation3()).length()
      inspect(dist >= 0.5F, content="true")
    }
  }
}

///|
/// Ported from rapier/src/geometry/narrow_phase.rs (dim3,f32 collider_set_parent_no_self_intersection).
test "collider set parent no self intersection (dim3)" {
  let rigid_body_set = @dynamics.RigidBodySet3::new()
  let collider_set = @collision.ColliderSet3::new()
  let collider = @collision.ColliderBuilder3::ball(0.5F)
  let body_1_handle = rigid_body_set.insert(
    @dynamics.RigidBodyBuilder3::dynamic()
    .translation(@core.Vec3::zero())
    .build(),
  )
  let collider_1_handle = collider_set.insert_with_parent(
    collider.build(),
    body_1_handle,
    rigid_body_set,
  )
  let body_2_handle = rigid_body_set.insert(
    @dynamics.RigidBodyBuilder3::dynamic()
    .translation(@core.Vec3::zero())
    .build(),
  )
  let collider_2_handle = collider_set.insert_with_parent(
    collider.build(),
    body_2_handle,
    rigid_body_set,
  )
  let gravity = @core.Vec3::zero()
  let integration_parameters = @dynamics.IntegrationParameters::default()
  let physics_pipeline = PhysicsPipeline3::new()
  let island_manager = @dynamics.IslandManager3::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let impulse_joint_set = @dynamics.ImpulseJointSet3::new()
  let multibody_joint_set = @dynamics.MultibodyJointSet3::new()
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
    PhysicsHooks::new(),
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
    PhysicsHooks::new(),
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
    PhysicsHooks::new(),
    EventHandler::new(),
  )
  let contact_pair3 = narrow_phase.contact_pair(
    collider_1_handle, collider_2_handle,
  )
  if contact_pair3 is Some(pair) {
    inspect(pair.manifold_count() == 1, content="true")
  }
}
