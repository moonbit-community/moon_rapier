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
    PhysicsHooks::new(),
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
    PhysicsHooks::new(),
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
      PhysicsHooks::new(),
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
    PhysicsHooks::new(),
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
test "narrow phase contact manifolds: basic shapes" {
  // Ball-ball.
  {
    let rigid_body_set = @dynamics.RigidBodySet::new()
    let collider_set = @collision.ColliderSet::new()
    let narrow_phase = @collision.NarrowPhase::new()
    let b1 = rigid_body_set.insert(
      @dynamics.RigidBodyBuilder::dynamic()
      .translation(@core.Vec2::zero())
      .build(),
    )
    let b2 = rigid_body_set.insert(
      @dynamics.RigidBodyBuilder::dynamic()
      .translation(@core.Vec2::new(0.75F, 0.0F))
      .build(),
    )
    let c1 = collider_set.insert_with_parent(
      @collision.ColliderBuilder::ball(0.5F).build(),
      b1,
      rigid_body_set,
    )
    let c2 = collider_set.insert_with_parent(
      @collision.ColliderBuilder::ball(0.5F).build(),
      b2,
      rigid_body_set,
    )
    narrow_phase.update(rigid_body_set, collider_set)
    let pair = narrow_phase.contact_pair(c1, c2)
    if pair is Some(cp) {
      inspect(cp.manifold_count() == 1, content="true")
      let m = cp.manifolds()[0]
      inspect(@core.abs(m.normal().x - 1.0F) < 1.0e-3F, content="true")
      inspect(@core.abs(m.normal().y) < 1.0e-3F, content="true")
      let pt = m.points()[0]
      inspect(@core.abs(pt.dist() + 0.25F) < 1.0e-3F, content="true")
    } else {
      inspect(false, content="true")
    }
  }

  // Ball-cuboid (ball is collider1, cuboid is collider2).
  {
    let rigid_body_set = @dynamics.RigidBodySet::new()
    let collider_set = @collision.ColliderSet::new()
    let narrow_phase = @collision.NarrowPhase::new()
    let b3 = rigid_body_set.insert(
      @dynamics.RigidBodyBuilder::dynamic()
      .translation(@core.Vec2::zero())
      .build(),
    )
    let b4 = rigid_body_set.insert(
      @dynamics.RigidBodyBuilder::fixed()
      .translation(@core.Vec2::new(1.5F, 0.0F))
      .build(),
    )
    let ball = collider_set.insert_with_parent(
      @collision.ColliderBuilder::ball(1.0F).build(),
      b3,
      rigid_body_set,
    )
    let cuboid = collider_set.insert_with_parent(
      @collision.ColliderBuilder::cuboid(1.0F, 1.0F).build(),
      b4,
      rigid_body_set,
    )
    narrow_phase.update(rigid_body_set, collider_set)
    let pair2 = narrow_phase.contact_pair(ball, cuboid)
    if pair2 is Some(cp) {
      inspect(cp.manifold_count() == 1, content="true")
      let m = cp.manifolds()[0]
      inspect(@core.abs(m.normal().x - 1.0F) < 1.0e-3F, content="true")
      let pt = m.points()[0]
      inspect(@core.abs(pt.dist() + 0.5F) < 1.0e-3F, content="true")
      match pt.fid2() {
        @collision.FeatureId::Edge(id) => inspect(id == 1, content="true")
        _ => inspect(false, content="true")
      }
    } else {
      inspect(false, content="true")
    }
  }

  // Cuboid-cuboid (expect two contact points with stable vertex ids).
  {
    let rigid_body_set = @dynamics.RigidBodySet::new()
    let collider_set = @collision.ColliderSet::new()
    let narrow_phase = @collision.NarrowPhase::new()
    let a = rigid_body_set.insert(
      @dynamics.RigidBodyBuilder::dynamic()
      .translation(@core.Vec2::zero())
      .build(),
    )
    let b = rigid_body_set.insert(
      @dynamics.RigidBodyBuilder::fixed()
      .translation(@core.Vec2::new(1.5F, 0.0F))
      .build(),
    )
    let ca = collider_set.insert_with_parent(
      @collision.ColliderBuilder::cuboid(1.0F, 1.0F).build(),
      a,
      rigid_body_set,
    )
    let cb = collider_set.insert_with_parent(
      @collision.ColliderBuilder::cuboid(1.0F, 1.0F).build(),
      b,
      rigid_body_set,
    )
    narrow_phase.update(rigid_body_set, collider_set)
    let pair = narrow_phase.contact_pair(ca, cb)
    if pair is Some(cp) {
      inspect(cp.manifold_count() == 1, content="true")
      let m = cp.manifolds()[0]
      inspect(m.points().length() == 2, content="true")
      let pts = m.points()
      inspect(@core.abs(pts[0].dist() + 0.5F) < 1.0e-3F, content="true")
      inspect(@core.abs(pts[1].dist() + 0.5F) < 1.0e-3F, content="true")
      let mut vertex_count = 0
      for i in 0..<pts.length() {
        let p = pts[i]
        match (p.fid1(), p.fid2()) {
          (@collision.FeatureId::Vertex(_), _) =>
            vertex_count = vertex_count + 1
          (_, @collision.FeatureId::Vertex(_)) =>
            vertex_count = vertex_count + 1
          _ => ()
        }
      }
      inspect(vertex_count == 2, content="true")
    } else {
      inspect(false, content="true")
    }
  }

  // Capsule-ball (capsule is collider1).
  let rigid_body_set = @dynamics.RigidBodySet::new()
  let collider_set = @collision.ColliderSet::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let a = rigid_body_set.insert(
    @dynamics.RigidBodyBuilder::fixed().translation(@core.Vec2::zero()).build(),
  )
  let b = rigid_body_set.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(1.8F, 0.0F))
    .build(),
  )
  let capsule = collider_set.insert_with_parent(
    @collision.ColliderBuilder::capsule_x(1.0F, 0.5F).build(),
    a,
    rigid_body_set,
  )
  let ball = collider_set.insert_with_parent(
    @collision.ColliderBuilder::ball(0.5F).build(),
    b,
    rigid_body_set,
  )
  narrow_phase.update(rigid_body_set, collider_set)
  let pair = narrow_phase.contact_pair(capsule, ball)
  if pair is Some(cp) {
    let pt = cp.manifolds()[0].points()[0]
    inspect(@core.abs(pt.dist() + 0.2F) < 1.0e-2F, content="true")
    match pt.fid1() {
      @collision.FeatureId::Vertex(id) => inspect(id == 1, content="true")
      _ => inspect(false, content="true")
    }
  } else {
    inspect(false, content="true")
  }
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
    PhysicsHooks::new(),
    EventHandler::new(),
  )
  inspect(
    narrow_phase.contact_pair(collider_fixed_handle, collider_kinematic_handle)
    is None,
    content="true",
  )
}
