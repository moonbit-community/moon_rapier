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
test "modify_solver_contacts_with_solver_contacts can disable impulse computation" {
  let pipeline = PhysicsPipeline::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = @collision.ColliderSet::new()
  let islands = @dynamics.IslandManager::new()
  let impulse_joints = @dynamics.ImpulseJointSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()
  let handler = EventHandler::new()
  let ccd_solver = @dynamics.CCDSolver::new()
  let parameters = IntegrationParameters::default()
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let floor = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, -0.1F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(5.0F, 0.1F)
    .active_hooks(@collision.ActiveHooks::modify_solver_contacts())
    .build(),
    floor,
    bodies,
  )
  |> ignore
  let ball = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(0.0F, 1.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::ball(0.5F)
    .active_hooks(@collision.ActiveHooks::modify_solver_contacts())
    .build(),
    ball,
    bodies,
  )
  |> ignore
  let mut inspected = false
  let mut saw_compute_impulses_default = false
  let mut saw_contacts_nonempty = false
  let mut saw_solver_contact_mutations = false
  let hooks = PhysicsHooks::new().modify_solver_contacts_with_solver_contacts(fn(
    _bodies : @dynamics.RigidBodySet,
    _colliders : @collision.ColliderSet,
    _collider1 : @collision.ColliderHandle,
    _collider2 : @collision.ColliderHandle,
    _manifold : @collision.ContactManifold,
    solver_contacts : Array[@collision.SolverContact],
    solver_flags : @collision.SolverFlags,
  ) -> @collision.SolverFlags {
    if !inspected {
      inspected = true
      saw_compute_impulses_default = solver_flags.contains(
        @collision.SolverFlags::compute_impulses(),
      )
      saw_contacts_nonempty = solver_contacts.length() > 0
      let sc = solver_contacts[0]
      sc.set_friction(0.25F)
      sc.set_restitution(0.5F)
      sc.set_tangent_velocity(@core.Vec2::new(1.0F, 2.0F))
      saw_solver_contact_mutations = sc.friction() == 0.25F &&
        sc.restitution() == 0.5F &&
        sc.tangent_velocity().x == 1.0F &&
        sc.tangent_velocity().y == 2.0F
    }
    @collision.SolverFlags::empty()
  })
  for _ in 0..<240 {
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
      impulse_joints, multibody_joints, ccd_solver, hooks, handler,
    )
  }
  if bodies.get(ball) is Some(body) {
    // Without impulses the body should keep falling through the floor.
    inspect(body.translation().y < -1.0F, content="true")
    inspect(saw_compute_impulses_default, content="true")
    inspect(saw_contacts_nonempty, content="true")
    inspect(saw_solver_contact_mutations, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "modify_solver_contacts_with_solver_contacts can apply tangent velocity" {
  let pipeline = PhysicsPipeline::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = @collision.ColliderSet::new()
  let islands = @dynamics.IslandManager::new()
  let impulse_joints = @dynamics.ImpulseJointSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()
  let handler = EventHandler::new()
  let ccd_solver = @dynamics.CCDSolver::new()
  let parameters = IntegrationParameters::default()
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let floor = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, -0.1F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(5.0F, 0.1F)
    .active_hooks(@collision.ActiveHooks::modify_solver_contacts())
    .build(),
    floor,
    bodies,
  )
  |> ignore
  let ball = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(0.0F, 0.75F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::ball(0.5F)
    .active_hooks(@collision.ActiveHooks::modify_solver_contacts())
    .build(),
    ball,
    bodies,
  )
  |> ignore
  let hooks = PhysicsHooks::new().modify_solver_contacts_with_solver_contacts(fn(
    _bodies : @dynamics.RigidBodySet,
    _colliders : @collision.ColliderSet,
    _collider1 : @collision.ColliderHandle,
    _collider2 : @collision.ColliderHandle,
    _manifold : @collision.ContactManifold,
    solver_contacts : Array[@collision.SolverContact],
    solver_flags : @collision.SolverFlags,
  ) -> @collision.SolverFlags {
    for i in 0..<solver_contacts.length() {
      let sc = solver_contacts[i]
      sc.set_friction(10.0F)
      sc.set_tangent_velocity(@core.Vec2::new(5.0F, 0.0F))
    }
    solver_flags
  })
  for _ in 0..<240 {
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
      impulse_joints, multibody_joints, ccd_solver, hooks, handler,
    )
  }
  if bodies.get(ball) is Some(body) {
    inspect(@core.abs(body.translation().x) > 1.0F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "filter_contact_pair_with_user_data resolves collider and body user_data" {
  let pipeline = PhysicsPipeline::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = @collision.ColliderSet::new()
  let islands = @dynamics.IslandManager::new()
  let impulse_joints = @dynamics.ImpulseJointSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()
  let handler = EventHandler::new()
  let ccd_solver = @dynamics.CCDSolver::new()
  let parameters = IntegrationParameters::default()
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let floor_collider_ud = @core.UserData128::from_int_truncate(11)
  let ball_collider_ud = @core.UserData128::from_int_truncate(22)
  let floor_body_ud = @core.UserData128::from_int_truncate(111)
  let ball_body_ud = @core.UserData128::from_int_truncate(222)
  let floor = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, -0.1F))
    .user_data128(floor_body_ud)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(5.0F, 0.1F)
    .active_hooks(@collision.ActiveHooks::filter_contact_pairs())
    .user_data128(floor_collider_ud)
    .build(),
    floor,
    bodies,
  )
  |> ignore
  let ball = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(0.0F, 1.0F))
    .can_sleep(false)
    .user_data128(ball_body_ud)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::ball(0.5F)
    .active_hooks(@collision.ActiveHooks::filter_contact_pairs())
    .user_data128(ball_collider_ud)
    .build(),
    ball,
    bodies,
  )
  |> ignore
  let mut called = false
  let mut saw_collider_ud = false
  let mut saw_body_ud = false
  let hooks = PhysicsHooks::new().filter_contact_pair_with_user_data(fn(
    _bodies : @dynamics.RigidBodySet,
    _colliders : @collision.ColliderSet,
    _collider1 : @collision.ColliderHandle,
    _collider2 : @collision.ColliderHandle,
    ud1 : @core.UserData128,
    ud2 : @core.UserData128,
    rb1 : @dynamics.RigidBodyHandle?,
    rb2 : @dynamics.RigidBodyHandle?,
    rb_ud1 : @core.UserData128?,
    rb_ud2 : @core.UserData128?,
  ) -> Bool {
    called = true
    saw_collider_ud = (
        ud1.equals(floor_collider_ud) && ud2.equals(ball_collider_ud)
      ) ||
      (ud1.equals(ball_collider_ud) && ud2.equals(floor_collider_ud))
    saw_body_ud = rb1 is Some(_) &&
      rb2 is Some(_) &&
      rb_ud1 is Some(bud1) &&
      rb_ud2 is Some(bud2) &&
      (
        (bud1.equals(floor_body_ud) && bud2.equals(ball_body_ud)) ||
        (bud1.equals(ball_body_ud) && bud2.equals(floor_body_ud))
      )
    // Disable contacts for this pair so the ball falls through.
    false
  })
  for _ in 0..<240 {
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
      impulse_joints, multibody_joints, ccd_solver, hooks, handler,
    )
  }
  inspect(called, content="true")
  inspect(saw_collider_ud, content="true")
  inspect(saw_body_ud, content="true")
  if bodies.get(ball) is Some(body) {
    inspect(body.translation().y < -1.0F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "modify_solver_contacts_with_solver_contacts_with_user_data sees collider user_data" {
  let pipeline = PhysicsPipeline::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = @collision.ColliderSet::new()
  let islands = @dynamics.IslandManager::new()
  let impulse_joints = @dynamics.ImpulseJointSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()
  let handler = EventHandler::new()
  let ccd_solver = @dynamics.CCDSolver::new()
  let parameters = IntegrationParameters::default()
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let floor_ud = @core.UserData128::from_int_truncate(1001)
  let ball_ud = @core.UserData128::from_int_truncate(1002)
  let floor = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, -0.1F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(5.0F, 0.1F)
    .active_hooks(@collision.ActiveHooks::modify_solver_contacts())
    .user_data128(floor_ud)
    .build(),
    floor,
    bodies,
  )
  |> ignore
  let ball = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(0.0F, 0.75F))
    .can_sleep(false)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::ball(0.5F)
    .active_hooks(@collision.ActiveHooks::modify_solver_contacts())
    .user_data128(ball_ud)
    .build(),
    ball,
    bodies,
  )
  |> ignore
  let mut inspected = false
  let hooks = PhysicsHooks::new().modify_solver_contacts_with_solver_contacts_with_user_data(fn(
      _bodies : @dynamics.RigidBodySet,
      _colliders : @collision.ColliderSet,
      _collider1 : @collision.ColliderHandle,
      _collider2 : @collision.ColliderHandle,
      ud1 : @core.UserData128,
      ud2 : @core.UserData128,
      _rb1 : @dynamics.RigidBodyHandle?,
      _rb2 : @dynamics.RigidBodyHandle?,
      _rb_ud1 : @core.UserData128?,
      _rb_ud2 : @core.UserData128?,
      _manifold : @collision.ContactManifold,
      _solver_contacts : Array[@collision.SolverContact],
      solver_flags : @collision.SolverFlags,
    ) -> @collision.SolverFlags {
      if !inspected {
        inspected = (ud1.equals(floor_ud) && ud2.equals(ball_ud)) ||
          (ud1.equals(ball_ud) && ud2.equals(floor_ud))
      }
      solver_flags
    },
  )
  for _ in 0..<240 {
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
      impulse_joints, multibody_joints, ccd_solver, hooks, handler,
    )
  }
  inspect(inspected, content="true")
}
