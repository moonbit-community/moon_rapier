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
