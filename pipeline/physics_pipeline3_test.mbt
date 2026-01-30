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
/// Ported from rapier/src/pipeline/physics_pipeline.rs (kinematic_and_fixed_contact_crash) for dim3.
test "physics pipeline3 kinematic and fixed contact crash" {
  let colliders = @collision.ColliderSet3::new()
  let impulse_joints = @dynamics.ImpulseJointSet3::new()
  let multibody_joints = @dynamics.MultibodyJointSet3::new()
  let pipeline = PhysicsPipeline3::new()
  let bf = @collision.BroadPhaseBvh::new()
  let nf = @collision.NarrowPhase::new()
  let bodies = @dynamics.RigidBodySet3::new()
  let islands = @dynamics.IslandManager3::new()
  let rb_fixed = @dynamics.RigidBodyBuilder3::fixed().build()
  let h1 = bodies.insert(rb_fixed)
  let co = @collision.ColliderBuilder3::ball(10.0F).build()
  colliders.insert_with_parent(co, h1, bodies) |> ignore
  let rb_kinematic = @dynamics.RigidBodyBuilder3::kinematic_position_based().build()
  let h2 = bodies.insert(rb_kinematic)
  colliders.insert_with_parent(
    @collision.ColliderBuilder3::ball(10.0F).build(),
    h2,
    bodies,
  )
  |> ignore
  pipeline.step(
    @core.Vec3::zero(),
    @dynamics.IntegrationParameters::default(),
    islands,
    bf,
    nf,
    bodies,
    colliders,
    impulse_joints,
    multibody_joints,
    @dynamics.CCDSolver::new(),
    PhysicsHooks::new(),
    EventHandler::new(),
  )
}

///|
/// Ported from rapier/src/pipeline/physics_pipeline.rs (rigid_body_removal_before_step) for dim3.
test "physics pipeline3 rigid body removal before step" {
  let colliders = @collision.ColliderSet3::new()
  let impulse_joints = @dynamics.ImpulseJointSet3::new()
  let multibody_joints = @dynamics.MultibodyJointSet3::new()
  let pipeline = PhysicsPipeline3::new()
  let bf = @collision.BroadPhaseBvh::new()
  let nf = @collision.NarrowPhase::new()
  let islands = @dynamics.IslandManager3::new()
  let bodies = @dynamics.RigidBodySet3::new()
  let rb_dyn = @dynamics.RigidBodyBuilder3::dynamic().build()
  let h1 = bodies.insert(rb_dyn)
  let h2 = bodies.insert(@dynamics.RigidBodyBuilder3::dynamic().build())
  let h3 = bodies.insert(
    @dynamics.RigidBodyBuilder3::kinematic_position_based().build(),
  )
  let h4 = bodies.insert(@dynamics.RigidBodyBuilder3::fixed().build())
  for h in [h1, h2, h3, h4] {
    bodies.remove(h, islands, (), impulse_joints, multibody_joints, true)
    |> ignore
  }
  pipeline.step(
    @core.Vec3::zero(),
    @dynamics.IntegrationParameters::default(),
    islands,
    bf,
    nf,
    bodies,
    colliders,
    impulse_joints,
    multibody_joints,
    @dynamics.CCDSolver::new(),
    PhysicsHooks::new(),
    EventHandler::new(),
  )
}

///|
/// Ported from rapier/src/pipeline/physics_pipeline.rs (collider_removal_before_step) for dim3.
test "physics pipeline3 collider removal before step" {
  let pipeline = PhysicsPipeline3::new()
  let gravity = @core.Vec3::new(0.0F, -9.81F, 0.0F)
  let integration_parameters = @dynamics.IntegrationParameters::default()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let bodies = @dynamics.RigidBodySet3::new()
  let colliders = @collision.ColliderSet3::new()
  let ccd = @dynamics.CCDSolver::new()
  let impulse_joints = @dynamics.ImpulseJointSet3::new()
  let multibody_joints = @dynamics.MultibodyJointSet3::new()
  let islands = @dynamics.IslandManager3::new()
  let b_handle = bodies.insert(@dynamics.RigidBodyBuilder3::dynamic().build())
  let c_handle = colliders.insert_with_parent(
    @collision.ColliderBuilder3::ball(1.0F).build(),
    b_handle,
    bodies,
  )
  colliders.remove(c_handle, islands, bodies, true)
  bodies.remove(b_handle, islands, (), impulse_joints, multibody_joints, true)
  |> ignore
  for _ in 0..<10 {
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
      ccd,
      PhysicsHooks::new(),
      EventHandler::new(),
    )
  }
}

///|
/// Ported from rapier/src/pipeline/physics_pipeline.rs (rigid_body_type_changed_dynamic_is_in_active_set) for dim3.
test "physics pipeline3 rigid body type changed dynamic is in active set" {
  let colliders = @collision.ColliderSet3::new()
  let impulse_joints = @dynamics.ImpulseJointSet3::new()
  let multibody_joints = @dynamics.MultibodyJointSet3::new()
  let pipeline = PhysicsPipeline3::new()
  let bf = @collision.BroadPhaseBvh::new()
  let nf = @collision.NarrowPhase::new()
  let islands = @dynamics.IslandManager3::new()
  let bodies = @dynamics.RigidBodySet3::new()
  let rb = @dynamics.RigidBodyBuilder3::kinematic_position_based()
    .additional_mass(1.0F)
    .build()
  let h = bodies.insert(rb)
  let gravity = @core.Vec3::new(0.0F, -9.81F, 0.0F)
  pipeline.step(
    gravity,
    @dynamics.IntegrationParameters::default(),
    islands,
    bf,
    nf,
    bodies,
    colliders,
    impulse_joints,
    multibody_joints,
    @dynamics.CCDSolver::new(),
    PhysicsHooks::new(),
    EventHandler::new(),
  )
  if bodies.get_mut(h) is Some(body_mut) {
    body_mut.set_body_type(@dynamics.RigidBodyType::Dynamic, true) |> ignore
  }
  pipeline.step(
    gravity,
    @dynamics.IntegrationParameters::default(),
    islands,
    bf,
    nf,
    bodies,
    colliders,
    impulse_joints,
    multibody_joints,
    @dynamics.CCDSolver::new(),
    PhysicsHooks::new(),
    EventHandler::new(),
  )
  if bodies.get(h) is Some(body_after) {
    let position = body_after.translation()
    inspect(position.y < 0.0F, content="true")
    inspect(body_after.is_sleeping(), content="false")
  }
}

///|
/// Ported from rapier/src/pipeline/physics_pipeline.rs (joint_step_delta_time_0) for dim3.
test "physics pipeline3 joint step delta time 0" {
  let colliders = @collision.ColliderSet3::new()
  let impulse_joints = @dynamics.ImpulseJointSet3::new()
  let multibody_joints = @dynamics.MultibodyJointSet3::new()
  let pipeline = PhysicsPipeline3::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let islands = @dynamics.IslandManager3::new()
  let bodies = @dynamics.RigidBodySet3::new()
  let h_fixed = bodies.insert(
    @dynamics.RigidBodyBuilder3::fixed().additional_mass(1.0F).build(),
  )
  let h_dynamic = bodies.insert(
    @dynamics.RigidBodyBuilder3::dynamic().additional_mass(1.0F).build(),
  )
  let joint = @dynamics.GenericJoint::from_revolute(
    @dynamics.RevoluteJointBuilder3::new(@core.Vec3::new(0.0F, 0.0F, 1.0F))
    .local_anchor1(@core.Vec3::new(0.0F, 1.0F, 0.0F))
    .local_anchor2(@core.Vec3::new(0.0F, -3.0F, 0.0F))
    .build(),
  )
  impulse_joints.insert(h_fixed, h_dynamic, joint, true) |> ignore
  let parameters = @dynamics.IntegrationParameters::default().set_dt(0.0F)
  let gravity = @core.Vec3::new(0.0F, -9.81F, 0.0F)
  pipeline.step(
    gravity,
    parameters,
    islands,
    broad_phase,
    narrow_phase,
    bodies,
    colliders,
    impulse_joints,
    multibody_joints,
    @dynamics.CCDSolver::new(),
    PhysicsHooks::new(),
    EventHandler::new(),
  )
  let translation = bodies.translation(h_dynamic)
  let rotation = bodies.rotation(h_dynamic)
  inspect(translation.x == translation.x, content="true")
  inspect(translation.y == translation.y, content="true")
  inspect(translation.z == translation.z, content="true")
  inspect(rotation.x == rotation.x, content="true")
  inspect(rotation.y == rotation.y, content="true")
  inspect(rotation.z == rotation.z, content="true")
  inspect(rotation.w == rotation.w, content="true")
}
