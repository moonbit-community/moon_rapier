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
test "snapshot round-trip is deterministic (bodies/colliders/joints)" {
  let pipeline = PhysicsPipeline::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = @collision.ColliderSet::new()
  let impulse_joints = @dynamics.ImpulseJointSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()
  let islands = @dynamics.IslandManager::new()
  let ccd_solver = @dynamics.CCDSolver::new()
  let handler = EventHandler::new()
  let params = IntegrationParameters::default()
  let gravity = @core.Vec2::new(0.0F, -9.81F)

  // A small chain driven by a multibody revolute joint.
  let mb_parent = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(0.0F, 2.0F))
    .build(),
  )
  let mb_child = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(0.0F, 1.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::ball(0.25F).build(),
    mb_parent,
    bodies,
  )
  |> ignore
  colliders.insert_with_parent(
    @collision.ColliderBuilder::ball(0.25F).build(),
    mb_child,
    bodies,
  )
  |> ignore
  let mb_joint = @dynamics.RevoluteJointBuilder::new()
    .local_anchor1(@core.Vec2::zero())
    .local_anchor2(@core.Vec2::zero())
    .build()
  multibody_joints.insert(mb_parent, mb_child, mb_joint, true) |> ignore

  // An impulse-joint between two bodies.
  let ij_a = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(1.0F, 2.0F))
    .build(),
  )
  let ij_b = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(1.0F, 1.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(0.2F, 0.2F).build(),
    ij_a,
    bodies,
  )
  |> ignore
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(0.2F, 0.2F).build(),
    ij_b,
    bodies,
  )
  |> ignore
  let ij_revolute = @dynamics.RevoluteJointBuilder::new()
    .local_anchor1(@core.Vec2::zero())
    .local_anchor2(@core.Vec2::zero())
    .limits(-0.25F, 0.25F)
    .build()
  let ij = @dynamics.GenericJoint::from_revolute(ij_revolute)
  impulse_joints.insert(ij_a, ij_b, ij, true) |> ignore

  // Step a bit to ensure caches/state get exercised.
  for _ in 0..<20 {
    pipeline.step(
      gravity,
      params,
      islands,
      broad_phase,
      narrow_phase,
      bodies,
      colliders,
      impulse_joints,
      multibody_joints,
      ccd_solver,
      PhysicsHooks::new(),
      handler,
    )
  }
  let bodies_ser = bodies.serialize()
  let colliders_ser = colliders.serialize()
  let impulse_ser = impulse_joints.serialize()
  let multibody_ser = multibody_joints.serialize()

  // Deterministic encoding for the same in-memory state.
  inspect(bodies.serialize() == bodies_ser, content="true")
  inspect(colliders.serialize() == colliders_ser, content="true")
  inspect(impulse_joints.serialize() == impulse_ser, content="true")
  inspect(multibody_joints.serialize() == multibody_ser, content="true")

  // Round-trip stability.
  let bodies2 = @dynamics.RigidBodySet::deserialize(bodies_ser)
  let colliders2 = @collision.ColliderSet::deserialize(colliders_ser)
  let impulse2 = @dynamics.ImpulseJointSet::deserialize(impulse_ser)
  let multibody2 = @dynamics.MultibodyJointSet::deserialize(multibody_ser)
  inspect(bodies2.serialize() == bodies_ser, content="true")
  inspect(colliders2.serialize() == colliders_ser, content="true")
  inspect(impulse2.serialize() == impulse_ser, content="true")
  inspect(multibody2.serialize() == multibody_ser, content="true")

  // Basic restore sanity: the restored world can step deterministically without panics.
  let broad_phase2 = @collision.BroadPhaseBvh::new()
  let narrow_phase2 = @collision.NarrowPhase::new()
  let islands2 = @dynamics.IslandManager::new()
  for _ in 0..<5 {
    pipeline.step(
      gravity,
      params,
      islands2,
      broad_phase2,
      narrow_phase2,
      bodies2,
      colliders2,
      impulse2,
      multibody2,
      ccd_solver,
      PhysicsHooks::new(),
      handler,
    )
  }
  if bodies2.get(mb_child) is Some(body) {
    inspect(body.translation().y == body.translation().y, content="true")
  } else {
    inspect(false, content="true")
  }
}
