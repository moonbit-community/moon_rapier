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
test "rapier3d reexports use real 3d physics surface" {
  let pipeline = PhysicsPipeline::new()
  let broad_phase = BroadPhase::new()
  let narrow_phase = NarrowPhase::new()
  let bodies = RigidBodySet::new()
  let colliders = ColliderSet::new()
  let islands = IslandManager::new()
  let gravity = Vec3::new(0.0F, -9.81F, 0.0F)
  let params = IntegrationParameters::default()

  let ground = bodies.insert(
    RigidBodyBuilder::fixed().translation(Vec3::new(0.0F, -0.5F, 0.0F)).build(),
  )
  colliders.insert_with_parent(
    ColliderBuilder::cuboid(5.0F, 0.5F, 5.0F).build(),
    ground,
    bodies,
  )
  |> ignore

  let body = bodies.insert(
    RigidBodyBuilder::dynamic().translation(Vec3::new(0.0F, 3.0F, 1.0F)).build(),
  )
  let collider = colliders.insert_with_parent(
    ColliderBuilder::ball(0.5F).build(),
    body,
    bodies,
  )

  for _ in 0..<30 {
    pipeline.step(
      gravity, params, islands, broad_phase, narrow_phase, bodies, colliders,
    )
  }

  if bodies.get(body) is Some(rb) {
    inspect(rb.translation().y < 3.0F, content="true")
    inspect(rb.translation().z == 1.0F, content="true")
  } else {
    inspect(false, content="true")
  }

  if colliders.get(collider) is Some(co) {
    inspect(co.compute_aabb().maxs.z > 0.0F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "rapier3d multibody reexports expose real 3d API" {
  let bodies = RigidBodySet::new()
  let joints = MultibodyJointSet::new()
  let root = bodies.insert(
    RigidBodyBuilder::dynamic()
    .translation(Vec3::new(0.0F, 0.0F, 0.5F))
    .rotation_scaled_axis(Vec3::new(0.1F, 0.2F, 0.3F))
    .build(),
  )
  let child = bodies.insert(RigidBodyBuilder::dynamic().build())
  let joint = GenericJoint::from_revolute(
    RevoluteJointBuilder::new(Vec3::new(0.0F, 1.0F, 0.0F))
    .local_anchor2(Vec3::new(0.0F, 0.0F, -1.0F))
    .build(),
  )
  let inserted = joints.insert(root, child, joint, true)
  if inserted is Some(handle) {
    if joints.get_mut(handle) is Some((multibody, link)) {
      multibody.apply_displacements([0.0F, 0.0F, 1.0F, 0.2F, 0.0F, 0.0F, 0.5F])
      multibody.forward_kinematics(bodies, false)
      if multibody.link_pose(link) is Some(pose) {
        inspect(pose.translation.z > 1.0F, content="true")
      } else {
        inspect(false, content="true")
      }
    } else {
      inspect(false, content="true")
    }
  } else {
    inspect(false, content="true")
  }
}
