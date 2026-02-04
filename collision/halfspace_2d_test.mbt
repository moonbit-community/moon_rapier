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
/// Regression test for Issue #14: half-space must not rely on any fixed extent.
test "halfspace broad-phase pairs at large tangential offsets" {
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = ColliderSet::new()
  let broad_phase = BroadPhaseBvh::new()
  let narrow_phase = NarrowPhase::new()
  let impulse_joints = @dynamics.ImpulseJointSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()
  let ground_body = bodies.insert(@dynamics.RigidBodyBuilder::fixed().build())
  let ground = colliders.insert_with_parent(
    ColliderBuilder::halfspace(@core.Vec2::new(0.0F, 1.0F)).build(),
    ground_body,
    bodies,
  )

  // Tangential translation far beyond the old HALFSPACE_EXTENT approximation.
  let ball_body = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(1.0e6F, 0.5F))
    .build(),
  )
  let ball = colliders.insert_with_parent(
    ColliderBuilder::ball(1.0F).build(),
    ball_body,
    bodies,
  )
  broad_phase.update(0.0F, bodies, colliders)
  let pairs = broad_phase.pairs()
  let mut found = false
  for i in 0..<pairs.length() {
    let (a, b) = pairs[i]
    if (ColliderHandle::equals(a, ground) && ColliderHandle::equals(b, ball)) ||
      (ColliderHandle::equals(a, ball) && ColliderHandle::equals(b, ground)) {
      found = true
      break
    }
  }
  inspect(found, content="true")
  narrow_phase.update_with_pairs(
    bodies, colliders, 0.0F, pairs, impulse_joints, multibody_joints,
  )
  if narrow_phase.contact_pair(ground, ball) is Some(pair) {
    inspect(pair.manifold_count() > 0, content="true")
  } else {
    inspect(false, content="true")
  }
}
