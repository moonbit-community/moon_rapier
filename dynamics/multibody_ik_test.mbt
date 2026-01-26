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
/// Ported from rapier/src/dynamics/joint/multibody_joint/multibody_ik.rs (one_link_fwd_kinematics).
fn isometry2_max_abs_diff(
  left : @core.Isometry2,
  right : @core.Isometry2,
) -> @core.Real {
  let dx = @core.abs(left.translation.x - right.translation.x)
  let dy = @core.abs(left.translation.y - right.translation.y)
  let dsin = @core.abs(left.rotation.sin - right.rotation.sin)
  let dcos = @core.abs(left.rotation.cos - right.rotation.cos)
  let mut max_diff = dx
  if dy > max_diff {
    max_diff = dy
  }
  if dsin > max_diff {
    max_diff = dsin
  }
  if dcos > max_diff {
    max_diff = dcos
  }
  max_diff
}

///|
fn jacobian_max_abs_diff(left : Jacobian, right : Jacobian) -> @core.Real {
  if left.cols() != right.cols() {
    return 1.0e9F
  }
  let rows = Jacobian::spatial_dim()
  let cols = left.cols()
  let mut max_diff = 0.0F
  for row_index in 0..<rows {
    for col_index in 0..<cols {
      let diff = @core.abs(
        left.value_at(row_index, col_index) -
        right.value_at(row_index, col_index),
      )
      if diff > max_diff {
        max_diff = diff
      }
    }
  }
  max_diff
}

///|
test "one link fwd kinematics" {
  let bodies = RigidBodySet::new()
  let multibodies = MultibodyJointSet::new()
  let num_segments = 10
  let num_segments_real = Float::from_double(num_segments.to_double())
  let body = RigidBodyBuilder::fixed().build()
  let mut last_body = bodies.insert(body)
  let mut last_link = MultibodyJointHandle::invalid()
  for _ in 0..<num_segments {
    let body_dyn = RigidBodyBuilder::dynamic().can_sleep(false).build()
    let new_body = bodies.insert(body_dyn)
    let anchor_delta = 0.5F / num_segments_real
    let builder = RevoluteJointBuilder::new()
      .local_anchor1(@core.Vec2::new(0.0F, anchor_delta))
      .local_anchor2(@core.Vec2::new(0.0F, -anchor_delta))
    let link = multibodies.insert(last_body, new_body, builder.build(), true)
    if link is Some(handle) {
      last_link = handle
    }
    last_body = new_body
  }
  if multibodies.get_mut(last_link) is Some(result) {
    let (multibody, last_id) = result
    multibody.forward_kinematics(bodies, true)
    inspect(multibody.ndofs() == num_segments, content="true")
    let jacobian2 = Jacobian::zeros(0)
    let mut link_pose1 = @core.Isometry2::identity()
    let mut link_pose1_valid = false
    if multibody.link(last_id) is Some(link_value) {
      link_pose1 = link_value.local_to_world()
      link_pose1_valid = true
    }
    let jacobian1 = multibody.body_jacobian(last_id)
    let link_pose2 = multibody.forward_kinematics_single_link(
      bodies,
      last_id,
      None,
      Some(jacobian2),
    )
    inspect(link_pose1_valid, content="true")
    let pose_diff = isometry2_max_abs_diff(link_pose1, link_pose2)
    let jacobian_diff = jacobian_max_abs_diff(jacobian1, jacobian2)
    inspect(pose_diff < 1.0e-5F, content="true")
    inspect(jacobian_diff < 1.0e-5F, content="true")
    let niter = 100
    let niter_real = Float::from_double(niter.to_double())
    let displacement_part : Array[@core.Real] = []
    for index in 0..<multibody.ndofs() {
      let value = Float::from_double(index.to_double()) * -0.1F / niter_real
      displacement_part.push(value)
    }
    let displacement_total : Array[@core.Real] = []
    for value in displacement_part {
      displacement_total.push(value * niter_real)
    }
    let link_pose2_after = multibody.forward_kinematics_single_link(
      bodies,
      last_id,
      Some(displacement_total),
      Some(jacobian2),
    )
    for _ in 0..<niter {
      multibody.apply_displacements(displacement_part)
      multibody.forward_kinematics(bodies, false)
    }
    let mut link_pose1_after = @core.Isometry2::identity()
    let mut link_pose1_after_valid = false
    if multibody.link(last_id) is Some(link_value) {
      link_pose1_after = link_value.local_to_world()
      link_pose1_after_valid = true
    }
    let jacobian1_after = multibody.body_jacobian(last_id)
    inspect(link_pose1_after_valid, content="true")
    let pose_diff_after = isometry2_max_abs_diff(
      link_pose1_after, link_pose2_after,
    )
    let jacobian_diff_after = jacobian_max_abs_diff(jacobian1_after, jacobian2)
    inspect(pose_diff_after < 1.0e-5F, content="true")
    inspect(jacobian_diff_after < 1.0e-5F, content="true")
  }
}
