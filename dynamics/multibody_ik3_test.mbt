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
/// Ported from rapier/src/dynamics/joint/multibody_joint/multibody_ik.rs (one_link_fwd_kinematics) for dim3.
fn isometry3_max_abs_diff(
  left : @core.Isometry3,
  right : @core.Isometry3,
) -> @core.Real {
  let dx = @core.abs(left.translation.x - right.translation.x)
  let dy = @core.abs(left.translation.y - right.translation.y)
  let dz = @core.abs(left.translation.z - right.translation.z)
  let dq = quat_max_abs_diff_sign_invariant3(left.rotation, right.rotation)
  let mut max_diff = dx
  if dy > max_diff {
    max_diff = dy
  }
  if dz > max_diff {
    max_diff = dz
  }
  if dq > max_diff {
    max_diff = dq
  }
  max_diff
}

///|
fn quat_max_abs_diff_sign_invariant3(
  left : @core.Quat,
  right : @core.Quat,
) -> @core.Real {
  let mut r = right
  if left.dot(right) < 0.0F {
    r = @core.Quat::new(-right.x, -right.y, -right.z, -right.w)
  }
  let dx = @core.abs(left.x - r.x)
  let dy = @core.abs(left.y - r.y)
  let dz = @core.abs(left.z - r.z)
  let dw = @core.abs(left.w - r.w)
  let mut max_diff = dx
  if dy > max_diff {
    max_diff = dy
  }
  if dz > max_diff {
    max_diff = dz
  }
  if dw > max_diff {
    max_diff = dw
  }
  max_diff
}

///|
fn jacobian3_max_abs_diff(left : Jacobian3, right : Jacobian3) -> @core.Real {
  if left.cols() != right.cols() {
    return 1.0e9F
  }
  let rows = 6
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
test "one link fwd kinematics (dim3)" {
  let bodies = RigidBodySet3::new()
  let multibodies = MultibodyJointSet3::new()
  let num_segments = 10
  let num_segments_real = Float::from_double(num_segments.to_double())
  let mut last_body = bodies.insert(RigidBodyBuilder3::fixed().build())
  let mut last_link = MultibodyJointHandle::invalid()
  for _ in 0..<num_segments {
    let new_body = bodies.insert(
      RigidBodyBuilder3::dynamic().can_sleep(false).build(),
    )
    let anchor_delta = 0.5F / num_segments_real
    let builder = RevoluteJointBuilder3::new(@core.Vec3::new(0.0F, 0.0F, 1.0F))
      .local_anchor1(@core.Vec3::new(0.0F, anchor_delta, 0.0F))
      .local_anchor2(@core.Vec3::new(0.0F, -anchor_delta, 0.0F))
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

    // No displacement.
    let jacobian2 = Jacobian3::zeros(0)
    let link_pose1 = if multibody.link_pose(last_id) is Some(p) {
      p
    } else {
      @core.Isometry3::identity()
    }
    let jacobian1 = multibody.body_jacobian(last_id)
    let link_pose2 = multibody.forward_kinematics_single_link(
      bodies,
      last_id,
      None,
      Some(jacobian2),
    )
    let pose_diff = isometry3_max_abs_diff(link_pose1, link_pose2)
    let jacobian_diff = jacobian3_max_abs_diff(jacobian1, jacobian2)
    inspect(pose_diff < 1.0e-5F, content="true")
    inspect(jacobian_diff < 1.0e-5F, content="true")

    // Arbitrary displacement.
    let niter = 100
    let niter_real = Float::from_double(niter.to_double())
    let displacement_part : Array[@core.Real] = []
    for index in 0..<multibody.ndofs() {
      displacement_part.push(
        Float::from_double(index.to_double()) * -0.1F / niter_real,
      )
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
    let link_pose1_after = if multibody.link_pose(last_id) is Some(p) {
      p
    } else {
      @core.Isometry3::identity()
    }
    let jacobian1_after = multibody.body_jacobian(last_id)
    let pose_diff_after = isometry3_max_abs_diff(
      link_pose1_after, link_pose2_after,
    )
    let jacobian_diff_after = jacobian3_max_abs_diff(jacobian1_after, jacobian2)
    inspect(pose_diff_after < 1.0e-5F, content="true")
    inspect(jacobian_diff_after < 1.0e-5F, content="true")
  }
}
