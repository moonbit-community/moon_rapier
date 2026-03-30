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
const TEST_SPATIAL_DIM3_REAL : Int = 6

///|
test "multibody append (dim3 real bodies)" {
  let bodies = RigidBodySet3D::new()
  let joints = MultibodyJointSet3DReal::new()
  let a = bodies.insert(RigidBodyBuilder3D::dynamic().build())
  let b = bodies.insert(RigidBodyBuilder3D::dynamic().build())
  let c = bodies.insert(RigidBodyBuilder3D::dynamic().build())
  let d = bodies.insert(RigidBodyBuilder3D::dynamic().build())
  let joint = RevoluteJoint3DRealBuilder::new(@core.Vec3::new(1.0F, 0.0F, 0.0F)).build()
  let mb_handle = joints.insert(a, b, joint, true)
  joints.insert(c, d, joint, true) |> ignore
  joints.insert(b, c, joint, true) |> ignore
  if mb_handle is Some(handle) {
    if joints.get(handle) is Some((multibody, _)) {
      inspect(multibody.ndofs() == TEST_SPATIAL_DIM3_REAL + 3, content="true")
    } else {
      inspect(false, content="true")
    }
  } else {
    inspect(false, content="true")
  }
}

///|
test "multibody jacobian cols match ndofs for dynamic root (dim3 real bodies)" {
  let bodies = RigidBodySet3D::new()
  let joints = MultibodyJointSet3DReal::new()
  let root = bodies.insert(
    RigidBodyBuilder3D::dynamic()
    .translation(@core.Vec3::new(0.0F, 0.5F, 1.0F))
    .rotation_scaled_axis(@core.Vec3::new(0.2F, -0.1F, 0.3F))
    .build(),
  )
  let child = bodies.insert(RigidBodyBuilder3D::dynamic().build())
  let joint = RevoluteJoint3DRealBuilder::new(@core.Vec3::new(0.0F, 1.0F, 0.0F))
    .local_anchor2(@core.Vec3::new(0.0F, 0.0F, -0.5F))
    .build()
  let handle = joints.insert(root, child, joint, true)
  if handle is Some(link_handle) {
    if joints.get_mut(link_handle) is Some((multibody, link_id)) {
      multibody.forward_kinematics(bodies, true)
      let jacobian = multibody.body_jacobian(link_id)
      inspect(jacobian.cols() == multibody.ndofs(), content="true")
    } else {
      inspect(false, content="true")
    }
  } else {
    inspect(false, content="true")
  }
}

///|
test "multibody apply_displacements preserves dynamic root 3d dofs (real bodies)" {
  let bodies = RigidBodySet3D::new()
  let joints = MultibodyJointSet3DReal::new()
  let root = bodies.insert(
    RigidBodyBuilder3D::dynamic()
    .translation(@core.Vec3::new(1.0F, 2.0F, 3.0F))
    .rotation_scaled_axis(@core.Vec3::new(0.25F, -0.1F, 0.2F))
    .build(),
  )
  let child = bodies.insert(RigidBodyBuilder3D::dynamic().build())
  let joint = RevoluteJoint3DRealBuilder::new(@core.Vec3::new(0.0F, 0.0F, 1.0F)).build()
  let handle = joints.insert(root, child, joint, true)
  if handle is Some(link_handle) {
    if joints.get_mut(link_handle) is Some((multibody, link_id)) {
      multibody.forward_kinematics(bodies, true)
      multibody.apply_displacements([
        0.0F,
        0.0F,
        2.0F,
        0.25F,
        -0.5F,
        0.15F,
        0.0F,
      ])
      multibody.forward_kinematics(bodies, false)
      if multibody.link_pose(link_id) is Some(pose) {
        inspect(@core.abs(pose.translation.z - 5.0F) < 1.0e-4F, content="true")
        let axis = pose.rotation.to_scaled_axis()
        inspect(@core.abs(axis.x) > 1.0e-3F, content="true")
        inspect(@core.abs(axis.y) > 1.0e-3F, content="true")
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

///|
test "multibody sync_rigid_bodies writes back full 3d poses" {
  let bodies = RigidBodySet3D::new()
  let joints = MultibodyJointSet3DReal::new()
  let root = bodies.insert(
    RigidBodyBuilder3D::dynamic()
    .translation(@core.Vec3::new(0.0F, 0.0F, 0.5F))
    .rotation_scaled_axis(@core.Vec3::new(0.2F, 0.0F, 0.0F))
    .build(),
  )
  let child = bodies.insert(RigidBodyBuilder3D::dynamic().build())
  let joint = RevoluteJoint3DRealBuilder::new(@core.Vec3::new(0.0F, 1.0F, 0.0F))
    .local_anchor2(@core.Vec3::new(0.0F, 0.0F, -1.0F))
    .build()
  let handle = joints.insert(root, child, joint, true)
  if handle is Some(link_handle) {
    if joints.get_mut(link_handle) is Some((multibody, _)) {
      multibody.apply_displacements([0.0F, 0.0F, 1.0F, 0.1F, 0.2F, 0.0F, 0.75F])
    } else {
      inspect(false, content="true")
    }
    joints.sync_rigid_bodies_from_multibodies(bodies)
    if bodies.get(child) is Some(body) {
      inspect(body.translation().z > 0.5F, content="true")
      let axis = body.rotation().to_scaled_axis()
      inspect(@core.abs(axis.x) > 1.0e-3F, content="true")
    } else {
      inspect(false, content="true")
    }
  } else {
    inspect(false, content="true")
  }
}
