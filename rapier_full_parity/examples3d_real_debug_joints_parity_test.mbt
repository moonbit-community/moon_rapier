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
/// Headless parity tests for joint-heavy `rapier-reference/examples3d/debug_*.rs` scenes against the
/// real dim3 backend.

///|
test "examples3d/debug_long_chain3.rs parity (3DReal headless)" {
  // Ported from `rapier-reference/examples3d/debug_long_chain3.rs` (headless).
  let pipeline = @pipeline.PhysicsPipeline3DReal::new()
  let broad_phase = @collision.BroadPhase3D::new()
  let narrow_phase = @collision.NarrowPhase3D::new()
  let bodies = @dynamics.RigidBodySet3D::new()
  let colliders = @collision.ColliderSet3D::new()
  let islands = @dynamics.IslandManager3D::new()
  let joints = @dynamics.JointSet3DReal::new()
  let gravity = @core.Vec3::new(0.0F, -9.81F, 0.0F)
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )
  let num = 20
  let rad = 0.2F
  let shift = rad * 2.2F
  let handles : Array[@dynamics.RigidBodyHandle] = []
  for i in 0..<num {
    let fi = Float::from_int(i)
    let rb = bodies.insert(
      (if i == 0 {
        @dynamics.RigidBodyBuilder3D::fixed()
      } else {
        @dynamics.RigidBodyBuilder3D::dynamic()
      })
      .translation(@core.Vec3::new(0.0F, 0.0F, fi * shift))
      .can_sleep(false)
      .build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder3D::ball(rad).build(),
      rb,
      bodies,
    )
    |> ignore
    if i > 0 {
      let parent = handles[handles.length() - 1]
      if i == 1 {
        joints.insert_spherical(
          parent,
          rb,
          @core.Vec3::zero(),
          @core.Vec3::new(0.0F, 0.0F, -shift),
        )
      } else {
        joints.insert_spherical(
          parent,
          rb,
          @core.Vec3::new(0.0F, 0.0F, shift / 2.0F),
          @core.Vec3::new(0.0F, 0.0F, -shift / 2.0F),
        )
      }
    }
    handles.push(rb)
  }
  for _ in 0..<240 {
    pipeline.step_with_joints(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
      joints,
    )
  }

  // Sanity: the chain remains finite and doesn't explode.
  for i in 0..<handles.length() {
    let h = handles[i]
    if bodies.get(h) is Some(rb) {
      let p = rb.translation()
      inspect(p.x == p.x && p.y == p.y && p.z == p.z, content="true")
      inspect(p.y > -200.0F, content="true")
    } else {
      inspect(false, content="true")
    }
  }
}

///|
test "examples3d/debug_chain_high_mass_ratio3.rs parity (3DReal headless)" {
  // Ported from `rapier-reference/examples3d/debug_chain_high_mass_ratio3.rs` (headless).
  let pipeline = @pipeline.PhysicsPipeline3DReal::new()
  let broad_phase = @collision.BroadPhase3D::new()
  let narrow_phase = @collision.NarrowPhase3D::new()
  let bodies = @dynamics.RigidBodySet3D::new()
  let colliders = @collision.ColliderSet3D::new()
  let islands = @dynamics.IslandManager3D::new()
  let joints = @dynamics.JointSet3DReal::new()
  let gravity = @core.Vec3::new(0.0F, -9.81F, 0.0F)
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )
  let num = 10
  let rad = 0.2F
  let handles : Array[@dynamics.RigidBodyHandle] = []
  for i in 0..<num {
    let fi = Float::from_int(i)
    let ball_rad = if i == num - 1 { rad * 10.0F } else { rad }
    let shift1 = rad * 1.1F
    let shift2 = ball_rad + rad * 0.1F
    let z = if i == 0 {
      0.0F
    } else {
      (fi - 1.0F) * 2.0F * shift1 + shift1 + shift2
    }
    let rb = bodies.insert(
      (if i == 0 {
        @dynamics.RigidBodyBuilder3D::fixed()
      } else {
        @dynamics.RigidBodyBuilder3D::dynamic()
      })
      .translation(@core.Vec3::new(0.0F, 5.0F, z))
      .can_sleep(false)
      .build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder3D::ball(ball_rad).build(),
      rb,
      bodies,
    )
    |> ignore
    if i > 0 {
      let parent = handles[handles.length() - 1]
      if i == 1 {
        joints.insert_spherical(
          parent,
          rb,
          @core.Vec3::zero(),
          @core.Vec3::new(0.0F, 0.0F, -shift1 * 2.0F),
        )
      } else {
        joints.insert_spherical(
          parent,
          rb,
          @core.Vec3::new(0.0F, 0.0F, shift1),
          @core.Vec3::new(0.0F, 0.0F, -shift2),
        )
      }
    }
    handles.push(rb)
  }
  for _ in 0..<240 {
    pipeline.step_with_joints(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
      joints,
    )
  }

  // Sanity: the heavy end doesn't go NaN and stays within a reasonable range.
  let last = handles[handles.length() - 1]
  if bodies.get(last) is Some(rb) {
    let p = rb.translation()
    inspect(p.x == p.x && p.y == p.y && p.z == p.z, content="true")
    inspect(p.y > -200.0F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "examples3d/debug_prismatic3.rs parity (3DReal headless)" {
  // Ported from `rapier-reference/examples3d/debug_prismatic3.rs` (headless).
  let pipeline = @pipeline.PhysicsPipeline3DReal::new()
  let broad_phase = @collision.BroadPhase3D::new()
  let narrow_phase = @collision.NarrowPhase3D::new()
  let bodies = @dynamics.RigidBodySet3D::new()
  let colliders = @collision.ColliderSet3D::new()
  let islands = @dynamics.IslandManager3D::new()
  let joints = @dynamics.JointSet3DReal::new()
  let gravity = @core.Vec3::new(0.0F, -9.81F, 0.0F)
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )

  // Ground.
  let ground_size = 50.0F
  let ground_height = 0.1F
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder3D::fixed()
    .translation(@core.Vec3::new(0.0F, -ground_height, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(
      ground_size, ground_height, ground_size,
    ).build(),
    ground,
    bodies,
  )
  |> ignore

  // Chassis.
  let box_center = @core.Vec3::new(0.0F, 5.0F, 0.0F)
  let box_rb = bodies.insert(
    @dynamics.RigidBodyBuilder3D::dynamic()
    .translation(box_center)
    .can_sleep(false)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(1.0F, 0.25F, 1.0F).build(),
    box_rb,
    bodies,
  )
  |> ignore
  let wheel_y = -1.0F
  let wheel_positions = [
    @core.Vec3::new(1.0F, wheel_y, -1.0F),
    @core.Vec3::new(-1.0F, wheel_y, -1.0F),
    @core.Vec3::new(1.0F, wheel_y, 1.0F),
    @core.Vec3::new(-1.0F, wheel_y, 1.0F),
  ]
  for pos in wheel_positions {
    let wheel_world = box_center.add(pos)
    let wheel_rb = bodies.insert(
      @dynamics.RigidBodyBuilder3D::dynamic()
      .translation(wheel_world)
      .can_sleep(false)
      .build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder3D::ball(0.5F).build(),
      wheel_rb,
      bodies,
    )
    |> ignore
    joints.insert_prismatic_motor_position(
      box_rb,
      wheel_rb,
      @core.Vec3::new(0.0F, 1.0F, 0.0F),
      pos,
      @core.Vec3::zero(),
      0.0F,
      0.05F,
      0.2F,
    )
  }

  // Small "gravel" under one wheel.
  let gravel = bodies.insert(
    @dynamics.RigidBodyBuilder3D::dynamic()
    .translation(
      @core.Vec3::new(box_center.x + 1.0F, box_center.y - 2.4F, -1.0F),
    )
    .can_sleep(false)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(0.5F, 0.1F, 0.5F).build(),
    gravel,
    bodies,
  )
  |> ignore
  for _ in 0..<240 {
    pipeline.step_with_joints(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
      joints,
    )
  }

  // Sanity: chassis stays finite and above the ground.
  if bodies.get(box_rb) is Some(rb) {
    let p = rb.translation()
    inspect(p.x == p.x && p.y == p.y && p.z == p.z, content="true")
    inspect(p.y > -10.0F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "examples3d/debug_articulations3.rs parity (3DReal headless)" {
  // Ported from `rapier-reference/examples3d/debug_articulations3.rs` (headless; uses spherical
  // constraints for both multibody and impulse joints).
  let pipeline = @pipeline.PhysicsPipeline3DReal::new()
  let broad_phase = @collision.BroadPhase3D::new()
  let narrow_phase = @collision.NarrowPhase3D::new()
  let bodies = @dynamics.RigidBodySet3D::new()
  let colliders = @collision.ColliderSet3D::new()
  let islands = @dynamics.IslandManager3D::new()
  let joints = @dynamics.JointSet3DReal::new()
  let gravity = @core.Vec3::new(0.0F, -9.81F, 0.0F)
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )

  // A large tilted ground.
  let ground = bodies.insert(@dynamics.RigidBodyBuilder3D::fixed().build())
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(30.0F, 0.01F, 30.0F)
    .translation(@core.Vec3::new(0.0F, -3.02F, 0.0F))
    .rotation(
      @core.rotation_from_scaled_axis(@core.Vec3::new(0.1F, 0.0F, 0.1F)),
    )
    .build(),
    ground,
    bodies,
  )
  |> ignore

  // A dynamic platform.
  let platform = bodies.insert(
    @dynamics.RigidBodyBuilder3D::dynamic().can_sleep(false).build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(30.0F, 0.01F, 30.0F)
    .translation(@core.Vec3::new(0.0F, -3.0F, 0.0F))
    .rotation(
      @core.rotation_from_scaled_axis(@core.Vec3::new(0.1F, 0.0F, 0.1F)),
    )
    .build(),
    platform,
    bodies,
  )
  |> ignore
  let num = 6
  let rad = 0.4F
  let shift = 1.0F
  let body_handles : Array[@dynamics.RigidBodyHandle] = []
  let cap_rot = @core.rotation_from_scaled_axis(
    @core.Vec3::new(1.5707964F, 0.0F, 0.0F),
  )
  for k in 0..<num {
    for i in 0..<num {
      let fk = Float::from_int(k)
      let fi = Float::from_int(i)
      let fixed = i == 0
      let rb = bodies.insert(
        (if fixed {
          @dynamics.RigidBodyBuilder3D::fixed()
        } else {
          @dynamics.RigidBodyBuilder3D::dynamic().can_sleep(false)
        })
        .translation(@core.Vec3::new(fk * shift, 0.0F, fi * shift * 2.0F))
        .build(),
      )
      colliders.insert_with_parent(
        @collision.ColliderBuilder3D::capsule_y(rad, rad * 1.25F)
        .rotation(cap_rot)
        .build(),
        rb,
        bodies,
      )
      |> ignore
      if i > 0 {
        // Vertical link (along Z).
        let parent = body_handles[body_handles.length() - 1]
        joints.insert_spherical(
          parent,
          rb,
          @core.Vec3::zero(),
          @core.Vec3::new(0.0F, 0.0F, -shift * 2.0F),
        )
      }
      if k > 0 && i > 0 {
        // Horizontal link (along X).
        let parent_index = body_handles.length() - num
        let parent = body_handles[parent_index]
        joints.insert_spherical(
          parent,
          rb,
          @core.Vec3::zero(),
          @core.Vec3::new(-shift, 0.0F, 0.0F),
        )
      }
      body_handles.push(rb)
    }
  }
  for _ in 0..<240 {
    pipeline.step_with_joints(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
      joints,
    )
  }

  // Sanity: no NaNs on a few representative bodies.
  for idx in [0, body_handles.length() / 2, body_handles.length() - 1] {
    let h = body_handles[idx]
    if bodies.get(h) is Some(rb) {
      let p = rb.translation()
      inspect(p.x == p.x && p.y == p.y && p.z == p.z, content="true")
      inspect(p.y > -200.0F, content="true")
    } else {
      inspect(false, content="true")
    }
  }
}

///|
test "examples3d/debug_multibody_ang_motor_pos3.rs parity (3DReal headless)" {
  // Ported from `rapier-reference/examples3d/debug_multibody_ang_motor_pos3.rs` (headless).
  let pipeline = @pipeline.PhysicsPipeline3DReal::new()
  let broad_phase = @collision.BroadPhase3D::new()
  let narrow_phase = @collision.NarrowPhase3D::new()
  let bodies = @dynamics.RigidBodySet3D::new()
  let colliders = @collision.ColliderSet3D::new()
  let islands = @dynamics.IslandManager3D::new()
  let joints = @dynamics.JointSet3DReal::new()
  let gravity = @core.Vec3::new(0.0F, -9.81F, 0.0F)
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )
  let ground = bodies.insert(@dynamics.RigidBodyBuilder3D::fixed().build())
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(1.0F, 1.0F, 1.0F).build(),
    ground,
    bodies,
  )
  |> ignore
  let body_part = bodies.insert(
    @dynamics.RigidBodyBuilder3D::dynamic()
    .translation(@core.Vec3::new(0.0F, 1.0F, 0.0F))
    .can_sleep(false)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(1.0F, 1.0F, 1.0F).density(1.0F).build(),
    body_part,
    bodies,
  )
  |> ignore
  joints.insert_spherical(
    ground,
    body_part,
    @core.Vec3::new(0.0F, 4.0F, 0.0F),
    @core.Vec3::zero(),
  )
  joints.insert_spherical_motor_rotation(
    ground,
    body_part,
    @core.rotation_from_scaled_axis(@core.Vec3::new(1.0F, 0.0F, 0.0F)),
    1000.0F,
    1.0F,
  )
  for _ in 0..<240 {
    pipeline.step_with_joints(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
      joints,
    )
  }
  if bodies.get(body_part) is Some(rb) {
    let axis = rb.rotation().to_scaled_axis()
    inspect(
      axis.x == axis.x && axis.y == axis.y && axis.z == axis.z,
      content="true",
    )
    inspect(@core.abs(axis.x) > 0.1F, content="true")
  } else {
    inspect(false, content="true")
  }
}
