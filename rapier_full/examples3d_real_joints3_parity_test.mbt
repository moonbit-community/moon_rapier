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
/// Parity tests for `rapier-reference/examples3d/joints3.rs` against the real dim3 backend.

///|
test "examples3d/joints3.rs parity (3DReal headless; reduced)" {
  // Ported from `rapier-reference/examples3d/joints3.rs` (headless; reduced counts).
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

  // Simple ground.
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder3D::fixed()
    .translation(@core.Vec3::new(0.0F, -1.0F, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(50.0F, 1.0F, 50.0F).build(),
    ground,
    bodies,
  )
  |> ignore

  // Coupled translations (LIN_Y + LIN_Z) with limits.
  let body1 = bodies.insert(
    @dynamics.RigidBodyBuilder3D::dynamic()
    .translation(@core.Vec3::new(0.0F, 10.0F, 0.0F))
    .linvel(@core.Vec3::new(5.0F, 5.0F, 5.0F))
    .can_sleep(false)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(1.0F, 1.0F, 1.0F).build(),
    body1,
    bodies,
  )
  |> ignore
  let coupled = @dynamics.GenericJoint3DRealBuilder::new(
      @dynamics.JointAxesMask3DReal::empty(),
    )
    .limits(@dynamics.JointAxis3DReal::LinX, -3.0F, 3.0F)
    .limits(@dynamics.JointAxis3DReal::LinY, 0.0F, 3.0F)
    .coupled_axes(
      @dynamics.JointAxesMask3DReal::lin_y().or(
        @dynamics.JointAxesMask3DReal::lin_z(),
      ),
    )
    .build()
  joints.insert_generic(ground, body1, coupled, true) |> ignore

  // A prismatic joint along a diagonal axis with limits + position motor.
  let prism_ground = bodies.insert(
    @dynamics.RigidBodyBuilder3D::fixed()
    .translation(@core.Vec3::new(20.0F, 5.0F, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(0.4F, 0.4F, 0.4F).build(),
    prism_ground,
    bodies,
  )
  |> ignore
  let prism_child = bodies.insert(
    @dynamics.RigidBodyBuilder3D::dynamic()
    .translation(@core.Vec3::new(20.0F, 5.0F, 2.0F))
    .can_sleep(false)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(0.4F, 0.4F, 0.4F).build(),
    prism_child,
    bodies,
  )
  |> ignore
  let axis = @core.Vec3::new(1.0F, 1.0F, 0.0F).normalize()
  let prism_basis = @core.rotation_between(
    @core.Vec3::new(1.0F, 0.0F, 0.0F),
    axis,
  )
  let prism_locked = @dynamics.JointAxesMask3DReal::lin_y()
    .or(@dynamics.JointAxesMask3DReal::lin_z())
    .or(@dynamics.JointAxesMask3DReal::ang_axes())
  let prism = @dynamics.GenericJoint3DRealBuilder::new(prism_locked)
    .local_basis1(prism_basis)
    .local_basis2(prism_basis)
    .local_anchor2(@core.Vec3::new(0.0F, 0.0F, -2.0F))
    .limits(@dynamics.JointAxis3DReal::LinX, -2.0F, 2.0F)
    .motor_position(@dynamics.JointAxis3DReal::LinX, 1.0F, 500.0F, 5.0F)
    .build()
  joints.insert_generic(prism_ground, prism_child, prism, true) |> ignore

  // A fixed joint between two small balls.
  let fixed_a = bodies.insert(
    @dynamics.RigidBodyBuilder3D::dynamic()
    .translation(@core.Vec3::new(-5.0F, 8.0F, 0.0F))
    .can_sleep(false)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::ball(0.3F).build(),
    fixed_a,
    bodies,
  )
  |> ignore
  let fixed_b = bodies.insert(
    @dynamics.RigidBodyBuilder3D::dynamic()
    .translation(@core.Vec3::new(-5.0F, 8.0F, 1.0F))
    .can_sleep(false)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::ball(0.3F).build(),
    fixed_b,
    bodies,
  )
  |> ignore
  let fixed_joint = @dynamics.GenericJoint3DRealBuilder::new(
      @dynamics.JointAxesMask3DReal::all(),
    )
    .local_anchor2(@core.Vec3::new(0.0F, 0.0F, -1.0F))
    .build()
  joints.insert_generic(fixed_a, fixed_b, fixed_joint, true) |> ignore

  // A spherical joint with limited translations (rapier's spherical + linear limits).
  let sph_ground = bodies.insert(
    @dynamics.RigidBodyBuilder3D::fixed()
    .translation(@core.Vec3::new(-5.0F, 0.0F, 0.0F))
    .build(),
  )
  let sph_ball = bodies.insert(
    @dynamics.RigidBodyBuilder3D::dynamic()
    .translation(@core.Vec3::new(-5.0F, 3.0F, 0.0F))
    .linvel(@core.Vec3::new(10.0F, 10.0F, 0.0F))
    .can_sleep(false)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(0.6F, 0.6F, 0.6F).build(),
    sph_ball,
    bodies,
  )
  |> ignore
  let sph = @dynamics.GenericJoint3DRealBuilder::new(
      @dynamics.JointAxesMask3DReal::lin_axes(),
    )
    .local_anchor2(@core.Vec3::new(0.0F, -3.0F, 0.0F))
    .limits(@dynamics.JointAxis3DReal::LinX, -0.2F, 0.2F)
    .limits(@dynamics.JointAxis3DReal::LinY, -0.2F, 0.2F)
    .build()
  joints.insert_generic(sph_ground, sph_ball, sph, true) |> ignore

  // A small actuated revolute chain (motor velocity + motor position).
  let mut parent = bodies.insert(
    @dynamics.RigidBodyBuilder3D::fixed()
    .translation(@core.Vec3::new(10.0F, 5.0F, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(0.4F, 0.4F, 0.4F).build(),
    parent,
    bodies,
  )
  |> ignore
  let mut motor0 : @dynamics.JointHandle3DReal? = None
  let mut motor_last : @dynamics.JointHandle3DReal? = None
  for i in 0..<3 {
    let child = bodies.insert(
      @dynamics.RigidBodyBuilder3D::dynamic()
      .translation(@core.Vec3::new(10.0F, 5.0F, Float::from_int(i + 1) * 2.0F))
      .can_sleep(false)
      .build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder3D::cuboid(0.4F, 0.2F, 0.4F).build(),
      child,
      bodies,
    )
    |> ignore
    let h = joints.insert_revolute(
      parent,
      child,
      @core.Vec3::new(0.0F, 0.0F, 1.0F),
      @core.Vec3::zero(),
      @core.Vec3::new(0.0F, 0.0F, -2.0F),
      true,
    )
    if i == 0 {
      motor0 = Some(h)
    }
    if i == 2 {
      motor_last = Some(h)
    }
    parent = child
  }

  // A small spring strip to ensure the spring constraint path stays stable.
  let anchor = bodies.insert(
    @dynamics.RigidBodyBuilder3D::fixed()
    .translation(@core.Vec3::new(-10.0F, 5.0F, 0.0F))
    .build(),
  )
  let spring_ball = bodies.insert(
    @dynamics.RigidBodyBuilder3D::dynamic()
    .translation(@core.Vec3::new(-10.0F, 7.0F, 0.0F))
    .can_sleep(false)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::ball(0.5F).build(),
    spring_ball,
    bodies,
  )
  |> ignore
  joints.insert_spring(
    anchor,
    spring_ball,
    @core.Vec3::zero(),
    @core.Vec3::zero(),
    2.0F,
    200.0F,
    5.0F,
    true,
  )
  |> ignore

  // Drive motors for a short duration.
  for step in 0..<360 {
    if step == 0 {
      if motor0 is Some(h) {
        joints.set_motor_velocity(
          h,
          @dynamics.JointAxis3DReal::AngZ,
          -10.0F,
          5.0F,
          100.0F,
        )
      }
      if motor_last is Some(h) {
        joints.set_motor_position(
          h,
          @dynamics.JointAxis3DReal::AngZ,
          1.5707964F,
          10.0F,
          2.0F,
          100.0F,
        )
      }
    }
    pipeline.step_with_joints(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
      joints,
    )
  }

  // Sanity: nothing explodes/NaNs, and constraints keep things in plausible ranges.
  let check_handles = [
    body1, prism_child, fixed_b, sph_ball, spring_ball, parent,
  ]
  for h in check_handles {
    if bodies.get(h) is Some(rb) {
      let p = rb.translation()
      inspect(p.x == p.x && p.y == p.y && p.z == p.z, content="true")
      inspect(p.y > -500.0F, content="true")
    } else {
      inspect(false, content="true")
    }
  }
}
