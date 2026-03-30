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
/// Parity tests for `rapier-reference/examples3d/vehicle_joints3.rs` against the real dim3 backend.

///|
test "examples3d/vehicle_joints3.rs parity (3DReal headless)" {
  // Ported from `rapier-reference/examples3d/vehicle_joints3.rs` (headless).
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

  // Ground heightfield.
  let ground_size = @core.Vec3::new(60.0F, 0.4F, 60.0F)
  let nsubdivs = 100
  let heights : Array[@core.Real] = []
  for i in 0..<(nsubdivs + 1) {
    for j in 0..<(nsubdivs + 1) {
      let x = Float::from_int(i) *
        ground_size.x /
        Float::from_int(nsubdivs) /
        2.0F
      let z = Float::from_int(j) *
        ground_size.z /
        Float::from_int(nsubdivs) /
        2.0F
      heights.push(
        -Float::from_double(@math.cos(x.to_double())) -
        Float::from_double(@math.cos(z.to_double())),
      )
    }
  }
  let ground = bodies.insert(@dynamics.RigidBodyBuilder3D::fixed().build())
  let hf = @collision.ColliderBuilder3D::heightfield(
      heights,
      nsubdivs + 1,
      nsubdivs + 1,
      ground_size,
    )
    .unwrap()
    .translation(@core.Vec3::new(-7.0F, 0.0F, 0.0F))
    .friction(1.0F)
    .build()
  colliders.insert_with_parent(hf, ground, bodies) |> ignore

  // Vehicle chassis + wheels, simulated with joints.
  let wheel_params = [
    @core.Vec3::new(0.6874F, 0.2783F, -0.7802F),
    @core.Vec3::new(-0.6874F, 0.2783F, -0.7802F),
    @core.Vec3::new(0.64F, 0.2783F, 1.0254F),
    @core.Vec3::new(-0.64F, 0.2783F, 1.0254F),
  ]
  let suspension_height = 0.12F
  let max_steering_angle = 0.61086524F // 35 degrees in radians.
  let drive_strength = 1.0F
  let wheel_radius = 0.28F
  let car_position = @core.Vec3::new(
    0.0F,
    wheel_radius + suspension_height,
    0.0F,
  )
  let body_position_in_car_space = @core.Vec3::new(0.0F, 0.4739F, 0.0F)
  let body_position = car_position.add(body_position_in_car_space)
  let chassis = bodies.insert(
    @dynamics.RigidBodyBuilder3D::dynamic()
    .translation(body_position)
    .can_sleep(false)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(0.65F, 0.3F, 0.9F)
    .density(100.0F)
    .build(),
    chassis,
    bodies,
  )
  |> ignore
  let steering_joints : Array[@dynamics.JointHandle3DReal] = []
  let motor_joints : Array[@dynamics.JointHandle3DReal] = []
  for wheel_id in 0..<wheel_params.length() {
    let wheel_pos_in_car_space = wheel_params[wheel_id]
    let is_front = wheel_id >= 2
    let wheel_center = car_position.add(wheel_pos_in_car_space)

    // Axle rigid-body.
    let axle = bodies.insert(
      @dynamics.RigidBodyBuilder3D::dynamic()
      .translation(wheel_center)
      .can_sleep(false)
      .build(),
    )

    // Wheel rigid-body.
    let wheel = bodies.insert(
      @dynamics.RigidBodyBuilder3D::dynamic()
      .translation(wheel_center)
      .can_sleep(false)
      .build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder3D::ball(wheel_radius)
      .density(100.0F)
      .friction(1.0F)
      .build(),
      wheel,
      bodies,
    )
    |> ignore

    // Suspension between the chassis and the axle: allow LinY within [0, suspension_height],
    // motorize LinY toward 0, and optionally allow/limit AngY for steering.
    let suspension_attachment_in_body_space = wheel_pos_in_car_space.sub(
      body_position_in_car_space,
    )
    let mut locked_axes = @dynamics.JointAxesMask3DReal::lin_x()
      .or(@dynamics.JointAxesMask3DReal::lin_z())
      .or(@dynamics.JointAxesMask3DReal::ang_x())
      .or(@dynamics.JointAxesMask3DReal::ang_z())
    if !is_front {
      locked_axes = locked_axes.or(@dynamics.JointAxesMask3DReal::ang_y())
    }
    let mut suspension_joint = @dynamics.GenericJoint3DRealBuilder::new(
        locked_axes,
      )
      .local_anchor1(suspension_attachment_in_body_space)
      .limits(@dynamics.JointAxis3DReal::LinY, 0.0F, suspension_height)
      .motor_position(@dynamics.JointAxis3DReal::LinY, 0.0F, 1.0e4F, 1.0e3F)
    if is_front {
      suspension_joint = suspension_joint.limits(
        @dynamics.JointAxis3DReal::AngY,
        -max_steering_angle,
        max_steering_angle,
      )
    }
    let suspension_handle = joints.insert_generic(
      chassis,
      axle,
      suspension_joint.build(),
      true,
    )
    if is_front {
      steering_joints.push(suspension_handle)
    }

    // Joint between the axle and the wheel: revolute about X.
    let wheel_joint = joints.insert_revolute(
      axle,
      wheel,
      @core.Vec3::new(1.0F, 0.0F, 0.0F),
      @core.Vec3::zero(),
      @core.Vec3::zero(),
      true,
    )
    if is_front {
      motor_joints.push(wheel_joint)
    }
  }

  // Drive forward and steer for a while.
  for step in 0..<480 {
    let steering = if step < 240 { 0.5F } else { -0.25F }
    let thrust = if step < 360 { drive_strength } else { 0.0F }
    let boost = if step < 240 { 1.5F } else { 1.0F }
    let differential_strength = 0.5F
    let sideways_shift = Float::from_double(
        @math.sin((max_steering_angle * steering).to_double()),
      ) *
      differential_strength
    let speed_diff = if sideways_shift > 0.0F {
      Float::sqrt(1.0F + sideways_shift * sideways_shift)
    } else {
      1.0F / Float::sqrt(1.0F + sideways_shift * sideways_shift)
    }
    let ms = [1.0F / speed_diff, speed_diff]
    for h in steering_joints {
      joints.set_motor_position(
        h,
        @dynamics.JointAxis3DReal::AngY,
        max_steering_angle * steering,
        1.0e4F,
        1.0e3F,
        1.0e4F,
      )
    }
    let mut msi = 0
    for h in motor_joints {
      let scale = ms[msi % 2]
      msi = msi + 1
      joints.set_motor_velocity(
        h,
        @dynamics.JointAxis3DReal::AngX,
        -30.0F * thrust * scale * boost,
        1.0e2F,
        1.0e4F,
      )
    }
    pipeline.step_with_joints(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
      joints,
    )
  }

  // Sanity: the chassis moved and stays finite.
  if bodies.get(chassis) is Some(rb) {
    let p = rb.translation()
    inspect(p.x == p.x && p.y == p.y && p.z == p.z, content="true")
    inspect(p.y > -50.0F, content="true")
    inspect(@core.abs(p.x) + @core.abs(p.z) > 0.2F, content="true")
  } else {
    inspect(false, content="true")
  }
}
