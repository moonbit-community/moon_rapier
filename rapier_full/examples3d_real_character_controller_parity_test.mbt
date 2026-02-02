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
/// 3DReal parity tests for `rapier-reference/examples3d/character_controller3.rs`.

///|
test "examples3d/character_controller3.rs parity (3DReal headless)" {
  // Ported from `rapier-reference/examples3d/character_controller3.rs` (headless).
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
  let inv_dt = parameters.inv_dt()
  let dt = if inv_dt == 0.0F { 0.0F } else { 1.0F / inv_dt }

  // Ground.
  let scale = 1.0F
  let ground_size = 5.0F * scale
  let ground_height = 0.1F * scale
  let floor = bodies.insert(
    @dynamics.RigidBodyBuilder3D::fixed()
    .translation(@core.Vec3::new(0.0F, -ground_height, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(
      ground_size, ground_height, ground_size,
    ).build(),
    floor,
    bodies,
  )
  |> ignore
  // Vertical wall.
  let wall = bodies.insert(
    @dynamics.RigidBodyBuilder3D::fixed()
    .translation(@core.Vec3::new(0.0F, -ground_height, -ground_size))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(
      ground_size, ground_size, ground_height,
    ).build(),
    wall,
    bodies,
  )
  |> ignore

  // Character (kinematic position-based capsule).
  let character = bodies.insert(
    @dynamics.RigidBodyBuilder3D::kinematic_position_based()
    .translation(@core.Vec3::new(0.0F, 0.5F, 0.0F))
    .gravity_scale(10.0F)
    .soft_ccd_prediction(10.0F)
    .can_sleep(false)
    .build(),
  )
  let character_collider = colliders.insert_with_parent(
    @collision.ColliderBuilder3D::capsule_y(0.3F, 0.15F).build(),
    character,
    bodies,
  )
  character_collider |> ignore

  // Cubes.
  let num = 8
  let rad = 0.1F * scale
  let shift = rad * 2.0F
  let centerx = shift * Float::from_int(num / 2)
  let centery = rad
  for j in 0..<4 {
    for k in 0..<4 {
      for i in 0..<num {
        let x = Float::from_int(i) * shift - centerx
        let y = Float::from_int(j) * shift + centery
        let z = Float::from_int(k) * shift + centerx
        let h = bodies.insert(
          @dynamics.RigidBodyBuilder3D::dynamic()
          .translation(@core.Vec3::new(x, y, z))
          .build(),
        )
        colliders.insert_with_parent(
          @collision.ColliderBuilder3D::cuboid(rad, rad, rad).build(),
          h,
          bodies,
        )
        |> ignore
      }
    }
  }

  // Stairs.
  let stair_width = 1.0F * scale
  let stair_height = 0.1F * scale
  for i in 0..<10 {
    let x = Float::from_int(i) * stair_width / 2.0F
    let y = Float::from_int(i) * stair_height * 1.5F + 3.0F * scale
    colliders.insert(
      @collision.ColliderBuilder3D::cuboid(
        stair_width / 2.0F,
        stair_height / 2.0F,
        stair_width,
      )
      .translation(@core.Vec3::new(x, y, 0.0F))
      .build(),
    )
    |> ignore
  }

  // Climbable slope.
  let slope_angle = 0.2F
  let slope_size = 2.0F
  colliders.insert(
    @collision.ColliderBuilder3D::cuboid(slope_size, ground_height, slope_size)
    .translation(
      @core.Vec3::new(0.1F + slope_size, -ground_height + 0.4F, 0.0F),
    )
    .rotation(
      @core.rotation_from_scaled_axis(@core.Vec3::new(0.0F, 0.0F, slope_angle)),
    )
    .build(),
  )
  |> ignore

  // Too-steep slope to block movement.
  let impossible_slope_angle = 0.6F
  let impossible_slope_size = 2.0F
  colliders.insert(
    @collision.ColliderBuilder3D::cuboid(
      slope_size * scale,
      ground_height,
      ground_size,
    )
    .translation(
      @core.Vec3::new(
        0.1F + slope_size * 2.0F + impossible_slope_size - 0.9F,
        -ground_height + 1.7F,
        0.0F,
      ),
    )
    .rotation(
      @core.rotation_from_scaled_axis(
        @core.Vec3::new(0.0F, 0.0F, impossible_slope_angle),
      ),
    )
    .build(),
  )
  |> ignore

  // Moving platform.
  let platform = bodies.insert(
    @dynamics.RigidBodyBuilder3D::kinematic_velocity_based()
    .translation(@core.Vec3::new(-8.0F, 0.0F, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(
      2.0F * scale,
      ground_height,
      2.0F * scale,
    ).build(),
    platform,
    bodies,
  )
  |> ignore

  // More complex ground.
  let ground_size2 = @core.Vec3::new(10.0F, 1.0F, 10.0F)
  let nsubdivs = 20
  let heights : Array[@core.Real] = []
  for i in 0..<(nsubdivs + 1) {
    for j in 0..<(nsubdivs + 1) {
      let x = Float::from_int(i) *
        ground_size2.x /
        Float::from_int(nsubdivs) /
        2.0F
      let z = Float::from_int(j) *
        ground_size2.z /
        Float::from_int(nsubdivs) /
        2.0F
      heights.push(
        Float::from_double(@math.cos(x.to_double())) +
        Float::from_double(@math.cos(z.to_double())),
      )
    }
  }
  let hf = @collision.ColliderBuilder3D::heightfield(
      heights,
      nsubdivs + 1,
      nsubdivs + 1,
      ground_size2,
    )
    .unwrap()
    .translation(@core.Vec3::new(-8.0F, 5.0F, 0.0F))
    .build()
  colliders.insert(hf) |> ignore

  // A tilting dynamic body with a limited joint.
  let joint_ground = bodies.insert(
    @dynamics.RigidBodyBuilder3D::fixed()
    .translation(@core.Vec3::new(0.0F, 5.0F, 0.0F))
    .build(),
  )
  let tilting = bodies.insert(
    @dynamics.RigidBodyBuilder3D::dynamic()
    .translation(@core.Vec3::new(0.0F, 5.0F, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(
      1.0F * scale,
      0.1F * scale,
      2.0F * scale,
    ).build(),
    tilting,
    bodies,
  )
  |> ignore
  let locked = @dynamics.JointAxesMask3DReal::lin_axes()
    .or(@dynamics.JointAxesMask3DReal::ang_x())
    .or(@dynamics.JointAxesMask3DReal::ang_y())
  let joint = @dynamics.GenericJoint3DRealBuilder::new(locked)
    .limits(@dynamics.JointAxis3DReal::AngZ, -0.3F, 0.3F)
    .build()
  joints.insert_generic(joint_ground, tilting, joint, true) |> ignore
  let queries = @collision.QueryPipelineMut3DReal::new(
    @collision.QueryFilter3DReal::new(),
    bodies,
    colliders,
  )
  let controller = @control.KinematicCharacterController3DReal::default()
    .with_max_slope_climb_angle(impossible_slope_angle - 0.02F)
    .with_min_slope_slide_angle(impossible_slope_angle - 0.02F)
  let speed = 5.0F
  let mut time = 0.0F
  for _ in 0..<240 {
    let linvel = @core.Vec3::new(
      Float::from_double(@math.sin((time * 2.0F).to_double())) * 2.0F,
      Float::from_double(@math.sin((time * 5.0F).to_double())) * 1.5F,
      0.0F,
    )
    if bodies.get_mut(platform) is Some(rb) {
      rb.set_linvel(linvel)
    }
    colliders.sync_with_bodies(bodies)
    queries.update()
    let qp = queries
      .as_ref()
      .with_filter(
        @collision.QueryFilter3DReal::new().exclude_rigid_body(character),
      )
    let collisions : Array[@control.CharacterCollision3DReal] = []
    if colliders.get(character_collider) is Some(cc) {
      let movement = controller.move_shape(
        dt,
        qp,
        cc.shape(),
        cc.position(),
        @core.Vec3::new(speed * dt, 0.0F, 0.0F),
        c => collisions.push(c),
      )
      controller.solve_character_collision_impulses(
        dt,
        qp,
        cc.shape(),
        1.0F,
        collisions,
      )
      if bodies.get_mut(character) is Some(rb) {
        let next = cc.position().translation.add(movement.translation())
        rb.set_next_kinematic_translation(next)
      }
    } else {
      inspect(false, content="true")
    }
    pipeline.step_with_joints(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
      joints,
    )
    time = time + dt
  }
  if bodies.get(character) is Some(rb) {
    let p = rb.translation()
    inspect(p.x == p.x && p.y == p.y && p.z == p.z, content="true")
    // The character moved forward before being blocked by the steep slope.
    inspect(p.x > 2.0F, content="true")
    inspect(p.x < 7.0F, content="true")
    inspect(p.y > -50.0F, content="true")
  } else {
    inspect(false, content="true")
  }
}
