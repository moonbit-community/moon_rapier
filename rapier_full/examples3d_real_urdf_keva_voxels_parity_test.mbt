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
/// Parity tests for large-scene examples against the real dim3 backend.

///|
test "examples3d/keva3.rs parity (3DReal headless; reduced)" {
  // Ported from `rapier-reference/examples3d/keva3.rs` (headless; reduced counts).
  let pipeline = @pipeline.PhysicsPipeline3DReal::new()
  let broad_phase = @collision.BroadPhase3D::new()
  let narrow_phase = @collision.NarrowPhase3D::new()
  let bodies = @dynamics.RigidBodySet3D::new()
  let colliders = @collision.ColliderSet3D::new()
  let islands = @dynamics.IslandManager3D::new()
  let gravity = @core.Vec3::new(0.0F, -9.81F, 0.0F)
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )

  // Ground.
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder3D::fixed()
    .translation(@core.Vec3::new(0.0F, -0.1F, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(50.0F, 0.1F, 50.0F).build(),
    ground,
    bodies,
  )
  |> ignore

  // A single small keva block "tower" (much smaller than upstream to keep O(n^2) broadphase cheap).
  let half_extents = @core.Vec3::new(0.1F, 0.05F, 0.3F)
  let handles : Array[@dynamics.RigidBodyHandle] = []
  for i in 0..<24 {
    let y = Float::from_int(i) * half_extents.y * 2.2F + 0.5F
    let x = if i % 2 == 0 { 0.0F } else { half_extents.z * 0.8F }
    let z = if i % 2 == 0 { half_extents.z * 0.8F } else { 0.0F }
    let rb = bodies.insert(
      @dynamics.RigidBodyBuilder3D::dynamic()
      .translation(@core.Vec3::new(x, y, z))
      .can_sleep(false)
      .build(),
    )
    handles.push(rb)
    colliders.insert_with_parent(
      @collision.ColliderBuilder3D::cuboid(
        half_extents.x,
        half_extents.y,
        half_extents.z,
      ).build(),
      rb,
      bodies,
    )
    |> ignore
  }
  for _ in 0..<180 {
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    )
  }
  // Sanity: blocks remain finite and above a very low threshold.
  for h in handles {
    if bodies.get(h) is Some(rb) {
      let p = rb.translation()
      inspect(p.x == p.x && p.y == p.y && p.z == p.z, content="true")
      inspect(p.y > -50.0F, content="true")
    } else {
      inspect(false, content="true")
    }
  }
}

///|
test "examples3d/urdf3.rs parity (3DReal headless; T12 subset)" {
  // Ported from `rapier-reference/examples3d/urdf3.rs` (headless).
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
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder3D::fixed()
    .translation(@core.Vec3::new(0.0F, -0.1F, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(50.0F, 0.1F, 50.0F).build(),
    ground,
    bodies,
  )
  |> ignore

  // Load the URDF and insert it twice (the original example uses impulse joints and multibody
  // joints; the real backend currently uses the `JointSet3DReal` constraint set for both).
  let options = @urdf.UrdfLoaderOptions3DReal::default()
    .create_colliders_from_visual_shapes(true)
    .create_colliders_from_collision_shapes(false)
    .make_roots_fixed(true)
    // Z-up to Y-up (rotate around +X by +pi/2).
    .shift(
      @core.Isometry3::new(
        @core.Vec3::zero(),
        @core.rotation_from_scaled_axis(
          @core.Vec3::new(@core.pi() * 0.5F, 0.0F, 0.0F),
        ),
      ),
    )
  let xml = t12_urdf_xml()
  let bounds = t12_mesh_bounds_map()
  if @urdf.UrdfRobot3DReal::from_xml(xml) is Some(robot) {
    robot.insert_3d_real(
      options,
      bounds,
      bodies,
      colliders,
      joints,
      @core.Isometry3::identity(),
    )
    robot.insert_3d_real(
      options,
      bounds,
      bodies,
      colliders,
      joints,
      @core.Isometry3::from_translation(@core.Vec3::new(10.0F, 0.0F, 0.0F)),
    )
  } else {
    inspect(false, content="true")
  }
  for _ in 0..<300 {
    pipeline.step_with_joints(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
      joints,
    )
  }

  // Sanity: all bodies remain finite and above a very low threshold.
  for h in bodies.all_handles() {
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
test "examples3d/voxels3.rs parity (3DReal headless; reduced)" {
  // Ported from `rapier-reference/examples3d/voxels3.rs` (headless; reduced and without editing).
  let pipeline = @pipeline.PhysicsPipeline3DReal::new()
  let broad_phase = @collision.BroadPhase3D::new()
  let narrow_phase = @collision.NarrowPhase3D::new()
  let bodies = @dynamics.RigidBodySet3D::new()
  let colliders = @collision.ColliderSet3D::new()
  let islands = @dynamics.IslandManager3D::new()
  let gravity = @core.Vec3::new(0.0F, -9.81F, 0.0F)
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )

  // A small wavy voxel floor.
  let voxel_size = @core.Vec3::new(1.0F, 1.0F, 1.0F)
  let points : Array[@core.Vec3] = []
  let n = 10
  for i in 0..<n {
    for j in 0..<n {
      let x = Float::from_int(i)
      let z = Float::from_int(j)
      let y = Float::from_double(
          @math.sin(x.to_double() / Float::from_int(n).to_double() * 10.0),
        ) *
        Float::from_double(
          @math.cos(z.to_double() / Float::from_int(n).to_double() * 10.0),
        ) *
        3.0F
      points.push(@core.Vec3::new(x, y, z))
      // Boundary "walls".
      if i == 0 || i == n - 1 || j == 0 || j == n - 1 {
        for k in 1..<5 {
          points.push(@core.Vec3::new(x, y + Float::from_int(k), z))
        }
      }
    }
  }
  if @collision.ColliderBuilder3D::voxels_from_points(voxel_size, points)
    is Some(builder) {
    colliders.insert(builder.build()) |> ignore
  } else {
    inspect(false, content="true")
  }

  // Basic raycast sanity on the voxel collider (before adding dynamic bodies, to avoid accidental
  // hits on corner-touching boxes).
  let qp0 = @collision.QueryPipeline3DReal::new(
    @collision.QueryFilter3DReal::new(),
    bodies,
    colliders,
  )
  let ray0 = @collision.Ray3::new(
    @core.Vec3::new(5.0F, 50.0F, 5.0F),
    @core.Vec3::new(0.0F, -1.0F, 0.0F),
  )
  if qp0.cast_ray_and_get_voxel_key(ray0, 200.0F, true) is Some((_, _, _)) {
    inspect(true, content="true")
  } else {
    inspect(false, content="true")
  }

  // A few falling primitives.
  let handles : Array[@dynamics.RigidBodyHandle] = []
  for i in 0..<6 {
    let x = Float::from_int(i % 4) * 2.5F + 2.0F
    let z = Float::from_int(i / 4) * 2.5F + 2.0F
    let rb = bodies.insert(
      @dynamics.RigidBodyBuilder3D::dynamic()
      .translation(@core.Vec3::new(x, 15.0F, z))
      .can_sleep(false)
      .build(),
    )
    handles.push(rb)
    let co = if i % 2 == 0 {
      @collision.ColliderBuilder3D::ball(0.5F).build()
    } else {
      @collision.ColliderBuilder3D::cuboid(0.5F, 0.5F, 0.5F).build()
    }
    colliders.insert_with_parent(co, rb, bodies) |> ignore
  }
  for _ in 0..<80 {
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    )
  }
  for h in handles {
    if bodies.get(h) is Some(rb) {
      let p = rb.translation()
      inspect(p.x == p.x && p.y == p.y && p.z == p.z, content="true")
      inspect(p.y > -200.0F, content="true")
    } else {
      inspect(false, content="true")
    }
  }
}
