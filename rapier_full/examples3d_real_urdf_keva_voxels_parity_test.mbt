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
test "examples3d/urdf3.rs parity (3DReal headless; stand-in robot)" {
  // Ported from `rapier-reference/examples3d/urdf3.rs` (headless).
  //
  // Full URDF parsing isn't available in this MoonBit port. Instead we build a small articulated
  // chain to exercise joint limits + motors in the real dim3 backend.
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
  let axis_z = @core.Vec3::new(0.0F, 0.0F, 1.0F)
  let basis = @core.Quat::identity()
  let locked = @dynamics.JointAxesMask3DReal::lin_axes()
    .or(@dynamics.JointAxesMask3DReal::ang_x())
    .or(@dynamics.JointAxesMask3DReal::ang_y())
  let base = bodies.insert(
    @dynamics.RigidBodyBuilder3D::fixed()
    .translation(@core.Vec3::new(0.0F, 2.0F, 0.0F))
    .build(),
  )
  let links : Array[@dynamics.RigidBodyHandle] = [base]
  for i in 0..<4 {
    let x = Float::from_int(i + 1) * 1.0F
    let link = bodies.insert(
      @dynamics.RigidBodyBuilder3D::dynamic()
      .translation(@core.Vec3::new(x, 2.0F, 0.0F))
      .can_sleep(false)
      .build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder3D::cuboid(0.6F, 0.1F, 0.1F).build(),
      link,
      bodies,
    )
    |> ignore
    let parent = links[links.length() - 1]
    let min_angle = -0.5F
    let max_angle = 0.5F
    let joint = @dynamics.GenericJoint3DRealBuilder::new(locked)
      .local_basis1(basis)
      .local_basis2(basis)
      .local_anchor1(@core.Vec3::new(1.0F, 0.0F, 0.0F))
      .local_anchor2(@core.Vec3::new(-1.0F, 0.0F, 0.0F))
      .limits(@dynamics.JointAxis3DReal::AngZ, min_angle, max_angle)
      .build()
    let h = joints.insert_generic(parent, link, joint, true)
    // Drive the first joint a bit.
    if i == 0 {
      joints.set_motor_position(
        h,
        @dynamics.JointAxis3DReal::AngZ,
        0.35F,
        100.0F,
        10.0F,
        1000.0F,
      )
    }
    links.push(link)
  }
  for _ in 0..<240 {
    pipeline.step_with_joints(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
      joints,
    )
  }
  let tip = links[links.length() - 1]
  if bodies.get(tip) is Some(rb) {
    let p = rb.translation()
    inspect(p.x == p.x && p.y == p.y && p.z == p.z, content="true")
    inspect(p.y > -50.0F, content="true")
  } else {
    inspect(false, content="true")
  }
  axis_z |> ignore
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

  // Basic raycast sanity on the voxel collider.
  let qp = @collision.QueryPipeline3DReal::new(
    @collision.QueryFilter3DReal::new(),
    bodies,
    colliders,
  )
  let ray = @collision.Ray3::new(
    @core.Vec3::new(5.0F, 50.0F, 5.0F),
    @core.Vec3::new(0.0F, -1.0F, 0.0F),
  )
  if qp.cast_ray_and_get_normal(ray, 200.0F, true) is Some(_) {
    inspect(true, content="true")
  } else {
    inspect(false, content="true")
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
