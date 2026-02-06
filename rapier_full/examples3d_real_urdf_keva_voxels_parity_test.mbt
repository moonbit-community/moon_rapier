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
test "examples3d/keva3.rs parity (3DReal headless)" {
  // Ported from `rapier-reference/examples3d/keva3.rs` (headless).
  let pipeline = @pipeline.PhysicsPipeline3DReal::new()
  let broad_phase = @collision.BroadPhase3D::new()
  let narrow_phase = @collision.NarrowPhase3D::new()
  let bodies = @dynamics.RigidBodySet3D::new()
  let colliders = @collision.ColliderSet3D::new()
  let islands = @dynamics.IslandManager3D::new()
  let gravity = @core.Vec3::new(0.0F, -9.81F, 0.0F)
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    8,
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
  fn build_block(
    bodies : @dynamics.RigidBodySet3D,
    colliders : @collision.ColliderSet3D,
    half_extents : @core.Vec3,
    shift : @core.Vec3,
    numx : Int,
    numy : Int,
    numz : Int,
  ) -> Unit {
    let dimensions = [
      half_extents,
      @core.Vec3::new(half_extents.z, half_extents.y, half_extents.x),
    ]
    let block_width = 2.0F * half_extents.z * Float::from_int(numx)
    let block_height = 2.0F * half_extents.y * Float::from_int(numy)
    let spacing = (half_extents.z * Float::from_int(numx) - half_extents.x) /
      (Float::from_int(numz) - 1.0F)
    let mut nx = numx
    let mut nz = numz
    for i in 0..<numy {
      // Matches the upstream pattern: swap at the beginning of each layer.
      let tmp = nx
      nx = nz
      nz = tmp
      let dim = dimensions[i % 2]
      let y = dim.y * Float::from_int(i) * 2.0F
      for j in 0..<nx {
        let x = if i % 2 == 0 {
          spacing * Float::from_int(j) * 2.0F
        } else {
          dim.x * Float::from_int(j) * 2.0F
        }
        for k in 0..<nz {
          let z = if i % 2 == 0 {
            dim.z * Float::from_int(k) * 2.0F
          } else {
            spacing * Float::from_int(k) * 2.0F
          }
          let rb = bodies.insert(
            @dynamics.RigidBodyBuilder3D::dynamic()
            .translation(
              @core.Vec3::new(
                x + dim.x + shift.x,
                y + dim.y + shift.y,
                z + dim.z + shift.z,
              ),
            )
            .build(),
          )
          colliders.insert_with_parent(
            @collision.ColliderBuilder3D::cuboid(dim.x, dim.y, dim.z).build(),
            rb,
            bodies,
          )
          |> ignore
        }
      }
    }

    // Close the top.
    let dim = @core.Vec3::new(half_extents.z, half_extents.x, half_extents.y)
    let nx_top = @math.floor((block_width / (dim.x * 2.0F)).to_double()).to_int()
    let nz_top = @math.floor((block_width / (dim.z * 2.0F)).to_double()).to_int()
    for i in 0..<nx_top {
      for j in 0..<nz_top {
        let rb = bodies.insert(
          @dynamics.RigidBodyBuilder3D::dynamic()
          .translation(
            @core.Vec3::new(
              Float::from_int(i) * dim.x * 2.0F + dim.x + shift.x,
              dim.y + shift.y + block_height,
              Float::from_int(j) * dim.z * 2.0F + dim.z + shift.z,
            ),
          )
          .build(),
        )
        colliders.insert_with_parent(
          @collision.ColliderBuilder3D::cuboid(dim.x, dim.y, dim.z).build(),
          rb,
          bodies,
        )
        |> ignore
      }
    }
  }

  // Reduced keva scene to keep runtime tractable on the pure MoonBit backend.
  let half_extents = @core.Vec3::new(0.1F, 0.5F, 2.0F)
  let numx = 2
  let ny = 8
  let numz = numx * 3 + 1
  let block_width = Float::from_int(numx) * half_extents.z * 2.0F
  build_block(
    bodies,
    colliders,
    half_extents,
    @core.Vec3::new(-block_width / 2.0F, 0.0F, -block_width / 2.0F),
    numx,
    ny,
    numz,
  )
  for _ in 0..<40 {
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    )
  }
  // Sanity: blocks remain finite and above a very low threshold.
  for h in bodies.all_handles() {
    if bodies.get(h) is Some(rb) {
      let p = rb.translation()
      inspect(p.x == p.x && p.y == p.y && p.z == p.z, content="true")
      inspect(p.y > -500.0F, content="true")
    } else {
      inspect(false, content="true")
    }
  }
}

///|
test "examples3d/urdf3.rs parity (3DReal headless)" {
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
    8,
  )

  // Load the URDF once (smoke-scale parity scenario).
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
  } else {
    inspect(false, content="true")
  }
  for _ in 0..<120 {
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
test "examples3d/voxels3.rs parity (3DReal headless)" {
  // Ported from `rapier-reference/examples3d/voxels3.rs` (headless; settings fixed to defaults).
  let pipeline = @pipeline.PhysicsPipeline3DReal::new()
  let broad_phase = @collision.BroadPhase3D::new()
  let narrow_phase = @collision.NarrowPhase3D::new()
  let bodies = @dynamics.RigidBodySet3D::new()
  let colliders = @collision.ColliderSet3D::new()
  let islands = @dynamics.IslandManager3D::new()
  let gravity = @core.Vec3::new(0.0F, -9.81F, 0.0F)
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    8,
  )

  // Defaults in the upstream testbed UI:
  // - Falling objects: Mixed
  // - Voxel size y: 1.0
  // - Test CCD: false
  let falling_objects = 5
  let voxel_size_y = 1.0F
  let voxel_size = @core.Vec3::new(1.0F, voxel_size_y, 1.0F)
  fn clamp(v : Float, lo : Float, hi : Float) -> Float {
    if v < lo {
      lo
    } else if v > hi {
      hi
    } else {
      v
    }
  }

  // A voxelized wavy floor.
  let points : Array[@core.Vec3] = []
  let n = 24
  for i in 0..<n {
    for j in 0..<n {
      let x = Float::from_int(i)
      let z = Float::from_int(j)
      let tx = x / Float::from_int(n) * 10.0F
      let tz = z / Float::from_int(n) * 10.0F
      let sx = clamp(Float::from_double(@math.sin(tx.to_double())), -0.8F, 0.8F)
      let cz = clamp(Float::from_double(@math.cos(tz.to_double())), -0.8F, 0.8F)
      let y = sx * cz * 16.0F
      points.push(@core.Vec3::new(x, y * voxel_size_y, z))
      // Boundary "walls".
      if i == 0 || i == n - 1 || j == 0 || j == n - 1 {
        for k in 0..<2 {
          points.push(
            @core.Vec3::new(x, (y + Float::from_int(k)) * voxel_size_y, z),
          )
        }
      }
    }
  }
  let floor_handle = if @collision.ColliderBuilder3D::voxels_from_points(
      voxel_size, points,
    )
    is Some(builder) {
    colliders.insert(builder.build())
  } else {
    inspect(false, content="true")
    colliders.insert(
      @collision.ColliderBuilder3D::cuboid(1.0F, 1.0F, 1.0F).build(),
    )
  }
  let floor_aabb = if colliders.get(floor_handle) is Some(floor) {
    floor.compute_aabb()
  } else {
    @core.Aabb3::new(@core.Vec3::zero(), @core.Vec3::zero())
  }

  // Basic raycast sanity on the voxel collider (before adding dynamic bodies, to avoid accidental
  // hits on corner-touching boxes).
  let qp0 = @collision.QueryPipeline3DReal::new(
    @collision.QueryFilter3DReal::new().predicate((_, co) => co.voxel_key_for_triangle(
        0,
      )
      is Some(_)),
    bodies,
    colliders,
  )
  let ray0 = @collision.Ray3::new(
    @core.Vec3::new(
      (floor_aabb.mins.x + floor_aabb.maxs.x) * 0.5F,
      floor_aabb.maxs.y + 50.0F,
      (floor_aabb.mins.z + floor_aabb.maxs.z) * 0.5F,
    ),
    @core.Vec3::new(0.0F, -1.0F, 0.0F),
  )
  if qp0.cast_ray_and_get_voxel_key(bodies, colliders, ray0, 200.0F, true)
    is Some((_, _, _)) {
    inspect(true, content="true")
  } else {
    inspect(false, content="true")
  }

  // Dynamic primitives.
  // Keep the runtime bounded while still exercising voxel contacts with multiple shape types.
  let nik = 3
  let floor_size = floor_aabb.maxs.sub(floor_aabb.mins)
  let extents = floor_size.scale(0.75F)
  let margin = floor_size.sub(extents).scale(0.5F)
  let ball_radius = 0.5F
  let handles : Array[@dynamics.RigidBodyHandle] = []
  for i in 0..<nik {
    for j in 0..<3 {
      for k in 0..<nik {
        let rb = bodies.insert(
          @dynamics.RigidBodyBuilder3D::dynamic()
          .translation(
            @core.Vec3::new(
              floor_aabb.mins.x +
              margin.x +
              Float::from_int(i) * extents.x / Float::from_int(nik),
              floor_aabb.maxs.y + Float::from_int(j) * 2.0F,
              floor_aabb.mins.z +
              margin.z +
              Float::from_int(k) * extents.z / Float::from_int(nik),
            ),
          )
          .build(),
        )
        handles.push(rb)
        let kind = if falling_objects == 5 { j % 5 } else { falling_objects }
        let co = match kind {
          0 => @collision.ColliderBuilder3D::ball(ball_radius).build()
          1 =>
            @collision.ColliderBuilder3D::cuboid(
              ball_radius, ball_radius, ball_radius,
            ).build()
          2 =>
            @collision.ColliderBuilder3D::cylinder(ball_radius, ball_radius).build()
          3 =>
            @collision.ColliderBuilder3D::cone(ball_radius, ball_radius).build()
          _ =>
            @collision.ColliderBuilder3D::capsule_y(ball_radius, ball_radius).build()
        }
        colliders.insert_with_parent(co, rb, bodies) |> ignore
      }
    }
  }
  for _ in 0..<20 {
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
