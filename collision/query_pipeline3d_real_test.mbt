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
test "query_pipeline3d_real: cast_ray_and_get_normal cuboid" {
  let bodies = @dynamics.RigidBodySet3D::new()
  let colliders = ColliderSet3D::new()
  let ground = bodies.insert(@dynamics.RigidBodyBuilder3D::fixed().build())
  colliders.insert_with_parent(
    ColliderBuilder3D::cuboid(10.0F, 1.0F, 10.0F).build(),
    ground,
    bodies,
  )
  |> ignore
  colliders.sync_with_bodies(bodies)
  let filter = QueryFilter3DReal::new()
  let qp = QueryPipeline3DReal::new(filter, bodies, colliders)
  let ray = Ray3::new(
    @core.Vec3::new(0.0F, 5.0F, 0.0F),
    @core.Vec3::new(0.0F, -1.0F, 0.0F),
  )
  if qp.cast_ray_and_get_normal(ray, 10.0F, true) is Some((_h, hit)) {
    inspect(hit.toi() > 3.0F, content="true")
    inspect(hit.normal().y > 0.9F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "query_pipeline3d_real: cast_ray_and_get_normal trimesh triangle" {
  let bodies = @dynamics.RigidBodySet3D::new()
  let colliders = ColliderSet3D::new()
  let ground = bodies.insert(@dynamics.RigidBodyBuilder3D::fixed().build())
  let vtx = [
    @core.Vec3::new(-1.0F, 0.0F, -1.0F),
    @core.Vec3::new(1.0F, 0.0F, -1.0F),
    @core.Vec3::new(0.0F, 0.0F, 1.0F),
  ]
  let idx = [(0, 1, 2)]
  let tri = ColliderBuilder3D::trimesh(vtx, idx).unwrap().build()
  colliders.insert_with_parent(tri, ground, bodies) |> ignore
  colliders.sync_with_bodies(bodies)
  let filter = QueryFilter3DReal::new()
  let qp = QueryPipeline3DReal::new(filter, bodies, colliders)
  let ray = Ray3::new(
    @core.Vec3::new(0.0F, 2.0F, 0.0F),
    @core.Vec3::new(0.0F, -1.0F, 0.0F),
  )
  if qp.cast_ray_and_get_normal(ray, 10.0F, true) is Some((_h, hit)) {
    inspect(hit.toi() > 1.0F, content="true")
    let n = hit.normal()
    inspect(n.x == n.x && n.y == n.y && n.z == n.z, content="true")
    inspect(n.y > 0.9F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "query_pipeline3d_real: cast_shape capsule vs cuboid" {
  let bodies = @dynamics.RigidBodySet3D::new()
  let colliders = ColliderSet3D::new()
  let ground = bodies.insert(@dynamics.RigidBodyBuilder3D::fixed().build())
  colliders.insert_with_parent(
    ColliderBuilder3D::cuboid(1.0F, 1.0F, 1.0F).build(),
    ground,
    bodies,
  )
  |> ignore
  colliders.sync_with_bodies(bodies)
  let qp = QueryPipeline3DReal::new(QueryFilter3DReal::new(), bodies, colliders)
  let shape_pos = @core.Isometry3::from_translation(
    @core.Vec3::new(-5.0F, 0.0F, 0.0F),
  )
  let shape_vel = @core.Vec3::new(1.0F, 0.0F, 0.0F)
  let shape = Shape3D::capsule_y(0.5F, 1.0F)
  let options = ShapeCastOptions3::new(10.0F, true)
  if qp.cast_shape(shape_pos, shape_vel, shape, options) is Some((_h, hit)) {
    inspect(hit.toi() > 3.0F, content="true")
    inspect(hit.toi() < 4.0F, content="true")
    inspect(hit.normal().x < -0.9F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "query_pipeline3d_real: cast_ray_and_get_voxel_key voxels" {
  let bodies = @dynamics.RigidBodySet3D::new()
  let colliders = ColliderSet3D::new()
  let voxel_size = @core.Vec3::new(1.0F, 1.0F, 1.0F)
  let points = [@core.Vec3::new(0.2F, 0.2F, 0.2F)]
  let floor_handle = if ColliderBuilder3D::voxels_from_points(
      voxel_size, points,
    )
    is Some(builder) {
    colliders.insert(builder.build())
  } else {
    inspect(false, content="true")
    colliders.insert(ColliderBuilder3D::cuboid(1.0F, 1.0F, 1.0F).build())
  }
  // Ensure the test filter selects the voxel collider.
  if colliders.get(floor_handle) is Some(co) {
    inspect(co.voxel_key_for_triangle(0) is Some(_), content="true")
  } else {
    inspect(false, content="true")
  }
  let qp = QueryPipeline3DReal::new(
    QueryFilter3DReal::new().predicate((_, co) => co.voxel_key_for_triangle(0)
      is Some(_)),
    bodies,
    colliders,
  )
  let ray = Ray3::new(
    @core.Vec3::new(0.5F, 10.0F, 0.5F),
    @core.Vec3::new(0.0F, -1.0F, 0.0F),
  )
  if qp.cast_ray_and_get_voxel_key(ray, 20.0F, true) is Some((_h, hit, key)) {
    inspect(hit.toi() > 8.5F && hit.toi() < 9.5F, content="true")
    inspect(hit.normal().y > 0.9F, content="true")
    inspect(key.0 == 0 && key.1 == 0 && key.2 == 0, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "query_pipeline3d_real: cast_ray_and_get_voxel_key voxels (wavy floor)" {
  let bodies = @dynamics.RigidBodySet3D::new()
  let colliders = ColliderSet3D::new()
  fn clamp(v : Float, lo : Float, hi : Float) -> Float {
    if v < lo {
      lo
    } else if v > hi {
      hi
    } else {
      v
    }
  }

  let voxel_size = @core.Vec3::new(1.0F, 1.0F, 1.0F)
  let points : Array[@core.Vec3] = []
  let n = 40
  for i in 0..<n {
    for j in 0..<n {
      let x = Float::from_int(i)
      let z = Float::from_int(j)
      let tx = x / Float::from_int(n) * 10.0F
      let tz = z / Float::from_int(n) * 10.0F
      let sx = clamp(Float::from_double(@math.sin(tx.to_double())), -0.8F, 0.8F)
      let cz = clamp(Float::from_double(@math.cos(tz.to_double())), -0.8F, 0.8F)
      let y = sx * cz * 16.0F
      points.push(@core.Vec3::new(x, y, z))
    }
  }
  let floor_handle = if ColliderBuilder3D::voxels_from_points(
      voxel_size, points,
    )
    is Some(builder) {
    colliders.insert(builder.build())
  } else {
    inspect(false, content="true")
    colliders.insert(ColliderBuilder3D::cuboid(1.0F, 1.0F, 1.0F).build())
  }
  let floor_aabb = if colliders.get(floor_handle) is Some(floor) {
    floor.compute_aabb()
  } else {
    @core.Aabb3::new(@core.Vec3::zero(), @core.Vec3::zero())
  }
  let qp = QueryPipeline3DReal::new(
    QueryFilter3DReal::new().predicate((_, co) => co.voxel_key_for_triangle(0)
      is Some(_)),
    bodies,
    colliders,
  )
  let center_x = (floor_aabb.mins.x + floor_aabb.maxs.x) * 0.5F
  let center_z = (floor_aabb.mins.z + floor_aabb.maxs.z) * 0.5F
  let origin_y = floor_aabb.maxs.y + 50.0F
  let ray0 = Ray3::new(
    @core.Vec3::new(center_x, origin_y, center_z),
    @core.Vec3::new(0.0F, -1.0F, 0.0F),
  )
  if qp.cast_ray_and_get_voxel_key(ray0, 200.0F, true) is Some((_h, _hit, _key)) {
    inspect(true, content="true")
  } else {
    inspect(false, content="true")
  }
  let ray1 = Ray3::new(
    @core.Vec3::new(center_x + 0.5F, origin_y, center_z + 0.5F),
    @core.Vec3::new(0.0F, -1.0F, 0.0F),
  )
  if qp.cast_ray_and_get_voxel_key(ray1, 200.0F, true) is Some((_h, _hit, _key)) {
    inspect(true, content="true")
  } else {
    inspect(false, content="true")
  }
}
