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
