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
test "collider3d ball aabb" {
  let co = ColliderBuilder3D::ball(0.5F)
    .translation(@core.Vec3::new(1.0F, 2.0F, 3.0F))
    .build()
  let aabb = co.compute_aabb()
  inspect(@core.abs(aabb.mins.x - 0.5F) < 1.0e-6F, content="true")
  inspect(@core.abs(aabb.mins.y - 1.5F) < 1.0e-6F, content="true")
  inspect(@core.abs(aabb.mins.z - 2.5F) < 1.0e-6F, content="true")
  inspect(@core.abs(aabb.maxs.x - 1.5F) < 1.0e-6F, content="true")
  inspect(@core.abs(aabb.maxs.y - 2.5F) < 1.0e-6F, content="true")
  inspect(@core.abs(aabb.maxs.z - 3.5F) < 1.0e-6F, content="true")
}

///|
test "collider3d cuboid aabb identity rotation" {
  let co = ColliderBuilder3D::cuboid(1.0F, 2.0F, 3.0F).build()
  let aabb = co.compute_aabb()
  inspect(@core.abs(aabb.mins.x - -1.0F) < 1.0e-6F, content="true")
  inspect(@core.abs(aabb.mins.y - -2.0F) < 1.0e-6F, content="true")
  inspect(@core.abs(aabb.mins.z - -3.0F) < 1.0e-6F, content="true")
  inspect(@core.abs(aabb.maxs.x - 1.0F) < 1.0e-6F, content="true")
  inspect(@core.abs(aabb.maxs.y - 2.0F) < 1.0e-6F, content="true")
  inspect(@core.abs(aabb.maxs.z - 3.0F) < 1.0e-6F, content="true")
}

///|
test "collider3d cuboid aabb rotated around y swaps x/z extents" {
  let rot = @core.rotation_from_scaled_axis(
    @core.Vec3::new(0.0F, @core.pi() * 0.5F, 0.0F),
  )
  let co = ColliderBuilder3D::cuboid(1.0F, 2.0F, 3.0F).rotation(rot).build()
  let aabb = co.compute_aabb()
  // With 90deg around Y, the local X half-extent maps to world Z, local Z maps to world X.
  inspect(@core.abs(aabb.mins.x - -3.0F) < 1.0e-5F, content="true")
  inspect(@core.abs(aabb.mins.y - -2.0F) < 1.0e-5F, content="true")
  inspect(@core.abs(aabb.mins.z - -1.0F) < 1.0e-5F, content="true")
  inspect(@core.abs(aabb.maxs.x - 3.0F) < 1.0e-5F, content="true")
  inspect(@core.abs(aabb.maxs.y - 2.0F) < 1.0e-5F, content="true")
  inspect(@core.abs(aabb.maxs.z - 1.0F) < 1.0e-5F, content="true")
}
