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
test "ball-ball contact: no overlap" {
  let c = compute_ball_ball_contact(
    @core.Vec3::zero(),
    1.0F,
    @core.Vec3::new(3.0F, 0.0F, 0.0F),
    1.0F,
  )
  inspect(c is None, content="true")
}

///|
test "ball-ball contact: overlap penetration and normal" {
  let c = compute_ball_ball_contact(
    @core.Vec3::zero(),
    1.0F,
    @core.Vec3::new(1.5F, 0.0F, 0.0F),
    1.0F,
  )
  if c is Some(cp) {
    inspect(@core.abs(cp.normal.x - 1.0F) < 1.0e-6F, content="true")
    inspect(@core.abs(cp.normal.y - 0.0F) < 1.0e-6F, content="true")
    inspect(@core.abs(cp.normal.z - 0.0F) < 1.0e-6F, content="true")
    inspect(@core.abs(cp.penetration - 0.5F) < 1.0e-6F, content="true")
    // Points lie on the spheres along the contact normal.
    inspect(@core.abs(cp.point1.x - 1.0F) < 1.0e-6F, content="true")
    inspect(@core.abs(cp.point2.x - 0.5F) < 1.0e-6F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "ball-ball contact: coincident centers has deterministic normal" {
  let c = compute_ball_ball_contact(
    @core.Vec3::zero(),
    1.0F,
    @core.Vec3::zero(),
    1.0F,
  )
  if c is Some(cp) {
    inspect(cp.normal.x == 1.0F, content="true")
    inspect(cp.normal.y == 0.0F, content="true")
    inspect(cp.normal.z == 0.0F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "ball-cuboid contact: face overlap" {
  let cuboid_pos = @core.Isometry3::identity()
  let he = @core.Vec3::new(1.0F, 1.0F, 1.0F)
  let c = compute_ball_cuboid_contact(
    @core.Vec3::new(1.4F, 0.0F, 0.0F),
    0.5F,
    cuboid_pos,
    he,
  )
  if c is Some(cp) {
    inspect(@core.abs(cp.normal.x - -1.0F) < 1.0e-6F, content="true")
    inspect(@core.abs(cp.penetration - 0.1F) < 1.0e-6F, content="true")
    inspect(@core.abs(cp.point2.x - 1.0F) < 1.0e-6F, content="true")
    inspect(@core.abs(cp.point1.x - 0.9F) < 1.0e-6F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "cuboid-cuboid contact: axis-aligned overlap along x" {
  let he = @core.Vec3::new(1.0F, 1.0F, 1.0F)
  let a = @core.Isometry3::identity()
  let b = @core.Isometry3::from_translation(@core.Vec3::new(1.9F, 0.0F, 0.0F))
  let c = compute_cuboid_cuboid_contact(a, he, b, he)
  if c is Some(cp) {
    inspect(@core.abs(cp.normal.x - 1.0F) < 1.0e-5F, content="true")
    inspect(@core.abs(cp.normal.y) < 1.0e-5F, content="true")
    inspect(@core.abs(cp.normal.z) < 1.0e-5F, content="true")
    inspect(@core.abs(cp.penetration - 0.1F) < 2.0e-4F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "cuboid-cuboid contact: separated" {
  let he = @core.Vec3::new(1.0F, 1.0F, 1.0F)
  let a = @core.Isometry3::identity()
  let b = @core.Isometry3::from_translation(@core.Vec3::new(2.1F, 0.0F, 0.0F))
  let c = compute_cuboid_cuboid_contact(a, he, b, he)
  inspect(c is None, content="true")
}
