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
test "ball-ball contact: symmetry when swapping arguments" {
  let c12 = compute_ball_ball_contact(
    @core.Vec3::zero(),
    1.0F,
    @core.Vec3::new(1.5F, 0.0F, 0.0F),
    1.0F,
  )
  let c21 = compute_ball_ball_contact(
    @core.Vec3::new(1.5F, 0.0F, 0.0F),
    1.0F,
    @core.Vec3::zero(),
    1.0F,
  )
  if c12 is Some(a) && c21 is Some(b) {
    inspect(@core.abs(a.penetration - b.penetration) < 1.0e-6F, content="true")
    inspect(@core.abs(a.normal.x + b.normal.x) < 1.0e-6F, content="true")
    inspect(@core.abs(a.normal.y + b.normal.y) < 1.0e-6F, content="true")
    inspect(@core.abs(a.normal.z + b.normal.z) < 1.0e-6F, content="true")
    // Contact points swap when swapping shapes.
    inspect(@core.abs(a.point1.x - b.point2.x) < 1.0e-6F, content="true")
    inspect(@core.abs(a.point1.y - b.point2.y) < 1.0e-6F, content="true")
    inspect(@core.abs(a.point1.z - b.point2.z) < 1.0e-6F, content="true")
    inspect(@core.abs(a.point2.x - b.point1.x) < 1.0e-6F, content="true")
    inspect(@core.abs(a.point2.y - b.point1.y) < 1.0e-6F, content="true")
    inspect(@core.abs(a.point2.z - b.point1.z) < 1.0e-6F, content="true")
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
test "ball-cuboid contact: ball center inside selects deterministic face normal" {
  let cuboid_pos = @core.Isometry3::identity()
  let he = @core.Vec3::new(1.0F, 1.0F, 1.0F)
  let c = compute_ball_cuboid_contact(@core.Vec3::zero(), 0.5F, cuboid_pos, he)
  if c is Some(cp) {
    inspect(cp.normal.x == 1.0F, content="true")
    inspect(cp.normal.y == 0.0F, content="true")
    inspect(cp.normal.z == 0.0F, content="true")
    inspect(@core.abs(cp.penetration - 1.5F) < 1.0e-6F, content="true")
    inspect(@core.abs(cp.point2.x - 1.0F) < 1.0e-6F, content="true")
    inspect(@core.abs(cp.point1.x - 0.5F) < 1.0e-6F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "ball-trimesh contact: triangle floor overlap" {
  let mesh_pos = @core.Isometry3::identity()
  let vertices = [
    @core.Vec3::new(0.0F, 0.0F, 0.0F),
    @core.Vec3::new(1.0F, 0.0F, 0.0F),
    @core.Vec3::new(0.0F, 0.0F, 1.0F),
  ]
  let indices = [(0, 1, 2)]
  let c = compute_ball_trimesh_contact(
    @core.Vec3::new(0.2F, 0.4F, 0.2F),
    0.5F,
    mesh_pos,
    vertices,
    indices,
  )
  if c is Some(cp) {
    inspect(@core.abs(cp.normal.x) < 1.0e-6F, content="true")
    inspect(@core.abs(cp.normal.y - -1.0F) < 1.0e-6F, content="true")
    inspect(@core.abs(cp.normal.z) < 1.0e-6F, content="true")
    inspect(@core.abs(cp.penetration - 0.1F) < 1.0e-5F, content="true")
    inspect(@core.abs(cp.point2.y - 0.0F) < 1.0e-6F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "cylinder-trimesh contact: triangle floor shallow overlap" {
  let mesh_pos = @core.Isometry3::identity()
  let vertices = [
    @core.Vec3::new(0.0F, 0.0F, 0.0F),
    @core.Vec3::new(2.0F, 0.0F, 0.0F),
    @core.Vec3::new(0.0F, 0.0F, 2.0F),
  ]
  let indices = [(0, 1, 2)]
  let cyl_pos = @core.Isometry3::from_translation(
    @core.Vec3::new(0.5F, 0.45F, 0.5F),
  )
  let c = compute_cylinder_trimesh_contact(
    cyl_pos, 0.5F, 0.5F, mesh_pos, vertices, indices,
  )
  if c is Some(cp) {
    inspect(cp.penetration > 0.0F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "cone-cone contact: shallow overlap produces contact" {
  let cone1_pos = @core.Isometry3::identity()
  let cone2_pos = @core.Isometry3::from_translation(
    @core.Vec3::new(1.9F, 0.0F, 0.0F),
  )
  let c = compute_cone_cone_contact(
    cone1_pos, 1.0F, 1.0F, cone2_pos, 1.0F, 1.0F,
  )
  if c is Some(cp) {
    inspect(cp.penetration > 0.0F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "cone-cuboid contact: shallow overlap produces contact" {
  // Slightly penetrate in the Y axis (GJK+EPA reports contacts only for intersecting shapes).
  let cone_pos = @core.Isometry3::from_translation(
    @core.Vec3::new(1.4F, 0.05F, 0.0F),
  )
  let cuboid_pos = @core.Isometry3::identity()
  let he = @core.Vec3::new(1.0F, 1.0F, 1.0F)
  let c = compute_cone_cuboid_contact(cone_pos, 1.0F, 1.0F, cuboid_pos, he)
  if c is Some(cp) {
    inspect(cp.penetration > 0.0F, content="true")
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
test "cuboid-cuboid contact: axis-aligned overlap along y" {
  let he = @core.Vec3::new(1.0F, 1.0F, 1.0F)
  let a = @core.Isometry3::identity()
  let b = @core.Isometry3::from_translation(@core.Vec3::new(0.0F, 1.9F, 0.0F))
  let c = compute_cuboid_cuboid_contact(a, he, b, he)
  if c is Some(cp) {
    inspect(@core.abs(cp.normal.x) < 1.0e-5F, content="true")
    inspect(@core.abs(cp.normal.y - 1.0F) < 1.0e-5F, content="true")
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

///|
test "ball-capsule contact: overlap on side" {
  let capsule = @core.Isometry3::identity()
  let c = compute_ball_capsule_contact(
    @core.Vec3::new(0.9F, 0.0F, 0.0F),
    0.5F,
    capsule,
    0.5F,
    1.0F,
  )
  if c is Some(cp) {
    inspect(@core.abs(cp.normal.x - -1.0F) < 1.0e-6F, content="true")
    inspect(@core.abs(cp.penetration - 0.1F) < 1.0e-6F, content="true")
    inspect(@core.abs(cp.point2.x - 0.5F) < 1.0e-6F, content="true")
    inspect(@core.abs(cp.point1.x - 0.4F) < 1.0e-6F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "convex contact: cylinder-cuboid overlap produces contact" {
  let cyl_pos = @core.Isometry3::identity()
  let cub_pos = @core.Isometry3::from_translation(
    @core.Vec3::new(0.0F, -0.5F, 0.0F),
  )
  let c = compute_convex_contact(
    cyl_pos,
    Shape3D::Cylinder(1.0F, 1.0F),
    cub_pos,
    Shape3D::Cuboid(@core.Vec3::new(5.0F, 0.25F, 5.0F)),
  )
  if c is Some(cp) {
    inspect(cp.normal.y < 0.0F, content="true")
    inspect(cp.penetration > 0.0F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "capsule-cuboid contact: face overlap" {
  let capsule = @core.Isometry3::from_translation(
    @core.Vec3::new(1.4F, 0.0F, 0.0F),
  )
  let cuboid = @core.Isometry3::identity()
  let he = @core.Vec3::new(1.0F, 1.0F, 1.0F)
  let c = compute_capsule_cuboid_contact(capsule, 0.5F, 1.0F, cuboid, he)
  if c is Some(cp) {
    inspect(@core.abs(cp.normal.x - -1.0F) < 1.0e-6F, content="true")
    inspect(@core.abs(cp.penetration - 0.1F) < 2.0e-4F, content="true")
    inspect(@core.abs(cp.point2.x - 1.0F) < 1.0e-5F, content="true")
    inspect(@core.abs(cp.point1.x - 0.9F) < 1.0e-5F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "capsule-capsule contact: overlap" {
  let a = @core.Isometry3::identity()
  let b = @core.Isometry3::from_translation(@core.Vec3::new(0.8F, 0.0F, 0.0F))
  let c = compute_capsule_capsule_contact(a, 0.5F, 1.0F, b, 0.5F, 1.0F)
  if c is Some(cp) {
    inspect(@core.abs(cp.normal.x - 1.0F) < 1.0e-5F, content="true")
    inspect(cp.penetration > 0.0F, content="true")
  } else {
    inspect(false, content="true")
  }
}
