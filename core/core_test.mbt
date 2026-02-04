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
test "vec2 basics" {
  let v = Vec2::new(1.0F, 2.0F)
  let w = Vec2::new(3.0F, 4.0F)
  let sum = v.add(w)
  inspect(sum.x.to_int(), content="4")
  inspect(sum.y.to_int(), content="6")
  inspect(v.dot(w).to_int(), content="11")
}

///|
test "vec2 normalize" {
  let v = Vec2::new(3.0F, 4.0F).normalize()
  inspect(v.length().to_int(), content="1")
  let zero = Vec2::zero().normalize()
  inspect(zero.length().to_int(), content="0")
}

///|
test "aabb contains" {
  let aabb = Aabb::from_points(Vec2::new(-1.0F, -1.0F), Vec2::new(1.0F, 1.0F))
  inspect(aabb.contains_point(Vec2::new(0.5F, 0.5F)), content="true")
  inspect(aabb.contains_point(Vec2::new(2.0F, 0.0F)), content="false")
}

///|
test "isometry identity" {
  let iso = Isometry2::identity()
  let point = Vec2::new(2.0F, -3.0F)
  let transformed = iso.transform_point(point)
  inspect(transformed.x.to_int(), content="2")
  inspect(transformed.y.to_int(), content="-3")
}

///|
test "vec3 basics" {
  let v = Vec3::new(1.0F, 2.0F, 3.0F)
  let w = Vec3::new(4.0F, 5.0F, 6.0F)
  let sum = v.add(w)
  inspect(sum.x.to_int(), content="5")
  inspect(sum.y.to_int(), content="7")
  inspect(sum.z.to_int(), content="9")
  inspect(v.dot(w).to_int(), content="32")
  let c = v.cross(w)
  inspect(c.x.to_int(), content="-3")
  inspect(c.y.to_int(), content="6")
  inspect(c.z.to_int(), content="-3")
}

///|
test "quat rotation from scaled axis (y, +pi/2)" {
  let q = rotation_from_scaled_axis(Vec3::new(0.0F, pi() * 0.5F, 0.0F))
  let v = Vec3::new(1.0F, 0.0F, 0.0F)
  let r = q.rotate_vec3(v)
  inspect(abs(r.x - 0.0F) < 1.0e-5F, content="true")
  inspect(abs(r.y - 0.0F) < 1.0e-5F, content="true")
  inspect(abs(r.z - -1.0F) < 1.0e-5F, content="true")
}

///|
test "isometry3 inverse roundtrip" {
  let rot = rotation_from_scaled_axis(Vec3::new(0.0F, pi() * 0.3F, 0.0F))
  let iso = Isometry3::new(Vec3::new(1.0F, 2.0F, 3.0F), rot)
  let p = Vec3::new(-4.0F, 5.0F, -6.0F)
  let moved = iso.transform_point(p)
  let back = iso.inverse().transform_point(moved)
  inspect(abs(back.x - p.x) < 1.0e-5F, content="true")
  inspect(abs(back.y - p.y) < 1.0e-5F, content="true")
  inspect(abs(back.z - p.z) < 1.0e-5F, content="true")
}

///|
test "aabb3 intersects + dilated" {
  let a = Aabb3::from_points(
    Vec3::new(-1.0F, -1.0F, -1.0F),
    Vec3::new(1.0F, 1.0F, 1.0F),
  )
  let b = Aabb3::from_points(
    Vec3::new(2.0F, 0.0F, 0.0F),
    Vec3::new(3.0F, 1.0F, 1.0F),
  )
  inspect(a.intersects(b), content="false")
  inspect(a.dilated(1.1F).intersects(b), content="true")
}

///|
test "rapier reference constants" {
  inspect(RAPIER_REFERENCE_VERSION, content="0.32.0")
  inspect(SPATIAL_DIM2, content="3")
  inspect(ANG_DIM2, content="1")
  inspect(MAX_MANIFOLD_POINTS2, content="2")
  inspect(SPATIAL_DIM3, content="6")
  inspect(ANG_DIM3, content="3")
  inspect(MAX_MANIFOLD_POINTS3, content="4")
}

///|
test "dmatrix identity" {
  let m = DMatrix::identity(3)
  if m.get(0, 0) is Some(v) {
    inspect(v.to_int(), content="1")
  } else {
    inspect(false, content="true")
  }
  if m.get(0, 1) is Some(v) {
    inspect(v.to_int(), content="0")
  } else {
    inspect(false, content="true")
  }
  if m.get(2, 2) is Some(v) {
    inspect(v.to_int(), content="1")
  } else {
    inspect(false, content="true")
  }
}
