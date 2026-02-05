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
test "mesh converter aabb returns cuboid + translation pose" {
  let vertices = [
    @core.Vec2::new(0.0F, 0.0F),
    @core.Vec2::new(2.0F, 0.0F),
    @core.Vec2::new(2.0F, 4.0F),
    @core.Vec2::new(0.0F, 4.0F),
  ]
  let result = MeshConverter::aabb().apply(vertices, [])
  if result is Ok((shape, pose)) {
    if shape.as_shape() is Shape::Cuboid(hx, hy) {
      inspect(@core.abs(hx - 1.0F) < 1.0e-6F, content="true")
      inspect(@core.abs(hy - 2.0F) < 1.0e-6F, content="true")
    } else {
      inspect(false, content="true")
    }
    let p = pose.translation
    inspect(@core.abs(p.x - 1.0F) < 1.0e-6F, content="true")
    inspect(@core.abs(p.y - 2.0F) < 1.0e-6F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "mesh converter convex hull returns convex polygon shape" {
  let vertices = [
    @core.Vec2::new(0.0F, 0.0F),
    @core.Vec2::new(2.0F, 0.0F),
    @core.Vec2::new(0.0F, 1.0F),
    @core.Vec2::new(0.1F, 0.1F),
  ]
  let result = MeshConverter::convex_hull().apply(vertices, [])
  if result is Ok((shape, _)) {
    if shape.as_shape() is Shape::ConvexPolygon(hull) {
      inspect(hull.length() >= 3, content="true")
    } else {
      inspect(false, content="true")
    }
  } else {
    inspect(false, content="true")
  }
}

///|
test "mesh converter convex decomposition honors max_convex_hulls" {
  // A simple concave pentagon: (0,0)->(2,0)->(2,2)->(1,1)->(0,2).
  let vertices = [
    @core.Vec2::new(0.0F, 0.0F),
    @core.Vec2::new(2.0F, 0.0F),
    @core.Vec2::new(2.0F, 2.0F),
    @core.Vec2::new(1.0F, 1.0F),
    @core.Vec2::new(0.0F, 2.0F),
  ]
  let indices = [(0, 1, 3), (1, 2, 3), (0, 3, 4)]
  let hull = MeshConverter::convex_hull().apply(vertices, indices)
  if hull is Ok((shape, _)) {
    if shape.as_shape() is Shape::ConvexPolygon(h) {
      inspect(h.length() >= 3, content="true")
    } else {
      inspect(false, content="true")
    }
  } else {
    inspect(false, content="true")
  }
  let params_one = VHACDParameters::default().set_max_convex_hulls(1)
  let decomp_one = MeshConverter::ConvexDecompositionWithParams(params_one).apply(
    vertices, indices,
  )
  if decomp_one is Ok((shape, _)) {
    // With a budget of 1 hull, we fall back to a single convex hull.
    if shape.as_shape() is Shape::ConvexPolygon(_) {
      inspect(true, content="true")
    } else {
      inspect(false, content="true")
    }
  } else {
    inspect(false, content="true")
  }
  let params_many = VHACDParameters::default().set_max_convex_hulls(16)
  let decomp_many = MeshConverter::ConvexDecompositionWithParams(params_many).apply(
    vertices, indices,
  )
  if decomp_many is Ok((shape, _)) {
    // With a larger budget, we should get a compound of multiple convex pieces (not a single hull).
    if shape.as_shape() is Shape::Compound(parts) {
      inspect(parts.length() > 1, content="true")
    } else {
      inspect(false, content="true")
    }
  } else {
    inspect(false, content="true")
  }
}
