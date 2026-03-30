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

///|
test "vhacd parameters expose upstream-equivalent configuration surface" {
  let defaults = VHACDParameters::default()
  inspect(@core.abs(defaults.concavity() - 0.01F) < 1.0e-6F, content="true")
  inspect(@core.abs(defaults.alpha() - 0.05F) < 1.0e-6F, content="true")
  inspect(@core.abs(defaults.beta() - 0.05F) < 1.0e-6F, content="true")
  inspect(defaults.resolution() == 64, content="true")
  inspect(defaults.plane_downsampling() == 4, content="true")
  inspect(defaults.convex_hull_downsampling() == 4, content="true")
  inspect(defaults.convex_hull_approximation(), content="true")
  inspect(defaults.max_convex_hulls() == 1024, content="true")
  if defaults.fill_mode() is FillMode::FloodFill(false, false) {
    inspect(true, content="true")
  } else {
    inspect(false, content="true")
  }

  let tuned = defaults
    .set_concavity(0.02F)
    .set_alpha(0.1F)
    .set_beta(0.15F)
    .set_resolution(96)
    .set_plane_downsampling(2)
    .set_convex_hull_downsampling(3)
    .set_fill_mode(FillMode::SurfaceOnly)
    .set_convex_hull_approximation(false)
    .set_max_convex_hulls(24)
  inspect(@core.abs(tuned.concavity() - 0.02F) < 1.0e-6F, content="true")
  inspect(@core.abs(tuned.alpha() - 0.1F) < 1.0e-6F, content="true")
  inspect(@core.abs(tuned.beta() - 0.15F) < 1.0e-6F, content="true")
  inspect(tuned.resolution() == 96, content="true")
  inspect(tuned.plane_downsampling() == 2, content="true")
  inspect(tuned.convex_hull_downsampling() == 3, content="true")
  inspect(!tuned.convex_hull_approximation(), content="true")
  inspect(tuned.max_convex_hulls() == 24, content="true")
  if tuned.fill_mode() is FillMode::SurfaceOnly {
    inspect(true, content="true")
  } else {
    inspect(false, content="true")
  }
}
