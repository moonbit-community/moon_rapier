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
pub struct SharedShape {
  shape : Shape
}

///|
pub fn SharedShape::new(shape : Shape) -> SharedShape {
  { shape, }
}

///|
pub fn SharedShape::as_shape(self : SharedShape) -> Shape {
  self.shape
}

///|
pub fn SharedShape::ball(radius : @core.Real) -> SharedShape {
  SharedShape::new(Shape::Ball(radius))
}

///|
pub fn SharedShape::cuboid(
  half_width : @core.Real,
  half_height : @core.Real,
) -> SharedShape {
  SharedShape::new(Shape::Cuboid(half_width, half_height))
}

///|
pub fn SharedShape::capsule_x(
  half_height : @core.Real,
  radius : @core.Real,
) -> SharedShape {
  SharedShape::new(Shape::CapsuleX(half_height, radius))
}

///|
pub fn SharedShape::capsule_y(
  half_height : @core.Real,
  radius : @core.Real,
) -> SharedShape {
  SharedShape::new(Shape::CapsuleY(half_height, radius))
}

///|
pub fn SharedShape::segment(a : @core.Vec2, b : @core.Vec2) -> SharedShape {
  SharedShape::new(Shape::Segment(a, b))
}

///|
pub fn SharedShape::triangle(
  a : @core.Vec2,
  b : @core.Vec2,
  c : @core.Vec2,
) -> SharedShape {
  let builder = ColliderBuilder::triangle(a, b, c)
  SharedShape::new(builder.shape)
}

///|
pub fn SharedShape::polyline(
  vertices : Array[@core.Vec2],
  indices : Array[(Int, Int)]?,
) -> SharedShape {
  SharedShape::new(Shape::Polyline(vertices, indices))
}

///|
pub fn SharedShape::trimesh(
  vertices : Array[@core.Vec2],
  indices : Array[(Int, Int, Int)],
) -> SharedShape? {
  match ColliderBuilder::trimesh(vertices, indices) {
    None => None
    Some(builder) => Some(SharedShape::new(builder.shape))
  }
}

///|
pub fn SharedShape::heightfield(
  heights : Array[@core.Real],
  scale : @core.Vec2,
) -> SharedShape {
  let builder = ColliderBuilder::heightfield(heights, scale)
  SharedShape::new(builder.shape)
}

///|
pub struct Triangle {
  a : @core.Vec2
  b : @core.Vec2
  c : @core.Vec2
}

///|
pub fn Triangle::new(
  a : @core.Vec2,
  b : @core.Vec2,
  c : @core.Vec2,
) -> Triangle {
  { a, b, c }
}

///|
pub fn Triangle::as_shape(self : Triangle) -> Shape {
  let builder = ColliderBuilder::triangle(self.a, self.b, self.c)
  builder.shape
}

///|
pub struct HeightField {
  heights : Array[@core.Real]
  scale : @core.Vec2
}

///|
pub fn HeightField::new(
  heights : Array[@core.Real],
  scale : @core.Vec2,
) -> HeightField {
  { heights, scale }
}

///|
pub fn HeightField::as_shape(self : HeightField) -> Shape {
  let builder = ColliderBuilder::heightfield(self.heights, self.scale)
  builder.shape
}

///|
pub(all) enum MeshConverterError {
  ConvexHullFailed
  TriMeshBuilderError
  EmptyVertices
}

///|
pub fn MeshConverterError::to_string(self : MeshConverterError) -> String {
  match self {
    ConvexHullFailed => "ConvexHullFailed"
    TriMeshBuilderError => "TriMeshBuilderError"
    EmptyVertices => "EmptyVertices"
  }
}

///|
pub struct TriMeshFlags {
  value : Int
}

///|
pub fn TriMeshFlags::empty() -> TriMeshFlags {
  { value: 0 }
}

///|
pub fn TriMeshFlags::from_bits(bits : Int) -> TriMeshFlags {
  { value: bits }
}

///|
pub fn TriMeshFlags::bits(self : TriMeshFlags) -> Int {
  self.value
}

///|
pub(all) enum MeshConverter {
  TriMesh
  TriMeshWithFlags(TriMeshFlags)
  Aabb
  Obb
  ConvexHull
  ConvexDecomposition
  ConvexDecompositionWithParams(VHACDParameters)
}

///|
pub fn MeshConverter::trimesh() -> MeshConverter {
  MeshConverter::TriMesh
}

///|
pub fn MeshConverter::trimesh_with_flags(flags : TriMeshFlags) -> MeshConverter {
  MeshConverter::TriMeshWithFlags(flags)
}

///|
pub fn MeshConverter::aabb() -> MeshConverter {
  MeshConverter::Aabb
}

///|
pub fn MeshConverter::obb() -> MeshConverter {
  MeshConverter::Obb
}

///|
pub fn MeshConverter::convex_hull() -> MeshConverter {
  MeshConverter::ConvexHull
}

///|
fn compute_aabb(vertices : Array[@core.Vec2]) -> (@core.Vec2, @core.Vec2)? {
  if vertices.length() == 0 {
    return None
  }
  let mut min_x = vertices[0].x
  let mut min_y = vertices[0].y
  let mut max_x = vertices[0].x
  let mut max_y = vertices[0].y
  for i in 1..<vertices.length() {
    let p = vertices[i]
    if p.x < min_x {
      min_x = p.x
    }
    if p.y < min_y {
      min_y = p.y
    }
    if p.x > max_x {
      max_x = p.x
    }
    if p.y > max_y {
      max_y = p.y
    }
  }
  Some((@core.Vec2::new(min_x, min_y), @core.Vec2::new(max_x, max_y)))
}

///|
pub fn MeshConverter::apply(
  self : MeshConverter,
  vertices : Array[@core.Vec2],
  indices : Array[(Int, Int, Int)],
) -> Result[(SharedShape, @core.Isometry2), MeshConverterError] {
  match self {
    MeshConverter::TriMesh =>
      match SharedShape::trimesh(vertices, indices) {
        None => Err(MeshConverterError::TriMeshBuilderError)
        Some(shape) => Ok((shape, @core.Isometry2::identity()))
      }
    MeshConverter::TriMeshWithFlags(_) =>
      // This port does not implement TriMesh pre-processing flags; treat as regular trimesh.
      match SharedShape::trimesh(vertices, indices) {
        None => Err(MeshConverterError::TriMeshBuilderError)
        Some(shape) => Ok((shape, @core.Isometry2::identity()))
      }
    MeshConverter::Aabb =>
      match compute_aabb(vertices) {
        None => Err(MeshConverterError::EmptyVertices)
        Some((min, max)) => {
          let center = @core.Vec2::new(
            0.5F * (min.x + max.x),
            0.5F * (min.y + max.y),
          )
          let half = @core.Vec2::new(
            0.5F * (max.x - min.x),
            0.5F * (max.y - min.y),
          )
          let shape = SharedShape::cuboid(half.x, half.y)
          Ok((shape, @core.Isometry2::from_translation(center)))
        }
      }
    MeshConverter::Obb =>
      // NOTE: This is a 2D port and we currently fall back to the axis-aligned bounding box.
      MeshConverter::Aabb.apply(vertices, indices)
    MeshConverter::ConvexHull =>
      match ColliderBuilder::convex_hull(vertices) {
        None => Err(MeshConverterError::ConvexHullFailed)
        Some(builder) =>
          Ok((SharedShape::new(builder.shape), @core.Isometry2::identity()))
      }
    MeshConverter::ConvexDecomposition =>
      MeshConverter::ConvexDecompositionWithParams(VHACDParameters::default()).apply(
        vertices, indices,
      )
    MeshConverter::ConvexDecompositionWithParams(params) => {
      fn boundary_segments_from_trimesh(
        indices : Array[(Int, Int, Int)],
      ) -> Array[(Int, Int)] {
        fn min_int(a : Int, b : Int) -> Int {
          if a < b {
            a
          } else {
            b
          }
        }

        fn max_int(a : Int, b : Int) -> Int {
          if a > b {
            a
          } else {
            b
          }
        }

        let counts : @hashmap.HashMap[(Int, Int), Int] = @hashmap.new(
          capacity=indices.length() * 3,
        )
        let keys : Array[(Int, Int)] = []
        fn bump(
          counts : @hashmap.HashMap[(Int, Int), Int],
          keys : Array[(Int, Int)],
          a : Int,
          b : Int,
        ) -> Unit {
          let u = min_int(a, b)
          let v = max_int(a, b)
          let key = (u, v)
          if counts.get(key) is Some(c) {
            counts.set(key, c + 1)
          } else {
            counts.set(key, 1)
            keys.push(key)
          }
        }

        for t in indices {
          let (i0, i1, i2) = t
          bump(counts, keys, i0, i1)
          bump(counts, keys, i1, i2)
          bump(counts, keys, i2, i0)
        }
        let out : Array[(Int, Int)] = []
        for k in keys {
          if counts.get(k) is Some(c) && c == 1 {
            out.push(k)
          }
        }
        out
      }

      let segments = boundary_segments_from_trimesh(indices)
      if segments.length() == 0 {
        return MeshConverter::ConvexHull.apply(vertices, indices)
      }
      let builder = ColliderBuilder::convex_decomposition_with_params(
        vertices, segments, params,
      )
      Ok((SharedShape::new(builder.shape), @core.Isometry2::identity()))
    }
  }
}

///|
pub fn MeshConverter::convert(
  self : MeshConverter,
  vertices : Array[@core.Vec2],
  indices : Array[(Int, Int, Int)],
) -> Result[(SharedShape, @core.Isometry2), MeshConverterError] {
  self.apply(vertices, indices)
}

///|
pub struct ColliderPair {
  collider1 : ColliderHandle
  collider2 : ColliderHandle
}

///|
pub fn ColliderPair::new(
  a : ColliderHandle,
  b : ColliderHandle,
) -> ColliderPair {
  { collider1: a, collider2: b }
}

///|
pub fn ColliderPair::collider1(self : ColliderPair) -> ColliderHandle {
  self.collider1
}

///|
pub fn ColliderPair::collider2(self : ColliderPair) -> ColliderHandle {
  self.collider2
}

///|
pub fn ColliderPair::swap(self : ColliderPair) -> ColliderPair {
  ColliderPair::new(self.collider2, self.collider1)
}

///|
pub(all) enum BroadPhasePairEvent {
  AddPair(ColliderPair)
  DeletePair(ColliderPair)
}

///|
pub(all) enum BvhOptimizationStrategy {
  SubtreeOptimizer
  None
}

///|
pub fn BvhOptimizationStrategy::none() -> BvhOptimizationStrategy {
  BvhOptimizationStrategy::None
}

///|
pub fn BvhOptimizationStrategy::subtree_optimizer() -> BvhOptimizationStrategy {
  BvhOptimizationStrategy::SubtreeOptimizer
}
