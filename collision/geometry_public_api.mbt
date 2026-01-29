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
pub(all) enum MeshConverter {
  TriMesh
  Aabb
  Obb
  ConvexHull
  ConvexDecomposition
}

///|
pub fn MeshConverter::trimesh() -> MeshConverter {
  MeshConverter::TriMesh
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
        Some(builder) => Ok((SharedShape::new(builder.shape), @core.Isometry2::identity()))
      }
    MeshConverter::ConvexDecomposition =>
      // NOTE: Convex decomposition isn't implemented yet; we approximate with the convex hull
      // for determinism and a usable fallback.
      MeshConverter::ConvexHull.apply(vertices, indices)
  }
}

///|
pub struct ColliderPair {
  collider1 : ColliderHandle
  collider2 : ColliderHandle
}

///|
pub fn ColliderPair::new(a : ColliderHandle, b : ColliderHandle) -> ColliderPair {
  if handle_less(a, b) {
    { collider1: a, collider2: b }
  } else {
    { collider1: b, collider2: a }
  }
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
