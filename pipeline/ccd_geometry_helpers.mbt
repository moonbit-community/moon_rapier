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
fn shape_ccd_thickness(shape : @collision.Shape) -> @core.Real {
  match shape {
    @collision.Shape::Round(inner, radius) =>
      shape_ccd_thickness(inner.val) + radius
    @collision.Shape::Ball(radius) => radius
    @collision.Shape::Cuboid(hw, hh) => if hw < hh { hw } else { hh }
    @collision.Shape::HalfSpace(_) => 0.0F
    @collision.Shape::CapsuleX(_, radius) => radius
    @collision.Shape::CapsuleY(_, radius) => radius
    @collision.Shape::Segment(_, _) => 0.0F
    @collision.Shape::Polyline(_, _) => 0.0F
    @collision.Shape::HeightField(_, _) => 0.0F
    @collision.Shape::ConvexPolygon(_) => 0.0F
    @collision.Shape::TriMesh(_, _) => 0.0F
    @collision.Shape::Compound(parts) => {
      if parts.length() == 0 {
        return 0.0F
      }
      let mut best = 1.0e30F
      for i in 0..<parts.length() {
        let t = shape_ccd_thickness(parts[i].1)
        if t < best {
          best = t
        }
      }
      if best >= 1.0e29F {
        0.0F
      } else {
        best
      }
    }
  }
}

///|
fn shape_bounding_radius(shape : @collision.Shape) -> @core.Real {
  match shape {
    @collision.Shape::Round(inner, radius) =>
      shape_bounding_radius(inner.val) + radius
    @collision.Shape::Ball(radius) => radius
    @collision.Shape::Cuboid(hw, hh) => Float::sqrt(hw * hw + hh * hh)
    @collision.Shape::HalfSpace(_) => 0.0F
    @collision.Shape::CapsuleX(half_height, radius)
    | @collision.Shape::CapsuleY(half_height, radius) => half_height + radius
    @collision.Shape::Segment(a, b) => {
      let la = a.length()
      let lb = b.length()
      if la > lb {
        la
      } else {
        lb
      }
    }
    @collision.Shape::Polyline(vertices, _) => {
      let mut best = 0.0F
      for i in 0..<vertices.length() {
        let d = vertices[i].length()
        if d > best {
          best = d
        }
      }
      best
    }
    @collision.Shape::HeightField(heights, scale) => {
      let mut max_abs_y = 0.0F
      for i in 0..<heights.length() {
        let ay = @core.abs(heights[i] * scale.y)
        if ay > max_abs_y {
          max_abs_y = ay
        }
      }
      Float::sqrt(0.5F * scale.x * (0.5F * scale.x) + max_abs_y * max_abs_y)
    }
    @collision.Shape::ConvexPolygon(vertices) => {
      let mut best = 0.0F
      for i in 0..<vertices.length() {
        let d = vertices[i].length()
        if d > best {
          best = d
        }
      }
      best
    }
    @collision.Shape::TriMesh(vertices, _) => {
      let mut best = 0.0F
      for i in 0..<vertices.length() {
        let d = vertices[i].length()
        if d > best {
          best = d
        }
      }
      best
    }
    @collision.Shape::Compound(parts) => {
      let mut best = 0.0F
      for i in 0..<parts.length() {
        let (pose, part_shape) = parts[i]
        let t = pose.transform_point(@core.Vec2::zero())
        let d = t.length() + shape_bounding_radius(part_shape)
        if d > best {
          best = d
        }
      }
      best
    }
  }
}
