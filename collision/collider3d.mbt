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
/// Minimal dim3 shape set needed for the real 3D backend.
pub(all) enum Shape3D {
  Ball(@core.Real)
  Cuboid(@core.Vec3) // half extents
}

///|
pub fn Shape3D::ball(radius : @core.Real) -> Shape3D {
  Shape3D::Ball(radius)
}

///|
pub fn Shape3D::cuboid(
  hx : @core.Real,
  hy : @core.Real,
  hz : @core.Real,
) -> Shape3D {
  Shape3D::Cuboid(@core.Vec3::new(hx, hy, hz))
}

///|
pub fn Shape3D::local_aabb(self : Shape3D) -> @core.Aabb3 {
  match self {
    Shape3D::Ball(r) =>
      @core.Aabb3::new(@core.Vec3::new(-r, -r, -r), @core.Vec3::new(r, r, r))
    Shape3D::Cuboid(he) =>
      @core.Aabb3::new(
        @core.Vec3::new(-he.x, -he.y, -he.z),
        @core.Vec3::new(he.x, he.y, he.z),
      )
  }
}

///|
fn abs_mat3(m : @core.Mat3) -> @core.Mat3 {
  @core.Mat3::new(
    @core.abs(m.m00),
    @core.abs(m.m01),
    @core.abs(m.m02),
    @core.abs(m.m10),
    @core.abs(m.m11),
    @core.abs(m.m12),
    @core.abs(m.m20),
    @core.abs(m.m21),
    @core.abs(m.m22),
  )
}

///|
fn aabb3_transform(iso : @core.Isometry3, aabb : @core.Aabb3) -> @core.Aabb3 {
  // Conservative (and exact for OBBs) transform using center+extents.
  let center = aabb.mins.add(aabb.maxs).scale(0.5F)
  let half = aabb.maxs.sub(aabb.mins).scale(0.5F)
  let rot = abs_mat3(iso.rotation.to_mat3())
  let ext = rot.mul_vec3(half)
  let world_center = iso.transform_point(center)
  @core.Aabb3::new(world_center.sub(ext), world_center.add(ext))
}

///|
pub struct Collider3D {
  mut position : @core.Isometry3
  shape : Shape3D
  mut enabled : Bool
}

///|
pub fn Collider3D::position(self : Collider3D) -> @core.Isometry3 {
  self.position
}

///|
pub fn Collider3D::set_position(
  self : Collider3D,
  pos : @core.Isometry3,
) -> Unit {
  self.position = pos
}

///|
pub fn Collider3D::shape(self : Collider3D) -> Shape3D {
  self.shape
}

///|
pub fn Collider3D::compute_aabb(self : Collider3D) -> @core.Aabb3 {
  aabb3_transform(self.position, self.shape.local_aabb())
}

///|
pub fn Collider3D::compute_collision_aabb(
  self : Collider3D,
  prediction_distance : @core.Real,
) -> @core.Aabb3 {
  self.compute_aabb().dilated(prediction_distance)
}

///|
pub fn Collider3D::enabled(self : Collider3D) -> Bool {
  self.enabled
}

///|
pub fn Collider3D::set_enabled(self : Collider3D, enabled : Bool) -> Collider3D {
  self.enabled = enabled
  self
}

///|
pub struct ColliderBuilder3D {
  mut position : @core.Isometry3
  shape : Shape3D
  mut enabled : Bool
}

///|
pub fn ColliderBuilder3D::ball(radius : @core.Real) -> ColliderBuilder3D {
  {
    position: @core.Isometry3::identity(),
    shape: Shape3D::ball(radius),
    enabled: true,
  }
}

///|
pub fn ColliderBuilder3D::cuboid(
  hx : @core.Real,
  hy : @core.Real,
  hz : @core.Real,
) -> ColliderBuilder3D {
  {
    position: @core.Isometry3::identity(),
    shape: Shape3D::cuboid(hx, hy, hz),
    enabled: true,
  }
}

///|
pub fn ColliderBuilder3D::position(
  self : ColliderBuilder3D,
  pos : @core.Isometry3,
) -> ColliderBuilder3D {
  self.position = pos
  self
}

///|
pub fn ColliderBuilder3D::translation(
  self : ColliderBuilder3D,
  t : @core.Vec3,
) -> ColliderBuilder3D {
  self.position = @core.Isometry3::new(t, self.position.rotation)
  self
}

///|
pub fn ColliderBuilder3D::rotation(
  self : ColliderBuilder3D,
  r : @core.Quat,
) -> ColliderBuilder3D {
  self.position = @core.Isometry3::new(self.position.translation, r)
  self
}

///|
pub fn ColliderBuilder3D::enabled(
  self : ColliderBuilder3D,
  enabled : Bool,
) -> ColliderBuilder3D {
  self.enabled = enabled
  self
}

///|
pub fn ColliderBuilder3D::build(self : ColliderBuilder3D) -> Collider3D {
  { position: self.position, shape: self.shape, enabled: self.enabled }
}
