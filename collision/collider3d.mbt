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
  CapsuleY(@core.Real, @core.Real) // radius, half_height
  Cylinder(@core.Real, @core.Real) // radius, half_height (aligned with Y)
  RoundCylinder(@core.Real, @core.Real, @core.Real) // radius, half_height, border_radius
  Cone(@core.Real, @core.Real) // radius, half_height (aligned with Y)
  TriMesh(Array[@core.Vec3], Array[(Int, Int, Int)])
}

///|
const HALF_PI_3D : @core.Real = 1.5707964F

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
pub fn Shape3D::capsule_y(
  radius : @core.Real,
  half_height : @core.Real,
) -> Shape3D {
  Shape3D::CapsuleY(radius, half_height)
}

///|
pub fn Shape3D::cylinder(
  radius : @core.Real,
  half_height : @core.Real,
) -> Shape3D {
  Shape3D::Cylinder(radius, half_height)
}

///|
pub fn Shape3D::round_cylinder(
  radius : @core.Real,
  half_height : @core.Real,
  border_radius : @core.Real,
) -> Shape3D {
  Shape3D::RoundCylinder(radius, half_height, border_radius)
}

///|
pub fn Shape3D::cone(radius : @core.Real, half_height : @core.Real) -> Shape3D {
  Shape3D::Cone(radius, half_height)
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
    Shape3D::CapsuleY(r, hh) =>
      @core.Aabb3::new(
        @core.Vec3::new(-r, -(hh + r), -r),
        @core.Vec3::new(r, hh + r, r),
      )
    Shape3D::Cylinder(r, hh) =>
      @core.Aabb3::new(@core.Vec3::new(-r, -hh, -r), @core.Vec3::new(r, hh, r))
    Shape3D::RoundCylinder(r, hh, br) => {
      let rr = r + br
      let yy = hh + br
      @core.Aabb3::new(
        @core.Vec3::new(-rr, -yy, -rr),
        @core.Vec3::new(rr, yy, rr),
      )
    }
    Shape3D::Cone(r, hh) =>
      @core.Aabb3::new(@core.Vec3::new(-r, -hh, -r), @core.Vec3::new(r, hh, r))
    Shape3D::TriMesh(vertices, _) =>
      if vertices.length() == 0 {
        @core.Aabb3::new(@core.Vec3::zero(), @core.Vec3::zero())
      } else {
        let mut min = vertices[0]
        let mut max = vertices[0]
        for i in 1..<vertices.length() {
          let v = vertices[i]
          if v.x < min.x {
            min = @core.Vec3::new(v.x, min.y, min.z)
          }
          if v.y < min.y {
            min = @core.Vec3::new(min.x, v.y, min.z)
          }
          if v.z < min.z {
            min = @core.Vec3::new(min.x, min.y, v.z)
          }
          if v.x > max.x {
            max = @core.Vec3::new(v.x, max.y, max.z)
          }
          if v.y > max.y {
            max = @core.Vec3::new(max.x, v.y, max.z)
          }
          if v.z > max.z {
            max = @core.Vec3::new(max.x, max.y, v.z)
          }
        }
        @core.Aabb3::new(min, max)
      }
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
  mut local_position : @core.Isometry3
  mut parent : @dynamics.RigidBodyHandle?
  mut shape : Shape3D
  mut enabled : Bool
  sensor : Bool
  // Solver-only features used by a subset of examples/tests (e.g. one-way platforms).
  mut surface_velocity : @core.Vec3
  mut one_way_offset : @core.Real?
  mut one_way_direction : Int // 1 = allow from above, -1 = allow from below, 0 = disabled
  active_events : ActiveEvents
  collision_groups : @dynamics.InteractionGroups
  friction : @core.Real
  restitution : @core.Real
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
pub fn Collider3D::local_position(self : Collider3D) -> @core.Isometry3 {
  self.local_position
}

///|
pub fn Collider3D::set_local_position(
  self : Collider3D,
  pos : @core.Isometry3,
) -> Unit {
  self.local_position = pos
}

///|
pub fn Collider3D::parent(self : Collider3D) -> @dynamics.RigidBodyHandle? {
  self.parent
}

///|
pub fn Collider3D::set_parent(
  self : Collider3D,
  parent : @dynamics.RigidBodyHandle?,
) -> Unit {
  self.parent = parent
}

///|
pub fn Collider3D::shape(self : Collider3D) -> Shape3D {
  self.shape
}

///|
pub fn Collider3D::set_shape(self : Collider3D, shape : Shape3D) -> Unit {
  self.shape = shape
}

///|
pub fn Collider3D::is_sensor(self : Collider3D) -> Bool {
  self.sensor
}

///|
pub fn Collider3D::surface_velocity(self : Collider3D) -> @core.Vec3 {
  self.surface_velocity
}

///|
pub fn Collider3D::set_surface_velocity(
  self : Collider3D,
  v : @core.Vec3,
) -> Unit {
  self.surface_velocity = v
}

///|
pub fn Collider3D::one_way(self : Collider3D) -> (@core.Real, Int)? {
  if self.one_way_direction == 0 || self.one_way_offset is None {
    None
  } else if self.one_way_offset is Some(offset) {
    Some((offset, self.one_way_direction))
  } else {
    None
  }
}

///|
pub fn Collider3D::set_one_way_above(
  self : Collider3D,
  offset : @core.Real,
) -> Unit {
  self.one_way_offset = Some(offset)
  self.one_way_direction = 1
}

///|
pub fn Collider3D::set_one_way_below(
  self : Collider3D,
  offset : @core.Real,
) -> Unit {
  self.one_way_offset = Some(offset)
  self.one_way_direction = -1
}

///|
pub fn Collider3D::clear_one_way(self : Collider3D) -> Unit {
  self.one_way_offset = None
  self.one_way_direction = 0
}

///|
pub fn Collider3D::active_events(self : Collider3D) -> ActiveEvents {
  self.active_events
}

///|
pub fn Collider3D::collision_groups(
  self : Collider3D,
) -> @dynamics.InteractionGroups {
  self.collision_groups
}

///|
pub fn Collider3D::friction(self : Collider3D) -> @core.Real {
  self.friction
}

///|
pub fn Collider3D::restitution(self : Collider3D) -> @core.Real {
  self.restitution
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
  mut local_position : @core.Isometry3
  parent : @dynamics.RigidBodyHandle?
  shape : Shape3D
  mut enabled : Bool
  mut sensor : Bool
  mut surface_velocity : @core.Vec3
  mut one_way_offset : @core.Real?
  mut one_way_direction : Int
  mut active_events : ActiveEvents
  mut collision_groups : @dynamics.InteractionGroups
  mut friction : @core.Real
  mut restitution : @core.Real
}

///|
pub fn ColliderBuilder3D::ball(radius : @core.Real) -> ColliderBuilder3D {
  {
    position: @core.Isometry3::identity(),
    local_position: @core.Isometry3::identity(),
    parent: None,
    shape: Shape3D::ball(radius),
    enabled: true,
    sensor: false,
    surface_velocity: @core.Vec3::zero(),
    one_way_offset: None,
    one_way_direction: 0,
    active_events: ActiveEvents::empty(),
    collision_groups: @dynamics.InteractionGroups::all(),
    friction: 0.7F,
    restitution: 0.0F,
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
    local_position: @core.Isometry3::identity(),
    parent: None,
    shape: Shape3D::cuboid(hx, hy, hz),
    enabled: true,
    sensor: false,
    surface_velocity: @core.Vec3::zero(),
    one_way_offset: None,
    one_way_direction: 0,
    active_events: ActiveEvents::empty(),
    collision_groups: @dynamics.InteractionGroups::all(),
    friction: 0.7F,
    restitution: 0.0F,
  }
}

///|
pub fn ColliderBuilder3D::capsule_y(
  radius : @core.Real,
  half_height : @core.Real,
) -> ColliderBuilder3D {
  {
    position: @core.Isometry3::identity(),
    local_position: @core.Isometry3::identity(),
    parent: None,
    shape: Shape3D::capsule_y(radius, half_height),
    enabled: true,
    sensor: false,
    surface_velocity: @core.Vec3::zero(),
    one_way_offset: None,
    one_way_direction: 0,
    active_events: ActiveEvents::empty(),
    collision_groups: @dynamics.InteractionGroups::all(),
    friction: 0.7F,
    restitution: 0.0F,
  }
}

///|
pub fn ColliderBuilder3D::capsule_x(
  radius : @core.Real,
  half_height : @core.Real,
) -> ColliderBuilder3D {
  ColliderBuilder3D::capsule_y(radius, half_height).rotation(
    @core.rotation_from_scaled_axis(@core.Vec3::new(0.0F, 0.0F, HALF_PI_3D)),
  )
}

///|
pub fn ColliderBuilder3D::cylinder(
  radius : @core.Real,
  half_height : @core.Real,
) -> ColliderBuilder3D {
  {
    position: @core.Isometry3::identity(),
    local_position: @core.Isometry3::identity(),
    parent: None,
    shape: Shape3D::cylinder(radius, half_height),
    enabled: true,
    sensor: false,
    surface_velocity: @core.Vec3::zero(),
    one_way_offset: None,
    one_way_direction: 0,
    active_events: ActiveEvents::empty(),
    collision_groups: @dynamics.InteractionGroups::all(),
    friction: 0.7F,
    restitution: 0.0F,
  }
}

///|
pub fn ColliderBuilder3D::round_cylinder(
  radius : @core.Real,
  half_height : @core.Real,
  border_radius : @core.Real,
) -> ColliderBuilder3D {
  {
    position: @core.Isometry3::identity(),
    local_position: @core.Isometry3::identity(),
    parent: None,
    shape: Shape3D::round_cylinder(radius, half_height, border_radius),
    enabled: true,
    sensor: false,
    surface_velocity: @core.Vec3::zero(),
    one_way_offset: None,
    one_way_direction: 0,
    active_events: ActiveEvents::empty(),
    collision_groups: @dynamics.InteractionGroups::all(),
    friction: 0.7F,
    restitution: 0.0F,
  }
}

///|
pub fn ColliderBuilder3D::cone(
  radius : @core.Real,
  half_height : @core.Real,
) -> ColliderBuilder3D {
  {
    position: @core.Isometry3::identity(),
    local_position: @core.Isometry3::identity(),
    parent: None,
    shape: Shape3D::cone(radius, half_height),
    enabled: true,
    sensor: false,
    surface_velocity: @core.Vec3::zero(),
    one_way_offset: None,
    one_way_direction: 0,
    active_events: ActiveEvents::empty(),
    collision_groups: @dynamics.InteractionGroups::all(),
    friction: 0.7F,
    restitution: 0.0F,
  }
}

///|
pub fn ColliderBuilder3D::trimesh(
  vertices : Array[@core.Vec3],
  indices : Array[(Int, Int, Int)],
) -> ColliderBuilder3D? {
  if vertices.length() == 0 || indices.length() == 0 {
    return None
  }
  let valid : Array[(Int, Int, Int)] = []
  for i in 0..<indices.length() {
    let (i0, i1, i2) = indices[i]
    if i0 < 0 || i1 < 0 || i2 < 0 {
      continue
    }
    if i0 >= vertices.length() ||
      i1 >= vertices.length() ||
      i2 >= vertices.length() {
      continue
    }
    if i0 == i1 || i1 == i2 || i2 == i0 {
      continue
    }
    let a = vertices[i0]
    let b = vertices[i1]
    let c = vertices[i2]
    let area2 = b.sub(a).cross(c.sub(a)).length_squared()
    if area2 <= 1.0e-12F {
      continue
    }
    valid.push((i0, i1, i2))
  }
  if valid.length() == 0 {
    return None
  }
  Some({
    position: @core.Isometry3::identity(),
    local_position: @core.Isometry3::identity(),
    parent: None,
    shape: Shape3D::TriMesh(vertices, valid),
    enabled: true,
    sensor: false,
    surface_velocity: @core.Vec3::zero(),
    one_way_offset: None,
    one_way_direction: 0,
    active_events: ActiveEvents::empty(),
    collision_groups: @dynamics.InteractionGroups::all(),
    friction: 0.7F,
    restitution: 0.0F,
  })
}

///|
pub fn ColliderBuilder3D::heightfield(
  heights : Array[@core.Real],
  nrows : Int,
  ncols : Int,
  scale : @core.Vec3,
) -> ColliderBuilder3D? {
  if nrows <= 1 || ncols <= 1 {
    return None
  }
  if heights.length() != nrows * ncols {
    return None
  }
  let dx = scale.x / Float::from_int(nrows - 1)
  let dz = scale.z / Float::from_int(ncols - 1)
  let origin_x = -scale.x / 2.0F
  let origin_z = -scale.z / 2.0F
  let vertices : Array[@core.Vec3] = []
  for i in 0..<nrows {
    for j in 0..<ncols {
      let x = origin_x + Float::from_int(i) * dx
      let z = origin_z + Float::from_int(j) * dz
      let y = heights[i * ncols + j] * scale.y
      vertices.push(@core.Vec3::new(x, y, z))
    }
  }
  fn vid(i : Int, j : Int, ncols : Int) -> Int {
    i * ncols + j
  }

  let indices : Array[(Int, Int, Int)] = []
  for i in 0..<(nrows - 1) {
    for j in 0..<(ncols - 1) {
      let a = vid(i, j, ncols)
      let b = vid(i + 1, j, ncols)
      let c = vid(i + 1, j + 1, ncols)
      let d = vid(i, j + 1, ncols)
      indices.push((a, c, b))
      indices.push((a, d, c))
    }
  }
  ColliderBuilder3D::trimesh(vertices, indices)
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
pub fn ColliderBuilder3D::local_position(
  self : ColliderBuilder3D,
  pos : @core.Isometry3,
) -> ColliderBuilder3D {
  self.local_position = pos
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
pub fn ColliderBuilder3D::sensor(
  self : ColliderBuilder3D,
  sensor : Bool,
) -> ColliderBuilder3D {
  self.sensor = sensor
  self
}

///|
pub fn ColliderBuilder3D::surface_velocity(
  self : ColliderBuilder3D,
  v : @core.Vec3,
) -> ColliderBuilder3D {
  self.surface_velocity = v
  self
}

///|
pub fn ColliderBuilder3D::one_way_above(
  self : ColliderBuilder3D,
  offset : @core.Real,
) -> ColliderBuilder3D {
  self.one_way_offset = Some(offset)
  self.one_way_direction = 1
  self
}

///|
pub fn ColliderBuilder3D::one_way_below(
  self : ColliderBuilder3D,
  offset : @core.Real,
) -> ColliderBuilder3D {
  self.one_way_offset = Some(offset)
  self.one_way_direction = -1
  self
}

///|
pub fn ColliderBuilder3D::active_events(
  self : ColliderBuilder3D,
  active_events : ActiveEvents,
) -> ColliderBuilder3D {
  self.active_events = active_events
  self
}

///|
pub fn ColliderBuilder3D::density(
  self : ColliderBuilder3D,
  density : @core.Real,
) -> ColliderBuilder3D {
  density |> ignore
  self
}

///|
pub fn ColliderBuilder3D::collision_groups(
  self : ColliderBuilder3D,
  groups : @dynamics.InteractionGroups,
) -> ColliderBuilder3D {
  self.collision_groups = groups
  self
}

///|
pub fn ColliderBuilder3D::friction(
  self : ColliderBuilder3D,
  friction : @core.Real,
) -> ColliderBuilder3D {
  self.friction = friction
  self
}

///|
pub fn ColliderBuilder3D::restitution(
  self : ColliderBuilder3D,
  restitution : @core.Real,
) -> ColliderBuilder3D {
  self.restitution = restitution
  self
}

///|
pub fn ColliderBuilder3D::build(self : ColliderBuilder3D) -> Collider3D {
  {
    position: self.position,
    local_position: self.local_position,
    parent: self.parent,
    shape: self.shape,
    enabled: self.enabled,
    sensor: self.sensor,
    surface_velocity: self.surface_velocity,
    one_way_offset: self.one_way_offset,
    one_way_direction: self.one_way_direction,
    active_events: self.active_events,
    collision_groups: self.collision_groups,
    friction: self.friction,
    restitution: self.restitution,
  }
}
