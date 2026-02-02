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
  // Convex hull represented by its support points (may include interior points).
  // The optional border radius represents the Minkowski sum with a ball.
  ConvexHull(Array[@core.Vec3], @core.Real)
  // A compound made of a list of (local pose, shape) pairs.
  Compound(Array[(@core.Isometry3, Shape3D)])
  // Heightfield stored as a triangle mesh plus grid dimensions (rows/cols) for future optimizations.
  Heightfield(Array[@core.Vec3], Array[(Int, Int, Int)], Int, Int)
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
    Shape3D::ConvexHull(points, br) =>
      if points.length() == 0 {
        @core.Aabb3::new(@core.Vec3::zero(), @core.Vec3::zero())
      } else {
        let mut min = points[0]
        let mut max = points[0]
        for i in 1..<points.length() {
          let p = points[i]
          if p.x < min.x {
            min = @core.Vec3::new(p.x, min.y, min.z)
          }
          if p.y < min.y {
            min = @core.Vec3::new(min.x, p.y, min.z)
          }
          if p.z < min.z {
            min = @core.Vec3::new(min.x, min.y, p.z)
          }
          if p.x > max.x {
            max = @core.Vec3::new(p.x, max.y, max.z)
          }
          if p.y > max.y {
            max = @core.Vec3::new(max.x, p.y, max.z)
          }
          if p.z > max.z {
            max = @core.Vec3::new(max.x, max.y, p.z)
          }
        }
        let pad = @core.Vec3::new(br, br, br)
        @core.Aabb3::new(min.sub(pad), max.add(pad))
      }
    Shape3D::Compound(parts) =>
      if parts.length() == 0 {
        @core.Aabb3::new(@core.Vec3::zero(), @core.Vec3::zero())
      } else {
        // Union all child AABBs transformed into the compound's local space.
        let mut aabb = aabb3_transform(parts[0].0, parts[0].1.local_aabb())
        for i in 1..<parts.length() {
          let (iso, shape) = parts[i]
          let child = aabb3_transform(iso, shape.local_aabb())
          aabb = aabb.combine(child)
        }
        aabb
      }
    Shape3D::Heightfield(vertices, _, _, _) =>
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
  // Optional per-triangle metadata for voxel meshes created with `voxels_from_points`.
  voxel_tri_map : Array[(Int, Int, Int)]?
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
pub fn Collider3D::voxel_key_for_triangle(
  self : Collider3D,
  triangle_index : Int,
) -> (Int, Int, Int)? {
  if self.voxel_tri_map is Some(map) {
    if triangle_index >= 0 && triangle_index < map.length() {
      Some(map[triangle_index])
    } else {
      None
    }
  } else {
    None
  }
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
  voxel_tri_map : Array[(Int, Int, Int)]?
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
    voxel_tri_map: None,
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
    voxel_tri_map: None,
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
    voxel_tri_map: None,
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
    voxel_tri_map: None,
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
    voxel_tri_map: None,
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
    voxel_tri_map: None,
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
    voxel_tri_map: None,
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
/// Build a compound collider from multiple (local pose, child shape) pairs.
///
/// This matches Rapier's `ColliderBuilder::compound` pattern: the compound has a single set of
/// material/flags, while each child contributes its local geometry.
///
/// Note: we accept child shapes as `ColliderBuilder3D` so callers can reuse existing constructor
/// helpers (`ball`, `cuboid`, ...). Only the child's `shape` and `position` are used; all other
/// fields (friction, events, etc.) are ignored.
pub fn ColliderBuilder3D::compound(
  parts : Array[(@core.Isometry3, ColliderBuilder3D)],
) -> ColliderBuilder3D? {
  if parts.length() == 0 {
    return None
  }
  let shapes : Array[(@core.Isometry3, Shape3D)] = []
  for i in 0..<parts.length() {
    let (iso, part) = parts[i]
    shapes.push((iso.mul(part.position), part.shape))
  }
  Some({
    position: @core.Isometry3::identity(),
    local_position: @core.Isometry3::identity(),
    parent: None,
    shape: Shape3D::Compound(shapes),
    voxel_tri_map: None,
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
/// Build a convex hull collider from an arbitrary point cloud.
///
/// The hull is represented by its support points (the input points); interior points are allowed
/// and do not affect correctness of support-based collision detection.
pub fn ColliderBuilder3D::round_convex_hull(
  points : Array[@core.Vec3],
  border_radius : @core.Real,
) -> ColliderBuilder3D? {
  if points.length() == 0 {
    return None
  }
  let pts : Array[@core.Vec3] = []
  for i in 0..<points.length() {
    pts.push(points[i])
  }
  Some({
    position: @core.Isometry3::identity(),
    local_position: @core.Isometry3::identity(),
    parent: None,
    shape: Shape3D::ConvexHull(pts, border_radius),
    voxel_tri_map: None,
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
pub fn ColliderBuilder3D::convex_hull(
  points : Array[@core.Vec3],
) -> ColliderBuilder3D? {
  ColliderBuilder3D::round_convex_hull(points, 0.0F)
}

///|
pub fn ColliderBuilder3D::convex_decomposition(
  vertices : Array[@core.Vec3],
  indices : Array[(Int, Int, Int)],
) -> ColliderBuilder3D? {
  ColliderBuilder3D::round_convex_decomposition(vertices, indices, 0.0F)
}

///|
pub fn ColliderBuilder3D::round_convex_decomposition(
  vertices : Array[@core.Vec3],
  indices : Array[(Int, Int, Int)],
  border_radius : @core.Real,
) -> ColliderBuilder3D? {
  if vertices.length() == 0 {
    return None
  }
  if indices.length() == 0 {
    return ColliderBuilder3D::round_convex_hull(vertices, border_radius)
  }
  // Very small, deterministic decomposition: split triangles along the dominant axis of triangle
  // centroids and build one convex part per chunk.
  let tris = indices
  let mut min = vertices[0]
  let mut max = vertices[0]
  for i in 1..<vertices.length() {
    let p = vertices[i]
    if p.x < min.x {
      min = @core.Vec3::new(p.x, min.y, min.z)
    }
    if p.y < min.y {
      min = @core.Vec3::new(min.x, p.y, min.z)
    }
    if p.z < min.z {
      min = @core.Vec3::new(min.x, min.y, p.z)
    }
    if p.x > max.x {
      max = @core.Vec3::new(p.x, max.y, max.z)
    }
    if p.y > max.y {
      max = @core.Vec3::new(max.x, p.y, max.z)
    }
    if p.z > max.z {
      max = @core.Vec3::new(max.x, max.y, p.z)
    }
  }
  let ext = max.sub(min)
  let axis_i = if ext.x >= ext.y && ext.x >= ext.z {
    0
  } else if ext.y >= ext.z {
    1
  } else {
    2
  }
  fn axis_value(v : @core.Vec3, axis_i : Int) -> @core.Real {
    if axis_i == 0 {
      v.x
    } else if axis_i == 1 {
      v.y
    } else {
      v.z
    }
  }

  let tri_ids : Array[Int] = []
  for i in 0..<tris.length() {
    tri_ids.push(i)
  }
  // Sort by centroid coordinate on the chosen axis.
  for i in 1..<tri_ids.length() {
    let key = tri_ids[i]
    let (a0, a1, a2) = tris[key]
    let ca = vertices[a0].add(vertices[a1]).add(vertices[a2]).scale(1.0F / 3.0F)
    let key_val = axis_value(ca, axis_i)
    let mut j = i - 1
    while j >= 0 {
      let idj = tri_ids[j]
      let (b0, b1, b2) = tris[idj]
      let cb = vertices[b0]
        .add(vertices[b1])
        .add(vertices[b2])
        .scale(1.0F / 3.0F)
      let vj = axis_value(cb, axis_i)
      if key_val >= vj {
        break
      }
      tri_ids[j + 1] = tri_ids[j]
      j = j - 1
    }
    tri_ids[j + 1] = key
  }
  let tri_count = tri_ids.length()
  let num_parts = if tri_count >= 24 {
    4
  } else if tri_count >= 12 {
    2
  } else {
    1
  }
  if num_parts <= 1 {
    return ColliderBuilder3D::round_convex_hull(vertices, border_radius)
  }
  let parts : Array[(@core.Isometry3, Shape3D)] = []
  let chunk = (tri_count + num_parts - 1) / num_parts
  for p in 0..<num_parts {
    let start = p * chunk
    if start >= tri_count {
      continue
    }
    let end = if start + chunk < tri_count { start + chunk } else { tri_count }
    let used : @hashset.HashSet[Int] = @hashset.new(capacity=chunk * 3)
    for ii in start..<end {
      let tid = tri_ids[ii]
      let (i0, i1, i2) = tris[tid]
      used.add(i0)
      used.add(i1)
      used.add(i2)
    }
    let pts : Array[@core.Vec3] = []
    for vid in used.to_array() {
      pts.push(vertices[vid])
    }
    if pts.length() > 0 {
      parts.push(
        (@core.Isometry3::identity(), Shape3D::ConvexHull(pts, border_radius)),
      )
    }
  }
  if parts.length() <= 1 {
    ColliderBuilder3D::round_convex_hull(vertices, border_radius)
  } else {
    Some({
      position: @core.Isometry3::identity(),
      local_position: @core.Isometry3::identity(),
      parent: None,
      shape: Shape3D::Compound(parts),
      voxel_tri_map: None,
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
}

///|
/// Build a voxel mesh from a point cloud by snapping points to a regular grid.
///
/// This is a minimal dim3 counterpart of Rapier/Parry's `voxels_from_points`. It outputs a single
/// trimesh representing the surface of all occupied voxels.
pub fn ColliderBuilder3D::voxels_from_points(
  voxel_size : @core.Vec3,
  points : Array[@core.Vec3],
) -> ColliderBuilder3D? {
  if points.length() == 0 {
    return None
  }
  if voxel_size.x == 0.0F || voxel_size.y == 0.0F || voxel_size.z == 0.0F {
    return None
  }
  fn floor_div_to_int(value : @core.Real, denom : @core.Real) -> Int {
    if denom == 0.0F {
      0
    } else {
      @math.floor((value / denom).to_double()).to_int()
    }
  }

  let occupied : @hashset.HashSet[(Int, Int, Int)] = @hashset.new(
    capacity=points.length(),
  )
  for i in 0..<points.length() {
    let p = points[i]
    let ix = floor_div_to_int(p.x, voxel_size.x)
    let iy = floor_div_to_int(p.y, voxel_size.y)
    let iz = floor_div_to_int(p.z, voxel_size.z)
    occupied.add((ix, iy, iz))
  }
  if occupied.is_empty() {
    return None
  }
  let voxels = occupied.to_array()
  // Deterministic order for stable triangle->voxel metadata.
  fn less(a : (Int, Int, Int), b : (Int, Int, Int)) -> Bool {
    if a.0 != b.0 {
      a.0 < b.0
    } else if a.1 != b.1 {
      a.1 < b.1
    } else {
      a.2 < b.2
    }
  }

  for i in 1..<voxels.length() {
    let key = voxels[i]
    let mut j = i - 1
    while j >= 0 && less(key, voxels[j]) {
      voxels[j + 1] = voxels[j]
      j = j - 1
    }
    voxels[j + 1] = key
  }
  let vertices : Array[@core.Vec3] = []
  let indices : Array[(Int, Int, Int)] = []
  let tri_voxel : Array[(Int, Int, Int)] = []
  fn push_face(
    vertices : Array[@core.Vec3],
    indices : Array[(Int, Int, Int)],
    tri_voxel : Array[(Int, Int, Int)],
    vkey : (Int, Int, Int),
    a : @core.Vec3,
    b : @core.Vec3,
    c : @core.Vec3,
    d : @core.Vec3,
  ) -> Unit {
    let base = vertices.length()
    vertices.push(a)
    vertices.push(b)
    vertices.push(c)
    vertices.push(d)
    indices.push((base, base + 1, base + 2))
    indices.push((base, base + 2, base + 3))
    tri_voxel.push(vkey)
    tri_voxel.push(vkey)
  }

  for i in 0..<voxels.length() {
    let (ix, iy, iz) = voxels[i]
    let x0 = Float::from_int(ix) * voxel_size.x
    let y0 = Float::from_int(iy) * voxel_size.y
    let z0 = Float::from_int(iz) * voxel_size.z
    let x1 = x0 + voxel_size.x
    let y1 = y0 + voxel_size.y
    let z1 = z0 + voxel_size.z

    // +X
    if !occupied.contains((ix + 1, iy, iz)) {
      push_face(
        vertices,
        indices,
        tri_voxel,
        (ix, iy, iz),
        @core.Vec3::new(x1, y0, z0),
        @core.Vec3::new(x1, y1, z0),
        @core.Vec3::new(x1, y1, z1),
        @core.Vec3::new(x1, y0, z1),
      )
    }
    // -X
    if !occupied.contains((ix - 1, iy, iz)) {
      push_face(
        vertices,
        indices,
        tri_voxel,
        (ix, iy, iz),
        @core.Vec3::new(x0, y0, z1),
        @core.Vec3::new(x0, y1, z1),
        @core.Vec3::new(x0, y1, z0),
        @core.Vec3::new(x0, y0, z0),
      )
    }
    // +Y
    if !occupied.contains((ix, iy + 1, iz)) {
      push_face(
        vertices,
        indices,
        tri_voxel,
        (ix, iy, iz),
        @core.Vec3::new(x0, y1, z0),
        @core.Vec3::new(x0, y1, z1),
        @core.Vec3::new(x1, y1, z1),
        @core.Vec3::new(x1, y1, z0),
      )
    }
    // -Y
    if !occupied.contains((ix, iy - 1, iz)) {
      push_face(
        vertices,
        indices,
        tri_voxel,
        (ix, iy, iz),
        @core.Vec3::new(x0, y0, z1),
        @core.Vec3::new(x0, y0, z0),
        @core.Vec3::new(x1, y0, z0),
        @core.Vec3::new(x1, y0, z1),
      )
    }
    // +Z
    if !occupied.contains((ix, iy, iz + 1)) {
      push_face(
        vertices,
        indices,
        tri_voxel,
        (ix, iy, iz),
        @core.Vec3::new(x1, y0, z1),
        @core.Vec3::new(x1, y1, z1),
        @core.Vec3::new(x0, y1, z1),
        @core.Vec3::new(x0, y0, z1),
      )
    }
    // -Z
    if !occupied.contains((ix, iy, iz - 1)) {
      push_face(
        vertices,
        indices,
        tri_voxel,
        (ix, iy, iz),
        @core.Vec3::new(x0, y0, z0),
        @core.Vec3::new(x0, y1, z0),
        @core.Vec3::new(x1, y1, z0),
        @core.Vec3::new(x1, y0, z0),
      )
    }
  }
  // Build the trimesh while filtering invalid triangles, keeping metadata aligned.
  if vertices.length() == 0 || indices.length() == 0 {
    return None
  }
  let valid : Array[(Int, Int, Int)] = []
  let valid_map : Array[(Int, Int, Int)] = []
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
    valid_map.push(tri_voxel[i])
  }
  if valid.length() == 0 {
    return None
  }
  Some({
    position: @core.Isometry3::identity(),
    local_position: @core.Isometry3::identity(),
    parent: None,
    shape: Shape3D::TriMesh(vertices, valid),
    voxel_tri_map: Some(valid_map),
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
  if ColliderBuilder3D::trimesh(vertices, indices) is Some(builder) {
    match builder.shape {
      Shape3D::TriMesh(vtx, idx) =>
        Some({
          position: @core.Isometry3::identity(),
          local_position: @core.Isometry3::identity(),
          parent: None,
          shape: Shape3D::Heightfield(vtx, idx, nrows, ncols),
          voxel_tri_map: None,
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
      _ => None
    }
  } else {
    None
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
    voxel_tri_map: self.voxel_tri_map,
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
