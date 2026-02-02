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
pub type Real = Float

///|
const EPSILON : Real = 1.0e-6F

///|
pub fn sin(value : Real) -> Real {
  Float::from_double(@math.sin(value.to_double()))
}

///|
pub fn cos(value : Real) -> Real {
  Float::from_double(@math.cos(value.to_double()))
}

///|
pub fn atan2(y : Real, x : Real) -> Real {
  Float::from_double(@math.atan2(y.to_double(), x.to_double()))
}

///|
fn sqrt(value : Real) -> Real {
  if value <= 0.0F {
    0.0F
  } else {
    let mut x = value
    for _ in 0..<8 {
      x = 0.5F * (x + value / x)
    }
    x
  }
}

///|
fn pi_value() -> Real {
  Float::from_double(@math.PI)
}

///|
fn min(a : Real, b : Real) -> Real {
  if a < b {
    a
  } else {
    b
  }
}

///|
fn max(a : Real, b : Real) -> Real {
  if a > b {
    a
  } else {
    b
  }
}

///|
pub struct Vec2 {
  x : Real
  y : Real
}

///|
pub fn Vec2::new(x : Real, y : Real) -> Vec2 {
  Vec2::{ x, y }
}

///|
pub fn Vec2::zero() -> Vec2 {
  Vec2::new(0.0F, 0.0F)
}

///|
pub fn Vec2::add(self : Vec2, other : Vec2) -> Vec2 {
  Vec2::new(self.x + other.x, self.y + other.y)
}

///|
pub fn Vec2::sub(self : Vec2, other : Vec2) -> Vec2 {
  Vec2::new(self.x - other.x, self.y - other.y)
}

///|
pub fn Vec2::dot(self : Vec2, other : Vec2) -> Real {
  self.x * other.x + self.y * other.y
}

///|
pub fn Vec2::cross(self : Vec2, other : Vec2) -> Real {
  self.x * other.y - self.y * other.x
}

///|
pub fn Vec2::length_squared(self : Vec2) -> Real {
  self.dot(self)
}

///|
pub fn Vec2::length(self : Vec2) -> Real {
  sqrt(self.length_squared())
}

///|
pub fn Vec2::normalize(self : Vec2) -> Vec2 {
  let len = self.length()
  if len <= EPSILON {
    Vec2::zero()
  } else {
    Vec2::new(self.x / len, self.y / len)
  }
}

///|
pub fn pi() -> Real {
  pi_value()
}

///|
pub fn two_pi() -> Real {
  pi_value() * 2.0F
}

///|
pub fn abs(value : Real) -> Real {
  if value < 0.0F {
    -value
  } else {
    value
  }
}

///|
pub fn asin(value : Real) -> Real {
  Float::from_double(@math.asin(value.to_double()))
}

///|
pub fn acos(value : Real) -> Real {
  Float::from_double(@math.acos(value.to_double()))
}

///|
pub fn clamp(value : Real, min_value : Real, max_value : Real) -> Real {
  if value < min_value {
    min_value
  } else if value > max_value {
    max_value
  } else {
    value
  }
}

///|
pub struct Rot2 {
  sin : Real
  cos : Real
}

///|
pub fn Rot2::identity() -> Rot2 {
  Rot2::{ sin: 0.0F, cos: 1.0F }
}

///|
pub fn Rot2::from_angle(angle : Real) -> Rot2 {
  Rot2::{ sin: sin(angle), cos: cos(angle) }
}

///|
pub fn Rot2::angle(self : Rot2) -> Real {
  atan2(self.sin, self.cos)
}

///|
pub fn Rot2::rotate_vec2(self : Rot2, v : Vec2) -> Vec2 {
  Vec2::new(self.cos * v.x - self.sin * v.y, self.sin * v.x + self.cos * v.y)
}

///|
pub fn Rot2::mul(self : Rot2, other : Rot2) -> Rot2 {
  Rot2::{
    sin: self.sin * other.cos + self.cos * other.sin,
    cos: self.cos * other.cos - self.sin * other.sin,
  }
}

///|
pub fn Rot2::inverse(self : Rot2) -> Rot2 {
  Rot2::{ sin: -self.sin, cos: self.cos }
}

///|
pub fn rotation_from_angle(angle : Real) -> Rot2 {
  Rot2::from_angle(angle)
}

///|
pub struct Isometry2 {
  translation : Vec2
  rotation : Rot2
}

///|
pub fn Isometry2::identity() -> Isometry2 {
  Isometry2::{ translation: Vec2::zero(), rotation: Rot2::identity() }
}

///|
pub fn Isometry2::new(translation : Vec2, rotation : Rot2) -> Isometry2 {
  Isometry2::{ translation, rotation }
}

///|
pub fn Isometry2::from_translation(translation : Vec2) -> Isometry2 {
  Isometry2::new(translation, Rot2::identity())
}

///|
pub fn Isometry2::mul(self : Isometry2, other : Isometry2) -> Isometry2 {
  let rotated = self.rotation.rotate_vec2(other.translation)
  Isometry2::new(
    self.translation.add(rotated),
    self.rotation.mul(other.rotation),
  )
}

///|
pub fn Isometry2::inverse(self : Isometry2) -> Isometry2 {
  let inv_rot = self.rotation.inverse()
  let neg_translation = Vec2::new(-self.translation.x, -self.translation.y)
  let inv_translation = inv_rot.rotate_vec2(neg_translation)
  Isometry2::new(inv_translation, inv_rot)
}

///|
pub fn Isometry2::transform_point(self : Isometry2, point : Vec2) -> Vec2 {
  self.rotation.rotate_vec2(point).add(self.translation)
}

///|
pub struct Aabb {
  mins : Vec2
  maxs : Vec2
}

///|
pub fn Aabb::new(mins : Vec2, maxs : Vec2) -> Aabb {
  Aabb::{ mins, maxs }
}

///|
pub fn Aabb::from_points(a : Vec2, b : Vec2) -> Aabb {
  Aabb::new(
    Vec2::new(min(a.x, b.x), min(a.y, b.y)),
    Vec2::new(max(a.x, b.x), max(a.y, b.y)),
  )
}

///|
pub fn Aabb::combine(self : Aabb, other : Aabb) -> Aabb {
  Aabb::new(
    Vec2::new(min(self.mins.x, other.mins.x), min(self.mins.y, other.mins.y)),
    Vec2::new(max(self.maxs.x, other.maxs.x), max(self.maxs.y, other.maxs.y)),
  )
}

///|
/// 3D vector. This is the dim3 counterpart of `Vec2`.
pub struct Vec3 {
  x : Real
  y : Real
  z : Real
}

///|
pub fn Vec3::new(x : Real, y : Real, z : Real) -> Vec3 {
  Vec3::{ x, y, z }
}

///|
pub fn Vec3::zero() -> Vec3 {
  Vec3::new(0.0F, 0.0F, 0.0F)
}

///|
pub fn Vec3::add(self : Vec3, other : Vec3) -> Vec3 {
  Vec3::new(self.x + other.x, self.y + other.y, self.z + other.z)
}

///|
pub fn Vec3::sub(self : Vec3, other : Vec3) -> Vec3 {
  Vec3::new(self.x - other.x, self.y - other.y, self.z - other.z)
}

///|
pub fn Vec3::scale(self : Vec3, s : Real) -> Vec3 {
  Vec3::new(self.x * s, self.y * s, self.z * s)
}

///|
pub fn Vec3::dot(self : Vec3, other : Vec3) -> Real {
  self.x * other.x + self.y * other.y + self.z * other.z
}

///|
pub fn Vec3::cross(self : Vec3, other : Vec3) -> Vec3 {
  Vec3::new(
    self.y * other.z - self.z * other.y,
    self.z * other.x - self.x * other.z,
    self.x * other.y - self.y * other.x,
  )
}

///|
pub fn Vec3::length_squared(self : Vec3) -> Real {
  self.dot(self)
}

///|
pub fn Vec3::length(self : Vec3) -> Real {
  sqrt(self.length_squared())
}

///|
pub fn Vec3::normalize(self : Vec3) -> Vec3 {
  let len = self.length()
  if len <= EPSILON {
    Vec3::zero()
  } else {
    self.scale(1.0F / len)
  }
}

///|
/// Unit quaternion used as 3D rotation.
pub struct Quat {
  x : Real
  y : Real
  z : Real
  w : Real
}

///|
pub fn Quat::imag(self : Quat) -> Vec3 {
  Vec3::new(self.x, self.y, self.z)
}

///|
pub fn Quat::real(self : Quat) -> Real {
  self.w
}

///|
pub fn Quat::negated(self : Quat) -> Quat {
  Quat::new(-self.x, -self.y, -self.z, -self.w)
}

///|
pub fn Quat::new(x : Real, y : Real, z : Real, w : Real) -> Quat {
  Quat::{ x, y, z, w }
}

///|
pub fn Quat::identity() -> Quat {
  Quat::new(0.0F, 0.0F, 0.0F, 1.0F)
}

///|
pub fn Quat::conjugate(self : Quat) -> Quat {
  Quat::new(-self.x, -self.y, -self.z, self.w)
}

///|
pub fn Quat::dot(self : Quat, other : Quat) -> Real {
  self.x * other.x + self.y * other.y + self.z * other.z + self.w * other.w
}

///|
pub fn Quat::norm_squared(self : Quat) -> Real {
  self.x * self.x + self.y * self.y + self.z * self.z + self.w * self.w
}

///|
pub fn Quat::normalize(self : Quat) -> Quat {
  let n2 = self.norm_squared()
  if n2 <= EPSILON {
    Quat::identity()
  } else {
    let inv_n = 1.0F / sqrt(n2)
    Quat::new(self.x * inv_n, self.y * inv_n, self.z * inv_n, self.w * inv_n)
  }
}

///|
/// Hamilton product (applies `other` then `self` when used as rotation composition).
pub fn Quat::mul(self : Quat, other : Quat) -> Quat {
  Quat::new(
    self.w * other.x + self.x * other.w + self.y * other.z - self.z * other.y,
    self.w * other.y - self.x * other.z + self.y * other.w + self.z * other.x,
    self.w * other.z + self.x * other.y - self.y * other.x + self.z * other.w,
    self.w * other.w - self.x * other.x - self.y * other.y - self.z * other.z,
  )
}

///|
pub fn Quat::inverse(self : Quat) -> Quat {
  self.conjugate().normalize()
}

///|
/// Construct a rotation from a scaled axis (axis * angle).
pub fn rotation_from_scaled_axis(axis_angle : Vec3) -> Quat {
  let angle = axis_angle.length()
  if angle <= EPSILON {
    Quat::identity()
  } else {
    let half = 0.5F * angle
    let axis = axis_angle.scale(1.0F / angle)
    let s = sin(half)
    Quat::new(axis.x * s, axis.y * s, axis.z * s, cos(half)).normalize()
  }
}

///|
/// Rotate a vector by this quaternion.
pub fn Quat::rotate_vec3(self : Quat, v : Vec3) -> Vec3 {
  // For unit quaternion q = (u, s), v' = 2*dot(u,v)*u + (s^2 - dot(u,u))*v + 2*s*cross(u,v).
  let q = self.normalize()
  let u = Vec3::new(q.x, q.y, q.z)
  let s = q.w
  let uv = u.dot(v)
  let uu = u.dot(u)
  u.scale(2.0F * uv).add(v.scale(s * s - uu)).add(u.cross(v).scale(2.0F * s))
}

///|
/// Convert this quaternion to a scaled axis representation (axis * angle).
/// This always returns the shortest representation (angle in [0, pi]).
pub fn Quat::to_scaled_axis(self : Quat) -> Vec3 {
  let mut q = self.normalize()
  if q.w < 0.0F {
    q = Quat::new(-q.x, -q.y, -q.z, -q.w)
  }
  let w = clamp(q.w, -1.0F, 1.0F)
  let angle = 2.0F * acos(w)
  let s = sqrt(1.0F - w * w) // sin(angle/2)
  if s <= 1.0e-6F {
    // Small-angle approximation: scaled axis ~= 2 * (x,y,z).
    Vec3::new(q.x * 2.0F, q.y * 2.0F, q.z * 2.0F)
  } else {
    Vec3::new(q.x / s, q.y / s, q.z / s).scale(angle)
  }
}

///|
/// Construct a rotation that maps the direction `from` to the direction `to`.
pub fn rotation_between(from : Vec3, to : Vec3) -> Quat {
  let f = from.normalize()
  let t = to.normalize()
  let d = f.dot(t)

  // Same direction.
  if d > 1.0F - 1.0e-6F {
    return Quat::identity()
  }

  // Opposite directions: pick a deterministic orthogonal axis.
  if d < -1.0F + 1.0e-6F {
    let fallback = if abs(f.x) < 0.9F {
      Vec3::new(1.0F, 0.0F, 0.0F)
    } else {
      Vec3::new(0.0F, 1.0F, 0.0F)
    }
    let axis = f.cross(fallback).normalize()
    return rotation_from_scaled_axis(axis.scale(pi()))
  }
  let c = f.cross(t)
  Quat::new(c.x, c.y, c.z, 1.0F + d).normalize()
}

///|
pub struct Isometry3 {
  translation : Vec3
  rotation : Quat
}

///|
pub fn Isometry3::identity() -> Isometry3 {
  Isometry3::{ translation: Vec3::zero(), rotation: Quat::identity() }
}

///|
pub fn Isometry3::new(translation : Vec3, rotation : Quat) -> Isometry3 {
  Isometry3::{ translation, rotation }
}

///|
pub fn Isometry3::from_translation(translation : Vec3) -> Isometry3 {
  Isometry3::new(translation, Quat::identity())
}

///|
pub fn Isometry3::mul(self : Isometry3, other : Isometry3) -> Isometry3 {
  let rotated = self.rotation.rotate_vec3(other.translation)
  Isometry3::new(
    self.translation.add(rotated),
    self.rotation.mul(other.rotation),
  )
}

///|
pub fn Isometry3::inverse(self : Isometry3) -> Isometry3 {
  let inv_rot = self.rotation.conjugate().normalize()
  let neg_translation = Vec3::new(
    -self.translation.x,
    -self.translation.y,
    -self.translation.z,
  )
  let inv_translation = inv_rot.rotate_vec3(neg_translation)
  Isometry3::new(inv_translation, inv_rot)
}

///|
pub fn Isometry3::transform_point(self : Isometry3, point : Vec3) -> Vec3 {
  self.rotation.rotate_vec3(point).add(self.translation)
}

///|
pub struct Aabb3 {
  mins : Vec3
  maxs : Vec3
}

///|
pub fn Aabb3::new(mins : Vec3, maxs : Vec3) -> Aabb3 {
  Aabb3::{ mins, maxs }
}

///|
pub fn Aabb3::from_points(a : Vec3, b : Vec3) -> Aabb3 {
  Aabb3::new(
    Vec3::new(min(a.x, b.x), min(a.y, b.y), min(a.z, b.z)),
    Vec3::new(max(a.x, b.x), max(a.y, b.y), max(a.z, b.z)),
  )
}

///|
pub fn Aabb3::combine(self : Aabb3, other : Aabb3) -> Aabb3 {
  Aabb3::new(
    Vec3::new(
      min(self.mins.x, other.mins.x),
      min(self.mins.y, other.mins.y),
      min(self.mins.z, other.mins.z),
    ),
    Vec3::new(
      max(self.maxs.x, other.maxs.x),
      max(self.maxs.y, other.maxs.y),
      max(self.maxs.z, other.maxs.z),
    ),
  )
}

///|
pub fn Aabb3::intersects(self : Aabb3, other : Aabb3) -> Bool {
  !(self.maxs.x < other.mins.x ||
  self.mins.x > other.maxs.x ||
  self.maxs.y < other.mins.y ||
  self.mins.y > other.maxs.y ||
  self.maxs.z < other.mins.z ||
  self.mins.z > other.maxs.z)
}

///|
pub fn Aabb3::dilated(self : Aabb3, amount : Real) -> Aabb3 {
  if amount <= 0.0F {
    self
  } else {
    Aabb3::new(
      Vec3::new(
        self.mins.x - amount,
        self.mins.y - amount,
        self.mins.z - amount,
      ),
      Vec3::new(
        self.maxs.x + amount,
        self.maxs.y + amount,
        self.maxs.z + amount,
      ),
    )
  }
}

///|
pub fn Aabb::contains_point(self : Aabb, point : Vec2) -> Bool {
  point.x >= self.mins.x &&
  point.x <= self.maxs.x &&
  point.y >= self.mins.y &&
  point.y <= self.maxs.y
}

///|
pub fn Aabb::intersects(self : Aabb, other : Aabb) -> Bool {
  !(self.maxs.x < other.mins.x ||
  self.mins.x > other.maxs.x ||
  self.maxs.y < other.mins.y ||
  self.mins.y > other.maxs.y)
}

///|
pub struct MassProperties {
  mass : Real
  inv_mass : Real
  inertia : Real
  inv_inertia : Real
  center_of_mass : Vec2
}

///|
pub fn MassProperties::new(
  mass : Real,
  inertia : Real,
  center_of_mass : Vec2,
) -> MassProperties {
  let inv_mass = if mass <= EPSILON { 0.0F } else { 1.0F / mass }
  let inv_inertia = if inertia <= EPSILON { 0.0F } else { 1.0F / inertia }
  MassProperties::{ mass, inv_mass, inertia, inv_inertia, center_of_mass }
}

///|
pub fn MassProperties::default() -> MassProperties {
  MassProperties::new(0.0F, 0.0F, Vec2::zero())
}

///|
pub fn MassProperties::transform_by(
  self : MassProperties,
  transform : Isometry2,
) -> MassProperties {
  // `inertia` is expressed about the center-of-mass, so translation does not affect it.
  MassProperties::new(
    self.mass,
    self.inertia,
    transform.transform_point(self.center_of_mass),
  )
}

///|
pub fn MassProperties::add(
  self : MassProperties,
  other : MassProperties,
) -> MassProperties {
  if self.mass <= EPSILON {
    return other
  }
  if other.mass <= EPSILON {
    return self
  }
  let total_mass = self.mass + other.mass
  let inv_total = 1.0F / total_mass
  let com = Vec2::new(
    (self.center_of_mass.x * self.mass + other.center_of_mass.x * other.mass) *
    inv_total,
    (self.center_of_mass.y * self.mass + other.center_of_mass.y * other.mass) *
    inv_total,
  )
  let dx1 = self.center_of_mass.x - com.x
  let dy1 = self.center_of_mass.y - com.y
  let dx2 = other.center_of_mass.x - com.x
  let dy2 = other.center_of_mass.y - com.y
  let inertia = self.inertia +
    self.mass * (dx1 * dx1 + dy1 * dy1) +
    other.inertia +
    other.mass * (dx2 * dx2 + dy2 * dy2)
  MassProperties::new(total_mass, inertia, com)
}

///|
pub fn MassProperties::set_mass(
  self : MassProperties,
  new_mass : Real,
  update_inertia : Bool,
) -> MassProperties {
  if self.mass <= EPSILON {
    let inertia = if update_inertia { 0.0F } else { self.inertia }
    return MassProperties::new(new_mass, inertia, self.center_of_mass)
  }
  let inertia = if update_inertia {
    self.inertia * (new_mass / self.mass)
  } else {
    self.inertia
  }
  MassProperties::new(new_mass, inertia, self.center_of_mass)
}
