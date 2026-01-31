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
/// 3x3 matrix (row-major).
pub struct Mat3 {
  m00 : Real
  m01 : Real
  m02 : Real
  m10 : Real
  m11 : Real
  m12 : Real
  m20 : Real
  m21 : Real
  m22 : Real
}

///|
pub fn Mat3::new(
  m00 : Real,
  m01 : Real,
  m02 : Real,
  m10 : Real,
  m11 : Real,
  m12 : Real,
  m20 : Real,
  m21 : Real,
  m22 : Real,
) -> Mat3 {
  Mat3::{ m00, m01, m02, m10, m11, m12, m20, m21, m22 }
}

///|
pub fn Mat3::zero() -> Mat3 {
  Mat3::new(0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F)
}

///|
pub fn Mat3::identity() -> Mat3 {
  Mat3::new(1.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 1.0F)
}

///|
pub fn Mat3::from_diagonal(diag : Vec3) -> Mat3 {
  Mat3::new(diag.x, 0.0F, 0.0F, 0.0F, diag.y, 0.0F, 0.0F, 0.0F, diag.z)
}

///|
pub fn Mat3::transpose(self : Mat3) -> Mat3 {
  Mat3::new(
    self.m00,
    self.m10,
    self.m20,
    self.m01,
    self.m11,
    self.m21,
    self.m02,
    self.m12,
    self.m22,
  )
}

///|
pub fn Mat3::add(self : Mat3, other : Mat3) -> Mat3 {
  Mat3::new(
    self.m00 + other.m00,
    self.m01 + other.m01,
    self.m02 + other.m02,
    self.m10 + other.m10,
    self.m11 + other.m11,
    self.m12 + other.m12,
    self.m20 + other.m20,
    self.m21 + other.m21,
    self.m22 + other.m22,
  )
}

///|
pub fn Mat3::sub(self : Mat3, other : Mat3) -> Mat3 {
  Mat3::new(
    self.m00 - other.m00,
    self.m01 - other.m01,
    self.m02 - other.m02,
    self.m10 - other.m10,
    self.m11 - other.m11,
    self.m12 - other.m12,
    self.m20 - other.m20,
    self.m21 - other.m21,
    self.m22 - other.m22,
  )
}

///|
pub fn Mat3::scale(self : Mat3, s : Real) -> Mat3 {
  Mat3::new(
    self.m00 * s,
    self.m01 * s,
    self.m02 * s,
    self.m10 * s,
    self.m11 * s,
    self.m12 * s,
    self.m20 * s,
    self.m21 * s,
    self.m22 * s,
  )
}

///|
pub fn Mat3::mul_vec3(self : Mat3, v : Vec3) -> Vec3 {
  Vec3::new(
    self.m00 * v.x + self.m01 * v.y + self.m02 * v.z,
    self.m10 * v.x + self.m11 * v.y + self.m12 * v.z,
    self.m20 * v.x + self.m21 * v.y + self.m22 * v.z,
  )
}

///|
pub fn Mat3::mul(self : Mat3, other : Mat3) -> Mat3 {
  Mat3::new(
    self.m00 * other.m00 + self.m01 * other.m10 + self.m02 * other.m20,
    self.m00 * other.m01 + self.m01 * other.m11 + self.m02 * other.m21,
    self.m00 * other.m02 + self.m01 * other.m12 + self.m02 * other.m22,
    self.m10 * other.m00 + self.m11 * other.m10 + self.m12 * other.m20,
    self.m10 * other.m01 + self.m11 * other.m11 + self.m12 * other.m21,
    self.m10 * other.m02 + self.m11 * other.m12 + self.m12 * other.m22,
    self.m20 * other.m00 + self.m21 * other.m10 + self.m22 * other.m20,
    self.m20 * other.m01 + self.m21 * other.m11 + self.m22 * other.m21,
    self.m20 * other.m02 + self.m21 * other.m12 + self.m22 * other.m22,
  )
}

///|
pub fn Mat3::determinant(self : Mat3) -> Real {
  self.m00 * (self.m11 * self.m22 - self.m12 * self.m21) -
  self.m01 * (self.m10 * self.m22 - self.m12 * self.m20) +
  self.m02 * (self.m10 * self.m21 - self.m11 * self.m20)
}

///|
pub fn Mat3::inverse(self : Mat3) -> Mat3 {
  let det = self.determinant()
  if abs(det) <= EPSILON {
    return Mat3::zero()
  }
  let inv_det = 1.0F / det
  Mat3::new(
    (self.m11 * self.m22 - self.m12 * self.m21) * inv_det,
    (self.m02 * self.m21 - self.m01 * self.m22) * inv_det,
    (self.m01 * self.m12 - self.m02 * self.m11) * inv_det,
    (self.m12 * self.m20 - self.m10 * self.m22) * inv_det,
    (self.m00 * self.m22 - self.m02 * self.m20) * inv_det,
    (self.m02 * self.m10 - self.m00 * self.m12) * inv_det,
    (self.m10 * self.m21 - self.m11 * self.m20) * inv_det,
    (self.m01 * self.m20 - self.m00 * self.m21) * inv_det,
    (self.m00 * self.m11 - self.m01 * self.m10) * inv_det,
  )
}

///|
/// Symmetric 3x3 matrix stored in a compact form (6 values).
///
/// This matches the Rapier/Parry `SdpMatrix3` storage pattern and is used for
/// angular inertia tensors.
pub struct SdpMat3 {
  m11 : Real
  m12 : Real
  m13 : Real
  m22 : Real
  m23 : Real
  m33 : Real
}

///|
pub fn SdpMat3::new(
  m11 : Real,
  m12 : Real,
  m13 : Real,
  m22 : Real,
  m23 : Real,
  m33 : Real,
) -> SdpMat3 {
  SdpMat3::{ m11, m12, m13, m22, m23, m33 }
}

///|
pub fn SdpMat3::zero() -> SdpMat3 {
  SdpMat3::new(0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F)
}

///|
pub fn SdpMat3::from_diagonal(diag : Vec3) -> SdpMat3 {
  SdpMat3::new(diag.x, 0.0F, 0.0F, diag.y, 0.0F, diag.z)
}

///|
pub fn SdpMat3::into_mat3(self : SdpMat3) -> Mat3 {
  Mat3::new(
    self.m11,
    self.m12,
    self.m13,
    self.m12,
    self.m22,
    self.m23,
    self.m13,
    self.m23,
    self.m33,
  )
}

///|
pub fn SdpMat3::transform_vec3(self : SdpMat3, v : Vec3) -> Vec3 {
  let x = self.m11 * v.x + self.m12 * v.y + self.m13 * v.z
  let y = self.m12 * v.x + self.m22 * v.y + self.m23 * v.z
  let z = self.m13 * v.x + self.m23 * v.y + self.m33 * v.z
  Vec3::new(x, y, z)
}

///|
pub fn SdpMat3::inverse(self : SdpMat3) -> SdpMat3 {
  // Ported from rapier-reference/src/utils/angular_inertia_ops.rs (dim3).
  let minor_m12_m23 = self.m22 * self.m33 - self.m23 * self.m23
  let minor_m11_m23 = self.m12 * self.m33 - self.m13 * self.m23
  let minor_m11_m22 = self.m12 * self.m23 - self.m13 * self.m22
  let determinant = self.m11 * minor_m12_m23 -
    self.m12 * minor_m11_m23 +
    self.m13 * minor_m11_m22
  if abs(determinant) <= EPSILON {
    SdpMat3::zero()
  } else {
    SdpMat3::new(
      minor_m12_m23 / determinant,
      -minor_m11_m23 / determinant,
      minor_m11_m22 / determinant,
      (self.m11 * self.m33 - self.m13 * self.m13) / determinant,
      (self.m13 * self.m12 - self.m23 * self.m11) / determinant,
      (self.m11 * self.m22 - self.m12 * self.m12) / determinant,
    )
  }
}

///|
/// 3D mass properties: mass, center-of-mass, and angular inertia expressed about the COM.
pub struct MassProperties3 {
  mass : Real
  inv_mass : Real
  inertia : SdpMat3
  inv_inertia : SdpMat3
  center_of_mass : Vec3
}

///|
pub fn MassProperties3::new(
  mass : Real,
  inertia : SdpMat3,
  center_of_mass : Vec3,
) -> MassProperties3 {
  let inv_mass = if mass <= EPSILON { 0.0F } else { 1.0F / mass }
  let inv_inertia = inertia.inverse()
  MassProperties3::{ mass, inv_mass, inertia, inv_inertia, center_of_mass }
}

///|
pub fn MassProperties3::default() -> MassProperties3 {
  MassProperties3::new(0.0F, SdpMat3::zero(), Vec3::zero())
}

///|
pub fn MassProperties3::transform_by(
  self : MassProperties3,
  transform : Isometry3,
) -> MassProperties3 {
  let rot = transform.rotation.to_mat3()
  let inertia_m = rot.mul(self.inertia.into_mat3()).mul(rot.transpose())
  // Preserve symmetry by re-extracting the symmetric components.
  let inertia = SdpMat3::new(
    inertia_m.m00,
    inertia_m.m01,
    inertia_m.m02,
    inertia_m.m11,
    inertia_m.m12,
    inertia_m.m22,
  )
  MassProperties3::new(
    self.mass,
    inertia,
    transform.transform_point(self.center_of_mass),
  )
}

///|
pub fn MassProperties3::add(
  self : MassProperties3,
  other : MassProperties3,
) -> MassProperties3 {
  if self.mass <= EPSILON {
    return other
  }
  if other.mass <= EPSILON {
    return self
  }
  let total_mass = self.mass + other.mass
  let inv_total = 1.0F / total_mass
  let com = Vec3::new(
    (self.center_of_mass.x * self.mass + other.center_of_mass.x * other.mass) *
    inv_total,
    (self.center_of_mass.y * self.mass + other.center_of_mass.y * other.mass) *
    inv_total,
    (self.center_of_mass.z * self.mass + other.center_of_mass.z * other.mass) *
    inv_total,
  )
  let r1 = self.center_of_mass.sub(com)
  let r2 = other.center_of_mass.sub(com)

  // Parallel axis theorem:
  // I_com = I1 + m1*(||r1||^2*I - r1*r1^T) + I2 + m2*(||r2||^2*I - r2*r2^T)
  fn shift_inertia(m : Real, r : Vec3) -> Mat3 {
    let r2 = r.length_squared()
    let xx = r.x * r.x
    let yy = r.y * r.y
    let zz = r.z * r.z
    let xy = r.x * r.y
    let xz = r.x * r.z
    let yz = r.y * r.z
    Mat3::new(
      (r2 - xx) * m,
      -xy * m,
      -xz * m,
      -xy * m,
      (r2 - yy) * m,
      -yz * m,
      -xz * m,
      -yz * m,
      (r2 - zz) * m,
    )
  }

  let inertia_m = self.inertia
    .into_mat3()
    .add(shift_inertia(self.mass, r1))
    .add(other.inertia.into_mat3())
    .add(shift_inertia(other.mass, r2))
  let inertia = SdpMat3::new(
    inertia_m.m00,
    inertia_m.m01,
    inertia_m.m02,
    inertia_m.m11,
    inertia_m.m12,
    inertia_m.m22,
  )
  MassProperties3::new(total_mass, inertia, com)
}

///|
pub fn MassProperties3::set_mass(
  self : MassProperties3,
  new_mass : Real,
  update_inertia : Bool,
) -> MassProperties3 {
  if self.mass <= EPSILON {
    let inertia = if update_inertia { SdpMat3::zero() } else { self.inertia }
    return MassProperties3::new(new_mass, inertia, self.center_of_mass)
  }
  let inertia = if update_inertia {
    let scale = new_mass / self.mass
    let m = self.inertia
    SdpMat3::new(
      m.m11 * scale,
      m.m12 * scale,
      m.m13 * scale,
      m.m22 * scale,
      m.m23 * scale,
      m.m33 * scale,
    )
  } else {
    self.inertia
  }
  MassProperties3::new(new_mass, inertia, self.center_of_mass)
}

///|
pub fn Quat::to_mat3(self : Quat) -> Mat3 {
  // Standard quaternion-to-matrix conversion (assumes unit quaternion).
  let q = self.normalize()
  let x2 = q.x + q.x
  let y2 = q.y + q.y
  let z2 = q.z + q.z
  let xx = q.x * x2
  let yy = q.y * y2
  let zz = q.z * z2
  let xy = q.x * y2
  let xz = q.x * z2
  let yz = q.y * z2
  let wx = q.w * x2
  let wy = q.w * y2
  let wz = q.w * z2
  Mat3::new(
    1.0F - (yy + zz),
    xy - wz,
    xz + wy,
    xy + wz,
    1.0F - (xx + zz),
    yz - wx,
    xz - wy,
    yz + wx,
    1.0F - (xx + yy),
  )
}
