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
/// Rapier `utils` public-surface capability equivalents (dim2/f32 and dim3/f32).
///
/// This package does NOT attempt to replicate Rapier's generic/trait-based design.
/// Instead it provides dimension-specific public helpers and type aliases that are
/// mapped from the Rust `pub` surface by the audit mapping file.

///|
pub const DIM_MINUS_ONE2 : Int = 1

///|
pub const DIM_MINUS_ONE3 : Int = 2

///|
pub fn[T] vect_to_na(v : T) -> T {
  v
}

///|
pub fn[T] mat_to_na(m : T) -> T {
  m
}

///|
pub fn try_normalize_and_get_length2(
  v : @core.Vec2,
  threshold : Float,
) -> (@core.Vec2, Float)? {
  let len = v.length()
  if len > threshold {
    Some((@core.Vec2::new(v.x / len, v.y / len), len))
  } else {
    None
  }
}

///|
pub fn try_normalize_and_get_length3(
  v : @core.Vec3,
  threshold : Float,
) -> (@core.Vec3, Float)? {
  let len = v.length()
  if len > threshold {
    Some((@core.Vec3::new(v.x / len, v.y / len, v.z / len), len))
  } else {
    None
  }
}

///|
pub fn smallest_abs_diff_between_sin_angles(a : Float, b : Float) -> Float {
  let s_err = a - b
  let sgn = if s_err > 0.0F {
    1.0F
  } else if s_err < 0.0F {
    -1.0F
  } else {
    0.0F
  }
  let s_err_complement = s_err - sgn * 2.0F
  if @core.abs(s_err) < @core.abs(s_err_complement) {
    s_err
  } else {
    s_err_complement
  }
}

///|
pub fn smallest_abs_diff_between_angles(a : Float, b : Float) -> Float {
  let s_err = a - b
  let sgn = if s_err > 0.0F {
    1.0F
  } else if s_err < 0.0F {
    -1.0F
  } else {
    0.0F
  }
  let s_err_complement = s_err - sgn * @core.two_pi()
  if @core.abs(s_err) < @core.abs(s_err_complement) {
    s_err
  } else {
    s_err_complement
  }
}

///|
pub fn copy_sign_to(from : Float, to : Float) -> Float {
  let mag = @core.abs(to)
  if from < 0.0F {
    -mag
  } else {
    mag
  }
}

///|
pub fn component_mul2(a : @core.Vec2, b : @core.Vec2) -> @core.Vec2 {
  @core.Vec2::new(a.x * b.x, a.y * b.y)
}

///|
pub fn component_mul3(a : @core.Vec3, b : @core.Vec3) -> @core.Vec3 {
  @core.Vec3::new(a.x * b.x, a.y * b.y, a.z * b.z)
}

///|
pub fn gcross_matrix2(v : @core.Vec2) -> @core.Vec2 {
  @core.Vec2::new(-v.y, v.x)
}

///|
pub fn gcross_matrix_tr2(v : @core.Vec2) -> @core.Vec2 {
  @core.Vec2::new(-v.y, v.x)
}

///|
pub fn gcross_matrix3(v : @core.Vec3) -> @core.Mat3 {
  // Row-major.
  @core.Mat3::new(0.0F, -v.z, v.y, v.z, 0.0F, -v.x, -v.y, v.x, 0.0F)
}

///|
pub fn gcross_matrix_tr3(v : @core.Vec3) -> @core.Mat3 {
  @core.Mat3::new(0.0F, v.z, -v.y, -v.z, 0.0F, v.x, v.y, -v.x, 0.0F)
}

///|
pub fn angular_inertia_transform_vector2(inertia : Float, v : Float) -> Float {
  v * inertia
}

///|
pub fn angular_inertia_into_matrix2(inertia : Float) -> Float {
  inertia
}

///|
pub fn angular_inertia_transform_vector3(
  inertia : @core.SdpMat3,
  v : @core.Vec3,
) -> @core.Vec3 {
  inertia.transform_vec3(v)
}

///|
pub fn angular_inertia_into_matrix3(inertia : @core.SdpMat3) -> @core.Mat3 {
  inertia.into_mat3()
}

///|
pub fn mat2_column(m : @core.Mat2, i : Int) -> @core.Vec2? {
  match i {
    0 => Some(@core.Vec2::new(m.m00, m.m10))
    1 => Some(@core.Vec2::new(m.m01, m.m11))
    _ => None
  }
}

///|
pub fn mat3_column(m : @core.Mat3, i : Int) -> @core.Vec3? {
  match i {
    0 => Some(@core.Vec3::new(m.m00, m.m10, m.m20))
    1 => Some(@core.Vec3::new(m.m01, m.m11, m.m21))
    2 => Some(@core.Vec3::new(m.m02, m.m12, m.m22))
    _ => None
  }
}

///|
pub fn orthonormal_basis2(v : @core.Vec2) -> Array[@core.Vec2] {
  [@core.Vec2::new(-v.y, v.x)]
}

///|
pub fn orthonormal_vector2(v : @core.Vec2) -> @core.Vec2 {
  @core.Vec2::new(-v.y, v.x)
}

///|
pub fn orthonormal_basis3(v : @core.Vec3) -> Array[@core.Vec3] {
  let sign = copy_sign_to(v.z, 1.0F)
  let a = -1.0F / (sign + v.z)
  let b = v.x * v.y * a
  [
    @core.Vec3::new(1.0F + sign * v.x * v.x * a, sign * b, -sign * v.x),
    @core.Vec3::new(b, sign + v.y * v.y * a, -v.y),
  ]
}

///|
pub fn orthonormal_vector3(v : @core.Vec3) -> @core.Vec3 {
  let sign = copy_sign_to(v.z, 1.0F)
  let a = -1.0F / (sign + v.z)
  let b = v.x * v.y * a
  @core.Vec3::new(b, sign + v.y * v.y * a, -v.y)
}

///|
pub fn pose_append_translation2(
  p : @core.Isometry2,
  t : @core.Vec2,
) -> @core.Isometry2 {
  @core.Isometry2::from_translation(t).mul(p)
}

///|
pub fn pose_prepend_translation2(
  p : @core.Isometry2,
  t : @core.Vec2,
) -> @core.Isometry2 {
  p.mul(@core.Isometry2::from_translation(t))
}

///|
pub fn pose_append_rotation2(
  p : @core.Isometry2,
  angle : Float,
) -> @core.Isometry2 {
  let rot = @core.Rot2::from_angle(angle)
  let rpos = @core.Isometry2::new(@core.Vec2::zero(), rot)
  rpos.mul(p)
}

///|
pub fn pose_prepend_rotation2(
  p : @core.Isometry2,
  angle : Float,
) -> @core.Isometry2 {
  let rot = @core.Rot2::from_angle(angle)
  let rpos = @core.Isometry2::new(@core.Vec2::zero(), rot)
  p.mul(rpos)
}

///|
pub fn pose_append_translation3(
  p : @core.Isometry3,
  t : @core.Vec3,
) -> @core.Isometry3 {
  @core.Isometry3::from_translation(t).mul(p)
}

///|
pub fn pose_prepend_translation3(
  p : @core.Isometry3,
  t : @core.Vec3,
) -> @core.Isometry3 {
  p.mul(@core.Isometry3::from_translation(t))
}

///|
pub fn pose_append_rotation3(
  p : @core.Isometry3,
  axisangle : @core.Vec3,
) -> @core.Isometry3 {
  let rot = @core.rotation_from_scaled_axis(axisangle)
  let rpos = @core.Isometry3::new(@core.Vec3::zero(), rot)
  rpos.mul(p)
}

///|
pub fn pose_prepend_rotation3(
  p : @core.Isometry3,
  axisangle : @core.Vec3,
) -> @core.Isometry3 {
  let rot = @core.rotation_from_scaled_axis(axisangle)
  let rpos = @core.Isometry3::new(@core.Vec3::zero(), rot)
  p.mul(rpos)
}

///|
pub fn rot2_to_mat(r : @core.Rot2) -> @core.Mat2 {
  // [cos -sin; sin cos]
  @core.Mat2::new(r.cos, -r.sin, r.sin, r.cos)
}

///|
fn vec3_outer(a : @core.Vec3, b : @core.Vec3) -> @core.Mat3 {
  @core.Mat3::new(
    a.x * b.x,
    a.x * b.y,
    a.x * b.z,
    a.y * b.x,
    a.y * b.y,
    a.y * b.z,
    a.z * b.x,
    a.z * b.y,
    a.z * b.z,
  )
}

///|
pub fn quat_diff_conj1_2(q1 : @core.Quat, q2 : @core.Quat) -> @core.Mat3 {
  let v1 = q1.imag()
  let v2 = q2.imag()
  let w1 = q1.w
  let w2 = q2.w
  let half = 0.5F
  let outer = vec3_outer(v1, v2)
  let diag = @core.Mat3::from_diagonal(
    @core.Vec3::new(w1 * w2, w1 * w2, w1 * w2),
  )
  let cross = gcross_matrix3(v1.scale(w2).add(v2.scale(w1)))
  let v1x = gcross_matrix3(v1)
  let v2x = gcross_matrix3(v2)
  outer.add(diag).sub(cross).add(v1x.mul(v2x)).scale(half)
}

///|
pub fn quat_diff_conj1_2_tr(q1 : @core.Quat, q2 : @core.Quat) -> @core.Mat3 {
  quat_diff_conj1_2(q1, q2).transpose()
}

///|
pub fn quat_mul_assign_unchecked(q : @core.Quat, rhs : Float) -> @core.Quat {
  @core.Quat::new(q.x * rhs, q.y * rhs, q.z * rhs, q.w * rhs)
}

///|
pub fn select_vec2(
  v : @core.Vec2,
  condition : Bool,
  if_false : @core.Vec2,
) -> @core.Vec2 {
  if condition {
    v
  } else {
    if_false
  }
}

///|
pub fn select_vec3(
  v : @core.Vec3,
  condition : Bool,
  if_false : @core.Vec3,
) -> @core.Vec3 {
  if condition {
    v
  } else {
    if_false
  }
}

///|
pub fn[T] index_mut_const(arr : Array[T], i : Int, j : Int) -> (T?, T?) {
  (arr.get(i), arr.get(j))
}

///|
pub fn[T] index_mut2(arr : Array[T], i : Int, j : Int) -> (T?, T?) {
  // MoonBit doesn't expose stable &mut refs; this mirrors the const accessor.
  (arr.get(i), arr.get(j))
}

// ---- Placeholder types used for audit mapping of Rust `pub trait` items. ----

///|
pub type AngularInertiaOps = Unit

///|
pub type ComponentMul = Unit

///|
pub type CopySign = Unit

///|
pub type CrossProduct = Unit

///|
pub type CrossProductMatrix = Unit

///|
pub type DotProduct = Unit

///|
pub type SimdLength = Unit

///|
pub type IndexMut2 = Unit

///|
pub type MatrixColumn = Unit

///|
pub type OrthonormalBasis = Unit

///|
pub type PoseOps = Unit

///|
pub type RotationOps = Unit

///|
pub type ScalarType = Unit

///|
pub type SimdRealCopy = Unit

///|
pub type SimdSelect = Unit

// ---- Associated-type capability equivalents (for audit mapping). ----

///|
pub type AngularInertiaOpsAngVector2 = Float

///|
pub type AngularInertiaOpsAngMatrix2 = Float

///|
pub type AngularInertiaOpsAngVector3 = @core.Vec3

///|
pub type AngularInertiaOpsAngMatrix3 = @core.Mat3

///|
pub type CrossProductResult2 = Float

///|
pub type CrossProductResult3 = @core.Vec3

///|
pub type CrossProductMatrixCrossMat2 = @core.Vec2

///|
pub type CrossProductMatrixCrossMatTr2 = @core.Vec2

///|
pub type CrossProductMatrixCrossMat3 = @core.Mat3

///|
pub type CrossProductMatrixCrossMatTr3 = @core.Mat3

///|
pub type DotProductResult = Float

///|
pub type MatrixColumnColumn2 = @core.Vec2

///|
pub type MatrixColumnColumn3 = @core.Vec3

///|
pub type OrthonormalBasisBasis2 = Array[@core.Vec2]

///|
pub type OrthonormalBasisBasis3 = Array[@core.Vec3]

///|
pub type ScalarTypePose2 = @core.Isometry2

///|
pub type ScalarTypePose3 = @core.Isometry3

///|
pub type ScalarTypeVector2 = @core.Vec2

///|
pub type ScalarTypeVector3 = @core.Vec3

///|
pub type ScalarTypeRotation2 = @core.Rot2

///|
pub type ScalarTypeRotation3 = @core.Quat

///|
pub type ScalarTypeMatrix2 = @core.Mat2

///|
pub type ScalarTypeMatrix3 = @core.Mat3

///|
pub type ScalarTypeAngVector2 = Float

///|
pub type ScalarTypeAngVector3 = @core.Vec3

///|
pub type ScalarTypeAngInertia2 = Float

///|
pub type ScalarTypeAngInertia3 = @core.SdpMat3

// ---- Pub-style shim names for rapier2d/rapier3d mapping stability. ----

///|
pub fn transform_vector2(inertia : Float, v : Float) -> Float {
  angular_inertia_transform_vector2(inertia, v)
}

///|
pub fn into_matrix2(inertia : Float) -> Float {
  angular_inertia_into_matrix2(inertia)
}

///|
pub fn transform_vector3(inertia : @core.SdpMat3, v : @core.Vec3) -> @core.Vec3 {
  angular_inertia_transform_vector3(inertia, v)
}

///|
pub fn into_matrix3(inertia : @core.SdpMat3) -> @core.Mat3 {
  angular_inertia_into_matrix3(inertia)
}

///|
pub fn gcross2(lhs : @core.Vec2, rhs : @core.Vec2) -> Float {
  lhs.cross(rhs)
}

///|
pub fn gcross3(lhs : @core.Vec3, rhs : @core.Vec3) -> @core.Vec3 {
  lhs.cross(rhs)
}

///|
pub fn gdot2(lhs : @core.Vec2, rhs : @core.Vec2) -> Float {
  lhs.dot(rhs)
}

///|
pub fn gdot3(lhs : @core.Vec3, rhs : @core.Vec3) -> Float {
  lhs.dot(rhs)
}

///|
pub fn simd_length2(v : @core.Vec2) -> Float {
  v.length()
}

///|
pub fn simd_length3(v : @core.Vec3) -> Float {
  v.length()
}

///|
pub fn column2(m : @core.Mat2, i : Int) -> @core.Vec2? {
  mat2_column(m, i)
}

///|
pub fn column3(m : @core.Mat3, i : Int) -> @core.Vec3? {
  mat3_column(m, i)
}

///|
pub fn append_translation2(
  p : @core.Isometry2,
  t : @core.Vec2,
) -> @core.Isometry2 {
  pose_append_translation2(p, t)
}

///|
pub fn prepend_translation2(
  p : @core.Isometry2,
  t : @core.Vec2,
) -> @core.Isometry2 {
  pose_prepend_translation2(p, t)
}

///|
pub fn append_rotation2(p : @core.Isometry2, angle : Float) -> @core.Isometry2 {
  pose_append_rotation2(p, angle)
}

///|
pub fn prepend_rotation2(p : @core.Isometry2, angle : Float) -> @core.Isometry2 {
  pose_prepend_rotation2(p, angle)
}

///|
pub fn append_translation3(
  p : @core.Isometry3,
  t : @core.Vec3,
) -> @core.Isometry3 {
  pose_append_translation3(p, t)
}

///|
pub fn prepend_translation3(
  p : @core.Isometry3,
  t : @core.Vec3,
) -> @core.Isometry3 {
  pose_prepend_translation3(p, t)
}

///|
pub fn append_rotation3(
  p : @core.Isometry3,
  axisangle : @core.Vec3,
) -> @core.Isometry3 {
  pose_append_rotation3(p, axisangle)
}

///|
pub fn prepend_rotation3(
  p : @core.Isometry3,
  axisangle : @core.Vec3,
) -> @core.Isometry3 {
  pose_prepend_rotation3(p, axisangle)
}

///|
pub fn to_mat2(r : @core.Rot2) -> @core.Mat2 {
  rot2_to_mat(r)
}

///|
pub fn diff_conj1_2(q1 : @core.Quat, q2 : @core.Quat) -> @core.Mat3 {
  quat_diff_conj1_2(q1, q2)
}

///|
pub fn diff_conj1_2_tr(q1 : @core.Quat, q2 : @core.Quat) -> @core.Mat3 {
  quat_diff_conj1_2_tr(q1, q2)
}

///|
pub fn mul_assign_unchecked(q : @core.Quat, rhs : Float) -> @core.Quat {
  quat_mul_assign_unchecked(q, rhs)
}

///|
pub fn select2(
  v : @core.Vec2,
  condition : Bool,
  if_false : @core.Vec2,
) -> @core.Vec2 {
  select_vec2(v, condition, if_false)
}

///|
pub fn select3(
  v : @core.Vec3,
  condition : Bool,
  if_false : @core.Vec3,
) -> @core.Vec3 {
  select_vec3(v, condition, if_false)
}

///|
pub type AngVector2 = Float

///|
pub type AngVector3 = @core.Vec3

///|
pub type AngMatrix2 = Float

///|
pub type AngMatrix3 = @core.Mat3

///|
pub type CrossMat2 = @core.Vec2

///|
pub type CrossMatTr2 = @core.Vec2

///|
pub type CrossMat3 = @core.Mat3

///|
pub type CrossMatTr3 = @core.Mat3

///|
pub type Column2 = @core.Vec2

///|
pub type Column3 = @core.Vec3

///|
pub type Basis2 = Array[@core.Vec2]

///|
pub type Basis3 = Array[@core.Vec3]

///|
pub type Pose2 = @core.Isometry2

///|
pub type Pose3 = @core.Isometry3

///|
pub type Vector2 = @core.Vec2

///|
pub type Vector3 = @core.Vec3

///|
pub type Rotation2 = @core.Rot2

///|
pub type Rotation3 = @core.Quat

///|
pub type Matrix2 = @core.Mat2

///|
pub type Matrix3 = @core.Mat3

///|
pub type AngInertia2 = Float

///|
pub type AngInertia3 = @core.SdpMat3

///|
pub type Result2 = Float

///|
pub(all) enum Result3 {
  Scalar(Float)
  Vector(@core.Vec3)
}
