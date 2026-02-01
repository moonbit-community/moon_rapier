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
/// Minimal joint constraints for the real dim3 backend.
///
/// This is intentionally much smaller than Rapier's impulse-based joint solver and is used to
/// migrate 3D scenes to the `PhysicsPipeline3DReal` backend incrementally.
pub struct SphericalJoint3DReal {
  body1 : RigidBodyHandle
  body2 : RigidBodyHandle
  local_anchor1 : @core.Vec3
  local_anchor2 : @core.Vec3
}

///|
pub struct PrismaticJoint3DReal {
  body1 : RigidBodyHandle
  body2 : RigidBodyHandle
  local_anchor1 : @core.Vec3
  local_anchor2 : @core.Vec3
  // Axis expressed in body1 local-space.
  local_axis1 : @core.Vec3
  motor_target : @core.Real
  motor_stiffness : @core.Real
  motor_damping : @core.Real
  motor_enabled : Bool
}

///|
pub struct SphericalMotor3DReal {
  body1 : RigidBodyHandle
  body2 : RigidBodyHandle
  // Target relative rotation: q_target = q1^{-1} * q2 (in body1 local-space).
  target_rel_rotation : @core.Quat
  stiffness : @core.Real
  damping : @core.Real
}

///|
pub struct JointSet3DReal {
  spherical : Array[SphericalJoint3DReal]
  prismatic : Array[PrismaticJoint3DReal]
  spherical_motors : Array[SphericalMotor3DReal]
}

///|
pub fn JointSet3DReal::new() -> JointSet3DReal {
  { spherical: [], prismatic: [], spherical_motors: [] }
}

///|
pub fn JointSet3DReal::insert_spherical(
  self : JointSet3DReal,
  body1 : RigidBodyHandle,
  body2 : RigidBodyHandle,
  local_anchor1 : @core.Vec3,
  local_anchor2 : @core.Vec3,
) -> Unit {
  self.spherical.push({ body1, body2, local_anchor1, local_anchor2 })
}

///|
pub fn JointSet3DReal::insert_prismatic_motor_position(
  self : JointSet3DReal,
  body1 : RigidBodyHandle,
  body2 : RigidBodyHandle,
  local_axis1 : @core.Vec3,
  local_anchor1 : @core.Vec3,
  local_anchor2 : @core.Vec3,
  target : @core.Real,
  stiffness : @core.Real,
  damping : @core.Real,
) -> Unit {
  self.prismatic.push({
    body1,
    body2,
    local_anchor1,
    local_anchor2,
    local_axis1,
    motor_target: target,
    motor_stiffness: stiffness,
    motor_damping: damping,
    motor_enabled: true,
  })
}

///|
pub fn JointSet3DReal::insert_spherical_motor_rotation(
  self : JointSet3DReal,
  body1 : RigidBodyHandle,
  body2 : RigidBodyHandle,
  target_rel_rotation : @core.Quat,
  stiffness : @core.Real,
  damping : @core.Real,
) -> Unit {
  self.spherical_motors.push({
    body1,
    body2,
    target_rel_rotation,
    stiffness,
    damping,
  })
}

///|
fn inv_mass3(rb : RigidBody3D) -> @core.Real {
  if rb.body_type().is_dynamic() {
    rb.mass_properties().inv_mass
  } else {
    0.0F
  }
}

///|
fn solve_pair_translation(
  rb1 : RigidBody3D,
  rb2 : RigidBody3D,
  corr : @core.Vec3,
) -> Unit {
  let inv_m1 = inv_mass3(rb1)
  let inv_m2 = inv_mass3(rb2)
  let inv_sum = inv_m1 + inv_m2
  if inv_sum <= 0.0F {
    return
  }
  let w1 = if inv_m1 > 0.0F { inv_m1 / inv_sum } else { 0.0F }
  let w2 = if inv_m2 > 0.0F { inv_m2 / inv_sum } else { 0.0F }
  if inv_m1 > 0.0F {
    rb1.set_translation(rb1.translation().add(corr.scale(w1)))
    rb1.wake_up()
  }
  if inv_m2 > 0.0F {
    rb2.set_translation(rb2.translation().sub(corr.scale(w2)))
    rb2.wake_up()
  }
}

///|
fn damp_pair_along(
  rb1 : RigidBody3D,
  rb2 : RigidBody3D,
  dir : @core.Vec3,
  factor : @core.Real,
) -> Unit {
  let inv_m1 = inv_mass3(rb1)
  let inv_m2 = inv_mass3(rb2)
  let inv_sum = inv_m1 + inv_m2
  if inv_sum <= 0.0F {
    return
  }
  let w1 = if inv_m1 > 0.0F { inv_m1 / inv_sum } else { 0.0F }
  let w2 = if inv_m2 > 0.0F { inv_m2 / inv_sum } else { 0.0F }
  let rel = rb2.linvel().sub(rb1.linvel()).dot(dir)
  if rel > 0.0F {
    let dv = dir.scale(rel * factor)
    if inv_m1 > 0.0F {
      rb1.set_linvel(rb1.linvel().add(dv.scale(w1)))
    }
    if inv_m2 > 0.0F {
      rb2.set_linvel(rb2.linvel().sub(dv.scale(w2)))
    }
  }
}

///|
fn normalized_or_default(v : @core.Vec3) -> @core.Vec3 {
  let n2 = v.length_squared()
  if n2 <= 1.0e-12F {
    @core.Vec3::new(1.0F, 0.0F, 0.0F)
  } else {
    v.scale(1.0F / Float::sqrt(n2))
  }
}

///|
pub fn JointSet3DReal::solve(
  self : JointSet3DReal,
  bodies : RigidBodySet3D,
  dt : @core.Real,
) -> Unit {
  // Deterministic, light positional solver.
  let iters = 8
  let inv_dt = if dt > 1.0e-12F { 1.0F / dt } else { 0.0F }
  for _ in 0..<iters {
    // Spherical (ball) joints.
    for i in 0..<self.spherical.length() {
      let j = self.spherical[i]
      if bodies.get(j.body1) is Some(b1) && bodies.get(j.body2) is Some(b2) {
        let p1 = b1.position().transform_point(j.local_anchor1)
        let p2 = b2.position().transform_point(j.local_anchor2)
        let err = p2.sub(p1)
        if err.length_squared() <= 1.0e-10F {
          continue
        }
        solve_pair_translation(b1, b2, err)
        damp_pair_along(b1, b2, normalized_or_default(err), 0.25F)
      }
    }

    // Prismatic joints (positional; only the translation constraints are enforced here).
    for i in 0..<self.prismatic.length() {
      let j = self.prismatic[i]
      if bodies.get(j.body1) is Some(b1) && bodies.get(j.body2) is Some(b2) {
        let p1 = b1.position().transform_point(j.local_anchor1)
        let p2 = b2.position().transform_point(j.local_anchor2)
        let axis = normalized_or_default(
          b1.rotation().rotate_vec3(j.local_axis1),
        )
        let diff = p2.sub(p1)

        // Lock the 2 translations orthogonal to `axis`.
        let along = axis.scale(diff.dot(axis))
        let perp = diff.sub(along)
        if perp.length_squared() > 1.0e-10F {
          solve_pair_translation(b1, b2, perp)
          damp_pair_along(b1, b2, normalized_or_default(perp), 0.25F)
        }

        // Optional motor position along the axis.
        if j.motor_enabled {
          let s = diff.dot(axis)
          let err = s - j.motor_target
          // Scale to a unitless compliance factor. This is not a physical model, just a stable knob
          // that matches the typical stiffness range used in the reference examples.
          let k = @core.clamp(j.motor_stiffness * 10.0F, 0.0F, 1.0F)
          if @core.abs(err) > 1.0e-6F && k > 0.0F {
            solve_pair_translation(b1, b2, axis.scale(err * k))
          }
          // Damp along the axis using the provided damping coefficient.
          let c = @core.clamp(j.motor_damping, 0.0F, 1.0F)
          if c > 0.0F && inv_dt > 0.0F {
            // The factor scales the velocity correction and is intentionally small.
            damp_pair_along(b1, b2, axis, c * 0.25F)
          }
        }
      }
    }

    // Spherical angular motors (very approximate).
    for i in 0..<self.spherical_motors.length() {
      let m = self.spherical_motors[i]
      if bodies.get(m.body1) is Some(b1) && bodies.get(m.body2) is Some(b2) {
        if !b2.body_type().is_dynamic() {
          continue
        }
        let q1 = b1.rotation()
        let q2 = b2.rotation()
        let curr_rel = q1.inverse().mul(q2)
        let err = m.target_rel_rotation.mul(curr_rel.inverse())
        let axis_err = err.to_scaled_axis()
        if axis_err.length_squared() <= 1.0e-12F {
          continue
        }
        let k = @core.clamp(m.stiffness * dt * 0.01F, 0.0F, 0.5F)
        if k > 0.0F {
          let dq = @core.rotation_from_scaled_axis(axis_err.scale(k))
          b2.set_rotation(dq.mul(q2))
          b2.wake_up()
        }
        let c = @core.clamp(m.damping, 0.0F, 1.0F)
        if c > 0.0F {
          // Damp angular velocity in proportion to the current error direction.
          b2.set_angvel(b2.angvel().sub(axis_err.scale(c * 0.1F)))
        }
      }
    }
  }
}
