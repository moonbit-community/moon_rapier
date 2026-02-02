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
/// Joint constraints for the real dim3 backend.
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
/// A handle to a joint stored inside a `JointSet3DReal`.
///
/// This currently only references joints created via `insert_generic` and its helpers.
pub struct JointHandle3DReal {
  index : Int
}

///|
/// Generic 3D joint configuration used by the real dim3 backend.
///
/// The constraint is defined in a joint frame attached to each body:
/// `body.pos * local_frame1` and `body.pos * local_frame2`.
/// Axis limits/motors are expressed in the joint frame of body1.
pub struct GenericJoint3DReal {
  local_frame1 : @core.Isometry3
  local_frame2 : @core.Isometry3
  axis_states : Array[AxisState3DReal]
  motors : Array[MotorState3DReal]
  coupled_axes : JointAxesMask3DReal
  contacts_enabled : Bool
}

///|
/// Internal storage type for generic constraints in `JointSet3DReal`.
pub struct GenericJointConstraint3DReal {
  body1 : RigidBodyHandle
  body2 : RigidBodyHandle
  data : GenericJoint3DReal
}

///|
pub struct SpringJoint3DReal {
  body1 : RigidBodyHandle
  body2 : RigidBodyHandle
  local_anchor1 : @core.Vec3
  local_anchor2 : @core.Vec3
  rest_length : @core.Real
  stiffness : @core.Real
  damping : @core.Real
}

///|
pub struct JointSet3DReal {
  spherical : Array[SphericalJoint3DReal]
  prismatic : Array[PrismaticJoint3DReal]
  spherical_motors : Array[SphericalMotor3DReal]
  generic : Array[GenericJointConstraint3DReal]
  springs : Array[SpringJoint3DReal]
}

///|
pub fn JointSet3DReal::disabled_contact_pairs(
  self : JointSet3DReal,
) -> Array[(RigidBodyHandle, RigidBodyHandle)] {
  let out : Array[(RigidBodyHandle, RigidBodyHandle)] = []
  for g in self.generic {
    if !g.data.contacts_enabled {
      out.push((g.body1, g.body2))
    }
  }
  out
}

///|
pub fn JointSet3DReal::new() -> JointSet3DReal {
  {
    spherical: [],
    prismatic: [],
    spherical_motors: [],
    generic: [],
    springs: [],
  }
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
pub fn JointSet3DReal::insert_generic(
  self : JointSet3DReal,
  body1 : RigidBodyHandle,
  body2 : RigidBodyHandle,
  data : GenericJoint3DReal,
  wake_up : Bool,
) -> JointHandle3DReal {
  wake_up |> ignore
  let idx = self.generic.length()
  self.generic.push({ body1, body2, data })
  { index: idx }
}

///|
pub fn JointSet3DReal::insert_revolute(
  self : JointSet3DReal,
  body1 : RigidBodyHandle,
  body2 : RigidBodyHandle,
  axis : @core.Vec3,
  local_anchor1 : @core.Vec3,
  local_anchor2 : @core.Vec3,
  wake_up : Bool,
) -> JointHandle3DReal {
  let a = normalized_or_default(axis)
  // Use the joint Z axis as the revolute axis.
  let basis = @core.rotation_between(@core.Vec3::new(0.0F, 0.0F, 1.0F), a)
  let locked = JointAxesMask3DReal::lin_axes()
    .or(JointAxesMask3DReal::ang_x())
    .or(JointAxesMask3DReal::ang_y())
  let joint = GenericJoint3DRealBuilder::new(locked)
    .local_anchor1(local_anchor1)
    .local_anchor2(local_anchor2)
    .local_basis1(basis)
    .local_basis2(basis)
    .build()
  JointSet3DReal::insert_generic(self, body1, body2, joint, wake_up)
}

///|
pub fn JointSet3DReal::insert_spring(
  self : JointSet3DReal,
  body1 : RigidBodyHandle,
  body2 : RigidBodyHandle,
  local_anchor1 : @core.Vec3,
  local_anchor2 : @core.Vec3,
  rest_length : @core.Real,
  stiffness : @core.Real,
  damping : @core.Real,
  wake_up : Bool,
) -> Unit {
  wake_up |> ignore
  self.springs.push({
    body1,
    body2,
    local_anchor1,
    local_anchor2,
    rest_length,
    stiffness,
    damping,
  })
}

///|
pub fn JointSet3DReal::set_motor_position(
  self : JointSet3DReal,
  handle : JointHandle3DReal,
  axis : JointAxis3DReal,
  target_pos : @core.Real,
  stiffness : @core.Real,
  damping : @core.Real,
  max_force : @core.Real,
) -> Unit {
  let i = axis_index_3d_real(axis)
  let idx = handle.index
  if idx < 0 || idx >= self.generic.length() {
    return
  }
  let j = self.generic[idx]
  j.data.motors[i] = MotorState3DReal::Position(
    target_pos, stiffness, damping, max_force,
  )
  self.generic[idx] = j
}

///|
pub fn JointSet3DReal::set_motor_velocity(
  self : JointSet3DReal,
  handle : JointHandle3DReal,
  axis : JointAxis3DReal,
  target_vel : @core.Real,
  damping : @core.Real,
  max_force : @core.Real,
) -> Unit {
  let i = axis_index_3d_real(axis)
  let idx = handle.index
  if idx < 0 || idx >= self.generic.length() {
    return
  }
  let j = self.generic[idx]
  j.data.motors[i] = MotorState3DReal::Velocity(target_vel, damping, max_force)
  self.generic[idx] = j
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
fn solve_pair_rotation(
  rb1 : RigidBody3D,
  rb2 : RigidBody3D,
  scaled_axis_world : @core.Vec3,
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
    let dq = @core.rotation_from_scaled_axis(scaled_axis_world.scale(-w1))
    rb1.set_rotation(dq.mul(rb1.rotation()))
    rb1.wake_up()
  }
  if inv_m2 > 0.0F {
    let dq = @core.rotation_from_scaled_axis(scaled_axis_world.scale(w2))
    rb2.set_rotation(dq.mul(rb2.rotation()))
    rb2.wake_up()
  }
}

///|
fn axis_value_from_vec3(v : @core.Vec3, axis_i : Int) -> @core.Real {
  if axis_i == 0 {
    v.x
  } else if axis_i == 1 {
    v.y
  } else {
    v.z
  }
}

///|
fn axis_unit(axis_i : Int) -> @core.Vec3 {
  if axis_i == 0 {
    @core.Vec3::new(1.0F, 0.0F, 0.0F)
  } else if axis_i == 1 {
    @core.Vec3::new(0.0F, 1.0F, 0.0F)
  } else {
    @core.Vec3::new(0.0F, 0.0F, 1.0F)
  }
}

///|
fn clamp_real(x : @core.Real, lo : @core.Real, hi : @core.Real) -> @core.Real {
  if x < lo {
    lo
  } else if x > hi {
    hi
  } else {
    x
  }
}

///|
fn shortest_angle_diff(curr : @core.Real, target : @core.Real) -> @core.Real {
  // Returns the signed smallest difference in [-pi, pi] such that curr + diff ~= target.
  let mut diff = target - curr
  let pi = @core.pi()
  let two_pi = @core.two_pi()
  while diff > pi {
    diff = diff - two_pi
  }
  while diff < -pi {
    diff = diff + two_pi
  }
  diff
}

///|
fn shortest_quat(q : @core.Quat) -> @core.Quat {
  let qq = q.normalize()
  if qq.real() < 0.0F {
    qq.negated()
  } else {
    qq
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

    // Generic joints (axis locks/limits/motors).
    for i in 0..<self.generic.length() {
      let gj = self.generic[i]
      if bodies.get(gj.body1) is Some(rb1) && bodies.get(gj.body2) is Some(rb2) {
        let frame1 = rb1.position().mul(gj.data.local_frame1)
        let frame2 = rb2.position().mul(gj.data.local_frame2)
        let rot1 = frame1.rotation
        let rot1_inv = rot1.inverse()
        let diff_world = frame2.translation.sub(frame1.translation)
        let diff_local = rot1_inv.rotate_vec3(diff_world)

        // Coupled LIN_Y/LIN_Z (like rapier's `coupled_axes(LIN_Y | LIN_Z)`).
        let coupled = gj.data.coupled_axes
        if coupled.contains(JointAxesMask3DReal::lin_y()) &&
          coupled.contains(JointAxesMask3DReal::lin_z()) {
          let err = diff_local.y - diff_local.z
          if @core.abs(err) > 1.0e-6F {
            let ay = rot1.rotate_vec3(@core.Vec3::new(0.0F, 1.0F, 0.0F))
            let az = rot1.rotate_vec3(@core.Vec3::new(0.0F, 0.0F, 1.0F))
            solve_pair_translation(
              rb1,
              rb2,
              ay.scale(err * 0.5F).add(az.scale(-err * 0.5F)),
            )
          }
        }

        // Linear constraints + motors.
        for axis_i in 0..<3 {
          let state = gj.data.axis_states[axis_i]
          let comp = axis_value_from_vec3(diff_local, axis_i)
          let axis_w = rot1.rotate_vec3(axis_unit(axis_i))
          match state {
            AxisState3DReal::Locked =>
              if @core.abs(comp) > 1.0e-6F {
                solve_pair_translation(rb1, rb2, axis_w.scale(comp))
              }
            AxisState3DReal::Limited(min, max) => {
              let clamped = clamp_real(comp, min, max)
              let err = comp - clamped
              if @core.abs(err) > 1.0e-6F {
                solve_pair_translation(rb1, rb2, axis_w.scale(err))
              }
            }
            AxisState3DReal::Free => ()
          }

          // Motors apply even on free/limited axes.
          match gj.data.motors[axis_i] {
            MotorState3DReal::Disabled => ()
            MotorState3DReal::Position(target, stiffness, damping, max_force) => {
              let err = comp - target
              let k = clamp_real(stiffness * dt * 0.001F, 0.0F, 0.5F)
              if @core.abs(err) > 1.0e-6F && k > 0.0F {
                let max_corr = max_force * dt * 0.001F
                let corr = clamp_real(err * k, -max_corr, max_corr)
                solve_pair_translation(rb1, rb2, axis_w.scale(corr))
              }
              let c = clamp_real(damping * dt * 0.01F, 0.0F, 1.0F)
              if c > 0.0F {
                damp_pair_along(rb1, rb2, axis_w, c * 0.25F)
              }
            }
            MotorState3DReal::Velocity(target_vel, damping, max_force) => {
              let rel = rb2.linvel().sub(rb1.linvel()).dot(axis_w)
              let err = rel - target_vel
              let c = clamp_real(damping * dt * 0.01F, 0.0F, 1.0F)
              if @core.abs(err) > 1.0e-6F {
                // Velocity correction is intentionally mild to avoid instability.
                let dv = axis_w.scale(err * clamp_real(0.25F + c, 0.25F, 1.0F))
                let inv_m1 = inv_mass3(rb1)
                let inv_m2 = inv_mass3(rb2)
                let inv_sum = inv_m1 + inv_m2
                if inv_sum > 0.0F {
                  let w1 = if inv_m1 > 0.0F { inv_m1 / inv_sum } else { 0.0F }
                  let w2 = if inv_m2 > 0.0F { inv_m2 / inv_sum } else { 0.0F }
                  let max_dv = if max_force > 0.0F {
                    max_force * dt / inv_sum
                  } else {
                    0.0F
                  }
                  let dv_mag = dv.length()
                  let dv_clamped = if max_dv > 0.0F && dv_mag > max_dv {
                    dv.scale(max_dv / dv_mag)
                  } else {
                    dv
                  }
                  if inv_m1 > 0.0F {
                    rb1.set_linvel(rb1.linvel().add(dv_clamped.scale(w1)))
                  }
                  if inv_m2 > 0.0F {
                    rb2.set_linvel(rb2.linvel().sub(dv_clamped.scale(w2)))
                  }
                }
              }
            }
          }
        }

        // Angular constraints + motors (approximate; uses scaled-axis in the joint frame).
        // Follow Rapier's convention: use the quaternion imaginary components (sin(theta/2) * axis)
        // as the per-axis angular coordinates, and keep the shortest-arc representation.
        let q_rel = shortest_quat(
          frame1.rotation.inverse().mul(frame2.rotation),
        )
        let ang_sin_local = q_rel.imag()
        for axis_i in 0..<3 {
          let state = gj.data.axis_states[3 + axis_i]
          let comp_s = axis_value_from_vec3(ang_sin_local, axis_i)
          let axis_w = rot1.rotate_vec3(axis_unit(axis_i))
          match state {
            AxisState3DReal::Locked =>
              if @core.abs(comp_s) > 1.0e-6F {
                // Small-angle: sin(theta/2) ~= theta/2.
                solve_pair_rotation(rb1, rb2, axis_w.scale(comp_s * 2.0F))
              }
            AxisState3DReal::Limited(min, max) => {
              let smin = @core.sin(min * 0.5F)
              let smax = @core.sin(max * 0.5F)
              let clamped = clamp_real(comp_s, smin, smax)
              let err = comp_s - clamped
              if @core.abs(err) > 1.0e-6F {
                solve_pair_rotation(rb1, rb2, axis_w.scale(err * 2.0F))
              }
            }
            AxisState3DReal::Free => ()
          }
          match gj.data.motors[3 + axis_i] {
            MotorState3DReal::Disabled => ()
            MotorState3DReal::Position(target, stiffness, damping, max_force) => {
              let s_clamped = clamp_real(comp_s, -1.0F, 1.0F)
              let ang_dist = @core.asin(s_clamped) * 2.0F
              let diff = shortest_angle_diff(ang_dist, target)
              let k = clamp_real(stiffness * dt * 0.001F, 0.0F, 0.5F)
              if @core.abs(diff) > 1.0e-6F && k > 0.0F {
                let max_corr = max_force * dt * 0.001F
                let corr = clamp_real(diff * k, -max_corr, max_corr)
                solve_pair_rotation(rb1, rb2, axis_w.scale(corr))
              }
              let c = clamp_real(damping * dt * 0.01F, 0.0F, 1.0F)
              if c > 0.0F {
                // Rough angular damping along the axis.
                let rel = rb2.angvel().sub(rb1.angvel()).dot(axis_w)
                let dw = axis_w.scale(rel * c * 0.25F)
                let inv_m1 = inv_mass3(rb1)
                let inv_m2 = inv_mass3(rb2)
                let inv_sum = inv_m1 + inv_m2
                if inv_sum > 0.0F {
                  let w1 = if inv_m1 > 0.0F { inv_m1 / inv_sum } else { 0.0F }
                  let w2 = if inv_m2 > 0.0F { inv_m2 / inv_sum } else { 0.0F }
                  if inv_m1 > 0.0F {
                    rb1.set_angvel(rb1.angvel().add(dw.scale(w1)))
                  }
                  if inv_m2 > 0.0F {
                    rb2.set_angvel(rb2.angvel().sub(dw.scale(w2)))
                  }
                }
              }
            }
            MotorState3DReal::Velocity(target_vel, damping, max_force) => {
              let rel = rb2.angvel().sub(rb1.angvel()).dot(axis_w)
              let err = rel - target_vel
              let c = clamp_real(damping * dt * 0.01F, 0.0F, 1.0F)
              if @core.abs(err) > 1.0e-6F {
                let dw = axis_w.scale(err * clamp_real(0.25F + c, 0.25F, 1.0F))
                let inv_m1 = inv_mass3(rb1)
                let inv_m2 = inv_mass3(rb2)
                let inv_sum = inv_m1 + inv_m2
                if inv_sum > 0.0F {
                  let w1 = if inv_m1 > 0.0F { inv_m1 / inv_sum } else { 0.0F }
                  let w2 = if inv_m2 > 0.0F { inv_m2 / inv_sum } else { 0.0F }
                  let max_dw = if max_force > 0.0F {
                    max_force * dt / inv_sum
                  } else {
                    0.0F
                  }
                  let dw_mag = dw.length()
                  let dw_clamped = if max_dw > 0.0F && dw_mag > max_dw {
                    dw.scale(max_dw / dw_mag)
                  } else {
                    dw
                  }
                  if inv_m1 > 0.0F {
                    rb1.set_angvel(rb1.angvel().add(dw_clamped.scale(w1)))
                  }
                  if inv_m2 > 0.0F {
                    rb2.set_angvel(rb2.angvel().sub(dw_clamped.scale(w2)))
                  }
                }
              }
            }
          }
        }
      }
    }

    // Spring joints (distance constraint with compliance).
    for i in 0..<self.springs.length() {
      let s = self.springs[i]
      if bodies.get(s.body1) is Some(b1) && bodies.get(s.body2) is Some(b2) {
        let p1 = b1.position().transform_point(s.local_anchor1)
        let p2 = b2.position().transform_point(s.local_anchor2)
        let d = p2.sub(p1)
        let dist2 = d.length_squared()
        if dist2 <= 1.0e-12F {
          continue
        }
        let dist = Float::sqrt(dist2)
        let err = dist - s.rest_length
        let dir = d.scale(1.0F / dist)
        let k = clamp_real(s.stiffness * dt * 0.001F, 0.0F, 0.5F)
        if @core.abs(err) > 1.0e-6F && k > 0.0F {
          solve_pair_translation(b1, b2, dir.scale(err * k))
        }
        let c = clamp_real(s.damping * dt * 0.01F, 0.0F, 1.0F)
        if c > 0.0F {
          damp_pair_along(b1, b2, dir, c)
        }
      }
    }
  }
}
