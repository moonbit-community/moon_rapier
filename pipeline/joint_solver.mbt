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
const JOINT_SOLVER_BAUMGARTE : @core.Real = 0.2F

///|
const JOINT_SOLVER_INV_EPSILON : @core.Real = 1.0e-6F

///|
fn joint_inv(val : @core.Real) -> @core.Real {
  if @core.abs(val) <= JOINT_SOLVER_INV_EPSILON {
    0.0F
  } else {
    1.0F / val
  }
}

///|
fn joint_motor_coefficients(
  model : @dynamics.MotorModel,
  dt : @core.Real,
  stiffness : @core.Real,
  damping : @core.Real,
) -> (@core.Real, @core.Real, @core.Real) {
  // Keep consistent with Rapier's MotorModel::combine_coefficients.
  let erp_inv_dt = stiffness * joint_inv(dt * stiffness + damping)
  let inv_cfm = joint_inv(dt * dt * stiffness + dt * damping)
  match model {
    @dynamics.MotorModel::AccelerationBased => (erp_inv_dt, inv_cfm, 0.0F)
    @dynamics.MotorModel::ForceBased => (erp_inv_dt, 0.0F, inv_cfm)
  }
}

///|
fn joint_vec2_scale(v : @core.Vec2, s : @core.Real) -> @core.Vec2 {
  @core.Vec2::new(v.x * s, v.y * s)
}

///|
fn joint_cross_z_scalar(w : @core.Real, v : @core.Vec2) -> @core.Vec2 {
  @core.Vec2::new(-w * v.y, w * v.x)
}

///|
fn joint_apply_impulse_delta_dynamic(
  bodies : @dynamics.RigidBodySet,
  handle : @dynamics.RigidBodyHandle?,
  impulse : @core.Vec2,
  point : @core.Vec2,
) -> Unit {
  if handle is Some(h) {
    if bodies.get_mut(h) is Some(body) {
      if body.is_dynamic() && body.is_enabled() {
        let inv_m = if body.locked_translations {
          0.0F
        } else {
          body.inv_mass()
        }
        let inv_i = if body.locked_rotations {
          0.0F
        } else {
          body.inv_inertia()
        }
        body.set_linvel(
          body.linvel().add(joint_vec2_scale(impulse, inv_m)),
          true,
        )
        |> ignore
        let r = point.sub(body.world_com())
        let torque_impulse = r.cross(impulse)
        body.set_angvel(body.angvel() + torque_impulse * inv_i, true) |> ignore
      }
    }
  }
}

///|
fn joint_get_body_kinematic_state(
  bodies : @dynamics.RigidBodySet,
  handle : @dynamics.RigidBodyHandle,
) -> (@core.Vec2, @core.Real, @core.Vec2) {
  if bodies.get(handle) is Some(rb) {
    (rb.linvel(), rb.angvel(), rb.world_com())
  } else {
    (@core.Vec2::zero(), 0.0F, @core.Vec2::zero())
  }
}

///|
fn joint_get_body_rotation(
  bodies : @dynamics.RigidBodySet,
  handle : @dynamics.RigidBodyHandle,
) -> @core.Rot2 {
  if bodies.get(handle) is Some(rb) {
    rb.rotation()
  } else {
    @core.Rot2::identity()
  }
}

///|
fn joint_apply_angular_impulse_delta_dynamic(
  bodies : @dynamics.RigidBodySet,
  handle : @dynamics.RigidBodyHandle?,
  impulse : @core.Real,
) -> Unit {
  if handle is Some(h) {
    if bodies.get_mut(h) is Some(body) {
      if body.is_dynamic() && body.is_enabled() {
        let inv_i = if body.locked_rotations {
          0.0F
        } else {
          body.inv_inertia()
        }
        body.set_angvel(body.angvel() + impulse * inv_i, true) |> ignore
      }
    }
  }
}

///|
fn joint_wrap_angle_pi(angle : @core.Real) -> @core.Real {
  let pi_value = @core.pi()
  let two_pi = @core.two_pi()
  let mut a = angle
  if a > pi_value {
    a = a - two_pi
  } else if a < -pi_value {
    a = a + two_pi
  }
  a
}

///|
priv struct JointConstraint {
  handle : @dynamics.ImpulseJointHandle
  body1 : @dynamics.RigidBodyHandle
  body2 : @dynamics.RigidBodyHandle
  local_anchor1 : @core.Vec2
  local_anchor2 : @core.Vec2
  local_rot1 : @core.Rot2
  local_rot2 : @core.Rot2
  anchor1 : @core.Vec2
  anchor2 : @core.Vec2
  bias : @core.Vec2
  inv_mass00 : @core.Real
  inv_mass01 : @core.Real
  inv_mass11 : @core.Real
  inv_mass_ang : @core.Real
  motor_enabled : Bool
  motor : @dynamics.JointMotor
  mut motor_impulse : @core.Real
  limits_enabled : Bool
  limit_min : @core.Real
  limit_max : @core.Real
  mut limit_impulse : @core.Real
  mut impulses : @core.Vec2
}

///|
fn build_joint_constraints(
  parameters : IntegrationParameters,
  bodies : @dynamics.RigidBodySet,
  impulse_joints : @dynamics.ImpulseJointSet,
  dt : @core.Real,
) -> Array[JointConstraint] {
  let constraints : Array[JointConstraint] = []
  let joints = impulse_joints.iter()
  let warmstart = parameters.warmstart_coefficient
  for i in 0..<joints.length() {
    let (handle, joint) = joints[i]
    if !joint.is_enabled() {
      continue
    }
    if bodies.get(joint.body1) is None || bodies.get(joint.body2) is None {
      continue
    }
    let rb1 = bodies.get(joint.body1).unwrap()
    let rb2 = bodies.get(joint.body2).unwrap()
    if !rb1.is_enabled() || !rb2.is_enabled() {
      continue
    }
    let local_anchor1 = joint.data.local_anchor1()
    let local_anchor2 = joint.data.local_anchor2()
    let frame1 = joint.data.local_frame1()
    let frame2 = joint.data.local_frame2()
    let local_rot1 = frame1.rotation
    let local_rot2 = frame2.rotation
    let world1 = rb1.position().transform_point(local_anchor1)
    let world2 = rb2.position().transform_point(local_anchor2)
    let error = world1.sub(world2)
    let bias = if dt > 0.0F {
      joint_vec2_scale(error, JOINT_SOLVER_BAUMGARTE / dt)
    } else {
      @core.Vec2::zero()
    }

    // Effective mass denominators (diagonal approximation in world X/Y).
    let (_, _, com1) = joint_get_body_kinematic_state(bodies, joint.body1)
    let (_, _, com2) = joint_get_body_kinematic_state(bodies, joint.body2)
    let r1 = world1.sub(com1)
    let r2 = world2.sub(com2)
    let mut inv_mass1 = 0.0F
    let mut inv_inertia1 = 0.0F
    if rb1.is_dynamic() && rb1.is_enabled() {
      inv_mass1 = if rb1.locked_translations { 0.0F } else { rb1.inv_mass() }
      inv_inertia1 = if rb1.locked_rotations { 0.0F } else { rb1.inv_inertia() }
    }
    let mut inv_mass2 = 0.0F
    let mut inv_inertia2 = 0.0F
    if rb2.is_dynamic() && rb2.is_enabled() {
      inv_mass2 = if rb2.locked_translations { 0.0F } else { rb2.inv_mass() }
      inv_inertia2 = if rb2.locked_rotations { 0.0F } else { rb2.inv_inertia() }
    }

    // For u=(1,0), cross(r,u) = -r.y. For u=(0,1), cross(r,u) = r.x.
    let k00 = inv_mass1 +
      inv_mass2 +
      inv_inertia1 * r1.y * r1.y +
      inv_inertia2 * r2.y * r2.y
    let k01 = -inv_inertia1 * r1.x * r1.y - inv_inertia2 * r2.x * r2.y
    let k11 = inv_mass1 +
      inv_mass2 +
      inv_inertia1 * r1.x * r1.x +
      inv_inertia2 * r2.x * r2.x
    let denom_ang = inv_inertia1 + inv_inertia2
    let det = k00 * k11 - k01 * k01
    let inv_mass00 = if @core.abs(det) <= 1.0e-12F { 0.0F } else { k11 / det }
    let inv_mass01 = if @core.abs(det) <= 1.0e-12F { 0.0F } else { -k01 / det }
    let inv_mass11 = if @core.abs(det) <= 1.0e-12F { 0.0F } else { k00 / det }
    let inv_mass_ang = if denom_ang <= 1.0e-12F {
      0.0F
    } else {
      1.0F / denom_ang
    }
    let motor_axes = joint.data.motor_axes()
    let motor_enabled = motor_axes.contains(@dynamics.JointAxesMask::ang_z())
    let motor = joint.data.motors().ang_z
    let limit_axes = joint.data.limit_axes()
    let limits_enabled = limit_axes.contains(@dynamics.JointAxesMask::ang_z())
    let limits = joint.data.limits().ang_z
    let impulses = joint_vec2_scale(joint.impulses, warmstart)
    constraints.push(JointConstraint::{
      handle,
      body1: joint.body1,
      body2: joint.body2,
      local_anchor1,
      local_anchor2,
      local_rot1,
      local_rot2,
      anchor1: world1,
      anchor2: world2,
      bias,
      inv_mass00,
      inv_mass01,
      inv_mass11,
      inv_mass_ang,
      motor_enabled,
      motor,
      motor_impulse: 0.0F,
      limits_enabled,
      limit_min: limits.min(),
      limit_max: limits.max(),
      limit_impulse: 0.0F,
      impulses,
    })
  }
  constraints
}

///|
fn warmstart_joints(
  bodies : @dynamics.RigidBodySet,
  constraints : Array[JointConstraint],
) -> Unit {
  for i in 0..<constraints.length() {
    let c = constraints[i]
    let impulse = c.impulses
    joint_apply_impulse_delta_dynamic(bodies, Some(c.body1), impulse, c.anchor1)
    joint_apply_impulse_delta_dynamic(
      bodies,
      Some(c.body2),
      @core.Vec2::new(-impulse.x, -impulse.y),
      c.anchor2,
    )
  }
}

///|
fn solve_joint_velocity_constraints(
  bodies : @dynamics.RigidBodySet,
  constraints : Array[JointConstraint],
  dt : @core.Real,
  iterations : Int,
) -> Unit {
  if dt <= 0.0F {
    return
  }
  let iters = if iterations <= 0 { 1 } else { iterations }
  for _ in 0..<iters {
    for i in 0..<constraints.length() {
      let c = constraints[i]
      let (v1, w1, com1) = joint_get_body_kinematic_state(bodies, c.body1)
      let (v2, w2, com2) = joint_get_body_kinematic_state(bodies, c.body2)
      let r1 = c.anchor1.sub(com1)
      let r2 = c.anchor2.sub(com2)
      let vel1 = v1.add(joint_cross_z_scalar(w1, r1))
      let vel2 = v2.add(joint_cross_z_scalar(w2, r2))
      let rv = vel1.sub(vel2)
      let rhs_x = rv.x + c.bias.x
      let rhs_y = rv.y + c.bias.y
      let lambda_x = -(c.inv_mass00 * rhs_x + c.inv_mass01 * rhs_y)
      let lambda_y = -(c.inv_mass01 * rhs_x + c.inv_mass11 * rhs_y)
      constraints[i].impulses = @core.Vec2::new(
        constraints[i].impulses.x + lambda_x,
        constraints[i].impulses.y + lambda_y,
      )
      joint_apply_impulse_delta_dynamic(
        bodies,
        Some(c.body1),
        @core.Vec2::new(lambda_x, lambda_y),
        c.anchor1,
      )
      joint_apply_impulse_delta_dynamic(
        bodies,
        Some(c.body2),
        @core.Vec2::new(-lambda_x, -lambda_y),
        c.anchor2,
      )

      // Solve angular motor and limits around Z (2D).
      if c.inv_mass_ang > 0.0F {
        let w1a = if bodies.get(c.body1) is Some(rb) {
          rb.angvel()
        } else {
          0.0F
        }
        let w2a = if bodies.get(c.body2) is Some(rb) {
          rb.angvel()
        } else {
          0.0F
        }
        let rel_w = w2a - w1a
        let rot1 = joint_get_body_rotation(bodies, c.body1)
        let rot2 = joint_get_body_rotation(bodies, c.body2)
        let joint_rot1 = rot1.mul(c.local_rot1)
        let joint_rot2 = rot2.mul(c.local_rot2)
        let rel_rot = joint_rot1.inverse().mul(joint_rot2)
        let rel_angle = rel_rot.angle()
        if c.motor_enabled {
          let (erp_inv_dt, cfm_coeff, cfm_gain) = joint_motor_coefficients(
            c.motor.model,
            dt,
            c.motor.stiffness,
            c.motor.damping,
          )
          let mut rhs_wo_bias = -c.motor.target_vel
          if erp_inv_dt != 0.0F {
            let error = joint_wrap_angle_pi(rel_angle - c.motor.target_pos)
            rhs_wo_bias = rhs_wo_bias + error * erp_inv_dt
          }
          // Match Rapier's motor constraint solve:
          //   total = impulse + inv_lhs * (rhs - cfm_gain_total * impulse)
          // where rhs = rel_w + rhs_wo_bias.
          let dot_jj = 1.0F / c.inv_mass_ang
          let cfm_gain_total = dot_jj * cfm_coeff + cfm_gain
          let inv_lhs = joint_inv(dot_jj + cfm_gain_total)
          let rhs = rel_w + rhs_wo_bias
          let max_impulse = c.motor.max_force * dt
          let old = constraints[i].motor_impulse
          let mut new_impulse = old + inv_lhs * (rhs - cfm_gain_total * old)
          if new_impulse > max_impulse {
            new_impulse = max_impulse
          } else if new_impulse < -max_impulse {
            new_impulse = -max_impulse
          }
          let applied = new_impulse - old
          constraints[i].motor_impulse = new_impulse
          // Positive impulses reduce `rel_w = w2 - w1`.
          joint_apply_angular_impulse_delta_dynamic(
            bodies,
            Some(c.body1),
            applied,
          )
          joint_apply_angular_impulse_delta_dynamic(
            bodies,
            Some(c.body2),
            -applied,
          )
        }
        if c.limits_enabled {
          if rel_angle < c.limit_min {
            let error = rel_angle - c.limit_min
            let bias = JOINT_SOLVER_BAUMGARTE / dt * error
            let lambda = -(rel_w + bias) * c.inv_mass_ang
            let old = constraints[i].limit_impulse
            let mut new_impulse = old + lambda
            if new_impulse < 0.0F {
              new_impulse = 0.0F
            }
            let applied = new_impulse - old
            constraints[i].limit_impulse = new_impulse
            joint_apply_angular_impulse_delta_dynamic(
              bodies,
              Some(c.body1),
              -applied,
            )
            joint_apply_angular_impulse_delta_dynamic(
              bodies,
              Some(c.body2),
              applied,
            )
          } else if rel_angle > c.limit_max {
            let error = rel_angle - c.limit_max
            let bias = JOINT_SOLVER_BAUMGARTE / dt * error
            let lambda = -(rel_w + bias) * c.inv_mass_ang
            let old = constraints[i].limit_impulse
            let mut new_impulse = old + lambda
            if new_impulse > 0.0F {
              new_impulse = 0.0F
            }
            let applied = new_impulse - old
            constraints[i].limit_impulse = new_impulse
            joint_apply_angular_impulse_delta_dynamic(
              bodies,
              Some(c.body1),
              -applied,
            )
            joint_apply_angular_impulse_delta_dynamic(
              bodies,
              Some(c.body2),
              applied,
            )
          }
        }
      }
    }
  }
}

///|
fn position_correction_joints(
  bodies : @dynamics.RigidBodySet,
  constraints : Array[JointConstraint],
) -> Unit {
  for i in 0..<constraints.length() {
    let c = constraints[i]
    if bodies.get(c.body1) is None || bodies.get(c.body2) is None {
      continue
    }
    let rb1 = bodies.get(c.body1).unwrap()
    let rb2 = bodies.get(c.body2).unwrap()
    if !rb1.is_enabled() || !rb2.is_enabled() {
      continue
    }
    let world1 = rb1.position().transform_point(c.local_anchor1)
    let world2 = rb2.position().transform_point(c.local_anchor2)
    let error = world1.sub(world2)
    if error.length_squared() <= 1.0e-10F {
      continue
    }
    let (v1, w1, com1) = joint_get_body_kinematic_state(bodies, c.body1)
    let (v2, w2, com2) = joint_get_body_kinematic_state(bodies, c.body2)
    v1 |> ignore
    v2 |> ignore
    w1 |> ignore
    w2 |> ignore
    let r1 = world1.sub(com1)
    let r2 = world2.sub(com2)
    let mut inv_m1 = 0.0F
    let mut inv_i1 = 0.0F
    if rb1.is_dynamic() && rb1.is_enabled() {
      inv_m1 = if rb1.locked_translations { 0.0F } else { rb1.inv_mass() }
      inv_i1 = if rb1.locked_rotations { 0.0F } else { rb1.inv_inertia() }
    }
    let mut inv_m2 = 0.0F
    let mut inv_i2 = 0.0F
    if rb2.is_dynamic() && rb2.is_enabled() {
      inv_m2 = if rb2.locked_translations { 0.0F } else { rb2.inv_mass() }
      inv_i2 = if rb2.locked_rotations { 0.0F } else { rb2.inv_inertia() }
    }
    let k00 = inv_m1 + inv_m2 + inv_i1 * r1.y * r1.y + inv_i2 * r2.y * r2.y
    let k01 = -inv_i1 * r1.x * r1.y - inv_i2 * r2.x * r2.y
    let k11 = inv_m1 + inv_m2 + inv_i1 * r1.x * r1.x + inv_i2 * r2.x * r2.x
    let det = k00 * k11 - k01 * k01
    if @core.abs(det) <= 1.0e-12F {
      continue
    }
    let inv00 = k11 / det
    let inv01 = -k01 / det
    let inv11 = k00 / det
    let scaled_error = joint_vec2_scale(error, JOINT_SOLVER_BAUMGARTE)
    let lambda_x = -(inv00 * scaled_error.x + inv01 * scaled_error.y)
    let lambda_y = -(inv01 * scaled_error.x + inv11 * scaled_error.y)
    let impulse = @core.Vec2::new(lambda_x, lambda_y)
    if bodies.get_mut(c.body1) is Some(b1) {
      if b1.is_dynamic() && b1.is_enabled() {
        if !b1.locked_translations {
          b1.set_translation(
            b1.translation().add(joint_vec2_scale(impulse, inv_m1)),
            true,
          )
          |> ignore
        }
        if !b1.locked_rotations {
          let ang = b1.rotation().angle() + r1.cross(impulse) * inv_i1
          b1.set_rotation(@core.Rot2::from_angle(ang), true) |> ignore
        }
      }
    }
    if bodies.get_mut(c.body2) is Some(b2) {
      if b2.is_dynamic() && b2.is_enabled() {
        if !b2.locked_translations {
          b2.set_translation(
            b2.translation().sub(joint_vec2_scale(impulse, inv_m2)),
            true,
          )
          |> ignore
        }
        if !b2.locked_rotations {
          let ang = b2.rotation().angle() - r2.cross(impulse) * inv_i2
          b2.set_rotation(@core.Rot2::from_angle(ang), true) |> ignore
        }
      }
    }
  }
}

///|
fn writeback_joint_impulses(
  impulse_joints : @dynamics.ImpulseJointSet,
  constraints : Array[JointConstraint],
) -> Unit {
  for i in 0..<constraints.length() {
    let c = constraints[i]
    impulse_joints.set_impulses(c.handle, c.impulses)
  }
}
