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
        let inv_m = body.inv_mass()
        let inv_i = body.inv_inertia()
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
        let inv_i = body.inv_inertia()
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
  inv_mass_x : @core.Real
  inv_mass_y : @core.Real
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
      inv_mass1 = rb1.inv_mass()
      inv_inertia1 = rb1.inv_inertia()
    }
    let mut inv_mass2 = 0.0F
    let mut inv_inertia2 = 0.0F
    if rb2.is_dynamic() && rb2.is_enabled() {
      inv_mass2 = rb2.inv_mass()
      inv_inertia2 = rb2.inv_inertia()
    }

    // For u=(1,0), cross(r,u) = -r.y. For u=(0,1), cross(r,u) = r.x.
    let denom_x = inv_mass1 +
      inv_mass2 +
      inv_inertia1 * r1.y * r1.y +
      inv_inertia2 * r2.y * r2.y
    let denom_y = inv_mass1 +
      inv_mass2 +
      inv_inertia1 * r1.x * r1.x +
      inv_inertia2 * r2.x * r2.x
    let denom_ang = inv_inertia1 + inv_inertia2
    let inv_mass_x = if denom_x <= 1.0e-12F { 0.0F } else { 1.0F / denom_x }
    let inv_mass_y = if denom_y <= 1.0e-12F { 0.0F } else { 1.0F / denom_y }
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
      inv_mass_x,
      inv_mass_y,
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
      // Solve along world X.
      let (v1, w1, com1) = joint_get_body_kinematic_state(bodies, c.body1)
      let (v2, w2, com2) = joint_get_body_kinematic_state(bodies, c.body2)
      let r1 = c.anchor1.sub(com1)
      let r2 = c.anchor2.sub(com2)
      let vel1 = v1.add(joint_cross_z_scalar(w1, r1))
      let vel2 = v2.add(joint_cross_z_scalar(w2, r2))
      let rv = vel1.sub(vel2)
      let lambda_x = -(rv.x + c.bias.x) * c.inv_mass_x
      constraints[i].impulses = @core.Vec2::new(
        constraints[i].impulses.x + lambda_x,
        constraints[i].impulses.y,
      )
      joint_apply_impulse_delta_dynamic(
        bodies,
        Some(c.body1),
        @core.Vec2::new(lambda_x, 0.0F),
        c.anchor1,
      )
      joint_apply_impulse_delta_dynamic(
        bodies,
        Some(c.body2),
        @core.Vec2::new(-lambda_x, 0.0F),
        c.anchor2,
      )

      // Solve along world Y (after X impulse is applied).
      let (v1y, w1y, com1y) = joint_get_body_kinematic_state(bodies, c.body1)
      let (v2y, w2y, com2y) = joint_get_body_kinematic_state(bodies, c.body2)
      let r1y = c.anchor1.sub(com1y)
      let r2y = c.anchor2.sub(com2y)
      let vel1y = v1y.add(joint_cross_z_scalar(w1y, r1y))
      let vel2y = v2y.add(joint_cross_z_scalar(w2y, r2y))
      let rvy = vel1y.sub(vel2y)
      let lambda_y = -(rvy.y + c.bias.y) * c.inv_mass_y
      constraints[i].impulses = @core.Vec2::new(
        constraints[i].impulses.x,
        constraints[i].impulses.y + lambda_y,
      )
      joint_apply_impulse_delta_dynamic(
        bodies,
        Some(c.body1),
        @core.Vec2::new(0.0F, lambda_y),
        c.anchor1,
      )
      joint_apply_impulse_delta_dynamic(
        bodies,
        Some(c.body2),
        @core.Vec2::new(0.0F, -lambda_y),
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
          let error = joint_wrap_angle_pi(rel_angle - c.motor.target_pos)
          let vel_error = rel_w - c.motor.target_vel
          // Treat `stiffness` and `damping` as torque spring constants (rapier style) and
          // compute a stable angular impulse using an implicit damping term.
          let inv_i_sum = 1.0F / c.inv_mass_ang
          let denom = 1.0F + c.motor.damping * dt * inv_i_sum
          let lambda = -(c.motor.stiffness * error + c.motor.damping * vel_error) *
            dt /
            denom
          let max_impulse = c.motor.max_force * dt
          let old = constraints[i].motor_impulse
          let mut new_impulse = old + lambda
          if new_impulse > max_impulse {
            new_impulse = max_impulse
          } else if new_impulse < -max_impulse {
            new_impulse = -max_impulse
          }
          let applied = new_impulse - old
          constraints[i].motor_impulse = new_impulse
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
    let mut inv_m1 = 0.0F
    if rb1.is_dynamic() && rb1.is_enabled() {
      inv_m1 = rb1.inv_mass()
    }
    let mut inv_m2 = 0.0F
    if rb2.is_dynamic() && rb2.is_enabled() {
      inv_m2 = rb2.inv_mass()
    }
    let denom = inv_m1 + inv_m2
    if denom <= 1.0e-12F {
      continue
    }
    let ratio1 = inv_m1 / denom
    let ratio2 = inv_m2 / denom
    if bodies.get_mut(c.body1) is Some(b1) {
      if b1.is_dynamic() && b1.is_enabled() {
        b1.set_translation(
          b1.translation().sub(joint_vec2_scale(error, ratio1)),
          true,
        )
        |> ignore
      }
    }
    if bodies.get_mut(c.body2) is Some(b2) {
      if b2.is_dynamic() && b2.is_enabled() {
        b2.set_translation(
          b2.translation().add(joint_vec2_scale(error, ratio2)),
          true,
        )
        |> ignore
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
