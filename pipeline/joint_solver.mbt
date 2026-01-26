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
priv struct JointConstraint {
  handle : @dynamics.ImpulseJointHandle
  body1 : @dynamics.RigidBodyHandle
  body2 : @dynamics.RigidBodyHandle
  local_anchor1 : @core.Vec2
  local_anchor2 : @core.Vec2
  anchor1 : @core.Vec2
  anchor2 : @core.Vec2
  bias : @core.Vec2
  inv_mass_x : @core.Real
  inv_mass_y : @core.Real
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
    let local_anchor1 = joint.data.data.local_anchor1()
    let local_anchor2 = joint.data.data.local_anchor2()
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
    let inv_mass_x = if denom_x <= 1.0e-12F { 0.0F } else { 1.0F / denom_x }
    let inv_mass_y = if denom_y <= 1.0e-12F { 0.0F } else { 1.0F / denom_y }
    let impulses = joint_vec2_scale(joint.impulses, warmstart)
    constraints.push(JointConstraint::{
      handle,
      body1: joint.body1,
      body2: joint.body2,
      local_anchor1,
      local_anchor2,
      anchor1: world1,
      anchor2: world2,
      bias,
      inv_mass_x,
      inv_mass_y,
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

///|
fn solve_impulse_joints(
  parameters : IntegrationParameters,
  bodies : @dynamics.RigidBodySet,
  impulse_joints : @dynamics.ImpulseJointSet,
  dt : @core.Real,
) -> Unit {
  if dt <= 0.0F || impulse_joints.len() == 0 {
    return
  }
  let constraints = build_joint_constraints(
    parameters, bodies, impulse_joints, dt,
  )
  if constraints.length() == 0 {
    return
  }
  warmstart_joints(bodies, constraints)
  solve_joint_velocity_constraints(
    bodies,
    constraints,
    dt,
    parameters.num_solver_iterations,
  )
  position_correction_joints(bodies, constraints)
  writeback_joint_impulses(impulse_joints, constraints)
}
