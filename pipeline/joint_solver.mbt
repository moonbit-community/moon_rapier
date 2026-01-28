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
  handle : @dynamics.ImpulseJointHandle?
  mb_link : @dynamics.MultibodyLinkId?
  body1 : @dynamics.RigidBodyHandle
  body2 : @dynamics.RigidBodyHandle
  local_anchor1 : @core.Vec2
  local_anchor2 : @core.Vec2
  local_rot1 : @core.Rot2
  local_rot2 : @core.Rot2
  anchor1 : @core.Vec2
  anchor2 : @core.Vec2
  basis_x : @core.Vec2
  basis_y : @core.Vec2
  lock_x : Bool
  lock_y : Bool
  lock_ang : Bool
  bias : @core.Vec2 // bias in joint-frame coordinates (x/y).
  inv_mass00 : @core.Real
  inv_mass01 : @core.Real
  inv_mass11 : @core.Real
  inv_mass_x : @core.Real
  inv_mass_y : @core.Real
  inv_mass_ang : @core.Real
  motor_enabled : Bool
  motor : @dynamics.JointMotor
  mut motor_impulse : @core.Real
  motor_lin_x_enabled : Bool
  motor_lin_x : @dynamics.JointMotor
  mut motor_lin_x_impulse : @core.Real
  motor_lin_y_enabled : Bool
  motor_lin_y : @dynamics.JointMotor
  mut motor_lin_y_impulse : @core.Real
  limits_enabled : Bool
  limit_min : @core.Real
  limit_max : @core.Real
  mut limit_impulse : @core.Real
  // Linear limit along X/Y in joint frame (unilateral, like prismatic/pin-slot).
  lin_limit_x_min : @core.Real
  lin_limit_x_max : @core.Real
  lin_limit_x_active : Int
  lin_limit_x_bias : @core.Real
  mut lin_limit_x_impulse : @core.Real
  lin_limit_y_min : @core.Real
  lin_limit_y_max : @core.Real
  lin_limit_y_active : Int
  lin_limit_y_bias : @core.Real
  mut lin_limit_y_impulse : @core.Real
  // Equality constraint impulses for locked linear axes (joint frame X/Y).
  mut impulses : @core.Vec2
  // Angular equality impulse (locked ang_z).
  mut ang_impulse : @core.Real
  // Coupled linear axis constraint (e.g. rope joint in Rapier): distance between anchors is
  // constrained using a single axis stored in `limits/motors` on LinX and an implicit direction.
  coupled_dir : @core.Vec2
  coupled_dist : @core.Real
  coupled_inv_mass : @core.Real
  coupled_limit_enabled : Bool
  coupled_limit_min : @core.Real
  coupled_limit_max : @core.Real
  coupled_limit_active : Int // -1=min, +1=max, 0=inactive
  coupled_limit_bias : @core.Real
  mut coupled_limit_impulse : @core.Real
  coupled_motor_enabled : Bool
  coupled_motor : @dynamics.JointMotor
  mut coupled_motor_impulse : @core.Real
}

///|
fn build_joint_constraints(
  parameters : IntegrationParameters,
  bodies : @dynamics.RigidBodySet,
  impulse_joints : @dynamics.ImpulseJointSet,
  multibody_joints : @dynamics.MultibodyJointSet,
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
    let lock_axes = joint.data.locked_axes()
    let lock_x = lock_axes.contains(@dynamics.JointAxesMask::lin_x())
    let lock_y = lock_axes.contains(@dynamics.JointAxesMask::lin_y())
    let lock_ang = lock_axes.contains(@dynamics.JointAxesMask::ang_z())

    // Joint basis expressed in world-space.
    // For a fully locked linear constraint (revolute/fixed), any orthonormal basis is valid.
    // Using world axes improves stability for large joint grids (Rapier parity tests).
    let joint_rot1 = rb1.rotation().mul(local_rot1)
    let (basis_x, basis_y) = if lock_x && lock_y {
      (@core.Vec2::new(1.0F, 0.0F), @core.Vec2::new(0.0F, 1.0F))
    } else {
      let x = joint_rot1.rotate_vec2(@core.Vec2::new(1.0F, 0.0F))
      (x, @core.Vec2::new(-x.y, x.x))
    }
    let bias = if dt > 0.0F {
      @core.Vec2::new(
        if lock_x {
          error.dot(basis_x) * JOINT_SOLVER_BAUMGARTE / dt
        } else {
          0.0F
        },
        if lock_y {
          error.dot(basis_y) * JOINT_SOLVER_BAUMGARTE / dt
        } else {
          0.0F
        },
      )
    } else {
      @core.Vec2::zero()
    }

    // Effective mass denominators (joint-frame X/Y with a 2x2 solve when both axes are locked).
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
    let ru1 = r1.cross(basis_x)
    let rv1 = r1.cross(basis_y)
    let ru2 = r2.cross(basis_x)
    let rv2 = r2.cross(basis_y)
    let k00 = inv_mass1 +
      inv_mass2 +
      inv_inertia1 * ru1 * ru1 +
      inv_inertia2 * ru2 * ru2
    let k01 = inv_inertia1 * ru1 * rv1 + inv_inertia2 * ru2 * rv2
    let k11 = inv_mass1 +
      inv_mass2 +
      inv_inertia1 * rv1 * rv1 +
      inv_inertia2 * rv2 * rv2
    let denom_ang = inv_inertia1 + inv_inertia2
    let det = k00 * k11 - k01 * k01
    let inv_mass00 = if @core.abs(det) <= 1.0e-12F { 0.0F } else { k11 / det }
    let inv_mass01 = if @core.abs(det) <= 1.0e-12F { 0.0F } else { -k01 / det }
    let inv_mass11 = if @core.abs(det) <= 1.0e-12F { 0.0F } else { k00 / det }
    let inv_mass_x = joint_inv(k00)
    let inv_mass_y = joint_inv(k11)
    let inv_mass_ang = if denom_ang <= 1.0e-12F {
      0.0F
    } else {
      1.0F / denom_ang
    }
    let coupled_axes = joint.data.coupled_axes()
    let coupled_lin = coupled_axes.contains(@dynamics.JointAxesMask::lin_axes())
    let motor_axes = joint.data.motor_axes()
    let motor_enabled = motor_axes.contains(@dynamics.JointAxesMask::ang_z())
    let mut motor_lin_x_enabled = motor_axes.contains(
      @dynamics.JointAxesMask::lin_x(),
    )
    let mut motor_lin_y_enabled = motor_axes.contains(
      @dynamics.JointAxesMask::lin_y(),
    )
    let motors = joint.data.motors()
    let motor = motors.ang_z
    let motor_lin_x = motors.lin_x
    let motor_lin_y = motors.lin_y
    let limit_axes = joint.data.limit_axes()
    let limits_all = joint.data.limits()

    // Coupled linear motor (distance motor). The coupled solver reads settings from LinX.
    let mut coupled_motor_enabled = false
    let mut coupled_motor = motors.lin_x
    let mut coupled_motor_impulse = 0.0F
    if coupled_lin && motor_axes.contains(@dynamics.JointAxesMask::lin_x()) {
      coupled_motor_enabled = true
      coupled_motor = motors.lin_x
      coupled_motor_impulse = motors.lin_x.impulse() * warmstart
      motor_lin_x_enabled = false
      motor_lin_y_enabled = false
    }

    // Coupled linear constraint geometry (used by coupled limits/motors).
    let mut coupled_dir = @core.Vec2::zero()
    let mut coupled_dist = 0.0F
    let mut coupled_inv_mass = 0.0F
    if coupled_lin &&
      (
        limit_axes.contains(@dynamics.JointAxesMask::lin_x()) ||
        motor_axes.contains(@dynamics.JointAxesMask::lin_x())
      ) {
      let delta = world2.sub(world1)
      let dist = delta.length()
      coupled_dist = dist
      coupled_dir = if dist > 1.0e-12F {
        joint_vec2_scale(delta, 1.0F / dist)
      } else {
        @core.Vec2::new(1.0F, 0.0F)
      }
      let rn1 = r1.cross(coupled_dir)
      let rn2 = r2.cross(coupled_dir)
      let denom = inv_mass1 +
        inv_mass2 +
        inv_inertia1 * rn1 * rn1 +
        inv_inertia2 * rn2 * rn2
      coupled_inv_mass = joint_inv(denom)
    }

    // Coupled linear limits (distance limits).
    let mut coupled_limit_enabled = false
    let mut coupled_limit_min = 0.0F
    let mut coupled_limit_max = 0.0F
    let mut coupled_limit_active = 0
    let mut coupled_limit_bias = 0.0F
    let mut coupled_limit_impulse = 0.0F
    if coupled_lin &&
      limit_axes.contains(@dynamics.JointAxesMask::lin_x()) &&
      dt > 0.0F {
      coupled_limit_enabled = true
      coupled_limit_min = limits_all.lin_x.min()
      coupled_limit_max = limits_all.lin_x.max()
      if coupled_dist < coupled_limit_min - 1.0e-6F {
        coupled_limit_active = -1
        coupled_limit_bias = (coupled_dist - coupled_limit_min) *
          JOINT_SOLVER_BAUMGARTE /
          dt
        coupled_limit_impulse = limits_all.lin_x.impulse() * warmstart
      } else if coupled_dist > coupled_limit_max + 1.0e-6F {
        coupled_limit_active = 1
        coupled_limit_bias = (coupled_dist - coupled_limit_max) *
          JOINT_SOLVER_BAUMGARTE /
          dt
        coupled_limit_impulse = limits_all.lin_x.impulse() * warmstart
      }
    }

    // Build unilateral linear limits in joint-frame coordinates.
    let mut lin_limit_x_min = 0.0F
    let mut lin_limit_x_max = 0.0F
    let mut lin_limit_x_active = 0
    let mut lin_limit_x_bias = 0.0F
    let mut lin_limit_x_impulse = 0.0F
    if !coupled_lin &&
      limit_axes.contains(@dynamics.JointAxesMask::lin_x()) &&
      dt > 0.0F {
      let bounds_min = -limits_all.lin_x.max()
      let bounds_max = -limits_all.lin_x.min()
      lin_limit_x_min = bounds_min
      lin_limit_x_max = bounds_max
      let coord = error.dot(basis_x)
      if coord < bounds_min {
        lin_limit_x_active = -1
        lin_limit_x_bias = (coord - bounds_min) * JOINT_SOLVER_BAUMGARTE / dt
        lin_limit_x_impulse = limits_all.lin_x.impulse() * warmstart
      } else if coord > bounds_max {
        lin_limit_x_active = 1
        lin_limit_x_bias = (coord - bounds_max) * JOINT_SOLVER_BAUMGARTE / dt
        lin_limit_x_impulse = limits_all.lin_x.impulse() * warmstart
      }
    }
    let mut lin_limit_y_min = 0.0F
    let mut lin_limit_y_max = 0.0F
    let mut lin_limit_y_active = 0
    let mut lin_limit_y_bias = 0.0F
    let mut lin_limit_y_impulse = 0.0F
    if !coupled_lin &&
      limit_axes.contains(@dynamics.JointAxesMask::lin_y()) &&
      dt > 0.0F {
      let bounds_min = -limits_all.lin_y.max()
      let bounds_max = -limits_all.lin_y.min()
      lin_limit_y_min = bounds_min
      lin_limit_y_max = bounds_max
      let coord = error.dot(basis_y)
      if coord < bounds_min {
        lin_limit_y_active = -1
        lin_limit_y_bias = (coord - bounds_min) * JOINT_SOLVER_BAUMGARTE / dt
        lin_limit_y_impulse = limits_all.lin_y.impulse() * warmstart
      } else if coord > bounds_max {
        lin_limit_y_active = 1
        lin_limit_y_bias = (coord - bounds_max) * JOINT_SOLVER_BAUMGARTE / dt
        lin_limit_y_impulse = limits_all.lin_y.impulse() * warmstart
      }
    }

    // Angular limits are only warmstarted when currently violated.
    let limits_enabled = limit_axes.contains(@dynamics.JointAxesMask::ang_z())
    let limits = limits_all.ang_z
    let rot2 = rb2.rotation()
    let joint_rot2 = rot2.mul(local_rot2)
    let rel_rot = joint_rot1.inverse().mul(joint_rot2)
    let rel_angle = rel_rot.angle()
    let mut limit_impulse = 0.0F
    if limits_enabled && (rel_angle < limits.min() || rel_angle > limits.max()) {
      limit_impulse = limits.impulse() * warmstart
    }
    let mut impulses = joint_vec2_scale(joint.impulses, warmstart)
    if !lock_x {
      impulses = @core.Vec2::new(0.0F, impulses.y)
    }
    if !lock_y {
      impulses = @core.Vec2::new(impulses.x, 0.0F)
    }
    let ang_impulse = if lock_ang {
      joint.ang_impulse * warmstart
    } else {
      0.0F
    }
    let motor_impulse = if motor_enabled {
      motors.ang_z.impulse() * warmstart
    } else {
      0.0F
    }
    let motor_lin_x_impulse = if motor_lin_x_enabled {
      motors.lin_x.impulse() * warmstart
    } else {
      0.0F
    }
    let motor_lin_y_impulse = if motor_lin_y_enabled {
      motors.lin_y.impulse() * warmstart
    } else {
      0.0F
    }
    constraints.push(JointConstraint::{
      handle: Some(handle),
      mb_link: None,
      body1: joint.body1,
      body2: joint.body2,
      local_anchor1,
      local_anchor2,
      local_rot1,
      local_rot2,
      anchor1: world1,
      anchor2: world2,
      basis_x,
      basis_y,
      lock_x,
      lock_y,
      lock_ang,
      bias,
      inv_mass00,
      inv_mass01,
      inv_mass11,
      inv_mass_x,
      inv_mass_y,
      inv_mass_ang,
      motor_enabled,
      motor,
      motor_impulse,
      motor_lin_x_enabled,
      motor_lin_x,
      motor_lin_x_impulse,
      motor_lin_y_enabled,
      motor_lin_y,
      motor_lin_y_impulse,
      limits_enabled,
      limit_min: limits.min(),
      limit_max: limits.max(),
      limit_impulse,
      lin_limit_x_active,
      lin_limit_x_bias,
      lin_limit_x_impulse,
      lin_limit_x_min,
      lin_limit_x_max,
      lin_limit_y_active,
      lin_limit_y_bias,
      lin_limit_y_impulse,
      lin_limit_y_min,
      lin_limit_y_max,
      impulses,
      ang_impulse,
      coupled_dir,
      coupled_dist,
      coupled_inv_mass,
      coupled_limit_enabled,
      coupled_limit_min,
      coupled_limit_max,
      coupled_limit_active,
      coupled_limit_bias,
      coupled_limit_impulse,
      coupled_motor_enabled,
      coupled_motor,
      coupled_motor_impulse,
    })
  }
  let mb_joints = multibody_joints.revolute_joint_descriptors()
  for i in 0..<mb_joints.length() {
    let desc = mb_joints[i]
    if bodies.get(desc.parent) is None || bodies.get(desc.child) is None {
      continue
    }
    let rb1 = bodies.get(desc.parent).unwrap()
    let rb2 = bodies.get(desc.child).unwrap()
    if !rb1.is_enabled() || !rb2.is_enabled() {
      continue
    }
    // Convert to a GenericJoint so we can reuse the same constraint builder.
    let data = @dynamics.GenericJoint::from_revolute(desc.joint)
    if !data.is_enabled() {
      continue
    }
    let local_anchor1 = data.local_anchor1()
    let local_anchor2 = data.local_anchor2()
    let frame1 = data.local_frame1()
    let frame2 = data.local_frame2()
    let local_rot1 = frame1.rotation
    let local_rot2 = frame2.rotation
    let world1 = rb1.position().transform_point(local_anchor1)
    let world2 = rb2.position().transform_point(local_anchor2)
    let error = world1.sub(world2)
    let lock_axes = data.locked_axes()
    let lock_x = lock_axes.contains(@dynamics.JointAxesMask::lin_x())
    let lock_y = lock_axes.contains(@dynamics.JointAxesMask::lin_y())
    let lock_ang = lock_axes.contains(@dynamics.JointAxesMask::ang_z())
    let joint_rot1 = rb1.rotation().mul(local_rot1)
    let (basis_x, basis_y) = if lock_x && lock_y {
      (@core.Vec2::new(1.0F, 0.0F), @core.Vec2::new(0.0F, 1.0F))
    } else {
      let x = joint_rot1.rotate_vec2(@core.Vec2::new(1.0F, 0.0F))
      (x, @core.Vec2::new(-x.y, x.x))
    }
    let bias = if dt > 0.0F {
      @core.Vec2::new(
        if lock_x {
          error.dot(basis_x) * JOINT_SOLVER_BAUMGARTE / dt
        } else {
          0.0F
        },
        if lock_y {
          error.dot(basis_y) * JOINT_SOLVER_BAUMGARTE / dt
        } else {
          0.0F
        },
      )
    } else {
      @core.Vec2::zero()
    }
    let inv_mass1 = if rb1.is_dynamic() &&
      rb1.is_enabled() &&
      !rb1.locked_translations {
      rb1.inv_mass()
    } else {
      0.0F
    }
    let inv_mass2 = if rb2.is_dynamic() &&
      rb2.is_enabled() &&
      !rb2.locked_translations {
      rb2.inv_mass()
    } else {
      0.0F
    }
    let inv_inertia1 = if rb1.is_dynamic() &&
      rb1.is_enabled() &&
      !rb1.locked_rotations {
      rb1.inv_inertia()
    } else {
      0.0F
    }
    let inv_inertia2 = if rb2.is_dynamic() &&
      rb2.is_enabled() &&
      !rb2.locked_rotations {
      rb2.inv_inertia()
    } else {
      0.0F
    }
    let com1 = rb1.world_com()
    let com2 = rb2.world_com()
    let r1 = world1.sub(com1)
    let r2 = world2.sub(com2)
    let rn1x = r1.cross(basis_x)
    let rn2x = r2.cross(basis_x)
    let rn1y = r1.cross(basis_y)
    let rn2y = r2.cross(basis_y)
    let k00 = inv_mass1 +
      inv_mass2 +
      inv_inertia1 * rn1x * rn1x +
      inv_inertia2 * rn2x * rn2x
    let k01 = inv_inertia1 * rn1x * rn1y + inv_inertia2 * rn2x * rn2y
    let k11 = inv_mass1 +
      inv_mass2 +
      inv_inertia1 * rn1y * rn1y +
      inv_inertia2 * rn2y * rn2y
    let denom_ang = inv_inertia1 + inv_inertia2
    let det = k00 * k11 - k01 * k01
    let inv_mass00 = if @core.abs(det) <= 1.0e-12F { 0.0F } else { k11 / det }
    let inv_mass01 = if @core.abs(det) <= 1.0e-12F { 0.0F } else { -k01 / det }
    let inv_mass11 = if @core.abs(det) <= 1.0e-12F { 0.0F } else { k00 / det }
    let inv_mass_x = joint_inv(k00)
    let inv_mass_y = joint_inv(k11)
    let inv_mass_ang = if denom_ang <= 1.0e-12F {
      0.0F
    } else {
      1.0F / denom_ang
    }
    let motor_axes = data.motor_axes()
    let motor_enabled = motor_axes.contains(@dynamics.JointAxesMask::ang_z())
    let motor_lin_x_enabled = motor_axes.contains(
      @dynamics.JointAxesMask::lin_x(),
    )
    let motor_lin_y_enabled = motor_axes.contains(
      @dynamics.JointAxesMask::lin_y(),
    )
    let motors = data.motors()
    let motor = motors.ang_z
    let motor_lin_x = motors.lin_x
    let motor_lin_y = motors.lin_y
    let limit_axes = data.limit_axes()
    let limits_all = data.limits()
    let mut lin_limit_x_min = 0.0F
    let mut lin_limit_x_max = 0.0F
    let mut lin_limit_x_active = 0
    let mut lin_limit_x_bias = 0.0F
    let mut lin_limit_x_impulse = 0.0F
    if limit_axes.contains(@dynamics.JointAxesMask::lin_x()) && dt > 0.0F {
      let bounds_min = -limits_all.lin_x.max()
      let bounds_max = -limits_all.lin_x.min()
      lin_limit_x_min = bounds_min
      lin_limit_x_max = bounds_max
      let coord = error.dot(basis_x)
      if coord < bounds_min {
        lin_limit_x_active = -1
        lin_limit_x_bias = (coord - bounds_min) * JOINT_SOLVER_BAUMGARTE / dt
        lin_limit_x_impulse = limits_all.lin_x.impulse() * warmstart
      } else if coord > bounds_max {
        lin_limit_x_active = 1
        lin_limit_x_bias = (coord - bounds_max) * JOINT_SOLVER_BAUMGARTE / dt
        lin_limit_x_impulse = limits_all.lin_x.impulse() * warmstart
      }
    }
    let mut lin_limit_y_min = 0.0F
    let mut lin_limit_y_max = 0.0F
    let mut lin_limit_y_active = 0
    let mut lin_limit_y_bias = 0.0F
    let mut lin_limit_y_impulse = 0.0F
    if limit_axes.contains(@dynamics.JointAxesMask::lin_y()) && dt > 0.0F {
      let bounds_min = -limits_all.lin_y.max()
      let bounds_max = -limits_all.lin_y.min()
      lin_limit_y_min = bounds_min
      lin_limit_y_max = bounds_max
      let coord = error.dot(basis_y)
      if coord < bounds_min {
        lin_limit_y_active = -1
        lin_limit_y_bias = (coord - bounds_min) * JOINT_SOLVER_BAUMGARTE / dt
        lin_limit_y_impulse = limits_all.lin_y.impulse() * warmstart
      } else if coord > bounds_max {
        lin_limit_y_active = 1
        lin_limit_y_bias = (coord - bounds_max) * JOINT_SOLVER_BAUMGARTE / dt
        lin_limit_y_impulse = limits_all.lin_y.impulse() * warmstart
      }
    }
    let limits_enabled = limit_axes.contains(@dynamics.JointAxesMask::ang_z())
    let limits = limits_all.ang_z
    let rot2 = rb2.rotation()
    let joint_rot2 = rot2.mul(local_rot2)
    let rel_rot = joint_rot1.inverse().mul(joint_rot2)
    let rel_angle = rel_rot.angle()
    let mut limit_impulse = 0.0F
    if limits_enabled && (rel_angle < limits.min() || rel_angle > limits.max()) {
      limit_impulse = limits.impulse() * warmstart
    }
    let mut impulses = joint_vec2_scale(desc.solver_lin_impulses, warmstart)
    if !lock_x {
      impulses = @core.Vec2::new(0.0F, impulses.y)
    }
    if !lock_y {
      impulses = @core.Vec2::new(impulses.x, 0.0F)
    }
    let ang_impulse = 0.0F
    let motor_impulse = if motor_enabled {
      motors.ang_z.impulse() * warmstart
    } else {
      0.0F
    }
    let motor_lin_x_impulse = if motor_lin_x_enabled {
      motors.lin_x.impulse() * warmstart
    } else {
      0.0F
    }
    let motor_lin_y_impulse = if motor_lin_y_enabled {
      motors.lin_y.impulse() * warmstart
    } else {
      0.0F
    }
    constraints.push(JointConstraint::{
      handle: None,
      mb_link: Some(desc.link),
      body1: desc.parent,
      body2: desc.child,
      local_anchor1,
      local_anchor2,
      local_rot1,
      local_rot2,
      anchor1: world1,
      anchor2: world2,
      basis_x,
      basis_y,
      lock_x,
      lock_y,
      lock_ang,
      bias,
      inv_mass00,
      inv_mass01,
      inv_mass11,
      inv_mass_x,
      inv_mass_y,
      inv_mass_ang,
      motor_enabled,
      motor,
      motor_impulse,
      motor_lin_x_enabled,
      motor_lin_x,
      motor_lin_x_impulse,
      motor_lin_y_enabled,
      motor_lin_y,
      motor_lin_y_impulse,
      limits_enabled,
      limit_min: limits.min(),
      limit_max: limits.max(),
      limit_impulse,
      lin_limit_x_active,
      lin_limit_x_bias,
      lin_limit_x_impulse,
      lin_limit_x_min,
      lin_limit_x_max,
      lin_limit_y_active,
      lin_limit_y_bias,
      lin_limit_y_impulse,
      lin_limit_y_min,
      lin_limit_y_max,
      impulses,
      ang_impulse,
      coupled_dir: @core.Vec2::zero(),
      coupled_dist: 0.0F,
      coupled_inv_mass: 0.0F,
      coupled_limit_enabled: false,
      coupled_limit_min: 0.0F,
      coupled_limit_max: 0.0F,
      coupled_limit_active: 0,
      coupled_limit_bias: 0.0F,
      coupled_limit_impulse: 0.0F,
      coupled_motor_enabled: false,
      coupled_motor: motors.lin_x,
      coupled_motor_impulse: 0.0F,
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
    let lin_eq = joint_vec2_scale(c.basis_x, c.impulses.x).add(
      joint_vec2_scale(c.basis_y, c.impulses.y),
    )
    let lin_lim = joint_vec2_scale(c.basis_x, c.lin_limit_x_impulse).add(
      joint_vec2_scale(c.basis_y, c.lin_limit_y_impulse),
    )
    let coupled_limit = if c.coupled_limit_active != 0 {
      joint_vec2_scale(c.coupled_dir, c.coupled_limit_impulse)
    } else {
      @core.Vec2::zero()
    }
    let coupled_motor = if c.coupled_motor_enabled {
      joint_vec2_scale(c.coupled_dir, c.coupled_motor_impulse)
    } else {
      @core.Vec2::zero()
    }
    let lin_motor = joint_vec2_scale(c.basis_x, c.motor_lin_x_impulse).add(
      joint_vec2_scale(c.basis_y, c.motor_lin_y_impulse),
    )
    let impulse = lin_eq
      .add(lin_lim)
      .add(coupled_limit)
      .add(coupled_motor)
      .add(lin_motor)
    joint_apply_impulse_delta_dynamic(bodies, Some(c.body1), impulse, c.anchor1)
    joint_apply_impulse_delta_dynamic(
      bodies,
      Some(c.body2),
      @core.Vec2::new(-impulse.x, -impulse.y),
      c.anchor2,
    )
    if c.inv_mass_ang > 0.0F {
      if c.lock_ang && c.ang_impulse != 0.0F {
        joint_apply_angular_impulse_delta_dynamic(
          bodies,
          Some(c.body1),
          c.ang_impulse,
        )
        joint_apply_angular_impulse_delta_dynamic(
          bodies,
          Some(c.body2),
          -c.ang_impulse,
        )
      }
      if c.motor_enabled && c.motor_impulse != 0.0F {
        joint_apply_angular_impulse_delta_dynamic(
          bodies,
          Some(c.body1),
          c.motor_impulse,
        )
        joint_apply_angular_impulse_delta_dynamic(
          bodies,
          Some(c.body2),
          -c.motor_impulse,
        )
      }
      if c.limits_enabled && c.limit_impulse != 0.0F {
        joint_apply_angular_impulse_delta_dynamic(
          bodies,
          Some(c.body1),
          -c.limit_impulse,
        )
        joint_apply_angular_impulse_delta_dynamic(
          bodies,
          Some(c.body2),
          c.limit_impulse,
        )
      }
    }
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
      // Solve linear equality constraints for locked joint-frame axes.
      if c.lock_x || c.lock_y {
        if c.lock_x && c.lock_y {
          let rhs_x = rv.dot(c.basis_x) + c.bias.x
          let rhs_y = rv.dot(c.basis_y) + c.bias.y
          let lambda_x = -(c.inv_mass00 * rhs_x + c.inv_mass01 * rhs_y)
          let lambda_y = -(c.inv_mass01 * rhs_x + c.inv_mass11 * rhs_y)
          constraints[i].impulses = @core.Vec2::new(
            constraints[i].impulses.x + lambda_x,
            constraints[i].impulses.y + lambda_y,
          )
          let impulse = joint_vec2_scale(c.basis_x, lambda_x).add(
            joint_vec2_scale(c.basis_y, lambda_y),
          )
          joint_apply_impulse_delta_dynamic(
            bodies,
            Some(c.body1),
            impulse,
            c.anchor1,
          )
          joint_apply_impulse_delta_dynamic(
            bodies,
            Some(c.body2),
            @core.Vec2::new(-impulse.x, -impulse.y),
            c.anchor2,
          )
        } else if c.lock_x {
          let rhs = rv.dot(c.basis_x) + c.bias.x
          let lambda = -rhs * c.inv_mass_x
          constraints[i].impulses = @core.Vec2::new(
            constraints[i].impulses.x + lambda,
            constraints[i].impulses.y,
          )
          let impulse = joint_vec2_scale(c.basis_x, lambda)
          joint_apply_impulse_delta_dynamic(
            bodies,
            Some(c.body1),
            impulse,
            c.anchor1,
          )
          joint_apply_impulse_delta_dynamic(
            bodies,
            Some(c.body2),
            @core.Vec2::new(-impulse.x, -impulse.y),
            c.anchor2,
          )
        } else if c.lock_y {
          let rhs = rv.dot(c.basis_y) + c.bias.y
          let lambda = -rhs * c.inv_mass_y
          constraints[i].impulses = @core.Vec2::new(
            constraints[i].impulses.x,
            constraints[i].impulses.y + lambda,
          )
          let impulse = joint_vec2_scale(c.basis_y, lambda)
          joint_apply_impulse_delta_dynamic(
            bodies,
            Some(c.body1),
            impulse,
            c.anchor1,
          )
          joint_apply_impulse_delta_dynamic(
            bodies,
            Some(c.body2),
            @core.Vec2::new(-impulse.x, -impulse.y),
            c.anchor2,
          )
        }
      }

      // Coupled linear limits (distance between anchors).
      if c.coupled_limit_active != 0 && c.coupled_inv_mass > 0.0F {
        let vn = rv.dot(c.coupled_dir)
        let lambda = -(vn + c.coupled_limit_bias) * c.coupled_inv_mass
        let old = constraints[i].coupled_limit_impulse
        let mut new_impulse = old + lambda
        if c.coupled_limit_active < 0 {
          // Min violation: enforce impulse <= 0 (push anchors apart).
          if new_impulse > 0.0F {
            new_impulse = 0.0F
          }
          // Max violation: enforce impulse >= 0 (pull anchors together).
        } else if new_impulse < 0.0F {
          new_impulse = 0.0F
        }
        let applied = new_impulse - old
        constraints[i].coupled_limit_impulse = new_impulse
        if applied != 0.0F {
          let impulse = joint_vec2_scale(c.coupled_dir, applied)
          joint_apply_impulse_delta_dynamic(
            bodies,
            Some(c.body1),
            impulse,
            c.anchor1,
          )
          joint_apply_impulse_delta_dynamic(
            bodies,
            Some(c.body2),
            @core.Vec2::new(-impulse.x, -impulse.y),
            c.anchor2,
          )
        }
      }

      // Linear unilateral limits along joint-frame X/Y.
      if c.lin_limit_x_active != 0 && c.inv_mass_x > 0.0F {
        let rel_v = rv.dot(c.basis_x)
        let lambda = -(rel_v + c.lin_limit_x_bias) * c.inv_mass_x
        let old = constraints[i].lin_limit_x_impulse
        let mut new_impulse = old + lambda
        if c.lin_limit_x_active < 0 {
          if new_impulse < 0.0F {
            new_impulse = 0.0F
          }
        } else if new_impulse > 0.0F {
          new_impulse = 0.0F
        }
        let applied = new_impulse - old
        constraints[i].lin_limit_x_impulse = new_impulse
        if applied != 0.0F {
          let impulse = joint_vec2_scale(c.basis_x, applied)
          joint_apply_impulse_delta_dynamic(
            bodies,
            Some(c.body1),
            impulse,
            c.anchor1,
          )
          joint_apply_impulse_delta_dynamic(
            bodies,
            Some(c.body2),
            @core.Vec2::new(-impulse.x, -impulse.y),
            c.anchor2,
          )
        }
      }
      if c.lin_limit_y_active != 0 && c.inv_mass_y > 0.0F {
        let rel_v = rv.dot(c.basis_y)
        let lambda = -(rel_v + c.lin_limit_y_bias) * c.inv_mass_y
        let old = constraints[i].lin_limit_y_impulse
        let mut new_impulse = old + lambda
        if c.lin_limit_y_active < 0 {
          if new_impulse < 0.0F {
            new_impulse = 0.0F
          }
        } else if new_impulse > 0.0F {
          new_impulse = 0.0F
        }
        let applied = new_impulse - old
        constraints[i].lin_limit_y_impulse = new_impulse
        if applied != 0.0F {
          let impulse = joint_vec2_scale(c.basis_y, applied)
          joint_apply_impulse_delta_dynamic(
            bodies,
            Some(c.body1),
            impulse,
            c.anchor1,
          )
          joint_apply_impulse_delta_dynamic(
            bodies,
            Some(c.body2),
            @core.Vec2::new(-impulse.x, -impulse.y),
            c.anchor2,
          )
        }
      }

      // Coupled linear motor (distance motor).
      if c.coupled_motor_enabled && c.coupled_inv_mass > 0.0F {
        let (v1m, w1m, com1m) = joint_get_body_kinematic_state(bodies, c.body1)
        let (v2m, w2m, com2m) = joint_get_body_kinematic_state(bodies, c.body2)
        let r1m = c.anchor1.sub(com1m)
        let r2m = c.anchor2.sub(com2m)
        let vel1m = v1m.add(joint_cross_z_scalar(w1m, r1m))
        let vel2m = v2m.add(joint_cross_z_scalar(w2m, r2m))
        let rvm = vel1m.sub(vel2m)
        // Match angular motor convention: use relative velocity (body2 - body1).
        let rel_v = -rvm.dot(c.coupled_dir)
        let (erp_inv_dt, cfm_coeff, cfm_gain) = joint_motor_coefficients(
          c.coupled_motor.model,
          dt,
          c.coupled_motor.stiffness,
          c.coupled_motor.damping,
        )
        let coord = c.coupled_dist
        let mut rhs_wo_bias = -c.coupled_motor.target_vel
        if erp_inv_dt != 0.0F {
          let error = coord - c.coupled_motor.target_pos
          rhs_wo_bias = rhs_wo_bias + error * erp_inv_dt
        }
        let dot_jj = 1.0F / c.coupled_inv_mass
        let cfm_gain_total = dot_jj * cfm_coeff + cfm_gain
        let inv_lhs = joint_inv(dot_jj + cfm_gain_total)
        let rhs = rel_v + rhs_wo_bias
        let max_impulse = c.coupled_motor.max_force * dt
        let old = constraints[i].coupled_motor_impulse
        let mut new_impulse = old + inv_lhs * (rhs - cfm_gain_total * old)
        if new_impulse > max_impulse {
          new_impulse = max_impulse
        } else if new_impulse < -max_impulse {
          new_impulse = -max_impulse
        }
        let applied = new_impulse - old
        constraints[i].coupled_motor_impulse = new_impulse
        if applied != 0.0F {
          let impulse = joint_vec2_scale(c.coupled_dir, applied)
          joint_apply_impulse_delta_dynamic(
            bodies,
            Some(c.body1),
            impulse,
            c.anchor1,
          )
          joint_apply_impulse_delta_dynamic(
            bodies,
            Some(c.body2),
            @core.Vec2::new(-impulse.x, -impulse.y),
            c.anchor2,
          )
        }
      }

      // Solve linear motors along joint-frame X/Y.
      if c.motor_lin_x_enabled && c.inv_mass_x > 0.0F {
        let (v1m, w1m, com1m) = joint_get_body_kinematic_state(bodies, c.body1)
        let (v2m, w2m, com2m) = joint_get_body_kinematic_state(bodies, c.body2)
        let r1m = c.anchor1.sub(com1m)
        let r2m = c.anchor2.sub(com2m)
        let vel1m = v1m.add(joint_cross_z_scalar(w1m, r1m))
        let vel2m = v2m.add(joint_cross_z_scalar(w2m, r2m))
        let rvm = vel1m.sub(vel2m)
        // Match angular motor convention: use relative velocity (body2 - body1).
        let rel_v = -rvm.dot(c.basis_x)
        let (erp_inv_dt, cfm_coeff, cfm_gain) = joint_motor_coefficients(
          c.motor_lin_x.model,
          dt,
          c.motor_lin_x.stiffness,
          c.motor_lin_x.damping,
        )
        let delta = c.anchor2.sub(c.anchor1)
        let coord = delta.dot(c.basis_x)
        let mut rhs_wo_bias = -c.motor_lin_x.target_vel
        if erp_inv_dt != 0.0F {
          let error = coord - c.motor_lin_x.target_pos
          rhs_wo_bias = rhs_wo_bias + error * erp_inv_dt
        }
        let dot_jj = 1.0F / c.inv_mass_x
        let cfm_gain_total = dot_jj * cfm_coeff + cfm_gain
        let inv_lhs = joint_inv(dot_jj + cfm_gain_total)
        let rhs = rel_v + rhs_wo_bias
        let max_impulse = c.motor_lin_x.max_force * dt
        let old = constraints[i].motor_lin_x_impulse
        let mut new_impulse = old + inv_lhs * (rhs - cfm_gain_total * old)
        if new_impulse > max_impulse {
          new_impulse = max_impulse
        } else if new_impulse < -max_impulse {
          new_impulse = -max_impulse
        }
        let applied = new_impulse - old
        constraints[i].motor_lin_x_impulse = new_impulse
        if applied != 0.0F {
          let impulse = joint_vec2_scale(c.basis_x, applied)
          joint_apply_impulse_delta_dynamic(
            bodies,
            Some(c.body1),
            impulse,
            c.anchor1,
          )
          joint_apply_impulse_delta_dynamic(
            bodies,
            Some(c.body2),
            @core.Vec2::new(-impulse.x, -impulse.y),
            c.anchor2,
          )
        }
      }
      if c.motor_lin_y_enabled && c.inv_mass_y > 0.0F {
        let (v1m, w1m, com1m) = joint_get_body_kinematic_state(bodies, c.body1)
        let (v2m, w2m, com2m) = joint_get_body_kinematic_state(bodies, c.body2)
        let r1m = c.anchor1.sub(com1m)
        let r2m = c.anchor2.sub(com2m)
        let vel1m = v1m.add(joint_cross_z_scalar(w1m, r1m))
        let vel2m = v2m.add(joint_cross_z_scalar(w2m, r2m))
        let rvm = vel1m.sub(vel2m)
        let rel_v = -rvm.dot(c.basis_y)
        let (erp_inv_dt, cfm_coeff, cfm_gain) = joint_motor_coefficients(
          c.motor_lin_y.model,
          dt,
          c.motor_lin_y.stiffness,
          c.motor_lin_y.damping,
        )
        let delta = c.anchor2.sub(c.anchor1)
        let coord = delta.dot(c.basis_y)
        let mut rhs_wo_bias = -c.motor_lin_y.target_vel
        if erp_inv_dt != 0.0F {
          let error = coord - c.motor_lin_y.target_pos
          rhs_wo_bias = rhs_wo_bias + error * erp_inv_dt
        }
        let dot_jj = 1.0F / c.inv_mass_y
        let cfm_gain_total = dot_jj * cfm_coeff + cfm_gain
        let inv_lhs = joint_inv(dot_jj + cfm_gain_total)
        let rhs = rel_v + rhs_wo_bias
        let max_impulse = c.motor_lin_y.max_force * dt
        let old = constraints[i].motor_lin_y_impulse
        let mut new_impulse = old + inv_lhs * (rhs - cfm_gain_total * old)
        if new_impulse > max_impulse {
          new_impulse = max_impulse
        } else if new_impulse < -max_impulse {
          new_impulse = -max_impulse
        }
        let applied = new_impulse - old
        constraints[i].motor_lin_y_impulse = new_impulse
        if applied != 0.0F {
          let impulse = joint_vec2_scale(c.basis_y, applied)
          joint_apply_impulse_delta_dynamic(
            bodies,
            Some(c.body1),
            impulse,
            c.anchor1,
          )
          joint_apply_impulse_delta_dynamic(
            bodies,
            Some(c.body2),
            @core.Vec2::new(-impulse.x, -impulse.y),
            c.anchor2,
          )
        }
      }

      // Solve angular motor and limits around Z (2D).
      if c.inv_mass_ang > 0.0F {
        let mut w1a = if bodies.get(c.body1) is Some(rb) {
          rb.angvel()
        } else {
          0.0F
        }
        let mut w2a = if bodies.get(c.body2) is Some(rb) {
          rb.angvel()
        } else {
          0.0F
        }
        let mut rel_w = w2a - w1a
        let rot1 = joint_get_body_rotation(bodies, c.body1)
        let rot2 = joint_get_body_rotation(bodies, c.body2)
        let joint_rot1 = rot1.mul(c.local_rot1)
        let joint_rot2 = rot2.mul(c.local_rot2)
        let rel_rot = joint_rot1.inverse().mul(joint_rot2)
        let rel_angle = rel_rot.angle()

        // Angular lock (fixed joint) about Z.
        if c.lock_ang {
          let error = joint_wrap_angle_pi(rel_angle)
          let bias = JOINT_SOLVER_BAUMGARTE / dt * error
          let lambda = (rel_w + bias) * c.inv_mass_ang
          constraints[i].ang_impulse = constraints[i].ang_impulse + lambda
          joint_apply_angular_impulse_delta_dynamic(
            bodies,
            Some(c.body1),
            lambda,
          )
          joint_apply_angular_impulse_delta_dynamic(
            bodies,
            Some(c.body2),
            -lambda,
          )
          // Refresh angular velocity for subsequent motor/limit solves.
          w1a = if bodies.get(c.body1) is Some(rb) { rb.angvel() } else { 0.0F }
          w2a = if bodies.get(c.body2) is Some(rb) { rb.angvel() } else { 0.0F }
          rel_w = w2a - w1a
        }
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
    let (_, _, com1) = joint_get_body_kinematic_state(bodies, c.body1)
    let (_, _, com2) = joint_get_body_kinematic_state(bodies, c.body2)
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
    let joint_rot1 = rb1.rotation().mul(c.local_rot1)
    let basis_x = joint_rot1.rotate_vec2(@core.Vec2::new(1.0F, 0.0F))
    let basis_y = @core.Vec2::new(-basis_x.y, basis_x.x)
    fn apply_linear_correction(
      bodies : @dynamics.RigidBodySet,
      b1h : @dynamics.RigidBodyHandle,
      b2h : @dynamics.RigidBodyHandle,
      axis : @core.Vec2,
      err : @core.Real,
      r1 : @core.Vec2,
      r2 : @core.Vec2,
      inv_m1 : @core.Real,
      inv_m2 : @core.Real,
      inv_i1 : @core.Real,
      inv_i2 : @core.Real,
    ) -> Unit {
      if @core.abs(err) <= 1.0e-8F {
        return
      }
      let rn1 = r1.cross(axis)
      let rn2 = r2.cross(axis)
      let denom = inv_m1 + inv_m2 + inv_i1 * rn1 * rn1 + inv_i2 * rn2 * rn2
      let inv = joint_inv(denom)
      if inv == 0.0F {
        return
      }
      // Solve for a positional impulse along `axis` that reduces `err` (scaled by Baumgarte).
      let lambda = -err * JOINT_SOLVER_BAUMGARTE * inv
      let impulse = joint_vec2_scale(axis, lambda)
      if bodies.get_mut(b1h) is Some(b1) {
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
      if bodies.get_mut(b2h) is Some(b2) {
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

    if c.lock_x && c.lock_y {
      // Use a coupled 2x2 solve for full anchor coincidence (better stability for joint grids).
      let k00 = inv_m1 + inv_m2 + inv_i1 * r1.y * r1.y + inv_i2 * r2.y * r2.y
      let k01 = -inv_i1 * r1.x * r1.y - inv_i2 * r2.x * r2.y
      let k11 = inv_m1 + inv_m2 + inv_i1 * r1.x * r1.x + inv_i2 * r2.x * r2.x
      let det = k00 * k11 - k01 * k01
      if @core.abs(det) > 1.0e-12F {
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
    } else {
      if c.lock_x {
        apply_linear_correction(
          bodies,
          c.body1,
          c.body2,
          basis_x,
          error.dot(basis_x),
          r1,
          r2,
          inv_m1,
          inv_m2,
          inv_i1,
          inv_i2,
        )
      }
      if c.lock_y {
        apply_linear_correction(
          bodies,
          c.body1,
          c.body2,
          basis_y,
          error.dot(basis_y),
          r1,
          r2,
          inv_m1,
          inv_m2,
          inv_i1,
          inv_i2,
        )
      }
    }
    // Linear limit corrections (if bounds were enabled at build time).
    if c.lin_limit_x_min != 0.0F || c.lin_limit_x_max != 0.0F {
      let coord = error.dot(basis_x)
      let mut err = 0.0F
      if coord < c.lin_limit_x_min {
        err = coord - c.lin_limit_x_min
      } else if coord > c.lin_limit_x_max {
        err = coord - c.lin_limit_x_max
      }
      if err != 0.0F {
        apply_linear_correction(
          bodies,
          c.body1,
          c.body2,
          basis_x,
          err,
          r1,
          r2,
          inv_m1,
          inv_m2,
          inv_i1,
          inv_i2,
        )
      }
    }
    if c.lin_limit_y_min != 0.0F || c.lin_limit_y_max != 0.0F {
      let coord = error.dot(basis_y)
      let mut err = 0.0F
      if coord < c.lin_limit_y_min {
        err = coord - c.lin_limit_y_min
      } else if coord > c.lin_limit_y_max {
        err = coord - c.lin_limit_y_max
      }
      if err != 0.0F {
        apply_linear_correction(
          bodies,
          c.body1,
          c.body2,
          basis_y,
          err,
          r1,
          r2,
          inv_m1,
          inv_m2,
          inv_i1,
          inv_i2,
        )
      }
    }
    if c.coupled_limit_enabled {
      let delta = world2.sub(world1)
      let dist = delta.length()
      let mut err = 0.0F
      if dist < c.coupled_limit_min {
        err = c.coupled_limit_min - dist
      } else if dist > c.coupled_limit_max {
        err = c.coupled_limit_max - dist
      }
      if err != 0.0F {
        let dir = if dist > 1.0e-12F {
          joint_vec2_scale(delta, 1.0F / dist)
        } else {
          @core.Vec2::new(1.0F, 0.0F)
        }
        apply_linear_correction(
          bodies,
          c.body1,
          c.body2,
          dir,
          err,
          r1,
          r2,
          inv_m1,
          inv_m2,
          inv_i1,
          inv_i2,
        )
      }
    }
    if c.lock_ang && inv_i1 + inv_i2 > 1.0e-12F {
      let rot2 = rb2.rotation()
      let joint_rot2 = rot2.mul(c.local_rot2)
      let rel_rot = joint_rot1.inverse().mul(joint_rot2)
      let rel_angle = joint_wrap_angle_pi(rel_rot.angle())
      if @core.abs(rel_angle) > 1.0e-6F {
        let denom = inv_i1 + inv_i2
        let corr = rel_angle * JOINT_SOLVER_BAUMGARTE / denom
        if bodies.get_mut(c.body1) is Some(b1) {
          if b1.is_dynamic() && b1.is_enabled() && !b1.locked_rotations {
            b1.set_rotation(
              @core.Rot2::from_angle(b1.rotation().angle() + corr * inv_i1),
              true,
            )
            |> ignore
          }
        }
        if bodies.get_mut(c.body2) is Some(b2) {
          if b2.is_dynamic() && b2.is_enabled() && !b2.locked_rotations {
            b2.set_rotation(
              @core.Rot2::from_angle(b2.rotation().angle() - corr * inv_i2),
              true,
            )
            |> ignore
          }
        }
      }
    }
  }
}

///|
fn writeback_joint_impulses(
  impulse_joints : @dynamics.ImpulseJointSet,
  multibody_joints : @dynamics.MultibodyJointSet,
  constraints : Array[JointConstraint],
) -> Unit {
  for i in 0..<constraints.length() {
    let c = constraints[i]
    if c.handle is Some(handle) {
      let motor_lin_x = if c.coupled_motor_enabled {
        c.coupled_motor_impulse
      } else {
        c.motor_lin_x_impulse
      }
      let motor_lin_y = if c.coupled_motor_enabled {
        0.0F
      } else {
        c.motor_lin_y_impulse
      }
      let limit_lin_x = if c.coupled_limit_enabled {
        c.coupled_limit_impulse
      } else {
        c.lin_limit_x_impulse
      }
      let limit_lin_y = if c.coupled_limit_enabled {
        0.0F
      } else {
        c.lin_limit_y_impulse
      }
      impulse_joints.set_solver_impulses(
        handle,
        c.impulses,
        c.ang_impulse,
        motor_lin_x,
        motor_lin_y,
        c.motor_impulse,
        limit_lin_x,
        limit_lin_y,
        c.limit_impulse,
      )
    } else if c.mb_link is Some(link) {
      multibody_joints.set_revolute_joint_solver_impulses(
        link,
        c.impulses,
        c.motor_impulse,
        c.limit_impulse,
      )
    }
  }
}
