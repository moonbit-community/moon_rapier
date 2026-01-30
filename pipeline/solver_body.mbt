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
/// Internal solver-side views of rigid bodies.
///
/// Ported from `rapier-reference/src/dynamics/solver/solver_body.rs` (scalar, non-SIMD path).
/// These types are intentionally **not** exported from the `pipeline` package.

///|
priv struct SolverVel {
  mut linear : @core.Vec2
  mut angular : @core.Real
}

///|
fn SolverVel::zero() -> SolverVel {
  { linear: @core.Vec2::zero(), angular: 0.0F }
}

///|
priv struct SolverPose {
  /// World-space translation of the rigid-body center of mass.
  mut translation : @core.Vec2
  /// World-space rotation.
  mut rotation : @core.Rot2
  /// Effective inverse mass (per-axis for dim2).
  mut im : @core.Vec2
  /// Effective world-space inverse inertia (scalar for dim2).
  mut ii : @core.Real
}

///|
fn SolverPose::zero() -> SolverPose {
  {
    translation: @core.Vec2::zero(),
    rotation: @core.Rot2::identity(),
    im: @core.Vec2::zero(),
    ii: 0.0F,
  }
}

///|
priv struct SolverBodies {
  vels : Array[SolverVel]
  poses : Array[SolverPose]
}

///|
fn SolverBodies::default() -> SolverBodies {
  { vels: [], poses: [] }
}

///|
fn SolverBodies::clear(self : SolverBodies) -> Unit {
  self.vels.clear()
  self.poses.clear()
}

///|
fn SolverBodies::len(self : SolverBodies) -> Int {
  self.vels.length()
}

///|
fn SolverBodies::resize(self : SolverBodies, sz : Int) -> Unit {
  let target = if sz < 0 { 0 } else { sz }
  self.vels.clear()
  self.poses.clear()
  for _ in 0..<target {
    self.vels.push(SolverVel::zero())
    self.poses.push(SolverPose::zero())
  }
}

///|
/// Write back solver velocities into the live rigid bodies.
///
/// This mirrors Rapier's "scatter solver vels" step and is required because the sequential impulse
/// solvers operate on `SolverBodies.vels` rather than mutating `RigidBodySet` directly.
fn SolverBodies::writeback_vels(
  self : SolverBodies,
  bodies : @dynamics.RigidBodySet,
) -> Unit {
  let n = self.vels.length()
  for i in 0..<n {
    if bodies.bodies[i] is Some(body) {
      if !body.is_enabled() || body.is_sleeping() {
        continue
      }
      if body.body_type.is_dynamic_or_kinematic() {
        let v = self.vels[i]
        let wake = match body.body_type {
          @dynamics.RigidBodyType::Dynamic => true
          _ => false
        }
        body.set_linvel(v.linear, wake) |> ignore
        body.set_angvel(v.angular, wake) |> ignore
      }
    }
  }
}

///|
/// Copy the current rigid-body velocity/pose into this solver slot.
fn SolverBodies::copy_from(
  self : SolverBodies,
  i : Int,
  rb : @dynamics.RigidBody,
) -> Unit {
  if i < 0 || i >= self.len() {
    return
  }
  let v = self.vels[i]
  let p = self.poses[i]
  v.angular = rb.angvel()
  v.linear = rb.linvel()
  p.rotation = rb.rotation()
  p.translation = rb.world_com()
  if rb.is_dynamic_or_kinematic() {
    let mprops = rb.mass_props()
    p.im = mprops.effective_inv_mass
    p.ii = mprops.effective_world_inv_inertia
  } else {
    p.im = @core.Vec2::zero()
    p.ii = 0.0F
  }
  self.vels[i] = v
  self.poses[i] = p
}
