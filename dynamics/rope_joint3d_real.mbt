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
/// Rope joint constraint for the real dim3 backend.
///
/// This only enforces a maximum distance between the bodies' centers of mass.

///|
pub struct RopeJoint3DReal {
  body1 : RigidBodyHandle
  body2 : RigidBodyHandle
  max_length : @core.Real
}

///|
pub struct RopeJointSet3DReal {
  joints : Array[RopeJoint3DReal]
}

///|
pub fn RopeJointSet3DReal::new() -> RopeJointSet3DReal {
  { joints: [] }
}

///|
pub fn RopeJointSet3DReal::insert(
  self : RopeJointSet3DReal,
  body1 : RigidBodyHandle,
  body2 : RigidBodyHandle,
  max_length : @core.Real,
) -> Unit {
  self.joints.push({ body1, body2, max_length })
}

///|
pub fn RopeJointSet3DReal::solve(
  self : RopeJointSet3DReal,
  bodies : RigidBodySet3D,
  dt : @core.Real,
) -> Unit {
  dt |> ignore
  for i in 0..<self.joints.length() {
    let j = self.joints[i]
    if bodies.get(j.body1) is Some(b1) && bodies.get(j.body2) is Some(b2) {
      let p1 = b1.world_com()
      let p2 = b2.world_com()
      let d = p2.sub(p1)
      let dist2 = d.length_squared()
      let max_len = j.max_length
      if dist2 <= max_len * max_len {
        continue
      }
      let dist = Float::sqrt(dist2)
      if dist <= 1.0e-6F {
        continue
      }
      let dir = d.scale(1.0F / dist)
      let err = dist - max_len
      let dyn1 = b1.body_type().is_dynamic()
      let dyn2 = b2.body_type().is_dynamic()
      let inv_m1 = if dyn1 { b1.mass_properties().inv_mass } else { 0.0F }
      let inv_m2 = if dyn2 { b2.mass_properties().inv_mass } else { 0.0F }
      let inv_sum = inv_m1 + inv_m2
      if inv_sum <= 0.0F {
        continue
      }
      let w1 = if inv_m1 > 0.0F { inv_m1 / inv_sum } else { 0.0F }
      let w2 = if inv_m2 > 0.0F { inv_m2 / inv_sum } else { 0.0F }

      // Position correction.
      let corr = dir.scale(err)
      if dyn1 {
        b1.set_translation(b1.translation().add(corr.scale(w1)))
        b1.wake_up()
      }
      if dyn2 {
        b2.set_translation(b2.translation().sub(corr.scale(w2)))
        b2.wake_up()
      }

      // Velocity correction: prevent further separation along the rope direction.
      let rel = b2.linvel().sub(b1.linvel()).dot(dir)
      if rel > 0.0F {
        if dyn1 {
          b1.set_linvel(b1.linvel().add(dir.scale(rel * w1)))
        }
        if dyn2 {
          b2.set_linvel(b2.linvel().sub(dir.scale(rel * w2)))
        }
      }
    }
  }
}
