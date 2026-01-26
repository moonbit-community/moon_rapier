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
/// Ported from rapier/src/dynamics/joint/revolute_joint.rs (test_revolute_joint_angle).
test "revolute joint angle" {
  let revolute = RevoluteJointBuilder::new().build()
  let rot1 = @core.rotation_from_angle(1.0F)
  let steps = 100
  let steps_real = Float::from_double(steps.to_double())
  for step_index in 1..<steps {
    let step_real = Float::from_double(step_index.to_double())
    let delta = -@core.pi() + step_real * @core.two_pi() / steps_real
    let rot2 = @core.rotation_from_angle(1.0F + delta)
    let angle = revolute.angle(rot1, rot2)
    let diff = @core.abs(angle - delta)
    inspect(diff < 1.0e-5F, content="true")
  }
  let boundary_deltas = [-@core.pi(), @core.pi()]
  for delta in boundary_deltas {
    let rot2 = @core.rotation_from_angle(1.0F + delta)
    let angle = revolute.angle(rot1, rot2)
    let diff = @core.abs(@core.abs(angle) - @core.abs(delta))
    inspect(diff < 1.0e-2F, content="true")
  }
}
