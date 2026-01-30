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
/// Ported from rapier/src/dynamics/joint/revolute_joint.rs (test_revolute_joint_angle) for dim3.
test "revolute joint3 angle" {
  let revolute = RevoluteJoint3Builder::new(@core.Vec3::new(0.0F, 1.0F, 0.0F)).build()
  let rot1 = @core.rotation_from_scaled_axis(@core.Vec3::new(0.0F, 1.0F, 0.0F))
  let steps = 100
  let steps_real = Float::from_double(steps.to_double())

  // The -pi and pi values are checked separately.
  for step_index in 1..<steps {
    let step_real = Float::from_double(step_index.to_double())
    let delta = -@core.pi() + step_real * @core.two_pi() / steps_real
    let rot2 = @core.rotation_from_scaled_axis(
      @core.Vec3::new(0.0F, 1.0F + delta, 0.0F),
    )
    let angle = revolute.angle(rot1, rot2)
    inspect(@core.abs(angle - delta) < 1.0e-5F, content="true")
  }

  // Check the special case for -pi and pi that may return an angle with a flipped sign.
  for delta in [-@core.pi(), @core.pi()] {
    let rot2 = @core.rotation_from_scaled_axis(
      @core.Vec3::new(0.0F, 1.0F + delta, 0.0F),
    )
    let angle = revolute.angle(rot1, rot2)
    inspect(
      @core.abs(@core.abs(angle) - @core.abs(delta)) < 1.0e-2F,
      content="true",
    )
  }
}
