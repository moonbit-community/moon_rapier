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
test "rigid_body3d: apply_impulse_at_point affects angvel" {
  let bodies = RigidBodySet3D::new()
  let h = bodies.insert(
    RigidBodyBuilder3D::dynamic()
    .translation(@core.Vec3::new(0.0F, 0.0F, 0.0F))
    .build(),
  )
  if bodies.get_mut(h) is Some(rb) {
    // Apply an impulse along +X at a point offset along +Y => should generate a torque about Z.
    rb.apply_impulse_at_point(
      @core.Vec3::new(1.0F, 0.0F, 0.0F),
      @core.Vec3::new(0.0F, 1.0F, 0.0F),
      true,
    )
    let w = rb.angvel()
    inspect(w.z == w.z, content="true")
    inspect(@core.abs(w.z) > 0.0F, content="true")
  } else {
    inspect(false, content="true")
  }
}
