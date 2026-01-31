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
test "rigid_body3d integrates gravity (semi-implicit Euler)" {
  let gravity = @core.Vec3::new(0.0F, -9.81F, 0.0F)
  let rb = RigidBodyBuilder3D::dynamic()
    .translation(@core.Vec3::new(0.0F, 0.0F, 0.0F))
    .linvel(@core.Vec3::zero())
    .build()
  rb.integrate(gravity, 1.0F)
  inspect(@core.abs(rb.linvel().y - -9.81F) < 1.0e-6F, content="true")
  inspect(@core.abs(rb.translation().y - -9.81F) < 1.0e-6F, content="true")
}

///|
test "rigid_body3d integrates angular velocity into quaternion" {
  let rb = RigidBodyBuilder3D::dynamic()
    .rotation(@core.Quat::identity())
    .angvel(@core.Vec3::new(0.0F, @core.pi(), 0.0F))
    .build()
  rb.integrate(@core.Vec3::zero(), 1.0F)
  let v = @core.Vec3::new(1.0F, 0.0F, 0.0F)
  let r = rb.rotation().rotate_vec3(v)
  inspect(@core.abs(r.x - -1.0F) < 1.0e-5F, content="true")
  inspect(@core.abs(r.y - 0.0F) < 1.0e-5F, content="true")
  inspect(@core.abs(r.z - 0.0F) < 1.0e-5F, content="true")
}
