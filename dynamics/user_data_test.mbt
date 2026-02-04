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
test "rigid body user_data128 roundtrip through RigidBodySet serialize/deserialize" {
  let bodies = RigidBodySet::new()
  let data = @core.UserData128::from_parts(42UL, 99UL)
  let h = bodies.insert(RigidBodyBuilder::dynamic().user_data128(data).build())
  let snapshot = bodies.serialize()
  let bodies2 = RigidBodySet::deserialize(snapshot)
  if bodies2.get(h) is Some(rb) {
    inspect(rb.user_data128().equals(data), content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "rigid body 3D user_data128 stored on build" {
  let data = @core.UserData128::from_parts(1UL, 2UL)
  let rb = RigidBodyBuilder3D::dynamic().user_data128(data).build()
  inspect(rb.user_data128().equals(data), content="true")
}
