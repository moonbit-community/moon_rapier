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
test "solver bodies copy_from captures pose + velocity (whitebox)" {
  let bodies_set = @dynamics.RigidBodySet::new()
  let handle = bodies_set.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(1.0F, 2.0F))
    .linvel(@core.Vec2::new(3.0F, -4.0F))
    .angvel(5.0F)
    .build(),
  )
  let rb = match bodies_set.get(handle) {
    Some(b) => b
    None => return
  }
  let bodies = SolverBodies::default()
  bodies.resize(1)
  bodies.copy_from(0, rb)
  inspect(bodies.vels[0].linear.x, content="3")
  inspect(bodies.vels[0].linear.y, content="-4")
  inspect(bodies.vels[0].angular, content="5")
}
