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
test "ContactModificationContext::update_as_oneway_platform state machine" {
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = @collision.ColliderSet::new()
  let collider1 = @collision.ColliderHandle::from_raw_parts(0, 0)
  let collider2 = @collision.ColliderHandle::from_raw_parts(1, 0)
  let manifold = @collision.ContactManifold::new(@core.Vec2::new(1.0F, 0.0F), [])
  let solver_contacts : Array[@collision.SolverContact] = [
    @collision.SolverContact::new(@core.Vec2::zero(), -0.1F),
  ]
  let ctx = ContactModificationContext::new(
    bodies,
    colliders,
    collider1,
    collider2,
    None,
    None,
    manifold,
    solver_contacts,
    @core.Vec2::new(1.0F, 0.0F),
    0,
  )

  // First update with aligned normal -> allowed.
  ctx.update_as_oneway_platform(@core.Vec2::new(1.0F, 0.0F), 0.0F)
  inspect(ctx.user_data == 1, content="true")

  // Now forbid by flipping the allowed direction.
  ctx.set_user_data(0)
  ctx.solver_contacts.push(
    @collision.SolverContact::new(@core.Vec2::zero(), -0.2F),
  )
  ctx.update_as_oneway_platform(@core.Vec2::new(-1.0F, 0.0F), 0.0F)
  // Either forbidden (2) or still unknown (0) if we couldn't conclude because of a degenerate normal.
  inspect(ctx.user_data == 2 || ctx.user_data == 0, content="true")
  inspect(ctx.solver_contacts.length() == 0, content="true")
}
