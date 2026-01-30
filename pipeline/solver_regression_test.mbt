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
test "solver regression: stack stability + deterministic stepping" {
  fn simulate() -> Array[@core.Vec2] {
    let pipeline = PhysicsPipeline::new()
    let broad_phase = @collision.BroadPhaseBvh::new()
    let narrow_phase = @collision.NarrowPhase::new()
    let bodies = @dynamics.RigidBodySet::new()
    let colliders = @collision.ColliderSet::new()
    let impulse_joints = @dynamics.ImpulseJointSet::new()
    let multibody_joints = @dynamics.MultibodyJointSet::new()
    let islands = @dynamics.IslandManager::new()
    let hooks = PhysicsHooks::new()
    let events = EventHandler::new()
    let gravity = @core.Vec2::new(0.0F, -9.81F)
    let params = IntegrationParameters::default()

    // Ground.
    let ground = bodies.insert(
      @dynamics.RigidBodyBuilder::fixed()
      .translation(@core.Vec2::new(0.0F, -0.1F))
      .build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder::cuboid(10.0F, 0.1F).friction(1.0F).build(),
      ground,
      bodies,
    )
    |> ignore

    // Stack of boxes.
    let n = 8
    let rad = 0.2F
    let gap = 0.02F
    let handles : Array[@dynamics.RigidBodyHandle] = []
    for i in 0..<n {
      let y = 0.2F + Float::from_int(i) * (rad * 2.0F + gap)
      let h = bodies.insert(
        @dynamics.RigidBodyBuilder::dynamic()
        .translation(@core.Vec2::new(0.0F, y))
        .can_sleep(false)
        .build(),
      )
      handles.push(h)
      colliders.insert_with_parent(
        @collision.ColliderBuilder::cuboid(rad, rad)
        .density(10.0F)
        .friction(0.8F)
        .build(),
        h,
        bodies,
      )
      |> ignore
    }

    // Step forward.
    for _ in 0..<240 {
      pipeline.step(
        gravity,
        params,
        islands,
        broad_phase,
        narrow_phase,
        bodies,
        colliders,
        impulse_joints,
        multibody_joints,
        @dynamics.CCDSolver::new(),
        hooks,
        events,
      )
    }

    // Collect final positions.
    let out : Array[@core.Vec2] = []
    for i in 0..<handles.length() {
      let h = handles[i]
      if bodies.get(h) is Some(rb) {
        out.push(rb.translation())
      } else {
        out.push(@core.Vec2::zero())
      }
    }
    out
  }

  let a = simulate()
  let b = simulate()
  inspect(a.length() == b.length(), content="true")

  // Determinism: same initial state -> same end state.
  let eps = 1.0e-6F
  let eps2 = eps * eps
  for i in 0..<a.length() {
    let da = a[i]
    let db = b[i]
    let dx = da.x - db.x
    let dy = da.y - db.y
    inspect(dx * dx + dy * dy <= eps2, content="true")
    inspect(da.x == da.x && da.y == da.y, content="true")
  }

  // Basic stack sanity: the top box shouldn't fall through the ground.
  if a.length() > 0 {
    let top = a[a.length() - 1]
    inspect(top.y > 1.0F, content="true")
  } else {
    inspect(false, content="true")
  }
}
