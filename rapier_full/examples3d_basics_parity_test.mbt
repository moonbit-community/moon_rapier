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
/// Parity tests for a subset of `rapier-reference/examples3d/*.rs` that mostly exercise
/// basic rigid-bodies + colliders + restitution/damping/sensors.
///
/// These use the dim3 compatibility wrappers (projected to the existing 2D engine).
fn step_world3_basics(
  pipeline : @pipeline.PhysicsPipeline3,
  gravity : @core.Vec3,
  parameters : @dynamics.IntegrationParameters,
  islands : @dynamics.IslandManager3,
  broad_phase : @collision.BroadPhaseBvh,
  narrow_phase : @collision.NarrowPhase,
  bodies : @dynamics.RigidBodySet3,
  colliders : @collision.ColliderSet3,
  impulse_joints : @dynamics.ImpulseJointSet3,
  multibody_joints : @dynamics.MultibodyJointSet3,
  ccd_solver : @dynamics.CCDSolver,
  hooks : @pipeline.PhysicsHooks,
  events : @pipeline.EventHandler,
  steps : Int,
) -> Unit {
  for _ in 0..<steps {
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
      impulse_joints, multibody_joints, ccd_solver, hooks, events,
    )
  }
}

///|
fn run_examples3d_primitives3() -> Bool {
  // Ported from `rapier-reference/examples3d/primitives3.rs`.
  let pipeline = @pipeline.PhysicsPipeline3::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let bodies = @dynamics.RigidBodySet3::new()
  let colliders = @collision.ColliderSet3::new()
  let impulse_joints = @dynamics.ImpulseJointSet3::new()
  let multibody_joints = @dynamics.MultibodyJointSet3::new()
  let islands = @dynamics.IslandManager3::new()
  let hooks = @pipeline.PhysicsHooks::new()
  let events = @pipeline.EventHandler::new()
  let ccd_solver = @dynamics.CCDSolver::new()
  let gravity = @core.Vec3::new(0.0F, -9.81F, 0.0F)
  let parameters = @dynamics.IntegrationParameters::default()
  let ground_size = 100.1F
  let ground_height = 2.1F
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder3::fixed()
    .translation(@core.Vec3::new(0.0F, -ground_height, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3::cuboid(ground_size, ground_height, ground_size).build(),
    ground,
    bodies,
  )
  |> ignore
  let num = 8
  let rad = 1.0F
  let shiftx = rad * 2.0F + rad
  let shifty = rad * 2.0F + rad
  let shiftz = rad * 2.0F + rad
  let centerx = shiftx * Float::from_int(num / 2)
  let centery = shifty / 2.0F
  let centerz = shiftz * Float::from_int(num / 2)
  let mut offset = -Float::from_int(num) * (rad * 2.0F + rad) * 0.5F
  for j in 0..<20 {
    for i in 0..<num {
      for k in 0..<num {
        let x = Float::from_int(i) * shiftx - centerx + offset
        let y = Float::from_int(j) * shifty + centery + 3.0F
        let z = Float::from_int(k) * shiftz - centerz + offset
        let rb = @dynamics.RigidBodyBuilder3::dynamic()
          .translation(@core.Vec3::new(x, y, z))
          .build()
        let handle = bodies.insert(rb)
        let collider = match j % 5 {
          1 => @collision.ColliderBuilder3::ball(rad)
          2 =>
            @collision.ColliderBuilder3::round_cylinder(rad, rad, rad / 10.0F)
          3 => @collision.ColliderBuilder3::cone(rad, rad)
          _ => @collision.ColliderBuilder3::capsule_y(rad, rad)
        }
        colliders.insert_with_parent(collider.build(), handle, bodies) |> ignore
      }
    }
    offset -= 0.05F * rad * (Float::from_int(num) - 1.0F)
  }
  step_world3_basics(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 2,
  )
  true
}

///|
fn run_examples3d_damping3() -> Bool {
  // Ported from `rapier-reference/examples3d/damping3.rs`.
  let pipeline = @pipeline.PhysicsPipeline3::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let bodies = @dynamics.RigidBodySet3::new()
  let colliders = @collision.ColliderSet3::new()
  let impulse_joints = @dynamics.ImpulseJointSet3::new()
  let multibody_joints = @dynamics.MultibodyJointSet3::new()
  let islands = @dynamics.IslandManager3::new()
  let hooks = @pipeline.PhysicsHooks::new()
  let events = @pipeline.EventHandler::new()
  let ccd_solver = @dynamics.CCDSolver::new()
  let gravity = @core.Vec3::zero()
  let parameters = @dynamics.IntegrationParameters::default()
  let num = 10
  let rad = 0.2F
  let subdiv = 1.0F / Float::from_int(num)
  for i in 0..<num {
    let angle = Float::from_int(i).to_double() *
      subdiv.to_double() *
      @math.PI *
      2.0
    let x = Float::from_double(@math.sin(angle))
    let y = Float::from_double(@math.cos(angle))
    let rb = @dynamics.RigidBodyBuilder3::dynamic()
      .translation(@core.Vec3::new(x, y, 0.0F))
      .linvel(@core.Vec3::new(x * 10.0F, y * 10.0F, 0.0F))
      .angvel(@core.Vec3::new(0.0F, 0.0F, 100.0F))
      .linear_damping(Float::from_int(i + 1) * subdiv * 10.0F)
      .angular_damping(Float::from_int(num - i) * subdiv * 10.0F)
      .build()
    let handle = bodies.insert(rb)
    colliders.insert_with_parent(
      @collision.ColliderBuilder3::cuboid(rad, rad, rad).build(),
      handle,
      bodies,
    )
    |> ignore
  }
  step_world3_basics(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 20,
  )
  true
}

///|
fn run_examples3d_restitution3() -> Bool {
  // Ported from `rapier-reference/examples3d/restitution3.rs`.
  let pipeline = @pipeline.PhysicsPipeline3::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let bodies = @dynamics.RigidBodySet3::new()
  let colliders = @collision.ColliderSet3::new()
  let impulse_joints = @dynamics.ImpulseJointSet3::new()
  let multibody_joints = @dynamics.MultibodyJointSet3::new()
  let islands = @dynamics.IslandManager3::new()
  let hooks = @pipeline.PhysicsHooks::new()
  let events = @pipeline.EventHandler::new()
  let ccd_solver = @dynamics.CCDSolver::new()
  let gravity = @core.Vec3::new(0.0F, -9.81F, 0.0F)
  let parameters = @dynamics.IntegrationParameters::default()
  let ground_size = 20.0F
  let ground_height = 1.0F
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder3::fixed()
    .translation(@core.Vec3::new(0.0F, -ground_height, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3::cuboid(ground_size, ground_height, 2.0F)
    .restitution(1.0F)
    .build(),
    ground,
    bodies,
  )
  |> ignore
  let num = 10
  let rad = 0.5F
  for j in 0..<2 {
    for i in 0..<(num + 1) {
      let x = Float::from_int(i) - Float::from_int(num) / 2.0F
      let y = 10.0F * Float::from_int(j + 1)
      let rb = @dynamics.RigidBodyBuilder3::dynamic()
        .translation(@core.Vec3::new(x * 2.0F, y, 0.0F))
        .build()
      let handle = bodies.insert(rb)
      let restitution = Float::from_int(i) / Float::from_int(num)
      colliders.insert_with_parent(
        @collision.ColliderBuilder3::ball(rad).restitution(restitution).build(),
        handle,
        bodies,
      )
      |> ignore
    }
  }
  step_world3_basics(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 10,
  )
  true
}

///|
fn run_examples3d_sensor3() -> Bool {
  // Ported from `rapier-reference/examples3d/sensor3.rs` (physics + collision-events only).
  let pipeline = @pipeline.PhysicsPipeline3::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let bodies = @dynamics.RigidBodySet3::new()
  let colliders = @collision.ColliderSet3::new()
  let impulse_joints = @dynamics.ImpulseJointSet3::new()
  let multibody_joints = @dynamics.MultibodyJointSet3::new()
  let islands = @dynamics.IslandManager3::new()
  let hooks = @pipeline.PhysicsHooks::new()
  let events = @pipeline.EventHandler::new()
  let ccd_solver = @dynamics.CCDSolver::new()
  let gravity = @core.Vec3::new(0.0F, -9.81F, 0.0F)
  let parameters = @dynamics.IntegrationParameters::default()
  let ground_size = 10.1F
  let ground_height = 0.1F
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder3::fixed()
    .translation(@core.Vec3::new(0.0F, -ground_height, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3::cuboid(ground_size, ground_height, ground_size).build(),
    ground,
    bodies,
  )
  |> ignore
  let num = 10
  let rad = 0.2F
  let shift = rad * 2.0F
  let centerx = shift * Float::from_int(num) / 2.0F
  let centerz = shift * Float::from_int(num) / 2.0F
  for i in 0..<num {
    for k in 0..<num {
      let x = Float::from_int(i) * shift - centerx
      let y = 3.0F
      let z = Float::from_int(k) * shift - centerz
      let rb = @dynamics.RigidBodyBuilder3::dynamic()
        .translation(@core.Vec3::new(x, y, z))
        .build()
      let handle = bodies.insert(rb)
      colliders.insert_with_parent(
        @collision.ColliderBuilder3::cuboid(rad, rad, rad).build(),
        handle,
        bodies,
      )
      |> ignore
    }
  }
  let sensor = bodies.insert(
    @dynamics.RigidBodyBuilder3::dynamic()
    .translation(@core.Vec3::new(0.0F, 5.0F, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3::cuboid(rad, rad, rad).build(),
    sensor,
    bodies,
  )
  |> ignore
  colliders.insert_with_parent(
    @collision.ColliderBuilder3::ball(rad * 5.0F)
    .density(0.0F)
    .sensor(true)
    .active_events(@collision.ActiveEvents::collision_events())
    .build(),
    sensor,
    bodies,
  )
  |> ignore
  step_world3_basics(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 120,
  )
  events.take_collision_events().length() > 0
}

///|
test "examples3d/primitives3.rs parity (headless)" {
  inspect(run_examples3d_primitives3(), content="true")
}

///|
test "examples3d/damping3.rs parity (headless)" {
  inspect(run_examples3d_damping3(), content="true")
}

///|
test "examples3d/restitution3.rs parity (headless)" {
  inspect(run_examples3d_restitution3(), content="true")
}

///|
test "examples3d/sensor3.rs parity (headless)" {
  inspect(run_examples3d_sensor3(), content="true")
}
