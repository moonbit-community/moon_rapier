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
test "raycast vehicle controller: accelerates on flat ground (2D)" {
  let pipeline = @pipeline.PhysicsPipeline::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = @collision.ColliderSet::new()
  let impulse_joints = @dynamics.ImpulseJointSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()
  let islands = @dynamics.IslandManager::new()
  let hooks = @pipeline.PhysicsHooks::new()
  let events = @pipeline.EventHandler::new()
  let ccd_solver = @dynamics.CCDSolver::new()
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let parameters = @dynamics.IntegrationParameters::default()

  // Ground.
  let ground_height = 0.1F
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, -ground_height))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(50.0F, ground_height).build(),
    ground,
    bodies,
  )
  |> ignore

  // Vehicle chassis.
  let chassis = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(0.0F, 1.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(0.7F, 0.15F).density(30.0F).build(),
    chassis,
    bodies,
  )
  |> ignore
  let tuning = WheelTuning::default()
  let vehicle = DynamicRayCastVehicleController::new(chassis)
  vehicle.add_wheel(
    @core.Vec2::new(0.6F, -0.15F),
    @core.Vec2::new(0.0F, -1.0F),
    0.35F,
    0.15F,
    tuning,
  )
  vehicle.add_wheel(
    @core.Vec2::new(-0.6F, -0.15F),
    @core.Vec2::new(0.0F, -1.0F),
    0.35F,
    0.15F,
    tuning,
  )
  vehicle.set_engine_force(0, 40.0F)
  vehicle.set_engine_force(1, 40.0F)
  let inv_dt = parameters.inv_dt()
  let dt = if inv_dt == 0.0F { 0.0F } else { 1.0F / inv_dt }
  for _ in 0..<240 {
    let filter = @collision.QueryFilter::new().exclude_rigid_body(chassis)
    let queries = @collision.QueryPipelineMut::new(filter, bodies, colliders)
    vehicle.update_vehicle(dt, queries)
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
      impulse_joints, multibody_joints, ccd_solver, hooks, events,
    )
  }

  // Refresh wheel contact info without adding impulses.
  let filter = @collision.QueryFilter::new().exclude_rigid_body(chassis)
  let queries = @collision.QueryPipelineMut::new(filter, bodies, colliders)
  vehicle.update_vehicle(0.0F, queries)
  inspect(vehicle.wheels.length() == 2, content="true")
  inspect(vehicle.wheels[0].raycast_info().is_in_contact, content="true")
  if bodies.get(chassis) is Some(rb) {
    let p = rb.translation()
    inspect(p.x > 0.5F, content="true")
    inspect(p.y > -10.0F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "raycast vehicle controller: climbs a mild slope (2D-adapted)" {
  let pipeline = @pipeline.PhysicsPipeline::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = @collision.ColliderSet::new()
  let impulse_joints = @dynamics.ImpulseJointSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()
  let islands = @dynamics.IslandManager::new()
  let hooks = @pipeline.PhysicsHooks::new()
  let events = @pipeline.EventHandler::new()
  let ccd_solver = @dynamics.CCDSolver::new()
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let parameters = @dynamics.IntegrationParameters::default()

  // Ground.
  let ground_height = 0.1F
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, -ground_height))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(100.0F, ground_height).build(),
    ground,
    bodies,
  )
  |> ignore

  // A slope we can climb.
  colliders.insert(
    @collision.ColliderBuilder::cuboid(2.0F, ground_height)
    .translation(@core.Vec2::new(4.0F, 0.8F))
    .rotation(0.35F)
    .build(),
  )
  |> ignore

  // Vehicle chassis.
  let chassis = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(-3.0F, 0.8F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(0.7F, 0.15F).density(30.0F).build(),
    chassis,
    bodies,
  )
  |> ignore
  // Use a stiffer suspension so the chassis height reacts visibly to the slope.
  let tuning = WheelTuning::default()
    .with_suspension_stiffness(100.0F)
    .with_suspension_compression(10.0F)
    .with_suspension_damping(10.0F)
    .with_max_suspension_travel(2.0F)
  let vehicle = DynamicRayCastVehicleController::new(chassis)
  vehicle.add_wheel(
    @core.Vec2::new(0.6F, -0.15F),
    @core.Vec2::new(0.0F, -1.0F),
    0.35F,
    0.15F,
    tuning,
  )
  vehicle.add_wheel(
    @core.Vec2::new(-0.6F, -0.15F),
    @core.Vec2::new(0.0F, -1.0F),
    0.35F,
    0.15F,
    tuning,
  )
  vehicle.set_engine_force(0, 50.0F)
  vehicle.set_engine_force(1, 50.0F)
  let inv_dt = parameters.inv_dt()
  let dt = if inv_dt == 0.0F { 0.0F } else { 1.0F / inv_dt }
  let mut max_y_on_ramp = -1.0F
  for _ in 0..<480 {
    let filter = @collision.QueryFilter::new().exclude_rigid_body(chassis)
    let queries = @collision.QueryPipelineMut::new(filter, bodies, colliders)
    vehicle.update_vehicle(dt, queries)
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
      impulse_joints, multibody_joints, ccd_solver, hooks, events,
    )
    if bodies.get(chassis) is Some(rb) {
      let p = rb.translation()
      // Only measure after we are in the slope region (to avoid the initial falling phase).
      if p.x > 2.0F && p.x < 8.0F && p.y > max_y_on_ramp {
        max_y_on_ramp = p.y
      }
    }
  }
  if bodies.get(chassis) is Some(rb) {
    let p = rb.translation()
    inspect(p.x > 6.0F, content="true")
    inspect(max_y_on_ramp > 0.35F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "raycast vehicle controller: braking reduces forward speed (2D)" {
  let pipeline = @pipeline.PhysicsPipeline::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = @collision.ColliderSet::new()
  let impulse_joints = @dynamics.ImpulseJointSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()
  let islands = @dynamics.IslandManager::new()
  let hooks = @pipeline.PhysicsHooks::new()
  let events = @pipeline.EventHandler::new()
  let ccd_solver = @dynamics.CCDSolver::new()
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let parameters = @dynamics.IntegrationParameters::default()

  // Ground.
  let ground_height = 0.1F
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, -ground_height))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(50.0F, ground_height).build(),
    ground,
    bodies,
  )
  |> ignore

  // Vehicle chassis.
  let chassis = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(0.0F, 1.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(0.7F, 0.15F).density(30.0F).build(),
    chassis,
    bodies,
  )
  |> ignore
  let tuning = WheelTuning::default()
  let vehicle = DynamicRayCastVehicleController::new(chassis)
  vehicle.add_wheel(
    @core.Vec2::new(0.6F, -0.15F),
    @core.Vec2::new(0.0F, -1.0F),
    0.35F,
    0.15F,
    tuning,
  )
  vehicle.add_wheel(
    @core.Vec2::new(-0.6F, -0.15F),
    @core.Vec2::new(0.0F, -1.0F),
    0.35F,
    0.15F,
    tuning,
  )
  let inv_dt = parameters.inv_dt()
  let dt = if inv_dt == 0.0F { 0.0F } else { 1.0F / inv_dt }

  // Accelerate.
  vehicle.set_engine_force(0, 60.0F)
  vehicle.set_engine_force(1, 60.0F)
  for _ in 0..<180 {
    let filter = @collision.QueryFilter::new().exclude_rigid_body(chassis)
    let queries = @collision.QueryPipelineMut::new(filter, bodies, colliders)
    vehicle.update_vehicle(dt, queries)
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
      impulse_joints, multibody_joints, ccd_solver, hooks, events,
    )
  }
  let speed_before = match bodies.get(chassis) {
    Some(rb) => rb.linvel().x
    None => 0.0F
  }
  inspect(speed_before > 0.5F, content="true")

  // Brake.
  vehicle.set_engine_force(0, 0.0F)
  vehicle.set_engine_force(1, 0.0F)
  vehicle.set_brake(0, 2.0F)
  vehicle.set_brake(1, 2.0F)
  for _ in 0..<180 {
    let filter = @collision.QueryFilter::new().exclude_rigid_body(chassis)
    let queries = @collision.QueryPipelineMut::new(filter, bodies, colliders)
    vehicle.update_vehicle(dt, queries)
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
      impulse_joints, multibody_joints, ccd_solver, hooks, events,
    )
  }
  if bodies.get(chassis) is Some(rb) {
    let speed_after = rb.linvel().x
    inspect(speed_after < speed_before, content="true")
  } else {
    inspect(false, content="true")
  }
}
