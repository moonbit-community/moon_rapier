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
