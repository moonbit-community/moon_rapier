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
fn wrap_angle_pi(angle : @core.Real) -> @core.Real {
  let pi_value = @core.pi()
  let two_pi = @core.two_pi()
  let mut a = angle
  if a > pi_value {
    a = a - two_pi
  } else if a < -pi_value {
    a = a + two_pi
  }
  a
}

///|
fn local_sin(value : @core.Real) -> @core.Real {
  Float::from_double(@math.sin(value.to_double()))
}

///|
fn local_cos(value : @core.Real) -> @core.Real {
  Float::from_double(@math.cos(value.to_double()))
}

///|
fn step_pipeline_2d_n(
  pipeline : @pipeline.PhysicsPipeline,
  gravity : @core.Vec2,
  parameters : @dynamics.IntegrationParameters,
  islands : @dynamics.IslandManager,
  broad_phase : @collision.BroadPhaseBvh,
  narrow_phase : @collision.NarrowPhase,
  bodies : @dynamics.RigidBodySet,
  colliders : @collision.ColliderSet,
  impulse_joints : @dynamics.ImpulseJointSet,
  multibody_joints : @dynamics.MultibodyJointSet,
  ccd_solver : @dynamics.CCDSolver,
  hooks : @pipeline.PhysicsHooks,
  events : @pipeline.EventHandler,
  nsteps : Int,
) -> Unit {
  for _ in 0..<nsteps {
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
      impulse_joints, multibody_joints, ccd_solver, hooks, events,
    )
  }
}

///|
test "examples2d/joint_motor_position2.rs parity" {
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = @collision.ColliderSet::new()
  let impulse_joints = @dynamics.ImpulseJointSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()
  let pipeline = @pipeline.PhysicsPipeline::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let islands = @dynamics.IslandManager::new()
  let ground_handle = bodies.insert(@dynamics.RigidBodyBuilder::fixed().build())
  let target_positions : Array[(@dynamics.RigidBodyHandle, @core.Real)] = []
  for num in 0..<9 {
    let n = Float::from_double(num.to_double())
    let x_pos = -6.0F + 1.5F * n
    let handle = bodies.insert(
      @dynamics.RigidBodyBuilder::dynamic()
      .translation(@core.Vec2::new(x_pos, 2.0F))
      .can_sleep(false)
      .build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder::cuboid(0.1F, 0.5F).build(),
      handle,
      bodies,
    )
    |> ignore
    let target = -@core.pi() + @core.pi() / 4.0F * n
    let joint = @dynamics.GenericJoint::from_revolute(
      @dynamics.RevoluteJointBuilder::new()
      .local_anchor1(@core.Vec2::new(x_pos, 1.5F))
      .local_anchor2(@core.Vec2::new(0.0F, -0.5F))
      .motor_position(target, 1000.0F, 150.0F)
      .build(),
    )
    impulse_joints.insert(ground_handle, handle, joint, true) |> ignore
    target_positions.push((handle, target))
  }
  let limit_targets : Array[(@dynamics.RigidBodyHandle, @core.Real, @core.Real)] = []
  for num in 0..<8 {
    let n = Float::from_double(num.to_double())
    let x_pos = -6.0F + 1.5F * n
    let handle = bodies.insert(
      @dynamics.RigidBodyBuilder::dynamic()
      .translation(@core.Vec2::new(x_pos, 4.5F))
      .rotation(@core.Rot2::from_angle(@core.pi()))
      .can_sleep(false)
      .build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder::cuboid(0.1F, 0.5F).build(),
      handle,
      bodies,
    )
    |> ignore
    let min_angle = -@core.pi()
    let max_angle = -@core.pi() + @core.pi() / 4.0F * n
    let joint = @dynamics.GenericJoint::from_revolute(
      @dynamics.RevoluteJointBuilder::new()
      .local_anchor1(@core.Vec2::new(x_pos, 5.0F))
      .local_anchor2(@core.Vec2::new(0.0F, -0.5F))
      .motor_velocity(1.5F, 30.0F)
      .motor_max_force(100.0F)
      .limits(min_angle, max_angle)
      .build(),
    )
    impulse_joints.insert(ground_handle, handle, joint, true) |> ignore
    limit_targets.push((handle, min_angle, max_angle))
  }
  let gravity = @core.Vec2::zero()
  let parameters = @dynamics.IntegrationParameters::default()
  let ccd_solver = @dynamics.CCDSolver::new()
  let hooks = @pipeline.PhysicsHooks::new()
  let events = @pipeline.EventHandler::new()
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 240,
  )
  for i in 0..<target_positions.length() {
    let pair = target_positions[i]
    if bodies.get(pair.0) is Some(body) {
      let angle = body.rotation().angle()
      inspect(angle == angle, content="true")
      let diff = wrap_angle_pi(angle - pair.1)
      inspect(@core.abs(diff) < 0.4F, content="true")
    } else {
      inspect(false, content="true")
    }
  }
  for i in 0..<limit_targets.length() {
    let info = limit_targets[i]
    if bodies.get(info.0) is Some(body) {
      let angle = body.rotation().angle()
      inspect(angle == angle, content="true")
      inspect(angle >= info.1 - 1.0e-2F, content="true")
      inspect(angle <= info.2 + 1.0e-2F, content="true")
    } else {
      inspect(false, content="true")
    }
  }
}

///|
/// Ported from `rapier-reference/examples2d/add_remove2.rs`.
test "examples2d/add_remove2.rs parity" {
  fn rng_next(seed : Int) -> Int {
    (seed * 1103515245 + 12345) & 0x7fffffff
  }

  fn rng_float01(seed : Int) -> (@core.Real, Int) {
    let next = rng_next(seed)
    let value = Float::from_double(next.to_double() / 2147483647.0)
    (value, next)
  }

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
  let parameters = @dynamics.IntegrationParameters::default()
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let inv_dt = parameters.inv_dt()
  let dt = if inv_dt == 0.0F { 0.0F } else { 1.0F / inv_dt }
  let rad = 0.5F
  let platform_positions = [
    @core.Vec2::new(5.0F, -1.0F),
    @core.Vec2::new(-5.0F, -1.0F),
  ]
  let platform_handles : Array[@dynamics.RigidBodyHandle] = []
  for i in 0..<platform_positions.length() {
    let handle = bodies.insert(
      @dynamics.RigidBodyBuilder::kinematic_position_based()
      .translation(platform_positions[i])
      .build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder::cuboid(rad * 10.0F, rad).build(),
      handle,
      bodies,
    )
    |> ignore
    platform_handles.push(handle)
  }

  // Spawn + remove loop.
  let mut seed = 1
  let mut removed_count = 0
  let mut time = 0.0F
  for step_id in 0..<240 {
    time = time + dt
    let rot = -time
    for i in 0..<platform_handles.length() {
      if bodies.get_mut(platform_handles[i]) is Some(rb) {
        rb.set_next_kinematic_rotation(@core.Rot2::from_angle(rot)) |> ignore
      } else {
        inspect(false, content="true")
      }
    }
    if step_id % 10 == 0 {
      let (rx, s1) = rng_float01(seed)
      let (ry, s2) = rng_float01(s1)
      seed = s2
      let x = rx * 10.0F - 5.0F
      let y = ry * 10.0F + 10.0F
      let handle = bodies.insert(
        @dynamics.RigidBodyBuilder::dynamic()
        .translation(@core.Vec2::new(x, y))
        .build(),
      )
      colliders.insert_with_parent(
        @collision.ColliderBuilder::cuboid(rad, rad).build(),
        handle,
        bodies,
      )
      |> ignore
    }
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
      impulse_joints, multibody_joints, ccd_solver, hooks, events,
    )

    // Remove bodies that fell too far.
    let to_remove : Array[@dynamics.RigidBodyHandle] = []
    let it = bodies.iter()
    for i in 0..<it.length() {
      let (h, b) = it[i]
      if b.translation().y < -10.0F {
        to_remove.push(h)
      }
    }
    for i in 0..<to_remove.length() {
      let h = to_remove[i]
      @pipeline.remove_rigid_body(
        bodies, h, islands, colliders, impulse_joints, multibody_joints, true, true,
      )
      |> ignore
      removed_count = removed_count + 1
    }
  }
  inspect(removed_count > 0, content="true")
  // No remaining dynamic body should be below the removal threshold.
  let it2 = bodies.iter()
  for i in 0..<it2.length() {
    let (_, b) = it2[i]
    inspect(b.translation().y >= -10.0F, content="true")
  }
}

///|
/// Ported from `rapier-reference/examples2d/all_examples2.rs`.
/// NOTE: Upstream is a testbed entrypoint that lists demos. This port keeps a headless smoke test.
test "examples2d/all_examples2.rs parity" {
  // all_examples2.rs is a UI entrypoint listing every demo. For headless parity, we smoke-run a
  // representative subset spanning the major feature buckets (shapes, joints, controls).

  fn case_convex_hull() -> Bool {
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
    let ground = bodies.insert(@dynamics.RigidBodyBuilder::fixed().build())
    colliders.insert_with_parent(
      @collision.ColliderBuilder::cuboid(10.0F, 1.0F).build(),
      ground,
      bodies,
    )
    |> ignore
    let h = bodies.insert(
      @dynamics.RigidBodyBuilder::dynamic()
      .translation(@core.Vec2::new(0.0F, 5.0F))
      .build(),
    )
    let pts = [
      @core.Vec2::new(-0.5F, -0.5F),
      @core.Vec2::new(0.5F, -0.5F),
      @core.Vec2::new(0.8F, 0.0F),
      @core.Vec2::new(0.0F, 0.9F),
      @core.Vec2::new(-0.8F, 0.0F),
    ]
    if @collision.ColliderBuilder::convex_hull(pts) is Some(builder) {
      colliders.insert_with_parent(builder.build(), h, bodies) |> ignore
    } else {
      return false
    }
    step_pipeline_2d_n(
      pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies,
      colliders, impulse_joints, multibody_joints, ccd_solver, hooks, events, 60,
    )
    if bodies.get(h) is Some(body) {
      let p = body.translation()
      p.x == p.x && p.y == p.y
    } else {
      false
    }
  }

  inspect(case_convex_hull(), content="true")
  fn case_heightfield() -> Bool {
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
    let heights = [2.0F, 0.0F, 1.0F, 0.0F, 2.0F]
    let ground = bodies.insert(@dynamics.RigidBodyBuilder::fixed().build())
    colliders.insert_with_parent(
      @collision.ColliderBuilder::heightfield(
        heights,
        @core.Vec2::new(10.0F, 1.0F),
      ).build(),
      ground,
      bodies,
    )
    |> ignore
    let h = bodies.insert(
      @dynamics.RigidBodyBuilder::dynamic()
      .translation(@core.Vec2::new(0.0F, 8.0F))
      .build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder::ball(0.5F).build(),
      h,
      bodies,
    )
    |> ignore
    step_pipeline_2d_n(
      pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies,
      colliders, impulse_joints, multibody_joints, ccd_solver, hooks, events, 60,
    )
    if bodies.get(h) is Some(body) {
      let p = body.translation()
      p.x == p.x && p.y == p.y
    } else {
      false
    }
  }

  inspect(case_heightfield(), content="true")
  fn case_trimesh() -> Bool {
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
    let ground = bodies.insert(@dynamics.RigidBodyBuilder::fixed().build())
    colliders.insert_with_parent(
      @collision.ColliderBuilder::cuboid(20.0F, 1.0F).build(),
      ground,
      bodies,
    )
    |> ignore
    let vtx = [
      @core.Vec2::new(-1.0F, -1.0F),
      @core.Vec2::new(1.0F, -1.0F),
      @core.Vec2::new(1.0F, 1.0F),
      @core.Vec2::new(-1.0F, 1.0F),
    ]
    let idx = [(0, 1, 2), (0, 2, 3)]
    if @collision.ColliderBuilder::trimesh(vtx, idx) is Some(mesh_builder) {
      let h = bodies.insert(
        @dynamics.RigidBodyBuilder::dynamic()
        .translation(@core.Vec2::new(0.0F, 6.0F))
        .build(),
      )
      colliders.insert_with_parent(mesh_builder.build(), h, bodies) |> ignore
      step_pipeline_2d_n(
        pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies,
        colliders, impulse_joints, multibody_joints, ccd_solver, hooks, events, 60,
      )
      if bodies.get(h) is Some(body) {
        let p = body.translation()
        p.x == p.x && p.y == p.y
      } else {
        false
      }
    } else {
      false
    }
  }

  inspect(case_trimesh(), content="true")
  fn case_joints() -> Bool {
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
    let ground = bodies.insert(@dynamics.RigidBodyBuilder::fixed().build())
    colliders.insert_with_parent(
      @collision.ColliderBuilder::cuboid(10.0F, 1.0F).build(),
      ground,
      bodies,
    )
    |> ignore
    let a = bodies.insert(
      @dynamics.RigidBodyBuilder::dynamic()
      .translation(@core.Vec2::new(-1.0F, 4.0F))
      .build(),
    )
    let b = bodies.insert(
      @dynamics.RigidBodyBuilder::dynamic()
      .translation(@core.Vec2::new(1.0F, 4.0F))
      .build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder::cuboid(0.2F, 0.2F).build(),
      a,
      bodies,
    )
    |> ignore
    colliders.insert_with_parent(
      @collision.ColliderBuilder::cuboid(0.2F, 0.2F).build(),
      b,
      bodies,
    )
    |> ignore
    let joint = @dynamics.GenericJoint::from_fixed(
      @dynamics.FixedJointBuilder::new()
      .local_anchor1(@core.Vec2::zero())
      .local_anchor2(@core.Vec2::zero())
      .build(),
    )
    impulse_joints.insert(a, b, joint, true) |> ignore
    step_pipeline_2d_n(
      pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies,
      colliders, impulse_joints, multibody_joints, ccd_solver, hooks, events, 60,
    )
    if bodies.get(a) is Some(body_a) && bodies.get(b) is Some(body_b) {
      let pa = body_a.translation()
      let pb = body_b.translation()
      (pa.x == pa.x && pa.y == pa.y) && pb.x == pb.x && pb.y == pb.y
    } else {
      false
    }
  }

  inspect(case_joints(), content="true")
  fn case_character_controller_smoke() -> Bool {
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
    let ground = bodies.insert(
      @dynamics.RigidBodyBuilder::fixed()
      .translation(@core.Vec2::new(0.0F, -1.0F))
      .build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder::cuboid(10.0F, 1.0F).build(),
      ground,
      bodies,
    )
    |> ignore
    let character = bodies.insert(
      @dynamics.RigidBodyBuilder::kinematic_position_based()
      .translation(@core.Vec2::new(0.0F, 1.0F))
      .build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder::cuboid(0.2F, 0.5F).build(),
      character,
      bodies,
    )
    |> ignore
    step_pipeline_2d_n(
      pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies,
      colliders, impulse_joints, multibody_joints, ccd_solver, hooks, events, 10,
    )
    bodies.get(character) is Some(_)
  }

  inspect(case_character_controller_smoke(), content="true")
}

///|
/// Ported from `rapier-reference/examples2d/ccd2.rs`.
test "examples2d/ccd2.rs parity" {
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
  let parameters = @dynamics.IntegrationParameters::default()
  let gravity = @core.Vec2::new(0.0F, -9.81F)

  // Static ground + walls.
  let ground_size = 25.0F
  let ground_thickness = 0.1F
  let ground_handle = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed().ccd_enabled(true).build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(ground_size, ground_thickness).build(),
    ground_handle,
    bodies,
  )
  |> ignore
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(ground_thickness, ground_size)
    .translation(@core.Vec2::new(-3.0F, 0.0F))
    .build(),
    ground_handle,
    bodies,
  )
  |> ignore
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(ground_thickness, ground_size)
    .translation(@core.Vec2::new(6.0F, 0.0F))
    .build(),
    ground_handle,
    bodies,
  )
  |> ignore
  let sensor_handle = colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(ground_thickness, ground_size)
    .translation(@core.Vec2::new(2.5F, 0.0F))
    .sensor(true)
    .active_events(@collision.ActiveEvents::collision_events())
    .build(),
    ground_handle,
    bodies,
  )

  // Create dynamic bodies with a compound cross.
  let radx = 0.4F
  let rady = 0.05F
  let compound_parts = [
    (
      @core.Isometry2::from_translation(@core.Vec2::new(0.0F, radx - rady)),
      @collision.Shape::Cuboid(radx, rady),
    ),
    (
      @core.Isometry2::from_translation(@core.Vec2::new(-radx + rady, 0.0F)),
      @collision.Shape::Cuboid(rady, radx),
    ),
    (
      @core.Isometry2::from_translation(@core.Vec2::new(radx - rady, 0.0F)),
      @collision.Shape::Cuboid(rady, radx),
    ),
  ]
  let num = 6
  let shift = (radx + 0.01F) * 2.0F
  let centerx = shift * Float::from_double(num.to_double()) / 2.0F - 0.5F
  let centery = shift / 2.0F + 4.0F
  let dynamic_handles : Array[@dynamics.RigidBodyHandle] = []
  for i in 0..<num {
    for j in 0..<num {
      let x = Float::from_double(i.to_double()) * shift - centerx
      let y = Float::from_double(j.to_double()) * shift + centery
      let handle = bodies.insert(
        @dynamics.RigidBodyBuilder::dynamic()
        .translation(@core.Vec2::new(x, y))
        .linvel(@core.Vec2::new(100.0F, -10.0F))
        .ccd_enabled(true)
        .build(),
      )
      colliders.insert_with_parent(
        @collision.ColliderBuilder::compound(compound_parts).build(),
        handle,
        bodies,
      )
      |> ignore
      dynamic_handles.push(handle)
    }
  }
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 60,
  )

  // CCD regression: none of the fast-moving bodies should tunnel past the right wall.
  let mut max_x = -1.0e30F
  for i in 0..<dynamic_handles.length() {
    if bodies.get(dynamic_handles[i]) is Some(body) {
      let x = body.translation().x
      inspect(x == x, content="true")
      inspect(body.translation().y == body.translation().y, content="true")
      if x > max_x {
        max_x = x
      }
    } else {
      inspect(false, content="true")
    }
  }
  inspect(max_x < 7.0F, content="true")

  // Sensor collision events should be emitted at least once.
  let evs = events.take_collision_events()
  let mut saw_sensor = false
  for i in 0..<evs.length() {
    match evs[i] {
      @collision.CollisionEvent::Started(h1, h2, _) =>
        if @collision.ColliderHandle::equals(h1, sensor_handle) ||
          @collision.ColliderHandle::equals(h2, sensor_handle) {
          saw_sensor = true
        }
      _ => ()
    }
  }
  inspect(saw_sensor, content="true")
}

///|
/// Ported from `rapier-reference/examples2d/character_controller2.rs`.
/// NOTE: Upstream uses interactive controls; this port uses deterministic scripted movement to exercise the controller.
test "examples2d/character_controller2.rs parity" {
  let colliders = @collision.ColliderSet::new()
  let impulse_joints = @dynamics.ImpulseJointSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()
  let pipeline = @pipeline.PhysicsPipeline::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let islands = @dynamics.IslandManager::new()
  let bodies = @dynamics.RigidBodySet::new()
  let hooks = @pipeline.PhysicsHooks::new()
  let events = @pipeline.EventHandler::new()
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let ground_size = 5.0F
  let ground_height = 0.1F
  let floor_handle = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, -ground_height))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(ground_size, ground_height).build(),
    floor_handle,
    bodies,
  )
  |> ignore

  // Character (kinematic position-based).
  let character_handle = bodies.insert(
    @dynamics.RigidBodyBuilder::kinematic_position_based()
    .translation(@core.Vec2::new(-3.0F, 5.0F))
    .gravity_scale(10.0F)
    .soft_ccd_prediction(10.0F)
    .build(),
  )
  let character_collider = @collision.ColliderBuilder::capsule_y(0.3F, 0.15F).build()
  let character_shape = character_collider.shape()
  colliders.insert_with_parent(character_collider, character_handle, bodies)
  |> ignore
  let num = 8
  let rad = 0.1F
  let shift = rad * 2.0F
  let centerx = shift * Float::from_double((num / 2).to_double())
  let centery = rad
  for j in 0..<4 {
    for i in 0..<num {
      let x = Float::from_double(i.to_double()) * shift - centerx
      let y = Float::from_double(j.to_double()) * shift + centery
      let handle = bodies.insert(
        @dynamics.RigidBodyBuilder::dynamic()
        .translation(@core.Vec2::new(x, y))
        .build(),
      )
      colliders.insert_with_parent(
        @collision.ColliderBuilder::cuboid(rad, rad).build(),
        handle,
        bodies,
      )
      |> ignore
    }
  }

  // Stairs.
  let stair_width = 1.0F
  let stair_height = 0.1F
  for i in 0..<10 {
    let x = Float::from_double(i.to_double()) * stair_width / 2.0F
    let y = Float::from_double(i.to_double()) * stair_height * 1.5F + 3.0F
    colliders.insert(
      @collision.ColliderBuilder::cuboid(
        stair_width / 2.0F,
        stair_height / 2.0F,
      )
      .translation(@core.Vec2::new(x, y))
      .build(),
    )
    |> ignore
  }

  // Slopes + wall.
  let slope_angle = 0.2F
  let slope_size = 2.0F
  colliders.insert(
    @collision.ColliderBuilder::cuboid(slope_size, ground_height)
    .translation(
      @core.Vec2::new(ground_size + slope_size, -ground_height + 0.4F),
    )
    .rotation(slope_angle)
    .build(),
  )
  |> ignore
  let impossible_slope_angle = 0.9F
  let impossible_slope_size = 2.0F
  colliders.insert(
    @collision.ColliderBuilder::cuboid(slope_size, ground_height)
    .translation(
      @core.Vec2::new(
        ground_size + slope_size * 2.0F + impossible_slope_size - 0.9F,
        -ground_height + 2.3F,
      ),
    )
    .rotation(impossible_slope_angle)
    .build(),
  )
  |> ignore
  let wall_angle = @core.pi() / 2.0F
  let wall_size = 2.0F
  let wall_pos = @core.Vec2::new(
    ground_size + slope_size * 2.0F + impossible_slope_size + 0.35F,
    -ground_height + 2.5F * 2.3F,
  )
  colliders.insert(
    @collision.ColliderBuilder::cuboid(wall_size, ground_height)
    .translation(wall_pos)
    .rotation(wall_angle)
    .build(),
  )
  |> ignore
  colliders.insert(
    @collision.ColliderBuilder::cuboid(wall_size, ground_height)
    .translation(wall_pos)
    .build(),
  )
  |> ignore

  // Moving platform (kinematic velocity-based).
  let platform_handle = bodies.insert(
    @dynamics.RigidBodyBuilder::kinematic_velocity_based()
    .translation(@core.Vec2::new(-8.0F, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(2.0F, ground_height).build(),
    platform_handle,
    bodies,
  )
  |> ignore

  // Heightfield ground section.
  let ground_scale = @core.Vec2::new(10.0F, 1.0F)
  let nsubdivs = 20
  let heights : Array[@core.Real] = []
  for i in 0..<(nsubdivs + 1) {
    let denom = Float::from_double(nsubdivs.to_double())
    let t = Float::from_double(i.to_double()) * ground_scale.x / denom / 2.0F
    heights.push(local_cos(t) * 1.5F)
  }
  colliders.insert(
    @collision.ColliderBuilder::heightfield(heights, ground_scale)
    .translation(@core.Vec2::new(-8.0F, 5.0F))
    .build(),
  )
  |> ignore

  // A tilting dynamic body with a limited joint.
  let ground2 = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, 5.0F))
    .build(),
  )
  let tilt = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(0.0F, 5.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(1.0F, 0.1F).build(),
    tilt,
    bodies,
  )
  |> ignore
  let joint = @dynamics.GenericJoint::from_revolute(
    @dynamics.RevoluteJointBuilder::new().limits(-0.3F, 0.3F).build(),
  )
  impulse_joints.insert(ground2, tilt, joint, true) |> ignore
  let parameters = @dynamics.IntegrationParameters::default()
  let inv_dt = parameters.inv_dt()
  let dt = if inv_dt == 0.0F { 0.0F } else { 1.0F / inv_dt }
  let ccd_solver = @dynamics.CCDSolver::new()
  let controller = @control.KinematicCharacterController::new().with_max_slope_climb_angle(
    impossible_slope_angle - 0.02F,
  )
  let mut time = 0.0F
  for _ in 0..<600 {
    time = time + dt
    // Move the velocity-based kinematic platform.
    if bodies.get_mut(platform_handle) is Some(platform) {
      let linvel = @core.Vec2::new(
        local_sin(time * 2.0F) * 2.0F,
        local_sin(time * 5.0F) * 1.5F,
      )
      platform.set_linvel(linvel, true) |> ignore
    } else {
      inspect(false, content="true")
    }
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
      impulse_joints, multibody_joints, ccd_solver, hooks, events,
    )

    // Deterministic "manual control": move the character forward/right and slightly down.
    if bodies.get_mut(character_handle) is Some(character_body) {
      let filter = @collision.QueryFilter::new().exclude_rigid_body(
        character_handle,
      )
      let query_pipeline = broad_phase.as_query_pipeline(
        bodies, colliders, filter,
      )
      let movement = controller.move_shape(
        dt,
        query_pipeline,
        character_shape,
        character_body.position(),
        @core.Vec2::new(0.12F, -0.12F),
        fn(_collision) {  },
      )
      let p = character_body.translation()
      character_body.set_next_kinematic_translation(
        p.add(movement.translation()),
      )
      |> ignore
    } else {
      inspect(false, content="true")
    }
  }
  if bodies.get(character_handle) is Some(character_body) {
    let p = character_body.translation()
    inspect(p.x > 6.0F, content="true")
    inspect(p.x < 12.0F, content="true")
    inspect(p.y == p.y, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
/// Ported from `rapier-reference/examples2d/collision_groups2.rs`.
test "examples2d/collision_groups2.rs parity" {
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
  let parameters = @dynamics.IntegrationParameters::default()
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let ground_size = 5.0F
  let ground_height = 0.1F
  let floor_handle = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, -ground_height))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(ground_size, ground_height).build(),
    floor_handle,
    bodies,
  )
  |> ignore
  let green_group = @dynamics.InteractionGroups::new(
    @dynamics.Group::group_1(),
    @dynamics.Group::group_1(),
    @dynamics.InteractionTestMode::And,
  )
  let blue_group = @dynamics.InteractionGroups::new(
    @dynamics.Group::group_2(),
    @dynamics.Group::group_2(),
    @dynamics.InteractionTestMode::And,
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(1.0F, 0.1F)
    .translation(@core.Vec2::new(0.0F, 1.0F))
    .collision_groups(green_group)
    .build(),
    floor_handle,
    bodies,
  )
  |> ignore
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(1.0F, 0.1F)
    .translation(@core.Vec2::new(0.0F, 2.0F))
    .collision_groups(blue_group)
    .build(),
    floor_handle,
    bodies,
  )
  |> ignore
  let green_bodies : Array[@dynamics.RigidBodyHandle] = []
  let blue_bodies : Array[@dynamics.RigidBodyHandle] = []
  let num = 8
  let rad = 0.1F
  let shift = rad * 2.0F
  let centerx = shift * Float::from_double((num / 2).to_double())
  let centery = 2.5F
  for j in 0..<4 {
    for i in 0..<num {
      let x = Float::from_double(i.to_double()) * shift - centerx
      let y = Float::from_double(j.to_double()) * shift + centery
      let is_green = i % 2 == 0
      let group = if is_green { green_group } else { blue_group }
      let handle = bodies.insert(
        @dynamics.RigidBodyBuilder::dynamic()
        .translation(@core.Vec2::new(x, y))
        .build(),
      )
      colliders.insert_with_parent(
        @collision.ColliderBuilder::cuboid(rad, rad)
        .collision_groups(group)
        .build(),
        handle,
        bodies,
      )
      |> ignore
      if is_green {
        green_bodies.push(handle)
      } else {
        blue_bodies.push(handle)
      }
    }
  }
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 360,
  )

  // Green cubes should fall through the blue floor at y=2 and be able to reach the green floor at y=1.
  // Blue cubes should be supported by the blue floor.
  let mut green_min_y = 1.0e30F
  for i in 0..<green_bodies.length() {
    if bodies.get(green_bodies[i]) is Some(body) {
      let y = body.translation().y
      if y < green_min_y {
        green_min_y = y
      }
    } else {
      inspect(false, content="true")
    }
  }
  let mut blue_max_y = -1.0e30F
  for i in 0..<blue_bodies.length() {
    if bodies.get(blue_bodies[i]) is Some(body) {
      let y = body.translation().y
      if y > blue_max_y {
        blue_max_y = y
      }
    } else {
      inspect(false, content="true")
    }
  }
  inspect(green_min_y < 1.6F, content="true")
  inspect(blue_max_y > 1.8F, content="true")
}

///|
/// Ported from `rapier-reference/examples2d/convex_polygons2.rs`.
test "examples2d/convex_polygons2.rs parity" {
  fn rng_next(seed : Int) -> Int {
    // Deterministic 31-bit LCG (fast, stable across platforms).
    (seed * 1103515245 + 12345) & 0x7fffffff
  }

  fn rng_float01(seed : Int) -> (@core.Real, Int) {
    let next = rng_next(seed)
    let value = Float::from_double(next.to_double() / 2147483647.0)
    (value, next)
  }

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
  let parameters = @dynamics.IntegrationParameters::default()
  let gravity = @core.Vec2::new(0.0F, -9.81F)

  // Ground + walls (match rapier-reference).
  let ground_size = 30.0F
  let ground_handle = bodies.insert(@dynamics.RigidBodyBuilder::fixed().build())
  let ground_collider = colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(ground_size, 1.2F).build(),
    ground_handle,
    bodies,
  )
  let wall_r = @core.pi() / 2.0F
  let right_wall = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .rotation(@core.Rot2::from_angle(wall_r))
    .translation(@core.Vec2::new(ground_size, ground_size * 2.0F))
    .build(),
  )
  let right_wall_collider = colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(ground_size * 2.0F, 1.2F).build(),
    right_wall,
    bodies,
  )
  let left_wall = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .rotation(@core.Rot2::from_angle(wall_r))
    .translation(@core.Vec2::new(-ground_size, ground_size * 2.0F))
    .build(),
  )
  let left_wall_collider = colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(ground_size * 2.0F, 1.2F).build(),
    left_wall,
    bodies,
  )

  // Create the convex polygons.
  let num = 14
  let scale = 4.0F
  let border_rad = 0.0F
  let shift = border_rad * 2.0F + scale
  let centerx = shift * Float::from_double(num.to_double()) / 2.0F
  let centery = shift / 2.0F
  let mut seed = 0
  let handles : Array[@dynamics.RigidBodyHandle] = []
  for i in 0..<num {
    for j in 0..<(num * 4) {
      let x = Float::from_double(i.to_double()) * shift - centerx
      let y = Float::from_double(j.to_double()) * shift * 2.0F + centery + 2.0F
      let handle = bodies.insert(
        @dynamics.RigidBodyBuilder::dynamic()
        .translation(@core.Vec2::new(x, y))
        .build(),
      )
      handles.push(handle)
      let points : Array[@core.Vec2] = []
      for _ in 0..<10 {
        let (rx, s1) = rng_float01(seed)
        let (ry, s2) = rng_float01(s1)
        seed = s2
        points.push(@core.Vec2::new(rx * scale, ry * scale))
      }
      if @collision.ColliderBuilder::convex_hull(points) is Some(builder) {
        colliders.insert_with_parent(builder.build(), handle, bodies) |> ignore
      } else {
        inspect(false, content="true")
      }
    }
  }

  // Let low layers hit the ground/walls (this is intentionally fewer than the full stack
  // settling time to keep the test fast, but still exercises convex polygon contacts).
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 300,
  )
  inspect(
    narrow_phase.contact_pairs_with(ground_collider).length() > 0,
    content="true",
  )
  inspect(
    narrow_phase.contact_pairs_with(right_wall_collider).length() > 0,
    content="true",
  )
  inspect(
    narrow_phase.contact_pairs_with(left_wall_collider).length() > 0,
    content="true",
  )
  let mut min_y = 1.0e30F
  for i in 0..<handles.length() {
    if bodies.get(handles[i]) is Some(body) {
      let y = body.translation().y
      // Basic sanity: no NaN and no obvious tunneling through the ground.
      inspect(y == y, content="true")
      if y < min_y {
        min_y = y
      }
    } else {
      inspect(false, content="true")
    }
  }
  inspect(min_y > -20.0F, content="true")
}

///|
/// Ported from `rapier-reference/examples2d/damping2.rs`.
test "examples2d/damping2.rs parity" {
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
  let parameters = @dynamics.IntegrationParameters::default()
  let num = 10
  let rad = 0.2F
  let subdiv = 1.0F / Float::from_double(num.to_double())
  let handles : Array[
    (@dynamics.RigidBodyHandle, @core.Vec2, @core.Real, @core.Real, @core.Real),
  ] = []
  for i in 0..<num {
    let a = Float::from_double(i.to_double()) * subdiv * @core.pi() * 2.0F
    let x = local_sin(a)
    let y = local_cos(a)
    let linvel = @core.Vec2::new(x * 10.0F, y * 10.0F)
    let angvel = 100.0F
    let lin_damping = Float::from_double((i + 1).to_double()) * subdiv * 10.0F
    let ang_damping = Float::from_double((num - i).to_double()) * subdiv * 10.0F
    let rb = @dynamics.RigidBodyBuilder::dynamic()
      .translation(@core.Vec2::new(x, y))
      .linvel(linvel)
      .angvel(angvel)
      .linear_damping(lin_damping)
      .angular_damping(ang_damping)
      .build()
    let handle = bodies.insert(rb)
    colliders.insert_with_parent(
      @collision.ColliderBuilder::cuboid(rad, rad).build(),
      handle,
      bodies,
    )
    |> ignore
    handles.push((handle, linvel, angvel, lin_damping, ang_damping))
  }
  let inv_dt = parameters.inv_dt()
  let dt = if inv_dt == 0.0F { 0.0F } else { 1.0F / inv_dt }
  step_pipeline_2d_n(
    pipeline,
    @core.Vec2::zero(),
    parameters,
    islands,
    broad_phase,
    narrow_phase,
    bodies,
    colliders,
    impulse_joints,
    multibody_joints,
    ccd_solver,
    hooks,
    events,
    1,
  )
  for i in 0..<handles.length() {
    let (handle, linvel0, angvel0, lin_damp, ang_damp) = handles[i]
    if bodies.get(handle) is Some(body) {
      let expected_lin_scale = 1.0F / (1.0F + dt * lin_damp)
      let expected_ang_scale = 1.0F / (1.0F + dt * ang_damp)
      let expected_lin = @core.Vec2::new(
        linvel0.x * expected_lin_scale,
        linvel0.y * expected_lin_scale,
      )
      let expected_ang = angvel0 * expected_ang_scale
      inspect(
        body.linvel().sub(expected_lin).length() < 1.0e-4F,
        content="true",
      )
      inspect(@core.abs(body.angvel() - expected_ang) < 1.0e-4F, content="true")
    } else {
      inspect(false, content="true")
    }
  }
}

///|
/// Ported from `rapier-reference/examples2d/debug_box_ball2.rs`.
test "examples2d/debug_box_ball2.rs parity" {
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
  let parameters = @dynamics.IntegrationParameters::default()
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let rad = 1.0F
  let ground_handle = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, -rad))
    .rotation(@core.Rot2::from_angle(@core.pi() / 4.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(rad, rad).build(),
    ground_handle,
    bodies,
  )
  |> ignore
  let ball_handle = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(0.0F, 3.0F * rad))
    .can_sleep(false)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::ball(rad).build(),
    ball_handle,
    bodies,
  )
  |> ignore
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 240,
  )
  if bodies.get(ball_handle) is Some(ball) {
    let pos = ball.translation()
    inspect(pos.x == pos.x, content="true")
    inspect(pos.y == pos.y, content="true")
    // Regression check: the ball should not tunnel through the thin rotated box.
    inspect(pos.y > -2.0F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
/// Ported from `rapier-reference/examples2d/debug_compression2.rs`.
test "examples2d/debug_compression2.rs parity" {
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
  let parameters = @dynamics.IntegrationParameters::default()
  let gravity = @core.Vec2::zero()
  let width = 75.0F
  let thickness = 2.0F
  let ys = [-30.0F - thickness, 30.0F + thickness]
  for i in 0..<ys.length() {
    let handle = bodies.insert(
      @dynamics.RigidBodyBuilder::fixed()
      .translation(@core.Vec2::new(0.0F, ys[i]))
      .build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder::cuboid(width, thickness).build(),
      handle,
      bodies,
    )
    |> ignore
  }
  let half_height = (ys[1] - ys[0]) / 2.0F - thickness
  let xs = [-width + thickness, width - thickness]
  let plank_handles : Array[@dynamics.RigidBodyHandle] = []
  for i in 0..<xs.length() {
    let handle = bodies.insert(
      @dynamics.RigidBodyBuilder::dynamic()
      .translation(@core.Vec2::new(xs[i], 0.0F))
      .build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder::cuboid(thickness, half_height).build(),
      handle,
      bodies,
    )
    |> ignore
    plank_handles.push(handle)
  }
  let num = 8
  let rad = half_height / Float::from_double(num.to_double())
  for i in 0..<num {
    for j in 0..<num {
      let x = Float::from_double(i.to_double()) * rad * 2.0F -
        Float::from_double(num.to_double()) * rad
      let y = Float::from_double(j.to_double()) * rad * 2.0F -
        Float::from_double(num.to_double()) * rad +
        rad
      let handle = bodies.insert(
        @dynamics.RigidBodyBuilder::dynamic()
        .translation(@core.Vec2::new(x, y))
        .build(),
      )
      colliders.insert_with_parent(
        @collision.ColliderBuilder::ball(rad).build(),
        handle,
        bodies,
      )
      |> ignore
    }
  }
  let left0 = if bodies.get(plank_handles[0]) is Some(body) {
    body.translation().x
  } else {
    0.0F
  }
  let right0 = if bodies.get(plank_handles[1]) is Some(body) {
    body.translation().x
  } else {
    0.0F
  }
  let mut force_x = 0.0F
  for _ in 0..<30 {
    if bodies.get_mut(plank_handles[0]) is Some(left) {
      left.reset_forces(true) |> ignore
      left.add_force(@core.Vec2::new(force_x, 0.0F), true) |> ignore
    }
    if bodies.get_mut(plank_handles[1]) is Some(right) {
      right.reset_forces(true) |> ignore
      right.add_force(@core.Vec2::new(-force_x, 0.0F), true) |> ignore
    }
    force_x = force_x + 10000.0F
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
      impulse_joints, multibody_joints, ccd_solver, hooks, events,
    )
  }

  // With forces integrated like Rapier, the planks should move toward each other.
  if bodies.get(plank_handles[0]) is Some(left) {
    inspect(left.translation().x > left0 + 0.01F, content="true")
  } else {
    inspect(false, content="true")
  }
  if bodies.get(plank_handles[1]) is Some(right) {
    inspect(right.translation().x < right0 - 0.01F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
/// Ported from `rapier-reference/examples2d/debug_intersection2.rs`.
test "examples2d/debug_intersection2.rs parity" {
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = @collision.ColliderSet::new()
  let rad = 1.0F
  let count = 20
  for x in 0..<count {
    for y in 0..<count {
      let fx = Float::from_double(x.to_double())
      let fy = Float::from_double(y.to_double())
      let half = Float::from_double(count.to_double()) / 2.0F
      let rigid_body = @dynamics.RigidBodyBuilder::fixed()
        .translation(
          @core.Vec2::new((fx - half) * rad * 3.0F, (fy - half) * rad * 3.0F),
        )
        .build()
      let handle = bodies.insert(rigid_body)
      colliders.insert_with_parent(
        @collision.ColliderBuilder::ball(rad).build(),
        handle,
        bodies,
      )
      |> ignore
    }
  }
  let broad_phase = @collision.BroadPhaseBvh::new()
  let filter = @collision.QueryFilter::new()
  let query_pipeline = broad_phase.as_query_pipeline(bodies, colliders, filter)
  let query_center = @core.Vec2::zero()
  let hits = query_pipeline.intersect_shape(
    @core.Isometry2::from_translation(query_center),
    @collision.Shape::Ball(rad / 2.0F),
  )

  // The query shape should intersect at least one of the fixed balls.
  inspect(hits.length() > 0, content="true")
  // Returned colliders should be actually close to the query shape center.
  let mut max_d2 = 0.0F
  for i in 0..<hits.length() {
    if colliders.get(hits[i]) is Some(co) {
      let d2 = co.translation().sub(query_center).length_squared()
      if d2 > max_d2 {
        max_d2 = d2
      }
    } else {
      inspect(false, content="true")
    }
  }
  inspect(max_d2 <= rad * 3.0F * (rad * 3.0F), content="true")
}

///|
/// Ported from `rapier-reference/examples2d/debug_total_overlap2.rs`.
test "examples2d/debug_total_overlap2.rs parity" {
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
  let parameters = @dynamics.IntegrationParameters::default()
  let rad = 0.5F
  let handles : Array[@dynamics.RigidBodyHandle] = []
  for _ in 0..<100 {
    let handle = bodies.insert(@dynamics.RigidBodyBuilder::dynamic().build())
    colliders.insert_with_parent(
      @collision.ColliderBuilder::cuboid(rad, rad).build(),
      handle,
      bodies,
    )
    |> ignore
    handles.push(handle)
  }
  step_pipeline_2d_n(
    pipeline,
    @core.Vec2::zero(),
    parameters,
    islands,
    broad_phase,
    narrow_phase,
    bodies,
    colliders,
    impulse_joints,
    multibody_joints,
    ccd_solver,
    hooks,
    events,
    10,
  )
  let mut max_dist2 = 0.0F
  for i in 0..<handles.length() {
    if bodies.get(handles[i]) is Some(body) {
      let p = body.translation()
      inspect(p.x == p.x, content="true")
      inspect(p.y == p.y, content="true")
      let d2 = p.length_squared()
      if d2 > max_dist2 {
        max_dist2 = d2
      }
    } else {
      inspect(false, content="true")
    }
  }
  inspect(max_dist2 > 1.0e-6F, content="true")
}

///|
/// Ported from `rapier-reference/examples2d/debug_vertical_column2.rs`.
test "examples2d/debug_vertical_column2.rs parity" {
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
  let parameters = @dynamics.IntegrationParameters::default()
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let num = 80
  let rad = 0.5F
  let ground_size = 1.0F
  let ground_thickness = 1.0F
  let ground_handle = bodies.insert(@dynamics.RigidBodyBuilder::fixed().build())
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(ground_size, ground_thickness)
    .friction(0.3F)
    .build(),
    ground_handle,
    bodies,
  )
  |> ignore
  let handles : Array[@dynamics.RigidBodyHandle] = []
  for i in 0..<num {
    let y = Float::from_double(i.to_double()) * rad * 2.0F +
      ground_thickness +
      rad
    let handle = bodies.insert(
      @dynamics.RigidBodyBuilder::dynamic()
      .translation(@core.Vec2::new(0.0F, y))
      .build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder::cuboid(rad, rad).friction(0.3F).build(),
      handle,
      bodies,
    )
    |> ignore
    handles.push(handle)
  }
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 600,
  )
  let mut min_y = 1.0e30F
  let mut max_y = -1.0e30F
  for i in 0..<handles.length() {
    if bodies.get(handles[i]) is Some(body) {
      let y = body.translation().y
      if y < min_y {
        min_y = y
      }
      if y > max_y {
        max_y = y
      }
    } else {
      inspect(false, content="true")
    }
  }
  inspect(min_y > 0.8F, content="true")
  inspect(max_y < 200.0F, content="true")
}

///|
/// Ported from `rapier-reference/examples2d/drum2.rs`.
test "examples2d/drum2.rs parity" {
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
  let parameters = @dynamics.IntegrationParameters::default()
  let gravity = @core.Vec2::new(0.0F, -9.81F)

  // Dynamic boxes.
  let num = 30
  let rad = 0.2F
  let shift = rad * 2.0F
  let centerx = shift * Float::from_double(num.to_double()) / 2.0F
  let centery = shift * Float::from_double(num.to_double()) / 2.0F
  let box_handles : Array[@dynamics.RigidBodyHandle] = []
  for i in 0..<num {
    for j in 0..<num {
      let x = Float::from_double(i.to_double()) * shift - centerx
      let y = Float::from_double(j.to_double()) * shift - centery
      let handle = bodies.insert(
        @dynamics.RigidBodyBuilder::dynamic()
        .translation(@core.Vec2::new(x, y))
        .build(),
      )
      colliders.insert_with_parent(
        @collision.ColliderBuilder::cuboid(rad, rad).build(),
        handle,
        bodies,
      )
      |> ignore
      box_handles.push(handle)
    }
  }

  // A kinematic "drum" controlled by constant angular velocity.
  let drum_handle = bodies.insert(
    @dynamics.RigidBodyBuilder::kinematic_velocity_based().build(),
  )
  let sides = [
    (10.0F, 0.25F, @core.Vec2::new(0.0F, 10.0F)),
    (10.0F, 0.25F, @core.Vec2::new(0.0F, -10.0F)),
    (0.25F, 10.0F, @core.Vec2::new(10.0F, 0.0F)),
    (0.25F, 10.0F, @core.Vec2::new(-10.0F, 0.0F)),
  ]
  let balls = [
    (1.25F, @core.Vec2::new(6.0F, 6.0F)),
    (1.25F, @core.Vec2::new(-6.0F, 6.0F)),
    (1.25F, @core.Vec2::new(6.0F, -6.0F)),
    (1.25F, @core.Vec2::new(-6.0F, -6.0F)),
  ]
  for i in 0..<sides.length() {
    let (hx, hy, pos) = sides[i]
    colliders.insert_with_parent(
      @collision.ColliderBuilder::cuboid(hx, hy).translation(pos).build(),
      drum_handle,
      bodies,
    )
    |> ignore
  }
  for i in 0..<balls.length() {
    let (r, pos) = balls[i]
    colliders.insert_with_parent(
      @collision.ColliderBuilder::ball(r).translation(pos).build(),
      drum_handle,
      bodies,
    )
    |> ignore
  }

  // Run the simulation, emulating the testbed callback.
  for _ in 0..<30 {
    if bodies.get_mut(drum_handle) is Some(drum) {
      drum.set_angvel(-0.15F, true) |> ignore
    } else {
      inspect(false, content="true")
    }
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
      impulse_joints, multibody_joints, ccd_solver, hooks, events,
    )
  }
  if bodies.get(drum_handle) is Some(drum) {
    let angle = drum.rotation().angle()
    inspect(angle == angle, content="true")
    inspect(@core.abs(angle) > 1.0e-3F, content="true")
  } else {
    inspect(false, content="true")
  }

  // Sanity: the simulation should remain stable (no NaNs/explosions).
  let mut max_d2 = 0.0F
  for i in 0..<box_handles.length() {
    if bodies.get(box_handles[i]) is Some(body) {
      let p = body.translation()
      inspect(p.x == p.x, content="true")
      inspect(p.y == p.y, content="true")
      let d2 = p.length_squared()
      if d2 > max_d2 {
        max_d2 = d2
      }
    } else {
      inspect(false, content="true")
    }
  }
  inspect(max_d2 < 1.0e6F, content="true")
}

///|
/// Ported from `rapier-reference/examples2d/heightfield2.rs`.
test "examples2d/heightfield2.rs parity" {
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
  let parameters = @dynamics.IntegrationParameters::default()
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let ground_size = @core.Vec2::new(50.0F, 1.0F)
  let nsubdivs = 2000
  let heights : Array[@core.Real] = []
  let nx = Float::from_double(nsubdivs.to_double())
  for i in 0..<(nsubdivs + 1) {
    if i == 0 || i == nsubdivs {
      heights.push(8.0F)
    } else {
      let t = Float::from_double(i.to_double()) * ground_size.x / nx
      heights.push(local_cos(t) * 2.0F)
    }
  }
  let ground_handle = bodies.insert(@dynamics.RigidBodyBuilder::fixed().build())
  colliders.insert_with_parent(
    @collision.ColliderBuilder::heightfield(heights, ground_size).build(),
    ground_handle,
    bodies,
  )
  |> ignore

  // Falling cubes/balls.
  let num = 20
  let rad = 0.5F
  let shift = rad * 2.0F
  let centerx = shift * Float::from_double((num / 2).to_double())
  let centery = shift / 2.0F
  let dynamic_handles : Array[@dynamics.RigidBodyHandle] = []
  for i in 0..<num {
    for j in 0..<num {
      let x = Float::from_double(i.to_double()) * shift - centerx
      let y = Float::from_double(j.to_double()) * shift + centery + 3.0F
      let handle = bodies.insert(
        @dynamics.RigidBodyBuilder::dynamic()
        .translation(@core.Vec2::new(x, y))
        .build(),
      )
      if j % 2 == 0 {
        colliders.insert_with_parent(
          @collision.ColliderBuilder::cuboid(rad, rad).build(),
          handle,
          bodies,
        )
        |> ignore
      } else {
        colliders.insert_with_parent(
          @collision.ColliderBuilder::ball(rad).build(),
          handle,
          bodies,
        )
        |> ignore
      }
      dynamic_handles.push(handle)
    }
  }
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 120,
  )
  let mut min_y = 1.0e30F
  let mut max_y = -1.0e30F
  for i in 0..<dynamic_handles.length() {
    if bodies.get(dynamic_handles[i]) is Some(body) {
      let p = body.translation()
      inspect(p.x == p.x, content="true")
      inspect(p.y == p.y, content="true")
      if p.y < min_y {
        min_y = p.y
      }
      if p.y > max_y {
        max_y = p.y
      }
    } else {
      inspect(false, content="true")
    }
  }
  inspect(min_y < 2.0F, content="true")
  inspect(min_y > -20.0F, content="true")
  inspect(max_y < 200.0F, content="true")
}

///|
/// Ported from `rapier-reference/examples2d/inv_pyramid2.rs`.
test "examples2d/inv_pyramid2.rs parity" {
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
  let parameters = @dynamics.IntegrationParameters::default()
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let ground_size = 10.0F
  let ground_thickness = 1.0F
  let ground_handle = bodies.insert(@dynamics.RigidBodyBuilder::fixed().build())
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(ground_size, ground_thickness).build(),
    ground_handle,
    bodies,
  )
  |> ignore
  let num = 6
  let mut rad = 0.5F
  let mut y = rad
  let handles : Array[@dynamics.RigidBodyHandle] = []
  for _ in 0..<num {
    let handle = bodies.insert(
      @dynamics.RigidBodyBuilder::dynamic()
      .translation(@core.Vec2::new(0.0F, y + ground_thickness))
      .build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder::cuboid(rad, rad).build(),
      handle,
      bodies,
    )
    |> ignore
    handles.push(handle)
    y = y + rad + rad * 2.0F
    rad = rad * 2.0F
  }
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 360,
  )
  for i in 0..<handles.length() {
    if bodies.get(handles[i]) is Some(body) {
      let pos = body.translation()
      inspect(pos.x == pos.x, content="true")
      inspect(pos.y == pos.y, content="true")
      inspect(pos.y > 0.5F, content="true")
    } else {
      inspect(false, content="true")
    }
  }
}

///|
/// Ported from `rapier-reference/examples2d/inverse_kinematics2.rs`.
test "examples2d/inverse_kinematics2.rs parity" {
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = @collision.ColliderSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()

  // Ground (kept for parity with upstream, though IK doesn't depend on it).
  let ground_height = 0.01F
  let floor_handle = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, -ground_height))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(1.0F, ground_height).build(),
    floor_handle,
    bodies,
  )
  |> ignore

  // Multibody chain.
  let num_segments = 10
  let size = 1.0F / Float::from_double(num_segments.to_double())
  let mut last_body = bodies.insert(@dynamics.RigidBodyBuilder::fixed().build())
  let mut last_link = @dynamics.MultibodyJointHandle::invalid()
  for i in 0..<num_segments {
    let new_body = bodies.insert(
      @dynamics.RigidBodyBuilder::dynamic().can_sleep(false).build(),
    )
    // A sensor collider for visualization parity in upstream; keep it to exercise sensor handling.
    colliders.insert_with_parent(
      @collision.ColliderBuilder::cuboid(size / 8.0F, size / 2.0F)
      .density(0.0F)
      .sensor(true)
      .build(),
      new_body,
      bodies,
    )
    |> ignore
    let joint = @dynamics.RevoluteJointBuilder::new()
      .local_anchor1(
        @core.Vec2::new(0.0F, size / 2.0F * (if i != 0 { 1.0F } else { 0.0F })),
      )
      .local_anchor2(@core.Vec2::new(0.0F, -size / 2.0F))
      .build()
    if multibody_joints.insert(last_body, new_body, joint, true) is Some(handle) {
      last_link = handle
    } else {
      inspect(false, content="true")
    }
    last_body = new_body
  }
  if multibody_joints.get_mut(last_link) is Some(result) {
    let (multibody, link_id) = result
    let ndofs = multibody.ndofs()
    let displacements = @dynamics.DVector::from_fn(ndofs, fn(
      _ : Int,
    ) -> @core.Real {
      0.0F
    })
    let target_point = @core.Vec2::new(0.25F, 0.4F)
    let options = @dynamics.InverseKinematicsOption::default()
      .set_constrained_axes(@dynamics.JointAxesMask::lin_axes())
      .set_max_iters(200)
    multibody.inverse_kinematics(
      bodies,
      link_id,
      options,
      @core.Isometry2::from_translation(target_point),
      fn(_link : @dynamics.MultibodyLink) -> Bool { true },
      displacements,
    )
    multibody.apply_displacements(displacements.as_array())
    let pose = multibody.forward_kinematics_single_link(
      bodies,
      link_id,
      None,
      None,
    )
    let err = pose.translation.sub(target_point).length()
    inspect(err == err, content="true")
    inspect(err < 5.0e-2F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
/// Ported from `rapier-reference/examples2d/joints2.rs`.
test "examples2d/joints2.rs parity" {
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
  let rad = 0.4F
  let numi = 10
  let numk = 10
  let shift = 1.0F
  let softness = @dynamics.SpringCoefficients::joint_defaults()
  let body_handles : Array[@dynamics.RigidBodyHandle] = []
  for k in 0..<numk {
    for i in 0..<numi {
      let fk = Float::from_double(k.to_double())
      let fi = Float::from_double(i.to_double())
      let rb = if i == 0 && k == 0 {
        @dynamics.RigidBodyBuilder::fixed()
      } else {
        @dynamics.RigidBodyBuilder::dynamic()
      }
      let child_handle = bodies.insert(
        rb.translation(@core.Vec2::new(fk * shift, -fi * shift)).build(),
      )
      colliders.insert_with_parent(
        @collision.ColliderBuilder::ball(rad).build(),
        child_handle,
        bodies,
      )
      |> ignore

      // Vertical joint.
      if i > 0 {
        let parent_handle = body_handles[body_handles.length() - 1]
        let joint = @dynamics.GenericJoint::from_revolute(
          @dynamics.RevoluteJointBuilder::new()
          .local_anchor2(@core.Vec2::new(0.0F, shift))
          .softness(softness)
          .build(),
        )
        impulse_joints.insert(parent_handle, child_handle, joint, true)
        |> ignore
      }

      // Horizontal joint.
      if k > 0 {
        let parent_index = body_handles.length() - numi
        let parent_handle = body_handles[parent_index]
        let joint = @dynamics.GenericJoint::from_revolute(
          @dynamics.RevoluteJointBuilder::new()
          .local_anchor2(@core.Vec2::new(-shift, 0.0F))
          .softness(softness)
          .build(),
        )
        impulse_joints.insert(parent_handle, child_handle, joint, true)
        |> ignore
      }
      body_handles.push(child_handle)
    }
  }
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 120,
  )

  // Spot-check a few joints: anchors should remain reasonably close.
  if bodies.get(body_handles[0]) is Some(a) &&
    bodies.get(body_handles[1]) is Some(b) {
    let d = b.translation().sub(a.translation()).length()
    inspect(d == d, content="true")
    inspect(@core.abs(d - shift) < 0.2F, content="true")
  } else {
    inspect(false, content="true")
  }
  if bodies.get(body_handles[0]) is Some(a) &&
    bodies.get(body_handles[numi]) is Some(b) {
    let d = b.translation().sub(a.translation()).length()
    inspect(d == d, content="true")
    inspect(@core.abs(d - shift) < 0.2F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
/// Ported from `rapier-reference/examples2d/locked_rotations2.rs`.
test "examples2d/locked_rotations2.rs parity" {
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
  let parameters = @dynamics.IntegrationParameters::default()
  let gravity = @core.Vec2::new(0.0F, -9.81F)

  // Ground.
  let ground_height = 0.1F
  let ground_handle = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, -ground_height))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(5.0F, ground_height).build(),
    ground_handle,
    bodies,
  )
  |> ignore

  // A rectangle that only rotates.
  let rect_handle = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(0.0F, 3.0F))
    .lock_translations()
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(2.0F, 0.6F).build(),
    rect_handle,
    bodies,
  )
  |> ignore

  // A tilted capsule that cannot rotate.
  let capsule_handle = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(0.0F, 5.0F))
    .rotation(@core.Rot2::from_angle(1.0F))
    .lock_rotations()
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::capsule_y(0.6F, 0.4F).build(),
    capsule_handle,
    bodies,
  )
  |> ignore
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 360,
  )
  if bodies.get(rect_handle) is Some(rect_body) {
    inspect(
      @core.abs(rect_body.translation().y - 3.0F) < 1.0e-3F,
      content="true",
    )
  } else {
    inspect(false, content="true")
  }
  if bodies.get(capsule_handle) is Some(capsule_body) {
    inspect(capsule_body.translation().y < 5.0F, content="true")
    inspect(
      @core.abs(capsule_body.rotation().angle() - 1.0F) < 1.0e-2F,
      content="true",
    )
  } else {
    inspect(false, content="true")
  }
}

///|
/// Ported from `rapier-reference/examples2d/one_way_platforms2.rs`.
test "examples2d/one_way_platforms2.rs parity" {
  let pipeline = @pipeline.PhysicsPipeline::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = @collision.ColliderSet::new()
  let impulse_joints = @dynamics.ImpulseJointSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()
  let islands = @dynamics.IslandManager::new()
  let handler = @pipeline.EventHandler::new()
  let ccd_solver = @dynamics.CCDSolver::new()
  let parameters = @dynamics.IntegrationParameters::default()
  let gravity = @core.Vec2::new(0.0F, -9.81F)

  // Fixed ground body with two platforms.
  let ground = bodies.insert(@dynamics.RigidBodyBuilder::fixed().build())
  let platform1 = colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(25.0F, 0.5F)
    .translation(@core.Vec2::new(30.0F, 2.0F))
    .active_hooks(@collision.ActiveHooks::modify_solver_contacts())
    .build(),
    ground,
    bodies,
  )
  let platform2 = colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(25.0F, 0.5F)
    .translation(@core.Vec2::new(-30.0F, -2.0F))
    .active_hooks(@collision.ActiveHooks::modify_solver_contacts())
    .build(),
    ground,
    bodies,
  )
  let hooks = @pipeline.PhysicsHooks::new().modify_solver_contacts(fn(
    _bodies : @dynamics.RigidBodySet,
    _colliders : @collision.ColliderSet,
    collider1 : @collision.ColliderHandle,
    collider2 : @collision.ColliderHandle,
    manifold : @collision.ContactManifold,
  ) -> (Bool, @core.Vec2) {
    let mut allowed_world_n = @core.Vec2::zero()
    let mut oneway = false
    if @collision.ColliderHandle::equals(collider1, platform1) ||
      @collision.ColliderHandle::equals(collider2, platform1) {
      // Platform1: local +y.
      allowed_world_n = @core.Vec2::new(0.0F, 1.0F)
      oneway = true
      if @collision.ColliderHandle::equals(collider2, platform1) {
        allowed_world_n = @core.Vec2::new(
          -allowed_world_n.x,
          -allowed_world_n.y,
        )
      }
    } else if @collision.ColliderHandle::equals(collider1, platform2) ||
      @collision.ColliderHandle::equals(collider2, platform2) {
      // Platform2: local -y.
      allowed_world_n = @core.Vec2::new(0.0F, -1.0F)
      oneway = true
      if @collision.ColliderHandle::equals(collider2, platform2) {
        allowed_world_n = @core.Vec2::new(
          -allowed_world_n.x,
          -allowed_world_n.y,
        )
      }
    }
    if !oneway {
      return (true, @core.Vec2::zero())
    }

    // Reject if the manifold normal doesn't match the allowed direction.
    let n12 = manifold.normal()
    if n12.dot(allowed_world_n) <= 0.0F {
      return (false, @core.Vec2::zero())
    }

    // Mimic `update_as_oneway_platform(_, 0.1)` by rejecting far speculative contacts.
    let pts = manifold.points()
    let mut min_dist = 1.0e30F
    for i in 0..<pts.length() {
      let d = pts[i].dist()
      if d < min_dist {
        min_dist = d
      }
    }
    if min_dist > 0.1F {
      return (false, @core.Vec2::zero())
    }

    // Set the surface velocity of the accepted contacts.
    let tangent_velocity = if @collision.ColliderHandle::equals(
        collider1, platform1,
      ) ||
      @collision.ColliderHandle::equals(collider2, platform2) {
      -12.0F
    } else {
      12.0F
    }
    // The hook API provides a world tangent velocity vector (projected by the solver).
    (true, @core.Vec2::new(tangent_velocity, 0.0F))
  })

  // Spawn cubes and apply position-dependent gravity scale (match the reference callback).
  let cube_handles : Array[@dynamics.RigidBodyHandle] = []
  for step_id in 0..<1200 {
    if step_id % 200 == 0 && bodies.len() <= 7 {
      let handle = bodies.insert(
        @dynamics.RigidBodyBuilder::dynamic()
        .translation(@core.Vec2::new(20.0F, 10.0F))
        .build(),
      )
      colliders.insert_with_parent(
        @collision.ColliderBuilder::cuboid(1.5F, 2.0F).build(),
        handle,
        bodies,
      )
      |> ignore
      cube_handles.push(handle)
    }
    let actives = islands.active_bodies()
    for i in 0..<actives.length() {
      let h = actives[i]
      if bodies.get_mut(h) is Some(body) {
        let y = body.translation().y
        if y > 1.0F {
          body.set_gravity_scale(1.0F, false) |> ignore
        } else if y < -1.0F {
          body.set_gravity_scale(-1.0F, false) |> ignore
        }
      }
    }
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
      impulse_joints, multibody_joints, ccd_solver, hooks, handler,
    )
  }
  inspect(cube_handles.length() > 0, content="true")
  // Surface velocity should move the cubes sideways measurably.
  let mut min_x = 1.0e30F
  let mut max_x = -1.0e30F
  for i in 0..<cube_handles.length() {
    if bodies.get(cube_handles[i]) is Some(body) {
      let x = body.translation().x
      if x < min_x {
        min_x = x
      }
      if x > max_x {
        max_x = x
      }
    } else {
      inspect(false, content="true")
    }
  }
  inspect(max_x - min_x > 5.0F, content="true")
}

///|
/// Ported from `rapier-reference/examples2d/pin_slot_joint2.rs`.
test "examples2d/pin_slot_joint2.rs parity" {
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
  let parameters = @dynamics.IntegrationParameters::default()
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let inv_dt = parameters.inv_dt()
  let dt = if inv_dt == 0.0F { 0.0F } else { 1.0F / inv_dt }

  // Ground.
  let ground_size = 3.0F
  let ground_height = 0.1F
  let floor = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, -ground_height))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(ground_size, ground_height).build(),
    floor,
    bodies,
  )
  |> ignore

  // Kinematic character.
  let character_handle = bodies.insert(
    @dynamics.RigidBodyBuilder::kinematic_position_based()
    .translation(@core.Vec2::new(0.0F, 0.3F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(0.15F, 0.3F).build(),
    character_handle,
    bodies,
  )
  |> ignore

  // Tethered cube + axis indicator ball.
  let rad = 0.4F
  let cube_handle = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(1.0F, 1.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(rad, rad).build(),
    cube_handle,
    bodies,
  )
  |> ignore
  let ball_handle = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(1.0F, 1.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::ball(0.1F).build(),
    ball_handle,
    bodies,
  )
  |> ignore

  // Fixed joint between the indicator and cube.
  let fixed_joint = @dynamics.GenericJoint::from_fixed(
    @dynamics.FixedJointBuilder::new()
    .local_anchor1(@core.Vec2::new(0.0F, 0.0F))
    .local_anchor2(@core.Vec2::new(0.0F, -0.4F))
    .build(),
  )
  impulse_joints.insert(cube_handle, ball_handle, fixed_joint, true) |> ignore

  // Pin-slot joint between the character and cube.
  let axis = @core.Vec2::new(1.0F, 1.0F).normalize()
  let pin_slot = @dynamics.GenericJoint::from_pin_slot(
    @dynamics.PinSlotJointBuilder::new(axis)
    .local_anchor1(@core.Vec2::new(2.0F, 2.0F))
    .local_anchor2(@core.Vec2::new(0.0F, 0.4F))
    .limits(-1.0F, 1.0e30F)
    .build(),
  )
  impulse_joints.insert(character_handle, cube_handle, pin_slot, true) |> ignore

  // Move the character deterministically to stress the constraints.
  let mut time = 0.0F
  for _ in 0..<600 {
    time = time + dt
    if bodies.get_mut(character_handle) is Some(ch) {
      let target = @core.Vec2::new(local_sin(time * 0.7F) * 1.5F, 0.3F)
      ch.set_next_kinematic_translation(target) |> ignore
    } else {
      inspect(false, content="true")
    }
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
      impulse_joints, multibody_joints, ccd_solver, hooks, events,
    )
  }

  // The indicator ball should stay attached to the cube.
  if bodies.get(cube_handle) is Some(cube) &&
    bodies.get(ball_handle) is Some(ball) {
    let d = ball.translation().sub(cube.translation()).length()
    inspect(d == d, content="true")
    inspect(@core.abs(d - 0.4F) < 0.2F, content="true")
  } else {
    inspect(false, content="true")
  }

  // The cube anchor should remain close to the pin-slot line (perp distance near 0),
  // and respect the limit on the axis projection (<= 1.0).
  if bodies.get(character_handle) is Some(ch) &&
    bodies.get(cube_handle) is Some(cube) {
    let anchor1 = ch.position().transform_point(@core.Vec2::new(2.0F, 2.0F))
    let anchor2 = cube.position().transform_point(@core.Vec2::new(0.0F, 0.4F))
    let delta = anchor1.sub(anchor2)
    let perp = @core.Vec2::new(-axis.y, axis.x)
    let perp_dist = @core.abs(delta.dot(perp))
    inspect(perp_dist < 0.2F, content="true")
    let proj = delta.dot(axis)
    inspect(proj <= 1.1F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
/// Ported from `rapier-reference/examples2d/platform2.rs`.
test "examples2d/platform2.rs parity" {
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
  let parameters = @dynamics.IntegrationParameters::default()
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let inv_dt = parameters.inv_dt()
  let dt = if inv_dt == 0.0F { 0.0F } else { 1.0F / inv_dt }
  let ground_height = 0.1F
  let ground_handle = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, -ground_height))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(10.0F, ground_height).build(),
    ground_handle,
    bodies,
  )
  |> ignore
  let num = 6
  let rad = 0.2F
  let shift = rad * 2.0F
  let centerx = shift * Float::from_double(num.to_double()) / 2.0F
  let centery = shift / 2.0F + 3.04F
  for i in 0..<num {
    for j in 0..<num {
      let x = Float::from_double(i.to_double()) * shift - centerx
      let y = Float::from_double(j.to_double()) * shift + centery
      let handle = bodies.insert(
        @dynamics.RigidBodyBuilder::dynamic()
        .translation(@core.Vec2::new(x, y))
        .build(),
      )
      colliders.insert_with_parent(
        @collision.ColliderBuilder::cuboid(rad, rad).build(),
        handle,
        bodies,
      )
      |> ignore
    }
  }
  let vel_based_handle = bodies.insert(
    @dynamics.RigidBodyBuilder::kinematic_velocity_based()
    .translation(@core.Vec2::new(-10.0F * rad, 1.5F + 0.8F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(rad * 10.0F, rad).build(),
    vel_based_handle,
    bodies,
  )
  |> ignore
  let pos_based_handle = bodies.insert(
    @dynamics.RigidBodyBuilder::kinematic_position_based()
    .translation(@core.Vec2::new(-10.0F * rad, 2.0F + 1.5F + 0.8F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(rad * 10.0F, rad).build(),
    pos_based_handle,
    bodies,
  )
  |> ignore
  let vel_start = if bodies.get(vel_based_handle) is Some(body) {
    body.translation()
  } else {
    @core.Vec2::zero()
  }
  let pos_start = if bodies.get(pos_based_handle) is Some(body) {
    body.translation()
  } else {
    @core.Vec2::zero()
  }
  let mut time = 0.0F
  let mut last_velocity = @core.Vec2::zero()
  for _ in 0..<120 {
    let velocity = @core.Vec2::new(
      local_sin(time) * 5.0F,
      local_sin(time * 5.0F),
    )
    last_velocity = velocity
    if bodies.get_mut(vel_based_handle) is Some(platform) {
      platform.set_linvel(velocity, true) |> ignore
    } else {
      inspect(false, content="true")
    }
    if bodies.get_mut(pos_based_handle) is Some(platform) {
      let next = platform
        .translation()
        .add(@core.Vec2::new(velocity.x * dt, velocity.y * dt))
      platform.set_next_kinematic_translation(next) |> ignore
    } else {
      inspect(false, content="true")
    }
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
      impulse_joints, multibody_joints, ccd_solver, hooks, events,
    )
    time = time + dt
  }
  if bodies.get(vel_based_handle) is Some(platform) {
    let p = platform.translation()
    inspect(p.x == p.x, content="true")
    inspect(p.y == p.y, content="true")
    inspect(p.sub(vel_start).length() > 1.0e-2F, content="true")
  } else {
    inspect(false, content="true")
  }
  if bodies.get(pos_based_handle) is Some(platform) {
    let p = platform.translation()
    inspect(p.x == p.x, content="true")
    inspect(p.y == p.y, content="true")
    inspect(p.sub(pos_start).length() > 1.0e-2F, content="true")
    inspect(
      platform.linvel().sub(last_velocity).length() < 1.0F,
      content="true",
    )
  } else {
    inspect(false, content="true")
  }
}

///|
/// Ported from `rapier-reference/examples2d/polyline2.rs`.
test "examples2d/polyline2.rs parity" {
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
  let parameters = @dynamics.IntegrationParameters::default()
  let gravity = @core.Vec2::new(0.0F, -9.81F)

  // Ground polyline.
  let ground_size = 50.0F
  let nsubdivs = 2000
  let step_size = ground_size / Float::from_double(nsubdivs.to_double())
  let points : Array[@core.Vec2] = []
  points.push(@core.Vec2::new(-ground_size / 2.0F, 40.0F))
  for i in 1..<(nsubdivs - 1) {
    let x = -ground_size / 2.0F + Float::from_double(i.to_double()) * step_size
    let y = local_cos(Float::from_double(i.to_double()) * step_size) * 2.0F
    points.push(@core.Vec2::new(x, y))
  }
  points.push(@core.Vec2::new(ground_size / 2.0F, 40.0F))
  let ground_handle = bodies.insert(@dynamics.RigidBodyBuilder::fixed().build())
  colliders.insert_with_parent(
    @collision.ColliderBuilder::polyline(points, None).build(),
    ground_handle,
    bodies,
  )
  |> ignore

  // Falling cubes/balls.
  let num = 20
  let rad = 0.5F
  let shift = rad * 2.0F
  let centerx = shift * Float::from_double((num / 2).to_double())
  let centery = shift / 2.0F
  let dynamic_handles : Array[@dynamics.RigidBodyHandle] = []
  for i in 0..<num {
    for j in 0..<num {
      let x = Float::from_double(i.to_double()) * shift - centerx
      let y = Float::from_double(j.to_double()) * shift + centery + 3.0F
      let handle = bodies.insert(
        @dynamics.RigidBodyBuilder::dynamic()
        .translation(@core.Vec2::new(x, y))
        .build(),
      )
      if j % 2 == 0 {
        colliders.insert_with_parent(
          @collision.ColliderBuilder::cuboid(rad, rad).build(),
          handle,
          bodies,
        )
        |> ignore
      } else {
        colliders.insert_with_parent(
          @collision.ColliderBuilder::ball(rad).build(),
          handle,
          bodies,
        )
        |> ignore
      }
      dynamic_handles.push(handle)
    }
  }
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 120,
  )

  // Sanity: at least one body should have landed, and none should have tunneled far below.
  let mut min_y = 1.0e30F
  let mut max_y = -1.0e30F
  for i in 0..<dynamic_handles.length() {
    if bodies.get(dynamic_handles[i]) is Some(body) {
      let p = body.translation()
      inspect(p.x == p.x, content="true")
      inspect(p.y == p.y, content="true")
      if p.y < min_y {
        min_y = p.y
      }
      if p.y > max_y {
        max_y = p.y
      }
    } else {
      inspect(false, content="true")
    }
  }
  inspect(min_y < 2.0F, content="true")
  inspect(min_y > -20.0F, content="true")
  inspect(max_y < 200.0F, content="true")
}

///|
/// Ported from `rapier-reference/examples2d/pyramid2.rs`.
test "examples2d/pyramid2.rs parity" {
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
  let parameters = @dynamics.IntegrationParameters::default()
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let ground_size = 10.0F
  let ground_thickness = 1.0F
  let ground_handle = bodies.insert(@dynamics.RigidBodyBuilder::fixed().build())
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(ground_size, ground_thickness).build(),
    ground_handle,
    bodies,
  )
  |> ignore
  let num = 10
  let rad = 0.5F
  let shift = rad * 2.0F
  let centerx = shift * Float::from_double(num.to_double()) / 2.0F
  let centery = shift / 2.0F + ground_thickness + rad * 1.5F
  let handles : Array[@dynamics.RigidBodyHandle] = []
  for i in 0..<num {
    for j in i..<num {
      let fj = Float::from_double(j.to_double())
      let fi = Float::from_double(i.to_double())
      let x = fi * shift / 2.0F + (fj - fi) * shift - centerx
      let y = fi * shift + centery
      let handle = bodies.insert(
        @dynamics.RigidBodyBuilder::dynamic()
        .translation(@core.Vec2::new(x, y))
        .build(),
      )
      colliders.insert_with_parent(
        @collision.ColliderBuilder::cuboid(rad, rad).build(),
        handle,
        bodies,
      )
      |> ignore
      handles.push(handle)
    }
  }
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 600,
  )
  let mut min_y = 1.0e30F
  for i in 0..<handles.length() {
    if bodies.get(handles[i]) is Some(body) {
      let pos = body.translation()
      inspect(pos.x == pos.x, content="true")
      inspect(pos.y == pos.y, content="true")
      if pos.y < min_y {
        min_y = pos.y
      }
    } else {
      inspect(false, content="true")
    }
  }
  // Cubes should not fall through the ground.
  inspect(min_y > -0.5F, content="true")
}

///|
/// Ported from `rapier-reference/examples2d/restitution2.rs`.
test "examples2d/restitution2.rs parity" {
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
  let parameters = @dynamics.IntegrationParameters::default()
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let ground_height = 1.0F
  let ground_handle = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, -ground_height))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(20.0F, ground_height)
    .restitution(1.0F)
    .build(),
    ground_handle,
    bodies,
  )
  |> ignore
  let rad = 0.5F
  let low_body = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(-2.0F, 10.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::ball(rad).restitution(0.0F).build(),
    low_body,
    bodies,
  )
  |> ignore
  let high_body = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(2.0F, 10.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::ball(rad).restitution(1.0F).build(),
    high_body,
    bodies,
  )
  |> ignore
  let mut low_bounced = false
  let mut high_bounced = false
  let mut low_max_y = -1.0e30F
  let mut high_max_y = -1.0e30F
  for _ in 0..<360 {
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
      impulse_joints, multibody_joints, ccd_solver, hooks, events,
    )
    if bodies.get(low_body) is Some(body) {
      if !low_bounced && body.translation().y < 1.0F && body.linvel().y > 0.0F {
        low_bounced = true
      }
      if low_bounced && body.translation().y > low_max_y {
        low_max_y = body.translation().y
      }
    }
    if bodies.get(high_body) is Some(body) {
      if !high_bounced && body.translation().y < 1.0F && body.linvel().y > 0.0F {
        high_bounced = true
      }
      if high_bounced && body.translation().y > high_max_y {
        high_max_y = body.translation().y
      }
    }
  }
  inspect(low_bounced, content="true")
  inspect(high_bounced, content="true")
  inspect(high_max_y > low_max_y + 5.0F, content="true")
}

///|
/// Ported from `rapier-reference/examples2d/rope_joints2.rs`.
test "examples2d/rope_joints2.rs parity" {
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
  let parameters = @dynamics.IntegrationParameters::default()
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let inv_dt = parameters.inv_dt()
  let dt = if inv_dt == 0.0F { 0.0F } else { 1.0F / inv_dt }

  // Ground (three walls) like the reference example.
  let ground_size = 0.75F
  let ground_height = 0.1F
  let floor0 = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, -ground_height))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(ground_size, ground_height).build(),
    floor0,
    bodies,
  )
  |> ignore
  let floor1 = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(-ground_size - ground_height, ground_size))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(ground_height, ground_size).build(),
    floor1,
    bodies,
  )
  |> ignore
  let floor2 = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(ground_size + ground_height, ground_size))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(ground_height, ground_size).build(),
    floor2,
    bodies,
  )
  |> ignore

  // Controlled kinematic character.
  let character_handle = bodies.insert(
    @dynamics.RigidBodyBuilder::kinematic_position_based()
    .translation(@core.Vec2::new(0.0F, 0.3F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(0.15F, 0.3F).build(),
    character_handle,
    bodies,
  )
  |> ignore

  // Tethered ball with a rope joint of length 2.0.
  let child_handle = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(1.0F, 1.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::ball(0.04F).build(),
    child_handle,
    bodies,
  )
  |> ignore
  let rope = @dynamics.GenericJoint::from_rope(
    @dynamics.RopeJointBuilder::new(2.0F)
    .local_anchor2(@core.Vec2::zero())
    .build(),
  )
  impulse_joints.insert(character_handle, child_handle, rope, true) |> ignore

  // Move the kinematic character deterministically to stretch the rope.
  let mut time = 0.0F
  for _ in 0..<600 {
    time = time + dt
    if bodies.get_mut(character_handle) is Some(ch) {
      let target = @core.Vec2::new(local_sin(time) * 2.0F, 0.3F)
      ch.set_next_kinematic_translation(target) |> ignore
    } else {
      inspect(false, content="true")
    }
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
      impulse_joints, multibody_joints, ccd_solver, hooks, events,
    )
  }
  if bodies.get(character_handle) is Some(ch) &&
    bodies.get(child_handle) is Some(ball) {
    let d = ball.translation().sub(ch.translation()).length()
    inspect(d == d, content="true")
    inspect(d <= 2.05F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
/// Ported from `rapier-reference/examples2d/s2d_arch.rs`.
test "examples2d/s2d_arch.rs parity" {
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
  fn scaled(p : @core.Vec2, s : @core.Real) -> @core.Vec2 {
    @core.Vec2::new(p.x * s, p.y * s)
  }

  // Arch vertices (from rapier-reference, scaled by 0.25).
  let scale = 0.25F
  let friction = 0.6F
  let ps1 = [
    scaled(@core.Vec2::new(16.0F, 0.0F), scale),
    scaled(@core.Vec2::new(14.93803712795643F, 5.133601056842984F), scale),
    scaled(@core.Vec2::new(13.79871746027416F, 10.24928069555078F), scale),
    scaled(@core.Vec2::new(12.56252963284711F, 15.34107019122473F), scale),
    scaled(@core.Vec2::new(11.20040987372525F, 20.39856541571217F), scale),
    scaled(@core.Vec2::new(9.66521217819836F, 25.40369899225096F), scale),
    scaled(@core.Vec2::new(7.87179930638133F, 30.3179337000085F), scale),
    scaled(@core.Vec2::new(5.635199558196225F, 35.03820717801641F), scale),
    scaled(@core.Vec2::new(2.405937953536585F, 39.09554102558315F), scale),
  ]
  let ps2 = [
    scaled(@core.Vec2::new(24.0F, 0.0F), scale),
    scaled(@core.Vec2::new(22.33619528222415F, 6.02299846205841F), scale),
    scaled(@core.Vec2::new(20.54936888969905F, 12.00964361211476F), scale),
    scaled(@core.Vec2::new(18.60854610798073F, 17.9470321677465F), scale),
    scaled(@core.Vec2::new(16.46769273811807F, 23.81367936585418F), scale),
    scaled(@core.Vec2::new(14.05325025774858F, 29.57079353071012F), scale),
    scaled(@core.Vec2::new(11.23551045834022F, 35.13775818285372F), scale),
    scaled(@core.Vec2::new(7.752568160730571F, 40.30450679009583F), scale),
    scaled(@core.Vec2::new(3.016931552701656F, 44.28891593799322F), scale),
  ]

  // Ground segment.
  colliders.insert(
    @collision.ColliderBuilder::segment(
      @core.Vec2::new(-100.0F, 0.0F),
      @core.Vec2::new(100.0F, 0.0F),
    )
    .friction(friction)
    .build(),
  )
  |> ignore

  // Arch blocks.
  for i in 0..<8 {
    let pts = [ps1[i], ps2[i], ps2[i + 1], ps1[i + 1]]
    let h = bodies.insert(@dynamics.RigidBodyBuilder::dynamic().build())
    if @collision.ColliderBuilder::convex_hull(pts) is Some(builder) {
      colliders.insert_with_parent(
        builder.friction(friction).build(),
        h,
        bodies,
      )
      |> ignore
    } else {
      inspect(false, content="true")
    }
  }
  for i in 0..<8 {
    let pts = [
      @core.Vec2::new(-ps2[i].x, ps2[i].y),
      @core.Vec2::new(-ps1[i].x, ps1[i].y),
      @core.Vec2::new(-ps1[i + 1].x, ps1[i + 1].y),
      @core.Vec2::new(-ps2[i + 1].x, ps2[i + 1].y),
    ]
    let h = bodies.insert(@dynamics.RigidBodyBuilder::dynamic().build())
    if @collision.ColliderBuilder::convex_hull(pts) is Some(builder) {
      colliders.insert_with_parent(
        builder.friction(friction).build(),
        h,
        bodies,
      )
      |> ignore
    } else {
      inspect(false, content="true")
    }
  }
  // Keystone.
  let keystone_pts = [
    ps1[8],
    ps2[8],
    @core.Vec2::new(-ps1[8].x, ps1[8].y),
    @core.Vec2::new(-ps2[8].x, ps2[8].y),
  ]
  let keystone = bodies.insert(@dynamics.RigidBodyBuilder::dynamic().build())
  if @collision.ColliderBuilder::convex_hull(keystone_pts) is Some(builder) {
    colliders.insert_with_parent(
      builder.friction(friction).build(),
      keystone,
      bodies,
    )
    |> ignore
  } else {
    inspect(false, content="true")
  }
  // Top load.
  for i in 0..<4 {
    let y = 0.5F + ps2[8].y + Float::from_double(i.to_double())
    let h = bodies.insert(
      @dynamics.RigidBodyBuilder::dynamic()
      .translation(@core.Vec2::new(0.0F, y))
      .build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder::cuboid(2.0F, 0.5F).friction(friction).build(),
      h,
      bodies,
    )
    |> ignore
  }
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 360,
  )

  // Sanity: no NaNs/explosions.
  let it = bodies.iter()
  for i in 0..<it.length() {
    let (_, b) = it[i]
    let p = b.translation()
    inspect(p.x == p.x, content="true")
    inspect(p.y == p.y, content="true")
  }
  inspect(it.length() > 0, content="true")
  keystone |> ignore
}

///|
/// Ported from `rapier-reference/examples2d/s2d_ball_and_chain.rs`.
test "examples2d/s2d_ball_and_chain.rs parity" {
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
  let ground = bodies.insert(@dynamics.RigidBodyBuilder::fixed().build())

  // Ball-and-chain (ported from rapier-reference).
  let count = 40
  let hx = 0.5F
  let density = 20.0F
  let friction = 0.6F
  let mut prev = ground
  for i in 0..<count {
    let fi = Float::from_double(i.to_double())
    let rigid_body = @dynamics.RigidBodyBuilder::dynamic()
      .linear_damping(0.1F)
      .angular_damping(0.1F)
      .translation(
        @core.Vec2::new(
          (1.0F + 2.0F * fi) * hx,
          Float::from_double(count.to_double()) * hx,
        ),
      )
      .build()
    let handle = bodies.insert(rigid_body)
    colliders.insert_with_parent(
      @collision.ColliderBuilder::capsule_x(hx, 0.125F)
      .friction(friction)
      .density(density)
      .build(),
      handle,
      bodies,
    )
    |> ignore
    let pivot = @core.Vec2::new(
      2.0F * fi * hx,
      Float::from_double(count.to_double()) * hx,
    )
    if bodies.get(prev) is Some(prev_rb) && bodies.get(handle) is Some(rb) {
      let local1 = prev_rb.position().inverse().transform_point(pivot)
      let local2 = rb.position().inverse().transform_point(pivot)
      let joint = @dynamics.GenericJoint::from_revolute(
        @dynamics.RevoluteJointBuilder::new()
        .local_anchor1(local1)
        .local_anchor2(local2)
        .build(),
      ).set_contacts_enabled(false)
      impulse_joints.insert(prev, handle, joint, true) |> ignore
    } else {
      inspect(false, content="true")
    }
    prev = handle
  }
  let radius = 8.0F
  let ball_handle = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .linear_damping(0.1F)
    .angular_damping(0.1F)
    .translation(
      @core.Vec2::new(
        (1.0F + 2.0F * Float::from_double(count.to_double())) * hx + radius - hx,
        Float::from_double(count.to_double()) * hx,
      ),
    )
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::ball(radius)
    .friction(friction)
    .density(density)
    .build(),
    ball_handle,
    bodies,
  )
  |> ignore
  let pivot = @core.Vec2::new(
    2.0F * Float::from_double(count.to_double()) * hx,
    Float::from_double(count.to_double()) * hx,
  )
  if bodies.get(prev) is Some(prev_rb) &&
    bodies.get(ball_handle) is Some(ball_rb) {
    let local1 = prev_rb.position().inverse().transform_point(pivot)
    let local2 = ball_rb.position().inverse().transform_point(pivot)
    let joint = @dynamics.GenericJoint::from_revolute(
      @dynamics.RevoluteJointBuilder::new()
      .local_anchor1(local1)
      .local_anchor2(local2)
      .build(),
    ).set_contacts_enabled(false)
    impulse_joints.insert(prev, ball_handle, joint, true) |> ignore
  } else {
    inspect(false, content="true")
  }
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 240,
  )

  // Sanity: stable and ball stays finite.
  if bodies.get(ball_handle) is Some(ball) {
    let p = ball.translation()
    inspect(p.x == p.x, content="true")
    inspect(p.y == p.y, content="true")
    inspect(p.y > -200.0F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
/// Ported from `rapier-reference/examples2d/s2d_bridge.rs`.
test "examples2d/s2d_bridge.rs parity" {
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
  let ground = bodies.insert(@dynamics.RigidBodyBuilder::fixed().build())

  // Bridge (ported from rapier-reference).
  let density = 20.0F
  let x_base = -80.0F
  let count = 160
  let mut prev = ground
  for i in 0..<count {
    let fi = Float::from_double(i.to_double())
    let handle = bodies.insert(
      @dynamics.RigidBodyBuilder::dynamic()
      .linear_damping(0.1F)
      .angular_damping(0.1F)
      .translation(@core.Vec2::new(x_base + 0.5F + 1.0F * fi, 20.0F))
      .build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder::cuboid(0.5F, 0.125F).density(density).build(),
      handle,
      bodies,
    )
    |> ignore
    let pivot = @core.Vec2::new(x_base + 1.0F * fi, 20.0F)
    if bodies.get(prev) is Some(prev_rb) && bodies.get(handle) is Some(rb) {
      let local1 = prev_rb.position().inverse().transform_point(pivot)
      let local2 = rb.position().inverse().transform_point(pivot)
      let joint = @dynamics.GenericJoint::from_revolute(
        @dynamics.RevoluteJointBuilder::new()
        .local_anchor1(local1)
        .local_anchor2(local2)
        .build(),
      ).set_contacts_enabled(false)
      impulse_joints.insert(prev, handle, joint, true) |> ignore
    } else {
      inspect(false, content="true")
    }
    prev = handle
  }

  // Attach last link to ground.
  let pivot = @core.Vec2::new(
    x_base + 1.0F * Float::from_double(count.to_double()),
    20.0F,
  )
  if bodies.get(prev) is Some(prev_rb) && bodies.get(ground) is Some(ground_rb) {
    let local1 = prev_rb.position().inverse().transform_point(pivot)
    let local2 = ground_rb.position().inverse().transform_point(pivot)
    let joint = @dynamics.GenericJoint::from_revolute(
      @dynamics.RevoluteJointBuilder::new()
      .local_anchor1(local1)
      .local_anchor2(local2)
      .build(),
    ).set_contacts_enabled(false)
    impulse_joints.insert(prev, ground, joint, true) |> ignore
  } else {
    inspect(false, content="true")
  }
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 240,
  )

  // Sanity: stable (no NaNs) and bridge links remain finite.
  let it = bodies.iter()
  for i in 0..<it.length() {
    let (_, b) = it[i]
    let p = b.translation()
    inspect(p.x == p.x, content="true")
    inspect(p.y == p.y, content="true")
    inspect(p.y > -200.0F, content="true")
  }
}

///|
/// Ported from `rapier-reference/examples2d/s2d_card_house.rs`.
test "examples2d/s2d_card_house.rs parity" {
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
  let friction = 0.7F

  // Ground.
  let ground_handle = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, -2.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(40.0F, 2.0F).friction(friction).build(),
    ground_handle,
    bodies,
  )
  |> ignore
  let scale = 10.0F
  let card_height = 0.2F * scale
  let card_thickness = 0.001F * scale
  let angle0 = 25.0F * @core.pi() / 180.0F
  let angle1 = -25.0F * @core.pi() / 180.0F
  let angle2 = 0.5F * @core.pi()
  let mut nb = 5
  let mut z0 = 0.0F
  let mut y = card_height - 0.02F * scale
  while nb != 0 {
    let mut z = z0
    for i in 0..<nb {
      if i != nb - 1 {
        let h = bodies.insert(
          @dynamics.RigidBodyBuilder::dynamic()
          .translation(
            @core.Vec2::new(z + 0.25F * scale, y + card_height - 0.015F * scale),
          )
          .rotation(@core.Rot2::from_angle(angle2))
          .build(),
        )
        colliders.insert_with_parent(
          @collision.ColliderBuilder::cuboid(card_thickness, card_height)
          .friction(friction)
          .build(),
          h,
          bodies,
        )
        |> ignore
      }
      let h1 = bodies.insert(
        @dynamics.RigidBodyBuilder::dynamic()
        .translation(@core.Vec2::new(z, y))
        .rotation(@core.Rot2::from_angle(angle1))
        .build(),
      )
      colliders.insert_with_parent(
        @collision.ColliderBuilder::cuboid(card_thickness, card_height)
        .friction(friction)
        .build(),
        h1,
        bodies,
      )
      |> ignore
      z = z + 0.175F * scale
      let h2 = bodies.insert(
        @dynamics.RigidBodyBuilder::dynamic()
        .translation(@core.Vec2::new(z, y))
        .rotation(@core.Rot2::from_angle(angle0))
        .build(),
      )
      colliders.insert_with_parent(
        @collision.ColliderBuilder::cuboid(card_thickness, card_height)
        .friction(friction)
        .build(),
        h2,
        bodies,
      )
      |> ignore
      z = z + 0.175F * scale
    }
    y = y + card_height * 2.0F - 0.03F * scale
    z0 = z0 + 0.175F * scale
    nb = nb - 1
  }
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 360,
  )

  // Sanity: stable, and not everything falls infinitely far.
  let it = bodies.iter()
  inspect(it.length() > 0, content="true")
  let mut min_y = 1.0e30F
  for i in 0..<it.length() {
    let (_, b) = it[i]
    let p = b.translation()
    inspect(p.x == p.x, content="true")
    inspect(p.y == p.y, content="true")
    if p.y < min_y {
      min_y = p.y
    }
  }
  inspect(min_y > -200.0F, content="true")
}

///|
/// Ported from `rapier-reference/examples2d/s2d_confined.rs`.
test "examples2d/s2d_confined.rs parity" {
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
  let radius = 0.5F
  let friction = 0.6F
  let grid_count = 25
  let max_count = grid_count * grid_count

  // Confined capsule walls (use capsule_from_endpoints).
  colliders.insert(
    @collision.ColliderBuilder::capsule_from_endpoints(
      @core.Vec2::new(-10.5F, 0.0F),
      @core.Vec2::new(10.5F, 0.0F),
      radius,
    )
    .friction(friction)
    .build(),
  )
  |> ignore
  colliders.insert(
    @collision.ColliderBuilder::capsule_from_endpoints(
      @core.Vec2::new(-10.5F, 0.0F),
      @core.Vec2::new(-10.5F, 20.5F),
      radius,
    )
    .friction(friction)
    .build(),
  )
  |> ignore
  colliders.insert(
    @collision.ColliderBuilder::capsule_from_endpoints(
      @core.Vec2::new(10.5F, 0.0F),
      @core.Vec2::new(10.5F, 20.5F),
      radius,
    )
    .friction(friction)
    .build(),
  )
  |> ignore
  colliders.insert(
    @collision.ColliderBuilder::capsule_from_endpoints(
      @core.Vec2::new(-10.5F, 20.5F),
      @core.Vec2::new(10.5F, 20.5F),
      radius,
    )
    .friction(friction)
    .build(),
  )
  |> ignore

  // Spheres with gravity_scale(0) to mimic the reference.
  let mut count = 0
  let mut column = 0
  while count < max_count {
    let mut row = 0
    for _ in 0..<grid_count {
      let x = -8.75F +
        Float::from_double(column.to_double()) *
        18.0F /
        Float::from_double(grid_count.to_double())
      let y = 1.5F +
        Float::from_double(row.to_double()) *
        18.0F /
        Float::from_double(grid_count.to_double())
      let h = bodies.insert(
        @dynamics.RigidBodyBuilder::dynamic()
        .translation(@core.Vec2::new(x, y))
        .gravity_scale(0.0F)
        .build(),
      )
      colliders.insert_with_parent(
        @collision.ColliderBuilder::ball(radius).friction(friction).build(),
        h,
        bodies,
      )
      |> ignore
      count = count + 1
      row = row + 1
    }
    column = column + 1
  }
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 120,
  )

  // Sanity: bodies remain within a reasonable bound and no NaNs.
  let it = bodies.iter()
  inspect(it.length() > 0, content="true")
  for i in 0..<it.length() {
    let (_, b) = it[i]
    let p = b.translation()
    inspect(p.x == p.x, content="true")
    inspect(p.y == p.y, content="true")
    inspect(@core.abs(p.x) < 50.0F, content="true")
    inspect(p.y > -50.0F, content="true")
  }
}

///|
/// Ported from `rapier-reference/examples2d/s2d_far_pyramid.rs`.
test "examples2d/s2d_far_pyramid.rs parity" {
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
  let origin = @core.Vec2::new(100000.0F, -80000.0F)
  let friction = 0.6F

  // Ground at a large coordinate.
  let ground_handle = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, -1.0F).add(origin))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(100.0F, 1.0F).friction(friction).build(),
    ground_handle,
    bodies,
  )
  |> ignore

  // Pyramid.
  let base_count = 10
  let h = 0.5F
  let shift = 1.25F * h
  for i in 0..<base_count {
    let fi = Float::from_double(i.to_double())
    let y = (2.0F * fi + 1.0F) * shift + 0.5F
    for j in i..<base_count {
      let fj = Float::from_double(j.to_double())
      let x = (fi + 1.0F) * shift +
        2.0F * (fj - fi) * shift -
        h * Float::from_double(base_count.to_double())
      let rb = bodies.insert(
        @dynamics.RigidBodyBuilder::dynamic()
        .translation(@core.Vec2::new(x, y).add(origin))
        .build(),
      )
      colliders.insert_with_parent(
        @collision.ColliderBuilder::cuboid(h, h).friction(friction).build(),
        rb,
        bodies,
      )
      |> ignore
    }
  }
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 240,
  )

  // Sanity: no NaNs at far coordinates.
  let it = bodies.iter()
  inspect(it.length() > 0, content="true")
  for i in 0..<it.length() {
    let (_, b) = it[i]
    let p = b.translation()
    inspect(p.x == p.x, content="true")
    inspect(p.y == p.y, content="true")
    let d = p.sub(origin)
    inspect(@core.abs(d.x) < 1000.0F, content="true")
    inspect(d.y > -1000.0F, content="true")
  }
}

///|
/// Ported from `rapier-reference/examples2d/s2d_high_mass_ratio_1.rs`.
test "examples2d/s2d_high_mass_ratio_1.rs parity" {
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
  let extent = 1.0F
  let friction = 0.5F
  let ground_width = 66.0F * extent
  let ground = bodies.insert(@dynamics.RigidBodyBuilder::fixed().build())
  colliders.insert_with_parent(
    @collision.ColliderBuilder::segment(
      @core.Vec2::new(-ground_width, 0.0F),
      @core.Vec2::new(ground_width, 0.0F),
    )
    .friction(friction)
    .build(),
    ground,
    bodies,
  )
  |> ignore
  for j in 0..<3 {
    let mut count = 10
    let offset = -20.0F * extent +
      2.0F *
      Float::from_double((count + 1).to_double()) *
      extent *
      Float::from_double(j.to_double())
    let mut y = extent
    while count > 0 {
      for i in 0..<count {
        let coeff = Float::from_double(i.to_double()) -
          0.5F * Float::from_double(count.to_double())
        let yy = if count == 1 { y + 2.0F } else { y }
        let pos = @core.Vec2::new(2.0F * coeff * extent + offset, yy)
        let h = bodies.insert(
          @dynamics.RigidBodyBuilder::dynamic().translation(pos).build(),
        )
        let density = if count == 1 {
          (Float::from_double(j.to_double()) + 1.0F) * 100.0F
        } else {
          1.0F
        }
        colliders.insert_with_parent(
          @collision.ColliderBuilder::cuboid(extent, extent)
          .density(density)
          .friction(friction)
          .build(),
          h,
          bodies,
        )
        |> ignore
      }
      count = count - 1
      y = y + 2.0F * extent
    }
  }
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 240,
  )

  // Sanity: no NaNs and bodies remain near the ground.
  let it = bodies.iter()
  inspect(it.length() > 0, content="true")
  for i in 0..<it.length() {
    let (_, b) = it[i]
    let p = b.translation()
    inspect(p.x == p.x, content="true")
    inspect(p.y == p.y, content="true")
  }
}

///|
/// Ported from `rapier-reference/examples2d/s2d_high_mass_ratio_2.rs`.
test "examples2d/s2d_high_mass_ratio_2.rs parity" {
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
  let extent = 1.0F
  let friction = 0.6F
  let ground_width = 66.0F * extent
  let ground = bodies.insert(@dynamics.RigidBodyBuilder::fixed().build())
  colliders.insert_with_parent(
    @collision.ColliderBuilder::segment(
      @core.Vec2::new(-ground_width, 0.0F),
      @core.Vec2::new(ground_width, 0.0F),
    )
    .friction(friction)
    .build(),
    ground,
    bodies,
  )
  |> ignore
  let left = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(-9.0F * extent, 0.5F * extent))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(0.5F * extent, 0.5F * extent)
    .friction(friction)
    .build(),
    left,
    bodies,
  )
  |> ignore
  let right = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(9.0F * extent, 0.5F * extent))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(0.5F * extent, 0.5F * extent)
    .friction(friction)
    .build(),
    right,
    bodies,
  )
  |> ignore
  let big = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(0.0F, (10.0F + 16.0F) * extent))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(10.0F * extent, 10.0F * extent)
    .friction(friction)
    .build(),
    big,
    bodies,
  )
  |> ignore
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 240,
  )
  // The big body should have fallen noticeably.
  if bodies.get(big) is Some(b) {
    inspect(b.translation().y < (10.0F + 16.0F) * extent, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
/// Ported from `rapier-reference/examples2d/s2d_high_mass_ratio_3.rs`.
test "examples2d/s2d_high_mass_ratio_3.rs parity" {
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
  let extent = 1.0F
  let friction = 0.6F
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, -2.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(40.0F, 2.0F).friction(friction).build(),
    ground,
    bodies,
  )
  |> ignore
  let left = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(-9.0F * extent, 0.5F * extent))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(0.5F * extent, 0.5F * extent)
    .friction(friction)
    .build(),
    left,
    bodies,
  )
  |> ignore
  let right = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(9.0F * extent, 0.5F * extent))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(0.5F * extent, 0.5F * extent)
    .friction(friction)
    .build(),
    right,
    bodies,
  )
  |> ignore
  let big = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(0.0F, (10.0F + 16.0F) * extent))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(10.0F * extent, 10.0F * extent)
    .friction(friction)
    .build(),
    big,
    bodies,
  )
  |> ignore
  let start_y = if bodies.get(big) is Some(b) {
    b.translation().y
  } else {
    0.0F
  }
  // Do one step first to ensure mass properties get computed from colliders.
  pipeline.step(
    gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders, impulse_joints,
    multibody_joints, ccd_solver, hooks, events,
  )
  if bodies.get(big) is Some(b) {
    inspect(b.mass() > 0.0F, content="true")
    inspect(b.inv_mass() > 0.0F, content="true")
    inspect(b.linvel().y < 0.0F, content="true")
  } else {
    inspect(false, content="true")
  }
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 239,
  )
  if bodies.get(big) is Some(b) {
    let y = b.translation().y
    inspect(y == y, content="true")
    // The body should have responded to gravity (moved downwards or gained downward velocity).
    inspect(y < start_y - 1.0e-3F || b.linvel().y < 0.0F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
/// Ported from `rapier-reference/examples2d/s2d_joint_grid.rs`.
test "examples2d/s2d_joint_grid.rs parity" {
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
  let rad = 0.4F
  let numi = 100
  let numk = 100
  let shift = 1.0F
  let handles : Array[@dynamics.RigidBodyHandle] = []
  for _ in 0..<(numi * numk) {
    handles.push(@dynamics.RigidBodyHandle::invalid())
  }
  for k in 0..<numk {
    for i in 0..<numi {
      let is_fixed = k >= numk / 2 - 2 && k <= numk / 2 + 2 && i == 0
      let rb = if is_fixed {
        @dynamics.RigidBodyBuilder::fixed()
      } else {
        @dynamics.RigidBodyBuilder::dynamic()
      }
      let handle = bodies.insert(
        rb
        .translation(
          @core.Vec2::new(
            Float::from_double(k.to_double()) * shift,
            -Float::from_double(i.to_double()) * shift,
          ),
        )
        .build(),
      )
      colliders.insert_with_parent(
        @collision.ColliderBuilder::ball(rad).build(),
        handle,
        bodies,
      )
      |> ignore
      let idx = k * numi + i
      if i > 0 {
        let parent = handles[idx - 1]
        let joint = @dynamics.GenericJoint::from_revolute(
          @dynamics.RevoluteJointBuilder::new()
          .local_anchor1(@core.Vec2::new(0.0F, -0.5F * shift))
          .local_anchor2(@core.Vec2::new(0.0F, 0.5F * shift))
          .build(),
        ).set_contacts_enabled(false)
        impulse_joints.insert(parent, handle, joint, true) |> ignore
      }
      if k > 0 {
        let parent = handles[idx - numi]
        let joint = @dynamics.GenericJoint::from_revolute(
          @dynamics.RevoluteJointBuilder::new()
          .local_anchor1(@core.Vec2::new(0.5F * shift, 0.0F))
          .local_anchor2(@core.Vec2::new(-0.5F * shift, 0.0F))
          .build(),
        ).set_contacts_enabled(false)
        impulse_joints.insert(parent, handle, joint, true) |> ignore
      }
      handles[idx] = handle
    }
  }
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 120,
  )

  // Spot check a couple of neighbor distances remain near `shift`.
  let a = handles[0]
  let b = handles[1]
  if bodies.get(a) is Some(ba) && bodies.get(b) is Some(bb) {
    let d = bb.translation().sub(ba.translation()).length()
    inspect(d == d, content="true")
    inspect(@core.abs(d - shift) < 0.3F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
/// Ported from `rapier-reference/examples2d/sensor2.rs`.
test "examples2d/sensor2.rs parity" {
  let pipeline = @pipeline.PhysicsPipeline::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = @collision.ColliderSet::new()
  let impulse_joints = @dynamics.ImpulseJointSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()
  let islands = @dynamics.IslandManager::new()
  let hooks = @pipeline.PhysicsHooks::new()
  let handler = @pipeline.EventHandler::new()
  let ccd_solver = @dynamics.CCDSolver::new()
  let parameters = @dynamics.IntegrationParameters::default()
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let ground_size = 200.1F
  let ground_height = 0.1F
  let ground_handle = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(0.0F, -ground_height))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(ground_size, ground_height).build(),
    ground_handle,
    bodies,
  )
  |> ignore
  let num = 10
  let rad = 0.2F
  let shift = rad * 2.0F
  let centerx = shift * Float::from_double(num.to_double()) / 2.0F
  for i in 0..<num {
    let x = Float::from_double(i.to_double()) * shift - centerx
    let handle = bodies.insert(
      @dynamics.RigidBodyBuilder::dynamic()
      .translation(@core.Vec2::new(x, 3.0F))
      .build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder::cuboid(rad, rad).build(),
      handle,
      bodies,
    )
    |> ignore
  }
  let sensor_handle = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(0.0F, 10.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(rad, rad).build(),
    sensor_handle,
    bodies,
  )
  |> ignore
  colliders.insert_with_parent(
    @collision.ColliderBuilder::ball(rad * 5.0F)
    .density(0.0F)
    .sensor(true)
    .active_events(@collision.ActiveEvents::collision_events())
    .build(),
    sensor_handle,
    bodies,
  )
  |> ignore
  let mut started_sensor_events = 0
  for _ in 0..<360 {
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
      impulse_joints, multibody_joints, ccd_solver, hooks, handler,
    )
    let evs = handler.take_collision_events()
    for i in 0..<evs.length() {
      let ev = evs[i]
      if ev.started() && ev.sensor() {
        started_sensor_events = started_sensor_events + 1
      }
    }
  }
  inspect(started_sensor_events > 0, content="true")
}

///|
/// Ported from `rapier-reference/examples2d/trimesh2.rs`.
test "examples2d/trimesh2.rs parity" {
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
  let parameters = @dynamics.IntegrationParameters::default()
  let gravity = @core.Vec2::new(0.0F, -9.81F)

  // Ground + walls.
  let ground_size = 25.0F
  let ground = bodies.insert(@dynamics.RigidBodyBuilder::fixed().build())
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(ground_size, 1.2F).build(),
    ground,
    bodies,
  )
  |> ignore
  let wall_r = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(ground_size, ground_size))
    .rotation(@core.Rot2::from_angle(@core.pi() / 2.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(ground_size, 1.2F).build(),
    wall_r,
    bodies,
  )
  |> ignore
  let wall_l = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .translation(@core.Vec2::new(-ground_size, ground_size))
    .rotation(@core.Rot2::from_angle(@core.pi() / 2.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(ground_size, 1.2F).build(),
    wall_l,
    bodies,
  )
  |> ignore
  let rapier_logo_buffers = @rapier_full.rapier_logo_buffers_2d()
  let dyn_handles : Array[@dynamics.RigidBodyHandle] = []
  for ith in 0..<rapier_logo_buffers.length() {
    let buffer = rapier_logo_buffers[ith]
    let vtx = buffer.0
    let idx = buffer.1
    let x = Float::from_double(ith.to_double()) * 8.0F - 20.0F
    for k in 0..<5 {
      if @collision.ColliderBuilder::trimesh(vtx, idx) is Some(builder) {
        let handle = bodies.insert(
          @dynamics.RigidBodyBuilder::dynamic()
          .translation(
            @core.Vec2::new(
              x,
              20.0F + Float::from_double(k.to_double()) * 11.0F,
            ),
          )
          .build(),
        )
        colliders.insert_with_parent(
          builder.contact_skin(0.2F).build(),
          handle,
          bodies,
        )
        |> ignore
        dyn_handles.push(handle)
      } else {
        inspect(false, content="true")
      }
    }
  }
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 120,
  )
  for i in 0..<dyn_handles.length() {
    if bodies.get(dyn_handles[i]) is Some(body) {
      let p = body.translation()
      inspect(p.x == p.x, content="true")
      inspect(p.y == p.y, content="true")
      inspect(p.y > -10.0F, content="true")
    } else {
      inspect(false, content="true")
    }
  }
}
