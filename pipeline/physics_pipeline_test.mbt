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
/// Ported from rapier/src/pipeline/physics_pipeline.rs (kinematic_and_fixed_contact_crash).
test "kinematic and fixed contact crash" {
  let colliders = @collision.ColliderSet::new()
  let impulse_joints = @dynamics.ImpulseJointSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()
  let pipeline = PhysicsPipeline::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let bodies = @dynamics.RigidBodySet::new()
  let islands = @dynamics.IslandManager::new()
  let rb_fixed = @dynamics.RigidBodyBuilder::fixed().build()
  let h1 = bodies.insert(rb_fixed)
  let co = @collision.ColliderBuilder::ball(10.0F).build()
  colliders.insert_with_parent(co, h1, bodies) |> ignore
  let rb_kinematic = @dynamics.RigidBodyBuilder::kinematic_position_based().build()
  let h2 = bodies.insert(rb_kinematic)
  let co2 = @collision.ColliderBuilder::ball(10.0F).build()
  colliders.insert_with_parent(co2, h2, bodies) |> ignore
  pipeline.step(
    @core.Vec2::zero(),
    IntegrationParameters::default(),
    islands,
    broad_phase,
    narrow_phase,
    bodies,
    colliders,
    impulse_joints,
    multibody_joints,
    @dynamics.CCDSolver::new(),
    PhysicsHooks::new(),
    EventHandler::new(),
  )
}

///|
test "mass properties recomputed from attached colliders (insert/remove)" {
  let pipeline = PhysicsPipeline::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = @collision.ColliderSet::new()
  let islands = @dynamics.IslandManager::new()
  let impulse_joints = @dynamics.ImpulseJointSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()
  let rb = @dynamics.RigidBodyBuilder::dynamic().build()
  let rb_handle = bodies.insert(rb)
  let co = @collision.ColliderBuilder::ball(1.0F).build()
  let co_handle = colliders.insert_with_parent(co, rb_handle, bodies)
  pipeline.step(
    @core.Vec2::zero(),
    IntegrationParameters::default(),
    islands,
    broad_phase,
    narrow_phase,
    bodies,
    colliders,
    impulse_joints,
    multibody_joints,
    @dynamics.CCDSolver::new(),
    PhysicsHooks::new(),
    EventHandler::new(),
  )
  if bodies.get(rb_handle) is Some(body_after_insert) {
    inspect(body_after_insert.inv_mass() > 0.0F, content="true")
    inspect(body_after_insert.inv_inertia() > 0.0F, content="true")
  } else {
    inspect(false, content="true")
  }
  colliders.remove(co_handle, islands, bodies, true)
  pipeline.step(
    @core.Vec2::zero(),
    IntegrationParameters::default(),
    islands,
    broad_phase,
    narrow_phase,
    bodies,
    colliders,
    impulse_joints,
    multibody_joints,
    @dynamics.CCDSolver::new(),
    PhysicsHooks::new(),
    EventHandler::new(),
  )
  if bodies.get(rb_handle) is Some(body_after_remove) {
    inspect(body_after_remove.inv_mass() == 0.0F, content="true")
    inspect(body_after_remove.inv_inertia() == 0.0F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "kinematic position-based next_position advances and affects contacts" {
  let pipeline = PhysicsPipeline::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = @collision.ColliderSet::new()
  let islands = @dynamics.IslandManager::new()
  let impulse_joints = @dynamics.ImpulseJointSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()
  let params = IntegrationParameters::default()
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder::kinematic_position_based()
    .translation(@core.Vec2::new(0.0F, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(5.0F, 0.1F).friction(10.0F).build(),
    ground,
    bodies,
  )
  |> ignore
  let ball = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(0.0F, 1.1F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::ball(1.0F)
    .friction(10.0F)
    .restitution(0.0F)
    .build(),
    ball,
    bodies,
  )
  |> ignore

  // Let the ball settle on the kinematic ground.
  for _ in 0..<30 {
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
      PhysicsHooks::new(),
      EventHandler::new(),
    )
  }
  let start_x = if bodies.get(ball) is Some(body) {
    body.translation().x
  } else {
    0.0F
  }

  // Move the kinematic body along x using next_position. With friction, the ball should follow.
  for _ in 0..<10 {
    if bodies.get(ground) is Some(g) {
      let t = g.translation()
      if bodies.get_mut(ground) is Some(gm) {
        gm.set_next_kinematic_translation(@core.Vec2::new(t.x + 0.05F, t.y))
        |> ignore
      }
    }
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
      PhysicsHooks::new(),
      EventHandler::new(),
    )
  }
  if bodies.get(ground) is Some(g) {
    inspect(g.translation().x > 0.0F, content="true")
    inspect(@core.abs(g.linvel().x) > 0.0F, content="true")
  } else {
    inspect(false, content="true")
  }
  if bodies.get(ball) is Some(b) {
    inspect(b.translation().x > start_x + 0.01F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
/// Ported from rapier/src/pipeline/physics_pipeline.rs (rigid_body_removal_before_step).
test "rigid body removal before step" {
  let colliders = @collision.ColliderSet::new()
  let impulse_joints = @dynamics.ImpulseJointSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()
  let pipeline = PhysicsPipeline::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let islands = @dynamics.IslandManager::new()
  let bodies = @dynamics.RigidBodySet::new()
  let h1 = bodies.insert(@dynamics.RigidBodyBuilder::dynamic().build())
  let h2 = bodies.insert(@dynamics.RigidBodyBuilder::dynamic().build())
  let h3 = bodies.insert(
    @dynamics.RigidBodyBuilder::kinematic_position_based().build(),
  )
  let h4 = bodies.insert(@dynamics.RigidBodyBuilder::fixed().build())
  let to_delete = [h1, h2, h3, h4]
  for h in to_delete {
    bodies.remove(h, islands, (), impulse_joints, multibody_joints, true)
    |> ignore
  }
  pipeline.step(
    @core.Vec2::zero(),
    IntegrationParameters::default(),
    islands,
    broad_phase,
    narrow_phase,
    bodies,
    colliders,
    impulse_joints,
    multibody_joints,
    @dynamics.CCDSolver::new(),
    PhysicsHooks::new(),
    EventHandler::new(),
  )
}

///|
/// Ported from rapier/src/pipeline/physics_pipeline.rs (collider_removal_before_step).
test "collider removal before step" {
  let pipeline = PhysicsPipeline::new()
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let integration_parameters = IntegrationParameters::default()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = @collision.ColliderSet::new()
  let ccd = @dynamics.CCDSolver::new()
  let impulse_joints = @dynamics.ImpulseJointSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()
  let islands = @dynamics.IslandManager::new()
  let body = @dynamics.RigidBodyBuilder::dynamic().build()
  let b_handle = bodies.insert(body)
  let collider = @collision.ColliderBuilder::ball(1.0F).build()
  let c_handle = colliders.insert_with_parent(collider, b_handle, bodies)
  colliders.remove(c_handle, islands, bodies, true)
  bodies.remove(b_handle, islands, (), impulse_joints, multibody_joints, true)
  |> ignore
  for _ in 0..<10 {
    pipeline.step(
      gravity,
      integration_parameters,
      islands,
      broad_phase,
      narrow_phase,
      bodies,
      colliders,
      impulse_joints,
      multibody_joints,
      ccd,
      PhysicsHooks::new(),
      EventHandler::new(),
    )
  }
}

///|
/// Ported from rapier/src/pipeline/physics_pipeline.rs (rigid_body_type_changed_dynamic_is_in_active_set).
test "rigid body type changed dynamic is in active set" {
  let colliders = @collision.ColliderSet::new()
  let impulse_joints = @dynamics.ImpulseJointSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()
  let pipeline = PhysicsPipeline::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let islands = @dynamics.IslandManager::new()
  let bodies = @dynamics.RigidBodySet::new()
  let rb = @dynamics.RigidBodyBuilder::kinematic_position_based()
    .additional_mass(1.0F)
    .build()
  let h = bodies.insert(rb)
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  pipeline.step(
    gravity,
    IntegrationParameters::default(),
    islands,
    broad_phase,
    narrow_phase,
    bodies,
    colliders,
    impulse_joints,
    multibody_joints,
    @dynamics.CCDSolver::new(),
    PhysicsHooks::new(),
    EventHandler::new(),
  )
  if bodies.get_mut(h) is Some(body) {
    body.set_body_type(@dynamics.RigidBodyType::Dynamic, true) |> ignore
  }
  pipeline.step(
    gravity,
    IntegrationParameters::default(),
    islands,
    broad_phase,
    narrow_phase,
    bodies,
    colliders,
    impulse_joints,
    multibody_joints,
    @dynamics.CCDSolver::new(),
    PhysicsHooks::new(),
    EventHandler::new(),
  )
  if bodies.get(h) is Some(body_after) {
    let position = body_after.translation()
    inspect(position.y < 0.0F, content="true")
    inspect(body_after.is_sleeping(), content="false")
  }
}

///|
/// Ported from rapier/src/pipeline/physics_pipeline.rs (joint_step_delta_time_0).
test "joint step delta time 0" {
  let colliders = @collision.ColliderSet::new()
  let impulse_joints = @dynamics.ImpulseJointSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()
  let pipeline = PhysicsPipeline::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let islands = @dynamics.IslandManager::new()
  let bodies = @dynamics.RigidBodySet::new()
  let rb_fixed = @dynamics.RigidBodyBuilder::fixed()
    .additional_mass(1.0F)
    .build()
  let h_fixed = bodies.insert(rb_fixed)
  let rb_dynamic = @dynamics.RigidBodyBuilder::dynamic()
    .additional_mass(1.0F)
    .build()
  let h_dynamic = bodies.insert(rb_dynamic)
  let joint = @dynamics.GenericJoint::from_revolute(
    @dynamics.RevoluteJointBuilder::new()
    .local_anchor1(@core.Vec2::new(0.0F, 1.0F))
    .local_anchor2(@core.Vec2::new(0.0F, -3.0F))
    .build(),
  )
  impulse_joints.insert(h_fixed, h_dynamic, joint, true) |> ignore
  let parameters = @dynamics.IntegrationParameters::default().set_dt(0.0F)
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  pipeline.step(
    gravity,
    parameters,
    islands,
    broad_phase,
    narrow_phase,
    bodies,
    colliders,
    impulse_joints,
    multibody_joints,
    @dynamics.CCDSolver::new(),
    PhysicsHooks::new(),
    EventHandler::new(),
  )
  if bodies.get(h_dynamic) is Some(body) {
    let translation = body.translation()
    let rotation = body.rotation()
    inspect(translation.x == translation.x, content="true")
    inspect(translation.y == translation.y, content="true")
    inspect(rotation.sin == rotation.sin, content="true")
    inspect(rotation.cos == rotation.cos, content="true")
  }
}

///|
test "revolute impulse-joint keeps anchors coincident" {
  let colliders = @collision.ColliderSet::new()
  let impulse_joints = @dynamics.ImpulseJointSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()
  let pipeline = PhysicsPipeline::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let islands = @dynamics.IslandManager::new()
  let bodies = @dynamics.RigidBodySet::new()
  let h_fixed = bodies.insert(@dynamics.RigidBodyBuilder::fixed().build())
  let h_dynamic = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .additional_mass(1.0F)
    .translation(@core.Vec2::new(1.0F, 0.0F))
    .build(),
  )
  let local_anchor1 = @core.Vec2::zero()
  let local_anchor2 = @core.Vec2::new(-1.0F, 0.0F)
  let joint = @dynamics.GenericJoint::from_revolute(
    @dynamics.RevoluteJointBuilder::new()
    .local_anchor1(local_anchor1)
    .local_anchor2(local_anchor2)
    .build(),
  )
  impulse_joints.insert(h_fixed, h_dynamic, joint, true) |> ignore
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    8,
  )
  let mut max_err = 0.0F
  for _ in 0..<120 {
    pipeline.step(
      gravity,
      parameters,
      islands,
      broad_phase,
      narrow_phase,
      bodies,
      colliders,
      impulse_joints,
      multibody_joints,
      @dynamics.CCDSolver::new(),
      PhysicsHooks::new(),
      EventHandler::new(),
    )
    if bodies.get(h_fixed) is Some(b1) && bodies.get(h_dynamic) is Some(b2) {
      let world1 = b1.position().transform_point(local_anchor1)
      let world2 = b2.position().transform_point(local_anchor2)
      let err = world1.sub(world2).length()
      if err > max_err {
        max_err = err
      }
    }
  }
  inspect(max_err < 1.0e-2F, content="true")
}

///|
test "revolute impulse-joint motor position reaches target angle" {
  let colliders = @collision.ColliderSet::new()
  let impulse_joints = @dynamics.ImpulseJointSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()
  let pipeline = PhysicsPipeline::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let islands = @dynamics.IslandManager::new()
  let bodies = @dynamics.RigidBodySet::new()
  let h_fixed = bodies.insert(@dynamics.RigidBodyBuilder::fixed().build())
  let h_dynamic = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .additional_mass(1.0F)
    .translation(@core.Vec2::zero())
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::ball(0.5F).build(),
    h_dynamic,
    bodies,
  )
  |> ignore
  // Sanity check: angular velocity integration affects rotation.
  if bodies.get_mut(h_dynamic) is Some(bm) {
    bm.set_angvel(10.0F, true) |> ignore
  }
  pipeline.step(
    @core.Vec2::zero(),
    @dynamics.IntegrationParameters::default().set_num_solver_iterations(1),
    islands,
    broad_phase,
    narrow_phase,
    bodies,
    colliders,
    impulse_joints,
    multibody_joints,
    @dynamics.CCDSolver::new(),
    PhysicsHooks::new(),
    EventHandler::new(),
  )
  if bodies.get(h_dynamic) is Some(after_spin) {
    inspect(@core.abs(after_spin.rotation().angle()) > 0.01F, content="true")
  } else {
    inspect(false, content="true")
  }
  if bodies.get_mut(h_dynamic) is Some(bm) {
    bm.set_angvel(0.0F, true) |> ignore
    bm.set_rotation(@core.Rot2::identity(), true) |> ignore
  }
  let target = @core.pi() / 2.0F
  let joint = @dynamics.GenericJoint::from_revolute(
    @dynamics.RevoluteJointBuilder::new()
    .motor_position(target, 20.0F, 1.0F)
    .motor_max_force(1000.0F)
    .build(),
  )
  let joint_handle = impulse_joints.insert(h_fixed, h_dynamic, joint, true)
  if impulse_joints.get(joint_handle) is Some(stored) {
    let motor_axes = stored.data.motor_axes()
    inspect(
      motor_axes.contains(@dynamics.JointAxesMask::ang_z()),
      content="true",
    )
    let motor = stored.data.motors().ang_z
    inspect(@core.abs(motor.target_pos - target) < 1.0e-6F, content="true")
    inspect(motor.stiffness > 0.0F, content="true")
  } else {
    inspect(false, content="true")
  }
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )
  for _ in 0..<240 {
    pipeline.step(
      @core.Vec2::zero(),
      parameters,
      islands,
      broad_phase,
      narrow_phase,
      bodies,
      colliders,
      impulse_joints,
      multibody_joints,
      @dynamics.CCDSolver::new(),
      PhysicsHooks::new(),
      EventHandler::new(),
    )
  }
  if bodies.get(h_dynamic) is Some(body) {
    inspect(body.inv_inertia() > 0.0F, content="true")
    let angle = body.rotation().angle()
    inspect(angle == angle, content="true")
    inspect(@core.abs(angle) > 0.1F, content="true")
    let diff = @core.abs(angle - target)
    inspect(diff < 0.2F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
test "segment cuboid deep penetration resolves (no residual intersection)" {
  fn segment_world_endpoints(
    center : @core.Vec2,
    rotation : @core.Real,
    a : @core.Vec2,
    b : @core.Vec2,
  ) -> (@core.Vec2, @core.Vec2) {
    let rot = @core.Rot2::from_angle(rotation)
    (center.add(rot.rotate_vec2(a)), center.add(rot.rotate_vec2(b)))
  }

  fn segment_aabb_intersect(
    a : @core.Vec2,
    b : @core.Vec2,
    hw : @core.Real,
    hh : @core.Real,
  ) -> Bool {
    let mut t0 = 0.0F
    let mut t1 = 1.0F
    let d = b.sub(a)
    let p = [-d.x, d.x, -d.y, d.y]
    let q = [a.x + hw, hw - a.x, a.y + hh, hh - a.y]
    for i in 0..<4 {
      let pi = p[i]
      let qi = q[i]
      if @core.abs(pi) <= 1.0e-12F {
        if qi < 0.0F {
          return false
        }
      } else {
        let r = qi / pi
        if pi < 0.0F {
          if r > t1 {
            return false
          }
          if r > t0 {
            t0 = r
          }
        } else {
          if r < t0 {
            return false
          }
          if r < t1 {
            t1 = r
          }
        }
      }
    }
    true
  }

  let pipeline = PhysicsPipeline::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = @collision.ColliderSet::new()
  let islands = @dynamics.IslandManager::new()
  let impulse_joints = @dynamics.ImpulseJointSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()
  let ccd = @dynamics.CCDSolver::new()
  let fixed = bodies.insert(@dynamics.RigidBodyBuilder::fixed().build())
  let seg = @collision.ColliderBuilder::segment(
    @core.Vec2::new(-2.0F, -2.0F),
    @core.Vec2::new(2.0F, 2.0F),
  ).build()
  let seg_handle = colliders.insert_with_parent(seg, fixed, bodies)
  let dyn_body = bodies.insert(@dynamics.RigidBodyBuilder::dynamic().build())
  let cub = @collision.ColliderBuilder::cuboid(0.5F, 0.2F)
    .rotation(0.3F)
    .build()
  let cub_handle = colliders.insert_with_parent(cub, dyn_body, bodies)

  // Start with a deep intersection: both shapes overlap at the origin.
  pipeline.step(
    @core.Vec2::zero(),
    IntegrationParameters::default(),
    islands,
    broad_phase,
    narrow_phase,
    bodies,
    colliders,
    impulse_joints,
    multibody_joints,
    ccd,
    PhysicsHooks::new(),
    EventHandler::new(),
  )
  if colliders.get(seg_handle) is Some(seg_after) &&
    colliders.get(cub_handle) is Some(cub_after) {
    match (seg_after.shape(), cub_after.shape()) {
      (@collision.Shape::Segment(a, b), @collision.Shape::Cuboid(hw, hh)) => {
        let (sa, sb) = segment_world_endpoints(
          seg_after.translation(),
          seg_after.rotation(),
          a,
          b,
        )
        let rot = @core.Rot2::from_angle(cub_after.rotation())
        let inv = rot.inverse()
        let la = inv.rotate_vec2(sa.sub(cub_after.translation()))
        let lb = inv.rotate_vec2(sb.sub(cub_after.translation()))
        inspect(segment_aabb_intersect(la, lb, hw, hh) == false, content="true")
      }
      _ => inspect(false, content="true")
    }
  } else {
    inspect(false, content="true")
  }
}

///|
test "physics pipeline emits collision started/stopped events" {
  let pipeline = PhysicsPipeline::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = @collision.ColliderSet::new()
  let impulse_joints = @dynamics.ImpulseJointSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()
  let islands = @dynamics.IslandManager::new()
  let callback_events : Array[@collision.CollisionEvent] = []
  let handler = EventHandler::new().on_collision_event(fn(
    _bodies : @dynamics.RigidBodySet,
    _colliders : @collision.ColliderSet,
    event : @collision.CollisionEvent,
  ) -> Unit {
    callback_events.push(event)
  })
  let params = IntegrationParameters::default()
  let b1 = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(0.0F, 0.0F))
    .build(),
  )
  let b2 = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(0.5F, 0.0F))
    .build(),
  )
  let co = @collision.ColliderBuilder::ball(1.0F)
    .sensor(true)
    .active_events(@collision.ActiveEvents::collision_events())
    .build()
  colliders.insert_with_parent(co, b1, bodies) |> ignore
  let co2 = @collision.ColliderBuilder::ball(1.0F)
    .sensor(true)
    .active_events(@collision.ActiveEvents::collision_events())
    .build()
  colliders.insert_with_parent(co2, b2, bodies) |> ignore
  pipeline.step(
    @core.Vec2::zero(),
    params,
    islands,
    broad_phase,
    narrow_phase,
    bodies,
    colliders,
    impulse_joints,
    multibody_joints,
    @dynamics.CCDSolver::new(),
    PhysicsHooks::new(),
    handler,
  )
  let ev1 = handler.take_collision_events()
  inspect(ev1.length() == 1, content="true")
  inspect(callback_events.length() == 1, content="true")
  match ev1[0] {
    @collision.CollisionEvent::Started(_, _, flags) =>
      inspect(
        flags.contains(@collision.CollisionEventFlags::sensor()),
        content="true",
      )
    _ => inspect(false, content="true")
  }
  pipeline.step(
    @core.Vec2::zero(),
    params,
    islands,
    broad_phase,
    narrow_phase,
    bodies,
    colliders,
    impulse_joints,
    multibody_joints,
    @dynamics.CCDSolver::new(),
    PhysicsHooks::new(),
    handler,
  )
  inspect(handler.take_collision_events().length() == 0, content="true")
  inspect(callback_events.length() == 1, content="true")
  if bodies.get_mut(b2) is Some(body2) {
    body2.set_translation(@core.Vec2::new(10.0F, 0.0F), true) |> ignore
  }
  pipeline.step(
    @core.Vec2::zero(),
    params,
    islands,
    broad_phase,
    narrow_phase,
    bodies,
    colliders,
    impulse_joints,
    multibody_joints,
    @dynamics.CCDSolver::new(),
    PhysicsHooks::new(),
    handler,
  )
  let ev2 = handler.take_collision_events()
  inspect(ev2.length() == 1, content="true")
  inspect(callback_events.length() == 2, content="true")
  match ev2[0] {
    @collision.CollisionEvent::Stopped(_, _, _) => inspect(true, content="true")
    _ => inspect(false, content="true")
  }
}

///|
test "physics pipeline stopped event includes removed flag (sensor pair)" {
  let pipeline = PhysicsPipeline::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = @collision.ColliderSet::new()
  let impulse_joints = @dynamics.ImpulseJointSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()
  let islands = @dynamics.IslandManager::new()
  let handler = EventHandler::new()
  let params = IntegrationParameters::default()
  let b1 = bodies.insert(@dynamics.RigidBodyBuilder::dynamic().build())
  let b2 = bodies.insert(@dynamics.RigidBodyBuilder::dynamic().build())
  let c1 = @collision.ColliderBuilder::ball(1.0F)
    .sensor(true)
    .active_events(@collision.ActiveEvents::collision_events())
    .build()
  let c2 = @collision.ColliderBuilder::ball(1.0F)
    .sensor(true)
    .active_events(@collision.ActiveEvents::collision_events())
    .build()
  colliders.insert_with_parent(c1, b1, bodies) |> ignore
  let h2 = colliders.insert_with_parent(c2, b2, bodies)
  pipeline.step(
    @core.Vec2::zero(),
    params,
    islands,
    broad_phase,
    narrow_phase,
    bodies,
    colliders,
    impulse_joints,
    multibody_joints,
    @dynamics.CCDSolver::new(),
    PhysicsHooks::new(),
    handler,
  )
  handler.take_collision_events() |> ignore
  colliders.remove(h2, islands, bodies, true)
  pipeline.step(
    @core.Vec2::zero(),
    params,
    islands,
    broad_phase,
    narrow_phase,
    bodies,
    colliders,
    impulse_joints,
    multibody_joints,
    @dynamics.CCDSolver::new(),
    PhysicsHooks::new(),
    handler,
  )
  let ev = handler.take_collision_events()
  inspect(ev.length() == 1, content="true")
  match ev[0] {
    @collision.CollisionEvent::Stopped(_, _, flags) => {
      inspect(
        flags.contains(@collision.CollisionEventFlags::sensor()),
        content="true",
      )
      inspect(
        flags.contains(@collision.CollisionEventFlags::removed()),
        content="true",
      )
    }
    _ => inspect(false, content="true")
  }
}

///|
test "physics pipeline emits stopped event after removal even if other collider has no events" {
  let pipeline = PhysicsPipeline::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = @collision.ColliderSet::new()
  let impulse_joints = @dynamics.ImpulseJointSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()
  let islands = @dynamics.IslandManager::new()
  let handler = EventHandler::new()
  let params = IntegrationParameters::default()
  let b1 = bodies.insert(@dynamics.RigidBodyBuilder::dynamic().build())
  let b2 = bodies.insert(@dynamics.RigidBodyBuilder::dynamic().build())
  let c1 = @collision.ColliderBuilder::ball(1.0F)
    .sensor(true)
    .active_events(@collision.ActiveEvents::collision_events())
    .build()
  let c2 = @collision.ColliderBuilder::ball(1.0F).sensor(true).build()
  let h1 = colliders.insert_with_parent(c1, b1, bodies)
  colliders.insert_with_parent(c2, b2, bodies) |> ignore
  pipeline.step(
    @core.Vec2::zero(),
    params,
    islands,
    broad_phase,
    narrow_phase,
    bodies,
    colliders,
    impulse_joints,
    multibody_joints,
    @dynamics.CCDSolver::new(),
    PhysicsHooks::new(),
    handler,
  )
  let started = handler.take_collision_events()
  inspect(started.length() == 1, content="true")
  colliders.remove(h1, islands, bodies, true)
  pipeline.step(
    @core.Vec2::zero(),
    params,
    islands,
    broad_phase,
    narrow_phase,
    bodies,
    colliders,
    impulse_joints,
    multibody_joints,
    @dynamics.CCDSolver::new(),
    PhysicsHooks::new(),
    handler,
  )
  let stopped = handler.take_collision_events()
  inspect(stopped.length() == 1, content="true")
  match stopped[0] {
    @collision.CollisionEvent::Stopped(_, _, flags) =>
      inspect(
        flags.contains(@collision.CollisionEventFlags::removed()),
        content="true",
      )
    _ => inspect(false, content="true")
  }
}

///|
test "physics pipeline emits contact force events when enabled" {
  let pipeline = PhysicsPipeline::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = @collision.ColliderSet::new()
  let impulse_joints = @dynamics.ImpulseJointSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()
  let islands = @dynamics.IslandManager::new()
  let handler = EventHandler::new()
  let params = IntegrationParameters::default()
  let b1 = bodies.insert(@dynamics.RigidBodyBuilder::dynamic().build())
  let b2 = bodies.insert(@dynamics.RigidBodyBuilder::dynamic().build())
  let c1 = @collision.ColliderBuilder::ball(1.0F)
    .active_events(@collision.ActiveEvents::contact_force_events())
    .contact_force_event_threshold(0.0F)
    .build()
  let c2 = @collision.ColliderBuilder::ball(1.0F).build()
  let h1 = colliders.insert_with_parent(c1, b1, bodies)
  let h2 = colliders.insert_with_parent(c2, b2, bodies)
  pipeline.step(
    @core.Vec2::zero(),
    params,
    islands,
    broad_phase,
    narrow_phase,
    bodies,
    colliders,
    impulse_joints,
    multibody_joints,
    @dynamics.CCDSolver::new(),
    PhysicsHooks::new(),
    handler,
  )
  let events = handler.take_contact_force_events()
  inspect(events.length() == 1, content="true")
  let ev = events[0]
  let ok = (
      @collision.ColliderHandle::equals(ev.collider1(), h1) &&
      @collision.ColliderHandle::equals(ev.collider2(), h2)
    ) ||
    (
      @collision.ColliderHandle::equals(ev.collider1(), h2) &&
      @collision.ColliderHandle::equals(ev.collider2(), h1)
    )
  inspect(ok, content="true")
  inspect(ev.total_force_magnitude() > 0.0F, content="true")
}

///|
test "physics pipeline contact force events respect thresholds" {
  let pipeline = PhysicsPipeline::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = @collision.ColliderSet::new()
  let impulse_joints = @dynamics.ImpulseJointSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()
  let islands = @dynamics.IslandManager::new()
  let handler = EventHandler::new()
  let params = IntegrationParameters::default()
  let b1 = bodies.insert(@dynamics.RigidBodyBuilder::dynamic().build())
  let b2 = bodies.insert(@dynamics.RigidBodyBuilder::dynamic().build())
  let c1 = @collision.ColliderBuilder::ball(1.0F)
    .active_events(@collision.ActiveEvents::contact_force_events())
    .contact_force_event_threshold(1.0e9F)
    .build()
  let c2 = @collision.ColliderBuilder::ball(1.0F).build()
  colliders.insert_with_parent(c1, b1, bodies) |> ignore
  colliders.insert_with_parent(c2, b2, bodies) |> ignore
  pipeline.step(
    @core.Vec2::zero(),
    params,
    islands,
    broad_phase,
    narrow_phase,
    bodies,
    colliders,
    impulse_joints,
    multibody_joints,
    @dynamics.CCDSolver::new(),
    PhysicsHooks::new(),
    handler,
  )
  inspect(handler.take_contact_force_events().length() == 0, content="true")
}

///|
/// Ported from rapier/src/pipeline/physics_pipeline.rs (test_multi_sap_disable_body).
test "multi sap disable body" {
  let rigid_body_set = @dynamics.RigidBodySet::new()
  let collider_set = @collision.ColliderSet::new()
  let ground = @collision.ColliderBuilder::cuboid(100.0F, 0.1F).build()
  collider_set.insert(ground) |> ignore
  let rigid_body = @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(0.0F, 10.0F))
    .build()
  let collider = @collision.ColliderBuilder::ball(0.5F)
    .restitution(0.7F)
    .build()
  let ball_body_handle = rigid_body_set.insert(rigid_body)
  collider_set.insert_with_parent(collider, ball_body_handle, rigid_body_set)
  |> ignore
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let integration_parameters = IntegrationParameters::default()
  let physics_pipeline = PhysicsPipeline::new()
  let island_manager = @dynamics.IslandManager::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let impulse_joint_set = @dynamics.ImpulseJointSet::new()
  let multibody_joint_set = @dynamics.MultibodyJointSet::new()
  let ccd_solver = @dynamics.CCDSolver::new()
  physics_pipeline.step(
    gravity,
    integration_parameters,
    island_manager,
    broad_phase,
    narrow_phase,
    rigid_body_set,
    collider_set,
    impulse_joint_set,
    multibody_joint_set,
    ccd_solver,
    PhysicsHooks::new(),
    EventHandler::new(),
  )
  if rigid_body_set.get_mut(ball_body_handle) is Some(ball_body) {
    ball_body.set_translation(@core.Vec2::new(1.0F, 1.0F), true) |> ignore
    ball_body.set_rotation(@core.Rot2::from_angle(1.0F), true) |> ignore
    ball_body.set_enabled(false) |> ignore
  }
  physics_pipeline.step(
    gravity,
    integration_parameters,
    island_manager,
    broad_phase,
    narrow_phase,
    rigid_body_set,
    collider_set,
    impulse_joint_set,
    multibody_joint_set,
    ccd_solver,
    PhysicsHooks::new(),
    EventHandler::new(),
  )
  if rigid_body_set.get_mut(ball_body_handle) is Some(ball_body) {
    ball_body.set_translation(@core.Vec2::new(0.0F, 0.0F), true) |> ignore
    ball_body.set_rotation(@core.Rot2::from_angle(0.0F), true) |> ignore
    ball_body.set_enabled(true) |> ignore
  }
  physics_pipeline.step(
    gravity,
    integration_parameters,
    island_manager,
    broad_phase,
    narrow_phase,
    rigid_body_set,
    collider_set,
    impulse_joint_set,
    multibody_joint_set,
    ccd_solver,
    PhysicsHooks::new(),
    EventHandler::new(),
  )
}

///|
test "multibody joints participate in pipeline solver (revolute constraint)" {
  let pipeline = PhysicsPipeline::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = @collision.ColliderSet::new()
  let islands = @dynamics.IslandManager::new()
  let impulse_joints = @dynamics.ImpulseJointSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()
  let ccd_solver = @dynamics.CCDSolver::new()
  let params = IntegrationParameters::default()
  let gravity = @core.Vec2::zero()
  let parent = bodies.insert(@dynamics.RigidBodyBuilder::dynamic().build())
  let child = bodies.insert(@dynamics.RigidBodyBuilder::dynamic().build())
  colliders.insert_with_parent(
    @collision.ColliderBuilder::ball(0.5F).build(),
    parent,
    bodies,
  )
  |> ignore
  colliders.insert_with_parent(
    @collision.ColliderBuilder::ball(0.5F).build(),
    child,
    bodies,
  )
  |> ignore

  // A simple multibody with one revolute joint (locks the two linear DOFs).
  let joint = @dynamics.RevoluteJointBuilder::new().build()
  multibody_joints.insert(parent, child, joint, true) |> ignore

  // Give the child an initial velocity; without a solver-backed joint, it would drift away.
  if bodies.get_mut(child) is Some(rb) {
    rb.set_linvel(@core.Vec2::new(10.0F, 0.0F), true) |> ignore
  }
  for _ in 0..<30 {
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
      ccd_solver,
      PhysicsHooks::new(),
      EventHandler::new(),
    )
  }
  if bodies.get(parent) is Some(a) && bodies.get(child) is Some(b) {
    let d = a.translation().sub(b.translation()).length()
    inspect(d == d, content="true")
    inspect(d < 0.5F, content="true")
  } else {
    inspect(false, content="true")
  }
}

///|
/// Ported from rapier/src/pipeline/physics_pipeline.rs (rigid_body_removal_snapshot_handle_determinism).
test "rigid body removal snapshot handle determinism" {
  let colliders = @collision.ColliderSet::new()
  let impulse_joints = @dynamics.ImpulseJointSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()
  let islands = @dynamics.IslandManager::new()
  let bodies = @dynamics.RigidBodySet::new()
  let rb = @dynamics.RigidBodyBuilder::dynamic().build()
  let h1 = bodies.insert(rb)
  let h2 = bodies.insert(rb)
  let h3 = bodies.insert(rb)
  bodies.remove(h1, islands, (), impulse_joints, multibody_joints, true)
  |> ignore
  bodies.remove(h3, islands, (), impulse_joints, multibody_joints, true)
  |> ignore
  bodies.remove(h2, islands, (), impulse_joints, multibody_joints, true)
  |> ignore
  let snapshot = bodies.serialize()
  let bodies2 = @dynamics.RigidBodySet::deserialize(snapshot)
  let h1a = bodies.insert(rb)
  let h2a = bodies.insert(rb)
  let h3a = bodies.insert(rb)
  let h1b = bodies2.insert(rb)
  let h2b = bodies2.insert(rb)
  let h3b = bodies2.insert(rb)
  inspect(@dynamics.RigidBodyHandle::equals(h1a, h1b), content="true")
  inspect(@dynamics.RigidBodyHandle::equals(h2a, h2b), content="true")
  inspect(@dynamics.RigidBodyHandle::equals(h3a, h3b), content="true")
  colliders |> ignore
}

///|
test "rotated cuboid does not generate false contacts" {
  let colliders = @collision.ColliderSet::new()
  let impulse_joints = @dynamics.ImpulseJointSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()
  let pipeline = PhysicsPipeline::new()
  let broad_phase = @collision.BroadPhaseBvh::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let bodies = @dynamics.RigidBodySet::new()
  let islands = @dynamics.IslandManager::new()
  let cuboid_body = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .rotation(@core.Rot2::from_angle(@core.pi() * 0.5F))
    .build(),
  )
  let cuboid_collider = @collision.ColliderBuilder::cuboid(2.0F, 0.5F).build()
  colliders.insert_with_parent(cuboid_collider, cuboid_body, bodies) |> ignore
  let ball_body = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(1.2F, 0.0F))
    .build(),
  )
  let ball_collider = @collision.ColliderBuilder::ball(0.4F).build()
  colliders.insert_with_parent(ball_collider, ball_body, bodies) |> ignore
  pipeline.step(
    @core.Vec2::zero(),
    IntegrationParameters::default(),
    islands,
    broad_phase,
    narrow_phase,
    bodies,
    colliders,
    impulse_joints,
    multibody_joints,
    @dynamics.CCDSolver::new(),
    PhysicsHooks::new(),
    EventHandler::new(),
  )
  inspect(narrow_phase.contacts.length() == 0, content="true")
  if bodies.get(ball_body) is Some(body) {
    let tol = 1.0e-5F
    let pos = body.translation()
    inspect(@core.abs(pos.x - 1.2F) < tol, content="true")
    inspect(@core.abs(pos.y) < tol, content="true")
  } else {
    inspect(false, content="true")
  }
}
