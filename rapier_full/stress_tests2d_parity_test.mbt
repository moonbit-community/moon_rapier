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
fn no_nan_and_reasonable(
  bodies : @dynamics.RigidBodySet,
  handles : Array[@dynamics.RigidBodyHandle],
) -> Bool {
  for i in 0..<handles.length() {
    if bodies.get(handles[i]) is Some(body) {
      let p = body.translation()
      // This is a stress test: we only want to catch obvious explosions.
      if !(p.x == p.x) || !(p.y == p.y) {
        return false
      }
      if @core.abs(p.x) >= 1.0e6F || @core.abs(p.y) >= 1.0e6F {
        return false
      }
    } else {
      return false
    }
  }
  true
}

///|
test "examples2d/stress_tests/balls2.rs parity" {
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
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let num = 10
  let rad = 1.0F
  let shiftx = rad * 2.5F
  let shifty = rad * 2.0F
  let centerx = shiftx * Float::from_double(num.to_double()) / 2.0F
  let centery = shifty / 2.0F
  let handles : Array[@dynamics.RigidBodyHandle] = []
  for i in 0..<num {
    for j in 0..<(num * 3) {
      let x = Float::from_double(i.to_double()) * shiftx - centerx
      let y = Float::from_double(j.to_double()) * shifty + centery
      let rb = if j == 0 {
        @dynamics.RigidBodyBuilder::fixed()
      } else {
        @dynamics.RigidBodyBuilder::dynamic()
      }
      let h = bodies.insert(rb.translation(@core.Vec2::new(x, y)).build())
      colliders.insert_with_parent(
        @collision.ColliderBuilder::ball(rad).build(),
        h,
        bodies,
      )
      |> ignore
      handles.push(h)
    }
  }
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 120,
  )
  inspect(no_nan_and_reasonable(bodies, handles), content="true")
}

///|
test "examples2d/stress_tests/boxes2.rs parity" {
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
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )
  let gravity = @core.Vec2::new(0.0F, -9.81F)
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
    .rotation(@core.Rot2::from_angle(@core.pi() / 2.0F))
    .translation(@core.Vec2::new(ground_size, ground_size * 2.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(ground_size * 2.0F, 1.2F).build(),
    wall_r,
    bodies,
  )
  |> ignore
  let wall_l = bodies.insert(
    @dynamics.RigidBodyBuilder::fixed()
    .rotation(@core.Rot2::from_angle(@core.pi() / 2.0F))
    .translation(@core.Vec2::new(-ground_size, ground_size * 2.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(ground_size * 2.0F, 1.2F).build(),
    wall_l,
    bodies,
  )
  |> ignore
  let num = 8
  let rad = 0.5F
  let shift = rad * 2.0F
  let centerx = shift * Float::from_double(num.to_double()) / 2.0F
  let centery = shift / 2.0F
  let handles : Array[@dynamics.RigidBodyHandle] = []
  for i in 0..<num {
    for j in 0..<(num * 3) {
      let x = Float::from_double(i.to_double()) * shift - centerx
      let y = Float::from_double(j.to_double()) * shift + centery + 2.0F
      let h = bodies.insert(
        @dynamics.RigidBodyBuilder::dynamic()
        .translation(@core.Vec2::new(x, y))
        .build(),
      )
      colliders.insert_with_parent(
        @collision.ColliderBuilder::cuboid(rad, rad).build(),
        h,
        bodies,
      )
      |> ignore
      handles.push(h)
    }
  }
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 120,
  )
  inspect(no_nan_and_reasonable(bodies, handles), content="true")
  inspect(narrow_phase.contact_pairs().length() > 0, content="true")
}

///|
test "examples2d/stress_tests/capsules2.rs parity" {
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
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let ground = bodies.insert(@dynamics.RigidBodyBuilder::fixed().build())
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(30.0F, 1.2F).build(),
    ground,
    bodies,
  )
  |> ignore
  let handles : Array[@dynamics.RigidBodyHandle] = []
  let num = 8
  for i in 0..<num {
    for j in 0..<(num * 2) {
      let x = Float::from_double(i.to_double()) - 4.0F
      let y = Float::from_double(j.to_double()) * 1.2F + 3.0F
      let h = bodies.insert(
        @dynamics.RigidBodyBuilder::dynamic()
        .translation(@core.Vec2::new(x, y))
        .build(),
      )
      colliders.insert_with_parent(
        @collision.ColliderBuilder::capsule_y(0.4F, 0.3F).build(),
        h,
        bodies,
      )
      |> ignore
      handles.push(h)
    }
  }
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 120,
  )
  inspect(no_nan_and_reasonable(bodies, handles), content="true")
}

///|
test "examples2d/stress_tests/joint_ball2.rs parity" {
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
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let rad = 0.4F
  let numi = 8
  let numk = 8
  let shift = 1.0F
  let handles : Array[@dynamics.RigidBodyHandle] = []
  for k in 0..<numk {
    for i in 0..<numi {
      let status = if k >= numk / 2 - 1 && k <= numk / 2 + 1 && i == 0 {
        @dynamics.RigidBodyBuilder::fixed()
      } else {
        @dynamics.RigidBodyBuilder::dynamic()
      }
      let child = bodies.insert(
        status
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
        child,
        bodies,
      )
      |> ignore
      if i > 0 {
        let parent = handles[handles.length() - 1]
        let joint = @dynamics.GenericJoint::from_revolute(
          @dynamics.RevoluteJointBuilder::new()
          .local_anchor2(@core.Vec2::new(0.0F, shift))
          .build(),
        )
        impulse_joints.insert(parent, child, joint, true) |> ignore
      }
      if k > 0 {
        let parent_index = handles.length() - numi
        let parent = handles[parent_index]
        let joint = @dynamics.GenericJoint::from_revolute(
          @dynamics.RevoluteJointBuilder::new()
          .local_anchor2(@core.Vec2::new(-shift, 0.0F))
          .build(),
        )
        impulse_joints.insert(parent, child, joint, true) |> ignore
      }
      handles.push(child)
    }
  }
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 60,
  )
  inspect(no_nan_and_reasonable(bodies, handles), content="true")
  inspect(impulse_joints.len() > 0, content="true")
}

///|
test "examples2d/stress_tests/joint_fixed2.rs parity" {
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
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let rad = 0.4F
  let num = 6
  let shift = 1.0F
  let handles : Array[@dynamics.RigidBodyHandle] = []
  for k in 0..<num {
    for i in 0..<num {
      let status = if k == 0 {
        @dynamics.RigidBodyBuilder::fixed()
      } else {
        @dynamics.RigidBodyBuilder::dynamic()
      }
      let child = bodies.insert(
        status
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
        child,
        bodies,
      )
      |> ignore
      if i > 0 {
        let parent = handles[handles.length() - 1]
        let joint = @dynamics.GenericJoint::from_fixed(
          @dynamics.FixedJointBuilder::new()
          .local_anchor2(@core.Vec2::new(0.0F, shift))
          .build(),
        )
        impulse_joints.insert(parent, child, joint, true) |> ignore
      }
      if k > 0 {
        let parent_index = handles.length() - num
        let parent = handles[parent_index]
        let joint = @dynamics.GenericJoint::from_fixed(
          @dynamics.FixedJointBuilder::new()
          .local_anchor2(@core.Vec2::new(-shift, 0.0F))
          .build(),
        )
        impulse_joints.insert(parent, child, joint, true) |> ignore
      }
      handles.push(child)
    }
  }
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 60,
  )
  inspect(no_nan_and_reasonable(bodies, handles), content="true")
  inspect(impulse_joints.len() > 0, content="true")
}

///|
test "examples2d/stress_tests/joint_prismatic2.rs parity" {
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
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let rad = 0.4F
  let num = 4
  let shift = 1.0F
  let handles : Array[@dynamics.RigidBodyHandle] = []
  for l in 0..<3 {
    let base_y = Float::from_double(l.to_double()) *
      shift *
      Float::from_double((num + 2).to_double()) *
      2.0F
    for j in 0..<6 {
      let base_x = Float::from_double(j.to_double()) * shift * 4.0F
      let ground = bodies.insert(
        @dynamics.RigidBodyBuilder::fixed()
        .translation(@core.Vec2::new(base_x, base_y))
        .build(),
      )
      colliders.insert_with_parent(
        @collision.ColliderBuilder::cuboid(rad, rad).build(),
        ground,
        bodies,
      )
      |> ignore
      handles.push(ground)
      let mut parent = ground
      for i in 0..<num {
        let y = base_y - Float::from_double((i + 1).to_double()) * shift
        let child = bodies.insert(
          @dynamics.RigidBodyBuilder::dynamic()
          .translation(@core.Vec2::new(base_x, y))
          .build(),
        )
        colliders.insert_with_parent(
          @collision.ColliderBuilder::cuboid(rad, rad).density(1.0F).build(),
          child,
          bodies,
        )
        |> ignore
        handles.push(child)
        let axis = if i % 2 == 0 {
          @core.Vec2::new(1.0F, 1.0F).normalize()
        } else {
          @core.Vec2::new(-1.0F, 1.0F).normalize()
        }
        let joint = @dynamics.GenericJoint::from_prismatic(
          @dynamics.PrismaticJointBuilder::new(axis)
          .local_anchor2(@core.Vec2::new(0.0F, shift))
          .limits(-1.5F, 1.5F)
          .build(),
        )
        impulse_joints.insert(parent, child, joint, true) |> ignore
        parent = child
      }
    }
  }
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 60,
  )
  inspect(no_nan_and_reasonable(bodies, handles), content="true")
  inspect(impulse_joints.len() > 0, content="true")
}

///|
test "examples2d/stress_tests/convex_polygons2.rs parity" {
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
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let ground_size = 30.0F
  let ground = bodies.insert(@dynamics.RigidBodyBuilder::fixed().build())
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(ground_size, 1.2F).build(),
    ground,
    bodies,
  )
  |> ignore
  let num = 8
  let scale = 2.0F
  let shift = scale
  let centerx = shift * Float::from_double(num.to_double()) / 2.0F
  let centery = shift / 2.0F
  let mut seed = 0
  let handles : Array[@dynamics.RigidBodyHandle] = []
  for i in 0..<num {
    for j in 0..<(num * 3) {
      let x = Float::from_double(i.to_double()) * shift - centerx
      let y = Float::from_double(j.to_double()) * shift * 2.0F + centery + 2.0F
      let h = bodies.insert(
        @dynamics.RigidBodyBuilder::dynamic()
        .translation(@core.Vec2::new(x, y))
        .build(),
      )
      let points : Array[@core.Vec2] = []
      for _ in 0..<10 {
        let (rx, s1) = rng_float01(seed)
        let (ry, s2) = rng_float01(s1)
        seed = s2
        points.push(@core.Vec2::new(rx * scale, ry * scale))
      }
      if @collision.ColliderBuilder::convex_hull(points) is Some(builder) {
        colliders.insert_with_parent(builder.build(), h, bodies) |> ignore
      } else {
        inspect(false, content="true")
      }
      handles.push(h)
    }
  }
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 120,
  )
  inspect(no_nan_and_reasonable(bodies, handles), content="true")
  inspect(narrow_phase.contact_pairs().length() > 0, content="true")
}

///|
test "examples2d/stress_tests/heightfield2.rs parity" {
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
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let ground_size = @core.Vec2::new(50.0F, 1.0F)
  let nsubdivs = 200
  let heights : Array[@core.Real] = []
  let denom = Float::from_double(nsubdivs.to_double())
  for i in 0..<(nsubdivs + 1) {
    if i == 0 || i == nsubdivs {
      heights.push(80.0F)
    } else {
      let t = Float::from_double(i.to_double()) * ground_size.x / denom
      heights.push(local_cos(t) * 2.0F)
    }
  }
  let ground = bodies.insert(@dynamics.RigidBodyBuilder::fixed().build())
  colliders.insert_with_parent(
    @collision.ColliderBuilder::heightfield(heights, ground_size).build(),
    ground,
    bodies,
  )
  |> ignore
  let num = 8
  let rad = 0.5F
  let shift = rad * 2.0F
  let centerx = shift * Float::from_double((num / 2).to_double())
  let centery = shift / 2.0F
  let handles : Array[@dynamics.RigidBodyHandle] = []
  for i in 0..<num {
    for j in 0..<(num * 3) {
      let x = Float::from_double(i.to_double()) * shift - centerx
      let y = Float::from_double(j.to_double()) * shift + centery + 3.0F
      let h = bodies.insert(
        @dynamics.RigidBodyBuilder::dynamic()
        .translation(@core.Vec2::new(x, y))
        .build(),
      )
      if j % 2 == 0 {
        colliders.insert_with_parent(
          @collision.ColliderBuilder::cuboid(rad, rad).build(),
          h,
          bodies,
        )
        |> ignore
      } else {
        colliders.insert_with_parent(
          @collision.ColliderBuilder::ball(rad).build(),
          h,
          bodies,
        )
        |> ignore
      }
      handles.push(h)
    }
  }
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 120,
  )
  inspect(no_nan_and_reasonable(bodies, handles), content="true")
  inspect(narrow_phase.contact_pairs().length() > 0, content="true")
}

///|
test "examples2d/stress_tests/pyramid2.rs parity" {
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
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let ground_size = 100.0F
  let ground_thickness = 1.0F
  let ground = bodies.insert(@dynamics.RigidBodyBuilder::fixed().build())
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(ground_size, ground_thickness).build(),
    ground,
    bodies,
  )
  |> ignore
  let num = 18
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
      let h = bodies.insert(
        @dynamics.RigidBodyBuilder::dynamic()
        .translation(@core.Vec2::new(x, y))
        .build(),
      )
      colliders.insert_with_parent(
        @collision.ColliderBuilder::cuboid(rad, rad).build(),
        h,
        bodies,
      )
      |> ignore
      handles.push(h)
    }
  }
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 180,
  )
  inspect(no_nan_and_reasonable(bodies, handles), content="true")
  inspect(narrow_phase.contact_pairs().length() > 0, content="true")
}

///|
test "examples2d/stress_tests/vertical_stacks2.rs parity" {
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
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )
  let gravity = @core.Vec2::new(0.0F, -9.81F)
  let num = 16
  let rad = 0.5F
  let ground_size = Float::from_double(num.to_double()) * rad * 10.0F
  let ground_thickness = 1.0F
  let ground = bodies.insert(@dynamics.RigidBodyBuilder::fixed().build())
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(ground_size, ground_thickness).build(),
    ground,
    bodies,
  )
  |> ignore
  let shiftx_centerx = [
    (
      rad * 2.0F + 0.0002F,
      -Float::from_double(num.to_double()) * rad * 2.0F * 1.5F,
    ),
    (rad * 2.0F + rad, Float::from_double(num.to_double()) * rad * 2.0F * 1.5F),
  ]
  let handles : Array[@dynamics.RigidBodyHandle] = []
  for t in 0..<shiftx_centerx.length() {
    let (shiftx, centerx) = shiftx_centerx[t]
    let shifty = rad * 2.0F
    let centery = shifty / 2.0F + ground_thickness
    for i in 0..<num {
      for j in 0..<(1 + i * 2) {
        let fj = Float::from_double(j.to_double())
        let fi = Float::from_double(i.to_double())
        let x = (fj - fi) * shiftx + centerx
        let y = (Float::from_double(num.to_double()) - fi - 1.0F) * shifty +
          centery
        let h = bodies.insert(
          @dynamics.RigidBodyBuilder::dynamic()
          .translation(@core.Vec2::new(x, y))
          .build(),
        )
        colliders.insert_with_parent(
          @collision.ColliderBuilder::cuboid(rad, rad).build(),
          h,
          bodies,
        )
        |> ignore
        handles.push(h)
      }
    }
  }
  step_pipeline_2d_n(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    impulse_joints, multibody_joints, ccd_solver, hooks, events, 180,
  )
  inspect(no_nan_and_reasonable(bodies, handles), content="true")
  inspect(narrow_phase.contact_pairs().length() > 0, content="true")
}
