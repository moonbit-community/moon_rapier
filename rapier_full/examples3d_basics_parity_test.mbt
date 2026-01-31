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
/// These are progressively migrating from the dim3 compatibility wrappers (2D projection) to the
/// real dim3 backend.
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
  // Adapted from `rapier-reference/examples3d/primitives3.rs` (headless).
  //
  // We run this against the real dim3 pipeline, but with a reduced number of bodies because the
  // current BroadPhase3D is O(n^2).
  let pipeline = @pipeline.PhysicsPipeline3DReal::new()
  let broad_phase = @collision.BroadPhase3D::new()
  let narrow_phase = @collision.NarrowPhase3D::new()
  let bodies = @dynamics.RigidBodySet3D::new()
  let colliders = @collision.ColliderSet3D::new()
  let islands = @dynamics.IslandManager3D::new()
  let gravity = @core.Vec3::new(0.0F, -9.81F, 0.0F)
  let parameters = @dynamics.IntegrationParameters::default()
  let handles : Array[@dynamics.RigidBodyHandle] = []
  let ground_height = 2.1F
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder3D::fixed()
    .translation(@core.Vec3::new(0.0F, -ground_height, 0.0F))
    .build(),
  )
  handles.push(ground)
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(20.0F, ground_height, 20.0F).build(),
    ground,
    bodies,
  )
  |> ignore
  let num = 4
  let rad = 1.0F
  let shift = rad * 2.0F + rad
  let center = shift * Float::from_int(num / 2)
  for j in 0..<3 {
    for i in 0..<num {
      for k in 0..<num {
        let x = Float::from_int(i) * shift - center
        let y = Float::from_int(j) * shift + 5.0F
        let z = Float::from_int(k) * shift - center
        let handle = bodies.insert(
          @dynamics.RigidBodyBuilder3D::dynamic()
          .translation(@core.Vec3::new(x, y, z))
          .build(),
        )
        handles.push(handle)
        let collider = match j % 5 {
          1 => @collision.ColliderBuilder3D::ball(rad)
          2 =>
            @collision.ColliderBuilder3D::round_cylinder(rad, rad, rad / 10.0F)
          3 => @collision.ColliderBuilder3D::cone(rad, rad)
          _ => @collision.ColliderBuilder3D::capsule_y(rad, rad)
        }
        colliders.insert_with_parent(collider.build(), handle, bodies) |> ignore
      }
    }
  }
  for _ in 0..<2 {
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    )
  }

  // Basic sanity: no NaNs, and z is preserved (not projected to 2D).
  let mut saw_nonzero_z = false
  for i in 0..<handles.length() {
    if bodies.get(handles[i]) is Some(rb) {
      let p = rb.translation()
      if p.z != 0.0F {
        saw_nonzero_z = true
      }
      if !(p.x == p.x) || !(p.y == p.y) || !(p.z == p.z) {
        return false
      }
    } else {
      return false
    }
  }
  saw_nonzero_z
}

///|
fn run_examples3d_damping3() -> Bool {
  // Adapted from `rapier-reference/examples3d/damping3.rs` (headless).
  let pipeline = @pipeline.PhysicsPipeline3DReal::new()
  let broad_phase = @collision.BroadPhase3D::new()
  let narrow_phase = @collision.NarrowPhase3D::new()
  let bodies = @dynamics.RigidBodySet3D::new()
  let colliders = @collision.ColliderSet3D::new()
  let islands = @dynamics.IslandManager3D::new()
  let gravity = @core.Vec3::zero()
  let parameters = @dynamics.IntegrationParameters::default()
  let handles : Array[@dynamics.RigidBodyHandle] = []
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
    let z = Float::from_int(i) * 0.1F
    let rb = @dynamics.RigidBodyBuilder3D::dynamic()
      .translation(@core.Vec3::new(x, y, z))
      .linvel(@core.Vec3::new(x * 10.0F, y * 10.0F, 0.0F))
      .angvel(@core.Vec3::new(0.0F, 0.0F, 100.0F))
      .linear_damping(Float::from_int(i + 1) * subdiv * 10.0F)
      .angular_damping(Float::from_int(num - i) * subdiv * 10.0F)
      .build()
    let handle = bodies.insert(rb)
    handles.push(handle)
    colliders.insert_with_parent(
      @collision.ColliderBuilder3D::cuboid(rad, rad, rad).build(),
      handle,
      bodies,
    )
    |> ignore
  }
  for _ in 0..<20 {
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    )
  }
  let mut saw_rotation = false
  for i in 0..<handles.length() {
    if bodies.get(handles[i]) is Some(rb) {
      let p = rb.translation()
      if !(p.x == p.x) || !(p.y == p.y) || !(p.z == p.z) {
        return false
      }
      let axis = rb.rotation().to_scaled_axis()
      if axis.length_squared() > 1.0e-6F {
        saw_rotation = true
      }
    } else {
      return false
    }
  }
  saw_rotation
}

///|
fn run_examples3d_restitution3() -> Bool {
  // Adapted from `rapier-reference/examples3d/restitution3.rs` (headless).
  let pipeline = @pipeline.PhysicsPipeline3DReal::new()
  let broad_phase = @collision.BroadPhase3D::new()
  let narrow_phase = @collision.NarrowPhase3D::new()
  let bodies = @dynamics.RigidBodySet3D::new()
  let colliders = @collision.ColliderSet3D::new()
  let islands = @dynamics.IslandManager3D::new()
  let gravity = @core.Vec3::new(0.0F, -9.81F, 0.0F)
  let parameters = @dynamics.IntegrationParameters::default()
  let handles : Array[@dynamics.RigidBodyHandle] = []
  let ground_height = 1.0F
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder3D::fixed()
    .translation(@core.Vec3::new(0.0F, -ground_height, 0.0F))
    .build(),
  )
  handles.push(ground)
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(20.0F, ground_height, 2.0F).build(),
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
      let z = Float::from_int(i) * 0.2F
      let rb = @dynamics.RigidBodyBuilder3D::dynamic()
        .translation(@core.Vec3::new(x * 2.0F, y, z))
        .build()
      let handle = bodies.insert(rb)
      handles.push(handle)
      colliders.insert_with_parent(
        @collision.ColliderBuilder3D::ball(rad).build(),
        handle,
        bodies,
      )
      |> ignore
    }
  }
  for _ in 0..<10 {
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    )
  }
  let mut saw_nonzero_z = false
  for i in 0..<handles.length() {
    if bodies.get(handles[i]) is Some(rb) {
      let p = rb.translation()
      if p.z != 0.0F {
        saw_nonzero_z = true
      }
      if !(p.x == p.x) || !(p.y == p.y) || !(p.z == p.z) {
        return false
      }
    } else {
      return false
    }
  }
  saw_nonzero_z
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
