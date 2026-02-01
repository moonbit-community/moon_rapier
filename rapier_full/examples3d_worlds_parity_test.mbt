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
/// Parity tests for additional `rapier-reference/examples3d/*.rs` scenes.

///|
fn step_world3_real(
  pipeline : @pipeline.PhysicsPipeline3DReal,
  gravity : @core.Vec3,
  parameters : @dynamics.IntegrationParameters,
  islands : @dynamics.IslandManager3D,
  broad_phase : @collision.BroadPhase3D,
  narrow_phase : @collision.NarrowPhase3D,
  bodies : @dynamics.RigidBodySet3D,
  colliders : @collision.ColliderSet3D,
  steps : Int,
) -> Unit {
  for _ in 0..<steps {
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    )
  }
}

///|
fn run_examples3d_platform3() -> Bool {
  // Ported from `rapier-reference/examples3d/platform3.rs` (headless; reduced assertions).
  let pipeline = @pipeline.PhysicsPipeline3DReal::new()
  let broad_phase = @collision.BroadPhase3D::new()
  let narrow_phase = @collision.NarrowPhase3D::new()
  let bodies = @dynamics.RigidBodySet3D::new()
  let colliders = @collision.ColliderSet3D::new()
  let islands = @dynamics.IslandManager3D::new()
  let gravity = @core.Vec3::new(0.0F, -9.81F, 0.0F)
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )
  let inv_dt = parameters.inv_dt()
  let dt = if inv_dt == 0.0F { 0.0F } else { 1.0F / inv_dt }

  // Ground.
  let ground_size = 10.0F
  let ground_height = 0.1F
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder3D::fixed()
    .translation(@core.Vec3::new(0.0F, -ground_height, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(
      ground_size, ground_height, ground_size,
    ).build(),
    ground,
    bodies,
  )
  |> ignore

  // Boxes.
  let num = 6
  let rad = 0.2F
  let shift = rad * 2.0F
  let centerx = shift * Float::from_int(num) / 2.0F
  let centerz = shift * Float::from_int(num) / 2.0F
  let dynamic_handles : Array[@dynamics.RigidBodyHandle] = []
  for i in 0..<num {
    for j in 0..<num {
      for k in 0..<num {
        let centery = if j >= num / 2 { 5.0F } else { 3.0F }
        let x = Float::from_int(i) * shift - centerx
        let y = Float::from_int(j) * shift + centery
        let z = Float::from_int(k) * shift - centerz
        let handle = bodies.insert(
          @dynamics.RigidBodyBuilder3D::dynamic()
          .translation(@core.Vec3::new(x, y, z))
          .build(),
        )
        dynamic_handles.push(handle)
        colliders.insert_with_parent(
          @collision.ColliderBuilder3D::cuboid(rad, rad, rad).build(),
          handle,
          bodies,
        )
        |> ignore
      }
    }
  }

  // Kinematic platforms.
  let v_platform = bodies.insert(
    @dynamics.RigidBodyBuilder3D::kinematic_velocity_based()
    .translation(@core.Vec3::new(0.0F, 1.5F + 0.8F, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(rad * 10.0F, rad, rad * 10.0F).build(),
    v_platform,
    bodies,
  )
  |> ignore
  let p_platform = bodies.insert(
    @dynamics.RigidBodyBuilder3D::kinematic_position_based()
    .translation(@core.Vec3::new(0.0F, 3.0F + 1.5F + 0.8F, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(rad * 10.0F, rad, rad * 10.0F).build(),
    p_platform,
    bodies,
  )
  |> ignore

  // Drive platforms for a while.
  let mut time = 0.0F
  for _ in 0..<240 {
    time = time + dt
    let vel = @core.Vec3::new(
      0.0F,
      Float::from_double(@math.cos((time * 2.0F).to_double())),
      Float::from_double(@math.sin(time.to_double())) * 2.0F,
    )
    if bodies.get_mut(v_platform) is Some(rb) {
      rb.set_linvel(vel)
      rb.set_angvel(@core.Vec3::new(0.0F, 1.0F, 0.0F))
    } else {
      return false
    }
    if bodies.get_mut(p_platform) is Some(rb) {
      let next_tra = rb.translation().add(vel.scale(-dt))
      let delta = @core.rotation_from_scaled_axis(
        @core.Vec3::new(0.0F, 1.0F, 0.0F).scale(-0.5F * dt),
      )
      rb.set_next_kinematic_translation(next_tra)
      rb.set_next_kinematic_rotation(delta.mul(rb.rotation()))
    } else {
      return false
    }
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    )
  }

  // At least some dynamic bodies should have been moved in z by the platform motion.
  let mut moved = false
  for h in dynamic_handles {
    if bodies.get(h) is Some(rb) {
      let p = rb.translation()
      if !(p.x == p.x) || !(p.y == p.y) || !(p.z == p.z) {
        return false
      }
      if @core.abs(p.z) > 0.05F {
        moved = true
      }
    } else {
      return false
    }
  }
  moved
}

///|
fn run_examples3d_domino3() -> Bool {
  // Ported from `rapier-reference/examples3d/domino3.rs` (headless; reduced size).
  let pipeline = @pipeline.PhysicsPipeline3DReal::new()
  let broad_phase = @collision.BroadPhase3D::new()
  let narrow_phase = @collision.NarrowPhase3D::new()
  let bodies = @dynamics.RigidBodySet3D::new()
  let colliders = @collision.ColliderSet3D::new()
  let islands = @dynamics.IslandManager3D::new()
  let gravity = @core.Vec3::new(0.0F, -9.81F, 0.0F)
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )

  // Ground.
  let ground_size = 50.0F
  let ground_height = 0.1F
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder3D::fixed()
    .translation(@core.Vec3::new(0.0F, -ground_height, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(
      ground_size, ground_height, ground_size,
    ).build(),
    ground,
    bodies,
  )
  |> ignore

  // Reduced domino count (upstream uses 4000).
  let num = 400
  let width = 1.0F
  let thickness = 0.1F
  let mut curr_angle = 0.0F
  let mut curr_rad = 10.0F
  let mut skip = 0
  let handles : Array[@dynamics.RigidBodyHandle] = []
  for i in 0..<num {
    let perimeter = 2.0F * @core.pi() * curr_rad
    let spacing = thickness * 4.0F
    let prev_angle = curr_angle
    curr_angle = curr_angle + 2.0F * @core.pi() * spacing / perimeter
    let x = Float::from_double(@math.sin(curr_angle.to_double()))
    let z = Float::from_double(@math.cos(curr_angle.to_double()))
    let two_pi = @core.two_pi()
    let a0 = curr_angle -
      Float::from_int((curr_angle / two_pi).to_int()) * two_pi
    let a1 = prev_angle -
      Float::from_int((prev_angle / two_pi).to_int()) * two_pi
    let nudged = a0 < a1
    let tilt = if nudged || i == num - 1 { 0.2F } else { 0.0F }
    if skip == 0 {
      let rot = @core.rotation_from_scaled_axis(
        @core.Vec3::new(0.0F, 1.0F, 0.0F).scale(curr_angle),
      )
      let tilt_axis = rot.rotate_vec3(@core.Vec3::new(0.0F, 0.0F, 1.0F))
      let tilt_rot = @core.rotation_from_scaled_axis(tilt_axis.scale(tilt))
      let position = @core.Isometry3::new(
        @core.Vec3::new(
          x * curr_rad,
          width * 2.0F + ground_height,
          z * curr_rad,
        ),
        tilt_rot.mul(rot),
      )
      let handle = bodies.insert(
        @dynamics.RigidBodyBuilder3D::dynamic()
        .translation(position.translation)
        .rotation(position.rotation)
        .build(),
      )
      handles.push(handle)
      colliders.insert_with_parent(
        @collision.ColliderBuilder3D::cuboid(thickness, width * 2.0F, width).build(),
        handle,
        bodies,
      )
      |> ignore
    } else {
      skip = skip - 1
    }
    if nudged {
      skip = 5
    }
    curr_rad = curr_rad + 1.5F / perimeter
  }
  step_world3_real(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    360,
  )

  // The chain should have started falling: at least one domino reaches near the ground.
  let mut near_ground = false
  for h in handles {
    if bodies.get(h) is Some(rb) {
      let p = rb.translation()
      if !(p.x == p.x) || !(p.y == p.y) || !(p.z == p.z) {
        return false
      }
      if p.y < 1.0F {
        near_ground = true
      }
    } else {
      return false
    }
  }
  near_ground
}

///|
fn run_examples3d_fountain3() -> Bool {
  // Ported from `rapier-reference/examples3d/fountain3.rs` (headless).
  //
  // Upstream spawns round cylinders + cones + cuboids and removes bodies when the count exceeds a
  // threshold. Our minimal dim3 backend doesn't yet support all shape pairs, so we focus on the
  // add/remove behavior and spawn ball/cuboid alternately (both collide with the ground).
  let pipeline = @pipeline.PhysicsPipeline3DReal::new()
  let broad_phase = @collision.BroadPhase3D::new()
  let narrow_phase = @collision.NarrowPhase3D::new()
  let bodies = @dynamics.RigidBodySet3D::new()
  let colliders = @collision.ColliderSet3D::new()
  let islands = @dynamics.IslandManager3D::new()
  let gravity = @core.Vec3::new(0.0F, -9.81F, 0.0F)
  let parameters = @dynamics.IntegrationParameters::default().set_num_solver_iterations(
    16,
  )

  // Ground (3 stacked layers).
  let ground_size = 40.0F
  let ground_height = 2.1F
  for k in 0..<3 {
    let handle = bodies.insert(
      @dynamics.RigidBodyBuilder3D::fixed()
      .translation(
        @core.Vec3::new(0.0F, -ground_height - Float::from_int(k), 0.0F),
      )
      .build(),
    )
    colliders.insert_with_parent(
      @collision.ColliderBuilder3D::cuboid(
        ground_size, ground_height, ground_size,
      ).build(),
      handle,
      bodies,
    )
    |> ignore
  }
  let max_bodies = 30
  fn dynamic_count(bodies : @dynamics.RigidBodySet3D) -> Int {
    let mut count = 0
    for h in bodies.all_handles() {
      if bodies.get(h) is Some(rb) && rb.body_type().is_dynamic() {
        count = count + 1
      }
    }
    count
  }

  let rad = 0.5F
  let spawned : Array[@dynamics.RigidBodyHandle] = []
  for step in 0..<120 {
    let angle = Float::from_int(step) * 0.3F
    let spawn_x = Float::from_double(@math.sin(angle.to_double())) * 2.0F
    let spawn_z = Float::from_double(@math.cos(angle.to_double())) * 2.0F
    let h = bodies.insert(
      @dynamics.RigidBodyBuilder3D::dynamic()
      .translation(@core.Vec3::new(spawn_x, 10.0F, spawn_z))
      .build(),
    )
    spawned.push(h)
    let co = if step % 2 == 0 {
      @collision.ColliderBuilder3D::ball(rad).build()
    } else {
      @collision.ColliderBuilder3D::cuboid(rad, rad, rad).build()
    }
    colliders.insert_with_parent(co, h, bodies) |> ignore

    // Trim.
    if dynamic_count(bodies) > max_bodies {
      // Pick some dynamic bodies far from the origin (x/z), like upstream.
      let candidates : Array[(@dynamics.RigidBodyHandle, Float)] = []
      for bh in bodies.all_handles() {
        if bodies.get(bh) is Some(rb) {
          if rb.body_type().is_dynamic() {
            let p = rb.translation()
            candidates.push((bh, @core.abs(p.x) + @core.abs(p.z)))
          }
        }
      }
      // Sort descending by score.
      for i in 1..<candidates.length() {
        let mut j = i
        while j > 0 && candidates[j].1 > candidates[j - 1].1 {
          let tmp = candidates[j - 1]
          candidates[j - 1] = candidates[j]
          candidates[j] = tmp
          j = j - 1
        }
      }
      let to_remove = candidates.length() - max_bodies
      if to_remove > 0 {
        for i in 0..<to_remove {
          let bh = candidates[i].0
          colliders.remove_attached_to(bh) |> ignore
          bodies.remove(bh) |> ignore
        }
      }
    }
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    )
  }

  // Body count should remain bounded and all remaining dynamic bodies should be finite.
  let dyn_final = dynamic_count(bodies)
  if dyn_final > max_bodies {
    return false
  }
  let mut min_y = 1.0e30F
  for bh in bodies.all_handles() {
    if bodies.get(bh) is Some(rb) {
      if rb.body_type().is_dynamic() {
        let p = rb.translation()
        if !(p.x == p.x) || !(p.y == p.y) || !(p.z == p.z) {
          return false
        }
        if p.y < min_y {
          min_y = p.y
        }
        if p.y < -100.0F {
          return false
        }
      }
    }
  }
  true
}

///|
test "examples3d/platform3.rs parity (headless)" {
  inspect(run_examples3d_platform3(), content="true")
}

///|
test "examples3d/domino3.rs parity (headless)" {
  inspect(run_examples3d_domino3(), content="true")
}

///|
test "examples3d/fountain3.rs parity (headless)" {
  inspect(run_examples3d_fountain3(), content="true")
}
