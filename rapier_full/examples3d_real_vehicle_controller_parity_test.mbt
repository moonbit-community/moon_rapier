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
/// 3DReal parity tests for `rapier-reference/examples3d/vehicle_controller3.rs`.

///|
test "examples3d/vehicle_controller3.rs parity (3DReal headless)" {
  // Ported from `rapier-reference/examples3d/vehicle_controller3.rs` (headless).
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
  let ground_size = 5.0F
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

  // Vehicle chassis.
  let hw = 0.3F
  let hh = 0.15F
  let vehicle_body = bodies.insert(
    @dynamics.RigidBodyBuilder3D::dynamic()
    .translation(@core.Vec3::new(0.0F, 1.0F, 0.0F))
    .can_sleep(false)
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(hw * 2.0F, hh, hw)
    .density(100.0F)
    .build(),
    vehicle_body,
    bodies,
  )
  |> ignore
  let tuning = @control.WheelTuning::default()
    .with_suspension_stiffness(100.0F)
    .with_suspension_damping(10.0F)
  let vehicle = @control.DynamicRayCastVehicleController3DReal::new(
    vehicle_body,
  )
  let wheel_positions = [
    @core.Vec3::new(hw * 1.5F, -hh, hw),
    @core.Vec3::new(hw * 1.5F, -hh, -hw),
    @core.Vec3::new(-hw * 1.5F, -hh, hw),
    @core.Vec3::new(-hw * 1.5F, -hh, -hw),
  ]
  for pos in wheel_positions {
    vehicle.add_wheel(
      pos,
      @core.Vec3::new(0.0F, -1.0F, 0.0F),
      @core.Vec3::new(0.0F, 0.0F, 1.0F),
      hh,
      hh / 4.0F,
      tuning,
    )
    |> ignore
  }
  for i in 0..<vehicle.wheels().length() {
    vehicle.set_engine_force(i, 20.0F)
  }

  // Dynamic cubes to push around.
  let num = 8
  let rad = 0.1F
  let shift = rad * 2.0F
  let centerx = shift * Float::from_int(num / 2)
  let centery = rad
  for j in 0..<1 {
    for k in 0..<4 {
      for i in 0..<num {
        let x = Float::from_int(i) * shift - centerx
        let y = Float::from_int(j) * shift + centery
        let z = Float::from_int(k) * shift + centerx
        let h = bodies.insert(
          @dynamics.RigidBodyBuilder3D::dynamic()
          .translation(@core.Vec3::new(x, y, z))
          .build(),
        )
        colliders.insert_with_parent(
          @collision.ColliderBuilder3D::cuboid(rad, rad, rad).build(),
          h,
          bodies,
        )
        |> ignore
      }
    }
  }

  // A slope we can climb.
  let slope_angle = 0.2F
  let slope_size = 2.0F
  colliders.insert(
    @collision.ColliderBuilder3D::cuboid(slope_size, ground_height, ground_size)
    .translation(
      @core.Vec3::new(ground_size + slope_size, -ground_height + 0.4F, 0.0F),
    )
    .rotation(
      @core.rotation_from_scaled_axis(@core.Vec3::new(0.0F, 0.0F, slope_angle)),
    )
    .build(),
  )
  |> ignore

  // A slope we can't climb.
  let impossible_slope_angle = 0.9F
  let impossible_slope_size = 2.0F
  colliders.insert(
    @collision.ColliderBuilder3D::cuboid(slope_size, ground_height, ground_size)
    .translation(
      @core.Vec3::new(
        ground_size + slope_size * 2.0F + impossible_slope_size - 0.9F,
        -ground_height + 2.3F,
        0.0F,
      ),
    )
    .rotation(
      @core.rotation_from_scaled_axis(
        @core.Vec3::new(0.0F, 0.0F, impossible_slope_angle),
      ),
    )
    .build(),
  )
  |> ignore

  // Heightfield ground.
  let nsubdivs = 20
  let ground_scale = @core.Vec3::new(10.0F, 0.4F, 10.0F)
  let heights : Array[@core.Real] = []
  for i in 0..<(nsubdivs + 1) {
    for j in 0..<(nsubdivs + 1) {
      let x = Float::from_int(i) *
        ground_scale.x /
        Float::from_int(nsubdivs) /
        2.0F
      let z = Float::from_int(j) *
        ground_scale.z /
        Float::from_int(nsubdivs) /
        2.0F
      heights.push(
        -Float::from_double(@math.cos(x.to_double())) -
        Float::from_double(@math.cos(z.to_double())),
      )
    }
  }
  let hf = @collision.ColliderBuilder3D::heightfield(
      heights,
      nsubdivs + 1,
      nsubdivs + 1,
      ground_scale,
    )
    .unwrap()
    .translation(@core.Vec3::new(-7.0F, 0.0F, 0.0F))
    .build()
  colliders.insert(hf) |> ignore

  // Query pipeline used by the controller.
  let queries = @collision.QueryPipelineMut3DReal::new(
    @collision.QueryFilter3DReal::new(),
    bodies,
    colliders,
  )
  for _ in 0..<240 {
    vehicle.update_vehicle(dt, queries)
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    )
  }
  if bodies.get(vehicle_body) is Some(rb) {
    let p = rb.translation()
    inspect(p.x == p.x && p.y == p.y && p.z == p.z, content="true")
    // Vehicle moved forward.
    inspect(p.x > 0.1F, content="true")
    // Vehicle didn't explode.
    inspect(p.y > -50.0F, content="true")
  } else {
    inspect(false, content="true")
  }
}
