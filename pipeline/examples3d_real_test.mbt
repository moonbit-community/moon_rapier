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
fn step_world3_real(
  pipeline : PhysicsPipeline3DReal,
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
test "examples3d primitives3: real backend smoke" {
  let pipeline = PhysicsPipeline3DReal::new()
  let broad_phase = @collision.BroadPhase3D::new()
  let narrow_phase = @collision.NarrowPhase3D::new()
  let islands = @dynamics.IslandManager3D::new()
  let bodies = @dynamics.RigidBodySet3D::new()
  let colliders = @collision.ColliderSet3D::new()
  let gravity = @core.Vec3::new(0.0F, -9.81F, 0.0F)
  let parameters = @dynamics.IntegrationParameters::default()
  let handles : Array[@dynamics.RigidBodyHandle] = []
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder3D::fixed()
    .translation(@core.Vec3::new(0.0F, -2.1F, 0.0F))
    .build(),
  )
  handles.push(ground)
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(20.0F, 2.1F, 20.0F).build(),
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
  step_world3_real(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    2,
  )

  // Basic sanity: no NaNs, and z is preserved (not flattened to 2D).
  let mut saw_nonzero_z = false
  for i in 0..<handles.length() {
    if bodies.get(handles[i]) is Some(rb) {
      let p = rb.translation()
      inspect(p.x == p.x, content="true")
      inspect(p.y == p.y, content="true")
      inspect(p.z == p.z, content="true")
      if p.z != 0.0F {
        saw_nonzero_z = true
      }
    }
  }
  inspect(saw_nonzero_z, content="true")
}

///|
test "examples3d damping3: real backend smoke" {
  let pipeline = PhysicsPipeline3DReal::new()
  let broad_phase = @collision.BroadPhase3D::new()
  let narrow_phase = @collision.NarrowPhase3D::new()
  let islands = @dynamics.IslandManager3D::new()
  let bodies = @dynamics.RigidBodySet3D::new()
  let colliders = @collision.ColliderSet3D::new()
  let gravity = @core.Vec3::zero()
  let parameters = @dynamics.IntegrationParameters::default()
  let handles : Array[@dynamics.RigidBodyHandle] = []
  let num = 8
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
  step_world3_real(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    20,
  )
  for i in 0..<handles.length() {
    if bodies.get(handles[i]) is Some(rb) {
      let p = rb.translation()
      inspect(p.x == p.x, content="true")
      inspect(p.y == p.y, content="true")
      inspect(p.z == p.z, content="true")
    }
  }
}

///|
test "examples3d restitution3: real backend smoke" {
  let pipeline = PhysicsPipeline3DReal::new()
  let broad_phase = @collision.BroadPhase3D::new()
  let narrow_phase = @collision.NarrowPhase3D::new()
  let islands = @dynamics.IslandManager3D::new()
  let bodies = @dynamics.RigidBodySet3D::new()
  let colliders = @collision.ColliderSet3D::new()
  let gravity = @core.Vec3::new(0.0F, -9.81F, 0.0F)
  let parameters = @dynamics.IntegrationParameters::default()
  let handles : Array[@dynamics.RigidBodyHandle] = []
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder3D::fixed()
    .translation(@core.Vec3::new(0.0F, -1.0F, 0.0F))
    .build(),
  )
  handles.push(ground)
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(20.0F, 1.0F, 2.0F).build(),
    ground,
    bodies,
  )
  |> ignore
  let num = 6
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
  step_world3_real(
    pipeline, gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    10,
  )
  for i in 0..<handles.length() {
    if bodies.get(handles[i]) is Some(rb) {
      let p = rb.translation()
      inspect(p.x == p.x, content="true")
      inspect(p.y == p.y, content="true")
      inspect(p.z == p.z, content="true")
    }
  }
}
