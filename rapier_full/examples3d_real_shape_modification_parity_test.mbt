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
/// Parity tests for `rapier-reference/examples3d/debug_shape_modification3.rs` against the real
/// dim3 backend (reduced, headless).

///|
test "examples3d/debug_shape_modification3.rs parity (3DReal headless; reduced)" {
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
  let ground_size = 20.0F
  let ground_height = 0.1F
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder3D::fixed()
    .translation(@core.Vec3::new(0.0F, -ground_height, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(
      ground_size, ground_height, ground_size,
    )
    .friction(0.15F)
    .build(),
    ground,
    bodies,
  )
  |> ignore

  // Rolling ball body.
  let ball_rad = 0.1F
  let ball = bodies.insert(
    @dynamics.RigidBodyBuilder3D::dynamic()
    .translation(@core.Vec3::new(0.0F, 0.2F, 0.0F))
    .linvel(@core.Vec3::new(10.0F, 0.0F, 0.0F))
    .can_sleep(false)
    .build(),
  )
  let ball_co = colliders.insert_with_parent(
    @collision.ColliderBuilder3D::ball(ball_rad).build(),
    ball,
    bodies,
  )

  // Static collider without body.
  let shape_size = 3.0F
  let static_ball = colliders.insert(
    @collision.ColliderBuilder3D::ball(shape_size)
    .translation(@core.Vec3::new(-15.0F, shape_size, 18.0F))
    .build(),
  )
  static_ball |> ignore

  // Shapeshifting collider without body.
  let shapeshifting = colliders.insert(
    @collision.ColliderBuilder3D::ball(shape_size)
    .translation(@core.Vec3::new(-15.0F, shape_size, 9.0F))
    .build(),
  )

  // Snapshot/restore of the ball.
  let snapped_frame = 25
  let mut snap_linvel = @core.Vec3::zero()
  let mut snap_angvel = @core.Vec3::zero()
  let mut snap_pos = @core.Isometry3::identity()
  let mut shape_idx = 0
  for step in 1..<=120 {
    // Snap the ball velocity or restore it.
    if bodies.get(ball) is Some(rb) && step == snapped_frame {
      snap_linvel = rb.linvel()
      snap_angvel = rb.angvel()
      snap_pos = rb.position()
    }
    if step % 20 == 0 {
      shape_idx = (shape_idx + 1) % 4
      if colliders.get_mut(shapeshifting) is Some(co) {
        let next = if shape_idx == 0 {
          @collision.Shape3D::ball(shape_size)
        } else if shape_idx == 1 {
          @collision.Shape3D::cuboid(shape_size, shape_size, shape_size)
        } else if shape_idx == 2 {
          @collision.Shape3D::cone(shape_size, shape_size)
        } else {
          @collision.Shape3D::cylinder(shape_size, shape_size)
        }
        co.set_shape(next)
      }
    }
    if step == 50 {
      if bodies.get_mut(ball) is Some(rb) {
        rb.set_linvel(snap_linvel)
        rb.set_angvel(snap_angvel)
        rb.set_translation(snap_pos.translation)
        rb.set_rotation(snap_pos.rotation)
      }
    }

    // Grow the ball collider (reduced growth vs rapier-reference).
    let new_r = ball_rad * (1.0F + 0.02F * Float::from_int(step))
    if colliders.get_mut(ball_co) is Some(co) {
      co.set_shape(@collision.Shape3D::ball(new_r))
    }
    pipeline.step(
      gravity, parameters, islands, broad_phase, narrow_phase, bodies, colliders,
    )
  }

  // Sanity: both the ball and shapeshifting collider remain valid and the ball stays finite.
  if bodies.get(ball) is Some(rb) {
    let p = rb.translation()
    inspect(p.x == p.x && p.y == p.y && p.z == p.z, content="true")
    inspect(p.y > -10.0F, content="true")
  } else {
    inspect(false, content="true")
  }
  if colliders.get(shapeshifting) is Some(co) {
    let aabb = co.compute_aabb()
    inspect(aabb.mins.x == aabb.mins.x, content="true")
    inspect(aabb.maxs.x == aabb.maxs.x, content="true")
  } else {
    inspect(false, content="true")
  }
}
