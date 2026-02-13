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
test "debug-render output is deterministic (best-effort)" {
  let bodies = @dynamics.RigidBodySet::new()
  let colliders = @collision.ColliderSet::new()
  let impulse_joints = @dynamics.ImpulseJointSet::new()
  let multibody_joints = @dynamics.MultibodyJointSet::new()
  let narrow_phase = @collision.NarrowPhase::new()
  let a = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(0.0F, 0.0F))
    .build(),
  )
  let b = bodies.insert(
    @dynamics.RigidBodyBuilder::dynamic()
    .translation(@core.Vec2::new(2.0F, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder::cuboid(0.5F, 0.5F).build(),
    a,
    bodies,
  )
  |> ignore
  colliders.insert_with_parent(
    @collision.ColliderBuilder::ball(0.5F).build(),
    b,
    bodies,
  )
  |> ignore
  let revolute = @dynamics.RevoluteJointBuilder::new()
    .local_anchor1(@core.Vec2::zero())
    .local_anchor2(@core.Vec2::zero())
    .build()
  impulse_joints.insert(
    a,
    b,
    @dynamics.GenericJoint::from_revolute(revolute),
    true,
  )
  |> ignore
  multibody_joints.insert(a, b, revolute, true) |> ignore
  fn render_snapshot() -> String {
    let renderer = DebugRenderPipeline::default()
    let backend = DebugRenderBackend::new()
    renderer.render(
      backend, bodies, colliders, impulse_joints, multibody_joints, narrow_phase,
    )
    backend.serialize_lines()
  }

  let s1 = render_snapshot()
  let s2 = render_snapshot()
  inspect(s1.length() > 0, content="true")
  inspect(s1 == s2, content="true")
}

///|
test "debug-render backend polyline helpers and collider-aabb object are exported" {
  let backend = DebugRenderBackend::new()
  let color = DebugColor::new(0.0F, 0.0F, 0.0F, 1.0F)
  let obj = DebugRenderObject::ColliderAabb(
    @collision.ColliderHandle::from_raw_parts(7, 0),
  )
  let vertices = [
    @core.Vec2::new(0.0F, 0.0F),
    @core.Vec2::new(1.0F, 0.0F),
    @core.Vec2::new(1.0F, 1.0F),
  ]
  inspect(backend.filter_object(obj), content="true")

  backend.draw_line_strip(obj, vertices, color, true)
  inspect(backend.lines().length(), content="3")

  let backend2 = DebugRenderBackend::new()
  backend2.draw_polyline(obj, vertices, color, false)
  inspect(backend2.lines().length(), content="2")
}
