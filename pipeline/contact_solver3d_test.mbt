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
test "contact solver 3d resolves approaching velocity (ball-ball)" {
  let bodies = @dynamics.RigidBodySet3D::new()
  let colliders = @collision.ColliderSet3D::new()
  let b1 = bodies.insert(
    @dynamics.RigidBodyBuilder3D::dynamic()
    .translation(@core.Vec3::new(0.0F, 0.0F, 0.0F))
    .build(),
  )
  let b2 = bodies.insert(
    @dynamics.RigidBodyBuilder3D::dynamic()
    .translation(@core.Vec3::new(1.8F, 0.0F, 0.0F))
    .linvel(@core.Vec3::new(-1.0F, 0.0F, 0.0F))
    .build(),
  )
  let c1 = colliders.insert_with_parent(
    @collision.ColliderBuilder3D::ball(1.0F).build(),
    b1,
    bodies,
  )
  let c2 = colliders.insert_with_parent(
    @collision.ColliderBuilder3D::ball(1.0F).build(),
    b2,
    bodies,
  )
  colliders.sync_with_bodies(bodies)
  let bp = @collision.BroadPhase3D::new()
  bp.update(0.0F, colliders)
  let nf = @collision.NarrowPhase3D::new()
  nf.update(bp.pairs(), bodies, colliders)
  let params = @dynamics.IntegrationParameters::default().set_dt(1.0F / 60.0F)
  let cache0 = ContactSolverCache3D::new()
  let cache1 = ContactSolverCache3D::new()
  solve_contacts_3d(params, bodies, colliders, nf, cache0, cache1)

  // Ensure the relative normal velocity is no longer strongly negative.
  if nf.contact_pair(c1, c2) is Some(pair) && pair.manifolds_len() == 1 {
    let cp = pair.manifolds()[0]
    if bodies.get(b1) is Some(rb1) && bodies.get(b2) is Some(rb2) {
      let n = cp.normal
      let v1 = rb1
        .linvel()
        .add(rb1.angvel().cross(cp.point1.sub(rb1.world_com())))
      let v2 = rb2
        .linvel()
        .add(rb2.angvel().cross(cp.point2.sub(rb2.world_com())))
      let vn = v2.sub(v1).dot(n)
      inspect(vn > -1.0e-3F, content="true")
    } else {
      inspect(false, content="true")
    }
  } else {
    inspect(false, content="true")
  }

  // Warmstart: applying the cache again should not drastically change the already-solved state.
  let v1_before = if bodies.get(b1) is Some(rb1) {
    rb1.linvel()
  } else {
    @core.Vec3::zero()
  }
  let v2_before = if bodies.get(b2) is Some(rb2) {
    rb2.linvel()
  } else {
    @core.Vec3::zero()
  }
  let cache2 = ContactSolverCache3D::new()
  solve_contacts_3d(params, bodies, colliders, nf, cache1, cache2)
  let v1_after = if bodies.get(b1) is Some(rb1) {
    rb1.linvel()
  } else {
    @core.Vec3::zero()
  }
  let v2_after = if bodies.get(b2) is Some(rb2) {
    rb2.linvel()
  } else {
    @core.Vec3::zero()
  }
  inspect(v1_after.sub(v1_before).length() < 1.0e-2F, content="true")
  inspect(v2_after.sub(v2_before).length() < 1.0e-2F, content="true")
}

///|
test "contact solver 3d restitution bounces (ball-ball)" {
  let bodies = @dynamics.RigidBodySet3D::new()
  let colliders = @collision.ColliderSet3D::new()
  let b1 = bodies.insert(
    @dynamics.RigidBodyBuilder3D::dynamic()
    .translation(@core.Vec3::new(0.0F, 0.0F, 0.0F))
    .build(),
  )
  let b2 = bodies.insert(
    @dynamics.RigidBodyBuilder3D::dynamic()
    .translation(@core.Vec3::new(2.0F, 0.0F, 0.0F))
    .linvel(@core.Vec3::new(-1.0F, 0.0F, 0.0F))
    .build(),
  )
  let c1 = colliders.insert_with_parent(
    @collision.ColliderBuilder3D::ball(1.0F).restitution(1.0F).build(),
    b1,
    bodies,
  )
  let c2 = colliders.insert_with_parent(
    @collision.ColliderBuilder3D::ball(1.0F).restitution(1.0F).build(),
    b2,
    bodies,
  )
  colliders.sync_with_bodies(bodies)
  let bp = @collision.BroadPhase3D::new()
  bp.update(0.0F, colliders)
  let nf = @collision.NarrowPhase3D::new()
  nf.update(bp.pairs(), bodies, colliders)
  let params = @dynamics.IntegrationParameters::default()
    .set_dt(1.0F / 60.0F)
    .set_num_solver_iterations(20)
  let cache0 = ContactSolverCache3D::new()
  let cache1 = ContactSolverCache3D::new()
  solve_contacts_3d(params, bodies, colliders, nf, cache0, cache1)
  if nf.contact_pair(c1, c2) is Some(pair) && pair.manifolds_len() == 1 {
    let cp = pair.manifolds()[0]
    if bodies.get(b1) is Some(rb1) && bodies.get(b2) is Some(rb2) {
      let n = cp.normal
      let v1 = rb1
        .linvel()
        .add(rb1.angvel().cross(cp.point1.sub(rb1.world_com())))
      let v2 = rb2
        .linvel()
        .add(rb2.angvel().cross(cp.point2.sub(rb2.world_com())))
      let vn = v2.sub(v1).dot(n)
      // With e=1, the relative velocity should flip to be positive.
      inspect(vn > 0.5F, content="true")
    } else {
      inspect(false, content="true")
    }
  } else {
    inspect(false, content="true")
  }
}

///|
test "contact solver 3d friction reduces tangential velocity (ball-ground)" {
  let bodies = @dynamics.RigidBodySet3D::new()
  let colliders = @collision.ColliderSet3D::new()
  let ground = bodies.insert(
    @dynamics.RigidBodyBuilder3D::fixed()
    .translation(@core.Vec3::new(0.0F, -1.0F, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::cuboid(10.0F, 1.0F, 10.0F)
    .friction(1.0F)
    .build(),
    ground,
    bodies,
  )
  |> ignore
  let ball = bodies.insert(
    @dynamics.RigidBodyBuilder3D::dynamic()
    .translation(@core.Vec3::new(0.0F, 0.0F, 0.0F))
    .linvel(@core.Vec3::new(1.0F, 0.0F, 0.0F))
    .build(),
  )
  colliders.insert_with_parent(
    @collision.ColliderBuilder3D::ball(0.5F).friction(1.0F).build(),
    ball,
    bodies,
  )
  |> ignore
  colliders.sync_with_bodies(bodies)
  let bp = @collision.BroadPhase3D::new()
  bp.update(0.0F, colliders)
  let nf = @collision.NarrowPhase3D::new()
  nf.update(bp.pairs(), bodies, colliders)
  let params = @dynamics.IntegrationParameters::default()
    .set_dt(1.0F / 60.0F)
    .set_num_solver_iterations(20)
  let cache0 = ContactSolverCache3D::new()
  let cache1 = ContactSolverCache3D::new()
  solve_contacts_3d(params, bodies, colliders, nf, cache0, cache1)
  if bodies.get(ball) is Some(rb) {
    inspect(rb.linvel().x < 0.2F, content="true")
  } else {
    inspect(false, content="true")
  }
}
