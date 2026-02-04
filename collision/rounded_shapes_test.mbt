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
test "round_* builders create rounded geometry (not contact_skin)" {
  let collider = ColliderBuilder::round_cuboid(1.0F, 1.0F, 0.2F).build()
  inspect(collider.contact_skin() == 0.0F, content="true")
  match collider.shape() {
    Shape::Round(_, r) => inspect(@core.abs(r - 0.2F) < 1.0e-6F, content="true")
    _ => inspect(false, content="true")
  }
}

///|
test "rounded shapes affect query projections and rays" {
  fn collider_is_round(colliders : ColliderSet, id : Int) -> Bool {
    if id < 0 || id >= colliders.colliders.length() {
      return false
    }
    match colliders.colliders[id] {
      Some(co) =>
        match co.shape() {
          Shape::Round(_, _) => true
          _ => false
        }
      None => false
    }
  }

  fn project_left_x(
    bodies : @dynamics.RigidBodySet,
    colliders : ColliderSet,
  ) -> @core.Real {
    let pipeline = QueryPipeline::new(QueryFilter::new(), bodies, colliders)
    let point = @core.Vec2::new(-3.0F, 0.0F)
    match pipeline.project_point(bodies, colliders, point, 0.0F, false) {
      Some(hit) => hit.1.point().x
      None => 1.0e30F
    }
  }

  fn cast_left_hit_toi(
    bodies : @dynamics.RigidBodySet,
    colliders : ColliderSet,
    max_toi : @core.Real,
  ) -> @core.Real {
    let pipeline = QueryPipeline::new(QueryFilter::new(), bodies, colliders)
    let ray = Ray::new(
      @core.Vec2::new(-3.0F, 0.0F),
      @core.Vec2::new(1.0F, 0.0F),
    )
    match pipeline.cast_ray(bodies, colliders, ray, max_toi, true) {
      Some(hit) => hit.1
      None => -1.0F
    }
  }

  let bodies0 = @dynamics.RigidBodySet::new()
  let colliders_round0 = ColliderSet::new()
  colliders_round0.insert(
    ColliderBuilder::round_cuboid(1.0F, 1.0F, 0.2F).build(),
  )
  |> ignore
  inspect(collider_is_round(colliders_round0, 0), content="true")
  let proj_round = project_left_x(bodies0, colliders_round0)
  inspect(colliders_round0.colliders[0] is Some(_), content="true")
  inspect(collider_is_round(colliders_round0, 0), content="true")
  inspect(@core.abs(proj_round - -1.2F) < 1.0e-3F, content="true")
  let toi_round_short = cast_left_hit_toi(bodies0, colliders_round0, 1.9F)
  inspect(toi_round_short >= 0.0F, content="true")
  let bodies1 = @dynamics.RigidBodySet::new()
  let colliders_skin1 = ColliderSet::new()
  colliders_skin1.insert(
    ColliderBuilder::cuboid(1.0F, 1.0F).contact_skin(0.2F).build(),
  )
  |> ignore
  let proj_skin = project_left_x(bodies1, colliders_skin1)
  inspect(@core.abs(proj_skin - -1.0F) < 1.0e-3F, content="true")
  let toi_skin_short = cast_left_hit_toi(bodies1, colliders_skin1, 1.9F)
  inspect(toi_skin_short < 0.0F, content="true")
}
