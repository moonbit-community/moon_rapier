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
test "rapier full query smoke" {
  inspect(smoke_query(), content="true")
}

///|
test "rapier full character controller default move" {
  let controller = @control.KinematicCharacterController::new()
  let query = @collision.QueryPipeline::new(
    @collision.QueryFilter::new(),
    @dynamics.RigidBodySet::new(),
    @collision.ColliderSet::new(),
  )
  let movement = controller.move_shape(
    1.0F / 60.0F,
    query,
    @collision.Shape::Ball(0.5F),
    @core.Isometry2::identity(),
    @core.Vec2::zero(),
    fn(_collision : @control.CharacterCollision) -> Unit { () },
  )
  inspect(movement.translation().length_squared() == 0.0F, content="true")
}
