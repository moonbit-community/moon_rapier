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
test "voxels_from_points merges adjacent voxels" {
  let pts = [
    @core.Vec2::new(0.1F, 0.1F),
    @core.Vec2::new(0.9F, 0.1F),
    @core.Vec2::new(1.1F, 0.1F),
  ]
  let co = ColliderBuilder::voxels_from_points(@core.Vec2::new(1.0F, 1.0F), pts).build()
  if co.shape() is Shape::Compound(parts) {
    inspect(parts.length(), content="1")
    let part = parts[0]
    inspect(part.0.translation.x, content="1")
    inspect(part.0.translation.y, content="0.5")
    if part.1 is Shape::Cuboid(hx, hy) {
      inspect(hx, content="1")
      inspect(hy, content="0.5")
    } else {
      inspect(false, content="true")
    }
  } else {
    inspect(false, content="true")
  }
}

///|
test "voxelized_mesh SurfaceOnly produces voxels" {
  let vertices = [@core.Vec2::new(0.0F, 0.0F), @core.Vec2::new(2.0F, 0.0F)]
  let indices = [(0, 1)]
  let shape = Shape::voxelized_mesh(
    vertices,
    indices,
    1.0F,
    FillMode::surface_only(),
  )
  if shape is Shape::Compound(parts) {
    inspect(parts.length() > 0, content="true")
  } else {
    inspect(false, content="true")
  }
}
