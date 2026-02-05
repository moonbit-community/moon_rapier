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
test "heightfield flags: FIX_INTERNAL_EDGES smoothes closest-point normal" {
  let heights0 = [0.0F, 0.0F, 1.0F, 0.0F]
  let heights1 = [0.0F, 0.0F, 1.0F, 0.0F]
  let scale = @core.Vec3::new(1.0F, 1.0F, 1.0F)
  let point = @core.Vec3::new(0.5F, 0.0F, 0.5F)
  let pos = @core.Isometry3::identity()
  let mut q0 = @core.Vec3::zero()
  let mut n0 = @core.Vec3::zero()
  let mut d20 = 1.0e30F
  if ColliderBuilder3D::heightfield(heights0, 2, 2, scale) is Some(b0) {
    let co0 = b0.build()
    match co0.shape() {
      Shape3D::Heightfield(vertices, _, rows, cols, flags) => {
        inspect(flags == 0, content="true")
        if heightfield_closest_point_query_with_flags(
            point, pos, vertices, rows, cols, flags,
          )
          is Some((q, n, d2)) {
          q0 = q
          n0 = n
          d20 = d2
        } else {
          inspect(false, content="true")
        }
      }
      _ => inspect(false, content="true")
    }
  } else {
    inspect(false, content="true")
  }
  let mut q1 = @core.Vec3::zero()
  let mut n1 = @core.Vec3::zero()
  let mut d21 = 1.0e30F
  if ColliderBuilder3D::heightfield_with_flags(heights1, 2, 2, scale, 1)
    is Some(b1) {
    let co1 = b1.build()
    match co1.shape() {
      Shape3D::Heightfield(vertices, _, rows, cols, flags) => {
        inspect(flags == 1, content="true")
        if heightfield_closest_point_query_with_flags(
            point, pos, vertices, rows, cols, flags,
          )
          is Some((q, n, d2)) {
          q1 = q
          n1 = n
          d21 = d2
        } else {
          inspect(false, content="true")
        }
      }
      _ => inspect(false, content="true")
    }
  } else {
    inspect(false, content="true")
  }

  // Both should project exactly onto the shared diagonal edge.
  inspect(d20 < 1.0e-8F, content="true")
  inspect(d21 < 1.0e-8F, content="true")
  inspect(@core.abs(q0.x - 0.5F) < 1.0e-6F, content="true")
  inspect(@core.abs(q0.y - 0.0F) < 1.0e-6F, content="true")
  inspect(@core.abs(q0.z - 0.5F) < 1.0e-6F, content="true")
  inspect(@core.abs(q1.x - 0.5F) < 1.0e-6F, content="true")
  inspect(@core.abs(q1.y - 0.0F) < 1.0e-6F, content="true")
  inspect(@core.abs(q1.z - 0.5F) < 1.0e-6F, content="true")

  // FIX_INTERNAL_EDGES should average normals across equally-close triangles, increasing the Y component.
  inspect(n1.y > n0.y + 0.2F, content="true")
}
