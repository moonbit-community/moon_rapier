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
/// Minimal dim3 island manager.
///
/// Rapier's island manager builds connected components of the interaction graph.
/// The real 3D backend will refine this further, but for now we maintain a list
/// of awake bodies that can be used by the pipeline.
pub struct IslandManager3D {
  active_bodies : Array[RigidBodyHandle]
}

///|
pub fn IslandManager3D::new() -> IslandManager3D {
  { active_bodies: [] }
}

///|
pub fn IslandManager3D::active_bodies(
  self : IslandManager3D,
) -> Array[RigidBodyHandle] {
  self.active_bodies
}

///|
pub fn IslandManager3D::update(
  self : IslandManager3D,
  bodies : RigidBodySet3D,
) -> Unit {
  self.active_bodies.clear()
  for i in 0..<bodies.bodies.length() {
    if bodies.bodies[i] is Some(body) {
      if body.is_active() {
        self.active_bodies.push(
          RigidBodyHandle::from_raw_parts(i, bodies.generations[i]),
        )
      }
    }
  }
}
