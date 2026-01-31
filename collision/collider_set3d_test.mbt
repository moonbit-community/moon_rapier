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
test "collider_set3d insert/remove generation" {
  let set = ColliderSet3D::new()
  let h0 = set.insert(ColliderBuilder3D::ball(0.5F).build())
  inspect(set.len() == 1, content="true")
  let old = h0
  let removed = set.remove(h0)
  inspect(removed is Some(_), content="true")
  inspect(set.len() == 0, content="true")
  // Old handle becomes invalid after removal.
  inspect(set.get(old) is None, content="true")
  // Re-insert should reuse the slot but bump generation.
  let h1 = set.insert(ColliderBuilder3D::ball(1.0F).build())
  inspect(!h1.equals(old), content="true")
  inspect(set.get(h1) is Some(_), content="true")
}
