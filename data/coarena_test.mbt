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
test "coarena insert/get/remove uses generation for validity" {
  let co : Coarena[Int] = Coarena::new()
  let idx = Index::from_raw_parts(5, 7)
  co.insert(idx, 42)
  inspect(co.get(idx) == Some(42), content="true")
  inspect(co.get(Index::from_raw_parts(5, 6)) is None, content="true")
  let removed = co.remove(idx, 0)
  inspect(removed == Some(42), content="true")
  inspect(co.get(idx) is None, content="true")
  // The unknown-gen accessor sees the "removed_value" we replaced with.
  inspect(co.get_unknown_gen(5) == Some(0), content="true")
}
