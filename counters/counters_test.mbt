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
test "timer is deterministic by default" {
  let t = Timer::new()
  t.start()
  t.pause()
  inspect(t.time_ms() == 0.0, content="true")
  inspect(t.to_string().has_suffix("ms"[:]), content="true")
}

///|
test "counters gating + setters + formatting" {
  let c = Counters::new(false)
  inspect(!c.enabled(), content="true")
  c.step_started()
  c.step_completed()
  inspect(c.step_time_ms() == 0.0, content="true")
  c.enable()
  inspect(c.enabled(), content="true")
  c.set_nconstraints(12)
  c.set_ncontacts(34)
  c.set_ncontact_pairs(56)
  c.step_started()
  c.step_completed()
  let s = c.to_string()
  inspect(s.contains("Total timestep time:"), content="true")
  inspect(s.contains("Update time:"), content="true")
  inspect(s.contains("Number of contact pairs:"), content="true")
  inspect(s.contains("Number of constraints:"), content="true")
  inspect(s.contains("Custom timer:"), content="true")
}
