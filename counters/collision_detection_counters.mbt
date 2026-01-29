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
pub struct CollisionDetectionCounters {
  mut ncontact_pairs : Int
  broad_phase_time : Timer
  final_broad_phase_time : Timer
  narrow_phase_time : Timer
}

///|
pub fn CollisionDetectionCounters::new() -> CollisionDetectionCounters {
  {
    ncontact_pairs: 0,
    broad_phase_time: Timer::new(),
    final_broad_phase_time: Timer::new(),
    narrow_phase_time: Timer::new(),
  }
}

///|
pub fn CollisionDetectionCounters::reset(self : CollisionDetectionCounters) -> Unit {
  self.ncontact_pairs = 0
  self.broad_phase_time.reset()
  self.final_broad_phase_time.reset()
  self.narrow_phase_time.reset()
}

///|
pub fn CollisionDetectionCounters::ncontact_pairs(
  self : CollisionDetectionCounters,
) -> Int {
  self.ncontact_pairs
}

///|
pub fn CollisionDetectionCounters::set_ncontact_pairs(
  self : CollisionDetectionCounters,
  n : Int,
) -> Unit {
  self.ncontact_pairs = n
}

///|
pub fn CollisionDetectionCounters::to_string(
  self : CollisionDetectionCounters,
) -> String {
  "Number of contact pairs: " + self.ncontact_pairs.to_string() + "\n" +
  "Broad-phase time: " + self.broad_phase_time.to_string() + "\n" +
  "Final broad-phase time: " + self.final_broad_phase_time.to_string() + "\n" +
  "Narrow-phase time: " + self.narrow_phase_time.to_string()
}

