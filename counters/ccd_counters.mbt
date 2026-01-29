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
pub struct CCDCounters {
  mut num_substeps : Int
  toi_computation_time : Timer
  solver_time : Timer
  broad_phase_time : Timer
  narrow_phase_time : Timer
}

///|
pub fn CCDCounters::new() -> CCDCounters {
  {
    num_substeps: 0,
    toi_computation_time: Timer::new(),
    solver_time: Timer::new(),
    broad_phase_time: Timer::new(),
    narrow_phase_time: Timer::new(),
  }
}

///|
pub fn CCDCounters::reset(self : CCDCounters) -> Unit {
  self.num_substeps = 0
  self.toi_computation_time.reset()
  self.solver_time.reset()
  self.broad_phase_time.reset()
  self.narrow_phase_time.reset()
}

///|
pub fn CCDCounters::num_substeps(self : CCDCounters) -> Int {
  self.num_substeps
}

///|
pub fn CCDCounters::set_num_substeps(self : CCDCounters, n : Int) -> Unit {
  self.num_substeps = n
}

///|
pub fn CCDCounters::to_string(self : CCDCounters) -> String {
  "Number of substeps: " + self.num_substeps.to_string() + "\n" +
  "TOI computation time: " + self.toi_computation_time.to_string() + "\n" +
  "Constraints solver time: " + self.solver_time.to_string() + "\n" +
  "Broad-phase time: " + self.broad_phase_time.to_string() + "\n" +
  "Narrow-phase time: " + self.narrow_phase_time.to_string()
}

