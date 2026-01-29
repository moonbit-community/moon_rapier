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
pub struct StagesCounters {
  update_time : Timer
  collision_detection_time : Timer
  island_construction_time : Timer
  island_constraints_collection_time : Timer
  solver_time : Timer
  ccd_time : Timer
  user_changes : Timer
}

///|
pub fn StagesCounters::new() -> StagesCounters {
  {
    update_time: Timer::new(),
    collision_detection_time: Timer::new(),
    island_construction_time: Timer::new(),
    island_constraints_collection_time: Timer::new(),
    solver_time: Timer::new(),
    ccd_time: Timer::new(),
    user_changes: Timer::new(),
  }
}

///|
pub fn StagesCounters::reset(self : StagesCounters) -> Unit {
  self.update_time.reset()
  self.collision_detection_time.reset()
  self.island_construction_time.reset()
  self.island_constraints_collection_time.reset()
  self.solver_time.reset()
  self.ccd_time.reset()
  self.user_changes.reset()
}

///|
pub fn StagesCounters::to_string(self : StagesCounters) -> String {
  let s = "Update time: " +
    self.update_time.to_string() +
    "\n" +
    "Collision detection time: " +
    self.collision_detection_time.to_string() +
    "\n" +
    "Island construction time: " +
    self.island_construction_time.to_string() +
    "\n" +
    "Island construction time: " +
    self.island_constraints_collection_time.to_string() +
    "\n" +
    "Solver time: " +
    self.solver_time.to_string() +
    "\n" +
    "CCD time: " +
    self.ccd_time.to_string() +
    "\n" +
    "User changes time: " +
    self.user_changes.to_string()
  s
}
