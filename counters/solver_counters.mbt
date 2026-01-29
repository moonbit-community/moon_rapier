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
pub struct SolverCounters {
  mut nconstraints : Int
  mut ncontacts : Int
  velocity_resolution_time : Timer
  velocity_assembly_time : Timer
  velocity_assembly_time_solver_bodies : Timer
  velocity_assembly_time_constraints_init : Timer
  velocity_update_time : Timer
  velocity_writeback_time : Timer
}

///|
pub fn SolverCounters::new() -> SolverCounters {
  {
    nconstraints: 0,
    ncontacts: 0,
    velocity_assembly_time: Timer::new(),
    velocity_assembly_time_solver_bodies: Timer::new(),
    velocity_assembly_time_constraints_init: Timer::new(),
    velocity_resolution_time: Timer::new(),
    velocity_update_time: Timer::new(),
    velocity_writeback_time: Timer::new(),
  }
}

///|
pub fn SolverCounters::reset(self : SolverCounters) -> Unit {
  self.nconstraints = 0
  self.ncontacts = 0
  self.velocity_resolution_time.reset()
  self.velocity_assembly_time.reset()
  self.velocity_assembly_time_solver_bodies.reset()
  self.velocity_assembly_time_constraints_init.reset()
  self.velocity_update_time.reset()
  self.velocity_writeback_time.reset()
}

///|
pub fn SolverCounters::nconstraints(self : SolverCounters) -> Int {
  self.nconstraints
}

///|
pub fn SolverCounters::set_nconstraints(self : SolverCounters, n : Int) -> Unit {
  self.nconstraints = n
}

///|
pub fn SolverCounters::ncontacts(self : SolverCounters) -> Int {
  self.ncontacts
}

///|
pub fn SolverCounters::set_ncontacts(self : SolverCounters, n : Int) -> Unit {
  self.ncontacts = n
}

///|
pub fn SolverCounters::to_string(self : SolverCounters) -> String {
  "Number of contacts: " +
  self.ncontacts.to_string() +
  "\n" +
  "Number of constraints: " +
  self.nconstraints.to_string() +
  "\n" +
  "Velocity assembly time: " +
  self.velocity_assembly_time.to_string() +
  "\n" +
  "Velocity resolution time: " +
  self.velocity_resolution_time.to_string() +
  "\n" +
  "Velocity update time: " +
  self.velocity_update_time.to_string() +
  "\n" +
  "Velocity writeback time: " +
  self.velocity_writeback_time.to_string()
}
