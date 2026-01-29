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
pub struct Counters {
  mut enabled : Bool
  step_time : Timer
  custom : Timer
  stages : StagesCounters
  cd : CollisionDetectionCounters
  solver : SolverCounters
  ccd : CCDCounters
}

///|
pub fn Counters::new(enabled : Bool) -> Counters {
  {
    enabled,
    step_time: Timer::new(),
    custom: Timer::new(),
    stages: StagesCounters::new(),
    cd: CollisionDetectionCounters::new(),
    solver: SolverCounters::new(),
    ccd: CCDCounters::new(),
  }
}

///|
pub fn Counters::default() -> Counters {
  Counters::new(false)
}

///|
pub fn Counters::enable(self : Counters) -> Unit {
  self.enabled = true
}

///|
pub fn Counters::disable(self : Counters) -> Unit {
  self.enabled = false
}

///|
pub fn Counters::enabled(self : Counters) -> Bool {
  self.enabled
}

///|
pub fn Counters::step_started(self : Counters) -> Unit {
  if self.enabled {
    self.step_time.start()
  }
}

///|
pub fn Counters::step_completed(self : Counters) -> Unit {
  if self.enabled {
    self.step_time.pause()
  }
}

///|
pub fn Counters::step_time_ms(self : Counters) -> Double {
  if self.enabled {
    self.step_time.time_ms()
  } else {
    0.0
  }
}

///|
pub fn Counters::custom_started(self : Counters) -> Unit {
  if self.enabled {
    self.custom.start()
  }
}

///|
pub fn Counters::custom_completed(self : Counters) -> Unit {
  if self.enabled {
    self.custom.pause()
  }
}

///|
pub fn Counters::custom_time_ms(self : Counters) -> Double {
  if self.enabled {
    self.custom.time_ms()
  } else {
    0.0
  }
}

///|
pub fn Counters::set_nconstraints(self : Counters, n : Int) -> Unit {
  if self.enabled {
    self.solver.set_nconstraints(n)
  }
}

///|
pub fn Counters::set_ncontacts(self : Counters, n : Int) -> Unit {
  if self.enabled {
    self.solver.set_ncontacts(n)
  }
}

///|
pub fn Counters::set_ncontact_pairs(self : Counters, n : Int) -> Unit {
  if self.enabled {
    self.cd.set_ncontact_pairs(n)
  }
}

///|
pub fn Counters::reset(self : Counters) -> Unit {
  if self.enabled {
    self.step_time.reset()
    self.custom.reset()
    self.stages.reset()
    self.cd.reset()
    self.solver.reset()
    self.ccd.reset()
  }
}

///|
fn measure_started(enabled : Bool, timer : Timer) -> Unit {
  if enabled {
    timer.start()
  }
}

///|
fn measure_completed(enabled : Bool, timer : Timer) -> Unit {
  if enabled {
    timer.pause()
  }
}

///|
fn measure_time_ms(enabled : Bool, timer : Timer) -> Double {
  if enabled {
    timer.time_ms()
  } else {
    0.0
  }
}

///|
pub fn Counters::update_started(self : Counters) -> Unit {
  measure_started(self.enabled, self.stages.update_time)
}

///|
pub fn Counters::update_completed(self : Counters) -> Unit {
  measure_completed(self.enabled, self.stages.update_time)
}

///|
pub fn Counters::update_time_ms(self : Counters) -> Double {
  measure_time_ms(self.enabled, self.stages.update_time)
}

///|
pub fn Counters::collision_detection_started(self : Counters) -> Unit {
  measure_started(self.enabled, self.stages.collision_detection_time)
}

///|
pub fn Counters::collision_detection_completed(self : Counters) -> Unit {
  measure_completed(self.enabled, self.stages.collision_detection_time)
}

///|
pub fn Counters::collision_detection_time_ms(self : Counters) -> Double {
  measure_time_ms(self.enabled, self.stages.collision_detection_time)
}

///|
pub fn Counters::island_construction_started(self : Counters) -> Unit {
  measure_started(self.enabled, self.stages.island_construction_time)
}

///|
pub fn Counters::island_construction_completed(self : Counters) -> Unit {
  measure_completed(self.enabled, self.stages.island_construction_time)
}

///|
pub fn Counters::island_construction_time_ms(self : Counters) -> Double {
  measure_time_ms(self.enabled, self.stages.island_construction_time)
}

///|
pub fn Counters::solver_started(self : Counters) -> Unit {
  measure_started(self.enabled, self.stages.solver_time)
}

///|
pub fn Counters::solver_completed(self : Counters) -> Unit {
  measure_completed(self.enabled, self.stages.solver_time)
}

///|
pub fn Counters::solver_time_ms(self : Counters) -> Double {
  measure_time_ms(self.enabled, self.stages.solver_time)
}

///|
pub fn Counters::ccd_started(self : Counters) -> Unit {
  measure_started(self.enabled, self.stages.ccd_time)
}

///|
pub fn Counters::ccd_completed(self : Counters) -> Unit {
  measure_completed(self.enabled, self.stages.ccd_time)
}

///|
pub fn Counters::ccd_time_ms(self : Counters) -> Double {
  measure_time_ms(self.enabled, self.stages.ccd_time)
}

///|
pub fn Counters::assembly_started(self : Counters) -> Unit {
  measure_started(self.enabled, self.solver.velocity_assembly_time)
}

///|
pub fn Counters::assembly_completed(self : Counters) -> Unit {
  measure_completed(self.enabled, self.solver.velocity_assembly_time)
}

///|
pub fn Counters::assembly_time_ms(self : Counters) -> Double {
  measure_time_ms(self.enabled, self.solver.velocity_assembly_time)
}

///|
pub fn Counters::velocity_resolution_started(self : Counters) -> Unit {
  measure_started(self.enabled, self.solver.velocity_resolution_time)
}

///|
pub fn Counters::velocity_resolution_completed(self : Counters) -> Unit {
  measure_completed(self.enabled, self.solver.velocity_resolution_time)
}

///|
pub fn Counters::velocity_resolution_time_ms(self : Counters) -> Double {
  measure_time_ms(self.enabled, self.solver.velocity_resolution_time)
}

///|
pub fn Counters::velocity_update_started(self : Counters) -> Unit {
  measure_started(self.enabled, self.solver.velocity_update_time)
}

///|
pub fn Counters::velocity_update_completed(self : Counters) -> Unit {
  measure_completed(self.enabled, self.solver.velocity_update_time)
}

///|
pub fn Counters::velocity_update_time_ms(self : Counters) -> Double {
  measure_time_ms(self.enabled, self.solver.velocity_update_time)
}

///|
pub fn Counters::broad_phase_started(self : Counters) -> Unit {
  measure_started(self.enabled, self.cd.broad_phase_time)
}

///|
pub fn Counters::broad_phase_completed(self : Counters) -> Unit {
  measure_completed(self.enabled, self.cd.broad_phase_time)
}

///|
pub fn Counters::broad_phase_time_ms(self : Counters) -> Double {
  measure_time_ms(self.enabled, self.cd.broad_phase_time)
}

///|
pub fn Counters::narrow_phase_started(self : Counters) -> Unit {
  measure_started(self.enabled, self.cd.narrow_phase_time)
}

///|
pub fn Counters::narrow_phase_completed(self : Counters) -> Unit {
  measure_completed(self.enabled, self.cd.narrow_phase_time)
}

///|
pub fn Counters::narrow_phase_time_ms(self : Counters) -> Double {
  measure_time_ms(self.enabled, self.cd.narrow_phase_time)
}

///|
pub fn Counters::to_string(self : Counters) -> String {
  let s = "Total timestep time: " +
    self.step_time.to_string() +
    "\n" +
    self.stages.to_string() +
    "\n" +
    self.cd.to_string() +
    "\n" +
    self.solver.to_string() +
    "\n" +
    "Custom timer: " +
    self.custom.to_string()
  s
}
