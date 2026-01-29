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
/// Performance counters for profiling/benchmarking the physics engine.
/// This mirrors the public surface of `rapier2d::counters` where feasible.

///|
#declaration_only
pub type Timer

///|
#declaration_only
pub fn Timer::new() -> Timer {
  ...
}

///|
#declaration_only
pub fn Timer::reset(self : Timer) -> Unit {
  ...
}

///|
#declaration_only
pub fn Timer::start(self : Timer) -> Unit {
  ...
}

///|
#declaration_only
pub fn Timer::pause(self : Timer) -> Unit {
  ...
}

///|
#declaration_only
pub fn Timer::resume_timer(self : Timer) -> Unit {
  ...
}

///|
#declaration_only
pub fn Timer::time_ms(self : Timer) -> Double {
  ...
}

///|
#declaration_only
pub fn Timer::to_string(self : Timer) -> String {
  ...
}

///|
#declaration_only
pub type StagesCounters

///|
#declaration_only
pub fn StagesCounters::new() -> StagesCounters {
  ...
}

///|
#declaration_only
pub fn StagesCounters::reset(self : StagesCounters) -> Unit {
  ...
}

///|
#declaration_only
pub fn StagesCounters::to_string(self : StagesCounters) -> String {
  ...
}

///|
#declaration_only
pub type CollisionDetectionCounters

///|
#declaration_only
pub fn CollisionDetectionCounters::new() -> CollisionDetectionCounters {
  ...
}

///|
#declaration_only
pub fn CollisionDetectionCounters::reset(
  self : CollisionDetectionCounters,
) -> Unit {
  ...
}

///|
#declaration_only
pub fn CollisionDetectionCounters::ncontact_pairs(
  self : CollisionDetectionCounters,
) -> Int {
  ...
}

///|
#declaration_only
pub fn CollisionDetectionCounters::set_ncontact_pairs(
  self : CollisionDetectionCounters,
  n : Int,
) -> Unit {
  ...
}

///|
#declaration_only
pub fn CollisionDetectionCounters::to_string(
  self : CollisionDetectionCounters,
) -> String {
  ...
}

///|
#declaration_only
pub type SolverCounters

///|
#declaration_only
pub fn SolverCounters::new() -> SolverCounters {
  ...
}

///|
#declaration_only
pub fn SolverCounters::reset(self : SolverCounters) -> Unit {
  ...
}

///|
#declaration_only
pub fn SolverCounters::nconstraints(self : SolverCounters) -> Int {
  ...
}

///|
#declaration_only
pub fn SolverCounters::set_nconstraints(self : SolverCounters, n : Int) -> Unit {
  ...
}

///|
#declaration_only
pub fn SolverCounters::ncontacts(self : SolverCounters) -> Int {
  ...
}

///|
#declaration_only
pub fn SolverCounters::set_ncontacts(self : SolverCounters, n : Int) -> Unit {
  ...
}

///|
#declaration_only
pub fn SolverCounters::to_string(self : SolverCounters) -> String {
  ...
}

///|
#declaration_only
pub type CCDCounters

///|
#declaration_only
pub fn CCDCounters::new() -> CCDCounters {
  ...
}

///|
#declaration_only
pub fn CCDCounters::reset(self : CCDCounters) -> Unit {
  ...
}

///|
#declaration_only
pub fn CCDCounters::num_substeps(self : CCDCounters) -> Int {
  ...
}

///|
#declaration_only
pub fn CCDCounters::set_num_substeps(self : CCDCounters, n : Int) -> Unit {
  ...
}

///|
#declaration_only
pub fn CCDCounters::to_string(self : CCDCounters) -> String {
  ...
}

///|
#declaration_only
pub type Counters

///|
#declaration_only
pub fn Counters::new(enabled : Bool) -> Counters {
  ...
}

///|
#declaration_only
pub fn Counters::default() -> Counters {
  ...
}

///|
#declaration_only
pub fn Counters::enable(self : Counters) -> Unit {
  ...
}

///|
#declaration_only
pub fn Counters::disable(self : Counters) -> Unit {
  ...
}

///|
#declaration_only
pub fn Counters::enabled(self : Counters) -> Bool {
  ...
}

///|
#declaration_only
pub fn Counters::reset(self : Counters) -> Unit {
  ...
}

///|
#declaration_only
pub fn Counters::step_started(self : Counters) -> Unit {
  ...
}

///|
#declaration_only
pub fn Counters::step_completed(self : Counters) -> Unit {
  ...
}

///|
#declaration_only
pub fn Counters::step_time_ms(self : Counters) -> Double {
  ...
}

///|
#declaration_only
pub fn Counters::custom_started(self : Counters) -> Unit {
  ...
}

///|
#declaration_only
pub fn Counters::custom_completed(self : Counters) -> Unit {
  ...
}

///|
#declaration_only
pub fn Counters::custom_time_ms(self : Counters) -> Double {
  ...
}

///|
#declaration_only
pub fn Counters::set_nconstraints(self : Counters, n : Int) -> Unit {
  ...
}

///|
#declaration_only
pub fn Counters::set_ncontacts(self : Counters, n : Int) -> Unit {
  ...
}

///|
#declaration_only
pub fn Counters::set_ncontact_pairs(self : Counters, n : Int) -> Unit {
  ...
}

///|
#declaration_only
pub fn Counters::update_started(self : Counters) -> Unit {
  ...
}

///|
#declaration_only
pub fn Counters::update_completed(self : Counters) -> Unit {
  ...
}

///|
#declaration_only
pub fn Counters::update_time_ms(self : Counters) -> Double {
  ...
}

///|
#declaration_only
pub fn Counters::collision_detection_started(self : Counters) -> Unit {
  ...
}

///|
#declaration_only
pub fn Counters::collision_detection_completed(self : Counters) -> Unit {
  ...
}

///|
#declaration_only
pub fn Counters::collision_detection_time_ms(self : Counters) -> Double {
  ...
}

///|
#declaration_only
pub fn Counters::island_construction_started(self : Counters) -> Unit {
  ...
}

///|
#declaration_only
pub fn Counters::island_construction_completed(self : Counters) -> Unit {
  ...
}

///|
#declaration_only
pub fn Counters::island_construction_time_ms(self : Counters) -> Double {
  ...
}

///|
#declaration_only
pub fn Counters::solver_started(self : Counters) -> Unit {
  ...
}

///|
#declaration_only
pub fn Counters::solver_completed(self : Counters) -> Unit {
  ...
}

///|
#declaration_only
pub fn Counters::solver_time_ms(self : Counters) -> Double {
  ...
}

///|
#declaration_only
pub fn Counters::ccd_started(self : Counters) -> Unit {
  ...
}

///|
#declaration_only
pub fn Counters::ccd_completed(self : Counters) -> Unit {
  ...
}

///|
#declaration_only
pub fn Counters::ccd_time_ms(self : Counters) -> Double {
  ...
}

///|
#declaration_only
pub fn Counters::assembly_started(self : Counters) -> Unit {
  ...
}

///|
#declaration_only
pub fn Counters::assembly_completed(self : Counters) -> Unit {
  ...
}

///|
#declaration_only
pub fn Counters::assembly_time_ms(self : Counters) -> Double {
  ...
}

///|
#declaration_only
pub fn Counters::velocity_resolution_started(self : Counters) -> Unit {
  ...
}

///|
#declaration_only
pub fn Counters::velocity_resolution_completed(self : Counters) -> Unit {
  ...
}

///|
#declaration_only
pub fn Counters::velocity_resolution_time_ms(self : Counters) -> Double {
  ...
}

///|
#declaration_only
pub fn Counters::velocity_update_started(self : Counters) -> Unit {
  ...
}

///|
#declaration_only
pub fn Counters::velocity_update_completed(self : Counters) -> Unit {
  ...
}

///|
#declaration_only
pub fn Counters::velocity_update_time_ms(self : Counters) -> Double {
  ...
}

///|
#declaration_only
pub fn Counters::broad_phase_started(self : Counters) -> Unit {
  ...
}

///|
#declaration_only
pub fn Counters::broad_phase_completed(self : Counters) -> Unit {
  ...
}

///|
#declaration_only
pub fn Counters::broad_phase_time_ms(self : Counters) -> Double {
  ...
}

///|
#declaration_only
pub fn Counters::narrow_phase_started(self : Counters) -> Unit {
  ...
}

///|
#declaration_only
pub fn Counters::narrow_phase_completed(self : Counters) -> Unit {
  ...
}

///|
#declaration_only
pub fn Counters::narrow_phase_time_ms(self : Counters) -> Double {
  ...
}

///|
#declaration_only
pub fn Counters::to_string(self : Counters) -> String {
  ...
}
