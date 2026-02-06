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
pub struct Timer {
  // In rapier, the timer only measures when the `profiler` feature is enabled.
  // This port keeps timers deterministic by default (always 0 unless a future profiler is added).
  mut time_ms : Double
}

///|
pub fn Timer::new() -> Timer {
  { time_ms: 0.0 }
}

///|
pub fn Timer::reset(self : Timer) -> Unit {
  self.time_ms = 0.0
}

///|
pub fn Timer::start(self : Timer) -> Unit {
  // Deterministic no-op profiling (mirrors rapier without `profiler` feature).
  self.time_ms = 0.0
}

///|
pub fn Timer::pause(self : Timer) -> Unit {
  // Deterministic no-op profiling (mirrors rapier without `profiler` feature).
  self.time_ms = self.time_ms
}

///|
pub fn Timer::resume_timer(self : Timer) -> Unit {
  // Deterministic no-op profiling (mirrors rapier without `profiler` feature).
  self.time_ms = self.time_ms
}

///|
pub fn Timer::time_ms(self : Timer) -> Double {
  self.time_ms
}

///|
pub fn Timer::time(self : Timer) -> Double {
  self.time_ms()
}

///|
pub fn Timer::to_string(self : Timer) -> String {
  self.time_ms.to_string() + "ms"
}
