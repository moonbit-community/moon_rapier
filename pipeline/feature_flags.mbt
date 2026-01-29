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
/// Feature parity note:
/// - Rapier's `parallel` feature (rayon) is not available in this MoonBit port.
/// - Rapier's `simd-stable` / `simd-nightly` features are not available in this MoonBit port.
///
/// This port always runs deterministically on a single thread. Public APIs are provided with
/// sequential semantics, and tests must not depend on parallel scheduling.
pub fn parallel_enabled() -> Bool {
  false
}

///|
pub fn simd_enabled() -> Bool {
  false
}

///|
pub fn parallel_strategy() -> String {
  "sequential"
}

///|
pub fn simd_strategy() -> String {
  "none"
}
