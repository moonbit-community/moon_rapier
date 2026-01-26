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
/// Upstream parity test list for full Rapier functionality.
/// These are intentionally skipped until the corresponding features are ported.
#skip
test "rapier full: joint limits and motors parity" {
  inspect(true, content="true")
}

///|
#skip
test "rapier full: generic joint coupled axes parity" {
  inspect(true, content="true")
}

///|
#skip
test "rapier full: multibody self-contact filtering parity" {
  inspect(true, content="true")
}

///|
#skip
test "rapier full: CCD motion clamping regression vectors" {
  inspect(true, content="true")
}

///|
#skip
test "rapier full: pipeline event stream parity (collision/contact force)" {
  inspect(true, content="true")
}

///|
#skip
test "rapier full: query pipeline shapecast vectors" {
  inspect(true, content="true")
}

///|
#skip
test "rapier full: character controller autostep parity" {
  inspect(true, content="true")
}
