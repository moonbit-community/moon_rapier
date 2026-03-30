#!/usr/bin/env bash
# Copyright 2025 International Digital Economy Academy
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

# Run all package tests except rapier_full to avoid default parity-test execution.
moon test --frozen \
  -p collision \
  -p control \
  -p core \
  -p counters \
  -p data \
  -p dynamics \
  -p pipeline \
  -p rapier3d \
  -p testbed \
  -p urdf \
  -p utils

# Run remaining non-parity tests in rapier_full.
moon test --frozen -p Milky2018/moon_rapier/rapier_full
