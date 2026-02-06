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

LOG_DIR="${LOG_DIR:-_build}"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/rapier_full_heavy_gate.log"
: > "$LOG_FILE"

run_file() {
  local file="$1"
  local filter="$2"
  echo "==> HEAVY ${file} (${filter})" | tee -a "$LOG_FILE"
  /usr/bin/time -p moon test --frozen --release --target native \
    --include-skipped -p rapier_full -f "$file" -F "$filter" \
    2>&1 | tee -a "$LOG_FILE"
  echo | tee -a "$LOG_FILE"
}

run_file "examples3d_worlds_parity_test.mbt" "HEAVY examples3d/domino3.rs*"

echo "HEAVY_GATE_DONE" | tee -a "$LOG_FILE"
