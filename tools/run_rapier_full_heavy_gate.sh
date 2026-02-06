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
PROFILE="${RAPIER_FULL_PROFILE:-heavy}"
PROFILE_UPPER="$(printf '%s' "$PROFILE" | tr '[:lower:]' '[:upper:]')"

if [[ "$PROFILE_UPPER" != "MEDIUM" && "$PROFILE_UPPER" != "HEAVY" && "$PROFILE_UPPER" != "FULLSCALE" ]]; then
  echo "Unsupported RAPIER_FULL_PROFILE='$PROFILE'. Use medium|heavy|fullscale." | tee -a "$LOG_FILE"
  exit 1
fi

run_file() {
  local file="$1"
  local filter="$2"
  echo "==> ${PROFILE_UPPER} ${file} (${filter})" | tee -a "$LOG_FILE"
  /usr/bin/time -p moon test --frozen --release --target native \
    --include-skipped -p rapier_full -f "$file" -F "$filter" \
    2>&1 | tee -a "$LOG_FILE"
  echo | tee -a "$LOG_FILE"
}

if [[ "$PROFILE_UPPER" == "HEAVY" ]]; then
  run_file "examples2d_s2d_pyramid_parity_test.mbt" "HEAVY examples2d/s2d_pyramid.rs*"
  run_file "examples3d_real_heightfield_parity_test.mbt" "HEAVY examples3d/heightfield3.rs*"
  run_file "examples3d_real_primitive_contacts_parity_test.mbt" "HEAVY examples3d/debug_cylinder3.rs*"
  run_file "examples3d_real_urdf_keva_voxels_parity_test.mbt" "HEAVY examples3d/voxels3.rs*"
fi

run_file "examples3d_trimesh_parity_test.mbt" "${PROFILE_UPPER} examples3d/trimesh3.rs*"
run_file "examples3d_worlds_parity_test.mbt" "${PROFILE_UPPER} examples3d/domino3.rs*"

echo "${PROFILE_UPPER}_GATE_DONE" | tee -a "$LOG_FILE"
