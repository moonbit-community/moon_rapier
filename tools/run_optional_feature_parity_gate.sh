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

if [[ ! -d "$ROOT/rapier-reference" ]]; then
  echo "[optional-gate] rapier-reference missing; cloning Rapier v0.32.0"
  git clone --depth 1 --branch v0.32.0 https://github.com/dimforge/rapier.git "$ROOT/rapier-reference"
fi

validate_report() {
  local report_path="$1"
  shift
  python3 - "$report_path" "$@" <<'PY'
import json
import sys
from pathlib import Path

report = json.loads(Path(sys.argv[1]).read_text())
for crate in sys.argv[2:]:
    totals = report[crate]["totals"]
    missing = int(totals["missing"])
    if missing != 0:
        raise SystemExit(f"{crate} optional parity missing items: {missing}")
    print(f"{crate}: items={totals['items']} covered={totals['covered']} missing={missing}")
PY
}

run_audit_profile() {
  local name="$1"
  local mode="$2"
  local features="$3"
  local outdir="$4"
  shift 4

  echo "[optional-gate] audit profile=${name} mode=${mode} features=${features}"
  python3 tools/rapier_pub_audit.py "${mode}" --features "${features}" --outdir "${outdir}"
  validate_report "${outdir}/report.json" "$@"
}

# NOTE: Rapier/Parry forbids enabling SIMD and enhanced-determinism together.
run_audit_profile \
  "f32-simd-stable" \
  "run" \
  "parallel,serde-serialize,debug-render,profiler,simd-stable" \
  "_build/rapier_pub_audit_opt_simd_stable" \
  "rapier2d" "rapier3d"

run_audit_profile \
  "f32-simd-nightly" \
  "run" \
  "parallel,serde-serialize,debug-render,profiler,simd-nightly" \
  "_build/rapier_pub_audit_opt_simd_nightly" \
  "rapier2d" "rapier3d"

run_audit_profile \
  "f32-enhanced-determinism" \
  "run" \
  "parallel,serde-serialize,debug-render,profiler,enhanced-determinism" \
  "_build/rapier_pub_audit_opt_enhanced_determinism" \
  "rapier2d" "rapier3d"

run_audit_profile \
  "f64-enhanced-determinism" \
  "run-f64" \
  "parallel,serde-serialize,debug-render,profiler,enhanced-determinism" \
  "_build/rapier_pub_audit_f64_opt_enhanced_determinism" \
  "rapier2d_f64" "rapier3d_f64"

echo "[optional-gate] run targeted semantic parity tests"
moon test --frozen -p pipeline -f feature_flags_test.mbt
moon test --frozen -p pipeline -f debug_render_pipeline_test.mbt
moon test --frozen -p pipeline -f snapshot_roundtrip_test.mbt
moon test --frozen -p counters -f counters_test.mbt
moon test --frozen -p utils -f serde_test.mbt

echo "[optional-gate] success"
