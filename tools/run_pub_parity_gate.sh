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
  echo "[parity-gate] rapier-reference missing; cloning Rapier v0.32.0"
  git clone --depth 1 --branch v0.32.0 https://github.com/dimforge/rapier.git "$ROOT/rapier-reference"
fi

echo "[parity-gate] run pub audit (rapier2d + rapier3d default features)"
python3 tools/rapier_pub_audit.py run

echo "[parity-gate] run pub style audit"
python3 tools/rapier_pub_style_audit.py run

echo "[parity-gate] validate audit outputs"
python3 - <<'PY'
import json
from pathlib import Path

root = Path.cwd()
report = json.loads((root / "_build/rapier_pub_audit/report.json").read_text())
style = json.loads((root / "_build/rapier_pub_audit/style_report.json").read_text())

for crate in ("rapier2d", "rapier3d"):
    totals = report[crate]["totals"]
    missing = int(totals["missing"])
    if missing != 0:
        raise SystemExit(f"{crate} parity missing items: {missing}")
    print(f"{crate}: items={totals['items']} covered={totals['covered']} missing={missing}")

renamed = int(style.get("totals", {}).get("renamed", 0))
if renamed != 0:
    raise SystemExit(f"style parity has renamed entries: {renamed}")
print(f"style parity: renamed={renamed}")
PY

echo "[parity-gate] run representative 2D/3D parity tests"
moon test --frozen -p rapier_full -f examples2d_parity_test.mbt
moon test --frozen -p rapier_full -f examples3d_parity_test.mbt

echo "[parity-gate] success"
