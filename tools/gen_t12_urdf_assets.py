#!/usr/bin/env python3
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

"""
Generate test-only embedded assets for the 3DReal URDF parity test.

Outputs:
  - rapier_full/t12_urdf_assets_test.mbt

This script reads:
  - rapier-reference/assets/3d/T12/urdf/T12.URDF
  - rapier-reference/assets/3d/T12/meshes/*.STL

and emits:
  - a function returning the URDF XML as a String (built via StringBuilder)
  - a function returning a HashMap mesh_filename -> (mins, maxs) AABB
"""

from __future__ import annotations

import glob
import os
import struct
from typing import Dict, Tuple, List


URDF_PATH = "rapier-reference/assets/3d/T12/urdf/T12.URDF"
MESH_DIR = "rapier-reference/assets/3d/T12/meshes"
OUT_PATH = "rapier_full/t12_urdf_assets_test.mbt"


def read_stl_bounds(path: str) -> Tuple[List[float], List[float]]:
    with open(path, "rb") as f:
        f.read(80)
        n_bytes = f.read(4)
        if len(n_bytes) != 4:
            raise ValueError(f"invalid STL (missing tri count): {path}")
        n = struct.unpack("<I", n_bytes)[0]

        minv = [float("inf"), float("inf"), float("inf")]
        maxv = [float("-inf"), float("-inf"), float("-inf")]

        for _ in range(n):
            data = f.read(50)
            if len(data) < 50:
                break
            # normal (3f) + v1 (3f) + v2 (3f) + v3 (3f) + attr (H)
            vals = struct.unpack("<12fH", data)
            vs = vals[3:12]
            for i in range(0, 9, 3):
                x, y, z = vs[i], vs[i + 1], vs[i + 2]
                if x < minv[0]:
                    minv[0] = x
                if y < minv[1]:
                    minv[1] = y
                if z < minv[2]:
                    minv[2] = z
                if x > maxv[0]:
                    maxv[0] = x
                if y > maxv[1]:
                    maxv[1] = y
                if z > maxv[2]:
                    maxv[2] = z

        return minv, maxv


def esc_mbt_string(s: str) -> str:
    # ASCII-only output expected.
    return s.replace("\\", "\\\\").replace('"', '\\"')


def main() -> None:
    meshes = sorted(glob.glob(os.path.join(MESH_DIR, "*.STL")))
    if not meshes:
        raise SystemExit(f"no meshes found under: {MESH_DIR}")

    bounds: Dict[str, Tuple[List[float], List[float]]] = {}
    for p in meshes:
        bounds[os.path.basename(p)] = read_stl_bounds(p)

    with open(URDF_PATH, "r", encoding="utf-8") as f:
        urdf_lines = f.read().splitlines()

    license_header = """// Copyright 2025 International Digital Economy Academy
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
"""

    os.makedirs(os.path.dirname(OUT_PATH), exist_ok=True)
    with open(OUT_PATH, "w", encoding="ascii") as out:
        out.write(license_header)
        out.write("\n///|\n/// Test-only embedded assets for the T12 URDF parity test.\n\n")

        out.write("pub fn t12_urdf_xml() -> String {\n")
        out.write("  let b = StringBuilder::new()\n")
        for line in urdf_lines:
            out.write(f'  b.write_string("{esc_mbt_string(line)}\\n")\n')
        out.write("  b.to_string()\n")
        out.write("}\n\n")

        out.write(
            "pub fn t12_mesh_bounds_map() -> @hashmap.HashMap[String, (@core.Vec3, @core.Vec3)] {\n"
        )
        out.write(
            f"  let m : @hashmap.HashMap[String, (@core.Vec3, @core.Vec3)] = @hashmap.new(capacity={len(bounds)})\n"
        )
        for name, (mn, mx) in sorted(bounds.items()):
            out.write(
                "  m.set(\"%s\", (@core.Vec3::new(%0.6fF, %0.6fF, %0.6fF), @core.Vec3::new(%0.6fF, %0.6fF, %0.6fF)))\n"
                % (name, mn[0], mn[1], mn[2], mx[0], mx[1], mx[2])
            )
        out.write("  m\n")
        out.write("}\n")

    print(f"wrote {OUT_PATH}: {len(urdf_lines)} lines, {len(bounds)} meshes")


if __name__ == "__main__":
    main()

