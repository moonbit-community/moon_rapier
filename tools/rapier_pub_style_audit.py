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

from __future__ import annotations

import argparse
import json
import pathlib
from collections import Counter, defaultdict
from typing import Dict, List, Tuple

from rapier_pub_audit import DEFAULT_MAPPING, ROOT, _parse_mapping_toml

OUTDIR = ROOT / "_build" / "rapier_pub_audit"
OUTFILE = OUTDIR / "style_report.json"


def bucket_for(path: str) -> str:
    segs = path.split("::")
    if len(segs) >= 2 and segs[1] in {
        "control",
        "counters",
        "data",
        "dynamics",
        "geometry",
        "pipeline",
        "utils",
        "math",
        "prelude",
    }:
        return segs[1]
    return "lib"


def leaf(path: str) -> str:
    return path.split("::")[-1]


def normalize_leaf(name: str) -> str:
    name2 = name.replace("3DReal", "")
    if name2.endswith("3D"):
        name2 = name2[:-2]
    if name2.endswith("2D"):
        name2 = name2[:-2]
    if name2.endswith("3"):
        name2 = name2[:-1]
    if name2.endswith("2"):
        name2 = name2[:-1]
    return name2


def classify_leaf_pair(rapier_leaf: str, moon_leaf: str) -> str:
    if rapier_leaf == moon_leaf:
        return "exact"
    if rapier_leaf == "test" and moon_leaf == "passes":
        return "keyword_alias"
    if rapier_leaf.isupper() and moon_leaf.lower() == rapier_leaf.lower():
        return "const_case_alias"
    if normalize_leaf(rapier_leaf) == normalize_leaf(moon_leaf):
        return "dimension_suffix_alias"
    if rapier_leaf.lower() == moon_leaf.lower():
        return "case_alias"
    if rapier_leaf.replace("_", "").lower() == moon_leaf.replace("_", "").lower():
        return "spelling_alias"
    return "renamed"


def best_class_for_mapping(rapier_path: str, moon_symbols: List[str]) -> Tuple[str, str]:
    rapier_leaf = leaf(rapier_path)
    ranked = {
        "exact": 0,
        "const_case_alias": 1,
        "case_alias": 2,
        "spelling_alias": 3,
        "dimension_suffix_alias": 4,
        "keyword_alias": 5,
        "renamed": 6,
    }
    best_cls = "renamed"
    best_sym = moon_symbols[0] if moon_symbols else ""
    for sym in moon_symbols:
        cls = classify_leaf_pair(rapier_leaf, leaf(sym))
        if ranked[cls] < ranked[best_cls]:
            best_cls = cls
            best_sym = sym
    return best_cls, best_sym


def build_style_report(mapping_path: pathlib.Path) -> Dict[str, object]:
    mapping, _ = _parse_mapping_toml(mapping_path)
    totals = Counter()
    by_bucket_counts: Dict[str, Counter] = defaultdict(Counter)
    by_bucket_samples: Dict[str, List[Dict[str, str]]] = defaultdict(list)

    for rapier_path, moon_symbols in sorted(mapping.items()):
        if not (rapier_path.startswith("rapier2d::") or rapier_path.startswith("rapier3d::")):
            continue
        bucket = bucket_for(rapier_path)
        cls, best_sym = best_class_for_mapping(rapier_path, moon_symbols)
        totals[cls] += 1
        totals["entries"] += 1
        by_bucket_counts[bucket][cls] += 1
        by_bucket_counts[bucket]["entries"] += 1
        if cls != "exact" and len(by_bucket_samples[bucket]) < 30:
            by_bucket_samples[bucket].append(
                {
                    "rapier": rapier_path,
                    "moon": best_sym,
                    "class": cls,
                }
            )

    buckets = {}
    for bucket in sorted(by_bucket_counts.keys()):
        buckets[bucket] = {
            "counts": dict(sorted(by_bucket_counts[bucket].items())),
            "samples": by_bucket_samples[bucket],
        }

    return {
        "mapping_file": str(mapping_path.relative_to(ROOT)),
        "totals": dict(sorted(totals.items())),
        "buckets": buckets,
    }


def cmd_run(mapping_path: pathlib.Path) -> int:
    report = build_style_report(mapping_path)
    OUTDIR.mkdir(parents=True, exist_ok=True)
    OUTFILE.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    totals = report["totals"]  # type: ignore[index]
    print(f"mapped entries: {totals.get('entries', 0)}")
    for key in sorted(totals.keys()):
        if key == "entries":
            continue
        print(f"{key}: {totals[key]}")
    print(f"wrote: {OUTFILE}")
    return 0


def main(argv: List[str]) -> int:
    parser = argparse.ArgumentParser(description="Rapier pub style-alignment audit")
    parser.add_argument("cmd", choices=["run"])
    parser.add_argument("--mapping", type=pathlib.Path, default=DEFAULT_MAPPING)
    args = parser.parse_args(argv)
    if args.cmd == "run":
        return cmd_run(args.mapping)
    return 1


if __name__ == "__main__":
    raise SystemExit(main(__import__("sys").argv[1:]))
