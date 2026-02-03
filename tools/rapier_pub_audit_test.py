#!/usr/bin/env python3
#
# Minimal regression tests for tools/rapier_pub_audit.py parsing.
#
# Run:
#   python3 tools/rapier_pub_audit_test.py
#

from __future__ import annotations

import importlib.util
import pathlib
import tempfile


ROOT = pathlib.Path(__file__).resolve().parents[1]


def _load_audit_module():
    audit_path = ROOT / "tools" / "rapier_pub_audit.py"
    spec = importlib.util.spec_from_file_location("rapier_pub_audit", audit_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"failed to load module spec from {audit_path}")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def main() -> int:
    audit = _load_audit_module()

    with tempfile.TemporaryDirectory() as td:
        root = pathlib.Path(td)
        (root / "data").mkdir(parents=True, exist_ok=True)

        mbti = """// Generated using `moon info`, DON'T EDIT IT
package "Milky2018/moon_rapier/data"

pub struct Arena[T] {
  items : Array[Int]
}
pub fn[T] Arena::capacity(Self[T]) -> Int

pub(open) trait HasModifiedFlag {
  has_modified_flag(Self) -> Bool
  set_modified_flag(Self) -> Unit
}

pub const ActiveEvents::COLLISION_EVENTS : Int
"""
        (root / "data" / "pkg.generated.mbti").write_text(mbti, encoding="utf-8")

        out = audit.extract_moon_exports(root)
        syms = {e["symbol"] for e in out["exports"]}

        assert (
            "Milky2018/moon_rapier/data::Arena::capacity" in syms
        ), "generic fn parsing failed (Arena::capacity)"
        assert (
            "Milky2018/moon_rapier/data::HasModifiedFlag::has_modified_flag" in syms
        ), "trait method parsing failed (HasModifiedFlag::has_modified_flag)"
        assert (
            "Milky2018/moon_rapier/data::ActiveEvents::COLLISION_EVENTS" in syms
        ), "associated const parsing failed (ActiveEvents::COLLISION_EVENTS)"

    print("ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

