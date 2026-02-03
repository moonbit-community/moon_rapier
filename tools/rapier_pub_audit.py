#!/usr/bin/env python3
#
# Rapier public surface audit for this MoonBit port.
#
# This script produces:
# - rustdoc-derived public API surface for rapier2d and rapier3d (v0.32.0) using
#   rustdoc JSON (unstable rustdoc format; enabled with RUSTC_BOOTSTRAP=1).
# - MoonBit export surface from */pkg.generated.mbti and */spec.mbt.
# - A report with per-bucket missing lists (control/counters/data/dynamics/geometry/
#   pipeline/utils + lib.rs bucket).
#
# Usage:
#   python3 tools/rapier_pub_audit.py run
#
# Outputs (by default) under _build/rapier_pub_audit/ (gitignored).

from __future__ import annotations

import argparse
import json
import os
import pathlib
import re
import subprocess
import sys
from typing import Any, Dict, Iterable, List, Optional, Sequence, Set, Tuple


ROOT = pathlib.Path(__file__).resolve().parents[1]
DEFAULT_OUTDIR = ROOT / "_build" / "rapier_pub_audit"
DEFAULT_MAPPING = ROOT / "tools" / "rapier_pub_mapping.toml"


def _eprint(*args: object) -> None:
    print(*args, file=sys.stderr)


def _run(cmd: Sequence[str], cwd: pathlib.Path, env: Optional[Dict[str, str]] = None) -> None:
    _eprint("+", " ".join(cmd))
    subprocess.run(cmd, cwd=str(cwd), env=env, check=True)


def _read_json(path: pathlib.Path) -> Any:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def _write_json(path: pathlib.Path, obj: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(obj, f, indent=2, sort_keys=True)
        f.write("\n")


def _normalize_name(s: str) -> str:
    # Heuristic normalization for MoonBit dim-suffixed names used by this port.
    # This is only for audit matching; it must not drive API design.
    s2 = s.replace("3DReal", "")
    if s2.endswith("2D"):
        s2 = s2[:-2]
    if s2.endswith("3D"):
        s2 = s2[:-2]
    return s2


def _parse_mapping_toml(path: pathlib.Path) -> Tuple[Dict[str, List[str]], Dict[str, str]]:
    # Minimal TOML subset parser:
    # - [map] and [ignore] tables
    # - quoted keys: "a::b::c" = "x" OR = ["x", "y"]
    table: Optional[str] = None
    mapping: Dict[str, List[str]] = {}
    ignore: Dict[str, str] = {}

    if not path.exists():
        return mapping, ignore

    raw = path.read_text(encoding="utf-8", errors="ignore").splitlines()
    for ln, line in enumerate(raw, start=1):
        s = line.strip()
        if not s or s.startswith("#"):
            continue
        if s.startswith("[") and s.endswith("]"):
            table = s[1:-1].strip()
            continue
        m = re.match(r'^"(?P<key>[^"]+)"\s*=\s*(?P<val>.+)$', s)
        if not m or table not in ("map", "ignore"):
            raise ValueError(f"{path}:{ln}: unsupported TOML line: {line}")
        key = m.group("key")
        val = m.group("val").strip()

        if table == "ignore":
            m2 = re.match(r'^"(?P<reason>[^"]*)"\s*$', val)
            if not m2:
                raise ValueError(f"{path}:{ln}: ignore values must be quoted strings")
            ignore[key] = m2.group("reason")
            continue

        # table == map
        if val.startswith("["):
            items = re.findall(r'"([^"]+)"', val)
            mapping[key] = items
        else:
            m2 = re.match(r'^"(?P<v>[^"]+)"\s*$', val)
            if not m2:
                raise ValueError(f"{path}:{ln}: map values must be quoted string or string array")
            mapping[key] = [m2.group("v")]

    return mapping, ignore


def _rustdoc_json_path(rapier_ref: pathlib.Path, crate: str) -> pathlib.Path:
    return rapier_ref / "target" / "doc" / f"{crate}.json"


def build_rustdoc_json(rapier_ref: pathlib.Path, crate: str) -> pathlib.Path:
    # rustdoc JSON is still "unstable options" on stable toolchains.
    # RUSTC_BOOTSTRAP=1 enables -Z for local auditing purposes.
    env = dict(os.environ)
    env["RUSTC_BOOTSTRAP"] = "1"
    _run(
        ["cargo", "rustdoc", "-p", crate, "--", "-Z", "unstable-options", "--output-format", "json"],
        cwd=rapier_ref,
        env=env,
    )
    out = _rustdoc_json_path(rapier_ref, crate)
    if not out.exists():
        raise FileNotFoundError(f"rustdoc JSON not found at {out}")
    return out


def _visibility_is_public(vis: str) -> bool:
    return vis == "public"


def _join_path(segs: Sequence[str]) -> str:
    return "::".join(segs)


def extract_rapier_pub_surface(rustdoc_json: pathlib.Path, crate_name: str) -> Dict[str, Any]:
    data = _read_json(rustdoc_json)
    index: Dict[str, Any] = data["index"]

    root_id = str(data["root"])
    root_item = index[root_id]

    # rustdoc JSON exposes the publicly documented surface via `paths`.
    # This includes items re-exported through `pub use` (including globs),
    # so we use it as the primary source of public symbols.
    paths_map: Dict[str, Any] = data.get("paths", {})
    id_to_path: Dict[str, str] = {}
    id_to_kind: Dict[str, str] = {}
    for item_id, info in paths_map.items():
        path = info.get("path", [])
        if not path:
            continue
        id_to_path[str(item_id)] = _join_path(path)
        id_to_kind[str(item_id)] = str(info.get("kind") or "")

    items: List[Dict[str, Any]] = []
    seen: Set[str] = set()

    def add_item(path: str, kind: str, bucket: str, meta: Optional[Dict[str, Any]] = None) -> None:
        key = f"{path}#{kind}"
        if key in seen:
            return
        seen.add(key)
        rec: Dict[str, Any] = {"path": path, "kind": kind, "bucket": bucket}
        if meta:
            rec.update(meta)
        items.append(rec)

    def bucket_for(path: str) -> str:
        segs = path.split("::")
        if len(segs) >= 2 and segs[1] in (
            "control",
            "counters",
            "data",
            "dynamics",
            "geometry",
            "pipeline",
            "utils",
        ):
            return segs[1]
        return "lib"

    def process_inherent_impls(type_path: str, impl_ids: List[int]) -> None:
        for impl_id in impl_ids:
            impl_item = index.get(str(impl_id))
            if not impl_item:
                continue
            imp = impl_item.get("inner", {}).get("impl")
            if not imp:
                continue
            if imp.get("trait") is not None:
                continue  # Skip trait impls (behavioral, not additional pub symbols).
            # Only include inherent associated items that are explicitly public.
            for assoc_id in imp.get("items", []):
                assoc = index.get(str(assoc_id))
                if not assoc:
                    continue
                if not _visibility_is_public(assoc.get("visibility", "")):
                    continue
                name = assoc.get("name")
                if not isinstance(name, str):
                    continue
                assoc_inner = assoc.get("inner", {})
                if "function" in assoc_inner:
                    add_item(f"{type_path}::{name}", "method", bucket_for(type_path))
                elif "assoc_const" in assoc_inner:
                    add_item(f"{type_path}::{name}", "assoc_const", bucket_for(type_path))
                elif "assoc_type" in assoc_inner:
                    add_item(f"{type_path}::{name}", "assoc_type", bucket_for(type_path))

    # Root module is stored under the crate item.
    root_mod = root_item.get("inner", {}).get("module")
    if not isinstance(root_mod, dict) or not root_mod.get("is_crate"):
        raise ValueError(f"unexpected rustdoc JSON root module shape for {crate_name}")

    # Start with everything rustdoc exposes under `paths` for this crate.
    add_item(crate_name, "mod", "lib")

    kind_map = {
        "module": "mod",
        "function": "fn",
        "constant": "const",
        "type_alias": "type",
        "variant": "variant",
        "struct": "struct",
        "enum": "enum",
        "trait": "trait",
    }

    public_ids: List[str] = []
    for item_id, path in id_to_path.items():
        if not path.startswith(crate_name + "::") and path != crate_name:
            continue
        public_ids.append(item_id)

    for item_id in sorted(public_ids, key=lambda i: (id_to_kind.get(i, ""), id_to_path.get(i, ""))):
        path = id_to_path[item_id]
        kind0 = kind_map.get(id_to_kind.get(item_id, ""), id_to_kind.get(item_id, "unknown"))
        add_item(path, kind0, bucket_for(path), {"item_id": int(item_id)})

        item = index.get(item_id)
        if not item:
            continue
        inner = item.get("inner", {})

        # Enrich: trait items, inherent methods, public fields.
        if "trait" in inner:
            tr = inner["trait"]
            for assoc_id in tr.get("items", []):
                assoc = index.get(str(assoc_id))
                if not assoc:
                    continue
                aname = assoc.get("name")
                if not isinstance(aname, str):
                    continue
                assoc_inner = assoc.get("inner", {})
                if "function" in assoc_inner:
                    add_item(f"{path}::{aname}", "trait_method", bucket_for(path))
                elif "assoc_type" in assoc_inner:
                    add_item(f"{path}::{aname}", "assoc_type", bucket_for(path))
                elif "assoc_const" in assoc_inner:
                    add_item(f"{path}::{aname}", "assoc_const", bucket_for(path))

        if "struct" in inner:
            st = inner["struct"]
            kind = st.get("kind", {})
            plain = kind.get("plain") if isinstance(kind, dict) else None
            if isinstance(plain, dict):
                for field_id in plain.get("fields", []):
                    fld = index.get(str(field_id))
                    if not fld:
                        continue
                    if not _visibility_is_public(fld.get("visibility", "")):
                        continue
                    fname = fld.get("name")
                    if isinstance(fname, str):
                        add_item(f"{path}::{fname}", "field", bucket_for(path))
            process_inherent_impls(path, st.get("impls", []))

        if "enum" in inner:
            en = inner["enum"]
            process_inherent_impls(path, en.get("impls", []))

    # Deterministic ordering.
    items_sorted = sorted(items, key=lambda x: (x["bucket"], x["kind"], x["path"]))
    return {
        "crate": crate_name,
        "items": items_sorted,
    }


MBTI_PKG_RE = re.compile(r'^\s*package\s+"([^"]+)"\s*$')
MBTI_TYPE_RE = re.compile(r"^\s*pub(?:\([^)]*\))?\s+(struct|enum|trait|type)\s+([A-Za-z_][A-Za-z0-9_]*)")
MBTI_FN_RE = re.compile(
    # Example:
    # - pub fn foo(Int) -> Int
    # - pub fn Foo::bar(Self) -> Unit
    # - pub fn[T] Foo::bar(Self[T]) -> Unit
    # - pub fn[T : Default] Foo::bar(Self[T]) -> Unit
    r"^\s*pub(?:\([^)]*\))?\s+fn(?:\[[^\]]+\])?\s+([A-Za-z_][A-Za-z0-9_]*)(?:::([A-Za-z_][A-Za-z0-9_]*))?"
)
MBTI_CONST_RE = re.compile(r"^\s*pub(?:\([^)]*\))?\s+const\s+([A-Za-z_][A-Za-z0-9_]*)\b")
MBTI_ASSOC_CONST_RE = re.compile(
    r"^\s*pub(?:\([^)]*\))?\s+const\s+([A-Za-z_][A-Za-z0-9_]*)(?:::([A-Za-z_][A-Za-z0-9_]*))\b"
)
MBTI_USING_RE = re.compile(r"^\s*pub\s+using\s+@[^\\s]+\s+\{([^}]*)\}\s*$")
MBTI_ALIAS_RE = re.compile(r"^\s*#alias\(([A-Za-z_][A-Za-z0-9_]*)\)\s*$")


def extract_moon_exports(root: pathlib.Path) -> Dict[str, Any]:
    exports: List[Dict[str, Any]] = []
    seen: Set[str] = set()

    def add(pkg: str, sym: str, kind: str, src: str) -> None:
        key = f"{pkg}::{sym}#{kind}"
        if key in seen:
            return
        seen.add(key)
        exports.append({"pkg": pkg, "symbol": f"{pkg}::{sym}", "kind": kind, "src": src})

    for fp in sorted(root.glob("*/pkg.generated.mbti")):
        pkg = None
        current_struct: Optional[str] = None
        current_enum: Optional[str] = None
        current_trait: Optional[str] = None
        src = str(fp.relative_to(root))
        for line in fp.read_text(encoding="utf-8", errors="ignore").splitlines():
            m = MBTI_PKG_RE.match(line)
            if m:
                pkg = m.group(1)
                continue
            if pkg is None:
                continue

            # Inside a struct block: collect field symbols.
            if current_struct is not None:
                if line.strip() == "}":
                    current_struct = None
                else:
                    mfield = re.match(r"^\s*(?:mut\s+)?([A-Za-z_][A-Za-z0-9_]*)\s*:\s*", line)
                    if mfield:
                        add(pkg, f"{current_struct}::{mfield.group(1)}", "field", src)
                continue

            # Inside an enum block: collect variant symbols.
            if current_enum is not None:
                if line.strip() == "}":
                    current_enum = None
                else:
                    # Example variants:
                    # - Foo
                    # - Bar(Int, String)
                    mvar = re.match(r"^\s*([A-Za-z_][A-Za-z0-9_]*)\b", line)
                    if mvar:
                        add(pkg, f"{current_enum}::{mvar.group(1)}", "variant", src)
                continue

            # Inside a trait block: collect method symbols.
            if current_trait is not None:
                if line.strip() == "}":
                    current_trait = None
                else:
                    # Example:
                    #   foo(Self) -> Unit
                    #   bar(Self, Int) -> Bool
                    mth = re.match(r"^\s*([A-Za-z_][A-Za-z0-9_]*)\s*\(", line)
                    if mth:
                        add(pkg, f"{current_trait}::{mth.group(1)}", "trait_method", src)
                continue

            # Block starts.
            mblock = re.match(
                r"^\s*pub(?:\([^)]*\))?\s+struct\s+([A-Za-z_][A-Za-z0-9_]*)\s*\{\s*$",
                line,
            )
            if mblock:
                current_struct = mblock.group(1)
                add(pkg, current_struct, "struct", src)
                continue
            mblock = re.match(
                r"^\s*pub(?:\([^)]*\))?\s+enum\s+([A-Za-z_][A-Za-z0-9_]*)\s*\{\s*$",
                line,
            )
            if mblock:
                current_enum = mblock.group(1)
                add(pkg, current_enum, "enum", src)
                continue
            mblock = re.match(
                r"^\s*pub(?:\([^)]*\))?\s+trait\s+([A-Za-z_][A-Za-z0-9_]*)\s*\{\s*$",
                line,
            )
            if mblock:
                current_trait = mblock.group(1)
                add(pkg, current_trait, "trait", src)
                continue

            m = MBTI_USING_RE.match(line)
            if m:
                # Example: pub using @collision {type ColliderBuilder}
                body = m.group(1)
                for name in re.findall(r"\btype\s+([A-Za-z_][A-Za-z0-9_]*)\b", body):
                    add(pkg, name, "type", src)
                for name in re.findall(r"\btrait\s+([A-Za-z_][A-Za-z0-9_]*)\b", body):
                    add(pkg, name, "trait", src)
                continue
            m = MBTI_ASSOC_CONST_RE.match(line)
            if m:
                add(pkg, f"{m.group(1)}::{m.group(2)}", "assoc_const", src)
                continue
            m = MBTI_CONST_RE.match(line)
            if m:
                add(pkg, m.group(1), "const", src)
                continue
            m = MBTI_ALIAS_RE.match(line)
            if m:
                add(pkg, m.group(1), "type", src)
                continue
            m = MBTI_TYPE_RE.match(line)
            if m:
                kind, name = m.group(1), m.group(2)
                add(pkg, name, kind, src)
                continue
            m = MBTI_FN_RE.match(line)
            if m:
                fn_name = m.group(1)
                method = m.group(2)
                if method:
                    add(pkg, f"{fn_name}::{method}", "method", src)
                else:
                    add(pkg, fn_name, "fn", src)
                continue

    for fp in sorted(root.glob("*/spec.mbt")):
        # spec.mbt isn't a strict signature file, but we can still capture
        # obvious exported identifiers to avoid missing things before `moon info`.
        pkg = None
        # Derive package name from folder.
        folder = fp.parent.name
        pkg = f"Milky2018/moon_rapier/{folder}"
        for line in fp.read_text(encoding="utf-8", errors="ignore").splitlines():
            m = re.match(r"^\s*pub\s+(struct|enum|trait|type)\s+([A-Za-z_][A-Za-z0-9_]*)", line)
            if m:
                add(pkg, m.group(2), m.group(1), str(fp.relative_to(root)))
            m = re.match(r"^\s*pub\s+fn\s+([A-Za-z_][A-Za-z0-9_]*)", line)
            if m:
                add(pkg, m.group(1), "fn", str(fp.relative_to(root)))

    exports_sorted = sorted(exports, key=lambda x: (x["pkg"], x["kind"], x["symbol"]))
    return {"exports": exports_sorted}


def report_missing(
    rapier2d: Dict[str, Any],
    rapier3d: Dict[str, Any],
    moon: Dict[str, Any],
    mapping_path: pathlib.Path,
) -> Dict[str, Any]:
    mapping, ignore = _parse_mapping_toml(mapping_path)

    moon_syms: Set[str] = set(e["symbol"] for e in moon["exports"])
    moon_names: Set[str] = set(e["symbol"].split("::")[-1] for e in moon["exports"])
    moon_norm_names: Set[str] = set(_normalize_name(n) for n in moon_names)

    def is_covered(rust_path: str) -> Tuple[bool, str]:
        if rust_path in ignore:
            return True, "ignored"
        mapped = mapping.get(rust_path)
        if mapped:
            for s in mapped:
                if s in moon_syms:
                    return True, "mapped"
            return False, "mapped-missing"
        # Heuristic match (names only).
        last = rust_path.split("::")[-1]
        if last in moon_names or _normalize_name(last) in moon_norm_names:
            return True, "name"
        # Methods: match Type::method by normalized suffix.
        if "::" in rust_path:
            parts = rust_path.split("::")
            if len(parts) >= 2:
                suffix2 = "::".join(parts[-2:])
                n1 = _normalize_name(parts[-2])
                suffix2n = f"{n1}::{parts[-1]}"
                # Compare against MoonBit "Type::method" suffix.
                for sym in moon_syms:
                    tail = "::".join(sym.split("::")[-2:])
                    if tail == suffix2 or _normalize_name(tail.split('::')[0]) + '::' + tail.split('::')[1] == suffix2n:
                        return True, "method-name"
        return False, "missing"

    def summarize(rapier: Dict[str, Any]) -> Dict[str, Any]:
        missing_by_bucket: Dict[str, List[Dict[str, Any]]] = {}
        totals = {"items": 0, "covered": 0, "missing": 0}
        for it in rapier["items"]:
            # Only count symbols that matter for parity.
            if it["kind"] in ("mod",):
                continue
            rust_path = it["path"]
            ok, how = is_covered(rust_path)
            totals["items"] += 1
            if ok:
                totals["covered"] += 1
                continue
            totals["missing"] += 1
            b = it["bucket"]
            missing_by_bucket.setdefault(b, []).append({"path": rust_path, "kind": it["kind"], "match": how})
        for b in missing_by_bucket:
            missing_by_bucket[b] = sorted(missing_by_bucket[b], key=lambda x: (x["kind"], x["path"]))
        return {"totals": totals, "missing_by_bucket": missing_by_bucket}

    return {
        "rapier2d": summarize(rapier2d),
        "rapier3d": summarize(rapier3d),
        "mapping_file": str(mapping_path.relative_to(ROOT)) if mapping_path.exists() else str(mapping_path),
        "ignored_count": len(ignore),
        "mapped_count": len(mapping),
    }


def cmd_run(args: argparse.Namespace) -> int:
    outdir = pathlib.Path(args.outdir).resolve()
    rapier_ref = pathlib.Path(args.rapier_ref).resolve()
    mapping = pathlib.Path(args.mapping).resolve()

    if not rapier_ref.exists():
        _eprint(f"error: rapier-reference not found at {rapier_ref}")
        return 2

    outdir.mkdir(parents=True, exist_ok=True)

    rapier2d_json = build_rustdoc_json(rapier_ref, "rapier2d")
    rapier3d_json = build_rustdoc_json(rapier_ref, "rapier3d")

    rapier2d = extract_rapier_pub_surface(rapier2d_json, "rapier2d")
    rapier3d = extract_rapier_pub_surface(rapier3d_json, "rapier3d")
    moon = extract_moon_exports(ROOT)
    rep = report_missing(rapier2d, rapier3d, moon, mapping)

    _write_json(outdir / "rapier2d_pub.json", rapier2d)
    _write_json(outdir / "rapier3d_pub.json", rapier3d)
    _write_json(outdir / "moon_exports.json", moon)
    _write_json(outdir / "report.json", rep)

    # Print a compact summary for humans.
    for label in ("rapier2d", "rapier3d"):
        t = rep[label]["totals"]
        _eprint(f"{label}: items={t['items']} covered={t['covered']} missing={t['missing']}")
    _eprint(f"wrote: {outdir}")
    return 0


def main(argv: Sequence[str]) -> int:
    ap = argparse.ArgumentParser(description="Audit Rapier pub surface vs MoonBit exports.")
    sub = ap.add_subparsers(dest="cmd", required=True)

    runp = sub.add_parser("run", help="Build rustdoc JSON + export MoonBit surface + write report to _build.")
    runp.add_argument("--rapier-ref", default=str(ROOT / "rapier-reference"), help="Path to rapier-reference checkout.")
    runp.add_argument("--mapping", default=str(DEFAULT_MAPPING), help="Path to rapier_pub_mapping.toml")
    runp.add_argument("--outdir", default=str(DEFAULT_OUTDIR), help="Output directory (default: _build/rapier_pub_audit)")
    runp.set_defaults(func=cmd_run)

    args = ap.parse_args(list(argv))
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
