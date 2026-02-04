# Rapier Pub Audit (v0.32.0)

This repository tracks semantic parity with Rapier by:

- Running parity tests against `rapier-reference/examples2d/*` and `examples3d/*` (see `rapier_full/`).
- Auditing the **crate-level public surface** of `rapier2d` and `rapier3d` v0.32.0 (default features only)
  and comparing it against this MoonBit port's exported surface.

## Requirements

- A local checkout of `rapier-reference/` at the repository root (this folder is gitignored).
- Rust toolchain available (`cargo`, `rustc`).
- Python 3.

## Run

```bash
python3 tools/rapier_pub_audit.py run
```

Outputs are written under:

```text
_build/rapier_pub_audit/
```

Key files:

- `rapier2d_pub.json`: rustdoc-derived public symbols for `rapier2d`.
- `rapier3d_pub.json`: rustdoc-derived public symbols for `rapier3d`.
- `moon_exports.json`: exported MoonBit symbols from `*/pkg.generated.mbti` + `*/spec.mbt`.
- `report.json`: per-bucket missing lists.

## Mapping (Non-1:1 API names)

API names do not need to match. Add equivalences to:

```text
tools/rapier_pub_mapping.toml
```

The audit will count a Rapier symbol as "covered" if it maps to any existing MoonBit symbol.

