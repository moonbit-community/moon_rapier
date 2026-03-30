# Unified 2D/3D Parity Workflow

This repository now uses one parity workflow for both dimensions.

## Scope

- Reference source: `rapier-reference/src/**`.
- Crates audited together:
  - `rapier2d` (default features)
  - `rapier3d` (default features)
- Policy: semantics/design/algorithms align with Rapier; MoonBit API spelling may differ via explicit mapping.

## Single Gate Command

Run:

```bash
tools/run_pub_parity_gate.sh
```

The gate enforces all of the following in one run:

1. `python3 tools/rapier_pub_audit.py run`
   - requires `rapier2d` and `rapier3d` to both have `missing=0`.
2. `python3 tools/rapier_pub_style_audit.py run`
   - requires style class `renamed=0`.
3. Representative parity tests:
   - `moon test --frozen -p Milky2018/moon_rapier/rapier_full_parity -f examples2d_parity_test.mbt`
   - `moon test --frozen -p Milky2018/moon_rapier/rapier_full_parity -f examples3d_parity_test.mbt`

## CI Integration

GitHub Actions job `Pub parity gate (2D + 3D)` runs `tools/run_pub_parity_gate.sh` on every PR and push to `main`.

This makes 3D follow the exact same parity pipeline as 2D: audit -> style audit -> parity tests.
