# rapier_full Parity Test Profiles

This document defines the current runtime profiles for selected parity tests in `rapier_full`.

## Goals

- Keep default CI/local `moon test` runs tractable.
- Provide an opt-in heavier profile for stronger scenario coverage.
- Track target scales for eventual upstream-like full-scene validation.

## Running profiles

- Default profile (fast):
  - `moon test --frozen --release --target native -p rapier_full`
- Heavy profile gate (opt-in):
  - `tools/run_rapier_full_heavy_gate.sh`
  - This executes selected skipped tests prefixed with `HEAVY` using `--include-skipped`.

## Profile matrix

| File | Default profile (current) | Heavy profile (implemented) | Full-scale target (tracking) |
| --- | --- | --- | --- |
| `examples2d_s2d_pyramid_parity_test.mbt` | `base_count=30`, `steps=80` | _not yet wired_ | `base_count=100`, `steps=240` |
| `examples3d_real_heightfield_parity_test.mbt` | `nsubdivs=6`, `num=2`, `numy=2`, `steps=60` | _not yet wired_ | `nsubdivs=10`, `num=3`, `numy=3`, `steps=180` |
| `examples3d_real_primitive_contacts_parity_test.mbt` | broad stability thresholds | _not yet wired_ | stricter contact/stability thresholds |
| `examples3d_real_urdf_keva_voxels_parity_test.mbt` | reduced keva/urdf/voxels scales | _not yet wired_ | upstream-like scene density and step counts |
| `examples3d_trimesh_parity_test.mbt` | `nsubdivs=1`, `steps={20,5,5}` | `nsubdivs=2`, `steps={30,10,8}` via skipped HEAVY tests (manual run) | restore upstream-like mesh density and longer settling |
| `examples3d_worlds_parity_test.mbt` | reduced `platform/domino/fountain` scales (`steps={20,30,20}`) | heavier scales (`steps={30,60,40}`) via `HEAVY examples3d/domino3.rs*` | restore upstream-like scene density and longer settling |

## Notes

- Heavy tests are intentionally marked with `#skip(...)` and are only run when `--include-skipped` is provided.
- The scripted heavy gate currently runs the representative `worlds/domino` heavy scenario only, to keep wall-clock time bounded.
- Full-scale targets remain tracked by BD issue `moon_rapier-restore-fullscale-rapier_full-parity-tests`.
