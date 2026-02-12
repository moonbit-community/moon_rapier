# rapier_full Parity Test Profiles

This document defines the current runtime profiles for selected parity tests in `rapier_full`.

## Goals

- Keep default CI/local `moon test` runs tractable.
- Provide an opt-in heavier profile for stronger scenario coverage.
- Track target scales for eventual upstream-like full-scene validation.

## Running profiles

- Build behavior update:
  - `moon build` now defaults to debug output.
  - Any script requiring optimized binaries must use `moon build --release`.
- Default profile (fast):
  - `moon test --frozen --release --target native -p rapier_full`
- Tiered profile gate (opt-in):
  - `tools/run_rapier_full_heavy_gate.sh` (defaults to `RAPIER_FULL_PROFILE=heavy`)
  - `RAPIER_FULL_PROFILE=medium tools/run_rapier_full_heavy_gate.sh`
  - `RAPIER_FULL_PROFILE=fullscale tools/run_rapier_full_heavy_gate.sh`
  - The script executes skipped tests prefixed with `MEDIUM`/`HEAVY`/`FULLSCALE` using `--include-skipped`.
  - Runtime budgets are enforced by profile:
    - `MEDIUM`: per-scenario `<= 90s`, total `<= 180s`
    - `HEAVY`: per-scenario `<= 120s`, total `<= 300s`
    - `FULLSCALE`: per-scenario `<= 180s`, total `<= 360s`
  - Budget overrides:
    - `RAPIER_FULL_SCENARIO_BUDGET_SEC=<seconds>`
    - `RAPIER_FULL_TOTAL_BUDGET_SEC=<seconds>`
  - Native warmup uses `moon build --frozen --release --target native rapier_full`
    (excluded from budget); disable with:
    - `RAPIER_FULL_SKIP_WARMUP=1`

## Profile matrix

| File | Default profile (current) | Heavy profile (implemented) | Full-scale target (tracking) |
| --- | --- | --- | --- |
| `examples2d_s2d_pyramid_parity_test.mbt` | `base_count=30`, `steps=80` | `base_count=45`, `steps=120` via `HEAVY examples2d/s2d_pyramid.rs*` | `base_count=100`, `steps=240` |
| `examples3d_real_heightfield_parity_test.mbt` | `nsubdivs=6`, `num=2`, `numy=2`, `steps=60` | `nsubdivs=7`, `num=3`, `numy=2`, `steps=80` via `HEAVY examples3d/heightfield3.rs*` | `nsubdivs=10`, `num=3`, `numy=3`, `steps=180` |
| `examples3d_real_primitive_contacts_parity_test.mbt` | baseline contact/stability thresholds | higher solver iteration via `HEAVY examples3d/debug_cylinder3.rs*` | stricter contact/stability thresholds |
| `examples3d_real_urdf_keva_voxels_parity_test.mbt` | reduced keva/urdf/voxels scales | denser voxel scene via `HEAVY examples3d/voxels3.rs*` | upstream-like scene density and step counts |
| `examples3d_trimesh_parity_test.mbt` | `nsubdivs=1`, low stack (`num=1`, `numy=1`), `steps={20,5,5}` | `MEDIUM`: `nsubdivs=1`, stack (`num=2`, `numy=1`), `steps={24,6,10}`; `HEAVY`: `nsubdivs=2`, stack (`num=2`, `numy=1`), `steps={30,8,16}` | `FULLSCALE`: `nsubdivs=2`, stack (`num=2`, `numy=2`), `steps={45,12,24}` |
| `examples3d_worlds_parity_test.mbt` | reduced `platform/domino/fountain` scales (`steps={20,30,20}`) | `MEDIUM`: `steps={25,60,30}`; `HEAVY`: `steps={30,60,40}` | `FULLSCALE`: denser scenes + longer runs (`steps={60,120,80}`) |

## Notes

- Non-default profiles are intentionally marked with `#skip(...)` and are only run when `--include-skipped` is provided.
- The gate script validates one representative tiered scenario per `trimesh/worlds` file; in `HEAVY` mode it also runs the previously wired heavy-only scenes.
- Full-scale targets remain tracked by BD issue `moon_rapier-restore-fullscale-rapier_full-parity-tests`.

## CI Strategy

- Pull requests:
  - `moon check --frozen`
  - `moon test --frozen`
  - `tools/run_rapier_full_heavy_gate.sh` (default `HEAVY`)
- Nightly:
  - `RAPIER_FULL_PROFILE=medium tools/run_rapier_full_heavy_gate.sh`
- Weekly:
  - `RAPIER_FULL_PROFILE=fullscale tools/run_rapier_full_heavy_gate.sh`

This keeps PR feedback fast while continuously validating stronger scene scales.
