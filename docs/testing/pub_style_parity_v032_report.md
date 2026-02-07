# Pub Style Parity Report (rapier2d/rapier3d v0.32.0 default features)

## Scope

- Reference: `rapier-reference` (`rapier2d` + `rapier3d`, default features).
- Target: mapped public style parity with MoonBit representability constraints documented.

## Audit Commands

- `python3 tools/rapier_pub_audit.py run`
- `python3 tools/rapier_pub_style_audit.py run`

## Public Coverage Result

- rapier2d: `items=1834`, `covered=1834`, `missing=0`
- rapier3d: `items=1956`, `covered=1956`, `missing=0`

## Style Classification Totals

- `exact`: 50
- `const_case_alias`: 159
- `dimension_suffix_alias`: 96
- `keyword_alias`: 10
- `renamed`: 0

## Bucket Breakdown

- control: `exact=3`
- counters: `exact=4`, `keyword_alias=2`
- dynamics: `const_case_alias=59`
- geometry: `const_case_alias=74`, `keyword_alias=4`
- lib: `exact=2`
- math: `exact=2`, `dimension_suffix_alias=30`
- pipeline: `const_case_alias=26`, `keyword_alias=4`
- utils: `exact=39`, `dimension_suffix_alias=66`

## Documented Constraints

1. `keyword_alias`
- Rust leaf names such as `test` and `resume` map to MoonBit-safe identifiers (`passes`, `resume_`) because MoonBit reserves keywords and rejects methods named `test`.

2. `const_case_alias`
- Rapier associated constants (uppercase leaf names) map to MoonBit callable constructors/accessors for bitflags/flags (`collision_events`, `filter_contact_pairs`, `exclude_fixed`, etc.).
- MoonBit does not currently support Rust-like associated constant surface (`Type::CONST`) in this code style.

3. `dimension_suffix_alias`
- Rapier single-name generic symbols are represented by explicit dim-specialized 2D/3D symbols in MoonBit (`*2`, `*3`, `*3DReal`).

## Validation Evidence

- `moon test --frozen -p dynamics` passed.
- `moon test --frozen -p collision` passed.
- `moon test --frozen -p utils` passed.
- `moon test --frozen -p core` passed.

## Conclusion

- Style drift class `renamed` is eliminated (`0`) across scope.
- Remaining aliases are constrained and documented above.
