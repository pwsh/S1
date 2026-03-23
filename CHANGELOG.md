# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).

---

## [1.2.9] - 2026-03-18

### Added
- `components/s1/zone_helpers.h`: shared inline helpers (`zone_nz`, `zone_in_poly`)
  extracted from the YAML lambda blocks. The ray-casting point-in-polygon algorithm
  and NaN-safe float read now live in one place instead of being copy-pasted into
  each of the 9 zone template-sensor lambdas.
- `Zone Sensor Interval` number entity (`id: zone_update_interval`, 0.1–60 s, default
  1.0 s, NVS-backed): controls how often the 6 zone binary sensor lambdas re-evaluate
  the polygon test. Exposed in the GUI under the config entity category.
- Object placement: 6 number entities (`obj1_x/y`, `obj2_x/y`, `obj3_x/y`, ±600 / 0–600 cm,
  NVS-backed, config category) define up to 3 fixed points on the radar plane.
- 3 enable switches (`obj1_enabled`, `obj2_enabled`, `obj3_enabled`, default OFF) mark
  whether each object is defined. Distance sensors return unknown when the switch is off.
- 9 template sensors (`obj{1–3}_t{1–3}_dist`) report the Euclidean distance from each
  fixed object to each of the 3 radar targets, rounded to the nearest whole centimetre
  and clamped to [0, 600] cm. Return unknown (`NAN`) when the object is disabled or the
  target is not detected. Uses `obj_dist()` added to `zone_helpers.h`.

### Changed
- All 9 zone template-sensor lambdas (Zone 1–3 × target-count / presence / movement)
  now call `zone_in_poly()` and `zone_nz()` from `zone_helpers.h` instead of
  defining identical local lambdas. No behavioral change.
- Zone binary sensor polygon evaluation (Presence and Movement for Zones 1–3) is now
  throttled by a software timer inside each lambda using the new `zone_update_interval`
  number entity. `update_interval` is not valid on template binary sensors; the
  software throttle achieves the same effect while keeping the presence-delay logic
  running on every tick. Default is 1.0 s; adjustable from 0.1 s to 60 s in the GUI.
- `esphome.version` bumped to `v1.2.9`.

### Removed
- `flip_y` (internal template switch) — the user-facing `flip_y_switch` now acts
  as the sole source of truth for the Y-axis flip state and is passed directly to
  the `s1` component.
- `holding_switch_internal` (internal template switch) — replaced by passing
  `holding_switch` directly to the `s1` component's `holding_enabled` parameter.
- `flip_y_state` global variable — NVS persistence is handled by
  `flip_y_switch`'s `restore_mode: RESTORE_DEFAULT_OFF`.
- `holding_enabled_bool` global variable — NVS persistence is handled by
  `holding_switch`'s `restore_mode: RESTORE_DEFAULT_ON`.
- `on_boot` synchronization blocks for Holding Engine and Flip Y Axis — no longer
  needed now that a single switch per feature restores its own state.

### Fixed
- Zone `in_poly` division guard now uses `std::fabs` + `y_straddle` short-circuit
  (matching the optimised approach already in `LD2450::is_in_exclusion_zone` in
  `s1.h`) instead of the previous `(dy == 0) ? 1e-6f : dy` branch.

---

## [1.2.8] - prior release

- Radar Control Refactor & State Persistence Overhaul
- Holding Engine Switch, Edge Logic & Extended Stationary Time
