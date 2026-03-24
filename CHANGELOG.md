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
- 6 text inputs (`zone1_name`–`zone3_name`, `obj1_name`–`obj3_name`, max 32 chars,
  NVS-backed) allow labelling each zone and object from the GUI. Default values are
  "Zone 1"–"Zone 3" and "Object 1"–"Object 3". Note: ESPHome 2024.x removed the
  public `set_name()` API from `EntityBase`; entity display names are therefore
  static (set at compile time). The text inputs serve as user-defined labels visible
  in the device GUI and stored in NVS across reboots.
- 3 closest-target sensors (`obj{1–3}_closest_dist`): distance from each object to the
  nearest active target using 5-sample smoothed positions, rounded to the nearest whole
  cm. Returns unknown when disabled or no target is detected. Replaces the previous 9
  per-target distance sensors.
- 3 halo radius numbers (`obj{1–3}_halo`, 0–600 cm, default 50 cm, NVS-backed): define
  a configurable detection radius around each object.
- 3 halo binary sensors (`obj{1–3}_in_halo`): true when any smoothed target position is
  within the object's halo radius.
- Target position smoothing: `TargetSmoothBuf` added to `zone_helpers.h`; shared buffers
  `g_smooth_t1/t2/t3` store the last 5 active positions per target. Zone presence/
  movement checks and all object calculations use the 5-sample mean. Buffers updated
  once per cycle in the `obj1_closest_dist` lambda.
- `Processing Time` sensor (`processing_time`, 5 s interval): re-runs all 6 zone polygon
  evaluations and 3 object distance checks in a single timed pass and reports elapsed
  microseconds. Uses `volatile` intermediates to prevent dead-code elimination.

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
