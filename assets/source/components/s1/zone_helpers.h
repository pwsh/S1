#pragma once

// [OPT] Shared zone polygon helpers extracted from main.yaml.
// Previously, zone_nz() and zone_in_poly() were copy-pasted verbatim into each
// of the 9 zone template-sensor lambdas (Zone 1-3 × target-count / presence /
// movement). Any fix to the algorithm had to be applied 9 times; a missed copy
// caused a silent divergence. Extracting them here makes each lambda a single
// call site and keeps the algorithm in one place.
//
// Usage in ESPHome lambdas (after adding 'includes: [components/s1/zone_helpers.h]'
// to the esphome: block):
//
//   float xs[8], ys[8];
//   int n = (int)id(zoneN_points_count).state;
//   xs[0] = zone_nz(id(zoneN_p1_x).state); ys[0] = zone_nz(id(zoneN_p1_y).state);
//   ...
//   bool hit = zone_in_poly(px, py, xs, ys, n);

#include <cmath>

// NaN-safe float read.  Returns 0.0f when the ESPHome number has not yet
// received a state (which ESPHome represents as NaN).
inline float zone_nz(float v) {
  return std::isnan(v) ? 0.0f : v;
}

// Ray-casting point-in-polygon test.
// xs, ys : polygon vertex arrays (length >= n)
// n      : number of active vertices (must be >= 3; caller is responsible)
// Returns true when (px, py) lies inside the polygon.
//
// [FIX] Division guard uses std::fabs + y_straddle short-circuit instead of
// the previous '== 0 ? 1e-6f' branch.  This matches the optimised approach
// already used in LD2450::is_in_exclusion_zone() in s1.h.
inline bool zone_in_poly(float px, float py, const float* xs, const float* ys, int n) {
  bool inside = false;
  for (int i = 0, j = n - 1; i < n; j = i++) {
    bool y_straddle = ((ys[i] > py) != (ys[j] > py));
    if (y_straddle) {
      float dy = ys[j] - ys[i];
      float x_cross = (xs[j] - xs[i]) * (py - ys[i]) / (std::fabs(dy) < 1e-6f ? 1e-6f : dy) + xs[i];
      if (px < x_cross) inside = !inside;
    }
  }
  return inside;
}
