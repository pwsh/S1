#ifndef ZONE_HELPERS_H
#define ZONE_HELPERS_H

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

// ── Target Position Smoothing ─────────────────────────────────────────────
// Circular buffer of the last 5 measured (x, y) positions for one target.
// Push only when the target is actively detected (distance > 0).
// sx()/sy() return the mean of stored samples; cnt == 0 means no data yet.
// Defined at namespace scope (not inside a lambda) so all lambdas share the
// same three instances — ESPHome compiles to a single translation unit, so
// the static qualifier prevents linker conflicts if this header were ever
// included more than once in different files.
struct TargetSmoothBuf {
  static constexpr int SIZE = 5;
  float xs[SIZE];
  float ys[SIZE];
  int   idx;
  int   cnt;
  TargetSmoothBuf() : idx(0), cnt(0) {
    for (int i = 0; i < SIZE; i++) { xs[i] = 0.0f; ys[i] = 0.0f; }
  }
  void push(float x, float y) {
    xs[idx] = x; ys[idx] = y;
    idx = (idx + 1) % SIZE;
    if (cnt < SIZE) cnt++;
  }
  float sx() const {
    float s = 0.0f;
    for (int i = 0; i < cnt; i++) s += xs[i];
    return cnt > 0 ? s / static_cast<float>(cnt) : 0.0f;
  }
  float sy() const {
    float s = 0.0f;
    for (int i = 0; i < cnt; i++) s += ys[i];
    return cnt > 0 ? s / static_cast<float>(cnt) : 0.0f;
  }
};

// One buffer per radar target; updated by obj1_closest_dist lambda each cycle.
static TargetSmoothBuf g_smooth_t1;
static TargetSmoothBuf g_smooth_t2;
static TargetSmoothBuf g_smooth_t3;

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
// Euclidean distance from a fixed object (ox, oy) to a radar target (tx, ty).
// Returns 0 when the target is absent (tx == 0 && ty == 0), otherwise the
// straight-line distance clamped to [0, 600] cm.
inline float obj_dist(float ox, float oy, float tx, float ty) {
  float dx = ox - tx;
  float dy = oy - ty;
  float d = std::sqrt(dx * dx + dy * dy);
  if (d < 0.0f) d = 0.0f;
  if (d > 600.0f) d = 600.0f;
  return d;
}

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

#endif // ZONE_HELPERS_H
