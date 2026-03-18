#pragma once

#include "esphome.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/number/number.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include <cmath>

namespace esphome {
namespace s1 {

class LD2450 : public Component, public uart::UARTDevice {
 public:
  LD2450();

  void set_detection_range(number::Number *range) { detection_range = range; }
  void set_flip_y(switch_::Switch *sw) { flip_y = sw; }
  void set_bluetooth_state_sensor(text_sensor::TextSensor *ts) { bluetooth_state_ = ts; }

  void set_target1_state_sensor(text_sensor::TextSensor *ts) { target1_state = ts; }
  void set_target2_state_sensor(text_sensor::TextSensor *ts) { target2_state = ts; }
  void set_target3_state_sensor(text_sensor::TextSensor *ts) { target3_state = ts; }

  void set_exclusion_zone_points_count(number::Number *count) { exclusion_zone_points_count = count; }
  void set_exclusion_zone_p1_x(number::Number *p) { exclusion_zone_points[0][0] = p; }
  void set_exclusion_zone_p1_y(number::Number *p) { exclusion_zone_points[0][1] = p; }
  void set_exclusion_zone_p2_x(number::Number *p) { exclusion_zone_points[1][0] = p; }
  void set_exclusion_zone_p2_y(number::Number *p) { exclusion_zone_points[1][1] = p; }
  void set_exclusion_zone_p3_x(number::Number *p) { exclusion_zone_points[2][0] = p; }
  void set_exclusion_zone_p3_y(number::Number *p) { exclusion_zone_points[2][1] = p; }
  void set_exclusion_zone_p4_x(number::Number *p) { exclusion_zone_points[3][0] = p; }
  void set_exclusion_zone_p4_y(number::Number *p) { exclusion_zone_points[3][1] = p; }
  void set_exclusion_zone_p5_x(number::Number *p) { exclusion_zone_points[4][0] = p; }
  void set_exclusion_zone_p5_y(number::Number *p) { exclusion_zone_points[4][1] = p; }
  void set_exclusion_zone_p6_x(number::Number *p) { exclusion_zone_points[5][0] = p; }
  void set_exclusion_zone_p6_y(number::Number *p) { exclusion_zone_points[5][1] = p; }
  void set_exclusion_zone_p7_x(number::Number *p) { exclusion_zone_points[6][0] = p; }
  void set_exclusion_zone_p7_y(number::Number *p) { exclusion_zone_points[6][1] = p; }
  void set_exclusion_zone_p8_x(number::Number *p) { exclusion_zone_points[7][0] = p; }
  void set_exclusion_zone_p8_y(number::Number *p) { exclusion_zone_points[7][1] = p; }

  void set_gate_radius_cm(number::Number *n) { gate_radius_cm = n; }
  void set_stationary_speed_thresh(number::Number *n) { stationary_speed_thresh = n; }
  void set_stationary_time_s(number::Number *n) { stationary_time_s = n; }
  void set_dropout_hold_m(number::Number *n) { dropout_hold_m = n; }
  void set_holding_enabled(switch_::Switch *sw) { holding_enabled = sw; }

  void set_single_target_tracking();
  void set_multi_target_tracking();
  void restart_module();
  void restore_factory_settings();
  void turn_bluetooth_on();
  void turn_bluetooth_off();

  void setup() override;
  void loop() override;

 protected:
  void handle_ack_data(const uint8_t *buffer, int len);
  void parse_frame(const uint8_t *b);
  bool is_in_exclusion_zone(float x, float y);

  struct Track {
    bool   has_fix{false};
    float  x{0}, y{0}, angle{0}, speed{0}, dist{0};
    bool   stationary{false};
    bool   held{false};
    uint32_t last_update_ms{0};
    uint32_t stationary_since_ms{0};
    uint32_t held_since_ms{0};
  } tracks_[3];

  // [OPT] Tuning values bundled into a struct so parse_frame can compute them
  // once per tick and pass them to update_track_, replacing 3x redundant nz_()
  // calls (one per target slot) that previously occurred inside update_track_.
  struct TrackParams {
    float    gate_cm;
    float    v_thr;
    uint32_t stat_s;
    uint32_t hold_min;
    bool     hold_perm;
    float    range_cm;
  };

  void publish_zero_(int t);
  void publish_track_(int t, const Track &tr);
  void publish_state_text_(int t, const char *state);
  // [OPT] Accepts pre-computed TrackParams instead of calling nz_() internally.
  void update_track_(int t, bool meas_valid,
                     float x, float y, float angle, float speed, float dist,
                     uint32_t now, const TrackParams &p);

  static inline float nz_(number::Number *ptr, float def) {
    if (!ptr) return def;
    float v = ptr->state;
    return std::isnan(v) ? def : v;
  }

 public:
  // [OPT] All 15 sensor pointers stored in a 2D member array sensors_[target][field]
  // (fields: 0=x, 1=y, 2=angle, 3=speed, 4=dist). The named aliases below are
  // assigned from this array in the constructor so sensor.py registration is unaffected.
  sensor::Sensor *sensors_[3][5];

  sensor::Sensor *target1_x, *target1_y, *target1_angle, *target1_speed, *target1_distance;
  sensor::Sensor *target2_x, *target2_y, *target2_angle, *target2_speed, *target2_distance;
  sensor::Sensor *target3_x, *target3_y, *target3_angle, *target3_speed, *target3_distance;

  text_sensor::TextSensor *target1_state;
  text_sensor::TextSensor *target2_state{nullptr};
  text_sensor::TextSensor *target3_state{nullptr};

  number::Number *detection_range{nullptr};
  switch_::Switch *flip_y{nullptr};
  text_sensor::TextSensor *bluetooth_state_{nullptr};

  number::Number *exclusion_zone_points_count{nullptr};
  number::Number *exclusion_zone_points[8][2]{};

  number::Number *gate_radius_cm{nullptr};
  number::Number *stationary_speed_thresh{nullptr};
  number::Number *stationary_time_s{nullptr};
  number::Number *dropout_hold_m{nullptr};
  switch_::Switch *holding_enabled{nullptr};

 private:
  void send_command(const uint8_t *command, size_t length);
  uint16_t twoByteToUint(uint8_t firstByte, uint8_t secondByte);
};

inline LD2450::LD2450()
    : Component(), uart::UARTDevice() {
  // [OPT] Allocate all 15 sensors via sensors_[3][5] in a single loop instead of
  // 15 individual assignments. Named aliases are set from the array so external
  // code (sensor.py registration via getattr) continues to work unchanged.
  for (int t = 0; t < 3; ++t)
    for (int f = 0; f < 5; ++f)
      sensors_[t][f] = new sensor::Sensor();

  target1_x = sensors_[0][0]; target1_y = sensors_[0][1];
  target1_angle = sensors_[0][2]; target1_speed = sensors_[0][3]; target1_distance = sensors_[0][4];

  target2_x = sensors_[1][0]; target2_y = sensors_[1][1];
  target2_angle = sensors_[1][2]; target2_speed = sensors_[1][3]; target2_distance = sensors_[1][4];

  target3_x = sensors_[2][0]; target3_y = sensors_[2][1];
  target3_angle = sensors_[2][2]; target3_speed = sensors_[2][3]; target3_distance = sensors_[2][4];

  bluetooth_state_ = new text_sensor::TextSensor();
  target1_state    = new text_sensor::TextSensor();
  target2_state    = new text_sensor::TextSensor();
  target3_state    = new text_sensor::TextSensor();
}

inline void LD2450::set_single_target_tracking() {
  static const uint8_t CMD[] = {0x02,0x00,0x80,0x00,0x04,0x03,0x02,0x01};
  send_command(CMD, sizeof(CMD));
}

inline void LD2450::set_multi_target_tracking() {
  static const uint8_t CMD[] = {0x02,0x00,0x90,0x00,0x04,0x03,0x02,0x01};
  send_command(CMD, sizeof(CMD));
}

inline void LD2450::restart_module() {
  static const uint8_t CMD[] = {0x02,0x00,0xA3,0x00,0x04,0x03,0x02,0x01};
  send_command(CMD, sizeof(CMD));
}

inline void LD2450::restore_factory_settings() {
  static const uint8_t CMD[] = {0x02,0x00,0xA2,0x00,0x04,0x03,0x02,0x01};
  send_command(CMD, sizeof(CMD));
}

inline void LD2450::turn_bluetooth_on() {
  static const uint8_t CMD[] = {0x04, 0x00, 0xA4, 0x00, 0x01, 0x00, 0x04, 0x03, 0x02, 0x01};
  send_command(CMD, sizeof(CMD));
  if (bluetooth_state_) bluetooth_state_->publish_state("On");
}

inline void LD2450::turn_bluetooth_off() {
  static const uint8_t CMD[] = {0x04, 0x00, 0xA4, 0x00, 0x00, 0x00, 0x04, 0x03,0x02,0x01};
  send_command(CMD, sizeof(CMD));
  if (bluetooth_state_) bluetooth_state_->publish_state("Off");
}

inline void LD2450::setup() {
}

inline void LD2450::loop() {
  static const int MAX = 160;
  static uint8_t buf[MAX];
  static int pos = 0;

  while (available()) {
    int c = read();
    if (c < 0) break;

    // [FIX] On buffer overflow, scan backward for the data-frame preamble
    // (0xAA FF 03 00) and compact it to the front of the buffer instead of
    // blindly discarding everything. This prevents a partial frame that straddles
    // the overflow boundary from being processed as garbage on the next tick.
    if (pos >= MAX) {
      int resync = -1;
      for (int i = 1; i <= MAX - 4; ++i) {
        if (buf[i] == 0xAA && buf[i+1] == 0xFF && buf[i+2] == 0x03 && buf[i+3] == 0x00) {
          resync = i;
          break;
        }
      }
      if (resync >= 0) {
        pos = MAX - resync;
        memmove(buf, buf + resync, pos);
      } else {
        pos = 0;
      }
    }
    buf[pos++] = static_cast<uint8_t>(c);

    if (pos >= 10 && buf[pos-4]==0x04 && buf[pos-3]==0x03 && buf[pos-2]==0x02 && buf[pos-1]==0x01) {
      handle_ack_data(buf, pos);
      pos = 0;
      continue;
    }

    if (pos >= 30 && buf[0]==0xAA && buf[1]==0xFF && buf[2]==0x03 && buf[3]==0x00 && buf[28]==0x55 && buf[29]==0xCC) {
      parse_frame(buf);
      pos = 0;
    }
  }
}

inline void LD2450::handle_ack_data(const uint8_t *buffer, int len) {
  if (len < 10) return;
  if (buffer[0]!=0xFD||buffer[1]!=0xFC||buffer[2]!=0xFB||buffer[3]!=0xFA) return;
  if (buffer[7]!=0x01) return;
  if (twoByteToUint(buffer[8],buffer[9])!=0x00) return;
}

inline bool LD2450::is_in_exclusion_zone(float x, float y) {
  if (!exclusion_zone_points_count) return false;
  int n = (int) exclusion_zone_points_count->state;
  if (n < 3) return false;
  if (n > 8) n = 8;

  static int   cached_n = -1;
  static float xs[8], ys[8], inv_dy[8];
  static float minx = 0, maxx = 0, miny = 0, maxy = 0;

  bool changed = (cached_n != n);
  float txs[8], tys[8];

  for (int i = 0; i < n; ++i) {
    txs[i] = exclusion_zone_points[i][0] ? exclusion_zone_points[i][0]->state : 0.0f;
    tys[i] = exclusion_zone_points[i][1] ? exclusion_zone_points[i][1]->state : 0.0f;
    if (!changed && (txs[i] != xs[i] || tys[i] != ys[i])) changed = true;
  }

  if (changed) {
    cached_n = n;
    minx = maxx = txs[0];
    miny = maxy = tys[0];
    for (int i = 0; i < n; ++i) {
      xs[i] = txs[i];
      ys[i] = tys[i];
      if (xs[i] < minx) minx = xs[i];
      if (xs[i] > maxx) maxx = xs[i];
      if (ys[i] < miny) miny = ys[i];
      if (ys[i] > maxy) maxy = ys[i];
    }
    for (int i = 0, j = n - 1; i < n; j = i++) {
      float dy = ys[j] - ys[i];
      inv_dy[i] = (std::fabs(dy) < 1e-6f) ? 0.0f : 1.0f / dy;
    }
  }

  if (x < minx || x > maxx || y < miny || y > maxy) return false;

  bool inside = false;
  for (int i = 0, j = n - 1; i < n; j = i++) {
    bool y_straddle = ((ys[i] > y) != (ys[j] > y));
    if (y_straddle) {
      float x_cross = (xs[j] - xs[i]) * (y - ys[i]) * inv_dy[i] + xs[i];
      if (x < x_cross) inside = !inside;
    }
  }
  return inside;
}

inline void LD2450::publish_zero_(int t) {
  // [OPT] Replaced 5 locally-rebuilt pointer arrays with direct sensors_[t] indexing.
  for (int f = 0; f < 5; ++f)
    if (sensors_[t][f]) sensors_[t][f]->publish_state(0.0f);
  publish_state_text_(t, "No target");
}

inline void LD2450::publish_track_(int t, const Track &tr) {
  // [OPT] Replaced 5 locally-rebuilt pointer arrays with direct sensors_[t] indexing.
  if (sensors_[t][0]) sensors_[t][0]->publish_state(tr.x);
  if (sensors_[t][1]) sensors_[t][1]->publish_state(tr.y);
  if (sensors_[t][2]) sensors_[t][2]->publish_state(tr.angle);
  if (sensors_[t][3]) sensors_[t][3]->publish_state(tr.speed);
  if (sensors_[t][4]) sensors_[t][4]->publish_state(tr.dist);
  if (tr.held)                publish_state_text_(t, "Holding");
  else if (tr.stationary)     publish_state_text_(t, "Stationary");
  else                        publish_state_text_(t, "Moving");
}

inline void LD2450::publish_state_text_(int t, const char *state) {
  text_sensor::TextSensor* sts[3] = {target1_state, target2_state, target3_state};
  if (t >= 0 && t < 3 && sts[t]) sts[t]->publish_state(state);
}

inline void LD2450::update_track_(int t, bool meas_valid,
                                  float x, float y, float angle, float speed, float dist,
                                  uint32_t now, const TrackParams &p) {
  // [OPT] All tuning values received via TrackParams; nz_() calls removed from here.
  // Previously each of the 3 per-frame update_track_ calls re-evaluated 5 nz_()
  // reads; now they are computed once in parse_frame and passed in.

  static constexpr float FOV_DEGREES    = 120.0f;
  static constexpr float EDGE_MARGIN_CM = 30.0f;

  Track &tr = tracks_[t];

  if (meas_valid) {
    if (tr.held && std::fabs(speed) <= p.v_thr) {
      tr.last_update_ms = now;
      publish_track_(t, tr);
      return;
    }

    if (std::fabs(speed) > p.v_thr) {
      tr.held = false;
      tr.stationary = false;
      tr.stationary_since_ms = 0;
      tr.held_since_ms = 0;
    }

    if (!tr.held) {
      bool accept = !tr.has_fix;
      if (tr.has_fix) {
        float dx = x - tr.x, dy = y - tr.y;
        float d  = std::sqrt(dx*dx + dy*dy);
        accept = (d <= p.gate_cm) || (tr.stationary && d <= p.gate_cm * 1.5f);
      }
      if (!accept) tr = Track{};
    }

    if (!tr.held) {
      tr.x = x; tr.y = y; tr.angle = angle; tr.speed = speed; tr.dist = dist;
    }
    tr.has_fix = true;
    tr.last_update_ms = now;

    if (std::fabs(speed) <= p.v_thr) {
      if (!tr.stationary) {
        tr.stationary = true;
        tr.stationary_since_ms = now;
      }
      if (!tr.held && p.hold_perm && (now - tr.stationary_since_ms) >= p.stat_s * 1000UL) {
        tr.held = true;
        tr.held_since_ms = now;
      }
    }

    publish_track_(t, tr);
    return;

  } else {
    if (!p.hold_perm) {
      tr = Track{};
      publish_zero_(t);
      return;
    }

    if (tr.has_fix) {
      const bool near_max_range = (p.range_cm > 0.0f) && (tr.dist >= (p.range_cm - EDGE_MARGIN_CM));

      bool near_fov_edge_or_outside = false;
      const float half_fov_rad = (FOV_DEGREES * 0.5f) * (M_PI / 180.0f);
      const float abs_ang_rad  = std::fabs(tr.angle) * (M_PI / 180.0f);
      if (abs_ang_rad > half_fov_rad) {
        near_fov_edge_or_outside = true;
      } else {
        const float dtheta_to_edge = half_fov_rad - abs_ang_rad;
        const float lateral_cm     = tr.dist * dtheta_to_edge;
        near_fov_edge_or_outside   = (lateral_cm <= EDGE_MARGIN_CM);
      }
      if (near_max_range || near_fov_edge_or_outside) {
        tr = Track{};
        publish_zero_(t);
        return;
      }
    }

    if (tr.has_fix && tr.held) {
      if ((now - tr.last_update_ms) <= p.hold_min * 60000UL) {
        publish_track_(t, tr);
        return;
      }
    }

    tr = Track{};
    publish_zero_(t);
  }
}

inline void LD2450::parse_frame(const uint8_t *b) {
  // [OPT] Tuning parameters computed once here and passed to update_track_ via
  // TrackParams, replacing 3 separate per-target nz_() evaluations (15 pointer
  // reads per frame reduced to 5).
  TrackParams p;
  p.range_cm  = nz_(detection_range, 600.0f);
  // [FIX] Removed redundant std::isnan() guard that preceded this check.
  // nz_() already substitutes the default value for NaN, so isnan(range_cm)
  // can never be true here. Only the <= 0 guard is needed.
  if (p.range_cm <= 0.0f) p.range_cm = 600.0f;
  p.gate_cm   = nz_(gate_radius_cm,          120.0f);
  p.v_thr     = nz_(stationary_speed_thresh,   45.0f);
  p.stat_s    = (uint32_t) std::lround(nz_(stationary_time_s,   5.0f));
  p.hold_min  = (uint32_t) std::lround(nz_(dropout_hold_m,     15.0f));
  p.hold_perm = (holding_enabled ? holding_enabled->state : true);

  bool do_flip = (flip_y ? flip_y->state : false);
  uint32_t now = millis();

  for (int t = 0; t < 3; ++t) {
    int base = 4 + t*8;
    auto raw16 = [&](int off){ return uint16_t(b[base+off])|(uint16_t(b[base+off+1])<<8); };
    auto to_s  = [&](uint16_t v){ return (v&0x8000)?int16_t(v&0x7FFF):-int16_t(v&0x7FFF); };

    float x_cm  = float(to_s(raw16(0)))/10.0f;
    float y_cm  = float(to_s(raw16(2)))/10.0f;
    float speed = float(to_s(raw16(4)));
    float dist  = std::sqrt(x_cm*x_cm + y_cm*y_cm);
    float angle = (dist>0.0f)?(std::atan2(x_cm, y_cm) * 180.0f / M_PI):0.0f;

    if (do_flip) { x_cm = -x_cm; angle = -angle; }

    bool valid = (dist > 0.0f) && (dist <= p.range_cm) && !is_in_exclusion_zone(x_cm, y_cm);
    update_track_(t, valid, x_cm, y_cm, angle, speed, dist, now, p);
  }
}

inline void LD2450::send_command(const uint8_t *command, size_t length) {
  static const uint8_t enable_cmd[]={0xFD,0xFC,0xFB,0xFA,0x04,0x00,0xFF,0x00,0x01,0x00,0x04,0x03,0x02,0x01};
  write_array(enable_cmd, sizeof(enable_cmd));

  static const uint8_t header[] = {0xFD, 0xFC, 0xFB, 0xFA};
  // [FIX] Replaced VLA (uint8_t full_cmd[sizeof(header)+length]) with a fixed-size
  // buffer. VLAs are non-standard in C++14/17 and risky on stack-limited embedded
  // targets. The largest observed command payload is 10 bytes; 48 bytes is ample margin.
  uint8_t full_cmd[4 + 48];
  memcpy(full_cmd, header, sizeof(header));
  memcpy(full_cmd + sizeof(header), command, length);
  write_array(full_cmd, sizeof(header) + length);

  static const uint8_t disable_cmd[]={0xFD,0xFC,0xFB,0xFA,0x02,0x00,0xFE,0x00,0x04,0x03,0x02,0x01};
  write_array(disable_cmd, sizeof(disable_cmd));
}

inline uint16_t LD2450::twoByteToUint(uint8_t firstByte, uint8_t secondByte) {
  return ((uint16_t)secondByte<<8)|firstByte;
}

}
}
