#include <cassert>
#include <cstdio>
#include "../src/pages/rel_page.cpp"
ImuState g_imu;
portMUX_TYPE g_imuMux;
static int successes = 0, failures = 0;
void app_mark_user_activity() {}
AppMode app_mode_snapshot() { return AppMode::RelAngle; }
void app_start_tone(uint16_t hz, uint32_t) { assert(hz == 220); ++failures; }
void app_play_rel_zero_sound() { ++successes; }
static void feed(float deg) {
  test_ms += 8;
  g_imu.ax_g = std::sin(deg / accel_rel::kDeg);
  g_imu.az_g = std::cos(deg / accel_rel::kDeg);
  ui_pages::rel_update_angle_sample();
}
int main() {
  for (int i = 0; i < 82; ++i) feed(10 + (i % 2 ? .1f : -.1f));
  ui_pages::rel_on_tap_zero();
  assert(successes == 1 && failures == 0 && g_imu.ref_valid);
  assert(std::fabs(g_imu.ref_angle - 10) < .005f);
  assert(std::fabs(g_imu.ref_ax_g - std::sin(10 / accel_rel::kDeg)) < .00001f);
  const float old = g_imu.ref_angle;
  test_ms += 51; // Still displayable at 500 ms, but too old for zero capture.
  ui_pages::rel_on_tap_zero();
  assert(successes == 1 && failures == 1 && g_imu.ref_angle == old);
  lv_obj_t button;
  ImuUiData imu; imu.ref_valid = true;
  ui_pages::rel_update(nullptr, nullptr, &button, imu);
  assert(button.text == "Keep still\nHold 3s again");
  for (int i = 0; i < 82; ++i) feed(20);
  ui_pages::rel_on_tap_zero();
  assert(successes == 2 && failures == 1);
  ui_pages::rel_update(nullptr, nullptr, &button, imu);
  assert(button.text == "Reset zero");
  feed(30);
  ui_pages::rel_on_tap_zero();
  assert(successes == 2 && failures == 2);
  assert(std::fabs(g_imu.ref_angle - 20) < .001f);
  puts("PASS REL averaged reference / capture freshness / success-only sound / retry label / motion preserves zero");
}
