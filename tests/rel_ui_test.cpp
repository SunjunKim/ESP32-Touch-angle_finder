#include <cassert>
#include <cstdio>
#include "../src/pages/rel_page.cpp"
ImuState g_imu;
portMUX_TYPE g_imuMux;
static int successes = 0, failures = 0;
void app_mark_user_activity() {}
static AppMode mode = AppMode::RelAngle;
AppMode app_mode_snapshot() { return mode; }
void app_start_tone(uint16_t hz, uint32_t) { assert(hz == 220); ++failures; }
void app_play_rel_zero_sound() { ++successes; }
static void feed(float deg) {
  test_ms += 8;
  g_imu.ax_g = std::sin(deg / accel_rel::kDeg);
  g_imu.az_g = std::cos(deg / accel_rel::kDeg);
  ui_pages::rel_update_angle_sample();
}
static void event(int code) {
  lv_event_t e; e.code = code;
  ui_pages::rel_zero_btn_event_cb(&e);
}
static void refresh(lv_obj_t &angle) {
  ImuUiData imu;
  imu.ref_valid = g_imu.ref_valid;
  imu.sample_valid = g_imu.sample_valid && uint32_t(test_ms - g_imu.sample_ms) < 500;
  imu.rel_angle_deg = g_imu.rel_angle_deg;
  ui_pages::rel_update(&angle, nullptr, nullptr, imu);
}
static void countdown_tests() {
  lv_obj_t angle;
  // Initial set and re-zero follow the same 1-second hold -> angle sequence.
  g_imu = ImuState{};
  for (int attempt = 0; attempt < 2; ++attempt) {
    for (int i = 0; i < 82; ++i) feed(20);
    assert(g_imu.ref_valid == (attempt != 0));
    const int before = successes;
    event(LV_EVENT_PRESSED);
    const uint32_t started = test_ms;
    for (uint32_t elapsed : {0u, 500u, 999u, 1000u}) {
      while (uint32_t(test_ms - started) + 8 <= elapsed) feed(20);
      test_ms = started + elapsed;
      event(LV_EVENT_PRESSING);
      refresh(angle);
      if (elapsed < 1000) {
        const char *expected = "Hold 1s";
        assert(angle.text == expected && angle.font == 28 && angle.color == LV_PALETTE_ORANGE);
        assert(successes == before);
      } else {
        assert(angle.text == "0.0°" && angle.font == 48);
        assert(successes == before + 1 && g_imu.ref_valid);
      }
    }
    event(LV_EVENT_PRESSING); // Keeping the button down must not capture twice.
    assert(successes == before + 1);
    event(LV_EVENT_RELEASED);
  }
  for (int stop : {LV_EVENT_RELEASED, LV_EVENT_PRESS_LOST}) {
    for (int i = 0; i < 82; ++i) feed(20);
    const int before = successes;
    event(LV_EVENT_PRESSED); refresh(angle); assert(angle.text == "Hold 1s");
    event(stop); refresh(angle); assert(angle.text == "0.0°");
    test_ms += 1000; event(LV_EVENT_PRESSING);
    assert(successes == before);
  }
  event(LV_EVENT_PRESSED);
  ui_pages::rel_cancel_zero_hold(); // Mode-transition hook in ui_refresh.
  refresh(angle); assert(angle.text == "--");
  mode = AppMode::Calibrate;
  event(LV_EVENT_PRESSED); assert(!ui_pages::s_zero_hold_active);
  mode = AppMode::RelAngle;
  const int previous_failures = failures;
  event(LV_EVENT_PRESSED); test_ms += 1000; event(LV_EVENT_PRESSING);
  refresh(angle);
  assert(angle.text == "--" && failures == previous_failures + 1);
  assert(g_imu.ref_valid); // Failed reset ends the warning but keeps the old zero.
  // Unsigned elapsed time also counts down across millis() wraparound.
  test_ms = UINT32_MAX - 500;
  event(LV_EVENT_PRESSED); test_ms += 600; refresh(angle);
  assert(angle.text == "Hold 1s");
  event(LV_EVENT_RELEASED);
  puts("PASS zero countdown initial/reset / boundaries / cancel / one-shot / failure / clock wrap / fonts");
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
  assert(button.text == "Keep still\nHold 1s again");
  for (int i = 0; i < 82; ++i) feed(20);
  ui_pages::rel_on_tap_zero();
  assert(successes == 2 && failures == 1);
  ui_pages::rel_update(nullptr, nullptr, &button, imu);
  assert(button.text == "Reset zero");
  feed(30);
  ui_pages::rel_on_tap_zero();
  assert(successes == 2 && failures == 2);
  assert(std::fabs(g_imu.ref_angle - 20) < .001f);
  countdown_tests();
  puts("PASS REL averaged reference / capture freshness / success-only sound / retry label / motion preserves zero");
}
