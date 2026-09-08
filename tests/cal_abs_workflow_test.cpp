#include <cassert>
#include <cstdio>
// Exercise production wizard and ABS code, with only platform/UI/storage mocked.
#include "../src/pages/cal_page.cpp"
#include "../src/pages/abs_page.cpp"

static AppMode mode = AppMode::RelAngle;
void app_mark_user_activity() {}
AppMode app_mode_snapshot() { return mode; }
void app_start_tone(uint16_t, uint32_t) {}
void app_play_rel_zero_sound() {}

static void enter_cal() {
  mode = AppMode::RelAngle;
  ui_pages::cal_wizard_imu_sync(false);
  mode = AppMode::Calibrate;
  ui_pages::cal_wizard_imu_sync(true);
  assert(ui_pages::cal_ui_for_refresh().awaiting_tap);
}
static void start_cal() {
  lv_event_t tap;
  ui_pages::cal_intro_tap_event(&tap);
  // UI posting must not mutate computation state. Only the IMU consumes it.
  assert(ui_pages::s_calAwaitingTapToStart);
  ui_pages::cal_wizard_imu_sync(true);
  assert(!ui_pages::s_calAwaitingTapToStart);
  assert(!ui_pages::cal_ui_for_refresh().awaiting_tap);
}
static void finish_cal() {
  const float faces[6][3] = {{0,0,-1},{0,0,1},{-1,0,0},{1,0,0},{0,1,0},{0,-1,0}};
  for (unsigned step = 0; step < 6; ++step) {
    for (int i = 0; i < 700 && ui_pages::s_calStep == step; ++i) {
      test_ms += 8;
      ui_pages::cal_wizard_imu_sample(faces[step][0], faces[step][1], faces[step][2]);
    }
    assert(ui_pages::s_calStep == step + 1);
  }
}
int main() {
  enter_cal(); start_cal();
  test_ms += 8;
  ui_pages::cal_wizard_imu_sample(0,0,-1);
  assert(ui_pages::s_calStillStartMs != 0);
  enter_cal();
  assert(ui_pages::s_calStep == 0 && ui_pages::s_calSumCount == 0);
  assert(ui_pages::s_calStillStartMs == 0);
  // Pending start discarded on exit; it must not start a subsequent session.
  lv_event_t tap;
  ui_pages::cal_intro_tap_event(&tap);
  enter_cal(); assert(ui_pages::s_calAwaitingTapToStart);
  start_cal();
  ui_pages::s_accBias[0] = .123f;
  ui_pages::s_accCalValid = true;
  Preferences::fail_write = true;
  finish_cal();
  auto failed = ui_pages::cal_ui_for_refresh();
  assert(failed.save_failed && !failed.done_saved);
  assert(ui_pages::s_accBias[0] == .123f); // Preserve active calibration on failure.
  enter_cal(); start_cal();
  Preferences::fail_write = false;
  finish_cal();
  auto saved = ui_pages::cal_ui_for_refresh();
  assert(saved.done_saved && !saved.save_failed);
  assert(std::fabs(ui_pages::s_accBias[0]) < 1e-6f);
  assert(ui_pages::cal_load_accel_prefs());

  puts("PASS CAL IMU-owned reset/start/cancel / save failure and reload");
}
