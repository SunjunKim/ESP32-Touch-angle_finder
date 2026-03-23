#pragma once

#include <lvgl.h>
#include "page_types.h"

namespace ui_pages {

// Call when (re)entering CAL so label caches match on-screen text after other pages
// used the same lv_label_t widgets (abs_page has its own caches).
void cal_invalidate_label_caches();

void cal_hide_edges();

void cal_show(lv_obj_t *incl_container,
              lv_obj_t *rel_angle_big,
              lv_obj_t *primary,
              lv_obj_t *secondary,
              lv_obj_t *debug,
              lv_obj_t *rel_zero_btn);

void cal_update(lv_obj_t *primary,
                lv_obj_t *secondary,
                lv_obj_t *debug,
                const CalUiData &cal);

// ---- Accel calibration (prefs + 6-face wizard, IMU task / loop integration) ----
// Raw → bias + scale + mount rotation → device-frame g vector + Euler-style angles.
struct ImuCalibratedSample {
  float ax_g = 0.0f;
  float ay_g = 0.0f;
  float az_g = 0.0f;
  float pitch_deg = 0.0f;
  float roll_deg = 0.0f;
  float tilt_deg = 0.0f;
};

bool cal_load_accel_prefs();
void cal_apply_accel(float &ax, float &ay, float &az);
bool cal_accel_cal_valid();
// Single entry point: applies saved calibration unless apply_saved_calibration is false
// (use false during the CAL wizard so live samples stay raw).
bool imu_calibrated_reading(float raw_ax,
                            float raw_ay,
                            float raw_az,
                            ImuCalibratedSample *out,
                            bool apply_saved_calibration = true);

void cal_wizard_reset();
void cal_wizard_imu_sample(float ax, float ay, float az);
void cal_flush_pending_beep();
void cal_poll_delayed_beep();
CalUiData cal_ui_for_refresh();

void cal_intro_tap_event(lv_event_t *e);
lv_obj_t *cal_create_intro_tap_layer(lv_obj_t *parent);

} // namespace ui_pages
