#pragma once

#include <lvgl.h>
#include "page_types.h"

namespace ui_pages {

void abs_show(lv_obj_t *incl_container,
              lv_obj_t *rel_angle_big,
              lv_obj_t *primary,
              lv_obj_t *secondary,
              lv_obj_t *rel_zero_btn);

void abs_update(lv_obj_t *primary,
                lv_obj_t *secondary,
                lv_obj_t *incl_container,
                lv_obj_t *incl_bubble,
                const ImuUiData &imu);

// Expects device-frame accelerometer (g), e.g. after imu_calibrated_reading / cal_apply_accel.
void abs_accel_to_pitch_roll(float ax, float ay, float az, float *pitch_deg, float *roll_deg);

} // namespace ui_pages

