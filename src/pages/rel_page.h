#pragma once

#include <lvgl.h>
#include "page_types.h"

namespace ui_pages {

const lv_font_t *rel_angle_font();

lv_obj_t *rel_create_angle_label(lv_obj_t *parent);
lv_obj_t *rel_create_zero_button(lv_obj_t *parent);

void rel_capture_reference_now();
// Uses g_imu.ax_g / ay_g / az_g (calibrated device frame) under the same mutex as ref_*.
void rel_update_angle_sample();
void rel_on_tap_zero();

void rel_show(lv_obj_t *incl_container,
              lv_obj_t *rel_angle_big,
              lv_obj_t *primary,
              lv_obj_t *secondary,
              lv_obj_t *rel_zero_btn);

void rel_update(lv_obj_t *rel_angle_big,
                lv_obj_t *debug_label,
                lv_obj_t *rel_zero_btn,
                const ImuUiData &imu);

} // namespace ui_pages
