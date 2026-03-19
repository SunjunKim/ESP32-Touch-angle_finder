#pragma once

#include <lvgl.h>
#include "page_types.h"

namespace ui_pages {

void rel_show(lv_obj_t *incl_container,
              lv_obj_t *rel_angle_big,
              lv_obj_t *primary,
              lv_obj_t *secondary,
              lv_obj_t *rel_zero_btn);

void rel_update(lv_obj_t *rel_angle_big,
                lv_obj_t *debug_label,
                const ImuUiData &imu);

} // namespace ui_pages

