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

} // namespace ui_pages
