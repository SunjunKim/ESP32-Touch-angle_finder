#include "abs_page.h"

#include "imu_math.h"
#include <Arduino.h>
#include <climits>
#include <cstring>
#include <cstdio>
#include <math.h>

namespace ui_pages {
namespace {

static char s_last_line1[64] = {0};
static char s_last_line2[64] = {0};
static int s_last_bubble_x = INT32_MIN;
static int s_last_bubble_y = INT32_MIN;

static inline float clampf(float v, float lo, float hi)
{
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static void set_label_if_changed(lv_obj_t *label, char *cache, size_t cache_len, const char *next)
{
  if (!label || !next) return;
  if (strncmp(cache, next, cache_len) == 0) return;
  strncpy(cache, next, cache_len - 1);
  cache[cache_len - 1] = '\0';
  lv_label_set_text(label, cache);
}

static void update_inclinometer(lv_obj_t *incl_container, lv_obj_t *incl_bubble, float pitch_deg, float roll_deg)
{
  if (!incl_container || !incl_bubble) return;

  const float max_deg = 45.0f;
  const float px_range = 40.0f;

  const float dx = clampf(roll_deg / max_deg, -1.0f, 1.0f) * px_range;
  const float dy = clampf(pitch_deg / max_deg, -1.0f, 1.0f) * px_range * -1;

  const float cx = lv_obj_get_width(incl_container) / 2.0f;
  const float cy = lv_obj_get_height(incl_container) / 2.0f;
  const float bubble_w = (float)lv_obj_get_width(incl_bubble);
  const float bubble_h = (float)lv_obj_get_height(incl_bubble);
  const int bx = (int)lroundf(cx + dx - bubble_w / 2.0f);
  const int by = (int)lroundf(cy + dy - bubble_h / 2.0f);
  if (bx != s_last_bubble_x || by != s_last_bubble_y) {
    s_last_bubble_x = bx;
    s_last_bubble_y = by;
    lv_obj_set_pos(incl_bubble, bx, by);
  }
}

} // namespace

void abs_show(lv_obj_t *incl_container,
              lv_obj_t *rel_angle_big,
              lv_obj_t *primary,
              lv_obj_t *secondary,
              lv_obj_t *rel_zero_btn)
{
  if (incl_container) lv_obj_clear_flag(incl_container, LV_OBJ_FLAG_HIDDEN);
  if (rel_angle_big) lv_obj_add_flag(rel_angle_big, LV_OBJ_FLAG_HIDDEN);
  if (primary) {
    lv_obj_clear_flag(primary, LV_OBJ_FLAG_HIDDEN);
    lv_label_set_long_mode(primary, LV_LABEL_LONG_CLIP);
    lv_obj_align(primary, LV_ALIGN_CENTER, 0, 60);
  }
  if (secondary) {
    lv_obj_clear_flag(secondary, LV_OBJ_FLAG_HIDDEN);
    lv_label_set_long_mode(secondary, LV_LABEL_LONG_CLIP);
    lv_obj_align(secondary, LV_ALIGN_CENTER, 0, 95);
  }
  if (rel_zero_btn) lv_obj_add_flag(rel_zero_btn, LV_OBJ_FLAG_HIDDEN);
}

void abs_update(lv_obj_t *primary,
                lv_obj_t *secondary,
                lv_obj_t *incl_container,
                lv_obj_t *incl_bubble,
                const ImuUiData &imu)
{
  if (!primary || !secondary) return;

  char line1[64];
  char line2[64];
  snprintf(line1, sizeof(line1), "Net Tilt: %.2f°", imu.tilt_deg);
  snprintf(line2, sizeof(line2), "Pitch: %.2f°\nRoll: %.2f°", imu.pitch_deg, imu.roll_deg);

  set_label_if_changed(primary, s_last_line1, sizeof(s_last_line1), line1);
  set_label_if_changed(secondary, s_last_line2, sizeof(s_last_line2), line2);
  update_inclinometer(incl_container, incl_bubble, imu.pitch_deg, imu.roll_deg);
}

void abs_accel_to_pitch_roll(float ax, float ay, float az, float *pitch_deg, float *roll_deg)
{
  imu_math::accel_vector_to_tilt_pitch_roll(ax, ay, az, pitch_deg, roll_deg, nullptr);
}

} // namespace ui_pages

