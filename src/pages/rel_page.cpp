#include "rel_page.h"

#include <Arduino.h>
#include <cstring>
#include <cstdio>
#include <math.h>

namespace ui_pages {
namespace {

static char s_last_rel_big[64] = {0};
static char s_last_dbg[96] = {0};

static inline float round_to_step(float v, float step)
{
  if (!(step > 0.0f) || !isfinite(v)) return v;
  return roundf(v / step) * step;
}

static void set_label_if_changed(lv_obj_t *label, char *cache, size_t cache_len, const char *next)
{
  if (!label || !next) return;
  if (strncmp(cache, next, cache_len) == 0) return;
  strncpy(cache, next, cache_len - 1);
  cache[cache_len - 1] = '\0';
  lv_label_set_text(label, cache);
}

} // namespace

void rel_show(lv_obj_t *incl_container,
              lv_obj_t *rel_angle_big,
              lv_obj_t *primary,
              lv_obj_t *secondary,
              lv_obj_t *rel_zero_btn)
{
  if (incl_container) lv_obj_add_flag(incl_container, LV_OBJ_FLAG_HIDDEN);
  if (rel_angle_big) lv_obj_clear_flag(rel_angle_big, LV_OBJ_FLAG_HIDDEN);
  if (primary) lv_obj_add_flag(primary, LV_OBJ_FLAG_HIDDEN);
  if (secondary) lv_obj_add_flag(secondary, LV_OBJ_FLAG_HIDDEN);
  if (rel_zero_btn) lv_obj_clear_flag(rel_zero_btn, LV_OBJ_FLAG_HIDDEN);
}

void rel_update(lv_obj_t *rel_angle_big,
                lv_obj_t *debug_label,
                const ImuUiData &imu)
{
  if (rel_angle_big) {
    const float rel_rounded = round_to_step(imu.rel_angle_deg, 0.1f);
    char rel_big[64];
    snprintf(rel_big, sizeof(rel_big), "%.1f°", rel_rounded);
    set_label_if_changed(rel_angle_big, s_last_rel_big, sizeof(s_last_rel_big), rel_big);
  }

  if (debug_label) {
    char dbg[96];
    snprintf(dbg, sizeof(dbg), "Ref: %s", imu.ref_valid ? "SET" : "NOT SET");
    set_label_if_changed(debug_label, s_last_dbg, sizeof(s_last_dbg), dbg);
  }
}

} // namespace ui_pages

