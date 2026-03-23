#include <Arduino.h>

#include "rel_page.h"

#include "app_hooks.h"
#include "imu_math.h"
#include "imu_state.h"
#include "pin_config.h"
#include <cstring>
#include <cstdio>
#include <math.h>

LV_FONT_DECLARE(lv_font_roboto_mono_48);

namespace ui_pages {
namespace {

static char s_last_rel_big[64] = {0};
static char s_last_dbg[96] = {0};

// Idle noise on Ax/Ay slightly inflates sqrt(Ax²+Ay²); squash sub-threshold jitter.
static constexpr float kHorizonTiltDeadbandDeg = 0.35f;

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

const lv_font_t *rel_angle_font()
{
  return &lv_font_roboto_mono_48;
}

lv_obj_t *rel_create_angle_label(lv_obj_t *parent)
{
  lv_obj_t *lbl = lv_label_create(parent);
  lv_obj_set_style_text_font(lbl, rel_angle_font(), 0);
  lv_obj_set_style_text_color(lbl, lv_color_white(), 0);
  lv_obj_align(lbl, LV_ALIGN_CENTER, 0, -20);
  lv_label_set_text(lbl, "");
  lv_obj_add_flag(lbl, LV_OBJ_FLAG_HIDDEN);
  return lbl;
}

lv_obj_t *rel_create_zero_button(lv_obj_t *parent, lv_event_cb_t on_click)
{
  lv_obj_t *btn = lv_btn_create(parent);
  lv_obj_set_size(btn, LCD_WIDTH, LCD_HEIGHT / 3);
  lv_obj_align(btn, LV_ALIGN_BOTTOM_MID, 0, 0);
  lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);
  lv_obj_set_style_bg_color(btn, lv_palette_main(LV_PALETTE_ORANGE), 0);
  lv_obj_set_style_radius(btn, 0, 0);
  lv_obj_set_style_border_width(btn, 0, 0);
  lv_obj_clear_flag(btn, LV_OBJ_FLAG_SCROLLABLE);
  if (on_click) {
    lv_obj_add_event_cb(btn, on_click, LV_EVENT_CLICKED, nullptr);
  }

  lv_obj_t *btn_label = lv_label_create(btn);
  lv_obj_set_style_text_font(btn_label, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(btn_label, lv_color_white(), 0);
  lv_label_set_text(btn_label, "Tap to set zero");
  lv_obj_center(btn_label);
  return btn;
}

void rel_capture_reference_now()
{
  portENTER_CRITICAL(&g_imuMux);
  g_imu.ref_ax_g = g_imu.ax_g;
  g_imu.ref_ay_g = g_imu.ay_g;
  g_imu.ref_az_g = g_imu.az_g;
  g_imu.ref_valid =
      (imu_math::safe_norm3(g_imu.ref_ax_g, g_imu.ref_ay_g, g_imu.ref_az_g) > 0.0f);
  portEXIT_CRITICAL(&g_imuMux);
}

void rel_update_angle_sample(float ax, float ay, float az)
{
  portENTER_CRITICAL(&g_imuMux);
  if (g_imu.ref_valid) {
    const float nCur = imu_math::safe_norm3(ax, ay, az);
    const float nRef = imu_math::safe_norm3(g_imu.ref_ax_g, g_imu.ref_ay_g, g_imu.ref_az_g);
    if (nCur > 0.0f && nRef > 0.0f) {
      float c = (ax * g_imu.ref_ax_g + ay * g_imu.ref_ay_g + az * g_imu.ref_az_g) / (nCur * nRef);
      c = imu_math::clampf(c, -1.0f, 1.0f);
      g_imu.rel_angle_deg = imu_math::rad2deg(acosf(c));
    }
  } else {
    // No user reference: zenith / inclination angle of XY plane vs horizon from gravity.
    // α = atan2(||g_xy||, g_z); normalize (Ax,Ay,Az) to 1g for stable atan2.
    const float n = imu_math::safe_norm3(ax, ay, az);
    if (n > 0.0f) {
      const float nx = ax / n;
      const float ny = ay / n;
      const float nz = az / n;
      const float horiz = sqrtf(nx * nx + ny * ny);
      float alpha_deg = imu_math::rad2deg(atan2f(horiz, nz));
      if (alpha_deg < kHorizonTiltDeadbandDeg) alpha_deg = 0.0f;
      g_imu.rel_angle_deg = alpha_deg;
    } else {
      g_imu.rel_angle_deg = 0.0f;
    }
  }
  portEXIT_CRITICAL(&g_imuMux);
}

void rel_on_tap_zero()
{
  rel_capture_reference_now();
  app_play_rel_zero_sound();
}

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

