#include <Arduino.h>

#include "rel_page.h"

#include "app_hooks.h"
#include "imu_state.h"
#include "accel_rel.h"
#include "pin_config.h"
#include <cstring>
#include <cstdio>
#include <math.h>

LV_FONT_DECLARE(lv_font_roboto_mono_48);

namespace ui_pages {

static constexpr uint32_t kRelZeroHoldMs = 3000;

namespace {

static char s_last_rel_big[64] = {0};
static char s_last_dbg[96] = {0};
static char s_last_zero_btn_label[40] = {0};
// All accesses, including zero capture, are protected by g_imuMux.
static accel_rel::ContactRel s_rel;

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

static void rel_zero_btn_event_cb(lv_event_t *e);

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

lv_obj_t *rel_create_zero_button(lv_obj_t *parent)
{
  lv_obj_t *btn = lv_btn_create(parent);
  lv_obj_set_size(btn, LCD_WIDTH, LCD_HEIGHT / 3);
  lv_obj_align(btn, LV_ALIGN_BOTTOM_MID, 0, 0);
  lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);
  lv_obj_set_style_bg_color(btn, lv_palette_main(LV_PALETTE_ORANGE), 0);
  lv_obj_set_style_radius(btn, 0, 0);
  lv_obj_set_style_border_width(btn, 0, 0);
  lv_obj_clear_flag(btn, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_event_cb(btn, rel_zero_btn_event_cb, LV_EVENT_PRESSED, nullptr);
  lv_obj_add_event_cb(btn, rel_zero_btn_event_cb, LV_EVENT_PRESSING, nullptr);
  lv_obj_add_event_cb(btn, rel_zero_btn_event_cb, LV_EVENT_RELEASED, nullptr);
  lv_obj_add_event_cb(btn, rel_zero_btn_event_cb, LV_EVENT_PRESS_LOST, nullptr);

  lv_obj_t *btn_label = lv_label_create(btn);
  lv_obj_set_style_text_font(btn_label, &lv_font_montserrat_20, 0);
  lv_obj_set_style_text_color(btn_label, lv_color_white(), 0);

  if(g_imu.ref_valid) {
    lv_label_set_text(btn_label, "Reset zero");
  } else {
    lv_label_set_text(btn_label, "Hold 3s to set zero");
  }
  
  lv_obj_center(btn_label);
  return btn;
}

void rel_capture_reference_now()
{
  portENTER_CRITICAL(&g_imuMux);
  if(!g_imu.sample_valid||(uint32_t)(millis()-g_imu.sample_ms)>500){portEXIT_CRITICAL(&g_imuMux);return;}
  if(!s_rel.capture()){portEXIT_CRITICAL(&g_imuMux);return;}
  g_imu.ref_ax_g = g_imu.ax_g;
  g_imu.ref_ay_g = g_imu.ay_g;
  g_imu.ref_az_g = g_imu.az_g;
  g_imu.ref_angle = s_rel.reference;
  g_imu.ref_normal_axis = s_rel.normal_axis;
  g_imu.abs_angle_deg = s_rel.tilt;
  g_imu.rel_angle_deg = 0;
  g_imu.ref_valid = true;
  portEXIT_CRITICAL(&g_imuMux);
}

void rel_update_angle_sample()
{
  portENTER_CRITICAL(&g_imuMux);
  const uint32_t now=millis();
  const float dt=g_imu.sample_ms?(uint32_t)(now-g_imu.sample_ms)*.001f:0;
  if(!g_imu.ref_valid)s_rel.clear_reference();
  if(!g_imu.sample_valid)s_rel.valid=false;
  g_imu.sample_valid=s_rel.update(g_imu.ax_g,g_imu.ay_g,g_imu.az_g,dt);
  g_imu.moving=s_rel.moving;
  g_imu.sample_ms=now;
  if(g_imu.sample_valid){
    g_imu.abs_angle_deg=s_rel.tilt;
    g_imu.rel_angle_deg=s_rel.angle;
  }
  portEXIT_CRITICAL(&g_imuMux);
}

void rel_on_tap_zero()
{
  portENTER_CRITICAL(&g_imuMux);
  const bool fresh=g_imu.sample_valid&&(uint32_t)(millis()-g_imu.sample_ms)<500;
  portEXIT_CRITICAL(&g_imuMux);
  if(!fresh)return;
  rel_capture_reference_now();
  app_play_rel_zero_sound();
}

static void rel_zero_btn_event_cb(lv_event_t *e)
{
  const lv_event_code_t code = lv_event_get_code(e);
  static uint32_t s_press_start_ms = 0;
  static bool s_fired_this_hold = false;

  if (code == LV_EVENT_PRESSED) {
    s_press_start_ms = millis();
    s_fired_this_hold = false;
    return;
  }
  if (code == LV_EVENT_PRESSING) {
    if (s_fired_this_hold) {
      return;
    }
    if ((uint32_t)(millis() - s_press_start_ms) >= kRelZeroHoldMs) {
      s_fired_this_hold = true;
      rel_on_tap_zero();
    }
    return;
  }
  if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) {
    return;
  }
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
                lv_obj_t *rel_zero_btn,
                const ImuUiData &imu)
{
  if (rel_angle_big) {
    const float rel_rounded = round_to_step(imu.rel_angle_deg, 0.1f);
    char rel_big[64];
    if(imu.sample_valid)snprintf(rel_big, sizeof(rel_big), "%.1f°", rel_rounded);
    else snprintf(rel_big,sizeof(rel_big),"--");
    set_label_if_changed(rel_angle_big, s_last_rel_big, sizeof(s_last_rel_big), rel_big);
    lv_obj_set_style_text_color(rel_angle_big,imu.moving?lv_palette_main(LV_PALETTE_RED):lv_color_white(),0);
  }

  if (debug_label) {
    char dbg[96];
    const char axis="XYZ"[imu.ref_normal_axis<3?imu.ref_normal_axis:2];
    if(imu.ref_valid)snprintf(dbg,sizeof(dbg),"Ref: SET / N:%c locked",axis);
    else snprintf(dbg,sizeof(dbg),"Ref: NOT SET");
    set_label_if_changed(debug_label, s_last_dbg, sizeof(s_last_dbg), dbg);
  }

  if (rel_zero_btn) {
    lv_obj_t *btn_label = lv_obj_get_child(rel_zero_btn, 0);
    const char *zero_caption =
        imu.ref_valid ? "Reset zero" : "Hold 3s to set zero";
    set_label_if_changed(btn_label, s_last_zero_btn_label,
                         sizeof(s_last_zero_btn_label), zero_caption);
  }
}

} // namespace ui_pages
