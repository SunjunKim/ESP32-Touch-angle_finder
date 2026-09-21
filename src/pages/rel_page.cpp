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

static constexpr uint32_t kRelZeroHoldMs = 1000;
static constexpr uint32_t kRelZeroMaxSampleAgeMs = 50;

namespace {

static char s_last_rel_big[64] = {0};
static char s_last_dbg[96] = {0};
static char s_last_zero_btn_label[40] = {0};
// All accesses, including zero capture, are protected by g_imuMux.
static accel_rel::ContactRel s_rel;
// UI task only: failed captures keep the previous reference and show a retry hint.
static bool s_zero_retry = false;
static bool s_zero_hold_active = false;
static uint32_t s_zero_press_start_ms = 0;

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
    lv_label_set_text(btn_label, "Hold 1s to set zero");
  }
  
  lv_obj_center(btn_label);
  return btn;
}

bool rel_capture_reference_now()
{
  portENTER_CRITICAL(&g_imuMux);
  if(!g_imu.sample_valid||(uint32_t)(millis()-g_imu.sample_ms)>kRelZeroMaxSampleAgeMs){portEXIT_CRITICAL(&g_imuMux);return false;}
  float average[3];
  if(!s_rel.capture(average)){portEXIT_CRITICAL(&g_imuMux);return false;}
  g_imu.ref_ax_g = average[0];
  g_imu.ref_ay_g = average[1];
  g_imu.ref_az_g = average[2];
  g_imu.ref_angle = s_rel.reference;
  g_imu.ref_normal_axis = s_rel.normal_axis;
  g_imu.abs_angle_deg = s_rel.tilt;
  g_imu.rel_angle_deg = 0;
  g_imu.ref_valid = true;
  portEXIT_CRITICAL(&g_imuMux);
  return true;
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
  s_zero_retry = !rel_capture_reference_now();
  if (s_zero_retry) app_start_tone(220, 150);
  else app_play_rel_zero_sound();
}

void rel_cancel_zero_hold()
{
  s_zero_hold_active = false;
}

static void rel_zero_btn_event_cb(lv_event_t *e)
{
  const lv_event_code_t code = lv_event_get_code(e);
  if (app_mode_snapshot() != AppMode::RelAngle) {
    rel_cancel_zero_hold();
    return;
  }
  if (code == LV_EVENT_PRESSED) {
    s_zero_press_start_ms = millis();
    s_zero_hold_active = true;
    return;
  }
  if (code == LV_EVENT_PRESSING) {
    if (s_zero_hold_active &&
        (uint32_t)(millis() - s_zero_press_start_ms) >= kRelZeroHoldMs) {
      rel_cancel_zero_hold();
      rel_on_tap_zero();
    }
    return;
  }
  if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) {
    rel_cancel_zero_hold();
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
    char rel_big[64];
    if (s_zero_hold_active) {
      const uint32_t elapsed = millis() - s_zero_press_start_ms;
      const unsigned seconds = elapsed < kRelZeroHoldMs
          ? (kRelZeroHoldMs - elapsed + 999) / 1000 : 1;
      snprintf(rel_big, sizeof(rel_big), "Hold %us", seconds);
    } else if (imu.sample_valid) {
      const float rel_rounded = round_to_step(imu.rel_angle_deg, 0.1f);
      snprintf(rel_big, sizeof(rel_big), "%.1f°", rel_rounded);
    } else {
      snprintf(rel_big, sizeof(rel_big), "--");
    }
    // The number font has no letters; use a readable warning font while held.
    lv_obj_set_style_text_font(rel_angle_big,
        s_zero_hold_active ? &lv_font_montserrat_28 : rel_angle_font(), 0);
    set_label_if_changed(rel_angle_big, s_last_rel_big, sizeof(s_last_rel_big), rel_big);
    const lv_color_t color = s_zero_hold_active ? lv_palette_main(LV_PALETTE_ORANGE)
        : imu.moving ? lv_palette_main(LV_PALETTE_RED) : lv_color_white();
    lv_obj_set_style_text_color(rel_angle_big, color, 0);
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
    const char *zero_caption = s_zero_retry ? "Keep still\nHold 1s again" :
        imu.ref_valid ? "Reset zero" : "Hold 1s to set zero";
    set_label_if_changed(btn_label, s_last_zero_btn_label,
                         sizeof(s_last_zero_btn_label), zero_caption);
  }
}

} // namespace ui_pages
