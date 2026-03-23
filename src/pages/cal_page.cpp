#include "cal_page.h"

#include "app_hooks.h"
#include "imu_math.h"
#include <Arduino.h>
#include "pin_config.h"
#include <Preferences.h>
#include <cstdio>
#include <cstring>
#include <math.h>

namespace ui_pages {
namespace {

static char s_line1[96] = {0};
static char s_line2[96] = {0};
static char s_line3[96] = {0};

static constexpr lv_coord_t kCalEdgeThick = 5;

static lv_obj_t *s_edge_top = nullptr;
static lv_obj_t *s_edge_bottom = nullptr;
static lv_obj_t *s_edge_left = nullptr;
static lv_obj_t *s_edge_right = nullptr;

// ---- Accel calibration (6-face: bias + diagonal scale + mount rotation) ----
static constexpr char kAccCalPrefsNs[] = "imu_acc6";
static constexpr uint8_t kAccCalPrefsVer = 2;
static float s_accBias[3] = {0.0f, 0.0f, 0.0f};
static float s_accScale[3] = {1.0f, 1.0f, 1.0f};
// Row-major R: v_device = R * v_rect; v_rect_i = scale_i * (raw_i - bias_i)
static float s_accR[9] = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
static bool s_accCalValid = false;

static inline void apply_accel_cal_impl(float &ax, float &ay, float &az)
{
  if (!s_accCalValid) return;
  const float vx = s_accScale[0] * (ax - s_accBias[0]);
  const float vy = s_accScale[1] * (ay - s_accBias[1]);
  const float vz = s_accScale[2] * (az - s_accBias[2]);
  imu_math::mat3_vec3_rowmajor(s_accR, vx, vy, vz, &ax, &ay, &az);
}

static void save_accel_cal_prefs()
{
  Preferences p;
  if (!p.begin(kAccCalPrefsNs, false)) {
    return;
  }
  p.putBool("ok", true);
  p.putFloat("bx", s_accBias[0]);
  p.putFloat("by", s_accBias[1]);
  p.putFloat("bz", s_accBias[2]);
  p.putFloat("sx", s_accScale[0]);
  p.putFloat("sy", s_accScale[1]);
  p.putFloat("sz", s_accScale[2]);
  p.putUChar("ver", kAccCalPrefsVer);
  for (int i = 0; i < 9; i++) {
    char key[] = "r0";
    key[1] = (char)('0' + i);
    p.putFloat(key, s_accR[i]);
  }
  p.end();
}

static bool compute_accel_cal_from_6face(const float s[6][3])
{
  const float dx = s[2][0] - s[3][0];
  const float dy = s[4][1] - s[5][1];
  const float dz = s[0][2] - s[1][2];
  if (fabsf(dx) < 0.12f || fabsf(dy) < 0.12f || fabsf(dz) < 0.12f) {
    return false;
  }
  s_accBias[0] = (s[2][0] + s[3][0]) * 0.5f;
  s_accBias[1] = (s[4][1] + s[5][1]) * 0.5f;
  s_accBias[2] = (s[0][2] + s[1][2]) * 0.5f;
  s_accScale[0] = 2.0f / dx;
  s_accScale[1] = 2.0f / dy;
  s_accScale[2] = 2.0f / dz;
  if (!isfinite(s_accScale[0]) || !isfinite(s_accScale[1]) || !isfinite(s_accScale[2])) {
    return false;
  }
  float Rmount[9];
  if (!imu_math::acc_mount_matrix_from_6face(s, s_accBias, s_accScale, Rmount)) {
    return false;
  }
  for (int i = 0; i < 9; i++) {
    s_accR[i] = Rmount[i];
  }
  s_accCalValid = true;
  save_accel_cal_prefs();
  return true;
}

// ---- CAL wizard ----
static constexpr uint8_t kCalSteps = 6;
static const float kCalExpected[kCalSteps][3] = {
  {0.0f, 0.0f, -1.0f},
  {0.0f, 0.0f, 1.0f},
  {-1.0f, 0.0f, 0.0f},
  {1.0f, 0.0f, 0.0f},
  {0.0f, 1.0f, 0.0f},
  {0.0f, -1.0f, 0.0f},
};
static float s_calSamples[kCalSteps][3];
static uint8_t s_calStep = 0;
static uint32_t s_calStillStartMs = 0;
static float s_calSumAx = 0.0f, s_calSumAy = 0.0f, s_calSumAz = 0.0f;
static uint32_t s_calSumCount = 0;
static uint32_t s_calStableAllCount = 0;
static float s_calLastRaw[3] = {0.0f, 0.0f, 0.0f};
static bool s_calPrevValid = false;
static uint8_t s_calBadStreak = 0;
static volatile bool s_calAwaitingTapToStart = true;

static constexpr float kCalMagMinG = 0.86f;
static constexpr float kCalMagMaxG = 1.14f;
static constexpr float kCalDotMin = 0.90f;
static constexpr float kCalJitterMaxG = 0.14f;
static constexpr uint32_t kCalHoldMs = 650;
static constexpr uint32_t kCalMinStableSamples = 20;
// Discard this many stable samples at the start of each hold (settling after placement).
static constexpr uint32_t kCalStableDiscardFirst = kCalMinStableSamples / 2;
static constexpr uint8_t kCalBadStreakToReset = 8;
static CalUiData s_calUi;
static portMUX_TYPE s_calUiMux = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE s_calToneMux = portMUX_INITIALIZER_UNLOCKED;
static uint16_t s_calPendingBeepHz = 0;
static uint32_t s_calPendingBeepMs = 0;

static constexpr uint32_t kCalSecondBeepAfterMs = 200;
static constexpr uint16_t kCalSecondBeepHz = 660;
static constexpr uint32_t kCalSecondBeepMs = 55;
static uint32_t s_calDelayedBeepAtMs = 0;
static uint16_t s_calDelayedBeepHz = 0;
static uint32_t s_calDelayedBeepDurMs = 0;

static void cal_request_beep(uint16_t hz, uint32_t ms)
{
  portENTER_CRITICAL(&s_calToneMux);
  s_calPendingBeepHz = hz;
  s_calPendingBeepMs = ms;
  portEXIT_CRITICAL(&s_calToneMux);
}

static void cal_schedule_delayed_beep(uint32_t delayMs, uint16_t hz, uint32_t durMs)
{
  portENTER_CRITICAL(&s_calToneMux);
  s_calDelayedBeepAtMs = millis() + delayMs;
  s_calDelayedBeepHz = hz;
  s_calDelayedBeepDurMs = durMs;
  portEXIT_CRITICAL(&s_calToneMux);
}

static void cal_cancel_delayed_beep()
{
  portENTER_CRITICAL(&s_calToneMux);
  s_calDelayedBeepAtMs = 0;
  s_calDelayedBeepHz = 0;
  s_calDelayedBeepDurMs = 0;
  portEXIT_CRITICAL(&s_calToneMux);
}

static void cal_ensure_edges()
{
  if (s_edge_top) {
    return;
  }
  lv_obj_t *scr = lv_scr_act();

  auto create_edge = [scr](lv_obj_t **out, lv_coord_t w, lv_coord_t h, lv_align_t align) {
    *out = lv_obj_create(scr);
    lv_obj_remove_style_all(*out);
    lv_obj_set_size(*out, w, h);
    lv_obj_align(*out, align, 0, 0);
    lv_obj_clear_flag(*out, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_style_radius(*out, 0, 0);
    lv_obj_set_style_bg_opa(*out, LV_OPA_COVER, 0);
    lv_obj_add_flag(*out, LV_OBJ_FLAG_HIDDEN);
  };

  const lv_coord_t dw = lv_disp_get_hor_res(nullptr);
  const lv_coord_t dh = lv_disp_get_ver_res(nullptr);
  create_edge(&s_edge_top, dw, kCalEdgeThick, LV_ALIGN_TOP_MID);
  create_edge(&s_edge_bottom, dw, kCalEdgeThick, LV_ALIGN_BOTTOM_MID);
  create_edge(&s_edge_left, kCalEdgeThick, dh, LV_ALIGN_LEFT_MID);
  create_edge(&s_edge_right, kCalEdgeThick, dh, LV_ALIGN_RIGHT_MID);
}

static void apply_edge(lv_obj_t *edge, bool highlight, bool active, lv_color_t ok, lv_color_t bad)
{
  if (!edge) {
    return;
  }
  if (!active) {
    lv_obj_add_flag(edge, LV_OBJ_FLAG_HIDDEN);
    return;
  }
  lv_obj_clear_flag(edge, LV_OBJ_FLAG_HIDDEN);
  lv_obj_set_style_bg_color(edge, highlight ? ok : bad, 0);
}

static void set_label(lv_obj_t *label, char *cache, size_t n, const char *txt)
{
  if (!label || !txt) return;
  if (strncmp(cache, txt, n) == 0) return;
  strncpy(cache, txt, n - 1);
  cache[n - 1] = '\0';
  lv_label_set_text(label, cache);
}

static const char *instruction_for_step(uint8_t step)
{
  switch (step) {
    case 0: return "Lay flat on a table,\nLCD facing up";
    case 1: return "Lay flat on a table,\nLCD facing down";
    case 2: return "Put TOP side down\n(highlighted edge toward table)";
    case 3: return "Put BOTTOM side down\n(highlighted edge toward table)";
    case 4: return "Put LEFT side down\n(highlighted edge toward table)";
    case 5: return "Put RIGHT side down\n(highlighted edge toward table)";
    default: return "Calibration saved.\nPress MODE key to exit.";
  }
}

static void wizard_imu_process_sample(float ax, float ay, float az)
{
  if (s_calStep >= kCalSteps) {
    return;
  }

  if (s_calAwaitingTapToStart) {
    const float *const e = kCalExpected[0];
    const float n2 = ax * ax + ay * ay + az * az;
    const float nm = sqrtf(n2);
    float dot = 0.0f;
    bool magOk = false;
    bool orientOk = false;
    bool jitterOk = true;

    if (nm > 1e-4f) {
      const float inv = 1.0f / nm;
      const float nx = ax * inv;
      const float ny = ay * inv;
      const float nz = az * inv;
      dot = nx * e[0] + ny * e[1] + nz * e[2];
      magOk = (nm > kCalMagMinG && nm < kCalMagMaxG);
      orientOk = (dot > kCalDotMin);
    }

    if (s_calPrevValid) {
      const float dx = ax - s_calLastRaw[0];
      const float dy = ay - s_calLastRaw[1];
      const float dz = az - s_calLastRaw[2];
      const float d = sqrtf(dx * dx + dy * dy + dz * dz);
      if (d >= kCalJitterMaxG) {
        jitterOk = false;
      }
    }
    s_calLastRaw[0] = ax;
    s_calLastRaw[1] = ay;
    s_calLastRaw[2] = az;
    s_calPrevValid = true;

    const bool stable = magOk && orientOk && jitterOk;

    portENTER_CRITICAL(&s_calUiMux);
    s_calUi.align_dot = dot;
    s_calUi.step = 0;
    s_calUi.stable = stable;
    portEXIT_CRITICAL(&s_calUiMux);
    return;
  }

  const float *const e = kCalExpected[s_calStep];
  const float n2 = ax * ax + ay * ay + az * az;
  const float nm = sqrtf(n2);
  float dot = 0.0f;
  bool magOk = false;
  bool orientOk = false;
  bool jitterOk = true;

  if (nm > 1e-4f) {
    const float inv = 1.0f / nm;
    const float nx = ax * inv;
    const float ny = ay * inv;
    const float nz = az * inv;
    dot = nx * e[0] + ny * e[1] + nz * e[2];
    magOk = (nm > kCalMagMinG && nm < kCalMagMaxG);
    orientOk = (dot > kCalDotMin);
  }

  if (s_calPrevValid) {
    const float dx = ax - s_calLastRaw[0];
    const float dy = ay - s_calLastRaw[1];
    const float dz = az - s_calLastRaw[2];
    const float d = sqrtf(dx * dx + dy * dy + dz * dz);
    if (d >= kCalJitterMaxG) {
      jitterOk = false;
    }
  }
  s_calLastRaw[0] = ax;
  s_calLastRaw[1] = ay;
  s_calLastRaw[2] = az;
  s_calPrevValid = true;

  const bool stable = magOk && orientOk && jitterOk;
  const uint32_t now = millis();

  portENTER_CRITICAL(&s_calUiMux);
  s_calUi.step = (s_calStep < kCalSteps) ? s_calStep : kCalSteps;
  s_calUi.total = kCalSteps;
  s_calUi.align_dot = dot;
  s_calUi.stable = stable;
  portEXIT_CRITICAL(&s_calUiMux);

  if (stable) {
    s_calBadStreak = 0;
    if (s_calStillStartMs == 0) {
      s_calStillStartMs = now;
      s_calStableAllCount = 0;
    }
    s_calStableAllCount++;
    if (s_calStableAllCount > kCalStableDiscardFirst) {
      s_calSumAx += ax;
      s_calSumAy += ay;
      s_calSumAz += az;
      s_calSumCount++;
    }

    const uint32_t held = now - s_calStillStartMs;
    if (held >= kCalHoldMs && s_calSumCount >= kCalMinStableSamples) {
      s_calSamples[s_calStep][0] = s_calSumAx / (float)s_calSumCount;
      s_calSamples[s_calStep][1] = s_calSumAy / (float)s_calSumCount;
      s_calSamples[s_calStep][2] = s_calSumAz / (float)s_calSumCount;
      s_calStep++;
      s_calStillStartMs = 0;
      s_calSumAx = s_calSumAy = s_calSumAz = 0.0f;
      s_calSumCount = 0;
      s_calStableAllCount = 0;
      s_calPrevValid = false;

      if (s_calStep < kCalSteps) {
        cal_request_beep(880, 85);
        cal_schedule_delayed_beep(kCalSecondBeepAfterMs, kCalSecondBeepHz, kCalSecondBeepMs);
      } else {
        cal_cancel_delayed_beep();
        const bool ok = compute_accel_cal_from_6face(s_calSamples);
        portENTER_CRITICAL(&s_calUiMux);
        s_calUi.done_saved = ok;
        s_calUi.save_failed = !ok;
        s_calUi.step = kCalSteps;
        s_calUi.stable = false;
        portEXIT_CRITICAL(&s_calUiMux);
        cal_request_beep(ok ? 1000 : 220, ok ? 140 : 300);
      }
    }
  } else {
    s_calBadStreak++;
    if (s_calBadStreak >= kCalBadStreakToReset) {
      s_calStillStartMs = 0;
      s_calSumAx = s_calSumAy = s_calSumAz = 0.0f;
      s_calSumCount = 0;
      s_calStableAllCount = 0;
      s_calBadStreak = 0;
    }
  }
}

} // namespace

void cal_invalidate_label_caches()
{
  memset(s_line1, 0, sizeof(s_line1));
  memset(s_line2, 0, sizeof(s_line2));
  memset(s_line3, 0, sizeof(s_line3));
}

void cal_hide_edges()
{
  if (!s_edge_top) {
    return;
  }
  lv_obj_add_flag(s_edge_top, LV_OBJ_FLAG_HIDDEN);
  lv_obj_add_flag(s_edge_bottom, LV_OBJ_FLAG_HIDDEN);
  lv_obj_add_flag(s_edge_left, LV_OBJ_FLAG_HIDDEN);
  lv_obj_add_flag(s_edge_right, LV_OBJ_FLAG_HIDDEN);
}

void cal_show(lv_obj_t *incl_container,
              lv_obj_t *rel_angle_big,
              lv_obj_t *primary,
              lv_obj_t *secondary,
              lv_obj_t *debug,
              lv_obj_t *rel_zero_btn)
{
  if (incl_container) lv_obj_add_flag(incl_container, LV_OBJ_FLAG_HIDDEN);
  if (rel_angle_big) lv_obj_add_flag(rel_angle_big, LV_OBJ_FLAG_HIDDEN);
  if (primary) {
    lv_obj_clear_flag(primary, LV_OBJ_FLAG_HIDDEN);
    lv_label_set_long_mode(primary, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(primary, LCD_WIDTH - 24);
    lv_obj_align(primary, LV_ALIGN_CENTER, 0, -36);
  }
  if (secondary) {
    lv_obj_clear_flag(secondary, LV_OBJ_FLAG_HIDDEN);
    lv_label_set_long_mode(secondary, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(secondary, LCD_WIDTH - 24);
    lv_obj_align(secondary, LV_ALIGN_CENTER, 0, 28);
  }
  if (debug) {
    lv_obj_clear_flag(debug, LV_OBJ_FLAG_HIDDEN);
    lv_obj_align(debug, LV_ALIGN_BOTTOM_MID, 0, -10);
  }
  if (rel_zero_btn) lv_obj_add_flag(rel_zero_btn, LV_OBJ_FLAG_HIDDEN);
  cal_ensure_edges();
}

void cal_update(lv_obj_t *primary,
                lv_obj_t *secondary,
                lv_obj_t *debug,
                const CalUiData &cal)
{
  char l1[96];
  char l2[96];
  char l3[96];

  if (cal.awaiting_tap) {
    snprintf(l1, sizeof(l1), "Calibration");
    snprintf(l2, sizeof(l2),
             "Find a flat surface.\nTap to start.");
    snprintf(l3, sizeof(l3), "Tap anywhere.");
  } else if (cal.save_failed) {
    snprintf(l1, sizeof(l1), "Failed");
    snprintf(l2, sizeof(l2), "Bad samples.\nMODE to exit, re-enter CAL.");
    snprintf(l3, sizeof(l3), "");
  } else if (cal.done_saved) {
    snprintf(l1, sizeof(l1), "Done");
    snprintf(l2, sizeof(l2), "Accel cal saved.\nPress MODE to leave.");
    snprintf(l3, sizeof(l3), "");
  } else if (cal.step >= cal.total) {
    snprintf(l1, sizeof(l1), "Processing…");
    snprintf(l2, sizeof(l2), "Saving calibration");
    snprintf(l3, sizeof(l3), "");
  } else {
    snprintf(l1, sizeof(l1), "Step %u / %u", (unsigned)(cal.step + 1), (unsigned)cal.total);
    snprintf(l2, sizeof(l2), "%s", instruction_for_step(cal.step));
    if (cal.stable) {
      snprintf(l3, sizeof(l3), "Hold still... OK (%.2f)", cal.align_dot);
    } else {
      snprintf(l3, sizeof(l3), "Orient & hold still\n(dot %.2f, need >0.90)", cal.align_dot);
    }
  }

  set_label(primary, s_line1, sizeof(s_line1), l1);
  set_label(secondary, s_line2, sizeof(s_line2), l2);
  set_label(debug, s_line3, sizeof(s_line3), l3);

  cal_ensure_edges();
  const lv_color_t col_ok = lv_palette_main(LV_PALETTE_GREEN);
  const lv_color_t col_bad = lv_palette_main(LV_PALETTE_RED);
  const bool ok = cal.stable;
  const bool edge_phase =
      !cal.awaiting_tap && !cal.done_saved && !cal.save_failed && cal.step < cal.total;

  bool hi_top = false, hi_bot = false, hi_left = false, hi_right = false;
  if (edge_phase) {
    if (cal.step <= 1) {
      hi_top = hi_bot = hi_left = hi_right = true;
    } else if (cal.step == 2) {
      hi_top = true;
    } else if (cal.step == 3) {
      hi_bot = true;
    } else if (cal.step == 4) {
      hi_left = true;
    } else if (cal.step == 5) {
      hi_right = true;
    }
  }

  apply_edge(s_edge_top, ok, edge_phase && hi_top, col_ok, col_bad);
  apply_edge(s_edge_bottom, ok, edge_phase && hi_bot, col_ok, col_bad);
  apply_edge(s_edge_left, ok, edge_phase && hi_left, col_ok, col_bad);
  apply_edge(s_edge_right, ok, edge_phase && hi_right, col_ok, col_bad);
}

bool cal_load_accel_prefs()
{
  Preferences p;
  if (!p.begin(kAccCalPrefsNs, true)) {
    return false;
  }
  const bool ok = p.getBool("ok", false);
  if (!ok) {
    p.end();
    return false;
  }
  s_accBias[0] = p.getFloat("bx", 0.0f);
  s_accBias[1] = p.getFloat("by", 0.0f);
  s_accBias[2] = p.getFloat("bz", 0.0f);
  s_accScale[0] = p.getFloat("sx", 1.0f);
  s_accScale[1] = p.getFloat("sy", 1.0f);
  s_accScale[2] = p.getFloat("sz", 1.0f);
  const uint8_t ver = p.getUChar("ver", 0);
  static const float kIdentR[9] = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
  if (ver >= 2) {
    for (int i = 0; i < 9; i++) {
      char key[] = "r0";
      key[1] = (char)('0' + i);
      s_accR[i] = p.getFloat(key, kIdentR[i]);
    }
  } else {
    memcpy(s_accR, kIdentR, sizeof(s_accR));
  }
  p.end();
  s_accCalValid = true;
  return true;
}

void cal_apply_accel(float &ax, float &ay, float &az)
{
  apply_accel_cal_impl(ax, ay, az);
}

bool imu_calibrated_reading(float raw_ax,
                            float raw_ay,
                            float raw_az,
                            ImuCalibratedSample *out,
                            bool apply_saved_calibration)
{
  if (!out) {
    return false;
  }
  float ax = raw_ax;
  float ay = raw_ay;
  float az = raw_az;
  if (apply_saved_calibration) {
    apply_accel_cal_impl(ax, ay, az);
  }
  out->ax_g = ax;
  out->ay_g = ay;
  out->az_g = az;
  imu_math::accel_vector_to_tilt_pitch_roll(ax, ay, az, &out->pitch_deg, &out->roll_deg, &out->tilt_deg);
  return true;
}

bool cal_accel_cal_valid()
{
  return s_accCalValid;
}

void cal_wizard_reset()
{
  cal_cancel_delayed_beep();
  cal_invalidate_label_caches();
  s_calAwaitingTapToStart = true;
  s_calStep = 0;
  s_calStillStartMs = 0;
  s_calSumAx = s_calSumAy = s_calSumAz = 0.0f;
  s_calSumCount = 0;
  s_calStableAllCount = 0;
  s_calPrevValid = false;
  s_calBadStreak = 0;
  portENTER_CRITICAL(&s_calUiMux);
  s_calUi = CalUiData{};
  portEXIT_CRITICAL(&s_calUiMux);
}

void cal_wizard_imu_sample(float ax, float ay, float az)
{
  wizard_imu_process_sample(ax, ay, az);
}

void cal_flush_pending_beep()
{
  uint16_t hz = 0;
  uint32_t ms = 0;
  portENTER_CRITICAL(&s_calToneMux);
  hz = s_calPendingBeepHz;
  ms = s_calPendingBeepMs;
  s_calPendingBeepHz = 0;
  s_calPendingBeepMs = 0;
  portEXIT_CRITICAL(&s_calToneMux);
  if (hz > 0 && ms > 0) {
    app_start_tone(hz, ms);
  }
}

void cal_poll_delayed_beep()
{
  uint32_t when = 0;
  uint16_t hz = 0;
  uint32_t dur = 0;
  portENTER_CRITICAL(&s_calToneMux);
  when = s_calDelayedBeepAtMs;
  hz = s_calDelayedBeepHz;
  dur = s_calDelayedBeepDurMs;
  if (when != 0 && hz > 0 && dur > 0 && (int32_t)(millis() - when) >= 0) {
    s_calDelayedBeepAtMs = 0;
    s_calDelayedBeepHz = 0;
    s_calDelayedBeepDurMs = 0;
    portEXIT_CRITICAL(&s_calToneMux);
    app_start_tone(hz, dur);
    return;
  }
  portEXIT_CRITICAL(&s_calToneMux);
}

CalUiData cal_ui_for_refresh()
{
  CalUiData cu;
  portENTER_CRITICAL(&s_calUiMux);
  cu = s_calUi;
  portEXIT_CRITICAL(&s_calUiMux);
  cu.awaiting_tap = s_calAwaitingTapToStart && !cu.done_saved && !cu.save_failed;
  return cu;
}

void cal_intro_tap_event(lv_event_t *e)
{
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
    return;
  }
  if (!s_calAwaitingTapToStart) {
    return;
  }
  if (app_mode_snapshot() != AppMode::Calibrate) {
    return;
  }
  s_calAwaitingTapToStart = false;
  app_mark_user_activity();
  app_start_tone(523, 50);
  lv_obj_t *layer = (lv_obj_t *)lv_event_get_user_data(e);
  if (layer != nullptr) {
    lv_obj_add_flag(layer, LV_OBJ_FLAG_HIDDEN);
  }
}

lv_obj_t *cal_create_intro_tap_layer(lv_obj_t *parent)
{
  lv_obj_t *layer = lv_btn_create(parent);
  lv_obj_set_size(layer, LCD_WIDTH, LCD_HEIGHT);
  lv_obj_align(layer, LV_ALIGN_TOP_LEFT, 0, 0);
  lv_obj_set_style_bg_opa(layer, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(layer, 0, 0);
  lv_obj_clear_flag(layer, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_add_event_cb(layer, cal_intro_tap_event, LV_EVENT_CLICKED, layer);
  lv_obj_add_flag(layer, LV_OBJ_FLAG_HIDDEN);
  return layer;
}

} // namespace ui_pages
