#include "cal_page.h"

#include <Arduino.h>
#include "pin_config.h"
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

} // namespace ui_pages
