#pragma once
#include <string>
using lv_coord_t = int;
using lv_align_t = int;
using lv_color_t = int;
constexpr int LV_OBJ_FLAG_HIDDEN = 1, LV_OBJ_FLAG_CLICKABLE = 2, LV_OBJ_FLAG_SCROLLABLE = 4;
constexpr int LV_ALIGN_CENTER = 0, LV_ALIGN_BOTTOM_MID = 1, LV_ALIGN_TOP_MID = 2,
              LV_ALIGN_LEFT_MID = 3, LV_ALIGN_RIGHT_MID = 4, LV_ALIGN_TOP_LEFT = 5;
constexpr int LV_LABEL_LONG_CLIP = 0, LV_LABEL_LONG_WRAP = 1, LV_EVENT_CLICKED = 1,
              LV_OPA_COVER = 255, LV_OPA_TRANSP = 0, LV_PALETTE_GREEN = 1, LV_PALETTE_RED = 2;
struct lv_obj_t { std::string text; int flags = 0, width = 100, height = 100, x = 0, y = 0, font = 0, color = 0; };
struct lv_event_t { int code = LV_EVENT_CLICKED; void *user_data = nullptr; };
inline lv_obj_t *lv_scr_act() { static lv_obj_t screen; return &screen; }
inline lv_obj_t *lv_obj_create(lv_obj_t *) { static lv_obj_t objects[16]; static int i = 0; return &objects[i++]; }
inline lv_obj_t *lv_btn_create(lv_obj_t *p) { return lv_obj_create(p); }
inline void lv_obj_add_flag(lv_obj_t *o, int flag) { o->flags |= flag; }
inline void lv_obj_clear_flag(lv_obj_t *o, int flag) { o->flags &= ~flag; }
inline void lv_label_set_text(lv_obj_t *o, const char *s) { o->text = s; }
inline void lv_obj_set_pos(lv_obj_t *o, int x, int y) { o->x = x; o->y = y; }
inline void lv_obj_set_width(lv_obj_t *o, int w) { o->width = w; }
inline void lv_obj_set_size(lv_obj_t *o, int w, int h) { o->width = w; o->height = h; }
inline int lv_obj_get_width(lv_obj_t *o) { return o->width; }
inline int lv_obj_get_height(lv_obj_t *o) { return o->height; }
inline int lv_disp_get_hor_res(void *) { return 240; }
inline int lv_disp_get_ver_res(void *) { return 280; }
inline int lv_event_get_code(lv_event_t *e) { return e->code; }
inline void *lv_event_get_user_data(lv_event_t *e) { return e->user_data; }
inline int lv_palette_main(int color) { return color; }
inline void lv_obj_remove_style_all(lv_obj_t *) {}
inline void lv_obj_align(lv_obj_t *, int, int, int) {}
inline void lv_label_set_long_mode(lv_obj_t *, int) {}
inline void lv_obj_add_event_cb(lv_obj_t *, void (*)(lv_event_t *), int, void *) {}
inline void lv_obj_set_style_radius(lv_obj_t *, int, int) {}
inline void lv_obj_set_style_bg_color(lv_obj_t *, int, int) {}
inline void lv_obj_set_style_bg_opa(lv_obj_t *, int, int) {}
inline void lv_obj_set_style_border_width(lv_obj_t *, int, int) {}
using lv_font_t = int;
inline const lv_font_t lv_font_montserrat_20 = 20;
inline const lv_font_t lv_font_montserrat_28 = 28;
#define LV_FONT_DECLARE(name) inline const lv_font_t name = 48
constexpr int LV_EVENT_PRESSED = 2, LV_EVENT_PRESSING = 3, LV_EVENT_RELEASED = 4,
              LV_EVENT_PRESS_LOST = 5, LV_PALETTE_ORANGE = 3;
using lv_event_code_t = int;
inline lv_obj_t *lv_label_create(lv_obj_t *p) { return lv_obj_create(p); }
inline lv_obj_t *lv_obj_get_child(lv_obj_t *p, int) { return p; }
inline void lv_obj_center(lv_obj_t *) {}
inline int lv_color_white() { return 0xffffff; }
inline void lv_obj_set_style_text_font(lv_obj_t *o, const lv_font_t *font, int) { o->font = *font; }
inline void lv_obj_set_style_text_color(lv_obj_t *o, int color, int) { o->color = color; }
