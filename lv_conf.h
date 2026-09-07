/**
 * Project-local LVGL configuration.
 *
 * Keep this file valid for both LVGL 8 and 9.  In particular, do not include C
 * headers here: LVGL 9 also includes this configuration while preprocessing
 * optional assembly sources.
 */
#ifndef LV_CONF_H
#define LV_CONF_H

#define LV_COLOR_DEPTH 16
#define LV_COLOR_16_SWAP 0

#define LV_USE_LOG 0

#define LV_FONT_MONTSERRAT_12 1
#define LV_FONT_MONTSERRAT_14 1
#define LV_FONT_MONTSERRAT_20 1
#define LV_FONT_MONTSERRAT_28 1
#define LV_FONT_DEFAULT &lv_font_montserrat_14

/* The ESP32-S3 has no ARM/RISC-V vector drawing extension. */
#define LV_USE_DRAW_SW_ASM 0

#endif /* LV_CONF_H */
