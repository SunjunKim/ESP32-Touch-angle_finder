#pragma once

#include <lvgl.h>
#include <stdint.h>

struct ImuUiData {
  float pitch_deg = 0.0f;
  float roll_deg = 0.0f;
  float rel_angle_deg = 0.0f;
  bool ref_valid = false;
};

struct CalUiData {
  uint8_t step = 0;       // 0..5 = capturing; 6 = saving/done
  uint8_t total = 6;
  float align_dot = 0.0f; // n·ê (1 = perfect)
  bool stable = false;    // magnitude + low jitter
  bool done_saved = false;
  bool save_failed = false;
  bool awaiting_tap = false; // intro: tap screen to begin 6-step capture
};

