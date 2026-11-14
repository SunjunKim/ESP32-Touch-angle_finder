#pragma once

#include <lvgl.h>
#include <stdint.h>

struct ImuUiData {
  float pitch_deg = 0.0f;
  float roll_deg = 0.0f;
  float tilt_deg = 0.0f;
  float rel_angle_deg = 0.0f;
  float rel_pitch_angle_deg = 0.0f;
  bool pitch_moving = false;
  bool ref_valid = false;
  uint8_t ref_normal_axis = 2;
  bool sample_valid=false,moving=false;
};

struct CalUiData {
  uint8_t step = 0;       // 0..5 = capturing; 6 = saving/done
  uint8_t total = 6;
  float align_dot = 0.0f; // n·ê (1 = perfect)
  bool stable = false;    // magnitude + low jitter
  bool measuring = false; // true during the 3-second averaging phase
  bool done_saved = false;
  bool save_failed = false;
  bool awaiting_tap = false; // intro: tap screen to begin 6-step capture
};
