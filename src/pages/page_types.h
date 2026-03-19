#pragma once

#include <lvgl.h>
#include <stdint.h>

struct ImuUiData {
  float pitch_deg = 0.0f;
  float roll_deg = 0.0f;
  float rel_angle_deg = 0.0f;
  bool ref_valid = false;
};

