#pragma once

// Do not include freertos/portmacro.h here: if this header is included before
// <Arduino.h>, the port layer loads without ESP32-S3 dual-core setup and
// FreeRTOS.h errors with "portYIELD_CORE() must be defined".
#include <Arduino.h>

struct ImuState {
  float ax_g = 0.0f;
  float ay_g = 0.0f;
  float az_g = 0.0f;

  float pitch_deg = 0.0f;
  float roll_deg = 0.0f;
  float tilt_deg = 0.0f;

  float ref_ax_g = 0.0f;
  float ref_ay_g = 0.0f;
  float ref_az_g = 0.0f;
  float ref_angle = 0.0f;
  bool ref_valid = false;

  float rel_angle_deg = 0.0f;
  float abs_angle_deg = 0.0f;

  bool inited = false;
};

extern ImuState g_imu;
extern portMUX_TYPE g_imuMux;
