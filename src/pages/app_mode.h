#pragma once

#include <stdint.h>

enum class AppMode : uint8_t {
  RelAngle = 0,
  AbsAngle = 1,
  Calibrate = 2,
};
