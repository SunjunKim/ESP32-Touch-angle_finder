#pragma once

#include <stdint.h>

enum class AppMode : uint8_t {
  Rel2D = 0,
  Rel1D = 1,
  AbsAngle = 2,
  Calibrate = 3,
};

inline bool is_rel_mode(AppMode mode) {
  return mode == AppMode::Rel1D || mode == AppMode::Rel2D;
}
