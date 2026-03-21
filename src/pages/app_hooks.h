#pragma once

#include <stdint.h>
#include "app_mode.h"

void app_mark_user_activity();
AppMode app_mode_snapshot();
void app_start_tone(uint16_t hz, uint32_t durationMs);
void app_play_rel_zero_sound();
