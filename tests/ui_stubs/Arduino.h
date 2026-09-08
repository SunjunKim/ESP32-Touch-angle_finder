#pragma once
#include <cstdint>
#include <mutex>
using portMUX_TYPE = std::mutex;
#define portMUX_INITIALIZER_UNLOCKED {}
#define portENTER_CRITICAL(m) (m)->lock()
#define portEXIT_CRITICAL(m) (m)->unlock()
inline uint32_t test_ms = 1;
inline uint32_t millis() { return test_ms; }
