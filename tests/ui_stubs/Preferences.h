#pragma once
#include <cstdint>
#include <cstring>
class Preferences {
 public:
  inline static bool fail_write = false;
  inline static float record[15] = {};
  bool begin(const char *, bool) { return true; }
  void end() {}
  size_t putBytes(const char *, const void *p, size_t n) {
    if (fail_write) return 0;
    std::memcpy(record, p, n); return n;
  }
  size_t getBytesLength(const char *) { return sizeof(record); }
  size_t getBytes(const char *, void *p, size_t n) {
    std::memcpy(p, record, n); return n;
  }
  bool getBool(const char *, bool fallback) { return fallback; }
  float getFloat(const char *, float fallback) { return fallback; }
  uint8_t getUChar(const char *, uint8_t fallback) { return fallback; }
};
