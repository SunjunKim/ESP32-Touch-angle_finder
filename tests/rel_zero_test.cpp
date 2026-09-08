#include <cassert>
#include <cmath>
#include <cstdio>
#include "../src/pages/accel_rel.h"

static void feed(accel_rel::ContactRel &f, float degrees, int count = 80) {
  const float r = degrees / accel_rel::kDeg;
  for (int i = 0; i < count; ++i) assert(f.update(std::sin(r), 0, std::cos(r), .008f));
}
int main() {
  accel_rel::ContactRel f;
  feed(f, 10, 1); assert(!f.capture());
  feed(f, 10, 50); assert(!f.capture());
  feed(f, 10, 30); assert(f.capture());
  const float old = f.reference;
  // Sustained movement, including slow drift, must not overwrite the old zero.
  for (int i = 0; i < 400; ++i) {
    feed(f, 11 + i * .02f, 1);
    assert(!f.capture());
  }
  assert(f.reference == old && f.has_reference);
  assert(f.update(0, 0, 1.2f, .008f)); assert(!f.capture());
  feed(f, 20); assert(f.capture());
  // Old stable history cannot survive invalid data or a sensor gap.
  assert(!f.update(NAN, 0, 0, .008f)); assert(!f.capture());
  feed(f, 20, 1); assert(!f.capture());
  feed(f, 20); assert(f.capture());
  assert(f.update(0, 0, 1, 1)); assert(!f.capture());
  feed(f, 0); assert(f.capture());
  // Average the window, not the final jittered sample (alternating +/-0.1 deg).
  for (int i = 0; i < 80; ++i) feed(f, 10 + (i % 2 ? .1f : -.1f), 1);
  assert(f.capture());
  assert(std::fabs(f.reference - 10) < .005f);
  // A later steady pose replaces the old window; no lifetime averaging.
  feed(f, 30, 160); assert(f.capture());
  assert(std::fabs(f.reference - 30) < .001f);
  // Timestamp-derived 8 ms / 16 ms streams both allow capture after settling.
  accel_rel::ContactRel slower;
  for (int i = 0; i < 40; ++i) slower.update(0, 0, 1, .016f);
  assert(slower.capture());
  puts("PASS stationary zero timing / moving rejection / old-zero preservation / gaps / mean / recent window");
}
