#include <cassert>
#include <cstring>
#include <cstdio>
#include "../src/pages/cal_storage.h"

struct Store {
  int opens = 0, closes = 0, failOpen = 0;
  bool failWrite = false, shortRead = false, corrupt = false, badLength = false;
  float saved[15] = {};
  bool begin(const char *, bool) { return ++opens != failOpen; }
  void end() { ++closes; }
  size_t putBytes(const char *, const void *data, size_t size) {
    if (failWrite) return 0;
    std::memcpy(saved, data, size); return size;
  }
  size_t getBytesLength(const char *) { return sizeof(saved) - (badLength ? 1 : 0); }
  size_t getBytes(const char *, void *out, size_t size) {
    std::memcpy(out, saved, size);
    if (corrupt) static_cast<float *>(out)[0] += 1;
    return size - (shortRead ? 1 : 0);
  }
};
int main() {
  float record[15];
  for (int i = 0; i < 15; ++i) record[i] = i * .25f;
  Store ok;
  assert(cal_storage::save_verified(ok, "imu_acc6", record));
  assert(ok.opens == 2 && ok.closes == 2);
  assert(std::memcmp(ok.saved, record, sizeof(record)) == 0);
  Store openFail; openFail.failOpen = 1;
  assert(!cal_storage::save_verified(openFail, "imu_acc6", record));
  assert(openFail.closes == 0);
  Store writeFail; writeFail.failWrite = true; writeFail.saved[0] = 42;
  assert(!cal_storage::save_verified(writeFail, "imu_acc6", record));
  assert(writeFail.saved[0] == 42 && writeFail.closes == 1);
  Store reopenFail; reopenFail.failOpen = 2;
  assert(!cal_storage::save_verified(reopenFail, "imu_acc6", record));
  assert(reopenFail.closes == 1);
  for (int fault = 0; fault < 3; ++fault) {
    Store s;
    s.shortRead = fault == 0; s.corrupt = fault == 1; s.badLength = fault == 2;
    assert(!cal_storage::save_verified(s, "imu_acc6", record));
    assert(s.closes == 2);
  }
  puts("PASS calibration store open/write/reopen/short-read/corruption/length failures");
}
