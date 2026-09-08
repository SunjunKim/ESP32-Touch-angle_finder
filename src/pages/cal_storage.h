#pragma once
#include <cstring>

namespace cal_storage {
// Preferences commits a blob as one NVS item. Keep legacy keys read-only:
// interrupted saves must never assemble a mix of old/new individual fields.
// Store is injected so open/write/read failures can be tested on the host.
template <typename Store>
bool save_verified(Store &store, const char *ns, const float (&record)[15])
{
  if (!store.begin(ns, false)) return false;
  const bool written = store.putBytes("data_v3", record, sizeof(record)) == sizeof(record);
  store.end();
  if (!written || !store.begin(ns, true)) return false;
  float readback[15] = {};
  const bool verified = store.getBytesLength("data_v3") == sizeof(record) &&
      store.getBytes("data_v3", readback, sizeof(readback)) == sizeof(readback) &&
      std::memcmp(record, readback, sizeof(record)) == 0;
  store.end();
  return verified;
}
} // namespace cal_storage
