#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
test_dir=$(mktemp -d "${TMPDIR:-/tmp}/axicube-tests.XXXXXX")
# Keep binaries outside the Arduino source tree. The printed directory is safe
# to remove after inspection; no wildcard cleanup is performed here.
printf 'Test binaries: %s\n' "$test_dir"
for name in contact_rel accel_rel stationary_rel_tracker; do
  "${CXX:-clang++}" -std=c++17 -Wall -Wextra -Werror \
    -fsanitize=address,undefined "tests/${name}_test.cpp" -o "$test_dir/$name"
  "$test_dir/$name"
done
node tests/reference_placement_test.cjs
node tests/spine_axis_test.cjs
node tests/plane_normal_tracker_test.cjs
