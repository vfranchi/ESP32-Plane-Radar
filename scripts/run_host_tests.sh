#!/usr/bin/env bash
# Compile and run host-side unit tests (pure C++, no Arduino).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUT="$(mktemp -d)"
g++ -std=c++17 -Wall -Wextra -Werror -I "$ROOT/include" \
    "$ROOT/scripts/host_tests/test_dead_reckoning.cpp" \
    "$ROOT/src/services/dead_reckoning.cpp" \
    -o "$OUT/test_dead_reckoning"
"$OUT/test_dead_reckoning"
