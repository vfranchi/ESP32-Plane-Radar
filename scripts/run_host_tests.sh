#!/usr/bin/env bash
# Host tests: the pure logic that should not need a board to check.
# Compiles and runs every scripts/host_tests/test_*.cpp with include/ on the
# search path.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CXX="${CXX:-c++}"
OUT="$(mktemp -d)"
trap 'rm -rf "$OUT"' EXIT

for src in "$ROOT"/scripts/host_tests/test_*.cpp; do
  name="$(basename "$src" .cpp)"
  echo "== $name"
  "$CXX" -std=c++17 -Wall -Wextra -Werror -I"$ROOT/include" "$src" -o "$OUT/$name"
  "$OUT/$name"
done
