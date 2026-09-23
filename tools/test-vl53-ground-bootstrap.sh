#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CC="${CC:-gcc}"
OUTPUT="${TMPDIR:-/tmp}/vl53_ground_bootstrap_test.$$"
trap 'rm -f "$OUTPUT"' EXIT

"$CC" -std=c11 -Wall -Wextra -Werror \
  -I"$ROOT/Core/Inc/VL53L1X" \
  "$ROOT/tools/test_vl53_ground_bootstrap.c" \
  "$ROOT/Core/Inc/VL53L1X/vl53_ground_bootstrap.c" \
  -o "$OUTPUT"

"$OUTPUT"
