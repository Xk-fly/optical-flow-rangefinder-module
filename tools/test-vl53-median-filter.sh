#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CC="${CC:-gcc}"
OUTPUT="${TMPDIR:-/tmp}/vl53_median_filter_test.$$"
trap 'rm -f "$OUTPUT"' EXIT

"$CC" -std=c11 -Wall -Wextra -Werror \
  -I"$ROOT/Core/Inc/VL53L1X" \
  "$ROOT/tools/test_vl53_median_filter.c" \
  "$ROOT/Core/Inc/VL53L1X/vl53_median_filter.c" \
  -o "$OUTPUT"

"$OUTPUT"
