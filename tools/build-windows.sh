#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CLT_BIN='/c/ST/STM32CubeCLT_1.21.0/GNU-tools-for-STM32/bin'
MAKE_EXE='/c/ST/STM32CubeIDE_2.1.1/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.make.win32_2.2.100.202601091506/tools/bin/make.exe'
BUILD_OUT="${BUILD_OUT:-C:/Users/Public/stm32-f-flow-build}"

if [[ ! -x "$CLT_BIN/arm-none-eabi-gcc.exe" ]]; then
  echo "ERROR: STM32CubeCLT compiler not found: $CLT_BIN/arm-none-eabi-gcc.exe" >&2
  exit 2
fi
if [[ ! -x "$MAKE_EXE" ]]; then
  echo "ERROR: STM32CubeIDE GNU Make not found: $MAKE_EXE" >&2
  exit 2
fi
if [[ "$BUILD_OUT" == *" "* || "$BUILD_OUT" == *"'"* ]]; then
  echo "ERROR: BUILD_OUT must not contain spaces or apostrophes: $BUILD_OUT" >&2
  exit 2
fi

existing_gcc="$(command -v arm-none-eabi-gcc || true)"
echo "Existing PATH compiler: ${existing_gcc:-not found}"
echo "Selected compiler: $CLT_BIN/arm-none-eabi-gcc.exe"
echo "Selected make: $MAKE_EXE"

export PATH="$CLT_BIN:$PATH"

echo '--- tool versions ---'
arm-none-eabi-gcc --version | sed -n '1p'
"$MAKE_EXE" --version | sed -n '1p'

echo '--- project inputs ---'
for input in Makefile STM32G030F6PX_FLASH.ld Core/Startup/startup_stm32g030f6px.s; do
  if [[ ! -f "$ROOT/$input" ]]; then
    echo "ERROR: required build input missing: $input" >&2
    exit 2
  fi
  echo "OK: $input"
done

echo "--- clean build: $BUILD_OUT ---"
cd "$ROOT"
"$MAKE_EXE" -f Makefile BUILD_DIR="$BUILD_OUT" clean
"$MAKE_EXE" -f Makefile BUILD_DIR="$BUILD_OUT" -j8 all

echo '--- firmware size ---'
arm-none-eabi-size "$BUILD_OUT/STM32G030F6P6TR.elf"

echo '--- firmware hashes ---'
sha256sum \
  "$BUILD_OUT/STM32G030F6P6TR.elf" \
  "$BUILD_OUT/STM32G030F6P6TR.hex" \
  "$BUILD_OUT/STM32G030F6P6TR.bin"
