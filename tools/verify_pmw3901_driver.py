#!/usr/bin/env python
"""Static regression checks for the PMW3901 bring-up contract.

These checks complement, but do not replace, target hardware tests.
"""
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[1]
DRIVER = (ROOT / "Core/Src/PMW3901MB/pmw_3901.c").read_text(encoding="utf-8")
HEADER = (ROOT / "Core/Src/PMW3901MB/pmw_3901.h").read_text(encoding="utf-8")
MAIN = (ROOT / "Core/Src/main.c").read_text(encoding="utf-8")
IOC = (ROOT / "STM32G030F6P6TR.ioc").read_text(encoding="utf-8")
MAKEFILE = (ROOT / "Makefile").read_text(encoding="utf-8")

failures = []

def require(condition: bool, message: str) -> None:
    if not condition:
        failures.append(message)

require("PMW3901_STATUS_OK" in HEADER, "typed PMW3901 status codes are missing")
require(
    re.search(r"product_id\s*!=\s*0x49U?.*inverse_id\s*!=\s*0xB6U?", DRIVER, re.S),
    "initialization must reject either a bad product ID or inverse ID",
)
require(
    re.search(r"0x7F\s*,\s*0x00.*0x4D\s*,\s*0x11", DRIVER, re.S),
    "bank 0 selection must precede register 0x4D=0x11",
)
require(
    re.search(r"0x7F\s*,\s*0x14.*0x65\s*,\s*0x67.*0x7F\s*,\s*0x15.*0x48\s*,\s*0x48", DRIVER, re.S),
    "bank 14 value and bank 15 transition do not match the selected PX4 sequence",
)
require("PMW3901_ReadMotionBurst" in DRIVER, "production Motion Burst reader is missing")
require(
    re.search(r"HAL_SPI_Transmit\([^;]+0x16|burst_address\s*=\s*0x16", DRIVER, re.S),
    "Motion Burst address 0x16 is missing",
)
require(
    "PMW3901_DelayUs(150U)" in DRIVER,
    "Motion Burst must wait 150 us after sending the address",
)
require(
    "HAL_SPI_Receive" in DRIVER,
    "Motion Burst must read the payload after the address phase",
)
require(
    "SPI_POLARITY_LOW" in MAIN and "SPI_PHASE_1EDGE" in MAIN,
    "SPI1 must be configured as Mode 0",
)
require(
    "SPI_BAUDRATEPRESCALER_16" in MAIN,
    "bring-up SPI clock must use prescaler 16 (1 MHz at 16 MHz PCLK)",
)
require(
    "SPI1.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_16" in IOC
    and "SPI1.CalculateBaudRate=1.0 MBits/s" in IOC,
    "CubeMX IOC must match the 1 MHz bring-up configuration",
)

require(
    "PMW3901_Diagnostics" in HEADER and "PMW3901_GetDiagnostics" in HEADER,
    "raw Motion Burst diagnostics API is missing",
)
require(
    "pmw_reject_count" in DRIVER and "pmw_motion_count" in DRIVER
    and "pmw_squal_nonzero_count" in DRIVER,
    "PMW raw/reject diagnostic counters are missing",
)
require(
    "#define FLOW_PERIOD_MS" in MAIN and "20U" in MAIN,
    "50 Hz optical-flow scheduler is missing",
)
require(
    "#define DISTANCE_PERIOD_MS" in MAIN and "100U" in MAIN,
    "10 Hz distance scheduler is missing",
)
require(
    "#define HEARTBEAT_PERIOD_MS" in MAIN and "1000U" in MAIN,
    "1 Hz main-loop heartbeat scheduler is missing",
)
require(
    "#define PMW_DIAG_PERIOD_MS" in MAIN and "1000U" in MAIN
    and "PMW_SendRawDiagnostics" in MAIN,
    "1 Hz raw PMW diagnostics scheduler is missing",
)
callback = re.search(r"HAL_TIM_PeriodElapsedCallback\s*\([^)]*\)\s*\{(.*?)\n\}", MAIN, re.S)
require(
    callback is None or "send_heart_beat" not in callback.group(1),
    "heartbeat must not transmit UART from the timer ISR",
)
require(
    "HAL_TIM_Base_Start_IT(&htim14)" not in MAIN,
    "unused timer heartbeat interrupt must not be started",
)
require(
    "PMW3901_ReadDirectSnapshot" in HEADER and "last_direct" in HEADER,
    "V3 direct-register snapshot API is missing",
)
require(
    all(token in DRIVER for token in (
        "PMW3901_REG_MOTION", "PMW3901_REG_DELTA_X_L",
        "PMW3901_REG_DELTA_Y_L", "PMW3901_REG_SQUAL",
        "PMW3901_REG_SHUTTER_LOWER", "PMW3901_REG_OBSERVATION",
    )),
    "V3 direct-register map is incomplete",
)
require(
    "PMW3901_CaptureConfigReadback" in DRIVER
    and "config_bank0_4d" in HEADER
    and "config_bank14_65" in HEADER
    and "config_bank15_48" in HEADER,
    "V3 configuration readback is missing",
)
require(
    all(token in MAIN for token in (
        "PMW_SendConfigDiagnostics", "PMW_SendDirectDiagnostics",
        "PMW_SendBurstStatistics",
    )),
    "V3 diagnostic telemetry is missing",
)
require(
    "EXTRA_DEFS" in MAKEFILE and "$(EXTRA_DEFS)" in MAKEFILE,
    "V4 build must accept an isolated acquisition-mode define",
)
require(
    all(token in HEADER for token in (
        "PMW3901_ACQ_BURST", "PMW3901_ACQ_DIRECT",
        "PMW3901_ACQUISITION_MODE",
    )),
    "V4 acquisition mode definitions are missing",
)
require(
    re.search(
        r"void\s+pollingMotion\s*\([^)]*\).*?PMW3901_ACQUISITION_MODE.*?PMW3901_ReadDirectSnapshot.*?#else.*?PMW3901_ReadMotionBurst",
        DRIVER,
        re.S,
    ) is not None,
    "pollingMotion must compile to exactly one V4 acquisition path",
)
raw_diag = re.search(r"static void PMW_SendRawDiagnostics\s*\(void\)\s*\{(.*?)\n\}", MAIN, re.S)
require(
    raw_diag is not None and "PMW3901_ReadDirectSnapshot" not in raw_diag.group(1),
    "1 Hz telemetry must not consume a direct motion sample",
)
require(
    "direct_motion_count" in HEADER and "direct_squal_nonzero_count" in HEADER
    and "PMW_SendDirectStatistics" in MAIN,
    "V4 direct-mode cumulative diagnostics are missing",
)
require(
    "PMW mode burst" in MAIN and "PMW mode direct" in MAIN,
    "V4 firmware must identify its isolated acquisition mode at startup",
)

if failures:
    print("PMW3901_STATIC_CHECK_FAIL")
    for failure in failures:
        print(f"- {failure}")
    sys.exit(1)

print("PMW3901_STATIC_CHECK_PASS")
