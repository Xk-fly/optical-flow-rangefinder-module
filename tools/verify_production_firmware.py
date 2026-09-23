#!/usr/bin/env python
"""Static production-firmware regression checks.

These checks verify source-level interface contracts; target hardware testing is still required.
"""
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
ADAPTER = (ROOT / "Core/Src/Adapter/massage_adapter.c").read_text(encoding="utf-8")
MAIN = (ROOT / "Core/Src/main.c").read_text(encoding="utf-8")
DRIVER = (ROOT / "Core/Src/PMW3901MB/pmw_3901.c").read_text(encoding="utf-8")
VL53 = (ROOT / "Core/Inc/VL53L1X/vl53l1x.c").read_text(encoding="utf-8")
VL53_HEADER = (ROOT / "Core/Inc/VL53L1X/vl53l1x.h").read_text(encoding="utf-8")
VL53_TYPES = (ROOT / "Core/Inc/VL53L1X/vl53_types.h").read_text(encoding="utf-8")
BOOTSTRAP = (ROOT / "Core/Inc/VL53L1X/vl53_ground_bootstrap.c").read_text(encoding="utf-8")
BOOTSTRAP_HEADER = (ROOT / "Core/Inc/VL53L1X/vl53_ground_bootstrap.h").read_text(encoding="utf-8")


class ProductionFirmwareContractTests(unittest.TestCase):
    def test_optical_flow_packet_is_initialized_and_preserves_wire_axes(self):
        self.assertIn("mavlink_optical_flow_t packet = {0};", ADAPTER)
        self.assertIn("packet.flow_x = flow_x;", ADAPTER)
        self.assertIn("packet.flow_y = flow_y;", ADAPTER)
        self.assertNotIn("packet.flow_x = -flow_x;", ADAPTER)
        self.assertNotIn("packet.flow_y = -flow_y;", ADAPTER)
        self.assertIn("send_optical_flow(&huart2, x, y, quality", MAIN)
        self.assertNotIn("send_optical_flow(&huart2, -x, -y", MAIN)

    def test_first_successful_burst_after_each_init_is_discarded(self):
        self.assertIn("pmw_discard_next_burst = 1U;", DRIVER)
        self.assertIn("if (pmw_discard_next_burst != 0U)", DRIVER)
        self.assertIn("pmw_discard_next_burst = 0U;", DRIVER)
        self.assertIn("PMW3901_STATUS_DISCARDED", DRIVER)

    def test_range_read_is_nonblocking_and_classifies_failures(self):
        self.assertIn("VL53ReadResult vl53_GetDistance(uint16_t *distance_mm)", VL53)
        self.assertNotIn("while (dataReady == 0)", VL53)
        self.assertIn("VL53L1X_ClearInterrupt", VL53)
        self.assertIn("VL53L1X_GetRangeStatus", VL53)
        self.assertIn("VL53_RANGE_STATUS_VALID", VL53)
        self.assertIn("VL53_RANGE_STATUS_MIN_RANGE_CLIPPED", VL53)
        self.assertIn("VL53_READ_NOT_READY", VL53)
        self.assertIn("VL53_READ_TOO_CLOSE", VL53)
        self.assertIn("VL53_READ_TOO_FAR", VL53)
        self.assertIn("VL53_READ_ERROR", VL53)
        self.assertIn("VL53ReadResult read_result = vl53_GetDistance(&new_distance);", MAIN)

    def test_range_filter_and_optical_flow_share_one_freshness_timeout(self):
        self.assertIn("#define VL53_DISTANCE_FRESH_TIMEOUT_MS 300U", VL53_HEADER)
        self.assertIn("#define VL53_REAL_MIN_DISTANCE_MM 50U", VL53_TYPES)
        self.assertIn("#define VL53_REAL_MAX_DISTANCE_MM 3600U", VL53_TYPES)
        self.assertIn("#define VL53_RANGE_STATUS_VALID 0U", VL53_TYPES)
        self.assertIn("#define VL53_RANGE_STATUS_MIN_RANGE_CLIPPED 3U", VL53_TYPES)
        self.assertIn("VL53Median3Filter_Update", VL53)
        self.assertIn("VL53_DISTANCE_FRESH_TIMEOUT_MS", VL53)
        self.assertIn("VL53_DISTANCE_FRESH_TIMEOUT_MS", MAIN)
        self.assertNotIn("#define DISTANCE_FRESH_TIMEOUT_MS", MAIN)
        self.assertIn("uint8_t has_distance = 0U;", MAIN)
        self.assertIn("(has_distance != 0U)", MAIN)
        self.assertIn("last_distance_update_ms = HAL_GetTick();", MAIN)
        self.assertNotIn("last_distance_update_ms = now;", MAIN)

    def test_ground_bootstrap_is_near_field_only_and_one_way(self):
        self.assertIn("#define VL53_GROUND_BOOTSTRAP_DISTANCE_MM 50U", BOOTSTRAP_HEADER)
        self.assertIn("#define VL53_GROUND_BOOTSTRAP_EXIT_MM 60U", BOOTSTRAP_HEADER)
        self.assertIn("#define VL53_GROUND_BOOTSTRAP_CONFIRM_COUNT 2U", BOOTSTRAP_HEADER)
        self.assertIn("case VL53_READ_TOO_CLOSE:", BOOTSTRAP)
        self.assertIn("case VL53_READ_ERROR:", BOOTSTRAP)
        self.assertIn("VL53_RANGE_REAL_LOCKED", BOOTSTRAP)
        self.assertIn("VL53GroundBootstrap_Update(&range_bootstrap", MAIN)
        self.assertIn('"VL53 ground bootstrap 5cm"', MAIN)
        self.assertIn('"VL53 real range locked"', MAIN)
        real_mode = BOOTSTRAP.split("if (state->mode == VL53_RANGE_REAL_LOCKED)", 1)[1]
        self.assertIn("if (result != VL53_READ_VALID)", real_mode)
        self.assertIn("return 0U;", real_mode)

    def test_real_range_is_encoded_in_both_mavlink_messages(self):
        self.assertIn("uint8_t distance_valid", ADAPTER)
        self.assertIn("distance_valid ? ((float)distance_mm / 1000.0f) : -1.0f", ADAPTER)
        self.assertIn("mavlink_distance_sensor_t mdst = {0};", ADAPTER)
        self.assertIn("mdst.min_distance = 5;", ADAPTER)
        self.assertIn("mdst.max_distance = 360;", ADAPTER)

    def test_production_scheduler_prioritizes_flow_and_disables_periodic_diagnostics(self):
        self.assertIn("#define PMW3901_ENABLE_PERIODIC_DIAGNOSTICS 0U", MAIN)
        flow_schedule = MAIN.index("if (now - last_flow_ms >= FLOW_PERIOD_MS)")
        distance_schedule = MAIN.index("if (now - last_distance_ms >= DISTANCE_PERIOD_MS)")
        heartbeat_schedule = MAIN.index("if (now - last_heartbeat_ms >= HEARTBEAT_PERIOD_MS)")
        self.assertLess(flow_schedule, distance_schedule)
        self.assertLess(distance_schedule, heartbeat_schedule)
        self.assertIn("#if PMW3901_ENABLE_PERIODIC_DIAGNOSTICS", MAIN)


if __name__ == "__main__":
    unittest.main(verbosity=2)
