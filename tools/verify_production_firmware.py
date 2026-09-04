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

    def test_range_read_is_nonblocking_and_clears_the_sensor_interrupt(self):
        self.assertIn("uint8_t vl53_GetDistance(uint16_t *distance_mm)", VL53)
        self.assertNotIn("while (dataReady == 0)", VL53)
        self.assertIn("VL53L1X_ClearInterrupt", VL53)
        self.assertIn("vl53_GetDistance(&distance)", MAIN)
        self.assertIn("if (distance_valid != 0U)", MAIN)

    def test_real_range_is_encoded_in_both_mavlink_messages(self):
        self.assertIn("uint8_t distance_valid", ADAPTER)
        self.assertIn("distance_valid ? ((float)distance_mm / 1000.0f) : -1.0f", ADAPTER)
        self.assertIn("mavlink_distance_sensor_t mdst = {0};", ADAPTER)

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
