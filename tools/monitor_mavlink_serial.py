#!/usr/bin/env python
"""Listen to this module's MAVLink v1 UART stream and write an analysis log.

The parser deliberately supports the four messages emitted by this firmware:
HEARTBEAT, OPTICAL_FLOW, DISTANCE_SENSOR, and STATUSTEXT.
"""
from __future__ import annotations

import argparse
from collections import Counter
from dataclasses import dataclass
from datetime import datetime
import json
from pathlib import Path
import re
import struct
import sys
import time
from typing import Any

MAVLINK_STX_V1 = 0xFE
MAVLINK_CRC_EXTRA = {
    0: 50,    # HEARTBEAT
    100: 175, # OPTICAL_FLOW
    132: 85,  # DISTANCE_SENSOR
    253: 83,  # STATUSTEXT
}
MESSAGE_NAMES = {
    0: "HEARTBEAT",
    100: "OPTICAL_FLOW",
    132: "DISTANCE_SENSOR",
    253: "STATUSTEXT",
}
FIRMWARE_VARIANTS = {
    "v4-burst": "8aeb7d27cfcecd791bf56710d7b755c638a7e56c4c890f6ac3f7d11d3cca695f",
    "v4-direct": "b2ac668b9e09182fcc1133a6acf500881c50c124bf3c144d10a72105745a0053",
    "unknown": None,
}

PMW_DIAGNOSTIC_PATTERN = re.compile(
    r"^P S(?P<status>\d+) M(?P<motion>[0-9A-Fa-f]{2}) "
    r"O(?P<observation>[0-9A-Fa-f]{2}) X(?P<raw_x>-?\d+) "
    r"Y(?P<raw_y>-?\d+) Q(?P<squal>\d+) "
    r"H(?P<shutter>[0-9A-Fa-f]{4}) E(?P<spi_errors>\d+) R(?P<rejects>\d+)$"
)
PMW_DIRECT_PATTERN = re.compile(
    r"^D S(?P<status>\d+) M(?P<motion>[0-9A-Fa-f]{2}) "
    r"O(?P<observation>[0-9A-Fa-f]{2}) X(?P<raw_x>-?\d+) "
    r"Y(?P<raw_y>-?\d+) Q(?P<squal>\d+) "
    r"H(?P<shutter>[0-9A-Fa-f]{4}) E(?P<spi_errors>\d+)$"
)
PMW_BURST_STATISTICS_PATTERN = re.compile(
    r"^R U(?P<raw_sum>\d+) A(?P<raw_max>\d+) I(?P<raw_min>\d+) "
    r"C(?P<sample_count>\d+) V(?P<motion_count>\d+) "
    r"Q(?P<squal_nonzero_count>\d+)$"
)
PMW_DIRECT_STATISTICS_PATTERN = re.compile(
    r"^T C(?P<sample_count>\d+) V(?P<motion_count>\d+) "
    r"Q(?P<squal_nonzero_count>\d+)$"
)
PMW_CONFIG_PATTERN = re.compile(
    r"^C S(?P<status>\d+) D(?P<bank0_4d>[0-9A-Fa-f]{2}) "
    r"A(?P<bank14_65>[0-9A-Fa-f]{2}) F(?P<bank15_48>[0-9A-Fa-f]{2}) "
    r"N(?P<mismatches>\d+)$"
)


@dataclass(frozen=True)
class MavlinkV1Frame:
    sequence: int
    system_id: int
    component_id: int
    message_id: int
    payload: bytes


def x25_crc(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        tmp = value ^ (crc & 0xFF)
        tmp ^= (tmp << 4) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


class MavlinkV1Parser:
    def __init__(self) -> None:
        self.buffer = bytearray()
        self.crc_errors = 0
        self.unknown_messages = 0
        self.discarded_bytes = 0

    def feed(self, data: bytes) -> list[MavlinkV1Frame]:
        self.buffer.extend(data)
        frames: list[MavlinkV1Frame] = []

        while True:
            start = self.buffer.find(bytes((MAVLINK_STX_V1,)))
            if start < 0:
                self.discarded_bytes += len(self.buffer)
                self.buffer.clear()
                break
            if start > 0:
                self.discarded_bytes += start
                del self.buffer[:start]
            if len(self.buffer) < 8:
                break

            payload_length = self.buffer[1]
            frame_length = payload_length + 8
            if len(self.buffer) < frame_length:
                break

            sequence = self.buffer[2]
            system_id = self.buffer[3]
            component_id = self.buffer[4]
            message_id = self.buffer[5]
            payload_end = 6 + payload_length
            payload = bytes(self.buffer[6:payload_end])
            received_crc = struct.unpack_from("<H", self.buffer, payload_end)[0]
            crc_extra = MAVLINK_CRC_EXTRA.get(message_id)

            if crc_extra is None:
                self.unknown_messages += 1
                del self.buffer[:frame_length]
                continue

            calculated_crc = x25_crc(bytes(self.buffer[1:payload_end]) + bytes((crc_extra,)))
            if received_crc != calculated_crc:
                self.crc_errors += 1
                # Drop only STX, then scan again so a valid following frame is retained.
                del self.buffer[0]
                continue

            frames.append(MavlinkV1Frame(sequence, system_id, component_id, message_id, payload))
            del self.buffer[:frame_length]

        return frames


def parse_pmw_diagnostic_text(text: str) -> dict[str, int] | None:
    match = PMW_DIAGNOSTIC_PATTERN.fullmatch(text)
    if match is None:
        return None
    values = match.groupdict()
    return {
        "status": int(values["status"]),
        "motion": int(values["motion"], 16),
        "observation": int(values["observation"], 16),
        "raw_x": int(values["raw_x"]),
        "raw_y": int(values["raw_y"]),
        "squal": int(values["squal"]),
        "shutter": int(values["shutter"], 16),
        "spi_errors": int(values["spi_errors"]),
        "rejects": int(values["rejects"]),
    }


def parse_pmw_direct_text(text: str) -> dict[str, int] | None:
    match = PMW_DIRECT_PATTERN.fullmatch(text)
    if match is None:
        return None
    values = match.groupdict()
    return {
        "status": int(values["status"]),
        "motion": int(values["motion"], 16),
        "observation": int(values["observation"], 16),
        "raw_x": int(values["raw_x"]),
        "raw_y": int(values["raw_y"]),
        "squal": int(values["squal"]),
        "shutter": int(values["shutter"], 16),
        "spi_errors": int(values["spi_errors"]),
    }


def parse_pmw_burst_statistics_text(text: str) -> dict[str, int] | None:
    match = PMW_BURST_STATISTICS_PATTERN.fullmatch(text)
    if match is None:
        return None
    return {key: int(value) for key, value in match.groupdict().items()}


def parse_pmw_direct_statistics_text(text: str) -> dict[str, int] | None:
    match = PMW_DIRECT_STATISTICS_PATTERN.fullmatch(text)
    if match is None:
        return None
    return {key: int(value) for key, value in match.groupdict().items()}


def parse_pmw_config_text(text: str) -> dict[str, int] | None:
    match = PMW_CONFIG_PATTERN.fullmatch(text)
    if match is None:
        return None
    values = match.groupdict()
    return {
        "status": int(values["status"]),
        "bank0_4d": int(values["bank0_4d"], 16),
        "bank14_65": int(values["bank14_65"], 16),
        "bank15_48": int(values["bank15_48"], 16),
        "mismatches": int(values["mismatches"]),
    }


def decode_frame(frame: MavlinkV1Frame) -> dict[str, Any]:
    payload = frame.payload
    result: dict[str, Any] = {
        "name": MESSAGE_NAMES[frame.message_id],
        "message_id": frame.message_id,
        "sequence": frame.sequence,
        "system_id": frame.system_id,
        "component_id": frame.component_id,
    }

    if frame.message_id == 0 and len(payload) == 9:
        custom_mode, mav_type, autopilot, base_mode, system_status, mavlink_version = struct.unpack("<IBBBBB", payload)
        result.update(custom_mode=custom_mode, type=mav_type, autopilot=autopilot,
                      base_mode=base_mode, system_status=system_status,
                      mavlink_version=mavlink_version)
    elif frame.message_id == 100 and len(payload) == 26:
        values = struct.unpack("<QfffhhBB", payload)
        result.update(time_usec=values[0], flow_comp_m_x=values[1], flow_comp_m_y=values[2],
                      ground_distance=values[3], flow_x=values[4], flow_y=values[5],
                      sensor_id=values[6], quality=values[7])
    elif frame.message_id == 132 and len(payload) == 14:
        values = struct.unpack("<IHHHBBBB", payload)
        result.update(time_boot_ms=values[0], min_distance_cm=values[1], max_distance_cm=values[2],
                      current_distance_cm=values[3], sensor_type=values[4], sensor_id=values[5],
                      orientation=values[6], covariance=values[7])
    elif frame.message_id == 253 and len(payload) == 51:
        severity = payload[0]
        text = payload[1:51].split(b"\0", 1)[0].decode("utf-8", errors="replace")
        result.update(severity=severity, text=text)
        pmw_diagnostic = parse_pmw_diagnostic_text(text)
        if pmw_diagnostic is not None:
            result["pmw_diagnostic"] = pmw_diagnostic
        pmw_direct = parse_pmw_direct_text(text)
        if pmw_direct is not None:
            result["pmw_direct"] = pmw_direct
        pmw_burst_statistics = parse_pmw_burst_statistics_text(text)
        if pmw_burst_statistics is not None:
            result["pmw_burst_statistics"] = pmw_burst_statistics
        pmw_direct_statistics = parse_pmw_direct_statistics_text(text)
        if pmw_direct_statistics is not None:
            result["pmw_direct_statistics"] = pmw_direct_statistics
        pmw_config = parse_pmw_config_text(text)
        if pmw_config is not None:
            result["pmw_config"] = pmw_config
    else:
        result.update(decode_error=f"unexpected payload length {len(payload)}")
    return result


def format_for_console(record: dict[str, Any]) -> str:
    name = record["name"]
    if name == "STATUSTEXT":
        return f"STATUSTEXT sev={record['severity']} {record['text']}"
    if name == "OPTICAL_FLOW":
        return (f"OPTICAL_FLOW x={record['flow_x']:6d} y={record['flow_y']:6d} "
                f"quality={record['quality']:3d} ground={record['ground_distance']:.3f}m")
    if name == "DISTANCE_SENSOR":
        return (f"DISTANCE_SENSOR distance={record['current_distance_cm']:4d}cm "
                f"range={record['min_distance_cm']}-{record['max_distance_cm']}cm")
    if name == "HEARTBEAT":
        return (f"HEARTBEAT sys={record['system_id']} comp={record['component_id']} "
                f"status={record['system_status']}")
    return name


def list_ports() -> int:
    try:
        from serial.tools import list_ports as serial_list_ports
    except ImportError:
        print_pyserial_help()
        return 2

    ports = list(serial_list_ports.comports())
    if not ports:
        print("没有检测到串口。请连接USB转串口后重试。")
        return 0
    for port in ports:
        print(f"{port.device:8s}  {port.description}  [{port.hwid}]")
    return 0


def print_pyserial_help() -> None:
    print("当前Python环境没有pyserial。为避免修改全局环境，请使用uv隔离运行：", file=sys.stderr)
    print("  uv run --with pyserial python tools/monitor_mavlink_serial.py --list", file=sys.stderr)
    print("  uv run --with pyserial python tools/monitor_mavlink_serial.py --port COM5", file=sys.stderr)


def monitor(args: argparse.Namespace) -> int:
    try:
        import serial
    except ImportError:
        print_pyserial_help()
        return 2

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    safe_port = args.port.replace("\\", "_").replace("/", "_").replace(":", "_")
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    output_path = output_dir / f"{stamp}-{safe_port}-mavlink.jsonl"

    parser = MavlinkV1Parser()
    message_counts: Counter[str] = Counter()
    start_monotonic = time.monotonic()
    last_display: dict[str, float] = {}

    print(f"正在监听 {args.port} @ {args.baud} 8N1")
    print(f"日志文件：{output_path.resolve()}")
    print("请在监听开始后重新上电或复位模块，以捕获 PMW ID/RET 启动消息。")
    print("按 Ctrl+C 停止并写入会话汇总。")

    try:
        with serial.Serial(args.port, args.baud, timeout=0.2) as port, output_path.open("w", encoding="utf-8") as log:
            header = {
                "record_type": "session_start",
                "local_time": datetime.now().astimezone().isoformat(timespec="milliseconds"),
                "port": args.port,
                "baud": args.baud,
                "firmware_variant": args.firmware_variant,
                "firmware_hex_sha256": FIRMWARE_VARIANTS[args.firmware_variant],
            }
            log.write(json.dumps(header, ensure_ascii=False) + "\n")
            log.flush()

            while True:
                chunk = port.read(port.in_waiting or 1)
                for frame in parser.feed(chunk):
                    decoded = decode_frame(frame)
                    name = decoded["name"]
                    message_counts[name] += 1
                    decoded["record_type"] = "mavlink_message"
                    decoded["local_time"] = datetime.now().astimezone().isoformat(timespec="milliseconds")
                    log.write(json.dumps(decoded, ensure_ascii=False, separators=(",", ":")) + "\n")
                    log.flush()

                    now = time.monotonic()
                    interval = 0.0 if name == "STATUSTEXT" else args.display_interval
                    if now - last_display.get(name, 0.0) >= interval:
                        print(f"{decoded['local_time']}  {format_for_console(decoded)}")
                        last_display[name] = now

                if args.duration > 0 and time.monotonic() - start_monotonic >= args.duration:
                    break
    except KeyboardInterrupt:
        print("\n收到停止指令。")
    except serial.SerialException as error:
        print(f"串口打开或读取失败：{error}", file=sys.stderr)
        print("请检查COM口是否被Mission Planner/串口助手占用，以及USB转串口驱动。", file=sys.stderr)
        return 3
    finally:
        summary = {
            "record_type": "session_summary",
            "local_time": datetime.now().astimezone().isoformat(timespec="milliseconds"),
            "duration_seconds": round(time.monotonic() - start_monotonic, 3),
            "message_counts": dict(message_counts),
            "crc_errors": parser.crc_errors,
            "unknown_messages": parser.unknown_messages,
            "discarded_bytes": parser.discarded_bytes,
        }
        try:
            with output_path.open("a", encoding="utf-8") as log:
                log.write(json.dumps(summary, ensure_ascii=False) + "\n")
        except OSError:
            pass
        print("会话汇总：" + json.dumps(summary, ensure_ascii=False))
        print(f"日志已保存：{output_path.resolve()}")

    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="监听并记录本项目STM32模块的MAVLink v1串口数据")
    parser.add_argument("--list", action="store_true", help="列出串口后退出")
    parser.add_argument("--port", help="串口，例如 COM5")
    parser.add_argument("--baud", type=int, default=115200, help="波特率，默认115200")
    parser.add_argument("--firmware-variant", choices=tuple(FIRMWARE_VARIANTS), default="unknown",
                        help="本次烧录版本：v4-burst或v4-direct")
    parser.add_argument("--output-dir", default="logs/serial", help="日志输出目录")
    parser.add_argument("--display-interval", type=float, default=0.2,
                        help="同类高频消息的终端显示间隔，日志仍记录每一帧")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="自动停止秒数；0表示直到Ctrl+C")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.list:
        return list_ports()
    if not args.port:
        print("必须指定 --port COMx，或使用 --list 查看可用串口。", file=sys.stderr)
        return 2
    return monitor(args)


if __name__ == "__main__":
    raise SystemExit(main())
