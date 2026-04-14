from __future__ import annotations

from typing import Iterable

import pandas as pd


NUMERIC_COLUMNS = [
    "host_time",
    "device_time_ms",
    "lf",
    "rf",
    "lr",
    "rr",
    "front",
    "rear",
    "yaw_error",
    "lat_error",
    "vy_cmd",
    "wz_cmd",
    "vy_meas",
    "wz_meas",
    "imu_yaw",
    "valid_mask",
    "raw_front",
    "raw_rear",
    "raw_lf",
    "raw_rf",
    "raw_lr",
    "raw_rr",
]


def _base_frame(raw_line: str, host_time: float) -> dict:
    return {
        "host_time": host_time,
        "raw_line": raw_line,
        "frame_type": None,
        "mode": None,
        "ack_code": None,
        "ack_text": None,
        "device_time_ms": None,
        "lf": None,
        "rf": None,
        "lr": None,
        "rr": None,
        "front": None,
        "rear": None,
        "yaw_error": None,
        "lat_error": None,
        "vy_cmd": None,
        "wz_cmd": None,
        "vy_meas": None,
        "wz_meas": None,
        "imu_yaw": None,
        "valid_mask": None,
        "raw_front": None,
        "raw_rear": None,
        "raw_lf": None,
        "raw_rf": None,
        "raw_lr": None,
        "raw_rr": None,
        "parse_error": None,
    }


def parse_line(raw_line: str, host_time: float) -> dict | None:
    line = raw_line.strip()
    if not line:
        return None

    parts = [part.strip() for part in line.split(",")]
    frame = _base_frame(line, host_time)
    frame_type = parts[0].upper()
    frame["frame_type"] = frame_type

    try:
        if frame_type == "ACK":
            frame["ack_code"] = parts[1] if len(parts) > 1 else None
            frame["ack_text"] = ",".join(parts[2:]) if len(parts) > 2 else None
        elif frame_type == "STATE" and len(parts) >= 17:
            frame["mode"] = parts[1]
            frame["device_time_ms"] = float(parts[2])
            frame["lf"] = float(parts[3])
            frame["rf"] = float(parts[4])
            frame["lr"] = float(parts[5])
            frame["rr"] = float(parts[6])
            frame["front"] = float(parts[7])
            frame["rear"] = float(parts[8])
            frame["yaw_error"] = float(parts[9])
            frame["lat_error"] = float(parts[10])
            frame["vy_cmd"] = float(parts[11])
            frame["wz_cmd"] = float(parts[12])
            frame["vy_meas"] = float(parts[13])
            frame["wz_meas"] = float(parts[14])
            frame["imu_yaw"] = float(parts[15])
            frame["valid_mask"] = int(parts[16]) if len(parts) > 16 else None
        elif frame_type == "TUNE" and len(parts) >= 13:
            frame["mode"] = "LINE"
            frame["device_time_ms"] = float(parts[1])
            frame["lf"] = float(parts[2])
            frame["rf"] = float(parts[3])
            frame["lr"] = float(parts[4])
            frame["rr"] = float(parts[5])
            frame["valid_mask"] = int(parts[6])
            frame["yaw_error"] = float(parts[7])
            frame["lat_error"] = float(parts[8])
            frame["vy_cmd"] = float(parts[9])
            frame["wz_cmd"] = float(parts[10])
            frame["vy_meas"] = float(parts[11])
            frame["wz_meas"] = float(parts[12])
            frame["imu_yaw"] = float(parts[13]) if len(parts) > 13 else None
        elif frame_type == "CAL" and len(parts) >= 13:
            frame["mode"] = "CAL"
            frame["device_time_ms"] = float(parts[1])
            frame["raw_front"] = float(parts[2])
            frame["raw_rear"] = float(parts[3])
            frame["raw_lf"] = float(parts[4])
            frame["raw_rf"] = float(parts[5])
            frame["raw_lr"] = float(parts[6])
            frame["raw_rr"] = float(parts[7])
            frame["valid_mask"] = int(parts[8])
            frame["lf"] = float(parts[9])
            frame["rf"] = float(parts[10])
            frame["lr"] = float(parts[11])
            frame["rr"] = float(parts[12]) if len(parts) > 12 else None
        else:
            frame["parse_error"] = "unsupported_or_short_frame"
    except (TypeError, ValueError) as exc:
        frame["parse_error"] = str(exc)

    return frame


def parse_records(records: Iterable[dict]) -> pd.DataFrame:
    frames = []
    last_device_time = None

    for record in records:
        frame = parse_line(record["line"], record["host_time"])
        if frame is None:
            continue

        device_time = frame.get("device_time_ms")
        if device_time is not None and last_device_time is not None and device_time < last_device_time:
            frame["parse_error"] = "device_time_rollback"
        if device_time is not None:
            last_device_time = device_time

        frames.append(frame)

    df = pd.DataFrame(frames)
    if df.empty:
        df = pd.DataFrame(columns=_base_frame("", 0.0).keys())

    for column in NUMERIC_COLUMNS:
        if column in df.columns:
            df[column] = pd.to_numeric(df[column], errors="coerce")

    return df
