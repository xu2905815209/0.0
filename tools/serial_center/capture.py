from __future__ import annotations

import time
from pathlib import Path

import serial

from analyze import build_summary
from common import create_session_dir
from parse import parse_records
from plot import create_plots


def drain_serial(ser: serial.Serial, settle_s: float = 0.25) -> None:
    end_time = time.monotonic() + settle_s
    while time.monotonic() < end_time:
        waiting = ser.in_waiting
        if waiting:
            ser.read(waiting)
        time.sleep(0.02)


def _write_raw_log(raw_path: Path, records: list[dict], command_text: str, payload: str) -> None:
    with raw_path.open("w", encoding="utf-8") as handle:
        handle.write(f"# TX {command_text} -> {payload}\n")
        for record in records:
            handle.write(f"{record['host_time']:.6f}\t{record['line']}\n")


def capture_after_command(
    ser: serial.Serial,
    *,
    payload: str,
    command_text: str,
    label: str,
    expected_frame: str,
    min_duration: float,
    max_duration: float,
    min_expected_frames: int,
    echo=print,
) -> dict:
    session_dir = create_session_dir(label)
    raw_path = session_dir / "raw.log"
    frames_path = session_dir / "frames.csv"

    drain_serial(ser)

    wire_payload = payload if payload.endswith(("\r", "\n")) else payload + "\r\n"
    ser.write(wire_payload.encode("ascii"))
    ser.flush()

    records: list[dict] = []
    start_time = time.monotonic()
    expected_count = 0

    while True:
        line = ser.readline()
        if line:
            host_time = time.time()
            decoded = line.decode("utf-8", errors="replace").strip()
            records.append({"host_time": host_time, "line": decoded})
            if decoded:
                echo(f"[RX] {decoded}")
            if decoded.upper().startswith(expected_frame + ","):
                expected_count += 1

        elapsed = time.monotonic() - start_time
        if elapsed >= min_duration and expected_count >= min_expected_frames:
            break
        if elapsed >= max_duration:
            break

    _write_raw_log(raw_path, records, command_text, wire_payload)
    df = parse_records(records)
    df.to_csv(frames_path, index=False, encoding="utf-8")

    summary_text, stats = build_summary(
        df,
        session_dir=session_dir,
        label=label,
        command_text=command_text,
        expected_frame=expected_frame,
        min_duration=min_duration,
        max_duration=max_duration,
    )
    summary_path = session_dir / "summary.md"
    summary_path.write_text(summary_text, encoding="utf-8")
    plot_paths = create_plots(df, expected_frame, session_dir)

    return {
        "session_dir": session_dir,
        "raw_path": raw_path,
        "frames_path": frames_path,
        "summary_path": summary_path,
        "plot_paths": plot_paths,
        "stats": stats,
        "dataframe": df,
    }
