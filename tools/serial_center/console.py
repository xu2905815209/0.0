from __future__ import annotations

import sys
from dataclasses import dataclass

import serial

from capture import capture_after_command
from common import DEFAULT_BAUDRATE, DEFAULT_PORT, DEFAULT_TIMEOUT_S, ensure_base_dirs


@dataclass(frozen=True)
class CommandSpec:
    payload: str
    label: str
    expected_frame: str
    min_duration: float
    max_duration: float
    min_expected_frames: int


COMMANDS = {
    "mode line": CommandSpec("1", "line_on", "TUNE", 3.0, 8.0, 6),
    "mode cal": CommandSpec("5", "cal_mode", "CAL", 3.0, 8.0, 6),
    "stop": CommandSpec("2", "line_off", "STATE", 2.0, 4.0, 1),
    "status": CommandSpec("?", "status_check", "STATE", 1.5, 3.0, 1),
}


HELP_TEXT = """Commands:
  mode line      send '1' and capture TUNE frames
  mode cal       send '5' and capture CAL frames
  stop           send '2' and capture STATE frames
  status         send '?' and capture STATE frames
  capture auto   clear duration override
  capture <sec>  override next capture min duration
  help           show this help
  quit           exit
"""


def _print_result(result: dict) -> None:
    print(f"[OK] session: {result['session_dir']}")
    print(f"[OK] raw: {result['raw_path']}")
    print(f"[OK] frames: {result['frames_path']}")
    print(f"[OK] summary: {result['summary_path']}")
    if result["plot_paths"]:
        for plot_path in result["plot_paths"]:
            print(f"[OK] plot: {plot_path}")
    print(result["summary_path"].read_text(encoding="utf-8"))


def main() -> int:
    ensure_base_dirs()
    next_duration_override: float | None = None

    try:
        ser = serial.Serial(DEFAULT_PORT, DEFAULT_BAUDRATE, timeout=DEFAULT_TIMEOUT_S)
    except serial.SerialException as exc:
        print(f"[ERR] cannot open {DEFAULT_PORT}: {exc}")
        return 1

    print(f"[OK] connected to {DEFAULT_PORT} @ {DEFAULT_BAUDRATE}")
    print(HELP_TEXT)

    try:
        while True:
            try:
                user_input = input("serial-center> ").strip()
            except EOFError:
                print()
                break

            if not user_input:
                continue
            command = user_input.lower()

            if command in {"quit", "exit"}:
                break
            if command == "help":
                print(HELP_TEXT)
                continue
            if command == "capture auto":
                next_duration_override = None
                print("[OK] using command default capture windows")
                continue
            if command.startswith("capture "):
                try:
                    next_duration_override = float(command.split(maxsplit=1)[1])
                except ValueError:
                    print("[ERR] invalid capture duration")
                    continue
                print(f"[OK] next capture min duration set to {next_duration_override:.1f}s")
                continue

            spec = COMMANDS.get(command)
            if spec is None:
                print("[ERR] unsupported command")
                continue

            min_duration = next_duration_override if next_duration_override is not None else spec.min_duration
            next_duration_override = None
            print(f"[TX] {spec.payload} ({command})")
            result = capture_after_command(
                ser,
                payload=spec.payload,
                command_text=command,
                label=spec.label,
                expected_frame=spec.expected_frame,
                min_duration=min_duration,
                max_duration=max(spec.max_duration, min_duration),
                min_expected_frames=spec.min_expected_frames,
            )
            _print_result(result)
    finally:
        ser.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())

