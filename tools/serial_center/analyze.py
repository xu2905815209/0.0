from __future__ import annotations

from pathlib import Path

import pandas as pd


FULL_SIDE_MASK = (1 << 2) | (1 << 3) | (1 << 4) | (1 << 5)
FULL_SENSOR_MASK = (1 << 6) - 1


def _table_text(df: pd.DataFrame) -> str:
    return "```text\n" + df.to_string() + "\n```"


def _first_expected_delay(df: pd.DataFrame, expected_frame: str) -> float | None:
    expected_df = df[df["frame_type"] == expected_frame]
    if expected_df.empty:
        return None
    return float(expected_df["host_time"].iloc[0] - df["host_time"].iloc[0])


def _valid_ratio(df: pd.DataFrame, expected_frame: str) -> float | None:
    expected_df = df[df["frame_type"] == expected_frame]
    if expected_df.empty or "valid_mask" not in expected_df:
        return None

    if expected_frame == "TUNE":
        return float((expected_df["valid_mask"] == FULL_SIDE_MASK).mean())
    if expected_frame == "CAL":
        return float((expected_df["valid_mask"] == FULL_SENSOR_MASK).mean())
    return float((expected_df["valid_mask"].fillna(0) > 0).mean())


def build_summary(
    df: pd.DataFrame,
    *,
    session_dir: Path,
    label: str,
    command_text: str,
    expected_frame: str,
    min_duration: float,
    max_duration: float,
) -> tuple[str, dict]:
    frame_counts = df["frame_type"].value_counts().to_dict() if not df.empty else {}
    expected_df = df[df["frame_type"] == expected_frame] if not df.empty else pd.DataFrame()
    delay_s = _first_expected_delay(df, expected_frame) if not df.empty else None
    valid_ratio = _valid_ratio(df, expected_frame)

    findings = []
    if df.empty:
        findings.append("未采集到任何串口数据。")
    elif expected_df.empty:
        findings.append(f"已发送 `{command_text}`，但未收到预期 `{expected_frame}` 连续帧，存在协议缺口。")
    else:
        findings.append(f"收到 `{len(expected_df)}` 条 `{expected_frame}` 帧。")
        if delay_s is not None:
            findings.append(f"首条 `{expected_frame}` 帧延迟约 `{delay_s:.2f}s`。")
        if valid_ratio is not None:
            findings.append(f"有效帧比例约 `{valid_ratio * 100.0:.1f}%`。")

    if not expected_df.empty and expected_frame == "TUNE":
        lat_mean = expected_df["lat_error"].mean()
        yaw_mean = expected_df["yaw_error"].mean()
        findings.append(f"`lat_error` 均值 `{lat_mean:.2f}`，`yaw_error` 均值 `{yaw_mean:.2f}`。")
        if abs(lat_mean) > 2.0:
            findings.append("横向误差存在明显偏置，后续优先核对居中目标或传感器零偏。")
        if abs(yaw_mean) > 2.0:
            findings.append("偏航误差存在明显偏置，后续优先核对侧向传感器映射或姿态目标。")

    if not expected_df.empty and expected_frame == "CAL":
        raw_cols = ["raw_front", "raw_rear", "raw_lf", "raw_rf", "raw_lr", "raw_rr"]
        available_cols = [column for column in raw_cols if column in expected_df.columns]
        if available_cols:
            raw_stats = expected_df[available_cols].agg(["mean", "std"]).round(2)
        else:
            raw_stats = pd.DataFrame()
    else:
        raw_stats = pd.DataFrame()

    summary_lines = [
        f"# Session `{session_dir.name}`",
        "",
        f"- Command: `{command_text}`",
        f"- Expected frame: `{expected_frame}`",
        f"- Capture window: `{min_duration:.1f}s` to `{max_duration:.1f}s`",
        f"- Frame counts: `{frame_counts}`",
        "",
        "## Findings",
    ]
    for finding in findings:
        summary_lines.append(f"- {finding}")

    if not expected_df.empty and expected_frame == "TUNE":
        tune_stats = expected_df[["lf", "rf", "lr", "rr", "lat_error", "yaw_error", "vy_cmd", "wz_cmd"]].agg(
            ["mean", "std", "min", "max"]
        ).round(3)
        summary_lines.extend(["", "## TUNE Stats", "", _table_text(tune_stats)])

    if not raw_stats.empty:
        summary_lines.extend(["", "## CAL Raw Stats", "", _table_text(raw_stats)])

    stats = {
        "frame_counts": frame_counts,
        "expected_count": int(len(expected_df)),
        "first_expected_delay_s": delay_s,
        "valid_ratio": valid_ratio,
        "summary_path": str(session_dir / "summary.md"),
    }
    return "\n".join(summary_lines) + "\n", stats
