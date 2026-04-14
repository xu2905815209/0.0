from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import pandas as pd


def _time_axis(df: pd.DataFrame) -> pd.Series:
    return df["host_time"] - df["host_time"].iloc[0]


def create_plots(df: pd.DataFrame, expected_frame: str, session_dir: Path) -> list[Path]:
    plot_paths: list[Path] = []
    if df.empty:
        return plot_paths

    expected_df = df[df["frame_type"] == expected_frame].copy()
    if expected_df.empty:
        return plot_paths

    expected_df.sort_values("host_time", inplace=True)
    time_axis = _time_axis(expected_df)

    if expected_frame == "TUNE":
        fig, ax = plt.subplots(figsize=(11, 5))
        for column in ["lf", "rf", "lr", "rr"]:
            ax.plot(time_axis, expected_df[column], label=column.upper())
        ax.set_title("Side Distances")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Distance (cm)")
        ax.grid(True, alpha=0.3)
        ax.legend()
        path = session_dir / "tune_distances.png"
        fig.tight_layout()
        fig.savefig(path, dpi=150)
        plt.close(fig)
        plot_paths.append(path)

        fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
        axes[0].plot(time_axis, expected_df["lat_error"], label="lat_error")
        axes[0].plot(time_axis, expected_df["yaw_error"], label="yaw_error")
        axes[0].set_ylabel("Error")
        axes[0].grid(True, alpha=0.3)
        axes[0].legend()

        axes[1].plot(time_axis, expected_df["vy_cmd"], label="vy_cmd")
        axes[1].plot(time_axis, expected_df["wz_cmd"], label="wz_cmd")
        axes[1].plot(time_axis, expected_df["vy_meas"], label="vy_meas")
        axes[1].plot(time_axis, expected_df["wz_meas"], label="wz_meas")
        axes[1].set_xlabel("Time (s)")
        axes[1].set_ylabel("Command / Measure")
        axes[1].grid(True, alpha=0.3)
        axes[1].legend()
        path = session_dir / "tune_control.png"
        fig.tight_layout()
        fig.savefig(path, dpi=150)
        plt.close(fig)
        plot_paths.append(path)

    if expected_frame == "CAL":
        fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
        for column in ["raw_front", "raw_rear", "raw_lf", "raw_rf", "raw_lr", "raw_rr"]:
            axes[0].plot(time_axis, expected_df[column], label=column)
        axes[0].set_ylabel("Raw distance")
        axes[0].grid(True, alpha=0.3)
        axes[0].legend(ncol=3)

        for column in ["lf", "rf", "lr", "rr"]:
            axes[1].plot(time_axis, expected_df[column], label=column.upper())
        axes[1].set_xlabel("Time (s)")
        axes[1].set_ylabel("Filtered distance (cm)")
        axes[1].grid(True, alpha=0.3)
        axes[1].legend()
        path = session_dir / "calibration.png"
        fig.tight_layout()
        fig.savefig(path, dpi=150)
        plt.close(fig)
        plot_paths.append(path)

    return plot_paths

