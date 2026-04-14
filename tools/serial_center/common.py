from __future__ import annotations

import re
from datetime import datetime
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[2]
TOOLS_DIR = ROOT_DIR / "tools" / "serial_center"
ANALYSIS_DIR = ROOT_DIR / "analysis" / "serial_center"
SESSIONS_DIR = ANALYSIS_DIR / "sessions"

DEFAULT_PORT = "COM13"
DEFAULT_BAUDRATE = 115200
DEFAULT_TIMEOUT_S = 0.2


def ensure_base_dirs() -> None:
    TOOLS_DIR.mkdir(parents=True, exist_ok=True)
    ANALYSIS_DIR.mkdir(parents=True, exist_ok=True)
    SESSIONS_DIR.mkdir(parents=True, exist_ok=True)


def sanitize_label(label: str) -> str:
    cleaned = re.sub(r"[^A-Za-z0-9_-]+", "_", label.strip())
    cleaned = cleaned.strip("_")
    return cleaned or "session"


def create_session_dir(label: str) -> Path:
    ensure_base_dirs()
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    session_dir = SESSIONS_DIR / f"{timestamp}_{sanitize_label(label)}"
    session_dir.mkdir(parents=True, exist_ok=False)
    return session_dir

