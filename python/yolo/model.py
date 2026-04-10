from __future__ import annotations

import os
from pathlib import Path

LOCAL_CONFIG_ROOT = Path(__file__).resolve().parent / ".config"
LOCAL_CONFIG_ROOT.mkdir(parents=True, exist_ok=True)
os.environ.setdefault("YOLO_CONFIG_DIR", str(LOCAL_CONFIG_ROOT))

from ultralytics import YOLO, settings


DEFAULT_WEIGHTS = "yolo11n-obb.pt"
LOCAL_PROJECT_ROOT = Path(__file__).resolve().parent

settings.update(
    {
        "datasets_dir": str((LOCAL_PROJECT_ROOT.parent / "dataset").resolve()),
        "weights_dir": str((LOCAL_PROJECT_ROOT / "weights").resolve()),
        "runs_dir": str((LOCAL_PROJECT_ROOT / "runs").resolve()),
    }
)


def build_model(weights: str = DEFAULT_WEIGHTS) -> YOLO:
    return YOLO(weights)
