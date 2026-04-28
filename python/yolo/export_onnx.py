from __future__ import annotations

import argparse
from pathlib import Path

import os

LOCAL_CONFIG_ROOT = Path(__file__).resolve().parent / ".config"
LOCAL_CONFIG_ROOT.mkdir(parents=True, exist_ok=True)
os.environ.setdefault("YOLO_CONFIG_DIR", str(LOCAL_CONFIG_ROOT))

from ultralytics import YOLO

DEFAULT_WEIGHTS = Path(__file__).resolve().parent / "runs/obb/runs/dart_yolov8_obb/weights/best.pt"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Export a YOLO OBB model to ONNX format.")
    parser.add_argument(
        "--weights",
        type=Path,
        default=DEFAULT_WEIGHTS,
        help="Path to the .pt checkpoint (default: %(default)s)",
    )
    parser.add_argument(
        "--imgsz",
        type=int,
        default=640,
        help="Input image size (default: 640)",
    )
    parser.add_argument(
        "--opset",
        type=int,
        default=17,
        help="ONNX opset version (default: 17)",
    )
    parser.add_argument(
        "--dynamic",
        action="store_true",
        help="Enable dynamic axes for batch size / spatial dims",
    )
    parser.add_argument(
        "--simplify",
        action="store_true",
        default=True,
        help="Run onnx-simplifier after export (default: True)",
    )
    parser.add_argument(
        "--half",
        action="store_true",
        help="Export in FP16 (requires GPU)",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    weights = args.weights.resolve()
    if not weights.exists():
        raise FileNotFoundError(f"Checkpoint not found: {weights}")

    print(f"Loading model from: {weights}")
    model = YOLO(str(weights))

    print(f"Exporting to ONNX (imgsz={args.imgsz}, opset={args.opset}, dynamic={args.dynamic}, half={args.half}) ...")
    exported_path = model.export(
        format="onnx",
        imgsz=args.imgsz,
        opset=args.opset,
        dynamic=args.dynamic,
        simplify=args.simplify,
        half=args.half,
    )

    print(f"Export complete: {exported_path}")


if __name__ == "__main__":
    main()
