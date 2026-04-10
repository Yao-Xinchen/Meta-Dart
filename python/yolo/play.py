from __future__ import annotations

import argparse
from pathlib import Path

from model import build_model


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run YOLOv8 inference for dart detection.")
    parser.add_argument("--weights", type=str, required=True, help="Trained checkpoint, e.g. runs/dart_yolov8/weights/best.pt")
    parser.add_argument("--source", type=str, required=True, help="Image, folder, video, webcam index, or stream URL")
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--iou", type=float, default=0.45)
    parser.add_argument("--device", type=str, default="cpu")
    parser.add_argument("--project", type=Path, default=Path("runs"))
    parser.add_argument("--name", type=str, default="dart_predict")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--save-txt", action="store_true")
    parser.add_argument("--save-conf", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    model = build_model(args.weights)
    results = model.predict(
        source=args.source,
        conf=args.conf,
        iou=args.iou,
        imgsz=args.imgsz,
        device=args.device,
        project=str(args.project),
        name=args.name,
        save=True,
        save_txt=args.save_txt,
        save_conf=args.save_conf,
    )

    if results:
        print(f"Prediction artifacts saved to {results[0].save_dir}")


if __name__ == "__main__":
    main()
