from __future__ import annotations

import argparse
from pathlib import Path

from dataset import DatasetSpec, ensure_dataset_yaml
from loss import describe_loss
from model import DEFAULT_WEIGHTS, build_model


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Train a YOLO OBB model for dart detection.")
    parser.add_argument("--data-root", type=Path, required=True, help="Dataset root with images/train, images/val, labels/train, labels/val")
    parser.add_argument("--classes", nargs="+", default=["dart"], help="Class names in dataset order")
    parser.add_argument("--weights", type=str, default=DEFAULT_WEIGHTS, help="Pretrained YOLO OBB weights or checkpoint path")
    parser.add_argument("--dataset-yaml", type=Path, default=Path("dart_dataset.yaml"), help="Path to generated dataset YAML")
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--batch", type=int, default=16)
    parser.add_argument("--device", type=str, default="cpu")
    parser.add_argument("--project", type=Path, default=Path("runs"))
    parser.add_argument("--name", type=str, default="dart_yolo11_obb")
    parser.add_argument("--workers", type=int, default=4)
    parser.add_argument("--patience", type=int, default=30)
    parser.add_argument("--cache", action="store_true")
    return parser.parse_args()


def infer_label_task(data_root: Path) -> str:
    label_dir = data_root / "labels" / "train"
    for label_path in sorted(label_dir.glob("*.txt")):
        for line in label_path.read_text().splitlines():
            parts = line.strip().split()
            if not parts:
                continue
            if len(parts) == 5:
                return "detect"
            if len(parts) == 9:
                return "obb"
            raise ValueError(f"Unsupported label format in {label_path}: expected 5 or 9 columns, got {len(parts)}")
    raise ValueError(f"No labels found in {label_dir}")


def validate_weights_for_task(weights: str, task: str) -> None:
    normalized = weights.lower()
    if task == "obb" and "obb" not in normalized:
        raise ValueError(
            "Your dataset uses OBB labels, but the selected weights are not an OBB model. "
            f"Use an OBB checkpoint such as '{DEFAULT_WEIGHTS}', not '{weights}'."
        )
    if task == "detect" and "obb" in normalized:
        raise ValueError(
            "Your dataset uses detect labels, but the selected weights are an OBB model. "
            f"Use a detect checkpoint instead of '{weights}'."
        )


def main() -> None:
    args = parse_args()
    task = infer_label_task(args.data_root)
    validate_weights_for_task(args.weights, task)

    test_path = args.data_root / "images" / "test"
    test_split = "images/test" if test_path.exists() else None

    spec = DatasetSpec(root=args.data_root, names=args.classes, test=test_split)
    dataset_yaml = ensure_dataset_yaml(spec, args.dataset_yaml)

    model = build_model(args.weights)
    print(describe_loss())
    print(f"Detected dataset task: {task}")
    print(f"Using dataset config: {dataset_yaml.resolve()}")

    results = model.train(
        data=str(dataset_yaml.resolve()),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        project=str(args.project),
        name=args.name,
        workers=args.workers,
        patience=args.patience,
        cache=args.cache,
    )

    save_dir = getattr(results, "save_dir", None)
    if save_dir is not None:
        print(f"Training artifacts saved to {save_dir}")


if __name__ == "__main__":
    main()
