from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


@dataclass
class DatasetSpec:
    root: Path
    names: list[str]
    train: str = "images/train"
    val: str = "images/val"
    test: str | None = None


def _validate_dataset_layout(spec: DatasetSpec) -> None:
    required_paths = [
        spec.root / spec.train,
        spec.root / spec.val,
        spec.root / spec.train.replace("images", "labels", 1),
        spec.root / spec.val.replace("images", "labels", 1),
    ]
    missing = [str(path) for path in required_paths if not path.exists()]
    if missing:
        raise FileNotFoundError(
            "Dataset layout is incomplete. Missing paths:\n" + "\n".join(missing)
        )


def ensure_dataset_yaml(spec: DatasetSpec, output_path: Path) -> Path:
    _validate_dataset_layout(spec)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    lines = [
        f"path: {spec.root.resolve()}",
        f"train: {spec.train}",
        f"val: {spec.val}",
    ]
    if spec.test:
        lines.append(f"test: {spec.test}")

    lines.append("names:")
    for class_id, class_name in enumerate(spec.names):
        lines.append(f"  {class_id}: {class_name}")

    output_path.write_text("\n".join(lines) + "\n")
    return output_path
