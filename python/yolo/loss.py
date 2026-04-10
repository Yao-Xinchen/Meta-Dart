from __future__ import annotations


def describe_loss() -> str:
    return (
        "Ultralytics YOLOv8 computes the detection loss internally during model.train(). "
        "For object detection the training logs typically expose box_loss, cls_loss, and dfl_loss. "
        "There is no separate custom loss implementation in this folder because the project now uses "
        "the official YOLOv8 training pipeline."
    )
