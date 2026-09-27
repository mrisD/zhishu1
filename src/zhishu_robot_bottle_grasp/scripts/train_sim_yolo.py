#!/usr/bin/env python3
"""Fine-tune YOLO26 segmentation for the overhead Gazebo bottle view."""

import argparse
from pathlib import Path

import yaml
from ultralytics import YOLO


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", default="/tmp/bottle_yolo_dataset")
    parser.add_argument("--weights", default="yolo26n-seg.pt")
    parser.add_argument("--output", default="models/bottle_yolo26n_seg.pt")
    parser.add_argument("--epochs", type=int, default=35)
    parser.add_argument("--imgsz", type=int, default=320)
    args = parser.parse_args()

    root = Path(args.dataset).resolve()
    descriptor = root / "data.yaml"
    descriptor.write_text(yaml.safe_dump({
        "path": str(root),
        "train": "images/train",
        "val": "images/val",
        "names": {0: "bottle"},
    }, sort_keys=False))
    project = root / "runs"
    model = YOLO(args.weights)
    result = model.train(
        data=str(descriptor), epochs=args.epochs, imgsz=args.imgsz, batch=8,
        device="cpu", workers=2, project=str(project), name="yolo26n_seg",
        exist_ok=True, patience=12, cache=True,
    )
    best = Path(result.save_dir) / "weights" / "best.pt"
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_bytes(best.read_bytes())
    print(output.resolve())


if __name__ == "__main__":
    main()
