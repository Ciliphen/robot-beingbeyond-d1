#!/usr/bin/env python3
"""
YOLO OBB 训练脚本

用法:
    conda activate bb_gpu          # 需装了 ultralytics + CUDA 的环境
    python tools/yolo_train/train.py

产物:
    tools/yolo_train/runs/train/weights/best.pt   (训练好的权重)
把 best.pt 拷到 skill 的 models/ 下即可上机推理（见本目录 README.md）。
"""
from __future__ import annotations
import os, warnings
warnings.filterwarnings("ignore")

from ultralytics import YOLO

HERE = os.path.dirname(os.path.abspath(__file__))

if __name__ == "__main__":
    model = YOLO("yolo11x-obb.pt")  # 首次运行会自动下载（约 56MB）

    model.train(
        data=os.path.join(HERE, "data.yaml"),
        imgsz=640,
        epochs=1000,
        batch=8,
        workers=4,
        close_mosaic=10,
        device="0",
        amp=False,
        optimizer="SGD",
        plots=False,       # skip PR-curve plotting (matplotlib font bug)
        project=os.path.join(HERE, "runs"),
        name="train",
    )
