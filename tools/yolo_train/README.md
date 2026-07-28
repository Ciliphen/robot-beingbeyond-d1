# YOLO-OBB cube-detection training

*[中文版](./README_CN.md)*

Trains the YOLO-OBB (oriented bounding box) cube detector used by the
`vertical_grasp_object` skill. The resulting `best.pt` is the source of this
deployment's `models/best.pt`, loaded by the skill's `detector.py::CubeDetector`
(via `detect.py::detect_objects_in_frame`) to serve `detect_cubes`.

Lifted from the Beingbeyond_D1 reference repo's `object_detect/`, with paths
rewritten to be self-contained (relative to this directory).

## Contents

| File | Purpose |
| --- | --- |
| `capture.py` | **(step 0)** on-robot capture: camera preview + live detection + head control, saving frames to `dataset/raw/` |
| `../vision.py` | the RealSense wrapper `capture.py` uses (needs `pyrealsense2`). Kept in `tools/` so every tool shares one copy |
| `json2label.py` | LabelMe JSON annotations → YOLO-OBB labels, with an 8:2 train/val split |
| `train.py` | trains YOLO11-OBB via ultralytics |
| `data.yaml` | dataset paths + class definitions (read by training) |
| `detect.py` | the inference functions (the same copy the skill runs, kept here for standalone testing/reference) |
| `.gitignore` | keeps `dataset/`, `runs/`, `*.pt` out of the repo |

> The dataset, training artefacts, and base weights are **not committed** (too
> large). Where each of them goes is described below.

## Environment

**Training** (`json2label.py` / `train.py`) needs a GPU build of ultralytics (the
reference repo uses the conda env `bb_gpu`):

```bash
conda activate bb_gpu          # or your own env
pip install ultralytics opencv-python numpy
```

**On-robot capture** (`capture.py`) additionally has to run on the machine with the
D1 arm and RealSense attached, and needs:

```bash
pip install pyrealsense2       # RealSense SDK (used by vision.py)
# beingbeyond_d1_sdk: the D1 head/arm SDK. The wheel ships in this repo under
#                     tools/func_verify/lib/ — install it into the run env.
```

> `capture.py` / `vision.py` depend on `pyrealsense2` and `beingbeyond_d1_sdk`, so
> they only run on the real robot. On a training-only machine, skip step 0 and
> start from images you already have.

## Full pipeline

### 0. Capture images (on the robot, optional)

Run the capture tool on the robot: live camera preview plus the current model's
detections, `Space` to save a frame.

```bash
conda activate bb_d1
python tools/yolo_train/capture.py                 # camera + head + detection preview
python tools/yolo_train/capture.py --no-head       # camera only, no arm
python tools/yolo_train/capture.py --model runs/train/weights/best.pt  # pick a model
```

Keys: `A/D` pan the head, `W/S` tilt it, `H` return to the calibration head pose
(yaw=-10° / pitch=35°), `Space` save a frame into `dataset/raw/` (auto-incrementing,
never overwrites), `Q`/`ESC` quit.

Frames land straight in `dataset/raw/`, which is the next step's input. Skip this
step if you already have a dataset — you can also just copy JPEGs into
`dataset/raw/` without touching a camera.

### 1. Prepare annotations

Annotate the cubes with [LabelMe](https://github.com/wkentaro/labelme): draw a
**polygon** around each cube and set `label` to a class name (it must be one of
`json2label.py::CLASS_NAMES`). Every `xxx.jpg` needs a matching `xxx.json`. Place
them in:

```
tools/yolo_train/dataset/raw/
├── 00000.jpg
├── 00000.json
├── 00001.jpg
├── 00001.json
└── ...
```

### 2. Convert and split

```bash
python tools/yolo_train/json2label.py
```

Converts the polygons to YOLO-OBB labels and splits them 8:2 at random (fixed seed
42), producing:

```
dataset/images/train/  dataset/images/val/
dataset/labels/train/  dataset/labels/val/
dataset/labels_preview/     # annotation overlays — skim them to check the boxes
```

### 3. Check the classes line up

The **order** of `json2label.py::CLASS_NAMES` determines the class indices and must
match `data.yaml`'s `names` exactly. The four defaults:

```
0: red_cube   1: blue_cube   2: green_cube   3: yellow_cube
```

When changing classes, edit both files and update `data.yaml`'s `nc` (class count).

### 4. Train

```bash
python tools/yolo_train/train.py
```

- The first run downloads the base `yolo11x-obb.pt` (~56 MB) automatically.
- The key hyperparameters live in `train.py`: `imgsz=640`, `epochs=1000`, `batch=8`,
  `device="0"` (GPU 0). Lower `batch` if you run out of VRAM; use `device="0,1"` for
  multiple GPUs, or `device="cpu"` (very slow) for none.
- Artefacts land in `runs/train/weights/`: `best.pt` and `last.pt`.

### 5. Deploy to the skill

Copy the trained weights into this deployment's `models/` dir as `best.pt`:

```bash
cp tools/yolo_train/runs/train/weights/best.pt models/best.pt
```

The skill loads it on `on_activate`. The path can also be overridden by the
`model_path` config key or the `BLOCK_GRASP_MODEL` env var (see the skill's
`config.spec`).

## Quick weight check

Without the robot, run `detect.py` over a single image and look at the boxes:

```python
import cv2
import numpy as np
from detect import load_model, detect_objects_in_frame, draw_box

model = load_model("runs/train/weights/best.pt")
frame = cv2.imread("dataset/images/val/xxx.jpg")
for (u, v, w, h, r), score, cid, name in detect_objects_in_frame(model, frame, conf_thres=0.85):
    draw_box(frame, u, v, w, h, np.rad2deg(r), f"{name}:{score:.2f}")
cv2.imwrite("check.jpg", frame)
```

## Notes

- **Oriented boxes, not axis-aligned ones**: cubes can sit at an angle, and the
  OBB's rotation becomes the grasp yaw (joint 6) through
  `coordinate_utils.estimate_grasp_angle_deg`. That is what makes a tilted cube
  grippable.
- **Keep training and inference aligned**: the skill's inference `conf`/`iou`
  thresholds live in `block_grasp.config` (`CONF_THRESHOLD` / `IOU_THRESHOLD`).
  Accuracy drops when the training resolution (`imgsz=640`) is far from what the
  camera delivers.
- Only cube classes are trained here. Detecting anything else goes through the
  skill's VLM path (`detect_objects`), which needs no training.
