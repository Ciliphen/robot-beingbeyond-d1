# YOLO-OBB 方块检测训练

训练 `vertical_grasp_object_skill` 用的 YOLO-OBB（有向框）方块检测模型。
产出的 `best.pt` 就是该 skill `models/best.pt` 的来源，被 `detector.py::CubeDetector`
加载（经 `detect.py::detect_objects_in_frame`）用于 `detect_cubes`。

从 Beingbeyond_D1 参考仓库 `object_detect/` 搬来，路径已改为自包含（相对本目录）。

## 目录内容

| 文件              | 作用                                                                                       |
| ----------------- | ------------------------------------------------------------------------------------------ |
| `capture.py`    | **（第 0 步）** 真机采集：相机预览 + 实时检测 + 头部控制 + 拍照存到 `dataset/raw/` |
| `vision.py`     | `capture.py` 用的 RealSense 相机封装（依赖 `pyrealsense2`）                            |
| `json2label.py` | LabelMe JSON 标注 → YOLO-OBB 标签，并按 8:2 划分 train/val                                |
| `train.py`      | 用 ultralytics 训练 YOLO11-OBB                                                             |
| `data.yaml`     | 数据集路径 + 类别定义（训练读它）                                                          |
| `detect.py`     | 推理函数（与 skill 运行时同一份，供独立测试/参考）                                         |
| `.gitignore`    | 挡住`dataset/`、`runs/`、`*.pt`（大文件不入库）                                      |

> 数据集、训练产物、基座权重都**不入库**（体积大）。下面说明各自放哪。

## 环境

**训练**（`json2label.py` / `train.py`）需要 GPU 版 ultralytics（参考仓库用 conda `bb_gpu`）：

```bash
conda activate bb_gpu          # 或你自己的环境
pip install ultralytics opencv-python numpy
```

**真机采集**（`capture.py`）还需要在**接了 D1 机械臂 + RealSense 相机的机器**上跑，额外依赖：

```bash
pip install pyrealsense2       # RealSense SDK（vision.py 用）
# beingbeyond_d1_sdk：D1 头部/机械臂 SDK，不在本仓库，需装到运行环境
#                     （参考仓库 conda 环境 bb_d1 里有）
```

> `capture.py` / `vision.py` 依赖 `pyrealsense2` 和 `beingbeyond_d1_sdk`，
> 这两个都**不在本仓库**，只能在真机环境跑；纯训练机上跳过第 0 步、直接从已有图片开始即可。

## 完整流程

### 0. 采集图片（真机，可选）

在机器人上跑采集工具：实时预览相机 + 当前模型检测效果，按 `空格` 存图。

```bash
conda activate bb_d1
python tools/yolo_train/capture.py                 # 相机 + 头部 + 检测预览
python tools/yolo_train/capture.py --no-head       # 无机械臂，仅相机采集
python tools/yolo_train/capture.py --model runs/train/weights/best.pt  # 指定模型
```

键盘：`A/D` 头部左右、`W/S` 头部上下、`H` 回标定头位（yaw=-10°/pitch=35°）、
`空格` 拍照存到 `dataset/raw/`（自动接续编号，不覆盖）、`Q`/`ESC` 退出。

存图直接落到 `dataset/raw/`，即下一步的输入。已有图片数据集可跳过这步。
（若手头已有图片，也可以不经相机，直接把 jpg 拷进 `dataset/raw/`。）

### 1. 准备标注数据

用 [LabelMe](https://github.com/wkentaro/labelme) 对方块拍照标注：每张图画**多边形**框住方块，
`label` 填类别名（须是 `json2label.py::CLASS_NAMES` 之一）。每张 `xxx.jpg` 配一个同名
`xxx.json`。把它们放到：

```
tools/yolo_train/dataset/raw/
├── 00000.jpg
├── 00000.json
├── 00001.jpg
├── 00001.json
└── ...
```

### 2. 转换 + 划分数据集

```bash
python tools/yolo_train/json2label.py
```

会把多边形转成 YOLO-OBB 标签、随机（固定种子 42）按 8:2 划分，生成：

```
dataset/images/train/  dataset/images/val/
dataset/labels/train/  dataset/labels/val/
dataset/labels_preview/     # 标注可视化，翻一眼确认框对不对
```

### 3. 核对类别一致

`json2label.py::CLASS_NAMES` 的**顺序**决定类别索引，必须和 `data.yaml` 的 `names`
一字不差。默认四类：

```
0: red_cube   1: blue_cube   2: green_cube   3: yellow_cube
```

改类别时，两个文件一起改，并同步 `data.yaml` 的 `nc`（类别数）。

### 4. 训练

```bash
python tools/yolo_train/train.py
```

- 首次运行自动下载基座 `yolo11x-obb.pt`（约 56MB）。
- 关键超参写在 `train.py`：`imgsz=640`、`epochs=1000`、`batch=8`、`device="0"`（第 0 号 GPU）。
  显存不够就调小 `batch`；多卡用 `device="0,1"`；纯 CPU 用 `device="cpu"`（很慢）。
- 产物在 `runs/train/weights/`：`best.pt`（最佳）和 `last.pt`（最后一轮）。

### 5. 部署到 skill

把训练好的权重拷到 skill 的 models 目录，命名 `best.pt`：

```bash
cp tools/yolo_train/runs/train/weights/best.pt \
   skills/vertical_grasp_object_skill/models/best.pt
```

`detect_cubes` 激活时（`on_activate`）就会加载它。模型路径也可用配置
`model_path` 或环境变量 `BLOCK_GRASP_MODEL` 覆盖（见 skill 的 `config.spec`）。

## 快速验证权重

不上机，用 `detect.py` 对单张图跑一遍看检测框：

```python
import cv2
from detect import load_model, detect_objects_in_frame, draw_box
import numpy as np

model = load_model("runs/train/weights/best.pt")
frame = cv2.imread("dataset/images/val/xxx.jpg")
for (u, v, w, h, r), score, cid, name in detect_objects_in_frame(model, frame, conf_thres=0.85):
    draw_box(frame, u, v, w, h, np.rad2deg(r), f"{name}:{score:.2f}")
cv2.imwrite("check.jpg", frame)
```

## 说明

- **OBB（有向框）而非普通框**：方块可能斜放，有向框的旋转角经
  `coordinate_utils.estimate_grasp_angle_deg` 变成抓取偏航角（joint-6），斜方块才夹得准。
- **训练/推理超参对齐**：skill 推理的 `conf`/`iou` 阈值在 `block_grasp.config`
  （`CONF_THRESHOLD`/`IOU_THRESHOLD`）；训练分辨率 `imgsz=640` 与相机来图差异过大时精度会掉。
- 只训练方块类别。要检测其他物体走的是 skill 的 VLM 路径（`detect_objects`），不在这里训练。
