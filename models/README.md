# 检测资产（本机专属，未提交）

`vertical_grasp_object` 技能的检测链路需要两个文件。它们与**这台机器**的相机安装位置和
桌面高度绑定，且体积大，所以既不提交进本仓，也不放在技能包里——技能包由
`robonix_manifest.yaml` 的 `url:` 拉取到 `rbnx-boot/cache/`，那份 `models/` 始终是空的。

| 文件 | 内容 | 来源 |
|---|---|---|
| `best.pt` | YOLO-OBB 方块检测权重（约 113 MB） | 用 `tools/yolo_train/` 训练链路（`json2label.py` → `train.py`），产物在 `runs/<run>/weights/best.pt` |
| `handeye_calib.npz` | 相机→base 手眼单应 + 头部位姿 + 桌面 Z | 在本台机器上跑 `tools/arm_calibration/calibrate_handeye.py` |

把两个文件直接放在本目录。清单以绝对路径 `${D1_DEPLOY_DIR}/models/...` 指向它们
（`D1_DEPLOY_DIR` 在清单的 `env:` 里设定），因此 `rbnx clean --cache` 重新拉取技能包
不会影响这两个文件。

只有 `detect_cubes`（YOLO）需要 `best.pt`；`handeye_calib.npz` 则是**两个检测器共用**的
几何真值——缺它 `detect_objects` 也无法工作。

相机重装或桌面高度变化后必须重新标定。
