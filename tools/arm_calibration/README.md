# 机械臂标定

标定 `vertical_grasp_object_skill` 抓取所需的两组参数：

1. **手眼标定**（`calibrate_handeye.py`）——相机像素 ↔ 桌面基座坐标的 2D 单应矩阵，
   产出 `handeye_calib.npz`，就是该 skill `models/handeye_calib.npz` 的来源，被
   `detector.py` 加载用于把检测到的像素点解析成基座系 XY。
2. **重力下垂标定**（`calibrate_sag.py`）——机械臂伸远时自身重力导致末端下垂，拟合
   `dz = factor·dist³` 得到 `GRAVITY_SAG_FACTOR`，填进 `block_grasp.config` 抬高目标点补偿下垂。

从 Beingbeyond_D1 参考仓库 `block_grasp/` 搬来，import/路径已改为自包含（相对本目录）。

## 目录内容

| 文件                     | 作用                                                                                         |
| ------------------------ | -------------------------------------------------------------------------------------------- |
| `calibrate_handeye.py` | **手眼标定**：相机预览 + 点图取参考点 + 键盘/拖拽移末端对准 + 记录点对 → 算单应       |
| `calibrate_sag.py`     | **重力下垂标定**：末端遍历多个径向距离、下降到抓取高度、量高度 → 拟合 sag factor      |
| `test_click_goto.py`   | **验证（主）**：点图后机械臂末端**实际移动**到点击位置，靠物理对准检验标定准不准 |
| `test_calib.py`        | **验证（轻量）**：点图上任意像素，只打印解算出的桌面世界坐标，不动机械臂               |
| `ik_scipy.py`          | SLSQP 数值 IK 求解器（标定脚本移动末端时用；与 skill 运行时同一份）                          |
| `config.py`            | 标定/抓取常量（头部姿态、EE 姿态、IK 容差等；与 skill 的`block_grasp.config` 对应）        |
| `../vision.py`         | RealSense 相机封装（依赖`pyrealsense2`）。放在 `tools/`，供各工具共用                    |
| `handeye_calib.npz`    | 一份示例标定产物，可直接部署或作为格式参考                                                   |

> `handeye_calib.npz` 内含：`H`（3×3 单应）、`plane`（桌面平面 z=ax+by+c）、
> `head_yaw`/`head_pitch`（标定时头部角，检测时必须复现）、`pixel_pts`/`world_pts`（原始点对）、
> `mean_err_mm`（标定误差）。

## 环境

⚠ **真机脚本**：直接开 `/dev/ttyUSB0` 串口、RealSense、CAN 总线（`calibrate_handeye.py` 的灵巧手）、
cv2 窗口，必须在**接了 D1 机械臂 + RealSense 相机的机器**上跑：

```bash
conda activate bb_d1
pip install pyrealsense2 opencv-python numpy scipy
# beingbeyond_d1_sdk：D1 头部/机械臂/灵巧手 SDK，不在本仓库，需装到运行环境
#                     （参考仓库 conda 环境 bb_d1 里有）
```

> `beingbeyond_d1_sdk` 和 `pyrealsense2` 都**不在本仓库**，只能在真机环境跑。
> `ik_scipy.py` / `config.py` 是同目录兄弟，`vision.py` 在父目录 `tools/`（各工具共用）。
> 脚本已把本目录和父目录加进 `sys.path`，无需 `PYTHONPATH`。

## 完整流程

### 1. 手眼标定（先做）

```bash
conda activate bb_d1
python tools/arm_calibration/calibrate_handeye.py
```

流程（详见脚本内提示）：

1. 相机看向桌面，头部固定在标定姿态（默认 yaw=-10°、pitch=35°，头部标定期间**不要动**）
2. 在画面上**点一个桌面参考点**（绿色十字）
3. 把末端移到那个物理点——两种方式：
   - **键盘/IK**：`WASD`=XY、`ZX`=Z、`UO/IK/JL`=RPY
   - **拖拽示教**：`T` 松开机械臂力矩后用手拖（松力矩前先扶住机械臂！）
4. `空格` 记录一对（像素, 世界）
5. 桌面上不同位置重复 **6 次以上**
6. `C` 计算单应并保存到 `handeye_calib.npz`（XY 误差 < 20mm 才存）

其他键：`T` 切换力矩（拖拽模式）、`B` 开合手、`R` 复位末端、`ESC` 退出。

### 2. 验证手眼标定

推荐用 `test_click_goto.py`：点图后机械臂末端**实际移动**过去，直接看末端有没有对准点击的物理点，
这是最真实的验证。

```bash
python tools/arm_calibration/test_click_goto.py
python tools/arm_calibration/test_click_goto.py --calib handeye_calib.20260717_095620.npz  # 回溯某份备份
```

⚠ **急停按钮请保持触手可及**——机械臂会移动到点击位置上方并下降。

加载 `handeye_calib.npz`（`--calib` 可指定某份备份回溯验证），把头部摆到标定姿态。左键点桌面上一点，
末端就移到该点上方 `z_offset`（默认 10cm）处，看指尖 XY 有没有对准。
键：`Z/X` 升降目标高度、`空格/B` 收紧/放松手、`ESC` 退出。末端明显没对准就重标。

也可以用轻量版 `test_calib.py`（不动机械臂，只在终端打印解算的世界坐标 `pixel=(u,v) → world=(x,y)`，
拿尺子核对），适合快速看数值：

```bash
python tools/arm_calibration/test_calib.py
python tools/arm_calibration/test_calib.py --calib handeye_calib.20260717_095620.npz
```

### 3. 重力下垂标定

```bash
python tools/arm_calibration/calibrate_sag.py
python tools/arm_calibration/calibrate_sag.py --reverse   # 从远到近，与正向对比迟滞（判断机械背隙）
```

⚠ **急停按钮请保持触手可及**——机械臂会下降到接近桌面的抓取高度。

脚本让末端在抓取姿态下遍历若干径向距离（`CALIB_X`），每点**关闭下垂补偿**下降到名义抓取高度并停住，
你用尺子量【同一个末端参考点】到桌面的垂直距离(mm)、回车录入。全部量完自动拟合并打印
建议的 `GRAVITY_SAG_FACTOR`（附 R²）。

### 4. 部署到 skill

- **手眼标定**：把 npz 拷到 skill 的 models 目录：

  ```bash
  cp tools/arm_calibration/handeye_calib.npz \
     skills/vertical_grasp_object_skill/models/handeye_calib.npz
  ```

  路径也可用配置 `calib_path` 或环境变量 `VERTICAL_GRASP_OBJECT_CALIB` 覆盖（见 skill 的 `main.py`）。
- **重力下垂**：把拟合出的值填进 skill 侧的 `block_grasp.config` 的 `GRAVITY_SAG_FACTOR`
  （`controller.py` 用 `GRAVITY_SAG_FACTOR * dist³` 抬高目标点）。

## 说明

- **头部姿态必须一致**：手眼标定、验证（`test_click_goto`/`test_calib`）、真机检测三处的头部 yaw/pitch
  必须相同，否则单应矩阵不成立。默认 yaw=-10°、pitch=35°（`config.py` 的 `HEAD_YAW_DEG`/`HEAD_PITCH_DEG`）。
- **相机分辨率**：标定用 1280×720（`config.py` 的 `CALIB_CAM_WIDTH/HEIGHT`）；检测端分辨率不同时
  像素坐标会自动缩放对齐。
- **下垂模型是径向距离的三次方**：`calibrate_sag.py` 固定 y=0 只沿 x 遍历即可拟合，
  因为下垂只跟径向距离有关；拟合用的是斜率，量到哪个参考点（指尖/法兰）不影响结果。
