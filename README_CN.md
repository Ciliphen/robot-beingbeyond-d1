# robot-beingbeyond-d1

*[English](./README.md)*

`robonix.robot.beingbeyond.d1` — BeingBeyond D1 桌面机器人的 robonix 整机部署仓：
固定底座 6 自由度机械臂 + 2 自由度头部云台 + 五指灵巧手 + 头部 RGB-D 相机。

核心是**灵巧手物体抓放技能**（`vertical_grasp_object`，8 个 MCP 工具），另带一个
`hand_gesture` 手势技能。由 `rbnx chat`（文字）或语音链路经 pilot 触发。

<img src="./assets/robot.jpg" alt="BeingBeyond D1 桌面机器人" width="560">

## 能力

| 工具 | 作用 |
|---|---|
| `detect_objects(instruction)` | VLM 开放词表检测任意物体（颜色/形状/文字/空间描述），返回 base 系抓取点 + 抓取角。**通用首选** |
| `detect_cubes(class_filter)` | YOLO-OBB 检测桌面方块（红/蓝/绿/黄），同 schema。**方块首选**，也作 VLM 未配置时的退路 |
| `pick_cube(position, angle_deg, gripper)` | 移到位置，按角度/夹紧度抓起并夹住 |
| `place_cube(position, angle_deg, gripper)` | 移到位置松开放下；`z` 决定堆叠高度，成功后自动泊车让开视野 |
| `stack_cubes(top_color, base_color, gripper)` | 单次调用完成"A 叠到 B 上"：检测→抓→放。**堆叠首选** |
| `put_cube_in_container(color, gripper)` | 单次调用把指定颜色方块放进固定容器位 |
| `sort_cubes(gripper)` | 单次调用把桌面所有方块按颜色分拣到各自固定位 |
| `verify_grasp(instruction, mode, position)` | VLM 复检抓/放是否到位，闭环重试的依据 |
| `gesture_dance(duration_s)` | 让灵巧手跳舞（手指开合摆动），结束后回到张开位 |

`position` 为 base 坐标系（米）的 `"x,y"` / `"x,y,z"`，或命名位置（如 `中间`）；半角/全角逗号均可。
检测几何走手眼标定单应 + 透视校正，深度为固定桌面 Z（两个检测器都不用 RealSense 深度流）。

## 平台与硬件盘点

**计算平台**：x86_64，Windows 11 宿主上的 WSL2 / Ubuntu 22.04，native（非容器）。
**无 ROS 2**——所有原语直连厂商 SDK 并以 gRPC 提供能力，因此不提供任何 `topic_out` 流式契约。
Python 3.10（conda env `bb_d1_robonix`）。本部署验证所用 Robonix commit：
`8c0baca2e9f95c334b9ef078419c7bd20739bac7`（2026-07-20）。

**硬件与连接**：

| 设备 | 型号 | 连接 |
|---|---|---|
| 机械臂 + 头部云台 | BeingBeyond D1（6 + 2 DOF） | 串口 `/dev/ttyUSB0` @ 1000000（**臂与头共用这一个口**） |
| 灵巧手 | Linker 五指六轴 | CAN `can0` @ 1000000 |
| 头部相机 | Intel RealSense D435i | USB 3.0，1280x720@30 |
| 麦克风 / 扬声器 | Windows 宿主设备 | 经 WSLg 的 PulseAudio `pulse` 设备 |

**坐标系**：整机只维护一棵 URDF（`urdf/robot_right_hand.urdf`），根 link 为 `link_base`，
臂链与头部链均由它分支，相机 frame 为 `camera`；所有固定变换都在这棵树里。
固定底座部署、无 ROS 2，故**没有 TF 发布方**——像素→base 的变换由技能内的手眼标定单应完成，
标定文件 `handeye_calib.npz` 是这条链路的唯一真值来源。

**安全边界**：

- **无软件急停接口**。只能靠硬件断电 / 物理急停。首次运动前必须验证硬件急停可独立停住臂与手。
- 关节限位由控制器固件与 URDF 约束；`move_pose` 仅在 IK 收敛且位置误差 ≤ `ik_pos_tol`（默认 0.05 m）时下发，否则报错不动作。
- 总线权限：串口需 `chmod 666 /dev/ttyUSB0` 或加入 `dialout` 组；CAN 由 DexHand 在 init 阶段拉起（非 root 时需预先拉起接口或导出 `PREFLIGHT_SUDO_PASS`）。
- 看门狗：任何提供方在输入超时、进程退出或收到 `CMD_SHUTDOWN` 时停止硬件输出并释放串口。
- `pick_z` 设过低会让手指压到桌面；灵巧手首次开合建议先用 `set_joint_torque_limits` 降低力矩上限。技能不做碰撞检查。
- `soma.yaml` 的 footprint 只是底座安装轮廓（0.20 m 方形），**不是可行驶区域**；机械臂动态包络不在静态 footprint 内。

## 部署构成

| 类型 | 实例名 | 包 | 来源 |
|---|---|---|---|
| primitive | `d1_arm` | `robonix.primitive.beingbeyond.d1.arm` | [primitive-beingbeyond-d1-arm-rbnx](https://github.com/syswonder/primitive-beingbeyond-d1-arm-rbnx) |
| primitive | `d1_hand` | `robonix.primitive.beingbeyond.d1.hand` | [primitive-beingbeyond-d1-hand-rbnx](https://github.com/syswonder/primitive-beingbeyond-d1-hand-rbnx) |
| primitive | `d1_camera` | `robonix.primitive.beingbeyond.d1.camera` | [primitive-beingbeyond-d1-camera-rbnx](https://github.com/syswonder/primitive-beingbeyond-d1-camera-rbnx) |
| primitive | `audio_driver` | `robonix.primitive.audio.alsa` | [primitive-audio-driver-rbnx](https://github.com/syswonder/primitive-audio-driver-rbnx) |
| service | `speech` | — | `${ROBONIX_SOURCE_PATH}/services/speech`（robonix 内置） |
| skill | `vertical_grasp_object` | `robonix.skill.vertical_grasp_object` | [skill-vertical-grasp-object-rbnx](https://github.com/syswonder/skill-vertical-grasp-object-rbnx) |
| skill | `hand_gesture` | `robonix.skill.hand_gesture` | [skill-hand-gesture-rbnx](https://github.com/syswonder/skill-hand-gesture-rbnx) |

本仓只做**组装与本体专属配置**：清单、soma/urdf、离线工具与检测资产。所有驱动与技能都在
各自的独立仓库，由清单的 `url:` + `branch:` 拉取到 `rbnx-boot/cache/<repo-name>/`。
包的能力表、配置字段与安全说明见各包自己的 README。

改包代码的流程因此变成：在包仓改 → push → 本仓 `rbnx clean -f robonix_manifest.yaml --cache`
→ 重新 `rbnx build`。

## 结构

```
robot-beingbeyond-d1/
├── robonix_manifest.yaml   # 部署清单：系统服务 + 语音 + 4 原语 + speech + 2 技能
├── soma.yaml               # D1 body model（urdf.path → ./urdf/）
├── .env.example            # VLM + 腾讯云 TTS 凭据模板（copy 为 .env）
├── assets/robot.jpg        # catalog 预览图，tools/func_verify/ 也引用它
├── urdf/
│   ├── robot_right_hand.urdf   # 整机 URDF（供 soma / IK 用），根 link = link_base
│   └── meshes/                 # URDF 引用的 STL（含 right_hand/）
├── models/                 # 检测资产：best.pt + handeye_calib.npz（本机专属，git-ignored）
└── tools/                  # 离线工具（不参与部署）
    ├── func_verify/        # D1 SDK 功能自检示例（独立 env bb_d1_rbnx，见其 README）
    ├── yolo_train/         # 方块检测数据标注+训练链路，产物 best.pt 拷进 skill models/
    ├── arm_calibration/    # 手眼标定 / 下垂补偿，产物 handeye_calib.npz
    └── vision.py           # RealSense 采集辅助
```

每个包的能力表、配置字段与安全说明见各包 `README.md`；调用语义见技能包的 `CAPABILITY.md`；
配置字段的类型/单位/默认值/失败条件见各包 `config.spec`。

技能是纯 robonix **消费者**：`on_activate` 经 atlas 发现并连接原语，通过其 gRPC 契约驱动；
自身不开串口/CAN/RealSense。IK/FK、YOLO、手眼投影均本地纯计算（`object_detect` +
`block_grasp` 已随技能包提供，FK/IK 来自 `beingbeyond_d1_sdk` wheel）。

## 语音

`rbnx boot` 会拉起语音链路：`audio_driver`（WSLg 下麦克风/扬声器走 PulseAudio `pulse` 设备，
WSLg 无真实声卡故 `arecord -l` 为空，设备必须显式设为 `pulse`）+ `speech` 服务。

- **ASR**：`custom` 后端，把 16k/mono/pcm_s16le 转发到**外部** PaddleSpeech 流式 WebSocket
  （`PADDLE_ASR_PORT=8090`；host 缺省解析到 WSL 网关 = Windows 宿主，故 WSL 重启换 IP 也不受影响）。
- **TTS**：腾讯云 TextToVoice（`voice_type=1001` 智宇基础音），返回 16 kHz mono pcm_s16le
  直接交扬声器原语播放。凭据 `TENCENTCLOUD_SECRET_ID` / `_SECRET_KEY` 由操作者 shell 导出。

选 `custom` 后端同时避免了 `rbnx build` 去拉 Torch/CUDA/FunASR/Whisper。

## 部署与启动

前置：Python 3.10 环境 `bb_d1_robonix`（`ultralytics` + robonix/mcp 依赖，外加
`pip install tools/func_verify/lib/beingbeyond_d1_sdk-0.2.0-cp310-*.whl` —— 本仓自带该
wheel，`cp310` + `manylinux_2_17_x86_64`，故环境必须是 Python 3.10/x86_64），
`d1_arm`/`d1_hand`/`d1_camera` 硬件就绪且总线权限已配，YOLO 权重与手眼标定放在
`./models/`（`best.pt` + `handeye_calib.npz`，见该目录 README），并确认清单 `env:` 里的
`D1_DEPLOY_DIR` 指向本仓的实际路径。
如需语音，先起外部 PaddleSpeech ASR 服务。

```bash
cp .env.example .env && $EDITOR .env    # VLM 端点 + 腾讯云 TTS 凭据
set -a && . ./.env && set +a            # 导出到当前 shell（清单按 ${VAR} 展开）
rbnx setup "$PWD"                       # 登记 ${ROBONIX_SOURCE_PATH}
rbnx build -f robonix_manifest.yaml     # 各包 codegen（技能用 --mcp）
rbnx boot -v -f robonix_manifest.yaml   # 前台运行，日志写 rbnx-boot/logs/
```

另一终端验收：

```bash
rbnx caps -v          # 4 原语 + speech 为 ACTIVE；2 个技能为 INACTIVE（预期）
rbnx logs -d ./rbnx-boot/logs -l warn
rbnx chat             # 例："把红色方块叠到蓝色方块上" / "把方块按颜色分类"
rbnx shutdown -f robonix_manifest.yaml
```

技能类包停在 `INACTIVE` 是规范行为——首次 MCP 调用由 executor 触发 `CMD_ACTIVATE`。
`rbnx build` 摘要应为 `Failed: 0` / `Skipped: 0`，且 `Built` 等于清单实例数（7）。

**首次上机不要从运动开始**：先只跑只读功能（`d1_camera` 快照、`arm/get_state`、
`detect_objects` / `detect_cubes`），核对返回坐标合理、时间戳与坐标系正确，
再逐个加回运动实例验证单关节、手指开合与急停。

## 说明

- 抓取高度 `pick_z`、堆叠高度 `block_height`、`table_z_offset`、检测资产路径、VLM 端点、
  `grasp_feedback` 等在 `robonix_manifest.yaml` 的 skill `config:` 中调整。
- `detect_objects` / `verify_grasp` 需要 OpenAI 兼容的 VLM 端点（`VLM_BASE_URL` /
  `VLM_API_KEY` / `VLM_MODEL`）；未配置时这两个工具不可用，其余六个不受影响。
- 手眼标定与相机安装位置、桌面高度绑定；头部相机重装或桌面高度变化后必须重新标定。
- `tools/` 下为离线工具，不参与 `rbnx boot`；`tools/func_verify/` 用独立环境 `bb_d1_rbnx`
  直连 SDK 做硬件自检，与部署环境 `bb_d1_robonix` 相互独立。

## License

MulanPSL-2.0
