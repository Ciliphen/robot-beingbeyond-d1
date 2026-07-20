# robot-beingbeyond-d1

BeingBeyond D1 桌面机器人的 robonix 部署仓,核心是**灵巧手物体抓放技能**
(`vertical_grasp_object`)。技能向大模型暴露 6 个 MCP 工具,由 `rbnx chat`(文字)
或语音链路经 pilot 触发:

- `detect_cubes(class_filter)` — YOLO-OBB 检测桌面方块(红/蓝/绿/黄),返回 base 系抓取点 + 抓取角。**方块首选**。
- `detect_objects(instruction)` — VLM 开放词表检测任意物体(按颜色/形状/文字/空间描述),schema 同上。**非方块物体首选**。
- `pick_cube(position, angle_deg, gripper)` — 移动到位置,按角度/夹紧度抓起并夹住。
- `place_cube(position, angle_deg, gripper)` — 移动到位置松开放下;`z` 决定堆叠高度,成功后自动泊车让开视野。
- `stack_cubes(top_color, base_color, gripper)` — 单次调用完成"A 叠到 B 上":检测→抓→放。**堆叠首选**。
- `verify_grasp(instruction, mode, position)` — VLM 复检抓/放是否到位,闭环重试的依据。

`position` 为 base 坐标系(米)的 `"x,y"` / `"x,y,z"`,或命名位置(如 `中间`);半角/全角逗号均可。
检测几何走手眼标定单应 + 透视校正,深度为固定桌面 Z(两个检测器都不用 RealSense 深度流)。

## 结构

```
robot-beingbeyond-d1/
├── robonix_manifest.yaml   # 部署清单:系统服务 + 语音 + d1_arm/d1_hand/d1_camera + audio_driver + skill
├── soma.yaml               # D1 body model(urdf.path → ./urdf/)
├── .env.example            # VLM 配置模板(copy 为 .env:VLM_BASE_URL/API_KEY/MODEL)
├── urdf/
│   ├── robot_right_hand.urdf   # D1 URDF(供 soma / IK 用)
│   └── meshes/                 # URDF 引用的 STL(含 right_hand/)
├── primitives/             # 硬件原语(各自持有硬件,仅依赖 beingbeyond_d1_sdk wheel + robonix_api)
│   ├── d1_arm/             # 6-DOF 机械臂 + 头部(HeadArm 串口 /dev/ttyUSB0)
│   ├── d1_hand/            # 五指灵巧手(CAN can0)
│   └── d1_camera/          # RealSense RGB 抓帧(1280x720,喂检测)
├── skills/vertical_grasp_object_skill/   # ← 符合 robonix 规范的技能包(社区提交单元)
│   ├── package_manifest.yaml / config.spec / CAPABILITY.md
│   ├── scripts/{build.sh, start.sh}
│   ├── vertical_grasp_object_skill/      # main.py / controller.py / primitive_clients.py / detector.py / vlm_detector.py
│   ├── object_detect/ + block_grasp/     # vendored:YOLO/几何检测 + IK/FK/抓取运动
│   ├── models/                           # best.pt + handeye_calib.npz(机器专属,git-ignored)
│   └── capabilities/*.v1.toml + lib/.../srv/*.srv   # 6 个能力契约(codegen 成 gRPC)
└── tools/                  # 离线工具(不参与部署)
    ├── func_verify/        # D1 SDK 功能自检示例(独立 env bb_d1_rbnx,见其 README)
    ├── yolo_train/         # 方块检测数据标注+训练链路,产物 best.pt 拷进 skill models/
    ├── arm_calibration/    # 手眼标定 / 下垂补偿,产物 handeye_calib.npz
    └── vision.py           # RealSense 采集辅助
```

技能是纯 robonix **消费者**:`on_activate` 经 atlas 发现并连接 `d1_arm`/`d1_hand`/`d1_camera`
三个 primitive,通过其 gRPC 契约驱动;自身不开串口/CAN/RealSense。IK/FK、YOLO、手眼投影
均本地纯计算(`object_detect` + `block_grasp` 已 vendored 进包,FK/IK 来自
`beingbeyond_d1_sdk` wheel,无需依赖外部仓库)。`detect_objects` / `verify_grasp`
额外需要一个 OpenAI 兼容 VLM 端点。

## 语音

`rbnx boot` 会拉起语音链路:`audio_driver`(WSLg 下麦克风走 PulseAudio `pulse` 设备)+
`speech` 服务(仅 ASR,无 TTS)。ASR 用 `custom` 后端,把 16k/mono/pcm_s16le 转发到外部
PaddleSpeech 流式 WebSocket(`PADDLE_ASR_PORT=8090`,host 缺省解析到 WSL 网关=Windows 宿主)。
语音回复回退为文字 PILOT 事件。

## 本地测试

前置:Python 3.10 环境 `bb_d1_robonix`(含 D1 SDK wheel + robonix/mcp 依赖),
`d1_arm`/`d1_hand`/`d1_camera` 硬件就绪,YOLO 权重与手眼标定放在
`skills/vertical_grasp_object_skill/models/`(`best.pt` + `handeye_calib.npz`,见该目录 README)。
如需语音,先起外部 PaddleSpeech ASR 服务。

```bash
cp .env.example .env && $EDITOR .env       # 填 VLM 配置(detect_objects / verify_grasp 用)
rbnx build                                  # 各包 codegen(skill 用 --mcp)
rbnx boot                                   # 起系统服务 + 语音 + primitive + skill
rbnx chat                                   # 例:"把红色方块叠到蓝色方块上" / "把 0.2,0.1 的积木放到 中间"
```

## 说明

- 本仓自包含:`d1_arm`/`d1_hand`/`d1_camera` 原语与 soma/urdf 均随仓提供。向 syswonder
  社区提交的单元是 `skills/vertical_grasp_object_skill/` 技能包(见 robonix package catalog
  集成指南);原语、语音与部署清单是本仓的运行环境。
- 原语仅依赖 `beingbeyond_d1_sdk`(env `bb_d1_robonix` 里的 wheel)与 `robonix_api`,
  不依赖任何仓库源码;`d1_camera` 另用 `vision.RealSenseCamera`。
- 抓取高度 `pick_z`、堆叠高度 `block_height`、URDF、检测资产路径、VLM 端点、
  `table_z_offset` 等在 `robonix_manifest.yaml` 的 skill `config:` 中调整
  (字段见 `skills/vertical_grasp_object_skill/config.spec`,能力细节见 `CAPABILITY.md`)。
- `tools/` 下为离线工具,不参与 `rbnx boot`;`tools/func_verify/` 用独立环境
  `bb_d1_rbnx` 直连 SDK 做硬件自检,与部署环境 `bb_d1_robonix` 相互独立。

