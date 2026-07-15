# robot-beingbeyond-d1

BeingBeyond D1 桌面机器人的 robonix 部署仓,当前提供**灵巧手积木抓放技能**
(`grasp_cube`)。技能对大模型暴露两个能力,均由 `rbnx chat` 经 pilot 触发:

- `pick_cube(position)` — 机械臂移动到指定位置,抓起那里的积木并夹住。
- `place_cube(position)` — 机械臂移动到指定位置,松开夹住的积木放下。

`position` 为 base 坐标系(米)的坐标串 `"x,y"` / `"x,y,z"`,或命名位置(如 `中间`)。
无视觉:位置由调用方给定,不做检测。

## 结构

```
robot-beingbeyond-d1/
├── robonix_manifest.yaml        # 部署:系统服务 + d1_arm/d1_hand primitive + grasp_cube skill
├── soma.yaml                    # D1 body model(urdf.path → ./urdf/）
├── urdf/
│   ├── robot_right_hand.urdf    # D1 URDF(供 soma / IK 用）
│   └── meshes/                  # URDF 引用的 STL(含 right_hand/）
├── primitives/                  # 硬件原语(自包含,仅依赖 beingbeyond_d1_sdk wheel）
│   ├── d1_arm/                  # 6-DOF 机械臂(HeadArm 串口）
│   └── d1_hand/                 # 五指灵巧手(CAN）
├── .env.example                 # VLM 配置模板(copy 为 .env)
└── skills/grasp_cube_skill/     # ← 符合 robonix 规范的技能包(社区提交单元)
    ├── package_manifest.yaml
    ├── config.spec
    ├── scripts/{build.sh, start.sh}
    ├── grasp_cube_skill/{__init__.py, main.py, controller.py, primitive_clients.py}
    └── capabilities/
        ├── lib/grasp_cube/srv/{PickCube.srv, PlaceCube.srv}
        ├── pick_cube.v1.toml
        ├── place_cube.v1.toml
        └── driver.v1.toml
```

技能是纯 robonix **消费者**:`on_activate` 时经 atlas 发现并连接 `d1_arm`/`d1_hand`
两个 primitive,通过其 gRPC 契约驱动;自身不开串口/CAN,也不用相机。IK/FK 与抓取
运动逻辑复用 `Beingbeyond_D1` 仓的 `block_grasp` 栈(经 `BEINGBEYOND_PATH` 导入)。

## 本地测试

前置:Python 3.10 环境 `bb_d1_robonix`(含 D1 SDK + robonix/mcp 依赖),`Beingbeyond_D1`
仓在 `$HOME/Beingbeyond_D1`,`d1_arm`/`d1_hand` 硬件就绪。

```bash
cp .env.example .env && $EDITOR .env      # 填 VLM 配置
rbnx build                                 # 各包 codegen（skill 用 --mcp）
rbnx boot                                  # 起系统服务 + primitive + skill
rbnx chat                                  # 输入“请把积木从 0.2,0.1 抓起来放到 中间”
```

## 说明

- 本仓自包含:`d1_arm`/`d1_hand` 原语与 soma/urdf 均已随仓提供。向 syswonder 社区
  提交的单元是 `skills/grasp_cube_skill/` 这个技能包(见 robonix package catalog
  集成指南);原语与部署清单是本仓的运行环境。
- 原语仅依赖 `beingbeyond_d1_sdk`(Python 3.10 env `bb_d1_robonix` 里的 wheel)与
  `robonix_api`,不依赖任何仓库源码。
- 硬件抓取高度 `pick_z`、URDF 等在 `robonix_manifest.yaml` 的 skill `config:` 中调整
  (字段见 `skills/grasp_cube_skill/config.spec`)。
