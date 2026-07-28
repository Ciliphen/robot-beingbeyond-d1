# skill-vertical-grasp-object-rbnx

`robonix.skill.vertical_grasp_object` — 用头部 RGB-D 相机检测桌面物体（YOLO-OBB 检测
彩色方块，或经视觉大模型开放词表检测任意物体），再用 D1 六自由度机械臂 + 五指灵巧手
垂直抓取 / 放置，并可复核结果。

调用语义、参数、约束、错误与闭环建议的完整说明见 [`CAPABILITY.md`](./CAPABILITY.md)
（注册阶段读入交给 Atlas，pilot 通过 `read_capability_doc` 取用）。本文件只讲部署与构建。

## 能力（8 个 MCP 工具 + 生命周期）

| 能力约定 | 传输 | 作用 |
|---|---|---|
| `robonix/skill/vertical_grasp_object/driver` | gRPC | 生命周期 |
| `.../detect_objects` | MCP | VLM 开放词表检测，按自然语言描述找物体，返回 base 系位置 + 抓取角。**通用首选** |
| `.../detect_cubes` | MCP | YOLO-OBB 检测彩色方块，同 schema。**方块首选**，也作 VLM 未配置时的退路 |
| `.../pick_cube` | MCP | 移到位置抓起并夹住，可选抓取角与夹紧度 |
| `.../place_cube` | MCP | 移到位置松开放下，成功后自动泊车让开视野 |
| `.../stack_cubes` | MCP | 单次调用完成"A 色叠到 B 色上"：检测→抓→放 |
| `.../put_cube_in_container` | MCP | 单次调用把指定颜色方块放进固定容器位 |
| `.../sort_cubes` | MCP | 单次调用把桌面所有方块按颜色分拣到各自固定位 |
| `.../verify_grasp` | MCP | VLM 复核抓 / 放是否到位，闭环重试的依据 |

`position` 为 base 坐标系（米）的 `"x,y"` / `"x,y,z"`，或命名位置；半角/全角逗号均可。

## 运行时依赖

技能是纯 robonix **消费者**，`on_activate` 经 atlas 发现并连接下列原语，全程通过 gRPC
驱动，自身不开串口 / CAN / RealSense：

| 原语 | 用到的契约 |
|---|---|
| `robonix.primitive.beingbeyond.d1.arm` | `arm/get_state`、`arm/move_joint`、`arm/set_head` |
| `robonix.primitive.beingbeyond.d1.hand` | `hand/move_joint`、`hand/get_state`、`hand/info` |
| `robonix.primitive.beingbeyond.d1.camera` | `camera/snapshot` |

IK/FK、YOLO 推理、手眼投影均为本地纯计算（`object_detect` + `block_grasp` 已随包提供，
FK/IK 来自 `beingbeyond_d1_sdk` wheel）。`detect_objects` 与 `verify_grasp` 额外需要一个
OpenAI 兼容的 VLM 端点；未配置时这两个工具不可用，其余六个不受影响。

## 资产（必需，未随包提交）

`models/` 下需要两个本机专属文件，见该目录 `README.md`：

| 文件 | 内容 | 来源 |
|---|---|---|
| `best.pt` | YOLO-OBB 方块检测权重（约 113 MB） | 用方块检测训练链路自行训练 |
| `handeye_calib.npz` | 相机→base 手眼单应 + 头部位姿 + 桌面 Z | 在**本台机器**上跑手眼标定 |

标定与相机安装位置、桌面高度绑定；头部相机重装或桌面高度变化后必须重新标定。

## 配置

字段、单位、默认值、失败条件见根目录 `config.spec`。常调的几项：

- `pick_z` / `block_height` — 抓取高度与单个方块高度（`stack_cubes` 的落点靠它）。
- `table_z_offset` — 对标定桌面 Z 的常量修正（全桌统一偏高就填负值）。
- `model_path` / `calib_path` — 留空则用包内 `./models/` 下的默认名。
- `vlm_base_url` / `vlm_api_key` / `vlm_model` — 留空回退环境变量
  `VLM_BASE_URL` / `VLM_API_KEY` / `VLM_MODEL`；模型须支持图像输入。
- `vlm_grasp_height` — VLM 无深度，按此高度假定抓取 Z。
- `verify_match_radius` — `verify_grasp` 判"在该位置"的半径。
- `grasp_feedback` — 是否用手指角度反馈判断抓取成功。反馈误报会让 `sort_cubes` 松手丢块，
  误报时置 `false`。

## 构建与启动

```bash
bash scripts/build.sh   # rbnx codegen --mcp，生成 gRPC stub + MCP Request/Response 类
rbnx caps -v | grep vertical_grasp_object
rbnx chat               # 例："把红色方块叠到蓝色方块上"
```

技能类包在 `rbnx boot` 后停在 `INACTIVE`，首次 MCP 调用由 executor 触发 `CMD_ACTIVATE`——
这是预期状态，不是启动失败。

`scripts/start.sh` 默认用 `$HOME/miniconda3/envs/bb_d1_robonix/bin/python3`（含
`beingbeyond_d1_sdk` wheel、`ultralytics`、`numpy`/`scipy`、`robonix_api`、`grpcio`），
可用 `VERTICAL_GRASP_OBJECT_PYTHON` 覆盖。无额外清理动作，故不提供 `scripts/stop.sh`。

## 安全

- 首次上机先只跑检测（`detect_objects` / `detect_cubes`），核对返回坐标合理后再下发抓取。
- 抓取前确认工作范围内无人手、无易碎物；`pick_z` 设置过低会让手指压到桌面。
- 检测前会先把手臂归位到 HOME 让开相机视野；这一步会产生真实运动。
- 技能不做碰撞检查，桌面上的既有堆叠需由调用方规划顺序。

## 目录

```
package_manifest.yaml   config.spec   CAPABILITY.md   README.md
scripts/{build.sh, start.sh}
capabilities/*.v1.toml + capabilities/lib/vertical_grasp_object/srv/*.srv
vertical_grasp_object_skill/{main.py, controller.py, primitive_clients.py, detector.py, vlm_detector.py}
object_detect/    # YOLO-OBB 检测 + 几何
block_grasp/      # IK/FK、坐标变换、抓取运动
models/           # best.pt + handeye_calib.npz（本机专属，未提交）
```

## License

MulanPSL-2.0
