# skill-hand-gesture-rbnx

`robonix.skill.hand_gesture` — 让 BeingBeyond D1 五指灵巧手"跳舞"：所有手指按节奏
开合摆动指定时长，结束后回到张开位。单个 MCP 工具，纯手部动作，不动机械臂、不用相机。

调用语义、参数、边界与依赖的完整说明见 [`CAPABILITY.md`](./CAPABILITY.md)（注册阶段
读入交给 Atlas，pilot 通过 `read_capability_doc` 取用）。本文件只讲部署与构建。

## 能力

| 能力约定 | 传输 | 作用 |
|---|---|---|
| `robonix/skill/hand_gesture/driver` | gRPC | 生命周期 |
| `robonix/skill/hand_gesture/gesture_dance` | MCP | 跳舞 `duration_s` 秒（默认 3，上限 30） |

## 运行时依赖

技能是纯 robonix **消费者**，`on_activate` 经 atlas 发现并连接下列原语，全程通过 gRPC
驱动，自身不开 CAN 总线：

| 原语 | 用到的契约 |
|---|---|
| `robonix.primitive.beingbeyond.d1.hand` | `hand/move_joint`、`hand/info` |

原语未上线时 `on_activate` 返回 `Deferred` 等待重试，不会直接失败。

## 配置

无。跳舞时长是逐次调用的 MCP 参数，摆幅（0.1）、步长间隔（0.1 s）与起始位姿固定为
SDK `gesture_dance` 默认值。部署清单里传 `config: {}`（见根目录 `config.spec`）。

## 构建与启动

```bash
bash scripts/build.sh   # rbnx codegen --mcp，生成 MCP Request/Response 类
rbnx caps -v | grep hand_gesture
rbnx chat               # 例："动动手指" / "跳个舞"
```

技能类包在 `rbnx boot` 后停在 `INACTIVE`，首次 MCP 调用由 executor 触发 `CMD_ACTIVATE`——
这是预期状态，不是启动失败。

`scripts/start.sh` 默认用 `$HOME/miniconda3/envs/bb_d1_robonix/bin/python3`（含
`robonix_api` + `grpcio`），可用 `HAND_GESTURE_PYTHON` 覆盖。无额外清理动作，
故不提供 `scripts/stop.sh`。

## 安全

跳舞是空载手势，不夹取任何物体。开始前确认手周围无异物、手指不与桌面或机械臂干涉。
任何中途出错都会把手命令到完全张开，不会停在半握状态。

## 目录

```
package_manifest.yaml   config.spec   CAPABILITY.md   README.md
scripts/{build.sh, start.sh}
capabilities/{driver.v1.toml, gesture_dance.v1.toml, lib/hand_gesture/srv/GestureDance.srv}
hand_gesture_skill/{__init__.py, main.py, primitive_clients.py}
```

## License

MulanPSL-2.0
