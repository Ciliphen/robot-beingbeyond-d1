# primitive-beingbeyond-d1-hand-rbnx

`robonix.primitive.beingbeyond.d1.hand` — BeingBeyond D1 五指灵巧手（Linker 六轴）的
robonix 原语。直连 `beingbeyond_d1_sdk` 的 DexHand CAN 链路，**无 ROS 后端**。

## 能力

| 能力约定 | 传输 | 作用 |
|---|---|---|
| `robonix/primitive/hand/driver` | gRPC | 生命周期（`CMD_INIT` / `ACTIVATE` / `DEACTIVATE` / `SHUTDOWN`） |
| `robonix/primitive/hand/info` | gRPC | 每个轴的所属手指与类型，供消费者做手指映射 |
| `robonix/primitive/hand/move_joint` | gRPC | 轴级位置控制 |
| `robonix/primitive/hand/set_joint_speed_limits` | gRPC | 逐轴速度上限 |
| `robonix/primitive/hand/set_joint_torque_limits` | gRPC | 逐轴力矩上限 |
| `robonix/primitive/hand/get_state` | gRPC | 读当前各轴角度 |

手指级契约（`move_finger` / `set_finger_*`）与 `state_joint` / `state_finger`
（`topic_out`，需要本部署未运行的 ROS 后端）**有意不提供**；消费者用 `info` 拿到
轴↔手指映射后走 `move_joint`。

## 配置

字段、单位、默认值见根目录 `config.spec`。要点：

- `hand_type`（`right` | `left`，默认 `right`）— 选择 DexHand SDK 内的轴符号/映射。
- `can_iface`（默认 `can0`）— 手所接的 CAN 网卡。
- `baudrate`（默认 `1000000`）— CAN 位速率，须与手部控制器固件一致。

每个字段都有对应的 `D1_HAND_*` 环境变量回退。

## 依赖与权限

- Python **3.10**（wheel 是 `cp310` + `manylinux_2_17_x86_64`，非 3.10 装不上）。
- `beingbeyond_d1_sdk`（≥ 0.2.0）与 `robonix_api`。SDK wheel 随 `robot-beingbeyond-d1`
  部署仓的 `tools/func_verify/lib/` 提供，`pip install` 该 whl 即可。
- CAN 权限：DexHand 在 init 阶段会执行 `ip link set …` 拉起接口。若不以 root 运行，
  需预先手动拉起接口，或导出 `PREFLIGHT_SUDO_PASS`。
- `scripts/start.sh` 默认用 `$HOME/miniconda3/envs/bb_d1_robonix/bin/python3`，
  可用 `BLOCK_GRASP_PYTHON` 覆盖。

## 构建与启动

```bash
bash scripts/build.sh      # rbnx codegen，生成 gRPC stub
bash scripts/start.sh      # 或由 rbnx boot 经部署清单拉起
rbnx caps -v | grep hand   # 验证 6 条能力已注册且 provider 为 ACTIVE
```

关闭由 Driver 的 `CMD_SHUTDOWN` 处理（`on_shutdown` 停止手部输出），无额外清理动作，
故不提供 `scripts/stop.sh`。

## 安全

- 先用 `info` / `get_state` 做只读验收，再下发 `move_joint`。
- 建议先用 `set_joint_torque_limits` 降低力矩上限再做首次开合，避免夹伤手指或损坏指节。
- 提供方在输入超时、退出或收到 `CMD_SHUTDOWN` 时停止硬件输出。

## 目录

```
package_manifest.yaml   config.spec   README.md
scripts/{build.sh, start.sh}
d1_hand/{__init__.py, main.py}
```

## License

MulanPSL-2.0
