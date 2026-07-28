# primitive-beingbeyond-d1-arm-rbnx

`robonix.primitive.beingbeyond.d1.arm` — BeingBeyond D1 六自由度机械臂 + 二自由度头部云台的
robonix 原语。直连 `beingbeyond_d1_sdk` 的 HeadArm 串口链路，**无 ROS 后端**。

## 能力

| 能力约定 | 传输 | 作用 |
|---|---|---|
| `robonix/primitive/arm/driver` | gRPC | 生命周期（`CMD_INIT` / `ACTIVATE` / `DEACTIVATE` / `SHUTDOWN`） |
| `robonix/primitive/arm/info` | gRPC | 关节数、名称、限位等静态信息 |
| `robonix/primitive/arm/move_joint` | gRPC | 6 关节绝对角度控制（rad） |
| `robonix/primitive/arm/get_state` | gRPC | 读当前关节角 / 末端位姿 |
| `robonix/primitive/arm/move_pose` | gRPC | 笛卡尔位姿，走 SDK 的 arm-only IK |
| `robonix/primitive/arm/set_head` | gRPC | 头部云台 yaw / pitch 定位 |

只暴露 6 个臂关节；2 个头关节共用同一串口，由 `set_head` 单独驱动，`move_joint`
期间保持当前位姿。状态读取是 rpc（无 ROS 后端，故不提供 `state_joint` 的 `topic_out` 流）。

## 配置

字段、单位、默认值与失败条件见根目录 `config.spec`。要点：

- `dev`（默认 `/dev/ttyUSB0`）— HeadArm 串口设备。**臂与头共用这一个口**，不要与其它进程同时打开。
- `baudrate`（默认 `1000000`）— 须与控制器固件一致。
- `urdf_path`（默认 `""`）— `move_pose` IK 用的 URDF；留空回退 SDK 自带模型。
- `ik_pos_tol`（默认 `0.05` m）— IK 位置容差，超差的解被拒绝。

每个字段都有对应的 `D1_ARM_*` 环境变量回退。

## 依赖与权限

- Python **3.10**（wheel 是 `cp310` + `manylinux_2_17_x86_64`，非 3.10 装不上）。
- `beingbeyond_d1_sdk`（≥ 0.2.0）与 `robonix_api`。SDK wheel 随 `robot-beingbeyond-d1`
  部署仓的 `tools/func_verify/lib/` 提供，`pip install` 该 whl 即可。
- 串口权限：`sudo chmod 666 /dev/ttyUSB0`，或把用户加入 `dialout` 组。
- `scripts/start.sh` 默认用 `$HOME/miniconda3/envs/bb_d1_robonix/bin/python3`，
  可用 `BLOCK_GRASP_PYTHON` 覆盖。

## 构建与启动

```bash
bash scripts/build.sh     # rbnx codegen，生成 gRPC stub
bash scripts/start.sh     # 或由 rbnx boot 经部署清单拉起
rbnx caps -v | grep arm   # 验证 6 条能力已注册且 provider 为 ACTIVE
```

关闭由 Driver 的 `CMD_SHUTDOWN` 处理（`on_shutdown` 会 `robot.close()` 释放串口），
无额外清理动作，故不提供 `scripts/stop.sh`。

## 安全

- 首次上机先做只读验收（`get_state` / `info`），确认硬件急停可独立断臂再下发运动。
- 提供方在输入超时、退出或收到 `CMD_SHUTDOWN` 时停止硬件输出。
- `move_pose` 只在 IK 收敛且误差 ≤ `ik_pos_tol` 时下发，否则返回错误而不动作。

## 目录

```
package_manifest.yaml   config.spec   README.md
scripts/{build.sh, start.sh}
d1_arm/{__init__.py, main.py}
```

## License

MulanPSL-2.0
