# primitive-beingbeyond-d1-camera-rbnx

`robonix.primitive.beingbeyond.d1.camera` — BeingBeyond D1 头部 Intel RealSense D435i
RGB-D 相机的 robonix 原语。直连 `pyrealsense2`，**无 ROS 后端**，只提供按需取帧。

## 能力

| 能力约定 | 传输 | 作用 |
|---|---|---|
| `robonix/primitive/camera/driver` | gRPC | 生命周期（`CMD_INIT` / `ACTIVATE` / `DEACTIVATE` / `SHUTDOWN`） |
| `robonix/primitive/camera/snapshot` | gRPC | 单张 RGB 帧，JPEG 编码的 `sensor_msgs/Image` |
| `robonix/primitive/camera/depth_snapshot` | MCP | 单张深度帧，归一化为 JPEG |

`rgb` / `depth` / `intrinsics` / `extrinsics` 的 `topic_out` 流式契约**有意不提供**——
本部署没有 ROS 后端。消费者按需拉帧即可。

## 配置

字段、单位、默认值见根目录 `config.spec`。要点：

- `width` / `height`（默认 `1280` / `720`）— **必须保持在手眼标定所用的分辨率**，
  否则下游检测的像素→世界单应失效。
- `fps`（默认 `30`）— 底层流帧率；snapshot 只按需取单帧。
- `frame_id`（默认 `camera`）— 写入 `sensor_msgs/Image` header 的 frame_id。

每个字段都有对应的 `D1_CAMERA_*` 环境变量回退。

## 依赖与权限

- Python ≥ 3.10，装有 `pyrealsense2` 与 `robonix_api`。
- USB 权限：RealSense 需要 udev 规则（`librealsense` 的 `99-realsense-libusb.rules`），
  否则以普通用户打不开设备。
- 相机需接 **USB 3.0** 口；USB 2.0 下 1280x720@30 不可用。
- `scripts/start.sh` 默认用 `$HOME/miniconda3/envs/bb_d1_robonix/bin/python3`，
  可用 `BLOCK_GRASP_PYTHON` 覆盖。

## 构建与启动

```bash
bash scripts/build.sh        # rbnx codegen（--mcp：depth_snapshot 是 MCP 契约）
bash scripts/start.sh        # 或由 rbnx boot 经部署清单拉起
rbnx caps -v | grep camera   # 验证 3 条能力已注册且 provider 为 ACTIVE
```

`init` 会打开相机并**确认取到一帧**后才报 ready，所以启动失败即代表硬件/权限问题。
关闭由 Driver 的 `CMD_SHUTDOWN` 处理，无额外清理动作，故不提供 `scripts/stop.sh`。

## 说明

按需 `snapshot` 可能拿到 RealSense 队列里积压的旧帧，取帧前会先 drain 队列，
确保返回的是当前画面。这一点对"移动头部后立刻拍照"的调用链是必要的。

## 目录

```
package_manifest.yaml   config.spec   README.md
scripts/{build.sh, start.sh}
d1_camera/{__init__.py, main.py, vision.py}
```

`vision.py`（`RealSenseCamera` 封装）随包提供，包自包含。

## License

MulanPSL-2.0
