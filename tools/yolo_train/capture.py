#!/usr/bin/env python3
"""
数据集采集工具 —— D1 相机 + 头部控制 + 实时 YOLO 检测预览 + 拍照存图。

训练流程的第 0 步：拍训练照，顺便实时看当前模型的检测效果。
存图落到 dataset/raw/，正是 json2label.py 的输入目录。

源自 Beingbeyond_D1 `block_grasp/test_detect_fixed.py`，import/路径已改为
相对本目录（vision.py / detect.py 是同目录兄弟）。"fixed" = 推理前把相机
RGB 转 BGR，匹配训练数据格式（否则颜色类别会认错）。

⚠ 真机脚本：直接开 /dev/ttyUSB0 串口、RealSense、cv2 窗口，需要
`beingbeyond_d1_sdk` + `pyrealsense2`（不在本仓库，需在真机环境跑，见 README）。

键盘:
    A / D     头部 yaw   左 / 右
    W / S     头部 pitch 上 / 下
    H         头部回标定位 (yaw=-10°, pitch=35°)
    SPACE     拍照存到 dataset/raw/
    Q / ESC   退出

模型可选：找不到权重时进入纯预览+采集模式（不做检测叠加），采集训练照不需要模型。
机械臂默认停靠在 HOME_POSITION（与 skill 的 move_home 一致），让开相机视野。

用法:
    conda activate bb_d1
    python tools/yolo_train/capture.py                 # 无模型也能跑：纯预览+采集
    python tools/yolo_train/capture.py --no-head       # 无机械臂，仅相机采集
    python tools/yolo_train/capture.py --model runs/train/weights/best.pt  # 带检测预览
"""
from __future__ import annotations

import argparse
import math
import os
import threading
import time
from typing import List, Optional

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R

from vision import RealSenseCamera
from detect import detect_objects_in_frame, draw_box, load_model

# ── Constants ──────────────────────────────────────────────────────────────
HEAD_YAW_STEP = 2.0
HEAD_PITCH_STEP = 1.0
HEAD_YAW_LIMIT = 90.0
HEAD_PITCH_LIMIT = 60.0
WINDOW = "D1  |  A/D yaw  W/S pitch  H home  SPACE save  Q quit"

# 和 click_goto / calibrate_handeye 保持一致的标定头部姿态
CALIB_HEAD_YAW_DEG = -10.0
CALIB_HEAD_PITCH_DEG = 35.0

# 机械臂默认停靠位（与 vertical_grasp_object_skill 的 move_home 完全一致），
# 采集时让机械臂停在这里、让开相机视野。
HOME_POSITION = (0.12, -0.23, 0.32)
# 末端目标朝向 + 重力下垂补偿——与 skill controller 的 move_home 一致
# （值取自 block_grasp.config；此脚本不 import block_grasp，故内联）。
EE_ROLL_DEG = 178.0
EE_PITCH_DEG = 61.0
EE_YAW_DEG = -175.0
GRAVITY_SAG_FACTOR = 0.3   # m of sag per m³ of horizontal distance


class HeadController:

    def __init__(self, dev="/dev/ttyUSB0", urdf_path="", baudrate=1_000_000,
                 init_yaw_deg=CALIB_HEAD_YAW_DEG, init_pitch_deg=CALIB_HEAD_PITCH_DEG):
        if not urdf_path:
            from beingbeyond_d1_sdk.urdf_path import get_default_urdf_path
            urdf_path = get_default_urdf_path()
        from beingbeyond_d1_sdk.head_arm import HeadArmRobot
        from beingbeyond_d1_sdk.pin_kinematics import D1Kinematics, D1KinematicsConfig
        self._r = HeadArmRobot(urdf_path=urdf_path, dev=dev, baudrate=baudrate)
        self._kin = D1Kinematics(D1KinematicsConfig(urdf_path=urdf_path))

        # ── Safe posture (same as test_ee_teleop.py) ────────────────────
        q_init = np.radians([0, 0, 0, -60, 60, 0, 0, 0])
        self._r.set_positions(q_init)
        self._r.wait_until_reached(q_init, active_joint_indices=range(8))
        time.sleep(0.3)

        # ── Set head to calibration position ────────────────────────────
        self._yaw = math.radians(init_yaw_deg)
        self._pitch = math.radians(init_pitch_deg)
        q = np.asarray(self._r.get_positions(), dtype=float)
        q[0] = self._yaw
        q[1] = self._pitch
        self._r.set_positions(q)
        self._r.wait_until_reached(q, active_joint_indices=[0, 1])
        time.sleep(0.3)

        # ── Park arm at HOME_POSITION, out of camera view ───────────────
        # 复刻 skill move_home 的位姿：目标朝向 R_target（EE RPY）+ 重力下垂补偿，
        # 否则臂停的位置/朝向和 move_home 对不上。
        q_full = np.asarray(self._r.get_positions(), dtype=float)
        q_head, q_arm = self._kin.split_q(q_full)
        hx, hy, hz = HOME_POSITION
        R_target = R.from_euler(
            "xyz", [EE_ROLL_DEG, EE_PITCH_DEG, EE_YAW_DEG], degrees=True
        ).as_matrix()
        z_sag = GRAVITY_SAG_FACTOR * math.sqrt(hx * hx + hy * hy) ** 3
        T_lift = np.eye(4)
        T_lift[:3, :3] = R_target
        T_lift[:3, 3] = [hx, hy, hz + z_sag]
        q_hs, q_as, err, _ = self._kin.ik_T_ee_with_arm_only(
            T_lift, q_head, q_arm,
        )
        print(f"[Init] HOME IK err={err:.4f} (target={HOME_POSITION})", flush=True)
        if err < 0.05:
            cmd = np.concatenate([q_hs, q_as])
            cmd[0] = math.radians(init_yaw_deg)
            cmd[1] = math.radians(init_pitch_deg)
            self._r.set_positions(cmd)
            self._r.wait_until_reached(cmd, active_joint_indices=range(2, 8))
            T_ach = self._kin.ee_in_base(*self._kin.split_q(
                np.asarray(self._r.get_positions(), dtype=float)))
            print(f"[Init] parked at EE={np.round(T_ach[:3, 3], 3)}", flush=True)
        else:
            print(f"[Init] ⚠ HOME IK 未收敛 (err={err:.4f} ≥ 0.05)，臂停在原姿态未移动。", flush=True)

    @property
    def yaw(self): return math.degrees(self._yaw)

    @property
    def pitch(self): return math.degrees(self._pitch)

    def step(self, dyaw=0.0, dpitch=0.0):
        self._yaw += math.radians(dyaw)
        self._pitch += math.radians(dpitch)
        self._yaw = max(-math.radians(HEAD_YAW_LIMIT), min(math.radians(HEAD_YAW_LIMIT), self._yaw))
        self._pitch = max(-math.radians(HEAD_PITCH_LIMIT), min(math.radians(HEAD_PITCH_LIMIT), self._pitch))
        q = self._r.get_positions()
        q[0] = self._yaw
        q[1] = self._pitch
        self._r.set_positions(q)

    def home(self):
        self._yaw = math.radians(CALIB_HEAD_YAW_DEG)
        self._pitch = math.radians(CALIB_HEAD_PITCH_DEG)
        q = self._r.get_positions()
        q[0] = self._yaw
        q[1] = self._pitch
        self._r.set_positions(q)

    def close(self):
        self._r.close()


def main():
    _HERE = os.path.dirname(os.path.abspath(__file__))

    p = argparse.ArgumentParser(description="D1 camera + head + detect + capture")
    p.add_argument("--model", default=os.path.join(_HERE, "runs", "train", "weights", "best.pt"))
    p.add_argument("--conf", type=float, default=0.85)
    p.add_argument("--iou", type=float, default=0.45)
    p.add_argument("--cam-width", type=int, default=1280)
    p.add_argument("--cam-height", type=int, default=720)
    p.add_argument("--cam-fps", type=int, default=30)
    p.add_argument("--arm-dev", default="/dev/ttyUSB0")
    p.add_argument("--arm-baud", type=int, default=1_000_000)
    p.add_argument("--urdf", default="")
    p.add_argument("--no-head", action="store_true")
    p.add_argument("--infer-w", type=int, default=416)
    p.add_argument("--infer-h", type=int, default=234)
    p.add_argument("--save-dir", default=os.path.join(_HERE, "dataset", "raw"))
    args = p.parse_args()

    # ── Camera ─────────────────────────────────────────────────────────
    print("[Init] Camera ...")
    cam = RealSenseCamera(width=args.cam_width, height=args.cam_height, hz=args.cam_fps)
    print(f"       Intel RealSense D435i {args.cam_width}x{args.cam_height}@{args.cam_fps}fps")

    # ── Head ───────────────────────────────────────────────────────────
    head: Optional[HeadController] = None
    if not args.no_head:
        print(f"[Init] Head on {args.arm_dev} ...")
        head = HeadController(dev=args.arm_dev, urdf_path=args.urdf, baudrate=args.arm_baud)
        print(f"       yaw={head.yaw:+.0f}°  pitch={head.pitch:+.0f}°")

    # ── YOLO model (optional: skip detection if the weights are missing) ──
    model = None
    if os.path.isfile(args.model):
        print(f"[Init] Model: {args.model}")
        model = load_model(args.model, device="cpu")
        print(f"       Classes: {list(model.names.values())}")
    else:
        print(f"[Init] 未找到模型 {args.model}")
        print("       → 仅预览 + 采集模式（不做检测叠加）。训练出 best.pt 后用 --model 指定即可。")

    # ── Save dir ───────────────────────────────────────────────────────
    os.makedirs(args.save_dir, exist_ok=True)
    # 从已有文件中找到最大编号，避免覆盖
    existing = [int(f.split(".")[0]) for f in os.listdir(args.save_dir)
                if f.endswith(".jpg") and f.split(".")[0].isdigit()]
    saved = max(existing) + 1 if existing else 0
    print(f"[Save] 从 {saved:05d}.jpg 开始编号 ({len(existing)} 张已有)")

    # ── Shared state (main ↔ inference thread) ─────────────────────────
    lock = threading.Lock()
    latest_frame: Optional[np.ndarray] = None
    latest_dets: List = []
    latest_t = 0.0
    running = True

    def _worker():
        nonlocal latest_dets, latest_t
        infer_size = (args.infer_w, args.infer_h)
        while running:
            with lock:
                f = latest_frame
            if f is None:
                time.sleep(0.01)
                continue
            try:
                f_bgr = cv2.cvtColor(f, cv2.COLOR_RGB2BGR)
                small = cv2.resize(f_bgr, infer_size, interpolation=cv2.INTER_AREA)
                t0 = time.time()
                dets_s = detect_objects_in_frame(model, small, args.conf, args.iou)
                dt = time.time() - t0
                sx = f.shape[1] / small.shape[1]
                sy = f.shape[0] / small.shape[0]
                with lock:
                    latest_dets = [((u*sx, v*sy, w*sx, h*sy, r), s, c, n)
                                   for (u, v, w, h, r), s, c, n in dets_s]
                    latest_t = dt
            except Exception as e:
                print(f"[Worker] {e}")

    if model is not None:
        threading.Thread(target=_worker, daemon=True).start()

    # ── Banner ─────────────────────────────────────────────────────────
    print("\n" + "=" * 60)
    print("  A/D yaw  |  W/S pitch  |  H home  |  SPACE save  |  Q quit")
    print(f"  Saving to: {args.save_dir}/")
    print("=" * 60 + "\n")

    # ── Main loop ──────────────────────────────────────────────────────
    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW, args.cam_width, args.cam_height)
    frames = 0

    try:
        while True:
            t0 = time.time()
            rgb, _ = cam.get_aligned_frames(filtered=False)

            # Feed worker
            with lock:
                latest_frame = rgb
                dets = list(latest_dets)
                t_inf = latest_t

            # Annotate
            vis = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            for (u, v, w, h, r), score, _, name in dets:
                draw_box(vis, u, v, w, h, np.rad2deg(r), f"{name}: {score:.2f}")
                cv2.circle(vis, (int(u), int(v)), 4, (0, 0, 255), -1)

            # Overlay
            fps = 1.0 / max(time.time() - t0, 1e-6)
            if model is not None:
                cv2.putText(vis, f"FPS: {fps:.1f}  infer: {t_inf*1000:.0f}ms",
                            (15, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)
                cv2.putText(vis, f"Dets: {len(dets)}  Saved: {saved}",
                            (15, 80), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
            else:
                cv2.putText(vis, f"FPS: {fps:.1f}  PREVIEW (no model)",
                            (15, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)
                cv2.putText(vis, f"Saved: {saved}",
                            (15, 80), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
            if head:
                cv2.putText(vis, f"Head: yaw={head.yaw:+.0f}  pitch={head.pitch:+.0f}",
                            (15, 120), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 200, 0), 2)

            cv2.imshow(WINDOW, vis)

            # Keys
            key = cv2.waitKey(5) & 0xFF
            if key == 27 or key in (ord('q'), ord('Q')):
                break
            if key == 32:  # SPACE
                path = os.path.join(args.save_dir, f"{saved:05d}.jpg")
                cv2.imwrite(path, cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
                print(f"[Save] {path}")
                saved += 1
            if head:
                if key in (ord('a'), ord('A')):   head.step(dyaw=+HEAD_YAW_STEP)
                elif key in (ord('d'), ord('D')): head.step(dyaw=-HEAD_YAW_STEP)
                elif key in (ord('w'), ord('W')): head.step(dpitch=-HEAD_PITCH_STEP)
                elif key in (ord('s'), ord('S')): head.step(dpitch=+HEAD_PITCH_STEP)
                elif key in (ord('h'), ord('H')): head.home(); print("[Head] 0°")

            frames += 1

    except KeyboardInterrupt:
        print("\n[Exit] Interrupted.")
    finally:
        running = False
        print(f"[Exit] {frames} frames, {saved} saved.")
        cam.stop()
        if head:
            head.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
