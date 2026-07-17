#!/usr/bin/env python3
"""Quick test: click on image → see computed world coordinates.

用法:
    python tools/arm_calibration/test_calib.py                    # 默认加载 handeye_calib.npz
    python tools/arm_calibration/test_calib.py --calib handeye_calib.20260717_095620.npz  # 回溯某份备份
"""
import argparse, math, os, sys, time
import cv2, numpy as np

# vision.py 是 tools/ 下共用的相机封装（父目录）；beingbeyond_d1_sdk 走真机环境的安装。
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(_HERE))
from vision import RealSenseCamera
from beingbeyond_d1_sdk.head_arm import HeadArmRobot
from beingbeyond_d1_sdk.urdf_path import get_default_urdf_path

_ap = argparse.ArgumentParser(description="验证手眼标定：点图看解算世界坐标")
_ap.add_argument("--calib", default="handeye_calib.npz",
                 help="要加载的标定 npz（相对本目录或绝对路径）；默认 handeye_calib.npz，"
                      "可指定某份备份回溯验证")
_args = _ap.parse_args()

CALIB = _args.calib if os.path.isabs(_args.calib) else os.path.join(_HERE, _args.calib)
if not os.path.exists(CALIB):
    sys.exit(f"标定文件不存在: {CALIB}")
print(f"[Calib] {CALIB}")
data = np.load(CALIB)
H = data["H"]
head_yaw = float(data["head_yaw"])
head_pitch = float(data["head_pitch"])
print(f"Loaded calib:\n  H=\n{H}")
print(f"  Head: yaw={math.degrees(head_yaw):.0f}°  pitch={math.degrees(head_pitch):.0f}°\n")

# Set head to calibration position
print("[Init] Robot + set head to calib position ...")
robot = HeadArmRobot(urdf_path=get_default_urdf_path(), dev="/dev/ttyUSB0", baudrate=1_000_000)
q_init = np.radians([0, 0, 0, -60, 60, 0, 0, 0])
robot.set_positions(q_init)
robot.wait_until_reached(q_init, active_joint_indices=range(8))
time.sleep(0.3)
q = np.asarray(robot.get_positions(), dtype=float)
q[0] = head_yaw
q[1] = head_pitch
robot.set_positions(q)
robot.wait_until_reached(q, active_joint_indices=[0, 1])
print(f"  Head set: yaw={math.degrees(q[0]):.0f}°  pitch={math.degrees(q[1]):.0f}°\n")

click = {"uv": None}
def _on_mouse(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        param["uv"] = (x, y)

cam = RealSenseCamera(width=1280, height=720, hz=30)
cv2.namedWindow("Test Calib", cv2.WINDOW_NORMAL)
cv2.setMouseCallback("Test Calib", _on_mouse, click)

try:
    while True:
        rgb, _ = cam.get_aligned_frames(filtered=False)
        vis = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

        if click["uv"] is not None:
            u, v = click["uv"]
            click["uv"] = None
            p = np.array([u, v, 1.0])
            w = H @ p; w /= w[2]
            print(f"pixel=({u},{v}) → world=({w[0]:.3f}, {w[1]:.3f})")

        cv2.imshow("Test Calib", vis)
        if cv2.waitKey(5) & 0xFF == 27:
            break
finally:
    cam.stop()
    robot.close()
    cv2.destroyAllWindows()
