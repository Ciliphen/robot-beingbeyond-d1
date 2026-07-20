<p align="center">
  <img src="bb_d1.png" width="400" alt="BeingBeyond D1">
</p>

# BeingBeyond D1 SDK Examples

This repository provides the BeingBeyond D1 SDK, example Python scripts, and basic guidance on environment setup and common issues.

> **Warning**
> Always keep the emergency stop button within easy reach.

---

## 1. Requirements

### 1.1 Hardware
- BeingBeyond D1 robot
  - Head + arm
  - Dexterous hand
  - RealSense camera
- Linux PC (Ubuntu 20.04 / 22.04) or Windows (coming soon)
- USB 3.0 port

### 1.2 Software
- Python 3.10
- pyrealsense2
  - `pip install pyrealsense2`
- SDK wheel file:
  - `lib/beingbeyond_d1_sdk-0.2.0-cp310-cp310-manylinux_2_17_x86_64.manylinux2014_x86_64.whl`
- Windows version to be released later.

---

## 2. Installation

### 2.1 Create a Conda environment

```bash
conda create -n bb_d1_rbnx python=3.10 -y
conda activate bb_d1_rbnx
```

### 2.2 Install the SDK

```bash
pip install -U pip
pip install lib/beingbeyond_d1_sdk-0.2.0-cp310-cp310-manylinux_2_17_x86_64.manylinux2014_x86_64.whl
```

### 2.3 Install other dependencies

```bash
pip install numpy pyrealsense2 opencv-python
```

---

## 3. USB and Permissions

### 3.1 Check the USB serial device
Remove conflicts:

```bash
sudo apt remove brltty
```

Re-plug the device's USB.

```bash
ls /dev/ttyUSB*
```

Expected output: `/dev/ttyUSB0` or `/dev/ttyUSB1`.

### 3.2 If the serial port cannot be opened

If you see an error like:

```bash
RuntimeError: Failed to open port /dev/ttyUSB0
```

Check the serial port permissions:

```bash
ls -l /dev/ttyUSB0
groups
```

If `/dev/ttyUSB0` belongs to the `dialout` group but the current user is not in the `dialout` group, run:

```bash
sudo usermod -aG dialout $USER
```

Then log out and log back in, or run:

```bash
newgrp dialout
```


---

## 4. Robot Hardware Preparation

1. Power on and wait for the dexterous hand to auto-calibrate.
2. Keep the emergency stop button within reach.
3. If abnormal motion occurs:

   Press the emergency stop immediately → stop the script → move the robot to a safe pose → release the emergency stop → retry.

4. If calibration fails:

   Press the emergency stop → release it → try again.

---

## 5. Running the Examples

```bash
cd examples
```

### 5.1 Dexterous hand control
```bash
python 1_control_hand.py
```

### 5.2 Head + arm control
```bash
python 2_control_head_arm.py
```

### 5.3 RealSense vision visualization
```bash
pip install pyrealsense2
python 3_show_vision.py
```

### 5.4 Full D1 motion example
```bash
python 4_control_d1.py
```

### 5.5 End-effector pose IK

```bash
python 5_ik_control.py
```

Features:

- Query the current end-effector pose
- Build a target pose in the base frame
- Run iterative IK solving
- Observe how end-effector displacement maps to joint motion

---

### 5.6 Keyboard teleoperation

```bash
python 6_keyboard_teleop.py
```

> **Important**
> This example must be run in a **real terminal** (Ubuntu Terminal / macOS Terminal).
> Running it inside IDEs such as VSCode, PyCharm, or Jupyter is not recommended, as the raw keyboard input mode will not work.

This script provides real-time Cartesian control of the D1 arm via the keyboard.

##### Keyboard commands

```
Translation:
  w / s : X+ / X-
  a / d : Y+ / Y-
  z / x : Z+ / Z-

Orientation:
  u / o : roll  + / -
  i / k : pitch + / -
  j / l : yaw   + / -

Dexterous hand:
  Space : toggle hand_pos between 0 and 1

Others:
  r     : reset end-effector target and joint state
  h     : print help
  q     : quit
```

This is the recommended **interactive teleoperation entry example**.

---

## 6. Quick Troubleshooting Guide

- **/dev/ttyUSB* does not appear**
  - Re-plug the USB
  - Uninstall brltty
  - Check `lsusb`

- **Permission denied**
  - Add the user to the `dialout` group

- **CAN fails to start**
  ```bash
  sudo ip link set can0 up type can bitrate 1000000
  ```

- **Dexterous hand zero position misaligned**
  - Power-cycle and wait for calibration

---

## 7. License

MIT / Apache-2.0
