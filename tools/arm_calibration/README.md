# Arm calibration

*[中文版](./README_CN.md)*

Calibrates the two sets of parameters the `vertical_grasp_object` skill's grasping
depends on:

1. **Hand-eye calibration** (`calibrate_handeye.py`) — the 2D homography between
   camera pixels and table base-frame coordinates. Produces `handeye_calib.npz`,
   the source of this deployment's `models/handeye_calib.npz`, which the skill's
   `detector.py` loads to resolve detected pixels into base-frame XY.
2. **Gravity sag calibration** (`calibrate_sag.py`) — the arm's own weight droops
   the end effector when extended. Fits `dz = factor·dist³` to get
   `GRAVITY_SAG_FACTOR`, which goes into `block_grasp.config` so the target point
   is raised to compensate.

Lifted from the Beingbeyond_D1 reference repo's `block_grasp/`, with imports and
paths rewritten to be self-contained (relative to this directory).

## Contents

| File | Purpose |
| --- | --- |
| `calibrate_handeye.py` | **hand-eye**: camera preview + click a reference point + move the EE there by keyboard/drag + record the pair → solve the homography |
| `calibrate_sag.py` | **gravity sag**: step the EE through several radial distances, descend to grasp height, measure → fit the sag factor |
| `test_click_goto.py` | **verification (primary)**: after a click the EE **actually moves** to that spot, so physical alignment shows whether the calibration is good |
| `test_calib.py` | **verification (lightweight)**: click any pixel and only print the resolved table world coordinate; the arm does not move |
| `ik_scipy.py` | the SLSQP numerical IK solver (used when the calibration scripts move the EE; same copy the skill runs) |
| `config.py` | calibration/grasp constants (head pose, EE pose, IK tolerances…; the counterpart of the skill's `block_grasp.config`) |
| `../vision.py` | the RealSense wrapper (needs `pyrealsense2`). Kept in `tools/` so every tool shares one copy |
| `handeye_calib.npz` | a sample calibration artefact — deployable as-is, or useful as a format reference |

> `handeye_calib.npz` contains: `H` (the 3×3 homography), `plane` (the table plane
> z=ax+by+c), `head_yaw` / `head_pitch` (the head angles at calibration time, which
> **must be reproduced** at detection time), `pixel_pts` / `world_pts` (the raw
> pairs), and `mean_err_mm` (the calibration error).

## Environment

⚠ **These are on-robot scripts**: they open the `/dev/ttyUSB0` serial link, the
RealSense, the CAN bus (the dexterous hand, in `calibrate_handeye.py`) and cv2
windows directly, so they must run on the machine with the D1 arm and RealSense
attached:

```bash
conda activate bb_d1
pip install pyrealsense2 opencv-python numpy scipy
# beingbeyond_d1_sdk: the D1 head/arm/hand SDK. The wheel ships in this repo under
#                     tools/func_verify/lib/ — install it into the run env.
```

> `ik_scipy.py` / `config.py` are siblings in this directory; `vision.py` sits in the
> parent `tools/` dir (shared by every tool). The scripts already add both to
> `sys.path`, so no `PYTHONPATH` is needed.

## Full pipeline

### 1. Hand-eye calibration (do this first)

```bash
conda activate bb_d1
python tools/arm_calibration/calibrate_handeye.py
```

The flow (the script prints its own prompts):

1. Point the camera at the table and hold the head at the calibration pose
   (default yaw=-10°, pitch=35° — **do not move the head** during calibration).
2. **Click one table reference point** in the image (a green cross).
3. Move the EE to that physical point, either way:
   - **keyboard / IK**: `WASD` = XY, `ZX` = Z, `UO/IK/JL` = RPY
   - **drag teaching**: `T` releases the arm's torque so you can move it by hand
     (support the arm before releasing the torque!)
4. `Space` records one (pixel, world) pair.
5. Repeat at **six or more** different spots on the table.
6. `C` solves the homography and saves `handeye_calib.npz` (only if the XY error is
   under 20 mm).

Other keys: `T` toggle torque (drag mode), `B` open/close the hand, `R` reset the EE,
`ESC` quit.

### 2. Verify the hand-eye calibration

`test_click_goto.py` is the recommended check: after a click the EE **actually
moves** there, so you can see directly whether it lines up with the physical point
you clicked. That is the most honest verification.

```bash
python tools/arm_calibration/test_click_goto.py
python tools/arm_calibration/test_click_goto.py --calib handeye_calib.20260717_095620.npz  # replay a backup
```

⚠ **Keep the e-stop within reach** — the arm moves above the clicked spot and
descends.

It loads `handeye_calib.npz` (`--calib` selects a specific backup to replay) and puts
the head at the calibration pose. Left-click a point on the table and the EE moves to
`z_offset` (10 cm by default) above it; check whether the fingertip lines up in XY.
Keys: `Z/X` raise/lower the target height, `Space`/`B` close/open the hand, `ESC`
quit. Re-calibrate if the EE is visibly off.

There is also the lightweight `test_calib.py`, which leaves the arm still and only
prints the resolved world coordinate (`pixel=(u,v) → world=(x,y)`) for you to check
with a ruler — handy for eyeballing the numbers:

```bash
python tools/arm_calibration/test_calib.py
python tools/arm_calibration/test_calib.py --calib handeye_calib.20260717_095620.npz
```

### 3. Gravity sag calibration

```bash
python tools/arm_calibration/calibrate_sag.py
python tools/arm_calibration/calibrate_sag.py --reverse   # far to near, to compare hysteresis against the forward pass (reveals mechanical backlash)
```

⚠ **Keep the e-stop within reach** — the arm descends to grasp height, close to the
table.

The script steps the EE through several radial distances (`CALIB_X`) in the grasp
pose. At each one it descends to the nominal grasp height **with sag compensation
off** and holds; you measure the vertical distance (mm) from **the same EE reference
point** to the table and type it in. Once every point is measured it fits the data
and prints the suggested `GRAVITY_SAG_FACTOR` (with R²).

### 4. Deploy

- **Hand-eye**: copy the npz into this deployment's `models/` dir:

  ```bash
  cp tools/arm_calibration/handeye_calib.npz models/handeye_calib.npz
  ```

  The path can also be overridden by the `calib_path` config key or the
  `VERTICAL_GRASP_OBJECT_CALIB` env var (see the skill's `main.py`).
- **Gravity sag**: put the fitted value into `GRAVITY_SAG_FACTOR` in the skill's
  `block_grasp.config` (`controller.py` raises the target by
  `GRAVITY_SAG_FACTOR * dist³`).

## Notes

- **The head pose must be identical** across hand-eye calibration, verification
  (`test_click_goto` / `test_calib`), and live detection — otherwise the homography
  does not hold. Defaults are yaw=-10°, pitch=35° (`HEAD_YAW_DEG` /
  `HEAD_PITCH_DEG` in `config.py`).
- **Camera resolution**: calibration uses 1280×720 (`CALIB_CAM_WIDTH/HEIGHT` in
  `config.py`). When the detection side runs at a different resolution, pixel
  coordinates are rescaled automatically.
- **The sag model is cubic in radial distance**, so `calibrate_sag.py` can fix y=0 and
  step along x alone: sag depends only on radial distance. The fit uses the slope, so
  which reference point you measure to (fingertip or flange) does not change the
  result.
