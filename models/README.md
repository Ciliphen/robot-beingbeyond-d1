# Detection assets (robot-specific, not committed)

*[中文版](./README_CN.md)*

The `vertical_grasp_object` skill's detection stage needs two files. They are tied
to **this** robot's camera mount and table height, and they are large, so they are
neither committed here nor kept in the skill package — the skill is fetched by the
manifest's `url:` into `rbnx-boot/cache/`, where its `models/` dir is always empty.

| File | What it is | Where it comes from |
|---|---|---|
| `best.pt` | YOLO-OBB cube-detection weights (~113 MB) | train with `tools/yolo_train/` (`json2label.py` → `train.py`); the artefact lands in `runs/<run>/weights/best.pt` |
| `handeye_calib.npz` | camera→base hand-eye homography + head pose + table Z | run `tools/arm_calibration/calibrate_handeye.py` on this robot |

Put both files directly in this directory. The manifest points at them by absolute
path as `${D1_DEPLOY_DIR}/models/...` (`D1_DEPLOY_DIR` is set in the manifest's
`env:` block), so `rbnx clean --cache` re-fetching the skill package does not
disturb them.

Only `detect_cubes` (YOLO) needs `best.pt`. `handeye_calib.npz` is the geometric
ground truth **shared by both detectors** — without it `detect_objects` cannot work
either.

Re-calibrate after remounting the camera or changing the table height.
