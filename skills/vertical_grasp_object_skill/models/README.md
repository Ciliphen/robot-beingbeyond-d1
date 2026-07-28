# vertical_grasp_object models

The `detect_cubes` capability needs two local files that are **not** committed
(they are large / deployment-specific — see the repo `.gitignore`):

| file                | what it is                                              | how to get it |
| ------------------- | ------------------------------------------------------- | ------------- |
| `best.pt`           | YOLO-OBB weights that detect the 4 coloured cubes (~113 MB) | train with the deploy repo's `tools/yolo_train/` chain (`json2label.py` → `train.py`), then copy `runs/<run>/weights/best.pt` here |
| `handeye_calib.npz` | camera→base hand-eye homography + head pose + table Z   | run the deploy repo's `tools/arm_calibration/calibrate_handeye.py` on this robot |

Place both files directly in this directory. The skill config in
`robonix_manifest.yaml` points at them via `model_path` / `calib_path`
(defaults: `./models/best.pt`, `./models/handeye_calib.npz`, resolved relative
to the skill package). Override with `BLOCK_GRASP_MODEL` / `VERTICAL_GRASP_OBJECT_CALIB`
env vars if you keep them elsewhere.

The calibration is specific to this robot's camera mount and table; re-run the
calibration if the head camera is remounted or the table height changes.
