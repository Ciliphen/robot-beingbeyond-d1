# vertical_grasp_object models

The `detect_cubes` capability needs two local files that are **not** committed
(they are large / deployment-specific — see the repo `.gitignore`):

| file                | what it is                                              | how to get it |
| ------------------- | ------------------------------------------------------- | ------------- |
| `best.pt`           | YOLO-OBB weights that detect the 4 coloured cubes (~113 MB) | train with `object_detect/train.py`, or copy an existing `object_detect/runs/<run>/weights/best.pt` |
| `handeye_calib.npz` | camera→base hand-eye homography + head pose + table Z   | run `block_grasp/calibrate_handeye.py` on this robot |

Place both files directly in this directory. The skill config in
`robonix_manifest.yaml` points at them via `model_path` / `calib_path`
(defaults: `./models/best.pt`, `./models/handeye_calib.npz`, resolved relative
to the skill package). Override with `BLOCK_GRASP_MODEL` / `VERTICAL_GRASP_OBJECT_CALIB`
env vars if you keep them elsewhere.

The calibration is specific to this robot's camera mount and table; re-run the
calibration if the head camera is remounted or the table height changes.
