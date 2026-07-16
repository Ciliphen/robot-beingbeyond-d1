# Runtime config accepted by the grasp_cube skill.
#
# This file documents the mapping passed as this package's `config:` value in
# robonix_manifest.yaml. An empty `config: {}` uses the defaults below.

config:
  # float (metres), default: 0.095.
  # Default grasp height (base-frame Z) used when a pick/place location is given
  # as "x,y" without an explicit Z. pick_cube / place_cube take coordinates
  # directly; when a Z is omitted this fixed, hand-measured table height is used.
  pick_z: 0.095

  # string, default: "" (SDK default URDF).
  # URDF used for the local IK/FK. Empty picks the beingbeyond_d1_sdk default.
  urdf_path: ""

  # string, default: "" → ./models/best.pt (env: BLOCK_GRASP_MODEL).
  # YOLO-OBB weights for detect_cubes. Relative paths resolve against the skill
  # package root; the file is git-ignored — see models/README.md.
  model_path: ""

  # string, default: "" → ./models/handeye_calib.npz (env: GRASP_CUBE_CALIB).
  # Hand-eye calibration (camera->base homography, head pose, table Z) for
  # detect_cubes. Robot-specific; git-ignored — see models/README.md.
  calib_path: ""
