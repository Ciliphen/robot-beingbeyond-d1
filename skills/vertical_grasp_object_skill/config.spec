# Runtime config accepted by the vertical_grasp_object skill.
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

  # string, default: "" → ./models/handeye_calib.npz (env: VERTICAL_GRASP_OBJECT_CALIB).
  # Hand-eye calibration (camera->base homography, head pose, table Z) for
  # detect_cubes AND detect_objects (both share the same projection). Robot-
  # specific; git-ignored — see models/README.md.
  calib_path: ""

  # VLM endpoint for detect_objects (open-vocabulary detection). Each falls back
  # to the deployment's env var when left empty; leave all empty to disable
  # detect_objects (pick/place/detect_cubes are unaffected).

  # string, default: "" → env VLM_BASE_URL.
  # OpenAI-compatible base URL; "/chat/completions" is appended if missing.
  vlm_base_url: ""

  # string, default: "" → env VLM_API_KEY. Bearer token for the endpoint.
  vlm_api_key: ""

  # string, default: "" → env VLM_MODEL. Model id; must accept image input.
  vlm_model: ""

  # float (metres), default: 0.025.
  # Grasp height above the table for detect_objects hits. The VLM gives no
  # depth, so this fixed value sets both the returned Z and the perspective
  # correction. Default = a 5 cm cube's centre; raise/lower for taller/flatter
  # objects resting on the table.
  vlm_grasp_height: 0.025
