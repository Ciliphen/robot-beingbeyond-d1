# Runtime config accepted by the grasp_cube skill.
#
# This file documents the mapping passed as this package's `config:` value in
# robonix_manifest.yaml. An empty `config: {}` uses the defaults below.

config:
  # float (metres), default: 0.095.
  # Default grasp height (base-frame Z) used when a pick/place location is given
  # as "x,y" without an explicit Z. There is no camera / hand-eye calibration in
  # this skill, so the table height is a fixed, hand-measured constant — measure
  # it once for your table and set it here.
  pick_z: 0.095

  # string, default: "" (SDK default URDF).
  # URDF used for the local IK/FK. Empty picks the beingbeyond_d1_sdk default.
  urdf_path: ""
