# Runtime config accepted by the D1 arm primitive.
#
# This file documents the mapping passed as this package's `config:` value in
# robonix_manifest.yaml. It is documentation for deployers and tooling; the
# provider continues to parse and validate the values in its own code
# (each key also falls back to the matching D1_ARM_* env var).

config:
  # string, default: /dev/ttyUSB0 (env: D1_ARM_DEV).
  # Serial device of the HeadArm chain (arm joints 1-6 + head yaw/pitch).
  # The head and arm share this one port — do not open it from another process
  # (e.g. the block_grasp skill) at the same time or they contend for the link.
  dev: /dev/ttyUSB0

  # int, default: 1000000 (env: D1_ARM_BAUD).
  # Serial baudrate for the HeadArm link. Must match the controller firmware.
  baudrate: 1000000

  # string, default: "" (env: D1_ARM_URDF).
  # Path to the robot URDF used to build the kinematics model for move_pose IK.
  # Empty falls back to the SDK's packaged default model.
  urdf_path: ""

  # float, metres, default: 0.05 (env: D1_ARM_IK_POS_TOL).
  # Position tolerance for the move_pose inverse-kinematics solve. A solution
  # whose end-effector error exceeds this is rejected. Smaller = stricter.
  ik_pos_tol: 0.05
