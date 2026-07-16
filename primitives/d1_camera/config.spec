# Runtime config accepted by the D1 head-camera primitive.
#
# This file documents the mapping passed as this package's `config:` value in
# robonix_manifest.yaml. It is documentation for deployers and tooling; the
# provider continues to parse and validate the values in its own code
# (each key also falls back to the matching D1_CAMERA_* env var).

config:
  # int, pixels, default: 1280 (env: D1_CAMERA_WIDTH).
  # RGB stream width. Keep at the hand-eye calibration resolution (1280x720)
  # so the vertical_grasp_object detector's pixel->world homography stays valid.
  width: 1280

  # int, pixels, default: 720 (env: D1_CAMERA_HEIGHT).
  # RGB stream height. See the note on width — must match the calibration.
  height: 720

  # int, Hz, default: 30 (env: D1_CAMERA_FPS).
  # RealSense frame rate. The snapshot contract only pulls single frames on
  # demand, so this mainly sets the underlying stream rate.
  fps: 30

  # string, default: camera (env: D1_CAMERA_FRAME_ID).
  # frame_id stamped into the emitted sensor_msgs/Image header.
  frame_id: camera
