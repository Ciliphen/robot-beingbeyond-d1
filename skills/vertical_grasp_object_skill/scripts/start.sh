#!/usr/bin/env bash
# Start the vertical_grasp_object skill node.
#
# Runs on a dedicated Python 3.10 env that has the D1 kinematics wheel
# (beingbeyond_d1_sdk cp310, scipy) plus the robonix skill deps (robonix_api,
# mcp, fastmcp, grpcio) and cv2 / ultralytics for detection. The YOLO detection
# (object_detect/) and the grasp geometry/IK/config (block_grasp/) are VENDORED
# into this package root, so no BEINGBEYOND_PATH is needed. Only beingbeyond_d1_sdk
# (D1Kinematics FK/IK) remains a normal conda/pip dependency in the env.
set -eo pipefail
PKG_ROOT="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG_ROOT"

# Python 3.10 env with the full stack (override VERTICAL_GRASP_OBJECT_PYTHON if needed).
PYTHON="${VERTICAL_GRASP_OBJECT_PYTHON:-$HOME/miniconda3/envs/bb_d1_robonix/bin/python3}"

# robonix_api is served from the robonix source tree, not pip-installed. PKG_ROOT
# is on the path so the vendored object_detect/ and block_grasp/ resolve.
export PYTHONPATH="$(rbnx path robonix-api):$PKG_ROOT:${PYTHONPATH:-}"

exec "$PYTHON" -m vertical_grasp_object_skill.main
