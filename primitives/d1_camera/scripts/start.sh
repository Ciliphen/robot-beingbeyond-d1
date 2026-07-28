#!/usr/bin/env bash
# Start the D1 head camera primitive.
#
# Runs on a Python 3.10 env holding pyrealsense2 AND robonix_api / mcp / grpcio
# (see README.md).
set -euo pipefail
echo "[d1_camera] starting..."

PKG_ROOT="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG_ROOT"

# Python 3.10 env with the full stack (override BLOCK_GRASP_PYTHON if needed).
PYTHON="${BLOCK_GRASP_PYTHON:-$HOME/miniconda3/envs/bb_d1_robonix/bin/python3}"

# robonix_api ships as a source dir; the Primitive constructor then adds
# rbnx-build/codegen/{proto_gen,robonix_mcp_types} to sys.path itself.
# PKG_ROOT is on the path so `from d1_camera.vision import ...` resolves the
# vision.py that build.sh vendored into d1_camera/.
ROBONIX_API="$(rbnx path robonix-api)"
export PYTHONPATH="${ROBONIX_API}:${PKG_ROOT}:${PYTHONPATH:-}"

exec "$PYTHON" -m d1_camera.main
