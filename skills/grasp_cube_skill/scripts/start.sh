#!/usr/bin/env bash
# Start the grasp_cube skill node.
#
# Runs on a dedicated Python 3.10 env that has BOTH the D1 SDK/kinematics stack
# (beingbeyond_d1_sdk cp310 wheel, scipy) AND the robonix skill deps
# (robonix_api, mcp, fastmcp, grpcio). The grasp motion/IK/config live in the
# Beingbeyond_D1 repo, imported via BEINGBEYOND_PATH.
set -eo pipefail
PKG_ROOT="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG_ROOT"

# Beingbeyond_D1 repo root (block_grasp kinematics/ik/config live here).
export BEINGBEYOND_PATH="${BEINGBEYOND_PATH:-$HOME/Beingbeyond_D1}"

# Python 3.10 env with the full stack (override GRASP_CUBE_PYTHON if needed).
PYTHON="${GRASP_CUBE_PYTHON:-$HOME/miniconda3/envs/bb_d1_robonix/bin/python3}"

# robonix_api is served from the robonix source tree, not pip-installed.
export PYTHONPATH="$(rbnx path robonix-api):$PKG_ROOT:$BEINGBEYOND_PATH:${PYTHONPATH:-}"

exec "$PYTHON" -m grasp_cube_skill.main
