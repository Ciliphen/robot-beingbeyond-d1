#!/usr/bin/env bash
# Start the hand_gesture skill node.
#
# Runs on the same Python 3.10 env as the other D1 packages (beingbeyond_d1_sdk
# is not imported here — the skill is a pure consumer of the d1_hand primitive
# over gRPC — but the env carries the robonix skill deps: robonix_api, mcp,
# fastmcp, grpcio). Nothing is vendored into this package; the dance is a plain
# open/close loop over hand/move_joint.
set -eo pipefail
PKG_ROOT="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG_ROOT"

# Python 3.10 env with the robonix skill stack (override HAND_GESTURE_PYTHON if needed).
PYTHON="${HAND_GESTURE_PYTHON:-$HOME/miniconda3/envs/bb_d1_robonix/bin/python3}"

# robonix_api is served from the robonix source tree, not pip-installed. PKG_ROOT
# is on the path so the skill package resolves.
export PYTHONPATH="$(rbnx path robonix-api):$PKG_ROOT:${PYTHONPATH:-}"

exec "$PYTHON" -m hand_gesture_skill.main
