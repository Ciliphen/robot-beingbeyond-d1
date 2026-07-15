#!/usr/bin/env bash
# Build phase for the grasp_cube skill: run rbnx codegen so the lifecycle
# gRPC stubs AND the typed MCP Request/Response classes are generated.
#   --mcp  → @skill.mcp tools need typed Request/Response classes; codegen
#            also picks up this package's own capabilities/ tree (driver +
#            pick_cube/place_cube toml + lib/grasp_cube/srv/) and emits the
#            `grasp_cube_mcp` module imported by main.py.
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"

# rbnx codegen invokes `python3` from PATH and needs grpcio-tools there. Put
# the Python 3.10 env (which has the full D1 stack + grpcio-tools) first so
# codegen uses it rather than base python.
ENV_PY="${GRASP_CUBE_PYTHON:-$HOME/miniconda3/envs/bb_d1_robonix/bin/python3}"
[ -x "$ENV_PY" ] && export PATH="$(dirname "$ENV_PY"):$PATH"

FLAGS=(--mcp)
[[ "${RBNX_BUILD_CLEAN:-}" == "1" ]] && FLAGS+=(--clean)
rbnx codegen -p "$PKG" "${FLAGS[@]}"
echo "[grasp_cube_skill] build done"
