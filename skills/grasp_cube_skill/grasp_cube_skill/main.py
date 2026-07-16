# SPDX-License-Identifier: MulanPSL-2.0
"""grasp_cube — detect cubes, pick one at a location, place it at another.

Three LLM-callable MCP tools for the D1 dexterous hand:

  * detect_cubes(class_filter) — YOLO on a head-camera frame; report each cube's
                                 base-frame position (feed x,y to pick_cube).
  * pick_cube(position)        — move to the location, grasp a cube, lift and hold.
  * place_cube(position)       — move to the location, release the held cube, lift.

pick_cube / place_cube take a coordinate string ("x,y" / "x,y,z", base frame,
metres) or a named spot; the caller (LLM/user) can get real coordinates from
detect_cubes first. The skill is a pure robonix consumer — it discovers the
d1_arm / d1_hand / d1_camera primitives via atlas and drives them over gRPC (see
connect_primitives); it never opens the serial link / CAN bus / RealSense
itself. IK/FK and YOLO stay local (pure compute, reused from the Beingbeyond_D1
block_grasp stack).

Lifecycle (skill-kind): rbnx boot stops at INACTIVE; the executor fires
CMD_ACTIVATE on the first MCP call. on_activate connects the primitives and
homes the arm (heavy); on_deactivate releases them. The shape
(declare contract → on_init → on_activate/on_deactivate → @skill.mcp) is what
the framework cares about.
"""
from __future__ import annotations

import json
import logging
import os
import threading

from robonix_api import Skill, Ok, Err, Deferred

grasp_cube = Skill(id="grasp_cube", namespace="robonix/skill/grasp_cube")

# Codegen output (rbnx codegen --mcp): typed dataclasses derived from
# capabilities/lib/grasp_cube/srv/{PickCube,PlaceCube,DetectCubes}.srv.
from grasp_cube_mcp import (  # noqa: E402
    PickCube_Request, PickCube_Response,
    PlaceCube_Request, PlaceCube_Response,
    DetectCubes_Request, DetectCubes_Response,
)

# Package root (the grasp_cube_skill dir holding models/, capabilities/, ...),
# used to resolve the default model / calibration paths.
_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _resolve_asset(path: str, default_rel: str) -> str:
    """Resolve a config-supplied asset path: empty → the package default;
    relative → anchored on the package root; absolute → used as-is."""
    p = (path or "").strip() or default_rel
    return p if os.path.isabs(p) else os.path.join(_PKG_ROOT, p)

log = logging.getLogger("grasp_cube")
logging.basicConfig(level=logging.INFO, format="[grasp_cube] %(levelname)s %(message)s")


# ── module state populated by lifecycle handlers ─────────────────────
_state: dict = {
    "pick_z":     0.095,   # default grasp height when a location omits Z
    "urdf_path":  "",
    "model_path": "",      # YOLO .pt (resolved in on_init; see models/README.md)
    "calib_path": "",      # hand-eye handeye_calib.npz (resolved in on_init)
    "controller": None,
    "detector":   None,    # CubeDetector, built in on_activate (camera + YOLO)
    "lock":       threading.Lock(),
}


# ── Location parsing ─────────────────────────────────────────────────
def _resolve_position(position: str):
    """Turn a location string into base-frame (x, y, z) metres.

    Accepts "x,y" (Z = configured pick_z), "x,y,z", or a named spot looked up in
    block_grasp's NAMED_POSITIONS / PLACE_POSITIONS (Z = pick_z). ASCII and
    full-width commas both work. Raises ValueError on empty / unknown / malformed.
    """
    text = (position or "").strip().replace("，", ",")
    if not text:
        raise ValueError("empty position")

    if "," in text:
        parts = [p for p in text.split(",") if p.strip() != ""]
        if len(parts) not in (2, 3):
            raise ValueError(f"expected 'x,y' or 'x,y,z', got {position!r}")
        try:
            nums = [float(p) for p in parts]
        except ValueError:
            raise ValueError(f"non-numeric coordinate {position!r}")
        x, y = nums[0], nums[1]
        z = nums[2] if len(nums) == 3 else _state["pick_z"]
        return x, y, z

    # Named spot (e.g. "中间", "red_cube") — reuse the D1 place map.
    from block_grasp.config import NAMED_POSITIONS, PLACE_POSITIONS
    spot = NAMED_POSITIONS.get(text) or PLACE_POSITIONS.get(text)
    if spot is None:
        raise ValueError(f"unknown named position {position!r}")
    return float(spot[0]), float(spot[1]), _state["pick_z"]


def _parse_opt_float(text: str, name: str):
    """Parse an optional numeric MCP field: empty/blank → None (use the
    controller default), otherwise the float. Raises ValueError with a clear
    message on a non-numeric value."""
    s = (text or "").strip()
    if not s:
        return None
    try:
        return float(s)
    except ValueError:
        raise ValueError(f"{name} must be a number, got {text!r}")


def _controller_or_none():
    return _state["controller"]


# ── @grasp_cube.mcp: the LLM-callable tools ──────────────────────────
@grasp_cube.mcp("robonix/skill/grasp_cube/pick_cube")
def pick_cube(req: PickCube_Request) -> PickCube_Response:
    """抓取物体：把机械臂移动到指定位置，抓起那里的物体并夹住。The LLM should call
    this when the user wants an object picked up from a place. `position` is a
    coordinate string "x,y" / "x,y,z" (base frame, metres) or a named spot
    (e.g. "中间"). Optional `angle_deg` sets the grasp yaw angle in degrees (hand
    rotation about vertical; empty = default orientation) and `gripper` sets how
    tightly to close, 0.0 (open) – 0.5 (standard grasp) – 1.0 (tightest, for
    smaller objects) (empty = standard grasp). The default grasp is tuned for the
    standard cube (5 cm side length), so leave `gripper` empty for those; use a
    larger value for objects smaller than 5 cm. Returns ok=true if an object was
    actually grasped."""
    ctrl = _controller_or_none()
    if ctrl is None:
        return PickCube_Response(ok=False, message="skill not activated (no arm/hand connection)")
    try:
        x, y, z = _resolve_position(req.position)
        angle_deg = _parse_opt_float(req.angle_deg, "angle_deg")
        gripper = _parse_opt_float(req.gripper, "gripper")
    except ValueError as exc:
        return PickCube_Response(ok=False, message=f"bad argument: {exc}")
    try:
        with _state["lock"]:
            res = ctrl.pick(x, y, z, angle_deg=angle_deg, gripper=gripper)
        return PickCube_Response(ok=bool(res["ok"]), message=str(res["message"]))
    except Exception as exc:  # noqa: BLE001
        log.exception("pick_cube failed")
        return PickCube_Response(ok=False, message=f"error: {exc}")


@grasp_cube.mcp("robonix/skill/grasp_cube/place_cube")
def place_cube(req: PlaceCube_Request) -> PlaceCube_Response:
    """放置物体：把机械臂移动到指定位置，松开夹住的物体放下。The LLM should call
    this after pick_cube to put the held object down. `position` is a coordinate
    string "x,y" / "x,y,z" (base frame, metres) or a named spot (e.g. "中间").
    Optional `angle_deg` sets the approach yaw angle in degrees (hand rotation
    about vertical; empty = default orientation) and `gripper` sets the release
    aperture on the same 0.0 (fully open) – 0.5 (grasp) – 1.0 (tightest) scale
    (empty = fully open)."""
    ctrl = _controller_or_none()
    if ctrl is None:
        return PlaceCube_Response(ok=False, message="skill not activated (no arm/hand connection)")
    try:
        x, y, z = _resolve_position(req.position)
        angle_deg = _parse_opt_float(req.angle_deg, "angle_deg")
        gripper = _parse_opt_float(req.gripper, "gripper")
    except ValueError as exc:
        return PlaceCube_Response(ok=False, message=f"bad argument: {exc}")
    try:
        with _state["lock"]:
            res = ctrl.place(x, y, z, angle_deg=angle_deg, gripper=gripper)
        return PlaceCube_Response(ok=bool(res["ok"]), message=str(res["message"]))
    except Exception as exc:  # noqa: BLE001
        log.exception("place_cube failed")
        return PlaceCube_Response(ok=False, message=f"error: {exc}")


@grasp_cube.mcp("robonix/skill/grasp_cube/detect_cubes")
def detect_cubes(req: DetectCubes_Request) -> DetectCubes_Response:
    """用头部相机拍一张图，跑 YOLO 识别桌面上的积木，返回每个积木的位置。The LLM
    should call this to see what cubes are on the table and where they are, then
    feed a cube's x,y to pick_cube. Optional `class_filter` (e.g. "red_cube")
    returns only that colour. `cubes` is a JSON array of
    {class_name, score, x, y, z, grasp_angle_deg} in the base frame (metres),
    sorted by score; x,y is the grasp point. ok=true even if no cube is found."""
    det = _state["detector"]
    if det is None:
        return DetectCubes_Response(ok=False, message="skill not activated (no camera connection)", cubes="[]")
    try:
        with _state["lock"]:
            cubes = det.detect()
    except Exception as exc:  # noqa: BLE001
        log.exception("detect_cubes failed")
        return DetectCubes_Response(ok=False, message=f"error: {exc}", cubes="[]")

    cls = (req.class_filter or "").strip()
    if cls:
        cubes = [c for c in cubes if c["class_name"] == cls]
    msg = (f"detected {len(cubes)} cube(s)" + (f" of class {cls}" if cls else "")
           if cubes else ("no cubes detected" + (f" for class {cls}" if cls else "")))
    return DetectCubes_Response(ok=True, message=msg, cubes=json.dumps(cubes, ensure_ascii=False))


# ── Lifecycle ────────────────────────────────────────────────────────
@grasp_cube.on_init
def init(cfg: dict):
    """REGISTERED → INACTIVE. Light: parse config only. Don't open the arm/hand
    yet — the user may have spawned us just to inspect the cap tree."""
    pick_z = cfg.get("pick_z", _state["pick_z"])
    try:
        _state["pick_z"] = float(pick_z)
    except (TypeError, ValueError):
        return Err(f"pick_z must be a number, got {pick_z!r}")
    _state["urdf_path"] = str(cfg.get("urdf_path", "") or "")
    # Detection assets: config path → env override → package default. Resolved
    # to absolute against the package root (see models/README.md).
    _state["model_path"] = _resolve_asset(
        str(cfg.get("model_path", "") or os.environ.get("BLOCK_GRASP_MODEL", "")),
        "models/best.pt",
    )
    _state["calib_path"] = _resolve_asset(
        str(cfg.get("calib_path", "") or os.environ.get("GRASP_CUBE_CALIB", "")),
        "models/handeye_calib.npz",
    )
    log.info("init ok: pick_z=%.3f urdf_path=%r model=%r calib=%r",
             _state["pick_z"], _state["urdf_path"], _state["model_path"], _state["calib_path"])
    return Ok()


@grasp_cube.on_activate
def activate():
    """INACTIVE → ACTIVE. Heavy: discover + connect the d1_arm / d1_hand
    primitives over gRPC and build the controller (which homes the arm).
    Returns Deferred(...) when a primitive isn't online yet — rbnx boot / the
    executor surface that and retry."""
    from grasp_cube_skill.controller import GraspCubeController
    from grasp_cube_skill.detector import CubeDetector
    from grasp_cube_skill.primitive_clients import connect_primitives

    try:
        log.info("connecting arm/hand/camera primitives via atlas ...")
        arm, hand, camera = connect_primitives(grasp_cube)
    except RuntimeError as exc:
        # Most likely the primitives are not up yet — let the framework retry.
        return Deferred(str(exc))

    log.info("initialising controller (homing arm) ...")
    _state["controller"] = GraspCubeController(
        robot=arm, hand=hand, urdf_path=_state["urdf_path"],
    )
    log.info("initialising cube detector (loading YOLO) ...")
    _state["detector"] = CubeDetector(
        arm=arm, camera=camera,
        model_path=_state["model_path"], calib_path=_state["calib_path"],
        urdf_path=_state["urdf_path"],
    )
    log.info("activated — ready to detect / pick / place")
    return Ok()


@grasp_cube.on_deactivate
def deactivate():
    """ACTIVE → INACTIVE. Release the held cube and drop the controller; the
    primitive channels are auto-closed by the Capability framework. Idempotent."""
    ctrl = _state["controller"]
    if ctrl is not None:
        try:
            ctrl.shutdown()
        except Exception:  # noqa: BLE001
            log.exception("controller shutdown failed")
    _state["controller"] = None
    _state["detector"] = None  # camera channel is auto-closed by the framework
    log.info("deactivated")
    return Ok()


@grasp_cube.on_shutdown
def shutdown():
    """any → TERMINATED. Last-chance cleanup."""
    ctrl = _state["controller"]
    if ctrl is not None:
        try:
            ctrl.shutdown()
        except Exception:  # noqa: BLE001
            pass
    _state["detector"] = None
    log.info("shutdown")


def main() -> int:
    grasp_cube.run()
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
