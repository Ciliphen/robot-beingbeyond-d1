# SPDX-License-Identifier: MulanPSL-2.0
"""grasp_cube — pick a cube at a location, place it at another.

Two LLM-callable MCP tools for the D1 dexterous hand:

  * pick_cube(position)  — move to the location, grasp a cube, lift and hold.
  * place_cube(position) — move to the location, release the held cube, lift.

No vision: the caller (LLM/user) supplies where to pick and where to place, as
a coordinate string ("x,y" / "x,y,z", base frame, metres) or a named spot. The
skill is a pure robonix consumer — it discovers the d1_arm / d1_hand primitives
via atlas and drives them over gRPC (see connect_primitives); it never opens the
serial link / CAN bus itself, and uses no camera. IK/FK stay local (pure
compute, reused from the Beingbeyond_D1 block_grasp stack).

Lifecycle (skill-kind): rbnx boot stops at INACTIVE; the executor fires
CMD_ACTIVATE on the first MCP call. on_activate connects the primitives and
homes the arm (heavy); on_deactivate releases them. The shape
(declare contract → on_init → on_activate/on_deactivate → @skill.mcp) is what
the framework cares about.
"""
from __future__ import annotations

import logging
import threading

from robonix_api import Skill, Ok, Err, Deferred

grasp_cube = Skill(id="grasp_cube", namespace="robonix/skill/grasp_cube")

# Codegen output (rbnx codegen --mcp): typed dataclasses derived from
# capabilities/lib/grasp_cube/srv/{PickCube,PlaceCube}.srv.
from grasp_cube_mcp import (  # noqa: E402
    PickCube_Request, PickCube_Response,
    PlaceCube_Request, PlaceCube_Response,
)

log = logging.getLogger("grasp_cube")
logging.basicConfig(level=logging.INFO, format="[grasp_cube] %(levelname)s %(message)s")


# ── module state populated by lifecycle handlers ─────────────────────
_state: dict = {
    "pick_z":     0.095,   # default grasp height when a location omits Z
    "urdf_path":  "",
    "controller": None,
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


def _controller_or_none():
    return _state["controller"]


# ── @grasp_cube.mcp: the LLM-callable tools ──────────────────────────
@grasp_cube.mcp("robonix/skill/grasp_cube/pick_cube")
def pick_cube(req: PickCube_Request) -> PickCube_Response:
    """把机械臂移动到指定位置，抓起那里的积木并夹住。The LLM should call this
    when the user wants a cube picked up from a place. `position` is a
    coordinate string "x,y" / "x,y,z" (base frame, metres) or a named spot
    (e.g. "中间"). Returns ok=true if a cube was actually grasped."""
    ctrl = _controller_or_none()
    if ctrl is None:
        return PickCube_Response(ok=False, message="skill not activated (no arm/hand connection)")
    try:
        x, y, z = _resolve_position(req.position)
    except ValueError as exc:
        return PickCube_Response(ok=False, message=f"bad position: {exc}")
    try:
        with _state["lock"]:
            res = ctrl.pick(x, y, z)
        return PickCube_Response(ok=bool(res["ok"]), message=str(res["message"]))
    except Exception as exc:  # noqa: BLE001
        log.exception("pick_cube failed")
        return PickCube_Response(ok=False, message=f"error: {exc}")


@grasp_cube.mcp("robonix/skill/grasp_cube/place_cube")
def place_cube(req: PlaceCube_Request) -> PlaceCube_Response:
    """把机械臂移动到指定位置，松开夹住的积木放下。The LLM should call this
    after pick_cube to put the held cube down. `position` is a coordinate string
    "x,y" / "x,y,z" (base frame, metres) or a named spot (e.g. "中间")."""
    ctrl = _controller_or_none()
    if ctrl is None:
        return PlaceCube_Response(ok=False, message="skill not activated (no arm/hand connection)")
    try:
        x, y, z = _resolve_position(req.position)
    except ValueError as exc:
        return PlaceCube_Response(ok=False, message=f"bad position: {exc}")
    try:
        with _state["lock"]:
            res = ctrl.place(x, y, z)
        return PlaceCube_Response(ok=bool(res["ok"]), message=str(res["message"]))
    except Exception as exc:  # noqa: BLE001
        log.exception("place_cube failed")
        return PlaceCube_Response(ok=False, message=f"error: {exc}")


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
    log.info("init ok: pick_z=%.3f urdf_path=%r", _state["pick_z"], _state["urdf_path"])
    return Ok()


@grasp_cube.on_activate
def activate():
    """INACTIVE → ACTIVE. Heavy: discover + connect the d1_arm / d1_hand
    primitives over gRPC and build the controller (which homes the arm).
    Returns Deferred(...) when a primitive isn't online yet — rbnx boot / the
    executor surface that and retry."""
    from grasp_cube_skill.controller import GraspCubeController
    from grasp_cube_skill.primitive_clients import connect_primitives

    try:
        log.info("connecting arm/hand primitives via atlas ...")
        arm, hand = connect_primitives(grasp_cube)
    except RuntimeError as exc:
        # Most likely the primitives are not up yet — let the framework retry.
        return Deferred(str(exc))

    log.info("initialising controller (homing arm) ...")
    _state["controller"] = GraspCubeController(
        robot=arm, hand=hand, urdf_path=_state["urdf_path"],
    )
    log.info("activated — ready to pick / place")
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
    log.info("shutdown")


def main() -> int:
    grasp_cube.run()
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
