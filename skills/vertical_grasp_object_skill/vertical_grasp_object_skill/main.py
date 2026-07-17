# SPDX-License-Identifier: MulanPSL-2.0
"""vertical_grasp_object — detect cubes, pick one at a location, place it at another.

Six LLM-callable MCP tools for the D1 dexterous hand:

  * detect_cubes(class_filter)  — YOLO on a head-camera frame; fixed cube
                                  classes. PREFERRED for cubes/blocks (feed
                                  x,y,z + angle to pick_cube).
  * detect_objects(instruction) — VLM on a head-camera frame; open-vocabulary,
                                  locate a NON-cube object by description. Use
                                  for anything YOLO can't (or if YOLO fails).
  * pick_cube(position)         — move to the location, grasp an object, lift and hold.
  * place_cube(position)        — move to the location, release the held object, lift.
  * verify_grasp(...)           — VLM re-check after a pick/place: did it work?
  * stack_cubes(top, base)      — PREFERRED for stacking: detect both cubes, pick
                                  the top-colour one, place it on the base-colour
                                  one, all in one call.

pick_cube / place_cube take a coordinate string ("x,y" / "x,y,z", base frame,
metres) or a named spot; the caller (LLM/user) can get real coordinates from
detect_cubes first. The skill is a pure robonix consumer — it discovers the
d1_arm / d1_hand / d1_camera primitives via atlas and drives them over gRPC (see
connect_primitives); it never opens the serial link / CAN bus / RealSense
itself. IK/FK and YOLO stay local (pure compute; the object_detect + block_grasp
stacks are vendored into this package, FK/IK from the beingbeyond_d1_sdk wheel).

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

vertical_grasp_object = Skill(id="vertical_grasp_object", namespace="robonix/skill/vertical_grasp_object")

# Codegen output (rbnx codegen --mcp): typed dataclasses derived from
# capabilities/lib/vertical_grasp_object/srv/{PickCube,PlaceCube,DetectCubes}.srv.
from vertical_grasp_object_mcp import (  # noqa: E402
    PickCube_Request, PickCube_Response,
    PlaceCube_Request, PlaceCube_Response,
    DetectCubes_Request, DetectCubes_Response,
    DetectObjects_Request, DetectObjects_Response,
    VerifyGrasp_Request, VerifyGrasp_Response,
    StackCubes_Request, StackCubes_Response,
)

# Package root (the vertical_grasp_object_skill dir holding models/, capabilities/, ...),
# used to resolve the default model / calibration paths.
_PKG_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _resolve_asset(path: str, default_rel: str) -> str:
    """Resolve a config-supplied asset path: empty → the package default;
    relative → anchored on the package root; absolute → used as-is."""
    p = (path or "").strip() or default_rel
    return p if os.path.isabs(p) else os.path.join(_PKG_ROOT, p)

log = logging.getLogger("vertical_grasp_object")
logging.basicConfig(level=logging.INFO, format="[vertical_grasp_object] %(levelname)s %(message)s")


# ── module state populated by lifecycle handlers ─────────────────────
_state: dict = {
    "pick_z":     0.095,   # default grasp height when a location omits Z
    "block_height": 0.05,  # one cube height; stack_cubes releases at base_z + this
    "urdf_path":  "",
    "model_path": "",      # YOLO .pt (resolved in on_init; see models/README.md)
    "calib_path": "",      # hand-eye handeye_calib.npz (resolved in on_init)
    "table_z_offset": 0.0,  # constant correction (m) to the calibrated table plane Z (both detectors). Negative if picks land too high
    "detect_debug": False,  # detect_cubes: save raw + annotated (all candidates) frames to logs/detect/ for debugging
    "vlm_base_url":     "",   # OpenAI-compatible VLM endpoint (env VLM_BASE_URL)
    "vlm_api_key":      "",   # bearer token (env VLM_API_KEY)
    "vlm_model":        "",   # model id, must accept images (env VLM_MODEL)
    "vlm_grasp_height": 0.025,  # grasp Z above table for VLM hits (no depth)
    "verify_match_radius": 0.05,  # verify_grasp: a detection within this (m) counts as "at" the position
    "controller":  None,
    "detector":    None,   # CubeDetector, built in on_activate (camera + YOLO)
    "vlm_detector": None,  # VLMDetector, built in on_activate (camera + VLM)
    "lock":        threading.Lock(),
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


# ── Colour → YOLO class mapping (used by stack_cubes) ────────────────
# The YOLO model is trained on these fixed classes (see tools/yolo_train/data.yaml).
_COLOR_ALIASES = {
    "red_cube": "red_cube", "red": "red_cube", "红": "red_cube",
    "红色": "red_cube", "红色积木": "red_cube", "红方块": "red_cube",
    "green_cube": "green_cube", "green": "green_cube", "绿": "green_cube",
    "绿色": "green_cube", "绿色积木": "green_cube", "绿方块": "green_cube",
    "yellow_cube": "yellow_cube", "yellow": "yellow_cube", "黄": "yellow_cube",
    "黄色": "yellow_cube", "黄色积木": "yellow_cube", "黄方块": "yellow_cube",
}


def _resolve_color(text: str) -> str:
    """Normalise a user-supplied colour into a trained YOLO class name.

    Accepts the class name ("red_cube"), an English colour ("red"), or a Chinese
    colour ("红" / "红色" / "红色积木"). Unrecognised words best-effort to
    "<word>_cube" so a newly-trained colour still works. Raises ValueError on
    empty input."""
    s = (text or "").strip().lower().replace(" ", "")
    if not s:
        raise ValueError("empty colour")
    if s in _COLOR_ALIASES:
        return _COLOR_ALIASES[s]
    return s if s.endswith("_cube") else (s[:-4] + "_cube" if s.endswith("cube") else s + "_cube")


def _pick_best(cubes: list, cls: str):
    """Highest-score detected cube whose class matches ``cls``, or None."""
    matches = [c for c in cubes if c.get("class_name") == cls]
    return max(matches, key=lambda c: c.get("score", 0.0)) if matches else None


# ── @vertical_grasp_object.mcp: the LLM-callable tools ──────────────────────────
@vertical_grasp_object.mcp("robonix/skill/vertical_grasp_object/pick_cube")
def pick_cube(req: PickCube_Request) -> PickCube_Response:
    """抓取物体：把机械臂移动到指定位置，抓起那里的物体并夹住。The LLM should call
    this when the user wants an object picked up from a place. `position` is a
    coordinate string "x,y" / "x,y,z" (base frame, metres) or a named spot
    (e.g. "中间"). Optional `angle_deg` sets the grasp yaw angle in degrees (hand
    rotation about vertical; empty = default orientation) and `gripper` sets how
    tightly to close, 0.0 (open) – 0.5 (standard grasp) – 1.0 (tightest, for
    smaller objects) (empty = standard grasp). The default grasp is tuned for the
    standard cube (5 cm side length), so leave `gripper` empty for those; use a
    larger value for objects smaller than 5 cm. Prefer passing the
    `grasp_angle_deg` from detect_objects/detect_cubes as `angle_deg` — for
    elongated/tilted objects the right yaw is what makes the grasp succeed.
    `angle_deg` is symmetric: the gripper has two fingers, so θ and θ±180° give
    the identical grasp (wrapped to [-90°,90°] automatically — never fight the
    wrist over a large angle, pass the small equivalent). A SQUARE / CUBE has
    extra symmetry: θ ≡ θ+90° and θ ≡ -θ, so e.g. 30° ≡ 60° ≡ -30° are the SAME
    grasp — just pass the detector's grasp_angle_deg as-is and don't try to hit a
    precise value (0°/90° grasp across the flat faces; the detector already
    returns a face-aligned angle for cubes). Returns ok=true if an object was
    actually grasped. Do NOT verify here — go straight to place_cube; success is
    checked once at the end (see place_cube / verify_grasp)."""
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


@vertical_grasp_object.mcp("robonix/skill/vertical_grasp_object/place_cube")
def place_cube(req: PlaceCube_Request) -> PlaceCube_Response:
    """放置物体：把机械臂移动到指定位置，松开夹住的物体放下。The LLM should call
    this after pick_cube to put the held object down. `position` is a coordinate
    string "x,y" / "x,y,z" (base frame, metres) or a named spot (e.g. "中间"). The
    given z is where the held object's centre is released, so it sets the stacking
    height. To STACK the held object on top of another object, pass an explicit z
    = that base object's z (from detect_cubes, which reports each object's centre)
    + one object height; for a standard 5 cm cube that is base_z + 0.05. Omit z
    ("x,y", z = the table-level pick_z) to place directly on the table.
    Optional `angle_deg` sets the approach yaw angle in degrees (hand rotation
    about vertical; empty = default orientation) and `gripper` sets the release
    aperture on the same 0.0 (fully open) – 0.5 (grasp) – 1.0 (tightest) scale
    (empty = fully open). After releasing, the arm automatically parks aside to
    clear the workspace so the camera has an unobstructed view. Then call
    verify_grasp(mode="place") ONCE to confirm the object actually reached the
    target; if success=false, re-detect and run the pick→place again. (Do NOT
    verify between pick and place — only after this final placed-and-parked
    state.)"""
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
            # Park aside so the placed spot is unobstructed for verify_grasp.
            # Only when the release succeeded — on failure leave the arm as-is
            # for diagnosis.
            if res.get("ok"):
                try:
                    ctrl.move_home()
                except Exception:  # noqa: BLE001
                    log.exception("move_home after place failed (placement still ok)")
        return PlaceCube_Response(ok=bool(res["ok"]), message=str(res["message"]))
    except Exception as exc:  # noqa: BLE001
        log.exception("place_cube failed")
        return PlaceCube_Response(ok=False, message=f"error: {exc}")


@vertical_grasp_object.mcp("robonix/skill/vertical_grasp_object/detect_cubes")
def detect_cubes(req: DetectCubes_Request) -> DetectCubes_Response:
    """（方块/积木的首选检测工具）用头部相机拍一张图，跑 YOLO 识别桌面上的积木，返回每个积木的位置。
    PREFERRED for CUBES / BLOCKS: whenever the target is a cube or block, call
    this FIRST — YOLO is faster and more reliable for cubes than the VLM. Only
    use detect_objects for cubes if this fails or YOLO is unavailable. (For any
    NON-cube object — described by colour/shape/text/spatial hint — use
    detect_objects instead; this tool is limited to the fixed YOLO cube classes.)
    Optional `class_filter` (e.g. "red_cube") returns only that colour. `cubes`
    is a JSON array of {class_name, score, x, y, z, grasp_angle_deg} in the base
    frame (metres), sorted by score; x,y,z is the grasp point (z = cube centre,
    ~2.5 cm above the table). Feed x,y,z + grasp_angle_deg to pick_cube. ok=true
    even if no cube is found. NOTE: z here is a FIXED table-level value (YOLO has
    no depth), so it is only valid for planar (x, y) tasks — do NOT use it to
    judge stacking height / whether a cube landed on top of another. For a
    stacking success check use the VLM path; YOLO can only confirm presence at an
    x, y."""
    det = _state["detector"]
    if det is None:
        return DetectCubes_Response(ok=False, message="skill not activated (no camera connection)", cubes="[]")
    try:
        with _state["lock"]:
            # Detect from the home posture: the workspace clamp is centred on the
            # current EE (x, y), and the arm must not occlude the camera, so park
            # at HOME_POSITION first for a consistent, unobstructed capture.
            ctrl = _controller_or_none()
            if ctrl is not None:
                ctrl.move_home()
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


@vertical_grasp_object.mcp("robonix/skill/vertical_grasp_object/detect_objects")
def detect_objects(req: DetectObjects_Request) -> DetectObjects_Response:
    """（非方块物体的首选检测工具）用头部相机拍一张图，让视觉大模型（VLM）按自然语言描述找出物体，返回其位置。
    Use this for any NON-cube perception need: "桌上有什么"、"你看到了什么"、或
    按描述找物体（颜色/形状/文字/空间或相对位置，如 "红色的杯子"、"离机械臂最近的
    螺丝刀"、"最右边的那个"）。It is open-vocabulary, so it handles arbitrary objects
    the YOLO detector was not trained on. For CUBES / BLOCKS, prefer detect_cubes
    (YOLO — faster and more reliable for cubes); only use this for a cube if
    detect_cubes fails. To list everything on the table, pass a broad instruction
    like "桌面上所有的物体". Pass the description as
    `instruction`. `objects` is a JSON array of
    {class_name, score, x, y, z, grasp_angle_deg} in the base frame (metres),
    which you feed to pick_cube (x,y,z + grasp_angle_deg → angle_deg; passing the
    angle matters for elongated/tilted objects). Note the VLM has no depth: z is
    assumed to be the standard table-level grasp height, so this is best for
    objects resting on the table. ok=true even if no matching object is found
    (objects=[])."""
    det = _state["vlm_detector"]
    if det is None:
        return DetectObjects_Response(
            ok=False,
            message="VLM detection not available (skill not activated, or VLM endpoint not configured — set vlm_base_url/vlm_model or env VLM_BASE_URL/VLM_MODEL)",
            objects="[]")
    try:
        with _state["lock"]:
            # Detect from the home posture (see detect_cubes): consistent
            # workspace-clamp centre + unobstructed camera view.
            ctrl = _controller_or_none()
            if ctrl is not None:
                ctrl.move_home()
            objs = det.detect(req.instruction)
    except ValueError as exc:
        return DetectObjects_Response(ok=False, message=f"bad argument: {exc}", objects="[]")
    except Exception as exc:  # noqa: BLE001
        log.exception("detect_objects failed")
        return DetectObjects_Response(ok=False, message=f"error: {exc}", objects="[]")

    msg = (f"detected {len(objs)} object(s) for {req.instruction!r}" if objs
           else f"no object found for {req.instruction!r}")
    return DetectObjects_Response(ok=True, message=msg, objects=json.dumps(objs, ensure_ascii=False))


@vertical_grasp_object.mcp("robonix/skill/vertical_grasp_object/verify_grasp")
def verify_grasp(req: VerifyGrasp_Request) -> VerifyGrasp_Response:
    """用头部相机复核物体是否被移动到了目标位置（视觉验证）。Call this ONCE after
    place_cube (the arm has parked aside by then, so the view is clear) — a whole
    pick→place is checked at the end, not between the two steps. It re-detects
    `instruction` with the VLM and checks whether the object is now at `position`
    ("x,y", base frame, metres):
      * mode="place" → success=true if the object is now AT the target (the
        normal check: did the pick+place move it there).
      * mode="pick"  → success=true if the object is GONE from `position`
        (available if you ever need to confirm a spot was cleared).
    On success=false, re-run detect_objects for the object's current position and
    redo the pick→place. Returns ok=true if the check ran (VLM reachable), with
    the pass/fail verdict in `success`.

    STACKING CHECK — IMPORTANT: this VLM check (and any detect_cubes fallback)
    only ever compares the PLANAR (x, y) position; neither reports a measured
    height. If the VLM is unavailable and you fall back to detect_cubes (YOLO)
    to re-detect, the returned z is a FIXED table-level value (not real depth),
    so you must NOT use z / height to decide whether a stack succeeded. YOLO can
    only confirm a planar task (there is / isn't a cube at that x, y) — it cannot
    verify that one cube actually landed on top of another. To confirm stacking
    height you need the VLM path (or an external check); with YOLO alone, treat
    only the x, y match as verified."""
    det = _state["vlm_detector"]
    if det is None:
        return VerifyGrasp_Response(
            ok=False, success=False,
            message="VLM not available (skill not activated, or VLM endpoint not configured)")
    try:
        x, y, _z = _resolve_position(req.position)
    except ValueError as exc:
        return VerifyGrasp_Response(ok=False, success=False, message=f"bad position: {exc}")
    try:
        with _state["lock"]:
            res = det.verify(req.instruction, req.mode, (x, y),
                             radius=_state["verify_match_radius"])
    except ValueError as exc:
        return VerifyGrasp_Response(ok=False, success=False, message=f"bad argument: {exc}")
    except Exception as exc:  # noqa: BLE001
        log.exception("verify_grasp failed")
        return VerifyGrasp_Response(ok=False, success=False, message=f"error: {exc}")
    return VerifyGrasp_Response(ok=True, success=bool(res["success"]), message=str(res["message"]))


@vertical_grasp_object.mcp("robonix/skill/vertical_grasp_object/stack_cubes")
def stack_cubes(req: StackCubes_Request) -> StackCubes_Response:
    """积木堆叠：把一个颜色的积木叠到另一个颜色的积木上面（一次调用完成）。
    PREFERRED whenever the user wants to STACK one cube on another, e.g. "把红色
    积木叠到绿色积木上" / "stack the red cube on the yellow one". Call THIS instead
    of orchestrating detect_cubes + pick_cube + place_cube yourself — it does the
    WHOLE job in one call: detect both cubes with the head camera + YOLO, pick up
    the `top_color` cube, and release it centred one cube-height above the
    `base_color` cube, then park the arm.

    `top_color`  — colour of the cube to pick up and place ON TOP.
    `base_color` — colour of the cube to leave on the table as the BASE.
    Both accept Chinese or English colour words or the class name: "红" / "红色" /
    "红色积木" / "red" / "red_cube". Trained cube colours are red / green / yellow.
    `top_color` and `base_color` must be DIFFERENT colours.

    Optional `gripper` sets how tightly to close on the top cube: 0.0 (open) /
    0.5 (standard grasp, the default for a 5 cm cube) / 1.0 (tightest); empty =
    standard grasp.

    Returns ok=true iff the top cube was actually grasped AND released on top of
    the base cube. On failure the message says which step failed (cube not
    detected / grasp failed / place failed) so you can retry. This is a
    planar+height stack: XY comes from the base cube's detected centre and the
    release Z = base cube centre Z + one cube height (config `block_height`, 5 cm
    default); the top cube's grasp angle aligns the pick, the base cube's angle
    aligns the release. NOTE: like detect_cubes, YOLO has no depth, so this does
    NOT visually verify the cube stayed stacked — use verify_grasp (VLM) after if
    you need a success check. To build a taller tower, call this repeatedly (each
    call re-detects the scene)."""
    ctrl = _controller_or_none()
    det = _state["detector"]
    if ctrl is None or det is None:
        return StackCubes_Response(
            ok=False, message="skill not activated (no arm/hand/camera connection)")

    try:
        top_cls = _resolve_color(req.top_color)
        base_cls = _resolve_color(req.base_color)
    except ValueError as exc:
        return StackCubes_Response(ok=False, message=f"bad argument: {exc}")
    if top_cls == base_cls:
        return StackCubes_Response(
            ok=False,
            message=f"top_color and base_color must differ (both resolved to {top_cls})")
    try:
        gripper = _parse_opt_float(req.gripper, "gripper")
    except ValueError as exc:
        return StackCubes_Response(ok=False, message=f"bad argument: {exc}")

    try:
        with _state["lock"]:
            # 1) Detect — park at home first (unobstructed camera view, fixed
            #    workspace-clamp centre), then run YOLO once for both cubes.
            ctrl.move_home()
            cubes = det.detect()

            top = _pick_best(cubes, top_cls)
            base = _pick_best(cubes, base_cls)
            found = sorted({c.get("class_name") for c in cubes})
            if top is None or base is None:
                missing = ", ".join(
                    c for c, o in ((top_cls, top), (base_cls, base)) if o is None)
                return StackCubes_Response(
                    ok=False,
                    message=f"cube(s) not found: {missing}. detected: {found or 'none'}")

            # 2) Pick the top cube (its own grasp angle aligns the gripper).
            pick = ctrl.pick(top["x"], top["y"], top["z"],
                             angle_deg=top.get("grasp_angle_deg"), gripper=gripper)
            if not pick.get("ok"):
                try:
                    ctrl.move_home()
                except Exception:  # noqa: BLE001
                    log.exception("move_home after failed pick failed")
                return StackCubes_Response(
                    ok=False,
                    message=f"failed to grasp {top_cls} at "
                            f"({top['x']}, {top['y']}, {top['z']}): {pick.get('message')}")

            # 3) Place on top of the base cube: same XY, Z = base centre + one
            #    cube height. Use the base cube's angle so the faces line up.
            place_z = round(float(base["z"]) + _state["block_height"], 3)
            place = ctrl.place(base["x"], base["y"], place_z,
                               angle_deg=base.get("grasp_angle_deg"))
            if place.get("ok"):
                ctrl.move_home()
            if not place.get("ok"):
                return StackCubes_Response(
                    ok=False,
                    message=f"grasped {top_cls} but failed to place it on {base_cls} at "
                            f"({base['x']}, {base['y']}, {place_z}): {place.get('message')}")
    except Exception as exc:  # noqa: BLE001
        log.exception("stack_cubes failed")
        return StackCubes_Response(ok=False, message=f"error: {exc}")

    return StackCubes_Response(
        ok=True,
        message=(f"stacked {top_cls} onto {base_cls}: picked ({top['x']}, {top['y']}, "
                 f"{top['z']}), released at ({base['x']}, {base['y']}, {place_z})"))


# ── Lifecycle ────────────────────────────────────────────────────────
@vertical_grasp_object.on_init
def init(cfg: dict):
    """REGISTERED → INACTIVE. Light: parse config only. Don't open the arm/hand
    yet — the user may have spawned us just to inspect the cap tree."""
    pick_z = cfg.get("pick_z", _state["pick_z"])
    try:
        _state["pick_z"] = float(pick_z)
    except (TypeError, ValueError):
        return Err(f"pick_z must be a number, got {pick_z!r}")
    bh = cfg.get("block_height", _state["block_height"])
    try:
        _state["block_height"] = float(bh)
    except (TypeError, ValueError):
        return Err(f"block_height must be a number, got {bh!r}")
    _state["urdf_path"] = str(cfg.get("urdf_path", "") or "")
    # Detection assets: config path → env override → package default. Resolved
    # to absolute against the package root (see models/README.md).
    _state["model_path"] = _resolve_asset(
        str(cfg.get("model_path", "") or os.environ.get("BLOCK_GRASP_MODEL", "")),
        "models/best.pt",
    )
    _state["calib_path"] = _resolve_asset(
        str(cfg.get("calib_path", "") or os.environ.get("VERTICAL_GRASP_OBJECT_CALIB", "")),
        "models/handeye_calib.npz",
    )
    # VLM endpoint for detect_objects: config → env (the deployment's VLM_*).
    # Left empty when unset — detect_objects then reports "not configured"
    # while pick/place/detect_cubes stay fully functional.
    _state["vlm_base_url"] = str(cfg.get("vlm_base_url", "") or os.environ.get("VLM_BASE_URL", ""))
    _state["vlm_api_key"] = str(cfg.get("vlm_api_key", "") or os.environ.get("VLM_API_KEY", ""))
    _state["vlm_model"] = str(cfg.get("vlm_model", "") or os.environ.get("VLM_MODEL", ""))
    gh = cfg.get("vlm_grasp_height", _state["vlm_grasp_height"])
    try:
        _state["vlm_grasp_height"] = float(gh)
    except (TypeError, ValueError):
        return Err(f"vlm_grasp_height must be a number, got {gh!r}")
    tzo = cfg.get("table_z_offset", _state["table_z_offset"])
    try:
        _state["table_z_offset"] = float(tzo)
    except (TypeError, ValueError):
        return Err(f"table_z_offset must be a number, got {tzo!r}")
    _state["detect_debug"] = bool(cfg.get("detect_debug", _state["detect_debug"]))
    vmr = cfg.get("verify_match_radius", _state["verify_match_radius"])
    try:
        _state["verify_match_radius"] = float(vmr)
    except (TypeError, ValueError):
        return Err(f"verify_match_radius must be a number, got {vmr!r}")
    log.info("init ok: pick_z=%.3f urdf_path=%r model=%r calib=%r vlm=%s",
             _state["pick_z"], _state["urdf_path"], _state["model_path"], _state["calib_path"],
             (_state["vlm_model"] or "<unconfigured>"))
    return Ok()


@vertical_grasp_object.on_activate
def activate():
    """INACTIVE → ACTIVE. Heavy: discover + connect the d1_arm / d1_hand
    primitives over gRPC and build the controller (which homes the arm).
    Returns Deferred(...) when a primitive isn't online yet — rbnx boot / the
    executor surface that and retry."""
    from vertical_grasp_object_skill.controller import GraspCubeController
    from vertical_grasp_object_skill.detector import CubeDetector
    from vertical_grasp_object_skill.primitive_clients import connect_primitives

    try:
        log.info("connecting arm/hand/camera primitives via atlas ...")
        arm, hand, camera = connect_primitives(vertical_grasp_object)
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
        urdf_path=_state["urdf_path"], table_z_offset=_state["table_z_offset"],
        debug_dir=(os.path.join(_PKG_ROOT, "logs", "detect")
                   if _state["detect_debug"] else ""),
    )
    # VLM detector is optional: only built when an endpoint is configured, and a
    # failure here must not block pick/place/detect_cubes.
    if _state["vlm_base_url"] and _state["vlm_model"]:
        try:
            from vertical_grasp_object_skill.vlm_detector import VLMDetector
            log.info("initialising VLM detector (model=%s) ...", _state["vlm_model"])
            _state["vlm_detector"] = VLMDetector(
                arm=arm, camera=camera, calib_path=_state["calib_path"],
                base_url=_state["vlm_base_url"], api_key=_state["vlm_api_key"],
                model=_state["vlm_model"], urdf_path=_state["urdf_path"],
                grasp_height=_state["vlm_grasp_height"],
                table_z_offset=_state["table_z_offset"],
            )
        except Exception:  # noqa: BLE001
            log.exception("VLM detector init failed — detect_objects disabled")
            _state["vlm_detector"] = None
    else:
        log.info("VLM endpoint not configured — detect_objects disabled")

    log.info("activated — ready to detect / pick / place")
    return Ok()


@vertical_grasp_object.on_deactivate
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
    _state["vlm_detector"] = None
    log.info("deactivated")
    return Ok()


@vertical_grasp_object.on_shutdown
def shutdown():
    """any → TERMINATED. Last-chance cleanup."""
    ctrl = _state["controller"]
    if ctrl is not None:
        try:
            ctrl.shutdown()
        except Exception:  # noqa: BLE001
            pass
    _state["detector"] = None
    _state["vlm_detector"] = None
    log.info("shutdown")


def main() -> int:
    vertical_grasp_object.run()
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
