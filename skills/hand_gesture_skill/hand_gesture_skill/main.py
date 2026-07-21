# SPDX-License-Identifier: MulanPSL-2.0
"""hand_gesture — make the D1 dexterous hand perform a finger "dance".

One LLM-callable MCP tool for the D1 dexterous hand:

  * gesture_dance(duration_s) — rhythmic open/close wiggle across all fingers
                                for the requested duration, then return to open.

The skill is a pure robonix consumer — it discovers the d1_hand primitive via
atlas and drives it over gRPC (hand/move_joint); it never opens the CAN bus
itself. The dance is a plain open/close ping-pong loop over the 6 hand axes,
lifted from beingbeyond_d1_sdk.dex_hand.DexHand.gesture_dance (the SDK's own
loop calls DexHand.set_joint_pos directly; here the identical [0,1] 6-vector is
sent through the primitive instead).

Lifecycle (skill-kind): rbnx boot stops at INACTIVE; the executor fires
CMD_ACTIVATE on the first MCP call. on_activate connects the hand primitive;
on_deactivate drops it. The shape (declare contract → on_init →
on_activate/on_deactivate → @skill.mcp) is what the framework cares about.
"""
from __future__ import annotations

import logging
import threading
import time

from robonix_api import Skill, Ok, Err, Deferred

hand_gesture = Skill(id="hand_gesture", namespace="robonix/skill/hand_gesture")

# Codegen output (rbnx codegen --mcp): typed dataclasses derived from
# capabilities/lib/hand_gesture/srv/GestureDance.srv.
from hand_gesture_mcp import (  # noqa: E402
    GestureDance_Request, GestureDance_Response,
)

log = logging.getLogger("hand_gesture")
logging.basicConfig(level=logging.INFO, format="[hand_gesture] %(levelname)s %(message)s")

# Dance parameters. Only duration is pilot-facing; the wiggle amplitude / step
# interval / start pose are the SDK gesture_dance defaults (see dex_hand.py).
_DEFAULT_DURATION_S = 3.0
_MAX_DURATION_S = 30.0    # clamp: pick/place-style calls are short; don't let one dance block for minutes
_STEP = 0.1               # m: per-step amplitude change (SDK default)
_INTERVAL_S = 0.1         # s: pause between steps (SDK default)
# Per-axis start pose and travel limits, in DexHand.joint_names order
# (thumb_cmc_pitch, thumb_cmc_yaw, index_mcp_pitch, middle_mcp_pitch,
# ring_mcp_pitch, pinky_mcp_pitch). Copied verbatim from
# DexHand.gesture_dance so the motion matches the SDK.
_START = [0.0, 0.4, 0.4, 0.6, 0.8, 1.0]
_LO = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
_HI = [0.4, 0.4, 1.0, 1.0, 1.0, 1.0]

_state: dict = {
    "hand": None,
    "lock": threading.Lock(),
}


def _parse_duration(text: str) -> float:
    """Parse the optional duration field: empty/blank → default, otherwise the
    float (clamped to (0, _MAX_DURATION_S]). Raises ValueError on a non-numeric
    or non-positive value."""
    s = (text or "").strip()
    if not s:
        return _DEFAULT_DURATION_S
    try:
        v = float(s)
    except ValueError:
        raise ValueError(f"duration_s must be a number, got {text!r}")
    if v <= 0:
        raise ValueError(f"duration_s must be positive, got {v}")
    return min(v, _MAX_DURATION_S)


def _dance(hand, duration_s: float) -> None:
    """Ping-pong the 6 hand axes between _LO and _HI for duration_s, then open.

    Direct re-implementation of DexHand.gesture_dance over the primitive: each
    axis starts at _START (clamped into [_LO, _HI]), all moving toward _LO;
    when an axis hits a bound it reverses. The final open_hand mirrors the
    SDK script's safe-open in its finally block."""
    n = hand.num_joints
    if n != len(_START):
        raise RuntimeError(
            f"hand reports {n} axes, dance is defined for {len(_START)} "
            "(expects the D1 6-axis DexHand layout)")

    cur = [min(max(v, _LO[i]), _HI[i]) for i, v in enumerate(_START)]
    direction = [-1.0] * n
    t_end = time.time() + duration_s
    try:
        while time.time() < t_end:
            hand.set_joint_pos(cur)
            time.sleep(_INTERVAL_S)
            for i in range(n):
                cur[i] += direction[i] * _STEP
                if cur[i] <= _LO[i]:
                    cur[i] = _LO[i]
                    direction[i] = 1.0
                elif cur[i] >= _HI[i]:
                    cur[i] = _HI[i]
                    direction[i] = -1.0
    finally:
        # Return to a safe open pose regardless of how the loop ended.
        hand.open_hand()


# ── @hand_gesture.mcp: the LLM-callable tool ─────────────────────────
@hand_gesture.mcp("robonix/skill/hand_gesture/gesture_dance")
def gesture_dance(req: GestureDance_Request) -> GestureDance_Response:
    """手指跳舞：让灵巧手的手指有节奏地开合摆动一段时间，像跳舞一样，结束后手张开。
    The LLM should call this when the user wants the hand to "dance", wiggle its
    fingers, or do a little gesture/performance, e.g. "跳个舞" / "动动手指" /
    "let the hand dance". Optional `duration_s` is how long to dance in seconds
    (empty = 3 s; capped at 30 s). Blocks until the dance finishes, then leaves
    the hand open. Returns ok=true if the dance ran to completion."""
    hand = _state["hand"]
    if hand is None:
        return GestureDance_Response(ok=False, message="skill not activated (no hand connection)")
    try:
        duration_s = _parse_duration(req.duration_s)
    except ValueError as exc:
        return GestureDance_Response(ok=False, message=f"bad argument: {exc}")
    try:
        with _state["lock"]:
            _dance(hand, duration_s)
    except Exception as exc:  # noqa: BLE001
        log.exception("gesture_dance failed")
        return GestureDance_Response(ok=False, message=f"error: {exc}")
    return GestureDance_Response(ok=True, message=f"danced for {duration_s:g}s")


# ── Lifecycle ────────────────────────────────────────────────────────
@hand_gesture.on_init
def init(cfg: dict):
    """REGISTERED → INACTIVE. Light: nothing to configure (the dance has no
    tunable config). Don't open the hand yet — the user may have spawned us
    just to inspect the cap tree."""
    _ = cfg
    log.info("init ok")
    return Ok()


@hand_gesture.on_activate
def activate():
    """INACTIVE → ACTIVE. Discover + connect the d1_hand primitive over gRPC.
    Returns Deferred(...) when the primitive isn't online yet — rbnx boot / the
    executor surface that and retry."""
    from hand_gesture_skill.primitive_clients import connect_primitives

    try:
        log.info("connecting hand primitive via atlas ...")
        hand = connect_primitives(hand_gesture)
    except RuntimeError as exc:
        # Most likely the primitive is not up yet — let the framework retry.
        return Deferred(str(exc))

    _state["hand"] = hand
    log.info("activated — ready to dance (%d axes)", hand.num_joints)
    return Ok()


@hand_gesture.on_deactivate
def deactivate():
    """ACTIVE → INACTIVE. Open the hand and drop the handle; the primitive
    channel is auto-closed by the Capability framework. Idempotent."""
    hand = _state["hand"]
    if hand is not None:
        try:
            hand.open_hand()
        except Exception:  # noqa: BLE001
            log.exception("open_hand on deactivate failed")
    _state["hand"] = None
    log.info("deactivated")
    return Ok()


@hand_gesture.on_shutdown
def shutdown():
    """any → TERMINATED. Last-chance cleanup."""
    _state["hand"] = None
    log.info("shutdown")


def main() -> int:
    hand_gesture.run()
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
