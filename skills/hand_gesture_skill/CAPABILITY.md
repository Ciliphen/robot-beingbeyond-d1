---
description: Make the D1 dexterous hand "dance" — a rhythmic open/close finger wiggle for a chosen duration, then the hand returns to open. One MCP tool; hand-only, no arm/camera. User-invocable via the LLM/pilot.
---

# 手指跳舞 (hand_gesture) — make the D1 hand dance

Drives the D1 dexterous hand through a rhythmic open/close "dance": all fingers
wiggle between open and closed for a requested duration, then the hand returns
to a safe open pose. Hand-only — it never moves the arm or uses the camera.
User-invocable via the LLM/pilot ("跳个舞" / "动动手指" / "let the hand dance").

The skill is a pure robonix **consumer**: it discovers the `d1_hand` primitive
via atlas and drives it over gRPC (`hand/move_joint`). It never opens the CAN
bus itself. The dance is a plain open/close ping-pong loop over the 6 hand axes,
lifted from `beingbeyond_d1_sdk.dex_hand.DexHand.gesture_dance`.

## Interface (1 MCP tool)

### `robonix/skill/hand_gesture/gesture_dance`

Make the hand dance for a duration, then leave it open.

| param        | type   | default | meaning                                                              |
|--------------|--------|---------|----------------------------------------------------------------------|
| `duration_s` | string | `"3"`   | Optional: how long to dance, in seconds (e.g. `"5"`). Empty = 3 s; capped at 30 s. |

Returns `{ok, message}`. Blocks until the dance finishes (a synchronous call),
then the hand is left open. `ok=true` if the dance ran to completion.

## Behaviour

- **Motion.** Each of the 6 hand axes (thumb pitch/yaw + index/middle/ring/pinky
  MCP pitch) ping-pongs between a per-axis low and high bound; axes start
  staggered so the fingers move out of phase (a wave-like wiggle rather than a
  fist clench). Amplitude step (0.1), step interval (0.1 s) and the start pose
  are the SDK `gesture_dance` defaults — only the duration is exposed.
- **Rest pose.** On completion (or on any error mid-loop) the hand is commanded
  fully open, so it never stops mid-clench.
- **Serialised.** The single tool holds an internal lock, so overlapping calls
  run one at a time.

## What this skill does NOT do

- No hardware access — the `d1_hand` primitive owns the CAN bus; the skill only
  drives it over atlas-resolved gRPC.
- No arm motion / no camera — the dance is hand-only.
- No async task-id / cancel surface — the dance is a short, synchronous call.

## Dependencies

Connected via atlas at `on_activate` (`connect_primitives`); the skill refuses
to activate — returns `Deferred` for retry — until the primitive is online:

| primitive | contracts used                   | transport |
|-----------|----------------------------------|-----------|
| `d1_hand` | `hand/move_joint`, `hand/info`   | gRPC      |

`hand/info` supplies the axis order (the DexHand 6-axis layout) so the dance
vector maps onto the primitive index-for-index.

## Assets & config

None. The dance has no tunable config and no model / calibration assets.

## Lifecycle

Skill-kind: `rbnx boot` stops at INACTIVE; the executor fires `CMD_ACTIVATE` on
the first MCP call. `on_init` is a no-op (no config). `on_activate` connects the
hand primitive. `on_deactivate` opens the hand and drops the handle.
