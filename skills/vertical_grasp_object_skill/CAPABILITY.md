---
description: Detect an object on the table (open-vocabulary via a VLM, or YOLO cubes), pick it up / place it down with the D1 dexterous hand, and verify success — optional grasp angle and gripper aperture.
---

# 垂直抓取物体 (vertical_grasp_object) — detect / pick / place objects on the table

Perceives objects on the table with the head camera and manipulates them with
the D1 6-DoF arm + dexterous hand. A VLM detector (open-vocabulary, preferred)
and a YOLO cube detector (fallback) report base-frame grasp points — with a
grasp yaw from the object's orientation — that feed the pick/place motion; a VLM
`verify_grasp` check closes the loop so a missed grasp can be retried.
User-invocable via the LLM/pilot.

The skill is a pure robonix **consumer**: it discovers the `d1_arm` / `d1_hand`
/ `d1_camera` primitives via atlas and drives them over gRPC. It never opens the
serial link / CAN bus / RealSense itself. IK/FK, YOLO, and the hand-eye
projection run locally (pure compute, reused from the Beingbeyond_D1
`block_grasp` stack via `BEINGBEYOND_PATH`).

## Interface (5 MCP tools)

All under the `robonix/skill/vertical_grasp_object/*` namespace. Coordinates are base-frame
metres; a location string is `"x,y"` (Z = configured `pick_z`), `"x,y,z"`, or a
named spot (e.g. `"中间"`). ASCII and full-width commas both parse.

### `robonix/skill/vertical_grasp_object/detect_objects` — **preferred detector**

Detect an arbitrary object by natural-language description with a
vision-language model (open vocabulary — **not** limited to the YOLO cube
classes). **Call this first** for any perception need — "what's on the table",
"what do you see", or finding an object by description (including cubes). Only
fall back to `detect_cubes` if this fails or the VLM is not configured.

| param         | type   | default | meaning                                                              |
|---------------|--------|---------|----------------------------------------------------------------------|
| `instruction` | string | —       | What to locate: colour / shape / text / spatial hint, e.g. `"红色的杯子"`, `"离机械臂最近的螺丝刀"`, `"最右边的那个"`. Use a broad phrase like `"桌面上所有的物体"` to list everything. |

Returns `{ok, message, objects}`. `objects` is a JSON array of
`{class_name, score, x, y, z, grasp_angle_deg}` in the base frame — the same
schema as `detect_cubes`, so it feeds straight into `pick_cube` (x,y,z +
`grasp_angle_deg` → `angle_deg`). The VLM reports each object's **orientation**,
so `grasp_angle_deg` is meaningful for tilted / elongated objects — pass it to
`pick_cube` or the grasp misses. `ok=true` even if no matching object is found
(`objects=[]`).

**No depth**: the VLM returns only a 2-D oriented box, so `z` is assumed to be a
fixed grasp height above the calibrated table plane (`vlm_grasp_height`). Best
for objects resting on the table; objects at other heights will be off in Z.

### `robonix/skill/vertical_grasp_object/detect_cubes` — **YOLO fallback**

Detect cubes on the table with YOLO-OBB (fixed, trained cube classes). Fallback
only — use when `detect_objects` failed or the VLM is unavailable.

| param          | type   | default | meaning                                          |
|----------------|--------|---------|--------------------------------------------------|
| `class_filter` | string | ""      | Optional: only return this class (e.g. `red_cube`); empty = all. |

Returns `{ok, message, cubes}`. `cubes` is a JSON array of
`{class_name, score, x, y, z, grasp_angle_deg}` in the base frame, sorted by
score; `x, y, z` is the grasp point (z = cube centre, ~2.5 cm above the table).
`ok=true` even if no cube is found.

### `robonix/skill/vertical_grasp_object/pick_cube`

Move to a location, grasp the object there, lift and hold.

| param       | type   | default | meaning                                                                                  |
|-------------|--------|---------|------------------------------------------------------------------------------------------|
| `position`  | string | —       | Where to pick — `"x,y"` / `"x,y,z"` (base frame, m) or a named spot.                      |
| `angle_deg` | string | ""      | Optional grasp yaw in degrees (hand rotation about vertical, applied as a joint-6 offset after IK); empty = default orientation. |
| `gripper`   | string | ""      | Optional close amount: `0.0` (open) / `0.5` (standard grasp) / `1.0` (tightest, for small objects); empty = standard grasp. Tuned for the standard 5 cm cube. |

Returns `{ok, message}`. `ok=true` iff the hand actually closed on an object.

### `robonix/skill/vertical_grasp_object/place_cube`

Move to a location, release the held object, lift.

| param       | type   | default | meaning                                                                                  |
|-------------|--------|---------|------------------------------------------------------------------------------------------|
| `position`  | string | —       | Where to release — the given `z` is where the held object's centre is released, so it sets the stacking height. |
| `angle_deg` | string | ""      | Optional approach yaw in degrees; empty = default orientation.                           |
| `gripper`   | string | ""      | Optional release aperture on the same `0.0`–`0.5`–`1.0` scale; empty = fully open.       |

Returns `{ok, message}`.

**Stacking**: to place the held object on top of another, pass an explicit `z` =
the base object's `z` (from `detect_objects` / `detect_cubes`, which report each
object's centre) + one object height; for a standard 5 cm cube that is
`base_z + 0.05`. Omit `z` (`"x,y"`) to place directly on the table.

### `robonix/skill/vertical_grasp_object/verify_grasp`

Visually confirm a pick / place with the VLM: re-detect the object and check
whether it is now at `position`. Call after every `pick_cube` / `place_cube`.

| param         | type   | default | meaning                                                          |
|---------------|--------|---------|------------------------------------------------------------------|
| `instruction` | string | —       | Description of the object (as given to `detect_objects`).        |
| `mode`        | string | —       | `"pick"` (success iff the object is now ABSENT there — lifted) or `"place"` (success iff now PRESENT there). |
| `position`    | string | —       | `"x,y"` (base frame, m) the object was picked from / placed at.  |

Returns `{ok, success, message}`. `ok=true` if the check ran (VLM reachable);
`success` is the pick/place verdict. On `success=false`, re-run `detect_objects`
for the current position and retry. A detection within `verify_match_radius`
(config, default 5 cm) of `position` counts as "at" it.

## Usage pattern (recommended closed loop)

1. **Detect** — call `detect_objects` with the target description to get its
   `x, y, z` and `grasp_angle_deg`. (Fall back to `detect_cubes` only if the VLM
   is unavailable.)
2. **Pick** — call `pick_cube` with that `position`, passing `grasp_angle_deg`
   as `angle_deg` (essential for tilted/elongated objects). Blocks until done.
3. **Verify pick** — call `verify_grasp(mode="pick", position=<pick spot>)`. If
   `success=false`, go back to step 1 (re-detect) and retry the pick.
4. **Place** — call `place_cube` with the destination `position` (add a `z`
   offset to stack).
5. **Verify place** — call `verify_grasp(mode="place", position=<place spot>)`.
   If `success=false`, re-detect and place again.

Detection, pick/place and verify share the arm, so calls are serialised by an
internal lock — issue them one at a time.

## Behaviour

- **Detection geometry (shared).** Both detectors aim the head at the hand-eye
  calibration pose, grab one RGB frame, and resolve pixels to base-frame XY via
  the calibration homography + perspective correction (object sits above the
  table plane) + a workspace clamp around the current EE. `z` = interpolated
  table Z at that XY + the object's grasp height. The shared plumbing lives in
  `HeadCameraProjector`; `CubeDetector` adds YOLO, `VLMDetector` adds the VLM.
- **Grasp angle** is applied as a joint-6 (wrist-roll) offset *after* IK, so the
  target position stays exact regardless of the requested yaw.
- **Gripper aperture** interpolates the D1 hand pose across three anchors:
  `0.0` = `HAND_OPEN`, `0.5` = `HAND_GRASP` (standard), `1.0` = `HAND_CLOSE`.
- **VLM call.** `detect_objects` POSTs the frame (base64 JPEG) + a Chinese
  detection prompt to an OpenAI-compatible `/chat/completions` endpoint over
  stdlib `urllib` (no extra dependency), constrained to a JSON reply, then
  parses each object's oriented box (centre + long/short side + rotation).
  Malformed / out-of-range boxes are dropped; rotation feeds
  `estimate_grasp_angle_deg` for the grasp yaw.
- **verify_grasp** re-runs the VLM detector and measures the nearest detection's
  distance to the queried `position`: within `verify_match_radius` counts as
  "present". `pick` passes iff now absent, `place` iff now present.

## What this skill does NOT do

- No hardware access — the arm / hand / camera primitives own the serial link /
  CAN bus / RealSense; the skill only drives them over atlas-resolved gRPC.
- No depth-based perception — both detectors use a 2-D homography onto a fixed
  table plane, not the RealSense depth stream.
- No base movement — D1 is a fixed-base robot.
- No async task-id / cancel surface — pick/place are short, synchronous calls.

## Dependencies

Connected via atlas at `on_activate` (`connect_primitives`); the skill refuses
to activate — returns `Deferred` for retry — until they are online:

| primitive   | contracts used                                              | transport |
|-------------|-------------------------------------------------------------|-----------|
| `d1_arm`    | `arm/get_state`, `arm/move_joint`, `arm/set_head`           | gRPC      |
| `d1_hand`   | `hand/move_joint`, `hand/get_state`, `hand/info`            | gRPC      |
| `d1_camera` | `camera/snapshot`                                           | gRPC      |

External (for `detect_objects` and `verify_grasp`): an OpenAI-compatible VLM
endpoint, configured via `vlm_base_url` / `vlm_api_key` / `vlm_model` (falling
back to env `VLM_BASE_URL` / `VLM_API_KEY` / `VLM_MODEL`). Left unset, those two
tools are disabled and report "not configured"; the other three are unaffected.

## Assets & config

- YOLO weights + hand-eye calibration under `./models/` (`best.pt`,
  `handeye_calib.npz`; robot-specific, git-ignored — see `models/README.md`).
  The calibration is shared by both detectors.
- Config keys (see `config.spec`): `pick_z`, `urdf_path`, `model_path`,
  `calib_path`, `vlm_base_url`, `vlm_api_key`, `vlm_model`, `vlm_grasp_height`,
  `verify_match_radius`.

## Lifecycle

Skill-kind: `rbnx boot` stops at INACTIVE; the executor fires `CMD_ACTIVATE` on
the first MCP call. `on_init` is light (parse config only). `on_activate` is
heavy — connects the primitives, homes the arm, loads YOLO, and (if configured)
builds the VLM detector; a VLM-build failure does not block the other tools.
`on_deactivate` releases the held object and drops the detectors.
