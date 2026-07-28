---
description: Detect an object on the table (YOLO for cubes/blocks, open-vocabulary VLM for other objects), pick it up / place it down with the D1 dexterous hand, stack one cube on another, drop a cube into a fixed container, sort cubes by colour into per-colour spots, and verify success — optional grasp angle and gripper aperture.
---

# 垂直抓取物体 (vertical_grasp_object) — detect / pick / place objects on the table

Perceives objects on the table with the head camera and manipulates them with
the D1 6-DoF arm + dexterous hand. A YOLO cube detector (preferred for
cubes/blocks — faster and more reliable) and a VLM detector (open-vocabulary,
for any other object) report base-frame grasp points — with a grasp yaw from the
object's orientation — that feed the pick/place motion; a VLM `verify_grasp`
check closes the loop so a missed grasp can be retried. A one-call `stack_cubes`
tool stacks one coloured cube on another (detect → pick → place). User-invocable
via the LLM/pilot.

The skill is a pure robonix **consumer**: it discovers the `d1_arm` / `d1_hand`
/ `d1_camera` primitives via atlas and drives them over gRPC. It never opens the
serial link / CAN bus / RealSense itself. IK/FK, YOLO, and the hand-eye
projection run locally (pure compute; the `object_detect` + `block_grasp` stacks
are vendored into this package, FK/IK from the `beingbeyond_d1_sdk` wheel — no
external repo needed).

## Interface (8 MCP tools)

All under the `robonix/skill/vertical_grasp_object/*` namespace. Coordinates are base-frame
metres; a location string is `"x,y"` (Z = configured `pick_z`), `"x,y,z"`, or a
named spot (e.g. `"中间"`). ASCII and full-width commas both parse.

For the common **"stack cube A on cube B"** request, prefer the one-call
`stack_cubes` tool (below) over orchestrating `detect_cubes` + `pick_cube` +
`place_cube` by hand.

### `robonix/skill/vertical_grasp_object/detect_objects` — **preferred for non-cube objects**

Detect an arbitrary object by natural-language description with a
vision-language model (open vocabulary — **not** limited to the YOLO cube
classes). Use this for any **non-cube** object, and for open-ended perception —
"what's on the table", "what do you see", or finding an object by description
(colour / shape / text / spatial hint). For **cubes / blocks**, prefer
`detect_cubes` (YOLO, faster and more reliable); only use this for a cube if
`detect_cubes` fails or YOLO is unavailable.

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

### `robonix/skill/vertical_grasp_object/detect_cubes` — **preferred for cubes/blocks**

Detect cubes on the table with YOLO-OBB (fixed, trained cube classes). **Call
this first whenever the target is a cube or block** — YOLO is faster and more
reliable for cubes than the VLM. Only fall back to `detect_objects` for a cube
if this fails or YOLO is unavailable.

| param          | type   | default | meaning                                          |
|----------------|--------|---------|--------------------------------------------------|
| `class_filter` | string | ""      | Optional: only return this class (e.g. `red_cube`); empty = all. |

Returns `{ok, message, cubes}`. `cubes` is a JSON array of
`{class_name, score, x, y, z, grasp_angle_deg}` in the base frame, sorted by
score; `x, y, z` is the grasp point (z = cube centre, ~2.5 cm above the table).
`ok=true` even if no cube is found. **`z` is a fixed table-level value** (YOLO
has no depth) — valid for planar (x, y) tasks only; do **not** use it to judge
stacking height or whether a cube landed on another (see `verify_grasp`).

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

Returns `{ok, message}`. On a successful release the arm automatically parks
aside (clears the workspace) so the placed spot is unobstructed for
`verify_grasp`.

**Stacking**: to place the held object on top of another, pass an explicit `z` =
the base object's `z` (from `detect_objects` / `detect_cubes`, which report each
object's centre) + one object height; for a standard 5 cm cube that is
`base_z + 0.05`. Omit `z` (`"x,y"`) to place directly on the table.

### `robonix/skill/vertical_grasp_object/verify_grasp`

Visually confirm the object reached its target with the VLM: re-detect the
object and check whether it is now at `position`. Call **once after
`place_cube`** (the arm has parked aside, so the view is clear) — a whole
pick→place is checked at the end, not between the two steps.

| param         | type   | default | meaning                                                          |
|---------------|--------|---------|------------------------------------------------------------------|
| `instruction` | string | —       | Description of the object (as given to `detect_objects`).        |
| `mode`        | string | —       | `"place"` (normal: success iff the object is now PRESENT at the target) or `"pick"` (success iff now ABSENT there — for confirming a spot was cleared). |
| `position`    | string | —       | `"x,y"` (base frame, m) the object was placed at (or picked from, for `pick`). |

Returns `{ok, success, message}`. `ok=true` if the check ran (VLM reachable);
`success` is the verdict. On `success=false`, re-run `detect_objects` for the
current position and redo the pick→place. A detection within
`verify_match_radius` (config, default 5 cm) of `position` counts as "at" it.

**Stacking verification is planar only.** This check compares the **(x, y)**
position — no measured height. If the VLM is unavailable and you fall back to
`detect_cubes` (YOLO) to re-detect, the returned `z` is a **fixed table-level
value**, not real depth, so it **cannot** confirm that one cube landed on top of
another. YOLO is limited to planar tasks (is / isn't a cube at that x, y);
verifying stacking height needs the VLM path (or an external check). With YOLO
alone, treat only the x, y match as verified.

### `robonix/skill/vertical_grasp_object/stack_cubes` — **preferred for stacking cubes**

Stack one coloured cube on top of another in a **single call**: detect both
cubes (YOLO), pick the `top_color` cube, and release it centred one cube-height
above the `base_color` cube, then park the arm. **Whenever the user wants to
stack cubes, call this** instead of chaining `detect_cubes` + `pick_cube` +
`place_cube` yourself.

| param        | type   | default | meaning                                                                                 |
|--------------|--------|---------|-----------------------------------------------------------------------------------------|
| `top_color`  | string | —       | Colour of the cube to pick up and place ON TOP. Chinese/English/class name: `"红色"` / `"red"` / `"red_cube"`. |
| `base_color` | string | —       | Colour of the BASE cube it is stacked onto (same forms). Must DIFFER from `top_color`.  |
| `gripper`    | string | ""      | Optional close amount on the top cube: `0.0` (open) / `0.5` (standard) / `1.0` (tightest); empty = standard grasp. |

Returns `{ok, message}`. `ok=true` **iff** the top cube was grasped **and**
released on top of the base cube. On failure `message` names the step that failed
(cube not detected / grasp failed / place failed) and lists which classes were
detected, so the caller can retry. Trained cube colours: **red / blue / green /
yellow**. The XY comes from the base cube's detected centre and the release
`z = base cube centre z + block_height` (config, 5 cm default); the top cube's
grasp angle aligns the pick, the base cube's angle aligns the release.

**No stacking-height verification.** Like `detect_cubes`, YOLO has no depth, so
this does not confirm the cube stayed stacked — it is a planar-XY + geometric-Z
placement. For a success check, call `verify_grasp` (VLM) afterwards. To build a
taller tower, call `stack_cubes` repeatedly (each call re-detects the scene).

### `robonix/skill/vertical_grasp_object/put_cube_in_container` — **preferred for "put a cube in the box/plate"**

Put one coloured cube into the **fixed container** (box / plate / bowl) in a
**single call**: detect the cube (YOLO), pick it up, and release it at the fixed
container drop-off position, then park the arm. **Whenever the user wants to put
/ drop a cube into a box / plate / bowl, call this** instead of chaining
`detect_cubes` + `pick_cube` + `place_cube` yourself.

| param     | type   | default | meaning                                                                                    |
|-----------|--------|---------|--------------------------------------------------------------------------------------------|
| `color`   | string | —       | Colour of the cube to pick up and drop in. Chinese/English/class name: `"红色"` / `"red"` / `"red_cube"`. Trained colours: red / blue / green / yellow. |
| `gripper` | string | ""      | Optional close amount on the cube: `0.0` (open) / `0.5` (standard) / `1.0` (tightest); empty = standard grasp. |

Returns `{ok, message}`. `ok=true` **iff** the cube was grasped **and** released
into the container. On failure `message` names the step that failed (cube not
detected / grasp failed / place failed) and lists which classes were detected, so
the caller can retry. The **destination is fixed** (the configured container spot,
base frame `(0.160, -0.245)`) — the caller does **not** supply a destination. The
cube is released at the **same Z it was grasped at** (the detected cube centre,
one table plane), so it drops just above the table inside the container.

**No landed-in-container verification.** Like `detect_cubes`, YOLO has no depth,
so this does not confirm the cube ended up inside the container — it is a
planar-XY placement at the fixed spot. For a success check, call `verify_grasp`
(VLM) afterwards.

### `robonix/skill/vertical_grasp_object/sort_cubes` — **preferred for "sort/classify cubes by colour"**

Sort every cube on the table by colour into its own **fixed per-colour spot** in
a **single call**: detect all cubes (YOLO), then for each cube pick it up and
release it at that colour's fixed position, parking the arm between cubes.
**Whenever the user wants to classify / sort cubes by colour, call this** instead
of chaining `detect_cubes` + `pick_cube` + `place_cube` yourself.

| param     | type   | default | meaning                                                                                    |
|-----------|--------|---------|--------------------------------------------------------------------------------------------|
| `gripper` | string | ""      | Optional close amount on each cube: `0.0` (open) / `0.5` (standard) / `1.0` (tightest); empty = standard grasp. |

Returns `{ok, message}`. `ok=true` **iff** every detected sortable cube was
grasped **and** released at its colour's spot. `message` lists each cube's
outcome (placed at / failure step). The **destinations are fixed** (configured in
the skill) — the caller supplies no positions:

| colour        | spot (base frame, m) |
|---------------|----------------------|
| `yellow_cube` | `(0.328, 0.398)`     |
| `green_cube`  | `(0.207, 0.422)`     |
| `red_cube`    | `(0.078, 0.446)`     |

The scene is detected **once** at the start (cubes only move because this arm
moves them), and cubes are sorted highest-score first. A detected cube whose
colour has **no configured spot** (e.g. blue) is skipped and named in `message`.
Each cube is released at the **same Z it was grasped at** (the detected cube
centre, one table plane), so it drops just above the table at its spot.

**No landed verification.** Like `detect_cubes`, YOLO has no depth, so this does
not visually confirm where each cube landed — it is planar-XY placement at fixed
spots. For a success check, call `verify_grasp` (VLM) afterwards.

## Usage pattern (recommended closed loop)

1. **Detect** — for a **cube/block**, call `detect_cubes` (YOLO); for any
   **other object**, call `detect_objects` (VLM) with the target description. Get
   its `x, y, z` and `grasp_angle_deg`. (Fall back to the other detector only if
   the preferred one fails or is unavailable.)
2. **Pick** — call `pick_cube` with that `position`, passing `grasp_angle_deg`
   as `angle_deg` (essential for tilted/elongated objects). Blocks until done.
   Do **not** verify here.
3. **Place** — call `place_cube` with the destination `position` (add a `z`
   offset to stack). On success the arm automatically parks aside, clearing the
   camera's view of the target.
4. **Verify (once, at the end)** — call
   `verify_grasp(mode="place", position=<place spot>)` to confirm the object
   actually reached the target. If `success=false`, go back to step 1 (re-detect)
   and redo the pick→place.

Verification happens only at this final placed-and-parked state, not between
pick and place. Detection, pick/place and verify share the arm, so calls are
serialised by an internal lock — issue them one at a time.

## Behaviour

- **Detection geometry (shared).** Both detectors aim the head at the hand-eye
  calibration pose, grab one RGB frame, and resolve pixels to base-frame XY via
  the calibration homography + perspective correction (object sits above the
  table plane) + a workspace clamp around the current EE. `z` = interpolated
  table Z at that XY + the object's grasp height. The shared plumbing lives in
  `HeadCameraProjector`; `CubeDetector` adds YOLO, `VLMDetector` adds the VLM.
- **Grasp angle** is applied as a joint-6 (wrist-roll) offset *after* IK, so the
  target position stays exact regardless of the requested yaw. It is wrapped to
  `[-90°, 90°]` first — a two-finger gripper is 180°-symmetric (θ ≡ θ±180°), so
  this keeps the wrist in range and takes the shorter roll. Object-shape
  symmetry is left to the caller: a square/cube also has θ ≡ θ+90° ≡ -θ (so
  30° ≡ 60° ≡ -30° are one grasp), which the pilot is told about in the
  `pick_cube` docstring rather than assumed for every target.
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
- Config keys (see `config.spec`): `pick_z`, `block_height`, `urdf_path`,
  `model_path`, `calib_path`, `vlm_base_url`, `vlm_api_key`, `vlm_model`,
  `vlm_grasp_height`, `verify_match_radius`.

## Lifecycle

Skill-kind: `rbnx boot` stops at INACTIVE; the executor fires `CMD_ACTIVATE` on
the first MCP call. `on_init` is light (parse config only). `on_activate` is
heavy — connects the primitives, homes the arm, loads YOLO, and (if configured)
builds the VLM detector; a VLM-build failure does not block the other tools.
`on_deactivate` releases the held object and drops the detectors.
