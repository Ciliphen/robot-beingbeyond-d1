# SPDX-License-Identifier: MulanPSL-2.0
"""VLM object detector for the vertical_grasp_object skill (open-vocabulary, report-only).

Where ``CubeDetector`` runs a fixed YOLO-OBB model (only the cube classes it was
trained on), this resolves an arbitrary natural-language instruction ("抓红色的
杯子", "拿离机械臂最近的螺丝刀") by asking a vision-language model to locate the
target in a head-camera frame and return its normalised ORIENTED box (centre +
long/short side + rotation, so tilted objects get a usable grasp yaw). The box
is then projected to a base-frame (x, y, z) through the *same* hand-eye calibration
and perspective correction as the YOLO path (shared ``HeadCameraProjector``), so
the result plugs straight into ``pick_cube`` just like ``detect_cubes`` output.

The VLM is any OpenAI-compatible chat endpoint (base_url / api_key / model). It
is called over stdlib ``urllib`` — no extra dependency — with the frame inlined
as a base64 JPEG and the reply constrained to JSON. The prompt mirrors roboarm's
``user_instruction_prompt``.

Limitation — no depth: the VLM gives only a 2-D box, so Z is assumed to be a
fixed grasp height above the calibrated table plane (``grasp_height``). This
suits objects resting on the table; objects at other heights will be off in Z.
"""
from __future__ import annotations

import base64
import json
import re
import urllib.request
from typing import Dict, List

import cv2
import numpy as np

from block_grasp.config import BLOCK_SIZE
from block_grasp.coordinate_utils import estimate_grasp_angle_deg

from vertical_grasp_object_skill.detector import HeadCameraProjector


# Prompt asking the VLM to locate the instructed target and return, for each,
# an ORIENTED box: centre + the object's own long/short side + its rotation.
# Mirrors roboarm's user_instruction_prompt; {instruction} is substituted at
# call time. Reply is constrained to a JSON object.
_DETECT_PROMPT = """你是一个智能机械臂的视觉控制中枢。请根据用户的自然语言指令，分析图片场景，\
找出机械臂需要抓取的目标物体，并用“有向边界框”标出。有向框要贴合物体本身的朝向：\
box_width 是物体较长的一边、box_height 是较短的一边，box_rotation_deg 是物体长边相对图像水平方向的倾角。

坐标系定义：图片左上角为原点 (0, 0)，向右为 x 轴 (0→1)，向下为 y 轴 (0→1)。box_center/width/height 都用\
相对整幅图的归一化比例表示，范围 0~1。机械臂底座通常位于图片下方（y 坐标较大）。

旋转角定义：box_rotation_deg 表示物体长边相对图像“水平向右”方向的夹角，单位度，范围 -90~90。\
0 表示长边水平；正值表示长边从水平位置顺时针（向下）旋转；接近 ±90 表示长边竖直。方形/圆形物体没有明显长边时填 0。

用户指令可能包含颜色/形状特征、相对距离（如“最近的”）或空间位置（如“最右边的”“中间的”）。指令来自\
语音转写，可能有歧义，请结合画面合理推断。若有多个符合条件的物体，任选其一即可。

请只输出一个纯净的 JSON 对象（不要 Markdown 代码块、不要多余文字），字段如下：
- "failed"：布尔值。未找到符合条件的目标时为 true，此时 "objects" 为空数组。
- "objects"：数组，每个元素代表一个目标物体，包含：
  - "class_name"：字符串，物体的具体类别，须包含颜色和形状信息，若有文字也一并写入（如“红色圆柱形杯子”）。
  - "box_center_x"：有向框中心 x，0~1，保留 3 位小数。
  - "box_center_y"：有向框中心 y，0~1，保留 3 位小数。
  - "box_width"：物体较长一边的长度，0~1，保留 3 位小数。
  - "box_height"：物体较短一边的长度，0~1，保留 3 位小数。
  - "box_rotation_deg"：物体长边倾角，-90~90，保留 1 位小数。

示例（用户指令“抓最右边的蓝色积木”，方块无明显长边）：
{"failed": false, "objects": [{"class_name": "蓝色方形积木", "box_center_x": 0.821, "box_center_y": 0.131, "box_width": 0.09, "box_height": 0.088, "box_rotation_deg": 0.0}]}

示例（用户指令“抓那支斜放的笔”，笔身从左下到右上倾斜）：
{"failed": false, "objects": [{"class_name": "黑色长杆钢笔", "box_center_x": 0.42, "box_center_y": 0.55, "box_width": 0.30, "box_height": 0.05, "box_rotation_deg": -35.0}]}

示例（用户指令“抓黄色的球”，画面中没有）：
{"failed": true, "objects": []}

现在，请根据图片执行任务。用户指令："{instruction}"
"""


def _extract_json(text: str) -> str:
    """Strip a Markdown code fence if the model wrapped its JSON in one."""
    m = re.search(r"```(?:json|JSON)?\s*(.*?)```", text, flags=re.S)
    return m.group(1).strip() if m else text.strip()


def _vlm_chat(base_url: str, api_key: str, model: str, prompt: str,
              image_b64: str, timeout: float) -> str:
    """POST an OpenAI-compatible multimodal chat request and return the reply
    text. Uses stdlib urllib (no openai SDK). Raises on transport / HTTP error.
    """
    url = base_url.rstrip("/")
    if not url.endswith("/chat/completions"):
        url = url + "/chat/completions"
    payload = {
        "model": model,
        "messages": [
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": prompt},
                    {"type": "image_url",
                     "image_url": {"url": f"data:image/jpeg;base64,{image_b64}"}},
                ],
            }
        ],
        "temperature": 0.0,
        "response_format": {"type": "json_object"},
    }
    req = urllib.request.Request(
        url,
        data=json.dumps(payload).encode("utf-8"),
        method="POST",
        headers={
            "Content-Type": "application/json",
            "Authorization": f"Bearer {api_key}",
        },
    )
    with urllib.request.urlopen(req, timeout=timeout) as resp:  # noqa: S310 (trusted endpoint)
        body = json.loads(resp.read().decode("utf-8"))
    choices = body.get("choices") or []
    if not choices:
        raise RuntimeError(f"VLM returned no choices: {body!r}")
    return choices[0]["message"]["content"]


class VLMDetector(HeadCameraProjector):
    """Head-camera + VLM open-vocabulary detector. Resolves a natural-language
    instruction to base-frame grasp points via the shared calibration."""

    def __init__(self, *, arm, camera, calib_path: str,
                 base_url: str, api_key: str, model: str,
                 urdf_path: str = "", grasp_height: float = BLOCK_SIZE / 2.0,
                 timeout: float = 60.0) -> None:
        """Load the hand-eye calibration and record the VLM endpoint.

        Args:
            arm, camera: primitive handles (see ``HeadCameraProjector``).
            calib_path:  ``handeye_calib.npz`` (shared with the YOLO path).
            base_url:    OpenAI-compatible base URL (``/chat/completions`` is
                         appended if missing).
            api_key:     bearer token for the endpoint.
            model:       model id (must accept image input).
            urdf_path:   robot URDF; empty for the SDK default.
            grasp_height: grasp point height above the table plane (m). No depth
                         is available, so this fixed value sets both the Z and
                         the perspective correction. Defaults to a 5 cm cube's
                         centre (2.5 cm).
            timeout:     VLM HTTP timeout (seconds).
        """
        super().__init__(arm=arm, camera=camera, calib_path=calib_path, urdf_path=urdf_path)
        if not base_url:
            raise ValueError("VLM base_url is empty — set config vlm_base_url or env VLM_BASE_URL")
        if not model:
            raise ValueError("VLM model is empty — set config vlm_model or env VLM_MODEL")
        self._base_url = base_url
        self._api_key = api_key
        self._model = model
        self._grasp_height = float(grasp_height)
        self._timeout = float(timeout)
        print(f"[vlm] detector ready (model={model})", flush=True)

    def _ask_vlm(self, cap, instruction: str) -> List[Dict[str, object]]:
        """Encode the captured frame, ask the VLM for the instructed target(s),
        and resolve each oriented box to a base-frame grasp point.

        Shared by :meth:`detect` and :meth:`verify` so both see the same
        detections. ``cap`` is a frame/geometry from ``_capture``. Returns the
        object dicts (possibly empty). Raises on VLM / parse failure.
        """
        h_img, w_img = cap.rgb.shape[0], cap.rgb.shape[1]

        # Encode as JPEG for the VLM. The frame is RGB; cv2.imencode expects BGR,
        # so convert first or the model sees swapped R/B channels (wrong colours).
        frame_bgr = cv2.cvtColor(cap.rgb, cv2.COLOR_RGB2BGR)
        ok, buf = cv2.imencode(".jpg", frame_bgr)
        if not ok:
            raise RuntimeError("failed to JPEG-encode the camera frame")
        image_b64 = base64.b64encode(buf.tobytes()).decode("utf-8")

        prompt = _DETECT_PROMPT.replace("{instruction}", instruction)
        reply = _vlm_chat(self._base_url, self._api_key, self._model,
                          prompt, image_b64, self._timeout)

        try:
            data = json.loads(_extract_json(reply))
        except json.JSONDecodeError as exc:
            raise RuntimeError(f"VLM reply was not valid JSON: {exc}; raw={reply!r}") from exc

        if data.get("failed"):
            return []
        raw_objs = data.get("objects") or []

        objects: List[Dict[str, object]] = []
        for obj in raw_objs:
            try:
                bcx = float(obj["box_center_x"])
                bcy = float(obj["box_center_y"])
                bw = float(obj["box_width"])
                bh = float(obj["box_height"])
            except (KeyError, TypeError, ValueError):
                continue  # skip a malformed entry rather than fail the whole call
            # Reject out-of-range normalised boxes (hallucinated / bad output).
            if not (0.0 <= bcx <= 1.0 and 0.0 <= bcy <= 1.0
                    and 0.0 < bw <= 1.0 and 0.0 < bh <= 1.0):
                continue
            # Rotation is optional; clamp to the prompt's [-90, 90] convention.
            try:
                rot = float(obj.get("box_rotation_deg", 0.0))
            except (TypeError, ValueError):
                rot = 0.0
            rot = float(np.clip(rot, -90.0, 90.0))

            # Normalised box -> live-resolution pixels.
            u_px = bcx * w_img
            v_px = bcy * h_img
            w_px = bw * w_img
            h_px = bh * h_img

            wx, wy, z_grasp = self._project(cap, u_px, v_px, self._grasp_height)
            # Oriented box: grasp perpendicular to the object's long edge. The
            # VLM's rotation is fed straight to estimate_grasp_angle_deg (same
            # OpenCV-convention pixel->yaw mapping as the YOLO path).
            grasp_angle = estimate_grasp_angle_deg(u_px, v_px, w_px, h_px, rot)

            objects.append({
                "class_name": str(obj.get("class_name", "object")),
                "score": round(float(obj.get("score", 1.0)), 3),
                "x": round(wx, 3),
                "y": round(wy, 3),
                "z": round(z_grasp, 3),
                "grasp_angle_deg": round(float(grasp_angle), 1),
            })

        return objects

    def detect(self, instruction: str) -> List[Dict[str, object]]:
        """Aim the head, grab a frame, ask the VLM to locate the instructed
        target(s), and return them as base-frame grasp points.

        Each object is ``{class_name, score, x, y, z, grasp_angle_deg}`` in the
        base frame (metres / degrees) — the same schema as ``detect_cubes`` — so
        the result feeds straight into ``pick_cube``. ``grasp_angle_deg`` comes
        from the object's VLM-reported orientation, so tilted/elongated objects
        get a usable grasp yaw. Returns an empty list when the VLM reports no
        matching target. Raises on VLM / parse failure.
        """
        instr = (instruction or "").strip()
        if not instr:
            raise ValueError("empty instruction")
        cap = self._capture()
        return self._ask_vlm(cap, instr)

    def verify(self, instruction: str, mode: str, position, *,
               radius: float) -> Dict[str, object]:
        """Visually confirm a pick / place by re-detecting the target and
        checking its presence near ``position`` (base-frame (x, y), metres).

        Args:
            instruction: description of the object to look for.
            mode:        ``"pick"`` — success iff the object is now ABSENT near
                         ``position`` (it was lifted away); ``"place"`` — success
                         iff the object is now PRESENT near ``position``.
            position:    ``(x, y)`` the object was picked from / placed at.
            radius:      match radius (m); a detection within it counts as "at"
                         the position.

        Returns ``{success, message, nearest, detected}`` — ``success`` is the
        pick/place verdict, ``nearest`` is the closest detection's distance (m)
        or None, ``detected`` is the count of matching detections. Raises on
        VLM / parse failure or an unknown ``mode``.
        """
        m = (mode or "").strip().lower()
        if m not in ("pick", "place"):
            raise ValueError(f"mode must be 'pick' or 'place', got {mode!r}")
        instr = (instruction or "").strip()
        if not instr:
            raise ValueError("empty instruction")
        px, py = float(position[0]), float(position[1])

        cap = self._capture()
        objs = self._ask_vlm(cap, instr)

        # Nearest detection to the queried position.
        nearest = None
        for o in objs:
            d = float(np.hypot(float(o["x"]) - px, float(o["y"]) - py))
            if nearest is None or d < nearest:
                nearest = d
        present = nearest is not None and nearest <= radius

        if m == "pick":
            success = not present  # gone from the table => lifted
            msg = (f"object no longer at ({px:.3f},{py:.3f}) — pick confirmed"
                   if success else
                   f"object still at ({px:.3f},{py:.3f}) (nearest {nearest:.3f}m) — pick likely failed")
        else:  # place
            success = present
            msg = (f"object present at ({px:.3f},{py:.3f}) (nearest {nearest:.3f}m) — place confirmed"
                   if success else
                   f"no object at ({px:.3f},{py:.3f}) — place likely failed")

        return {
            "success": bool(success),
            "message": msg,
            "nearest": (round(nearest, 3) if nearest is not None else None),
            "detected": len(objs),
        }
