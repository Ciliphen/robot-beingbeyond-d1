# SPDX-License-Identifier: MulanPSL-2.0
"""YOLO-OBB cube detector for the grasp_cube skill (perception, report-only).

Aims the head camera at the table (the hand-eye calibration pose), grabs one RGB
frame, runs YOLO-OBB, and resolves each detected cube to a base-frame (x, y, z)
via the calibration homography + perspective correction. Returns a plain list of
dicts for the ``detect_cubes`` MCP tool. It drives no motion beyond aiming the
head — picking/placing stays in ``controller.py``.

The maths mirror ``block_grasp/grasp_controller.py::detect_blocks`` so detected
positions line up with what the pick pipeline expects. The pure-compute
perception stack (YOLO loader/inference, OBB geometry helpers, tunables) is
reused from the Beingbeyond_D1 repo, imported via ``BEINGBEYOND_PATH``.
"""
from __future__ import annotations

import time
from typing import Dict, List

import cv2
import numpy as np

from beingbeyond_d1_sdk.pin_kinematics import D1Kinematics, D1KinematicsConfig
from beingbeyond_d1_sdk.urdf_path import get_default_urdf_path

from object_detect.detect import detect_objects_in_frame, load_model
from block_grasp.coordinate_utils import (
    estimate_grasp_angle_deg,
    obb_bottom_center,
    pixel_to_world_2d,
)
from block_grasp.config import (
    BLOCK_SIZE,
    CALIB_CAM_HEIGHT,
    CALIB_CAM_WIDTH,
    CONF_THRESHOLD,
    IOU_THRESHOLD,
    MAX_DXY,
    OBB_GRASP_RATIO,
)


class CubeDetector:
    """Head-camera + YOLO cube detector. Resolves detections to base-frame XY."""

    def __init__(self, *, arm, camera, model_path: str, calib_path: str,
                 urdf_path: str = "") -> None:
        """Load the hand-eye calibration and the YOLO model.

        Args:
            arm:        Arm primitive handle (needs ``set_head`` + ``get_positions``).
            camera:     Camera primitive handle (``get_aligned_frames`` -> RGB).
            model_path: Path to the YOLO-OBB ``.pt`` checkpoint.
            calib_path: Path to ``handeye_calib.npz`` (H, head pose, table points).
            urdf_path:  Robot URDF; empty for the SDK default.
        """
        self._arm = arm
        self._camera = camera

        if not urdf_path:
            urdf_path = get_default_urdf_path()
        self._kin = D1Kinematics(D1KinematicsConfig(urdf_path=urdf_path))

        calib = np.load(calib_path, allow_pickle=True)
        self._H: np.ndarray = calib["H"]                       # 3x3 homography
        self._head_yaw = float(calib["head_yaw"])
        self._head_pitch = float(calib["head_pitch"])
        if "world_pts" in calib:
            self._W = calib["world_pts"]                       # (N, 3) table pts
            self._z_table = float(np.median(self._W[:, 2]))
        else:
            self._W = None
            self._z_table = 0.08

        print(f"[detect] loading YOLO model {model_path} ...", flush=True)
        self._model = load_model(model_path)
        print("[detect] detector ready", flush=True)

    def _get_table_z(self, x: float, y: float) -> float:
        """Interpolate table Z at (x, y) — inverse-distance-weighted average of
        the 3 nearest calibration points (handles a slightly tilted table)."""
        if self._W is None or len(self._W) < 3:
            return self._z_table
        dists = np.sqrt((self._W[:, 0] - x) ** 2 + (self._W[:, 1] - y) ** 2)
        idx = np.argsort(dists)[:3]
        if dists[idx[0]] < 1e-6:
            return float(self._W[idx[0], 2])
        wgt = 1.0 / (dists[idx] + 0.001)
        wgt /= wgt.sum()
        return float(np.dot(wgt, self._W[idx, 2]))

    def detect(self) -> List[Dict[str, object]]:
        """Aim the head, grab a frame, run YOLO, and return the detected cubes.

        Each cube is ``{class_name, score, x, y, z, grasp_angle_deg}`` in the
        base frame (metres / degrees), sorted by score descending. ``x, y, z``
        are the grasp point (z = cube centre, ~2.5 cm above the table) — feed
        them straight to ``pick_cube``.
        """
        # 1. Aim the head camera at the table (the pose the homography was
        #    calibrated at) and let it settle before grabbing a frame.
        self._arm.set_head(self._head_yaw, self._head_pitch)
        time.sleep(0.5)

        # 2. One RGB frame from the head camera.
        rgb, _ = self._camera.get_aligned_frames(filtered=False)

        # 3. Camera + EE pose from FK (for perspective correction / clamp).
        q_full = np.asarray(self._arm.get_positions(), dtype=float)
        q_head, q_arm = self._kin.split_q(q_full)
        T_base_cam = self._kin.camera_in_base(q_head, q_arm)
        cx, cy, cz = T_base_cam[:3, 3]
        T_ee = self._kin.ee_in_base(q_head, q_arm)
        ws_x0, ws_y0 = float(T_ee[0, 3]), float(T_ee[1, 3])

        # 4. YOLO OBB (model trained on BGR; camera gives RGB).
        frame_bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        detections = detect_objects_in_frame(
            self._model, frame_bgr, conf_thres=CONF_THRESHOLD, iou_thres=IOU_THRESHOLD
        )

        # Scale pixels from the camera resolution to the calibration resolution.
        sx = CALIB_CAM_WIDTH / max(rgb.shape[1], 1)
        sy = CALIB_CAM_HEIGHT / max(rgb.shape[0], 1)

        cubes: List[Dict[str, object]] = []
        for (u, v, w, h, r), score, _cls_id, cls_name in detections:
            # OBB bottom-centre pixel -> table-plane XY (homography, first pass).
            u_bot, v_bot = obb_bottom_center(u, v, w, h, np.rad2deg(r), ratio=OBB_GRASP_RATIO)
            wx_hom, wy_hom = pixel_to_world_2d(u_bot * sx, v_bot * sy, self._H)

            # Perspective correction for the cube sitting above the table plane.
            z_tbl = self._get_table_z(wx_hom, wy_hom)
            z_obj = z_tbl + BLOCK_SIZE / 2.0
            denom = z_tbl - cz
            t_corr = (z_obj - cz) / denom if abs(denom) > 0.001 else 1.0
            wx = cx + (wx_hom - cx) * t_corr
            wy = cy + (wy_hom - cy) * t_corr

            # Clamp to the reachable workspace around the current EE.
            wx = float(np.clip(wx, ws_x0 - MAX_DXY, ws_x0 + MAX_DXY))
            wy = float(np.clip(wy, ws_y0 - MAX_DXY, ws_y0 + MAX_DXY))
            # Grasp height = cube centre (~2.5 cm above the table for a 5 cm
            # cube), NOT the top face — pick_cube descends straight to this z,
            # so reporting the top face would leave the hand grabbing above the
            # cube. z_tbl comes from the hand-eye calibration table plane.
            z_grasp = z_tbl + BLOCK_SIZE / 2.0

            grasp_angle = estimate_grasp_angle_deg(u, v, w, h, np.rad2deg(r))
            cubes.append({
                "class_name": cls_name,
                "score": round(float(score), 3),
                "x": round(wx, 3),
                "y": round(wy, 3),
                "z": round(float(z_grasp), 3),
                "grasp_angle_deg": round(float(grasp_angle), 1),
            })

        cubes.sort(key=lambda c: c["score"], reverse=True)
        return cubes
