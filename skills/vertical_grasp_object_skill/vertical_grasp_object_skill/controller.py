#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Location-based pick / place controller for the D1 arm + dexterous hand.

Two independent moves the skill exposes to the LLM:

  * ``pick(x, y, z)``  — go to the location, grasp a cube there, lift and hold.
  * ``place(x, y, z)`` — go to the location, release the held cube, lift.

There is **no vision**: no camera, no YOLO, no hand-eye calibration and no head
motion. The caller supplies where to pick and where to place; the end-effector
pose depends only on the 6 arm joints (head and arm are separate branches off
``link_base``), so the head is never touched.

It reuses the pure-compute stack the D1 grasp pipeline provides — the SLSQP
solver in ``block_grasp.ik_scipy`` and the motion/hand tunables in
``block_grasp.config`` (VENDORED into this package; see ``../block_grasp/``) plus
``D1Kinematics`` FK/IK from the ``beingbeyond_d1_sdk`` wheel — and drives the
hardware through injected primitive handles (``robot``, ``hand``).
"""
from __future__ import annotations

import math
import time
from typing import Dict, Tuple

import numpy as np
from scipy.spatial.transform import Rotation as R

from beingbeyond_d1_sdk.pin_kinematics import D1Kinematics, D1KinematicsConfig
from beingbeyond_d1_sdk.urdf_path import get_default_urdf_path

from block_grasp.ik_scipy import scipy_ik, scipy_ik_multi_restart
from block_grasp.config import (
    APPROACH_HEIGHT_OFFSET,
    CATCH_DELAY_S,
    EE_PITCH_DEG,
    EE_ROLL_DEG,
    EE_YAW_DEG,
    GRASP_OK_MAX,
    GRASP_OK_MIN,
    GRAVITY_SAG_FACTOR,
    HAND_CLOSE,
    HAND_GRASP,
    HAND_OPEN,
    IK_FAIL_THRESHOLD,
    IK_MAX_ITERS,
    IK_N_RESTARTS,
    IK_POS_TOL,
    IK_TILT_TOL_DEG,
    IK_YAW_TOL_DEG,
    IK_Z_WEIGHT,
    INTERP_STEP_SIZE,
    JOINT_JUMP_THR_DEG,
    Z_SAFE,
)

# Skill-local park / home pose (base frame, metres). move_home parks the arm
# here; _startup uses its (x, y) as the resting posture position. Kept local to
# this skill rather than reusing block_grasp's shared ASIDE_POSITION.
HOME_POSITION = (0.12, -0.23, 0.32)


class GraspCubeController:
    """Grasp a cube at a location, place it at another. Synchronous, headless,
    no vision. Hardware is driven through injected primitive handles."""

    def __init__(self, *, robot, hand, urdf_path: str = "", grasp_feedback: bool = True) -> None:
        """Set up kinematics and move the arm to a safe, table-perpendicular
        posture so the first approach has a well-conditioned IK start.

        Args:
            robot: Injected arm handle (HeadArmRobot-like) — the arm primitive.
            hand:  Injected hand handle (DexHand-like) — the hand primitive.
            urdf_path: Robot URDF; empty for the SDK default.
            grasp_feedback: When True, pick() judges grasp success from the hand's
                finger angles; when False, pick() assumes the grasp succeeded once
                the hand closes (use if the finger-angle feedback misfires and
                reports a false "grasp failed" on a cube that was actually held).
        """
        self._robot = robot
        self._hand = hand
        self._grasp_feedback = bool(grasp_feedback)
        self._holding = False  # True after a successful pick, until place

        if not urdf_path:
            urdf_path = get_default_urdf_path()

        # Target EE orientation: roll+pitch keep the hand perpendicular to the
        # table, yaw is the default. No per-object rotation (no vision).
        self._R_target = R.from_euler(
            "xyz", [EE_ROLL_DEG, EE_PITCH_DEG, EE_YAW_DEG], degrees=True
        ).as_matrix()

        print("[Init] Setting up kinematics ...", flush=True)
        self._kin = D1Kinematics(D1KinematicsConfig(urdf_path=urdf_path))

        self._startup()
        print("[Init] Ready.", flush=True)

    # ── Startup posture ─────────────────────────────────────────────────────

    def _startup(self) -> None:
        """Safe posture → lift to safe Z → rotate EE to target RPY → open hand,
        so IK starts from a good configuration instead of the folded home pose
        (no head-to-camera step — this skill has no camera)."""
        print("[Init] Moving to safe posture ...", flush=True)
        q_init = np.radians([0, 0, 0, -60, 60, 0, 0, 0])
        self._robot.set_positions(q_init)
        self._robot.wait_until_reached(q_init, active_joint_indices=range(8))
        time.sleep(0.3)

        print("[Init] Lifting to home posture ...", flush=True)
        q_full = np.asarray(self._robot.get_positions(), dtype=float)
        q_head, q_arm = self._kin.split_q(q_full)
        T_cur = self._kin.ee_in_base(q_head, q_arm)
        # Rest at the full HOME_POSITION so the initial gripper posture is
        # identical to move_home (same x, y, z; the RPY rotation below then
        # matches move_home's R_target orientation).
        p_lift = T_cur[:3, 3].copy()
        p_lift[0] = HOME_POSITION[0]
        p_lift[1] = HOME_POSITION[1]
        p_lift[2] = HOME_POSITION[2]
        T_lift = np.eye(4)
        T_lift[:3, :3] = T_cur[:3, :3]
        T_lift[:3, 3] = p_lift
        q_hs, q_as, err, _ = self._kin.ik_T_ee_with_arm_only(T_lift, q_head, q_arm)
        if err < 0.05:
            cmd = np.concatenate([q_hs, q_as])
            self._robot.set_positions(cmd)
            self._robot.wait_until_reached(cmd, active_joint_indices=range(2, 8))
            q_head, q_arm = self._kin.split_q(cmd)
        else:
            print(f"  ⚠ Lift IK error: {err:.3f}", flush=True)

        print("[Init] Rotating to target RPY ...", flush=True)
        T_cur = self._kin.ee_in_base(q_head, q_arm)
        p_cur = T_cur[:3, 3]
        q0 = R.from_matrix(T_cur[:3, :3]).as_quat()
        q1 = R.from_matrix(self._R_target).as_quat()
        if np.dot(q0, q1) < 0:  # shortest path (quaternion double-cover)
            q1 = -q1
        omega = float(np.arccos(np.clip(np.dot(q0, q1), -1.0, 1.0)))
        n_rot = max(1, int(math.ceil(omega * 2 / 0.05)))
        for i in range(n_rot):
            a = (i + 1) / n_rot
            if abs(omega) < 1e-10:
                qi = q0
            else:
                qi = (np.sin((1 - a) * omega) * q0 + np.sin(a * omega) * q1) / np.sin(omega)
            T_rt = np.eye(4)
            T_rt[:3, :3] = R.from_quat(qi).as_matrix()
            T_rt[:3, 3] = p_cur
            q_hs, q_as, err, _ = self._kin.ik_T_ee_with_arm_only(T_rt, q_head, q_arm)
            if err < 0.05:
                cmd = np.concatenate([q_hs, q_as])
                self._robot.set_positions(cmd)
                time.sleep(0.02)
                q_head, q_arm = self._kin.split_q(cmd)
            else:
                print(f"  ⚠ Rot IK err={err:.3f} at step {i + 1}/{n_rot}", flush=True)

        print("[Init] Opening hand ...", flush=True)
        self._hand.set_joint_pos(HAND_OPEN)
        time.sleep(0.3)

    # ── Motion helpers ──────────────────────────────────────────────────────

    def _get_joint_state(self) -> Tuple[np.ndarray, np.ndarray]:
        q_full = np.asarray(self._robot.get_positions(), dtype=float)
        return self._kin.split_q(q_full)

    def _z_sag(self, x: float, y: float) -> float:
        """Gravity-sag Z compensation: the arm droops the further it reaches,
        so raise the target Z to counteract it (cubic model, dz ∝ r³)."""
        dist = math.sqrt(x * x + y * y)
        return GRAVITY_SAG_FACTOR * dist ** 3

    @staticmethod
    def _j6_offset_rad(angle_deg: float | None) -> float:
        """Grasp/approach angle (degrees) → joint_6 (wrist roll) offset in
        radians. ``None`` → 0 (default orientation). The angle is applied to the
        wrist *after* IK (see ``_interpolate_and_move``), so it rotates the hand
        about the vertical without disturbing the solved EE position — matching
        the D1 block_grasp stack.

        Wrapped to [-90°, +90°] first: a two-finger gripper is 180°-symmetric, so
        grasping at θ and θ±180° is physically identical — folding to the
        half-turn keeps the wrist within range and picks the shorter roll (e.g. a
        requested 170° becomes -10°). This is the gripper's own symmetry and
        holds for ANY object. Object-shape symmetries (a cube's extra 90°, so 30°
        ≡ 60° ≡ -30°) are NOT applied here — the skill can't assume every target
        is square — so the caller may still pass a shape-equivalent angle; see the
        pick_cube docstring."""
        if angle_deg is None:
            return 0.0
        a = ((float(angle_deg) + 90.0) % 180.0) - 90.0   # wrap to [-90, 90)
        return math.radians(a)

    def _hand_vector(self, fraction: float) -> np.ndarray:
        """Hand joint vector for a given open/close amount, piecewise-interpolated
        across the three calibrated D1 poses (all 6-D normalised, 0=open..1=closed,
        thumb kept opposed):

            0.0 → HAND_OPEN   (fully open)
            0.5 → HAND_GRASP  (the tuned ~4.5 cm grasp — the default)
            1.0 → HAND_CLOSE  (tightest, for smaller objects)

        ``fraction`` is clamped to [0, 1]; 0.0 / 0.5 / 1.0 reproduce the anchor
        poses exactly. The two halves are interpolated separately so HAND_GRASP
        stays reachable rather than being averaged away."""
        f = float(np.clip(fraction, 0.0, 1.0))
        open_v = np.asarray(HAND_OPEN, dtype=float)
        grasp_v = np.asarray(HAND_GRASP, dtype=float)
        close_v = np.asarray(HAND_CLOSE, dtype=float)
        if f <= 0.5:
            return open_v + (f / 0.5) * (grasp_v - open_v)
        return grasp_v + ((f - 0.5) / 0.5) * (close_v - grasp_v)

    def _make_target_pose(self, x: float, y: float, z: float) -> np.ndarray:
        T = np.eye(4)
        T[:3, :3] = self._R_target
        T[:3, 3] = [x, y, z]
        return T

    def _interpolate_and_move(
        self,
        T_target: np.ndarray,
        step_size: float = INTERP_STEP_SIZE,
        z_weight: float = IK_Z_WEIGHT,
        j6_offset_rad: float = 0.0,
    ) -> bool:
        """Move EE to target pose using SLSQP IK with linear interpolation.

        ``j6_offset_rad`` is added to joint_6 after each IK step so the wrist
        rotates to the requested grasp angle without affecting position accuracy.
        """
        q_head, q_arm = self._get_joint_state()
        T_cur = self._kin.ee_in_base(q_head, q_arm)
        p_start = T_cur[:3, 3].copy()
        # Work in the uncompensated (nominal) frame: strip the sag already baked
        # into the current pose so the per-step sag below never double-counts it.
        p_start[2] -= self._z_sag(p_start[0], p_start[1])
        p_target = T_target[:3, 3]
        R_target = T_target[:3, :3]

        dist = float(np.linalg.norm(p_target - p_start))
        n_steps = max(1, int(dist / step_size))

        cmd = None
        for i in range(n_steps):
            alpha = (i + 1) / n_steps
            interp = p_start + alpha * (p_target - p_start)
            interp[2] += self._z_sag(interp[0], interp[1])   # gravity-sag comp
            T_step = np.eye(4)
            T_step[:3, :3] = R_target
            T_step[:3, 3] = interp
            try:
                q_hs, q_as, err, _ = scipy_ik(
                    self._kin, T_step, q_head, q_arm,
                    z_weight=z_weight, pos_tol=IK_POS_TOL,
                    tilt_tol_deg=IK_TILT_TOL_DEG, yaw_tol_deg=IK_YAW_TOL_DEG,
                    max_iters=IK_MAX_ITERS,
                )
                if np.isnan(err) or err > IK_FAIL_THRESHOLD:
                    print(f"  [IK] SLSQP fail at step {i}/{n_steps}: err={err:.3f}", flush=True)
                    return False
                # Reject near-singularity joint jumps (excl. j6 wrist roll).
                dq_max = float(np.max(np.abs(q_as[:5] - q_arm[:5])))
                if dq_max > math.radians(JOINT_JUMP_THR_DEG):
                    print(f"  [IK] joint jump {math.degrees(dq_max):.0f}° at step "
                          f"{i}/{n_steps} (near singularity), aborting.", flush=True)
                    return False
                # Apply the grasp angle directly to joint_6 (wrist roll),
                # normalised to the shortest path from the current joint_6.
                q_as[5] += j6_offset_rad
                diff = q_as[5] - q_arm[5]
                q_as[5] = q_arm[5] + (diff + math.pi) % (2 * math.pi) - math.pi
                cmd = np.concatenate([q_hs, q_as])
                self._robot.set_positions(cmd)
                time.sleep(0.02)
                q_head, q_arm = self._kin.split_q(cmd)
            except Exception as e:  # noqa: BLE001
                print(f"  [IK] Step {i} error: {e}", flush=True)
                return False

        # Wait for the arm to physically settle at the final pose so the caller
        # (descend / grasp) doesn't read a lagging, far-from-target position.
        if cmd is not None:
            self._robot.wait_until_reached(cmd, active_joint_indices=range(2, 8))
        return True

    def _refine_and_move(
        self, T_target: np.ndarray, z_weight: float = IK_Z_WEIGHT, j6_offset_rad: float = 0.0
    ) -> float:
        """Fine-positioning with SLSQP multi-restart IK. Returns the final
        position error (metres). ``j6_offset_rad`` is added to joint_6 after IK
        (wrist roll for the grasp angle)."""
        q_head, q_arm = self._get_joint_state()
        T_target = T_target.copy()
        T_target[2, 3] += self._z_sag(T_target[0, 3], T_target[1, 3])
        try:
            q_hs, q_as, best_err, _ = scipy_ik_multi_restart(
                self._kin, T_target, q_head, q_arm,
                n_restarts=IK_N_RESTARTS, z_weight=z_weight,
                pos_tol=IK_POS_TOL, tilt_tol_deg=IK_TILT_TOL_DEG,
                yaw_tol_deg=IK_YAW_TOL_DEG,
            )
            # Reject a distant IK branch: the arm is already at target from the
            # interpolation phase, so keep it put and report the pose it holds.
            dq_max = float(np.max(np.abs(q_as[:5] - q_arm[:5])))
            if dq_max > math.radians(JOINT_JUMP_THR_DEG):
                T_cur = self._kin.ee_in_base(q_head, q_arm)
                cur_err = float(np.linalg.norm(T_cur[:3, 3] - T_target[:3, 3]))
                print(f"  [IK] refine joint jump {math.degrees(dq_max):.0f}° "
                      f"(near singularity), keeping current pose (err={cur_err:.4f}).",
                      flush=True)
                return cur_err
            # Apply the grasp angle directly to joint_6 (wrist roll),
            # normalised to the shortest path from the current joint_6.
            q_as[5] += j6_offset_rad
            diff = q_as[5] - q_arm[5]
            q_as[5] = q_arm[5] + (diff + math.pi) % (2 * math.pi) - math.pi
            cmd = np.concatenate([q_hs, q_as])
            self._robot.set_positions(cmd)
            self._robot.wait_until_reached(cmd, active_joint_indices=range(2, 8))
            return float(best_err)
        except Exception as e:  # noqa: BLE001
            print(f"  [IK] SLSQP refine error: {e}", flush=True)
            return 999.0

    def _move_to_pose(
        self, T_target: np.ndarray, refine: bool = True, j6_offset_rad: float = 0.0
    ) -> bool:
        """Move EE to target pose: SLSQP interpolation + multi-restart refinement.
        ``j6_offset_rad`` rotates the wrist (joint_6) to the requested grasp angle."""
        if not self._interpolate_and_move(T_target, j6_offset_rad=j6_offset_rad):
            return False
        if refine:
            err = self._refine_and_move(T_target, j6_offset_rad=j6_offset_rad)
            p = T_target[:3, 3]
            print(f"  → ({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f})  err={err:.4f}", flush=True)
            return err < 0.05
        return True

    # ── Public API (called by the skill's MCP handlers) ─────────────────────

    def pick(
        self,
        x: float,
        y: float,
        z: float,
        angle_deg: float | None = None,
        gripper: float | None = None,
    ) -> Dict[str, object]:
        """Go to (x, y, z), grasp an object there, lift and hold.

        Sequence: open → approach above → descend → close → lift → check.
        ``angle_deg`` rotates the wrist (joint_6) to align the hand with the
        object (None → default orientation); ``gripper`` sets how tightly the
        hand closes, 0.0 (open) – 0.5 (standard grasp) – 1.0 (tightest) (None →
        standard grasp, i.e. the original behaviour).
        Returns a JSON-serialisable dict; sets the holding flag on success.
        """
        z_approach = z + APPROACH_HEIGHT_OFFSET
        loc = [round(x, 3), round(y, 3), round(z, 3)]
        j6 = self._j6_offset_rad(angle_deg)
        grasp_vec = self._hand_vector(0.5 if gripper is None else gripper)

        print("[Pick] Opening hand ...", flush=True)
        self._hand.set_joint_pos(HAND_OPEN)
        time.sleep(CATCH_DELAY_S)

        print(f"[Pick] Approaching above ({x:.3f}, {y:.3f}, {z_approach:.3f}) "
              f"j6={math.degrees(j6):.0f}°", flush=True)
        T_approach = self._make_target_pose(x, y, z_approach)
        if not self._interpolate_and_move(T_approach, j6_offset_rad=j6):
            if self._refine_and_move(T_approach, j6_offset_rad=j6) > 0.05:
                return {"ok": False, "location": loc, "message": "approach failed (IK)"}
        time.sleep(CATCH_DELAY_S)

        print(f"[Pick] Descending to ({x:.3f}, {y:.3f}, {z:.3f})", flush=True)
        if not self._move_to_pose(self._make_target_pose(x, y, z), refine=True, j6_offset_rad=j6):
            return {"ok": False, "location": loc, "message": "descent failed (IK)"}
        time.sleep(CATCH_DELAY_S)

        print("[Pick] Closing hand ...", flush=True)
        self._hand.set_joint_pos(grasp_vec)
        time.sleep(CATCH_DELAY_S)

        print(f"[Pick] Lifting to safe height Z={Z_SAFE:.2f} ...", flush=True)
        T_lift = self._make_target_pose(x, y, max(z_approach, Z_SAFE))
        if not self._interpolate_and_move(T_lift, j6_offset_rad=j6):
            print("[Pick] Lift failed (object may still be grasped).", flush=True)
        time.sleep(CATCH_DELAY_S)

        # Grasp success check. When the finger-angle feedback is disabled, assume
        # the grasp held once the hand closed (avoids a false "grasp failed" that
        # would make the caller open the hand and drop a cube it was really
        # holding); otherwise judge from the average of the 4 main fingers.
        if not self._grasp_feedback:
            self._holding = True
            print("[Pick] OK (grasp feedback disabled — assumed held)", flush=True)
            return {"ok": True, "location": loc, "message": "object grasped (feedback disabled)"}

        current_pos = self._hand.read_joint_pos()
        active = [current_pos[i] for i in [0, 2, 3, 4]]  # thumb, idx, mid, ring
        avg_pos = sum(active) / len(active)
        grasp_ok = GRASP_OK_MIN <= avg_pos <= GRASP_OK_MAX
        self._holding = bool(grasp_ok)
        print(f"[Pick] {'OK' if grasp_ok else 'FAILED'} "
              f"(avg 4-finger pos={avg_pos:.2f}, expected [{GRASP_OK_MIN:.1f}–{GRASP_OK_MAX:.1f}])",
              flush=True)
        return {"ok": bool(grasp_ok), "location": loc,
                "message": "object grasped" if grasp_ok else "grasp failed (no object / slipped)"}

    def place(
        self,
        x: float,
        y: float,
        z: float,
        angle_deg: float | None = None,
        gripper: float | None = None,
    ) -> Dict[str, object]:
        """Go to (x, y, z), release the held object, lift.

        Sequence: approach above → descend → open → lift. ``angle_deg`` rotates
        the wrist (joint_6) for the approach (None → default orientation);
        ``gripper`` sets the release aperture on the same 0.0 (fully open) – 0.5
        (standard grasp) – 1.0 (tightest) scale (None → fully open, i.e. the
        original behaviour — higher values keep the hand more closed and may not
        release). Clears the holding flag. Warns (but proceeds) if no successful
        pick preceded this.
        """
        # Travel to above the target at the safe height (the arm is holding the
        # object up at Z_SAFE after pick), so the horizontal move happens high and
        # the descent to the release point is purely vertical — reach the target
        # (x, y) first, then drop straight down.
        z_approach = max(z + APPROACH_HEIGHT_OFFSET, Z_SAFE)
        loc = [round(x, 3), round(y, 3), round(z, 3)]
        j6 = self._j6_offset_rad(angle_deg)
        release_vec = self._hand_vector(0.0 if gripper is None else gripper)
        if not self._holding:
            print("[Place] Warning: no object recorded as held; placing anyway.", flush=True)

        print(f"[Place] Approaching ({x:.3f}, {y:.3f}, {z_approach:.3f}) "
              f"j6={math.degrees(j6):.0f}°", flush=True)
        T_approach = self._make_target_pose(x, y, z_approach)
        if not self._interpolate_and_move(T_approach, j6_offset_rad=j6):
            return {"ok": False, "location": loc, "message": "approach failed (IK)"}
        time.sleep(CATCH_DELAY_S)

        print(f"[Place] Descending to ({x:.3f}, {y:.3f}, {z:.3f})", flush=True)
        if not self._move_to_pose(self._make_target_pose(x, y, z), refine=True, j6_offset_rad=j6):
            return {"ok": False, "location": loc, "message": "descent failed (IK)"}
        time.sleep(CATCH_DELAY_S)

        print("[Place] Opening hand ...", flush=True)
        self._hand.set_joint_pos(release_vec)
        time.sleep(CATCH_DELAY_S)
        self._holding = False

        if not self._interpolate_and_move(T_approach, j6_offset_rad=j6):
            return {"ok": False, "location": loc, "message": "released but lift failed"}
        return {"ok": True, "location": loc, "message": "object placed"}

    def move_home(self) -> None:
        """Open the hand and park the arm at a safe pose (clears the workspace)."""
        self._hand.set_joint_pos(HAND_OPEN)
        self._holding = False
        time.sleep(CATCH_DELAY_S)
        ax, ay, az = HOME_POSITION
        print(f"[Home] Parking at ({ax:.3f}, {ay:.3f}, {az:.3f})", flush=True)
        try:
            self._interpolate_and_move(self._make_target_pose(ax, ay, az))
        except Exception as e:  # noqa: BLE001
            print(f"[Home] Failed: {e}", flush=True)

    def shutdown(self) -> None:
        """Safe release on the way out; hardware is owned by the primitives."""
        print("[Shutdown] Opening hand ...", flush=True)
        try:
            self._hand.set_joint_pos(HAND_OPEN)
            time.sleep(0.3)
        except Exception:  # noqa: BLE001
            pass
        for closer in (getattr(self._hand, "close_can", None),
                       getattr(self._robot, "close", None)):
            try:
                if closer is not None:
                    closer()
            except Exception:  # noqa: BLE001
                pass
        print("[Shutdown] Done.", flush=True)
