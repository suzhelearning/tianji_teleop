"""Adaptive optimizer with analytical gradients for hand retargeting."""

from __future__ import annotations

import time
from pathlib import Path
from typing import Optional

import numpy as np

from .base import BaseOptimizer, M_TO_CM, TimingStats
from .._native import AdaptiveGeometry


class AdaptiveOptimizerAnalytical(BaseOptimizer):
    """Adaptive optimizer with analytical (hand-written) gradients.

    Same loss function as AdaptiveOptimizer but uses hand-written gradients
    instead of autograd for faster performance.
    """

    def __init__(self, config: dict):
        """Initialize AdaptiveOptimizerAnalytical."""
        super().__init__(config)

        # Initialize timing stats
        self._timing = TimingStats()
        self._enable_timing = True

        retarget_config = config.get('retarget', {})

        # TipDirVec parameters
        self.huber_delta_dir = retarget_config.get('huber_delta_dir', 0.5)
        self.w_pos = retarget_config.get('w_pos', 1.0)
        self.w_dir = retarget_config.get('w_dir', 10.0)
        self.scaling = retarget_config.get('scaling', 1.0)
        self.project_tip_dir = retarget_config.get('project_tip_dir', False)

        # FullHandVec parameters
        self.w_full_hand = retarget_config.get('w_full_hand', 1.0)
        segment_scaling_config = retarget_config.get('segment_scaling', {})
        finger_names = ['thumb', 'index', 'middle', 'ring', 'pinky']
        # For optimization: (5, 3) - PIP, DIP, TIP only
        self.segment_scaling = np.ones((5, 3), dtype=np.float64)
        # For visualization: (5, 4) - MCP, PIP, DIP, TIP (full version)
        self.segment_scaling_full = np.ones((5, 4), dtype=np.float64)
        for i, finger_name in enumerate(finger_names):
            if finger_name in segment_scaling_config:
                scales = np.array(segment_scaling_config[finger_name])
                if len(scales) == 4:
                    # 4-param format: [MCP, PIP, DIP, TIP]
                    self.segment_scaling_full[i] = scales
                    self.segment_scaling[i] = scales[1:4]  # PIP, DIP, TIP for optimization
                elif len(scales) == 3:
                    # 3-param format: [PIP, DIP, TIP] - assume MCP scale = 1.0
                    self.segment_scaling_full[i] = np.array([1.0, scales[0], scales[1], scales[2]])
                    self.segment_scaling[i] = scales

        # Pinch thresholds
        pinch_config = retarget_config.get('pinch_thresholds', {})
        self.d1 = np.array([
            pinch_config.get('index', {}).get('d1', 2.0),
            pinch_config.get('middle', {}).get('d1', 2.0),
            pinch_config.get('ring', {}).get('d1', 2.0),
            pinch_config.get('pinky', {}).get('d1', 2.0),
        ], dtype=np.float64)
        self.d2 = np.array([
            pinch_config.get('index', {}).get('d2', 4.0),
            pinch_config.get('middle', {}).get('d2', 4.0),
            pinch_config.get('ring', {}).get('d2', 4.0),
            pinch_config.get('pinky', {}).get('d2', 4.0),
        ], dtype=np.float64)

        # link1 (finger-plane) names come from BaseOptimizer._resolve_link_names,
        # so a custom optimizer.link_naming applies to them too.
        all_link_names = (
            [self.origin_link_name] +
            self.task_link_names +
            self.link3_names +
            self.link4_names +
            self.link1_names
        )
        self.computed_link_names = list(dict.fromkeys(all_link_names))
        self.computed_link_indices = [
            self.robot.get_link_index(name) for name in self.computed_link_names
        ]
        self.link1_indices = [
            self.computed_link_names.index(name) for name in self.link1_names
        ]

        # Optional extensions (default: inactive, equivalent to the base loss).
        # Thumb-specific: drop wrist->PIP loss term from FullHand on thumb (use DIP+TIP only).
        self.thumb_skip_pip = retarget_config.get('thumb_skip_pip', False)
        # Hyperextension soft constraint on PIP/DIP joints.
        self.w_hyper = retarget_config.get('w_hyper', 0.0)
        self.soft_min = retarget_config.get('soft_min', 0.0)
        # DIP<->PIP biomechanical coupling soft constraint.
        self.w_couple = retarget_config.get('w_couple', 0.0)
        self.couple_ratio = retarget_config.get('couple_ratio', 0.7)

        # The optimizer.urdf_path override (e.g. a Wuji Hand 2 model) is loaded up front
        # by BaseOptimizer, so self.robot is already the final hand here.

        # Resolve PIP/DIP qpos indices from the (possibly overridden) URDF.
        self._resolve_flex_indices()
        self._geometry = AdaptiveGeometry(
            self.origin_indices, self.task_indices, self.link3_indices, self.link4_indices
        )

    def _resolve_flex_indices(self):
        """Resolve PIP/DIP qpos indices dynamically from the URDF kinematic chain.

        Each finger's PIP/DIP joint is mapped to its qpos slot via the PIP/DIP
        link's parent joint, so the mapping follows the actual URDF instead of a
        fixed index layout. This keeps the hyperextension (``w_hyper``) and
        DIP<->PIP coupling (``w_couple``) soft constraints on the correct qpos
        dimensions even when a custom URDF (``optimizer.urdf_path``, e.g. Wuji Hand 2)
        declares joints in a different order or with a non-uniform DOF layout. A
        missing finger link or a duplicate resolved index raises immediately at
        load time instead of corrupting the optimization silently.
        """
        pip_idx = [self.robot.get_actuated_qpos_index(n) for n in self.link3_names]
        dip_idx = [self.robot.get_actuated_qpos_index(n) for n in self.link4_names]

        combined = pip_idx + dip_idx
        if len(set(combined)) != len(combined):
            raise RuntimeError(
                "Resolved PIP/DIP qpos indices contain duplicates "
                f"(pip={pip_idx}, dip={dip_idx}); check the URDF finger chain "
                "(finger{i}_link3 = PIP, finger{i}_link4 = DIP)."
            )

        self._pip_idx = np.array(pip_idx, dtype=np.int64)
        self._dip_idx = np.array(dip_idx, dtype=np.int64)
        # flex = PIP ∪ DIP, sorted (order is irrelevant: the penalty is an
        # elementwise sum and its gradient scatters back to the same indices).
        self._flex_idx = np.array(sorted(combined), dtype=np.int64)

    def _compute_pinch_alpha(self, mediapipe_keypoints: np.ndarray) -> np.ndarray:
        """Compute alpha weights for each finger."""
        thumb_tip = mediapipe_keypoints[self.MP_TIP_INDICES[0]]
        finger_tips = mediapipe_keypoints[self.MP_TIP_INDICES[1:]]
        distances = np.linalg.norm(finger_tips - thumb_tip, axis=1) * M_TO_CM
        alphas_4 = np.clip((self.d2 - distances) / (self.d2 - self.d1 + 1e-8), 0.0, 0.7)
        alpha_thumb = np.max(alphas_4)
        return np.concatenate([[alpha_thumb], alphas_4])

    def solve(
        self,
        mediapipe_keypoints: np.ndarray,
        last_qpos: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        """Solve for joint angles."""
        if self._enable_timing:
            t_total_start = time.perf_counter()
            t_preprocess_start = time.perf_counter()
            self._timing.start_frame()

        mediapipe_keypoints = np.asarray(mediapipe_keypoints, dtype=np.float64)
        if mediapipe_keypoints.shape != (21, 3):
            raise ValueError(f"Expected shape (21, 3), got {mediapipe_keypoints.shape}")

        reg_qpos = self._get_reg_qpos(last_qpos)
        init_qpos = self._get_init_qpos(last_qpos)

        alphas = self._compute_pinch_alpha(mediapipe_keypoints)
        target_tip_vectors = self._compute_tip_vectors(mediapipe_keypoints, self.scaling)
        target_tip_dirs = self._compute_tip_dirs(mediapipe_keypoints)
        target_full_hand_vectors = self._compute_full_hand_vectors(
            mediapipe_keypoints, self.segment_scaling
        )

        if self._enable_timing:
            self._timing.preprocess_ms += (time.perf_counter() - t_preprocess_start) * 1000
            t_nlopt_start = time.perf_counter()

        objective_fn = self._get_objective_analytical(
            target_tip_vectors, target_tip_dirs, target_full_hand_vectors, alphas, reg_qpos
        )
        result = self._run_optimization(objective_fn, init_qpos)

        if self._enable_timing:
            self._timing.nlopt_ms += (time.perf_counter() - t_nlopt_start) * 1000
            self._timing.total_ms += (time.perf_counter() - t_total_start) * 1000
            self._timing.call_count += 1
            self._timing.end_frame(self.opt.get_numevals())

        return result

    def compute_cost(
        self,
        qpos: np.ndarray,
        mediapipe_keypoints: np.ndarray,
    ) -> float:
        """Compute cost for given joint angles."""
        alphas = self._compute_pinch_alpha(mediapipe_keypoints)
        target_tip_vectors = self._compute_tip_vectors(mediapipe_keypoints, self.scaling)
        target_tip_dirs = self._compute_tip_dirs(mediapipe_keypoints)
        target_full_hand_vectors = self._compute_full_hand_vectors(
            mediapipe_keypoints, self.segment_scaling
        )
        loss, _ = self._loss_and_grad_analytical(
            qpos, target_tip_vectors, target_tip_dirs, target_full_hand_vectors, alphas, None
        )
        return float(loss)

    def _loss_and_grad_analytical(
        self,
        qpos: np.ndarray,
        target_tip_vectors: np.ndarray,
        target_tip_dirs: np.ndarray,
        target_full_hand_vectors: np.ndarray,
        alphas: np.ndarray,
        last_qpos: Optional[np.ndarray],
    ) -> tuple[float, np.ndarray]:
        """Compute loss and gradient analytically."""
        qpos = np.asarray(qpos, dtype=np.float64)

        # Forward kinematics
        if self._enable_timing:
            t_fk_start = time.perf_counter()

        self.robot.compute_forward_kinematics(qpos)
        positions = np.array([
            self.robot.get_link_pose(idx)[:3, 3] for idx in self.computed_link_indices
        ], dtype=np.float64) * M_TO_CM

        if self._enable_timing:
            self._timing.fk_ms += (time.perf_counter() - t_fk_start) * 1000
            t_jac_start = time.perf_counter()

        # Get Jacobians (num_links, 3, nq) - already in world frame
        Js = self.robot.compute_all_jacobians_batch(qpos, self.computed_link_indices) * M_TO_CM

        if self._enable_timing:
            self._timing.jacobian_ms += (time.perf_counter() - t_jac_start) * 1000
            t_grad_start = time.perf_counter()

        total_loss, total_grad = self._geometry.loss_gradient(
            positions, Js, target_tip_vectors, target_tip_dirs,
            target_full_hand_vectors, alphas, self.huber_delta, self.huber_delta_dir,
            self.w_pos, self.w_dir, self.w_full_hand, self.thumb_skip_pip,
        )

        # === Regularization ===
        if last_qpos is not None:
            total_loss += self.norm_delta * np.sum((qpos - last_qpos) ** 2)
            total_grad += 2.0 * self.norm_delta * (qpos - last_qpos)

        # === Hyperextension penalty (PIP/DIP only, gated by w_hyper) ===
        if self.w_hyper != 0.0:
            flex_qpos = qpos[self._flex_idx]
            penalty = np.maximum(self.soft_min - flex_qpos, 0.0)
            total_loss += self.w_hyper * np.sum(penalty ** 2)
            total_grad[self._flex_idx] += self.w_hyper * (-2.0 * penalty)

        # === Coupling penalty (DIP toward couple_ratio * PIP, gated by w_couple) ===
        if self.w_couple != 0.0:
            pip_q = qpos[self._pip_idx]
            dip_q = qpos[self._dip_idx]
            diff = dip_q - self.couple_ratio * pip_q
            total_loss += self.w_couple * np.sum(diff ** 2)
            total_grad[self._dip_idx] += self.w_couple * (2.0 * diff)
            total_grad[self._pip_idx] += self.w_couple * (-2.0 * self.couple_ratio * diff)

        if self._enable_timing:
            self._timing.gradient_ms += (time.perf_counter() - t_grad_start) * 1000

        return total_loss, total_grad

    def get_timing_stats(self) -> TimingStats:
        """Get timing statistics."""
        return self._timing

    def reset_timing_stats(self):
        """Reset timing statistics."""
        self._timing.reset()

    def set_timing_enabled(self, enabled: bool):
        """Enable or disable timing instrumentation."""
        self._enable_timing = enabled

    def _get_objective_analytical(
        self,
        target_tip_vectors: np.ndarray,
        target_tip_dirs: np.ndarray,
        target_full_hand_vectors: np.ndarray,
        alphas: np.ndarray,
        last_qpos: Optional[np.ndarray],
    ):
        """Create NLopt objective function with analytical gradient."""
        target_tip_vectors = np.asarray(target_tip_vectors, dtype=np.float64)
        target_tip_dirs = np.asarray(target_tip_dirs, dtype=np.float64)
        target_full_hand_vectors = np.asarray(target_full_hand_vectors, dtype=np.float64)
        alphas = np.asarray(alphas, dtype=np.float64)
        if last_qpos is not None:
            last_qpos = np.asarray(last_qpos, dtype=np.float64)

        def objective(x, grad_out):
            qpos = np.asarray(x, dtype=np.float64)
            loss, grad = self._loss_and_grad_analytical(
                qpos, target_tip_vectors, target_tip_dirs, target_full_hand_vectors, alphas, last_qpos
            )
            if grad_out.size > 0:
                grad_out[:] = grad
            # Record iteration loss for plotting
            if self._enable_timing:
                self._timing.record_iter_loss(float(loss))
            return float(loss)

        return objective
