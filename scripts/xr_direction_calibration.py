"""Calibrate horizontal heading while preserving the XR gravity-aligned Y up.

Two X-button taps: neutral, then forward. Vertical hand drift is NOT a frame
tilt. Old four-point records may be inspected with the same gravity constraint.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np


def direction_basis(points: list[np.ndarray]) -> np.ndarray:
    if len(points) not in (2, 4):
        raise ValueError("Need neutral and forward points (or an old four-point record)")
    vectors = np.asarray(points[1:], dtype=float) - np.asarray(points[0], dtype=float)
    lengths = np.linalg.norm(vectors, axis=1)
    if not np.all(np.isfinite(vectors)) or np.any(lengths < 0.08) or np.any(lengths > 0.60):
        raise ValueError("Move 8-60 cm from the original NEUTRAL point for each direction")
    # PICO's tracking WORLD is gravity-aligned: +Y is up. Inferring this
    # vector from a human hand stroke mixed ordinary fore/aft drift into Z.
    up = np.array([0., 1., 0.])
    forward = vectors[0].copy()
    forward[1] = 0.
    if np.linalg.norm(forward) < 0.08:
        raise ValueError("Move FORWARD horizontally by at least 8 cm")
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, up)
    # Raw XR is +X right, +Y up, -Z forward. No pitch/roll calibration.
    return np.column_stack([forward, right, up])


def calibrated_map(basis: np.ndarray, frame: str, azimuth: float, yaw_deg: float = 0.) -> np.ndarray:
    from sim_control import view_axes

    forward, right = view_axes(azimuth) if frame == "view" else (np.array([1., 0., 0.]), np.array([0., -1., 0.]))
    target = np.column_stack([forward, right, [0., 0., 1.]])
    yaw = np.deg2rad(yaw_deg)
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]]) @ target @ basis.T


class DirectionCalibration:
    prompts = (
        "[1/2] Grip RELEASED: hold controller at NEUTRAL, tap left X.",
        "[2/2] Move straight FORWARD 15-25 cm; tap X. UP is automatic.",
    )

    def __init__(self, report_path: Path):
        self.report_path = report_path
        self.points: list[np.ndarray] = []
        self.basis: np.ndarray | None = None
        self.last_button = False

    @property
    def ready(self) -> bool:
        return self.basis is not None

    @property
    def prompt(self) -> str:
        return "READY: hold Grip to follow; release Grip + tap X to recalibrate." if self.ready else self.prompts[len(self.points)]

    def update(self, sample) -> str | None:
        from sim_control import fresh

        down = bool(sample.primary_button)
        pressed = down and not self.last_button
        self.last_button = down
        if not pressed:
            return None
        if sample.grip >= 0.15:
            return "Calibration needs Grip fully RELEASED. " + self.prompt
        if not fresh(sample) or sample.pos is None:
            return "No fresh tracking. " + self.prompt
        if self.ready:
            self.basis = None
            self.points = []
        candidate = np.asarray(sample.pos, dtype=float).copy()
        if self.points:
            distance = np.linalg.norm(candidate - self.points[0])
            if not 0.08 <= distance <= 0.60:
                return "Move 8-60 cm from original NEUTRAL, then tap X again. " + self.prompt
        if len(self.points) == 1:
            try:
                basis = direction_basis(self.points + [candidate])
            except ValueError as exc:
                return str(exc) + ". Try again: " + self.prompt
            self.basis = basis
        self.points.append(candidate)
        if self.ready:
            # Audit record only: never silently reuse an old XR-world frame after recenter.
            try:
                self.report_path.parent.mkdir(parents=True, exist_ok=True)
                self.report_path.write_text(json.dumps({"method": "gravity_locked_yaw_v2", "points_m": [p.tolist() for p in self.points],
                                                       "forward_right_up_columns": self.basis.tolist()}, indent=2), encoding="utf-8")
            except OSError as exc:
                return f"Calibration OK; report could not be saved ({exc}). " + self.prompt
            return "Calibration OK. " + self.prompt
        return self.prompt
