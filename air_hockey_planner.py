"""Air hockey robot planning module for safe target generation.

This module accepts time-stamped puck observations, estimates puck velocity,
predicts simple future motion, selects a high-level behavior mode, and emits
smoothed paddle target coordinates plus debug/status information.

The planner does not output motor commands. It is intended to sit above a
future Raspberry Pi or other low-level controller that converts targets into
mechanism motion.
"""

from __future__ import annotations

import csv
import math
import random
import sys
from collections import deque
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Deque, Iterable, Optional


def is_finite_number(value: float) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(value)


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@dataclass(frozen=True)
class TableBounds:
    min_x: float
    max_x: float
    min_y: float
    max_y: float


@dataclass(frozen=True)
class PlannerConfig:
    table: TableBounds = TableBounds(min_x=0.0, max_x=2.0, min_y=-0.45, max_y=0.45)
    goal_x: float = 0.12
    defend_x: float = 0.18
    home_x: float = 0.20
    home_y: float = 0.0
    paddle_reach_x_min: float = 0.05
    paddle_reach_x_max: float = 0.75
    paddle_radius_margin: float = 0.04
    max_paddle_speed: float = 1.3
    max_paddle_accel: float = 3.5
    target_position_alpha: float = 0.45
    tracker_window_size: int = 8
    min_fit_samples: int = 3
    min_velocity_dt: float = 1e-3
    measurement_max_age: float = 0.25
    watchdog_timeout: float = 0.40
    timestamp_jump_reject: float = 1.0
    velocity_smoothing_alpha: float = 0.35
    defend_velocity_threshold: float = -0.12
    attack_velocity_threshold: float = -0.05
    slow_puck_speed_threshold: float = 0.25
    attack_zone_x_max: float = 0.65
    intercept_uncertainty_time: float = 2.0
    intercept_y_margin: float = 0.02
    mode_hysteresis_seconds: float = 0.20
    conservative_ready_band: float = 0.015
    camera_latency: float = 0.030
    command_latency: float = 0.025
    future_feedback_latency: float = 0.000
    enable_bounce_stub: bool = False
    csv_log_path: Optional[str] = None

    @property
    def total_control_delay(self) -> float:
        return self.camera_latency + self.command_latency + self.future_feedback_latency


@dataclass(frozen=True)
class PuckMeasurement:
    timestamp: float
    x: float
    y: float
    confidence: Optional[float] = None


@dataclass(frozen=True)
class PuckState:
    timestamp: float
    x: float
    y: float
    vx: float
    vy: float
    confidence: float
    valid_sample_count: int


@dataclass(frozen=True)
class PaddleState:
    timestamp: float
    x: float
    y: float
    vx: float = 0.0
    vy: float = 0.0


class Mode(str, Enum):
    DEFENSE = "DEFENSE"
    ATTACK = "ATTACK"
    HOME = "HOME"
    SAFE_HOLD = "SAFE_HOLD"


@dataclass(frozen=True)
class PredictionResult:
    is_valid: bool
    intercept_time: Optional[float]
    intercept_y: Optional[float]
    confidence: float
    reason: str


@dataclass(frozen=True)
class PlannerOutput:
    timestamp: float
    mode: Mode
    target_x: float
    target_y: float
    reason: str
    predicted_intercept_time: Optional[float]
    predicted_intercept_y: Optional[float]
    estimated_puck_vx: float
    estimated_puck_vy: float
    clamped: bool
    safety_limited: bool
    stale_data: bool
    debug_summary: str


class CSVDecisionLogger:
    def __init__(self, path: Optional[str]) -> None:
        self._path = Path(path) if path else None
        self._writer: Optional[csv.DictWriter] = None
        self._handle = None

    def log(self, output: PlannerOutput) -> None:
        if self._path is None:
            return
        if self._writer is None:
            self._path.parent.mkdir(parents=True, exist_ok=True)
            self._handle = self._path.open("w", newline="", encoding="utf-8")
            fieldnames = [
                "timestamp",
                "mode",
                "target_x",
                "target_y",
                "reason",
                "predicted_intercept_time",
                "predicted_intercept_y",
                "estimated_puck_vx",
                "estimated_puck_vy",
                "clamped",
                "safety_limited",
                "stale_data",
                "debug_summary",
            ]
            self._writer = csv.DictWriter(self._handle, fieldnames=fieldnames)
            self._writer.writeheader()
        self._writer.writerow(
            {
                "timestamp": f"{output.timestamp:.6f}",
                "mode": output.mode.value,
                "target_x": f"{output.target_x:.6f}",
                "target_y": f"{output.target_y:.6f}",
                "reason": output.reason,
                "predicted_intercept_time": output.predicted_intercept_time,
                "predicted_intercept_y": output.predicted_intercept_y,
                "estimated_puck_vx": f"{output.estimated_puck_vx:.6f}",
                "estimated_puck_vy": f"{output.estimated_puck_vy:.6f}",
                "clamped": output.clamped,
                "safety_limited": output.safety_limited,
                "stale_data": output.stale_data,
                "debug_summary": output.debug_summary,
            }
        )
        self._handle.flush()

    def close(self) -> None:
        if self._handle is not None:
            self._handle.close()
            self._handle = None
            self._writer = None


class PuckTracker:
    """Tracks recent valid measurements and estimates puck velocity."""

    def __init__(self, config: PlannerConfig) -> None:
        self.config = config
        self._history: Deque[PuckMeasurement] = deque(maxlen=config.tracker_window_size)
        self._last_velocity = (0.0, 0.0)

    def ingest(self, measurement: PuckMeasurement) -> bool:
        if not self._is_measurement_valid(measurement):
            print(
                f"[tracker] Ignoring invalid measurement:"
                f" t={measurement.timestamp!r} x={measurement.x!r} y={measurement.y!r}"
            )
            return False

        if self._history:
            last = self._history[-1]
            dt = measurement.timestamp - last.timestamp
            if dt <= 0.0:
                print(
                    "[tracker] Ignoring non-increasing timestamp:"
                    f" new_t={measurement.timestamp:.3f} last_t={last.timestamp:.3f}"
                )
                return False
            if dt > self.config.timestamp_jump_reject:
                print(
                    "[tracker] Large timestamp jump detected; clearing old history."
                    f" dt={dt:.3f}s"
                )
                self._history.clear()

        self._history.append(measurement)
        return True

    def current_state(self) -> Optional[PuckState]:
        if not self._history:
            return None

        newest = self._history[-1]
        vx, vy = self._estimate_velocity()
        confidence = self._estimate_confidence()
        return PuckState(
            timestamp=newest.timestamp,
            x=newest.x,
            y=newest.y,
            vx=vx,
            vy=vy,
            confidence=confidence,
            valid_sample_count=len(self._history),
        )

    def _is_measurement_valid(self, measurement: PuckMeasurement) -> bool:
        if not (
            is_finite_number(measurement.timestamp)
            and is_finite_number(measurement.x)
            and is_finite_number(measurement.y)
        ):
            return False
        if measurement.timestamp < 0.0:
            return False

        bounds = self.config.table
        x_margin = 0.15
        y_margin = 0.15
        if not (bounds.min_x - x_margin <= measurement.x <= bounds.max_x + x_margin):
            return False
        if not (bounds.min_y - y_margin <= measurement.y <= bounds.max_y + y_margin):
            return False

        if measurement.confidence is not None and (
            not is_finite_number(measurement.confidence)
            or measurement.confidence < 0.0
            or measurement.confidence > 1.0
        ):
            return False
        return True

    def _estimate_velocity(self) -> tuple[float, float]:
        samples = list(self._history)
        if len(samples) < self.config.min_fit_samples:
            return self._last_velocity

        vx_raw = self._least_squares_velocity(samples, axis="x")
        vy_raw = self._least_squares_velocity(samples, axis="y")
        alpha = self.config.velocity_smoothing_alpha
        vx = alpha * vx_raw + (1.0 - alpha) * self._last_velocity[0]
        vy = alpha * vy_raw + (1.0 - alpha) * self._last_velocity[1]
        self._last_velocity = (vx, vy)
        return vx, vy

    def _least_squares_velocity(self, samples: list[PuckMeasurement], axis: str) -> float:
        t0 = samples[0].timestamp
        times = [sample.timestamp - t0 for sample in samples]
        if max(times) - min(times) < self.config.min_velocity_dt:
            return 0.0

        values = [getattr(sample, axis) for sample in samples]
        t_mean = sum(times) / len(times)
        v_mean = sum(values) / len(values)
        denom = sum((time - t_mean) ** 2 for time in times)
        if denom < 1e-9:
            return 0.0
        numer = sum((time - t_mean) * (value - v_mean) for time, value in zip(times, values))
        return numer / denom

    def _estimate_confidence(self) -> float:
        if not self._history:
            return 0.0
        scores = [sample.confidence for sample in self._history if sample.confidence is not None]
        sample_factor = min(1.0, len(self._history) / max(1.0, self.config.min_fit_samples))
        if not scores:
            return 0.6 * sample_factor
        return sample_factor * (sum(scores) / len(scores))


class TrajectoryPredictor:
    """Predicts simple constant-velocity puck motion."""

    def __init__(self, config: PlannerConfig) -> None:
        self.config = config

    def predict_intercept(self, puck: PuckState, line_x: float) -> PredictionResult:
        effective_x = puck.x + puck.vx * self.config.total_control_delay
        effective_y = puck.y + puck.vy * self.config.total_control_delay

        if puck.vx >= -1e-6:
            return PredictionResult(False, None, None, 0.0, "Puck not moving toward robot.")

        time_to_line = (line_x - effective_x) / puck.vx
        if time_to_line < 0.0:
            return PredictionResult(False, None, None, 0.0, "Intercept would have occurred in the past.")
        if time_to_line > self.config.intercept_uncertainty_time:
            return PredictionResult(
                False,
                time_to_line,
                None,
                0.1,
                "Prediction horizon too long; uncertainty too high.",
            )

        intercept_y = effective_y + puck.vy * time_to_line
        if self.config.enable_bounce_stub:
            intercept_y = self.apply_wall_bounce_stub(intercept_y)

        confidence = self._prediction_confidence(puck, time_to_line, intercept_y)
        return PredictionResult(
            True,
            max(0.0, time_to_line - self.config.total_control_delay),
            intercept_y,
            confidence,
            "Constant-velocity intercept prediction.",
        )

    def apply_wall_bounce_stub(self, y_position: float) -> float:
        return y_position

    def _prediction_confidence(self, puck: PuckState, time_to_line: float, intercept_y: float) -> float:
        y_ok = self.config.table.min_y - 0.10 <= intercept_y <= self.config.table.max_y + 0.10
        horizon_score = clamp(1.0 - (time_to_line / self.config.intercept_uncertainty_time), 0.0, 1.0)
        base = 0.5 * puck.confidence + 0.5 * horizon_score
        if not y_ok:
            base *= 0.5
        return clamp(base, 0.0, 1.0)


class SafetyManager:
    """Applies workspace clamps and dynamic rate limits to targets."""

    def __init__(self, config: PlannerConfig) -> None:
        self.config = config
        self._last_command: Optional[PaddleState] = None

    def sanitize_target(self, now: float, requested_x: float, requested_y: float) -> tuple[PaddleState, bool, bool, str]:
        requested_x = 0.0 if not is_finite_number(requested_x) else requested_x
        requested_y = 0.0 if not is_finite_number(requested_y) else requested_y

        bounds = self._paddle_bounds()
        clamped_x = clamp(requested_x, bounds.min_x, bounds.max_x)
        clamped_y = clamp(requested_y, bounds.min_y, bounds.max_y)
        clamped = (clamped_x != requested_x) or (clamped_y != requested_y)

        requested = PaddleState(timestamp=now, x=clamped_x, y=clamped_y)
        if self._last_command is None:
            self._last_command = requested
            return requested, clamped, False, "Initial safe target."

        dt = max(now - self._last_command.timestamp, 1e-3)
        limited = self._apply_dynamics_limit(requested, dt)
        safety_limited = (limited.x != requested.x) or (limited.y != requested.y)
        self._last_command = limited

        reason_parts = []
        if clamped:
            reason_parts.append("workspace clamp")
        if safety_limited:
            reason_parts.append("speed/accel limit")
        if not reason_parts:
            reason_parts.append("within limits")
        return limited, clamped, safety_limited, ", ".join(reason_parts)

    def estimate_time_to_target(self, current: PaddleState, target_x: float, target_y: float) -> float:
        distance = math.hypot(target_x - current.x, target_y - current.y)
        vmax = max(self.config.max_paddle_speed, 1e-6)
        amax = max(self.config.max_paddle_accel, 1e-6)
        accel_time = vmax / amax
        accel_distance = 0.5 * amax * accel_time * accel_time
        if distance < 2.0 * accel_distance:
            return 2.0 * math.sqrt(distance / amax)
        cruise_distance = distance - 2.0 * accel_distance
        return 2.0 * accel_time + cruise_distance / vmax

    def _apply_dynamics_limit(self, requested: PaddleState, dt: float) -> PaddleState:
        prev = self._last_command
        assert prev is not None
        dx = requested.x - prev.x
        dy = requested.y - prev.y
        distance = math.hypot(dx, dy)

        max_step_speed = self.config.max_paddle_speed * dt
        prev_speed = math.hypot(prev.vx, prev.vy)
        max_allowed_speed = min(self.config.max_paddle_speed, prev_speed + self.config.max_paddle_accel * dt)
        max_step_accel = max_allowed_speed * dt
        max_step = min(max_step_speed, max_step_accel if max_step_accel > 0.0 else max_step_speed)

        if distance <= max_step or distance < 1e-9:
            return PaddleState(requested.timestamp, requested.x, requested.y, dx / dt, dy / dt)

        scale = max_step / distance
        x = prev.x + dx * scale
        y = prev.y + dy * scale
        return PaddleState(requested.timestamp, x, y, (x - prev.x) / dt, (y - prev.y) / dt)

    def _paddle_bounds(self) -> TableBounds:
        margin = self.config.paddle_radius_margin
        return TableBounds(
            min_x=max(self.config.paddle_reach_x_min, self.config.table.min_x + margin),
            max_x=min(self.config.paddle_reach_x_max, self.config.table.max_x - margin),
            min_y=self.config.table.min_y + margin,
            max_y=self.config.table.max_y - margin,
        )


class StrategyPlanner:
    """Selects a high-level mode and generates requested paddle targets."""

    def __init__(self, config: PlannerConfig, safety_manager: SafetyManager) -> None:
        self.config = config
        self.safety_manager = safety_manager
        self._mode = Mode.HOME
        self._last_mode_change_time = 0.0

    def plan(
        self,
        now: float,
        puck_state: Optional[PuckState],
        paddle_state: PaddleState,
        prediction: Optional[PredictionResult],
    ) -> PlannerOutput:
        stale = puck_state is None or (now - puck_state.timestamp) > self.config.watchdog_timeout
        if stale:
            return self._safe_hold_output(now, "No fresh puck data.")

        desired_mode, reason = self._select_mode(puck_state, paddle_state, prediction)
        self._update_mode(now, desired_mode)

        if self._mode == Mode.DEFENSE:
            requested_x, requested_y, reason = self._defense_target(puck_state, prediction, paddle_state)
        elif self._mode == Mode.ATTACK:
            requested_x, requested_y, reason = self._attack_target(puck_state, paddle_state)
        else:
            requested_x, requested_y, reason = self._home_target(paddle_state, reason)

        target_state, clamped, safety_limited, safety_reason = self.safety_manager.sanitize_target(
            now, requested_x, requested_y
        )
        debug_summary = self._build_debug_summary(self._mode, puck_state, prediction, reason, target_state, safety_reason, False)
        return PlannerOutput(
            now,
            self._mode,
            target_state.x,
            target_state.y,
            reason,
            prediction.intercept_time if prediction else None,
            prediction.intercept_y if prediction else None,
            puck_state.vx,
            puck_state.vy,
            clamped,
            safety_limited,
            False,
            debug_summary,
        )

    def _select_mode(
        self, puck_state: PuckState, paddle_state: PaddleState, prediction: Optional[PredictionResult]
    ) -> tuple[Mode, str]:
        puck_speed = math.hypot(puck_state.vx, puck_state.vy)
        moving_toward_robot = puck_state.vx <= self.config.defend_velocity_threshold
        mildly_toward_robot = puck_state.vx <= self.config.attack_velocity_threshold
        in_attack_zone = puck_state.x <= self.config.attack_zone_x_max

        if moving_toward_robot:
            if prediction and prediction.is_valid and prediction.confidence >= 0.25:
                return Mode.DEFENSE, "Puck moving toward robot with usable intercept prediction."
            return Mode.DEFENSE, "Puck moving toward robot; using conservative defense."

        if in_attack_zone and (puck_speed <= self.config.slow_puck_speed_threshold or mildly_toward_robot):
            travel_time = self.safety_manager.estimate_time_to_target(paddle_state, puck_state.x, puck_state.y)
            if travel_time <= 0.8:
                return Mode.ATTACK, "Slow/reachable puck inside attack zone."
            return Mode.HOME, "Puck is attackable in theory, but move would overcommit."

        if abs(paddle_state.x - self.config.home_x) <= self.config.conservative_ready_band and abs(
            paddle_state.y - self.config.home_y
        ) <= self.config.conservative_ready_band:
            return Mode.HOME, "Already near ready position."
        return Mode.HOME, "Puck not threatening; returning to ready position."

    def _update_mode(self, now: float, desired_mode: Mode) -> None:
        if desired_mode == self._mode:
            return
        if (now - self._last_mode_change_time) < self.config.mode_hysteresis_seconds:
            return
        print(f"[planner] Mode transition {self._mode.value} -> {desired_mode.value} at t={now:.3f}")
        self._mode = desired_mode
        self._last_mode_change_time = now

    def _defense_target(
        self, puck_state: PuckState, prediction: Optional[PredictionResult], paddle_state: PaddleState
    ) -> tuple[float, float, str]:
        target_x = self.config.defend_x
        if prediction and prediction.is_valid and prediction.intercept_y is not None and prediction.confidence >= 0.25:
            intercept_y = clamp(
                prediction.intercept_y,
                self.config.table.min_y + self.config.intercept_y_margin,
                self.config.table.max_y - self.config.intercept_y_margin,
            )
            return (
                target_x,
                intercept_y,
                f"Defense intercept at y={intercept_y:.3f}, t={prediction.intercept_time:.3f}s, conf={prediction.confidence:.2f}",
            )

        conservative_y = clamp(puck_state.y, self.config.table.min_y + 0.05, self.config.table.max_y - 0.05)
        smoothed_y = self.config.target_position_alpha * conservative_y + (1.0 - self.config.target_position_alpha) * paddle_state.y
        return target_x, smoothed_y, "Conservative defense due to uncertain prediction."

    def _attack_target(self, puck_state: PuckState, paddle_state: PaddleState) -> tuple[float, float, str]:
        desired_x = clamp(puck_state.x - 0.03, self.config.paddle_reach_x_min, self.config.attack_zone_x_max)
        desired_y = puck_state.y
        smoothed_x = self.config.target_position_alpha * desired_x + (1.0 - self.config.target_position_alpha) * paddle_state.x
        smoothed_y = self.config.target_position_alpha * desired_y + (1.0 - self.config.target_position_alpha) * paddle_state.y
        return smoothed_x, smoothed_y, "Basic attack: move behind slow/reachable puck."

    def _home_target(self, paddle_state: PaddleState, reason: str) -> tuple[float, float, str]:
        target_x = self.config.target_position_alpha * self.config.home_x + (1.0 - self.config.target_position_alpha) * paddle_state.x
        target_y = self.config.target_position_alpha * self.config.home_y + (1.0 - self.config.target_position_alpha) * paddle_state.y
        return target_x, target_y, reason

    def _safe_hold_output(self, now: float, reason: str) -> PlannerOutput:
        self._update_mode(now, Mode.SAFE_HOLD)
        target_state, clamped, safety_limited, safety_reason = self.safety_manager.sanitize_target(
            now, self.config.home_x, self.config.home_y
        )
        debug_summary = self._build_debug_summary(self._mode, None, None, reason, target_state, safety_reason, True)
        return PlannerOutput(now, self._mode, target_state.x, target_state.y, reason, None, None, 0.0, 0.0, clamped, safety_limited, True, debug_summary)

    def _build_debug_summary(
        self,
        mode: Mode,
        puck: Optional[PuckState],
        prediction: Optional[PredictionResult],
        reason: str,
        target: PaddleState,
        safety_reason: str,
        stale: bool,
    ) -> str:
        if puck is None:
            puck_summary = "puck=unavailable"
            prediction_summary = "prediction=none"
        else:
            puck_summary = f"puck=({puck.x:.3f}, {puck.y:.3f}) v=({puck.vx:.3f}, {puck.vy:.3f}) conf={puck.confidence:.2f}"
            if prediction and prediction.is_valid and prediction.intercept_time is not None and prediction.intercept_y is not None:
                prediction_summary = f"intercept=(t={prediction.intercept_time:.3f}, y={prediction.intercept_y:.3f}, conf={prediction.confidence:.2f})"
            else:
                prediction_summary = f"prediction=invalid({prediction.reason if prediction else 'none'})"
        stale_text = " stale_data" if stale else ""
        return (
            f"mode={mode.value}{stale_text}; {reason}; {puck_summary}; {prediction_summary}; "
            f"target=({target.x:.3f}, {target.y:.3f}); safety={safety_reason}"
        )


class AirHockeyPlanner:
    """Top-level planning pipeline for air hockey paddle targets."""

    def __init__(self, config: Optional[PlannerConfig] = None) -> None:
        self.config = config or PlannerConfig()
        self.tracker = PuckTracker(self.config)
        self.predictor = TrajectoryPredictor(self.config)
        self.safety = SafetyManager(self.config)
        self.strategy = StrategyPlanner(self.config, self.safety)
        self.logger = CSVDecisionLogger(self.config.csv_log_path)
        self._estimated_paddle_state = PaddleState(timestamp=0.0, x=self.config.home_x, y=self.config.home_y)
        self._last_debug_mode: Optional[Mode] = None

    def update(
        self,
        measurement: Optional[PuckMeasurement],
        paddle_feedback: Optional[PaddleState] = None,
        now: Optional[float] = None,
    ) -> PlannerOutput:
        if paddle_feedback is not None:
            self._estimated_paddle_state = paddle_feedback

        if measurement is not None:
            accepted = self.tracker.ingest(measurement)
            if not accepted:
                print(f"[planner] Bad measurement rejected at t={measurement.timestamp:.3f}")

        puck_state = self.tracker.current_state()
        cycle_time = now
        if cycle_time is None:
            if measurement is not None:
                cycle_time = measurement.timestamp
            elif paddle_feedback is not None:
                cycle_time = paddle_feedback.timestamp
            elif puck_state is not None:
                cycle_time = puck_state.timestamp
            else:
                cycle_time = self._estimated_paddle_state.timestamp

        prediction = None
        if puck_state is not None:
            prediction = self.predictor.predict_intercept(puck_state, self.config.goal_x)
            print(
                f"[planner] Velocity estimate at t={cycle_time:.3f}: vx={puck_state.vx:.3f}, "
                f"vy={puck_state.vy:.3f}, samples={puck_state.valid_sample_count}, conf={puck_state.confidence:.2f}"
            )
            if prediction.is_valid and prediction.intercept_time is not None and prediction.intercept_y is not None:
                print(
                    f"[planner] Predicted intercept: t={prediction.intercept_time:.3f}s "
                    f"y={prediction.intercept_y:.3f} conf={prediction.confidence:.2f}"
                )
            else:
                print(f"[planner] Prediction unavailable: {prediction.reason}")

        previous_state = self._estimated_paddle_state
        output = self.strategy.plan(cycle_time, puck_state, previous_state, prediction)
        dt = max(output.timestamp - previous_state.timestamp, 1e-3)
        self._estimated_paddle_state = PaddleState(
            timestamp=output.timestamp,
            x=output.target_x,
            y=output.target_y,
            vx=(output.target_x - previous_state.x) / dt,
            vy=(output.target_y - previous_state.y) / dt,
        )
        self.logger.log(output)
        self._print_status(output)
        return output

    def close(self) -> None:
        self.logger.close()

    def _print_status(self, output: PlannerOutput) -> None:
        transition = ""
        if self._last_debug_mode != output.mode:
            transition = " [mode change]"
            self._last_debug_mode = output.mode
        flags = []
        if output.clamped:
            flags.append("CLAMPED")
        if output.safety_limited:
            flags.append("LIMITED")
        if output.stale_data:
            flags.append("STALE")
        flag_text = f" flags={','.join(flags)}" if flags else ""
        print(
            f"[status]{transition} mode={output.mode.value} "
            f"target=({output.target_x:.3f}, {output.target_y:.3f}) "
            f"reason={output.reason}{flag_text}"
        )
        print(f"[status] {output.debug_summary}")


def generate_demo_measurements() -> Iterable[Optional[PuckMeasurement]]:
    random.seed(7)
    dt = 0.05
    t = 0.0
    x = 1.65
    y = 0.08
    vx = -0.95
    vy = 0.10

    for step in range(90):
        if step == 28:
            vx = -0.04
            vy = -0.02
        elif step == 45:
            vx = 0.18
            vy = 0.01
        elif step == 62:
            vx = -0.75
            vy = -0.16

        t += dt
        x += vx * dt
        y += vy * dt

        if y > 0.40 or y < -0.40:
            vy *= -1.0
            y = clamp(y, -0.40, 0.40)

        if step in {10, 11, 52}:
            yield None
            continue
        if step == 35:
            yield PuckMeasurement(timestamp=t, x=float("nan"), y=0.0, confidence=0.5)
            continue
        if step == 70:
            yield PuckMeasurement(timestamp=t - 0.02, x=x, y=y, confidence=0.9)
            continue

        noisy_x = x + random.uniform(-0.006, 0.006)
        noisy_y = y + random.uniform(-0.004, 0.004)
        confidence = clamp(0.80 + random.uniform(-0.15, 0.10), 0.0, 1.0)
        yield PuckMeasurement(timestamp=t, x=noisy_x, y=noisy_y, confidence=confidence)


def main() -> int:
    config = PlannerConfig(csv_log_path="air_hockey_planner_log.csv")
    planner = AirHockeyPlanner(config=config)
    print("[demo] Starting air hockey planner demo.")
    print(
        "[demo] Table x range="
        f"({config.table.min_x:.2f}, {config.table.max_x:.2f}) "
        f"y range=({config.table.min_y:.2f}, {config.table.max_y:.2f})"
    )
    print(f"[demo] Total control delay compensation: {config.total_control_delay:.3f}s")

    try:
        for measurement in generate_demo_measurements():
            if measurement is None:
                next_time = planner._estimated_paddle_state.timestamp + 0.05
                print(f"[demo] Simulated dropped frame at t={next_time:.3f}")
                planner.update(measurement=None, now=next_time)
            else:
                planner.update(measurement=measurement, now=measurement.timestamp)
    finally:
        planner.close()
        print("[demo] Demo finished.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
