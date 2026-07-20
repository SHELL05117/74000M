from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from statusmonitor.version import ANALYSIS_VERSION


def _save(fig: plt.Figure, path: Path, run_id: str) -> str:
    fig.suptitle(f"{run_id}  |  {ANALYSIS_VERSION}", fontsize=9, y=0.995)
    fig.tight_layout(rect=(0, 0, 1, 0.975))
    fig.savefig(path, dpi=150, bbox_inches="tight", metadata={"Creator": "OpenAI GPT-5.6 Codex"})
    plt.close(fig)
    return str(path)


def _break_gaps(time_s: np.ndarray, values: np.ndarray) -> np.ndarray:
    result = np.asarray(values, dtype=float).copy()
    if len(time_s) < 3:
        return result
    dt = np.diff(time_s)
    valid = dt[(dt > 0) & np.isfinite(dt)]
    if not len(valid):
        return result
    threshold = 3.0 * float(np.median(valid))
    result[np.flatnonzero((dt <= 0) | (dt > threshold)) + 1] = np.nan
    return result


def _placeholder(title: str, message: str) -> plt.Figure:
    fig, ax = plt.subplots(figsize=(10, 4.8))
    ax.axis("off")
    ax.set_title(title)
    ax.text(
        0.5,
        0.5,
        message,
        transform=ax.transAxes,
        ha="center",
        va="center",
        fontsize=16,
        color="#8b1e1e",
        bbox={"boxstyle": "round,pad=0.8", "facecolor": "#fff4f4", "edgecolor": "#cc7777"},
    )
    return fig


def generate_standard_plots(
    output_dir: Path,
    run_id: str,
    columns: dict[str, np.ndarray],
    velocity: np.ndarray,
    velocity_unit: str,
    acceleration: np.ndarray,
    jerk: np.ndarray,
    frequency: dict,
    pose_available: bool,
    raw_available: bool,
    actuator_available: bool,
) -> list[str]:
    output_dir.mkdir(parents=True, exist_ok=True)
    t = np.asarray(columns["time_s"], dtype=float)
    paths: list[str] = []

    if pose_available:
        fig, ax = plt.subplots(figsize=(7, 7))
        x = _break_gaps(t, columns["state.pose_x_m"])
        y = _break_gaps(t, columns["state.pose_y_m"])
        ax.plot(x, y, color="#0066aa", linewidth=1.6)
        ax.scatter([x[0]], [y[0]], label="start", color="#2b8a3e", s=35)
        ax.scatter([x[-1]], [y[-1]], label="end", color="#c92a2a", s=35)
        ax.set_aspect("equal", adjustable="datalim")
        ax.set_xlabel("world X [m]")
        ax.set_ylabel("world Y [m]")
        ax.set_title("2D trajectory (+X forward at theta=0, +Y left)")
        ax.legend()
        ax.grid(alpha=0.25)
    else:
        fig = _placeholder("2D trajectory", "POSE INVALID / NOT AVAILABLE")
    paths.append(_save(fig, output_dir / "01_trajectory.png", run_id))

    if pose_available:
        fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
        for ax, name, unit in zip(
            axes,
            ("state.pose_x_m", "state.pose_y_m", "state.pose_theta_rad"),
            ("x [m]", "y [m]", "theta [rad]"),
            strict=True,
        ):
            ax.plot(t, _break_gaps(t, columns[name]), linewidth=1.2)
            ax.set_ylabel(unit)
            ax.grid(alpha=0.25)
        axes[-1].set_xlabel("robot monotonic time [s]")
        axes[0].set_title("Pose versus time (target/error unavailable in schema 3.1)")
    else:
        fig = _placeholder("Position / target / error", "POSE AND TARGET NOT AVAILABLE")
    paths.append(_save(fig, output_dir / "02_position_error.png", run_id))

    fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    axes[0].plot(t, _break_gaps(t, velocity), label=f"primary velocity [{velocity_unit}]")
    axes[0].plot(t, _break_gaps(t, columns["request.vx_mps"]), label="requested vx [m/s]", alpha=0.7)
    axes[0].legend(loc="best")
    axes[0].grid(alpha=0.25)
    axes[1].plot(t, _break_gaps(t, columns["state.body_omega_radps"]), label="body omega [rad/s]")
    axes[1].plot(t, _break_gaps(t, columns["request.omega_radps"]), label="requested omega [rad/s]", alpha=0.7)
    axes[1].set_xlabel("robot monotonic time [s]")
    axes[1].legend(loc="best")
    axes[1].grid(alpha=0.25)
    axes[0].set_title("Velocity signals (source and units shown explicitly)")
    paths.append(_save(fig, output_dir / "03_velocity.png", run_id))

    fig, ax = plt.subplots(figsize=(11, 5))
    ax.plot(t, _break_gaps(t, acceleration), color="#d9480f", linewidth=1.0)
    ax.set_title("Robust offline acceleration estimate")
    ax.set_xlabel("robot monotonic time [s]")
    ax.set_ylabel(f"d({velocity_unit})/dt")
    ax.grid(alpha=0.25)
    paths.append(_save(fig, output_dir / "04_acceleration.png", run_id))

    fig, ax = plt.subplots(figsize=(11, 5))
    ax.plot(t, _break_gaps(t, jerk), color="#862e9c", linewidth=0.9)
    ax.set_title("Robust offline jerk estimate (noise-sensitive)")
    ax.set_xlabel("robot monotonic time [s]")
    ax.set_ylabel(f"d²({velocity_unit})/dt²")
    ax.grid(alpha=0.25)
    paths.append(_save(fig, output_dir / "05_jerk.png", run_id))

    fig = _placeholder(
        "PID / feedforward terms",
        "PID TERMS NOT AVAILABLE\nschema 3.1 availability gate is false",
    )
    paths.append(_save(fig, output_dir / "06_pid.png", run_id))

    if raw_available:
        fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
        colors = ["#1971c2", "#74c0fc", "#0b7285", "#e8590c", "#ffa94d", "#c92a2a"]
        labels = [f"L{i}" for i in range(1, 4)] + [f"R{i}" for i in range(1, 4)]
        for color, label in zip(colors, labels, strict=True):
            axes[0].plot(t, _break_gaps(t, columns[f"motor.{label}.velocity_radps"]), color=color, label=label)
            axes[1].plot(t, _break_gaps(t, columns[f"motor.{label}.current_A"]), color=color)
            axes[2].plot(t, _break_gaps(t, columns[f"motor.{label}.temperature_C"]), color=color)
        axes[0].set_ylabel("motor speed [rad/s]")
        axes[1].set_ylabel("current [A]")
        axes[2].set_ylabel("temperature [°C]")
        axes[2].set_xlabel("robot monotonic time [s]")
        axes[0].legend(ncol=6, loc="best")
        axes[0].set_title("Per-motor health")
        for ax in axes:
            ax.grid(alpha=0.25)
    else:
        fig = _placeholder("Per-motor health", "RAW MOTOR SIGNALS NOT AVAILABLE")
    paths.append(_save(fig, output_dir / "07_motors.png", run_id))

    if raw_available or actuator_available:
        fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
        if raw_available:
            axes[0].plot(t, _break_gaps(t, columns["raw.battery_V"]), label="battery [V]", color="#2b8a3e")
            axes[0].legend()
        else:
            axes[0].text(0.5, 0.5, "BATTERY NOT AVAILABLE", transform=axes[0].transAxes, ha="center")
        axes[0].grid(alpha=0.25)
        if actuator_available:
            axes[1].plot(t, _break_gaps(t, columns["actuator.final_left_V"]), label="final left [V]")
            axes[1].plot(t, _break_gaps(t, columns["actuator.final_right_V"]), label="final right [V]")
            axes[1].plot(t, _break_gaps(t, columns["actuator.derate_applied"]), label="derate [ratio]", alpha=0.8)
            axes[1].legend()
        else:
            axes[1].text(0.5, 0.5, "ACTUATOR INTENT NOT AVAILABLE", transform=axes[1].transAxes, ha="center")
        axes[1].set_xlabel("robot monotonic time [s]")
        axes[1].grid(alpha=0.25)
        axes[0].set_title("Battery, actuator voltage, and derating")
    else:
        fig = _placeholder("Energy and actuator output", "SIGNALS NOT AVAILABLE")
    paths.append(_save(fig, output_dir / "08_energy.png", run_id))

    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    axes[0].plot(t, 1e3 * _break_gaps(t, columns["timing.raw_dt_s"]), label="raw dt")
    axes[0].set_ylabel("dt [ms]")
    axes[1].plot(t, 1e3 * _break_gaps(t, columns["timing.exec_s"]), label="exec", color="#d9480f")
    axes[1].set_ylabel("exec [ms]")
    axes[2].step(t, columns["trace.deadline_missed"].astype(int), where="post", label="deadline missed")
    axes[2].plot(t, columns["trace.ring_high_watermark"], label="ring high-water")
    axes[2].set_xlabel("robot monotonic time [s]")
    axes[2].legend()
    axes[0].set_title("Runtime timing and logging pressure")
    for ax in axes:
        ax.grid(alpha=0.25)
    paths.append(_save(fig, output_dir / "09_timing.png", run_id))

    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    axes[0].step(t, columns["fault.active_bits"], where="post")
    axes[0].set_ylabel("active fault bits")
    axes[1].step(t, columns["recording.event_bits"], where="post")
    axes[1].set_ylabel("recording events")
    axes[2].step(t, columns["event.bits"], where="post")
    axes[2].set_ylabel("system events")
    axes[2].set_xlabel("robot monotonic time [s]")
    axes[0].set_title("Fault and event timeline (bit dictionaries are schema-bound)")
    for ax in axes:
        ax.grid(alpha=0.25)
    paths.append(_save(fig, output_dir / "10_events.png", run_id))

    if frequency.get("available"):
        fig, ax = plt.subplots(figsize=(11, 5))
        f = np.asarray(frequency["frequency_Hz"])
        psd = np.asarray(frequency["psd"])
        ax.semilogy(f, np.maximum(psd, np.finfo(float).tiny))
        ax.axvline(
            frequency["dominant_frequency_Hz"],
            color="#c92a2a",
            linestyle="--",
            label=f"dominant {frequency['dominant_frequency_Hz']:.3g} Hz",
        )
        ax.set_xlabel("frequency [Hz]")
        ax.set_ylabel("Welch PSD")
        ax.set_title("Frequency content (PSD, not a closed-loop FRF/Bode plot)")
        ax.legend()
        ax.grid(alpha=0.25)
    else:
        fig = _placeholder("Frequency analysis", f"NOT AVAILABLE\n{frequency.get('reason', '')}")
    paths.append(_save(fig, output_dir / "11_frequency.png", run_id))
    return paths
