from __future__ import annotations

import math

import numpy as np
from scipy import signal


def finite_stats(values: np.ndarray) -> dict[str, float | int | None]:
    values = np.asarray(values, dtype=float)
    finite = values[np.isfinite(values)]
    if finite.size == 0:
        return {
            "n": 0,
            "mean": None,
            "std": None,
            "median": None,
            "p95": None,
            "p99": None,
            "min": None,
            "max": None,
            "max_abs": None,
            "rms": None,
        }
    return {
        "n": int(finite.size),
        "mean": float(np.mean(finite)),
        "std": float(np.std(finite, ddof=1)) if finite.size > 1 else 0.0,
        "median": float(np.median(finite)),
        "p95": float(np.percentile(finite, 95)),
        "p99": float(np.percentile(finite, 99)),
        "min": float(np.min(finite)),
        "max": float(np.max(finite)),
        "max_abs": float(np.max(np.abs(finite))),
        "rms": float(np.sqrt(np.mean(np.square(finite)))),
    }


def split_continuous(time_s: np.ndarray, gap_factor: float = 3.0) -> list[slice]:
    time_s = np.asarray(time_s, dtype=float)
    if time_s.size == 0:
        return []
    dt = np.diff(time_s)
    positive = dt[(dt > 0) & np.isfinite(dt)]
    if positive.size == 0:
        return [slice(0, len(time_s))]
    median_dt = float(np.median(positive))
    breaks = np.flatnonzero((dt <= 0) | (~np.isfinite(dt)) | (dt > gap_factor * median_dt)) + 1
    starts = np.r_[0, breaks]
    stops = np.r_[breaks, len(time_s)]
    return [slice(int(a), int(b)) for a, b in zip(starts, stops, strict=True) if b > a]


def robust_derivative(
    time_s: np.ndarray,
    values: np.ndarray,
    order: int = 1,
    window_s: float = 0.11,
    polyorder: int = 3,
) -> np.ndarray:
    time_s = np.asarray(time_s, dtype=float)
    values = np.asarray(values, dtype=float)
    result = np.full(values.shape, np.nan, dtype=float)
    if len(values) < polyorder + 2:
        return result
    for part in split_continuous(time_s):
        t = time_s[part]
        y = values[part]
        valid = np.isfinite(t) & np.isfinite(y)
        if np.count_nonzero(valid) < polyorder + 2:
            continue
        tv = t[valid]
        yv = y[valid]
        dt = float(np.median(np.diff(tv)))
        if not math.isfinite(dt) or dt <= 0:
            continue
        window = int(round(window_s / dt))
        window = max(window, polyorder + 2)
        if window % 2 == 0:
            window += 1
        if window > len(yv):
            window = len(yv) if len(yv) % 2 else len(yv) - 1
        if window <= polyorder:
            continue
        grid_count = max(polyorder + 2, int(round((tv[-1] - tv[0]) / dt)) + 1)
        grid = np.linspace(tv[0], tv[-1], grid_count)
        grid_dt = float(grid[1] - grid[0])
        grid_values = np.interp(grid, tv, yv)
        if window > len(grid_values):
            window = len(grid_values) if len(grid_values) % 2 else len(grid_values) - 1
        if window <= polyorder:
            continue
        grid_derivative = signal.savgol_filter(
            grid_values,
            window_length=window,
            polyorder=min(polyorder, window - 1),
            deriv=order,
            delta=grid_dt,
            mode="interp",
        )
        filtered = np.interp(tv, grid, grid_derivative)
        local = np.full(len(t), np.nan)
        local[np.flatnonzero(valid)] = filtered
        result[part] = local
    return result


def uniform_resample(
    time_s: np.ndarray, values: np.ndarray
) -> tuple[np.ndarray, np.ndarray, float] | None:
    time_s = np.asarray(time_s, dtype=float)
    values = np.asarray(values, dtype=float)
    valid = np.isfinite(time_s) & np.isfinite(values)
    if np.count_nonzero(valid) < 8:
        return None
    t = time_s[valid]
    y = values[valid]
    if np.any(np.diff(t) <= 0):
        return None
    dt = float(np.median(np.diff(t)))
    if not math.isfinite(dt) or dt <= 0:
        return None
    grid = np.arange(t[0], t[-1] + 0.5 * dt, dt)
    if len(grid) < 8:
        return None
    return grid, np.interp(grid, t, y), dt


def frequency_metrics(time_s: np.ndarray, values: np.ndarray) -> dict:
    sampled = uniform_resample(time_s, values)
    if sampled is None:
        return {"available": False, "reason": "fewer than 8 finite monotonic samples"}
    grid, y, dt = sampled
    if len(y) < 32:
        return {"available": False, "reason": "window shorter than 32 resampled samples"}
    if float(np.std(y)) <= np.finfo(float).eps:
        return {"available": False, "reason": "signal has no measurable variance"}
    y = signal.detrend(y)
    fs = 1.0 / dt
    nperseg = min(1024, len(y))
    if nperseg < 32:
        return {"available": False, "reason": "insufficient Welch window"}
    frequencies, psd = signal.welch(
        y,
        fs=fs,
        window="hann",
        nperseg=nperseg,
        noverlap=nperseg // 2,
        detrend="constant",
        scaling="density",
    )
    positive = frequencies > 0
    if not np.any(positive):
        return {"available": False, "reason": "no positive frequency bins"}
    peak_index = np.flatnonzero(positive)[int(np.argmax(psd[positive]))]
    fft_values = np.fft.rfft(y * signal.windows.hann(len(y), sym=False))
    fft_frequency = np.fft.rfftfreq(len(y), d=dt)
    fft_amplitude = 2.0 * np.abs(fft_values) / max(1, len(y))
    stft_nperseg = min(256, len(y))
    stft_frequency, stft_time, stft_values = signal.stft(
        y,
        fs=fs,
        window="hann",
        nperseg=stft_nperseg,
        noverlap=stft_nperseg // 2,
        boundary=None,
    )
    stft_power = np.abs(stft_values) ** 2
    stft_peak_track = (
        stft_frequency[np.argmax(stft_power[1:, :], axis=0) + 1]
        if stft_power.shape[0] > 1 and stft_power.shape[1] > 0
        else np.array([], dtype=float)
    )
    return {
        "available": True,
        "sample_rate_Hz": fs,
        "nyquist_Hz": fs / 2.0,
        "resample_dt_s": dt,
        "resample_points": len(grid),
        "welch_window": "hann",
        "welch_nperseg": nperseg,
        "welch_overlap": nperseg // 2,
        "resolution_Hz": fs / nperseg,
        "dominant_frequency_Hz": float(frequencies[peak_index]),
        "dominant_psd": float(psd[peak_index]),
        "total_power": float(np.trapezoid(psd, frequencies)),
        "frequency_Hz": frequencies.tolist(),
        "psd": psd.tolist(),
        "fft": {
            "window": "hann",
            "frequency_Hz": fft_frequency.tolist(),
            "amplitude": fft_amplitude.tolist(),
        },
        "stft": {
            "window": "hann",
            "nperseg": stft_nperseg,
            "time_s": stft_time.tolist(),
            "dominant_frequency_track_Hz": stft_peak_track.tolist(),
            "note": "compact peak track; full complex STFT is intentionally not embedded in metrics.json",
        },
    }


def cross_spectral_metrics(
    time_s: np.ndarray, input_values: np.ndarray, output_values: np.ndarray
) -> dict:
    input_sampled = uniform_resample(time_s, input_values)
    output_sampled = uniform_resample(time_s, output_values)
    if input_sampled is None or output_sampled is None:
        return {"available": False, "reason": "insufficient monotonic finite paired samples"}
    grid_in, x, dt_in = input_sampled
    grid_out, y, dt_out = output_sampled
    dt = max(dt_in, dt_out)
    start = max(grid_in[0], grid_out[0])
    end = min(grid_in[-1], grid_out[-1])
    grid = np.arange(start, end + 0.5 * dt, dt)
    if len(grid) < 32:
        return {"available": False, "reason": "paired window shorter than 32 samples"}
    x = np.interp(grid, grid_in, x)
    y = np.interp(grid, grid_out, y)
    if float(np.std(x)) <= np.finfo(float).eps or float(np.std(y)) <= np.finfo(float).eps:
        return {
            "available": False,
            "reason": "input or output has no measurable variance for coherence",
        }
    fs = 1.0 / dt
    nperseg = min(512, len(grid))
    frequency, coherence = signal.coherence(
        x,
        y,
        fs=fs,
        window="hann",
        nperseg=nperseg,
        noverlap=nperseg // 2,
    )
    _, cross = signal.csd(
        x,
        y,
        fs=fs,
        window="hann",
        nperseg=nperseg,
        noverlap=nperseg // 2,
    )
    peak = int(np.argmax(coherence[1:]) + 1) if len(coherence) > 1 else 0
    return {
        "available": True,
        "frequency_Hz": frequency.tolist(),
        "coherence": coherence.tolist(),
        "cross_power_magnitude": np.abs(cross).tolist(),
        "cross_phase_rad": np.angle(cross).tolist(),
        "max_coherence_frequency_Hz": float(frequency[peak]),
        "max_coherence": float(coherence[peak]),
        "restriction": "observational coherence is not a closed-loop FRF without a known excitation",
    }


def periodic_fourier_series(
    time_s: np.ndarray,
    values: np.ndarray,
    harmonics: int = 8,
) -> dict:
    time_s = np.asarray(time_s, dtype=float)
    values = np.asarray(values, dtype=float)
    valid = np.isfinite(time_s) & np.isfinite(values)
    if np.count_nonzero(valid) < 4 * harmonics + 1:
        return {"available": False, "reason": "insufficient samples for requested harmonics"}
    t = time_s[valid]
    y = values[valid]
    period = float(t[-1] - t[0])
    if period <= 0:
        return {"available": False, "reason": "non-positive analysis period"}
    phase = 2 * np.pi * (t - t[0]) / period
    columns = [np.ones_like(phase)]
    names = ["a0"]
    for harmonic in range(1, harmonics + 1):
        columns.extend([np.cos(harmonic * phase), np.sin(harmonic * phase)])
        names.extend([f"a{harmonic}", f"b{harmonic}"])
    matrix = np.column_stack(columns)
    coefficients, _, rank, _ = np.linalg.lstsq(matrix, y, rcond=None)
    if rank != matrix.shape[1]:
        return {"available": False, "reason": "Fourier design matrix is rank deficient"}
    reconstructed = matrix @ coefficients
    residual = y - reconstructed
    return {
        "available": True,
        "period_s": period,
        "harmonics": harmonics,
        "coefficients": {
            name: float(value) for name, value in zip(names, coefficients, strict=True)
        },
        "reconstruction_rmse": float(np.sqrt(np.mean(residual**2))),
        "restriction": "valid only when the selected run represents one closed periodic cycle",
    }


def filter_ab(time_s: np.ndarray, values: np.ndarray) -> dict:
    sampled = uniform_resample(time_s, values)
    if sampled is None:
        return {"available": False, "reason": "insufficient finite monotonic samples"}
    _, y, dt = sampled
    if len(y) < 11:
        return {"available": False, "reason": "insufficient samples"}
    alpha = 0.25
    ema = np.empty_like(y)
    ema[0] = y[0]
    for index in range(1, len(y)):
        ema[index] = alpha * y[index] + (1.0 - alpha) * ema[index - 1]
    ma_window = min(9, len(y) if len(y) % 2 else len(y) - 1)
    moving = np.convolve(y, np.ones(ma_window) / ma_window, mode="same")
    sg_window = min(11, len(y) if len(y) % 2 else len(y) - 1)
    savgol = signal.savgol_filter(y, sg_window, 3, mode="interp")

    def item(filtered: np.ndarray, causal: bool, delay_s: float) -> dict:
        residual = y - filtered
        return {
            "noise_residual_rms": float(np.sqrt(np.mean(residual**2))),
            "peak_suppression": float(np.max(np.abs(y)) - np.max(np.abs(filtered))),
            "estimated_group_delay_s": delay_s,
            "causal_realtime_candidate": causal,
        }

    return {
        "available": True,
        "sample_dt_s": dt,
        "raw_rms": float(np.sqrt(np.mean(y**2))),
        "ema_alpha_0.25": item(ema, True, ((1.0 - alpha) / alpha) * dt),
        f"moving_average_{ma_window}": item(moving, True, (ma_window - 1) * dt / 2),
        f"savgol_{sg_window}_poly3_offline": item(savgol, False, 0.0),
        "note": "offline Savitzky-Golay output is not a deployable zero-phase real-time filter",
    }
