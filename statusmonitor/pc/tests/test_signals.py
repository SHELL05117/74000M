from __future__ import annotations

import numpy as np

from statusmonitor.analysis.signals import (
    cross_spectral_metrics,
    filter_ab,
    frequency_metrics,
    periodic_fourier_series,
    robust_derivative,
)
from statusmonitor.analysis.sysid import fit_feedforward
from statusmonitor.segmentation import segment_motion


def test_nonuniform_derivative_uses_real_timestamps():
    rng = np.random.default_rng(42)
    dt = 0.01 + rng.normal(0, 0.0003, 1000)
    t = np.cumsum(dt)
    y = 2.0 * t**2 + 3.0 * t + 1.0
    derivative = robust_derivative(t, y, order=1, window_s=0.11, polyorder=3)
    valid = np.isfinite(derivative)
    assert np.max(np.abs(derivative[valid] - (4 * t[valid] + 3))) < 0.05


def test_welch_recovers_known_frequency():
    t = np.arange(0, 10, 0.01)
    y = np.sin(2 * np.pi * 4.8 * t)
    result = frequency_metrics(t, y)
    assert result["available"]
    assert abs(result["dominant_frequency_Hz"] - 4.8) < result["resolution_Hz"] * 1.5
    assert np.isclose(result["nyquist_Hz"], 50.0)


def test_filter_ab_reports_causality():
    t = np.arange(0, 3, 0.01)
    y = np.sin(2 * np.pi * t) + 0.1 * np.sin(2 * np.pi * 20 * t)
    result = filter_ab(t, y)
    assert result["available"]
    assert result["ema_alpha_0.25"]["causal_realtime_candidate"]
    assert not result["savgol_11_poly3_offline"]["causal_realtime_candidate"]


def test_cross_spectral_and_periodic_fourier():
    t = np.arange(0, 8, 0.01)
    command = np.sin(2 * np.pi * 2.0 * t)
    response = np.sin(2 * np.pi * 2.0 * t - 0.3)
    cross = cross_spectral_metrics(t, command, response)
    assert cross["available"]
    assert cross["max_coherence"] > 0.95
    periodic_t = np.linspace(0, 1, 1001)
    periodic_y = 0.7 * np.sin(2 * np.pi * periodic_t) + 0.2 * np.cos(
        4 * np.pi * periodic_t
    )
    fourier = periodic_fourier_series(periodic_t, periodic_y, harmonics=4)
    assert fourier["available"]
    assert fourier["reconstruction_rmse"] < 0.01


def test_segmentation_finds_active_window():
    t = np.arange(0, 3, 0.01)
    voltage = np.zeros_like(t)
    voltage[100:200] = 4
    segments = segment_motion(t, voltage, voltage)
    assert any(segment.label == "Active" for segment in segments)


def test_sysid_training_validation_fixed_vector():
    rng = np.random.default_rng(7)
    velocity = np.r_[np.linspace(-2, -0.1, 200), np.linspace(0.1, 2, 200)]
    acceleration = rng.normal(0, 1.0, len(velocity))
    voltage = 0.4 * np.sign(velocity) + 2.2 * velocity + 0.3 * acceleration
    train = np.arange(len(velocity)) % 4 != 0
    validate = ~train
    result = fit_feedforward(voltage, velocity, acceleration, train, validate)
    assert result["available"]
    assert abs(result["kS_V"] - 0.4) < 1e-10
    assert abs(result["kV_Vs_per_m"] - 2.2) < 1e-10
    assert abs(result["kA_Vs2_per_m"] - 0.3) < 1e-10
