from __future__ import annotations

import numpy as np


def fit_feedforward(
    voltage_V: np.ndarray,
    velocity_mps: np.ndarray,
    acceleration_mps2: np.ndarray,
    training_mask: np.ndarray,
    validation_mask: np.ndarray,
) -> dict:
    voltage = np.asarray(voltage_V, dtype=float)
    velocity = np.asarray(velocity_mps, dtype=float)
    acceleration = np.asarray(acceleration_mps2, dtype=float)
    finite = np.isfinite(voltage) & np.isfinite(velocity) & np.isfinite(acceleration)
    train = finite & np.asarray(training_mask, dtype=bool) & (np.abs(velocity) > 1e-4)
    validate = finite & np.asarray(validation_mask, dtype=bool)
    if np.count_nonzero(train) < 12 or np.count_nonzero(validate) < 4:
        return {
            "available": False,
            "reason": "training/validation split does not contain enough valid linear-velocity samples",
        }
    matrix = np.column_stack(
        [np.sign(velocity[train]), velocity[train], acceleration[train]]
    )
    coefficients, _, rank, _ = np.linalg.lstsq(matrix, voltage[train], rcond=None)
    if rank < 3:
        return {"available": False, "reason": "SysId design matrix is rank deficient"}

    def metrics(mask: np.ndarray) -> dict:
        x = np.column_stack(
            [np.sign(velocity[mask]), velocity[mask], acceleration[mask]]
        )
        predicted = x @ coefficients
        residual = voltage[mask] - predicted
        ss_res = float(np.sum(residual**2))
        centered = voltage[mask] - float(np.mean(voltage[mask]))
        ss_tot = float(np.sum(centered**2))
        return {
            "n": int(np.count_nonzero(mask)),
            "rmse_V": float(np.sqrt(np.mean(residual**2))),
            "mae_V": float(np.mean(np.abs(residual))),
            "max_abs_residual_V": float(np.max(np.abs(residual))),
            "r2": 1.0 - ss_res / ss_tot if ss_tot > 0 else None,
        }

    return {
        "available": True,
        "status": "Draft",
        "model": "V = kS*sign(v) + kV*v + kA*a",
        "units": {"kS": "V", "kV": "V*s/m", "kA": "V*s^2/m"},
        "kS_V": float(coefficients[0]),
        "kV_Vs_per_m": float(coefficients[1]),
        "kA_Vs2_per_m": float(coefficients[2]),
        "training": metrics(train),
        "validation": metrics(validate),
        "restriction": "candidate only; never written to robot configuration automatically",
    }
