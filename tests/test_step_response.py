"""Tests for analysis/step_response.py.

All data here is synthetic and generated inside the tests, only to check the maths
against cases with known answers. None of it is a measurement of the test stand.
"""

import csv
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "analysis"))
import step_response as sr  # noqa: E402


def write_csv(path, t, temp, duty):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["time_iso", "elapsed_s", "temperature_c", "humidity_pct"])
        for ti, Ti, di in zip(t, temp, duty):
            w.writerow(["2026-01-01T00:00:00.000", f"{ti:.3f}", f"{Ti:.2f}", f"{di:.1f}"])


def fopdt_series(K=0.5, tau=100.0, theta=15.0, du=30.0, T0=22.0, base=30.0, total=1500.0, dt=0.25):
    t = np.arange(0.0, total, dt)
    ts = t - base
    y = np.where(ts < theta, 0.0, 1.0 - np.exp(-(ts - theta) / tau))
    temp = T0 + K * du * y
    duty = np.where(t >= base, du, 0.0)
    return t, temp, duty


@pytest.mark.req("REQ-15")
def test_load_and_find_step(tmp_path):
    t, temp, duty = fopdt_series()
    f = tmp_path / "ol.csv"
    write_csv(f, t, temp, duty)
    run = sr.load_run(f)
    assert len(run) == len(t)
    i0 = sr.find_step(run)
    assert run.t[i0] == pytest.approx(30.0)
    assert sr.find_step(run, step_time=100.0) == int(np.searchsorted(run.t, 100.0))


@pytest.mark.req("REQ-16")
def test_load_rejects_wrong_columns(tmp_path):
    f = tmp_path / "bad.csv"
    f.write_text("a,b\n1,2\n")
    with pytest.raises(ValueError):
        sr.load_run(f)


@pytest.mark.req("REQ-15")
def test_fopdt_fit_recovers_known_model(tmp_path):
    t, temp, duty = fopdt_series(K=0.5, tau=100.0, theta=15.0, du=30.0)
    f = tmp_path / "ol.csv"
    write_csv(f, t, temp, duty)
    fit = sr.fit_fopdt(sr.load_run(f))
    assert fit.settled
    assert fit.gain_c_per_pct == pytest.approx(0.5, rel=0.01)
    assert fit.tau_s == pytest.approx(100.0, rel=0.03)
    assert fit.dead_time_s == pytest.approx(15.0, abs=1.5)


@pytest.mark.req("REQ-16")
def test_fopdt_fit_flags_unsettled_run(tmp_path):
    t, temp, duty = fopdt_series(tau=300.0, total=400.0)
    f = tmp_path / "short.csv"
    write_csv(f, t, temp, duty)
    fit = sr.fit_fopdt(sr.load_run(f))
    assert not fit.settled


@pytest.mark.req("REQ-15")
def test_simc_pi():
    fit = sr.FopdtFit(0.5, 100.0, 10.0, 22.0, 37.0, 30.0, 0.0, True)
    kp, ki, tau_c = sr.simc_pi(fit)
    assert tau_c == 10.0
    assert kp == pytest.approx(100.0 / (0.5 * 20.0))  # 10 %/C
    assert ki == pytest.approx(kp / 80.0)              # Ti = min(100, 4*20) = 80 s


def second_order_series(zeta=0.4, wn=0.03, T0=22.0, sp=50.0, base=30.0, total=1200.0, dt=0.25):
    t = np.arange(0.0, total, dt)
    ts = np.clip(t - base, 0, None)
    wd = wn * math.sqrt(1 - zeta ** 2)
    phi = math.acos(zeta)
    y = 1 - np.exp(-zeta * wn * ts) / math.sqrt(1 - zeta ** 2) * np.sin(wd * ts + phi)
    temp = T0 + (sp - T0) * y
    duty = np.where(t >= base, 50.0, 0.0)
    return t, temp, duty


@pytest.mark.req("REQ-15")
def test_closed_loop_metrics_match_theory(tmp_path):
    zeta, wn = 0.4, 0.03
    t, temp, duty = second_order_series(zeta, wn)
    f = tmp_path / "pid.csv"
    write_csv(f, t, temp, duty)
    m = sr.step_metrics(sr.load_run(f), setpoint=50.0, band=0.02 * 28.0)
    expected_os = math.exp(-zeta * math.pi / math.sqrt(1 - zeta ** 2)) * 100
    assert m.t0_c == pytest.approx(22.0, abs=0.01)
    assert m.overshoot_pct == pytest.approx(expected_os, abs=0.3)
    # Settling must come after the first peak (pi/wd) and no later than the time at which the
    # exponential envelope drops inside the 2 % band: -ln(0.02*sqrt(1-zeta^2)) / (zeta*wn)
    t_peak = math.pi / (wn * math.sqrt(1 - zeta ** 2))
    t_env = -math.log(0.02 * math.sqrt(1 - zeta ** 2)) / (zeta * wn)
    assert t_peak < m.settling_time_s <= t_env + 0.25
    assert m.ss_mean_error_c == pytest.approx(0.0, abs=0.05)
    assert m.rise_time_s is not None and m.rise_time_s > 0


@pytest.mark.req("REQ-16")
def test_not_settled_is_reported(tmp_path):
    t = np.arange(0, 300, 0.25)
    temp = 22 + 0.05 * t  # still climbing, never reaches the band around 50
    duty = np.where(t >= 30, 100.0, 0.0)
    f = tmp_path / "ramp.csv"
    write_csv(f, t, temp, duty)
    m = sr.step_metrics(sr.load_run(f), setpoint=50.0)
    assert m.settling_time_s is None
    assert m.ss_mean_error_c is None
    assert "not settled" in sr.report_closed(m, "ramp")


@pytest.mark.req("REQ-16")
def test_setpoint_below_start_is_an_error(tmp_path):
    t, temp, duty = second_order_series()
    f = tmp_path / "x.csv"
    write_csv(f, t, temp, duty)
    with pytest.raises(ValueError):
        sr.step_metrics(sr.load_run(f), setpoint=10.0)


@pytest.mark.req("REQ-17")
def test_cli_runs_and_saves_plot(tmp_path):
    t, temp, duty = second_order_series()
    f = tmp_path / "pid.csv"
    write_csv(f, t, temp, duty)
    out = tmp_path / "plot.png"
    assert sr.main(["closed", str(f), "--setpoint", "50", "--save", str(out)]) == 0
    assert out.exists() and out.stat().st_size > 1000

    t, temp, duty = fopdt_series()
    f2 = tmp_path / "ol.csv"
    write_csv(f2, t, temp, duty)
    assert sr.main(["openloop", str(f2), "--no-plot"]) == 0
