"""Step-response analysis for the temperature control test stand.

Reads CSV files written by the sensor-data-logger toolkit
(columns: time_iso, elapsed_s, temperature_c, humidity_pct). On this stand the
firmware puts the heater duty (0..100 %) in the second channel, so
``humidity_pct`` is read as duty.

Two commands:

  openloop  Fit a first-order-plus-dead-time (FOPDT) model to an open-loop step
            and suggest starting PI gains (SIMC rules).
              python analysis/step_response.py openloop data/openloop.csv

  closed    Report rise time, overshoot, settling time and steady-state error of
            one or more closed-loop runs, and plot them together.
              python analysis/step_response.py closed data/onoff.csv data/pid.csv \
                  --setpoint 50 --label on/off --label PID --save results/step.png

The step is taken to start at the first sample where the duty becomes > 0 (the
firmware keeps the heater off during its baseline phase), unless --step-time is given.
All numbers this tool prints come from the CSV you give it; it does not invent or
smooth data.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence

import numpy as np


# --------------------------------------------------------------------------- loading

@dataclass
class Run:
    t: np.ndarray        # elapsed seconds since logger start
    temp: np.ndarray     # C
    duty: np.ndarray     # %
    path: str = ""

    def __len__(self) -> int:
        return len(self.t)


def load_run(path: str | Path) -> Run:
    t, temp, duty = [], [], []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        needed = {"elapsed_s", "temperature_c", "humidity_pct"}
        if reader.fieldnames is None or not needed <= set(reader.fieldnames):
            raise ValueError(f"{path}: expected columns {sorted(needed)}, got {reader.fieldnames}")
        for row in reader:
            t.append(float(row["elapsed_s"]))
            temp.append(float(row["temperature_c"]))
            duty.append(float(row["humidity_pct"]))
    if len(t) < 10:
        raise ValueError(f"{path}: only {len(t)} samples, need at least 10")
    return Run(np.array(t), np.array(temp), np.array(duty), str(path))


def find_step(run: Run, step_time: Optional[float] = None) -> int:
    """Index of the first sample of the step."""
    if step_time is not None:
        idx = int(np.searchsorted(run.t, step_time))
        if idx >= len(run):
            raise ValueError(f"step time {step_time} s is after the end of the log")
        return idx
    on = np.nonzero(run.duty > 0.0)[0]
    if len(on) == 0:
        raise ValueError(f"{run.path}: duty is never > 0, no step found (use --step-time)")
    return int(on[0])


def baseline(run: Run, i0: int) -> tuple[float, bool]:
    """Mean temperature before the step, and whether real baseline samples existed."""
    if i0 >= 3:
        return float(np.mean(run.temp[:i0])), True
    return float(run.temp[i0]), False


def data_quality(run: Run) -> dict:
    dt = np.diff(run.t)
    return {
        "samples": len(run),
        "duration_s": float(run.t[-1] - run.t[0]),
        "median_dt_s": float(np.median(dt)),
        "max_gap_s": float(np.max(dt)),
    }


# --------------------------------------------------------------------------- open loop

@dataclass
class FopdtFit:
    gain_c_per_pct: float   # K
    tau_s: float            # time constant
    dead_time_s: float      # theta
    t0_c: float             # baseline temperature
    final_c: float          # plateau temperature
    duty_step_pct: float
    plateau_slope_c_per_min: float  # slope over the last part of the run: ~0 if it settled
    settled: bool


def fit_fopdt(run: Run, step_time: Optional[float] = None, tail_fraction: float = 0.1,
              plateau_tol_c_per_min: float = 0.05) -> FopdtFit:
    """Two-point (Smith) FOPDT fit from the 28.3 % and 63.2 % crossing times.

    The plateau is the mean of the last ``tail_fraction`` of the post-step data. If the
    temperature is still rising there, the fit underestimates K and tau; ``settled``
    is then False.
    """
    i0 = find_step(run, step_time)
    t_step = run.t[i0]
    T0, _ = baseline(run, i0)
    du = float(np.median(run.duty[i0:]))
    if du <= 0:
        raise ValueError("duty after the step is 0")

    t = run.t[i0:] - t_step
    T = run.temp[i0:]
    n_tail = max(5, int(len(T) * tail_fraction))
    final = float(np.mean(T[-n_tail:]))
    slope = float(np.polyfit(t[-n_tail:], T[-n_tail:], 1)[0]) * 60.0
    dT = final - T0
    if dT <= 0:
        raise ValueError(f"temperature did not rise (start {T0:.2f} C, end {final:.2f} C)")

    def crossing(frac: float) -> float:
        level = T0 + frac * dT
        k = int(np.argmax(T >= level))
        if T[k] < level:
            raise ValueError(f"never reached {frac:.1%} of the step")
        if k == 0:
            return float(t[0])
        # linear interpolation between samples
        return float(t[k - 1] + (level - T[k - 1]) * (t[k] - t[k - 1]) / (T[k] - T[k - 1]))

    t28 = crossing(0.283)
    t63 = crossing(0.632)
    tau = 1.5 * (t63 - t28)
    theta = max(0.0, t63 - tau)
    return FopdtFit(dT / du, tau, theta, T0, final, du, slope, abs(slope) <= plateau_tol_c_per_min)


def simc_pi(fit: FopdtFit, tau_c: Optional[float] = None) -> tuple[float, float, float]:
    """SIMC PI tuning (Skogestad 2003). Returns (Kp, Ki, tau_c) in firmware units:
    Kp in %/C, Ki in %/(C*s). Default tau_c = max(theta, tau/10): a reasonable,
    not aggressive, starting point."""
    if tau_c is None:
        tau_c = max(fit.dead_time_s, fit.tau_s / 10.0)
    kp = fit.tau_s / (fit.gain_c_per_pct * (tau_c + fit.dead_time_s))
    ti = min(fit.tau_s, 4.0 * (tau_c + fit.dead_time_s))
    return kp, kp / ti, tau_c


def fopdt_model(fit: FopdtFit, t: np.ndarray) -> np.ndarray:
    y = np.where(t < fit.dead_time_s, 0.0, 1.0 - np.exp(-(t - fit.dead_time_s) / fit.tau_s))
    return fit.t0_c + fit.gain_c_per_pct * fit.duty_step_pct * y


# --------------------------------------------------------------------------- closed loop

@dataclass
class StepMetrics:
    setpoint_c: float
    t0_c: float
    has_baseline: bool
    rise_time_s: Optional[float]       # 10 % -> 90 % of the step
    peak_c: float
    overshoot_c: float
    overshoot_pct: float
    settling_time_s: Optional[float]   # time after the step until it stays within the band
    band_c: float
    ss_window_s: float
    ss_mean_error_c: Optional[float]   # setpoint - mean temperature in the final window
    ss_peak_to_peak_c: Optional[float]
    ss_mean_duty_pct: Optional[float]


def step_metrics(run: Run, setpoint: float, band: float = 1.0, ss_window: float = 120.0,
                 step_time: Optional[float] = None) -> StepMetrics:
    i0 = find_step(run, step_time)
    T0, has_base = baseline(run, i0)
    t = run.t[i0:] - run.t[i0]
    T = run.temp[i0:]
    D = run.duty[i0:]
    step = setpoint - T0
    if step <= 0:
        raise ValueError(f"setpoint {setpoint} C is not above the start temperature {T0:.2f} C")

    def first_time(level: float) -> Optional[float]:
        k = np.nonzero(T >= level)[0]
        return float(t[k[0]]) if len(k) else None

    t10, t90 = first_time(T0 + 0.1 * step), first_time(T0 + 0.9 * step)
    rise = t90 - t10 if t10 is not None and t90 is not None else None

    peak = float(np.max(T))
    over = max(0.0, peak - setpoint)

    outside = np.abs(T - setpoint) > band
    if outside[-1]:
        settling = None  # not inside the band at the end of the log
    elif not outside.any():
        settling = 0.0
    else:
        last_out = int(np.nonzero(outside)[0][-1])
        settling = float(t[last_out + 1])

    ss = t >= (t[-1] - ss_window)
    if settling is not None and t[-1] - settling >= ss_window and ss.sum() >= 5:
        err = float(setpoint - np.mean(T[ss]))
        p2p = float(np.ptp(T[ss]))
        duty = float(np.mean(D[ss]))
    else:
        err = p2p = duty = None  # not enough settled data for a steady-state figure

    return StepMetrics(setpoint, T0, has_base, rise, peak, over, 100.0 * over / step, settling,
                       band, ss_window, err, p2p, duty)


# --------------------------------------------------------------------------- reporting

def _fmt(x: Optional[float], unit: str, digits: int = 1) -> str:
    return "n/a" if x is None else f"{x:.{digits}f} {unit}"


def report_quality(run: Run) -> str:
    q = data_quality(run)
    line = (f"  data: {q['samples']} samples over {q['duration_s']:.0f} s, "
            f"median interval {q['median_dt_s']:.3f} s, largest gap {q['max_gap_s']:.2f} s")
    if q["max_gap_s"] > 5 * q["median_dt_s"]:
        line += "  (WARNING: gaps in the log)"
    return line


def report_closed(m: StepMetrics, label: str) -> str:
    lines = [f"{label}",
             f"  start temperature   {m.t0_c:.2f} C" + ("" if m.has_baseline else "  (no baseline samples)"),
             f"  setpoint            {m.setpoint_c:.2f} C",
             f"  rise time 10-90 %   {_fmt(m.rise_time_s, 's')}",
             f"  peak                {m.peak_c:.2f} C",
             f"  overshoot           {m.overshoot_c:.2f} C ({m.overshoot_pct:.1f} % of step)",
             f"  settling (+/-{m.band_c:g} C)  "
             + (_fmt(m.settling_time_s, "s") if m.settling_time_s is not None else "not settled by end of log")]
    if m.ss_mean_error_c is not None:
        lines += [f"  steady state (last {m.ss_window_s:.0f} s): error {m.ss_mean_error_c:+.2f} C, "
                  f"peak-to-peak {m.ss_peak_to_peak_c:.2f} C, mean duty {m.ss_mean_duty_pct:.1f} %"]
    else:
        lines += [f"  steady state: not enough settled data (need {m.ss_window_s:.0f} s inside the band)"]
    return "\n".join(lines)


def report_fopdt(fit: FopdtFit) -> str:
    kp, ki, tau_c = simc_pi(fit)
    lines = [f"  duty step           {fit.duty_step_pct:.1f} %",
             f"  temperature         {fit.t0_c:.2f} C -> {fit.final_c:.2f} C",
             f"  gain K              {fit.gain_c_per_pct:.4f} C per % duty",
             f"  time constant tau   {fit.tau_s:.1f} s",
             f"  dead time theta     {fit.dead_time_s:.1f} s",
             f"  end-of-run slope    {fit.plateau_slope_c_per_min:+.3f} C/min",
             "",
             f"  SIMC PI starting point (tau_c = {tau_c:.1f} s):",
             f"    -DPID_KP={kp:.3f}f -DPID_KI={ki:.5f}f -DPID_KD=0.0f",
             f"  Expected steady duty at 50 C: {(50 - fit.t0_c) / fit.gain_c_per_pct:.0f} % "
             f"(only valid if this is < 100 %)"]
    if not fit.settled:
        lines.insert(0, "  WARNING: temperature was still changing at the end of the run, so K and tau "
                        "are underestimated. Run longer (raise MAX_RUN_S) or use a smaller duty step.")
    return "\n".join(lines)


# --------------------------------------------------------------------------- plotting

def plot_closed(runs: Sequence[Run], labels: Sequence[str], setpoint: float, band: float,
                step_time: Optional[float], save: Optional[str], title: Optional[str]) -> None:
    import matplotlib
    if save:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, (ax_t, ax_d) = plt.subplots(2, 1, sharex=True, figsize=(9, 6),
                                     gridspec_kw={"height_ratios": [3, 1]})
    for run, label in zip(runs, labels):
        i0 = find_step(run, step_time)
        t = run.t - run.t[i0]
        ax_t.plot(t, run.temp, lw=1.4, label=label)
        ax_d.plot(t, run.duty, lw=1.0, label=label)
    ax_t.axhline(setpoint, color="0.3", ls="--", lw=1, label=f"setpoint {setpoint:g} C")
    ax_t.axhspan(setpoint - band, setpoint + band, color="0.5", alpha=0.12, lw=0,
                 label=f"+/-{band:g} C band")
    ax_t.axvline(0, color="0.6", lw=0.8)
    ax_d.axvline(0, color="0.6", lw=0.8)
    ax_t.set_ylabel("Temperature (C)")
    ax_d.set_ylabel("Heater duty (%)")
    ax_d.set_xlabel("Time since step (s)")
    ax_d.set_ylim(-5, 105)
    ax_t.grid(alpha=0.3)
    ax_d.grid(alpha=0.3)
    ax_t.legend(loc="lower right", fontsize=9)
    ax_t.set_title(title or "Closed-loop step response (measured)")
    fig.tight_layout()
    if save:
        Path(save).parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(save, dpi=150)
        print(f"saved {save}")
    else:
        plt.show()


def plot_openloop(run: Run, fit: FopdtFit, step_time: Optional[float], save: Optional[str],
                  title: Optional[str]) -> None:
    import matplotlib
    if save:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    i0 = find_step(run, step_time)
    t = run.t - run.t[i0]
    tm = np.linspace(0, t[-1], 500)
    fig, (ax_t, ax_d) = plt.subplots(2, 1, sharex=True, figsize=(9, 6),
                                     gridspec_kw={"height_ratios": [3, 1]})
    ax_t.plot(t, run.temp, lw=1.4, label="measured")
    ax_t.plot(tm, fopdt_model(fit, tm), "--", lw=1.2,
              label=f"FOPDT fit: K={fit.gain_c_per_pct:.3f} C/%, tau={fit.tau_s:.0f} s, "
                    f"theta={fit.dead_time_s:.0f} s")
    ax_d.plot(t, run.duty, lw=1.0)
    ax_t.set_ylabel("Temperature (C)")
    ax_d.set_ylabel("Heater duty (%)")
    ax_d.set_xlabel("Time since step (s)")
    ax_d.set_ylim(-5, 105)
    ax_t.grid(alpha=0.3)
    ax_d.grid(alpha=0.3)
    ax_t.legend(loc="lower right", fontsize=9)
    ax_t.set_title(title or "Open-loop step response (measured) with model fit")
    fig.tight_layout()
    if save:
        Path(save).parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(save, dpi=150)
        print(f"saved {save}")
    else:
        plt.show()


# --------------------------------------------------------------------------- CLI

def main(argv: Optional[Sequence[str]] = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = p.add_subparsers(dest="cmd", required=True)

    po = sub.add_parser("openloop", help="fit a FOPDT model and suggest PI gains")
    po.add_argument("csv")
    po.add_argument("--step-time", type=float, help="step start in logger seconds (default: first duty > 0)")
    po.add_argument("--save", help="save the plot to this PNG instead of showing it")
    po.add_argument("--no-plot", action="store_true")
    po.add_argument("--title")

    pc = sub.add_parser("closed", help="step-response metrics for closed-loop runs")
    pc.add_argument("csv", nargs="+")
    pc.add_argument("--setpoint", type=float, required=True, help="setpoint used in the firmware (C)")
    pc.add_argument("--band", type=float, default=1.0, help="settling band +/- C (default 1.0)")
    pc.add_argument("--ss-window", type=float, default=120.0, help="steady-state window at the end (s)")
    pc.add_argument("--label", action="append", help="legend label per CSV (repeat)")
    pc.add_argument("--step-time", type=float)
    pc.add_argument("--save")
    pc.add_argument("--no-plot", action="store_true")
    pc.add_argument("--title")

    a = p.parse_args(argv)
    try:
        if a.cmd == "openloop":
            run = load_run(a.csv)
            fit = fit_fopdt(run, a.step_time)
            print(a.csv)
            print(report_quality(run))
            print(report_fopdt(fit))
            if not a.no_plot:
                plot_openloop(run, fit, a.step_time, a.save, a.title)
        else:
            runs = [load_run(c) for c in a.csv]
            labels = a.label or [Path(c).stem for c in a.csv]
            if len(labels) != len(runs):
                p.error("give one --label per CSV")
            for run, label in zip(runs, labels):
                m = step_metrics(run, a.setpoint, a.band, a.ss_window, a.step_time)
                print(report_closed(m, f"{label}  ({run.path})"))
                print(report_quality(run))
                print()
            if not a.no_plot:
                plot_closed(runs, labels, a.setpoint, a.band, a.step_time, a.save, a.title)
    except ValueError as err:
        print(f"error: {err}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
