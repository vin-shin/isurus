#!/usr/bin/env python3
"""Sweep a MODEL parameter against a hardware A/B and plot where they agree.

    tools/param_sensitivity.py [-o outdir]

The question this answers is not "does the simulator look like the bench" - a
single trace can be made to look like anything - but "which value of a motor
parameter makes the simulator behave like the bench across a whole A/B".

It sweeps the model's inductance while leaving FOC_LD_H / FOC_LQ_H alone, so
the firmware keeps designing its gains and its w_e*Lq*iq feedforward for the
value it was given while the plant behaves like a motor that is wrong by that
factor. That is exactly the error the HV build is exposed to: at 917 Hz and
300 A the feedforward term is 441 V of a 600 V bus, so being wrong about Lq is
not survivable there, and prompt.md section E names characterising it as the
largest open risk.

Why the DECOUPLING BENEFIT is the discriminator, and not the step response:

  - Rise time saturates. The control period is 33.3 us, so a loop that settles
    inside two periods reads as 67 us whether the true answer is 67 or 20.
    Both the bench and the model hit that floor, so it cannot separate them.
  - Peak |id| with the feedforward ON is not diagnostic on its own. It was
    matched to within 2 mA by a model whose inductance was 3x wrong, because
    a mis-tuned feedforward leaves a residue of about the size of a correct
    one. The number agreed while the mechanism did not.
  - The benefit - peak |id| OFF against peak |id| ON - is diagnostic, because
    it changes SIGN. With the right inductance the feedforward cancels the
    cross-coupling; with an inductance well below the assumed one it
    over-corrects and the "correction" makes the disturbance worse. No amount
    of gain error mimics that.

So the plot to read is the second panel: where the simulator's benefit crosses
the band the bench measured, that is the inductance the bench motor is acting
like.
"""

import argparse
import csv
import io
import os
import subprocess
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "tools"))
from viz import REPORT_STYLE, _provenance, _find_sim_dump      # noqa: E402

L_NOM_H = 54.3e-6          # FOC_L_H, what the firmware believes

# Measured on the bench 2026-08-25, tools/step_trace.sh, 12 reps per arm.
# (label, iq_hi_ma, w_e rad/s, peak|id| decouple=1, peak|id| decouple=0)
HW = [
    ("400 -> 1200 mA", 1200, 1669, 105.0, 245.0),
    ("400 -> 2400 mA", 2400, 1942, 106.0, 336.0),
]


def sim_peak_id(exe, hi_ma, we, decouple, l_err, lo_ma=400, ms=6.0):
    """Peak |id| in the 3 ms after the step, mA."""
    args = [exe, "step", str(lo_ma), str(hi_ma), str(we), str(ms),
            f"decouple={decouple}", "delay=1", f"L_err={l_err}"]
    r = subprocess.run(args, capture_output=True, text=True)
    if r.returncode != 0:
        sys.exit(f"sim_dump failed: {r.stderr[:400]}")
    rows = list(csv.DictReader(io.StringIO(r.stdout)))
    t = np.array([float(x["t_ms"]) for x in rows])
    idd = np.array([float(x["id_a"]) for x in rows])
    m = (t >= 0) & (t <= 3.0)
    return float(np.abs(idd[m]).max() * 1000.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-o", "--outdir", default=os.path.join(ROOT, "reports"))
    ap.add_argument("--dpi", type=int, default=140)
    a = ap.parse_args()
    os.makedirs(a.outdir, exist_ok=True)

    exe = _find_sim_dump()
    l_err = np.array([1.0, 1.25, 1.5, 1.75, 2.0, 2.5, 3.0, 4.0])
    l_uh = L_NOM_H / l_err * 1e6

    plt.rcParams.update(REPORT_STYLE)
    fig, ax = plt.subplots(2, 1, figsize=(9.5, 8.0), sharex=True)

    colors = ["#1f77b4", "#d62728"]
    for (label, hi, we, hw_on, hw_off), c in zip(HW, colors):
        on = np.array([sim_peak_id(exe, hi, we, 1, e) for e in l_err])
        off = np.array([sim_peak_id(exe, hi, we, 0, e) for e in l_err])
        benefit = (on - off) / off * 100.0
        hw_benefit = (hw_on - hw_off) / hw_off * 100.0

        ax[0].plot(l_uh, on, "-o", color=c, ms=3.5, label=f"sim, {label}")
        ax[0].axhline(hw_on, color=c, ls=":", lw=1.2)

        ax[1].plot(l_uh, benefit, "-o", color=c, ms=3.5, label=f"sim, {label}")
        ax[1].axhline(hw_benefit, color=c, ls=":", lw=1.2)
        ax[1].annotate(f"bench {hw_benefit:+.0f}%", (57.5, hw_benefit),
                       xytext=(0, 4), textcoords="offset points",
                       ha="center", fontsize=8, color=c)

    # Both bench values land within 1 mA of each other, so one label serves
    # for the pair; two would just overprint.
    ax[0].annotate(f"bench {HW[0][3]:.0f}-{HW[1][3]:.0f} mA", (57.5, HW[0][3]),
                   xytext=(0, 6), textcoords="offset points",
                   ha="center", fontsize=8, color="0.25")
    ax[0].set_ylabel("peak |id| with decoupling ON  [mA]")
    ax[0].set_title("model inductance vs the bench - which L does the motor act like?")
    ax[0].legend(loc="upper center")
    ax[0].set_xlim(11, 62)
    ax[0].grid(alpha=0.3)
    ax[0].annotate("matched to 2 mA by a 3x-wrong model:\n"
                   "agreement here means nothing on its own",
                   xy=(18.1, 103), xytext=(23, 210), fontsize=8,
                   arrowprops=dict(arrowstyle="->", lw=0.8, color="0.4"),
                   color="0.35")

    ax[1].axhline(0, color="0.3", lw=1.0)
    ax[1].set_ylabel("decoupling benefit  [% change in peak |id|]")
    ax[1].set_xlabel("model inductance  [uH]      (firmware assumes 54.3)")
    ax[1].legend(loc="lower left")
    ax[1].grid(alpha=0.3)
    ax[1].annotate("below ~30 uH the feedforward OVER-corrects\n"
                   "and the benefit changes sign - the bench never does this",
                   xy=(20, 78), xytext=(27, 108), fontsize=8,
                   arrowprops=dict(arrowstyle="->", lw=0.8, color="0.4"),
                   color="0.35")
    ax[1].axvline(L_NOM_H * 1e6, color="0.5", ls="--", lw=1.0)
    ax[1].annotate("FOC_L_H = 54.3 uH", (L_NOM_H * 1e6, 20),
                   xytext=(-6, 0), textcoords="offset points",
                   ha="right", va="center", fontsize=8, color="0.4")

    fig.text(0.01, 0.005, _provenance(), fontsize=8, color="0.45")
    out = os.path.join(a.outdir, "08-inductance-sensitivity.png")
    fig.savefig(out, dpi=a.dpi)
    print("wrote", out)

    with open(os.path.join(a.outdir, "08-inductance-sensitivity.csv"), "w",
              newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["step", "L_err", "model_L_uH", "peak_id_on_ma",
                    "peak_id_off_ma", "benefit_pct"])
        for label, hi, we, _, _ in HW:
            for e, lu in zip(l_err, l_uh):
                on = sim_peak_id(exe, hi, we, 1, e)
                off = sim_peak_id(exe, hi, we, 0, e)
                w.writerow([label, f"{e:g}", f"{lu:.2f}", f"{on:.1f}",
                            f"{off:.1f}", f"{(on - off) / off * 100:.1f}"])
    print("wrote", os.path.join(a.outdir, "08-inductance-sensitivity.csv"))


if __name__ == "__main__":
    main()
