#!/usr/bin/env python3
"""Plot control-loop data from the simulator and from the board.

    tools/viz.py trace   <file> [--key K] [-o out.pdf] [--csv data.csv]
    tools/viz.py compare --hw <capture> --sim <csv> [--hw-key K] [-o out.pdf]
    tools/viz.py live    [--seconds N] [-o out.png]

Output format follows the extension. Use .pdf or .svg for anything going into
a document - they are vector and stay sharp at any size - and .png for chat or
an issue tracker. --csv writes the plotted numbers alongside, so a colleague
can re-plot rather than ask for the raw capture.

Every figure is stamped with the operating point and the repo revision it was
plotted from, marked +dirty if the tree had uncommitted changes. A plot sent
to someone else is close to worthless without that: "iq overshoots" is a
rumour until the reader can tell which firmware and which bus, and a figure
produced from uncommitted work is exactly the one that cannot be reproduced
later.

Two sources, one representation:

  * a HARDWARE capture, the raw text tools/step_trace.sh saves - 512 samples
    of id/iq/vd/vq at the full 30 kHz ISR rate;
  * a SIMULATOR run, the CSV test/host/sim_dump writes, from foc.c compiled
    natively against the PMSM model.

`compare` is the point of the whole thing. The simulator runs the same
firmware source against a model of the same machine, so laying the two over
each other answers the question neither can answer alone: when the bench does
something surprising, is it the firmware or is it the motor? A disagreement
localises the problem; agreement means the model can be trusted for the HV
work, where the bench cannot follow.

`live` deliberately does NOT energise anything. It polls the integer mirrors
and plots them; it never arms the bridge, never writes a setpoint, and is safe
to run against a drive that is sitting in READY. Use step_trace.sh when you
want a transient, because that one does spin the motor and says so.
"""

import argparse
import os
import re
import shutil
import subprocess
import sys
import tempfile

import numpy as np

import matplotlib
matplotlib.use("Agg")          # headless: this writes PNGs, it never opens a window
import matplotlib.pyplot as plt


FS_HZ = 30000.0                # control ISR rate; one trace sample per period
STEP_PRE_TICKS = 60            # firmware places the step here - see main.c

# Report styling. Defaults are tuned for a screen at 100 dpi and look thin and
# spidery once a figure is scaled into a document, so everything here is
# nudged up: a plot that has to be zoomed to read is a plot nobody reads.
REPORT_STYLE = {
    "font.size": 11,
    "axes.titlesize": 12,
    "axes.labelsize": 11,
    "legend.fontsize": 9,
    "xtick.labelsize": 10,
    "ytick.labelsize": 10,
    "axes.linewidth": 0.9,
    "lines.linewidth": 1.4,
    "figure.facecolor": "white",
    "savefig.facecolor": "white",
    "savefig.bbox": "tight",
}

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OPENOCD_SCRIPTS = os.environ.get("OPENOCD_SCRIPTS",
                                 "C:/msys64/mingw64/share/openocd/scripts")


# --------------------------------------------------------------------------
# a trace, whatever it came from
# --------------------------------------------------------------------------

class Trace:
    def __init__(self, name, t_ms, channels, meta=None):
        self.name = name
        self.t_ms = np.asarray(t_ms, dtype=float)
        self.ch = {k: np.asarray(v, dtype=float) for k, v in channels.items()}
        self.meta = meta or {}

    def __repr__(self):
        return f"<Trace {self.name} n={len(self.t_ms)} ch={list(self.ch)}>"


def _s16(v):
    return v - 65536 if v >= 32768 else v


def read_hw_capture(path):
    """Parse the raw text step_trace.sh saves.

    Lines are `T <keys...> <hex data...>` and `META <keys...>`. The number of
    key fields has changed as the tool grew, so rather than hard-coding it,
    anything that parses as 0x-hex is data and everything before it is a key.
    That keeps older saved captures readable, which matters because they are
    the record of what the hardware actually did on a given day.
    """
    caps, meta = {}, {}
    with open(path) as fh:
        for line in fh:
            tok = line.split()
            if not tok:
                continue
            kind = tok[0]
            keys, data = [], []
            for x in tok[1:]:
                (data if x.lower().startswith("0x") else keys).append(x)
            if kind == "META":
                # META carries no chunk index, so ALL of its keys identify the
                # capture. Dropping the last one here silently orphaned every
                # hardware capture's metadata - the plots still drew, they just
                # lost the operating point, which is the part that makes a
                # figure worth sending to anyone.
                meta[tuple(keys)] = data
            elif kind == "T":
                # last key is the chunk index; the rest identify the capture
                cap_key = tuple(keys[:-1])
                chunk = int(keys[-1])
                caps.setdefault(cap_key, {})[chunk] = [_s16(int(x, 16)) for x in data]

    out = []
    for key, chunks in sorted(caps.items()):
        flat = [v for i in sorted(chunks) for v in chunks[i]]
        n = len(flat) // 4
        arr = np.array(flat[: n * 4]).reshape(n, 4)
        idx = np.arange(n)

        iq = arr[:, 1]
        k = _find_step(iq)
        t_ms = (idx - k) / FS_HZ * 1000.0

        m = meta.get(key, [])
        info = {}
        if len(m) >= 5:
            info = {"we": _sig(int(m[0], 16)) / 10.0,
                    "vbus_mv": _sig(int(m[4], 16)),
                    "source": "hw"}

        out.append(Trace(
            name="hw " + " ".join(key),
            t_ms=t_ms,
            channels={"id_a": arr[:, 0] / 1000.0,
                      "iq_a": arr[:, 1] / 1000.0,
                      "vd": arr[:, 2] / 1000.0,
                      "vq": arr[:, 3] / 1000.0},
            meta=info))
    return out


def _sig(v):
    return v - (1 << 32) if v & 0x80000000 else v


def _find_step(iq):
    """Locate the commanded step, falling back to where firmware puts it.

    The firmware places the edge a fixed STEP_PRE_TICKS after arming, so that
    is the answer unless the capture predates that change - in which case the
    edge is wherever SWD latency dropped it and has to be found.
    """
    lo = np.median(iq[:30]) if len(iq) > 40 else iq[0]
    hi = np.median(iq[-100:]) if len(iq) > 200 else iq[-1]
    if abs(hi - lo) < 200:                    # no clear step: keep firmware's
        return STEP_PRE_TICKS
    mid = 0.5 * (lo + hi)
    crossing = np.where((iq - mid) * np.sign(hi - lo) > 0)[0]
    return int(crossing[0]) if len(crossing) else STEP_PRE_TICKS


def read_sim_csv(path):
    """Parse the CSV test/host/sim_dump writes."""
    rows = np.genfromtxt(path, delimiter=",", names=True)
    ch = {"id_a": rows["id_a"], "iq_a": rows["iq_a"],
          "vd": rows["vd"], "vq": rows["vq"]}
    meta = {}
    if "we" in rows.dtype.names:
        meta["we"] = float(np.median(rows["we"]))
    meta["source"] = "sim"
    return [Trace(name="sim " + os.path.basename(path),
                  t_ms=rows["t_ms"], channels=ch, meta=meta)]


def select(traces, key):
    """Keep only captures whose identifying keys contain `key`.

    An A/B capture holds both flag states in one file - averaging across them
    would compare the simulator against a blend of two different firmware
    configurations, which is worse than not comparing at all.
    """
    if key is None:
        return traces
    hits = [t for t in traces if key in t.name.split()]
    if not hits:
        avail = sorted({k for t in traces for k in t.name.split()[1:]})
        sys.exit(f"no captures matching '{key}'. keys present: {' '.join(avail)}")
    return hits


def load_any(path):
    """Sniff the format so the caller does not have to say which it is."""
    with open(path) as fh:
        head = fh.read(400)
    if head.lstrip().startswith("t_ms,"):
        return read_sim_csv(path)
    return read_hw_capture(path)


# --------------------------------------------------------------------------
# plotting
# --------------------------------------------------------------------------

def _provenance():
    """A one-line stamp so a figure can be interpreted and reproduced later.

    A plot mailed to someone else is worth very little without the operating
    point and the code that produced it - "iq overshoots" is not a finding, it
    is a rumour, until the reader can tell which firmware and which bus. The
    repo state is stamped rather than assumed clean, because a graph produced
    from uncommitted work is exactly the one that later cannot be reproduced.
    """
    import datetime
    stamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M")
    try:
        h = subprocess.run(["git", "-C", ROOT, "rev-parse", "--short", "HEAD"],
                           capture_output=True, text=True).stdout.strip()
        dirty = subprocess.run(["git", "-C", ROOT, "status", "--porcelain"],
                               capture_output=True, text=True).stdout.strip()
        rev = f"{h}{'+dirty' if dirty else ''}" if h else "unknown"
    except Exception:
        rev = "unknown"
    return f"Isurus @ {rev}   plotted {stamp}"


def _save(fig, out, dpi, footer_bits):
    """Write the figure. Format follows the extension - .pdf and .svg are
    vector and are what belong in a document; .png is for chat and issues."""
    lines = [b for b in footer_bits if b]
    lines.append(_provenance())
    fig.text(0.005, 0.002, "\n".join(lines), fontsize=7, color="0.35",
             va="bottom", ha="left")
    fig.savefig(out, dpi=dpi)
    print(f"wrote {out}")


def _write_csv(path, traces):
    """Ship the numbers next to the picture.

    Someone who disagrees with a plot should be able to re-plot it rather than
    ask for the raw capture, and a colleague without this repo checked out
    still gets something they can open.
    """
    import csv
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        keys = sorted({k for t in traces for k in t.ch})
        w.writerow(["trace", "t_ms"] + keys)
        for t in traces:
            for i, tm in enumerate(t.t_ms):
                w.writerow([t.name, f"{tm:.6f}"] +
                           [f"{t.ch[k][i]:.6f}" if k in t.ch else "" for k in keys])
    print(f"wrote {path}")


def _style(ax, ylabel):
    ax.grid(alpha=0.25, linewidth=0.6)
    ax.set_ylabel(ylabel)
    ax.axvline(0.0, color="0.6", linewidth=0.8, linestyle=":")


def _footer_bits(traces):
    """Condense the operating point into a line or two for the figure footer.

    Captures from one run share a speed and a bus, so they are collapsed
    rather than listed per trace - twenty identical lines is not provenance,
    it is noise.
    """
    groups = {}
    for t in traces:
        we = t.meta.get("we")
        if we is None:
            continue
        tag = t.meta.get("source") or t.name.split()[0]
        g = groups.setdefault(tag, {"we": [], "vb": []})
        g["we"].append(we)
        vb = t.meta.get("vbus_mv")
        if vb:
            g["vb"].append(vb / 1000.0)

    bits = []
    for tag, g in groups.items():
        we = np.array(g["we"])
        n = len(we)
        line = f"{tag}: w_e {we.mean():.0f}"
        # Quote the spread when there is one. Captures in a run are taken
        # while the machine coasts, so the speed is not identical across them
        # and pretending otherwise would overstate how controlled the
        # measurement was.
        if n > 1 and np.ptp(we) > 5:
            line += f" (+/-{np.ptp(we)/2:.0f})"
        line += f" rad/s, {we.mean()/(2*np.pi):.0f} Hz electrical"
        if g["vb"]:
            line += f", Vbus {np.mean(g['vb']):.2f} V"
        if n > 1:
            line += f", {n} captures"
        bits.append(line)
    return bits


def plot_traces(traces, out, title, dpi=200):
    fig, (a1, a2) = plt.subplots(2, 1, sharex=True, figsize=(10, 6.5))

    for tr in traces:
        a1.plot(tr.t_ms, tr.ch["iq_a"], linewidth=1.1, label=f"{tr.name}  iq")
        a1.plot(tr.t_ms, tr.ch["id_a"], linewidth=1.0, alpha=0.75,
                label=f"{tr.name}  id")
        a2.plot(tr.t_ms, tr.ch["vq"], linewidth=1.1, label=f"{tr.name}  vq")
        a2.plot(tr.t_ms, tr.ch["vd"], linewidth=1.0, alpha=0.75,
                label=f"{tr.name}  vd")

    _style(a1, "current  [A]")
    _style(a2, "voltage  [normalised duty]")
    a2.set_xlabel("time from the step  [ms]")
    a1.set_title(title)
    a1.legend(fontsize=7, ncol=2, loc="best")
    a2.legend(fontsize=7, ncol=2, loc="best")
    fig.tight_layout(rect=(0, 0.045, 1, 1))
    _save(fig, out, dpi, _footer_bits(traces))


def plot_compare(hw, sim, out, dpi=200):
    fig, (a1, a2, a3) = plt.subplots(3, 1, sharex=True, figsize=(10, 8.5))

    for tr in hw:
        a1.plot(tr.t_ms, tr.ch["iq_a"], color="C0", linewidth=1.0, alpha=0.55)
        a2.plot(tr.t_ms, tr.ch["id_a"], color="C0", linewidth=1.0, alpha=0.55)
        a3.plot(tr.t_ms, tr.ch["vq"], color="C0", linewidth=1.0, alpha=0.55)
    if hw:
        # One ensemble average over the captures: the per-trace ripple is
        # mostly uncorrelated and the mean is what can fairly be compared to a
        # noiseless model.
        grid = hw[0].t_ms
        for ax, key in ((a1, "iq_a"), (a2, "id_a"), (a3, "vq")):
            stack = np.vstack([np.interp(grid, t.t_ms, t.ch[key]) for t in hw])
            ax.plot(grid, stack.mean(axis=0), color="C0", linewidth=2.0,
                    label=f"hardware, mean of {len(hw)}")

    for tr in sim:
        a1.plot(tr.t_ms, tr.ch["iq_a"], color="C3", linewidth=1.8, label="simulator")
        a2.plot(tr.t_ms, tr.ch["id_a"], color="C3", linewidth=1.8, label="simulator")
        a3.plot(tr.t_ms, tr.ch["vq"], color="C3", linewidth=1.8, label="simulator")

    _style(a1, "iq  [A]")
    _style(a2, "id  [A]")
    _style(a3, "vq  [normalised duty]")
    a3.set_xlabel("time from the step  [ms]")
    a1.set_title("hardware vs simulator - same firmware source, same step")
    for ax in (a1, a2, a3):
        ax.legend(fontsize=8, loc="best")
    fig.tight_layout(rect=(0, 0.055, 1, 1))
    _save(fig, out, dpi, _footer_bits(hw + sim))


def plot_live(t_s, ch, out, dpi=200):
    fig, (a1, a2) = plt.subplots(2, 1, sharex=True, figsize=(10, 6.0))
    a1.plot(t_s, ch["iq_a"], linewidth=1.1, label="iq")
    a1.plot(t_s, ch["id_a"], linewidth=1.0, alpha=0.8, label="id")
    a1.plot(t_s, ch["iq_ref_a"], linewidth=1.0, linestyle="--", label="iq_ref")
    a2.plot(t_s, ch["we"], linewidth=1.1, color="C4", label="w_e")

    for ax, lab in ((a1, "current  [A]"), (a2, "electrical speed  [rad/s]")):
        ax.grid(alpha=0.25, linewidth=0.6)
        ax.set_ylabel(lab)
        ax.legend(fontsize=8, loc="best")
    a2.set_xlabel("time  [s]")
    a1.set_title("live telemetry - integer mirrors, bridge untouched")
    fig.tight_layout(rect=(0, 0.05, 1, 1))
    _save(fig, out, dpi, [])



# --------------------------------------------------------------------------
# sim-only report: a figure set that needs no hardware
# --------------------------------------------------------------------------

# Each entry: (slug, title, [(label, sim_dump args)], note)
#
# Deliberately a fixed set rather than anything configurable. The point is a
# document someone else can read, so the scenarios are the ones that carry
# this project's open questions - not whatever was interesting on the day.
REPORT_SCENARIOS = [
    ("01-step-response",
     "Current-loop step response, 0.4 -> 1.2 A",
     [("baseline", ["step", "400", "1200", "1673", "6", "decouple=0", "delay=0"])],
     "Loop characterisation with both corrections off. Rise, settle and the "
     "d-axis disturbance a q-axis step produces through cross-coupling."),

    ("02-decoupling",
     "Cross-coupling decoupling, on vs off",
     [("decoupling off", ["step", "400", "1200", "1673", "6", "decouple=0", "delay=0"]),
      ("decoupling on",  ["step", "400", "1200", "1673", "6", "decouple=1", "delay=0"])],
     "What phase 1b is for. With an exact lambda_m the feedforward removes "
     "almost all of the d-axis excursion, which is the clean version of the "
     "-41% the bench measured under its noise floor."),

    ("03-decoupling-wrong-lambda",
     "Decoupling with lambda_m 15% too high",
     [("lambda_m exact", ["step", "400", "1200", "1673", "6", "decouple=1", "delay=0"]),
      ("lambda_m +15%",  ["step", "400", "1200", "1673", "6", "decouple=1", "delay=0",
                          "lambda_err=1.15"])],
     "Why decoupling currently ships disabled. The feedforward is only as good "
     "as the motor parameter behind it, and this is the error the bench "
     "actually had - see foc.h."),

    ("04-saturation",
     "An unachievable command saturates",
     [("iq_ref = 8 A", ["step", "400", "8000", "1673", "8", "decouple=0", "delay=0"])],
     "The vector limit and the back-calculated anti-windup, asked for more "
     "current than the bus can deliver at this speed."),
]


def _find_sim_dump():
    """Build the simulator into the repo, rather than hunting for it.

    run.sh puts its binaries under the shell's $TMPDIR, which on Windows is an
    MSYS path like /tmp that native Python cannot open at all - so looking
    there fails in a way that reads like "you forgot to run the tests" when
    the tests ran fine. Compiling here removes the dependency and the
    confusion, and costs about a second.
    """
    out_dir = os.path.join(ROOT, "build", "host")
    os.makedirs(out_dir, exist_ok=True)
    exe = os.path.join(out_dir, "sim_dump.exe")

    cc = shutil.which("gcc") or "C:/msys64/mingw64/bin/gcc.exe"
    if not os.path.exists(cc) and not shutil.which(cc):
        sys.exit("no host gcc - see test/host/run.sh")

    cmd = [cc, "-std=gnu11", "-O2", "-Wall", "-Wextra",
           "-I", os.path.join(ROOT, "test/host/shim"),
           "-I", os.path.join(ROOT, "Core/Inc"),
           "-I", os.path.join(ROOT, "test/host"),
           os.path.join(ROOT, "test/host/sim_dump.c"),
           os.path.join(ROOT, "test/host/pmsm.c"),
           os.path.join(ROOT, "test/host/cordic_model.c"),
           os.path.join(ROOT, "Core/Src/foc.c"),
           "-o", exe, "-lm"]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        sys.exit("building sim_dump failed:\n" + r.stderr[:2000])
    return exe


def make_report(outdir, dpi):
    exe = _find_sim_dump()
    os.makedirs(outdir, exist_ok=True)
    index = ["# Isurus - simulation report", "",
             "Generated by `tools/viz.py report`. Every figure comes from "
             "`foc.c` compiled natively and closed around the PMSM model in "
             "`test/host/` - no hardware involved.", "",
             "The model carries a known limit worth stating up front: its "
             "transport delay is 1.5 control periods while the firmware "
             "compensates 1.65, because the real ADC is triggered before the "
             "period boundary and the model samples on it. So these figures "
             "are evidence about the DECOUPLING and not about the "
             "transport-delay compensation. See `test/host/sim.h`.", "",
             f"`{_provenance()}`", ""]

    for slug, title, runs, note in REPORT_SCENARIOS:
        traces = []
        for label, args in runs:
            csv_path = os.path.join(outdir, f"{slug}-{label.replace(' ', '_')}.csv")
            with open(csv_path, "w") as fh:
                r = subprocess.run([exe] + args, stdout=fh, stderr=subprocess.PIPE,
                                   text=True)
            if r.returncode != 0:
                sys.exit(f"sim_dump failed for {slug}: {r.stderr}")
            tr = read_sim_csv(csv_path)[0]
            tr.name = label
            traces.append(tr)

        fig_pdf = os.path.join(outdir, f"{slug}.pdf")
        fig_png = os.path.join(outdir, f"{slug}.png")
        plot_traces(traces, fig_pdf, title, dpi)
        plot_traces(traces, fig_png, title, dpi)

        peaks = ", ".join(
            f"{t.name}: peak |id| {np.max(np.abs(t.ch['id_a'][t.t_ms >= 0]))*1000:.1f} mA"
            for t in traces)
        index += [f"## {title}", "", note, "", f"![{title}]({slug}.png)", "",
                  f"- {peaks}",
                  f"- files: `{slug}.pdf`, `{slug}.png`, and the CSV behind each trace", ""]

    idx = os.path.join(outdir, "index.md")
    with open(idx, "w") as fh:
        fh.write("\n".join(index))
    print(f"wrote {idx}")
    print(f"\n{len(REPORT_SCENARIOS)} figures in {outdir}")


# --------------------------------------------------------------------------
# live poll - read only, never energises
# --------------------------------------------------------------------------

def _nm_symbols(elf):
    nm = shutil.which("arm-none-eabi-nm") or \
        "/c/Program Files (x86)/GNU Arm Embedded Toolchain/10 2021.10/bin/arm-none-eabi-nm"
    out = subprocess.run([nm, elf], capture_output=True, text=True).stdout
    syms = {}
    for line in out.splitlines():
        p = line.split()
        if len(p) == 3:
            syms[p[2]] = int(p[0], 16)
    return syms


def live_capture(seconds, interval_ms=8):
    """Poll g_foc's integer mirrors. Reads only - no writes of any kind."""
    elf = os.path.join(ROOT, "build", "Debug", "makolongfin2.elf")
    if not os.path.exists(elf):
        sys.exit("no ELF - build first")
    syms = _nm_symbols(elf)
    foc = syms["g_foc"]

    addrs = {"id_a": foc + 116, "iq_a": foc + 120,
             "iq_ref_a": foc + 152, "we": foc + 200}

    n = max(1, int(seconds * 1000 / interval_ms))
    lines = [
        "source [find interface/stlink.cfg]", "transport select hla_swd",
        "source [find target/stm32g4x.cfg]", "reset_config none",
        "adapter speed 4000", "init",
    ]
    for _ in range(n):
        reads = " ".join(f"[expr {{[read_memory 0x{addrs[k]:x} 32 1]}}]"
                         for k in ("id_a", "iq_a", "iq_ref_a", "we"))
        lines.append(f'echo "S {reads}"')
        lines.append(f"sleep {interval_ms}")
    lines.append("shutdown")

    with tempfile.NamedTemporaryFile("w", suffix=".cfg", delete=False) as fh:
        fh.write("\n".join(lines))
        cfg = fh.name
    try:
        res = subprocess.run(["openocd", "-s", OPENOCD_SCRIPTS, "-d0", "-f", cfg],
                             capture_output=True, text=True, timeout=seconds + 90)
    finally:
        os.unlink(cfg)

    rows = [l.split()[1:] for l in (res.stdout + res.stderr).splitlines()
            if l.startswith("S ")]
    if not rows:
        sys.exit("no samples - is the board attached?")

    vals = np.array([[_sig(int(x, 0)) for x in r] for r in rows], dtype=float)
    t = np.arange(len(vals)) * interval_ms / 1000.0
    return t, {"id_a": vals[:, 0] / 1000.0, "iq_a": vals[:, 1] / 1000.0,
               "iq_ref_a": vals[:, 2] / 1000.0, "we": vals[:, 3] / 10.0}


# --------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("trace", help="plot a saved capture (hardware or sim)")
    p.add_argument("file")
    p.add_argument("--key", default=None, help="keep only captures with this key")
    p.add_argument("-o", "--out", default="trace.png",
                   help="format follows the extension: .pdf/.svg vector, .png raster")
    p.add_argument("--dpi", type=int, default=200)
    p.add_argument("--csv", default=None, help="also write the plotted data")
    p.add_argument("--title", default=None)

    p = sub.add_parser("compare", help="overlay hardware and simulator")
    p.add_argument("--hw", required=True)
    p.add_argument("--sim", required=True)
    p.add_argument("--hw-key", default=None,
                   help="keep only captures with this key, e.g. the A/B flag value")
    p.add_argument("-o", "--out", default="compare.png")
    p.add_argument("--dpi", type=int, default=200)
    p.add_argument("--csv", default=None, help="also write the plotted data")

    p = sub.add_parser("report", help="generate the sim-only figure set (no hardware)")
    p.add_argument("-o", "--outdir", default=os.path.join(ROOT, "reports"))
    p.add_argument("--dpi", type=int, default=200)

    p = sub.add_parser("live", help="poll telemetry from the board (read-only)")
    p.add_argument("--seconds", type=float, default=5.0)
    p.add_argument("-o", "--out", default="live.png")
    p.add_argument("--dpi", type=int, default=200)

    a = ap.parse_args()
    plt.rcParams.update(REPORT_STYLE)

    if a.cmd == "trace":
        tr = select(load_any(a.file), a.key)
        plot_traces(tr, a.out, a.title or os.path.basename(a.file), a.dpi)
        if a.csv:
            _write_csv(a.csv, tr)
    elif a.cmd == "compare":
        hw = select(load_any(a.hw), a.hw_key)
        sim = load_any(a.sim)
        plot_compare(hw, sim, a.out, a.dpi)
        if a.csv:
            _write_csv(a.csv, hw + sim)
    elif a.cmd == "report":
        make_report(a.outdir, a.dpi)
    else:
        t, ch = live_capture(a.seconds)
        plot_live(t, ch, a.out, a.dpi)


if __name__ == "__main__":
    main()
