#!/usr/bin/env python3
"""Analyse the captures from tools/step_trace.sh.

Kept separate from the driver script because the driver spins a motor and this
does not - which means this half can be tested on saved or synthetic data as
often as needed, and was.

The method that matters here is ENSEMBLE AVERAGING, and it is the whole reason
this test can resolve anything. Measuring peak |id| on a single step does not
work: the d-axis disturbance from a q-axis step is real but sits under ripple,
cogging and speed drift that are several times larger, so a single trace is
mostly noise and its peak is the noisiest statistic available. Averaging the
PEAKS does not help either - noise raises every peak, so the average peak stays
biased upward no matter how many reps are taken.

Averaging the aligned WAVEFORMS does help. The disturbance is deterministic and
appears at the same place with the same sign after every step; the noise does
not. So the average of N traces has the same transient and 1/sqrt(N) of the
noise. The peak is then measured on the averaged waveform, and the pre-step
part of that same averaged waveform gives an empirical noise floor to judge it
against - printed, so the claim is checkable rather than asserted.
"""

import math
import sys

FS       = 30000.0   # ISR rate; the trace captures one sample per period
PRE      = 30        # samples kept before the step, for baseline and noise
POST     = 120       # samples kept after (4 ms)
MEAS     = 90        # window the peak/rms are measured over (3 ms)


def s16(v):
    return v - 65536 if v >= 32768 else v


def mean(v):
    return sum(v) / len(v) if v else 0.0


def sd(v):
    if len(v) < 2:
        return 0.0
    m = mean(v)
    return math.sqrt(sum((x - m) ** 2 for x in v) / (len(v) - 1))


def rms(v):
    return math.sqrt(sum(x * x for x in v) / len(v)) if v else 0.0


def load(path):
    chunks, meta = {}, {}
    for line in open(path):
        p = line.split()
        if not p:
            continue
        if p[0] == "META":
            meta[(int(p[1]), int(p[2]))] = tuple(int(x, 0) for x in p[3:])
        elif p[0] == "T":
            key = (int(p[1]), int(p[2]))
            chunks.setdefault(key, {})[int(p[3])] = [s16(int(x, 0)) for x in p[4:]]
    return chunks, meta


def extract(chunks, iq_lo, iq_hi):
    """Per capture: locate the step, then return baseline-corrected windows."""
    mid = (iq_lo + iq_hi) / 2.0
    out = {}
    # Rejections are counted and reported, never silent. An earlier version
    # dropped captures whose step landed too early to have enough pre-roll and
    # said nothing, so a run that kept 3 of 12 looked exactly like a run that
    # kept all 12 - with a quarter of the averaging and no sign of it.
    rej = {"no step": 0, "too early": 0, "too late": 0}
    for key, parts in chunks.items():
        flat = [v for i in sorted(parts) for v in parts[i]]
        smp = [tuple(flat[i:i + 4]) for i in range(0, len(flat), 4)]
        idc = [x[0] for x in smp]
        iqc = [x[1] for x in smp]
        # Locate the step rather than assume it: the gap between arming the
        # trace and writing the new setpoint is SWD latency, not a fixed
        # number of ticks, and it moves run to run.
        k = next((i for i, v in enumerate(iqc) if v > mid), None)
        if k is None:
            rej["no step"] += 1
            continue
        if k < PRE:
            rej["too early"] += 1
            continue
        if k + POST > len(idc):
            rej["too late"] += 1
            continue
        # Baseline-subtract on the pre-step mean. Standing d-axis offset is
        # not what the step caused, and it drifts between reps; leaving it in
        # would put that drift straight into the averaged transient.
        base = mean(idc[k - PRE:k])
        out[key] = ([v - base for v in idc[k - PRE:k + POST]],
                    [v for v in iqc[k - PRE:k + POST]])
    return out, rej


def ensemble(windows, dc):
    reps = [w for (d, _), w in sorted(windows.items()) if d == dc]
    if not reps:
        return None, 0
    n = len(reps)
    avg = [mean([r[0][i] for r in reps]) for i in range(len(reps[0][0]))]
    return avg, n


def spark(avg, span, rows=7, label=""):
    w = avg[PRE - 10:PRE + 80]
    print()
    print(f"  {label}")
    for r in range(rows, -rows - 1, -1):
        if r == 0:
            print("  0 |" + "-" * len(w))
            continue
        line = "".join(
            "#" if (r > 0 and v * rows / span >= r) or
                   (r < 0 and v * rows / span <= r) else " "
            for v in w)
        print("    |" + line)


def main():
    path, iq_lo, iq_hi = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
    flag = sys.argv[4] if len(sys.argv) > 4 else "decouple"
    chunks, meta = load(path)
    windows, rej = extract(chunks, iq_lo, iq_hi)
    dropped = sum(rej.values())
    if dropped:
        why = ", ".join(f"{n} {k}" for k, n in rej.items() if n)
        print()
        print(f"  WARNING: {dropped} capture(s) discarded ({why}).")
        print("  Averaging is only as good as the count that survived.")

    print()
    for dc in (1, 0):
        ms = [meta[k] for k in meta if k[0] == dc]
        if not ms:
            continue
        om = mean([m[0] for m in ms]) / 10.0
        adv = mean([m[5] for m in ms]) / 10.0 if len(ms[0]) > 5 else 0.0
        print(f"  {flag}={dc}: {om:7.1f} elec rad/s ({om / (2 * math.pi):5.1f} Hz), "
              f"vq_ff {mean([m[1] for m in ms]) / 1000.0:+.3f} of bus, "
              f"advance {adv:+.2f} deg, Vbus {mean([m[4] for m in ms]):.0f} mV, "
              f"{sum(1 for m in ms if m[2] == 1)}/{len(ms)} complete")

    print()
    print(f"  d-axis response to a {iq_lo} -> {iq_hi} mA step in iq_ref.")
    print(f"  Peak and rms are measured on the ENSEMBLE-AVERAGED waveform over")
    print(f"  the {MEAS / FS * 1000:.0f} ms after the step; 'noise' is the rms of that same")
    print(f"  averaged waveform BEFORE the step, which is what is left of the")
    print(f"  random part after averaging and the bar any peak has to clear.")
    print()
    print(f"  {flag:<12}{'reps':>6}{'peak |id|':>12}{'rms id':>10}"
          f"{'noise':>9}{'peak/noise':>12}")
    print(f"  {'-' * 59}")

    agg = {}
    for dc in (1, 0):
        avg, n = ensemble(windows, dc)
        if avg is None:
            continue
        post = avg[PRE:PRE + MEAS]
        pre = avg[:PRE]
        pk, rm, nz = max(abs(v) for v in post), rms(post), rms(pre)
        agg[dc] = (pk, rm, nz, avg)
        print(f"  {dc:<12}{n:>6}{pk:>12.0f}{rm:>10.1f}{nz:>9.1f}"
              f"{(pk / nz if nz else 0):>12.1f}")

    if 1 in agg and 0 in agg:
        pk1, rm1, _, _ = agg[1]
        pk0, rm0, _, _ = agg[0]
        print()
        print(f"  {flag} ON vs OFF:  peak {100 * (pk1 - pk0) / pk0:+.0f}%"
              f"   rms {100 * (rm1 - rm0) / rm0:+.0f}%")
        if min(agg[1][2], agg[0][2]) > 0 and max(pk1, pk0) / max(agg[1][2], agg[0][2]) < 3.0:
            print("  Both peaks are within 3x the post-averaging noise floor -")
            print("  treat this as unresolved and raise reps before believing it.")

    span = max(1.0, max(max(abs(v) for v in agg[dc][3]) for dc in agg)) if agg else 1.0
    for dc in (1, 0):
        if dc in agg:
            spark(agg[dc][3], span,
                  label=f"id, {flag}={dc}   (shared scale +/-{span:.0f} mA, 3 ms wide)")


if __name__ == "__main__":
    main()
