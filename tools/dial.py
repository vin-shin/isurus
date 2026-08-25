#!/usr/bin/env python3
"""Haptic dial: turn the motor into a force-feedback input device.

The motor is not driven anywhere here. It renders a force field - detents,
endstops, springs, damping - and your hand does the moving. Grab the shaft and
turn it; the firmware answers with torque 20000 times a second.

    python tools/dial.py            start on the "detent" preset
    python tools/dial.py fine       start on a named preset

The display is a fixed frame that repaints in place - it does not scroll, and
a half-typed command survives the repaint. Commands:

    <preset>       switch feel; the names are listed on screen
    set <k> <v>    change one parameter live, e.g. `set detent_ma 800`
    z              call the present position zero (re-centres the clicks)
    q              quit (disengages and opens the gates)

Everything is live - switching presets or nudging a parameter takes effect on
the next control tick, so the fastest way to find a feel you like is to keep
one hand on the shaft and the other on the keyboard.

Note on this motor: 20 pole pairs means noticeable cogging, roughly 20 bumps
per revolution that exist whether or not any detents are rendered. Light
settings will not hide it. Detent counts that are multiples of 20 line up with
it; others beat against it, which you can feel.
"""

import atexit
import os
import queue
import shutil
import socket
import subprocess
import sys
import tempfile
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
ELF = os.path.join(ROOT, "build", "Debug", "makolongfin2.elf")
OCD_SCRIPTS = os.environ.get("OPENOCD_SCRIPTS", "C:/msys64/mingw64/share/openocd/scripts")
NM = shutil.which("arm-none-eabi-nm") or \
    r"C:/Program Files (x86)/GNU Arm Embedded Toolchain/10 2021.10/bin/arm-none-eabi-nm"

MODE_IDLE, MODE_HAPTIC = 0, 4

# HapticState_t field offsets; see Core/Inc/haptic.h
FIELDS = {
    "detent_count":  0, "detent_ma":     4, "detent_shape":  8,
    "spring_k":     12, "spring_center":16, "damping":      20,
    "friction_ma":  24, "endstop_lo":   28, "endstop_hi":   32,
    "endstop_k":    36, "torque_max":   40,
}
T_INDEX, T_TORQUE, T_ENDSTOP = 44, 48, 52

# A preset is just a set of gains - the firmware sums whatever is non-zero, so
# these are combinations of one law rather than separate behaviours.
# spring_k / damping / endstop_k are x100; angles are tenths of a degree.
PRESETS = {
    "detent":  dict(detent_count=24, detent_ma=500, detent_shape=1, damping=40,
                    spring_k=0, endstop_k=0, friction_ma=0,
                    _d="24 clicks per turn, the default knob"),
    "fine":    dict(detent_count=72, detent_ma=260, detent_shape=1, damping=30,
                    spring_k=0, endstop_k=0, friction_ma=0,
                    _d="72 light clicks - a scroll wheel"),
    "coarse":  dict(detent_count=8, detent_ma=850, detent_shape=1, damping=60,
                    spring_k=0, endstop_k=0, friction_ma=0,
                    _d="8 heavy clicks - a rotary switch"),
    "smooth":  dict(detent_count=24, detent_ma=500, detent_shape=0, damping=40,
                    spring_k=0, endstop_k=0, friction_ma=0,
                    _d="24 SINE wells - softer, but has a dead band at each edge"),
    "knob":    dict(detent_count=24, detent_ma=500, detent_shape=1, damping=40,
                    spring_k=0, friction_ma=0,
                    endstop_lo=0, endstop_hi=3600, endstop_k=4000,
                    _d="volume knob: 24 clicks, hard stops at 0 and 360 deg"),
    "spring":  dict(detent_count=0, spring_k=1500, spring_center=0, damping=80,
                    endstop_k=0, friction_ma=0,
                    _d="sprung return to zero - a throttle"),
    "bounded": dict(detent_count=0, spring_k=0, damping=60, friction_ma=0,
                    endstop_lo=-900, endstop_hi=900, endstop_k=5000,
                    _d="free within +/-90 deg, hard walls outside"),
    "free":    dict(detent_count=0, spring_k=0, damping=25, friction_ma=0,
                    endstop_k=0, _d="no detents, just a little viscosity"),
    "brake":   dict(detent_count=0, spring_k=0, damping=0, friction_ma=260,
                    endstop_k=0, _d="dry friction only - a drag knob"),
    "off":     dict(detent_count=0, spring_k=0, damping=0, friction_ma=0,
                    endstop_k=0, _d="no force at all; the shaft spins free"),
}


def symbols():
    if not os.path.exists(ELF):
        sys.exit("No ELF at %s - run 'ninja -C build/Debug' first." % ELF)
    out = subprocess.run([NM, ELF], capture_output=True, text=True).stdout
    syms = {}
    for line in out.splitlines():
        f = line.split()
        if len(f) == 3:
            syms[f[2]] = int(f[0], 16)
    for need in ("g_haptic", "g_pos", "g_foc", "g_cmd", "g_faulted"):
        if need not in syms:
            sys.exit("Symbol %s not found - rebuild?" % need)
    return syms


class Ocd:
    SEP = b"\x1a"

    def __init__(self, port=6666):
        cfg = tempfile.NamedTemporaryFile("w", suffix=".cfg", delete=False)
        cfg.write("source [find interface/stlink.cfg]\n"
                  "transport select hla_swd\n"
                  "source [find target/stm32g4x.cfg]\n"
                  "reset_config none\nadapter speed 4000\n"
                  "tcl_port %d\ngdb_port disabled\ntelnet_port disabled\ninit\n" % port)
        cfg.close()
        self.cfg = cfg.name
        self.log = tempfile.NamedTemporaryFile("w", suffix=".log", delete=False)
        self.proc = subprocess.Popen(["openocd", "-s", OCD_SCRIPTS, "-d0", "-f", self.cfg],
                                     stdout=self.log, stderr=subprocess.STDOUT)
        self.sock = None
        deadline = time.time() + 12
        while time.time() < deadline:
            if self.proc.poll() is not None:
                sys.exit("OpenOCD exited during start-up; see %s" % self.log.name)
            try:
                self.sock = socket.create_connection(("127.0.0.1", port), timeout=5)
                break
            except OSError:
                time.sleep(0.25)
        if self.sock is None:
            sys.exit("could not reach OpenOCD's Tcl port")

    def cmd(self, text):
        self.sock.sendall(text.encode() + self.SEP)
        buf = b""
        while not buf.endswith(self.SEP):
            chunk = self.sock.recv(65536)
            if not chunk:
                raise IOError("OpenOCD closed the connection")
            buf += chunk
        return buf[:-1].decode(errors="replace")

    def read(self, addr, n=1):
        return [int(w, 0) for w in self.cmd("read_memory 0x%x 32 %d" % (addr, n)).split()]

    def write(self, addr, val):
        self.cmd("mww 0x%x %d" % (addr, val & 0xFFFFFFFF))

    def close(self):
        try:
            if self.sock:
                self.sock.close()
        except OSError:
            pass
        for fn in (self.proc.terminate, self.proc.kill):
            try:
                fn()
                self.proc.wait(timeout=5)
                break
            except Exception:
                pass
        for path in (self.cfg, self.log.name):
            try:
                os.unlink(path)
            except OSError:
                pass


def s32(v):
    return v - 0x100000000 if v > 0x7FFFFFFF else v


class Dial:
    def __init__(self, ocd, sym):
        self.o, self.h = ocd, sym["g_haptic"]
        self.pos, self.foc = sym["g_pos"], sym["g_foc"]
        self.cmdblk, self.flt = sym["g_cmd"], sym["g_faulted"]

    def set(self, field, value):
        self.o.write(self.h + FIELDS[field], int(value))

    def apply(self, preset):
        # Write every field, not just the ones the preset names: leaving a gain
        # from the previous preset behind is how you end up with a "spring"
        # that mysteriously still clicks.
        merged = {k: 0 for k in FIELDS}
        merged["torque_max"] = 900
        merged["detent_shape"] = 1
        merged.update({k: v for k, v in preset.items() if not k.startswith("_")})
        for k, v in merged.items():
            self.set(k, v)

    def telem(self):
        h = self.o.read(self.h, 15)
        p = self.o.read(self.pos, 30)
        return (s32(h[T_INDEX // 4]), s32(h[T_TORQUE // 4]), s32(h[T_ENDSTOP // 4]),
                s32(p[10]) / 10.0, s32(p[13]), s32(h[0]))

    def bridge_up(self):
        for off in (4, 8, 12):
            self.o.write(self.cmdblk + off, 0)
        self.o.write(self.cmdblk + 16, 1)
        self.o.write(self.cmdblk, 1)
        time.sleep(0.2)
        self.o.write(self.cmdblk + 20, 1)
        self.o.write(self.cmdblk, 1)
        time.sleep(0.2)
        self.o.write(self.flt, 0)
        self.o.write(self.foc + 0, 0)
        self.o.write(self.foc + 4, 0)
        self.o.write(self.foc + 112, 1)
        time.sleep(0.2)
        self.o.write(self.pos + 0, 1)        # enabled
        self.o.write(self.pos + 92, MODE_HAPTIC)
        time.sleep(0.2)
        self.o.write(self.pos + 36, 1)       # zero_here
        time.sleep(0.3)

    def safe_off(self):
        try:
            self.o.write(self.pos + 92, MODE_IDLE)
            self.o.write(self.pos + 0, 0)
            self.o.write(self.foc + 4, 0)
            self.o.write(self.foc + 0, 0)
            self.o.write(self.foc + 112, 0)
            self.o.write(self.cmdblk + 20, 0)
            self.o.write(self.cmdblk + 16, 0)
            self.o.write(self.cmdblk, 1)
            return True
        except Exception:
            return False


ESC = ""
FRAME_H = 21          # rows the frame occupies; the prompt sits just below it
PROMPT_ROW = FRAME_H + 1


def dial_art(angle, detents, idx, w=34, h=13):
    """The dial as a character grid: rim, one tick per detent, and a needle.

    Drawn CLOCKWISE-positive so the needle turns the way the shaft does. Screen
    y grows downward while the maths convention has angle growing anticlockwise,
    so the plotted angle is negated - the same correction pos_dash.sh needed.
    """
    import math
    g = [[" "] * w for _ in range(h)]
    cx, cy = w // 2, h // 2
    rx, ry = w // 2 - 2, h // 2 - 1

    def put(deg, r, ch):
        a = math.radians(-deg)
        x = int(round(cx + rx * r * math.cos(a)))
        y = int(round(cy - ry * r * math.sin(a)))
        if 0 <= x < w and 0 <= y < h:
            g[y][x] = ch

    for t in range(0, 360, 4):                      # rim
        put(t, 1.0, ".")
    if 0 < detents <= 48:                           # one tick per well
        for i in range(detents):
            put(i * 360.0 / detents, 1.0, "+")

    for r in (0.25, 0.4, 0.55, 0.7):                # needle
        put(angle, r, "=")
    put(angle, 0.85, "O")
    g[cy][cx] = "+"

    # The click count deliberately is NOT drawn inside the dial: it would sit
    # on the same rows the needle sweeps through and collide with it. It lives
    # in the readout column instead.
    return ["".join(r) for r in g]


def bar(val, lo, hi, width=22):
    span = (hi - lo) or 1
    n = int((val - lo) * width / span)
    n = max(0, min(width, n))
    mid = width // 2
    out = ""
    for i in range(width):
        if i == mid:
            out += "#" if i < n else "|"
        elif i < n:
            out += "#"
        else:
            out += "."
    return out


def build_frame(name, desc, idx, torque, endstop, angle, vel, params, msg):
    detents = params.get("detent_count", 0)
    tmax = params.get("torque_max", 900) or 900
    art = dial_art(angle, detents, idx)

    right = [
        "",
        "%sclick%s" % (ESC + "[90m", ESC + "[0m") if detents else
        "%sfree spin%s" % (ESC + "[90m", ESC + "[0m"),
        "   %s%+d%s" % (ESC + "[1;96m", idx, ESC + "[0m") if detents else "",
        "",
        "angle   %+9.1f deg" % angle,
        "vel     %+6d d/s" % vel,
        "torque  %+6d mA" % torque,
        "",
        "endstop   %s" % {-1: ESC + "[1;91m< LO" + ESC + "[0m",
                          1: ESC + "[1;91mHI >" + ESC + "[0m",
                          0: ESC + "[90m--" + ESC + "[0m"}[endstop],
        "",
        "",
        "",
        "",
    ]

    lines = []
    lines.append("  %sHAPTIC DIAL%s  makolongfin2      %s%-9s%s %s%s%s" % (
        ESC + "[1;97m", ESC + "[0m", ESC + "[1;95m", name, ESC + "[0m",
        ESC + "[90m", desc, ESC + "[0m"))
    lines.append("")
    for i, row in enumerate(art):
        lines.append("   %s   %s" % (row, right[i] if i < len(right) else ""))
    lines.append("")
    lines.append("   torque  %s" % bar(torque, -tmax, tmax))
    lines.append("   %sdetents%s %-4d %sstrength%s %-5d mA  %sdamping%s %.2f  %sshape%s %s" % (
        ESC + "[90m", ESC + "[0m", detents,
        ESC + "[90m", ESC + "[0m", params.get("detent_ma", 0),
        ESC + "[90m", ESC + "[0m", params.get("damping", 0) / 100.0,
        ESC + "[90m", ESC + "[0m",
        "ramp" if params.get("detent_shape", 1) else "sine"))
    lines.append("   %s%s%s" % (ESC + "[90m", "  ".join(PRESETS), ESC + "[0m"))
    lines.append("   %s%s%s" % (ESC + "[93m", msg, ESC + "[0m"))
    return lines


def draw(lines):
    """Repaint in place, without disturbing whatever is being typed.

    The cursor is saved first and restored last, and the frame is written with
    absolute row addressing that never touches the prompt row. That is what
    keeps a half-typed command on screen while the display keeps updating - a
    plain reprint would either erase it or scroll the frame away.
    """
    out = [ESC + "[s"]
    for i, line in enumerate(lines[:FRAME_H]):
        out.append("%s[%d;1H%s%s[K" % (ESC, i + 1, line, ESC))
    for i in range(len(lines), FRAME_H):
        out.append("%s[%d;1H%s[K" % (ESC, i + 1, ESC))
    out.append(ESC + "[u")
    sys.stdout.write("".join(out))
    sys.stdout.flush()


def prompt():
    sys.stdout.write("%s[%d;1H%s[K> " % (ESC, PROMPT_ROW, ESC))
    sys.stdout.flush()


def main():
    start = sys.argv[1] if len(sys.argv) > 1 else "detent"
    if start not in PRESETS:
        sys.exit("unknown preset %r; try: %s" % (start, ", ".join(PRESETS)))

    sym = symbols()
    print("starting OpenOCD ...")
    ocd = Ocd()
    d = Dial(ocd, sym)

    def teardown():
        ok = d.safe_off()
        ocd.close()
        # Leave the terminal below the frame, not on top of it.
        sys.stdout.write("%s[%d;1H%s[K\n" % (ESC, PROMPT_ROW + 1, ESC))
        print("power stage safe." if ok else
              "!! COULD NOT DISENGAGE - POWER DOWN THE BOARD MANUALLY !!")

    atexit.register(teardown)

    d.bridge_up()
    name = start
    params = dict(PRESETS[name])
    d.apply(params)
    msg = "grab the shaft and turn it - type a preset name to switch, q to quit"

    # stdin on its own thread: the frame has to keep repainting while you type,
    # and there is no portable non-blocking console read under mintty.
    q = queue.Queue()

    def reader():
        for line in sys.stdin:
            q.put(line.strip())
        q.put("q")

    threading.Thread(target=reader, daemon=True).start()

    sys.stdout.write(ESC + "[2J")
    prompt()

    while True:
        try:
            idx, torque, endstop, angle, vel, detents = d.telem()
            draw(build_frame(name, PRESETS[name]["_d"], idx, torque, endstop,
                             angle, vel, params, msg))
        except IOError as exc:
            msg = "lost the debug link: %s" % exc
            break

        try:
            line = q.get(timeout=0.08)
        except queue.Empty:
            continue

        parts = line.split()
        c = parts[0].lower() if parts else ""
        if c in ("q", "quit", "exit"):
            break
        elif c in PRESETS:
            name = c
            params = dict(PRESETS[name])
            d.apply(params)
            msg = "%s: %s" % (name, PRESETS[name]["_d"])
        elif c == "set" and len(parts) == 3 and parts[1] in FIELDS:
            try:
                v = int(round(float(parts[2])))
                d.set(parts[1], v)
                params[parts[1]] = v
                msg = "%s = %d" % (parts[1], v)
            except ValueError:
                msg = "not a number: %r" % parts[2]
        elif c == "set":
            msg = "fields: " + ", ".join(FIELDS)
        elif c == "z":
            d.o.write(d.pos + 36, 1)
            msg = "zeroed here"
        elif c == "":
            pass
        else:
            msg = "? preset name, `set <field> <value>`, `z`, or `q`"
        prompt()


if __name__ == "__main__":
    main()
