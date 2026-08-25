#!/usr/bin/env python3
"""Haptic dial: turn the motor into a force-feedback input device.

The motor is not driven anywhere here. It renders a force field - detents,
endstops, springs, damping - and your hand does the moving. Grab the shaft and
turn it; the firmware answers with torque 20000 times a second.

    python tools/dial.py            start on the "detent" preset
    python tools/dial.py fine       start on a named preset

While it runs the display streams and you can still type. Commands:

    <preset>       switch feel; bare `list` shows them all
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


def render(name, idx, torque, endstop, angle, vel, detents):
    """One line: the click count is the dial's value, the bar is where the
    shaft sits inside the current well."""
    if detents > 0:
        span = 360.0 / detents
        frac = ((angle + span / 2.0) % span) / span      # 0..1 across the well
        cells = 21
        pos = int(frac * (cells - 1))
        bar = "".join("O" if i == pos else ("|" if i == cells // 2 else "-")
                      for i in range(cells))
        value = "click %+5d" % idx
    else:
        bar = "-" * 21
        value = "  free    "
    wall = {-1: "<LO ", 1: " HI>", 0: "    "}[endstop]
    return ("  \033[1;97m%-8s\033[0m %s  [%s] %s  %+8.1f deg  %+5d d/s  "
            "\033[90mtorque\033[0m %+5d mA " % (name, value, bar, wall, angle, vel, torque))


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
        print("\npower stage safe." if ok else
              "\n!! COULD NOT DISENGAGE - POWER DOWN THE BOARD MANUALLY !!")

    atexit.register(teardown)

    d.bridge_up()
    name = start
    d.apply(PRESETS[name])
    print("\nGrab the shaft and turn it. Type a preset name to switch, `q` to quit.")
    print("presets: " + ", ".join(PRESETS) + "\n")

    # stdin on its own thread: the display has to keep streaming while you
    # type, and there is no portable non-blocking console read under mintty.
    q = queue.Queue()

    def reader():
        for line in sys.stdin:
            q.put(line.strip())
        q.put("q")

    threading.Thread(target=reader, daemon=True).start()

    while True:
        try:
            idx, torque, endstop, angle, vel, detents = d.telem()
            sys.stdout.write("\r\033[K" + render(name, idx, torque, endstop,
                                                 angle, vel, detents))
            sys.stdout.flush()
        except IOError as exc:
            print("\nlost the debug link: %s" % exc)
            break

        try:
            line = q.get(timeout=0.08)
        except queue.Empty:
            continue

        print()
        parts = line.split()
        if not parts:
            continue
        c = parts[0].lower()
        if c in ("q", "quit", "exit"):
            break
        elif c == "list":
            for k, v in PRESETS.items():
                print("  %-8s %s" % (k, v["_d"]))
        elif c == "z":
            d.o.write(d.pos + 36, 1)
        elif c == "set" and len(parts) == 3 and parts[1] in FIELDS:
            try:
                d.set(parts[1], float(parts[2]))
            except ValueError:
                print("  not a number: %r" % parts[2])
        elif c == "set":
            print("  fields: " + ", ".join(FIELDS))
        elif c in PRESETS:
            name = c
            d.apply(PRESETS[name])
            print("  -> %s: %s" % (name, PRESETS[name]["_d"]))
        else:
            print("  ? try a preset name, `list`, `set <field> <value>`, `z`, `q`")


if __name__ == "__main__":
    main()
