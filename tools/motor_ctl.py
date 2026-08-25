#!/usr/bin/env python3
"""Interactive motor console: type control requests straight at the board.

Talks to OpenOCD's Tcl RPC socket rather than driving OpenOCD from a config
file, which is what makes this interactive at all. The dashboards run their
whole render loop inside OpenOCD's own Tcl interpreter, and that interpreter
owns the ST-Link for the entire run - there is no way to get a keystroke in
edgewise. Here OpenOCD is a subprocess with `tcl_port` open, this process owns
the terminal, and memory reads and writes go over the socket.

Input is line-based on purpose. Raw key capture on Windows means msvcrt, which
reads the *console* - and under Git Bash's mintty stdin is a pipe with no
console attached, so it silently reads nothing. Typing a short command and
pressing Enter works in every terminal.

    python tools/motor_ctl.py

Commands (also `?` for this list):

    e / d          enable (engage servo) / disable
    i              idle - commands zero current, stays engaged
    t <mA>         torque mode, iq setpoint in milliamps
    v <deg/s>      velocity mode
    p <deg>        position mode, absolute, multi-turn
    pr <deg>       position mode, relative to the current target
    z              call the present position zero
    lim <mA>       output current clamp
    vmax <deg/s>   profile cruise speed
    acc <deg/s^2>  profile acceleration limit
    jerk <d/s^3>   profile jerk limit, 0 = plain trapezoid
    gains kp ki kd          position loop
    vgains vkp vki          velocity loop
    w [sec]        watch a live stream (default 5 s)
    s              one status line
    stop           idle + disable, leaves the bridge up
    q              quit (disengages and opens the gates)

Quitting - or dying - always disengages the servo and opens the gate drivers.
A position loop left running will fight anything that touches the shaft
indefinitely, so the teardown is not optional.
"""

import atexit
import os
import re
import shutil
import socket
import subprocess
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
ELF = os.path.join(ROOT, "build", "Debug", "makolongfin2.elf")
OCD_SCRIPTS = os.environ.get("OPENOCD_SCRIPTS", "C:/msys64/mingw64/share/openocd/scripts")

MODES = {0: "idle", 1: "torque", 2: "velocity", 3: "position"}


def find_tool(name, fallback):
    return shutil.which(name) or fallback


NM = find_tool(
    "arm-none-eabi-nm",
    r"C:/Program Files (x86)/GNU Arm Embedded Toolchain/10 2021.10/bin/arm-none-eabi-nm",
)


def symbols():
    """Symbol addresses come from the ELF every run, never hardcoded.

    Any code-size change moves the BSS layout. A stale address does not error -
    it reads or writes the wrong field and looks like the firmware ignoring
    you, which is a genuinely nasty thing to debug.
    """
    if not os.path.exists(ELF):
        sys.exit("No ELF at %s - run 'ninja -C build/Debug' first." % ELF)
    out = subprocess.run([NM, ELF], capture_output=True, text=True).stdout
    syms = {}
    for line in out.splitlines():
        f = line.split()
        if len(f) == 3:
            syms[f[2]] = int(f[0], 16)
    for need in ("g_pos", "g_foc", "g_cmd", "g_faulted", "g_cs"):
        if need not in syms:
            sys.exit("Symbol %s not found - rebuild?" % need)
    return syms


class Ocd:
    """OpenOCD subprocess plus a Tcl RPC connection to it."""

    SEP = b"\x1a"

    def __init__(self, port=6666):
        cfg = tempfile.NamedTemporaryFile("w", suffix=".cfg", delete=False)
        cfg.write(
            "source [find interface/stlink.cfg]\n"
            "transport select hla_swd\n"
            "source [find target/stm32g4x.cfg]\n"
            "reset_config none\n"
            "adapter speed 4000\n"
            "tcl_port %d\n"
            "gdb_port disabled\n"
            "telnet_port disabled\n"
            "init\n" % port
        )
        cfg.close()
        self.cfg = cfg.name
        self.log = tempfile.NamedTemporaryFile("w", suffix=".log", delete=False)
        self.proc = subprocess.Popen(
            ["openocd", "-s", OCD_SCRIPTS, "-d0", "-f", self.cfg],
            stdout=self.log, stderr=subprocess.STDOUT,
        )
        self.sock = None
        deadline = time.time() + 12
        while time.time() < deadline:
            if self.proc.poll() is not None:
                self._die("OpenOCD exited during start-up")
            try:
                self.sock = socket.create_connection(("127.0.0.1", port), timeout=5)
                break
            except OSError:
                time.sleep(0.25)
        if self.sock is None:
            self._die("could not reach OpenOCD's Tcl port")

    def _die(self, msg):
        self.log.close()
        try:
            tail = open(self.log.name).read().strip().splitlines()[-6:]
        except OSError:
            tail = []
        sys.exit("%s\n  %s" % (msg, "\n  ".join(tail)))

    def cmd(self, text):
        self.sock.sendall(text.encode() + self.SEP)
        buf = b""
        while not buf.endswith(self.SEP):
            chunk = self.sock.recv(65536)
            if not chunk:
                raise IOError("OpenOCD closed the connection")
            buf += chunk
        return buf[:-1].decode(errors="replace")

    def read(self, addr, count=1):
        raw = self.cmd("read_memory 0x%x 32 %d" % (addr, count))
        return [int(w, 0) for w in raw.split()]

    def write(self, addr, value):
        self.cmd("mww 0x%x %d" % (addr, value & 0xFFFFFFFF))

    def close(self):
        try:
            if self.sock:
                self.sock.close()
        except OSError:
            pass
        try:
            self.proc.terminate()
            self.proc.wait(timeout=5)
        except Exception:
            try:
                self.proc.kill()
            except Exception:
                pass
        for path in (self.cfg, self.log.name):
            try:
                os.unlink(path)
            except OSError:
                pass


def s32(v):
    return v - 0x100000000 if v > 0x7FFFFFFF else v


class Board:
    # g_pos 32-bit field offsets; see Core/Inc/position.h
    EN, CMD, KP, KI, KD, VMAX, ACC, JERK, IMAX, ZERO = 0, 4, 8, 12, 16, 20, 24, 28, 32, 36
    POS, TGT, ERR, VEL = 40, 44, 48, 52
    IQOUT, INTEG, TURNS = 64, 68, 72
    LPF, VFILT = 80, 84
    MODE, TRQ, VCMD, VKP, VKI, VREF = 92, 96, 100, 104, 108, 112

    def __init__(self, ocd, sym):
        self.o = ocd
        self.pos = sym["g_pos"]
        self.foc = sym["g_foc"]
        self.cmdblk = sym["g_cmd"]
        self.flt = sym["g_faulted"]
        self.cs = sym["g_cs"]

    # --- raw helpers -----------------------------------------------------
    def pset(self, off, val):
        self.o.write(self.pos + off, val)

    def pget(self, off):
        return s32(self.o.read(self.pos + off)[0])

    def snapshot(self):
        w = self.o.read(self.pos, 30)
        f = self.o.read(self.flt)[0]
        vbus = self.o.read(self.cs + 52)[0]
        foc_en = self.o.read(self.foc + 112)[0]
        g = lambda off: s32(w[off // 4])
        return dict(
            enabled=w[0], mode=w[self.MODE // 4], faulted=f, foc_en=foc_en,
            cmd=g(self.CMD), pos=g(self.POS), tgt=g(self.TGT), err=g(self.ERR),
            vel=g(self.VEL), vref=g(self.VREF), iq=g(self.IQOUT),
            trq=g(self.TRQ), vcmd=g(self.VCMD), imax=g(self.IMAX),
            turns=g(self.TURNS), vbus=vbus,
        )

    # --- power stage -----------------------------------------------------
    def bridge_up(self):
        """duty, then outputs, then gates - the order main.c enforces."""
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

    def safe_off(self):
        """Gates first on the way down - the only true all-off."""
        try:
            self.pset(self.EN, 0)
            self.pset(self.MODE, 0)
            self.o.write(self.foc + 4, 0)
            self.o.write(self.foc + 0, 0)
            self.o.write(self.foc + 112, 0)
            self.o.write(self.cmdblk + 20, 0)
            self.o.write(self.cmdblk + 16, 0)
            self.o.write(self.cmdblk, 1)
            return True
        except Exception:
            return False


def status_line(s):
    mode = MODES.get(s["mode"], "?")
    if s["faulted"]:
        state = "\033[1;101;97m OVERCURRENT \033[0m"
    elif s["enabled"] and s["foc_en"]:
        state = "\033[1;92mON \033[0m"
    else:
        state = "\033[90moff\033[0m"
    if s["mode"] == 1:
        want = "%+d mA" % s["trq"]
    elif s["mode"] == 2:
        want = "%+d d/s (ref %+d)" % (s["vcmd"], s["vref"])
    elif s["mode"] == 3:
        want = "%+.1f deg" % (s["cmd"] / 10.0)
    else:
        want = "-"
    return ("%s %-8s want %-22s pos %+9.1f turn %+3d  vel %+6d d/s  "
            "iq %+6d/%d mA  %.2f V" % (
                state, mode, want, s["pos"] / 10.0, s["turns"],
                s["vel"], s["iq"], s["imax"], s["vbus"] / 1000.0))


HELP = __doc__.split("Commands (also `?` for this list):", 1)[1].split(
    "Quitting", 1)[0].rstrip()


def main():
    sym = symbols()
    print("starting OpenOCD ...")
    ocd = Ocd()
    b = Board(ocd, sym)

    def teardown():
        ok = b.safe_off()
        ocd.close()
        print("\npower stage safe." if ok else
              "\n!! COULD NOT DISENGAGE - POWER DOWN THE BOARD MANUALLY !!")

    atexit.register(teardown)

    print("connected. `?` for commands, `q` to quit.")
    print("bringing the bridge up ...")
    b.bridge_up()
    b.pset(Board.MODE, 0)
    b.pset(Board.EN, 1)
    time.sleep(0.3)
    b.pset(Board.ZERO, 1)
    time.sleep(0.3)
    print(status_line(b.snapshot()))

    while True:
        try:
            line = input("\n> ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if not line:
            print(status_line(b.snapshot()))
            continue
        parts = line.split()
        c = parts[0].lower()
        args = parts[1:]

        def num(i=0, default=None):
            try:
                return int(round(float(args[i])))
            except (IndexError, ValueError):
                return default

        try:
            if c in ("q", "quit", "exit"):
                break
            elif c in ("?", "h", "help"):
                print(HELP)
                continue
            elif c == "e":
                b.pset(Board.EN, 1)
            elif c == "d":
                b.pset(Board.EN, 0)
            elif c == "i":
                b.pset(Board.MODE, 0)
            elif c == "stop":
                b.pset(Board.MODE, 0)
                b.pset(Board.EN, 0)
            elif c == "z":
                b.pset(Board.ZERO, 1)
                time.sleep(0.2)
            elif c == "t":
                v = num()
                if v is None:
                    print("usage: t <mA>"); continue
                b.pset(Board.TRQ, v)
                b.pset(Board.MODE, 1)
                b.pset(Board.EN, 1)
            elif c == "v":
                v = num()
                if v is None:
                    print("usage: v <deg/s>"); continue
                b.pset(Board.VCMD, v)
                b.pset(Board.MODE, 2)
                b.pset(Board.EN, 1)
            elif c in ("p", "pr"):
                v = num()
                if v is None:
                    print("usage: p <deg> (absolute)  |  pr <deg> (relative)")
                    continue
                # A relative move steps from the COMMANDED setpoint, not from
                # the measured position and not from the profile's live target.
                #
                # Measured position would accumulate the standing error at each
                # step. The profile target is worse: it is the ramp's current
                # value, so issuing "pr 90" while a move is still running adds
                # to wherever the ramp happens to have got to - measured once
                # as a 90 degree step landing on 124.5 instead of 180.
                # Chaining off the command makes "pr 90" mean "90 more than I
                # last asked for", whatever the shaft is doing right now.
                #
                # Absolute and relative are separate commands because "p -90"
                # is genuinely ambiguous otherwise.
                base = b.pget(Board.CMD) if c == "pr" else 0
                b.pset(Board.CMD, base + v * 10)
                b.pset(Board.MODE, 3)
                b.pset(Board.EN, 1)
            elif c == "lim":
                b.pset(Board.IMAX, num())
            elif c == "vmax":
                b.pset(Board.VMAX, num())
            elif c == "acc":
                b.pset(Board.ACC, num())
            elif c == "jerk":
                b.pset(Board.JERK, num())
            elif c == "gains":
                if len(args) != 3:
                    print("usage: gains <kp> <ki> <kd>"); continue
                for off, a in zip((Board.KP, Board.KI, Board.KD), args):
                    b.pset(off, int(round(float(a) * 1000)))
            elif c == "vgains":
                if len(args) != 2:
                    print("usage: vgains <vkp> <vki>"); continue
                for off, a in zip((Board.VKP, Board.VKI), args):
                    b.pset(off, int(round(float(a) * 1000)))
            elif c == "clear":
                ocd.write(b.flt, 0)
            elif c == "w":
                secs = num(0, 5) or 5
                end = time.time() + secs
                while time.time() < end:
                    sys.stdout.write("\r\033[K" + status_line(b.snapshot()))
                    sys.stdout.flush()
                    time.sleep(0.15)
                print()
                continue
            elif c == "s":
                pass
            else:
                print("unknown command %r - `?` for the list" % c)
                continue
            time.sleep(0.12)
            print(status_line(b.snapshot()))
        except IOError as exc:
            print("lost the debug link: %s" % exc)
            break


if __name__ == "__main__":
    main()
