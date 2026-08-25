# makolongfin2 — working notes for Claude Code

## Commit and PR convention

**Do not add AI attribution to anything.** No `Co-Authored-By: Claude ...`
trailer on commits, no "Generated with Claude Code" line in PR bodies, no
AI-authorship notes in code comments.

This overrides Claude Code's default of appending a Co-Authored-By trailer. The
trailers were stripped from all history and force-pushed on 2026-08-24; adding
them back would undo that.

## Before debugging a board that "doesn't run"

Read `HARDWARE_NOTES.md` first. Several of this board's failure modes look like
firmware bugs and are not — most notably PB8 doubling as BOOT0 and FDCAN1_RX,
which makes the chip boot the ST bootloader instead of flash.

## Writing OpenOCD scripts

Derive symbol addresses from the ELF at run time (`arm-none-eabi-nm`), the way
every tool in `tools/` does. Never bake an address into a script. Any code-size
change moves the BSS layout, and a stale address does not error — it reads or
writes a different variable and looks exactly like the firmware ignoring you.
This has cost real debugging time here, once by silently overwriting the
current-sense zero calibration and tripping overcurrent on a stationary motor.

## Touching the control ISR

The 30 kHz HRTIM ISR has a hard 33.3 us deadline; overrun it and control steps
are lost. It currently runs ~28 us — 84% loaded, so the margin is thin.
`Core/Src/{foc,position,haptic,csense,encoder,motor_pwm,main}.c` are built at
`-O2` for this reason while the rest of the project stays at `-O0` — see the
comment in `CMakeLists.txt` before changing that.

The deadline is `1 / PWM_FREQ_HZ` (`Core/Inc/motor_pwm.h`), not a constant.
Anything that assumes a control-loop rate must derive it from that symbol.
