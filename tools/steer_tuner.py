#!/usr/bin/env python3
"""
Steering PID tuner (SWD / pyOCD).

Talks to the running firmware over the ST-LINK by reading and writing the
global variables in motor_control.cpp - no UART, no firmware rebuild per tweak.

    g_kp, g_ki, g_kd   (float, write)  PID gains, per DEGREE of error
    g_target_angle     (float, write)  steering target, 0..180 deg, 90 = straight
    g_current_angle    (float, read)   measured steering angle, deg
    g_steer_duty       (float, read)   motor output, -100..100 %
    g_steer_raw        (uint32, read)  raw AS5600 count (for STEER_HOME_RAW calibration)
    g_steer_fault      (uint32, read)  1 = steering encoder not answering, motor held off
    g_steer_test_duty / g_steer_test_ms  (write)  open-loop pulse for the direction check
    g_enc_raw[4], g_enc_err[4], g_enc_ok  (read)  all four AS5600s behind the PCA9548A
    g_control_dt_ms    (uint32, read)  actual control period

Symbol addresses are taken from the firmware ELF, so they stay correct after
every rebuild. Point --elf at the .elf STM32CubeIDE built (usually
Debug/motor_controlh753zi.elf).

Usage:
    pip install -r tools/requirements.txt
    pyocd pack install stm32h753zitx            # once

    python tools/steer_tuner.py --elf Debug/motor_controlh753zi.elf          # live plot GUI
    python tools/steer_tuner.py --elf Debug/motor_controlh753zi.elf --cli    # text prompt
    python tools/steer_tuner.py --elf ... --log run1.csv                     # also log to CSV

Direction check ("dircheck" in the CLI, "Dir check" button in the GUI):
zeroes the gains, pulses the motor open-loop with +duty for a moment and
reports whether the steering angle went UP. If it went down, flip
STEER_MOTOR_DIR in motor_control.cpp. Do this before setting any gains.

Safety: if the measured angle leaves [-15, 195] deg (wheel past its lock),
the tuner zeroes all gains so the motor stops. That usually means
STEER_ENCODER_DIR or the DIR pin polarity is the wrong way round.
"""

import argparse
import csv
import struct
import sys
import time

try:
    from elftools.elf.elffile import ELFFile
    from elftools.elf.sections import SymbolTableSection
    from pyocd.core.helpers import ConnectHelper
except ImportError as e:
    sys.exit(f"Missing dependency ({e.name}). Run: pip install -r tools/requirements.txt")

MIN_ANGLE, HOME_ANGLE, MAX_ANGLE = 0.0, 90.0, 180.0
SAFE_LOW, SAFE_HIGH = -15.0, 195.0

NUM_ENCODERS = 4
FLOAT_VARS = ["g_kp", "g_ki", "g_kd", "g_target_angle", "g_current_angle", "g_steer_duty",
              "g_steer_test_duty"]
U32_VARS = ["g_steer_raw", "g_steer_fault", "g_steer_test_ms", "g_enc_raw", "g_enc_err",
            "g_enc_ok", "g_control_dt_ms"]


def load_symbols(elf_path):
    wanted = set(FLOAT_VARS + U32_VARS)
    found = {}
    with open(elf_path, "rb") as f:
        elf = ELFFile(f)
        for section in elf.iter_sections():
            if not isinstance(section, SymbolTableSection):
                continue
            for sym in section.iter_symbols():
                if sym.name in wanted:
                    found[sym.name] = sym["st_value"]
    missing = wanted - found.keys()
    if missing:
        sys.exit(f"Symbols not found in {elf_path}: {', '.join(sorted(missing))}\n"
                 "Is this an ELF built from the SteerMotor firmware?")
    return found


class Board:
    """Thin wrapper around pyOCD memory access for the tuner variables."""

    def __init__(self, target, symbols):
        self.t = target
        self.sym = symbols

    def read_f(self, name):
        return struct.unpack("<f", struct.pack("<I", self.t.read32(self.sym[name])))[0]

    def write_f(self, name, value):
        self.t.write32(self.sym[name], struct.unpack("<I", struct.pack("<f", float(value)))[0])

    def read_u32(self, name, index=0):
        return self.t.read32(self.sym[name] + 4 * index)

    def write_u32(self, name, value):
        self.t.write32(self.sym[name], int(value) & 0xFFFFFFFF)

    def encoders(self):
        raws = [self.read_u32("g_enc_raw", i) for i in range(NUM_ENCODERS)]
        errs = [self.read_u32("g_enc_err", i) for i in range(NUM_ENCODERS)]
        ok = self.read_u32("g_enc_ok")
        return raws, errs, ok

    def pulse(self, duty, ms):
        # duty first, then ms: the firmware starts the pulse when ms becomes > 0
        self.write_f("g_steer_test_duty", duty)
        self.write_u32("g_steer_test_ms", ms)

    def gains(self):
        return self.read_f("g_kp"), self.read_f("g_ki"), self.read_f("g_kd")

    def set_gains(self, kp, ki, kd):
        self.write_f("g_kp", kp)
        self.write_f("g_ki", ki)
        self.write_f("g_kd", kd)

    def kill(self):
        self.set_gains(0.0, 0.0, 0.0)  # PID output -> 0 -> duty 0

    def set_target(self, deg):
        deg = min(max(float(deg), MIN_ANGLE), MAX_ANGLE)
        self.write_f("g_target_angle", deg)
        return deg

    def sample(self):
        return {
            "target": self.read_f("g_target_angle"),
            "angle": self.read_f("g_current_angle"),
            "duty": self.read_f("g_steer_duty"),
            "raw": self.read_u32("g_steer_raw"),
            "fault": self.read_u32("g_steer_fault"),
        }


class Logger:
    def __init__(self, path):
        self.f = open(path, "w", newline="") if path else None
        self.w = None
        if self.f:
            self.w = csv.writer(self.f)
            self.w.writerow(["t", "target", "angle", "duty", "raw", "kp", "ki", "kd"])

    def log(self, t, s, gains):
        if self.w:
            self.w.writerow([f"{t:.4f}", f"{s['target']:.3f}", f"{s['angle']:.3f}",
                             f"{s['duty']:.2f}", s["raw"], *(f"{g:.4f}" for g in gains)])

    def close(self):
        if self.f:
            self.f.close()


def check_safety(board, s):
    if not (SAFE_LOW <= s["angle"] <= SAFE_HIGH):
        board.kill()
        print(f"\n!! Angle {s['angle']:.1f} deg is past the steering lock - gains zeroed.\n"
              "   Check STEER_ENCODER_DIR / DIR pin polarity and STEER_HOME_RAW.")
        return False
    return True


def avg_angle(board, n=5, gap=0.01):
    vals = []
    for _ in range(n):
        vals.append(board.read_f("g_current_angle"))
        time.sleep(gap)
    return sum(vals) / len(vals)


def direction_check(board, duty=30.0, ms=250):
    """Open-loop pulse, returns a human-readable verdict."""
    if board.read_u32("g_steer_fault"):
        return "Steering encoder is not answering (g_steer_fault = 1) - fix that first."
    board.kill()                      # no PID fighting / running away afterwards
    a0 = avg_angle(board)
    r0 = board.read_u32("g_steer_raw")
    board.pulse(abs(duty), ms)
    time.sleep(ms / 1000.0 + 0.15)    # pulse + settle
    a1 = avg_angle(board)
    r1 = board.read_u32("g_steer_raw")
    delta = a1 - a0
    board.pulse(-abs(duty), ms)       # roughly undo the move
    time.sleep(ms / 1000.0 + 0.15)

    head = (f"+{abs(duty):.0f}% for {ms} ms: angle {a0:.1f} -> {a1:.1f} deg "
            f"({delta:+.1f}), raw {r0} -> {r1}\n")
    if abs(delta) < 1.0:
        return head + ("Wheel barely moved. Raise duty/time (dircheck 40 400), check motor power, "
                       "and that STEER_ENC_CH is the encoder on THIS wheel.")
    if delta > 0:
        return head + ("OK: positive output raises the angle - STEER_MOTOR_DIR is right.\n"
                       "Also confirm by eye the wheel turned towards the side your Ackermann "
                       "controller calls 180 deg; if not, flip STEER_ENCODER_DIR AND STEER_MOTOR_DIR.")
    return head + ("WRONG WAY: positive output lowers the angle. Flip STEER_MOTOR_DIR "
                   "(+1 <-> -1) in motor_control.cpp, rebuild, flash, re-run dircheck.\n"
                   "Do NOT set gains until this passes - the loop would run to the lock.")


# --------------------------------------------------------------------------- CLI

CLI_HELP = """Commands:
  kp <v> | ki <v> | kd <v>     set one gain
  g <kp> <ki> <kd>             set all gains
  t <deg>                      set target (0..180, 90 = straight)
  home                         target = 90
  step <a> <b> [sec]           go to a, then b, recording for [sec] (default 2) and print stats
  watch [sec]                  print live readings for [sec] (default 3)
  s                            one-line status
  enc                          all 4 encoders: raw count, read errors, OK flag
  dircheck [duty] [ms]         motor polarity test (zeroes gains first; default 30 % / 250 ms)
  kill                         zero all gains (motor stops)
  q                            quit (gains are zeroed on exit)"""


def step_stats(samples, a, b):
    """Rise time (10-90 %), overshoot and final error for a step from a to b."""
    if not samples:
        return "no samples"
    span = b - a
    sign = 1.0 if span >= 0 else -1.0
    t0 = samples[0][0]
    t10 = t90 = None
    peak = None
    for t, ang in samples:
        prog = (ang - a) / span if span else 1.0
        if t10 is None and prog >= 0.1:
            t10 = t
        if t90 is None and prog >= 0.9:
            t90 = t
        val = sign * ang
        peak = val if peak is None or val > peak else peak
    overshoot = max(0.0, sign * peak - sign * b) if peak is not None else 0.0
    final = samples[-1][1]
    rise = f"{(t90 - t10) * 1000:.0f} ms" if t10 is not None and t90 is not None else "never reached 90 %"
    reach = f"{(t90 - t0) * 1000:.0f} ms" if t90 is not None else "-"
    return (f"rise(10-90%) {rise} | time to 90% {reach} | overshoot {overshoot:.2f} deg | "
            f"final {final:.2f} deg (err {b - final:+.2f})")


def run_cli(board, logger, period):
    print(CLI_HELP)
    t_start = time.monotonic()

    def record(duration):
        out = []
        end = time.monotonic() + duration
        while time.monotonic() < end:
            s = board.sample()
            now = time.monotonic() - t_start
            logger.log(now, s, board.gains())
            out.append((now, s))
            if not check_safety(board, s):
                break
            time.sleep(period)
        return out

    while True:
        try:
            line = input("tuner> ").strip().split()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if not line:
            continue
        cmd, args = line[0].lower(), line[1:]
        try:
            if cmd in ("q", "quit", "exit"):
                break
            elif cmd in ("kp", "ki", "kd"):
                g = list(board.gains())
                g[["kp", "ki", "kd"].index(cmd)] = float(args[0])
                board.set_gains(*g)
                print("gains kp=%.4f ki=%.4f kd=%.4f" % tuple(g))
            elif cmd == "g":
                board.set_gains(*map(float, args[:3]))
                print("gains kp=%.4f ki=%.4f kd=%.4f" % board.gains())
            elif cmd == "t":
                print(f"target {board.set_target(args[0]):.2f} deg")
            elif cmd == "home":
                board.set_target(HOME_ANGLE)
                print("target 90.00 deg")
            elif cmd == "step":
                a, b = float(args[0]), float(args[1])
                dur = float(args[2]) if len(args) > 2 else 2.0
                board.set_target(a)
                print(f"moving to {a:.1f} ...")
                record(dur)
                board.set_target(b)
                rec = record(dur)
                pts = [(t, s["angle"]) for t, s in rec]
                print(step_stats(pts, a, b))
                print("note: firmware rate-limits the setpoint (90 deg/s default), "
                      "so rise time includes the ramp.")
            elif cmd == "watch":
                dur = float(args[0]) if args else 3.0
                end = time.monotonic() + dur
                while time.monotonic() < end:
                    s = board.sample()
                    logger.log(time.monotonic() - t_start, s, board.gains())
                    print(f"\r target {s['target']:7.2f}  angle {s['angle']:7.2f}  "
                          f"duty {s['duty']:7.2f}%  raw {s['raw']:4d}", end="", flush=True)
                    if not check_safety(board, s):
                        break
                    time.sleep(period)
                print()
            elif cmd == "s":
                s = board.sample()
                print(f"target {s['target']:.2f}  angle {s['angle']:.2f}  duty {s['duty']:.2f}%  "
                      f"raw {s['raw']}{'  ENCODER FAULT' if s['fault'] else ''}  | kp={board.gains()[0]:.4f} ki={board.gains()[1]:.4f} "
                      f"kd={board.gains()[2]:.4f}")
            elif cmd == "enc":
                raws, errs, ok = board.encoders()
                for ch in range(NUM_ENCODERS):
                    flag = "OK " if ok & (1 << ch) else "ERR"
                    print(f"  ch{ch}: raw {raws[ch]:4d} ({raws[ch] * 360 / 4096:6.1f} deg)  "
                          f"{flag}  errors {errs[ch]}")
                print(f"  control period {board.read_u32('g_control_dt_ms')} ms")
            elif cmd == "dircheck":
                duty = float(args[0]) if args else 30.0
                ms = int(float(args[1])) if len(args) > 1 else 250
                print(direction_check(board, duty, ms))
            elif cmd == "kill":
                board.kill()
                print("gains zeroed")
            else:
                print(CLI_HELP)
        except (IndexError, ValueError):
            print("bad arguments\n" + CLI_HELP)


# --------------------------------------------------------------------------- GUI

def run_gui(board, logger, period, window):
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    from matplotlib.widgets import Button, Slider, TextBox

    fig = plt.figure(figsize=(11, 7))
    fig.canvas.manager.set_window_title("Steering PID tuner")
    ax_ang = fig.add_axes([0.07, 0.50, 0.90, 0.45])
    ax_duty = fig.add_axes([0.07, 0.30, 0.90, 0.15], sharex=ax_ang)

    (l_tgt,) = ax_ang.plot([], [], "--", label="target")
    (l_ang,) = ax_ang.plot([], [], label="angle")
    (l_duty,) = ax_duty.plot([], [], color="tab:red")
    ax_ang.set_ylabel("steering angle (deg)")
    ax_ang.axhline(HOME_ANGLE, color="0.8", lw=0.8, zorder=0)
    ax_ang.legend(loc="upper left")
    ax_ang.grid(alpha=0.3)
    ax_duty.set_ylabel("duty (%)")
    ax_duty.set_xlabel("time (s)")
    ax_duty.set_ylim(-105, 105)
    ax_duty.grid(alpha=0.3)
    status = ax_ang.text(0.99, 0.03, "", transform=ax_ang.transAxes, ha="right",
                         family="monospace", fontsize=9)

    kp0, ki0, kd0 = board.gains()
    tb_kp = TextBox(fig.add_axes([0.10, 0.19, 0.10, 0.045]), "Kp ", initial=f"{kp0:g}")
    tb_ki = TextBox(fig.add_axes([0.27, 0.19, 0.10, 0.045]), "Ki ", initial=f"{ki0:g}")
    tb_kd = TextBox(fig.add_axes([0.44, 0.19, 0.10, 0.045]), "Kd ", initial=f"{kd0:g}")
    b_apply = Button(fig.add_axes([0.57, 0.19, 0.10, 0.045]), "Apply gains")
    b_kill = Button(fig.add_axes([0.85, 0.19, 0.12, 0.045]), "KILL (gains=0)", color="#f4b4b4")

    tgt0 = board.read_f("g_target_angle")
    sl_tgt = Slider(fig.add_axes([0.10, 0.11, 0.57, 0.035]), "Target", MIN_ANGLE, MAX_ANGLE,
                    valinit=min(max(tgt0, MIN_ANGLE), MAX_ANGLE), valstep=0.5)
    b_home = Button(fig.add_axes([0.72, 0.11, 0.10, 0.045]), "Home (90)")

    tb_a = TextBox(fig.add_axes([0.10, 0.03, 0.07, 0.045]), "Step A ", initial="60")
    tb_b = TextBox(fig.add_axes([0.25, 0.03, 0.07, 0.045]), "B ", initial="120")
    b_step = Button(fig.add_axes([0.35, 0.03, 0.14, 0.045]), "Step A <-> B")
    b_clear = Button(fig.add_axes([0.52, 0.03, 0.10, 0.045]), "Clear plot")
    b_dir = Button(fig.add_axes([0.72, 0.03, 0.12, 0.045]), "Dir check")

    ts, tgts, angs, duties = [], [], [], []
    t_start = time.monotonic()
    state = {"step_to_b": True, "syncing": False}

    def apply_gains(_=None):
        try:
            board.set_gains(float(tb_kp.text), float(tb_ki.text), float(tb_kd.text))
        except ValueError:
            print("gains must be numbers")

    def set_target(val):
        if not state["syncing"]:
            board.set_target(val)

    def go(val):
        state["syncing"] = True
        sl_tgt.set_val(val)
        state["syncing"] = False
        board.set_target(val)

    def do_step(_):
        try:
            a, b = float(tb_a.text), float(tb_b.text)
        except ValueError:
            print("step angles must be numbers")
            return
        go(b if state["step_to_b"] else a)
        state["step_to_b"] = not state["step_to_b"]

    def clear(_):
        for buf in (ts, tgts, angs, duties):
            buf.clear()

    b_apply.on_clicked(apply_gains)
    for tb in (tb_kp, tb_ki, tb_kd):
        tb.on_submit(apply_gains)
    b_kill.on_clicked(lambda _: board.kill())
    sl_tgt.on_changed(set_target)
    b_home.on_clicked(lambda _: go(HOME_ANGLE))
    b_step.on_clicked(do_step)
    b_clear.on_clicked(clear)

    def dircheck(_):
        msg = direction_check(board)
        print(msg)
        for tb in (tb_kp, tb_ki, tb_kd):
            tb.set_val("0")           # direction_check zeroed the gains
        status.set_text(msg.splitlines()[-1][:90])

    b_dir.on_clicked(dircheck)

    def tick(_):
        try:
            s = board.sample()
        except Exception as e:  # probe unplugged, target reset, ...
            status.set_text(f"SWD read failed: {e}")
            return l_tgt, l_ang, l_duty, status
        now = time.monotonic() - t_start
        g = board.gains()
        logger.log(now, s, g)
        check_safety(board, s)

        ts.append(now)
        tgts.append(s["target"])
        angs.append(s["angle"])
        duties.append(s["duty"])
        while ts and ts[0] < now - window:
            for buf in (ts, tgts, angs, duties):
                buf.pop(0)

        l_tgt.set_data(ts, tgts)
        l_ang.set_data(ts, angs)
        l_duty.set_data(ts, duties)
        ax_ang.set_xlim(max(0.0, now - window), max(window, now))
        lo, hi = min(tgts + angs), max(tgts + angs)
        pad = max(2.0, 0.1 * (hi - lo))
        ax_ang.set_ylim(lo - pad, hi + pad)
        raws, _, ok = board.encoders()
        enc = "  ".join(f"{r:4d}" + ("" if ok & (1 << i) else "!") for i, r in enumerate(raws))
        status.set_text(f"angle {s['angle']:7.2f}  err {s['target'] - s['angle']:+7.2f}  "
                        f"duty {s['duty']:+6.1f}%  raw {s['raw']:4d}"
                        f"{'  ENCODER FAULT' if s['fault'] else ''}\n"
                        f"enc0-3 {enc}   kp={g[0]:.4g}  ki={g[1]:.4g}  kd={g[2]:.4g}")
        return l_tgt, l_ang, l_duty, status

    _anim = FuncAnimation(fig, tick, interval=int(period * 1000), cache_frame_data=False)
    plt.show()


# --------------------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser(description="Steering PID tuner over SWD (pyOCD)")
    ap.add_argument("--elf", required=True, help="firmware ELF (e.g. Debug/motor_controlh753zi.elf)")
    ap.add_argument("--target", default="stm32h753zitx", help="pyOCD target type")
    ap.add_argument("--probe", default=None, help="probe unique ID (if several are connected)")
    ap.add_argument("--cli", action="store_true", help="text prompt instead of the plot GUI")
    ap.add_argument("--rate", type=float, default=50.0, help="sample rate in Hz (default 50)")
    ap.add_argument("--window", type=float, default=10.0, help="plot window in seconds")
    ap.add_argument("--log", default=None, help="write samples to this CSV file")
    ap.add_argument("--keep-gains", action="store_true",
                    help="don't zero the gains on exit (motor keeps holding position)")
    args = ap.parse_args()

    symbols = load_symbols(args.elf)
    period = 1.0 / max(args.rate, 1.0)

    # attach: don't halt or reset the running firmware
    session = ConnectHelper.session_with_chosen_probe(
        unique_id=args.probe, target_override=args.target,
        connect_mode="attach", options={"frequency": 4000000})
    if session is None:
        sys.exit("No debug probe found.")

    logger = Logger(args.log)
    with session:
        board = Board(session.target, symbols)
        s = board.sample()
        print(f"Connected. angle {s['angle']:.2f} deg, target {s['target']:.2f} deg, raw {s['raw']}")
        print("kp=%.4f ki=%.4f kd=%.4f (per degree)" % board.gains())
        try:
            if args.cli:
                run_cli(board, logger, period)
            else:
                run_gui(board, logger, period, args.window)
        finally:
            if not args.keep_gains:
                board.kill()
                print("Gains zeroed on exit.")
            logger.close()


if __name__ == "__main__":
    main()
