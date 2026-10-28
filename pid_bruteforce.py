"""Brute-force PID grid search on the real motor over SWD.

For every (Kp, Ki, Kd) in the grid: stop motor, reset PID, step to --target RPM,
log RPM, compute settling time. Results -> CSV, best gains printed.

Needs firmware with g_pid_reset (motor_control.cpp). Reflash before running.

Example:
  python pid_bruteforce.py Debug/motor_control3.elf --target 50 \
      --kp 0.5:5:10 --ki 0:5:6 --kd 0:0.2:3
  python pid_bruteforce.py --selftest
"""
import argparse
import csv
import itertools
import struct
import sys
import time


def f2u(v):
    return struct.unpack('<I', struct.pack('<f', float(v)))[0]


def u2f(v):
    return struct.unpack('<f', struct.pack('<I', v))[0]


def frange(spec):
    """'start:stop:count' -> evenly spaced list (inclusive). Single number -> [number]."""
    parts = [float(x) for x in spec.split(':')]
    if len(parts) == 1:
        return parts
    a, b, n = parts[0], parts[1], int(parts[2])
    return [a] if n <= 1 else [a + (b - a) * i / (n - 1) for i in range(n)]


def settling_time(samples, target, band):
    """samples: [(t, rpm)] with t from step. Returns time after which rpm stays
    within +/-band*target, or None if it ends outside the band (never settled)."""
    tol = abs(target) * band
    if not samples or abs(samples[-1][1] - target) > tol:
        return None
    last_out = 0.0
    for t, y in samples:
        if abs(y - target) > tol:
            last_out = t
    return last_out


def overshoot_pct(samples, target):
    peak = max(y for _, y in samples)
    return max(0.0, (peak - target) / target * 100.0)


class Board:
    def __init__(self, elf, mcu):
        from pyocd.core.helpers import ConnectHelper
        from pyocd.debug.elf.symbols import ELFSymbolProvider
        self.session = ConnectHelper.session_with_chosen_probe(target_override=mcu, options={"elf": elf})
        if self.session is None:
            raise SystemExit("no debug probe found - plug in the Nucleo USB and close CubeIDE's debug session")
        self.session.open()
        self.t = self.session.target
        if self.t.is_halted():
            self.t.resume()
        if not self.t.elf:
            self.t.elf = elf  # same fallback as pid_tuner.py
        syms = ELFSymbolProvider(self.t.elf)
        self.a = {}
        for s in ("g_kp", "g_ki", "g_kd", "g_target_rpm", "g_current_rpm", "g_pid_reset"):
            addr = syms.get_symbol_value(s)
            if addr is None:
                raise SystemExit(f"symbol {s} not in ELF - rebuild & reflash the firmware")
            self.a[s] = addr

    def wf(self, sym, v):
        self.t.write32(self.a[sym], f2u(v))

    def rpm(self):
        return u2f(self.t.read32(self.a["g_current_rpm"]))

    def gains(self, kp, ki, kd):
        self.wf("g_kp", kp); self.wf("g_ki", ki); self.wf("g_kd", kd)

    def reset_pid(self):
        self.t.write32(self.a["g_pid_reset"], 1)
        t0 = time.monotonic()
        while self.t.read32(self.a["g_pid_reset"]) and time.monotonic() - t0 < 0.5:
            pass  # MCU clears it on the next 10 ms tick

    def safe_stop(self):
        self.wf("g_target_rpm", 0.0)
        self.gains(0, 0, 0)

    def wait_stopped(self, thresh=1.0, hold=0.3, timeout=5.0):
        t0 = still = time.monotonic()
        while time.monotonic() - t0 < timeout:
            if abs(self.rpm()) > thresh:
                still = time.monotonic()
            elif time.monotonic() - still >= hold:
                return True
            time.sleep(0.01)
        return False

    def trial(self, kp, ki, kd, target, duration, abort_rpm):
        self.safe_stop()                 # gains 0 -> output 0 -> coast
        self.wait_stopped()
        self.gains(kp, ki, kd)           # target still 0, rpm 0 -> no error builds up
        self.reset_pid()
        self.wf("g_target_rpm", target)
        t0 = time.monotonic()
        samples = []
        while (t := time.monotonic() - t0) < duration:
            y = self.rpm()
            samples.append((t, y))
            if abs(y) > abort_rpm or y * target < -0.2 * target * target:
                break                    # runaway or reversing hard: cut it short, it won't settle anyway
            time.sleep(0.005)            # MCU updates every 10 ms
        self.safe_stop()
        return samples


def selftest():
    s = [(0.0, 0), (0.1, 50), (0.2, 105), (0.3, 99), (0.4, 100.5), (0.5, 100)]
    assert settling_time(s, 100, 0.02) == 0.2
    assert settling_time(s + [(0.6, 90)], 100, 0.02) is None
    assert abs(overshoot_pct(s, 100) - 5.0) < 1e-9
    assert frange("0:1:3") == [0.0, 0.5, 1.0] and frange("2") == [2.0]
    print("selftest ok")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("elf", nargs="?")
    ap.add_argument("--mcu", default="stm32f410rbtx")
    ap.add_argument("--target", type=float, default=50.0, help="step setpoint, RPM (keep ~half of motor max so PID has headroom)")
    ap.add_argument("--kp", default="0.5:5:10", help="start:stop:count or single value")
    ap.add_argument("--ki", default="0:5:6")
    ap.add_argument("--kd", default="0")
    ap.add_argument("--duration", type=float, default=3.0, help="seconds recorded per trial")
    ap.add_argument("--band", type=float, default=0.02, help="settling band, fraction of target (raise if RPM is noisy)")
    ap.add_argument("--abort-rpm", type=float, default=None, help="abort trial above this |RPM| (default 2x target)")
    ap.add_argument("--rest", type=float, default=0.0, help="extra seconds idle between trials (motor/driver cooling)")
    ap.add_argument("--out", default="pid_bruteforce.csv")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()

    if a.selftest:
        return selftest()
    if not a.elf:
        ap.error("elf path required")

    grid = list(itertools.product(frange(a.kp), frange(a.ki), frange(a.kd)))
    est = len(grid) * (a.duration + 1.5 + a.rest) / 60
    print(f"{len(grid)} trials, ~{est:.0f} min. Ctrl+C stops safely (partial results saved).")

    abort_rpm = a.abort_rpm or 2 * abs(a.target)
    board = Board(a.elf, a.mcu)
    rows = []
    try:
        for i, (kp, ki, kd) in enumerate(grid, 1):
            s = board.trial(kp, ki, kd, a.target, a.duration, abort_rpm)
            ts = settling_time(s, a.target, a.band) if s[-1][0] >= a.duration * 0.9 else None
            os_ = overshoot_pct(s, a.target)
            rows.append((kp, ki, kd, ts, os_))
            print(f"[{i}/{len(grid)}] Kp={kp:.3f} Ki={ki:.3f} Kd={kd:.3f} -> "
                  f"settle={'--' if ts is None else f'{ts:.3f}s'} overshoot={os_:.1f}%")
            time.sleep(a.rest)
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        board.safe_stop()
        board.session.close()

    with open(a.out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["kp", "ki", "kd", "settling_s", "overshoot_pct"])
        w.writerows(rows)

    ok = sorted((r for r in rows if r[3] is not None), key=lambda r: (r[3], r[4]))
    print(f"\nsaved {len(rows)} trials to {a.out}")
    if not ok:
        print("no combination settled - widen the grid, lengthen --duration, or loosen --band")
        return
    print("best (settling, then overshoot):")
    for kp, ki, kd, ts, os_ in ok[:5]:
        print(f"  Kp={kp:.3f} Ki={ki:.3f} Kd={kd:.3f}  settle={ts:.3f}s  overshoot={os_:.1f}%")


if __name__ == "__main__":
    sys.exit(main())
