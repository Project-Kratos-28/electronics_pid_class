# PID brute-force tuner

`pid_bruteforce.py` tries every Kp/Ki/Kd combination in a grid on the real motor (over SWD),
measures the settling time of a speed step for each one, and prints the best gains.

## Setup (once)

1. Install Python deps: `pip install pyocd`
2. Rebuild and flash the firmware in CubeIDE. The script needs the `g_pid_reset` variable,
   which older builds don't have.
3. Plug in the Nucleo over USB and close CubeIDE's debug session (only one tool can hold the probe).

Check that the script works: `python pid_bruteforce.py --selftest`

## Run

```
python pid_bruteforce.py Debug/motor_control3.elf --target 50 --abort-rpm 90 \
    --kp 0.5:5:10 --ki 0:5:6 --kd 0 --rest 1
```

Ranges are `start:stop:count`, so `--kp 0.5:5:10` means 10 values from 0.5 to 5.
A single number (`--kd 0`) means "just this value". The script prints how many trials
and roughly how long it will take before starting.

Ctrl+C stops the motor safely and keeps the results so far.

| Option | Default | What it does |
|---|---|---|
| `--target` | 50 | Step setpoint in RPM. Keep it around half the motor's max (100 RPM) so the PID has headroom. |
| `--abort-rpm` | 2× target | Cuts a trial short above this speed. Use 90 for this motor. |
| `--duration` | 3 | Seconds recorded per trial. Raise it if slow gains never settle in time. |
| `--band` | 0.02 | Settling band (±2% of target). Use 0.05 if RPM readings are noisy and nothing settles. |
| `--rest` | 0 | Idle seconds between trials, to let the motor and driver cool. |
| `--out` | `pid_bruteforce.csv` | Where results are saved. |

## Results

Each trial prints its settling time (`--` means it never settled) and overshoot.
At the end you get the top 5, sorted by settling time and then overshoot, and every
trial is saved to the CSV.

Tip: start with a coarse grid, then run again with a narrow grid around the best result.

## Safety (11.1 V 3S LiPo + 100 RPM geared motor)

- Put a fuse between the LiPo and the driver. The battery won't limit current on a stall or short.
- Bad gains reverse the motor at full power, which is hard on the gearbox. Keep `--target` moderate
  and use `--rest 1` or more on long runs.
- Stop if the motor gets too hot to keep your hand on.
- Stop before the pack drops to about 10.5 V (3.5 V per cell). Motor speed per % duty drops as the
  battery sags, so recheck your best gains on a charged pack before using them.
