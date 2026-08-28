
# Motor PID Control - PositionMotor / SpeedMotor

Two independent motor classes, each wrapping its own PID loop, as requested:

- **`PositionMotor`** - closed-loop position control from an absolute encoder. Output is PWM duty cycle + direction.
- **`SpeedMotor`** - closed-loop speed control from an RPM reading. Output is PWM duty cycle + direction.

Both share a common, hardware-agnostic **`PID`** class (not a "motor" itself, just the control math), so tuning logic lives in one place and both classes benefit from the same anti-windup handling.

## Files

| File | Purpose |
|---|---|
| `PID.h` / `PID.cpp` | Generic PID controller: gains, output clamping, anti-windup, optional wraparound handling |
| `PositionMotor.h` / `.cpp` | Position-controlled motor (absolute encoder feedback) |
| `SpeedMotor.h` / `.cpp` | Speed-controlled motor (RPM feedback) |
| `motor_control.h` | `extern "C"` declarations so plain-C `main.c` can call into the C++ code |
| `motor_control_example.cpp` | Example wiring it all together - **replace the placeholders with your actual hardware** |

## Design choices worth knowing about

**Direction pins are dual-pin (IN1/IN2 style).** This matches common cheap H-bridge drivers (L298N, etc.). If your driver instead takes a single PWM + single DIR pin, the `driveOutput()` private method in each class is the only place that needs to change.

**Sensor reading is not inside these classes.** You call `update(currentValue, dt)` every control tick and pass in whatever your absolute encoder / speed calculation gives you. This keeps the classes portable across SPI encoders, I2C encoders, ADC-based ones, etc. — you're not locked into one sensor implementation.

**Position wraparound is handled automatically.** An absolute encoder reporting 0-4095 (or 0-360°) wraps around, so `PositionMotor` tells the PID controller the full-scale range (`positionRange` constructor arg) and the error calculation always takes the shortest path — a move from 4090 to 10 (on a 4096-count encoder) is treated as a 16-count step, not a 4080-count one.

**Anti-windup via back-calculation.** If the output saturates at ±100%, the integral term stops accumulating further in that direction — otherwise the integral term "winds up" while the motor is already maxed out, causing overshoot when the target is finally reached.

## Integration with CubeIDE

CubeMX regenerates `main.c` as plain C every time you touch the `.ioc` file, so don't convert `main.c` itself to C++. Instead:

1. Add all these files to your project (CubeIDE auto-detects `.cpp`/`.h` and builds them with the C++ toolchain — no project settings changes needed).
2. `#include "motor_control.h"` from `main.c`.
3. Call `MotorControl_Init();` after your `MX_TIMx_Init()` / `MX_GPIO_Init()` calls.
4. Call `MotorControl_Update(dt);` every control cycle — either:
   - from a hardware timer interrupt (recommended — gives you a consistent `dt`, e.g. `MotorControl_Update(0.01f);` on a 10ms timer, or
   - from your main `while(1)` loop, measuring elapsed time yourself with `HAL_GetTick()`.

## Tuning

Start with `Ki = Kd = 0`, raise `Kp` until the motor responds promptly but starts to oscillate, then back off ~30-50%. Add `Kd` to damp any remaining oscillation/overshoot. Add `Ki` last, in small steps, only if there's a persistent steady-state error (common on position control fighting gravity/friction) — too much `Ki` is the most common cause of slow oscillation on rovers.

Position and speed loops usually need very different gains — don't reuse the same numbers for both.

## Not included (hardware-specific, up to you)

- Actual encoder/speed sensor reading code (SPI/I2C/ADC drivers, pulse counting)
- CubeMX `.ioc` timer/GPIO configuration
- Safety limits (e.g. current sensing, stall detection, soft limits on steering range) — worth adding once the basic loop is working
