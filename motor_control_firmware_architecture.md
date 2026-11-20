# STM32 Nucleo-F446RE Closed-Loop DC Gearmotor Controller

This repository contains the embedded C/C++ firmware for closed-loop speed and position control of a DC gearmotor using an **STM32 Nucleo-F446RE** microcontroller and a **Cytron MDD20A** motor driver. The firmware leverages hardware timers for deterministic execution, quadrature encoder decoding, high-frequency PWM generation, and live parameter tuning over Serial Wire Debug (SWD) via `pyocd`.

---

## Hardware & Clock Topology

* **Microcontroller**: STM32F446RE ($180\text{ MHz}$ `SYSCLK` / `HCLK`)
* **Timer Bus Clocks**:
  * **APB1 Timer Clock**: $90\text{ MHz}$ (drives `TIM5` and `TIM6`)
  * **APB2 Timer Clock**: $180\text{ MHz}$ (drives `TIM1`)

### Pin & Peripheral Mapping

| Peripheral / Pin | Function | Configuration Details |
| :--- | :--- | :--- |
| **`TIM1_CH1` (`PA8`)** | Motor PWM Output | `PSC = 0`, `ARR = 4095` ($\approx 43.9\text{ kHz}$ switching frequency) |
| **`TIM5` (`PA0` / `PA1`)** | Quadrature Encoder | 32-bit counter, $4\times$ edge decoding ($13\text{ PPR} \times 181:1 \times 4 = 9,412\text{ ticks/rev}$) |
| **`TIM6`** | Control Loop Scheduler | Internal periodic timer triggering `HAL_TIM_PeriodElapsedCallback()` |
| **`GPIOB` (`PB4`)** | Drive Motor Direction | `HIGH` = Forward, `LOW` = Reverse |
| **`GPIOB` (`PB2`)** | Elbow Motor Direction | Reserved for `PositionMotor` instance |
| **`I2C1` (`PB8` / `PB9`)** | AS5600 Absolute Sensor | $12$-bit magnetic rotary encoder (`0x36` I2C address) |

---

## System Architecture Overview

The firmware uses a **dual-layer architecture** separating peripheral initialization from deterministic real-time execution:

1. **Main Thread (`main.c`)**: Initializes system clocks, HAL drivers, and peripheral hardware, then idles in an empty `while(1)` loop.
2. **Interrupt Service Routine (`TIM6` ISR)**: Executes the entire sensing, PID computation, and PWM actuation pipeline on every timer overflow interrupt.
3. **Live SWD Interface (`motor_control.cpp`)**: Exposes `volatile` global variables (`g_kp`, `g_ki`, `g_kd`, `g_target_rpm`, `g_current_rpm`, `g_output_duty`, `g_raw_ticks`) so a host Python/Tkinter GUI can read telemetry and adjust PID gains in real time via `pyocd` without reflashing or resetting the board.

---

## Phase 1: Boot & Initialization Sequence

On power-up or reset, execution flows sequentially through `main.c` to configure hardware timers and instantiate motor control objects:

```text
main() [main.c]
 ├── HAL_Init()                                           // Reset peripherals & SysTick
 ├── SystemClock_Config()                                 // Configure 180 MHz SYSCLK (PLL)
 ├── Peripheral Initialization
 │    ├── MX_GPIO_Init()                                  // Configure DIR pins (PB4, PB2)
 │    ├── MX_TIM1_Init()                                  // Configure PWM Timer (PA8)
 │    ├── MX_TIM5_Init()                                  // Configure 32-bit Quadrature Encoder (PA0/PA1)
 │    ├── MX_TIM6_Init()                                  // Configure Control Loop Scheduler
 │    └── MX_I2C1_Init()                                  // Configure AS5600 I2C bus
 │
 ├── Hardware Peripherals Start
 │    ├── HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1)        // Enable PA8 PWM output
 │    └── HAL_TIM_Base_Start_IT(&htim6)                   // Enable TIM6 periodic interrupt
 │
 └── Subsystem Initialization
      ├── MotorControl_Init() [motor_control.cpp]
      │    ├── HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL) // Start hardware tick counting
      │    └── driveMotor.begin() [SpeedMotor.cpp]
      │         ├── HAL_TIM_PWM_Start()                   // Enable PWM hardware channel
      │         ├── __HAL_TIM_MOE_ENABLE()                // Enable TIM1 Main Output (MOE)
      │         └── stop()                                // Set DIR LOW, PWM duty = 0%
      │
      └── Elbow_Control_Init() [motor_control.cpp]
           ├── elbowMotor.disableWrap() [PositionMotor.cpp] // Disable circular angle wrap
           └── elbowMotor.begin() [PositionMotor.cpp]       // Initialize position motor hardware
```

---

## Phase 2: Runtime Control Loop (Interrupt Execution Flow)

Each time `TIM6` overflows, hardware triggers `HAL_TIM_PeriodElapsedCallback()` in `main.c`. The complete closed-loop control cycle executes inside this interrupt context:

```text
HAL_TIM_PeriodElapsedCallback() [main.c]  <--- Triggered by TIM6 Hardware Interrupt
 │
 └── MotorControl_Update(dt_seconds) [motor_control.cpp]
      │
      ├── 1. Sync Live SWD Parameters
      │    ├── driveMotor.setGains(g_kp, g_ki, g_kd)      --> pid_.setGains() [PID.cpp]
      │    └── driveMotor.setTargetSpeed(g_target_rpm)    --> pid_.setSetpoint() [PID.cpp]
      │
      ├── 2. Sample Feedback Sensor
      │    └── ReadDriveSpeedRPM(dt_seconds) [motor_control.cpp]
      │         ├── Read raw TIM5 counter: __HAL_TIM_GET_COUNTER(&htim5)
      │         ├── Store g_raw_ticks = current_count
      │         ├── Shift sliding ring buffer (SPEED_WINDOW = 65)
      │         ├── Compute delta_ticks = current_count - oldest
      │         └── Return calculated RPM over window duration
      │
      ├── 3. Execute Closed-Loop PID Algorithm
      │    └── driveMotor.update(g_current_rpm, dt_seconds) [SpeedMotor.cpp]
      │         │
      │         ├── pid_.compute(currentSpeedRPM, dt) [PID.cpp]
      │         │    ├── Compute Error = setpoint_ - measurement
      │         │    ├── Compute P-Term = kp_ * error
      │         │    ├── Accumulate Integral: integral_ += error * dt
      │         │    ├── Compute I-Term = ki_ * integral_
      │         │    ├── Compute D-Term (Derivative on measurement)
      │         │    ├── Sum output = P + I + D
      │         │    ├── Clamp output to limits (-150.0 to +150.0)
      │         │    ├── Apply Anti-Windup (revert integral step if saturated)
      │         │    └── Return output (%)
      │         │
      │         └── driveOutput(output) [SpeedMotor.cpp]
      │              ├── Set Direction Pin PB4: HIGH if output >= 0, LOW if output < 0
      │              ├── Clamp magnitude to 0.0% – 100.0% duty cycle
      │              ├── Scale duty to ARR: compareValue = (duty / 100.0) * 4095
      │              └── Write Register: __HAL_TIM_SET_COMPARE(&htim1, CH1, compareValue)
      │
      └── 4. Export Telemetry
           └── g_output_duty = driveMotor.getOutput()
```

---

## Workspace Function Mapping

| Source File | Function / Method | Caller | Responsibility |
| :--- | :--- | :--- | :--- |
| **`main.c`** | `main()` | Boot / Startup | Configures system clocks, initializes peripherals/motors, and enters `while(1)`. |
| | `HAL_TIM_PeriodElapsedCallback()` | `TIM6` Hardware ISR | Periodic scheduler; invokes `MotorControl_Update(dt)`. |
| **`motor_control.cpp`** | `MotorControl_Init()` | `main()` | Starts `TIM5` encoder interface and initializes `driveMotor`. |
| | `MotorControl_Update()` | `HAL_TIM_PeriodElapsedCallback()` | Syncs SWD parameters, reads RPM, runs PID update, and logs output duty. |
| | `ReadDriveSpeedRPM()` | `MotorControl_Update()` | Reads `TIM5->CNT` across a sliding window buffer to compute shaft RPM. |
| | `ReadElbowAbsolutePosition()` | *(Currently Bypassed)* | Reads raw $12$-bit angle data from the AS5600 sensor via `I2C1`. |
| **`SpeedMotor.cpp`** | `SpeedMotor::begin()` | `MotorControl_Init()` | Enables `TIM1` PWM channels, activates Main Output (`MOE`), and stops motor. |
| | `SpeedMotor::update()` | `MotorControl_Update()` | Calls `PID::compute()` and forwards the result to `driveOutput()`. |
| | `SpeedMotor::driveOutput()` | `SpeedMotor::update()` / `stop()` | Updates `PB4` direction state and writes `TIM1->CCR1` duty cycle. |
| **`PID.cpp`** | `PID::setGains()` | `MotorControl_Update()` | Updates $K_p$, $K_i$, and $K_d$ gains dynamically from live SWD variables. |
| | `PID::compute()` | `SpeedMotor::update()` | Calculates PID terms, applies derivative-on-measurement, clamping, and anti-windup. |
| **`PositionMotor.cpp`**| `PositionMotor::begin()` | `Elbow_Control_Init()` | Initializes position controller instance (currently disabled in ISR to prevent pin conflict on `PA8`). |

---

## Live SWD Telemetry & Variable Dataflow

Host tools (`pyocd`) communicate directly with SRAM global variables declared inside an `extern "C"` block in `motor_control.cpp` to prevent C++ name mangling:

```text
[ Host Python / Tkinter GUI via pyOCD ]
       │
       │ Writes Live Setpoints & Gains over SWD
       ▼
   g_kp, g_ki, g_kd, g_target_rpm   (Global SRAM in motor_control.cpp)
       │
       │ Sampled every control tick inside MotorControl_Update()
       ▼
   SpeedMotor::setGains() & setTargetSpeed()
       │
       │ Evaluated against feedback from TIM5 Encoder Ticks
       ▼
   PID::compute()
       │
       │ Outputs Calculated Effort (%)
       ▼
   TIM1->CCR1 & GPIOB->PB4          (Physical PWM & Direction Signals to Cytron MDD20A)
       │
       │ Exports live state back to SRAM
       ▼
   g_current_rpm, g_output_duty, g_raw_ticks   (Read by Host GUI for Live Plotting)
```