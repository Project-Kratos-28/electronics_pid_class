#include "motor_control.h"
#include "PositionMotor.h"
#include "SpeedMotor.h"

/*
 * EXAMPLE ONLY - replace pin/timer assignments, encoder reads, and gains
 * with your actual rover hardware. This shows the intended integration
 * pattern: one instance per physical motor, updated every control tick.
 */

// --- Example: a steering motor (position-controlled) ---
// TIM1 CH1 for PWM, GPIOB pins 0/1 for direction, 12-bit absolute encoder (0-4095)
static PositionMotor steeringMotor(&htim1, TIM_CHANNEL_1, 999,
                                    GPIOB, GPIO_PIN_0, GPIO_PIN_1,
                                    4096.0f,
                                    2.0f, 0.05f, 0.1f); // starting gains - tune these

// --- Example: a drive wheel motor (speed-controlled) ---
static SpeedMotor driveMotor(&htim2, TIM_CHANNEL_1, 999,
                              GPIOB, GPIO_PIN_2, GPIO_PIN_3,
                              0.8f, 2.0f, 0.0f); // starting gains - tune these

// --- Your sensor reads go here ---

static float ReadSteeringPositionCounts() {
    // TODO: replace with your absolute encoder read (SPI/I2C/ADC),
    // scaled to 0-4095 to match the positionRange passed above.
    return 0.0f;
}

static float ReadDriveSpeedRPM() {
    // TODO: replace with your speed calculation, e.g. from encoder pulse
    // count over the last dt, converted to RPM. Signed if you can detect
    // direction, otherwise the motor will only be able to drive forward.
    return 0.0f;
}

// --- extern "C" entry points, callable from main.c ---

void MotorControl_Init(void) {
    steeringMotor.begin();
    driveMotor.begin();
}

// Call from your control loop - e.g. a 10ms TIM interrupt callback,
// or the main while(1) loop if you're timing it yourself.
void MotorControl_Update(float dt_seconds) {
    // Setpoints would normally come from your command/telemetry system
    // rather than being hardcoded like this.
    steeringMotor.setTargetPosition(2048.0f); // example: center position
    driveMotor.setTargetSpeed(150.0f);        // example: 150 RPM

    steeringMotor.update(ReadSteeringPositionCounts(), dt_seconds);
    driveMotor.update(ReadDriveSpeedRPM(), dt_seconds);
}
