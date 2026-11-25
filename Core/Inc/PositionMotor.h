#ifndef POSITION_MOTOR_H_
#define POSITION_MOTOR_H_

#include "main.h"   // CubeMX-generated - pulls in the correct stm32xxxx_hal.h
#include "PID.h"

/*
 * Closed-loop position control (e.g. steering, arm joints, anything with an
 * absolute encoder telling you WHERE it is rather than how fast it's going).
 *
 * You feed it a position reading from your encoder every control cycle; it
 * hands back nothing directly, but drives the PWM + direction pins for you.
 */
class PositionMotor {
public:
    // pwmTim / pwmChannel : timer + channel wired to the driver's PWM input
    // pwmPeriod           : ARR value of that timer (= 100% duty cycle)
    // dirPort/dirPin       : single GPIO for direction (PWM + DIR style driver)
    // positionRange       : full-scale range of your absolute encoder
    //                        (e.g. 360.0f for degrees, 4096.0f for a 12-bit encoder)
    // kp, ki, kd           : starting PID gains (tune on the bench first)
    PositionMotor(TIM_HandleTypeDef* pwmTim, uint32_t pwmChannel, uint32_t pwmPeriod,
                  GPIO_TypeDef* dirPort, uint16_t dirPin,
                  float positionRange,
                  float kp, float ki, float kd);

    void begin();                              // starts the PWM channel, motor at rest
    void setTargetPosition(float positionSetpoint);
    void setGains(float kp, float ki, float kd);

    // Call this every control cycle (e.g. every 10 ms from a timer interrupt).
    // currentPosition: your latest absolute-encoder reading, same units as positionRange.
    // dt: seconds since the previous call.
    void update(float currentPosition, float dt);

    void stop();                               // motor off + PID state reset
    void disableWrap(); // for linear actuators / joints that never wrap

    // Error (in position units) treated as "on target": motor rests, integral frozen.
    // Default: 0.5% of positionRange. Pass 0 to disable.
    void setDeadband(float deadband);

    // Low-pass time constant (seconds) on the D term. Default 0.02 s. 0 = off.
    void setDerivativeFilter(float tau);
    float getOutput() const;                   // last PID output, -100..100 (%)
    float getTargetPosition() const;

private:
    static constexpr float DEFAULT_DEADBAND_FRACTION = 0.005f;
    static constexpr float DEFAULT_D_FILTER_TAU = 0.02f;

    TIM_HandleTypeDef* pwmTim_;
    uint32_t pwmChannel_;
    uint32_t pwmPeriod_;

    GPIO_TypeDef* dirPort_;
    uint16_t dirPin_;

    PID pid_;
    float lastOutput_;

    void driveOutput(float pidOutput); // -100..100 -> direction pins + PWM duty
};

#endif /* POSITION_MOTOR_H_ */
