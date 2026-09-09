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
    // dirPort/dirPin1/dirPin2 : two GPIOs for direction (H-bridge IN1/IN2 style)
    // positionRange       : full-scale range of your absolute encoder
    //                        (e.g. 360.0f for degrees, 4096.0f for a 12-bit encoder)
    // kp, ki, kd           : starting PID gains (tune on the bench first)
    PositionMotor(TIM_HandleTypeDef* pwmTim, uint32_t pwmChannel, uint32_t pwmPeriod,
                  GPIO_TypeDef* dirPort, uint16_t dirPin1, uint16_t dirPin2,
                  float positionRange,
                  float kp, float ki, float kd);

    void begin();                              // starts the PWM channel, motor at rest
    void setTargetPosition(float positionSetpoint);
    void setGains(float kp, float ki, float kd);

    // Call this every control cycle (e.g. every 10 ms from a timer interrupt).
    // currentPosition: your latest absolute-encoder reading, same units as positionRange.
    // dt: seconds since the previous call.
    void update(float currentPosition, float dt);

    void stop();                               // zero PWM, release direction pins
    float getOutput() const;                   // last PID output, -100..100 (%)
    float getTargetPosition() const;

private:
    TIM_HandleTypeDef* pwmTim_;
    uint32_t pwmChannel_;
    uint32_t pwmPeriod_;

    GPIO_TypeDef* dirPort_;
    uint16_t dirPin1_;
    uint16_t dirPin2_;

    PID pid_;
    float lastOutput_;

    void driveOutput(float pidOutput); // -100..100 -> direction pins + PWM duty
};

#endif /* POSITION_MOTOR_H_ */
