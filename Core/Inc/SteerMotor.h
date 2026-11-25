#ifndef STEER_MOTOR_H_
#define STEER_MOTOR_H_

#include "main.h"
#include "PositionMotor.h"

/*
 * One steering joint of the double-Ackermann rover.
 *
 * Works in the high-level controller's frame:
 *     0 deg ... 90 deg (home / straight ahead) ... 180 deg
 * i.e. +/-90 deg of steering lock around home.
 *
 * The absolute encoder (AS5600, 12-bit) sits on the output shaft. Its raw
 * count at the home position is arbitrary (depends on how the magnet was
 * mounted), so each wheel gets a calibrated homeRaw. Raw counts are converted
 * to steering degrees here, folded so the 4095 -> 0 rollover can land
 * anywhere without breaking anything.
 *
 * Setpoints are rate-limited so all four wheels slew at the same speed and
 * the Ackermann geometry stays consistent while the wheels are moving.
 */
class SteerMotor {
public:
    // homeRaw     : raw encoder count when the wheel is pointing straight (90 deg)
    // encoderDir  : +1 if raw counts increase as the steering angle increases
    //               (towards 180), -1 otherwise.
    // motorDir    : +1 if driving with DIR pin HIGH moves the steering angle UP,
    //               -1 if DIR LOW moves it up. Set encoderDir first, then fix
    //               motorDir so a positive output raises the angle - otherwise
    //               the loop has positive feedback and runs to the lock.
    SteerMotor(TIM_HandleTypeDef* pwmTim, uint32_t pwmChannel, uint32_t pwmPeriod,
               GPIO_TypeDef* dirPort, uint16_t dirPin,
               uint16_t homeRaw, int encoderDir, int motorDir,
               float kp, float ki, float kd);

    void begin();
    void stop();

    // Target from the Ackermann controller, degrees. Clamped to [0, 180].
    void setTargetAngle(float deg);

    // Call every control cycle with the raw encoder reading (0..4095).
    void update(uint16_t rawCounts, float dt);

    void setGains(float kp, float ki, float kd);       // gains are per DEGREE of error
    void setDeadband(float deg);                       // default 0.5 deg
    void setMaxRate(float degPerSec);                  // default 90 deg/s, 0 = no limit
    void setHomeRaw(uint16_t homeRaw);                 // re-calibrate at runtime

    // Open-loop drive (-100..100 %) for the polarity check. Positive should
    // raise the angle. Re-seeds the ramp so closed-loop resumes from here.
    void driveOpenLoop(float duty);

    float rawToAngle(uint16_t rawCounts) const;        // raw counts -> 0..180 frame

    float getAngle() const;           // last measured steering angle, deg
    float getTargetAngle() const;     // final target from the controller, deg
    float getRampedTarget() const;    // rate-limited setpoint the PID is chasing, deg
    float getOutput() const;          // last PID output, -100..100 (%)

    static constexpr float MIN_ANGLE  = 0.0f;
    static constexpr float HOME_ANGLE = 90.0f;
    static constexpr float MAX_ANGLE  = 180.0f;

private:
    static constexpr int   COUNTS_PER_REV   = 4096;
    static constexpr float DEFAULT_DEADBAND = 0.5f;   // deg
    static constexpr float DEFAULT_MAX_RATE = 90.0f;  // deg/s

    PositionMotor motor_;
    uint16_t homeRaw_;
    int encoderDir_;

    float target_;        // where the controller wants the wheel
    float ramped_;        // rate-limited setpoint actually fed to the PID
    float maxRate_;
    float angle_;
    bool  rampInit_;      // false -> seed ramped_ from the next measurement
};

#endif /* STEER_MOTOR_H_ */
